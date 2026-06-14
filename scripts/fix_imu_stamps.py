#!/usr/bin/env python3
"""
fix_imu_stamps.py — IMU timestamp jitter fix via LSQ uniform grid rewriting.

배경:
  ROS2 bag의 /edie/sensor/lpf_imu 메시지는 두 가지 타임스탬프를 가진다.
    - bag_timestamp  : bag recorder가 수신 시각으로 기록 (비교적 양호)
    - cdr_header.stamp: ICM20948 드라이버가 Python GIL 폴링 시각으로 스탬핑
                        → 지터 + 중복(7.49%) + 큰 gap 발생 (원인 A/B)

  Kalibr는 cdr_header.stamp를 사용하므로 이 값을 LSQ 균일 그리드로 재작성한다.
  bag_timestamp도 동일 그리드로 덮어써서 재생 순서를 보장한다.

해결 범위 (Step 1 임시방편):
  ✓ 원인 A — Python GIL 지터로 인한 불균일 stamp
  ✓ 원인 B — race condition으로 인한 중복 stamp (0ms 간격)
  ✗ Step 2 — 7% 측정값 중복/유실 (데이터 자체는 그대로), 절대 오프셋 ~0.67s

Usage:
    python3 scripts/fix_imu_stamps.py <src_bag_dir> <dst_bag_dir>

    src_bag_dir : 원본 ROS2 bag 디렉토리
    dst_bag_dir : 출력 디렉토리 (없으면 생성, 있으면 덮어쓰기)
"""

import argparse
import shutil
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np

IMU_TOPIC = "/edie/sensor/lpf_imu"
IMU_RATE_HZ = 400

# CDR 직렬화 blob 내 header.stamp 위치
#   bytes 0-3 : CDR encapsulation header
#   bytes 4-7 : int32  sec
#   bytes 8-11: uint32 nanosec
CDR_SEC_OFFSET  = 4
CDR_NSEC_OFFSET = 8


def find_db3(bag_dir: Path) -> Path:
    dbs = sorted(bag_dir.glob("*.db3"))
    if not dbs:
        raise FileNotFoundError(f".db3 파일 없음: {bag_dir}")
    return dbs[0]


def analyze(label: str, ts: np.ndarray) -> dict:
    diffs = np.diff(ts) / 1e6  # ms
    dups  = int(np.sum(diffs == 0))
    gaps  = int(np.sum(diffs > 5))
    return dict(
        label=label, n=len(ts),
        median=float(np.median(diffs)), max=float(np.max(diffs)), min=float(np.min(diffs)),
        dups=dups, dup_pct=100*dups/len(ts),
        gaps=gaps, gap_pct=100*gaps/len(ts),
    )


def print_stats(s: dict):
    print(f"  [{s['label']}]  n={s['n']}")
    print(f"    interval  median={s['median']:.4f}ms  max={s['max']:.3f}ms  min={s['min']:.3f}ms")
    print(f"    duplicate(0ms): {s['dups']} ({s['dup_pct']:.2f}%)")
    print(f"    gap(>5ms):      {s['gaps']} ({s['gap_pct']:.2f}%)")


def check_data_duplication(rows: list) -> dict:
    """측정값 중복 확인 — Step 2 잔존 여부 검증."""
    prev_blob = None
    dup_count = 0
    for _, blob in rows:
        if prev_blob is not None and bytes(blob)[12:] == bytes(prev_blob)[12:]:
            dup_count += 1
        prev_blob = blob
    return dict(data_dups=dup_count, data_dup_pct=100*dup_count/len(rows))


def fix_stamps(src_dir: Path, dst_dir: Path):
    # 원본 복사
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir)
    print(f"복사 완료: {src_dir.name} → {dst_dir.name}")

    db_path = find_db3(dst_dir)
    con = sqlite3.connect(str(db_path))

    # IMU topic id 확인
    row = con.execute("SELECT id FROM topics WHERE name = ?", (IMU_TOPIC,)).fetchone()
    if row is None:
        raise RuntimeError(f"토픽 없음: {IMU_TOPIC}")
    topic_id = row[0]

    # 전체 IMU 메시지 로드 (bag_timestamp 순)
    rows = con.execute(
        "SELECT id, timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    ).fetchall()
    n = len(rows)
    print(f"\nIMU 메시지: {n}개")

    ids      = [r[0] for r in rows]
    bag_ts   = np.array([r[1] for r in rows], dtype=np.int64)
    blobs    = [r[2] for r in rows]

    cdr_ts = np.array([
        struct.unpack_from('<i', bytes(b), CDR_SEC_OFFSET)[0] * 1_000_000_000 +
        struct.unpack_from('<I', bytes(b), CDR_NSEC_OFFSET)[0]
        for b in blobs
    ], dtype=np.int64)

    # ── 수정 전 통계 ────────────────────────────────────────────
    print("\n[ 수정 전 ]")
    print_stats(analyze("bag_timestamp  ", bag_ts))
    print_stats(analyze("cdr_header.stamp", cdr_ts))
    dd = check_data_duplication(list(zip(ids, blobs)))
    print(f"  [측정값 중복]  {dd['data_dups']}개 ({dd['data_dup_pct']:.2f}%)  ← Step 2 (수정 안 됨)")

    # ── LSQ 직선 피팅: t[i] = t0 + i * dt ──────────────────────
    idx = np.arange(n, dtype=np.float64)

    # cdr_ts 기준으로 피팅 (Kalibr가 사용하는 값)
    coeffs  = np.polyfit(idx, cdr_ts.astype(np.float64), 1)
    dt_ns   = coeffs[0]
    t0_ns   = coeffs[1]
    dt_ms   = dt_ns / 1e6
    exp_ms  = 1000.0 / IMU_RATE_HZ

    print(f"\n[ LSQ 피팅 결과 ]")
    print(f"  t0  = {t0_ns:.0f} ns")
    print(f"  dt  = {dt_ns:.3f} ns  ({dt_ms:.4f} ms,  기대값 {exp_ms:.4f} ms)")
    print(f"  편차 = {abs(dt_ms - exp_ms)*1000:.2f} µs/sample")

    new_ts = np.round(t0_ns + idx * dt_ns).astype(np.int64)

    corrections_ms = (new_ts - cdr_ts) / 1e6
    print(f"  보정량  median={np.median(np.abs(corrections_ms)):.3f}ms  max={np.max(np.abs(corrections_ms)):.3f}ms")

    # ── DB 업데이트 ─────────────────────────────────────────────
    print(f"\n{n}개 타임스탬프 업데이트 중...")
    updates = []
    for i, (msg_id, blob) in enumerate(zip(ids, blobs)):
        t = int(new_ts[i])
        sec  = t // 1_000_000_000
        nsec = t %  1_000_000_000

        ba = bytearray(bytes(blob))
        struct.pack_into('<i', ba, CDR_SEC_OFFSET,  sec)
        struct.pack_into('<I', ba, CDR_NSEC_OFFSET, nsec)

        updates.append((t, bytes(ba), msg_id))

    con.executemany(
        "UPDATE messages SET timestamp = ?, data = ? WHERE id = ?",
        updates
    )
    con.commit()
    con.close()

    # ── 수정 후 검증 ─────────────────────────────────────────────
    con2 = sqlite3.connect(str(db_path))
    rows2 = con2.execute(
        "SELECT id, timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    ).fetchall()
    con2.close()

    new_bag_ts = np.array([r[1] for r in rows2], dtype=np.int64)
    new_cdr_ts = np.array([
        struct.unpack_from('<i', bytes(r[2]), CDR_SEC_OFFSET)[0] * 1_000_000_000 +
        struct.unpack_from('<I', bytes(r[2]), CDR_NSEC_OFFSET)[0]
        for r in rows2
    ], dtype=np.int64)

    print("\n[ 수정 후 ]")
    print_stats(analyze("bag_timestamp  ", new_bag_ts))
    print_stats(analyze("cdr_header.stamp", new_cdr_ts))

    blobs2 = [r[2] for r in rows2]
    dd2 = check_data_duplication(list(zip([r[0] for r in rows2], blobs2)))
    print(f"  [측정값 중복]  {dd2['data_dups']}개 ({dd2['data_dup_pct']:.2f}%)  ← Step 2 잔존 확인")

    # ── Step 1 / Step 2 판정 요약 ────────────────────────────────
    step1_ok = (analyze("cdr_header.stamp", new_cdr_ts)['dups'] == 0 and
                analyze("cdr_header.stamp", new_cdr_ts)['gaps'] == 0)
    step2_remain = dd2['data_dups'] > 0

    print("\n[ 결과 요약 ]")
    print(f"  Step 1 (stamp 지터/중복 제거): {'✓ 해소' if step1_ok else '✗ 미완'}")
    print(f"  Step 2 (측정값 중복 {dd2['data_dup_pct']:.1f}%): {'잔존 (예상대로)' if step2_remain else '없음'}")
    print(f"\n출력 bag: {dst_dir}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("src", help="원본 ROS2 bag 디렉토리")
    parser.add_argument("dst", help="출력 ROS2 bag 디렉토리")
    args = parser.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)

    if not src.exists():
        print(f"오류: 원본 bag 없음: {src}", file=sys.stderr)
        sys.exit(1)

    fix_stamps(src, dst)


if __name__ == "__main__":
    main()
