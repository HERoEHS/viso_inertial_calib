#!/usr/bin/env python3
"""
ROS2 bag(left/right image_gray)에서 Rolling vs Global shutter 판별
==================================================================
rolling_shutter_test.py(라이브 카메라 캡처 버전)를 변형하여,
이미 녹화된 ROS2 bag(.db3)을 프레임 단위로 읽어 판별한다.

특징
- ROS2 런타임 불필요: sqlite3로 .db3를 직접 열고 sensor_msgs/Image CDR을 수동 파싱.
- 원리(원본 아이디어 강화판):
    Global shutter는 모든 행(row)이 동시에 노출되므로, 연속 두 프레임 사이의
    '행별 수평 이동량 dx(y)'가 행 위치 y와 무관하게 균일하다(기울기 ≈ 0).
    Rolling shutter는 행마다 노출 시각이 (y/H)*t_readout 만큼 어긋나므로,
    카메라 속도가 변하는(가속) 구간에서 dx(y)가 y에 선형 의존한다(기울기 ≠ 0 = skew).
    손으로 흔든 calibration bag에는 가속이 풍부하므로 이 신호가 잘 드러난다.

    dx(y) 기울기 ∝ (각가속도) × (readout_time) × (frame_period)
    → GS: 기울기 0,  RS: 기울기 유의미하게 0이 아님.

사용법
    python3 detect_shutter_from_bag.py \
        --bag /path/to/vio_calib_20260612_162851 \
        [--topic /edie/sensors/camera/left/image_gray] \
        [--bands 6] [--topk 25]

    --topic 미지정 시 bag 안의 모든 Image 토픽을 자동으로 각각 분석.
"""

import argparse
import glob
import os
import sqlite3
import sys

import cv2
import numpy as np


# ─────────────────────────────────────────────────────────────
# 최소 CDR 디시리얼라이저 (sensor_msgs/msg/Image 전용)
#   ROS2 메시지는 4바이트 encapsulation 헤더 + CDR 본문 구조.
#   본문 정렬(alignment)은 헤더 다음(본문 시작)을 기준으로 계산한다.
# ─────────────────────────────────────────────────────────────
class _CDR:
    def __init__(self, buf: bytes):
        self.buf = buf
        # buf[0:4] = encapsulation 헤더. buf[1]==1 이면 little-endian CDR.
        self.le = (len(buf) >= 2 and buf[1] == 1)
        self.off = 4  # encapsulation 헤더 건너뜀
        self._end = "<" if self.le else ">"

    def _align(self, n: int):
        # 정렬은 '본문 시작(=offset 4)' 기준. body_pos가 n의 배수가 되도록 패딩.
        body_pos = self.off - 4
        pad = (-body_pos) % n
        self.off += pad

    def u8(self) -> int:
        v = self.buf[self.off]
        self.off += 1
        return v

    def u32(self) -> int:
        self._align(4)
        v = int.from_bytes(self.buf[self.off:self.off + 4],
                           "little" if self.le else "big", signed=False)
        self.off += 4
        return v

    def i32(self) -> int:
        self._align(4)
        v = int.from_bytes(self.buf[self.off:self.off + 4],
                           "little" if self.le else "big", signed=True)
        self.off += 4
        return v

    def string(self) -> str:
        ln = self.u32()                       # 길이(널 종료 포함)
        raw = self.buf[self.off:self.off + ln]
        self.off += ln
        return raw.split(b"\x00", 1)[0].decode("ascii", "replace")

    def bytes_seq(self) -> bytes:
        ln = self.u32()                       # 시퀀스 길이
        raw = self.buf[self.off:self.off + ln]
        self.off += ln
        return raw


def decode_image(buf: bytes):
    """sensor_msgs/Image CDR → (encoding, np.ndarray[H,W] uint8 grayscale)."""
    r = _CDR(buf)
    r.i32()             # header.stamp.sec
    r.u32()             # header.stamp.nanosec
    r.string()          # header.frame_id
    height = r.u32()
    width = r.u32()
    encoding = r.string()
    r.u8()              # is_bigendian
    step = r.u32()
    data = r.bytes_seq()

    arr = np.frombuffer(data, dtype=np.uint8)
    enc = encoding.lower()
    if enc in ("mono8", "8uc1"):
        img = arr.reshape(height, step)[:, :width]
    elif enc in ("bgr8", "rgb8"):
        img = arr.reshape(height, step // 3, 3)[:, :width, :]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    elif enc in ("mono16", "16uc1"):
        img16 = np.frombuffer(data, dtype=np.uint16).reshape(height, step // 2)[:, :width]
        img = cv2.normalize(img16, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    else:
        # 알 수 없는 인코딩은 mono8로 가정 시도
        img = arr[:height * width].reshape(height, width)
    return encoding, np.ascontiguousarray(img)


# ─────────────────────────────────────────────────────────────
# bag(.db3) 읽기
# ─────────────────────────────────────────────────────────────
def find_db3(bag_path: str) -> str:
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return bag_path
    cands = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
    if not cands:
        print(f"[ERR] .db3 파일을 찾지 못함: {bag_path}")
        sys.exit(1)
    return cands[0]


def list_image_topics(db3: str):
    con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    try:
        rows = con.execute(
            "SELECT name, type FROM topics ORDER BY name").fetchall()
    finally:
        con.close()
    return [n for (n, t) in rows if t == "sensor_msgs/msg/Image"]


def read_frames(db3: str, topic: str, max_frames: int | None = None):
    con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    try:
        row = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
        if row is None:
            raise RuntimeError(f"토픽 없음: {topic}")
        tid = row[0]
        q = con.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,))
        stamps, frames = [], []
        enc_seen = None
        for ts, blob in q:
            enc, img = decode_image(bytes(blob))
            enc_seen = enc
            stamps.append(ts)
            frames.append(img)
            if max_frames and len(frames) >= max_frames:
                break
        return np.array(stamps, dtype=np.int64), frames, enc_seen
    finally:
        con.close()


# ─────────────────────────────────────────────────────────────
# 핵심 분석: 행(row)별 수평 이동량의 기울기(skew) 측정
# ─────────────────────────────────────────────────────────────
def band_shift_slope(A: np.ndarray, B: np.ndarray, n_bands: int):
    """연속 두 프레임 A,B를 n_bands개의 가로 띠로 나눠 각 띠의 수평 이동량 dx를
    phaseCorrelate로 추정하고, dx(y)에 대한 1차 회귀로 기울기/skew를 구한다.

    반환: (skew_px, mean_shift_px, n_valid)
      skew_px     = 최상단 행 ~ 최하단 행 사이 dx 차이(px). RS면 0이 아님.
      mean_shift  = 전체 평균 수평 이동량(px). 모션 크기 프록시.
    """
    H, W = A.shape
    bh = H // n_bands
    if bh < 8:
        return None
    ys, dxs, ws = [], [], []
    for b in range(n_bands):
        y0 = b * bh
        y1 = H if b == n_bands - 1 else (b + 1) * bh
        pa = A[y0:y1, :].astype(np.float32)
        pb = B[y0:y1, :].astype(np.float32)
        pa -= pa.mean()
        pb -= pb.mean()
        win = cv2.createHanningWindow((pa.shape[1], pa.shape[0]), cv2.CV_32F)
        (dx, _dy), resp = cv2.phaseCorrelate(pa, pb, win)
        ys.append(0.5 * (y0 + y1))
        dxs.append(dx)
        ws.append(max(resp, 1e-6))
    ys = np.asarray(ys, np.float64)
    dxs = np.asarray(dxs, np.float64)
    ws = np.asarray(ws, np.float64)

    # response 가중 1차 회귀 dx = m*y + c
    coeffs = np.polyfit(ys, dxs, 1, w=ws)
    slope = coeffs[0]
    skew_px = slope * (H - 1)
    mean_shift = float(np.average(dxs, weights=ws))
    return skew_px, mean_shift, len(dxs)


def analyze_topic(stamps, frames, enc, n_bands, topk, label):
    n = len(frames)
    print(f"\n{'='*64}")
    print(f"[{label}]  frames={n}  encoding={enc}  size={frames[0].shape[1]}x{frames[0].shape[0]}")

    if n < 5:
        print("  프레임이 너무 적어 분석 불가")
        return None

    # fps 추정
    if len(stamps) >= 2:
        dt = np.diff(stamps) / 1e9
        dt = dt[(dt > 0) & (dt < 1.0)]
        if len(dt):
            print(f"  추정 fps={1.0/np.median(dt):.2f}  (frame period {np.median(dt)*1000:.1f} ms)")

    # 1) 연속 프레임 모션(mean abs diff) → 빠른 움직임 구간 선별
    motion = np.empty(n - 1, np.float64)
    for i in range(n - 1):
        motion[i] = np.mean(np.abs(frames[i].astype(np.int16) - frames[i + 1].astype(np.int16)))
    order = np.argsort(-motion)
    sel = order[:topk]

    skews, shifts = [], []
    for i in sel:
        if motion[i] < 1.0:       # 거의 정지 프레임 제외
            continue
        res = band_shift_slope(frames[i], frames[i + 1], n_bands)
        if res is None:
            continue
        skew_px, mean_shift, _ = res
        if abs(mean_shift) < 0.3:  # 유효 모션 없음
            continue
        skews.append(skew_px)
        shifts.append(mean_shift)

    if len(skews) < 3:
        print("  유효한 고속 모션 구간이 부족합니다 (bag에 빠른 움직임이 없을 수 있음).")
        return None

    skews = np.asarray(skews)
    shifts = np.asarray(shifts)
    abs_skew = np.abs(skews)
    abs_shift = np.abs(shifts)

    med_skew = float(np.median(abs_skew))
    p90_skew = float(np.percentile(abs_skew, 90))
    med_shift = float(np.median(abs_shift))
    max_shift = float(np.max(abs_shift))
    ratio = med_skew / (med_shift + 1e-6)        # skew / shift (RS 불변 프록시)
    # skew와 shift의 부호 상관(같은 부호로 함께 커지면 RS 특성)
    corr = float(np.corrcoef(skews, shifts)[0, 1]) if len(skews) > 2 else 0.0

    print(f"  분석 사용 고속 프레임쌍: {len(skews)}개")
    print(f"  평균 수평 이동량  median={med_shift:.2f}px  max={max_shift:.2f}px")
    print(f"  상-하단 skew(px)  median={med_skew:.2f}  p90={p90_skew:.2f}")
    print(f"  skew/shift ratio  {ratio:.3f}")
    print(f"  skew~shift 상관   {corr:+.2f}")

    # 2) 보조 지표: 원본 스크립트 방식(상/중/하 모션 비율, 수평 skew)
    top_pair = sel[0]
    d = np.abs(frames[top_pair].astype(float) - frames[top_pair + 1].astype(float))
    h = d.shape[0]; t = h // 3
    tt, mm, bb = d[:t].mean(), d[t:2*t].mean(), d[2*t:].mean()
    print(f"  (보조) 상/중/하 모션: {tt:.1f}/{mm:.1f}/{bb:.1f}  max/min={max(tt,mm,bb)/(min(tt,mm,bb)+1e-6):.2f}")

    # ── 판정 ─────────────────────────────────────────────
    # 충분한 모션이 있었는지 먼저 확인 (가속 신호가 있어야 신뢰 가능)
    verdict, reason, conf = _decide(med_skew, ratio, med_shift, max_shift, p90_skew)
    print(f"\n  >>> 판정: {verdict}  (신뢰도 {conf})")
    print(f"      근거: {reason}")
    return verdict


def _decide(med_skew, ratio, med_shift, max_shift, p90_skew):
    if max_shift < 2.0:
        return ("판별 불가(모션 부족)", "최대 수평 이동량이 너무 작음 — 빠른 좌우 흔들기 구간이 필요", "낮음")
    # Rolling shutter: skew가 절대적으로 유의미 + shift 대비 비율도 큼
    if med_skew >= 1.5 and ratio >= 0.15:
        return ("Rolling Shutter 가능성 높음",
                f"행별 이동량 기울기(skew median={med_skew:.2f}px)가 크고 "
                f"skew/shift={ratio:.2f}≥0.15", "높음" if p90_skew > 2.5 else "중간")
    # Global shutter: skew가 거의 0
    if med_skew < 0.8 and ratio < 0.10:
        return ("Global Shutter 가능성 높음",
                f"빠른 모션(max shift={max_shift:.1f}px)에도 행별 skew median={med_skew:.2f}px≈0, "
                f"skew/shift={ratio:.2f}<0.10", "높음")
    return ("판정 애매(경계값)",
            f"skew median={med_skew:.2f}px, ratio={ratio:.2f} — 추가 데이터 권장", "낮음")


def main():
    ap = argparse.ArgumentParser(description="ROS2 bag에서 Rolling/Global shutter 판별")
    ap.add_argument("--bag", "-b",
                    default="/home/higony/ros2_ws/src/edie9/edie_localization/third_party/"
                            "viso_inertial_calib/data/calib/vio_calib_20260612_162851",
                    help="bag 디렉토리 또는 .db3 경로")
    ap.add_argument("--topic", "-t", default=None,
                    help="분석할 Image 토픽 (미지정 시 모든 Image 토픽 분석)")
    ap.add_argument("--bands", type=int, default=6, help="가로 띠 분할 수")
    ap.add_argument("--topk", type=int, default=30, help="분석할 고속 모션 프레임쌍 수")
    ap.add_argument("--max-frames", type=int, default=None, help="토픽당 최대 읽을 프레임 수")
    args = ap.parse_args()

    db3 = find_db3(args.bag)
    print(f"bag db3: {db3}")

    topics = [args.topic] if args.topic else list_image_topics(db3)
    if not topics:
        print("[ERR] Image 토픽이 없습니다.")
        sys.exit(1)
    print(f"분석 대상 토픽: {topics}")

    results = {}
    for tp in topics:
        stamps, frames, enc = read_frames(db3, tp, args.max_frames)
        results[tp] = analyze_topic(stamps, frames, enc, args.bands, args.topk, tp)

    print(f"\n{'='*64}\n요약")
    for tp, v in results.items():
        print(f"  {tp}: {v}")


if __name__ == "__main__":
    main()
