#!/usr/bin/env python3
"""Kalibr imu-cam 결과(<bag>-results-imucam.txt)를 읽어 한 화면 요약 + PASS/WARN/FAIL 판정.

왜: Kalibr 는 49 px 로 발산해도 결과 파일을 다 쓰고 정상 종료한다(2026-11-12 .224 run1). 사람이 숫자를 안 보면
틀린 외부값이 장비 보정 자리로 게시된다. 그래서 끝난 뒤 기계가 먼저 본다. 문턱은 모두 임시값이다.

사용: python3 kalibr_quality.py <results-imucam.txt> [--ref <T_cam_imu 가 든 camchain yaml>] [--out <요약 파일>]
종료 코드: 0 PASS, 1 WARN, 2 FAIL, 3 파일을 못 읽음
numpy 없이 돈다(로봇에서도). --ref 를 줄 때만 PyYAML 이 필요하다.
"""
import argparse
import math
import re
import sys

# ── 문턱(임시값 — .224 성공 1.14 px·발산 49 px, 옛 35 px 결과 세 개로만 정함) ──────────────────
REPROJ_PASS_PX = 2.0         # 재투영 평균 < 2 px 이면 PASS
REPROJ_FAIL_PX = 5.0         # ≥ 5 px 이면 FAIL(최적화 전 관문과 같은 값), 그 사이는 WARN
NORM_PASS_LO, NORM_PASS_HI = 0.3, 3.0   # 정규화 잔차(잡음값으로 나눈 오차) 평균이 이 안이면 PASS(대략 1 이 정상)
NORM_FAIL_HI = 10.0          # 10 넘으면 FAIL(IMU 와 카메라가 서로 안 맞음), 0.3 미만은 WARN(잡음값을 너무 크게 줌)
TD_DIFF_PASS_MS = 3.0        # cam0·cam1 시간차 차이 < 3 ms 이면 PASS(두 카메라는 같은 시각에 찍힘)
TD_DIFF_FAIL_MS = 10.0       # ≥ 10 ms 이면 FAIL, 그 사이 WARN
REF_ROT_WARN_DEG = 3.0       # 기준 대비 회전 차 ≥ 3° 이면 WARN(기준값 자체가 틀렸을 수 있어 FAIL 은 주지 않음)
REF_TRANS_WARN_M = 0.03      # 기준 대비 카메라 위치(IMU 좌표) 차 ≥ 3 cm 이면 WARN

_NUM = r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?"
_RANK = {"PASS": 0, "WARN": 1, "FAIL": 2}


def _stats_line(text, label_regex):
    m = re.search(label_regex + r"\s*:?\s*mean\s+(" + _NUM + r")\s*,\s*median\s+(" + _NUM + r")", text)
    return (float(m.group(1)), float(m.group(2))) if m else None


def _section(text, start, end_markers):
    i = text.find(start)
    if i < 0:
        return ""
    j = len(text)
    for e in end_markers:
        k = text.find(e, i + len(start))
        if 0 <= k < j:
            j = k
    return text[i:j]


def parse_results(text):
    """results-imucam.txt 글을 dict 로. 못 찾은 값은 None."""
    norm = _section(text, "Normalized Residuals", ["\nResiduals"])
    resid = _section(text, "\nResiduals", ["Transformation", "Baselines", "Gravity"])
    cams = sorted(set(int(c) for c in re.findall(r"Reprojection error \(cam(\d+)\)", text)))
    out = {"cams": {}, "gyro_norm": None, "accel_norm": None, "gyro_rad_s": None, "accel_m_s2": None}
    for c in cams:
        no_corner = re.search(r"Reprojection error \(cam%d\)[^\n]*no corners" % c, text) is not None
        rp = _stats_line(resid, r"Reprojection error \(cam%d\) \[px\]" % c)
        nr = _stats_line(norm, r"Reprojection error \(cam%d\)" % c)
        T = None
        m = re.search(r"T_ci:\s*\(imu0 to cam%d\):\s*\n" % c, text)
        if m:
            nums = re.findall(_NUM, "\n".join(text[m.end():].splitlines()[:4]))
            if len(nums) >= 16:
                v = [float(x) for x in nums[:16]]
                T = [v[0:4], v[4:8], v[8:12], v[12:16]]
        td = None
        m = re.search(r"timeshift cam%d to imu0: \[s\][^\n]*\n\s*(" % c + _NUM + ")", text)
        if m:
            td = float(m.group(1))
        out["cams"][c] = {"reproj_px_mean": rp[0] if rp else None, "reproj_px_median": rp[1] if rp else None,
                          "reproj_norm_mean": nr[0] if nr else None, "no_corners": no_corner,
                          "T_ci": T, "timeshift_s": td}
    g = _stats_line(norm, r"Gyroscope error \(imu0\)")
    a = _stats_line(norm, r"Accelerometer error \(imu0\)")
    gr = _stats_line(resid, r"Gyroscope error \(imu0\) \[rad/s\]")
    ar = _stats_line(resid, r"Accelerometer error \(imu0\) \[m/s\^2\]")
    out["gyro_norm"] = g[0] if g else None
    out["accel_norm"] = a[0] if a else None
    out["gyro_rad_s"] = gr[0] if gr else None
    out["accel_m_s2"] = ar[0] if ar else None
    return out


# ── 작은 행렬 계산(numpy 없이) ───────────────────────────────────────────────
def _R(T):
    return [row[:3] for row in T[:3]]


def _t(T):
    return [T[0][3], T[1][3], T[2][3]]


def _matT(A):
    return [list(r) for r in zip(*A)]


def _matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def rotation_angle_deg(Ra, Rb):
    """두 회전 사이 각(도)."""
    D = _matmul(_matT(Ra), Rb)
    c = (D[0][0] + D[1][1] + D[2][2] - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def cam_position_in_imu(T_ci):
    """T_ci(IMU→카메라)에서 IMU 좌표로 본 카메라 위치 p = −Rᵀt. (critic M1: t_cam_imu 끼리 비교하면 회전이 섞인다)"""
    R, t = _R(T_ci), _t(T_ci)
    p = _matvec(_matT(R), t)
    return [-x for x in p]


def load_ref_T_cam_imu(path, cam="cam0"):
    """Kalibr camchain 꼴 yaml 에서 T_cam_imu(4×4)를 읽는다. 없으면 None."""
    import yaml
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f) or {}
    T = (d.get(cam) or {}).get("T_cam_imu")
    if T is None:
        return None
    return [[float(x) for x in row] for row in T]


def _verdict_reproj(v):
    if v is None:
        return "FAIL"
    return "PASS" if v < REPROJ_PASS_PX else ("WARN" if v < REPROJ_FAIL_PX else "FAIL")


def _verdict_norm(v):
    if v is None:
        return "FAIL"
    if v > NORM_FAIL_HI:
        return "FAIL"
    return "PASS" if NORM_PASS_LO <= v <= NORM_PASS_HI else "WARN"


def judge(res, ref_T=None):
    """parse_results 결과 → (종합 판정, [(항목, 값 문자열, 기준 문자열, 판정)])."""
    rows = []
    if not res["cams"]:
        rows.append(("재투영", "카메라 결과 없음", "-", "FAIL"))
    for c, d in sorted(res["cams"].items()):
        v = d["reproj_px_mean"]
        rows.append(("cam%d 재투영 평균" % c, "없음(no corners)" if v is None else "%.3f px" % v,
                     "PASS <%.0f, FAIL ≥%.0f px" % (REPROJ_PASS_PX, REPROJ_FAIL_PX), _verdict_reproj(v)))
    for key, label in (("gyro_norm", "자이로 정규화 잔차 평균"), ("accel_norm", "가속도 정규화 잔차 평균")):
        v = res[key]
        rows.append((label, "없음" if v is None else "%.3f" % v,
                     "PASS %.1f~%.0f, FAIL >%.0f" % (NORM_PASS_LO, NORM_PASS_HI, NORM_FAIL_HI), _verdict_norm(v)))
    tds = [(c, d["timeshift_s"]) for c, d in sorted(res["cams"].items())]
    if len(tds) >= 2:
        if all(t is not None for _, t in tds):
            ms = [t * 1000.0 for _, t in tds]
            diff = max(ms) - min(ms)
            v = "PASS" if diff < TD_DIFF_PASS_MS else ("WARN" if diff < TD_DIFF_FAIL_MS else "FAIL")
            rows.append(("카메라끼리 시간차 차이", "%.2f ms (%s)" % (diff, " / ".join("%.2f" % x for x in ms)),
                         "PASS <%.0f, FAIL ≥%.0f ms" % (TD_DIFF_PASS_MS, TD_DIFF_FAIL_MS), v))
        else:
            rows.append(("카메라끼리 시간차 차이", "시간차 값 없음", "-", "FAIL"))
    if ref_T is not None:
        T0 = res["cams"].get(0, {}).get("T_ci")
        if T0 is None:
            rows.append(("기준 대비 회전·이동(cam0)", "결과에 T_ci 없음", "-", "WARN"))
        else:
            ang = rotation_angle_deg(_R(ref_T), _R(T0))
            pa, pb = cam_position_in_imu(ref_T), cam_position_in_imu(T0)
            dist = math.sqrt(sum((x - y) ** 2 for x, y in zip(pa, pb)))
            rows.append(("기준 대비 회전 차(cam0)", "%.3f°" % ang, "WARN ≥%.0f° (FAIL 없음)" % REF_ROT_WARN_DEG,
                         "WARN" if ang >= REF_ROT_WARN_DEG else "PASS"))
            rows.append(("기준 대비 카메라 위치 차(cam0, IMU 좌표)", "%.1f mm" % (dist * 1000.0),
                         "WARN ≥%.0f cm (FAIL 없음)" % (REF_TRANS_WARN_M * 100), "WARN" if dist >= REF_TRANS_WARN_M else "PASS"))
    overall = "PASS"
    for r in rows:
        if _RANK[r[3]] > _RANK[overall]:
            overall = r[3]
    return overall, rows


def format_report(path, res, overall, rows, ref_path=None):
    lines = ["Kalibr 결과 요약 — %s" % path]
    if ref_path:
        lines.append("기준값: %s" % ref_path)
    lines.append("%-40s %-28s %-26s %s" % ("항목", "값", "기준(임시값)", "판정"))
    for r in rows:
        lines.append("%-40s %-28s %-26s %s" % r)
    for c, d in sorted(res["cams"].items()):
        if d["T_ci"] is not None:
            p = cam_position_in_imu(d["T_ci"])
            lines.append("cam%d: IMU 좌표 카메라 위치 (%.4f, %.4f, %.4f) m, 시간차 %s" % (
                c, p[0], p[1], p[2], "-" if d["timeshift_s"] is None else "%.2f ms" % (d["timeshift_s"] * 1000)))
    lines.append("종합: %s" % overall)
    if overall == "FAIL":
        lines.append("→ 장비 보정 자리로 게시하지 않습니다. 보드 설정(aprilgrid yaml)·camchain·imu.yaml·녹화 움직임을 확인하세요.")
    elif overall == "WARN":
        lines.append("→ 게시는 하지만 WARN 항목을 확인하세요.")
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("results")
    ap.add_argument("--ref", default=None, help="기준 T_cam_imu 가 든 camchain yaml(cam0.T_cam_imu)")
    ap.add_argument("--out", default=None, help="요약을 저장할 파일")
    a = ap.parse_args(argv)
    try:
        with open(a.results, encoding="utf-8", errors="replace") as f:
            text = f.read()
    except OSError as e:
        print("결과 파일을 못 읽음: %s" % e, file=sys.stderr)
        return 3
    res = parse_results(text)
    ref_T = None
    if a.ref:
        try:
            ref_T = load_ref_T_cam_imu(a.ref)
        except Exception as e:  # 기준 파일이 깨져도 요약은 낸다
            print("기준 파일을 못 읽음(%s) — 기준 비교 없이 진행" % e, file=sys.stderr)
        if ref_T is None:
            print("기준 파일에 cam0.T_cam_imu 가 없음 — 기준 비교 없이 진행: %s" % a.ref, file=sys.stderr)
    overall, rows = judge(res, ref_T)
    rep = format_report(a.results, res, overall, rows, a.ref if ref_T is not None else None)
    print(rep, end="")
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(rep)
    return _RANK[overall]


if __name__ == "__main__":
    sys.exit(main())
