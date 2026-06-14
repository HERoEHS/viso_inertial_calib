#!/usr/bin/env python3
"""
AprilGrid 코너 기반 Rolling-Shutter readout time(line delay) 추정기
====================================================================
detect_shutter_from_bag.py 가 "RS냐 GS냐"를 판별했다면, 이 스크립트는
한 발 더 나아가 RS의 핵심 파라미터인 **readout time t_ro(맨 윗줄→맨 아랫줄을
다 읽는 데 걸리는 시간)** 를 Kalibr 없이 근사 추정한다.

────────────────────────────────────────────────────────────────────
왜 코너를 "프레임 간"으로 추적하는가? (단일 프레임이 안 되는 이유)
────────────────────────────────────────────────────────────────────
단일 프레임 안에서 "직선이 기운 정도"만으로 t_ro를 구하려 하면, 보드의 자세
(평면 homography/affine)가 그 shear를 그대로 흡수해버려 분리 불가능하다.
  (fronto-parallel이면 affine, 기울어져도 같은 분모를 공유하는 homography로
   흡수됨 → 잔차가 0이 되어 측정 불가)
따라서 '움직임'이 반드시 필요하다.

────────────────────────────────────────────────────────────────────
추정 원리 (가속도 기반)
────────────────────────────────────────────────────────────────────
한 코너가 영상 행 y에서 노출되는 시각은 frame 시작 + r*y  (r = t_ro/H = 행당 지연).
연속 두 프레임에서 같은 코너의 수평 이동량 dx(y)는 (수평 병진 가정):

    dx(y) ≈ v_i*Δt + (a*Δt*r)*y          (v=픽셀 수평속도, a=픽셀 수평가속도)
          = α_i      +  β_i * y

  → β_i (행에 대한 dx 기울기) = a_i * Δt * r
  → α_i (평균 수평 이동)      = v_i * Δt        ⇒  v_i = α_i/Δt

가속도 a_i 는 속도 v_i 의 시간미분(중앙차분)으로 구한다.
여러 프레임쌍을 모아 다음 1차 관계(원점 통과)를 회귀하면 r 이 나온다:

    β = r * (a*Δt)              (예측변수 X=a*Δt, 응답 β, 기울기 r=t_ro/H)

    ⇒  t_ro = r * H ,   readout ratio = t_ro / Δt

β 와 X 의 강한 상관(R²↑) 자체가 "Rolling Shutter가 맞다"는 추가 증거이며,
GS라면 r≈0, 상관 없음.

장점: AprilGrid 코너는 ID가 있어 프레임 간 매칭이 정확(descriptor 불필요).
한계: 가속도(2차 정보)에 의존 → 다소 노이즈. 강한 모션 프레임은 모션블러로
      태그 검출이 줄어들 수 있음. 회전이 큰 쌍은 robust 회귀로 downweight.
      → '정밀 캘리브레이션'이 아니라 '차수(order-of-magnitude) 추정'으로 볼 것.

사용법
    python3 estimate_readout_time.py \
        [--bag /path/to/vio_calib_20260612_162851] \
        [--topic /edie/.../left/image_gray]   # 미지정 시 모든 Image 토픽
"""

import argparse
import sys

import cv2
import numpy as np

# bag/CDR 읽기 헬퍼는 detect_shutter_from_bag.py 에서 재사용
from detect_shutter_from_bag import find_db3, list_image_topics, read_frames

DEFAULT_BAG = ("/home/higony/ros2_ws/src/edie9/edie_localization/third_party/"
               "viso_inertial_calib/data/calib/vio_calib_20260612_162851")


# ─────────────────────────────────────────────────────────────
# AprilGrid(tag36h11) 코너 검출
# ─────────────────────────────────────────────────────────────
def make_detector():
    """OpenCV aruco 기반 tag36h11 검출기 (서브픽셀 코너 보정 포함)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    # 서브픽셀 코너 보정 → 코너 위치 정확도 향상
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    return cv2.aruco.ArucoDetector(dictionary, params)


def detect_corners(detector, gray):
    """프레임 1장에서 코너 검출.
    반환: dict{ corner_global_id : (u, v) }
      corner_global_id = tag_id*4 + k  (k=0..3, aruco 코너 순서)
    """
    corners, ids, _ = detector.detectMarkers(gray)
    out = {}
    if ids is None:
        return out
    ids = ids.flatten()
    for c, tid in zip(corners, ids):
        pts = c.reshape(4, 2)
        for k in range(4):
            out[int(tid) * 4 + k] = (float(pts[k, 0]), float(pts[k, 1]))
    return out


# ─────────────────────────────────────────────────────────────
# 프레임쌍 분석: dx(y) 의 (기울기 β, 절편/평균이동 α)
# ─────────────────────────────────────────────────────────────
def pair_alpha_beta(prev, cur, H, min_corners=8, min_row_span_frac=0.40):
    """연속 두 프레임의 공통 코너로 dx = α + β*y 회귀.
    반환: (alpha, beta, n, row_span, mean_dy) 또는 None.
      alpha : 평균 수평 이동량 프록시 (px)
      beta  : 행당 수평 이동 기울기 (px/row)  → RS skew 신호
      mean_dy: 평균 수직 이동(회전/스케일 오염 판단용)
    """
    common = prev.keys() & cur.keys()
    if len(common) < min_corners:
        return None
    ys, dxs, dys = [], [], []
    for cid in common:
        u0, v0 = prev[cid]
        u1, v1 = cur[cid]
        ys.append(v0)              # 행 위치는 이전 프레임 기준
        dxs.append(u1 - u0)
        dys.append(v1 - v0)
    ys = np.asarray(ys)
    dxs = np.asarray(dxs)
    dys = np.asarray(dys)

    row_span = ys.max() - ys.min()
    if row_span < min_row_span_frac * H:       # 행 범위가 좁으면 기울기 불안정
        return None

    # 1차 회귀 dx = β*y + α  (이상치 1회 제거)
    coeffs = np.polyfit(ys, dxs, 1)
    resid = dxs - np.polyval(coeffs, ys)
    s = np.std(resid)
    if s > 1e-6:
        keep = np.abs(resid) < 3.0 * s
        if keep.sum() >= min_corners:
            ys, dxs, dys = ys[keep], dxs[keep], dys[keep]
            coeffs = np.polyfit(ys, dxs, 1)
    beta, alpha = float(coeffs[0]), float(coeffs[1])
    # alpha를 화면 중앙 행 기준 평균이동으로 환산 (절편보다 안정적)
    mean_shift = beta * (H * 0.5) + alpha
    return mean_shift, beta, len(ys), float(row_span), float(np.mean(dys))


def analyze_topic(stamps, frames, enc, label, detector,
                  max_rot_dy=6.0):
    n = len(frames)
    H, W = frames[0].shape
    print(f"\n{'='*66}")
    print(f"[{label}]  frames={n}  encoding={enc}  size={W}x{H}")
    if n < 10:
        print("  프레임이 너무 적어 분석 불가")
        return None

    # frame period Δt
    ds = np.diff(stamps) / 1e9
    ds = ds[(ds > 0) & (ds < 1.0)]
    dt = float(np.median(ds)) if len(ds) else None
    if dt:
        print(f"  추정 fps={1.0/dt:.2f}  (frame period Δt={dt*1000:.2f} ms)")

    # 1) 전 프레임 코너 검출
    print("  AprilGrid(tag36h11) 코너 검출 중...", flush=True)
    det = [detect_corners(detector, f) for f in frames]
    n_det = sum(1 for d in det if len(d) >= 8)
    print(f"  코너 8개 이상 검출된 프레임: {n_det}/{n}")
    if n_det < 10:
        print("  >>> 보드(AprilGrid)가 충분히 안 보여 추정 불가")
        return None

    # 2) 프레임쌍별 (alpha, beta)
    alpha = np.full(n - 1, np.nan)
    beta = np.full(n - 1, np.nan)
    dyv = np.full(n - 1, np.nan)
    for i in range(n - 1):
        r = pair_alpha_beta(det[i], det[i + 1], H)
        if r is None:
            continue
        alpha[i], beta[i], _, _, dyv[i] = r[0], r[1], r[2], r[3], r[4]

    valid = ~np.isnan(alpha)
    print(f"  분석 가능한 프레임쌍: {valid.sum()}")
    if valid.sum() < 10 or dt is None:
        print("  >>> 유효 쌍 부족으로 추정 불가")
        return None

    # 3) 속도 v_i = alpha/Δt, 가속도 a_i = 중앙차분
    v = alpha / dt                                  # px/s (NaN 포함)
    aXdt = np.full(n - 1, np.nan)                   # 예측변수 X = a*Δt = Δv/2 (px/s)
    for i in range(1, n - 2):
        if not (np.isnan(v[i - 1]) or np.isnan(v[i + 1])):
            aXdt[i] = (v[i + 1] - v[i - 1]) * 0.5   # = a_i*Δt

    # 4) β = r * (a*Δt) 회귀용 표본 선별
    #    - 회전/스케일 오염(수직이동 큼) 쌍 제외
    #    - 가속이 충분히 큰 쌍만 (신호/잡음비)
    mask = (~np.isnan(beta)) & (~np.isnan(aXdt)) & (np.abs(dyv) < max_rot_dy)
    X = aXdt[mask]                                  # px/s   (가속 X=a·Δt)
    Y = beta[mask]                                  # px/row (skew 기울기)
    A = alpha[mask]                                 # px     (평균 수평 이동≈속도·Δt)

    # ── 결정적 검증: skew(β)가 '가속도'에 비례하나, '속도'에 비례하나? ──
    #   RS        → β ∝ a·Δt  : corr(β, a·Δt) 큼,  corr(β, α) 작음
    #   정적 왜곡 → β ∝ v ∝ α : corr(β, α) 큼,    corr(β, a·Δt) 작음
    #   (rectify remap의 행별 수평배율 차이나 시차(parallax)는 속도 비례)
    corr_acc = float(np.corrcoef(X, Y)[0, 1]) if len(X) > 2 else float("nan")
    corr_vel = float(np.corrcoef(A, Y)[0, 1]) if len(A) > 2 else float("nan")
    # 속도 비례 정적 게인: skew_over_height / shift  (앞 phaseCorrelate의 0.4와 비교)
    big_v = np.abs(A) > max(np.percentile(np.abs(A), 60), 1.0)
    static_gain = float(np.median((Y[big_v] * H) / A[big_v])) if big_v.sum() else float("nan")

    print(f"\n  ── skew 원인 분리 (RS vs 정적왜곡) ──")
    print(f"  corr(β, a·Δt) [RS 신호]      = {corr_acc:+.3f}")
    print(f"  corr(β, α)    [속도비례 정적] = {corr_vel:+.3f}")
    print(f"  속도비례 정적 게인(skew/shift) ≈ {static_gain:+.3f}  "
          f"(phaseCorrelate에서 본 0.4와 비교)")

    # 가속 충분한 표본만 (RS 회귀용)
    thr = np.percentile(np.abs(X), 60) if len(X) else 0.0
    big = np.abs(X) > max(thr, 1.0)
    Xb, Yb = X[big], Y[big]
    print(f"  readout 회귀 표본 수: {len(Xb)} (가속 유의 쌍)")
    if len(Xb) < 8:
        print("  >>> 가속 표본 부족으로 t_ro 추정 불가 (모션이 너무 일정)")
        return None

    # robust 기울기 r = median(β / (a·Δt)) + 원점통과 LS 교차검증
    ratios = Yb / Xb                                # s/row
    r_med = float(np.median(ratios))
    r_mad = float(np.median(np.abs(ratios - r_med))) * 1.4826
    r_ls = float(np.sum(Xb * Yb) / np.sum(Xb * Xb))  # 원점통과 최소제곱

    def to_ms(r):           # r[s/row] → t_ro[ms]
        return r * H * 1e3

    tro_med = to_ms(r_med)
    ratio_med = (r_med * H) / dt        # t_ro/Δt

    print(f"\n  ── (참고) readout time 가정 추정 ──")
    print(f"  [median 법]  t_ro ≈ {abs(tro_med):6.2f} ms (±{abs(to_ms(r_mad)):.1f}),"
          f"  readout ratio ≈ {abs(ratio_med):.3f}")

    # ── 최종 해석: 두 상관 비교가 핵심 ──
    #   RS로 인정하려면: 가속 상관이 (1) 충분히 크고 (2) 속도 상관을 명확히 능가.
    #   (참고: 진짜 RS면 corr_acc는 일관된 부호여야 하며, 속도비례 정적왜곡이
    #    우세하면 GS. 본 데이터는 모든 케이스에서 corr_vel 우세 → GS.)
    rs_dominant = (abs(corr_acc) >= 0.35) and (abs(corr_acc) > 1.3 * abs(corr_vel))
    if rs_dominant:
        tro = abs(tro_med)
        verdict = f"Rolling Shutter 확인 (가속 비례 신호 우세), t_ro ≈ {tro:.1f} ms"
    elif abs(corr_vel) >= abs(corr_acc):
        verdict = ("Global Shutter 판정. skew가 '속도'에 비례(corr_vel 우세) → "
                   "rectify 워프/시차에 의한 정적 왜곡이며 RS 아님. "
                   "(phaseCorrelate의 RS 판정은 이 정적 게인에 의한 위양성)")
        tro_med = 0.0
    else:
        verdict = ("판정 보류: 가속/속도 신호 모두 약함. "
                   "raw 이미지 bag으로 재측정 권장 (rectify 워프 영향 제거).")
        tro_med = 0.0
    print(f"  >>> 해석: {verdict}")

    return dict(label=label, dt_ms=dt * 1e3, corr_acc=corr_acc, corr_vel=corr_vel,
                static_gain=static_gain, tro_ms=tro_med, ratio=ratio_med, n=len(Xb))


def main():
    ap = argparse.ArgumentParser(
        description="AprilGrid 코너 기반 Rolling-Shutter readout time 추정")
    ap.add_argument("--bag", default=DEFAULT_BAG, help="bag 디렉토리 또는 .db3")
    ap.add_argument("--topic", default=None, help="특정 Image 토픽 (미지정 시 전체)")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="처리할 최대 프레임 수(디버그용)")
    args = ap.parse_args()

    db3 = find_db3(args.bag)
    print(f"bag db3: {db3}")
    topics = [args.topic] if args.topic else list_image_topics(db3)
    if not topics:
        print("[ERR] Image 토픽을 찾지 못함")
        sys.exit(1)
    print(f"분석 대상 토픽: {topics}")

    detector = make_detector()
    results = []
    for tp in topics:
        stamps, frames, enc = read_frames(db3, tp, args.max_frames)
        if not frames:
            print(f"[WARN] 프레임 없음: {tp}")
            continue
        res = analyze_topic(stamps, frames, enc, tp, detector)
        if res:
            results.append(res)

    if results:
        print(f"\n{'='*66}\n요약")
        for r in results:
            print(f"  {r['label']}: corr_acc(RS)={r['corr_acc']:+.2f}, "
                  f"corr_vel(정적)={r['corr_vel']:+.2f}, "
                  f"정적게인≈{abs(r['static_gain']):.2f}, t_ro≈{abs(r['tro_ms']):.1f}ms, n={r['n']}")


if __name__ == "__main__":
    main()
