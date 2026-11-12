"""Kalibr AprilGrid 모서리 배치(태그 안 네 모서리를 어느 자리에 붙이나) 도구 — 순수 파이썬.

왜 필요한가(2026-11-12 .224 사고):
  Kalibr 검출기는 태그를 찾은 뒤 네 모서리에 "왼쪽아래·오른쪽아래·오른쪽위·왼쪽위" 번호를 붙인다.
  그런데 보드 그림이 Kalibr 가 기대하는 방향과 다르면 번호가 태그마다 엇갈린다(.224 녹화는 180°,
  옛 녹화는 좌우 뒤집기). 태그 번호(ID)는 맞아서 검출은 다 되지만 장마다 자세가 틀려 49 px 로 발산했다.
  어긋나는 방식이 녹화·보드마다 다를 수 있어, 본계산 전에 8가지 배치를 시험해 고른다.

이 파일은 Docker(kalibr:ros1, python 3.8)와 PC(python 3.10) 양쪽에서 쓴다 → 3.8 문법만, numpy 는 필요한 함수 안에서만.
"""
import math

# ── 모서리 번호 규칙 (Kalibr GridCalibrationTargetAprilgrid.cpp 290~296행) ──────────────────
#   점 격자는 (2·tagRows) × (2·tagCols). 점 번호 i → 행 r = i // cols, 열 c = i % cols (cols = 2·tagCols).
#   태그 하나의 네 점은 태그 칸 안 자리 (r%2, c%2) 로 구별된다. 보드 좌표(x 오른쪽, y 위쪽)로
#     (0,0)=왼쪽아래  (0,1)=오른쪽아래  (1,1)=오른쪽위  (1,0)=왼쪽위   ← 반시계 순서
_CCW = [(0, 0), (0, 1), (1, 1), (1, 0)]


def _rot(k):
    """보드 좌표에서 태그 중심을 축으로 반시계 k×90° 돌린 자리로 옮기는 표."""
    return {_CCW[i]: _CCW[(i + k) % 4] for i in range(4)}


# 8가지 배치 = 회전 4 × 뒤집기 2. 값은 "검출기가 붙인 자리 → 바르게 붙일 자리".
# (옛 조사 run2_detcheck_old/perm_old.py 의 이름·정의와 같다: 그대로/90/180/270/좌우/상하/대각/반대각)
LAYOUTS = {
    "identity": _rot(0),                                   # 그대로(패치 없음)
    "rot90": _rot(1),                                      # 반시계 90°
    "rot180": _rot(2),                                     # 180° (.224 녹화)
    "rot270": _rot(3),                                     # 반시계 270°
    "flip_lr": {(a, b): (a, 1 - b) for a, b in _CCW},      # 좌우 뒤집기(옛 녹화)
    "flip_ud": {(a, b): (1 - a, b) for a, b in _CCW},      # 상하 뒤집기
    "transpose": {(a, b): (b, a) for a, b in _CCW},        # 대각선(왼쪽아래–오른쪽위) 축 뒤집기
    "antitranspose": {(a, b): (1 - b, 1 - a) for a, b in _CCW},  # 반대 대각선 축 뒤집기
}
LAYOUT_NAMES = list(LAYOUTS.keys())
LAYOUT_KO = {"identity": "그대로", "rot90": "90°", "rot180": "180°", "rot270": "270°",
             "flip_lr": "좌우", "flip_ud": "상하", "transpose": "대각", "antitranspose": "반대각"}

# ── 배치 채택 규칙 (임시값 — 녹화 두 개(.224·옛)로만 정함, 자료가 늘면 다시 볼 것) ──────────
LAYOUT_ACCEPT_MAX_PX = 2.0     # 1등 배치의 재투영(프레임별 평균의 중앙값)이 이보다 작아야 한다
LAYOUT_ACCEPT_RATIO = 10.0     # 2등 ÷ 1등 이 이보다 커야 한다(애매하면 고르지 않는다)
LAYOUT_MIN_FRAMES = 5          # 이 배치로 자세를 구한 장이 이보다 적으면 판단하지 않는다

# ── 모서리 이상치 거르기 (Kalibr GridDetector.cpp 165~193행·GridDetector.hpp 28~29행과 같은 규칙) ──
CORNER_SIGMA_THRESHOLD = 2.0   # e > 평균 + 2.0 × 표준편차 이고
CORNER_MIN_REPROJ_PX = 0.2     # e > 0.2 px 이면 그 모서리를 뺀다
# Kalibr 에 없는 추가(임시값): 다시 붙이고 걸러낸 뒤에도 프레임 평균 재투영이 이보다 크면 그 장을 통째로 뺀다.
# 바른 배치면 장마다 약 1 px 이고, 잘못 검출된 장(.224 의 2086·2087장)은 약 90 px 이었다.
FRAME_MAX_MEAN_PX = 10.0


def remap_index(i, cols, layout):
    """검출기가 붙인 점 번호 i 를 배치 layout 에 맞는 번호로 바꾼다. cols = 2·tagCols."""
    mp = LAYOUTS[layout]
    r, c = divmod(int(i), int(cols))
    a, b = mp[(r % 2, c % 2)]
    return (2 * (r // 2) + a) * cols + 2 * (c // 2) + b


def inverse_layout(layout):
    """layout 을 되돌리는 배치 이름(시험·모의용)."""
    mp = LAYOUTS[layout]
    inv = {v: k for k, v in mp.items()}
    for name, cand in LAYOUTS.items():
        if cand == inv:
            return name
    raise ValueError(layout)


def select_layout(medians, max_px=LAYOUT_ACCEPT_MAX_PX, min_ratio=LAYOUT_ACCEPT_RATIO,
                  counts=None, min_frames=LAYOUT_MIN_FRAMES):
    """배치별 재투영 중앙값 표를 보고 채택할 배치를 고른다.

    medians: {배치 이름: px 또는 None(자세를 못 구함)}, counts: {배치 이름: 쓸 수 있었던 장 수}(없으면 검사 안 함)
    반환: (채택 배치 이름 또는 None, 설명 문장, 상세 dict)
    """
    vals = []
    for name in LAYOUT_NAMES:
        v = medians.get(name)
        if v is None or (isinstance(v, float) and math.isnan(v)):
            continue
        if counts is not None and counts.get(name, 0) < min_frames:
            continue
        vals.append((float(v), name))
    vals.sort()
    detail = {"best": None, "best_px": None, "second": None, "second_px": None, "ratio": None}
    if not vals:
        return None, "쓸 수 있는 배치가 없음(모든 배치에서 자세를 구한 장이 %d장 미만)" % min_frames, detail
    best_px, best = vals[0]
    detail.update(best=best, best_px=best_px)
    if len(vals) > 1:
        second_px, second = vals[1]
        ratio = second_px / best_px if best_px > 0 else float("inf")
        detail.update(second=second, second_px=second_px, ratio=ratio)
    else:
        second_px, second, ratio = None, None, float("inf")
    if best_px >= max_px:
        return None, "1등 %s 도 %.2f px 로 %.1f px 이상 — 어느 배치도 맞지 않음" % (best, best_px, max_px), detail
    if ratio <= min_ratio:
        return None, ("1등 %s %.2f px 와 2등 %s %.2f px 의 차이가 %.1f배로 %.0f배 이하 — 애매함"
                      % (best, best_px, second, second_px, ratio, min_ratio)), detail
    if second is None:
        return best, "%s 채택(%.2f px, 비교할 2등 없음)" % (best, best_px), detail
    return best, ("%s 채택(%.2f px, 2등 %s %.2f px, %.0f배)" % (best, best_px, second, second_px, ratio)), detail


def outlier_keep_mask(errors, sigma=CORNER_SIGMA_THRESHOLD, min_px=CORNER_MIN_REPROJ_PX):
    """Kalibr GridDetector 와 같은 규칙으로 남길 모서리를 True 로 표시한다(한 프레임 안의 재투영 오차 목록)."""
    n = len(errors)
    if n == 0:
        return []
    mean = sum(errors) / n
    std = math.sqrt(sum((e - mean) ** 2 for e in errors) / n)   # C++ 과 같이 n 으로 나눈다
    return [not (e > mean + sigma * std and e > min_px) for e in errors]


def format_table(per_cam):
    """per_cam: [(카메라 이름, {배치: (중앙값 px 또는 None, 장 수)})] → 표 문자열."""
    head = "%-22s" % "배치"
    for cam, _ in per_cam:
        head += " | %-20s" % cam
    lines = [head, "-" * len(head)]
    for name in LAYOUT_NAMES:
        row = "%-22s" % ("%s(%s)" % (name, LAYOUT_KO[name]))
        for _, tab in per_cam:
            v, n = tab.get(name, (None, 0))
            row += " | %-20s" % (("%.2f px (%d장)" % (v, n)) if v is not None else "자세 못 구함")
        lines.append(row)
    return "\n".join(lines)


# ── Kalibr 관측 객체 다루기(덕 타이핑: Kalibr GridCalibrationTargetObservation·CameraGeometry 와 같은 이름의 메서드) ──
def corner_reprojection_errors(obs, geometry):
    """관측의 현재 자세(T_t_c)로 보드 점을 영상에 비춰 모서리마다 오차(px)를 낸다. 순서 = obs.getCornersIdx()."""
    import numpy as np
    T_c_t = np.linalg.inv(np.array(obs.T_t_c().T(), dtype=float))
    tp = np.array(obs.getCornersTargetFrame(), dtype=float).reshape(-1, 3)
    ip = np.array(obs.getCornersImageFrame(), dtype=float).reshape(-1, 2)
    pc = tp @ T_c_t[:3, :3].T + T_c_t[:3, 3]
    uv = np.array([np.asarray(geometry.euclideanToKeypoint(np.array(q, dtype=float))).ravel()[:2]
                   for q in pc]).reshape(-1, 2)
    return [float(e) for e in np.linalg.norm(uv - ip, axis=1)]


def apply_layout(obs, cols, layout, idx=None, pts=None):
    """관측의 모서리 번호를 layout 대로 다시 붙인다(idx·pts 를 주면 그 원래 값에서 다시 붙인다)."""
    import numpy as np
    cur = [int(i) for i in obs.getCornersIdx()]
    if idx is None:
        idx = cur
        pts = np.array(obs.getCornersImageFrame(), dtype=float).reshape(-1, 2)
    for i in cur:
        obs.removeImagePoint(int(i))
    for i, p in zip(idx, pts):
        obs.updateImagePoint(int(remap_index(i, cols, layout)), np.array(p, dtype=float))


def relabel_and_filter(observations, geometry, layout, tag_cols,
                       sigma=CORNER_SIGMA_THRESHOLD, min_px=CORNER_MIN_REPROJ_PX,
                       frame_max_px=FRAME_MAX_MEAN_PX):
    """본계산용: 검출만 된 관측들(자세 없음)을 layout 대로 다시 붙이고, 바른 모델로 자세를 구한 뒤
    Kalibr 와 같은 규칙으로 모서리 이상치를 거른다. 반환 (남은 관측 목록, 통계 dict).

    Kalibr 와 다른 점(일부러): 거른 뒤 자세를 한 번 더 구한다(Kalibr 는 안 구함 — 남은 모서리와 맞는 자세가 스플라인 시작값에
    더 낫다), 그리고 그래도 프레임 평균이 frame_max_px 를 넘는 장은 뺀다(Kalibr imu-cam 에는 장 단위 거르기가 없음).
    """
    cols = 2 * int(tag_cols)
    kept = []
    st = {"frames_in": len(observations), "pnp_fail": 0, "frames_big": 0,
          "corners_in": 0, "corners_removed": 0}
    for obs in observations:
        apply_layout(obs, cols, layout)
        st["corners_in"] += len(list(obs.getCornersIdx()))
        ok, T = geometry.estimateTransformation(obs)
        if not ok:
            st["pnp_fail"] += 1
            continue
        obs.set_T_t_c(T)
        err = corner_reprojection_errors(obs, geometry)
        keep = outlier_keep_mask(err, sigma, min_px)
        cur = [int(i) for i in obs.getCornersIdx()]
        removed = [i for i, k in zip(cur, keep) if not k]
        if removed:
            for i in removed:
                obs.removeImagePoint(i)
            st["corners_removed"] += len(removed)
            ok, T = geometry.estimateTransformation(obs)
            if not ok:
                st["pnp_fail"] += 1
                continue
            obs.set_T_t_c(T)
            err = corner_reprojection_errors(obs, geometry)
        if err and sum(err) / len(err) > frame_max_px:
            st["frames_big"] += 1
            continue
        kept.append(obs)
    st["frames_out"] = len(kept)
    return kept, st
