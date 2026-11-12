#!/usr/bin/env python3
"""(Docker kalibr:ros1 안에서 실행) 본계산 전 모서리 배치 자동 점검.

Kalibr 검출기 그대로 프레임 약 20장(녹화 전체에 고르게, 태그가 많이 보이는 장)에서 모서리를 뽑고,
태그 안 모서리 배치 8가지마다 장별 PnP 재투영 평균 → 그 중앙값을 카메라마다 표로 낸다.
채택 규칙(kalibr_layout.select_layout): 1등 < 2 px 이고 2등과 10배 넘게 차이(임시값).

종료 코드: 0 = 모든 카메라에서 채택, 3 = 어느 카메라든 못 고름(본계산을 시작하지 말 것), 1 = 실행 오류
출력: --out-choice 에 "토픽 배치" 줄, --out-table 에 표, --out-json 에 장별 값
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kalibr_layout as kl  # noqa: E402

import numpy as np  # noqa: E402
import aslam_cv as acv  # noqa: E402
import aslam_cameras_april as acv_april  # noqa: E402
import kalibr_common as kc  # noqa: E402


def make_detector(camera, targetConfig, border):
    """IccSensors.setupCalibrationTarget(97~116행)과 같은 설정. 단 이상치 거르기는 끈다 —
    틀린 배치로 구한 자세 기준 거르기는 멀쩡한 모서리를 빼므로 배치마다 같은 모서리로 비교한다."""
    if targetConfig.getTargetType() != "aprilgrid":
        raise SystemExit("모서리 배치 점검은 aprilgrid 보드만 지원합니다(지금: %s)" % targetConfig.getTargetType())
    p = targetConfig.getTargetParams()
    options = acv_april.AprilgridOptions()
    options.showExtractionVideo = False
    options.minTagsForValidObs = int(np.max([p["tagRows"], p["tagCols"]]) + 1)
    if border == 1:
        options.blackTagBorder = 1      # pipeline 의 '표준 AprilTag(1칸 테두리)' 답과 같게
    grid = acv_april.GridCalibrationTargetAprilgrid(p["tagRows"], p["tagCols"], p["tagSize"], p["tagSpacing"], options)
    dopt = acv.GridDetectorOptions()
    dopt.imageStepping = False
    dopt.plotCornerReprojection = False
    dopt.filterCornerOutliers = False
    n_corners = 4 * int(p["tagRows"]) * int(p["tagCols"])
    return acv.GridDetector(camera.geometry, grid, dopt), int(p["tagCols"]), n_corners


def pick_frames(ds, det, n_frames, n_corners, tries=6, good_frac=0.7):
    """녹화를 n_frames 구간으로 나눠 구간마다 tries 장을 보고 모서리가 가장 많은 장 하나를 고른다
    (전체의 good_frac 이상이 보이면 바로 채택). 반환 [(번호, 시각, 모서리 번호들, 영상 좌표)]."""
    n = ds.numImages()
    picks = []
    for s in range(n_frames):
        a, b = int(s * n / n_frames), int((s + 1) * n / n_frames)
        if b <= a:
            continue
        cands = sorted(set(int(x) for x in np.linspace(a, b - 1, num=min(tries, b - a))))
        best = None
        for i in cands:
            ts, img = ds.getImage(ds.indices[i])
            ok, obs = det.findTargetNoTransformation(ts, np.array(img))
            if not ok:
                continue
            idx = [int(q) for q in obs.getCornersIdx()]
            if best is None or len(idx) > len(best[2]):
                best = (i, ts, idx, np.array(obs.getCornersImageFrame(), dtype=float).reshape(-1, 2), obs)
            if len(idx) >= good_frac * n_corners:
                break
        if best is not None:
            picks.append(best)
    return picks


def evaluate(picks, geometry, tag_cols):
    """배치마다 장별 PnP 재투영 평균(px). 반환 {배치: [px, ...]}."""
    cols = 2 * tag_cols
    res = {name: [] for name in kl.LAYOUT_NAMES}
    for (_i, _ts, idx, pts, obs) in picks:
        for name in kl.LAYOUT_NAMES:
            kl.apply_layout(obs, cols, name, idx, pts)
            ok, T = geometry.estimateTransformation(obs)
            if not ok:
                continue
            obs.set_T_t_c(T)
            e = kl.corner_reprojection_errors(obs, geometry)
            res[name].append(float(np.mean(e)))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--cam", required=True, help="camchain yaml")
    ap.add_argument("--target", required=True, help="aprilgrid yaml")
    ap.add_argument("--border", type=int, choices=(1, 2), default=1)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--out-json", default=None)
    ap.add_argument("--out-table", default=None)
    ap.add_argument("--out-choice", default=None)
    a = ap.parse_args()

    chain = kc.CameraChainParameters(a.cam)
    targetConfig = kc.CalibrationTargetParameters(a.target)
    per_cam_tab, out_json, choices, msgs = [], {}, [], []
    all_ok = True
    for camNr in range(chain.numCameras()):
        camConfig = chain.getCameraParameters(camNr)
        topic = camConfig.getRosTopic()
        camera = kc.AslamCamera.fromParameters(camConfig)
        det, tag_cols, n_corners = make_detector(camera, targetConfig, a.border)
        ds = kc.BagImageDatasetReader(a.bag, topic)
        picks = pick_frames(ds, det, a.frames, n_corners)
        res = evaluate(picks, camera.geometry, tag_cols)
        med = {k: (float(np.median(v)) if v else None) for k, v in res.items()}
        cnt = {k: len(v) for k, v in res.items()}
        choice, why, detail = kl.select_layout(med, counts=cnt)
        name = "cam%d" % camNr
        per_cam_tab.append((name, {k: (med[k], cnt[k]) for k in kl.LAYOUT_NAMES}))
        msgs.append("%s %s: 점검 %d장(영상 %d장 중, 모서리 중앙 %d개/%d) → %s" % (
            name, topic, len(picks), ds.numImages(),
            int(np.median([len(p[2]) for p in picks])) if picks else 0, n_corners,
            why if choice else "못 고름 — " + why))
        out_json[name] = {"topic": topic, "frames": [int(p[0]) for p in picks],
                          "n_corners": [len(p[2]) for p in picks],
                          "per_frame_px": res, "median_px": med, "choice": choice, "reason": why, "detail": detail}
        if choice is None:
            all_ok = False
        else:
            choices.append("%s %s" % (topic, choice))
        print(msgs[-1], flush=True)

    table = ("모서리 배치 점검 — 장별 PnP 재투영 평균의 중앙값(px). 채택: 1등 < %.1f px 이고 2등 ÷ 1등 > %.0f (임시값)\n"
             % (kl.LAYOUT_ACCEPT_MAX_PX, kl.LAYOUT_ACCEPT_RATIO)) + kl.format_table(per_cam_tab) + "\n" + "\n".join(msgs) + "\n"
    print("")
    print(table)
    if a.out_table:
        with open(a.out_table, "w", encoding="utf-8") as f:
            f.write(table)
    if a.out_json:
        with open(a.out_json, "w", encoding="utf-8") as f:
            json.dump(out_json, f, indent=1)
    if a.out_choice:
        with open(a.out_choice, "w", encoding="utf-8") as f:
            f.write("\n".join(choices) + ("\n" if choices else ""))
    return 0 if all_ok else 3


if __name__ == "__main__":
    sys.exit(main())
