#!/usr/bin/env python3
"""(Docker kalibr:ros1 안에서, 본계산 직전에 실행) Kalibr 소스를 컨테이너 안에서만 고친다(이미지·저장소는 그대로).

  --border 1            표준 AprilTag(1칸 검은 테두리) → blackTagBorder=1. 2 면 Kalibr 기본(2칸) 그대로.
  --timeshift-ms auto   Kalibr 자체 추정(|ω_카메라|·|ω_IMU| 교차상관, IccSensors.findTimeshiftCameraImuPrior)을 그대로 쓴다.
  --timeshift-ms 35     시간차 시작값을 35 ms 로 덮어쓴다(Kalibr 추정 대신).
  --layout TOPIC=NAME   그 카메라 토픽의 모서리를 배치 NAME 으로 다시 붙인다(identity 면 손대지 않음). 카메라마다 줄 수 있다.
                        TOPIC 이 '*' 이면 따로 적지 않은 모든 카메라.
  --gate-px 5           "Before Optimization" 재투영 평균이 이 값을 넘는 카메라가 있으면 최적화 전에 종료 코드 4 로 멈춘다.

모든 바꿔치기는 원문이 그대로 있는지 assert 로 확인한다 — Kalibr 가 바뀌어 원문이 없으면 조용히 넘어가지 않고 멈춘다.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kalibr_layout as kl  # noqa: E402

KALIBR_PY = "/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/python"
ICC_SENSORS = KALIBR_PY + "/kalibr_imu_camera_calibration/IccSensors.py"
MAIN_SCRIPT = KALIBR_PY + "/kalibr_calibrate_imu_camera"
LIB_DIR = os.path.dirname(os.path.abspath(__file__))


def _replace_once(code, old, new, what):
    assert code.count(old) == 1, "Kalibr 원문을 못 찾음(%s) — Kalibr 버전이 바뀌었는지 확인" % what
    return code.replace(old, new)


def patch_sensors(code, border, timeshift_ms, layouts, frame_max_px):
    if border == 1:
        code = _replace_once(
            code,
            "options.showExtractionVideo = showExtraction\n            options.minTagsForValidObs",
            "options.showExtractionVideo = showExtraction\n            options.blackTagBorder = 1\n"
            "            options.minTagsForValidObs",
            "aprilgrid 옵션")
        print("[VISO PATCH] blackTagBorder=1")
    if timeshift_ms != "auto":
        ts = float(timeshift_ms) / 1000.0
        code = _replace_once(
            code,
            "        self.timeshiftCamToImuPrior = shift\n        \n",
            "        self.timeshiftCamToImuPrior = shift\n"
            "        self.timeshiftCamToImuPrior = %r\n"
            "        print(\"[VISO PATCH] 시간차 시작값을 %.2f ms 로 덮어씀(Kalibr 추정 %%.2f ms 대신)\" %% (shift * 1000))\n"
            "        \n" % (ts, ts * 1000),
            "시간차 추정")
        print("[VISO PATCH] 시간차 시작값 %.2f ms 로 덮어쓰기" % (ts * 1000))
    else:
        print("[VISO PATCH] 시간차 시작값 = Kalibr 자체 추정(덮어쓰지 않음)")
    if any(v != "identity" for v in layouts.values()):
        # 모듈 머리에 도구 경로·배치 표를 넣는다
        code = _replace_once(
            code, "import scipy.optimize\n",
            "import scipy.optimize\n"
            "# [VISO PATCH] 모서리 배치 다시 붙이기 — viso_inertial_calib/lib/kalibr_layout.py\n"
            "sys.path.insert(0, %r)\n"
            "import kalibr_layout as _viso_kl\n"
            "_VISO_LAYOUTS = %r\n"
            "_VISO_FRAME_MAX_PX = %r\n" % (LIB_DIR, layouts, frame_max_px),
            "import 줄")
        # 검출만 하고(자세·이상치 거르기는 틀린 배치로 하지 않음) → 다시 붙임 → 바른 모델로 자세·거르기
        old = ("        self.targetObservations = kc.extractCornersFromDataset(self.dataset, self.detector, "
               "multithreading=multithreading)\n")
        new = (
            "        _viso_layout = _VISO_LAYOUTS.get(self.camConfig.getRosTopic(), _VISO_LAYOUTS.get('*', 'identity'))\n"
            "        if _viso_layout == 'identity':\n"
            "    " + old +
            "        else:\n"
            "            _viso_obs = kc.extractCornersFromDataset(self.dataset, self.detector, "
            "multithreading=multithreading, noTransformation=True)\n"
            "            self.targetObservations, _viso_st = _viso_kl.relabel_and_filter(\n"
            "                list(_viso_obs), self.camera.geometry, _viso_layout,\n"
            "                int(targetConfig.getTargetParams()['tagCols']), frame_max_px=_VISO_FRAME_MAX_PX)\n"
            "            print('[VISO PATCH] %s: 모서리 배치 %s 로 다시 붙임 — 장 %d→%d (자세 실패 %d, 평균 %.0f px 넘어 뺀 장 %d), '\n"
            "                  '이상치 모서리 %d/%d 제거(Kalibr 규칙: 평균+2σ 초과 & 0.2 px 초과)' % (\n"
            "                  self.dataset.topic, _viso_layout, _viso_st['frames_in'], _viso_st['frames_out'],\n"
            "                  _viso_st['pnp_fail'], _VISO_FRAME_MAX_PX, _viso_st['frames_big'],\n"
            "                  _viso_st['corners_removed'], _viso_st['corners_in']))\n"
            "            if len(self.targetObservations) == 0:\n"
            "                sm.logFatal('모서리 배치를 다시 붙인 뒤 남은 장이 없습니다')\n"
            "                sys.exit(4)\n")
        code = _replace_once(code, old, new, "모서리 추출")
        print("[VISO PATCH] 모서리 배치: %s" % ", ".join("%s=%s" % kv for kv in sorted(layouts.items())))
    else:
        print("[VISO PATCH] 모서리 배치: 그대로(identity, Kalibr 기본)")
    return code


def patch_main(code, gate_px):
    old = '    print("Before Optimization")\n    print("===================")\n    util.printErrorStatistics(iCal)\n'
    new = old + (
        "    # [VISO PATCH] 발산 관문 — 최적화 전 재투영이 크면 모서리 배치·보드·camchain 이 틀린 것. 최적화해도 안 고쳐진다.\n"
        "    _viso_bad = []\n"
        "    for _ci, _cam in enumerate(iCal.CameraChain.camList):\n"
        "        if len(_cam.allReprojectionErrors) == 0:\n"
        "            _viso_bad.append('cam%%d 모서리 없음' %% _ci)\n"
        "            continue\n"
        "        _e = np.array([np.linalg.norm(_r.error()) for _rs in _cam.allReprojectionErrors for _r in _rs])\n"
        "        _m = float(np.mean(_e))\n"
        "        print('[VISO GATE] cam%%d 최적화 전 재투영 평균 %%.2f px (문턱 %%.1f px)' %% (_ci, _m, %r))\n"
        "        if not (_m <= %r):\n"
        "            _viso_bad.append('cam%%d %%.2f px' %% (_ci, _m))\n"
        "    if _viso_bad:\n"
        "        print('[VISO GATE] 멈춤: ' + ', '.join(_viso_bad) + ' — 최적화로 넘어가지 않습니다(종료 코드 4)')\n"
        "        sys.stdout.flush()\n"
        "        sys.exit(4)\n" % (gate_px, gate_px))
    return _replace_once(code, old, new, "Before Optimization")


def parse_layouts(items):
    out = {}
    for it in items or []:
        topic, sep, name = it.rpartition("=")
        if not sep or not topic or name not in kl.LAYOUTS:
            raise SystemExit("--layout 은 TOPIC=NAME 꼴, NAME 은 %s 중 하나: %r" % (", ".join(kl.LAYOUT_NAMES), it))
        out[topic] = name
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--border", type=int, choices=(1, 2), default=1)
    ap.add_argument("--timeshift-ms", default="auto")
    ap.add_argument("--layout", action="append", default=[])
    ap.add_argument("--gate-px", type=float, default=5.0)
    ap.add_argument("--frame-max-px", type=float, default=kl.FRAME_MAX_MEAN_PX)
    ap.add_argument("--sensors", default=ICC_SENSORS, help="(시험용) IccSensors.py 경로")
    ap.add_argument("--main-script", default=MAIN_SCRIPT, help="(시험용) kalibr_calibrate_imu_camera 경로")
    a = ap.parse_args(argv)
    if a.timeshift_ms != "auto":
        float(a.timeshift_ms)   # 숫자가 아니면 여기서 멈춘다
    layouts = parse_layouts(a.layout)
    with open(a.sensors, encoding="utf-8") as f:
        code = f.read()
    code = patch_sensors(code, a.border, a.timeshift_ms, layouts, a.frame_max_px)
    with open(a.sensors, "w", encoding="utf-8") as f:
        f.write(code)
    with open(a.main_script, encoding="utf-8") as f:
        code = f.read()
    code = patch_main(code, a.gate_px)
    with open(a.main_script, "w", encoding="utf-8") as f:
        f.write(code)
    print("[VISO PATCH] 발산 관문: 최적화 전 재투영 평균 > %.1f px 이면 멈춤" % a.gate_px)
    return 0


if __name__ == "__main__":
    sys.exit(main())
