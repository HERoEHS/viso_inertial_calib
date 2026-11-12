#!/usr/bin/env python3
"""lib/kalibr_layout.py·kalibr_quality.py·kalibr_patch.py 시험 — Docker·Kalibr 없이 돈다(numpy 필요).

모의 관측(FakeObs)·모의 핀홀 카메라(FakeGeometry)는 Kalibr 객체와 같은 이름의 메서드만 흉내 낸다.
"""
import math
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "lib"))
import kalibr_layout as kl  # noqa: E402
import kalibr_quality as kq  # noqa: E402
import kalibr_patch as kp  # noqa: E402

DATA = os.path.join(HERE, "data", "kalibr")
TAG_ROWS, TAG_COLS, TAG_SIZE, TAG_SPACING = 6, 6, 0.024, 0.3
COLS = 2 * TAG_COLS


def target_point(i):
    """Kalibr aprilgrid 점 번호 → 보드 좌표(x 오른쪽, y 위쪽, m). 점 (r, c) 는 태그 (r//2, c//2) 의 (r%2, c%2) 자리."""
    r, c = divmod(i, COLS)
    pitch = TAG_SIZE * (1 + TAG_SPACING)
    return np.array([(c // 2) * pitch + (c % 2) * TAG_SIZE, (r // 2) * pitch + (r % 2) * TAG_SIZE, 0.0])


class FakeT:
    def __init__(self, T):
        self._T = np.array(T, dtype=float)

    def T(self):
        return self._T


class FakeObs:
    def __init__(self, pts):
        self.pts = dict(pts)            # 번호 → 영상 좌표
        self._T = FakeT(np.eye(4))

    def getCornersIdx(self):
        return sorted(self.pts)

    def getCornersImageFrame(self):
        return np.array([self.pts[i] for i in sorted(self.pts)])

    def getCornersTargetFrame(self):
        return np.array([target_point(i) for i in sorted(self.pts)])

    def removeImagePoint(self, i):
        self.pts.pop(int(i), None)

    def updateImagePoint(self, i, p):
        self.pts[int(i)] = np.array(p, dtype=float)

    def T_t_c(self):
        return self._T

    def set_T_t_c(self, T):
        self._T = T


class FakeGeometry:
    """핀홀(왜곡 없음). estimateTransformation 은 평면 호모그래피로 자세를 구한다(오차 없는 자료면 정확)."""
    def __init__(self, fx=430.0, fy=430.0, cx=320.0, cy=240.0):
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])

    def euclideanToKeypoint(self, p):
        q = self.K @ np.asarray(p, dtype=float)
        return q[:2] / q[2]

    def estimateTransformation(self, obs):
        idx = obs.getCornersIdx()
        if len(idx) < 4:
            return False, None
        X = obs.getCornersTargetFrame()[:, :2]
        uv = obs.getCornersImageFrame()
        xn = (np.linalg.inv(self.K) @ np.c_[uv, np.ones(len(uv))].T).T
        A = []
        for (Xx, Xy), (x, y, _) in zip(X, xn):
            A.append([Xx, Xy, 1, 0, 0, 0, -x * Xx, -x * Xy, -x])
            A.append([0, 0, 0, Xx, Xy, 1, -y * Xx, -y * Xy, -y])
        H = np.linalg.svd(np.array(A))[2][-1].reshape(3, 3)
        lam = 1.0 / np.linalg.norm(H[:, 0])
        if H[2, 2] * lam < 0:
            lam = -lam
        r1, r2, t = lam * H[:, 0], lam * H[:, 1], lam * H[:, 2]
        R = np.c_[r1, r2, np.cross(r1, r2)]
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        T_c_t = np.eye(4)
        T_c_t[:3, :3], T_c_t[:3, 3] = R, t
        return True, FakeT(np.linalg.inv(T_c_t))


def true_pose():
    """보드가 카메라 앞 약 0.3 m 에 약간 기울어 있는 자세(T_c_t)."""
    a, b = math.radians(20), math.radians(-10)
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    T = np.eye(4)
    T[:3, :3] = Rx @ Ry @ np.diag([1, -1, -1])    # 보드 y 위쪽 → 영상 v 아래쪽
    T[:3, 3] = [-0.08, 0.08, 0.30]
    return T


def detected_obs(geom, mislabel, drop_every=0):
    """정답 번호 j 의 영상 점을, 검출기가 배치 mislabel 로 엇갈려 붙였다고 흉내 낸 관측(번호 = mislabel 의 역배치)."""
    T = true_pose()
    inv = kl.inverse_layout(mislabel)
    pts = {}
    for j in range(4 * TAG_ROWS * TAG_COLS):
        if drop_every and j % drop_every == 0:
            continue
        pc = T[:3, :3] @ target_point(j) + T[:3, 3]
        pts[kl.remap_index(j, COLS, inv)] = geom.euclideanToKeypoint(pc)
    return FakeObs(pts)


class TestRemap(unittest.TestCase):
    def test_identity(self):
        for i in range(144):
            self.assertEqual(kl.remap_index(i, COLS, "identity"), i)

    def test_rot180_equals_fix180_formula(self):
        # run3_fix180/kalibr_patch.py 의 식(같은 녹화에서 1.14 px 로 수렴한 식)과 같아야 한다
        for i in range(144):
            r, c = divmod(i, COLS)
            j = (2 * (r // 2) + 1 - r % 2) * COLS + 2 * (c // 2) + 1 - c % 2
            self.assertEqual(kl.remap_index(i, COLS, "rot180"), j)

    def test_each_layout_is_permutation_inside_tag(self):
        for name in kl.LAYOUT_NAMES:
            out = [kl.remap_index(i, COLS, name) for i in range(144)]
            self.assertEqual(sorted(out), list(range(144)), name)
            for i, j in zip(range(144), out):     # 같은 태그 안에서만 움직인다
                self.assertEqual((i // COLS // 2, i % COLS // 2), (j // COLS // 2, j % COLS // 2), name)

    def test_geometric_meaning(self):
        # 태그 칸 안 자리 (a, b) = (행%2, 열%2) → 보드 좌표 x = b, y = a (y 위쪽). 각 배치가 기대한 움직임인지
        expect = {
            "identity": lambda x, y: (x, y),
            "rot90": lambda x, y: (1 - y, x),          # 태그 중심 기준 반시계 90°
            "rot180": lambda x, y: (1 - x, 1 - y),
            "rot270": lambda x, y: (y, 1 - x),
            "flip_lr": lambda x, y: (1 - x, y),         # 좌우
            "flip_ud": lambda x, y: (x, 1 - y),         # 상하
            "transpose": lambda x, y: (y, x),
            "antitranspose": lambda x, y: (1 - y, 1 - x),
        }
        base = 2 * COLS * 2 + 2 * 3      # 아무 태그(행 2, 열 3)의 왼쪽아래 점
        for name, f in expect.items():
            for a in (0, 1):
                for b in (0, 1):
                    j = kl.remap_index(base + a * COLS + b, COLS, name)
                    a2, b2 = (j // COLS) % 2, (j % COLS) % 2
                    self.assertEqual((b2, a2), f(b, a), "%s (%d,%d)" % (name, a, b))

    def test_named_corners(self):
        BL, BR, TR, TL = 0, 1, COLS + 1, COLS
        self.assertEqual(kl.remap_index(BL, COLS, "rot90"), BR)
        self.assertEqual(kl.remap_index(BR, COLS, "rot90"), TR)
        self.assertEqual(kl.remap_index(BL, COLS, "flip_lr"), BR)
        self.assertEqual(kl.remap_index(TL, COLS, "flip_lr"), TR)
        self.assertEqual(kl.remap_index(BL, COLS, "flip_ud"), TL)

    def test_inverse_and_composition(self):
        for name in kl.LAYOUT_NAMES:
            inv = kl.inverse_layout(name)
            for i in range(144):
                self.assertEqual(kl.remap_index(kl.remap_index(i, COLS, name), COLS, inv), i)
        self.assertEqual(kl.inverse_layout("rot90"), "rot270")
        self.assertEqual(kl.inverse_layout("flip_lr"), "flip_lr")
        for i in range(144):
            j = i
            for _ in range(4):
                j = kl.remap_index(j, COLS, "rot90")
            self.assertEqual(j, i)

    def test_non_square_board(self):
        cols = 2 * 4   # 4열 보드
        self.assertEqual(kl.remap_index(0, cols, "rot180"), cols + 1)
        self.assertEqual(kl.remap_index(cols + 1, cols, "rot180"), 0)


class TestSelect(unittest.TestCase):
    def test_224_picks_rot180(self):
        # 11-12 조사값(run2_detcheck: 그대로 46.9, 180° 0.75, 90°·뒤집기 27~36)
        med = {"identity": 46.9, "rot90": 27.1, "rot180": 0.75, "rot270": 30.2,
               "flip_lr": 33.0, "flip_ud": 35.9, "transpose": 28.4, "antitranspose": 29.9}
        choice, why, d = kl.select_layout(med)
        self.assertEqual(choice, "rot180", why)
        self.assertGreater(d["ratio"], 10)

    def test_old_picks_flip_lr(self):
        med = {"identity": 36.3, "rot90": 30.0, "rot180": 32.9, "rot270": 31.0,
               "flip_lr": 0.77, "flip_ud": 34.0, "transpose": 29.0, "antitranspose": 33.0}
        self.assertEqual(kl.select_layout(med)[0], "flip_lr")

    def test_identity_chosen_means_no_patch(self):
        med = {n: 30.0 for n in kl.LAYOUT_NAMES}
        med["identity"] = 0.8
        self.assertEqual(kl.select_layout(med)[0], "identity")

    def test_stop_when_nothing_fits(self):
        med = {n: 20.0 + k for k, n in enumerate(kl.LAYOUT_NAMES)}
        med["rot180"] = 2.5                       # 2 px 이상
        choice, why, _ = kl.select_layout(med)
        self.assertIsNone(choice)
        self.assertIn("어느 배치도", why)

    def test_stop_when_ambiguous(self):
        med = {n: 30.0 for n in kl.LAYOUT_NAMES}
        med["rot180"], med["flip_lr"] = 0.9, 5.0  # 5.6배 → 애매
        choice, why, _ = kl.select_layout(med)
        self.assertIsNone(choice)
        self.assertIn("애매", why)

    def test_ratio_exactly_10_is_not_enough(self):
        med = {n: 30.0 for n in kl.LAYOUT_NAMES}
        med["rot180"], med["flip_lr"] = 1.0, 10.0
        self.assertIsNone(kl.select_layout(med)[0])

    def test_too_few_frames(self):
        med = {n: 30.0 for n in kl.LAYOUT_NAMES}
        med["rot180"] = 0.7
        cnt = {n: 20 for n in kl.LAYOUT_NAMES}
        cnt["rot180"] = 3
        self.assertNotEqual(kl.select_layout(med, counts=cnt)[0], "rot180")
        self.assertIsNone(kl.select_layout({n: None for n in kl.LAYOUT_NAMES})[0])

    def test_table_has_all_layouts(self):
        tab = kl.format_table([("cam0", {"rot180": (0.75, 20)}), ("cam1", {})])
        for n in kl.LAYOUT_NAMES:
            self.assertIn(n, tab)
        self.assertIn("0.75 px (20장)", tab)


class TestOutlierMask(unittest.TestCase):
    def test_kalibr_rule(self):
        e = [1.0] * 9 + [10.0]        # 평균 1.9, 표준편차 2.7 → 문턱 7.3
        self.assertEqual(kl.outlier_keep_mask(e), [True] * 9 + [False])

    def test_min_px_floor(self):
        e = [0.01] * 9 + [0.15]       # 평균+2σ 는 넘지만 0.2 px 이하 → 남긴다(Kalibr filterCornerMinReprojError)
        self.assertEqual(kl.outlier_keep_mask(e), [True] * 10)

    def test_population_std(self):
        e = [1.0, 1.0, 1.0, 1.0, 10.0]   # 평균 2.8, 모집단 σ 3.6 → 문턱 정확히 10.0 → 넘지 않음(>) → 남김
        self.assertEqual(kl.outlier_keep_mask(e), [True] * 5)
        self.assertEqual(kl.outlier_keep_mask([]), [])


class TestSyntheticLayouts(unittest.TestCase):
    """모의 카메라로 '검출기가 배치 L 로 엇갈려 붙인' 관측을 만들고, 8가지 시험이 L 을 고르는지·다시 붙이면 맞는지."""
    def setUp(self):
        self.geom = FakeGeometry()

    def errors_by_layout(self, obs):
        idx = obs.getCornersIdx()
        pts = obs.getCornersImageFrame()
        out = {}
        for name in kl.LAYOUT_NAMES:
            kl.apply_layout(obs, COLS, name, idx, pts)
            ok, T = self.geom.estimateTransformation(obs)
            obs.set_T_t_c(T)
            out[name] = float(np.mean(kl.corner_reprojection_errors(obs, self.geom)))
        return out

    def test_every_mislabel_is_found(self):
        for L in kl.LAYOUT_NAMES:
            obs = detected_obs(self.geom, L)
            med = self.errors_by_layout(obs)
            choice, why, _ = kl.select_layout(med)
            self.assertEqual(choice, L, "%s: %s %s" % (L, why, med))
            self.assertLess(med[L], 1e-6)

    def test_relabel_and_filter_restores_and_removes_outlier(self):
        geom = self.geom
        obs = detected_obs(geom, "rot180")
        # 모서리 하나를 8 px 밀어 진짜 이상치로 만든다(정답 번호 50 의 점)
        bad = kl.remap_index(50, COLS, kl.inverse_layout("rot180"))
        obs.pts[bad] = obs.pts[bad] + np.array([8.0, 0.0])
        kept, st = kl.relabel_and_filter([obs], geom, "rot180", TAG_COLS)
        self.assertEqual(len(kept), 1)
        self.assertEqual(st["corners_removed"], 1)
        self.assertNotIn(50, kept[0].pts)
        e = kl.corner_reprojection_errors(kept[0], geom)
        self.assertLess(max(e), 0.5)      # 이상치를 뺀 뒤 다시 구한 자세로 거의 0
        T_c_t = np.linalg.inv(kept[0].T_t_c().T())
        self.assertTrue(np.allclose(T_c_t, true_pose(), atol=2e-3))

    def test_relabel_drops_misdetected_frame(self):
        geom = self.geom
        good = detected_obs(geom, "rot180")
        junk = detected_obs(geom, "rot180")
        rng = np.random.RandomState(0)
        for k in list(junk.pts):        # 장 전체가 엉터리(약 90 px) — .224 의 2086·2087장 같은 경우
            junk.pts[k] = junk.pts[k] + rng.uniform(-90, 90, 2)
        kept, st = kl.relabel_and_filter([good, junk], geom, "rot180", TAG_COLS)
        self.assertEqual(st["frames_big"], 1)
        self.assertEqual(len(kept), 1)
        self.assertIs(kept[0], good)


def _read(name):
    with open(os.path.join(DATA, name), encoding="utf-8") as f:
        return f.read()


class TestQuality(unittest.TestCase):
    def test_parse_ok_224(self):
        r = kq.parse_results(_read("results_224_fix180_ok.txt"))
        self.assertAlmostEqual(r["cams"][0]["reproj_px_mean"], 1.13525, places=4)
        self.assertAlmostEqual(r["cams"][1]["reproj_px_mean"], 1.12088, places=4)
        self.assertAlmostEqual(r["gyro_norm"], 1.2578, places=3)
        self.assertAlmostEqual(r["accel_norm"], 0.7298, places=3)
        self.assertAlmostEqual(r["cams"][0]["timeshift_s"], 0.035239, places=5)
        self.assertAlmostEqual(r["cams"][1]["timeshift_s"], 0.035869, places=5)
        self.assertAlmostEqual(r["cams"][0]["T_ci"][0][1], -0.99998884, places=7)
        self.assertEqual(r["cams"][0]["T_ci"][3], [0.0, 0.0, 0.0, 1.0])
        overall, rows = kq.judge(r)
        self.assertEqual(overall, "PASS", rows)

    def test_diverged_224_fails(self):
        overall, rows = kq.judge(kq.parse_results(_read("results_224_raw_diverged.txt")))
        self.assertEqual(overall, "FAIL")
        bad = [x[0] for x in rows if x[3] == "FAIL"]
        self.assertIn("cam0 재투영 평균", bad)
        self.assertIn("카메라끼리 시간차 차이", bad)     # 41.5 대 30.9 ms

    def test_old_best_so_far_fails(self):
        r = kq.parse_results(_read("results_old_best_so_far.txt"))
        self.assertAlmostEqual(r["cams"][0]["reproj_px_mean"], 35.2956, places=3)
        overall, rows = kq.judge(r)
        self.assertEqual(overall, "FAIL")
        self.assertEqual([x[3] for x in rows if x[0] == "카메라끼리 시간차 차이"], ["PASS"])  # 재투영만 실패

    def test_reference_compare(self):
        ref = kq.load_ref_T_cam_imu(os.path.join(DATA, "camchain_imucam_224_fix180.yaml"))
        r = kq.parse_results(_read("results_224_fix180_ok.txt"))
        overall, rows = kq.judge(r, ref)
        self.assertEqual(overall, "PASS")
        rot = [x for x in rows if x[0].startswith("기준 대비 회전")][0]
        self.assertLess(float(rot[1].rstrip("°")), 0.05)
        # 옛 결과를 새 기준과 비교하면 회전 5.9° → WARN 줄이 생긴다(종합은 재투영 때문에 FAIL)
        overall, rows = kq.judge(kq.parse_results(_read("results_old_best_so_far.txt")), ref)
        rot = [x for x in rows if x[0].startswith("기준 대비 회전")][0]
        self.assertEqual(rot[3], "WARN")
        self.assertAlmostEqual(float(rot[1].rstrip("°")), 5.92, places=1)

    def test_warn_band(self):
        txt = _read("results_224_fix180_ok.txt").replace(
            "Reprojection error (cam1) [px]:     mean 1.1208769269593588", "Reprojection error (cam1) [px]:     mean 3.1")
        self.assertEqual(kq.judge(kq.parse_results(txt))[0], "WARN")

    def test_missing_corners_fails(self):
        txt = "Normalized Residuals\n----\nReprojection error (cam0):     no corners\n\nResiduals\n----\n" \
              "Reprojection error (cam0) [px]:     no corners\n"
        self.assertEqual(kq.judge(kq.parse_results(txt))[0], "FAIL")
        self.assertEqual(kq.judge(kq.parse_results(""))[0], "FAIL")

    def test_cli_exit_codes_and_file(self):
        tmp = tempfile.mkdtemp()
        try:
            out = os.path.join(tmp, "q.txt")
            self.assertEqual(kq.main([os.path.join(DATA, "results_224_fix180_ok.txt"), "--out", out]), 0)
            with open(out, encoding="utf-8") as f:
                self.assertIn("종합: PASS", f.read())
            self.assertEqual(kq.main([os.path.join(DATA, "results_224_raw_diverged.txt")]), 2)
            self.assertEqual(kq.main([os.path.join(tmp, "없음.txt")]), 3)
        finally:
            shutil.rmtree(tmp)


class TestPatch(unittest.TestCase):
    def test_patch_sensors_all(self):
        code = _read("IccSensors_excerpt.py.txt")
        layouts = {"/left": "rot180", "/right": "flip_lr"}
        out = kp.patch_sensors(code, 1, "35", layouts, 10.0)
        self.assertIn("options.blackTagBorder = 1", out)
        self.assertIn("self.timeshiftCamToImuPrior = 0.035", out)
        self.assertIn("noTransformation=True", out)
        self.assertIn("_VISO_LAYOUTS = {'/left': 'rot180', '/right': 'flip_lr'}", out)
        # 발췌라 파이썬 문법 검사는 못 한다 — 실제 파일 문법 검사는 Docker 시험(패치 뒤 import·compile)에서 한다

    def test_patch_sensors_auto_identity_leaves_kalibr_alone(self):
        code = _read("IccSensors_excerpt.py.txt")
        out = kp.patch_sensors(code, 2, "auto", {"/left": "identity"}, 10.0)
        self.assertEqual(out, code)         # 2칸 테두리·Kalibr 시간차 추정·배치 그대로면 한 글자도 안 바뀐다

    def test_wildcard_layout_is_used_for_unlisted_cameras(self):
        out = kp.patch_sensors(_read("IccSensors_excerpt.py.txt"), 1, "auto", {"*": "rot180"}, 10.0)
        self.assertIn("_VISO_LAYOUTS.get(self.camConfig.getRosTopic(), _VISO_LAYOUTS.get('*', 'identity'))", out)
        self.assertEqual(kp.patch_sensors(_read("IccSensors_excerpt.py.txt"), 2, "auto", {"*": "identity"}, 10.0),
                         _read("IccSensors_excerpt.py.txt"))

    def test_patch_main_gate(self):
        out = kp.patch_main(_read("kalibr_calibrate_imu_camera_excerpt.txt"), 5.0)
        self.assertIn("[VISO GATE]", out)
        self.assertIn("sys.exit(4)", out)
        self.assertLess(out.index("[VISO GATE]"), out.index('print("Optimizing...")'))

    def test_missing_anchor_stops(self):
        with self.assertRaises(AssertionError):
            kp.patch_main("print('다른 Kalibr')\n", 5.0)
        with self.assertRaises(AssertionError):
            kp.patch_sensors("import scipy.optimize\n", 1, "auto", {}, 10.0)

    def test_parse_layouts(self):
        self.assertEqual(kp.parse_layouts(["/a/b=rot180", "/c=identity"]), {"/a/b": "rot180", "/c": "identity"})
        self.assertEqual(kp.parse_layouts(["*=identity"]), {"*": "identity"})     # 모든 카메라(pipeline 의 배치 강제)
        for bad in (["/a/b"], ["/a=rot45"], ["=rot180"]):
            with self.assertRaises(SystemExit):
                kp.parse_layouts(bad)

    def test_main_rejects_bad_timeshift(self):
        tmp = tempfile.mkdtemp()
        try:
            s, m = os.path.join(tmp, "s.py"), os.path.join(tmp, "m.py")
            shutil.copy(os.path.join(DATA, "IccSensors_excerpt.py.txt"), s)
            shutil.copy(os.path.join(DATA, "kalibr_calibrate_imu_camera_excerpt.txt"), m)
            with self.assertRaises(ValueError):
                kp.main(["--timeshift-ms", "abc", "--sensors", s, "--main-script", m])
            self.assertEqual(kp.main(["--timeshift-ms", "auto", "--layout", "/x=rot180",
                                      "--sensors", s, "--main-script", m]), 0)
            with open(s, encoding="utf-8") as f:
                self.assertIn("_VISO_LAYOUTS = {'/x': 'rot180'}", f.read())
        finally:
            shutil.rmtree(tmp)


if __name__ == "__main__":
    unittest.main(verbosity=2)
