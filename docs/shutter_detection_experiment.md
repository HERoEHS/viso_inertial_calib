# 스테레오 카메라 셔터 타입 판별 실험 보고서

> **목적**: VIO 캘리브레이션 bag(`/edie/sensors/camera/left|right/image_gray`)의 영상만으로
> 카메라가 **Global Shutter**인지 **Rolling Shutter**인지를 ROS2 런타임·Kalibr 없이 판별한다.
> **결론**: 좌·우 카메라 모두 **Global Shutter**. (영상 분석 단독 결론이며, 별도 하드웨어 사양 확인은 본 문서 범위 밖)

---

## 1. 배경 / 문제

- Kalibr 캘리브레이션에서 25~30px 수준의 큰 reprojection error가 관찰됨.
- 원인 후보 중 하나로 "Rolling Shutter(RS) 미반영"이 의심되어, 영상만으로 셔터 타입을 판별할 필요가 있었다.
- 제약: ROS2 런타임 기동 불가, 추가 패키지 설치 최소화. → rosbag2 `.db3`(sqlite3)를 직접 읽고 CDR를 수동 역직렬화하여 프레임 단위로 분석.

---

## 2. 방법 A — phaseCorrelate skew (1차 시도, **위양성**)

- `detect_shutter_from_bag.py`: 프레임을 상·하 밴드로 나눠 `cv2.phaseCorrelate`로 수평 이동량을 구하고,
  행(y)에 따른 수평 skew를 측정.
- 결과: `skew / shift ≈ 0.4`로 나와 **"Rolling Shutter"**로 판정.
- **문제**: 이 판정은 **위양성(false positive)**. skew가 "행에 따라 존재한다"는 사실만으로는 RS를 증명하지 못한다.
  - 정류(rectification) remap의 행 의존 수평 게인, 씬 시차(parallax) 등 **정적 기하 왜곡**도 동일한 형태의 skew를 만든다.
  - 단일 프레임의 intra-frame shear는 보드의 호모그래피/affine pose에 그대로 흡수되어 **식별 불가능(non-identifiable)**.

---

## 3. 방법 B — AprilGrid 코너 추적 + 가속/속도 분리 (최종)

`estimate_readout_time.py` 로 구현. 핵심 아이디어는 **"skew가 무엇에 비례하는가"** 를 분리하는 것.

### 3.1 물리 모델

프레임 간(Δt) 공통 코너의 수평 이동량 `dx`를 행 `y`에 대해 1차 회귀:

$$ dx(y) = \beta \cdot y + \alpha $$

- $\alpha$ = 전체 평행 이동(translation) → 픽셀 속도 $v = \alpha/\Delta t$ 에 비례.
- $\beta$ = 행 의존 skew 기울기.

셔터 타입에 따른 $\beta$의 기원이 결정적으로 다르다:

| 셔터 | $\beta$가 비례하는 양 | 이유 |
|---|---|---|
| **Rolling Shutter** | **가속** ($a\cdot\Delta t$) | 행마다 노출 시각이 $r\cdot y$ 만큼 어긋나, 운동이 변할 때만 skew 발생 ($r = t_{ro}/H$) |
| **Global Shutter (정적 왜곡)** | **속도** ($\alpha$) | 정류 remap·시차에 의한 고정 기하 게인. 움직이는 양에 단순 비례 |

→ **판별식**: $\beta$ 가 **가속**에 상관되면 RS, **속도**에 상관되면 GS.

### 3.2 파이프라인

1. `cv2.aruco` `DICT_APRILTAG_36h11` + `CORNER_REFINE_SUBPIX`로 코너 검출.
2. 인접 프레임 공통 코너로 `dx = β·y + α` 회귀 (3σ 아웃라이어 제거, 유효 조건: 공통 코너 ≥ 8, 행 스팬 ≥ 0.40·H).
3. 픽셀 속도 `v[i] = α[i]/Δt`, 중앙차분으로 가속 `aXdt[i] = (v[i+1] − v[i−1]) / 2`.
4. 두 상관계수 계산:
   - `corr_acc = corr(β, a·Δt)` → **RS 신호**
   - `corr_vel = corr(β, α)` → **속도비례 정적 왜곡(GS) 신호**
5. 부가량: `static_gain = median(β·H/α)`, robust `r = median(β/(a·Δt))`, `t_ro = r·H`.

### 3.3 판정 로직 (스크립트 인용)

```python
rs_dominant = (abs(corr_acc) >= 0.35) and (abs(corr_acc) > 1.3 * abs(corr_vel))
if rs_dominant:
    verdict = "Rolling Shutter 확인 (가속 비례 신호 우세), t_ro ≈ ..."
elif abs(corr_vel) >= abs(corr_acc):
    verdict = "Global Shutter 판정 (skew가 '속도'에 비례, corr_vel 우세)"
else:
    verdict = "보류 (raw bag 재측정 권장)"
```

> ⚠️ 초기 버전은 `abs(corr_acc)`만으로 판정해 **음의 상관**도 RS로 오인할 수 있었음. → `corr_vel` 우세 시 GS로 가도록 수정.

---

## 4. 결과

분석 대상: 두 개의 캘리브레이션 bag, 각 좌·우 토픽 (mono8 640×480).

| Bag | 토픽 | corr_acc (RS) | corr_vel (정적) | static_gain | 판정 |
|---|---|---|---|---|---|
| `vio_calib_20260612_162851`<br/>(1036 frames, fps≈14.36, Δt≈69.6ms) | left | −0.06 | **+0.60** | 0.40 | **Global Shutter** |
| | right | −0.06 | **+0.54** | 0.35 | **Global Shutter** |
| `vio_calib_20260611_185550`<br/>(1561 frames) | left | −0.18 | **+0.31** | 0.32 | **Global Shutter** |
| | right | −0.29 | **+0.41** | 0.47 | **Global Shutter** |

- 모든 케이스에서 **`corr_vel`(속도비례) 우세, `corr_acc`(가속비례)는 미약 + 음수**.
  음의 corr_acc는 RS의 일관된 양의 line-delay와 물리적으로 모순 → RS 신호 아님.
- 방법 A가 본 `skew/shift ≈ 0.4`의 정체 = **정적 워프 게인**(정류·시차), 셔터 readout 서명이 아님.

**검증**: 입력 영상 육안 확인(`/tmp/left_frame_100.png`, `/tmp/left_frame_500.png`) — 실제 방 안 AprilGrid 보드, 프레임당 약 30~32개 태그 검출.

---

## 5. 결론 및 시사점

- **좌·우 카메라 모두 Global Shutter.** 1차 phaseCorrelate "RS" 판정은 위양성이었다.
- 따라서 **Kalibr/VIO에서 Rolling Shutter 모델링은 불필요**.
- 25~30px reprojection error의 원인은 셔터가 아니라 **정류(rectified) 영상 ↔ distortion 모델 불일치**(또는 cam-IMU time offset) 쪽에서 찾아야 한다.
  - 권장: raw(비정류) 영상을 쓰거나, pinhole + zero-distortion(P 행렬) 조합으로 일관성 확보.

### 핵심 교훈

> 프레임 간 운동으로 셔터 타입을 판별할 때는 반드시 **skew가 "속도"에 비례하는지 "가속"에 비례하는지**를 분리하라.
> 단일 프레임 intra-shear는 보드 pose에 흡수되어 식별 불가능하다.

---

## 6. 관련 스크립트

| 파일 | 역할 |
|---|---|
| `scripts/estimate_readout_time.py` | (최종) AprilGrid 코너 기반 가속/속도 분리 판별 + readout time 추정 |
| `scripts/detect_shutter_from_bag.py` | (1차) phaseCorrelate skew 측정 — 위양성, 비교용 보존 |
| `scripts/rolling_shutter_test.py` | 원본 라이브 카메라용 버전 |
