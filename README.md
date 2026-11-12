# Visual-Inertial Calibration Pipeline

This repository provides a complete pipeline for calibrating IMU and camera systems using [allan_ros2](https://github.com/CruxDevStuff/allan_ros2) and [Kalibr](https://github.com/ethz-asl/kalibr). The pipeline automatically handles the entire calibration process from IMU noise parameter estimation to final extrinsic calibration.

## Overview

The calibration pipeline consists of three main stages:

1. **IMU Allan Variance Analysis**: Estimates IMU noise parameters (white noise, bias instability, random walk) from stationary data
2. **Camera Intrinsics Calibration**: Computes camera intrinsic parameters using AprilGrid patterns
3. **Visual-Inertial Extrinsics**: Determines the spatial relationship between IMU and camera

## Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- **Docker** for running Kalibr
- **Python 3.8+** with pip
- **Git** with submodule support

## Quick Start

### 1. Clone and Setup

```bash
# Clone with submodules
git clone --recursive https://github.com/snktshrma/viso_inertial_calib.git
cd viso_inertial_calib

# Run setup script to install dependencies
./setup.sh
```

> **EDIE(HERoEHS) 사용 시** — 안쪽 서브모듈 kalibr·allan_ros2 는 EDIE 수정 커밋이 들어 있는 HERoEHS 포크의 `edie9` 브랜치를 쓴다(`.gitmodules`).
> 원본 저장소에는 이 커밋이 없어서 원본 주소로는 `--recursive` 갱신이 실패한다. 모든 주소는 https 라 ssh 키 없는 로봇에서도 받을 수 있다.
> 이미 클론해 둔 곳(PC·로봇)은 주소를 새로 맞춘 뒤 갱신한다:
>
> ```bash
> git submodule sync --recursive && git submodule update --init --recursive
> ```
>
> 캘리브레이션 데이터(`data/`, `best_so_far/`, `backup/`, `camchain*.yaml` 등)는 `.gitignore` 로 git 밖에 둔다 — 장비 사이에는 따로 옮긴다.
>
> **어디서 무엇을 돌리나 (EDIE)**
> - `allan` 모드(IMU 정지 bag → `imu.yaml`)는 **로봇에서도** 돈다. Docker·rosbags 가 없어도 된다.
>   - ROS 환경을 source 하지 않은 셸이면 `$ROS_WS/install/setup.bash`(기본 `~/ros2_ws`)를 먼저 불러온다.
>   - allan_ros2 가 이미 설치돼 있으면 빌드하지 않는다(`ALLAN_REBUILD=1` 이면 다시 빌드). analysis.py 의 파이썬 모듈이 없을 때만 pip 로 설치한다.
>   - IMU 주기는 bag 에서 잰다(rosbags → rosbag2_py). 토픽이 없거나 못 재면 멈춘다 — `IMU_RATE=<Hz>` 로 직접 줄 수 있다.
>   - 노드가 `ALLAN_TIMEOUT`(기본 1800 s) 안에 결과를 못 내면 멈춘다.
> - `kalibr`·`sweep`·`all` 모드는 Docker 와 `kalibr:ros1`(ROS1) 이미지가 필요하다. 기본은 **PC 에서 계산**한다. Docker 가 없는 장비에서는 무엇을 옮길지 안내하고 멈춘다(arm64 이미지 빌드는 검증 안 됨).
>   - kalibr·sweep: VIO bag 과 실행 폴더의 `imu.yaml`(로봇 allan 결과)·`camchain.yaml` 이 PC 실행 폴더에 있어야 한다.
>   - all: IMU 정지 bag·카메라 bag·VIO bag 이 모두 필요하고 allan 을 다시 돈다. 로봇에서는 `MODE=allan` 으로 imu.yaml 만 만들고, PC 에서는 `MODE=kalibr` 로 이어 가면 된다.
> - 결과는 실행 폴더와 함께 **`$EDIE_CALIB_DIR`(기본: root·edieOS 면 `/var/lib/edie/data/calib`, 그 밖은 `~/.edie/calib` — 규칙 `lib/calib_dir.sh`, 시험은 `bash test/test_calib_dir.sh` 로 직접 돌린다 — colcon 패키지가 아니라 자동으로 돌지 않음)** 에도 복사된다. 모드마다 이번 실행에서 만들거나 쓴 것만 복사한다.
>   - allan: `imu.yaml`
>   - kalibr: `imu.yaml`·`$CAMCHAIN_FILE`(기본 `camchain.yaml`)·`kalibr_quality.txt`·이번 실행이 만든 `<bag>-camchain-imucam.yaml`·`<bag>-results-imucam.txt`·`<bag>-imu.yaml` — 단 결과 요약이 FAIL 이면 아무것도 게시하지 않는다(아래 "Kalibr 단계 점검")
>   - all: `imu.yaml`·`camchain.yaml`·위 Kalibr 결과
>   - sweep: 복사하지 않음(탐색용)
>   - 결과를 쓰는 쪽(VINS 설정 등)은 `<bag>-camchain-imucam.yaml`(내부 파라미터 + T_cam_imu 포함)을 기준으로 읽는다. `$CAMCHAIN_FILE` 은 자기 이름 그대로 복사되므로 예전 `camchain.yaml` 과 함께 있을 수 있다.
>   - kalibr·all 에서 이번 실행이 만든 Kalibr 결과가 없으면 오류로 끝난다.
> - allan 파라미터는 실행할 때 `output/allan_params.yaml` 로 새로 만든다(저장소의 `external/allan_ros2/config/config.yaml` 은 건드리지 않는다).
> - `external/kalibr` 는 ROS1(catkin) 이라 `COLCON_IGNORE` 로 colcon 전체 빌드에서 빠진다(Docker 이미지 안에서만 catkin 으로 빌드). 같은 포크의 `.dockerignore` 가 이 표식을 이미지에 넣지 않는다 — catkin 도 COLCON_IGNORE 를 무시 표식으로 보기 때문(SW1-1951).

#### Kalibr 단계 점검 (EDIE, kalibr·sweep 모드)

**왜 있나 (2026-11-12 .224 사고).** 보드 그림 방향이 Kalibr 가 기대하는 방향과 다르면, Kalibr 검출기가 태그마다 네 모서리 번호를 엇갈려 붙인다(퍼즐 조각을 돌려 끼운 꼴). 태그 번호는 맞아서 검출은 다 되는데, 장마다 카메라 자세가 틀려 재투영 49 px 로 발산했다. 그래도 Kalibr 는 결과를 다 쓰고 끝났다. 어긋나는 방식은 녹화마다 달랐다(.224 녹화는 180°, 옛 녹화는 좌우 뒤집기). 그래서 고정 패치 대신 매번 점검해서 고른다.

1. **모서리 배치 자동 점검(본계산 전).** Kalibr 검출기 그대로 프레임 약 20장(녹화 전체에 고르게, 태그가 많이 보이는 장)에서 모서리를 뽑는다. 태그 안 배치 8가지(회전 4 × 뒤집기 2)마다 장별 PnP 재투영을 재서 표로 보여 준다(`output/kalibr_layout_check.txt`). 1등이 **2 px 미만**이고 2등과 **10배 넘게** 차이 나면 그 배치를 카메라마다 채택한다(그대로면 패치 없음). 본계산에서는 검출만 한 뒤 모서리를 다시 붙이고, 바른 배치로 자세를 구한 뒤에 Kalibr 와 같은 규칙(평균+2σ 초과이면서 0.2 px 초과)으로 이상치 모서리를 거른다. 그래도 평균 10 px 넘는 장(잘못 검출된 장)은 뺀다.
   - **멈췄을 때(종료 코드 3):** 보드 yaml 의 `tagSize`(검은 사각형 한 변, m)·`tagSpacing`(간격÷태그 크기)·`tagRows/tagCols`(번호 배치), 테두리 답(표준 AprilTag = 1칸, Kalibr 로 만든 보드 = 2칸), camchain(내부값·왜곡·토픽·해상도가 이 녹화 카메라 것인지)을 확인한다.
2. **발산 관문.** Kalibr 가 "Before Optimization" 에 찍는 카메라별 재투영 평균이 **5 px** 를 넘으면 최적화로 가지 않고 멈춘다(종료 코드 4).
3. **결과 요약(PASS/WARN/FAIL).** 끝나면 `<bag>-results-imucam.txt` 를 읽어 `kalibr_quality.txt` 로 한 화면 요약을 남긴다. FAIL 이면 장비 보정 자리(`$EDIE_CALIB_DIR`)로 게시하지 않는다(종료 코드 5). 성공 여부는 Kalibr 종료 코드가 아니라 "이번 실행의 결과 파일 + 이 요약"으로 판단한다(보고서 창은 `--dont-show-report`·`MPLBACKEND=Agg` 로 띄우지 않는다).

| 항목 | PASS | WARN | FAIL |
|---|---|---|---|
| 카메라별 재투영 평균 | < 2 px | 2~5 px | ≥ 5 px |
| 자이로·가속도 정규화 잔차 평균(잡음값으로 나눈 오차, 1 근처가 정상) | 0.3~3 | 0.3 미만, 3~10 | > 10 |
| 카메라끼리 시간차 차이 | < 3 ms | 3~10 ms | ≥ 10 ms |
| 기준 대비 회전 / 카메라 위치(기준을 줄 때만) | < 3° / < 3 cm | 그 이상 | (없음 — 기준값이 틀렸을 수 있다) |

문턱은 모두 **임시값**이다(.224 성공·발산, 옛 결과 세 개로만 정함). 기준값은 `KALIBR_REF_CAMCHAIN=<T_cam_imu 가 든 camchain yaml>` 로 주거나, 입력 camchain 에 `T_cam_imu` 가 있으면 그것을 쓴다.

**시간차 인자(7번째, ms).** `auto`(기본)는 Kalibr 자체 추정(카메라 자세에서 구한 각속도 크기와 자이로 크기의 교차상관)을 그대로 쓴다. 숫자를 주면 그 값으로 시작한다. 롤링셔터 카메라는 화면 가운데 줄 기준 시각이라 약 **+읽기시간/2** 가 더해진다(.224: 노출 지연 3.2 + 읽기 64.4/2 ≈ 35 ms). 최적화 중에는 시작값에서 ±50 ms(padding) 안에서만 움직인다.
- **롤링셔터 카메라는 `auto` 가 크게 빗나갈 수 있다.** .224 실측: Kalibr 추정 −18.5 ms, 바른 값 약 +35 ms → 53 ms 차이로 범위를 벗어나 Kalibr 가 "Spline Coefficient Buffer Exceeded" 로 죽었다. pipeline 은 이때 **종료 코드 6** 과 함께 숫자를 주라고 안내한다. 롤링셔터 카메라는 처음부터 숫자를 주는 것이 안전하다(예: `... kalibr /edie/sensor/offset_imu 35`).
- 옛 판의 `auto`(영상 직후 IMU stamp 까지 간격의 중앙값 → 1~2 ms)는 카메라–IMU 시간차가 아니라 지웠다. 옛 판이 Kalibr 추정(그때 50 ms)을 9 ms 등으로 덮어쓴 이유도 같은 "버퍼 초과"였다(git 4f5fb54 주석 — 그때는 모서리 배치도 틀린 상태였다).

**환경변수.**
- `KALIBR_TAG_BORDER=1|2` — 테두리 질문("표준 AprilTag36h11 입니까?")을 생략(1 = 표준 AprilTag 1칸, 2 = Kalibr 보드 2칸).
- `KALIBR_CORNER_LAYOUT=auto|identity|rot90|rot180|rot270|flip_lr|flip_ud|transpose|antitranspose` — `auto`(기본)가 아니면 점검 없이 그 배치를 모든 카메라에 강제(`identity` = 점검을 끈 옛 동작).
- `KALIBR_CHECK_ONLY=1` — 배치 점검만 하고 끝. `KALIBR_CHECK_FRAMES`(기본 20), `KALIBR_GATE_PX`(기본 5).
- VIO bag 이 ROS1 `.bag` 파일이면 변환하지 않고 하드링크(복사 아님)로 쓴다.
- 종료 코드: 0 성공(PASS·WARN, 게시함), 1 일반 오류, 3 배치 점검에서 멈춤, 4 최적화 전 관문, 5 결과 요약 FAIL(게시 안 함), 6 시간차 범위 초과.
- 시험: `python3 test/test_kalibr_tools.py`(배치 식·선택 규칙·요약 판정·패치), `bash test/test_pipeline_kalibr_flow.sh`(가짜 docker 로 종료 코드·게시 여부). Docker 없이 돈다.

### 2. Prepare Your Data

You need three ROS2 bag files:

- **IMU Stationary Bag**: IMU left completely still for ~2 hours
  - Topic: `/imu` (sensor_msgs/Imu)
  - Duration: 2+ hours recommended
  - Purpose: Allan variance analysis for noise parameters

- **Camera Intrinsics Bag**: Camera static, AprilGrid moving in view
  - Topic: `/camera/image_raw` (sensor_msgs/Image)
  - Duration: 5-10 minutes
  - Purpose: Camera intrinsic calibration

- **VIO Calibration Bag**: AprilGrid static, sensor stack moving
  - Topics: `/imu` and `/camera/image_raw`
  - Duration: 5-10 minutes
  - Purpose: IMU-camera extrinsic calibration

### 3. Run the Pipeline

```bash
./pipeline.sh imu_stationary.bag camera_intrinsics.bag vio_calibration.bag aprilgrid.yaml
```

## Output Files

The pipeline generates several calibration files:

- **`imu.yaml`**: IMU noise parameters (compatible with Kalibr)
- **`camchain.yaml`**: Camera intrinsic parameters
- **`imu-*.yaml`**: IMU-camera extrinsic calibration

These files are directly compatible with:
- VINS-Fusion
- OKVIS
- ROVIO
- Other visual-inertial SLAM systems

## Detailed Usage

### Manual Setup (Alternative to setup.sh)

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions

# Install Python packages
pip3 install matplotlib numpy scipy pyyaml rosbags

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Log out and back in, or run: newgrp docker
```

### Pipeline Stages

#### Stage 1: IMU Allan Variance Analysis
- Builds `allan_ros2` package
- Processes IMU data to compute Allan deviation
- Generates noise parameters (white noise, bias instability, random walk)

#### Stage 2: Camera Intrinsics
- Converts ROS2 bags to ROS1 format
- Runs Kalibr camera calibration in Docker
- Outputs camera intrinsic matrix and distortion coefficients

#### Stage 3: Visual-Inertial Extrinsics
- Combines IMU and camera data
- Computes spatial relationship between sensors
- Final calibration ready for SLAM systems

## Troubleshooting

### Common Issues

**"rosbags-convert not found"**
```bash
pip3 install rosbags
```

**"Docker permission denied"**
```bash
sudo usermod -aG docker $USER
# Log out and back in
```

**"Failed to build allan_ros2"**
```bash
# Check ROS2 environment
source /opt/ros/humble/setup.bash
rosdep update
```

**"Kalibr Docker build failed"**
```bash
# Ensure Docker is running
sudo systemctl start docker
# Check available disk space
df -h
```

### Debug Mode

For detailed debugging, you can modify the pipeline script:
```bash
# Add debug output
set -x  # Add at the beginning of pipeline.sh
```

### Manual Execution

If the pipeline fails, you can run stages manually:

```bash
# Stage 1: Allan variance
cd allan_ws
colcon build --packages-select allan_ros2
source install/setup.bash
ros2 launch allan_ros2 allan_node.py

# Stage 2: Camera calibration
docker run --rm -v $(pwd):/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_cameras --bag /data/camera.bag --target /data/aprilgrid.yaml --models pinhole-radtan --topics /camera/image_raw"

# Stage 3: IMU-camera calibration
docker run --rm -v $(pwd):/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_imu_camera --bag /data/vio.bag --target /data/aprilgrid.yaml --cam /data/camchain.yaml --imu /data/imu.yaml"
```

## Data Collection Guidelines

### IMU Stationary Data
- Place IMU on a stable surface
- Avoid vibrations and temperature changes
- Record for at least 2 hours
- Ensure no movement during recording

### Camera Intrinsics Data
- Keep camera completely still
- Move AprilGrid to cover entire field of view
- Include different distances and angles
- Ensure good lighting and focus

### VIO Calibration Data
- Keep AprilGrid stationary and visible
- Move sensor stack in 6DOF motion
- Include rotations around all axes
- Avoid motion blur in images

## AprilGrid Configuration

Create an AprilGrid YAML file:
```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088
tagSpacing: 0.3
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## References

- **[allan_ros2](https://github.com/CruxDevStuff/allan_ros2)**: ROS2 package for IMU Allan variance analysis
- **[Kalibr](https://github.com/ethz-asl/kalibr)**: ETH-ASL visual-inertial calibration toolbox
- **[ROS2 Humble](https://docs.ros.org/en/humble/)**: Robot Operating System 2
- **[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)**: Visual-Inertial SLAM system

---
---
*** A big shoutout to my buddy who helped with structuring this pipeline and helped with bash scripting; LLMs :) ***