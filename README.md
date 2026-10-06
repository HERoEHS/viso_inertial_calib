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
> - 결과는 실행 폴더와 함께 **`$EDIE_CALIB_DIR`(기본 `~/.edie/calib`)** 에도 복사된다. 모드마다 이번 실행에서 만들거나 쓴 것만 복사한다.
>   - allan: `imu.yaml`
>   - kalibr: `imu.yaml`·`$CAMCHAIN_FILE`(기본 `camchain.yaml`)·이번 실행이 만든 `<bag>-camchain-imucam.yaml`·`<bag>-results-imucam.txt`·`<bag>-imu.yaml`
>   - all: `imu.yaml`·`camchain.yaml`·위 Kalibr 결과
>   - sweep: 복사하지 않음(탐색용)
>   - 결과를 쓰는 쪽(VINS 설정 등)은 `<bag>-camchain-imucam.yaml`(내부 파라미터 + T_cam_imu 포함)을 기준으로 읽는다. `$CAMCHAIN_FILE` 은 자기 이름 그대로 복사되므로 예전 `camchain.yaml` 과 함께 있을 수 있다.
>   - kalibr·all 에서 이번 실행이 만든 Kalibr 결과가 없으면 오류로 끝난다.
> - allan 파라미터는 실행할 때 `output/allan_params.yaml` 로 새로 만든다(저장소의 `external/allan_ros2/config/config.yaml` 은 건드리지 않는다).
> - `external/kalibr` 는 ROS1(catkin) 이라 `COLCON_IGNORE` 로 colcon 전체 빌드에서 빠진다(Docker 이미지 안에서만 catkin 으로 빌드). 같은 포크의 `.dockerignore` 가 이 표식을 이미지에 넣지 않는다 — catkin 도 COLCON_IGNORE 를 무시 표식으로 보기 때문(SW1-1951).

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