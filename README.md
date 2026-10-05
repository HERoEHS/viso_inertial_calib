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
> - `allan` 모드(IMU 정지 bag → `imu.yaml`)는 **로봇에서도** 돈다. Docker·rosbags 가 없어도 되고, allan_ros2 가 설치돼 있으면 실행 중 pip·colcon 을 하지 않는다.
> - `kalibr`·`sweep`·`all` 모드는 Docker 와 `kalibr:ros1`(ROS1) 이미지가 필요하다. 기본은 **PC 에서 계산**한다 — 로봇에서 녹화한 VIO bag 을 PC 로 옮겨 실행한다. Docker 가 없는 장비에서는 안내를 띄우고 멈춘다(arm64 이미지 빌드는 검증 안 됨).
> - 결과(`imu.yaml`, `camchain.yaml`, 최신 `camchain-imucam-*.yaml`·`results-imucam-*.txt`)는 실행 폴더와 함께 **`$EDIE_CALIB_DIR`(기본 `~/.edie/calib`)** 에도 복사된다. PC 에서 계산한 결과를 로봇에 넣을 때도 같은 자리를 쓴다.
> - allan 파라미터는 실행할 때 `output/allan_params.yaml` 로 새로 만든다(저장소의 `external/allan_ros2/config/config.yaml` 은 건드리지 않는다).

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