#!/usr/bin/env bash
set -e

if [ "$#" -lt 4 ] || [ "$#" -gt 5 ]; then
  echo "Usage: $0 BAG_IMU BAG_CAM BAG_VIO APRILGRID_YAML [MODE]" >&2
  echo "  MODE: all (default) | allan" >&2
  echo "Example: $0 imu_stationary.bag camera_intrinsics.bag vio_calibration.bag aprilgrid.yaml allan" >&2
  exit 1
fi

imu_bag=$1
cam_bag=$2
vio_bag=$3
april=$4
mode=${5:-all}

if [ "$mode" != "all" ] && [ "$mode" != "allan" ]; then
  echo "Error: invalid MODE '$mode'. Use 'all' or 'allan'." >&2
  exit 1
fi

# check if files exist
if [ ! -f  "$imu_bag" ]; then
    echo "Error: IMU bag path '$imu_bag' not found" >&2
    exit 1
fi

if [ ! -f  "$cam_bag" ]; then
    echo "Error: Camera bag path '$cam_bag' not found" >&2
    exit 1
fi

if [ ! -f  "$vio_bag" ]; then
    echo "Error: VIO bag path '$vio_bag' not found" >&2
    exit 1
fi

if [ ! -f "$april" ]; then
    echo "Error: AprilGrid YAML file '$april' not found" >&2
    exit 1
fi

workdir=$(pwd)
ROS_WS=${ROS_WS:-$HOME/ros2_ws}
output_dir="$workdir/output"
mkdir -p "$output_dir"

# Make april yaml visible inside docker mount (/data)
april_abs=$(realpath "$april")
april_base=$(basename "$april_abs")
cp -f "$april_abs" "$workdir/$april_base"
april="$april_base"

echo "Starting calibration..."
echo "IMU: $imu_bag"
echo "Cam: $cam_bag"
echo "VIO: $vio_bag"
echo "Grid: $april"
echo ""

# check tools
if ! command -v rosbags-convert &> /dev/null; then
    echo "Error: rosbags-convert not found. Install with: pip install rosbags" >&2
    exit 1
fi

if ! command -v docker &> /dev/null; then
    echo "Error: docker not found." >&2
    exit 1
fi

# ROS2 bag → ROS1 bag 변환 (Kalibr는 ROS1 bag만 지원)
echo "Converting bags to ROS1 format..."
ros1_cam="cam_ros1.bag"
ros1_vio="vio_ros1.bag"

rosbags-convert --src "$cam_bag" --dst "$ros1_cam"
rosbags-convert --src "$vio_bag" --dst "$ros1_vio"

echo "Bags converted."
echo ""

# allan variance setup
echo "Running allan variance using existing workspace: $ROS_WS"

# params file for allan (공통 사용: ROS2 노드 + analysis.py)
# 기존 예제 파일(external/allan_ros2/config/config.yaml)을 실행 시점에 덮어써서 사용
allan_cfg="$workdir/external/allan_ros2/config/config.yaml"
cat > "$allan_cfg" <<EOF
allan_node:
  ros__parameters:
    topic: /edie/sensor/lpf_imu
    bag_path: $imu_bag
    msg_type: ros
    publish_rate: 400
    sample_rate: 400
EOF

# build in existing ROS2 workspace (no allan_ws)
pip3 install matplotlib numpy scipy pyyaml

cd "$ROS_WS"
allan_ros2_path="$ROS_WS/src/edie9/edie_localization/third_party/viso_inertial_calib/external/allan_ros2"
rosdep install --from-paths $allan_ros2_path -y --ignore-src
colcon build --packages-select allan_ros2

source "$ROS_WS/install/setup.bash"

# run allan analysis from output_dir so deviation.csv, plots, imu.yaml 모두 그 안에 생성되도록
cd "$output_dir"
echo "Running allan variance..."
ros2 run allan_ros2 allan_node --ros-args --params-file "$allan_cfg" &
allan_pid=$!

sleep 10
if ! kill -0 $allan_pid 2>/dev/null; then
    echo "allan node crashed" >&2
    exit 1
fi

# deviation.csv가 만들어질 때까지 대기(비어있지 않게 -s)
while [ ! -s "$output_dir/deviation.csv" ]; do
  sleep 1
done

# 파일 flush 시간 조금 주고, 노드 종료
sleep 2
kill -INT $allan_pid 2>/dev/null || true
wait $allan_pid 2>/dev/null || true

echo "Allan done."

# generate imu calib
echo "Generating IMU params..."
if [ -f "$output_dir/deviation.csv" ]; then
    # output_dir 안에서 실행하여 imu.yaml, acceleration.png, gyro.png 모두 output_dir에 생성
    cd "$output_dir"
    python3 "$workdir/external/allan_ros2/scripts/analysis.py" --data deviation.csv --config "$allan_cfg"

    if [ -f "$output_dir/imu.yaml" ]; then
        echo "IMU calib generated."
        # Kalibr 등 기존 파이프라인 호환을 위해 workdir 루트에도 복사
        cp "$output_dir/imu.yaml" "$workdir/imu.yaml"
        cd "$workdir"
    else
        echo "Failed to generate imu.yaml" >&2
        exit 1
    fi
else
    echo "deviation.csv not found" >&2
    exit 1
fi

echo ""
# MODE이 allan이면 여기서 종료 (카메라/VIO 캘립 생략)
if [ "$mode" = "allan" ]; then
    echo "Allan variance analysis finished."
    echo "Skipping camera and VIO calibration (mode=allan)."
    echo ""
    echo "Cleaning up..."
    rm -f "$ros1_cam" "$ros1_vio"
    echo "Done!"
    exit 0
fi

# camera calib
echo "Camera calibration..."

if ! docker image inspect kalibr:ros1 > /dev/null 2>&1; then
    echo "Building kalibr docker..."
    docker build -t kalibr:ros1 external/kalibr -f external/kalibr/Dockerfile_ros1_20_04
    
    if [ $? -ne 0 ]; then
        echo "Docker build failed" >&2
        exit 1
    fi
fi

echo "Running cam calib..."
docker run --rm -v "$workdir":/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_cameras --bag /data/$ros1_cam --target /data/$april --models pinhole-radtan --topics /camera/image_raw"

if [ $? -ne 0 ]; then
    echo "Camera calib failed" >&2
    exit 1
fi

# find camchain file
if ls camchain*.yaml 1> /dev/null 2>&1; then
    camchain_file=$(ls camchain*.yaml | head -1)
    cp "$camchain_file" camchain.yaml
    echo "Camera calib done: $camchain_file"
else
    echo "Camera calib file not found" >&2
    exit 1
fi

echo ""

# vio calib
echo "VIO calibration..."
echo "Running IMU-camera calib..."

docker run --rm -v "$workdir":/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_imu_camera --bag /data/$ros1_vio --target /data/$april --cam /data/camchain.yaml --imu /data/imu.yaml"

if [ $? -ne 0 ]; then
    echo "VIO calib failed" >&2
    exit 1
fi

echo ""

# done
echo "Calibration complete!"
echo ""
echo "Files generated:"
if [ -f "imu.yaml" ]; then
    echo "  imu.yaml"
fi
if [ -f "camchain.yaml" ]; then
    echo "  camchain.yaml"
fi
if ls imu-*.yaml 1> /dev/null 2>&1; then
    echo "  imu-*.yaml"
fi

echo ""
echo "Ready for VINS-Fusion etc."
echo ""

# cleanup
echo "Cleaning up..."
rm -f "$ros1_cam" "$ros1_vio"

echo "Done!"
