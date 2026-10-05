#!/usr/bin/env bash
set -e

if [ "$#" -lt 4 ] || [ "$#" -gt 7 ]; then
  echo "Usage: $0 BAG_IMU BAG_CAM BAG_VIO APRILGRID_YAML [MODE] [IMU_TOPIC] [TIMESHIFT_MS|auto]" >&2
  echo "  MODE: all (default) | allan | kalibr | sweep" >&2
  echo "  IMU_TOPIC: IMU 토픽 (기본값: /edie/sensor/offset_imu)" >&2
  echo "  TIMESHIFT_MS: timeshift prior (ms 단위). 생략 시 bag에서 자동 추정" >&2
  echo "  kalibr: allan 스킵, 기존 imu.yaml+camchain.yaml 사용, VIO 캘립만 실행" >&2
  echo "          BAG_IMU, BAG_CAM 은 '-' 로 생략 가능" >&2
  echo "  sweep:  tagSize 21~27mm 범위를 1mm 간격으로 Kalibr 반복 실행, reprojection error 최솟값 탐색" >&2
  echo "          kalibr 모드 전제 (imu.yaml + camchain.yaml 필요), BAG_IMU, BAG_CAM 은 '-' 로 생략 가능" >&2
  echo "Example: $0 imu_stationary.bag camera_intrinsics.bag vio_calibration.bag aprilgrid.yaml allan" >&2
  echo "Example: $0 imu_stationary.bag - - aprilgrid.yaml allan /edie/sensor/offset_imu" >&2
  echo "Example: $0 - - vio_calibration.bag aprilgrid.yaml kalibr" >&2
  echo "Example: $0 - - vio_calibration.bag aprilgrid.yaml kalibr /edie/sensor/offset_imu 9" >&2
  echo "Example: $0 - - vio_calibration.bag aprilgrid.yaml sweep" >&2
  exit 1
fi

imu_bag=$([ "$1" = "-" ] && echo "-" || realpath "$1" 2>/dev/null || echo "$1")
cam_bag=$([ "$2" = "-" ] && echo "-" || realpath "$2" 2>/dev/null || echo "$2")
vio_bag=$([ "$3" = "-" ] && echo "-" || realpath "$3" 2>/dev/null || echo "$3")
april=$4
mode=${5:-all}
imu_topic=${6:-/edie/sensor/offset_imu}
timeshift_arg=${7:-auto}   # ms 단위 숫자 또는 'auto' (bag에서 자동 추정)

if [ "$mode" != "all" ] && [ "$mode" != "allan" ] && [ "$mode" != "kalibr" ] && [ "$mode" != "sweep" ]; then
  echo "Error: invalid MODE '$mode'. Use 'all', 'allan', 'kalibr', or 'sweep'." >&2
  exit 1
fi

# check if files exist (kalibr/sweep 모드에서는 imu_bag/cam_bag 불필요)
if [ "$mode" != "kalibr" ] && [ "$mode" != "sweep" ] && [ ! -e "$imu_bag" ]; then
    echo "Error: IMU bag path '$imu_bag' not found" >&2
    exit 1
fi

if [ "$mode" = "all" ] && [ ! -f "$cam_bag" ]; then
    echo "Error: Camera bag path '$cam_bag' not found" >&2
    exit 1
fi

if [ "$mode" != "allan" ] && [ ! -e "$vio_bag" ]; then
    echo "Error: VIO bag path '$vio_bag' not found" >&2
    exit 1
fi

workdir=$(pwd)
# 스크립트가 있는 폴더 — external/(allan_ros2·kalibr) 는 실행 폴더가 아니라 여기 기준으로 찾는다
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROS_WS=${ROS_WS:-$HOME/ros2_ws}
# 장비별 캘리브 결과를 모아 두는 자리(실행 폴더와 별개). 최종 위치는 Yocto 설정 위치 결정에 따라 이 변수로 바꾼다.
calib_dir=${EDIE_CALIB_DIR:-$HOME/.edie/calib}
# 시험·특수 환경용: docker 실행 파일 이름(기본 docker)
DOCKER=${DOCKER:-docker}
camchain_file=${CAMCHAIN_FILE:-camchain.yaml}  # 환경변수로 override 가능: CAMCHAIN_FILE=camchain_raw.yaml
output_dir="$workdir/output"
mkdir -p "$output_dir"
# 이번 실행 시작 표식 — 게시할 Kalibr 결과를 "이 실행이 만든 것"으로 한정한다(옛 결과가 장비 캘리브 자리를 덮지 않게)
run_marker="$output_dir/.run_start"
touch "$run_marker"

# AprilGrid yaml은 allan 모드에서 불필요 — kalibr/all 모드에서만 검사
if [ "$mode" != "allan" ]; then
    if [ "$april" = "-" ] || [ ! -f "$april" ]; then
        echo "Error: AprilGrid YAML file '$april' not found" >&2
        exit 1
    fi
    april_abs=$(realpath "$april")
    april_base=$(basename "$april_abs")
    [ "$april_abs" != "$(realpath "$workdir/$april_base" 2>/dev/null)" ] && cp -f "$april_abs" "$workdir/$april_base"
    april="$april_base"
fi

echo "Starting calibration..."
echo "IMU: $imu_bag"
echo "Cam: $cam_bag"
echo "VIO: $vio_bag"
echo "Grid: $april"
echo ""

# 결과를 장비별 캘리브 자리(calib_dir)에도 둔다 — PC 에서 계산한 결과를 로봇에 넣을 때도 같은 경로를 쓰게.
# 사용: publish_results <실행 폴더의 파일 이름...> [--imucam]
#   모드마다 이번에 실제로 만들거나 쓴 파일만 넘긴다(allan → imu.yaml 하나). 실행 폴더의 옛 파일이 더 새 값을 덮지 않게.
#   --imucam: 이번 실행(run_marker 이후)이 만든 Kalibr 결과 <bag>-camchain-imucam.yaml·<bag>-results-imucam.txt·<bag>-imu.yaml
publish_results() {
    mkdir -p "$calib_dir"
    local f kf missing_imucam=0 found
    for f in "$@"; do
        if [ "$f" = "--imucam" ]; then
            found=0
            while IFS= read -r kf; do
                cp -f "$kf" "$calib_dir/"
                echo "  게시: $(basename "$kf")"
                found=1
            done < <(find "$workdir" -maxdepth 1 -newer "$run_marker" \( -name '*-camchain-imucam.yaml' -o -name '*-results-imucam.txt' -o -name '*-imu.yaml' \) | sort)
            if [ "$found" -eq 0 ]; then
                echo "Error: 이번 실행이 만든 Kalibr 결과(<bag>-camchain-imucam.yaml 등)를 $workdir 에서 못 찾았습니다 — Kalibr 가 결과를 쓰지 않았습니다." >&2
                missing_imucam=1
            fi
        elif [ -f "$workdir/$f" ]; then
            cp -f "$workdir/$f" "$calib_dir/$f"
            echo "  게시: $f"
        else
            echo "  게시 안 함(실행 폴더에 없음): $f"
        fi
    done
    echo "결과 게시 위치: $calib_dir (EDIE_CALIB_DIR 로 바꿀 수 있음)"
    return $missing_imucam
}

# check tools — Docker·rosbags-convert 는 Kalibr 단계에서만 쓴다. allan 모드는 둘 다 없어도 돈다(Docker 없는 로봇).
if [ "$mode" != "allan" ]; then
    if ! command -v "$DOCKER" &> /dev/null; then
        echo "Error: Kalibr 계산에는 Docker 와 kalibr:ros1 이미지가 필요한데 이 장비($(uname -m))에 '$DOCKER' 가 없습니다." >&2
        if [ "$mode" = "all" ]; then
            echo "  로봇이라면: all 은 IMU 정지 bag·카메라 bag·VIO bag 이 모두 필요합니다(allan 을 다시 돕니다). 세 bag 을 PC 로 옮겨 PC 에서 실행하거나," >&2
            echo "  로봇에서는 MODE=allan 으로 imu.yaml 만 만들고, VIO bag·imu.yaml·$camchain_file 을 PC 로 옮겨 MODE=kalibr 로 실행하세요." >&2
        else
            echo "  로봇이라면: VIO bag 과 imu.yaml·$camchain_file 을 PC 실행 폴더로 옮겨 PC 에서 같은 명령(MODE=$mode)을 실행하세요." >&2
        fi
        echo "  결과(<bag>-camchain-imucam.yaml·<bag>-results-imucam.txt)는 로봇의 \$EDIE_CALIB_DIR(기본 ~/.edie/calib)에 두면 됩니다." >&2
        exit 1
    fi
    if ! command -v rosbags-convert &> /dev/null; then
        echo "Error: rosbags-convert not found. Install with: pip install rosbags" >&2
        exit 1
    fi
    if [ "$(uname -m)" = "aarch64" ] && ! "$DOCKER" image inspect kalibr:ros1 > /dev/null 2>&1; then
        echo "Warning: aarch64 에서 kalibr:ros1 이미지를 새로 빌드합니다 — arm64 빌드는 검증되지 않았습니다(기본은 PC 계산)." >&2
    fi
fi

# timeoffset-padding: Kalibr의 cam-IMU time shift 탐색 범위 + spline 버퍼 크기
# padding은 spline 버퍼에도 영향 (buffer = [t_shifted_start - 2*padding, t_shifted_end + 2*padding]).
# pre-estimate가 50ms면 padding은 반드시 ≥ 50ms여야 버퍼 오버플로우를 막을 수 있음.
# 대신 timeshiftCamToImuPrior는 코드 패치로 직접 9ms로 오버라이드함.
compute_timeoffset_padding() {
    echo "0.05 (fixed: spline buffer requires >=50ms; timeshift overridden separately)"
}

# kalibr 모드는 allan 전체 스킵
if [ "$mode" = "kalibr" ] || [ "$mode" = "sweep" ]; then
    echo "Kalibr VIO calibration mode (allan variance 스킵)"
else

# allan variance setup
# allan_ros2·파이썬 의존성 준비 — 주기 측정(rosbag2_py)도 ROS 환경이 필요하므로 가장 먼저 한다.
# 이미 설치돼 있으면(로봇·install-only 배포) 실행 중 pip·colcon 을 하지 않는다.
if [ -z "$(command -v ros2)" ] && [ -f "$ROS_WS/install/setup.bash" ]; then
    source "$ROS_WS/install/setup.bash"
fi
if ! python3 -c "import matplotlib, numpy, scipy, yaml" 2>/dev/null; then
    echo "analysis.py 의존 모듈 설치(matplotlib numpy scipy pyyaml)..."
    pip3 install matplotlib numpy scipy pyyaml
fi
allan_prefix=$(ros2 pkg prefix allan_ros2 2>/dev/null || true)
if [ -z "$allan_prefix" ] || [ "${ALLAN_REBUILD:-0}" = "1" ]; then
    # 소스 체크아웃에서만 가능한 경로 — 스크립트 옆의 external/allan_ros2 만 골라 ROS_WS 에 빌드한다
    allan_ros2_path="$script_dir/external/allan_ros2"
    if [ ! -f "$allan_ros2_path/package.xml" ]; then
        echo "Error: allan_ros2 를 빌드할 소스가 없습니다($allan_ros2_path). git submodule update --init external/allan_ros2 를 하세요." >&2
        exit 1
    fi
    echo "allan_ros2 빌드: $allan_ros2_path → $ROS_WS/install"
    (cd "$ROS_WS" && rosdep install --from-paths "$allan_ros2_path" -y --ignore-src --skip-keys px4_msgs \
        && colcon build --symlink-install --base-paths "$allan_ros2_path" --packages-select allan_ros2)
    source "$ROS_WS/install/setup.bash"
    allan_prefix=$(ros2 pkg prefix allan_ros2)
fi
allan_bin="$allan_prefix/lib/allan_ros2/allan_node"
echo "allan_node: $allan_bin ($(date -r "$allan_bin" '+%Y-%m-%d %H:%M' 2>/dev/null || echo '시각 모름'))"
if [ ! -f "$allan_prefix/lib/allan_ros2/analysis.py" ]; then
    echo "Warning: 설치된 allan_ros2 가 옛 빌드입니다(analysis.py 미설치 = HERoEHS edie9 515693f 이전). ALLAN_REBUILD=1 로 다시 빌드하세요." >&2
fi
analysis_py="$allan_prefix/lib/allan_ros2/analysis.py"
[ -f "$analysis_py" ] || analysis_py="$script_dir/external/allan_ros2/scripts/analysis.py"
if [ ! -f "$analysis_py" ]; then
    echo "Error: analysis.py 를 찾지 못했습니다 — 설치본(옛 빌드)에도, 스크립트 옆 external/allan_ros2 에도 없습니다." >&2
    echo "  ALLAN_REBUILD=1 로 다시 빌드하거나 git submodule update --init external/allan_ros2 를 하세요." >&2
    exit 1
fi
echo "Running allan variance"

# params file for allan (공통 사용: ROS2 노드 + analysis.py)
# 결과 폴더에 새로 만든다 — 저장소 안 config.yaml 을 덮어쓰면 git 이 더러워지고, 설치본(share)은 읽기 전용이다.
allan_cfg="$output_dir/allan_params.yaml"

# bag에서 실제 샘플레이트 자동 계산 (header.stamp 기준). IMU_RATE=<Hz> 로 직접 줄 수도 있다.
if [ -n "${IMU_RATE:-}" ]; then
    imu_rate=$IMU_RATE
    echo "IMU rate: ${imu_rate} Hz (IMU_RATE 지정)"
else
echo "Detecting actual IMU sample rate from bag..."
imu_rate=$(python3 - <<PYEOF
import os, sys
# rosbag2(C++) 가 stdout 에 "closing." 같은 줄을 찍어 결과를 오염시킨다 → 표준출력을 stderr 로 돌리고 결과만 원래 출력으로
_result_fd = os.dup(1)
os.dup2(2, 1)
def stamps_rosbags(path, topic):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    ts = []
    with Reader(path) as r:
        conns = [c for c in r.connections if c.topic == topic]
        if not conns:  # 빈 목록이면 rosbags 가 "필터 없음"으로 모든 토픽을 읽는다 — 반드시 막는다
            raise LookupError(f"bag 에 토픽 {topic} 없음 (있는 토픽: {sorted({c.topic for c in r.connections})})")
        for conn, t, raw in r.messages(connections=conns):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            ts.append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
            if len(ts) > 5000:  # 5000샘플이면 충분
                break
    return ts
def stamps_rosbag2_py(path, topic):
    # 로봇처럼 pip rosbags 가 없는 곳 — ROS2 에 들어 있는 rosbag2_py 로 읽는다
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    topics = sorted(t.name for t in r.get_all_topics_and_types())
    if topic not in topics:
        raise LookupError(f"bag 에 토픽 {topic} 없음 (있는 토픽: {topics})")
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    ts = []
    while r.has_next() and len(ts) <= 5000:
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Imu)
        ts.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)
    return ts
import numpy as np
for fn in (stamps_rosbags, stamps_rosbag2_py):
    try:
        ts = fn("$imu_bag", "$imu_topic")
        if len(ts) >= 10:
            os.write(_result_fd, str(round(1e9 / np.mean(np.diff(ts[:5000])))).encode())
            sys.exit(0)
    except Exception as e:
        print(f"[rate] {fn.__name__} 실패: {e}", file=sys.stderr)
print("[rate] Error: bag 에서 IMU 주기를 재지 못했습니다 — 토픽 이름을 확인하거나 IMU_RATE=<Hz> 로 지정하세요", file=sys.stderr)
sys.exit(2)
PYEOF
) || { echo "Error: IMU 주기 측정 실패(위 [rate] 메시지 참고)" >&2; exit 1; }
fi
case "$imu_rate" in
    ''|*[!0-9]*) echo "Error: IMU 주기 값이 정수가 아닙니다: '$imu_rate'" >&2; exit 1 ;;
esac
echo "Detected IMU rate: ${imu_rate} Hz"

cat > "$allan_cfg" <<EOF
allan_node:
  ros__parameters:
    topic: $imu_topic
    bag_path: $imu_bag
    msg_type: ros
    publish_rate: $imu_rate
    sample_rate: $imu_rate
EOF

# run allan analysis from output_dir so deviation.csv, plots, imu.yaml 모두 그 안에 생성되도록
cd "$output_dir"
echo "Running allan variance..."
rm -f "$output_dir/deviation.csv"
# 'ros2 run' 래퍼를 거치지 않고 바이너리를 직접 띄운다 — 래퍼는 SIGINT 를 자식에게 넘기지 않아
# 래퍼만 죽고 allan_node 가 고아로 남았다(약 0.5 GB). $! 가 노드 자신이어야 아래 종료가 닿는다.
"$allan_bin" --ros-args --params-file "$allan_cfg" &
allan_pid=$!
# 중간에 실패해도 노드를 남기지 않는다 — 계산 중엔 SIGINT 를 늦게 받으므로 잠시 뒤 강제 종료
trap 'kill -INT $allan_pid 2>/dev/null; sleep 1; kill -9 $allan_pid 2>/dev/null || true' EXIT

# deviation.csv 가 만들어질 때까지 대기 — 노드가 먼저 끝나거나 상한(ALLAN_TIMEOUT 초)을 넘으면 실패
allan_timeout=${ALLAN_TIMEOUT:-1800}
waited=0
while [ ! -s "$output_dir/deviation.csv" ]; do
    if ! kill -0 $allan_pid 2>/dev/null; then
        echo "Error: allan_node 가 deviation.csv 없이 끝났습니다(토픽·bag 경로 확인)" >&2
        exit 1
    fi
    if [ "$waited" -ge "$allan_timeout" ]; then
        echo "Error: ${allan_timeout}s 안에 deviation.csv 가 생기지 않았습니다(ALLAN_TIMEOUT 으로 늘릴 수 있음)" >&2
        exit 1
    fi
    sleep 1
    waited=$((waited + 1))
done

# 파일 flush 시간 조금 주고 노드 종료(SIGINT → rclcpp 정상 종료, 안 되면 강제)
sleep 2
kill -INT $allan_pid 2>/dev/null || true
for _ in 1 2 3 4 5; do kill -0 $allan_pid 2>/dev/null || break; sleep 1; done
kill -9 $allan_pid 2>/dev/null || true
wait $allan_pid 2>/dev/null || true
trap - EXIT

echo "Allan done."

# generate imu calib
echo "Generating IMU params..."
if [ -f "$output_dir/deviation.csv" ]; then
    # output_dir 안에서 실행하여 imu.yaml, acceleration.png, gyro.png 모두 output_dir에 생성
    cd "$output_dir"
    python3 "$analysis_py" --data deviation.csv --config "$allan_cfg"

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
    publish_results imu.yaml
    echo "Done!"
    exit 0
fi

fi  # end of "kalibr 모드가 아닐 때" allan 블록

# MODE이 kalibr이면 allan 스킵하고 VIO 캘립만 실행
if [ "$mode" = "kalibr" ]; then
    echo "Kalibr VIO calibration mode (allan variance 스킵)"

    # imu.yaml 자동 탐색 (root → output/)
    if [ ! -f "$workdir/imu.yaml" ]; then
        if [ -f "$workdir/output/imu.yaml" ]; then
            cp "$workdir/output/imu.yaml" "$workdir/imu.yaml"
            echo "imu.yaml: output/imu.yaml 에서 복사"
        else
            echo "Error: imu.yaml 없음. allan 모드를 먼저 실행하세요." >&2
            exit 1
        fi
    else
        echo "imu.yaml: 기존 파일 사용"
    fi

    # camchain.yaml 확인
    if [ ! -f "$workdir/$camchain_file" ]; then
        echo "Error: $camchain_file 없음. 카메라 캘립을 먼저 실행하세요." >&2
        exit 1
    fi
    echo "$camchain_file: 기존 파일 사용"

    # april yaml 복사 (docker mount 경로)
    april_abs=$(realpath "$april")
    april_base=$(basename "$april_abs")
    [ "$april_abs" != "$(realpath "$workdir/$april_base" 2>/dev/null)" ] && cp -f "$april_abs" "$workdir/$april_base"
    april="$april_base"

    # kalibr docker 이미지 빌드 (없는 경우)
    if ! "$DOCKER" image inspect kalibr:ros1 > /dev/null 2>&1; then
        echo "kalibr docker 이미지 빌드 중..."
        "$DOCKER" build -t kalibr:ros1 "$script_dir/external/kalibr" -f "$script_dir/external/kalibr/Dockerfile_ros1_20_04"
        if [ $? -ne 0 ]; then
            echo "Docker build failed" >&2
            exit 1
        fi
    fi

    # ROS2 bag → ROS1 bag 변환
    ros1_vio="$workdir/vio_ros1.bag"
    rm -f "$ros1_vio"
    echo "ROS2 bag → ROS1 변환 중..."
    rosbags-convert --src "$vio_bag" --dst "$ros1_vio"
    echo "변환 완료."

    # ── AprilTag 규격 확인 ────────────────────────────────────────────────
    read -r -p "타깃이 표준 AprilTag36h11 (1-cell black border) 입니까? [Y/n]: " _tag_reply
    _tag_reply="${_tag_reply:-Y}"
    _apply_border=0
    if [[ "$_tag_reply" =~ ^[Yy]$ ]]; then
        _apply_border=1
        echo "  → blackTagBorder=1 패치 적용"
    else
        echo "  → blackTagBorder Kalibr 기본값 사용 (2-cell)"
    fi

    # ── timeshift prior 결정 ─────────────────────────────────────────────
    if [ "$timeshift_arg" != "auto" ]; then
        _timeshift_s=$(python3 -c "print(f'{float(\"$timeshift_arg\")/1000:.6f}')")
        echo "timeshift prior: ${timeshift_arg}ms (직접 지정)"
    else
        echo "bag 타임스탬프 분석으로 timeshift 자동 추정 중..."
        _timeshift_s=$(python3 - "$vio_bag" "$imu_topic" << 'PYEOF'
import sys, numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
bag_path, imu_topic = sys.argv[1], sys.argv[2]
typestore = get_typestore(Stores.ROS2_HUMBLE)
cam_ts, imu_ts, cam_topic = [], [], None
with Reader(bag_path) as r:
    for c in r.connections:
        if 'image' in c.topic and cam_topic is None:
            cam_topic = c.topic
    if cam_topic is None:
        print("0.009"); sys.exit(0)
    for conn, t, raw in r.messages():
        if conn.topic == cam_topic and len(cam_ts) < 300:
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            cam_ts.append(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        elif conn.topic == imu_topic and len(imu_ts) < 10000:
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            imu_ts.append(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
if len(cam_ts) < 10 or len(imu_ts) < 10:
    print("0.009"); sys.exit(0)
cam_ts = np.array(sorted(cam_ts))
imu_ts = np.array(sorted(imu_ts))
# shift = t_imu - t_cam (nearest IMU stamp after each camera stamp)
diffs = [(imu_ts[np.searchsorted(imu_ts, ct)] - ct) / 1e9
         for ct in cam_ts if np.searchsorted(imu_ts, ct) < len(imu_ts)]
est = float(np.median(diffs)) if diffs else 0.009
print(f"{max(-0.1, min(0.1, est)):.6f}")
PYEOF
        )
        echo "  → 추정된 timeshift prior: $(python3 -c "print(f'{float(\"$_timeshift_s\")*1000:.2f}ms')")"
    fi

    # ── Kalibr 패치 파일 생성 (Docker 내에서 실행될 Python 스크립트) ──
    # 첫 번째 heredoc: shell 변수 _apply_border, _timeshift_s 주입
    cat > "$workdir/kalibr_patch.py" << PATCH_HEAD
ts = ${_timeshift_s}
apply_border = ${_apply_border}
PATCH_HEAD
    # 두 번째 heredoc: 따옴표 탈출 없이 Python 코드 그대로 (single-quoted delimiter)
    cat >> "$workdir/kalibr_patch.py" << 'PATCH_BODY'
path = '/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py'
with open(path) as f:
    code = f.read()
if apply_border:
    code = code.replace(
        'options.showExtractionVideo = showExtraction\n            options.minTagsForValidObs',
        'options.showExtractionVideo = showExtraction\n            options.blackTagBorder = 1\n            options.minTagsForValidObs')
    print('patched blackTagBorder=1')
code = code.replace(
    '        self.timeshiftCamToImuPrior = shift\n        \n',
    '        self.timeshiftCamToImuPrior = shift\n        self.timeshiftCamToImuPrior = ' + str(ts) +
    '\n        print("[PATCH] Timeshift prior: ' + str(round(ts * 1000, 2)) + 'ms")\n        \n')
print(f'patched timeshiftCamToImuPrior={ts:.6f}s ({ts*1000:.2f}ms)')
with open(path, 'w') as f:
    f.write(code)
PATCH_BODY

    # kalibr IMU-camera 캘리브레이션
    echo "IMU-camera 캘리브레이션 실행 중..."
    echo "Measuring cam-IMU time offset for padding..."
    timeoffset_padding=$(compute_timeoffset_padding "$vio_bag" | awk '{print $1}')
    echo "timeoffset-padding: ${timeoffset_padding}s"
    "$DOCKER" run --rm -v "$workdir":/data -w /data --entrypoint bash kalibr:ros1 \
        -c "export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 && \
            source /catkin_ws/devel/setup.bash && \
            python3 /data/kalibr_patch.py && \
            /catkin_ws/devel/lib/kalibr/kalibr_calibrate_imu_camera --bag /data/vio_ros1.bag --target /data/$april --cam /data/$camchain_file --imu /data/imu.yaml --timeoffset-padding $timeoffset_padding"
    rm -f "$workdir/kalibr_patch.py"

    if [ $? -ne 0 ]; then
        echo "VIO calib failed" >&2
        exit 1
    fi

    echo ""
    echo "Kalibr VIO calibration 완료!"
    echo "결과 파일: <bag>-camchain-imucam.yaml (T_cam_imu 포함), <bag>-results-imucam.txt"
    publish_results imu.yaml "$camchain_file" --imucam
    rm -f "$ros1_vio"
    # workdir 루트에 복사된 april yaml 정리 (원본은 data/ 에 유지)
    [ "$april_abs" != "$workdir/$april_base" ] && rm -f "$workdir/$april_base"
    echo "Done!"
    exit 0
fi

# ── sweep モード ─────────────────────────────────────────────────────────────
if [ "$mode" = "sweep" ]; then
    echo "tagSize sweep モード — reprojection error 최솟값 탐색 (21~27mm, 1mm 간격)"

    # imu.yaml 확인
    if [ ! -f "$workdir/imu.yaml" ]; then
        if [ -f "$workdir/output/imu.yaml" ]; then
            cp "$workdir/output/imu.yaml" "$workdir/imu.yaml"
            echo "imu.yaml: output/imu.yaml 에서 복사"
        else
            echo "Error: imu.yaml 없음. allan 모드를 먼저 실행하세요." >&2
            exit 1
        fi
    else
        echo "imu.yaml: 기존 파일 사용"
    fi

    # camchain.yaml 확인
    if [ ! -f "$workdir/$camchain_file" ]; then
        echo "Error: $camchain_file 없음. 카메라 캘립을 먼저 실행하세요." >&2
        exit 1
    fi
    echo "$camchain_file: 기존 파일 사용"

    # april yaml 복사 (docker mount 경로)
    april_abs=$(realpath "$april")
    april_base=$(basename "$april_abs")
    [ "$april_abs" != "$(realpath "$workdir/$april_base" 2>/dev/null)" ] && cp -f "$april_abs" "$workdir/$april_base"
    april="$april_base"

    # kalibr docker 이미지 빌드 (없는 경우)
    if ! "$DOCKER" image inspect kalibr:ros1 > /dev/null 2>&1; then
        echo "kalibr docker 이미지 빌드 중..."
        "$DOCKER" build -t kalibr:ros1 "$script_dir/external/kalibr" -f "$script_dir/external/kalibr/Dockerfile_ros1_20_04"
        if [ $? -ne 0 ]; then
            echo "Docker build failed" >&2
            exit 1
        fi
    fi

    # ROS2 bag → ROS1 변환
    ros1_vio="$workdir/vio_ros1.bag"
    rm -f "$ros1_vio"
    echo "ROS2 bag → ROS1 변환 중..."
    rosbags-convert --src "$vio_bag" --dst "$ros1_vio"
    echo "변환 완료."

    # AprilTag 규격 확인
    read -r -p "타깃이 표준 AprilTag36h11 (1-cell black border) 입니까? [Y/n]: " _tag_reply
    _tag_reply="${_tag_reply:-Y}"
    _apply_border=0
    if [[ "$_tag_reply" =~ ^[Yy]$ ]]; then
        _apply_border=1
        echo "  → blackTagBorder=1 패치 적용"
    else
        echo "  → blackTagBorder Kalibr 기본값 사용 (2-cell)"
    fi

    # timeoffset-padding
    timeoffset_padding=$(compute_timeoffset_padding "$vio_bag" | awk '{print $1}')

    # timeshift prior 결정
    if [ "$timeshift_arg" != "auto" ]; then
        _timeshift_s=$(python3 -c "print(f'{float(\"$timeshift_arg\")/1000:.6f}')")
        echo "timeshift prior: ${timeshift_arg}ms (직접 지정)"
    else
        echo "bag 타임스탬프 분석으로 timeshift 자동 추정 중..."
        _timeshift_s=$(python3 - "$vio_bag" "$imu_topic" << 'PYEOF'
import sys, numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
bag_path, imu_topic = sys.argv[1], sys.argv[2]
typestore = get_typestore(Stores.ROS2_HUMBLE)
cam_ts, imu_ts, cam_topic = [], [], None
with Reader(bag_path) as r:
    for c in r.connections:
        if 'image' in c.topic and cam_topic is None:
            cam_topic = c.topic
    if cam_topic is None:
        print("0.009"); sys.exit(0)
    for conn, t, raw in r.messages():
        if conn.topic == cam_topic and len(cam_ts) < 300:
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            cam_ts.append(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        elif conn.topic == imu_topic and len(imu_ts) < 10000:
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            imu_ts.append(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
if len(cam_ts) < 10 or len(imu_ts) < 10:
    print("0.009"); sys.exit(0)
cam_ts = np.array(sorted(cam_ts))
imu_ts = np.array(sorted(imu_ts))
diffs = [(imu_ts[np.searchsorted(imu_ts, ct)] - ct) / 1e9
         for ct in cam_ts if np.searchsorted(imu_ts, ct) < len(imu_ts)]
est = float(np.median(diffs)) if diffs else 0.009
print(f"{max(-0.1, min(0.1, est)):.6f}")
PYEOF
        )
        echo "  → 추정된 timeshift prior: $(python3 -c "print(f'{float(\"$_timeshift_s\")*1000:.2f}ms')")"
    fi

    # 원본 aprilgrid yaml에서 tagCols, tagRows, tagSpacing 읽기
    _april_params=$(python3 -c "
import yaml
with open('$workdir/$april') as f:
    d = yaml.safe_load(f)
print(d.get('tagCols', 6), d.get('tagRows', 6), d.get('tagSpacing', 0.3))
")
    _tagCols=$(echo "$_april_params" | awk '{print $1}')
    _tagRows=$(echo "$_april_params" | awk '{print $2}')
    _tagSpacing=$(echo "$_april_params" | awk '{print $3}')
    echo "AprilGrid: ${_tagCols}×${_tagRows} tags, spacing=${_tagSpacing}"

    # 결과 파일 초기화
    RESULTS_FILE="$workdir/output/tagsize_sweep_results.txt"
    mkdir -p "$workdir/output"
    printf "%-12s | %-16s | %-16s\n" "tagSize(mm)" "cam0_mean(px)" "cam1_mean(px)" > "$RESULTS_FILE"
    printf -- "%-12s-+-%-16s-+-%-16s\n" "------------" "----------------" "----------------" >> "$RESULTS_FILE"

    BEST_MM=""
    BEST_ERR="99999"

    for MM in 21 22 23 24 25 26 27; do
        TAGSIZE=$(echo "scale=4; $MM / 1000" | bc)
        APRIL_YAML="$workdir/april_sweep_${MM}mm.yaml"
        OUTPUT_DIR="$workdir/output/sweep_${MM}mm"
        mkdir -p "$OUTPUT_DIR"

        cat > "$APRIL_YAML" << SWEEP_APRIL
target_type: 'aprilgrid'
tagCols: ${_tagCols}
tagRows: ${_tagRows}
tagSize: ${TAGSIZE}
tagSpacing: ${_tagSpacing}
SWEEP_APRIL

        echo ""
        echo "=========================================="
        echo "tagSize = ${MM}mm (${TAGSIZE}m)"
        echo "=========================================="

        # Kalibr 패치 파일 생성 (kalibr 모드와 동일한 패치)
        cat > "$workdir/kalibr_patch.py" << PATCH_HEAD
ts = ${_timeshift_s}
apply_border = ${_apply_border}
PATCH_HEAD
        cat >> "$workdir/kalibr_patch.py" << 'PATCH_BODY'
path = '/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py'
with open(path) as f:
    code = f.read()
if apply_border:
    code = code.replace(
        'options.showExtractionVideo = showExtraction\n            options.minTagsForValidObs',
        'options.showExtractionVideo = showExtraction\n            options.blackTagBorder = 1\n            options.minTagsForValidObs')
    print('patched blackTagBorder=1')
code = code.replace(
    '        self.timeshiftCamToImuPrior = shift\n        \n',
    '        self.timeshiftCamToImuPrior = shift\n        self.timeshiftCamToImuPrior = ' + str(ts) +
    '\n        print("[PATCH] Timeshift prior: ' + str(round(ts * 1000, 2)) + 'ms")\n        \n')
print(f'patched timeshiftCamToImuPrior={ts:.6f}s ({ts*1000:.2f}ms)')
with open(path, 'w') as f:
    f.write(code)
PATCH_BODY

        set +e
        "$DOCKER" run --rm \
            -v "$workdir":/data \
            -v "$OUTPUT_DIR":/output \
            -w /output \
            --entrypoint bash \
            -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
            kalibr:ros1 \
            -c "export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 && \
                source /catkin_ws/devel/setup.bash && \
                python3 /data/kalibr_patch.py && \
                /catkin_ws/devel/lib/kalibr/kalibr_calibrate_imu_camera \
                  --bag /data/vio_ros1.bag \
                  --target /data/april_sweep_${MM}mm.yaml \
                  --cam /data/$camchain_file \
                  --imu /data/imu.yaml \
                  --timeoffset-padding $timeoffset_padding \
                  \
                2>&1" | tee "$OUTPUT_DIR/kalibr.log"
        set -e

        # reprojection error 추출 (trailing comma 제거)
        CAM0=$(grep "Reprojection error (cam0) \[px\]" "$OUTPUT_DIR/kalibr.log" 2>/dev/null | awk '{print $6}' | tail -1 | tr -d ',')
        CAM1=$(grep "Reprojection error (cam1) \[px\]" "$OUTPUT_DIR/kalibr.log" 2>/dev/null | awk '{print $6}' | tail -1 | tr -d ',')

        printf "%-12s | %-16s | %-16s\n" "${MM}mm" "${CAM0:-N/A}" "${CAM1:-N/A}" >> "$RESULTS_FILE"
        echo ">>> tagSize ${MM}mm: cam0=${CAM0:-N/A}px, cam1=${CAM1:-N/A}px"

        # 최솟값 추적 (cam0 기준)
        if [ -n "$CAM0" ] && [ "$CAM0" != "N/A" ]; then
            IS_BETTER=$(python3 -c "print('yes' if float('${CAM0}') < float('${BEST_ERR}') else 'no')" 2>/dev/null || echo "no")
            if [ "$IS_BETTER" = "yes" ]; then
                BEST_ERR="$CAM0"
                BEST_MM="$MM"
            fi
        fi

        rm -f "$APRIL_YAML" "$workdir/kalibr_patch.py"
    done

    echo ""
    echo "=========================================="
    echo "sweep 완료. 결과:"
    cat "$RESULTS_FILE"
    echo "=========================================="
    if [ -n "$BEST_MM" ]; then
        BEST_TAGSIZE=$(echo "scale=4; $BEST_MM / 1000" | bc)
        echo "최솟값: tagSize=${BEST_MM}mm → cam0 mean=${BEST_ERR}px"
        echo "→ aprilgrid yaml의 tagSize를 ${BEST_TAGSIZE}m 로 업데이트 후 kalibr 모드를 재실행하세요."
        echo "   bash pipeline.sh - - <vio_bag> <aprilgrid_updated.yaml> kalibr"
    else
        echo "모든 tagSize에서 Kalibr 실패 — 로그 확인: $workdir/output/sweep_*mm/kalibr.log"
    fi
    echo "상세 로그: $workdir/output/sweep_*mm/kalibr.log"

    rm -f "$ros1_vio"
    [ "$april_abs" != "$workdir/$april_base" ] && rm -f "$workdir/$april_base"
    exit 0
fi

# ROS2 bag → ROS1 bag 변환 (Kalibr는 ROS1 bag만 지원)
echo "Converting bags to ROS1 format..."
ros1_cam="cam_ros1.bag"
ros1_vio="vio_ros1.bag"

rosbags-convert --src "$cam_bag" --dst "$ros1_cam"
rosbags-convert --src "$vio_bag" --dst "$ros1_vio"

echo "Bags converted."
echo ""

# camera calib
echo "Camera calibration..."

if ! "$DOCKER" image inspect kalibr:ros1 > /dev/null 2>&1; then
    echo "Building kalibr docker..."
    "$DOCKER" build -t kalibr:ros1 "$script_dir/external/kalibr" -f "$script_dir/external/kalibr/Dockerfile_ros1_20_04"
    
    if [ $? -ne 0 ]; then
        echo "Docker build failed" >&2
        exit 1
    fi
fi

echo "Running cam calib..."
"$DOCKER" run --rm -v "$workdir":/data -it kalibr:ros1 bash -c \
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
echo "Measuring cam-IMU time offset for padding..."
timeoffset_padding=$(compute_timeoffset_padding "$vio_bag" | awk '{print $1}')
echo "timeoffset-padding: ${timeoffset_padding}s"

"$DOCKER" run --rm -v "$workdir":/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_imu_camera --bag /data/$ros1_vio --target /data/$april --cam /data/$camchain_file --imu /data/imu.yaml --timeoffset-padding $timeoffset_padding"

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

publish_results imu.yaml camchain.yaml --imucam
echo ""
echo "Ready for VINS-Fusion etc."
echo ""

# cleanup
echo "Cleaning up..."
rm -f "$ros1_cam" "$ros1_vio"

echo "Done!"
