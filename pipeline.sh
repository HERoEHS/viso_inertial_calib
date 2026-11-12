#!/usr/bin/env bash
set -e

# ─────────────────────────────────────────────────────────────────────────────
# 셔터 종류에 따라 달라지는 것은 7번째 인자(시간차 시작값) 하나뿐이다. 나머지 인자·설정은 같다.
#
#   글로벌 셔터(화면 전체를 한순간에 찍음)  → 7번째 인자 생략 또는 auto
#       Kalibr 가 스스로 추정한다. 옛 카메라 녹화는 auto 로 9.1 ms 가 나왔다(11-12 재계산).
#   롤링 셔터(위에서 아래로 한 줄씩 찍음)   → 7번째 인자에 숫자(ms)를 꼭 준다
#       값 = "영상 stamp → 첫 줄" 시간 + 읽기 시간 / 2   (화면 가운데 줄이 찍힌 순간)
#       로봇 .224 새 카메라: 3.2 + 64.4 / 2 ≈ 35  → ... kalibr /edie/sensor/offset_imu 35
#       auto 로 두면 Kalibr 추정이 크게 빗나가(.224 실측 −18.5 ms) 탐색 범위를 벗어나 종료 코드 6 으로 멈춘다.
#
# 왜 이것뿐인가: Kalibr 의 카메라–IMU 계산에는 롤링 셔터 모델이 없다(셔터 종류를 넘길 인자가 없다).
#   그래서 롤링 셔터는 "화면 가운데 줄 시각"을 영상 시각으로 보는 근사로 풀고, 그 차이가 시간차로 나타난다.
#   남는 오차를 줄이려면 녹화할 때 천천히 움직인다(빠를수록 보드가 휘어 찍힌다).
#   카메라 종류·읽기 시간은 edie_vision config/gstreamer/camera_v4l2.yaml 의 shutter_type·rolling_shutter 에 있다.
# ─────────────────────────────────────────────────────────────────────────────

if [ "$#" -lt 4 ] || [ "$#" -gt 7 ]; then
  echo "Usage: $0 BAG_IMU BAG_CAM BAG_VIO APRILGRID_YAML [MODE] [IMU_TOPIC] [TIMESHIFT_MS|auto]" >&2
  echo "  MODE: all (default) | allan | kalibr | sweep" >&2
  echo "  IMU_TOPIC: IMU 토픽 (기본값: /edie/sensor/offset_imu, allan 용 — Kalibr 는 imu.yaml 의 rostopic 을 쓴다)" >&2
  echo "  TIMESHIFT_MS: 카메라→IMU 시간차 시작값(ms). auto(기본) = Kalibr 자체 추정(각속도 크기 교차상관)" >&2
  echo "          롤링셔터 카메라는 화면 가운데 줄 기준이라 약 +읽기시간/2 가 더해진다(.224: 약 35 ms)." >&2
  echo "          롤링셔터면 auto 가 크게 빗나가 멈출 수 있다(.224 실측 −18.5 ms → 종료 코드 6) — 숫자로 주자" >&2
  echo "  셔터 종류별 7번째 인자(다른 인자는 같다):" >&2
  echo "          글로벌 셔터 → 생략 또는 auto" >&2
  echo "          롤링 셔터   → 숫자(ms) = stamp→첫 줄 시간 + 읽기 시간/2   (.224: 3.2 + 64.4/2 ≈ 35)" >&2
  echo "  kalibr: allan 스킵, 기존 imu.yaml+camchain.yaml 사용, VIO 캘립만 실행" >&2
  echo "          BAG_IMU, BAG_CAM 은 '-' 로 생략 가능. BAG_VIO 가 ROS1 .bag 파일이면 변환 없이 그대로 쓴다" >&2
  echo "          본계산 전 모서리 배치 자동 점검(멈추면 종료 코드 3), 최적화 전 재투영 관문(종료 코드 4)," >&2
  echo "          끝난 뒤 결과 요약 kalibr_quality.txt(FAIL 이면 게시 안 함, 종료 코드 5)" >&2
  echo "  환경변수(kalibr·sweep): KALIBR_TAG_BORDER=1|2(테두리 질문 생략), KALIBR_CORNER_LAYOUT=auto|identity|rot180|...," >&2
  echo "          KALIBR_CHECK_ONLY=1(배치 점검만), KALIBR_CHECK_FRAMES=20, KALIBR_GATE_PX=5, KALIBR_REF_CAMCHAIN=<기준 yaml>" >&2
  echo "  sweep:  tagSize 21~27mm 범위를 1mm 간격으로 Kalibr 반복 실행, reprojection error 최솟값 탐색" >&2
  echo "          kalibr 모드 전제 (imu.yaml + camchain.yaml 필요), BAG_IMU, BAG_CAM 은 '-' 로 생략 가능" >&2
  echo "Example: $0 imu_stationary.bag camera_intrinsics.bag vio_calibration.bag aprilgrid.yaml allan" >&2
  echo "Example: $0 imu_stationary.bag - - aprilgrid.yaml allan /edie/sensor/offset_imu" >&2
  echo "Example: $0 - - vio_calibration.bag aprilgrid.yaml kalibr" >&2
  echo "Example: $0 - - vio_calibration.bag aprilgrid.yaml kalibr /edie/sensor/offset_imu 35   # 롤링 셔터(.224)" >&2
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
# VIO bag 이 이미 ROS1 .bag 파일이면(예: 전에 변환해 둔 것) kalibr·sweep 은 다시 변환하지 않는다 — 2 GB 넘는 파일을 또 만들지 않게.
vio_is_ros1=0
if [ "$mode" != "allan" ] && [ -f "$vio_bag" ] && [[ "$vio_bag" == *.bag ]]; then
    vio_is_ros1=1
fi

workdir=$(pwd)
# 스크립트가 있는 폴더 — external/(allan_ros2·kalibr) 는 실행 폴더가 아니라 여기 기준으로 찾는다
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROS_WS=${ROS_WS:-$HOME/ros2_ws}
# 장비별 캘리브 결과를 모아 두는 자리(실행 폴더와 별개) — 로봇이 "만드는" 파일이라 데이터 폴더 규칙을 따른다(SW1-1959):
#   EDIE_CALIB_DIR 가 있으면 그대로(최우선). 없으면 root(edieOS)이고 /var/lib/edie 를 만들어 쓸 수 있을 때
#   /var/lib/edie/data/calib, 그 밖(Ubuntu edie 계정)은 예전 그대로 $HOME/.edie/calib. 규칙은 lib/calib_dir.sh.
#   설치본(/opt/edie/current/share)과는 다른 자리다 — 거기엔 로봇이 "읽는" 캘리브 yaml 만 둔다.
# shellcheck source=lib/calib_dir.sh
. "$script_dir/lib/calib_dir.sh"
calib_dir=$(edie_resolve_calib_dir)
export EDIE_CALIB_DIR="$calib_dir"   # 자식 프로세스가 같은 값을 보게(지금 이 변수를 읽는 자식은 없음 — allan 설정은 259줄쯤에서 bag_path 를 직접 적는다)
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
        echo "  결과(<bag>-camchain-imucam.yaml·<bag>-results-imucam.txt)는 로봇의 \$EDIE_CALIB_DIR(기본: edieOS /var/lib/edie/data/calib, Ubuntu ~/.edie/calib)에 두면 됩니다." >&2
        exit 1
    fi
    if { [ "$mode" = "all" ] || [ "$vio_is_ros1" = "0" ]; } && ! command -v rosbags-convert &> /dev/null; then
        echo "Error: rosbags-convert not found. Install with: pip install rosbags" >&2
        exit 1
    fi
    if [ "$(uname -m)" = "aarch64" ] && ! "$DOCKER" image inspect kalibr:ros1 > /dev/null 2>&1; then
        echo "Warning: aarch64 에서 kalibr:ros1 이미지를 새로 빌드합니다 — arm64 빌드는 검증되지 않았습니다(기본은 PC 계산)." >&2
    fi
fi

# timeoffset-padding: Kalibr의 cam-IMU time shift 탐색 범위 + spline 버퍼 크기
# padding은 spline 버퍼에도 영향 (buffer = [t_shifted_start - 2*padding, t_shifted_end + 2*padding]).
# 최적화 중 시간차는 시작값에서 ±padding 안에서만 움직인다 → 시작값이 실제와 50 ms 넘게 틀리면 못 따라간다.
# (옛 판은 시작값을 9 ms 등으로 덮어썼다. 그때 Kalibr 추정 50 ms 를 의심한 것은 모서리 배치가 틀린 35 px 상태였다 —
#  2026-11-12 조사. 이제 TIMESHIFT_MS=auto 면 Kalibr 자체 추정을 그대로 쓴다.)
compute_timeoffset_padding() {
    echo "0.05 (fixed: spline buffer requires >=50ms)"
}

# ── Kalibr imu-cam 공통 준비(kalibr·sweep 모드) ─────────────────────────────────────────
# 왜(2026-11-12 .224 사고): 보드 그림 방향이 Kalibr 기대와 다르면 검출기가 태그마다 모서리 번호를 엇갈려 붙여
# 재투영 49 px 로 발산하는데도 Kalibr 는 결과를 다 쓰고 "완료" 했다. 어긋나는 방식은 녹화마다 달랐다(180°·좌우).
# 그래서 ① 본계산 전 8가지 배치 점검·선택 ② 최적화 전 재투영 관문 ③ 끝난 뒤 결과 요약·FAIL 이면 게시 안 함.
# 종료 코드 약속: 3 = 배치 점검에서 멈춤(본계산 안 함), 4 = 최적화 전 관문에서 멈춤, 5 = 결과 요약 FAIL(게시 안 함),
#                 6 = 시간차 시작값이 너무 틀려 최적화가 탐색 범위를 벗어남(결과 없음)
KALIBR_GATE_PX=${KALIBR_GATE_PX:-5}                # 최적화 전 재투영 평균 문턱 px(임시값)
KALIBR_CHECK_FRAMES=${KALIBR_CHECK_FRAMES:-20}     # 배치 점검에 쓸 프레임 수
KALIBR_CORNER_LAYOUT=${KALIBR_CORNER_LAYOUT:-auto} # auto = 점검해서 고름, 그 밖 = 그 배치를 모든 카메라에 강제(점검 안 함)
KALIBR_LAYOUT_NAMES="identity rot90 rot180 rot270 flip_lr flip_ud transpose antitranspose"   # lib/kalibr_layout.py 와 같게
ros1_vio="$workdir/vio_ros1.bag"
ros1_cleanup=0      # 1 이면 이 실행이 만든 ROS1 bag(변환본·하드링크) → 성공 뒤 지운다
ros1_mount=()       # 하드링크를 못 만들 때 원본 파일을 Docker 의 /data/vio_ros1.bag 로 바로 연결

# VIO bag 을 실행 폴더의 vio_ros1.bag 로 준비한다(ROS2 면 변환, ROS1 .bag 이면 하드링크 — 복사하지 않는다)
prepare_ros1_vio() {
    if [ "$vio_is_ros1" = "1" ]; then
        if [ "$(realpath "$vio_bag")" = "$(realpath -m "$ros1_vio")" ]; then
            echo "ROS1 bag: 실행 폴더의 vio_ros1.bag 그대로 사용(지우지 않음)"
        elif ln -f "$vio_bag" "$ros1_vio" 2>/dev/null; then
            ros1_cleanup=1
            echo "ROS1 bag: $vio_bag → 하드링크 $ros1_vio (복사 아님, 끝나면 링크만 지움)"
        else
            ros1_mount=(-v "$vio_bag":/data/vio_ros1.bag:ro)
            echo "ROS1 bag: 다른 디스크라 하드링크 불가 → Docker 에 파일째 연결"
        fi
    else
        rm -f "$ros1_vio"
        echo "ROS2 bag → ROS1 변환 중..."
        rosbags-convert --src "$vio_bag" --dst "$ros1_vio"
        ros1_cleanup=1
        echo "변환 완료."
    fi
}

# 실패로 끝날 때도 하드링크·Docker 가 만든 빈 연결 자리는 치운다(변환본은 다시 돌릴 수 있게 남긴다 — 예전과 같음)
cleanup_ros1_links() {
    if [ "$vio_is_ros1" = "1" ] && [ "$ros1_cleanup" = "1" ]; then rm -f "$ros1_vio"; fi
    if [ "${#ros1_mount[@]}" -gt 0 ] && [ -f "$ros1_vio" ] && [ ! -s "$ros1_vio" ]; then rm -f "$ros1_vio" 2>/dev/null || true; fi
}

# 보드 테두리: 표준 AprilTag36h11(1칸) → 1, Kalibr 로 만든 보드(2칸) → 2. KALIBR_TAG_BORDER 가 있으면 묻지 않는다.
choose_tag_border() {
    case "${KALIBR_TAG_BORDER:-}" in
        1|2)
            _tag_border=$KALIBR_TAG_BORDER
            echo "보드 테두리: KALIBR_TAG_BORDER=$_tag_border (질문 생략)"
            ;;
        "")
            read -r -p "타깃이 표준 AprilTag36h11 (1-cell black border) 입니까? [Y/n]: " _tag_reply || _tag_reply=""
            _tag_reply="${_tag_reply:-Y}"
            if [[ "$_tag_reply" =~ ^[Yy]$ ]]; then
                _tag_border=1
                echo "  → blackTagBorder=1 패치 적용"
            else
                _tag_border=2
                echo "  → blackTagBorder Kalibr 기본값 사용 (2-cell)"
            fi
            ;;
        *)
            echo "Error: KALIBR_TAG_BORDER 는 1(표준 AprilTag) 또는 2(Kalibr 보드)만 됩니다: '$KALIBR_TAG_BORDER'" >&2
            exit 1
            ;;
    esac
}

# 시간차 시작값: auto = Kalibr 자체 추정(덮어쓰지 않음), 숫자 = 그 ms 로 덮어씀
choose_timeshift() {
    if [ "$timeshift_arg" = "auto" ]; then
        _ts_patch=auto
        echo "시간차 시작값: Kalibr 자체 추정(카메라·자이로 각속도 크기 교차상관). 숫자로 직접 줄 수도 있음(7번째 인자, ms)"
        echo "  ※ 롤링 셔터 카메라라면 auto 대신 숫자를 주세요(stamp→첫 줄 + 읽기 시간/2, .224 는 35). 글로벌 셔터는 auto 그대로 됩니다."
    elif [[ "$timeshift_arg" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
        _ts_patch=$timeshift_arg
        echo "시간차 시작값: ${timeshift_arg} ms (직접 지정)"
    else
        echo "Error: TIMESHIFT_MS 는 auto 또는 ms 숫자여야 합니다: '$timeshift_arg'" >&2
        exit 1
    fi
}

# 모서리 배치 점검(Docker, 1코어 낮은 우선순위 — 단일 스레드 계산이라 느려지지 않는다). 결과로 layout_args 를 채운다.
#   인자: $1 = 강제 무시하고 점검만 할지(1)
layout_args=()
run_layout_check() {
    local force_check=${1:-0} rc name ok topic lay
    layout_args=()
    if [ "$KALIBR_CORNER_LAYOUT" != "auto" ] && [ "$force_check" != "1" ]; then
        ok=0
        for name in $KALIBR_LAYOUT_NAMES; do [ "$name" = "$KALIBR_CORNER_LAYOUT" ] && ok=1; done
        if [ "$ok" != "1" ]; then
            echo "Error: KALIBR_CORNER_LAYOUT 는 auto 또는 다음 중 하나: $KALIBR_LAYOUT_NAMES (지금 '$KALIBR_CORNER_LAYOUT')" >&2
            exit 1
        fi
        echo "모서리 배치: KALIBR_CORNER_LAYOUT=$KALIBR_CORNER_LAYOUT 를 모든 카메라에 강제(점검 생략)"
        layout_args=(--layout "*=$KALIBR_CORNER_LAYOUT")
        return 0
    fi
    echo "모서리 배치 자동 점검(프레임 ${KALIBR_CHECK_FRAMES}장, 배치 8가지)..."
    rm -f "$output_dir/kalibr_layout_choice.txt" "$output_dir/kalibr_layout_check.txt" "$output_dir/kalibr_layout_check.json"
    set +e
    "$DOCKER" run --rm --cpus=1 -v "$workdir":/data -v "$script_dir/lib":/viso_lib:ro "${ros1_mount[@]}" -w /data \
        -e LANG=C.UTF-8 -e MPLBACKEND=Agg -e OMP_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
        --entrypoint bash kalibr:ros1 \
        -c "source /catkin_ws/devel/setup.bash && nice -n 19 python3 /viso_lib/kalibr_layout_check.py \
              --bag /data/vio_ros1.bag --cam /data/$camchain_file --target /data/$april \
              --border $_tag_border --frames $KALIBR_CHECK_FRAMES \
              --out-json /data/output/kalibr_layout_check.json --out-table /data/output/kalibr_layout_check.txt \
              --out-choice /data/output/kalibr_layout_choice.txt" 2>&1 | tee "$output_dir/kalibr_layout_check.log"
    rc=${PIPESTATUS[0]}
    set -e
    if [ "$rc" = "3" ]; then
        echo "" >&2
        echo "멈춤: 모서리 배치를 고르지 못했습니다(8가지 모두 맞지 않거나 1·2등 차이가 작음) — Kalibr 본계산을 시작하지 않습니다." >&2
        echo "  확인할 것: 보드 설정 $april 의 tagSize(검은 사각형 한 변, m)·tagSpacing(간격÷태그 크기)·tagRows/tagCols(번호 배치)," >&2
        echo "             테두리 답(KALIBR_TAG_BORDER=$_tag_border: 표준 AprilTag 1칸 / Kalibr 보드 2칸)," >&2
        echo "             카메라 값 $camchain_file(내부값·왜곡·토픽·해상도가 이 녹화 카메라와 같은지)." >&2
        echo "  표: $output_dir/kalibr_layout_check.txt" >&2
        cleanup_ros1_links
        exit 3
    elif [ "$rc" != "0" ] || [ ! -s "$output_dir/kalibr_layout_choice.txt" ]; then
        echo "Error: 모서리 배치 점검 실행 실패(종료 코드 $rc). 로그: $output_dir/kalibr_layout_check.log" >&2
        cleanup_ros1_links
        exit 1
    fi
    while read -r topic lay; do
        [ -n "$topic" ] && layout_args+=(--layout "$topic=$lay")
    done < "$output_dir/kalibr_layout_choice.txt"
    echo "채택한 모서리 배치: ${layout_args[*]}"
}

# Kalibr imu-cam 본계산 한 번. 인자: $1 = 보드 yaml(실행 폴더 안 이름), $2 = 로그 파일. 결과: kalibr_rc
#   --dont-show-report + MPLBACKEND=Agg: 화면(X) 없는 Docker 에서 보고서 창을 못 띄워 exit 1 나던 것을 없앤다(결과 파일은 원래 다 써졌음).
run_kalibr_imucam() {
    local target=$1 log=$2 patch_cmd a
    patch_cmd="python3 /viso_lib/kalibr_patch.py --border $_tag_border --timeshift-ms $_ts_patch --gate-px $KALIBR_GATE_PX"
    for a in "${layout_args[@]}"; do patch_cmd+=" $(printf '%q' "$a")"; done
    set +e
    "$DOCKER" run --rm -v "$workdir":/data -v "$script_dir/lib":/viso_lib:ro "${ros1_mount[@]}" -w /data \
        -e LANG=C.UTF-8 -e MPLBACKEND=Agg -e PYTHONDONTWRITEBYTECODE=1 -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
        --entrypoint bash kalibr:ros1 \
        -c "source /catkin_ws/devel/setup.bash && $patch_cmd && \
            /catkin_ws/devel/lib/kalibr/kalibr_calibrate_imu_camera --bag /data/vio_ros1.bag --target /data/$target \
              --cam /data/$camchain_file --imu /data/imu.yaml --timeoffset-padding $timeoffset_padding --dont-show-report" \
        2>&1 | tee "$log"
    kalibr_rc=${PIPESTATUS[0]}
    set -e
}

# 최적화 첫 걸음에서 시간차가 탐색 범위(시작값 ±padding)를 벗어나면 Kalibr 가 "Spline Coefficient Buffer Exceeded" 로 죽는다.
# 2026-11-12 .224(롤링셔터): Kalibr 자체 추정 −18.5 ms, 바른 값 약 +35 ms → 53 ms 차이로 ±50 ms 밖 → 이렇게 멈췄다.
timeshift_stop_message() {
    local priors
    priors=$(grep -a -A1 "Time shift camera to imu" "$1" | grep -a -E '^-?[0-9.]+(e-?[0-9]+)?$' \
             | awk '{printf "%.1f ms ", $1*1000}')
    echo "" >&2
    echo "멈춤: 시간차 시작값(${priors:-?})이 실제와 ${timeoffset_padding:-0.05} s 넘게 달라 Kalibr 최적화가 탐색 범위를 벗어났습니다" >&2
    echo "      (Spline Coefficient Buffer Exceeded). 결과 파일은 없습니다." >&2
    echo "  롤링셔터 카메라는 Kalibr 자체 추정(auto)이 크게 빗나갈 수 있습니다(.224 실측: 추정 −18.5 ms, 바른 값 약 +35 ms)." >&2
    echo "  7번째 인자로 시작값(ms)을 직접 주세요: 대략 노출 지연 + 읽기시간/2 (.224: 3.2 + 64.4/2 ≈ 35)." >&2
}

gate_stop_message() {
    echo "" >&2
    echo "멈춤: 최적화 전 재투영 평균이 ${KALIBR_GATE_PX} px 를 넘었습니다(위 [VISO GATE] 줄) — 최적화해도 고쳐지지 않는 출발점입니다." >&2
    echo "  확인할 것: 모서리 배치(KALIBR_CORNER_LAYOUT 을 강제했다면 auto 로), 보드 설정(tagSize·tagSpacing·번호 배치·테두리)," >&2
    echo "             camchain(내부값·토픽·좌우 기준선 T_cn_cnm1), 시간차 시작값(7번째 인자), imu.yaml 의 rostopic." >&2
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

    # VIO bag → 실행 폴더의 vio_ros1.bag (ROS2 면 변환, ROS1 .bag 이면 하드링크)
    prepare_ros1_vio
    trap cleanup_ros1_links EXIT
    rm -f "$workdir/kalibr_quality.txt"   # 지난 실행의 요약을 이번 것으로 착각하지 않게

    # ── 입력 정하기: 보드 테두리 · 모서리 배치 · 시간차 시작값 ─────────────────────
    choose_tag_border
    if [ "${KALIBR_CHECK_ONLY:-0}" = "1" ]; then
        run_layout_check 1        # 배치 점검만 하고 끝(멈추면 여기서 exit 3)
        echo "배치 점검만 실행(KALIBR_CHECK_ONLY=1) — 표: $output_dir/kalibr_layout_check.txt"
        exit 0
    fi
    run_layout_check
    choose_timeshift

    # kalibr IMU-camera 캘리브레이션
    echo "IMU-camera 캘리브레이션 실행 중..."
    timeoffset_padding=$(compute_timeoffset_padding "$vio_bag" | awk '{print $1}')
    echo "timeoffset-padding: ${timeoffset_padding}s"
    kalibr_log="$output_dir/kalibr_imucam.log"
    results_txt="$workdir/vio_ros1-results-imucam.txt"
    run_kalibr_imucam "$april" "$kalibr_log"

    # ── 성공 판단: 종료 코드가 아니라 "이번 실행의 결과 파일 + 결과 요약"으로 ─────────
    results_new=0
    [ -f "$results_txt" ] && [ "$results_txt" -nt "$run_marker" ] && results_new=1
    if [ "$kalibr_rc" = "4" ]; then
        gate_stop_message
        echo "  로그: $kalibr_log" >&2
        exit 4
    elif [ "$kalibr_rc" != "0" ]; then
        if [ "$results_new" = "1" ]; then
            echo "Warning: Kalibr 종료 코드 $kalibr_rc 이지만 이번 실행의 결과 파일은 써졌습니다(보고서 단계 문제 등) — 결과 요약으로 판단합니다." >&2
        elif grep -a -q "Spline Coefficient Buffer Exceeded" "$kalibr_log"; then
            timeshift_stop_message "$kalibr_log"
            exit 6
        else
            echo "Error: Kalibr 실패(종료 코드 $kalibr_rc), 이번 실행의 결과 파일 없음. 로그: $kalibr_log" >&2
            exit 1
        fi
    elif [ "$results_new" != "1" ]; then
        echo "Error: Kalibr 는 정상 종료했지만 결과 파일 $results_txt 가 이번 실행에서 써지지 않았습니다. 로그: $kalibr_log" >&2
        exit 1
    fi

    # 결과 요약(PASS/WARN/FAIL) — 기준값: KALIBR_REF_CAMCHAIN, 없으면 입력 camchain 에 T_cam_imu 가 있을 때 그것
    ref_args=()
    if [ -n "${KALIBR_REF_CAMCHAIN:-}" ]; then
        ref_args=(--ref "$KALIBR_REF_CAMCHAIN")
    elif grep -q "T_cam_imu" "$workdir/$camchain_file"; then
        ref_args=(--ref "$workdir/$camchain_file")
    fi
    echo ""
    set +e
    python3 "$script_dir/lib/kalibr_quality.py" "$results_txt" "${ref_args[@]}" --out "$workdir/kalibr_quality.txt"
    quality_rc=$?
    set -e
    if [ "$quality_rc" != "0" ] && [ "$quality_rc" != "1" ]; then
        echo "" >&2
        echo "결과 요약 FAIL(또는 결과 파일을 못 읽음, 코드 $quality_rc) — 장비 보정 자리($calib_dir)로 게시하지 않습니다." >&2
        echo "  요약: $workdir/kalibr_quality.txt, 로그: $kalibr_log" >&2
        exit 5
    fi

    echo ""
    echo "Kalibr VIO calibration 완료! (결과 요약: $( [ "$quality_rc" = "0" ] && echo PASS || echo WARN ))"
    echo "결과 파일: <bag>-camchain-imucam.yaml (T_cam_imu 포함), <bag>-results-imucam.txt, kalibr_quality.txt"
    publish_results imu.yaml "$camchain_file" kalibr_quality.txt --imucam
    # 이 실행이 만든 ROS1 bag(변환본·하드링크)만 지운다 — 사용자가 준 원본은 그대로
    if [ "$ros1_cleanup" = "1" ]; then rm -f "$ros1_vio"; fi
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

    # VIO bag → 실행 폴더의 vio_ros1.bag (ROS2 면 변환, ROS1 .bag 이면 하드링크)
    prepare_ros1_vio
    trap cleanup_ros1_links EXIT

    # 보드 테두리 · 모서리 배치 · 시간차 시작값 — kalibr 모드와 같다.
    # 배치 점검은 한 번만: tagSize 만 바꾸면 보드 전체가 같은 비율로 커질 뿐이라 PnP 재투영은 그대로다.
    choose_tag_border
    run_layout_check
    choose_timeshift

    # timeoffset-padding
    timeoffset_padding=$(compute_timeoffset_padding "$vio_bag" | awk '{print $1}')

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

        # 패치(테두리·배치·시간차·관문)는 kalibr 모드와 같은 lib/kalibr_patch.py
        run_kalibr_imucam "april_sweep_${MM}mm.yaml" "$OUTPUT_DIR/kalibr.log"

        # reprojection error 추출 (trailing comma 제거)
        CAM0=$(grep "Reprojection error (cam0) \[px\]" "$OUTPUT_DIR/kalibr.log" 2>/dev/null | awk '{print $6}' | tail -1 | tr -d ',')
        CAM1=$(grep "Reprojection error (cam1) \[px\]" "$OUTPUT_DIR/kalibr.log" 2>/dev/null | awk '{print $6}' | tail -1 | tr -d ',')

        if [ "$kalibr_rc" = "4" ]; then
            # 최적화 전 관문에서 멈춤 — 로그의 재투영은 최적화 전 값이라 최솟값 비교에서 뺀다
            CAM0="관문(>${KALIBR_GATE_PX}px)"; CAM1="관문(>${KALIBR_GATE_PX}px)"
        elif grep -a -q "Spline Coefficient Buffer Exceeded" "$OUTPUT_DIR/kalibr.log"; then
            CAM0="시간차범위초과"; CAM1="시간차범위초과"; kalibr_rc=6
        fi
        printf "%-12s | %-16s | %-16s\n" "${MM}mm" "${CAM0:-N/A}" "${CAM1:-N/A}" >> "$RESULTS_FILE"
        echo ">>> tagSize ${MM}mm: cam0=${CAM0:-N/A}px, cam1=${CAM1:-N/A}px"

        # 최솟값 추적 (cam0 기준)
        if [ -n "$CAM0" ] && [ "$CAM0" != "N/A" ] && [ "$kalibr_rc" != "4" ] && [ "$kalibr_rc" != "6" ]; then
            IS_BETTER=$(python3 -c "print('yes' if float('${CAM0}') < float('${BEST_ERR}') else 'no')" 2>/dev/null || echo "no")
            if [ "$IS_BETTER" = "yes" ]; then
                BEST_ERR="$CAM0"
                BEST_MM="$MM"
            fi
        fi

        rm -f "$APRIL_YAML"
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

    if [ "$ros1_cleanup" = "1" ]; then rm -f "$ros1_vio"; fi
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
