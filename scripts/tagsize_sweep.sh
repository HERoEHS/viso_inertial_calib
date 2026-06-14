#!/bin/bash
# tagSize sweep 테스트 — reprojection error 최솟값 탐색
# 사용법: bash scripts/tagsize_sweep.sh [bag_path]
# 예시:   bash scripts/tagsize_sweep.sh best_so_far/vio_ros1.bag
set -e

WORKDIR=$(cd "$(dirname "$0")/.." && pwd)
BAG=${1:-vio_ros1.bag}
BAG_ABS=$(realpath "$BAG" 2>/dev/null || echo "$WORKDIR/$BAG")
BAG_NAME=$(basename "$BAG_ABS")

if [ ! -f "$BAG_ABS" ]; then
    echo "Error: bag not found: $BAG_ABS" >&2
    exit 1
fi

# bag이 workdir 안에 없으면 심볼릭 링크
if [ "$(dirname "$BAG_ABS")" != "$WORKDIR" ]; then
    ln -sf "$BAG_ABS" "$WORKDIR/$BAG_NAME"
fi

CAMCHAIN="$WORKDIR/camchain.yaml"
IMU="$WORKDIR/imu.yaml"
if [ ! -f "$CAMCHAIN" ] || [ ! -f "$IMU" ]; then
    echo "Error: camchain.yaml 또는 imu.yaml 없음 (kalibr 모드 전제)" >&2
    exit 1
fi

# timeoffset-padding 계산 (pipeline.sh와 동일)
TIMEOFFSET_PADDING=0.02

RESULTS_FILE="$WORKDIR/output/tagsize_sweep_results.txt"
mkdir -p "$WORKDIR/output"
echo "tagSize(mm) | cam0_mean(px) | cam1_mean(px)" > "$RESULTS_FILE"
echo "-------------------------------------------" >> "$RESULTS_FILE"

# 테스트 범위: 21~27mm (1mm 간격)
for MM in 21 22 23 24 25 26 27; do
    TAGSIZE=$(echo "scale=4; $MM / 1000" | bc)
    TAGSPACING=0.3
    APRIL_YAML="$WORKDIR/april_sweep_${MM}mm.yaml"

    cat > "$APRIL_YAML" << YAML
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: ${TAGSIZE}
tagSpacing: ${TAGSPACING}
YAML

    echo ""
    echo "=========================================="
    echo "tagSize = ${MM}mm (${TAGSIZE}m)"
    echo "=========================================="

    OUTPUT_DIR="$WORKDIR/output/sweep_${MM}mm"
    mkdir -p "$OUTPUT_DIR"

    # Kalibr 실행 (결과를 output 디렉토리에 저장)
    docker run --rm \
        -v "$WORKDIR":/data \
        -v "$OUTPUT_DIR":/output \
        kalibr:ros1 bash -c \
        "cd /output && kalibr_calibrate_imu_camera \
            --bag /data/$BAG_NAME \
            --target /data/april_sweep_${MM}mm.yaml \
            --cam /data/camchain.yaml \
            --imu /data/imu.yaml \
            --timeoffset-padding $TIMEOFFSET_PADDING \
            2>&1" | tee "$OUTPUT_DIR/kalibr.log"

    # 결과 추출
    CAM0=$(grep "Reprojection error (cam0) \[px\]" "$OUTPUT_DIR/kalibr.log" | awk '{print $6}' | tail -1)
    CAM1=$(grep "Reprojection error (cam1) \[px\]" "$OUTPUT_DIR/kalibr.log" | awk '{print $6}' | tail -1)

    echo "${MM}mm        | ${CAM0:-N/A}        | ${CAM1:-N/A}" >> "$RESULTS_FILE"
    echo ">>> tagSize ${MM}mm: cam0=${CAM0:-N/A}px, cam1=${CAM1:-N/A}px"

    # 임시 yaml 삭제
    rm -f "$APRIL_YAML"
done

echo ""
echo "=========================================="
echo "sweep 완료. 결과:"
cat "$RESULTS_FILE"
echo "=========================================="
echo "상세 결과: $WORKDIR/output/sweep_*mm/"
