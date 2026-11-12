#!/usr/bin/env bash
# pipeline.sh kalibr 모드 흐름 시험 — 진짜 Docker·Kalibr 없이, 가짜 docker(DOCKER=)로 종료 코드·게시 여부만 본다.
#   배치 점검 성공/실패, 최적화 전 관문(4), 결과 요약 PASS/FAIL(5), 그림 창 exit 1 이어도 결과가 있으면 성공,
#   테두리 질문 환경변수, 시간차 인자, ROS1 bag 하드링크 정리.
set -u
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
pipe="$here/../pipeline.sh"
data="$here/data/kalibr"
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
fail=0
check() { if [ "$2" = "$3" ]; then echo "ok   $1"; else echo "FAIL $1: got '$2' want '$3'"; fail=1; fi; }
has() { if grep -q -- "$2" "$3" 2>/dev/null; then echo "ok   $1"; else echo "FAIL $1: '$2' 없음 ($3)"; fail=1; fi; }
hasnt() { if grep -q -- "$2" "$3" 2>/dev/null; then echo "FAIL $1: '$2' 있음 ($3)"; fail=1; else echo "ok   $1"; fi; }

# 가짜 docker: -v 호스트:/data 를 찾아 결과 파일을 흉내 낸다. 동작은 FAKE_CHECK·FAKE_KALIBR 로 고른다.
cat > "$tmp/docker" <<'FAKE'
#!/usr/bin/env bash
[ "$1" = "image" ] && exit 0
host=""; prev=""; cmd=""
for a in "$@"; do
  if [ "$prev" = "-v" ] && [[ "$a" == *:/data ]]; then host=${a%:/data}; fi
  prev=$a; cmd=$a
done
echo "$cmd" >> "$FAKE_LOG"
if [[ "$cmd" == *kalibr_layout_check.py* ]]; then
  echo "점검표(가짜)" > "$host/output/kalibr_layout_check.txt"
  if [ "${FAKE_CHECK:-ok}" = "ok" ]; then
    printf '/edie/sensors/camera/left/image_gray rot180\n/edie/sensors/camera/right/image_gray rot180\n' > "$host/output/kalibr_layout_choice.txt"
    exit 0
  fi
  : > "$host/output/kalibr_layout_choice.txt"; exit 3
fi
if [[ "$cmd" == *kalibr_calibrate_imu_camera* ]]; then
  case "${FAKE_KALIBR:-pass}" in
    pass|pass_rc1)
      cp "$FAKE_DATA/results_224_fix180_ok.txt" "$host/vio_ros1-results-imucam.txt"
      cp "$FAKE_DATA/camchain_imucam_224_fix180.yaml" "$host/vio_ros1-camchain-imucam.yaml"
      [ "$FAKE_KALIBR" = "pass_rc1" ] && { echo "Unable to access the X Display"; exit 1; }
      exit 0 ;;
    diverged)
      cp "$FAKE_DATA/results_224_raw_diverged.txt" "$host/vio_ros1-results-imucam.txt"
      cp "$FAKE_DATA/camchain_imucam_224_fix180.yaml" "$host/vio_ros1-camchain-imucam.yaml"
      exit 0 ;;
    gate) echo "[VISO GATE] cam0 최적화 전 재투영 평균 49.50 px (문턱 5.0 px)"; exit 4 ;;
    crash) exit 1 ;;
    buffer)
      echo "  Time shift camera to imu (t_imu = t_cam + shift):"; echo "-0.018532924621361788"
      echo "Exception in thread block: [aslam::Exception] Spline Coefficient Buffer Exceeded. Set larger buffer margins!"
      echo "RuntimeError: Optimization failed!"; exit 1 ;;
  esac
fi
exit 0
FAKE
chmod +x "$tmp/docker"

# 실행 폴더 하나를 만들고 pipeline 을 돌린다. 인자: 이름, 추가 pipeline 인자(시간차). 환경변수는 부르는 쪽에서.
run() {
  local name=$1; shift
  local w="$tmp/$name"
  mkdir -p "$w"
  cp "$here/../example_aprilgrid.yaml" "$w/aprilgrid.yaml"
  printf 'rostopic: /edie/sensor/offset_imu\nupdate_rate: 382.0\n' > "$w/imu.yaml"
  printf 'cam0:\n  rostopic: /edie/sensors/camera/left/image_gray\ncam1:\n  rostopic: /edie/sensors/camera/right/image_gray\n' > "$w/camchain.yaml"
  echo "가짜 ROS1 bag" > "$tmp/src_$name.bag"
  : > "$tmp/log_$name"
  (cd "$w" && DOCKER="$tmp/docker" FAKE_LOG="$tmp/log_$name" FAKE_DATA="$data" EDIE_CALIB_DIR="$w/publish" \
      bash "$pipe" - - "$tmp/src_$name.bag" aprilgrid.yaml kalibr /edie/sensor/offset_imu "$@" > "$w/out.txt" 2>&1)
  echo $?
}

# 1) 정상: 배치 점검 rot180 → 본계산 → 요약 PASS → 게시, 하드링크 정리, 원본 bag 유지
rc=$(KALIBR_TAG_BORDER=1 run pass)
check "정상 → 종료 0" "$rc" 0
has "본계산에 채택 배치(cam0) 전달" "image_gray=rot180" "$tmp/log_pass"
has "보고서 창 끄기(--dont-show-report)" "--dont-show-report" "$tmp/log_pass"
has "시간차 auto = Kalibr 추정(--timeshift-ms auto)" "--timeshift-ms auto" "$tmp/log_pass"
has "테두리 1" "--border 1" "$tmp/log_pass"
has "관문 5 px" "--gate-px 5" "$tmp/log_pass"
has "요약 PASS" "종합: PASS" "$tmp/pass/kalibr_quality.txt"
check "결과 게시됨" "$([ -f "$tmp/pass/publish/vio_ros1-results-imucam.txt" ] && echo y)" y
check "요약도 게시됨" "$([ -f "$tmp/pass/publish/kalibr_quality.txt" ] && echo y)" y
check "하드링크 지움" "$([ -e "$tmp/pass/vio_ros1.bag" ] && echo 남음 || echo 지움)" 지움
check "원본 bag 유지" "$([ -f "$tmp/src_pass.bag" ] && echo y)" y

# 2) 그림 창 단계에서 exit 1 이어도 결과 파일이 이번 실행에서 써졌으면 요약으로 판단 → 0
rc=$(KALIBR_TAG_BORDER=1 FAKE_KALIBR=pass_rc1 run rc1)
check "Kalibr exit 1 + 결과 있음 → 종료 0" "$rc" 0
has "경고 문구" "결과 요약으로 판단" "$tmp/rc1/out.txt"

# 3) 발산 결과(49 px) → 요약 FAIL → 게시 안 함, 종료 5
rc=$(KALIBR_TAG_BORDER=1 FAKE_KALIBR=diverged run div)
check "발산 → 종료 5" "$rc" 5
check "발산 → 게시 안 함" "$([ -e "$tmp/div/publish/vio_ros1-results-imucam.txt" ] && echo 게시됨 || echo 안함)" 안함
hasnt "발산 → '완료!' 안 찍음" "calibration 완료" "$tmp/div/out.txt"

# 4) 최적화 전 관문 → 종료 4, 게시 안 함
rc=$(KALIBR_TAG_BORDER=1 FAKE_KALIBR=gate run gate)
check "관문 → 종료 4" "$rc" 4
has "관문 안내" "최적화 전 재투영 평균이 5 px" "$tmp/gate/out.txt"
check "관문 → 게시 안 함" "$([ -d "$tmp/gate/publish" ] && echo 게시됨 || echo 안함)" 안함

# 5) 배치 점검 실패 → 종료 3, 본계산 안 함
rc=$(KALIBR_TAG_BORDER=1 FAKE_CHECK=fail run nolayout)
check "배치 못 고름 → 종료 3" "$rc" 3
hasnt "배치 못 고름 → 본계산 안 함" "kalibr_calibrate_imu_camera" "$tmp/log_nolayout"
has "보드·camchain 확인 안내" "tagSize" "$tmp/nolayout/out.txt"
check "배치 못 고름 → 하드링크 정리" "$([ -e "$tmp/nolayout/vio_ros1.bag" ] && echo 남음 || echo 지움)" 지움

# 6) 배치 강제(identity = 점검 끔) → 점검 없이 그 배치로
rc=$(KALIBR_TAG_BORDER=1 KALIBR_CORNER_LAYOUT=identity run forced)
check "배치 강제 → 종료 0" "$rc" 0
hasnt "배치 강제 → 점검 안 함" "kalibr_layout_check.py" "$tmp/log_forced"
has "배치 강제 → 모든 카메라 identity" "=identity" "$tmp/log_forced"
rc=$(KALIBR_TAG_BORDER=1 KALIBR_CORNER_LAYOUT=rot45 run badlayout)
check "없는 배치 이름 → 종료 1" "$rc" 1

# 7) 시간차 숫자 지정
rc=$(KALIBR_TAG_BORDER=1 run ts35 35)
check "시간차 35 → 종료 0" "$rc" 0
has "시간차 35 전달" "--timeshift-ms 35" "$tmp/log_ts35"
rc=$(KALIBR_TAG_BORDER=1 run tsbad abc)
check "시간차 숫자 아님 → 종료 1" "$rc" 1

# 8) 테두리: 환경변수 없으면 질문(표준 입력 n → 2), 잘못된 값은 멈춤, 입력 없음(EOF)이면 기본 Y
rc=$(echo n | run ask)
check "질문에 n → 종료 0" "$rc" 0
has "질문에 n → --border 2" "--border 2" "$tmp/log_ask"
rc=$(run eof < /dev/null)
has "입력 없음 → 기본 Y(--border 1)" "--border 1" "$tmp/log_eof"
rc=$(KALIBR_TAG_BORDER=3 run badborder)
check "KALIBR_TAG_BORDER=3 → 종료 1" "$rc" 1

# 9) 배치 점검만
rc=$(KALIBR_TAG_BORDER=1 KALIBR_CHECK_ONLY=1 run checkonly)
check "점검만 → 종료 0" "$rc" 0
hasnt "점검만 → 본계산 안 함" "kalibr_calibrate_imu_camera" "$tmp/log_checkonly"

# 10) Kalibr 가 결과 없이 죽음 → 종료 1
rc=$(KALIBR_TAG_BORDER=1 FAKE_KALIBR=crash run crash)
check "Kalibr 실패·결과 없음 → 종료 1" "$rc" 1

# 10b) 시간차 시작값이 너무 틀려 탐색 범위 초과(.224 auto 실측) → 종료 6, 숫자로 주라는 안내
rc=$(KALIBR_TAG_BORDER=1 FAKE_KALIBR=buffer run buffer)
check "시간차 범위 초과 → 종료 6" "$rc" 6
has "시간차 안내(추정값 표시)" "-18.5 ms" "$tmp/buffer/out.txt"
has "숫자로 주라는 안내" "7번째 인자" "$tmp/buffer/out.txt"

# 11) 기준값 비교: KALIBR_REF_CAMCHAIN 을 주면 요약에 기준 대비 줄
rc=$(KALIBR_TAG_BORDER=1 KALIBR_REF_CAMCHAIN="$data/camchain_imucam_224_fix180.yaml" run ref)
check "기준값 → 종료 0" "$rc" 0
has "기준 대비 회전 줄" "기준 대비 회전 차" "$tmp/ref/kalibr_quality.txt"

exit $fail
