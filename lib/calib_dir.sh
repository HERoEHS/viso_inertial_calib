#!/usr/bin/env bash
# 캘리브 결과(로봇이 "만드는" 파일)를 두는 폴더 규칙 — 데이터 폴더 규칙(SW1-1959, 경로 문서 6절 11번)
#   EDIE_CALIB_DIR 가 있으면 그대로(최우선).
#   없으면: root(edieOS)이고 /var/lib/edie 를 만들어 쓸 수 있으면 /var/lib/edie/data/calib,
#           그 밖(Ubuntu edie 계정, 읽기 전용 등)은 예전 그대로 $HOME/.edie/calib.
#   설치본(/opt/edie/current/share)과는 다른 자리다 — 거기엔 로봇이 "읽는" 설정(캘리브 yaml 등)만 둔다.
# pipeline.sh 가 source 해서 쓰고, test/test_calib_dir.sh 가 인자를 바꿔 가며 시험한다.

# edie_default_calib_dir [uid] [system_root] [home]  — 인자는 시험용(기본: 실제 값)
edie_default_calib_dir() {
  local uid="${1:-$(id -u)}" root="${2:-/var/lib/edie}" home="${3:-${HOME:-}}"
  if [ "$uid" = "0" ] && { [ -d "$root" ] || mkdir -p "$root" 2>/dev/null; } && [ -w "$root" ]; then
    echo "$root/data/calib"
  elif [ -n "$home" ]; then
    echo "$home/.edie/calib"
  else
    # C++ 헬퍼(state_path.hpp)와 같은 판정: /var/lib/edie 도 HOME 도 못 쓰면 "정할 수 없음" — 호출부가 멈춘다
    echo "edie_default_calib_dir: 캘리브 결과 폴더를 정할 수 없다(/var/lib/edie·HOME 모두 없음)" >&2
    return 1
  fi
}

# edie_resolve_calib_dir [uid] [system_root] [home] — 환경변수 EDIE_CALIB_DIR 가 비어 있지 않으면 그 값
edie_resolve_calib_dir() {
  if [ -n "${EDIE_CALIB_DIR:-}" ]; then
    echo "$EDIE_CALIB_DIR"
  else
    edie_default_calib_dir "$@"
  fi
}
