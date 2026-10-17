#!/usr/bin/env bash
# lib/calib_dir.sh 규칙 시험 — 실제 /var/lib/edie 대신 임시 폴더, uid 는 인자로 흉내낸다.
set -u
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=../lib/calib_dir.sh
. "$here/../lib/calib_dir.sh"
tmp=$(mktemp -d); trap 'chmod -R u+w "$tmp" 2>/dev/null; rm -rf "$tmp"' EXIT
fail=0
check() { if [ "$2" = "$3" ]; then echo "ok   $1"; else echo "FAIL $1: got '$2' want '$3'"; fail=1; fi; }

unset EDIE_CALIB_DIR
# 1) root 아님 → 홈
check "root 아님 → \$HOME/.edie/calib" "$(edie_default_calib_dir 1000 "$tmp/sys" "$tmp/home")" "$tmp/home/.edie/calib"
[ -e "$tmp/sys" ] && { echo "FAIL root 아닌데 시스템 폴더를 만들었다"; fail=1; }
# 2) root + 없던 시스템 폴더 → 만들고 씀
check "root → system_root/data/calib (없으면 만듦)" "$(edie_default_calib_dir 0 "$tmp/sys" "$tmp/home")" "$tmp/sys/data/calib"
[ -d "$tmp/sys" ] || { echo "FAIL root 인데 시스템 폴더를 안 만들었다"; fail=1; }
# 3) root + 읽기 전용 시스템 폴더 → 홈으로 물러섬
mkdir -p "$tmp/ro" && chmod 555 "$tmp/ro"
if [ -w "$tmp/ro" ]; then echo "skip 읽기 전용 흉내 불가(이 계정은 어디든 쓸 수 있음)"; else
check "root + 읽기 전용 → 홈으로" "$(edie_default_calib_dir 0 "$tmp/ro" "$tmp/home")" "$tmp/home/.edie/calib"; fi
# 4) root + 못 만드는 자리(읽기 전용 폴더 아래) → 홈으로
if [ -w "$tmp/ro" ]; then echo "skip"; else
check "root + 만들 수 없음 → 홈으로" "$(edie_default_calib_dir 0 "$tmp/ro/edie" "$tmp/home")" "$tmp/home/.edie/calib"; fi
# 4b) root 아닌데 시스템 폴더가 이미 있고 쓸 수 있어도 → 홈 (계정으로만 가른다)
mkdir -p "$tmp/sys_exists" && chmod u+w "$tmp/sys_exists"
check "root 아님 + 시스템 폴더 있음 → 그래도 홈" "$(edie_default_calib_dir 1000 "$tmp/sys_exists" "$tmp/home")" "$tmp/home/.edie/calib"
# 4c) root + 같은 이름이 일반 파일 → 홈
: > "$tmp/a_file"
check "root + 같은 이름이 파일 → 홈으로" "$(edie_default_calib_dir 0 "$tmp/a_file" "$tmp/home")" "$tmp/home/.edie/calib"
# 4d) HOME 도 없고 root 도 아님 → 정할 수 없음(실패 반환)
#     (세 번째 인자를 비우면 HOME 을 쓰므로, HOME 자체를 지운 서브셸에서 돌린다)
if out=$(unset HOME; edie_default_calib_dir 1000 "$tmp/sys" 2>/dev/null); then echo "FAIL HOME 없음인데 성공('$out')"; fail=1; else echo "ok   HOME 없음 → 정할 수 없음(실패 반환)"; fi
# 5) 환경변수가 있으면 그대로(최우선)
EDIE_CALIB_DIR="$tmp/custom" check "EDIE_CALIB_DIR 우선" "$(EDIE_CALIB_DIR="$tmp/custom" edie_resolve_calib_dir 0 "$tmp/sys" "$tmp/home")" "$tmp/custom"
check "환경변수 없으면 기본 규칙" "$(edie_resolve_calib_dir 1000 "$tmp/sys" "$tmp/home")" "$tmp/home/.edie/calib"
exit $fail
