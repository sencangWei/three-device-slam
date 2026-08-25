#!/bin/bash
set -euo pipefail

fail() { echo "$1" >&2; exit 2; }

require_tools() {
  local tool
  for tool in "$@"; do
    command -v "$tool" >/dev/null 2>&1 || fail "FAIL/tool_missing:${tool}"
  done
}

require_verifier_tools() {
  require_tools sha256sum awk file grep nm stat readlink find sort id
}

require_trusted_path() {
  local target=$1 resolved current component owner group mode
  local -a components
  [[ ${target} == /* && -e ${target} && ! -L ${target} ]] || fail "FAIL/untrusted_installation"
  resolved=$(readlink -f -- "$target") || fail "FAIL/untrusted_installation"
  [[ ${resolved} == "${target}" ]] || fail "FAIL/untrusted_installation"
  current=/
  IFS='/' read -r -a components <<<"${target#/}"
  for component in "" "${components[@]}"; do
    [[ -z ${component} ]] || current="${current%/}/${component}"
    [[ ! -L ${current} ]] || fail "FAIL/untrusted_installation"
    owner=$(stat -c '%u' -- "$current") || fail "FAIL/untrusted_installation"
    group=$(stat -c '%g' -- "$current") || fail "FAIL/untrusted_installation"
    mode=$(stat -c '%a' -- "$current") || fail "FAIL/untrusted_installation"
    [[ ${owner} == 0 && ${group} == 0 ]] || fail "FAIL/untrusted_installation"
    (( (8#${mode} & 8#022) == 0 )) || fail "FAIL/untrusted_installation"
  done
}

require_trusted_file() {
  [[ -f $1 && ! -L $1 ]] || fail "FAIL/untrusted_installation"
  require_trusted_path "$1"
}

_trusted_tree_entry() {
  local entry=$1 root_device=$2 owner group mode entry_device
  [[ ! -L ${entry} && ( -d ${entry} || -f ${entry} ) ]] || return 1
  entry_device=$(stat -c '%d' -- "$entry") || return 1
  [[ ${entry_device} == "${root_device}" ]] || return 1
  owner=$(stat -c '%u' -- "$entry") || return 1
  group=$(stat -c '%g' -- "$entry") || return 1
  [[ ${owner} == 0 && ${group} == 0 ]] || return 1
  mode=$(stat -c '%a' -- "$entry") || return 1
  (( (8#${mode} & 8#022) == 0 ))
}

require_trusted_tree() {
  local root=$1 root_device entry find_pid find_fd control_fd invalid=false
  require_trusted_path "$root"
  root_device=$(stat -c '%d' -- "$root") || fail "FAIL/untrusted_installation"
  coproc TRUSTED_TREE_FIND {
    IFS= read -r _
    find "$root" -xdev -print0 2>/dev/null
  }
  find_pid=$TRUSTED_TREE_FIND_PID
  exec {find_fd}<&"${TRUSTED_TREE_FIND[0]}"
  control_fd=${TRUSTED_TREE_FIND[1]}
  printf 'start\n' >&"$control_fd"
  exec {control_fd}>&-
  while IFS= read -r -d '' -u "$find_fd" entry; do
    if ! _trusted_tree_entry "$entry" "$root_device"; then
      invalid=true
      break
    fi
  done
  exec {find_fd}<&-
  if [[ ${invalid} == true ]]; then
    kill "$find_pid" 2>/dev/null || true
    wait "$find_pid" 2>/dev/null || true
    fail "FAIL/untrusted_installation"
  fi
  wait "$find_pid" || fail "FAIL/untrusted_installation"
}

validate_bridge() {
  local bridge=$1 symbol
  file "$bridge" | grep -Eq 'ELF 64-bit.*x86-64' || fail "FAIL/xu_bridge_architecture"
  for symbol in ylx_open ylx_read_imu27 ylx_close; do
    nm -D --defined-only "$bridge" | grep -Eq "[[:space:]]${symbol}$" \
      || fail "FAIL/xu_bridge_symbol_missing"
  done
}

validate_bridge_manifest() {
  local bridge=$1 manifest=$2 manifest_bridge=$3 actual_hash
  local -a lines
  [[ -f ${manifest} && ! -L ${manifest} ]] || fail "FAIL/xu_bridge_existing_invalid"
  mapfile -t lines <"$manifest" || fail "FAIL/xu_bridge_existing_invalid"
  actual_hash=$(sha256sum "$bridge" | awk '{print $1}')
  [[ ${#lines[@]} -eq 1 && ${lines[0]} == "${actual_hash}  ${manifest_bridge}" ]] \
    || fail "FAIL/xu_bridge_hash"
}

main() {
  local install_root python lib bridge manifest members robot_uid robot_gid session_root
  export PATH=/usr/sbin:/usr/bin:/sbin:/bin
  [[ -r /etc/os-release ]] || fail "FAIL/unsupported_ubuntu"
  # shellcheck source=/etc/os-release
  source /etc/os-release
  [[ ${ID} == ubuntu && ${VERSION_ID} == 22.04 ]] || fail "FAIL/unsupported_ubuntu"
  require_verifier_tools
  [[ -r /opt/ros/humble/setup.bash ]] || fail "FAIL/ros_humble_missing"
  # shellcheck source=/opt/ros/humble/setup.bash
  source /opt/ros/humble/setup.bash
  [[ ${ROS_DISTRO:-} == humble ]] || fail "FAIL/ros_humble_missing"
  export PATH=/usr/sbin:/usr/bin:/sbin:/bin

  require_trusted_tree /etc/three-device-slam

  install_root=/opt/three-device-slam
  python="${install_root}/venv/bin/python"
  lib="${install_root}/lib"
  bridge="${lib}/libtwo_uq2_xu.so"
  manifest="${bridge}.sha256"
  require_trusted_tree "$install_root"
  require_trusted_path "${install_root}/venv"
  require_trusted_file "$python"
  [[ -x ${python} ]] || fail "FAIL/untrusted_installation"
  require_trusted_path "$lib"
  require_trusted_file "$bridge"
  require_trusted_file "$manifest"
  members=$(find "$lib" -mindepth 1 -maxdepth 1 -printf '%f\n' | sort)
  [[ ${members} == $'libtwo_uq2_xu.so\nlibtwo_uq2_xu.so.sha256' ]] \
    || fail "FAIL/xu_bridge_existing_invalid"

  "$python" -I -B -c 'import three_device_slam' || fail "FAIL/python_package_missing"
  validate_bridge_manifest "$bridge" "$manifest" "$bridge"
  validate_bridge "$bridge"

  session_root=/var/lib/three-device-slam/sessions
  require_trusted_path /var/lib/three-device-slam
  [[ -d ${session_root} && ! -L ${session_root} ]] || fail "FAIL/session_root_missing"
  [[ $(readlink -f -- "$session_root") == "${session_root}" ]] || fail "FAIL/untrusted_installation"
  robot_uid=$(id -u robot) || fail "FAIL/robot_account_missing"
  robot_gid=$(id -g robot) || fail "FAIL/robot_account_missing"
  [[ $(stat -c '%u' -- "$session_root") == "${robot_uid}" ]] || fail "FAIL/untrusted_installation"
  [[ $(stat -c '%g' -- "$session_root") == "${robot_gid}" ]] || fail "FAIL/untrusted_installation"
  [[ $(stat -c '%a' -- "$session_root") == 750 ]] || fail "FAIL/untrusted_installation"
  echo "PASS/ubuntu_environment"
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then main "$@"; fi
