#!/bin/bash
set -euo pipefail

fail() { echo "$1" >&2; exit 2; }

require_tools() {
  local tool
  for tool in "$@"; do
    command -v "$tool" >/dev/null 2>&1 || fail "FAIL/tool_missing:${tool}"
  done
}

require_installer_tools() {
  require_tools python3 dirname install make sha256sum awk file grep nm stat readlink \
    chmod mktemp mv rm sync find sort id cc mkdir flock
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

require_trusted_tree() {
  local root=$1 allowed_link=${2:-} entry owner group mode root_device entry_device link_target
  require_trusted_path "$root"
  root_device=$(stat -c '%d' -- "$root") || fail "FAIL/untrusted_installation"
  while IFS= read -r -d '' entry; do
    if [[ -L ${entry} ]]; then
      [[ -n ${allowed_link} && ${entry} == "${allowed_link}" ]] \
        || fail "FAIL/untrusted_installation"
      link_target=$(readlink -- "$entry") || fail "FAIL/untrusted_installation"
      [[ ${link_target} == lib ]] || fail "FAIL/untrusted_installation"
    else
      [[ -d ${entry} || -f ${entry} ]] || fail "FAIL/untrusted_installation"
    fi
    entry_device=$(stat -c '%d' -- "$entry") || fail "FAIL/untrusted_installation"
    [[ ${entry_device} == "${root_device}" ]] || fail "FAIL/untrusted_installation"
    owner=$(stat -c '%u' -- "$entry") || fail "FAIL/untrusted_installation"
    group=$(stat -c '%g' -- "$entry") || fail "FAIL/untrusted_installation"
    [[ ${owner} == 0 && ${group} == 0 ]] || fail "FAIL/untrusted_installation"
    if [[ -L ${entry} ]]; then
      require_trusted_path "${entry%/lib64}/lib"
      continue
    fi
    mode=$(stat -c '%a' -- "$entry") || fail "FAIL/untrusted_installation"
    (( (8#${mode} & 8#022) == 0 )) || fail "FAIL/untrusted_installation"
  done < <(find "$root" -xdev -print0)
}

ensure_root_directory() {
  local target=$1
  if [[ -e ${target} || -L ${target} ]]; then
    [[ -d ${target} ]] || fail "FAIL/untrusted_installation"
    require_trusted_path "$target"
  else
    require_trusted_path "$(dirname "$target")"
    install -d -o root -g root -m 0755 "$target"
    require_trusted_path "$target"
  fi
}

ensure_session_directory() {
  local target=$1 robot_uid robot_gid
  ensure_root_directory "$(dirname "$target")"
  robot_uid=$(id -u robot) || fail "FAIL/robot_account_missing"
  robot_gid=$(id -g robot) || fail "FAIL/robot_account_missing"
  if [[ -e ${target} || -L ${target} ]]; then
    [[ -d ${target} && ! -L ${target} ]] || fail "FAIL/untrusted_installation"
    [[ $(readlink -f -- "$target") == "${target}" ]] || fail "FAIL/untrusted_installation"
    [[ $(stat -c '%u' -- "$target") == "${robot_uid}" ]] || fail "FAIL/untrusted_installation"
    [[ $(stat -c '%g' -- "$target") == "${robot_gid}" ]] || fail "FAIL/untrusted_installation"
    [[ $(stat -c '%a' -- "$target") == 750 ]] || fail "FAIL/untrusted_installation"
  else
    install -d -o robot -g robot -m 0750 "$target"
  fi
}

validate_bridge() {
  local bridge=$1 symbol
  [[ -f ${bridge} && ! -L ${bridge} ]] || fail "FAIL/xu_bridge_artifact_invalid"
  file "$bridge" | grep -Eq 'ELF 64-bit.*x86-64' || fail "FAIL/xu_bridge_artifact_invalid"
  for symbol in ylx_open ylx_read_imu27 ylx_close; do
    nm -D --defined-only "$bridge" | grep -Eq "[[:space:]]${symbol}$" \
      || fail "FAIL/xu_bridge_artifact_invalid"
  done
  sha256sum "$bridge" | awk 'NF == 2 && $1 ~ /^[0-9a-f]{64}$/ { ok=1 } END { exit !ok }' \
    || fail "FAIL/xu_bridge_artifact_invalid"
}

validate_bridge_manifest() {
  local bridge=$1 manifest=$2 manifest_bridge=$3 actual_hash
  local -a lines
  [[ -f ${manifest} && ! -L ${manifest} ]] || fail "FAIL/xu_bridge_existing_invalid"
  mapfile -t lines <"$manifest" || fail "FAIL/xu_bridge_existing_invalid"
  actual_hash=$(sha256sum "$bridge" | awk '{print $1}')
  [[ ${#lines[@]} -eq 1 && ${lines[0]} == "${actual_hash}  ${manifest_bridge}" ]] \
    || fail "FAIL/xu_bridge_existing_invalid"
}

validate_existing_library() {
  local lib=$1 bridge="${1}/libtwo_uq2_xu.so" manifest members
  manifest="${bridge}.sha256"
  require_trusted_path "$lib"
  require_trusted_file "$bridge"
  require_trusted_file "$manifest"
  members=$(find "$lib" -mindepth 1 -maxdepth 1 -printf '%f\n' | sort)
  [[ ${members} == $'libtwo_uq2_xu.so\nlibtwo_uq2_xu.so.sha256' ]] \
    || fail "FAIL/xu_bridge_existing_invalid"
  validate_bridge_manifest "$bridge" "$manifest" "$bridge"
  validate_bridge "$bridge"
}

validate_python_runtime() {
  local install_root=$1 python=$2
  require_trusted_tree "${install_root}/venv"
  require_trusted_file "$python"
  [[ -x ${python} ]] || fail "FAIL/untrusted_installation"
  "$python" -I -B -m pip --version >/dev/null 2>&1 || fail "FAIL/tool_missing:pip"
}

normalize_venv_layout() {
  local install_root=$1 lib64="${1}/venv/lib64" target
  if [[ -L ${lib64} ]]; then
    target=$(readlink -- "$lib64") || fail "FAIL/untrusted_installation"
    [[ ${target} == lib ]] || fail "FAIL/untrusted_installation"
    rm -- "$lib64"
  fi
}

install_lock_fd=
acquire_install_lock() {
  local lock_path=$1
  if [[ ! -e ${lock_path} && ! -L ${lock_path} ]]; then
    (umask 0077; : >"$lock_path") || fail "FAIL/untrusted_installation"
  fi
  require_trusted_file "$lock_path"
  exec {install_lock_fd}<>"$lock_path" || fail "FAIL/untrusted_installation"
  flock -n "$install_lock_fd" || fail "FAIL/install_in_progress"
}

temporary_lib=
temporary_lib_parent=
prepared_bridge_hash=
prepared_existing_library=false
cleanup_temporary_lib() {
  if [[ -n ${temporary_lib} && -d ${temporary_lib} && ! -L ${temporary_lib} ]]; then
    case ${temporary_lib} in
      "${temporary_lib_parent}"/.lib.*.tmp) rm -rf -- "$temporary_lib" ;;
    esac
  fi
}

prepare_bridge() {
  local artifact=$1 install_root=$2 lib bridge installed_hash manifest
  lib="${install_root}/lib"
  bridge="${lib}/libtwo_uq2_xu.so"
  validate_bridge "$artifact"
  require_trusted_path "$install_root"

  temporary_lib_parent=$install_root
  temporary_lib=$(mktemp -d "${install_root}/.lib.XXXXXXXX.tmp")
  chmod 0755 "$temporary_lib"
  install -o root -g root -m 0755 "$artifact" "${temporary_lib}/libtwo_uq2_xu.so"
  validate_bridge "${temporary_lib}/libtwo_uq2_xu.so"
  prepared_bridge_hash=$(sha256sum "${temporary_lib}/libtwo_uq2_xu.so" | awk '{print $1}')
  manifest="${temporary_lib}/libtwo_uq2_xu.so.sha256"
  printf '%s  %s\n' "$prepared_bridge_hash" "$bridge" >"$manifest"
  chmod 0644 "$manifest"
  require_trusted_path "$temporary_lib"
  require_trusted_file "${temporary_lib}/libtwo_uq2_xu.so"
  require_trusted_file "$manifest"
  validate_bridge_manifest "${temporary_lib}/libtwo_uq2_xu.so" "$manifest" "$bridge"
  sync -f "${temporary_lib}/libtwo_uq2_xu.so"
  sync -f "$manifest"
  sync -f "$temporary_lib"

  prepared_existing_library=false
  if [[ -e ${lib} || -L ${lib} ]]; then
    [[ -d ${lib} && ! -L ${lib} ]] || fail "FAIL/untrusted_installation"
    validate_existing_library "$lib"
    installed_hash=$(sha256sum "$bridge" | awk '{print $1}')
    [[ ${installed_hash} == "${prepared_bridge_hash}" ]] || fail "FAIL/xu_bridge_conflict"
    prepared_existing_library=true
  fi
}

publish_prepared_bridge() {
  local install_root=$1 lib bridge installed_hash
  lib="${install_root}/lib"
  bridge="${lib}/libtwo_uq2_xu.so"
  if [[ ${prepared_existing_library} == true ]]; then
    validate_existing_library "$lib"
    installed_hash=$(sha256sum "$bridge" | awk '{print $1}')
    [[ ${installed_hash} == "${prepared_bridge_hash}" ]] || fail "FAIL/xu_bridge_conflict"
    rm -rf -- "$temporary_lib"
    temporary_lib=
    sync -f "$install_root"
    return
  fi
  mv -T --no-clobber "$temporary_lib" "$lib"
  if [[ -e ${temporary_lib} ]]; then
    if [[ -d ${lib} && ! -L ${lib} ]]; then
      validate_existing_library "$lib"
      installed_hash=$(sha256sum "$bridge" | awk '{print $1}')
      if [[ ${installed_hash} == "${prepared_bridge_hash}" ]]; then
        rm -rf -- "$temporary_lib"
        temporary_lib=
        sync -f "$install_root"
        return
      fi
    fi
    fail "FAIL/xu_bridge_publish_conflict"
  fi
  temporary_lib=
  sync -f "$install_root"
  validate_existing_library "$lib"
}

publish_bridge() {
  prepare_bridge "$1" "$2"
  publish_prepared_bridge "$2"
}

install_runtime() {
  local artifact=$1 install_root=$2 repo_root=$3 python
  prepare_bridge "$artifact" "$install_root"
  python="${install_root}/venv/bin/python"
  if [[ -e ${install_root}/venv || -L ${install_root}/venv ]]; then
    [[ -d ${install_root}/venv && ! -L ${install_root}/venv ]] \
      || fail "FAIL/untrusted_installation"
  else
    python3 -m venv --copies --system-site-packages "${install_root}/venv"
    chmod -R go-w "${install_root}/venv"
  fi
  normalize_venv_layout "$install_root"
  validate_python_runtime "$install_root" "$python"
  "$python" -I -B -m pip install "$repo_root"
  chmod -R go-w "${install_root}/venv"
  require_trusted_tree "$install_root"
  require_trusted_file "$python"
  [[ -x ${python} ]] || fail "FAIL/untrusted_installation"
  publish_prepared_bridge "$install_root"
  require_trusted_tree "$install_root"
}

main() {
  local repo_root vendor_sdk install_root artifact
  export PATH=/usr/sbin:/usr/bin:/sbin:/bin
  umask 0022
  [[ ${EUID} -eq 0 ]] || fail "FAIL/root_required"
  [[ -r /etc/os-release ]] || fail "FAIL/unsupported_ubuntu"
  # shellcheck source=/etc/os-release
  source /etc/os-release
  [[ ${ID} == ubuntu && ${VERSION_ID} == 22.04 ]] || fail "FAIL/unsupported_ubuntu"
  require_installer_tools
  [[ -r /opt/ros/humble/setup.bash ]] || fail "FAIL/ros_humble_missing"
  # shellcheck source=/opt/ros/humble/setup.bash
  source /opt/ros/humble/setup.bash
  [[ ${ROS_DISTRO:-} == humble ]] || fail "FAIL/ros_humble_missing"
  export PATH=/usr/sbin:/usr/bin:/sbin:/bin

  repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
  vendor_sdk=/home/robot/vendor/2uq2/YLX_XU_API_2026721
  [[ -f ${vendor_sdk}/include/V4L2/extunit.h ]] \
    || { echo "BLOCKED/2uq2_vendor_sdk_missing" >&2; exit 3; }
  id robot >/dev/null 2>&1 || fail "FAIL/robot_account_missing"
  ensure_root_directory /run/three-device-slam
  acquire_install_lock /run/three-device-slam/install.lock
  install_root=/opt/three-device-slam
  ensure_root_directory "$install_root"
  require_trusted_tree "$install_root" "${install_root}/venv/lib64"
  ensure_root_directory /etc/three-device-slam
  require_trusted_tree /etc/three-device-slam
  ensure_session_directory /var/lib/three-device-slam/sessions

  MAKEFLAGS= make -C "$repo_root/native/2uq2_xu_bridge" clean all \
    VENDOR_SDK="$vendor_sdk" CC=cc RM=rm
  artifact="$repo_root/native/2uq2_xu_bridge/build/libtwo_uq2_xu.so"
  trap cleanup_temporary_lib EXIT
  install_runtime "$artifact" "$install_root" "$repo_root"
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then main "$@"; fi
