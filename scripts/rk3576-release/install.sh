#!/usr/bin/env bash
# Install a verified application release without replacing OS, data, or services.
set -euo pipefail
UMI_SOURCE=$(cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)
UMI_INSTALL_ROOT=${1:?Usage: bash install.sh /absolute/path/to/umi-collector}
case "$UMI_INSTALL_ROOT" in /*) ;; *) echo 'Install path must be absolute'; exit 1;; esac
test "$UMI_INSTALL_ROOT" != / || exit 1
cd "$UMI_SOURCE"
sha256sum --strict --check SHA256SUMS
bash "$UMI_SOURCE/check-host.sh"
UMI_VERSION=$(python3 -c 'import json; print(json.load(open("release-manifest.json"))["version"])')
[[ "$UMI_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || exit 1
UMI_TARGET="$UMI_INSTALL_ROOT/releases/$UMI_VERSION"
test ! -e "$UMI_TARGET" || { echo "Version already installed: $UMI_TARGET"; exit 1; }
if [ -e "$UMI_INSTALL_ROOT/current" ] && [ ! -L "$UMI_INSTALL_ROOT/current" ]; then
  echo 'Refusing to replace a non-symlink current path'; exit 1
fi
mkdir -p -- "$UMI_INSTALL_ROOT/releases"
UMI_STAGE=$(mktemp -d "$UMI_INSTALL_ROOT/releases/.install-XXXXXX")
cp -a -- "$UMI_SOURCE/." "$UMI_STAGE/"
(cd "$UMI_STAGE" && sha256sum --strict --check SHA256SUMS)
chmod +x "$UMI_STAGE/bin/umi-record" "$UMI_STAGE/bin/umi-validate"
mv -T -- "$UMI_STAGE" "$UMI_TARGET"
UMI_LINK="$UMI_INSTALL_ROOT/.current-$$"
ln -s -- "$UMI_TARGET" "$UMI_LINK"
mv -Tf -- "$UMI_LINK" "$UMI_INSTALL_ROOT/current"
printf 'Installed: %s\nRun: %s/current/bin/umi-record --output-root /path/to/sessions --d405-sdk-serial SDK_ID --d405-usb-serial USB_ID --duration 60 --until-signal --preview-listen 0.0.0.0\n' "$UMI_TARGET" "$UMI_INSTALL_ROOT"
