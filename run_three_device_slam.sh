#!/usr/bin/env bash
set -euo pipefail

exec python3 -m three_device_slam.cli "$@"
