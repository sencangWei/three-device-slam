#!/bin/bash
set -euo pipefail

exec /opt/three-device-slam/venv/bin/python -I -B -m three_device_slam.cli "$@"
