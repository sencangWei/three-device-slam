#!/usr/bin/env python3
"""Export one accepted D435i Ego capture for official ORB-SLAM3 replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from three_device_slam.devices.d435i_ego.orbslam3_export import export


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--diagnostic-end-s",
        type=float,
        help="export a continuous prefix from a failed capture; never accepted",
    )
    args = parser.parse_args()
    print(
        json.dumps(
            export(
                args.source,
                args.output,
                diagnostic_end_s=args.diagnostic_end_s,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
