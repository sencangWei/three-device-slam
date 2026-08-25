"""Provisioned one-click product capture and offline processing entry point."""

from __future__ import annotations

import argparse
import signal
import sys
import threading
from pathlib import Path

from three_device_slam.acquisition import coordinator
from three_device_slam.config import ConfigError, load_product_config
from three_device_slam.quality import verify_session as session_verifier
from three_device_slam.synchronization import build_index as index_builder


DEFAULT_CONFIG_PATH = Path("/etc/three-device-slam/product.json")
EXIT_CODES = {"PASS": 0, "FAIL": 2, "BLOCKED": 3}

run_capture = coordinator.run_product_capture
build_index = index_builder.build_index
verify_session = session_verifier.verify_session


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run the three-device SLAM product.")
    commands = parser.add_subparsers(dest="command", required=True)
    product = commands.add_parser(
        "product", help="capture, seal, index, and verify one product session"
    )
    product.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"provisioned config (default: {DEFAULT_CONFIG_PATH})",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.command != "product":  # argparse currently makes this unreachable.
        raise ValueError("unsupported command")
    try:
        config = load_product_config(args.config)
    except FileNotFoundError:
        _error("BLOCKED/device_config_missing")
        return EXIT_CODES["BLOCKED"]
    except ConfigError:
        _error("FAIL/device_config_invalid")
        return EXIT_CODES["FAIL"]
    except OSError:
        _error("FAIL/device_config_unreadable")
        return EXIT_CODES["FAIL"]

    stop_event = threading.Event()

    def request_stop(_signum, _frame):
        stop_event.set()

    previous_handlers = {
        number: signal.signal(number, request_stop)
        for number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        return _run_product(config, stop_event)
    finally:
        for number, previous in previous_handlers.items():
            signal.signal(number, previous)


def _run_product(config, stop_event: threading.Event) -> int:
    try:
        capture_result = run_capture(config, stop_event.is_set)
        acquisition_status = _status(capture_result.report, "capture")
        calibration_status = _calibration_status(capture_result.report)
        session = Path(capture_result.session)
    except Exception:
        _error("FAIL/capture_exception")
        return EXIT_CODES["FAIL"]

    print(f"session={session}")
    print(f"acquisition={acquisition_status}")
    print(f"calibration={calibration_status}")
    capture_status = _combine_status(acquisition_status, calibration_status)
    if acquisition_status != "PASS" or calibration_status == "FAIL":
        print(f"overall={capture_status}")
        return EXIT_CODES[capture_status]

    try:
        build_index(session)
    except Exception:
        _error("FAIL/index_exception")
        return EXIT_CODES["FAIL"]
    print("index=PASS")

    try:
        verification_report = verify_session(session)
        verification_status = _status(verification_report, "verification")
        overall = session_verifier.compose_product_status(verification_status)[
            "overall"
        ]
    except Exception:
        _error("FAIL/verification_exception")
        return EXIT_CODES["FAIL"]
    print(f"verification={verification_status}")
    overall = _combine_status(overall, capture_status)
    print(f"overall={overall}")
    return EXIT_CODES[overall]


def _status(report, stage: str) -> str:
    if not isinstance(report, dict) or report.get("status") not in EXIT_CODES:
        raise ValueError(f"invalid {stage} report")
    return report["status"]


def _calibration_status(report) -> str:
    expectations = report.get("calibration_expectations")
    if not isinstance(expectations, dict) or set(expectations) != {
        "ego",
        "left",
        "right",
    }:
        return "BLOCKED"
    statuses = []
    for device in ("ego", "left", "right"):
        item = expectations[device]
        if not isinstance(item, dict) or item.get("status") not in EXIT_CODES:
            return "FAIL"
        statuses.append(item["status"])
    return max(statuses, key={"PASS": 0, "BLOCKED": 1, "FAIL": 2}.get)


def _combine_status(*statuses: str) -> str:
    return max(statuses, key={"PASS": 0, "BLOCKED": 1, "FAIL": 2}.get)


def _error(reason: str) -> None:
    print(reason, file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
