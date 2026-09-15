#!/usr/bin/env python3
"""Dual-camera, mixed80/40 bench preview. No raw capture or calibration activation."""
import argparse
import json
import signal
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from three_device_slam.spatial.apriltag_detector import CameraCalibration
from three_device_slam.spatial.mixed_size_crosscheck import (
    CORNER_REFINEMENTS, DEFAULT_CORNER_REFINEMENT, detect_pair, geometry,
    preview_resolved_roles, role_geometry,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=0, help="0 = until q/ESC")
    parser.add_argument("--corner-refinement", choices=CORNER_REFINEMENTS,
                        default=DEFAULT_CORNER_REFINEMENT)
    args = parser.parse_args()
    args.snapshot_dir.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    pipelines, calibrations = {}, {}
    try:
        for role, serial in (("ego", "327122078613"), ("umi", "260322279785")):
            pipeline, config = rs.pipeline(), rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
            profile = pipeline.start(config)
            pipelines[role] = pipeline
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            calibrations[role] = CameraCalibration(
                calibration_id=serial, frame_id=role, width=intr.width, height=intr.height,
                camera_matrix=np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]]),
                distortion_coefficients=np.array(intr.coeffs), distortion_model=str(intr.model))
            if role == "umi":
                sensor = profile.get_device().first_depth_sensor()
                sensor.set_option(rs.option.enable_auto_exposure, 0)
                sensor.set_option(rs.option.exposure, 30000)
                sensor.set_option(rs.option.gain, 96)
            else:
                color_sensor = profile.get_device().first_color_sensor()
                color_sensor.set_option(rs.option.enable_auto_exposure, 1)
                sensor = profile.get_device().first_depth_sensor()
                if sensor.supports(rs.option.emitter_enabled):
                    sensor.set_option(rs.option.emitter_enabled, 0)
            cv2.namedWindow(role.upper() + " mixed80/40", cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow(role.upper() + " mixed80/40", 20 if role == "ego" else 840, 60)
        print("PREVIEW: both cameras open; external ID1/2=80mm; mount ID1=40mm; q/ESC closes.", flush=True)
        print(f"PREVIEW: corner_refinement={args.corner_refinement}; backend=opencv_aruco_reference", flush=True)
        started, logged = time.monotonic(), 0
        while not args.duration or time.monotonic() - started < args.duration:
            images = {}
            try:
                for role, pipeline in pipelines.items():
                    color = pipeline.wait_for_frames(1000).get_color_frame()
                    if not color:
                        raise RuntimeError("missing color frame")
                    images[role] = np.asanyarray(color.get_data()).copy()
            except RuntimeError as exc:
                print(f"PREVIEW: NO FRESH PAIR: {exc}", flush=True)
                for role in pipelines:
                    blank = np.zeros((450, 800, 3), dtype=np.uint8)
                    cv2.putText(blank, "NO FRESH PAIR", (20, 80), 0, 1, (0, 0, 255), 2)
                    cv2.imshow(role.upper() + " mixed80/40", blank)
                if cv2.waitKey(100) & 255 in (27, ord("q")):
                    break
                continue
            stamp = time.monotonic_ns()
            roles, info, raw = detect_pair(
                cv2.cvtColor(images["ego"], cv2.COLOR_RGB2GRAY),
                cv2.cvtColor(images["umi"], cv2.COLOR_RGB2GRAY),
                calibrations["ego"], calibrations["umi"], stamp, stamp,
                corner_refinement=args.corner_refinement)
            geo = geometry(roles) if roles else None
            good = bool(geo and geo["good"])
            labels = preview_resolved_roles(roles, info, raw)
            canvases = {}
            for camera in images:
                canvas = cv2.cvtColor(images[camera], cv2.COLOR_RGB2BGR)
                for d in raw[camera]:
                    assigned = any(role.startswith(camera) and np.max(np.abs(d.corners_px - match.corners_px)) < 0.1
                                   for role, match in labels.items())
                    if not assigned:
                        tint = (160, 160, 160) if roles else (0, 190, 255)
                        text = "extra / ignored" if roles else "unassigned"
                        cv2.polylines(canvas, [d.corners_px.astype(np.int32)], True, tint, 2)
                        cv2.putText(canvas, f"ID{d.tag_id} {text}", tuple(d.corners_px[0].astype(int)),
                                    0, 0.6, tint, 2)
                for role, d in labels.items():
                    if not role.startswith(camera):
                        continue
                    g = role_geometry(role, d)
                    tint = (0, 220, 0) if g["good"] else (0, 180, 255)
                    cv2.polylines(canvas, [d.corners_px.astype(np.int32)], True, tint, 3)
                    label = f"{role}: {g['size_mm']}mm {g['min_edge_px']:.0f}px {g['incidence_deg']:.1f}deg"
                    origin = (int(np.clip(d.corners_px[:, 0].min(), 5, 750)),
                              int(np.clip(d.corners_px[:, 1].min() - 10, 120, 690)))
                    cv2.putText(canvas, label, origin, 0, 0.65, tint, 2)
                canvas = cv2.resize(canvas, (800, 450))
                cv2.rectangle(canvas, (0, 0), (800, 66), (20, 20, 20), -1)
                status = "GEOMETRY OK" if good else (
                    "IDENTITY OK / CHECK ANGLE, SIZE OR GAP" if roles else "IDENTITY CHECK / " + info["reason"])
                cv2.putText(canvas, status,
                            (8, 23), 0, 0.48, (0, 220, 0) if good else (0, 190, 255), 1)
                hint = f"{args.corner_refinement} | Ext80/back40 | >=60px <45deg | q:close"
                if geo:
                    hint += f" | gap {geo['external_spacing_m'][camera]*100:.1f}cm"
                cv2.putText(canvas, hint, (8, 49), 0, 0.42, (255, 255, 255), 1)
                cv2.imshow(camera.upper() + " mixed80/40", canvas)
                canvases[camera] = canvas
            if time.monotonic() - logged > 5:
                summary = dict(identity=info, geometry=geo,
                               display_resolved_roles=list(labels),
                               corner_refinement=args.corner_refinement,
                               preview_only="unsynchronized live frames; not calibration acceptance")
                (args.snapshot_dir / "latest.json").write_text(json.dumps(summary, indent=2))
                for camera, canvas in canvases.items():
                    cv2.imwrite(str(args.snapshot_dir / f"{camera}.png"), canvas)
                print("PREVIEW:", "GEOMETRY OK" if good else info["reason"],
                      json.dumps({r: [round(g["min_edge_px"], 1), round(g["incidence_deg"], 1)]
                                  for r, g in (geo or {}).get("roles", {}).items()}), flush=True)
                logged = time.monotonic()
            if cv2.waitKey(50) & 255 in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        for pipeline in pipelines.values():
            try:
                pipeline.stop()
            except RuntimeError:
                pass
        cv2.destroyAllWindows()
        print("PREVIEW: closed; camera ownership released.", flush=True)


if __name__ == "__main__":
    main()
