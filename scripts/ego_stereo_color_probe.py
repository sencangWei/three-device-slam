#!/usr/bin/env python3
"""Preview-first Ego RGB/IR stereo evidence, no calibration or SLAM changes."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.pair_color_tag_capture import _intrinsics_dict
from three_device_slam.devices.realsense_extrinsics import serialize_sdk_extrinsics

SERIAL = '327122078613'
STREAMS = ('color', 'ir_left', 'ir_right')
WINDOW = 'EGO RGB + IR LEFT + IR RIGHT | R record 10s | Q stop'


def main():
    import pyrealsense2 as rs
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record-now', action='store_true', help='Only after explicit operator capture authorization')
    args = parser.parse_args()
    out = cb.new_output(args.output)
    out.mkdir(parents=True, exist_ok=False)
    pipe, started, rows = rs.pipeline(), False, []
    report = dict(status='INCOMPLETE', activation='NOT_ACTIVATED', failure=None, cleanup_errors=[])
    previous, first, last_saved, last_preview = {}, None, -1., -1.
    handle = None
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(WINDOW, np.zeros((180, 960, 3), np.uint8))
        cv2.waitKey(1)
        cfg = rs.config()
        cfg.enable_device(SERIAL)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
        cfg.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
        cfg.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 30)
        profile = pipe.start(cfg)
        started = True
        device = profile.get_device()
        if device.get_info(rs.camera_info.serial_number) != SERIAL:
            raise ValueError('wrong device')
        sensor = device.first_depth_sensor()
        sensor.set_option(rs.option.emitter_enabled, 0)
        if sensor.get_option(rs.option.emitter_enabled) != 0:
            raise ValueError('emitter off not confirmed')
        profiles = dict(color=profile.get_stream(rs.stream.color).as_video_stream_profile(),
                        ir_left=profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile(),
                        ir_right=profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile())
        for role, p in profiles.items():
            expected = rs.format.rgb8 if role == 'color' else rs.format.y8
            if (p.width(), p.height(), p.fps(), p.format()) != (1280, 720, 30, expected):
                raise ValueError('resolved stream profile mismatch: '+role)
        setup = dict(schema='ego.stereo_color_probe.v1', serial=SERIAL,
                     factory_values_modified=False, activation='NOT_ACTIVATED',
                     intrinsics={r: _intrinsics_dict(p.get_intrinsics()) for r, p in profiles.items()},
                     extrinsics={r: serialize_sdk_extrinsics(profiles['ir_left'].get_extrinsics_to(profiles[r]))
                                 for r in ('ir_right', 'color')},
                     emitter_readback=0., duration_seconds=10., storage_sample_hz=2., native_fps=30,
                     timing='SDK frameset; preserve individual stamps, not a claim of RGB exposure sync',
                     operator_contract='camera and visible board stay static; R starts; Q keeps partial evidence',
                     code_sha256=cb.digest(Path(__file__).resolve()))
        cb.write_json(out/'setup.json', setup)
        handle = (out/'frames.jsonl').open('x')
        setup_hash = cb.digest(out/'setup.json')
        detector = cb.make_detector()
        ready_at = time.monotonic()+2.
        deadline = time.monotonic()+300.
        print('OPEN: Ego RGB+IR-L+IR-R 1280x720@30; emitter OFF. Preview only, R starts10s.', flush=True)
        while time.monotonic() < deadline:
            frame_set = pipe.wait_for_frames(1000)
            frames = dict(color=frame_set.get_color_frame(), ir_left=frame_set.get_infrared_frame(1),
                          ir_right=frame_set.get_infrared_frame(2))
            if not all(frames.values()):
                continue
            arrival_ns = time.monotonic_ns()
            now = time.monotonic()
            arrays = {r: np.asanyarray(f.get_data()).copy() for r, f in frames.items()}
            arrays['color'] = cv2.cvtColor(arrays['color'], cv2.COLOR_RGB2BGR)
            key = cv2.waitKey(1) & 255
            if key in (27, ord('q'), ord('Q')):
                raise RuntimeError('operator_aborted')
            if first is None and now >= ready_at and (args.record_now or key in (ord('r'), ord('R'))):
                first = now
                print('RECORDING: keep camera/board still; 10seconds, sampled2Hz.', flush=True)
            if now-last_preview >= .125:
                tiles, counts = [], {}
                for r in STREAMS:
                    gray = cv2.cvtColor(arrays[r], cv2.COLOR_BGR2GRAY) if r == 'color' else arrays[r]
                    found = cb.detect_grid(detector, gray)
                    counts[r] = len(found)
                    tile = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                    for d in found:
                        cv2.polylines(tile, [np.round(d.corners).astype(np.int32)], True, (0, 220, 0), 2)
                    tile = cv2.resize(tile, (480, 270))
                    cv2.putText(tile, f'{r}: {len(found)} tags', (8, 24), 0, .65, (0, 255, 255), 2)
                    tiles.append(tile)
                view = np.hstack(tiles)
                cv2.imshow(WINDOW, view)
                last_preview = now
            if first is not None and now-last_saved >= .5:
                row = dict(sample=len(rows), setup_sha256=setup_hash, streams={})
                for r in STREAMS:
                    f = frames[r]
                    stamp, count = f.get_timestamp(), f.get_frame_number()
                    domain = str(f.get_frame_timestamp_domain())
                    if not np.isfinite(stamp) or (r in previous and (stamp <= previous[r][0] or count <= previous[r][1] or domain != previous[r][2])):
                        raise ValueError('timestamp/counter regression')
                    previous[r] = (stamp, count, domain)
                    path = out/f'{len(rows):03d}_{r}.png'
                    if not cv2.imwrite(str(path), arrays[r]):
                        raise RuntimeError('PNG write failed')
                    row['streams'][r] = dict(file=path.name, sha256=cb.digest(path), sdk_timestamp_ms=stamp,
                                             sdk_timestamp_domain=str(f.get_frame_timestamp_domain()),
                                             frame_number=count, host_arrival_monotonic_ns=arrival_ns)
                rows.append(row)
                handle.write(json.dumps(row)+'\n')
                handle.flush()
                last_saved = now
                print(f'SAVED {len(rows)} triples; elapsed={now-first:.1f}/10s; tags={counts}', flush=True)
            if first is not None and now-first >= 10:
                report['status'] = 'CAPTURE_COMPLETE_REQUIRES_OFFLINE_VALIDATION'
                break
        if report['status'] == 'INCOMPLETE':
            raise RuntimeError('preview_or_capture_timeout')
    except (Exception, KeyboardInterrupt) as exc:
        report['failure'] = type(exc).__name__+':'+str(exc)
    finally:
        if handle:
            handle.close()
        if started:
            try:
                pipe.stop()
            except Exception as exc:
                report['cleanup_errors'].append(str(exc))
        try:
            cv2.destroyAllWindows()
        except Exception as exc:
            report['cleanup_errors'].append(str(exc))
        report.update(samples=len(rows), duration_s=None if first is None else time.monotonic()-first)
        report['hashes'] = {p.name: cb.digest(p) for p in (out/'setup.json', out/'frames.jsonl') if p.exists()}
        cb.write_json(out/'capture_report.json', report)
        print(json.dumps(report), flush=True)
    return 0 if report['status'].startswith('CAPTURE_COMPLETE') and not report['cleanup_errors'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
