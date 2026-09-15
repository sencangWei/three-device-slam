#!/usr/bin/env python3
"""Shared detection ladder for the sub-mm dual-IR capture + analysis.

Two board detectors exist in this repo for the same physical 6x6 board:
  * scripts.color_aprilgrid_capture.make_detector  -- OpenCV aruco, 2-bit
    border (the legacy Kalibr board print), used by the capture preview.
  * scripts.common_board_detector.make_board_detector('umi-aprilgrid') --
    the audited aprilgrid 0.5.0 lib detector, used by the analysis.

Both get the same robustness treatment here:
  * per-frame UNION of raw + CLAHE-boosted detections (more common tags on
    the soft-focus ego IR streams; both passes are real decodes)
  * ego D435i IR is soft-focus: extra decode tolerance (aruco
    errorCorrectionRate 0.9; aprilgrid decode_sharpening 0.7 +
    min_white_black_diff 3) plus an unsharp-mask rung for the mount ladder
  * the mount tag shares id1 with the board: hull rejection keeps only
    quads whose center falls outside the board-tag convex hull
"""
import cv2
import numpy as np

from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector as make_aprilgrid_detector


def clahe_boost(gray):
    return cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)


def unsharp(gray, sigma=2.0, amount=0.7):
    blur = cv2.GaussianBlur(gray, (0, 0), sigma)
    return cv2.addWeighted(gray, 1.0 + amount, blur, -amount, 0)


def make_board_detector_opencv():
    """cb.make_detector() (2-bit border board) + lenient error correction
    for the soft-focus IR decode. Fresh params each call, shared dict."""
    p = cv2.aruco.DetectorParameters()
    p.markerBorderBits = 2
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.errorCorrectionRate = 0.9
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)


def make_board_detector_aprilgrid():
    """The audited aprilgrid detector with soft-focus-tolerant decode.  Only
    Detector-level knobs change; the RawFamily decode hook is reused as-is."""
    det = make_aprilgrid_detector('umi-aprilgrid')
    from aprilgrid import Detector as AprilDetector
    tuned = AprilDetector('t36h11', refine_edges=True, decode_sharpening=0.7,
                          min_white_black_diff=3, large_image_threshold=2000)
    tuned.tag_family = det.detector.tag_family  # keep UmiBoardDetector's raw-capture hook
    det.detector = tuned
    return det


def detect_board_union(detector, gray):
    """Union of raw + CLAHE detections (max tag coverage per frame)."""
    found = {}
    for img in (gray, clahe_boost(gray)):
        for d in cb.detect_grid(detector, img):
            found.setdefault(int(d.tag_id), np.asarray(d.corners, dtype=float).reshape(4, 2))
    return found


def make_mount_detector(error_correction=0.6):
    p = cv2.aruco.DetectorParameters()
    p.markerBorderBits = 1
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    p.errorCorrectionRate = error_correction
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)


def make_mount_detector_relaxed():
    p = cv2.aruco.DetectorParameters()
    p.markerBorderBits = 1
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    p.errorCorrectionRate = 0.9
    p.minMarkerPerimeterRate = 0.015
    p.minOtsuStdDev = 2.0
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)


def detect_mount_ladder(gray):
    """default -> relaxed on CLAHE -> relaxed on inverted polarity (OpenCV
    36h11 polarity quirk) -> relaxed on unsharp. First rung with hits wins."""
    tags = cb.detect_grid(make_mount_detector(), gray)
    if tags:
        return tags
    relaxed = make_mount_detector_relaxed()
    for img in (clahe_boost(gray), 255 - gray, unsharp(gray)):
        tags = cb.detect_grid(relaxed, img)
        if tags:
            return tags
    return []


def reject_board_id1(candidates, board):
    """Mount quads whose center is inside the board hull are board id1
    duplicates; keep only outsiders (fall back to farthest when the board
    is barely visible and no outsider survives). Accepts either raw quads
    or detect_grid() namespace objects with a .corners attribute."""
    def quad_of(q):
        return np.asarray(getattr(q, 'corners', q), dtype=np.float32).reshape(4, 2)

    if not candidates:
        return [quad_of(q) for q in candidates]
    hull = None
    if len(board) >= 4:
        centers = np.concatenate([np.asarray(board[i], dtype=np.float32).reshape(4, 2).mean(axis=0, keepdims=True)
                                  for i in board]).astype(np.float32)
        hull = cv2.convexHull(centers)
    kept = []
    for q in candidates:
        q = quad_of(q)
        if hull is not None:
            c = q.mean(axis=0)
            if cv2.pointPolygonTest(hull, (float(c[0]), float(c[1])), False) >= 0:
                continue
        kept.append(q)
    if not kept and candidates and hull is not None:
        centroid = hull.reshape(-1, 2).mean(axis=0)
        kept = [max((quad_of(q) for q in candidates),
                    key=lambda q: np.linalg.norm(q.mean(axis=0) - centroid))]
    return kept
