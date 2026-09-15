"""UMI AprilGrid adapter for the legacy two-border board, not external tags.

Uses aprilgrid 0.5.0's existing detector without global monkey-patching. Preserve
decoded duplicate identities before upstream's largest-area ID suppression.
"""
from importlib.metadata import version
import numpy as np
from scripts.color_aprilgrid_capture import make_detector


class RawFamily:
    def __init__(self, family):
        self.family, self.raw = family, []

    def __getattr__(self, name):
        return getattr(self.family, name)

    def decodeQuad(self, *args):
        self.raw = self.family.decodeQuad(*args)
        return self.raw


class UmiBoardDetector:
    def __init__(self):
        if version('aprilgrid') != '0.5.0':
            raise RuntimeError('requires audited aprilgrid 0.5.0; no silent fallback')
        from aprilgrid import Detector
        # At native1280x720, avoid the upstream1000px downscale and its border
        # remapping cornerSubPix crash. No resize/temporal caching for detection.
        self.detector = Detector('t36h11', large_image_threshold=2000)
        self.family = RawFamily(self.detector.tag_family)
        self.detector.tag_family = self.family

    def detectMarkers(self, gray):
        self.family.raw = []
        self.detector.detect(gray)
        corners, ids = [], []
        for detection in self.family.raw:
            q = np.asarray(detection.corners,dtype=np.float32).reshape(4,2)
            # AprilGrid's decoded bit orientation -> frozen object_corners order.
            corners.append(q[[1,0,3,2]].reshape(1,4,2))
            ids.append(int(detection.tag_id))
        return corners, np.array(ids,dtype=np.int32).reshape(-1,1) if ids else None, []


def make_board_detector(backend):
    if backend == 'opencv':
        return make_detector()
    if backend == 'umi-aprilgrid':
        return UmiBoardDetector()
    raise ValueError('unknown board detector backend')
