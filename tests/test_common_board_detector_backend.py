from types import SimpleNamespace as NS
import numpy as np
import pytest
from scripts import common_board_calibration as cb
from scripts import common_board_capture as cc


def test_duplicate_display_keeps_other_tags_but_acceptance_still_blocks(monkeypatch):
    raw=[NS(tag_id=i,corners=np.array([[100.,100],[130,100],[130,130],[100,130]])+[i*2,0]) for i in range(18)]
    raw.append(NS(tag_id=1,corners=raw[1].corners+[300,0]))
    monkeypatch.setattr(cb,'detect_grid',lambda *a:raw)
    display={}
    _,reasons=cb.detect_sample({r:np.zeros((720,1280),np.uint8) for r in cb.ROLES},
                             'board',None,None,display=display)
    assert 'ego:duplicate_board_id' in reasons
    assert len(display['ego']['raw'])==19
    assert display['ego']['duplicate_ids']==[1]
    assert '同编号' in cc.quality_text(dict(reasons=reasons,roles={}))[1]


def test_umi_adapter_preserves_duplicate_ids_and_reorders_corners():
    from scripts.common_board_detector import UmiBoardDetector
    detector=UmiBoardDetector()
    raw=[NS(tag_id=1,corners=np.arange(8).reshape(4,1,2)),
         NS(tag_id=1,corners=np.arange(8).reshape(4,1,2)+100)]
    class Fake:
        def detect(self,image):
            detector.family.raw=raw
            return raw[:1]  # Upstream ID suppression must not discard second physical tag.
    detector.detector=Fake()
    corners,ids,_=detector.detectMarkers(np.zeros((80,80),np.uint8))
    assert ids.ravel().tolist()==[1,1]
    np.testing.assert_array_equal(corners[0].reshape(4,2),np.arange(8).reshape(4,2)[[1,0,3,2]])


def test_unknown_backend_rejected():
    from scripts.common_board_detector import make_board_detector
    with pytest.raises(ValueError):make_board_detector('unknown')


@pytest.mark.parametrize('rotation',range(4))
def test_real_decoder_canonical_corners_all_four_orientations(rotation):
    import cv2
    from scripts.common_board_detector import make_board_detector
    image=np.full((600,600),255,np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    image[180:420,180:420]=cv2.aruco.generateImageMarker(dictionary,19,240,borderBits=2)
    image=np.ascontiguousarray(np.rot90(image,rotation))
    old=cb.detect_grid(make_board_detector('opencv'),image)
    new=cb.detect_grid(make_board_detector('umi-aprilgrid'),image)
    assert len(old)==1 and new and all(d.tag_id==19 for d in new)
    # A synthetic isolated marker can generate nested quads; keep conflicts,
    # never use this minimum selection for production detection/acceptance.
    distances=[np.max(np.linalg.norm(d.corners-old[0].corners,axis=1)) for d in new]
    assert min(distances)<.01
    # Even nested quads must retain canonical bit orientation, not reversed/cyclic.
    assert max(distances)<15
