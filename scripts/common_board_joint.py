"""Same-frame public-board/mount identity, with fixed factory geometry.

Board ID1 is excluded BEFORE measurement in both cameras; all raw evidence stays.
The mount must be the unique ID1 outside the board footprint. A footprint fit is
only an identity check, never camera calibration or a point-rejection optimizer.
"""
import cv2
import numpy as np
from scripts import common_board_calibration as cb

POLICY = dict(name='board35_plus_separate_mount1_v1', excluded_board_ids=[1],
              board_h_p95_px=5., board_h_max_px=12., footprint_margin_m=cb.GAP_M,
              min_outside_gap_px=3., mount_min_edge_px=60., same_frame_mount_p95_px=1.)

STEPS=[*cb.STEPS[:12],*[('BOARD + MOUNT STATIC CHECK','mount','mount') for _ in range(3)],
       ('FINAL BOARD + MOUNT CHECK','board','holdout')]


def board_geometry(raw):
    ds=cb.valid_detections([d for d in raw if int(d.tag_id)!=1])
    if not cb.board_coverage(ds):
        raise ValueError('need_12_board_tags_3rows_3cols')
    ids=sorted(ds)
    obj=np.concatenate([cb.object_corners(i)[:,:2] for i in ids])
    pix=np.concatenate([ds[i] for i in ids])
    h,_=cv2.findHomography(obj,pix,0)  # all observations; no RANSAC/outlier deletion
    if h is None or not np.isfinite(h).all():
        raise ValueError('board_identity_geometry')
    err=np.linalg.norm(cv2.perspectiveTransform(obj.reshape(-1,1,2),h).reshape(-1,2)-pix,axis=1)
    if np.percentile(err,95)>POLICY['board_h_p95_px'] or err.max()>POLICY['board_h_max_px']:
        raise ValueError('board_identity_geometry')
    low=-cb.TAG_M/2-POLICY['footprint_margin_m']
    high=5*(cb.TAG_M+cb.GAP_M)+cb.TAG_M/2+POLICY['footprint_margin_m']
    outer=np.array([[low,low],[high,low],[high,high],[low,high]],np.float64)
    homogeneous=np.c_[outer,np.ones(4)]@h.T
    if np.any(abs(homogeneous[:,2])<1e-9) or np.any(homogeneous[:,2]*homogeneous[0,2]<=0):
        raise ValueError('board_identity_geometry')
    footprint=(homogeneous[:,:2]/homogeneous[:,2:]).astype(np.float32)
    if not cv2.isContourConvex(footprint):
        raise ValueError('board_identity_geometry')
    return ds,footprint,float(np.percentile(err,95))


def location(quad, footprint):
    q=np.asarray(quad,dtype=np.float32).reshape(4,2)
    if not np.isfinite(q).all() or not cv2.isContourConvex(q):
        raise ValueError('invalid_target_quad')
    signed=[cv2.pointPolygonTest(footprint,tuple(map(float,p)),True) for p in q]
    if min(signed)>=0:
        return 'board'
    if max(signed)<-POLICY['min_outside_gap_px'] and cv2.intersectConvexConvex(q,footprint)[0]==0:
        return 'outside'
    raise ValueError('mount_overlaps_board_outline')


def resolve_targets(raw, tags):
    ds,footprint,p95=board_geometry(raw)
    candidates=[]
    for tag in tags:
        if int(tag.tag_id)==1 and location(tag.corners,footprint)=='outside':
            candidates.append(tag)
    if len(candidates)!=1:
        raise ValueError('need_unique_mount1_outside_board')
    chosen=candidates[0]
    q=np.asarray(chosen.corners,dtype=float)
    if np.linalg.norm(q-np.roll(q,1,axis=0),axis=1).min()<POLICY['mount_min_edge_px']:
        raise ValueError('mount_edge_below_60px')
    # Raw two-border aliases can be the SAME physical mount, but cannot hide a
    # second external ID1 that the one-border detector missed.
    for tag in raw:
        if int(tag.tag_id)==1 and location(tag.corners,footprint)=='outside' and not cb.same_mount_quad(tag,chosen):
            raise ValueError('unmatched_external_id1')
    return ds,q,dict(excluded_board_ids=[1],footprint=footprint.tolist(),board_h_p95_px=p95)


def detect_joint_sample(images,kind,board_detector,tag_detector,*,display=None):
    roles=cb.role_pair(images);umi_role=roles[1]
    boards={r:{} for r in roles};mount={r:{} for r in roles};reasons=[]
    for role in roles:
        raw=cb.detect_grid(board_detector,images[role])
        tags=cb.detect_grid(tag_detector,images[role]) if role=='ego' else []
        if display is not None:
            ids=[int(d.tag_id) for d in raw if 0<=int(d.tag_id)<36 and int(d.tag_id)!=1]
            display[role]=dict(raw=raw,duplicate_ids=sorted(i for i in set(ids) if ids.count(i)>1),
                               excluded_board_ids=[1],mount_candidates=[d for d in tags if d.tag_id==1])
        try:
            ds,_,_=board_geometry(raw)
            boards[role]=cb.refine_legacy_corners(images[role],ds)
            if role=='ego':
                ds,q,proof=resolve_targets(raw,tags)
                mount['ego'][1]=q
                if display is not None:display[role]['identity']=proof
        except ValueError as exc:
            reasons.append(role+':'+str(exc))
    if not cb.board_coverage(set(boards['ego'])&set(boards[umi_role])):
        reasons.append('need_12_common_tags_3rows_3cols')
    selected=boards if kind=='board' else mount
    return dict(selected,_board=boards,_mount=mount),reasons


def joint_window_quality(samples):
    def quality(key,kind):
        return cb.window_quality([dict(s,detected=s['detected'][key]) for s in samples],kind)
    board,mount=quality('_board','board'),quality('_mount','mount')
    result=dict(board)
    result['reasons']=sorted(set(board['reasons']+['mount:'+r for r in mount['reasons']]))
    result['roles']=dict(board['roles'])
    if 'ego' in mount['roles']:result['roles']['mount']=mount['roles']['ego']
    result['simultaneous']=dict(policy=POLICY,board=board,mount=mount)
    return result


def same_frame_check(window, mount_pixels, calibrations, fixed_mount):
    """Board-derived X predicts the separate mount, without fitting M to this frame."""
    roles=cb.role_pair(calibrations);umi_role=roles[1]
    poses={r:cb.fit_pose(window['objects'],window['pixels'][r],calibrations[r]) for r in roles}
    x=poses['ego']@np.linalg.inv(poses[umi_role])
    prediction=cb.errors(cb.project(cb.MOUNT_OBJECTS,x@np.asarray(fixed_mount),calibrations['ego']),mount_pixels)
    candidates=[c for c in cb._solve_ippe_candidates(mount_pixels,calibration=calibrations['ego'],tag_size_m=.04) if c.positive_depth]
    return dict(slot=window['slot'],split=window['split'],T_Ecolor_Ucolor=x.tolist(),
        fixed_M_prediction=prediction,passed=prediction['p95_px']<=POLICY['same_frame_mount_p95_px'],
        diagnostic_mount_candidates=[dict(T_Ucolor_mount=(np.linalg.inv(x)@c.camera_from_tag).tolist(),
                                          reprojection_error_px=c.reprojection_error_px) for c in candidates],
        note='M fixed before this check; candidate branches are diagnostic, not selected by expected mechanical distance')
