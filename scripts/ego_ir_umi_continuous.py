"""Automatic static A/B gate; camera ownership remains in the capture process."""
import cv2
import numpy as np
from scripts import common_board_calibration as cb
from scripts.common_board_detector import make_board_detector
from scripts.common_board_joint import board_geometry
from scripts.common_board_resampling_analysis import restore_board_id1
from scripts.ego_joint_stereo_diagnostic import diagnostic_targets


class ContinuousAB:
    def __init__(self,return_to_a=False,tilt_contrast=False):
        if tilt_contrast and not return_to_a:raise ValueError('tilt requires locked ABA')
        self.tilt_contrast=tilt_contrast
        self.reference_normals=None
        self.tilt_degrees=None
        self.return_to_a=return_to_a
        self.phases=('A','B','C') if return_to_a else ('A','B')
        self.state='WAIT_A'
        self.history=[]
        self.record_start=None
        self.reference=None
        self.events=[]

    def update(self,now,points,normals=None):
        if self.state=='DONE':return None,'DONE'
        if self.state.startswith('RECORD'):
            if now-self.record_start<10:
                return self.state[-1],f'{self.state}: HOLD STILL {now-self.record_start:.1f}/10s'
            if self.state=='RECORD_'+self.phases[-1]:
                self.state='DONE';self.events.append(dict(time=now,state=self.state))
                return None,'DONE'
            self.state='WAIT_'+self.phases[self.phases.index(self.state[-1])+1];self.history=[]
            self.events.append(dict(time=now,state=self.state))
            return None,('B DONE. RETURN BOARD TO A OUTLINE; KEEP CAMERAS FIXED.' if self.state=='WAIT_C'
                         else 'A DONE. MOVE BOARD ONLY; KEEP CAMERAS FIXED; ALL36 VISIBLE.')
        if points is None:
            self.history=[]
            return None,self.state+': need all36 tags in EACH view and fresh frames'
        points=np.asarray(points,dtype=float)
        if points.shape!=(3,144,2) or not np.isfinite(points).all():raise ValueError('invalid gate points')
        if self.tilt_contrast:
            if normals is None:
                self.history=[]
                return None,self.state+': waiting for board orientation in all views'
            normals=np.asarray(normals,dtype=float)
            if normals.shape!=(3,3) or not np.isfinite(normals).all() or not np.allclose(np.linalg.norm(normals,axis=1),1.):
                raise ValueError('invalid board normals')
            if self.state=='WAIT_B':
                self.tilt_degrees=np.degrees(np.arccos(np.clip(np.sum(normals*self.reference_normals,axis=1),-1,1)))
                if self.tilt_degrees.min()<12.:
                    self.history=[]
                    return None,'TILT BOARD ONLY: angles '+ '/'.join(f'{a:.1f}' for a in self.tilt_degrees)+' deg; EACH >=12deg'
        if self.state=='WAIT_B' and not self.tilt_contrast and min(np.median(np.linalg.norm(points-self.reference,axis=2),axis=1))<20:
            self.history=[]
            return None,'WAIT_B: change board pose; >20px median change in EACH view'
        if self.state=='WAIT_C':
            distance=np.linalg.norm(points-self.reference,axis=2)
            if np.median(distance,axis=1).max()>5 or distance.max()>10:
                self.history=[]
                return None,f'RETURN A: match outline; median {np.median(distance,axis=1).max():.1f}/5px max {distance.max():.1f}/10px'
        self.history.append((now,points.copy()))
        self.history=[x for x in self.history if now-x[0]<=2.]
        if len(self.history)<4 or now-self.history[0][0]<1.5:
            return None,self.state+': all36 visible; hold still 1.5s'
        drift=max(float(np.linalg.norm(x[1]-points,axis=2).max()) for x in self.history)
        if drift>.75:
            return None,f'{self.state}: hold still; drift {drift:.2f}/0.75px'
        self.state='RECORD_'+self.state[-1];self.record_start=now
        if self.reference is None:
            self.reference=points.copy()
            if self.tilt_contrast:self.reference_normals=normals.copy()
        self.events.append(dict(time=now,state=self.state))
        return self.state[-1],self.state+': RECORDING 10s, DO NOT MOVE'


class BoardObserver:
    def __init__(self):
        self.detector=make_board_detector('umi-aprilgrid')
        self.mount=cb.mount_detector()

    def observe(self,arrays):
        points={};details={}
        for role,image in arrays.items():
            gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
            raw=cb.detect_grid(self.detector,gray)
            details[role]=dict(raw_count=len(raw),ids=[],failure=None)
            try:
                if role=='umi_color':ds,_,_=board_geometry(raw)
                else:
                    ds,_,proof=diagnostic_targets(raw,cb.detect_grid(self.mount,gray))
                    details[role]['mount_min_edge_px']=proof['mount_min_edge_px']
                    details[role]['mount_size_pass']=proof['mount_size_pass']
                ds[1],_=restore_board_id1(raw)
                details[role]['ids']=sorted(ds)
                if set(ds)!=set(range(36)):raise ValueError('need all36')
                ds=cb.refine_legacy_corners(gray,ds)
                points[role]=np.concatenate([ds[i] for i in range(36)])
            except ValueError as exc:details[role]['failure']=str(exc)
        names=('ir_left','ir_right','umi_color')
        return (np.array([points[k] for k in names]) if set(points)==set(names) else None),details
