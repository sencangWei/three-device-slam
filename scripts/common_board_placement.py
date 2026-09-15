"""Placement feasibility only; never calibrates or certifies camera immobility."""
import numpy as np


class PlacementCheck:
    def __init__(self):
        self.step = 0
        self.mount = None

    @property
    def kind(self):
        return ('mount','board','mount')[self.step]

    @property
    def instruction(self):
        return (
            '摆位1/3：先移走公共板，Ego完整看UMI背码。现在可调整相机；READY后按R锁定摆位。',
            '摆位2/3：两台相机都不准动，只放入公共板让两机共视；可遮住背码避免ID1冲突。READY后按R。',
            '摆位3/3：相机仍不动，只移走公共板、露出背码。若必须转Ego才看见，本摆位不可用；Q退出重新摆。'
        )[self.step]

    def advance(self, quality, detected):
        if quality['reasons']:
            return False, '本项预检未通过，不能前进'
        if self.step in (0,2):
            quad=np.asarray(detected['ego'][1],dtype=float)
            if self.step==0:
                self.mount=quad.copy()
            elif np.max(np.linalg.norm(quad-self.mount,axis=1))>.75:
                return False, '背码相对初始图像变化超过0.75px；不能证明摆位保持，请Q退出重新摆位，不要转相机补看'
        self.step+=1
        return True, '摆位检查通过；仅可见性/静态代理，不是空间标定通过' if self.step==3 else self.instruction
