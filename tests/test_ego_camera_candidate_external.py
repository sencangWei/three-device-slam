import pytest
from scripts.validate_ego_camera_candidate_external import frozen_candidate


def test_multidepth_candidate_explicit_model_and_serial_binding():
    prior=dict(status='DEVELOPMENT_ONLY_NOT_ACCEPTANCE',activation='NOT_ACTIVATED',
        training_sessions=[1,2,4,5],candidate=dict(K=[[1]],D=[.1,0,0,0,0],model='opencv_brown_conrady'),
        camera_serial='ego',factory_color_intrinsics={'fx':1})
    c,devices,label=frozen_candidate(prior)
    assert label=='multidepth_K4_k1' and c['distortion_model']=='opencv_brown_conrady'
    assert devices=={'ego':{'serial':'ego','intrinsics':{'fx':1}}}
    for changed in (dict(prior,status='PASS'),dict(prior,activation='ACTIVE')):
        with pytest.raises(ValueError):frozen_candidate(changed)
