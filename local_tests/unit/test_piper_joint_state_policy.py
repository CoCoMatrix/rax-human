"""Operator .01-radian policy: all joints, never millimetres or stale feedback."""
import json
from pathlib import Path
import numpy as np
import pytest
from services.piper.installed_settling import JOINTS, RestWindow, state_matches, settling_valid

@pytest.mark.parametrize('index',JOINTS)
@pytest.mark.parametrize('delta,ok',[(.0013264502315,True),(.006597344572538566,True),(.01,True),(.01001,False)])
def test_state_limit_per_joint(index,delta,ok):
    a=[0.]*14;b=a.copy();b[index]=delta
    assert state_matches(a,b) is ok
    assert b[index]==delta  # No zero filtering or in-place normalization.

@pytest.mark.parametrize('index',[6,13])
def test_jaws_do_not_inherit_radians(index):
    a=[0.]*14;b=a.copy();b[index]=.0005
    assert state_matches(a,b)
    b[index]=.000501
    assert not state_matches(a,b)

def test_recorded_j1_alternation_completes_current_point_three_second_rest():
    w=RestWindow();complete=False
    for i in range(11):
        q=[0.]*14;q[0]=-.0013264502315156904 if i%2 else 0.
        complete=w.add(q,1_000_000_000+i*100_000_000,1_000_000_000+i*100_000_000)
        if i<3:assert not complete
        else:assert complete
    assert complete and settling_valid(w.evidence())
    assert w.evidence()['max_joint_range_rad']>.0013

def test_new_mixed_unit_evidence_not_misread_and_excess_refused():
    e={'max_range':.005,'max_joint_range_rad':.005,'max_jaw_range_m':.0002,
       'joint_range_limit_rad':.01,'jaw_range_limit_m':.0003}
    assert settling_valid(e)
    assert not settling_valid({**e,'max_jaw_range_m':.000301})
    assert not settling_valid({**e,'max_joint_range_rad':.01001})
    assert not settling_valid({'max_range':.005})

@pytest.mark.parametrize('index,delta',[(0,.011),(6,.00031),(13,.00031)])
def test_outside_window_cannot_claim_rest(index,delta):
    w=RestWindow()
    for i in range(25):
        q=[0.]*14;q[index]=delta if i%2 else 0.
        assert not w.add(q,1_000_000_000+i*100_000_000,1_000_000_000+i*100_000_000)
