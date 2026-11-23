"""Recorded yellow release regression; no hardware transport."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from native_fk.geometry import Kinematics
from assembly_plan import orientation_evidence,NoPlan

def pose(tilt):
 t=np.eye(4);t[:3,:3]=Rotation.from_euler('x',180-tilt,degrees=True).as_matrix();return t

def test_recorded_release_arrival_error_uses_same_bound_with_or_without_reference():
 target=[-.4237328767776489,1.397109031677246,-1.2669427394866943,1.1257941722869873,-.1746528595685959,-.38806673884391785]
 actual=[-.42465606030274033,1.3965775608608226,-1.2681562344990798,1.1245854502300263,-.17379988691359535,-.38746309394274114]
 planned,measured=Kinematics().fk_batch([target,actual])
 plain=orientation_evidence(measured,'yellow')
 fact=orientation_evidence(measured,'yellow',planned)
 assert fact==plain and not fact['reference_used']
 assert fact['measured_tilt_deg']==pytest.approx(25.13602817281586)
 assert fact['accepted_limit_deg']==pytest.approx(25.6)

@pytest.mark.parametrize('color',['red','orange','green','yellow'])
@pytest.mark.parametrize('sign',[-1,1])
def test_shared_half_degree_bound_is_symmetric_and_not_overridden_by_reference(color,sign):
 limit=orientation_evidence(pose(0),color)['planning_limit_deg']
 for offset in (.2,.5):
  plain=orientation_evidence(pose(sign*(limit+offset)),color)
  assert plain==orientation_evidence(pose(sign*(limit+offset)),color,pose(sign*(limit-.1)))
 for reference in (None,pose(sign*(limit-.1)),pose(sign*(limit+.1))):
  with pytest.raises(NoPlan):orientation_evidence(pose(sign*(limit+.5001)),color,reference)

def test_recorded_yellow_pickup_matches_closed_jaw_acceptance():
 q=[-.5575628828421085,2.06212396452382,-1.6933533468699384,1.0447540902438057,.36777577998024513,.9674534576729769]
 measured=Kinematics().fk(q)
 fact=orientation_evidence(measured,'yellow')
 assert fact['measured_tilt_deg']==pytest.approx(25.12178214486539)
 assert fact==orientation_evidence(measured,'yellow',pose(25.))

@pytest.mark.parametrize('bad',[np.full((4,4),np.nan),np.eye(3)])
def test_invalid_transform_cannot_pass_orientation_comparison(bad):
 with pytest.raises(NoPlan):orientation_evidence(bad,'yellow')
