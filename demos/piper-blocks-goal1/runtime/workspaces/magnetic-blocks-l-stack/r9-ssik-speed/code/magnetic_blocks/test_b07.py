import copy,json,math
from pathlib import Path
import pytest
import numpy as np
from numeric_policy import *
import assembly_plan as ap

@pytest.mark.parametrize('nominal,minimum',[(.003,.0015),(.004,.0025),(.020,.0185),(.001,.001)])
def test_clearance_allowance_is_bounded(nominal,minimum):
 assert clearance_minimum(nominal)==pytest.approx(minimum)

def test_qualitative_orientation_adds_one_degree_not_arbitrary_tilt():
 assert math.acos(DOWN_COS)-math.acos(.995)==pytest.approx(math.radians(1))
 assert -math.cos(math.radians(6))<=-DOWN_COS
 assert -math.cos(math.radians(10))>-DOWN_COS

def test_boxes_declare_only_task_object_finger_tolerance():
 from test_composition import SceneContext
 boxes=ap.boxes(ap.initial_scene(SceneContext().scene))
 assert 'finger_clearance_tolerance_m' not in boxes[0]
 assert all(b['finger_clearance_tolerance_m']==.0025 for b in boxes[1:])

@pytest.fixture(scope='module')
def checker():
 from services.piper.installed_collision import load_checker
 return load_checker('/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025')

@pytest.fixture
def recorded():
 return json.loads((Path(__file__).parent.parent/'RETREAT-ARGS.json').read_text())

def test_current_retreat_passes_every_sample_and_reports_allowance(checker,recorded):
 r=checker.path([recorded['start14'],recorded['target14']],obstacles=recorded['obstacles'])
 assert r['cad_clear'] and r['whole_path_checked'] and r['checked_samples']==52
 assert min(x['distance_m'] for x in r['tolerated_clearances'])==pytest.approx(.002796455142070836)

def test_without_explicit_object_annotation_original_rule_remains(checker,recorded):
 for b in recorded['obstacles']:b.pop('finger_clearance_tolerance_m',None)
 r=checker.path([recorded['start14'],recorded['target14']],obstacles=recorded['obstacles'])
 assert not r['cad_clear'] and r['first_failing_sample']==17
 assert r['contacts'][0]['distance_m']==pytest.approx(.0029777773205233123)

@pytest.mark.parametrize('shrink',[.004,.010])
def test_sub_minimum_gap_and_collision_still_refused(checker,recorded,shrink):
 a,b=recorded['start14'],recorded['target14'];a[6]-=shrink;b[6]-=shrink
 r=checker.path([a,b],obstacles=recorded['obstacles'])
 assert not r['cad_clear']
 assert any(c['distance_m']<.0025 for c in r['contacts'])

@pytest.mark.parametrize('value',[True,-.001,.00250001,float('nan'),float('inf')])
def test_no_excessive_or_invalid_native_tolerance(checker,recorded,value):
 recorded['obstacles'][1]['finger_clearance_tolerance_m']=value
 with pytest.raises(ValueError,match='tolerance'):checker.check(recorded['start14'],obstacles=recorded['obstacles'])

def test_self_right_and_arm_clearance_not_relaxed(checker,recorded):
 from unittest.mock import patch
 import services.piper.installed_collision_math as m
 # Recompute static pairs to ensure right/self checks actually run.
 checker._right_pair_cache=None
 with patch.object(m.fcl,'distance',return_value=.00275):r=checker.check(recorded['start14'],obstacles=recorded['obstacles'])
 assert any(not any(x.startswith('environment:') for x in c['links']) for c in r['contacts'])
 assert all(c['links'][0] in ('left_gripper_link1','left_gripper_link2') and c['links'][1] in ('environment:red','environment:orange','environment:green') for c in r['tolerated_clearances'])
 checker._right_pair_cache=None

def test_geometry_refusal_is_not_reported_as_missing_images():
 import controller as c
 def op(report):raise ap.NoPlan('Native CAD path refused')
 from types import SimpleNamespace
 r=c._run(SimpleNamespace(),'retreat',op)
 assert r['failure_category']=='planning_or_geometry' and r['why'].startswith('Planning/geometry refusal:')
