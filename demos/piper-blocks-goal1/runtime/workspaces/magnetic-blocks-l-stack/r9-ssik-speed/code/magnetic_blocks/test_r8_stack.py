"""Synthetic failure cases, never hardware qualification."""
import copy
import unittest
from unittest.mock import patch
import numpy as np
import controller
from scene_checks import PerceptionUnknown
from stack_checks import check_stack_geometry, stack_target
from test_workflow import Fake, evidence

class Geometry:
    def __init__(self):self.block=False
    def value(self,tool,**kw):
        if tool=='piper_fk':
            p=kw['joints'][:3];T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=p
            return dict(position=p,T_base_flange=T.tolist(),quaternion_xyzw=[1,0,0,0])
        if tool=='piper_solve_ik':return {'q':kw['position']+[0,0,0]}
        if tool=='piper_check_path':return {'cad_clear':not self.block,'workspace_clear':True}
        raise AssertionError('No motion in geometry test: '+tool)

def fixture():
    scene={'blocks':{c:{'top_center':{'surface_position_m':p}} for c,p in
        [('red',[.3,0,.08]),('orange',[.3,-.0675,.08]),('green',[.3,-.28,.08])]},
        'table':{'z_ax_by_c':[0,0,.02]}}
    origin={'grasp_flange':[.3,-.28,.205],'flange_to_bottom_m':.185,'color':'green'}
    start=[.3,.003,.3,0,0,0,.051]+[0]*7
    fingers=[]
    for x in [.26,.34]:
        corners=[[x+dx,.003+dy,z] for dx in [-.01,.01] for dy in [-.025,.025] for z in [.13,.22]]
        fingers.append({'cad_corners_base_m':corners})
    obs={'measured14':start,'finger_geometry':{'fingers':fingers}}
    return scene,origin,obs

class Stack(unittest.TestCase):
    def test_supported_path_has_finger_and_neighbor_clearance(self):
        s,o,b=fixture();r=check_stack_geometry(Geometry(),b,s,o,[.3,.003,.265],[1,0,0,0],final=True)
        self.assertGreater(r['min_finger_base_top_m'],.010)
        self.assertGreater(r['min_green_orange_separation_m'],.001)
    def test_neighbor_overlap_refused(self):
        s,o,b=fixture();s['blocks']['orange']['top_center']['surface_position_m'][1]=-.060
        with self.assertRaises(PerceptionUnknown):check_stack_geometry(Geometry(),b,s,o,[.3,.003,.265],[1,0,0,0],final=True)
    def test_offcenter_and_below_support_refused(self):
        for p in ([.3,.010,.265],[.3,.003,.263]):
            s,o,b=fixture()
            with self.assertRaises(PerceptionUnknown):check_stack_geometry(Geometry(),b,s,o,p,[1,0,0,0],final=True)
    def test_lost_loaded_gap_and_native_collision_refused(self):
        s,o,b=fixture();b['measured14'][6]=.069
        with self.assertRaises(PerceptionUnknown):check_stack_geometry(Geometry(),b,s,o,[.3,.003,.265],[1,0,0,0],final=True)
        s,o,b=fixture();g=Geometry();g.block=True
        with self.assertRaises(PerceptionUnknown):check_stack_geometry(g,b,s,o,[.3,.003,.265],[1,0,0,0],final=True)
    def test_target_preserves_measured_grasp_offset(self):
        s,o,b=fixture();o['grasp_flange'][0]+=.004
        t=stack_target(s,o);self.assertAlmostEqual(t[0],.304);self.assertAlmostEqual(t[1],.003);self.assertAlmostEqual(t[2],.265)
    def test_unknown_support_cannot_open(self):
        f=Fake();f.q[6]=.051
        with patch('stack_checks.loaded_state',side_effect=PerceptionUnknown('support not established')):
            r=controller.green_stack_release(f,previous_report='unused',support_report='unused')
        self.assertFalse(r['ok']);self.assertFalse(r['executed']);self.assertFalse(any(t=='piper_gripper' for t,a in f.calls))
    def test_negative_fact_preserves_assessment_without_large_error_string(self):
        f=Fake();f.q[6]=.051
        def no(**kw):return {'ok':False,'frame_id':kw['after']['frame_id'],'why':'not supported','diagnostic':'x'*10000}
        r=controller.release(f,assess=no)
        self.assertFalse(r['executed']);self.assertLess(len(r['why']),200)
        self.assertEqual(r['review']['assessment']['why'],'not supported')
        self.assertEqual(r['status'],'rejected_scene_fact')
    def test_retreat_rejects_unexecuted_release(self):
        f=Fake();f.q[6]=.051
        original=f.value
        def value(tool,**kw):
            if tool=='piper_status':return {}
            return original(tool,**kw)
        f.value=value
        with patch('manipulation_checks.read_report',return_value={'result':{}}), patch('manipulation_checks.completed_action_state',side_effect=PerceptionUnknown('No completed release')):
            r=controller.green_stack_retreat(f,waypoints=[[.3,0,.3]],quaternion=[1,0,0,0],previous_report='unused')
        self.assertFalse(r['ok']);self.assertFalse(r['executed'])
        self.assertFalse(any(t in ('piper_gripper','piper_move_pose','piper_move_joints') for t,a in f.calls))
    def test_startup_transition_requests_metric_depth(self):
        f=Fake();f.q[6]=.069
        r=controller.empty_joint_waypoint(f,joints=[.1]*6,assess=evidence)
        self.assertTrue(r['ok']);self.assertTrue(next(a['require_depth'] for t,a in f.calls if t=='piper_observe'))
if __name__=='__main__':unittest.main()
