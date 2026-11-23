"""Offline contract checks. Fake contexts cannot connect to the robot."""
import unittest
import controller as c


class Fake:
    def __init__(self, fail=None):
        self.calls=[]; self.q=[0.0]*14; self.q[6]=.069; self.pos=[.314,-.274,.205]
        self.n=0; self.fail=fail
    def obs(self):
        self.n+=1
        return {'frame_id':str(self.n),'measured14':list(self.q)}
    def value(self, tool, **kw):
        self.calls.append((tool,kw))
        if tool==self.fail:
            raise RuntimeError('injected failure after possible dispatch')
        if tool=='status':
            return {'control_uncertain':False,'robot_status':{
                'controlled_link':'left_link6','frame':'left_base_link',
                'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36]}}
        if tool=='piper_observe':return self.obs()
        if tool=='piper_fk':return {'position':list(self.pos)}
        if tool=='piper_hold':return {'held':True}
        if tool=='piper_gripper':self.q[6]=kw['width_m']
        elif tool=='piper_move_pose':self.pos=kw['position']
        else:raise AssertionError(tool)
        return {'executed':True,'measured_arrival':True,'measured_settled':True,
                'observation':self.obs()}


def review(**kw):
    return {'ok':True,'frame_id':kw['after']['frame_id'],'why':'synthetic offline fixture only'}


class Tests(unittest.TestCase):
    def test_grasp_exact_50_and_10mm_lift(self):
        f=Fake(); r=c.grasp_and_trial_lift(f,quaternion=[1,0,0,0],assess=review)
        self.assertTrue(r['ok']); self.assertAlmostEqual(f.q[6],.050)
        self.assertAlmostEqual(f.pos[2],.215)
        cmds=[a['width_m'] for t,a in f.calls if t=='piper_gripper']
        self.assertAlmostEqual(cmds[-1],.050)
        for a,b in zip([.069]+cmds,cmds):self.assertLessEqual(abs(a-b),.010)
    def test_target_command_not_skipped_by_feedback_tolerance(self):
        f=Fake();f.q[6]=.051
        r=c.grasp_and_trial_lift(f,quaternion=[1,0,0,0],assess=review)
        self.assertTrue(r['ok'])
        self.assertAlmostEqual(next(a['width_m'] for t,a in f.calls if t=='piper_gripper'),.050)
    def test_retention_failure_holds_without_reopening(self):
        def assess(**kw):return {'ok':False} if kw['phase']=='retained_after_trial' else review(**kw)
        f=Fake();r=c.grasp_and_trial_lift(f,quaternion=[1,0,0,0],assess=assess)
        self.assertFalse(r['ok']);self.assertEqual(f.calls[-1][0],'piper_hold')
        self.assertAlmostEqual(f.q[6],.050)
    def test_ambiguous_motion_error_holds_and_never_retries(self):
        f=Fake(fail='piper_move_pose')
        r=c.transport(f,waypoints=[[.31,-.20,.28],[.31,-.15,.28]],quaternion=[1,0,0,0],assess=review)
        self.assertFalse(r['ok']);self.assertTrue(r['dispatch_attempted'])
        self.assertEqual(sum(t=='piper_move_pose' for t,a in f.calls),1)
        self.assertEqual(f.calls[-1][0],'piper_hold')
    def test_old_frame_cannot_authorize_motion(self):
        f=Fake();r=c.approach(f,waypoints=[[.31,-.2,.28]],quaternion=[1,0,0,0],
            assess=lambda **kw:{'ok':True,'frame_id':'old','why':'stale'})
        self.assertFalse(r['ok']);self.assertFalse(r['dispatch_attempted'])
    def test_unconfirmed_support_never_opens(self):
        def assess(**kw):return {'ok':False} if kw['phase']=='supported_before_release' else review(**kw)
        f=Fake();f.q[6]=.051
        r=c.place_release_retreat(f,descent=[[.304,-.049,.265]],retreat=[[.304,-.049,.3]],quaternion=[1,0,0,0],assess=assess)
        self.assertFalse(r['ok']);self.assertFalse(any(t=='piper_gripper' for t,a in f.calls))
    def test_bad_retreat_rejected_before_any_motion(self):
        f=Fake();r=c.place_release_retreat(f,descent=[[.304,-.049,.265]],retreat=[[.304,-.049,.5]],quaternion=[1,0,0,0],assess=review)
        self.assertFalse(r['ok']);self.assertFalse(r['dispatch_attempted'])
    def test_successful_release_and_known_withdrawal(self):
        f=Fake();f.q[6]=.051
        r=c.place_release_retreat(f,descent=[[.304,-.049,.265]],retreat=[[.304,-.049,.3]],quaternion=[1,0,0,0],assess=review)
        self.assertTrue(r['ok']);self.assertAlmostEqual(f.q[6],.069);self.assertAlmostEqual(f.pos[2],.3)
    def test_uncertain_control_does_not_dispatch(self):
        f=Fake();original=f.value
        f.value=lambda tool,**kw: {'control_uncertain':True} if tool=='status' else original(tool,**kw)
        r=c.open_empty_gripper(f,assess=review)
        self.assertFalse(r['ok']);self.assertFalse(r['dispatch_attempted'])
    def test_unknown_motion_receipt_is_not_success(self):
        f=Fake();original=f.value
        f.value=lambda tool,**kw: {'executed':True,'measured_arrival':False} if tool=='piper_move_pose' else original(tool,**kw)
        r=c.transport(f,waypoints=[[.31,-.20,.28]],quaternion=[1,0,0,0],assess=review)
        self.assertFalse(r['ok']);self.assertEqual(f.calls[-1][0],'piper_hold')


if __name__=='__main__':unittest.main()
