"""Offline checks only: these fixtures are never installed on hardware."""
import copy
import json
from pathlib import Path
import unittest
import cv2
import numpy as np
import controller
import perception as p


class Perception(unittest.TestCase):
    def test_color_components_and_false_strip(self):
        for color,bgr in [('red',(0,0,255)),('orange',(0,140,255)),('green',(0,190,0))]:
            im=np.zeros((720,1280,3),dtype=np.uint8)
            cv2.rectangle(im,(200,200),(290,290),bgr,-1)
            cv2.rectangle(im,(400,550),(900,580),bgr,-1)
            selected,_=p.component(im,color)
            self.assertEqual(selected['bbox_xywh'],[200,200,91,91])
            cv2.rectangle(im,(400,200),(490,290),bgr,-1)
            with self.assertRaises(p.PerceptionUnknown):p.component(im,color)

    def test_missing_color_and_wrong_dimensions_fail(self):
        with self.assertRaises(p.PerceptionUnknown):p.component(np.zeros((720,1280,3),np.uint8),'red')
        with self.assertRaises(p.PerceptionUnknown):p.component(np.zeros((360,640,3),np.uint8),'green')

    def test_budget_boundaries_and_missing_budget(self):
        for seconds,reserve in [(60,60),(1000,1000),(float('nan'),60),(1000,None)]:
            with self.assertRaises(RuntimeError):
                controller._budget({'wall_budget':{'remaining_seconds':seconds,'finish_reserve_seconds':reserve,'phase':'active'}})
        controller._budget({'wall_budget':{'remaining_seconds':1000,'finish_reserve_seconds':60,'phase':'active'}})
        with self.assertRaises(RuntimeError):controller._budget({})

    def test_depth_rejects_mismatches_invalid_and_spread(self):
        obs={'frame_id':'abc','calibration_sha256':'a'*64}
        selected={'frame_id':'abc','color':'red','pixels':[[10,10],[16,10],[10,16]]}
        good={'frame_id':'abc','calibration_sha256':'a'*64,'camera':'head','frame':'left_base_link',
              'base_transform_applied':True,'calibration_provisional':True,'robot_target_usable':True,
              'points':[{'pixel':px,'position':[.3,-.1,.08],'valid_samples':25,'depth_m':.6} for px in selected['pixels']]}
        r=p.measure_block(obs,selected,good);self.assertFalse(r['motion_permit'])
        for key,value in [('frame_id','other'),('calibration_sha256','other'),('frame','camera_optical'),('points',[])]:
            bad=copy.deepcopy(good);bad[key]=value
            with self.assertRaises(p.PerceptionUnknown):p.measure_block(obs,selected,bad)
        for position in [[.3,-.1,.1],[.3,-.1,float('nan')]]:
            bad=copy.deepcopy(good);bad['points'][0]['position']=position
            with self.assertRaises(p.PerceptionUnknown):p.measure_block(obs,selected,bad)

    def test_recorded_rgb_and_stale_frame_gate(self):
        # Historical evidence used offline, never injected into live execution.
        frame=Path('runtime/frames/3416ddeb272944778da7dfa780d52dcc/meta.json')
        self.assertTrue(frame.is_file(),'Run from workspace root; historical evidence required')
        obs=json.loads(frame.read_text())
        for color in ['red','orange','green']:
            selected=p.select_block(obs,color,now_ns=obs['captured_at_ns'])
            self.assertEqual(len(selected['pixels']),5)
            with self.assertRaises(p.PerceptionUnknown):
                p.select_block(obs,color,now_ns=obs['captured_at_ns']+31_000_000_000)
        bad=copy.deepcopy(obs);bad['cameras']['head']['binding']='different packet'
        with self.assertRaises(p.PerceptionUnknown):p.select_block(bad,'red',now_ns=obs['captured_at_ns'])
        with self.assertRaises(p.PerceptionUnknown):
            p.select_block(obs,'red',interior_pixels=[[1,1]]*3,now_ns=obs['captured_at_ns'])

    def test_three_live_reports_reproduce_offline(self):
        for color in ['red','orange','green']:
            path=Path(f'references/rehearsal-r3/r3-locate-{color}-01.json')
            record=json.loads(path.read_text());result=record['result']
            obs=result['measured']
            selected=p.select_block(obs,color,now_ns=obs['captured_at_ns'])
            self.assertEqual(selected,result['selection'])
            self.assertEqual(p.measure_block(obs,selected,result['depth']),result['localization'])
            self.assertEqual(record['calls'],3)
            self.assertFalse(result['dispatch_attempted'])

    def test_missing_calibration_and_malformed_depth_fail_closed(self):
        result=json.loads(Path('references/rehearsal-r3/r3-locate-red-01.json').read_text())['result']
        obs,selected,depth=(result[k] for k in ['measured','selection','depth'])
        no_obs=copy.deepcopy(obs);no_depth=copy.deepcopy(depth)
        no_obs.pop('calibration_sha256');no_depth.pop('calibration_sha256')
        with self.assertRaises(p.PerceptionUnknown):p.measure_block(no_obs,selected,no_depth)
        for key,value in [('points',[None]*5),('robot_target_usable',None),('calibration_provisional',None)]:
            bad=copy.deepcopy(depth);bad[key]=value
            with self.assertRaises(p.PerceptionUnknown):p.measure_block(obs,selected,bad)
        for key,value in [('valid_samples',True),('valid_samples','25'),('depth_m',True),
                          ('depth_m',0),('position',[True,0,.08])]:
            bad=copy.deepcopy(depth);bad['points'][0][key]=value
            with self.assertRaises(p.PerceptionUnknown):p.measure_block(obs,selected,bad)
        for bad in [None,[],{},dict(obs,cameras=None),dict(obs,captured_at_ns=True)]:
            with self.assertRaises(p.PerceptionUnknown):p.select_block(bad,'red')

    def test_missing_facts_return_needs_review(self):
        from test_workflow import Fake
        for bad in [{},{'cameras':None}]:
            fake=Fake();result=controller.locate_block(fake,observation=bad,color='red')
            self.assertTrue(result['needs_review']);self.assertFalse(result['dispatch_attempted'])
            self.assertEqual([name for name,args in fake.calls],['status'])

    def test_boolean_budgets_rejected(self):
        for seconds,reserve in [(True,60),(1000,True)]:
            with self.assertRaises(RuntimeError):
                controller._budget({'wall_budget':{'remaining_seconds':seconds,'finish_reserve_seconds':reserve,'phase':'active'}})


if __name__=='__main__':unittest.main()
