"""Phase-aware placement evidence; physical receipt checks remain mandatory."""
from unittest.mock import patch
import pytest
import ordinary_checks as o
from test_b02_flexibility import release_fixture, fkctx
from scene_checks import PerceptionUnknown


def scene(color):
 return {'blocks':{color:{'top_center':{'surface_position_m':[.3,0,.06]}},
   **({} if color=='red' else {'red':{'top_center':{'surface_position_m':[.5,.2,.06]}}})},
   'table':{'z_ax_by_c':[0,0,0]}}

@pytest.mark.parametrize('color',['orange','red','green'])
def test_first_or_last_placed_body_does_not_require_final_neighbor(color):
 ctx,obs,release,retreat=release_fixture()
 with patch.object(o,'visible_near_point',return_value={'visible':False}),patch.object(o,'wrist_body',return_value={'visible':True}),patch.object(o,'wrist_pair',side_effect=AssertionError('No repeated relation check')),patch('history_scene.reject_visible_displacement',return_value={}):
  assert o.released_endpoint(ctx,obs,scene(color),color,release,retreat)['ok']

@pytest.mark.parametrize('color',['orange','red'])
def test_withdrawn_recovery_accepts_wrist_with_verified_release_pose(color):
 origin={'color':color,'grasp_flange':[.3,0,.188],'flange_to_bottom_m':.188}
 with patch.object(o,'visible_near_point',return_value={'visible':False}),patch.object(o,'wrist_body',return_value={'visible':True}),patch.object(o,'wrist_pair',side_effect=AssertionError('No final neighbor needed')),patch('history_scene.reject_visible_displacement',return_value={}):
  r=o.placement(fkctx(),'release',{'frame_id':'test'},scene(color),origin,placed_pose=[.3,0,.188],withdrawn=True)
  assert r['ok']

@pytest.mark.parametrize('fault',['support','owner','closed','incomplete','no_retreat','binding','blank'])
def test_visible_color_cannot_replace_receipt_or_identity(fault):
 ctx,obs,release,retreat=release_fixture()
 if fault=='support':release['support']['ok']=False
 if fault=='owner':retreat['owner_generation']=4
 if fault=='closed':obs['measured14'][6]=.05
 if fault=='incomplete':retreat['measured_arrival']=False
 if fault=='no_retreat':obs['measured14'][2]=0
 with patch.object(o,'visible_near_point',return_value={'visible':False}),patch.object(o,'wrist_body',side_effect=PerceptionUnknown('bad binding') if fault=='binding' else None,return_value={'visible':fault!='blank'}),patch.object(o,'wrist_pair',return_value={'ok':None}),pytest.raises(PerceptionUnknown):
  o.released_endpoint(ctx,obs,scene('orange'),'orange',release,retreat)
