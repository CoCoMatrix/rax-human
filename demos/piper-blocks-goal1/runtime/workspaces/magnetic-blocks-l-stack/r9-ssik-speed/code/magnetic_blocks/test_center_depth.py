import copy
import numpy as np
import pytest
from perception import measure_block, measure_center_surface, PerceptionUnknown, CenterDepthInconsistent
from scene_checks import _recover_center_height

def fixture(z=(.080,.081,.079,.080,.081)):
 obs={'frame_id':'a'*32,'calibration_sha256':'b'*64}
 sel={'frame_id':obs['frame_id'],'color':'red','pixels':[[50,50],[60,50],[70,50],[50,60],[60,60]],'bbox_xywh':[25,25,80,80]}
 dep={'frame_id':obs['frame_id'],'camera':'head','frame':'left_base_link','base_transform_applied':True,'robot_target_usable':True,'calibration_provisional':True,'calibration_sha256':obs['calibration_sha256'],'points':[{'pixel':p,'position':[.3,.1,h],'depth_m':.6,'valid_samples':25} for p,h in zip(sel['pixels'],z)]}
 return obs,sel,dep

def test_normal_unchanged():
 o,s,d=fixture();p=measure_block(o,s,d)
 assert measure_center_surface(o,s,d,p)==p

@pytest.mark.parametrize('bad',[.103,.04])
def test_one_outlier_uses_majority(bad):
 o,s,d=fixture();p=measure_block(o,s,d);d['points'][1]['position'][2]=bad
 v=measure_center_surface(o,s,d,p)
 assert v['depth_consensus']['accepted_center_points']==4
 assert abs(v['surface_position_m'][2]-.080)<.001

@pytest.mark.parametrize('kind',['frame','calibration','invalid_sample','nonfinite','pixel','patch_identity'])
def test_identity_and_invalid_evidence_still_refused(kind):
 o,s,d=fixture();p=measure_block(o,s,d)
 if kind=='frame':d['frame_id']='c'*32
 if kind=='calibration':d['calibration_sha256']='c'*64
 if kind=='invalid_sample':d['points'][0]['valid_samples']=0
 if kind=='nonfinite':d['points'][0]['position'][2]=float('nan')
 if kind=='pixel':d['points'][0]['pixel']=[99,99]
 if kind=='patch_identity':p['frame_id']='c'*32
 with pytest.raises(PerceptionUnknown) as e:measure_center_surface(o,s,d,p)
 assert not isinstance(e.value,CenterDepthInconsistent)

@pytest.mark.parametrize('z',[(.1,.1,.1,.08,.08),(.1,.1,.1,.1,.1)])
def test_no_majority_not_accepted(z):
 o,s,d=fixture();p=measure_block(o,s,d);_,_,bad=fixture(z)
 with pytest.raises(CenterDepthInconsistent):measure_center_surface(o,s,bad,p)

class NoCalls:
 def value(self,*a,**kw):raise AssertionError('No extra query needed')

def test_good_top_patch_reused_without_rpc():
 o,s,d=fixture();p=measure_block(o,s,d)
 out,dep,pixels=_recover_center_height(NoCalls(),o,s,p,d,None,None,None,'center outliers')
 assert out['surface_position_m']==p['surface_position_m']
 assert out['depth_consensus']['reselected'] is False
 assert dep==d

@pytest.mark.parametrize('flat',[True,False])
def test_reselection_bounded_and_revalidated(flat):
 o,s,d=fixture((.08,.09,.08,.08,.08));p=measure_block(o,s,d)
 xyz=np.zeros((720,1280,3));xyz[:,:,2]=.08
 valid=np.zeros((720,1280),bool);valid[25:105,25:105]=True
 rgb=np.zeros((720,1280,3),np.uint8);rgb[25:105,25:105]=[0,0,200]
 class C:
  calls=0
  def value(self,tool,**kw):
   self.calls+=1;assert tool=='piper_pixel_to_position'
   v=copy.deepcopy(d);v['points']=[{'pixel':px,'position':[.3,.1,.08 if flat or i!=0 else .09],'depth_m':.6,'valid_samples':25} for i,px in enumerate(kw['pixels'])];return v
 c=C()
 if flat:
  out,_,_=_recover_center_height(c,o,s,p,d,xyz,valid,rgb,'center outliers')
  assert out['depth_consensus']['reselected']
 else:
  with pytest.raises(PerceptionUnknown):_recover_center_height(c,o,s,p,d,xyz,valid,rgb,'center outliers')
 assert c.calls==1
