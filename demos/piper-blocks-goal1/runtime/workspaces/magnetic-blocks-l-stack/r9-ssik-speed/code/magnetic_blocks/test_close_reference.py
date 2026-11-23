"""Exercise the real closing-alignment entry, including its reference branch."""
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
import qualitative as q
from scene_checks import PerceptionUnknown

@pytest.mark.parametrize('color',['red','orange','green','yellow'])
def test_closing_reference_accepts_measured_open_jaw_without_model(color):
 observed=[0.]*14;observed[6]=.0725
 nominal=observed.copy();nominal[6]=.073
 scene={'blocks':{color:{'top_center':{'surface_position_m':[.3,0,.06]}}}}
 T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=[.3,0,.188]
 def value(name,**kw):
  assert name=='piper_fk','No motion/model permitted'
  return {'T_base_flange':T.tolist()}
 ctx=SimpleNamespace(value=value)
 with patch('scene_checks.scene_geometry'),patch('ordinary_checks.visible_near_point',return_value={'visible':True}):
  result=q.alignment_evidence(ctx,{'frame_id':'offline','measured14':observed},color,scene=scene,reference14=nominal)
  assert result['ok']
  for bad in ([float('nan')]*14,[0.]*6,[.02]+nominal[1:]):
   with pytest.raises(PerceptionUnknown):q.alignment_evidence(ctx,{'frame_id':'offline','measured14':observed},color,scene=scene,reference14=bad)
