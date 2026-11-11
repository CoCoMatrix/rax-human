"""Offline regression for stale localization boundary; no Context or robot calls."""
import sys,tempfile,json
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rehearsal,approach_target
class Context:
    def value(self,name):
        assert name=='piper_status'
        return {}
with tempfile.TemporaryDirectory(dir='output') as folder:
    old={'ok':True,'observation':{'fresh':False},'visual_calls':1,'perception_s':5,'before_frame':'old'}
    fresh={'ok':True,'observation':{'fresh':True},'visual_calls':0,'perception_s':.1,'before_frame':'new'}
    def checked_create(loc,*a,**kw):
        assert loc is fresh
        raise ValueError('test stops before planning')
    with patch.object(rehearsal,'OUT',Path(folder)),patch.object(rehearsal,'localize_supported',side_effect=[old,fresh]) as localize,patch.object(rehearsal,'observation_recent',side_effect=lambda obs,**kw:obs['fresh']),patch.object(approach_target,'create',side_effect=checked_create),patch.object(approach_target,'invalidate'):
        r=rehearsal.prepare_for_approach(Context(),{})
        assert localize.call_count==2 and localize.call_args.args[1]=={'previous':old}
        assert r['metric_localization_count']==2 and r['visual_calls']==1 and r['perception_s']==5.1
        assert not r['executed'] and r['reason']=='test stops before planning'
    with patch.object(rehearsal,'OUT',Path(folder)),patch.object(rehearsal,'localize_supported',side_effect=[old,old]) as localize,patch.object(rehearsal,'observation_recent',return_value=False),patch.object(approach_target,'invalidate'):
        r=rehearsal.prepare_for_approach(Context(),{})
        assert localize.call_count==2 and not r['executed']
        assert r['reason']=='Target requires a fresh successful metric localization'
print(json.dumps({'ok':True,'cases':['one bounded refresh preserves costs','still stale stops before motion']}))
