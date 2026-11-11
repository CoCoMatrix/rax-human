"""Compact read-only context; never opens MCP or sends motion."""
import json
from pathlib import Path


def status(workspace=Path('.')):
    def read(name):
        path=workspace/name
        return json.loads(path.read_text()) if path.is_file() else {}
    protected=workspace/'.codex/rax_context.py'
    if protected.is_file() and not protected.is_symlink():
        import importlib.util
        spec=importlib.util.spec_from_file_location('rax_protected_context',protected)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        result=module.digest(workspace.resolve())
        result['next_context']='automatic continuation; RESUME.md fallback'
        return result
    state=read('output/runs/current-round/ROUND.json')
    latest=read('output/runs/current-round/latest.json')
    keys=('phase','last_report','needs_observation','pending_dispatch','execution_unknown',
          'controller_sha256','mixed_versions','measured_lift_m','task_success')
    result={'round':{k:state.get(k) for k in keys},'last_action':latest.get('action'),
        'executed':latest.get('executed'),'measured_arrival':latest.get('measured_arrival'),
        'next_context':'RESUME.md','preflight_available':(workspace/'output/preflight-summary.json').is_file(),
        'initial_segmentation':read('output/runs/current-round/INITIAL_SCENE.json').get('status'),
        'last_executed_actions':state.get('executed_actions',[])[-3:]}
    preflight=read('output/preflight-summary.json')
    if preflight:
        rows=preflight.get('candidates',[])
        result['preflight']={
            'hypothesis':str(preflight.get('hypothesis',''))[:1000],
            'checked':len(rows),'solved':sum(row.get('ok') is True for row in rows),
            'solved_labels':[row.get('candidate',{}).get('label','unnamed') for row in rows if row.get('ok') is True][:16],
            'failed_labels':[row.get('candidate',{}).get('label','unnamed') for row in rows if row.get('ok') is not True][:16],
            'report':'output/preflight-summary.json','history':'output/preflight-history',
            'planning_only':True,'motion_permit':False}
    geometry=read('output/geometry-analysis-latest.json')
    if geometry:result['geometry_analysis']=geometry
    return result


if __name__=='__main__':print(json.dumps(status(),ensure_ascii=False))
