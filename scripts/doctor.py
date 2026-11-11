"""Read-only deployment-file check. No robot, camera, SDK, model or key is opened."""
import argparse,json,sys
from pathlib import Path
from site_paths import ROOT,path,settings,value

def inspect():
    defaults={'legacy_root':ROOT/'integrations/legacy_piper','owner_root':ROOT/'integrations/xpolicy_owner',
      'host_python':sys.executable,'owner_python':ROOT/'.venv-owner/bin/python',
      'dora':ROOT/'.venv-owner/bin/dora','policy_adapter_root':ROOT/'integrations/policy_adapter',
      'ffs_python':ROOT/'.venv-ffs/bin/python','head_calibration':ROOT/'services/piper/assets/head-stereo-calibration.json',
      'ffs_manifest':ROOT/'runtime/models/fast-foundation-stereo/model-manifest.json',
      'ffs_warmup_packet':ROOT/'runtime/calibration/ffs-warmup','moveit_manifest':ROOT/'runtime/moveit/manifest.json',
      'agent_runtime':ROOT/'.venv-agent'}
    checks={name:{'path':str(path(name,default)),'exists':path(name,default).exists()} for name,default in defaults.items()}
    owner=path('owner_root',defaults['owner_root']);legacy=path('legacy_root',defaults['legacy_root'])
    for name,p in {'owner_graph':owner/'runtime/robot_dagger/graphs/piper-x-dual-policy-30hz-keyboard.yml',
       'collision_mesh_manifest':legacy/'assets/piper-32cm/asset-manifest.json',
       'native_rgb_registration_source':legacy/'services/piper/sensor/rgb_registration.cpp',
       'isolated_agent_python':path('agent_runtime',defaults['agent_runtime'])/'bin/python'}.items():
       checks[name]={'path':str(p),'exists':p.is_file()}
    url=value('relay_base_url','')
    return {'hardware_started':False,'credentials_read':False,'files_ready':all(x['exists'] for x in checks.values()),
      'relay_url_configured':bool(url and '.example' not in url),'checks':checks,
      'not_verified':['physical wiring and clearance','calibration accuracy','SDK/firmware pairing','FFS inference','network/API','IK/collision and motion qualification']}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--strict',action='store_true');a=p.parse_args()
    report=inspect();print(json.dumps(report,indent=2))
    if a.strict and not report['files_ready']:raise SystemExit(1)
