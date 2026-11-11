"""Installed geometry + formal ActionChunk contract; no hardware APIs."""
import sys,json,math,time
from pathlib import Path
import numpy as np
from types import SimpleNamespace
here=Path(__file__).parent;root=here/'runtime'
sys.path.insert(0,str(root/'scripts'))
from piper_motion_bridge import load_bridge
m=load_bridge(Path('/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025'),'faster')
from _rax_installed_piper.geometry import Kinematics
from real_robot.piper.contracts import PiperContract,vector_schema
kin=Kinematics();source=json.loads((here/'source-plans.json').read_text())['plans']
work=here/'integrated-review';(work/'bridge').mkdir(parents=True,exist_ok=True)
(work/'backend-options.json').write_text(json.dumps({'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36]}))
t=m.MotionTransport(m.PiperNoSendBackend(SimpleNamespace(backend_options={})),m.CollisionChecker(),arm_check=lambda:dict(armed=False),stop_client=lambda:(_ for _ in ()).throw(AssertionError('No hardware')),lease_file=work/'bridge/lease.json',max_displacement_rad=math.radians(60),joint_tolerance_rad=.01,allow_left_gripper=True,max_velocity_rad_s=math.radians(20),max_acceleration_rad_s2=math.radians(40),terminal_dwell_s=.2,tracking_velocity_ratio=.85)
owner=root/'owner-runtime'
contract=PiperContract.from_runtime(owner,horizon=600,action_prefix=600,trajectory_mode=True)
z=json.loads((here/'table.json').read_text())['table_z'];table=[{'name':'observed_task_table','center_m':[.255,.1225,z+.008-.025],'size_m':[.29,.445,.05]}]
rows=[]
for name,ids in [('approach',range(2,9)),('transfer_lower',range(12,14))]:
    start=time.monotonic();plans=[source[i] for i in ids];a=plans[0]['anchor14']
    p=t.preview_waypoints(a,[x['target14'][:6] for x in plans],table,blend_indices=[0,1,2,3] if name=='approach' else [0])
    wire=np.asarray(p['targets14'],np.float32)
    checked=contract.validate_candidate(wire,schema=vector_schema(action=True),frequency_hz=30,measured_state14=np.asarray(a),prefix_steps=len(wire))
    for q in wire:
        pos=kin.fk(q[:6].tolist())[:3,3]
        assert np.all(pos>=[.055,-.4,.17]) and np.all(pos<=[.5,.045,.36])
    (work/(name+'.json')).write_text(json.dumps(p))
    rows.append(dict(name=name,ok=True,samples=len(wire),duration_s=p['duration_s'],review_s=time.monotonic()-start,contract_passed=True,collision=p['collision']))
result=dict(offline_only=True,hardware_started=False,chunks=rows)
(here/'INTEGRATED-REVIEW.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
