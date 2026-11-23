"""Disjoint layout portfolios on private CPU numerical services. No motion."""
from pathlib import Path
import json,os,subprocess,sys,time,uuid

def partition_layouts(layouts,index,count):
    if not (type(index) is int and type(count) is int and 0<=index<count<=4):
        raise ValueError('Invalid layout partition')
    # Equal layout identities always belong to one worker, including duplicates.
    owners={};selected=[];ids=[]
    for layout in layouts:
        key=json.dumps(layout,separators=(',',':'))
        if key not in owners:owners[key]=len(owners)%count
        if owners[key]==index:selected.append(layout);ids.append(key)
    return selected,ids

def save(path,value):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,allow_nan=False));tmp.replace(path)

def restore_key(value):
    return tuple(restore_key(x) for x in value) if isinstance(value,list) else value

def execution_cache(planner):
    names=('native_cache','fk_cache','sample_fk_cache','ik_cache','pose_solutions')
    result={name:list(getattr(planner,name,{}).items()) for name in names}
    result['path_cache']=[(k,v) for k,v in planner.path_cache.items() if v.get('native',{}).get('cad_clear') is True]
    return result

def restore_execution_cache(planner,cache):
    for name,entries in cache.items():
        if name not in ('native_cache','fk_cache','sample_fk_cache','ik_cache','pose_solutions','path_cache'):
            raise ValueError('Unexpected worker cache field')
        setattr(planner,name,{restore_key(k):v for k,v in entries})

def validate_plan(plan,scene,start):
    from assembly_plan import digest
    if not plan.get('ok') or not plan.get('fully_preflighted') or plan.get('executed') is not False:
        raise ValueError('Worker did not return a certified unexecuted plan')
    if plan.get('start14')!=start or digest(plan['source_scene'])!=digest(scene):
        raise ValueError('Worker scene/start mismatch')
    for move in plan['moves']:
        for action in move['actions']:
            checks=[action['check']]+action.get('release_feedback_preflight',{}).get('checks',[])
            if 'next_pickup_check' in action:checks.append(action['next_pickup_check'])
            for check in checks:
                native=check.get('native',{})
                if '_pending_native' in check or native.get('cad_clear') is not True or native.get('workspace_clear') is not True:
                    raise ValueError('Worker plan has uncertified paths')

def plan_parallel(planner,start,count=4):
    from task_spec import PREFLIGHT_BUDGET_S
    from assembly_plan import PlanningTimeout,NoPlan
    begun=time.monotonic();folder=Path('output/parallel-preflight')/uuid.uuid4().hex;folder.mkdir(parents=True)
    pool=None;processes=[];winner=None;outcomes={};last_progress={};close_report=None
    result=None;cache_manager=None;shared_cache=None;cache_report=None
    try:
        pool=planner.ctx.value('piper_planning_pool_open',count=count)
        identities=[x['geometry_identity'] for x in pool['lanes']]
        if len({x['pid'] for x in pool['lanes']})!=count or any(x!=identities[0] for x in identities):
            raise ValueError('Worker process/geometry identity mismatch')
        from shared_ik import start_cache
        cache_manager,shared_cache,cache_packet=start_cache(folder,pool)
        for lane in pool['lanes']:
            i=lane['id'];work=folder/str(i);work.mkdir()
            packet={'scene':planner.scene,'robot':planner.robot,'start14':start,'partition':[i,count],
                    'socket':lane['socket'],'started':begun,'output':str(work),'shared_ik':cache_packet}
            # Use constants, not assumptions about future gripper configuration.
            from assembly_plan import OPEN,CLOSED
            packet['local_open']=planner.local[OPEN].tolist();packet['local_closed']=planner.local[CLOSED].tolist()
            save(work/'INPUT.json',packet)
            log=(work/'client.log').open('w')
            proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--worker',str(work/'INPUT.json')],
                stdout=log,stderr=subprocess.STDOUT,env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
            log.close();processes.append(proc)
        previous_event=None;last_emit=0.
        while time.monotonic()-begun<PREFLIGHT_BUDGET_S:
            for i,proc in enumerate(processes):
                f=folder/str(i)/'RESULT.json'
                if i not in outcomes and f.exists():
                    value=json.loads(f.read_text());outcomes[i]=value
                    if value.get('ok'):
                        validate_plan(value['plan'],planner.scene,start)
                        restore_execution_cache(planner,json.loads((folder/str(i)/'EXECUTION-CACHE.json').read_text()))
                        winner=i;result=value['plan'];break
                progress=folder/str(i)/'PROGRESS.json'
                if progress.exists():last_progress[i]=json.loads(progress.read_text())
                if proc.poll() is not None and i not in outcomes:
                    outcomes[i]={'ok':False,'error':'Worker process exited without result','exit_code':proc.returncode}
            if winner is not None:break
            callback=getattr(planner.ctx,'_blocks_preflight_event',None)
            tick=time.monotonic();event=json.dumps([last_progress,list(outcomes)],sort_keys=True)
            if callback and (event!=previous_event or tick-last_emit>=1.):
                callback({'phase':'parallel_search','elapsed_s':tick-begun,'workers':last_progress,'finished':list(outcomes)});previous_event=event;last_emit=tick
            if len(outcomes)==count:
                from collections import Counter
                bins=Counter()
                for result in outcomes.values():bins.update(result.get('screen_counts',{}))
                brief='；'.join(k+' ×'+str(v) for k,v in bins.most_common(8))
                raise NoPlan('候选预筛拒绝（含不同布局的重复尝试）：'+brief+'；各进程末项：'+str({i:v.get('error','')[:240] for i,v in outcomes.items()}))
            time.sleep(.15)
        if winner is None:raise PlanningTimeout('Parallel preflight exceeded unchanged180s global budget')
        result['parallel_search']={'workers':count,'winner':winner,'search_wall_s':time.monotonic()-begun,
                                   'partition':'unique layout identity modulo workers; all order/grasp/path descendants stay with owner',
                                   'pool':pool,'artifact':str(folder)}
        planner.ik_attempts=outcomes[winner].get('ik_attempts',[])
        planner.opening_check=result['opening_check']
        return result
    finally:
        for proc in processes:
            if proc.poll() is None:proc.terminate()
        for proc in processes:
            try:proc.wait(timeout=2)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        if shared_cache is not None:
            cache_report=shared_cache.stats()
            save(folder/'SHARED-IK-SAMPLES.json',shared_cache.verification_sample())
        if cache_manager is not None:cache_manager.shutdown()
        if pool is not None:close_report=planner.ctx.value('piper_planning_pool_close',pool_id=pool['pool_id'])
        assignments={str(i):json.loads((folder/str(i)/'ASSIGNMENT.json').read_text()) for i in range(len(processes)) if (folder/str(i)/'ASSIGNMENT.json').exists()}
        sets=[set(v) for v in assignments.values()]
        overlap=any(sets[i]&sets[j] for i in range(len(sets)) for j in range(i))
        if overlap:raise ValueError('Parallel layout ownership overlaps')
        summary={'winner':winner,'wall_s_including_cleanup':time.monotonic()-begun,'outcomes':{str(k):{a:b for a,b in v.items() if a!='plan'} for k,v in outcomes.items()},'last_progress':last_progress,'services':close_report,'hardware_dispatches':0}
        summary.update(assignments=assignments,overlapping_layouts=overlap,shared_ik_cache=cache_report)
        save(folder/'SUMMARY.json',summary)
        if result is not None:
            result['parallel_search']['total_wall_s']=summary['wall_s_including_cleanup']
            result['parallel_search']['shared_ik_cache']=cache_report

def worker(packet_path):
    sys.path.insert(0,os.getcwd())
    from api.runtime import Context
    from assembly_plan import Planner
    data=json.loads(Path(packet_path).read_text());folder=Path(data['output']);p=None
    with Context(data['socket']) as ctx:
        from shared_ik import attach_cache
        attach_cache(ctx,data['shared_ik'])
        ctx._blocks_batch_fk=True;ctx._blocks_local_fk=True;ctx._blocks_batch_ik=True
        ctx.allow_auxiliary_vision=False;ctx._blocks_layout_partition=data['partition']
        ctx._blocks_preflight_event=lambda detail:save(folder/'PROGRESS.json',detail)
        ctx._blocks_partition_event=lambda ids:save(folder/'ASSIGNMENT.json',ids)
        try:
            p=Planner(ctx,data['scene'],data['robot'],data['local_open'],data['local_closed'])
            plan=p.plan(data['start14']);out={'ok':True,'plan':plan,'ik_attempts':getattr(p,'ik_attempts',[])}
            save(folder/'EXECUTION-CACHE.json',execution_cache(p))
        except Exception as exc:out={'ok':False,'error':type(exc).__name__+': '+str(exc)}
        out['screen_counts']=getattr(p,'screen_counts',{});out['layout_ids']=getattr(p,'assigned_layout_ids',[]);out['hardware_dispatches']=0;out['shared_ik_cache']=dict(ctx._shared_ik_counts)
        save(folder/'RESULT.json',out)

if __name__=='__main__':worker(sys.argv[2])
