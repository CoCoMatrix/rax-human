"""goal1: add goal-description endpoints to the blocks web server. Run from the goal1 runtime root."""
from pathlib import Path

p = Path('services/dashboard/blocks_scale.py')
s = p.read_text()


def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (old[:80], s.count(old))
    s = s.replace(old, new)


rep("PYTHON='/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python'\n",
    "PYTHON='/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python'\n"
    "# goal1: goal photo -> fixed goal description. Host-side only; same key file and proxy as RAX.\n"
    "GEMINI_KEY_FILE='/home/ubuntu/code_junqi/gemini-key.py'\n"
    "GEMINI_PROXY='http://127.0.0.1:7892'\n"
    "GOAL_DAILY_CALLS=300\n"
    "GOAL_VOTES=3  # readings per photo, compared with each other\n"
    "GOAL_IMAGE_NAME=r'goal-\\d{8}-\\d{6}-[0-9a-f]{8}\\.jpg'\n"
    "PLANNER_SUPPORTS_CUSTOM_GOAL=True  # goal1 phase 5: the confirmed goal is handed to each run as GOAL.json.\n")

rep("        self.finished_detail_cache={}\n",
    "        self.finished_detail_cache={}\n"
    "        self.goal_dir=self.data/'goal'; self.goal_lock=threading.Lock(); self.goal_pending=None\n")

GOAL_METHODS = '''
    # ---- goal1: goal description. Reads/writes files under data/goal only; never starts devices. ----
    def goal_active(self):
        from services.dashboard import goal_spec
        saved=read(self.goal_dir/'active.json')
        if saved:
            checked=goal_spec.check(saved.get('spec'))
            if checked['ok'] and checked['digest']==saved.get('digest'):
                return {**checked,'source':saved.get('source'),'confirmed_at':saved.get('confirmed_at'),'image':saved.get('image')}
        checked=goal_spec.check(goal_spec.DEFAULT_SPEC)
        return {**checked,'source':'default','confirmed_at':None,'image':None}

    def goal_runnable(self,active=None):
        active=active or self.goal_active()
        return bool(active['is_default'] or PLANNER_SUPPORTS_CUSTOM_GOAL)

    def goal_state(self,body=None):
        active=self.goal_active()
        return {'active':active,'pending':self.goal_pending,'planner_supports_custom':PLANNER_SUPPORTS_CUSTOM_GOAL,
                'runnable':self.goal_runnable(active),'model':'gemini-3.8-flash'}

    def _goal_log(self,row):
        self.goal_dir.mkdir(parents=True,exist_ok=True)
        with (self.goal_dir/'analyses.jsonl').open('a') as f:f.write(json.dumps({'time':time.time(),**row},ensure_ascii=False)+'\\n')

    def goal_analyze(self,body):
        from services.dashboard import goal_spec,goal_gemini
        jpeg,meta=goal_gemini.load_image(body.get('image'))
        if not self.goal_lock.acquire(blocking=False):raise RuntimeError('上一张目标图还在解析，请稍候')
        try:
            day=time.strftime('%Y-%m-%d');calls=0
            try:
                for line in (self.goal_dir/'analyses.jsonl').read_text().splitlines():
                    row=json.loads(line)
                    if time.strftime('%Y-%m-%d',time.localtime(row.get('time',0)))==day:calls+=int(row.get('called') or 0)
            except (OSError,ValueError):pass
            if calls>=GOAL_DAILY_CALLS:raise RuntimeError(f'今天已解析 {calls} 次，达到每日上限 {GOAL_DAILY_CALLS} 次')
            self.goal_dir.mkdir(parents=True,exist_ok=True)
            name='goal-'+time.strftime('%Y%m%d-%H%M%S')+'-'+hashlib.sha256(jpeg).hexdigest()[:8]+'.jpg'
            (self.goal_dir/name).write_bytes(jpeg)
            result=goal_gemini.analyze_votes(jpeg,key_file=GEMINI_KEY_FILE,proxy_url=GEMINI_PROXY,votes=GOAL_VOTES)
            answers=[r['answer'] for r in result['results'] if 'answer' in r];failed=[r['error'] for r in result['results'] if 'error' in r]
            if not answers:
                self._goal_log({'image':name,'called':GOAL_VOTES,'ok':False,'errors':failed})
                raise RuntimeError('Gemini 解析失败：'+failed[0][:240]+'。可以重试，或直接在下方手动填写目标。')
            candidates,agreement=goal_spec.tally(answers);notes=[]
            best=candidates[0] if candidates else {'check':goal_spec.check(answers[0]),'answer':answers[0],'votes':0}
            answer=best['answer'];checked=best['check']
            if agreement in ('majority','split'):
                notes.append('同一张照片解析了 %d 次，结果不一致（%s）。请对照照片选一个方案，或者正对积木的一个面重拍。'%(len(result['results']),'、'.join('%d 次：%s'%(c['votes'],c['check']['summary']) for c in candidates)))
            if failed:notes.append('%d 次解析没有返回结果（%s）'%(len(failed),failed[0][:60]))
            if answer.get('viewpoint')=='corner_on':notes.append('这张照片是对着积木的棱角斜拍的，前后左右的位置关系容易读错；正对一个面拍更可靠')
            if checked['ok']:
                for line in goal_spec.contact_mismatch(checked['spec'],answer.get('contacts')):notes.append('前后矛盾，请重点核对：'+line)
            if answer.get('image_usable') is False:notes.append('Gemini 认为这张照片不能可靠读出四块积木的摆放')
            confidence=answer.get('confidence')
            if isinstance(confidence,(int,float)) and confidence<.7:notes.append(f'Gemini 的把握度只有 {confidence:.2f}，请仔细核对')
            self._goal_log({'image':name,'called':GOAL_VOTES,'ok':checked['ok'],'agreement':agreement,'answers':answers,'errors':failed,'digest':checked.get('digest'),
                            'tokens':result['tokens'],'cost_upper_bound_usd':result['cost_upper_bound_usd'],'duration_s':result['duration_s']})
            self.goal_pending={'image':name,'image_size':meta,'answer':answer,'check':checked,'notes':notes,'time':time.time(),
                               'agreement':agreement,'readings':len(result['results']),'candidates':[{'check':c['check'],'votes':c['votes']} for c in candidates],
                               'duration_s':result['duration_s'],'tokens':result['tokens'],'cost_upper_bound_usd':result['cost_upper_bound_usd']}
            self.event('目标图已解析（Gemini %d 次，%.1f 秒，%s）：%s'%(len(result['results']),result['duration_s'],{'unanimous':'结果一致','majority':'多数一致','split':'结果不一致','none':'无有效结果'}[agreement],checked.get('summary') or '格式校验未通过'))
            return self.goal_state()
        finally:self.goal_lock.release()

    def goal_validate(self,body):
        from services.dashboard import goal_spec
        raw=goal_spec.from_cells(body.get('cells'),body.get('spacing','tight')) if 'cells' in body else body.get('spec')
        return goal_spec.check(raw)

    def goal_confirm(self,body):
        import re
        if self.mode=='run' and self.thread and self.thread.is_alive():raise RuntimeError('任务运行中不能更换目标')
        checked=self.goal_validate(body)
        if not checked['ok']:raise ValueError('；'.join(checked['errors']))
        image=body.get('image');source=body.get('source') if body.get('source') in ('gemini','manual') else 'manual'
        if not (isinstance(image,str) and re.fullmatch(GOAL_IMAGE_NAME,image) and (self.goal_dir/image).is_file()):image=None
        self.goal_dir.mkdir(parents=True,exist_ok=True)
        save(self.goal_dir/'active.json',{'spec':checked['spec'],'digest':checked['digest'],'source':source,'image':image,'confirmed_at':time.time()})
        self.goal_pending=None
        self.event('已确认搭建目标：'+checked['summary']+('' if self.goal_runnable() else '（本版本规划器暂不支持，不能运行）'))
        return self.goal_state()

    def goal_reset(self,body=None):
        if self.mode=='run' and self.thread and self.thread.is_alive():raise RuntimeError('任务运行中不能更换目标')
        (self.goal_dir/'active.json').unlink(missing_ok=True);self.goal_pending=None
        self.event('搭建目标已恢复为默认结构。')
        return self.goal_state()

    def goal_image(self,name):
        import re
        if not re.fullmatch(GOAL_IMAGE_NAME,name):raise KeyError('goal image')
        try:return (self.goal_dir/name).read_bytes()
        except OSError:raise KeyError('goal image') from None

    def submit(self,action,body):'''
rep("\n    def submit(self,action,body):", GOAL_METHODS)

rep("                if not self.prepared: raise ValueError('先使用 XPolicy 回 Prepare，再重新摆位')\n",
    "                if not self.prepared: raise ValueError('先使用 XPolicy 回 Prepare，再重新摆位')\n"
    "                if not self.goal_runnable():\n"
    "                    raise ValueError('已确认的搭建目标不是默认结构。本版本的规划器还不支持自定义目标，没有发送任何动作。请先“恢复默认目标”再运行。')\n")

rep("        self.active_record={'id':out.name,'layout':'初始视觉定位中','version':meta.get('candidate_revision'),'commit':commit,\n"
    "            'started':self.started,'output':str(out),'outcome':'running','human_assisted':False}\n",
    "        goal=self.goal_active()\n"
    "        self.active_record={'id':out.name,'layout':'初始视觉定位中','version':meta.get('candidate_revision'),'commit':commit,\n"
    "            'started':self.started,'output':str(out),'outcome':'running','human_assisted':False,\n"
    "            'goal_digest':goal['digest'],'goal_summary':goal['summary'],'goal_source':goal['source'],'goal_image':goal['image']}\n")

rep("        return {'recovery_required':self.recovery_required,'phase':phase,",
    "        goal=self.goal_active()\n"
    "        return {'goal':{'summary':goal['summary'],'digest':goal['digest'],'source':goal['source'],'is_default':goal['is_default'],'runnable':self.goal_runnable(goal)},\n"
    "            'recovery_required':self.recovery_required,'phase':phase,")

rep("            elif path=='/api/status': self._json(200,self.manager.snapshot())\n",
    "            elif path=='/api/status': self._json(200,self.manager.snapshot())\n"
    "            elif path=='/api/goal': self._json(200,self.manager.goal_state())\n"
    "            elif path.startswith('/api/goal/image/'): self._send(200,self.manager.goal_image(path.rsplit('/',1)[-1]),'image/jpeg')\n")

rep("        if not path.startswith('/api/action/'): self._error(404,'not found'); return\n",
    "        if path in ('/api/goal/analyze','/api/goal/validate','/api/goal/confirm','/api/goal/reset'):\n"
    "            self._guarded(lambda:self._json(200,getattr(self.manager,'goal_'+path.rsplit('/',1)[-1])(self._goal_body()))); return\n"
    "        if not path.startswith('/api/action/'): self._error(404,'not found'); return\n")

rep("    def log_message(self,*args): pass  # Never log a token-bearing URL.\n",
    "    def log_message(self,*args): pass  # Never log a token-bearing URL.\n"
    "    def _goal_body(self):\n"
    "        # Same JSON-only rule as _body, with room for one downscaled photo.\n"
    "        if self.headers.get('Content-Type','').split(';')[0].strip()!='application/json':raise ValueError('POST bodies must be application/json')\n"
    "        length=int(self.headers.get('Content-Length') or 0)\n"
    "        if length>5*1024*1024:raise ValueError('图片太大，请缩小后再上传')\n"
    "        value=json.loads(self.rfile.read(length) or b'{}')\n"
    "        if not isinstance(value,dict):raise ValueError('expected a JSON object')\n"
    "        return value\n")

# goal1: Prepare is its own action, not a precondition of a run. The controller still starts
# only from native neutral with a closed empty jaw; when a session is connected the web checks
# that from live feedback and says so before launching anything.
rep("                if not self.prepared: raise ValueError('先使用 XPolicy 回 Prepare，再重新摆位')\n",
    "                self.check_run_start()\n")
rep("            if action=='run': self.verify_prepared()\n", "")
rep("    def verify_prepared(self):\n",
    "    NEUTRAL_JOINT_RAD=.01; CLOSED_JAW_M=.005  # same bounds as the controller's fresh-start check\n"
    "    def check_run_start(self):\n"
    "        \"\"\"Refuse early only on live evidence that the arm is away from neutral or the jaw is not closed.\"\"\"\n"
    "        if not (self.process and self.process.poll() is None and self.out):return None\n"
    "        try:q=self.owner()['robot_state']['values']\n"
    "        except Exception:return None  # unreadable: the controller's own start check decides\n"
    "        if max(abs(x) for x in q[:6])>self.NEUTRAL_JOINT_RAD or not 0<=q[6]<=self.CLOSED_JAW_M:\n"
    "            raise ValueError('机械臂当前不在零位或夹爪未闭合（左臂最大关节 %.3f rad，夹爪 %.1f mm）。本轮需要从零位、空爪闭合开始：请先点“回到 Prepare”，或确认现场后重试。'%(max(abs(x) for x in q[:6]),q[6]*1000))\n"
    "        return q\n"
    "\n"
    "    def verify_prepared(self):\n")

# ---- goal1 phase 5: hand the confirmed goal to the run ----
rep("        self.launch(self.root/'scripts/piper_explore.py',out,'--task-config',",
    "        self.goal_write(goal)  # copied into the session together with the controller code\n"
    "        self.launch(self.root/'scripts/piper_explore.py',out,'--task-config',")
rep("        ready=out/'REHEARSAL-READY.json'; self.wait(lambda:read(ready),300,'硬件/深度启动超时')\n",
    "        ready=out/'REHEARSAL-READY.json'; self.wait(lambda:read(ready),300,'硬件/深度启动超时')\n"
    "        self.goal_handed_over(read(ready),goal)  # before the controller starts: nothing has moved yet\n")
rep("        self.finish_record()\n        if not clean:\n",
    "        plans=list((out/'rax-session/agent-workspace/output/composition/runs').glob('*/WHOLE_TASK_PLAN.json'))\n"
    "        if plans:\n"
    "            planned=(read(plans[0],{}).get('goal') or {}).get('digest')\n"
    "            self.active_record.update(goal_digest_in_plan=planned,goal_used_by_planner=planned==goal['digest'])\n"
    "        self.finish_record()\n        if not clean:\n")
rep("            self.error=f'{type(exc).__name__}: {exc}'; self.event(self.error)\n",
    "            self.error=f'{type(exc).__name__}: {exc}'; self.event(self.error)\n"
    "            self.goal_file().unlink(missing_ok=True)\n")
rep("    def goal_image(self,name):\n",
    "    def goal_file(self):\n"
    "        return self.candidate/'code/magnetic_blocks/GOAL.json'\n\n"
    "    def goal_write(self,goal):\n"
    "        \"\"\"Write the confirmed goal next to the controller code; the run launcher copies the code tree into its session.\"\"\"\n"
    "        from services.dashboard import goal_spec\n"
    "        checked=goal_spec.check(goal['spec'])\n"
    "        if not checked['ok'] or checked['digest']!=goal['digest']:raise RuntimeError('已确认的目标没有通过校验，未启动运行')\n"
    "        f=self.goal_file();tmp=f.with_name('.GOAL.json.tmp')\n"
    "        tmp.write_text(json.dumps(checked['spec'],ensure_ascii=False));tmp.replace(f)\n\n"
    "    def goal_handed_over(self,ready,goal):\n"
    "        \"\"\"The session's own copy must be the confirmed goal; the version directory keeps no goal file.\"\"\"\n"
    "        from services.dashboard import goal_spec\n"
    "        try:\n"
    "            copy=Path(ready['workspace'])/'code/magnetic_blocks/GOAL.json'\n"
    "            got=goal_spec.check(json.loads(copy.read_text()))\n"
    "        except (KeyError,TypeError,OSError,ValueError):got={'ok':False}\n"
    "        finally:self.goal_file().unlink(missing_ok=True)\n"
    "        if not got['ok'] or got.get('digest')!=goal['digest']:raise RuntimeError('本轮会话里的目标与已确认目标不一致，未开始任务，机械臂没有动作')\n"
    "        self.active_record.update(goal_handed_over=True);save(self.data/(self.out.name+'.json'),self.active_record)\n"
    "        self.event('本轮目标已交给控制器：'+goal['summary'])\n\n"
    "    def goal_image(self,name):\n")

p.write_text(s)
print('server patched')
