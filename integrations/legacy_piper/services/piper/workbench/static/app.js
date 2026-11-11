'use strict';
let state = null, mode = 'explore', lastMessages = '', lastPrograms = '', errorDismissed = '', polling = false;
const $ = id => document.getElementById(id);
const phases = {disconnected:'尚未连接',connecting:'正在连接',connected:'已连接 · 待 Prepare',preparing:'Prepare 归零中',prepared:'Prepare 已就绪',running:'任务执行中',held:'Hold · 已请求停止',awaiting_result:'等待核对结果',error:'需要处理'};
function node(tag, cls, text) { const e=document.createElement(tag); if(cls)e.className=cls; if(text!==undefined)e.textContent=text; return e; }
function toast(text){$('toast').textContent=text;$('toast').classList.remove('hidden');setTimeout(()=>$('toast').classList.add('hidden'),3500);}
function showError(error){$('error-title').textContent=error.message||'操作失败';$('error-fix').textContent=error.fix||'';$('error-banner').classList.remove('hidden');}
async function api(path, body){const r=await fetch(path,body===undefined?{cache:'no-store',signal:AbortSignal.timeout(5000)}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const d=await r.json();if(!r.ok)throw d.error||{message:`HTTP ${r.status}`};return d;}
async function action(name, extra={}){if(!state){showError({message:'工作台连接尚未就绪'});return;}try{const d=await api('/api/action/'+name,{request_id:crypto.randomUUID(),epoch:state.epoch,...extra});if(d.accepted){toast('操作已接收，可在进度和事件中查看结果');if(['prepare','start','next-round','quick-start'].includes(name))$('scene-clear').checked=false;}await poll();return d;}catch(e){showError(e);}}
function badge(id,text,ok){const el=$(id);el.textContent=text;el.className=ok?'ok':'bad';}
function render(s){state=s;$('phase').textContent=phases[s.phase]||s.phase;$('phase').classList.toggle('online',['connected','prepared','running'].includes(s.phase));$('clock').textContent=new Date().toLocaleTimeString('zh-CN',{hour12:false});$('connection-status').textContent='工作台在线 · '+new Date().toLocaleTimeString('zh-CN',{hour12:false});
 $('auth-banner').classList.toggle('hidden',s.clock_broker_active&&(s.clock_broker?.expires||0)>Date.now()/1000);$('guard-command').textContent=s.guard_command;
 badge('health-hardware',s.robot_fresh?'机械臂正常':'机械臂未就绪',s.robot_fresh);badge('health-vision',s.vision_ready?'三视角正常':'视觉未就绪',s.vision_ready);badge('health-depth',s.vision_ready?'FFS 正常':'FFS 未就绪',s.vision_ready);badge('health-write',s.arming.armed?'真实写入已启用':(s.capabilities.real_write_available?'控制入口已连接':'写入关闭'),s.arming.armed);
 $('health-hardware').title=s.robot_error||`反馈年龄 ${s.robot_age_s==null?'—':(s.robot_age_s*1000).toFixed(0)} ms · 读取 ${s.health?.feedback?.duration_s==null?'—':(s.health.feedback.duration_s*1000).toFixed(0)} ms`;
 $('vision-quality').textContent=s.vision_ready?`视角跨度 ${(s.vision.span_s*1000).toFixed(0)} ms · 最近 ${(s.vision_age_s).toFixed(1)} s`:(s.vision_error||s.sensor_error||'等待三路新鲜图像');
 for(const c of ['head','left','right']){const img=$('camera-'+c), wrap=img.parentElement, card=wrap.parentElement, loaded=s.vision_ready&&img.dataset.loaded==='true'&&img.dataset.run===s.run&&img.dataset.epoch===s.epoch;wrap.classList.toggle('loaded',loaded);const b=card.querySelector('.live-badge');b.textContent=loaded?'LIVE':'OFFLINE';b.classList.toggle('online',loaded);$('meta-'+c).textContent=s.vision?.images?.[c]?.shape_hw?.slice().reverse().join(' × ')||'—';if(!s.vision_ready)img.dataset.loaded='false';}
 const q=s.robot_fresh?s.robot?.robot_state?.values:null;$('joint-values').textContent=q?q.slice(0,6).map(v=>(v*180/Math.PI).toFixed(1)).join('  '):'— — — — — —';$('jaw-value').textContent=q?(q[6]*1000).toFixed(1):'—';$('run-id').textContent=s.run||'NO SESSION';
 const busy=s.job?.status==='running';for(const id of ['connect','reconnect','disconnect','prepare','start-task','quick-start','freeze','next-round','segment'])$(id).disabled=busy; $('hold').disabled=false;$('quick-start').disabled=busy||s.phase==='running';
 $('job').classList.toggle('hidden',!busy);$('job').textContent=busy?`进行中：${s.job.name} · ${Math.round(Date.now()/1000-s.job.started)} 秒`:'';
 $('operation-note').textContent=s.phase==='held'?'Hold 已发出。检查实测反馈；继续任务须显式重新连接。':s.phase==='running'?'正在执行真实任务。Hold 随时可用；空格快捷键在输入框内不触发。':s.phase==='prepared'?(s.prepare_mode==='hold-current'?'当前位置与夹爪保持已验收；恢复任务从当前场景继续。':'左臂 Prepare 已验收，可勾选现场确认并输入本轮任务。'):'连接自检包含三路相机、FFS、实测关节与控制入口。';
 if(s.error&&s.error.at!==errorDismissed)showError(s.error);
 const step=s.phase==='running'?(s.task?.mode==='frozen_test'?'test':'explore'):s.phase==='awaiting_result'?'test':s.vision_ready?'observe':'connect';document.querySelectorAll('.flow>div').forEach(e=>e.classList.toggle('active',e.dataset.step===step));
 $('model-label').textContent=s.model?`${s.model} · ${s.model_effort||'high'}`:'gpt-6-astra · high';$('checkpoint-phase').textContent=s.progress?.phase||'等待任务';$('progress-message').textContent=s.task?.auto_freeze_status||s.progress?.message||'尚无本轮执行进度；以下是双视角的检查分工。';
 const pkey=JSON.stringify([s.programs,s.selected_program]);if(pkey!==lastPrograms){lastPrograms=pkey;$('programs').replaceChildren();$('batch-program').replaceChildren();for(const p of s.programs){const r=node('div','program'),left=node('div');left.append(node('b','',p.name),node('span','mono',p.version.slice(0,12)),node('small','',p.reason||p.status));r.append(left,node('span','tag '+(p.batch_ready?'':'neutral'),(p.id===s.selected_program?(p.batch_ready?'当前 · ':'参考基线 · '):'')+p.status));$('programs').append(r);const opt=node('option','',p.name+(p.batch_ready?'':' · 不可批测'));opt.value=p.id;opt.disabled=!p.batch_ready;$('batch-program').append(opt);}const ready=s.programs.find(p=>p.batch_ready);if(ready)$('batch-program').value=ready.id;}
 $('batch-next').classList.toggle('hidden',s.batch?.state!=='awaiting_reset');$('result-controls').classList.toggle('hidden',s.phase!=='awaiting_result'||s.task?.result_recorded);$('batch-progress').textContent=s.batch?`批量计划：${s.batch.results.length}/${s.batch.target_rounds} 轮 · ${s.batch.state==='awaiting_reset'?'等待换摆位':s.batch.state==='complete'?'已完成':s.batch.state==='interrupted'?'已中断':s.batch.state==='awaiting_result'?'等待核对结果':'执行中'}`:'';
 $('segment-images').classList.toggle('hidden',!s.segmentation);
 if(s.segmentation){
  const pack=s.segmentation.bundle_id;
  if($('segment-head').dataset.pack!==pack){
   for(const c of ['head','left']){const img=$('segment-'+c);img.onerror=()=>{$('segment-head').dataset.pack='';};img.src='/api/segmentation/'+c+'.jpg?pack='+encodeURIComponent(pack);}
   $('segment-head').dataset.pack=pack;
  }
  const rows=Object.entries(s.segmentation.views).map(([c,v])=>(c==='head'?'头部':'左腕')+' '+v.objects.length+' 个高置信检测、'+(v.candidates?.length||0)+' 个低置信候选'+(v.missing?.length?'，未达确认阈值 '+v.missing.join('/') : ''));
  const seconds=Number(s.segmentation.total_s??s.segmentation.inference_s??0).toFixed(2);
  const age=s.segmentation.source_created_ns?Math.max(0,Date.now()/1000-s.segmentation.source_created_ns/1e9).toFixed(1):'未知';
  $('segment-result').textContent=rows.join(' · ')+' · 推理 '+seconds+' 秒 · 原图 '+age+' 秒前 · '+(s.segmentation.origin==='agent_same_frame_broker'?'本轮初始目标参考':'手动快照')+'；分割不证明接触';
 }

 const mkey=JSON.stringify(s.messages);if(mkey!==lastMessages){lastMessages=mkey;if(s.messages.length){const box=$('messages'),near=box.scrollHeight-box.scrollTop-box.clientHeight<80;box.replaceChildren();for(const m of s.messages){const e=node('div','message '+m.kind);if(m.kind==='assistant')e.append(node('span','message-label','RAX / CODEX'));e.append(document.createTextNode(m.text));box.append(e);}if(near)box.scrollTop=box.scrollHeight;}else{$('messages').replaceChildren(node('p','hint','当前会话暂无 RAX 回答。启动任务后显示本轮真实记录。'));}}
 renderRequests(s.requests);$('events').replaceChildren();for(const e of s.events.slice(-12).reverse()){const row=node('div','event '+e.kind);row.append(node('time','',new Date(e.at*1000).toLocaleTimeString('zh-CN',{hour12:false})),node('span','',e.message));$('events').append(row);}$('event-count').textContent=s.events.length+' 条';$('logs').textContent=[...s.logs,...s.hardware_logs].join('\n')||'暂无日志';
 $('history').replaceChildren();if(!s.history.length)$('history').append(node('p','hint','还没有在工作台记录的完整试验。历史探索不自动计入本次固定版本验收。'));for(const row of s.history.slice().reverse()){const e=node('div','history-row');e.append(node('span','tag '+(row.success?'':'neutral'),row.success?'完整成功':'失败 / 未完成'),document.createTextNode(row.task),node('div','mono muted',(row.controller_sha256||'探索模式').slice(0,12)+' · '+new Date(row.at*1000).toLocaleString('zh-CN')),node('div','',row.note||''));$('history').append(e);}
 if(!$('plan-cards').childElementCount){const titles=['探索与纠正','冻结完整程序','连续跨摆位成功','扩展验证'];Object.values(s.plan).forEach((text,i)=>{const el=node('article','plan-card');el.append(node('b','',String(i+1).padStart(2,'0')),node('h3','',titles[i]),node('p','',text));$('plan-cards').append(el);});}
}
let requestsKey='';function renderRequests(rows){const k=JSON.stringify(rows);if(k===requestsKey)return;requestsKey=k;$('operator-requests').replaceChildren();for(const r of rows){const box=node('div','request');box.append(node('strong','',r.kind==='reset'?'需要现场复位':'RAX 需要你的帮助'),node('p','',r.message));const input=node('textarea');input.rows=2;input.placeholder='输入现场情况或处理结果';const button=node('button','secondary','确认完成并回复');button.onclick=()=>action('reply',{id:r.id,text:input.value,done:true});box.append(input,button);$('operator-requests').append(box);}}
async function poll(){if(polling)return;polling=true;try{render(await api('/api/state'));}catch(e){$('connection-status').textContent='工作台连接中断';showError(e);for(const c of ['head','left','right'])$('camera-'+c).parentElement.classList.remove('loaded');}finally{polling=false;}}
let camerasPolling=false;
async function cameras(){
 if(camerasPolling||!state?.vision_ready)return;
 camerasPolling=true;const run=state.run,epoch=state.epoch;
 try{await Promise.all(['head','left','right'].map(async c=>{
  const img=$('camera-'+c),wrap=img.parentElement,badge=wrap.parentElement.querySelector('.live-badge');
  try{
   const r=await fetch('/api/camera/'+c+'.jpg',{cache:'no-store',signal:AbortSignal.timeout(3000)});
   if(!r.ok)throw Error('stale');const blob=await r.blob();
   if(state?.run!==run||state?.epoch!==epoch||!state?.vision_ready)return;
   const url=URL.createObjectURL(blob),old=img.dataset.url;
   img.onload=()=>{const fresh=state?.run===run&&state?.epoch===epoch&&state?.vision_ready;img.dataset.loaded=String(fresh);wrap.classList.toggle('loaded',fresh);badge.textContent=fresh?'LIVE':'OFFLINE';badge.classList.toggle('online',fresh);};
   img.onerror=()=>{img.dataset.loaded='false';wrap.classList.remove('loaded');badge.textContent='OFFLINE';badge.classList.remove('online');};
   img.dataset.run=run;img.dataset.epoch=epoch;img.src=url;img.dataset.url=url;if(old)URL.revokeObjectURL(old);
  }catch(e){img.dataset.loaded='false';wrap.classList.remove('loaded');badge.textContent='OFFLINE';badge.classList.remove('online');}
 }));}finally{camerasPolling=false;}
}

$('hold').onclick=()=>action('hold');document.addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','TEXTAREA','SELECT','BUTTON'].includes(document.activeElement.tagName)){e.preventDefault();action('hold');}});
for(const key of ['connect','reconnect','disconnect'])$(key).onclick=()=>action(key);
$('quick-start').onclick=()=>action('quick-start',{scene_clear:$('scene-clear').checked,empty_jaw_confirmed:$('scene-clear').checked,lighting_ready:$('scene-clear').checked,minutes:Number($('minutes').value)});
$('prepare').onclick=()=>action('prepare',{scene_clear:$('scene-clear').checked});
$('start-task').onclick=()=>mode==='explore'?action('start',{scene_clear:$('scene-clear').checked,task:$('task-text').value,minutes:Number($('minutes').value)}):action('batch',{program:$('batch-program').value,rounds:Number($('rounds').value),task:$('task-text').value});
$('next-round').onclick=()=>action('next-round',{scene_clear:$('scene-clear').checked,layout:$('layout').value,minutes:Number($('minutes').value)});
$('success').onclick=()=>action('result',{success:true,note:$('result-note').value});$('failure').onclick=()=>action('result',{success:false,note:$('result-note').value});
$('segment').onclick=()=>action('segment',{prompts:$('segment-prompts').value});
$('freeze').onclick=()=>action('freeze',{directory:$('freeze-directory').value,name:$('freeze-name').value});
$('copy-guard').onclick=async()=>{try{await navigator.clipboard.writeText(state.guard_command);toast('已复制；请在 Mac 终端粘贴执行');}catch(e){toast('请选中下方命令复制');}};
$('dismiss-error').onclick=()=>{errorDismissed=state?.error?.at;$('error-banner').classList.add('hidden');};
document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>{mode=b.dataset.mode;document.querySelectorAll('[data-mode]').forEach(x=>x.classList.toggle('active',x===b));$('rounds-field').classList.toggle('hidden',mode!=='batch');$('program-selection').classList.toggle('hidden',mode!=='batch');$('start-task').textContent=mode==='batch'?'创建固定版本批测计划':'单独启动所填任务 ↗';});
document.querySelectorAll('[data-page]').forEach(b=>b.onclick=async()=>{document.querySelectorAll('.nav').forEach(x=>x.classList.toggle('active',x===b));document.querySelectorAll('.page').forEach(x=>x.classList.toggle('hidden',x.id!=='page-'+b.dataset.page));if(b.dataset.page==='guide')try{$('guide-text').textContent=(await api('/api/guide')).text;}catch(e){showError(e);}});
poll();setInterval(poll,1200);setInterval(cameras,1000);
