'use strict';
const $=id=>document.getElementById(id);
const labels={'green-to-yellow-empty-transition':'绿色退开→黄色上方（空载速度衔接）','orange-lift-transfer':'橙块抬升→搬运（速度衔接）','green-lift-transfer':'绿块抬升→搬运（速度衔接）','red-lift-transfer':'红块抬升→搬运（速度衔接）','whole-task-preflight':'抓取前完整任务预检','rebind-plan':'核对规划后的当前场景','orange-entry-blended':'空载三路点速度衔接','orange-entry':'空载入口（三段共用反馈检查，中间不取图）',"green-hover-inputs":"搜索绿色接近路径并刷新绑定","orange-raise-inputs": "规划橙块抬升及搬运", "orange-raise": "橙块自适应抬升", "green-raise-1-inputs": "规划绿色抬升及后续路径", "green-raise-1": "绿色自适应抬升一", "green-raise-2-inputs": "规划绿色最终搬运高度", "green-raise-2": "绿色自适应抬升二",'initial-state':'初始四色定位','open-empty':'张开空载夹爪','plan-orange-entry':'规划橙块入口','orange-approach':'接近橙块','orange-close':'抓橙块','orange-trial':'橙块试抬','orange-transfer':'搬运橙块','orange-release':'释放橙块','orange-retreat':'橙块退开','green-pregrasp':'绿色预抓取','refresh-stack-support':'核对底座','green-approach':'接近绿块','green-close':'抓绿块','green-trial':'绿块试抬','green-stack-lower':'下降到红块上','green-stack-release':'释放绿块','green-stack-retreat':'退开与完成'};
function stageLabel(name){if(labels[name])return labels[name];const key=name.replace(/-\d+$/,'');if(labels[key])return labels[key];const colors={red:'红块',orange:'橙块',green:'绿块',yellow:'黄块',empty:'空载'};const stages={hover:'到达抓取上方',entry:'下降接近',close:'闭爪',trial:'试抬',raise:'离障抬升',transfer:'搬运',lower:'下降放置',release:'张爪释放',retreat:'退开',open:'张爪'};const [color,stage]=key.split('-');return colors[color]&&stages[stage]?colors[color]+' · '+stages[stage]:name}
let visibleRuns=10,online=false,actionError="",state={},sending=false,latestImages={},polling=false,historySignature="";
const text=(id,v)=>{$(id).textContent=v??'—'};
function cell(row,v){const td=document.createElement('td');td.textContent=v??'—';row.append(td)}
function failureText(report){
 if(!report||report.ok===true||report.outcome==='success')return '';
 const categories={planning_or_geometry:'规划／几何拒绝',native_path_collision:'原生碰撞／工作区拒绝',ik_no_solution:'IK 无解',planning_budget:'预检预算耗尽',service_connection:'服务连接中断',service_timeout:'服务请求超时',io_error:'文件／I/O 错误',cancelled:'操作已取消',program_error:'程序错误',execution_error:'执行异常',scene_evidence:'场景证据不足'};
 const prefix=categories[report.failure_category];
 const why=(prefix?'['+prefix+'] ':'')+(report.why||report.reason||report.error||'');
 const marker='MCP tool failed: ';const at=why.indexOf(marker);
 if(at>=0){try{return JSON.parse(why.slice(at+marker.length)).error.reason||why}catch(e){}}
 return why;
}
async function action(name){
 if(sending)return;
 sending=true;buttons();actionError='';text('error','');
 try{
 if(name==='recover'){
   const health=await fetch('/api/health',{cache:'no-store',signal:AbortSignal.timeout(4000)});
   if(!health.ok)throw Error('无法连接控制台服务');
   if((await health.json()).busy){actionError='已重新连接页面，操作仍在执行。';return}
 }
 const r=await fetch('/api/action/'+name,{method:'POST',signal:AbortSignal.timeout(6000),headers:{'Content-Type':'application/json'},body:JSON.stringify({scene_ready:$('ready').checked})});const j=await r.json();if(!r.ok)throw Error(j.error);if(name==='run')$('ready').checked=false;}
 catch(e){actionError=e.name==='TimeoutError'?'请求超时，请恢复连接后核对状态。':e.message;text('error',actionError)}finally{sending=false;await refresh()}
}
function buttons(){
 const blocked=sending||!online||state.busy||state.recovery_required;
 for(const n of ['connect','prepare'])$(n).disabled=!!blocked;
 $('run').disabled=!!blocked||!state.prepared||!$('ready').checked;
 $('stop').disabled=sending||!online||(!state.busy&&!state.connected);
 $('recover').disabled=sending;
}
for(const name of ['recover','connect','prepare','run','stop'])$(name).onclick=()=>action(name);
$('more').onclick=()=>{visibleRuns+=10;historySignature='';refresh()};
$('ready').onchange=buttons;
buttons();
async function images(){
 await Promise.all(['head','left','right'].map(async name=>{
  if(!state.connected){$(name).removeAttribute('src');text(name+'-age','未连接');text(name+'-placeholder','未连接');return}
  const meta=state.telemetry?.cameras?.[name];text(name+'-age',meta?.available?'实时预览 · '+(Number(meta.age_seconds)*1000).toFixed(0)+' ms':'相机暂不可用');
  if(!meta?.available||!(meta.age_seconds>=0&&meta.age_seconds<=.5)){$(name).removeAttribute('src');text(name+'-placeholder','等待相机新画面');return}
  try{const r=await fetch('/api/camera/'+name+'.jpg',{signal:AbortSignal.timeout(3000)});if(!r.ok)throw Error('画面不可用');const url=URL.createObjectURL(await r.blob());$(name).src=url;if(latestImages[name])URL.revokeObjectURL(latestImages[name]);latestImages[name]=url;text(name+'-placeholder','')}
  catch(e){$(name).removeAttribute('src');text(name+'-placeholder','连接中 / 当前画面不可用')}
 }));
}
async function refresh(){
 if(polling)return;polling=true;
 try{
 const r=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(6500)});if(!r.ok)throw Error('控制台连接失败：'+r.status);state=await r.json();online=true;
 $('recovery-card').classList.toggle('warn',!!state.recovery_required);
 $('phase').dataset.state=state.recovery_required?'warn':'normal';
 text('recovery-note',state.recovery_required?'上次停止未确认。恢复会重新读取静止反馈；持物状态请现场核对。':state.busy?'操作进行中。恢复连接可刷新页面状态。':'核对设备并恢复只读预览，可用于会话到期或连接中断。');
 text('updated','更新于 '+new Date().toLocaleTimeString());
 const progress=state.report?.active_step==='whole-task-preflight'&&state.report?.status==='in_progress'?state.report.preflight_progress:null;const phaseNames={complete_candidate_geometry:'完整候选几何筛查',native_candidate_validation:'完整路径原生检查',long_budget_recheck:'长预算补查',source_endpoints:'抓取位姿可达性',endpoint_ik:'候选位姿',layout_endpoints:'整体摆位',hover:'接近路径',entry:'下降路径',trial:'试抬',raise:'抬升',transfer:'搬运',lower:'放置',retreat:'退开'};const colorNames={orange:'橙块',green:'绿块',red:'红块',yellow:'黄块'};const progressText=progress?' · '+(progress.ik_pass==='full'?'SSIK 定点补查':progress.ik_pass==='fast'?'SSIK 快速预检':'')+' · '+(colorNames[progress.color]||'')+(phaseNames[progress.phase]||progress.phase||'')+' · '+progress.elapsed_s.toFixed(1)+' s · 已通过 '+progress.checked_paths+' 项检查'+(progress.ik_queries?' · IK 短/长 '+progress.ik_queries.fast+'/'+progress.ik_queries.full+' · 复用 '+(progress.ik_successes_reused||0):'')+(progress.order?' · 候选顺序 '+progress.order.map(c=>colorNames[c]||c).join('→'):''):'';text('phase',state.phase+progressText);document.title='RAX · '+(state.version||'积木控制台');text('version',(state.version||'')+' · '+(state.commit||'').slice(0,10)+' · '+(state.ik_solver||'未知求解器')+' · SDK '+state.sdk_speed_percent+'% · '+('离线布局 '+(state.offline_layouts??'—')+'/'+(state.offline_layout_total??'—')+' · 历史场景 '+(state.offline_replay_scenes??'—')+' 组'));text('output',state.output||'');text('error',actionError||state.error||failureText(state.report)||state.telemetry_error||'');
 const p=state.report||{},stages=p.steps||[];text('elapsed',p.elapsed_s==null?'—':p.elapsed_s.toFixed(1)+' s');text('agent',p.main_agent_calls);text('vision',p.auxiliary_visual_calls);text('completed',stages.filter(x=>x.ok).length);
 $('stages').replaceChildren();const names=[...new Set([...stages.map(x=>x.stage),...(p.active_step?[p.active_step]:[])])];
 for(const name of names){const s=stages.find(x=>x.stage===name),div=document.createElement('div');div.className='stage'+(p.active_step===name&&!s?' active':s?.ok?' ok':'');for(const v of [stageLabel(name),s?s.ok?'完成':'未通过':p.active_step===name?'执行中':'待执行',s?.stage_elapsed_s!=null?s.stage_elapsed_s.toFixed(2)+' s':name==='whole-task-preflight'&&progress?progress.elapsed_s.toFixed(1)+' s':'—']){const span=document.createElement('span');span.textContent=v;div.append(span)}$('stages').append(div)}
 text('console',state.events.map(e=>new Date(e.time*1000).toLocaleTimeString()+'  '+e.message).join('\n')+(state.console?'\n\n'+state.console:''));text('startup',state.startup_log);text('telemetry',JSON.stringify(state.telemetry,null,2));
 const hs=JSON.stringify(state.records);if(hs!==historySignature){historySignature=hs;
 const expanded=new Set(Array.from($('runs').querySelectorAll('details[open]')).map(x=>x.dataset.run));
 $('runs').replaceChildren();$('more').hidden=state.records.length<=visibleRuns;for(const row of state.records.slice(0,visibleRuns)){const tr=document.createElement('tr');cell(tr,row.id+' · '+row.layout);cell(tr,row.version+' · '+(row.commit||'').slice(0,10));cell(tr,((({success:'成功',failed:'失败',running:'运行中',interrupted:'中断'})[row.outcome]||row.outcome)+(row.completion_mode==='released_in_place'?' · 已释放、未退开':'')));cell(tr,(row.elapsed_s==null?'—':row.elapsed_s.toFixed(2)+' s')+' / '+(row.total_wall_s==null?'—':row.total_wall_s.toFixed(2)+' s'));const td=document.createElement('td'),details=document.createElement('details'),summary=document.createElement('summary');details.dataset.run=row.id;details.open=expanded.has(row.id);summary.textContent=(row.stages||[]).length+' 个阶段';details.append(summary);for(const s of row.stages||[]){const line=document.createElement('div');line.textContent=(stageLabel(s.stage))+' · '+(s.stage_elapsed_s==null?'—':s.stage_elapsed_s.toFixed(2)+' s')+(s.ok?'':' · 未通过');details.append(line)}td.append(details);tr.append(td);cell(tr,(row.auxiliary_visual_calls??'—')+' / '+(row.main_agent_calls??'—'));cell(tr,[row.failed_step,failureText(row)].filter(Boolean).join(' · ')||'—');$('runs').append(tr)}}
 const done=state.records.filter(x=>x.outcome!=='running');text('summary',`${done.length} 轮已记录 · ${done.filter(x=>x.outcome==='success').length} 轮无人工接管成功。更换摆位验证，不把单次成功当作泛化结论。`);
 buttons();await images();
 }catch(e){online=false;text('phase','页面连接中断');text('recovery-note','状态未知。点击恢复连接重新核对服务；页面会自动重试。');$('recovery-card').classList.add('warn');text('error',actionError||e.message);for(const name of ['head','left','right'])$(name).removeAttribute('src');state={};buttons()}finally{polling=false}
}
refresh();setInterval(refresh,1200);
