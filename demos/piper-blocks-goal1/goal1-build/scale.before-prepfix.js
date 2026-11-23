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
 $('run').disabled=!!blocked||!state.prepared||!$('ready').checked||!!(state.goal&&!state.goal.runnable);
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
 const progress=state.report?.active_step==='whole-task-preflight'&&state.report?.status==='in_progress'?state.report.preflight_progress:null;const phaseNames={complete_candidate_geometry:'完整候选几何筛查',native_candidate_validation:'完整路径原生检查',long_budget_recheck:'长预算补查',source_endpoints:'抓取位姿可达性',endpoint_ik:'候选位姿',layout_endpoints:'整体摆位',hover:'接近路径',entry:'下降路径',trial:'试抬',raise:'抬升',transfer:'搬运',lower:'放置',retreat:'退开'};const colorNames={orange:'橙块',green:'绿块',red:'红块',yellow:'黄块'};const progressText=progress?' · '+(progress.ik_pass==='full'?'SSIK 定点补查':progress.ik_pass==='fast'?'SSIK 快速预检':'')+' · '+(colorNames[progress.color]||'')+(phaseNames[progress.phase]||progress.phase||'')+' · '+progress.elapsed_s.toFixed(1)+' s · 已通过 '+progress.checked_paths+' 项检查'+(progress.ik_queries?' · IK 短/长 '+progress.ik_queries.fast+'/'+progress.ik_queries.full+' · 复用 '+(progress.ik_successes_reused||0):'')+(progress.order?' · 候选顺序 '+progress.order.map(c=>colorNames[c]||c).join('→'):''):'';text('phase',state.phase+progressText);document.title='RAX · '+(state.version||'积木控制台');{const full=state.version||'',short=full.replace('r9-ssik-sharedik-portfolio-inward1-blend1-carry2-','');text('current-version',short||'未知版本');$('current-version').title=full;text('current-commit',state.commit?'提交 '+state.commit.slice(0,10):'')}if(state.goal){text('workspace','当前目标：'+state.goal.summary+(state.goal.runnable?'':' · 本版本规划器暂不支持，不能运行'));if(goal.loadedDigest&&goal.loadedDigest!==state.goal.digest&&!goal.busy)goalLoad(true)}text('version',(state.version||'')+' · '+(state.commit||'').slice(0,10)+' · '+(state.ik_solver||'未知求解器')+' · SDK '+state.sdk_speed_percent+'% · '+('离线布局 '+(state.offline_layouts??'—')+'/'+(state.offline_layout_total??'—')+' · 历史场景 '+(state.offline_replay_scenes??'—')+' 组'));text('output',state.output||'');text('error',actionError||state.error||failureText(state.report)||state.telemetry_error||'');
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
// ---- goal1: goal photo -> fixed goal description (operator confirms before any run) ----
const GOAL_COLORS=['red','orange','green','yellow'],GOAL_NAMES={red:'红',orange:'橙',green:'绿',yellow:'黄'},GOAL_FILL={red:'#d2382f',orange:'#ef8a24',green:'#86bf3c',yellow:'#f0c12e'};
const goal={server:null,cells:null,loose:false,source:'manual',image:null,upload:null,photoUrl:null,busy:false,checkTimer:null,lastCheck:null,loadedDigest:null};
function goalSvg(tag,attrs){const e=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const k in attrs)e.setAttribute(k,attrs[k]);return e}
function goalCellsFromSpec(spec){const cells={};for(const s of spec.stacks)s.blocks.forEach((c,i)=>{cells[c]={col:s.col,row:s.row,level:i+1}});return cells}
function goalDraw(){
 const box=$('goal-drawing');box.replaceChildren();if(!goal.cells)return;
 const u=54,pitch=u*(goal.loose?1.7:68/60),dx=.3,dy=-.42;
 const items=GOAL_COLORS.filter(c=>goal.cells[c]).map(c=>({color:c,...goal.cells[c]}));
 items.sort((a,b)=>b.row-a.row||a.col-b.col||a.level-b.level);
 const pos=b=>({x:b.col*pitch+b.row*pitch*dx,y:-(b.level-1)*u+b.row*pitch*dy});
 let minX=1e9,minY=1e9,maxX=-1e9,maxY=-1e9;
 for(const b of items){const p=pos(b);minX=Math.min(minX,p.x);maxX=Math.max(maxX,p.x+u+u*dx);minY=Math.min(minY,p.y-u+u*dy);maxY=Math.max(maxY,p.y)}
 const pad=26,w=maxX-minX+2*pad,h=maxY-minY+2*pad+18;
 const root=goalSvg('svg',{viewBox:`${minX-pad} ${minY-pad} ${w} ${h}`,role:'img','aria-label':'目标结构示意图'});
 root.style.maxWidth=Math.min(460,Math.max(240,w*1.5))+'px';
 for(const b of items){
  const p=pos(b),x=p.x,y=p.y,ox=u*dx,oy=u*dy,g=goalSvg('g',{});
  g.append(goalSvg('polygon',{points:`${x},${y-u} ${x+ox},${y-u+oy} ${x+u+ox},${y-u+oy} ${x+u},${y-u}`,fill:GOAL_FILL[b.color],stroke:'#172f38','stroke-width':1.2}));
  g.append(goalSvg('polygon',{points:`${x},${y-u} ${x+ox},${y-u+oy} ${x+u+ox},${y-u+oy} ${x+u},${y-u}`,fill:'#fff','fill-opacity':.28}));
  g.append(goalSvg('polygon',{points:`${x+u},${y-u} ${x+u+ox},${y-u+oy} ${x+u+ox},${y+oy} ${x+u},${y}`,fill:GOAL_FILL[b.color],stroke:'#172f38','stroke-width':1.2}));
  g.append(goalSvg('polygon',{points:`${x+u},${y-u} ${x+u+ox},${y-u+oy} ${x+u+ox},${y+oy} ${x+u},${y}`,fill:'#000','fill-opacity':.2}));
  g.append(goalSvg('rect',{x,y:y-u,width:u,height:u,fill:GOAL_FILL[b.color],stroke:'#172f38','stroke-width':1.2}));
  const label=goalSvg('text',{x:x+u/2,y:y-u/2+6,'text-anchor':'middle','font-size':17,'font-weight':700,fill:'#172f38'});label.textContent=GOAL_NAMES[b.color]+b.level;g.append(label);
  root.append(g);
 }
 const note=goalSvg('text',{x:minX-pad+8,y:maxY+pad+10,'font-size':12,fill:'#516872'});note.textContent='↓ 靠近机器人 / 拍照者一侧';root.append(note);
 box.append(root);
}
function goalRenderTable(){
 const body=$('goal-cells');body.replaceChildren();
 for(const c of GOAL_COLORS){
  const tr=document.createElement('tr'),name=document.createElement('td'),chip=document.createElement('span');
  chip.className='goal-chip';chip.style.background=GOAL_FILL[c];name.append(chip,GOAL_NAMES[c]+'色');tr.append(name);
  for(const [key,max] of [['col',4],['row',4],['level',3]]){
   const td=document.createElement('td'),sel=document.createElement('select');sel.setAttribute('aria-label',GOAL_NAMES[c]+'色 '+({col:'列',row:'排',level:'层'})[key]);
   for(let i=1;i<=max;i++){const o=document.createElement('option');o.value=i;o.textContent=key==='level'?'第 '+i+' 层':String(i);sel.append(o)}
   const cur=goal.cells?.[c];sel.value=cur?(key==='level'?cur.level:cur[key]+1):1;
   sel.onchange=()=>{goal.cells[c]=goal.cells[c]||{col:0,row:0,level:1};goal.cells[c][key]=key==='level'?Number(sel.value):Number(sel.value)-1;goal.source='manual';goalDraw();goalScheduleCheck()};
   td.append(sel);tr.append(td);
  }
  body.append(tr);
 }
 $('goal-loose').checked=goal.loose;
}
function goalPayload(){return {cells:goal.cells,spacing:goal.loose?'loose':'tight',source:goal.source,image:goal.image}}
async function goalPost(name,body,timeout){
 const r=await fetch('/api/goal/'+name,{method:'POST',signal:AbortSignal.timeout(timeout||8000),headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
 const j=await r.json();if(!r.ok)throw Error(j.error||('请求失败：'+r.status));return j;
}
function goalShowCheck(check){
 goal.lastCheck=check;
 text('goal-error',check.ok?'':check.errors.join('；'));
 text('goal-warnings',check.ok?check.warnings.join(' '):'');
 const active=goal.server?.active,same=check.ok&&active&&check.digest===active.digest;
 text('goal-summary',check.ok?check.summary+(check.is_default?' · 默认结构':'')+(same?' · 已是当前目标':' · 尚未确认'):'');
 $('goal-confirm').disabled=goal.busy||!check.ok||same;
}
function goalScheduleCheck(){
 clearTimeout(goal.checkTimer);
 goal.checkTimer=setTimeout(async()=>{try{goalShowCheck(await goalPost('validate',goalPayload()))}catch(e){text('goal-error',e.message)}},180);
}
function goalApplyServer(j,keepEdits){
 goal.server=j;const a=j.active,sources={default:'默认',gemini:'Gemini 解析',manual:'手动填写'};
 text('goal-active-note','当前目标：'+a.summary+' · 来源：'+(sources[a.source]||a.source)+(j.runnable?'':' · 本版本规划器暂不支持，不能运行'));
 $('goal-active-note').classList.toggle('goal-blocked',!j.runnable);
 goal.loadedDigest=a.digest;
 if(!keepEdits){
  const pending=j.pending&&j.pending.check&&j.pending.check.ok?j.pending:null,spec=pending?pending.check.spec:a.spec;
  goal.cells=goalCellsFromSpec(spec);goal.loose=spec.spacing==='loose';goal.source=pending?'gemini':a.source==='default'?'manual':a.source;goal.image=pending?pending.image:a.image;
  goalRenderTable();goalDraw();goalScheduleCheck();
 }
 const shown=goal.upload?null:(j.pending?.image||a.image);
 if(shown){$('goal-photo').src='/api/goal/image/'+shown;text('goal-photo-placeholder','')}
 else if(!goal.upload){$('goal-photo').removeAttribute('src');text('goal-photo-placeholder','尚未选择照片')}
 $('goal-reset').disabled=goal.busy||(a.is_default&&!j.pending);
 if(j.pending&&!goal.upload&&!$('goal-analyze-note').textContent)text('goal-analyze-note',goalAnalysisNote(j.pending));
 goalShowCandidates(j.pending);
}
async function goalLoad(keepEdits){try{goalApplyServer(await (await fetch('/api/goal',{cache:'no-store',signal:AbortSignal.timeout(6000)})).json(),keepEdits)}catch(e){text('goal-active-note','目标读取失败：'+e.message)}}
function goalAnalysisNote(p){
 if(!p)return '';
 const a=p.answer||{},agree={unanimous:p.readings+' 次解析结果一致',majority:p.readings+' 次解析多数一致',split:p.readings+' 次解析结果不一致',none:'没有得到有效结果'}[p.agreement]||'';
 const lines=['Gemini 用时 '+p.duration_s.toFixed(1)+' 秒 · '+agree+' · 视角 '+(({top_down:'俯拍',frontal:'正面',corner_on:'斜对角'})[a.viewpoint]||a.viewpoint||'—')];
 if(a.summary)lines.push('描述：'+a.summary);
 if((a.uncertain||[]).length)lines.push('不确定：'+a.uncertain.join('；'));
 for(const n of p.notes||[])lines.push('注意：'+n);
 if(!p.check.ok)lines.push('未通过格式校验：'+p.check.errors.join('；')+'。右侧保留原来的目标，可以手动修改或换一张照片。');
 else lines.push('请对照照片核对右侧示意图，无误后点“确认为本轮目标”。');
 return lines.join('\n');
}
function goalShowCandidates(p){
 const box=$('goal-candidates');box.replaceChildren();
 if(!p||!p.candidates||p.candidates.length<2)return;
 p.candidates.forEach((c,i)=>{
  const b=document.createElement('button');b.type='button';b.textContent='方案 '+(i+1)+'（'+c.votes+'/'+p.readings+' 次）：'+c.check.summary;
  b.onclick=()=>{goal.cells=goalCellsFromSpec(c.check.spec);goal.loose=c.check.spec.spacing==='loose';goal.source='gemini';goalRenderTable();goalDraw();goalScheduleCheck();for(const x of box.children)x.classList.toggle('chosen',x===b)};
  if(i===0)b.classList.add('chosen');box.append(b);
 });
}
$('goal-file').onchange=async()=>{
 const file=$('goal-file').files[0];if(!file)return;
 text('goal-analyze-note','');goal.upload=null;$('goal-analyze').disabled=true;
 try{
  const bitmap=await createImageBitmap(file,{imageOrientation:'from-image'});
  const scale=Math.min(1,1600/Math.max(bitmap.width,bitmap.height)),canvas=document.createElement('canvas');
  canvas.width=Math.round(bitmap.width*scale);canvas.height=Math.round(bitmap.height*scale);canvas.getContext('2d').drawImage(bitmap,0,0,canvas.width,canvas.height);bitmap.close();
  const blob=await new Promise(done=>canvas.toBlob(done,'image/jpeg',.88));if(!blob)throw Error('无法转换图片');
  goal.upload=await new Promise((done,fail)=>{const fr=new FileReader();fr.onload=()=>done(fr.result);fr.onerror=()=>fail(Error('读取图片失败'));fr.readAsDataURL(blob)});
  if(goal.photoUrl)URL.revokeObjectURL(goal.photoUrl);goal.photoUrl=URL.createObjectURL(blob);$('goal-photo').src=goal.photoUrl;text('goal-photo-placeholder','');
  text('goal-analyze-note','已选择 '+file.name+'（'+canvas.width+'×'+canvas.height+'）。点“用 Gemini 解析”。');$('goal-analyze').disabled=false;
 }catch(e){text('goal-analyze-note','这张图片浏览器打不开（'+e.message+'）。请用 JPG 或 PNG；iPhone 的 HEIC 照片请先转成 JPG，或在“设置 → 相机 → 格式”里选“兼容性最佳”。')}
};
$('goal-analyze').onclick=async()=>{
 if(!goal.upload||goal.busy)return;
 goal.busy=true;$('goal-analyze').disabled=true;$('goal-confirm').disabled=true;text('goal-error','');text('goal-analyze-note','Gemini 正在解析（同一张照片读 3 次互相比对），通常 10–20 秒，斜拍的照片可能要 1 分钟…');
 try{const j=await goalPost('analyze',{image:goal.upload},90000);goal.busy=false;goal.upload=null;goalApplyServer(j,!(j.pending&&j.pending.check.ok));text('goal-analyze-note',goalAnalysisNote(j.pending))}
 catch(e){goal.busy=false;text('goal-analyze-note','解析没有成功：'+(e.name==='TimeoutError'?'等待超时':e.message))}
 finally{goal.busy=false;$('goal-analyze').disabled=!goal.upload;if(goal.lastCheck)goalShowCheck(goal.lastCheck)}
};
$('goal-loose').onchange=()=>{goal.loose=$('goal-loose').checked;goal.source='manual';goalDraw();goalScheduleCheck()};
$('goal-confirm').onclick=async()=>{
 if(goal.busy)return;goal.busy=true;$('goal-confirm').disabled=true;
 try{const j=await goalPost('confirm',goalPayload());goal.busy=false;goalApplyServer(j,false);text('goal-analyze-note',j.runnable?'目标已确认。':'目标已确认并保存。注意：本版本的规划器还不能按自定义目标搭建，“执行一次任务”已禁用；恢复默认目标后可照常运行。');await refresh()}
 catch(e){text('goal-error',e.message)}finally{goal.busy=false;if(goal.lastCheck)goalShowCheck(goal.lastCheck)}
};
$('goal-reset').onclick=async()=>{
 if(goal.busy)return;goal.busy=true;
 try{const j=await goalPost('reset',{});goal.busy=false;goal.upload=null;goalApplyServer(j,false);text('goal-analyze-note','已恢复默认目标。');await refresh()}
 catch(e){text('goal-error',e.message)}finally{goal.busy=false}
};
goalLoad(false);
refresh();setInterval(refresh,1200);
