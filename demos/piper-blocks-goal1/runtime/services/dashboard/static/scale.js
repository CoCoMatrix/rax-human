'use strict';
const $=id=>document.getElementById(id);
const labels={prepare:'初始定位与接近准备',hover:'杯口上方',entry:'进入夹持位置',grasp:'闭爪抓取',trial_lift:'40 mm 试抬',localize_coaster:'读取本轮杯垫目标',transfer_1:'搬运到杯垫上方',lower_to_support:'下降到放置高度',release:'释放',retreat_1:'退开与最终检查',done:'已完成'};
let state={},sending=false,latestImages={},polling=false,historySignature="";
const text=(id,v)=>{$(id).textContent=v??'—'};
function cell(row,v){const td=document.createElement('td');td.textContent=v??'—';row.append(td)}
function failureText(report){
 if(!report||report.ok===true||report.outcome==='success')return '';
 const why=report.why||report.reason||report.error||'';
 const marker='MCP tool failed: ';const at=why.indexOf(marker);
 if(at>=0){try{return JSON.parse(why.slice(at+marker.length)).error.reason||why}catch(e){}}
 return why;
}
async function action(name){
 if(sending)return;
 sending=true;buttons();text('error','');
 try{const r=await fetch('/api/action/'+name,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({scene_ready:$('ready').checked})});const j=await r.json();if(!r.ok)throw Error(j.error);if(name==='run')$('ready').checked=false;}
 catch(e){text('error',e.message)}finally{sending=false;await refresh()}
}
function buttons(){for(const n of ['connect','prepare'])$(n).disabled=sending||state.busy;$('run').disabled=sending||state.busy||!state.prepared||!$('ready').checked;$('stop').disabled=sending||(!state.busy&&!state.connected)}
for(const name of ['connect','prepare','run','stop'])$(name).onclick=()=>action(name);
$('ready').onchange=buttons;
async function images(){
 await Promise.all(['head','left','right'].map(async name=>{
  if(!state.connected){$(name).removeAttribute('src');text(name+'-age','未连接');return}
  const meta=state.telemetry?.cameras?.[name];text(name+'-age',meta?.available?'实时预览 · '+(Number(meta.age_seconds)*1000).toFixed(0)+' ms':'相机暂不可用');
  if(!meta?.available||!(meta.age_seconds>=0&&meta.age_seconds<=.5)){$(name).removeAttribute('src');text(name+'-placeholder','等待相机新画面');return}
  try{const r=await fetch('/api/camera/'+name+'.jpg');if(!r.ok)throw Error('画面不可用');const url=URL.createObjectURL(await r.blob());$(name).src=url;if(latestImages[name])URL.revokeObjectURL(latestImages[name]);latestImages[name]=url;text(name+'-placeholder','')}
  catch(e){$(name).removeAttribute('src');text(name+'-placeholder','连接中 / 当前画面不可用')}
 }));
}
async function refresh(){
 if(polling)return;polling=true;
 try{
 const r=await fetch('/api/status');if(!r.ok)throw Error('控制台连接失败：'+r.status);state=await r.json();
 text('phase',state.phase);text('version',(state.version||'')+' · '+(state.commit||'').slice(0,10));text('output',state.output||'');text('error',state.error||failureText(state.report)||state.telemetry_error||'');
 const p=state.report||{},stages=p.steps||[];text('elapsed',p.elapsed_s==null?'—':p.elapsed_s.toFixed(1)+' s');text('agent',p.main_agent_calls);text('vision',p.auxiliary_visual_calls);text('completed',stages.filter(x=>x.ok).length);
 $('stages').replaceChildren();const names=[...new Set([...Object.keys(labels).filter(n=>n!=='done'),...stages.map(x=>x.stage)])];
 for(const name of names){const s=stages.find(x=>x.stage===name),div=document.createElement('div');div.className='stage'+(p.active_step===name&&!s?' active':s?.ok?' ok':'');for(const v of [labels[name]||name,s?s.ok?'完成':'未通过':p.active_step===name?'执行中':'待执行',s?.stage_elapsed_s!=null?s.stage_elapsed_s.toFixed(2)+' s':'—']){const span=document.createElement('span');span.textContent=v;div.append(span)}$('stages').append(div)}
 text('console',state.events.map(e=>new Date(e.time*1000).toLocaleTimeString()+'  '+e.message).join('\n')+(state.console?'\n\n'+state.console:''));text('startup',state.startup_log);text('telemetry',JSON.stringify(state.telemetry,null,2));
 const hs=JSON.stringify(state.records);if(hs!==historySignature){historySignature=hs;
 const expanded=new Set(Array.from($('runs').querySelectorAll('details[open]')).map(x=>x.dataset.run));
 $('runs').replaceChildren();for(const row of state.records){const tr=document.createElement('tr');cell(tr,row.id+' · '+row.layout);cell(tr,row.version+' · '+(row.commit||'').slice(0,10));cell(tr,({success:'成功',failed:'失败',running:'运行中',interrupted:'中断'})[row.outcome]||row.outcome);cell(tr,(row.elapsed_s==null?'—':row.elapsed_s.toFixed(2)+' s')+' / '+(row.total_wall_s==null?'—':row.total_wall_s.toFixed(2)+' s'));const td=document.createElement('td'),details=document.createElement('details'),summary=document.createElement('summary');details.dataset.run=row.id;details.open=expanded.has(row.id);summary.textContent=(row.stages||[]).length+' 个阶段';details.append(summary);for(const s of row.stages||[]){const line=document.createElement('div');line.textContent=(labels[s.stage]||s.stage)+' · '+(s.stage_elapsed_s==null?'—':s.stage_elapsed_s.toFixed(2)+' s')+(s.ok?'':' · 未通过');details.append(line)}td.append(details);tr.append(td);cell(tr,(row.auxiliary_visual_calls??'—')+' / '+(row.main_agent_calls??'—'));cell(tr,[row.failed_step,failureText(row)].filter(Boolean).join(' · ')||'—');$('runs').append(tr)}}
 const done=state.records.filter(x=>x.outcome!=='running');text('summary',`${done.length} 轮已记录 · ${done.filter(x=>x.outcome==='success').length} 轮无人工接管成功。更换摆位验证，不把单次成功当作泛化结论。`);
 buttons();await images();
 }catch(e){text('error',e.message);for(const name of ['head','left','right'])$(name).removeAttribute('src');state={};buttons()}finally{polling=false}
}
refresh();setInterval(refresh,1200);
