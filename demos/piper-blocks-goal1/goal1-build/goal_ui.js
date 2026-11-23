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
