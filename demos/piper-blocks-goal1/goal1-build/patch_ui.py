"""goal1: add the goal panel and a visible version line to the blocks web page. Run from the goal1 runtime root."""
import sys
from pathlib import Path

static = Path('services/dashboard/static/blocks')
h = static/'scale.html'
s = h.read_text()


def one(text, old, new):
    assert text.count(old) == 1, (old[:70], text.count(old))
    return text.replace(old, new)


css = (
    '.ver-line{margin-top:8px;font-size:15px;color:var(--muted);overflow-wrap:anywhere}'
    '.ver-line strong{color:var(--teal);font:700 16px/1.3 ui-monospace,SFMono-Regular,Menlo,monospace;background:#e2f0ec;padding:2px 8px;border-radius:5px}'
    '.ver-line span{font:13px ui-monospace,SFMono-Regular,Menlo,monospace}'
    '.goal{margin-top:20px}.goal-grid{display:grid;grid-template-columns:minmax(260px,2fr) minmax(0,3fr);gap:24px}'
    '.goal h3{margin-bottom:6px}.goal p{margin-top:10px}.goal .buttons{margin-top:12px}'
    '.goal-photo{aspect-ratio:4/3;border-radius:8px;overflow:hidden;margin-top:12px}'
    '.goal-drawing svg{display:block;width:100%;height:auto;background:var(--soft);border-radius:8px}'
    '.goal-table{border-collapse:collapse;margin-top:12px}.goal-table th,.goal-table td{padding:5px 14px 5px 0;text-align:left;font-size:14px}'
    '.goal-table th{color:var(--muted);font-weight:600}'
    '.goal select{font:inherit;min-height:38px;padding:6px 8px;border:1px solid #91aaa9;border-radius:6px;background:#fff;color:var(--ink)}'
    '.goal-chip{display:inline-block;width:14px;height:14px;border-radius:3px;border:1px solid #172f38;margin-right:8px;vertical-align:-2px}'
    '#goal-analyze-note{white-space:pre-line}#goal-summary{font-weight:600}'
    '#goal-warnings:empty,#goal-error:empty,#goal-summary:empty,#goal-analyze-note:empty{display:none}'
    '#goal-warnings{color:#784600;background:#fff0d8;padding:10px;border-radius:6px}'
    '#goal-error{color:#882d24;background:#fff2ed;padding:10px;border-radius:6px;overflow-wrap:anywhere}'
    '.goal-blocked{color:#784600}'
    '.goal-candidates{display:grid;gap:8px;margin-top:10px}.goal-candidates:empty{display:none}.goal-candidates button{text-align:left;font-weight:500}.goal-candidates button.chosen{border-color:var(--teal);box-shadow:inset 0 0 0 1px var(--teal);background:#eef6f3}'
    '.goal select:focus-visible,.goal label.link:focus-within{outline:3px solid #285ab2;outline-offset:3px}'
    '@media(max-width:820px){.goal-grid{grid-template-columns:1fr}}'
)
s = one(s, '</style>', css + '</style>')
s = one(s, '<h1 id="console-title">四色积木实验台</h1></div>',
        '<h1 id="console-title">四色积木实验台</h1><div class="ver-line">当前版本 <strong id="current-version">读取中…</strong> <span id="current-commit"></span></div></div>')
panel = '''<section class="panel goal" id="goal-panel">
 <div class="rowhead" style="margin:0 0 14px"><h2>搭建目标</h2><small id="goal-active-note">读取目标…</small></div>
 <div class="goal-grid">
  <div>
   <h3>1 · 上传目标照片</h3>
   <p class="muted">用手机拍一张四块积木摆好后的样子。<b>正对积木的一个面拍</b>，可以稍微俯视；不要对着棱角斜着拍。照片要是正的，不要横着或倒着。支持 JPG 和 PNG。</p>
   <div class="buttons"><label class="link" for="goal-file">选择照片</label><input id="goal-file" type="file" accept="image/jpeg,image/png" hidden><button id="goal-analyze" class="primary" disabled>用 Gemini 解析</button></div>
   <div class="view goal-photo"><span id="goal-photo-placeholder">尚未选择照片</span><img id="goal-photo" alt="目标照片"></div>
   <p id="goal-analyze-note" class="muted" role="status"></p>
   <div id="goal-candidates" class="goal-candidates"></div>
  </div>
  <div>
   <h3>2 · 核对并修改</h3>
   <div id="goal-drawing" class="goal-drawing"></div>
   <table class="goal-table"><thead><tr><th>积木</th><th>列（左→右）</th><th>排（近→远）</th><th>层</th></tr></thead><tbody id="goal-cells"></tbody></table>
   <label class="check"><input type="checkbox" id="goal-loose"><span>相邻积木分开摆，不紧挨</span></label>
   <p id="goal-summary"></p><p id="goal-warnings"></p><p role="alert" id="goal-error"></p>
   <div class="buttons"><button id="goal-confirm" class="primary" disabled>确认为本轮目标</button><button id="goal-reset" disabled>恢复默认目标</button></div>
   <p class="muted">解析结果只是建议，确认后才生效。运行过程中不调用 Gemini。</p>
  </div>
 </div>
</section>
<div class="grid">'''
s = one(s, '<div class="grid">', panel)
h.write_text(s)

j = static/'scale.js'
t = j.read_text()
old = "document.title='RAX · '+(state.version||'积木控制台');"
t = one(t, old, old + "{const full=state.version||'',short=full.replace('r9-ssik-sharedik-portfolio-inward1-blend1-carry2-','');text('current-version',short||'未知版本');$('current-version').title=full;text('current-commit',state.commit?'提交 '+state.commit.slice(0,10):'')}"
        "if(state.goal){text('workspace','当前目标：'+state.goal.summary+(state.goal.runnable?'':' · 本版本规划器暂不支持，不能运行'));if(goal.loadedDigest&&goal.loadedDigest!==state.goal.digest&&!goal.busy)goalLoad(true)}")
t = one(t, " $('run').disabled=!!blocked||!state.prepared||!$('ready').checked;",
        " $('run').disabled=!!blocked||!$('ready').checked||!!(state.goal&&!state.goal.runnable);")
goal_js = Path(sys.argv[1]).read_text()
t = one(t, "refresh();setInterval(refresh,1200);", goal_js + "refresh();setInterval(refresh,1200);")
t = one(t, "cell(tr,row.id+' · '+row.layout);", "cell(tr,row.id+' · '+row.layout+(row.goal_summary?' · 目标 '+row.goal_summary+(row.goal_used_by_planner===false?'（规划未使用该目标！）':''):''));")
h2=h.read_text()
h2=one(h2,'<p class="muted">连接仅打开只读反馈和三路相机。Prepare 使用 XPolicy 原生回零和闭爪，不经过模型。</p>',
       '<p class="muted">连接仅打开只读反馈和三路相机。Prepare 使用 XPolicy 原生回零和闭爪，不经过模型；它是单独的操作，不是执行任务的前提。</p>')
h2=one(h2,'<div class="notice">回到 Prepare 会让双臂回零并闭合夹爪；',
       '<div class="notice">执行任务需要机械臂已在零位、夹爪空载闭合；不在零位时请先点“回到 Prepare”，否则本轮会在开始检查时停止，不会动作。回到 Prepare 会让双臂回零并闭合夹爪；')
h.write_text(h2)
j.write_text(t)
print('ui patched')
