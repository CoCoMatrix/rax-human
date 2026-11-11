'use strict';
// Operator dashboard client. Same-origin fetches carry the HttpOnly token cookie.

const $ = (id) => document.getElementById(id);
const state = { sessions: [], selected: localStorage.getItem('selected') || null, tab: 'transcript', frameId: null };

function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (value === null || value === undefined || value === false) continue;
    if (key === 'class') node.className = value;
    else if (key.startsWith('on')) node.addEventListener(key.slice(2), value);
    else node.setAttribute(key, value);
  }
  for (const child of children.flat()) {
    if (child !== null && child !== undefined) node.append(child instanceof Node ? child : String(child));
  }
  return node;
}

async function copyText(text) {
  try {
    await navigator.clipboard.writeText(text);  // 127.0.0.1 counts as a secure context
  } catch {
    const area = el('textarea', { style: 'position:fixed;opacity:0' }, text);
    document.body.append(area);
    area.select();
    document.execCommand('copy');
    area.remove();
  }
  toast('Copied to the clipboard');
}

function commandBlock(label, text, primary = false) {
  return el('div', { class: 'command' },
    el('label', {}, label),
    el('div', { class: 'row', style: 'align-items:stretch;gap:6px;flex-wrap:nowrap' },
      el('pre', { style: 'flex:1' }, text),
      el('button', { class: primary ? 'primary' : null, title: 'Copy command', onclick: () => copyText(text) }, 'Copy')));
}

function toast(message, bad = false) {
  const box = $('toast');
  box.textContent = message;
  box.style.background = bad ? 'var(--bad)' : '';
  box.style.display = 'block';
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => { box.style.display = 'none'; }, bad ? 9000 : 4000);
}

async function api(path, body) {
  const options = body === undefined ? {} :
    { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
  const response = await fetch(path, options);
  const value = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(value.error || `${response.status}`);
  return value;
}

const when = (t) => (t ? new Date(t * 1000).toLocaleString() : '-');
const fmt = (n) => (typeof n === 'number' ? n.toLocaleString() : n);

function badgeFor(status, running) {
  if (running) return el('span', { class: 'badge ok' }, status);
  if (/failed|gone/.test(status)) return el('span', { class: 'badge bad' }, status);
  if (/usage_wait/.test(status)) return el('span', { class: 'badge warn' }, status);
  return el('span', { class: 'badge' }, status);
}

function renderSessions(value) {
  state.sessions = value.sessions;
  $('running-badge').textContent = value.running.length ? `running: ${value.running.join(', ')}` : 'no session running';
  $('running-badge').className = value.running.length ? 'badge ok' : 'badge';
  const list = $('sessions');
  list.replaceChildren();
  if (!value.sessions.length) list.append(el('p', { class: 'muted' }, 'No sessions yet.'));
  for (const s of value.sessions) {
    list.append(el('div', {
      class: 'session' + (s.id === state.selected ? ' selected' : ''),
      onclick: () => { state.selected = s.id; localStorage.setItem('selected', s.id); state.frameId = null; refresh(); },
    },
      el('div', { class: 'row' }, el('strong', {}, s.task), badgeFor(s.status, s.running),
        s.motion.armed ? el('span', { class: 'badge bad' }, 'ARMED') : null),
      el('div', { class: 'muted' }, `${s.id} · ${s.effort} · ${fmt(s.usage.calls)} calls`)));
  }
  renderDetail();
}

function renderDetail() {
  const s = state.sessions.find((x) => x.id === state.selected);
  const detail = $('detail');
  for (const id of ['safety', 'cameras-section', 'activity-section']) $(id).hidden = !s;
  if (!s) { detail.replaceChildren(el('h2', {}, 'Session'), el('p', { class: 'muted' }, 'Select a session.')); return; }
  const anyRunning = state.sessions.some((x) => x.running);
  const launched = !['configured'].includes(s.status);
  const u = s.usage;
  detail.replaceChildren(
    el('h2', {}, `Session ${s.task}`),
    el('table', { class: 'kv' },
      el('tr', {}, el('td', {}, 'id'), el('td', {}, el('code', {}, s.id))),
      el('tr', {}, el('td', {}, 'status'), el('td', {}, badgeFor(s.status, s.running),
        s.returncode !== undefined && s.returncode !== null ? ` returncode ${s.returncode}` : '',
        s.process ? ` pid ${s.process.pid} (${s.process.source})` : '')),
      el('tr', {}, el('td', {}, 'model'), el('td', {}, `${s.model} · ${s.provider} · effort ${s.effort}`)),
      el('tr', {}, el('td', {}, 'budget'), el('td', {}, `${s.session_hours} h · ${s.observation_profile} · attempts ${s.attempts}`)),
      el('tr', {}, el('td', {}, 'times'), el('td', {}, `created ${when(s.created)} · started ${when(s.started_at)} · finished ${when(s.finished_at)}`)),
      el('tr', {}, el('td', {}, 'usage'), el('td', {},
        `${fmt(u.calls)} calls · in ${fmt(u.input_tokens)} · out ${fmt(u.output_tokens)} · cache read ${fmt(u.cache_read_input_tokens)}` +
        (u.cost_usd ? ` · $${u.cost_usd}` : ''))),
    ),
    el('div', { class: 'row', style: 'margin:10px 0' },
      el('button', { class: 'primary', disabled: (anyRunning || launched) ? '' : null, onclick: () => act('launch') }, 'Launch headless'),
      el('button', { disabled: (anyRunning || !launched) ? '' : null, onclick: () => act('resume') }, 'Resume'),
      el('button', { disabled: s.running ? null : '', onclick: () => {
        if (confirm('Stop this session? The agent and robot service are shut down.')) act('stop');
      } }, 'Stop')),
    commandBlock('Start interactively (paste into a terminal)', s.commands.interactive, true),
    commandBlock('Arm this session (in a terminal; asks you to type ARM)', s.commands.arm),
    commandBlock('Resume interactively', s.commands.resume),
    commandBlock('Headless', s.commands.headless),
  );
  const motion = $('motion-state');
  motion.textContent = s.motion.armed ? `ARMED${s.motion.until ? ' until ' + when(s.motion.until) : ''}` :
    `disarmed: ${s.motion.reason}`;
  motion.className = s.motion.armed ? 'badge bad' : 'badge ok';
}

async function act(action) {
  try {
    await api(`/api/session/${state.selected}/${action}`, {});
    toast(`${action}: ok`);
  } catch (error) { toast(`${action} failed: ${error.message}`, true); }
  refresh();
}

function renderRobot(value) {
  $('robot-badge').textContent = `${value.kind || 'robot'}: ${value.ok ? 'available' : 'unavailable'}`;
  $('robot-badge').className = value.ok ? 'badge ok' : 'badge warn';
  $('robot').replaceChildren(el('pre', {}, JSON.stringify(value, null, 2)));
}

async function refreshFrame() {
  if (!state.selected) return;
  const { frame } = await api(`/api/session/${state.selected}/frame`);
  const box = $('cameras');
  if (!frame) { $('frame-info').textContent = 'No frames yet (the agent captures them with the observation tool).'; box.replaceChildren(); return; }
  $('frame-info').textContent = `${frame.frame_id} · ${when(frame.time)} · ${frame.count} frames in session`;
  if (frame.frame_id === state.frameId) return;
  state.frameId = frame.frame_id;
  box.replaceChildren(...frame.cameras.map((camera) => el('figure', {},
    el('img', { src: `/api/session/${state.selected}/frame/${frame.frame_id}/${camera}.jpg`, alt: camera, loading: 'lazy' }),
    el('figcaption', {}, camera))));
}

function renderActivity(value) {
  const box = $('activity');
  const atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 40;
  if (state.tab === 'transcript') {
    box.replaceChildren(...(value.transcript.length ? value.transcript.map((e) => el('div',
      { class: `event ${e.kind}${e.error ? ' error' : ''}` },
      e.kind === 'tool_call' ? `→ ${e.name} ${e.text}` : e.text)) :
      [el('p', { class: 'muted' }, 'No transcript yet.')]));
  } else if (state.tab === 'mcp') {
    box.replaceChildren(...(value.mcp.length ? value.mcp.map((e) => el('div',
      { class: `event ${e.event === 'mcp_error' ? 'tool_result error' : e.event === 'mcp_request' ? 'tool_call' : 'tool_result'}` },
      `${new Date((e.time || 0) * 1000).toLocaleTimeString()} ${e.event} ${e.tool || ''} ${e.text}`)) :
      [el('p', { class: 'muted' }, 'No robot calls yet.')]));
  } else {
    box.replaceChildren(
      el('label', {}, 'launcher.log'), el('pre', {}, value.launcher_log.join('\n') || '(empty)'),
      el('label', {}, 'frontend.log (robot service)'), el('pre', {}, value.frontend_log.join('\n') || '(empty)'),
      el('label', {}, 'camera-server.log'), el('pre', {}, value.camera_log.join('\n') || '(empty)'));
  }
  if (atBottom) box.scrollTop = box.scrollHeight;
}

async function refreshActivity() {
  if (!state.selected) return;
  renderActivity(await api(`/api/session/${state.selected}/activity`));
}

const seenRequests = new Set();

function beep() {
  try {
    const audio = new AudioContext(), tone = audio.createOscillator();
    tone.connect(audio.destination); tone.frequency.value = 880; tone.start(); tone.stop(audio.currentTime + 0.4);
  } catch { /* audio blocked until the page was clicked */ }
}

function renderRequests(value) {
  const box = $('requests');
  const open = value.sessions.flatMap((s) => (s.requests || []).map((r) => ({ ...r, session: s.id, task: s.task })));
  box.hidden = open.length === 0;
  document.title = open.length ? `(${open.length}) Agent needs you - RAX` : 'RAX operator';
  if (open.some((r) => !seenRequests.has(r.id))) beep();
  open.forEach((r) => seenRequests.add(r.id));
  if (box.contains(document.activeElement)) return;  // do not wipe a reply being typed
  box.replaceChildren(el('h2', {}, `The agent is asking for ${open.length > 1 ? 'you (' + open.length + ')' : 'you'}`),
    ...open.map((r) => {
      const input = el('textarea', { placeholder: r.kind === 'reset' ? 'Optional note, e.g. "both cubes back on the left"' : 'Your answer' });
      const send = async (text) => {
        try { await api(`/api/session/${r.session}/reply`, { request_id: r.id, text, done: true }); toast('Reply sent'); refresh(); }
        catch (error) { toast(error.message, true); }
      };
      return el('div', { class: 'request' },
        el('div', {}, el('b', {}, r.kind === 'reset' ? 'SCENE RESET' : 'HELP'), ` · ${r.task} · ${when(r.created)}`),
        el('pre', {}, r.message), input,
        el('div', { class: 'row', style: 'margin-top:6px' },
          r.kind === 'reset' ? el('button', { class: 'primary', onclick: () => send(input.value || 'Scene reset done.') }, 'Reset done') : null,
          el('button', { class: r.kind === 'reset' ? null : 'primary', onclick: () => send(input.value) }, 'Send reply')));
    }));
}

async function refresh() {
  try {
    const value = await api('/api/sessions');
    renderSessions(value); renderRequests(value);
  } catch (error) { toast(`sessions: ${error.message}`, true); }
  try { await refreshFrame(); await refreshActivity(); } catch (error) { /* a deleted session; ignore */ }
}

async function pollRobot() {
  try { renderRobot(await api('/api/robot')); } catch (error) {
    $('robot-badge').textContent = 'dashboard error'; $('robot-badge').className = 'badge bad';
  }
}

$('new-session').addEventListener('submit', async (event) => {
  event.preventDefault();
  const form = Object.fromEntries(new FormData(event.target).entries());
  try {
    const value = await api('/api/configure', form);
    state.selected = value.id; localStorage.setItem('selected', value.id);
    toast(`created ${value.id}`);
    refresh();
  } catch (error) { toast(`configure failed: ${error.message}`, true); }
});

$('arm').addEventListener('click', async () => {
  if (!$('estop').checked) { toast('Tick the emergency-stop confirmation first.', true); return; }
  if (!confirm('ARM robot motion for this session? The agent can then move the real robot.')) return;
  const minutes = $('arm-minutes').value ? Number($('arm-minutes').value) : null;
  try {
    await api(`/api/session/${state.selected}/arm`, { estop_confirmed: true, minutes });
    $('estop').checked = false;
    toast('motion ARMED');
  } catch (error) { toast(`arm failed: ${error.message}`, true); }
  refresh();
});

$('disarm').addEventListener('click', async () => {
  try {
    const value = await api(`/api/session/${state.selected}/disarm`, {});
    toast(`disarmed; hold: ${value.hold}`, false);
  } catch (error) { toast(`DISARM FAILED: ${error.message}`, true); }
  refresh();
});

$('disarm-all').addEventListener('click', async () => {
  try {
    const value = await api('/api/disarm-all', {});
    toast(`disarmed ${value.disarmed.length} session(s); hold: ${value.hold}`, false);
  } catch (error) { toast(`DISARM ALL FAILED: ${error.message}`, true); }
  refresh();
});

for (const button of document.querySelectorAll('.tabs button')) {
  button.addEventListener('click', () => {
    state.tab = button.dataset.tab;
    for (const b of document.querySelectorAll('.tabs button')) b.classList.toggle('active', b === button);
    refreshActivity().catch(() => {});
  });
}

refresh();
pollRobot();
setInterval(pollRobot, 1000);
setInterval(refresh, 2500);
