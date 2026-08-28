/** 日志回放 + 系统辨识 + "换参数重跑"离线重仿真 */
import { state, bus, metricCard, toast, setGains } from '../main.js';
import { t, lang } from '../i18n.js';
import { parseLog } from '../core/logparse.js';
import { identifyARX } from '../core/ident.js';
import { PID } from '../core/pid.js';
import { Chart } from '../chart.js';
import { fmt } from '../core/metrics.js';

const $ = (s) => document.querySelector(s);
let log = null, rawText = '', model = null, disturb = null, seg = null;
let chLog, chResim, playing = false, playT = 0, raf = 0, lastWall = 0;

const CH_STYLE = {
  error: { color: '#ff5f5f', name: '误差 error', nameEn: 'error', width: 1.8 },
  u:     { color: '#f0b429', name: '控制量 u', nameEn: 'control u', width: 1.6 },
  y:     { color: '#4da3ff', name: '输出 y', nameEn: 'output y', width: 1.6 },
  linePos: { color: '#35d0d8', name: '线中心', nameEn: 'line centre', width: 1.4 },
  nL: { color: '#b07cff', name: 'normL', nameEn: 'normL', width: 1.1, dash: [3, 3] },
  nM: { color: '#7f8fa6', name: 'normM', nameEn: 'normM', width: 1.1, dash: [3, 3] },
  nR: { color: '#ff7ab6', name: 'normR', nameEn: 'normR', width: 1.1, dash: [3, 3] },
  pwmL: { color: '#3fd68c', name: 'PWM L', nameEn: 'PWM L', width: 1.2 },
  pwmR: { color: '#f0b429', name: 'PWM R', nameEn: 'PWM R', width: 1.2, dash: [4, 3] },
  v: { color: '#3fd68c', name: '速度 v', nameEn: 'speed v', width: 1.3 },
};

export function init() {
  chLog = new Chart($('#chart-replay'), { xLabel: 't (s)', unitX: 't' });
  chResim = new Chart($('#chart-resim'), { xLabel: 't (s)', unitX: 't' });
  chLog.enableLegendToggle();

  const dz = $('#dropzone'), fi = $('#file-input');
  dz.addEventListener('click', (e) => { if (e.target.tagName !== 'BUTTON') fi.click(); });
  fi.addEventListener('change', () => fi.files[0] && readFile(fi.files[0]));
  ['dragenter', 'dragover'].forEach((ev) =>
    dz.addEventListener(ev, (e) => { e.preventDefault(); dz.classList.add('over'); }));
  ['dragleave', 'drop'].forEach((ev) =>
    dz.addEventListener(ev, (e) => { e.preventDefault(); dz.classList.remove('over'); }));
  dz.addEventListener('drop', (e) => {
    const f = e.dataTransfer.files[0];
    if (f) readFile(f);
  });
  dz.querySelectorAll('[data-sample]').forEach((b) =>
    b.addEventListener('click', (e) => { e.stopPropagation(); loadSample(b.dataset.sample); }));

  $('#btn-play').addEventListener('click', togglePlay);
  $('#btn-rewind').addEventListener('click', () => { playT = 0; paintPlayhead(); });
  $('#scrub').addEventListener('input', (e) => {
    if (!log) return;
    playT = (e.target.value / 100) * log.meta.duration;
    paintPlayhead();
  });
  $('#btn-reparse').addEventListener('click', () => rawText && parse(rawText));
  $('#btn-ident').addEventListener('click', doIdentify);
  $('#ident-integrator').addEventListener('change', () => { if (model) doIdentify(); });
  $('#btn-resim').addEventListener('click', () => resim(state.gains, true));
  $('#btn-resim-tune').addEventListener('click', tuneOnLog);

  bus.on('change', (w) => { if (model && (w === 'gains' || w === 'ctrl')) resim(state.gains, false); });
  bus.on('lang', () => { if (log) { paintMeta(); paintLog(); } });
}

async function loadSample(path) {
  try {
    const r = await fetch(path);
    if (!r.ok) throw new Error(r.status + ' ' + r.statusText);
    parse(await r.text(), path.split('/').pop());
  } catch (err) {
    toast(t('msg.parseFail') + err.message + '（本地直接打开 file:// 会被浏览器拦截，用 python3 -m http.server）', 5200);
  }
}

function readFile(file) {
  const fr = new FileReader();
  fr.onload = () => parse(fr.result, file.name);
  fr.readAsText(file);
}

function parse(text, name) {
  rawText = text;
  try {
    log = parseLog(text, { defaultTs: (parseFloat($('#replay-ts').value) || 200) / 1000 });
  } catch (err) {
    toast(t('msg.parseFail') + err.message, 4000);
    return;
  }
  model = null; disturb = null; seg = null;
  $('#resim-box').hidden = true;
  $('#ident-info').textContent = t('replay.identNone');
  $('#replay-body').hidden = false;
  $('#dropzone').classList.add('loaded');
  playT = 0;
  paintMeta(name);
  paintLog();
  paintPlayhead();
  toast(t('msg.loaded') + (name || '') + ` · ${log.meta.samples} ${lang === 'en' ? 'samples' : '个样本'}`);
}

function paintMeta(name) {
  const m = log.meta;
  $('#replay-meta').innerHTML = [
    metricCard('m.format', log.format, name || ''),
    metricCard('m.samples', String(m.samples)),
    metricCard('m.duration', m.duration.toFixed(2) + ' s'),
    metricCard('m.ts', (m.Ts * 1000).toFixed(1) + ' ms', (1 / m.Ts).toFixed(0) + ' Hz'),
    metricCard('m.channels', String(m.channels.length), m.channels.join(' ')),
  ].join('');
  $('#replay-warn').innerHTML = log.warnings.map((w) => `<div class="warnbox">${w}</div>`).join('');
}

function paintLog() {
  const series = [];
  for (const [key, arr] of Object.entries(log.ch)) {
    const st = CH_STYLE[key] || { color: '#7f8fa6', name: key, nameEn: key, width: 1.2 };
    series.push({
      name: lang === 'en' ? (st.nameEn || key) : (st.name || key),
      x: log.t, y: arr, color: st.color, width: st.width, dash: st.dash,
      hidden: ['pwmL', 'pwmR', 'nM', 'v'].includes(key),
    });
  }
  chLog.setSeries(series).draw();
}

function paintPlayhead() {
  if (!log) return;
  $('#scrub').value = (playT / Math.max(log.meta.duration, 1e-9)) * 100;
  $('#replay-time').textContent = playT.toFixed(2) + ' s';
  const i = Math.min(log.t.length - 1, Math.max(0, Math.round(playT / log.meta.Ts)));
  const st = log.states[i];
  $('#replay-state').textContent = st ? `state = ${st}` : '';
  chLog.setVLines([{ x: playT, color: '#3fd68c', dash: [], label: '' }]).draw();
}

function togglePlay() {
  playing = !playing;
  $('#btn-play').textContent = playing ? t('replay.pause') : t('replay.play');
  if (playing) { lastWall = performance.now(); raf = requestAnimationFrame(tick); }
  else cancelAnimationFrame(raf);
}

function tick(now) {
  if (!playing || !log) return;
  playT += (now - lastWall) / 1000;
  lastWall = now;
  if (playT >= log.meta.duration) { playT = 0; }
  paintPlayhead();
  raf = requestAnimationFrame(tick);
}

// ───────────────────── 辨识 ─────────────────────

function doIdentify() {
  if (!log) return;
  const u = log.ch.u, y = log.ch.error ?? log.ch.y;
  if (!u || !y) { toast(t('msg.needU'), 3500); return; }

  // 只用连续跟线的区间：丢线搜索段是开环全力打舵，会把模型带歪
  const mask = log.states.map((s) => !/SEARCH|LOST|WAIT|ALIGN/.test(s));
  seg = longestRun(mask);
  const U = [], Y = [];
  for (let i = seg.start; i < seg.end; i++) {
    if (Number.isFinite(u[i]) && Number.isFinite(y[i])) { U.push(u[i]); Y.push(y[i]); }
  }
  const integrator = $('#ident-integrator').checked;
  const m = identifyARX(U, Y, log.meta.Ts, { integrator });
  if (!m || m.r2 < 0.2) { toast(t('msg.identFail'), 4000); return; }
  model = m;

  // 残差 = 赛道曲率等外部扰动，重仿真时原样复现，才谈得上"同一段路"。
  // 只在辨识用的那一段里算：丢线搜索段是开环打舵，残差没有物理含义。
  disturb = new Float64Array(log.t.length);
  for (let k = seg.start + 2; k < seg.end; k++) {
    const pred = -m.a1 * y[k - 1] - m.a2 * y[k - 2] + m.b1 * u[k - 1] + m.b2 * u[k - 2];
    disturb[k] = Number.isFinite(y[k] - pred) ? y[k] - pred : 0;
  }

  const dyn = m.oscillatory
    ? `ωn=${fmt(m.wn, 2)} rad/s ζ=${fmt(m.zeta, 3)}`
    : `τ=${m.tau ? fmt(m.tau, 3) + ' s' : '—'}`;
  const gain = m.integrator ? `Kv=${fmt(m.Kv, 4)}/s` : `K=${fmt(m.K, 4)}`;
  $('#ident-info').innerHTML =
    `R²=<b>${m.r2.toFixed(3)}</b> · ${m.integrator ? '1/s · ' : ''}${dyn} · ${gain} · ` +
    `${lang === 'en' ? 'segment' : '用段'} ${(seg.start * log.meta.Ts).toFixed(1)}–${(seg.end * log.meta.Ts).toFixed(1)}s ` +
    `(${seg.end - seg.start} ${lang === 'en' ? 'samples' : '点'})` +
    (m.stable ? '' : ` · <span style="color:var(--warn)">${lang === 'en' ? 'model marginally stable' : '模型临界稳定'}</span>`);
  $('#resim-box').hidden = false;
  resim(state.gains, true);
}

function longestRun(mask) {
  let best = { start: 0, end: mask.length }, s = -1, bl = 0;
  for (let i = 0; i <= mask.length; i++) {
    if (i < mask.length && mask[i]) { if (s < 0) s = i; }
    else if (s >= 0) { if (i - s > bl) { bl = i - s; best = { start: s, end: i }; } s = -1; }
  }
  return bl > 30 ? best : { start: 0, end: mask.length };
}

/**
 * 用辨识模型 + 原扰动序列，换一组增益重跑。
 * 只跑辨识窗口内那一段 —— 模型只在那里成立，拿它外推整条日志是自欺欺人。
 * 采样周期用日志自己的 Ts，不是侧栏里的（回放的是当时那台车）。
 */
function simulateWith(gains) {
  const m = model, Ts = log.meta.Ts;
  const a = seg.start, b = seg.end;
  const rec = log.ch.error ?? log.ch.y;
  const pid = new PID({ ...state.pidCfg, ...gains });
  const t = log.t.slice(a, b);
  const y = new Float64Array(b - a), u = new Float64Array(b - a);
  let y1 = rec[a] ?? 0, y2 = rec[a] ?? 0, u1 = log.ch.u[a] ?? 0, u2 = u1;
  let sat = 0;
  for (let k = a; k < b; k++) {
    const i = k - a;
    const yk = i < 2 ? rec[k] : (-m.a1 * y1 - m.a2 * y2 + m.b1 * u1 + m.b2 * u2 + disturb[k]);
    const step = pid.updateFromError(yk, Ts);
    const uk = step.u;
    if (step.saturated) sat++;
    y[i] = yk; u[i] = uk;
    y2 = y1; y1 = yk; u2 = u1; u1 = uk;
    if (!Number.isFinite(yk) || Math.abs(yk) > 1e6) {   // 发散提前退出
      for (let j = i; j < y.length; j++) { y[j] = NaN; u[j] = NaN; }
      return { t, y, u, satFrac: sat / (b - a), diverged: true };
    }
  }
  return { t, y, u, satFrac: sat / (b - a), diverged: false };
}

function rms(arr) {
  let s = 0, n = 0;
  for (const v of arr) if (Number.isFinite(v)) { s += v * v; n++; }
  return n ? Math.sqrt(s / n) : NaN;
}

function resim(gains, announce) {
  if (!model) return;
  const r = simulateWith(gains);
  const recFull = log.ch.error ?? log.ch.y;
  const rec = recFull.slice(seg.start, seg.end);
  const rRec = rms(rec), rNew = rms(r.y);
  const better = (1 - rNew / rRec) * 100;

  $('#resim-info').innerHTML = r.diverged
    ? `<span style="color:var(--danger)">${t('msg.unstable')}</span>`
    : `${lang === 'en' ? 'error RMS' : '误差 RMS'} ${fmt(rRec, 3)} → <b>${fmt(rNew, 3)}</b> ` +
      `<span style="color:${better > 0 ? 'var(--ok)' : 'var(--danger)'}">` +
      `${better > 0 ? '↓' : '↑'} ${Math.abs(better).toFixed(1)}%</span>` +
      `<span class="dim"> · ${lang === 'en' ? 'saturated' : '饱和'} ${(r.satFrac * 100).toFixed(0)}%` +
      ` · ${lang === 'en' ? 'verify on the Line-follower tab' : '记得回“循线小车”页验一遍'}</span>`;

  chResim.setSeries([
    { name: lang === 'en' ? 'recorded' : '真机记录', x: r.t, y: rec, color: '#7f8fa6', width: 1.5, dash: [4, 3] },
    { name: `${lang === 'en' ? 'resim' : '重仿真'} Kp${+gains.kp.toPrecision(3)} Ki${+gains.ki.toPrecision(3)} Kd${+gains.kd.toPrecision(3)}`,
      x: r.t, y: r.y, color: '#4da3ff', width: 1.9 },
  ]).draw();
  if (announce && !r.diverged) toast(`${lang === 'en' ? 'Re-simulated' : '重仿真完成'} · RMS ${fmt(rNew, 3)}`);
}

function tuneOnLog() {
  if (!model) return;
  $('#resim-info').textContent = t('msg.tuning');
  setTimeout(() => {
    const cost = (g) => {
      if (g.kp < 0 || g.ki < 0 || g.kd < 0) return 1e9;
      const r = simulateWith(g);
      if (r.diverged) return 1e9;
      const e = rms(r.y), du = rms(diff(r.u));
      if (!Number.isFinite(e)) return 1e9;
      // 饱和惩罚：辨识出来的是单入单出航向模型，它看不见"执行器打满 → 前进速度垮掉"。
      // 不罚饱和，优化器就会推出一组横向偏差好看、实际却在原地扭的参数。
      const satPen = (r.satFrac || 0) ** 2 * 120;
      return e + 0.02 * du / Math.max(state.ctrl.uMax, 1) * 100 + satPen;
    };
    const base = state.gains;
    const starts = [base,
      { kp: base.kp * 2, ki: base.ki, kd: base.kd },
      { kp: base.kp * 0.5, ki: base.ki, kd: base.kd * 2 }];
    const bounds = { kp: [0, base.kp * 8 + 1], ki: [0, base.kp * 4 + 1], kd: [0, base.kd * 8 + base.kp + 1] };
    let best = null;
    for (const s of starts) {
      const r = simpleSearch(cost, s, bounds);
      if (!best || r.cost < best.cost) best = r;
    }
    setGains(best.gains, 'gains');
    resim(best.gains, false);
    toast(`${lang === 'en' ? 'Tuned on this log' : '基于该日志整定完成'} · Kp ${+best.gains.kp.toPrecision(4)} / Ki ${+best.gains.ki.toPrecision(4)} / Kd ${+best.gains.kd.toPrecision(4)}`, 4200);
  }, 30);
}

function diff(a) {
  const o = new Float64Array(Math.max(0, a.length - 1));
  for (let i = 1; i < a.length; i++) o[i - 1] = a[i] - a[i - 1];
  return o;
}

/** 坐标轮换 + 步长收缩：够用、快、不会跑到边界外 */
function simpleSearch(f, x0, b) {
  let g = { kp: x0.kp, ki: x0.ki, kd: x0.kd };
  let best = f(g);
  const keys = ['kp', 'ki', 'kd'];
  let step = { kp: Math.max(b.kp[1] * 0.12, 1e-4), ki: Math.max(b.ki[1] * 0.12, 1e-5), kd: Math.max(b.kd[1] * 0.12, 1e-4) };
  for (let round = 0; round < 26; round++) {
    let improved = false;
    for (const k of keys) {
      for (const sgn of [1, -1]) {
        const cand = { ...g, [k]: Math.min(b[k][1], Math.max(b[k][0], g[k] + sgn * step[k])) };
        const c = f(cand);
        if (c < best - 1e-12) { best = c; g = cand; improved = true; }
      }
    }
    if (!improved) for (const k of keys) step[k] *= 0.55;
  }
  return { gains: g, cost: best };
}
