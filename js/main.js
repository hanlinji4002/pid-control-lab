/** 应用状态、预设、侧栏联动、页签切换 */
import { t, lang, setLang, applyStatic } from './i18n.js';
import { arduinoToStandard, standardToArduino } from './core/pid.js';

// ───────────────────────── 预设工况 ─────────────────────────
// 每个预设都对应 GitHub 上一段真实代码，参数不是编出来的。

export const PRESETS = {
  arduinoLine: {
    label: 'Arduino 2WD 循线（Kp .6 / Kd 6）',
    labelEn: 'Arduino 2WD line follower',
    src: '2WD-Arduino-project-for-B37VR / 01_line_follower_pid.ino',
    gains: { kp: 0.6, ki: 0.0, kd: 6.0 },
    range: { kp: [0, 4, 0.01], ki: [0, 0.5, 0.001], kd: [0, 30, 0.05] },
    // uMax = 80 = BASE_SPEED：.ino 里 leftPWM = base + out，
    // |out| 超过 base 就有一侧轮子反转，前进速度垮掉。差分权限的真实上限在这儿，
    // 不是 MAX_SPEED(150)——那是单轮 PWM 的限幅。
    ctrl: { mode: 'arduino', Ts: 0.01, iMax: 100, uMax: 80, antiWindup: 'clamp',
            dOnMeasurement: false, dSmooth: 0, setpointWeight: 1 },
    plant: { type: 'heading', K: 55, tau: 0.12, wn: 6, zeta: 0.35, deadTime: 0.02,
             deadband: 0, uSat: 80, measNoise: 0.6, quantLSB: 0.4 },
    sim: { tEnd: 3, setpoint: 60 },
    unit: '误差单位（传感器加权，满量程 ±100）', unitEn: 'sensor-weighted error (±100)',
    uUnit: 'PWM', lineProfile: 'arduino',
  },
  robomasterLine: {
    label: 'RoboMaster 巡线（Kp 280 / Kd 30）',
    labelEn: 'RoboMaster line following',
    src: 'Summer-Semester-Project-for-Robomaster-S1 / robot_finalproject.py',
    gains: { kp: 280, ki: 0.0, kd: 30.0 },
    range: { kp: [0, 1200, 1], ki: [0, 400, 1], kd: [0, 200, 0.5] },
    ctrl: { mode: 'standard', Ts: 1 / 30, iMax: 100, uMax: 250, antiWindup: 'clamp',
            dOnMeasurement: false, dSmooth: 0, setpointWeight: 1 },
    plant: { type: 'heading', K: 0.0099, tau: 0.15, wn: 6, zeta: 0.35, deadTime: 0.066,
             deadband: 0, uSat: 250, measNoise: 0.004, quantLSB: 0 },
    sim: { tEnd: 4, setpoint: 0.25 },
    unit: '归一化画面偏差（蓝线中心 − 0.5）', unitEn: 'normalised image error',
    uUnit: 'deg/s', lineProfile: 'robomaster',
  },
  markerAlign: {
    label: 'RoboMaster Marker 对准（Kp 200）',
    labelEn: 'RoboMaster marker alignment',
    src: 'robot_finalproject.py / align_marker()',
    gains: { kp: 200, ki: 0.0, kd: 0.0 },
    range: { kp: [0, 900, 1], ki: [0, 300, 1], kd: [0, 120, 0.5] },
    ctrl: { mode: 'standard', Ts: 0.05, iMax: 100, uMax: 125, antiWindup: 'clamp',
            dOnMeasurement: false, dSmooth: 0, setpointWeight: 1 },
    plant: { type: 'heading', K: 0.01667, tau: 0.15, wn: 6, zeta: 0.35, deadTime: 0.10,
             deadband: 0, uSat: 125, measNoise: 0.003, quantLSB: 0 },
    sim: { tEnd: 5, setpoint: 0.15 },
    unit: 'Marker 归一化 x 偏差', unitEn: 'normalised marker x offset',
    uUnit: 'deg/s', lineProfile: 'robomaster',
  },
  textbook: {
    label: '教科书二阶系统（ζ = 0.35）',
    labelEn: 'Textbook 2nd-order plant',
    src: '—',
    gains: { kp: 1, ki: 0, kd: 0 },
    range: { kp: [0, 20, 0.01], ki: [0, 60, 0.05], kd: [0, 4, 0.005] },
    ctrl: { mode: 'standard', Ts: 0.005, iMax: 1000, uMax: 50, antiWindup: 'clamp',
            dOnMeasurement: false, dSmooth: 0, setpointWeight: 1 },
    plant: { type: 'generic', K: 1, tau: 0.2, wn: 6, zeta: 0.35, deadTime: 0.03,
             deadband: 0, uSat: 50, measNoise: 0, quantLSB: 0 },
    sim: { tEnd: 4, setpoint: 1 },
    unit: '归一化输出', unitEn: 'normalised output', uUnit: '', lineProfile: 'arduino',
  },
};

// ───────────────────────── 状态与事件 ─────────────────────────

const listeners = new Map();
export const bus = {
  on(evt, fn) {
    if (!listeners.has(evt)) listeners.set(evt, []);
    listeners.get(evt).push(fn);
  },
  emit(evt, payload) { (listeners.get(evt) || []).forEach((f) => f(payload)); },
};

export const state = {
  presetKey: 'arduinoLine',
  gains: {}, ctrl: {}, plant: {}, sim: {},
  get preset() { return PRESETS[this.presetKey]; },
  get pidCfg() {
    return {
      ...this.gains, mode: this.ctrl.mode, iMax: this.ctrl.iMax,
      uMin: -this.ctrl.uMax, uMax: this.ctrl.uMax,
      antiWindup: this.ctrl.antiWindup, dOnMeasurement: this.ctrl.dOnMeasurement,
      dSmooth: this.ctrl.dSmooth, setpointWeight: this.ctrl.setpointWeight,
    };
  },
  get simOpts() {
    return { tEnd: this.sim.tEnd, Ts: this.ctrl.Ts, setpoint: this.sim.setpoint,
             preTime: Math.min(0.2, this.sim.tEnd * 0.05) };
  },
};

export function loadPreset(key, silent = false) {
  const p = PRESETS[key];
  state.presetKey = key;
  state.gains = { ...p.gains };
  state.ctrl = { ...p.ctrl };
  state.plant = { ...p.plant };
  state.sim = { ...p.sim };
  syncSidebar();
  if (!silent) bus.emit('change', 'preset');
}

export function setGains(g, from) {
  Object.assign(state.gains, g);
  syncGainInputs();
  bus.emit('change', from || 'gains');
}

// ───────────────────────── 侧栏 ─────────────────────────

const $ = (s) => document.querySelector(s);
const GAIN_KEYS = ['kp', 'ki', 'kd'];

function syncGainInputs() {
  for (const k of GAIN_KEYS) {
    const [lo, hi, step] = state.preset.range[k];
    const rng = $(`#${k}-rng`), num = $(`#${k}-num`);
    rng.min = lo; rng.max = Math.max(hi, state.gains[k] * 1.2); rng.step = step;
    rng.value = state.gains[k];
    num.value = +state.gains[k].toPrecision(6);
    num.step = step;
  }
  updateConversion();
}

function syncSidebar() {
  syncGainInputs();
  $('#pid-mode').value = state.ctrl.mode;
  $('#ts').value = +state.ctrl.Ts.toFixed(5);
  $('#imax').value = state.ctrl.iMax;
  $('#antiwindup').value = state.ctrl.antiWindup;
  $('#umax').value = state.ctrl.uMax;
  $('#d-on-meas').checked = state.ctrl.dOnMeasurement;
  $('#dsmooth').value = state.ctrl.dSmooth;
  $('#spweight').value = state.ctrl.setpointWeight;
  $('#preset').value = state.presetKey;
  updateModeNote();
  bus.emit('sidebar');
}

function updateModeNote() {
  $('#mode-note').innerHTML = state.ctrl.mode === 'arduino'
    ? `<span style="color:var(--warn)">${t('ctrl.noteIno')}</span>` : t('ctrl.noteStd');
}

function updateConversion() {
  const Ts = state.ctrl.Ts;
  const isIno = state.ctrl.mode === 'arduino';
  const std = isIno ? arduinoToStandard(state.gains, Ts) : state.gains;
  const ino = isIno ? state.gains : standardToArduino(state.gains, Ts);
  const f = (v) => (Math.abs(v) >= 1000 || (v !== 0 && Math.abs(v) < 0.001)
    ? v.toExponential(2) : +v.toPrecision(4));
  $('#conv-table').innerHTML = `
    <tr><td>Ts</td><td>${(Ts * 1000).toFixed(1)} ms (${(1 / Ts).toFixed(0)} Hz)</td></tr>
    <tr><td>标准 Kp / Ki / Kd</td><td>${f(std.kp)} / ${f(std.ki)} / ${f(std.kd)}</td></tr>
    <tr><td>.ino Kp / Ki / Kd</td><td>${f(ino.kp)} / ${f(ino.ki)} / ${f(ino.kd)}</td></tr>`;
}

function bindSidebar() {
  for (const k of GAIN_KEYS) {
    const rng = $(`#${k}-rng`), num = $(`#${k}-num`);
    rng.addEventListener('input', () => {
      state.gains[k] = parseFloat(rng.value);
      num.value = +state.gains[k].toPrecision(6);
      updateConversion();
      bus.emit('change', 'gains');
    });
    num.addEventListener('change', () => {
      const v = parseFloat(num.value);
      if (!Number.isFinite(v)) return;
      state.gains[k] = v;
      if (v > +rng.max) rng.max = v * 1.2;
      rng.value = v;
      updateConversion();
      bus.emit('change', 'gains');
    });
  }

  const ctrlBind = [
    ['#pid-mode', 'mode', (e) => e.value],
    ['#ts', 'Ts', (e) => Math.max(0.0005, parseFloat(e.value) || 0.01)],
    ['#imax', 'iMax', (e) => parseFloat(e.value)],
    ['#antiwindup', 'antiWindup', (e) => e.value],
    ['#umax', 'uMax', (e) => parseFloat(e.value)],
    ['#d-on-meas', 'dOnMeasurement', (e) => e.checked],
    ['#dsmooth', 'dSmooth', (e) => parseFloat(e.value)],
    ['#spweight', 'setpointWeight', (e) => parseFloat(e.value)],
  ];
  for (const [sel, key, get] of ctrlBind) {
    $(sel).addEventListener('input', (ev) => {
      state.ctrl[key] = get(ev.target);
      if (key === 'mode') updateModeNote();
      updateConversion();
      bus.emit('change', 'ctrl');
    });
  }

  $('#btn-reset-gains').addEventListener('click', () => {
    state.gains = { ...state.preset.gains };
    syncGainInputs(); bus.emit('change', 'gains');
  });
  $('#btn-zero-id').addEventListener('click', () => setGains({ ki: 0, kd: 0 }));

  const sel = $('#preset');
  sel.innerHTML = Object.entries(PRESETS)
    .map(([k, p]) => `<option value="${k}">${lang === 'en' ? p.labelEn : p.label}</option>`).join('');
  sel.addEventListener('change', () => loadPreset(sel.value));
}

// ───────────────────────── 页签 ─────────────────────────

function bindTabs() {
  const help = $('#help');
  document.querySelectorAll('.tab').forEach((tab) => {
    tab.addEventListener('click', () => {
      document.querySelectorAll('.tab').forEach((x) => x.classList.remove('active'));
      document.querySelectorAll('.tabpane').forEach((x) => x.classList.remove('active'));
      tab.classList.add('active');
      $(`#pane-${tab.dataset.tab}`).classList.add('active');
      help.classList.remove('shown');
      document.querySelector('.tabs').style.display = '';
      bus.emit('tab', tab.dataset.tab);
    });
  });
  // 说明页独占主区，否则会挂在当前页签下面，看着像页面串了
  $('#btn-help').addEventListener('click', (e) => {
    e.preventDefault();
    document.querySelectorAll('.tabpane').forEach((x) => x.classList.remove('active'));
    help.classList.add('shown');
    window.scrollTo({ top: 0, behavior: 'smooth' });
  });
}

// ───────────────────────── 小工具 ─────────────────────────

export function toast(msg, ms = 1800) {
  const el = $('#toast');
  el.textContent = msg; el.hidden = false;
  clearTimeout(el._t);
  el._t = setTimeout(() => { el.hidden = true; }, ms);
}

export function metricCard(key, value, extra, cls) {
  return `<div class="metric ${cls || ''}"><div class="k">${t(key, key)}</div>
    <div class="v">${value}</div>${extra ? `<div class="x">${extra}</div>` : ''}</div>`;
}

export function el(tag, attrs = {}, ...kids) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') n.className = v;
    else if (k.startsWith('on')) n.addEventListener(k.slice(2).toLowerCase(), v);
    else n.setAttribute(k, v);
  }
  kids.flat().forEach((c) => n.append(c?.nodeType ? c : document.createTextNode(String(c))));
  return n;
}

/** 生成可直接贴回真机的代码片段 */
export function codeSnippet(gains, ctrl) {
  const Ts = ctrl.Ts;
  const ino = ctrl.mode === 'arduino' ? gains : standardToArduino(gains, Ts);
  const std = ctrl.mode === 'arduino' ? arduinoToStandard(gains, Ts) : gains;
  const n = (v) => (Math.abs(v) >= 1e4 || (v !== 0 && Math.abs(v) < 1e-3)
    ? v.toExponential(3) : (+v.toPrecision(4)).toString());
  const hz = (1 / Ts).toFixed(0);
  const en = lang === 'en';
  const c1 = en ? '// ── 01_line_follower_pid.ino · tuning block ──'
                : '// ── 01_line_follower_pid.ino · 调参区 ──';
  const c2 = en ? `// For a ${(Ts * 1000).toFixed(1)} ms loop (${hz} Hz). Change the loop rate and these must be recomputed.`
                : `// 对应循环周期 ${(Ts * 1000).toFixed(1)} ms (${hz} Hz)；改了循环频率必须重算`;
  const c3 = en ? '# ── robot_finalproject.py · parameter block ──'
                : '# ── robot_finalproject.py · 参数配置区 ──';
  const c4 = en ? '# PID.calculate() uses dt, so these are sample-rate independent'
                : '# PID.calculate() 用 dt，增益与循环频率无关';
  return `<span class="c">${c1}</span>
<span class="c">${c2}</span>
const float Kp = <span class="n">${n(ino.kp)}f</span>;
const float Ki = <span class="n">${n(ino.ki)}f</span>;
const float Kd = <span class="n">${n(ino.kd)}f</span>;
const float I_MAX = <span class="n">${n(ctrl.iMax)}f</span>;
const int   MAX_SPEED = <span class="n">${Math.round(ctrl.uMax)}</span>;

<span class="c">${c3}</span>
<span class="c">${c4}</span>
KP_LINE = <span class="n">${n(std.kp)}</span>
KI_LINE = <span class="n">${n(std.ki)}</span>
KD_LINE = <span class="n">${n(std.kd)}</span>
Z_MAX   = <span class="n">${n(ctrl.uMax)}</span>
LOOP_HZ = <span class="n">${hz}</span>`;
}

export function plainCode(gains, ctrl) {
  return codeSnippet(gains, ctrl).replace(/<[^>]+>/g, '');
}

// ───────────────────────── 启动 ─────────────────────────

function bindLang() {
  const btn = $('#btn-lang');
  const paint = () => { btn.textContent = lang === 'zh' ? 'EN' : '中'; };
  paint();
  btn.addEventListener('click', () => {
    setLang(lang === 'zh' ? 'en' : 'zh');
    paint();
    const sel = $('#preset');
    sel.innerHTML = Object.entries(PRESETS)
      .map(([k, p]) => `<option value="${k}">${lang === 'en' ? p.labelEn : p.label}</option>`).join('');
    sel.value = state.presetKey;
    updateModeNote(); updateConversion();
    bus.emit('lang');
    bus.emit('change', 'lang');
  });
}

async function boot() {
  applyStatic();
  bindSidebar();
  bindTabs();
  bindLang();
  loadPreset('arduinoLine', true);

  const mods = await Promise.all([
    import('./ui/step.js'), import('./ui/line.js'),
    import('./ui/replay.js'), import('./ui/tune.js'), import('./ui/help.js'),
  ]);
  mods.forEach((m) => m.init?.());
  bus.emit('change', 'boot');
}

document.addEventListener('DOMContentLoaded', boot);
