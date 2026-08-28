/** 循线小车仿真页 */
import { state, bus, metricCard, toast } from '../main.js';
import { t, lang } from '../i18n.js';
import { LineSim } from '../sim/car.js';
import { TRACK_TYPES } from '../sim/track.js';
import { Chart } from '../chart.js';

const $ = (s) => document.querySelector(s);
let sim, cv, ctx, chCte, chCtrl, running = false, raf = 0, lastWall = 0, speed = 1;

const HW_SPEC = {
  arduino: [
    ['baseSpeed', 'BASE_SPEED', 1, 'PWM 直道基础速度'],
    ['maxSpeed', 'MAX_SPEED', 1, '单侧轮 PWM 上限'],
    ['lostSpeed', 'LOST_SPEED', 1, '丢线搜索速度'],
    ['leftPercent', 'LEFT_MOTOR_PERCENT', 1, '左轮补偿 %'],
    ['rightPercent', 'RIGHT_MOTOR_PERCENT', 1, '右轮补偿 %'],
    ['lineThreshold', 'LINE_THRESHOLD', 0.01, '丢线判据'],
    ['sensorSpacing', '传感器间距 (m)', 0.001, '左右传感器到中间的距离'],
    ['lookahead', '前视距离 (m)', 0.005, '传感器排在车轴前多远'],
    ['motorImbalance', '左右电机差异', 0.01, '0.1 = 相差 10%，用补偿百分比抵消'],
    ['deadbandPWM', '电机死区 PWM', 1, '低于此值轮子不转'],
  ],
  robomaster: [
    ['vBase', 'V_BASE (m/s)', 0.01, '基础前进速度'],
    ['vMin', 'V_MIN (m/s)', 0.01, '偏差大时最低速'],
    ['vMax', 'V_MAX (m/s)', 0.01, '偏差小时最高速'],
    ['zMax', 'Z_MAX (deg/s)', 5, '底盘旋转限幅'],
    ['slowGain', '降速系数', 0.1, '按 |误差| 降速的强度'],
    ['camLookahead', '扫描行落点 (m)', 0.01, '对应 SCAN_ROW_RATIO 与云台俯角'],
    ['camHalfWidth', '视野半宽 (m)', 0.01, '扫描行覆盖的横向范围'],
    ['camDelayFrames', '相机延迟 (帧)', 1, '30 Hz 下每帧 33 ms'],
    ['camNoise', '检测噪声 σ', 0.001, 'HSV 掩码抖动'],
    ['detectFail', '漏检概率', 0.01, '光照突变导致的丢帧'],
  ],
};

export function init() {
  cv = $('#track-cv'); ctx = cv.getContext('2d');
  chCte = new Chart($('#chart-cte'), { xLabel: 't (s)', yLabel: 'mm', unitX: 't' });
  chCtrl = new Chart($('#chart-linectrl'), { xLabel: 't (s)', unitX: 't' });

  $('#line-track').innerHTML = Object.entries(TRACK_TYPES)
    .map(([k, v]) => `<option value="${k}">${lang === 'en' ? v.labelEn : v.label}</option>`).join('');

  makeSim();

  $('#line-profile').addEventListener('change', (e) => {
    const key = e.target.value;
    // 换机器人 = 换一整套真机配置，顺带把预设也切过去
    const presetFor = key === 'arduino' ? 'arduinoLine' : 'robomasterLine';
    import('../main.js').then((M) => M.loadPreset(presetFor));
    makeSim();
  });
  $('#line-track').addEventListener('change', () => makeSim());
  $('#line-adc').addEventListener('change', () => makeSim());
  $('#line-speed').addEventListener('change', (e) => { speed = +e.target.value; });
  $('#btn-run').addEventListener('click', toggle);
  $('#btn-reset-sim').addEventListener('click', () => { makeSim(); });
  $('#show-sensors').addEventListener('change', draw);
  $('#show-path').addEventListener('change', draw);

  bus.on('change', (what) => {
    if (!sim) return;
    if (what === 'gains') sim.setGains(state.gains);
    if (what === 'preset' || what === 'boot') makeSim();
    if (what === 'ctrl') { sim.pid.set({ iMax: state.ctrl.iMax, mode: state.ctrl.mode }); }
  });
  bus.on('lang', () => { buildHW(); paintMetrics(); });
  new ResizeObserver(draw).observe(cv.parentElement);
  draw(); paintMetrics(); buildHW();
}

function profileKey() { return $('#line-profile').value; }

function makeSim() {
  const key = profileKey();
  $('#adc-field').style.display = key === 'arduino' ? '' : 'none';
  const params = key === 'arduino' ? { adc: $('#line-adc').value } : {};
  sim = new LineSim({
    profile: key,
    trackType: $('#line-track').value,
    gains: state.gains,
    params,
  });
  buildHW();
  draw(); paintMetrics(); paintCharts();
}

function toggle() {
  running = !running;
  $('#btn-run').textContent = running ? t('line.pause') : t('line.run');
  $('#btn-run').classList.toggle('primary', !running);
  if (running) { lastWall = performance.now(); raf = requestAnimationFrame(loop); }
  else cancelAnimationFrame(raf);
}

function loop(now) {
  if (!running) return;
  const dtWall = Math.min(0.1, (now - lastWall) / 1000);
  lastWall = now;
  const budget = dtWall * speed;
  let acc = 0, guard = 0;
  while (acc < budget && guard++ < 4000 && !sim.finished) { sim.step(); acc += sim.Ts; }
  if (sim.finished) { toast(t('msg.offtrack')); toggle(); }
  draw(); paintMetrics(); paintCharts();
  raf = requestAnimationFrame(loop);
}

// ───────────────────────── 赛道绘制 ─────────────────────────

function transform() {
  const w = cv.clientWidth, h = cv.clientHeight;
  const b = sim.track.bounds, pad = 0.22;
  const bw = b.maxX - b.minX + pad * 2, bh = b.maxY - b.minY + pad * 2;
  const s = Math.min(w / bw, h / bh);
  const ox = w / 2 - ((b.minX + b.maxX) / 2) * s;
  const oy = h / 2 + ((b.minY + b.maxY) / 2) * s;
  return { s, X: (x) => ox + x * s, Y: (y) => oy - y * s };
}

function draw() {
  if (!sim) return;
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const { s, X, Y } = transform();
  const tr = sim.track;

  // 地面
  ctx.fillStyle = '#0e141c';
  ctx.fillRect(0, 0, w, h);

  // 赛道线
  ctx.strokeStyle = '#e9eef5';
  ctx.lineWidth = Math.max(2, tr.lineWidth * s);
  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  ctx.beginPath();
  tr.pts.forEach((p, i) => (i ? ctx.lineTo(X(p.x), Y(p.y)) : ctx.moveTo(X(p.x), Y(p.y))));
  ctx.closePath(); ctx.stroke();

  // 起跑线
  const n = tr.startNormal;
  ctx.strokeStyle = '#3fd68c'; ctx.lineWidth = 2.5; ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(X(tr.start.x - n.x * 0.09), Y(tr.start.y - n.y * 0.09));
  ctx.lineTo(X(tr.start.x + n.x * 0.09), Y(tr.start.y + n.y * 0.09));
  ctx.stroke(); ctx.setLineDash([]);

  // 行驶轨迹
  if ($('#show-path').checked && sim.trace.path.length > 1) {
    const p = sim.trace.path;
    const stride = Math.max(1, Math.floor(p.length / 3000));
    ctx.lineWidth = 1.6;
    let prevLost = null;
    ctx.beginPath();
    for (let i = 0; i < p.length; i += stride) {
      if (p[i].lost !== prevLost) {
        if (prevLost !== null) ctx.stroke();
        ctx.strokeStyle = p[i].lost ? '#ff5f5f' : '#4da3ff';
        ctx.beginPath(); ctx.moveTo(X(p[i].x), Y(p[i].y));
        prevLost = p[i].lost;
      } else ctx.lineTo(X(p[i].x), Y(p[i].y));
    }
    ctx.stroke();
  }

  drawCar(ctx, X, Y, s);
}

function drawCar(ctx, X, Y, s) {
  const p = sim.profile.params;
  const isIno = sim.profileKey === 'arduino';
  const L = isIno ? 0.16 : 0.24, W = p.wheelBase;
  const c = Math.cos(sim.theta), sn = Math.sin(sim.theta);
  const pt = (fwd, lat) => [X(sim.x + fwd * c - lat * sn), Y(sim.y + fwd * sn + lat * c)];

  // 车体
  ctx.save();
  ctx.beginPath();
  const corners = [[L * 0.55, W / 2], [L * 0.55, -W / 2], [-L * 0.45, -W / 2], [-L * 0.45, W / 2]];
  corners.forEach(([f, l], i) => { const [x, y] = pt(f, l); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.closePath();
  ctx.fillStyle = sim.lineSeen ? 'rgba(77,163,255,.20)' : 'rgba(255,95,95,.22)';
  ctx.strokeStyle = sim.lineSeen ? '#4da3ff' : '#ff5f5f';
  ctx.lineWidth = 1.8; ctx.fill(); ctx.stroke();

  // 车头朝向
  const [hx, hy] = pt(L * 0.55, 0), [tx, ty] = pt(L * 0.1, 0);
  ctx.strokeStyle = '#e3e9f0'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(hx, hy); ctx.stroke();

  // 轮子
  ctx.fillStyle = '#2a3646';
  for (const lat of [W / 2, -W / 2]) {
    const [wx, wy] = pt(0, lat);
    ctx.beginPath(); ctx.arc(wx, wy, Math.max(2.5, 0.022 * s), 0, Math.PI * 2); ctx.fill();
  }

  if ($('#show-sensors').checked) {
    if (isIno) {
      const lats = [+p.sensorSpacing, 0, -p.sensorSpacing];
      sim.sensors.forEach((v, i) => {
        const [sx, sy] = pt(p.lookahead, lats[i]);
        ctx.beginPath(); ctx.arc(sx, sy, Math.max(3, p.spotRadius * s * 1.6), 0, Math.PI * 2);
        ctx.fillStyle = `rgba(240,180,41,${0.18 + 0.8 * v})`;
        ctx.strokeStyle = '#f0b429'; ctx.lineWidth = 1.2;
        ctx.fill(); ctx.stroke();
      });
    } else {
      // 摄像头扫描行
      const [ax, ay] = pt(p.camLookahead, +p.camHalfWidth);
      const [bx, by] = pt(p.camLookahead, -p.camHalfWidth);
      ctx.strokeStyle = 'rgba(53,208,216,.75)'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
      const lat = (0.5 - sim.linePos) * 2 * p.camHalfWidth;
      const [mx, my] = pt(p.camLookahead, lat);
      ctx.fillStyle = sim.lineSeen ? '#35d0d8' : '#ff5f5f';
      ctx.beginPath(); ctx.arc(mx, my, 4, 0, Math.PI * 2); ctx.fill();
      // 视野边界
      ctx.strokeStyle = 'rgba(53,208,216,.22)'; ctx.lineWidth = 1;
      const [ox, oy] = pt(0, 0);
      ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ax, ay);
      ctx.moveTo(ox, oy); ctx.lineTo(bx, by); ctx.stroke();
    }
  }
  ctx.restore();
}

// ───────────────────────── 指标 / 图表 ─────────────────────────

function paintMetrics() {
  if (!sim) return;
  const m = sim.metrics;
  const mm = (v) => (v * 1000).toFixed(1) + ' mm';
  $('#line-metrics').innerHTML = [
    metricCard('m.meanCte', mm(m.meanAbsCte), '', m.meanAbsCte < 0.01 ? 'good' : m.meanAbsCte < 0.03 ? 'warn' : 'bad'),
    metricCard('m.maxCte', mm(m.maxAbsCte), '', m.maxAbsCte < 0.03 ? 'good' : m.maxAbsCte < 0.08 ? 'warn' : 'bad'),
    metricCard('m.lost', String(m.lostCount), m.lostTime > 0 ? m.lostTime.toFixed(1) + ' s' : '',
      m.lostCount === 0 ? 'good' : m.lostCount < 4 ? 'warn' : 'bad'),
    metricCard('m.laps', String(m.laps), `${(sim.lap.progress * 100).toFixed(0)}%`),
    metricCard('m.best', m.bestLap ? m.bestLap.toFixed(2) + ' s' : '—', m.lastLap ? `${t('m.last')} ${m.lastLap.toFixed(2)}s` : ''),
    metricCard('m.dist', m.distance.toFixed(2) + ' m', `${t('m.simtime')} ${m.time.toFixed(1)}s`),
  ].join('');
  $('#line-state').textContent = `${sim.state}  ·  ${(sim.stats.dist / Math.max(sim.t, 1e-9)).toFixed(2)} m/s`;

  const bars = $('#sensor-bars');
  if (sim.profileKey === 'arduino') {
    const names = lang === 'en' ? ['left', 'mid', 'right'] : ['左 L', '中 M', '右 R'];
    bars.innerHTML = sim.sensors.map((v, i) =>
      `<div class="sbar"><span class="lbl">${names[i]}</span>
        <span class="track-bg"><span class="fill" style="width:${(v * 100).toFixed(1)}%"></span></span>
        <span class="val">${v.toFixed(2)}</span></div>`).join('') +
      `<div class="sbar"><span class="lbl">err</span>
        <span class="track-bg"><span class="fill" style="width:${Math.min(100, Math.abs(sim.trace.error.at(-1) || 0))}%"></span></span>
        <span class="val">${(sim.trace.error.at(-1) || 0).toFixed(1)}</span></div>`;
  } else {
    const pos = sim.linePos;
    bars.innerHTML =
      `<div class="sbar"><span class="lbl">line</span>
        <span class="track-bg"><span class="fill" style="width:${(pos * 100).toFixed(1)}%"></span></span>
        <span class="val">${pos.toFixed(3)}</span></div>
       <div class="sbar"><span class="lbl">err</span>
        <span class="track-bg"><span class="fill" style="width:${Math.min(100, Math.abs(pos - 0.5) * 400).toFixed(0)}%"></span></span>
        <span class="val">${(pos - 0.5).toFixed(3)}</span></div>
       <div class="sbar"><span class="lbl">z</span>
        <span class="track-bg"><span class="fill" style="width:${Math.min(100, Math.abs(sim.trace.u.at(-1) || 0) / 2.5).toFixed(0)}%"></span></span>
        <span class="val">${(sim.trace.u.at(-1) || 0).toFixed(0)}</span></div>`;
  }
}

function window8(arr, ts, span = 10) {
  const n = ts.length;
  if (!n) return { x: [], y: [] };
  const t1 = ts[n - 1];
  let i0 = 0;
  while (i0 < n && ts[i0] < t1 - span) i0++;
  return { x: ts.slice(i0), y: arr.slice(i0) };
}

function paintCharts() {
  if (!sim) return;
  const tr = sim.trace;
  const cte = window8(tr.cte.map((v) => v * 1000), tr.t);
  chCte.setBands([{ y0: -10, y1: 10, color: 'rgba(63,214,140,.07)' }])
    .setSeries([{ name: lang === 'en' ? 'cross-track error' : '横向偏差', x: cte.x, y: cte.y, color: '#4da3ff', width: 1.8 }])
    .draw();

  const e = window8(tr.error, tr.t), u = window8(tr.u, tr.t);
  chCtrl.setSeries([
    { name: lang === 'en' ? 'error' : '误差', x: e.x, y: e.y, color: '#ff5f5f', width: 1.6 },
    { name: sim.profileKey === 'arduino' ? 'PID out (PWM)' : 'z (deg/s)', x: u.x, y: u.y, color: '#f0b429', width: 1.6 },
  ]).draw();
}

// ───────────────────────── 硬件参数面板 ─────────────────────────

function buildHW() {
  if (!sim) return;
  const spec = HW_SPEC[sim.profileKey];
  const box = $('#hw-params');
  box.innerHTML = '';
  for (const [key, label, step, tip] of spec) {
    const wrap = document.createElement('label');
    wrap.className = 'field inline';
    wrap.title = tip;
    wrap.innerHTML = `<span>${label}</span><input type="number" step="${step}" value="${sim.profile.params[key]}">`;
    wrap.querySelector('input').addEventListener('change', (ev) => {
      const v = parseFloat(ev.target.value);
      if (!Number.isFinite(v)) return;
      sim.setParams({ [key]: v });
      draw(); paintMetrics(); paintCharts();
    });
    box.appendChild(wrap);
  }
}

/** 自动整定页调用：用某组增益跑一整轮，返回评分 */
export function scoreGains(gains, seconds = 25) {
  const s = new LineSim({
    profile: profileKey(),
    trackType: $('#line-track').value,
    gains,
    params: profileKey() === 'arduino' ? { adc: $('#line-adc').value } : {},
  });
  const N = Math.floor(seconds / s.Ts);
  for (let k = 0; k < N && !s.finished; k++) s.step();
  return s;
}
