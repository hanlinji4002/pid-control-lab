/** 阶跃响应实验台 */
import { state, bus, metricCard } from '../main.js';
import { t, lang } from '../i18n.js';
import { PID } from '../core/pid.js';
import { simulateStep } from '../core/plant.js';
import { stepMetrics, fmt, fmtMs } from '../core/metrics.js';
import { Chart } from '../chart.js';

const $ = (s) => document.querySelector(s);
let chResp, chU, chE, refSim = null, refGains = null, pending = false;

const C = { r: '#7f8fa6', y: '#4da3ff', ref: '#ff7ab6', u: '#f0b429', p: '#4da3ff', i: '#b07cff', d: '#35d0d8', e: '#ff5f5f' };

export function init() {
  chResp = new Chart($('#chart-resp'), { xLabel: 't (s)', unitX: 't' });
  chU = new Chart($('#chart-u'), { xLabel: 't (s)', unitX: 't' });
  chE = new Chart($('#chart-e'), { xLabel: 't (s)', unitX: 't' });
  chResp.enableLegendToggle(); chU.enableLegendToggle();

  const bind = [
    ['#p-K', 'K'], ['#p-tau', 'tau'], ['#p-wn', 'wn'], ['#p-zeta', 'zeta'],
    ['#p-delay', 'deadTime'], ['#p-deadband', 'deadband'], ['#p-noise', 'measNoise'],
  ];
  for (const [sel, key] of bind) {
    $(sel).addEventListener('input', (e) => {
      const v = parseFloat(e.target.value);
      if (Number.isFinite(v)) { state.plant[key] = v; schedule(); }
    });
  }
  $('#plant-type').addEventListener('change', (e) => {
    state.plant.type = e.target.value; syncFields(); schedule();
  });
  $('#p-tend').addEventListener('input', (e) => {
    const v = parseFloat(e.target.value);
    if (v > 0) { state.sim.tEnd = v; schedule(); }
  });
  $('#p-dist').addEventListener('change', (e) => {
    state.plant.disturbance = e.target.checked ? state.sim.setpoint * 0.4 : 0;
    state.plant.tDist = e.target.checked ? state.sim.tEnd * 0.6 : 1e9;
    schedule();
  });
  $('#hold-ref').addEventListener('change', (e) => {
    if (e.target.checked) { refSim = lastSim; refGains = { ...state.gains }; }
    else { refSim = null; refGains = null; }
    render();
  });
  $('#btn-clear-ref').addEventListener('click', () => {
    refSim = null; refGains = null; $('#hold-ref').checked = false; render();
  });

  bus.on('sidebar', syncPlantInputs);
  bus.on('change', schedule);
  bus.on('lang', render);
  syncPlantInputs();
}

function syncFields() {
  const gen = state.plant.type === 'generic';
  document.querySelectorAll('[data-for=wn]').forEach((n) => { n.style.display = gen ? '' : 'none'; });
  document.querySelectorAll('[data-for=tau]').forEach((n) => { n.style.display = gen ? 'none' : ''; });
}

function syncPlantInputs() {
  const p = state.plant;
  $('#plant-type').value = p.type;
  $('#p-K').value = p.K; $('#p-tau').value = p.tau;
  $('#p-wn').value = p.wn; $('#p-zeta').value = p.zeta;
  $('#p-delay').value = p.deadTime; $('#p-deadband').value = p.deadband;
  $('#p-noise').value = p.measNoise; $('#p-tend').value = state.sim.tEnd;
  $('#p-dist').checked = !!p.disturbance;
  syncFields();
  schedule();
}

function schedule() {
  if (pending) return;
  pending = true;
  requestAnimationFrame(() => { pending = false; render(); });
}

let lastSim = null, lastMetrics = null;

/** 供自动整定页复用 */
export function currentSim() { return { sim: lastSim, metrics: lastMetrics }; }

export function render() {
  const pid = new PID(state.pidCfg);
  const sim = simulateStep(pid, state.plant, state.simOpts);
  const m = stepMetrics(sim);
  lastSim = sim; lastMetrics = m;

  const sp = state.sim.setpoint;
  const pre = state.simOpts.preTime;
  const P = state.preset;
  const yUnit = lang === 'en' ? P.unitEn : P.unit;

  // ── 指标卡
  const cls = (v, good, warn) => (v <= good ? 'good' : v <= warn ? 'warn' : 'bad');
  $('#metrics').innerHTML = [
    metricCard('m.overshoot', fmt(m.overshoot, 1, ' %'),
      `${lang === 'en' ? 'peak' : '峰值'} ${fmt(m.yPeak, 3)}`, cls(m.overshoot, 5, 20)),
    metricCard('m.ess', fmt(m.ess, 3),
      `${fmt(m.essPct, 2, ' %')} ${lang === 'en' ? 'of SP' : '相对设定值'}`,
      cls(Math.abs(m.essPct), 1, 5)),
    metricCard('m.rise', fmtMs(m.riseTime, '—'), '10% → 90%'),
    metricCard('m.settle', fmtMs(m.settleTime, lang === 'en' ? 'not settled' : '未收敛'),
      `±${(m.band * 100).toFixed(0)}%`,
      m.settleTime === null ? 'bad' : ''),
    metricCard('m.peak', fmtMs(m.peakTime, lang === 'en' ? 'no overshoot' : '无超调')),
    metricCard('m.itae', fmt(m.itae, 3), `IAE ${fmt(m.iae, 3)}`),
    metricCard('m.urms', fmt(m.uRms, 1, ' ' + (P.uUnit || '')),
      `${t('m.sat')} ${m.satPct.toFixed(0)}%`, m.satPct > 30 ? 'warn' : ''),
    metricCard('m.cross', String(m.crossings), '', m.crossings > 6 ? 'warn' : ''),
  ].join('');

  // ── 响应曲线
  const bandTol = Math.abs(sp) * m.band;
  chResp.opts.yLabel = yUnit;
  chResp.setBands([{ y0: sp - bandTol, y1: sp + bandTol, color: 'rgba(63,214,140,.07)' }])
    .setVLines([
      m.settleTime !== null
        ? { x: pre + m.settleTime, color: 'var(--ok)', label: `ts ${fmtMs(m.settleTime)}` } : null,
      m.overshoot > 0.5 ? { x: pre + m.peakTime, color: 'var(--warn)', label: `Mp ${m.overshoot.toFixed(1)}%` } : null,
    ].filter(Boolean))
    .setSeries([
      { name: lang === 'en' ? 'setpoint' : '设定值', x: sim.t, y: sim.r, color: C.r, dash: [5, 4], width: 1.3 },
      refSim ? { name: `${lang === 'en' ? 'ref' : '对比'} Kp${+refGains.kp.toPrecision(3)}`,
                 x: refSim.t, y: refSim.y, color: C.ref, width: 1.5, dash: [2, 3] } : null,
      { name: lang === 'en' ? 'output y' : '输出 y', x: sim.t, y: sim.y, color: C.y, width: 2.1 },
    ].filter(Boolean))
    .setMarkers(m.overshoot > 0.5 ? [{ x: pre + m.peakTime, y: m.yPeak, color: C.warn || '#f0b429' }] : [])
    .draw();

  // ── 控制量分解
  chU.opts.yLabel = P.uUnit || 'u';
  const uSat = state.ctrl.uMax;
  chU.setBands([{ y0: -uSat, y1: uSat, color: 'rgba(77,163,255,.04)' }])
    .setSeries([
      { name: 'u', x: sim.t, y: sim.u, color: C.u, width: 2 },
      { name: 'P', x: sim.t, y: sim.p, color: C.p, width: 1.2, dash: [4, 3] },
      { name: 'I', x: sim.t, y: sim.i, color: C.i, width: 1.2, dash: [4, 3] },
      { name: 'D', x: sim.t, y: sim.d, color: C.d, width: 1.2, dash: [4, 3] },
    ]).draw();

  // ── 误差
  const err = new Float64Array(sim.t.length);
  for (let k = 0; k < err.length; k++) err[k] = sim.r[k] - sim.y[k];
  chE.opts.yLabel = 'e';
  chE.setSeries([
    { name: 'e = r − y', x: sim.t, y: err, color: C.e, width: 1.8 },
    refSim ? { name: lang === 'en' ? 'ref' : '对比', x: refSim.t,
               y: refSim.r.map((v, i) => v - refSim.y[i]), color: C.ref, width: 1.2, dash: [3, 3] } : null,
  ].filter(Boolean)).draw();
}
