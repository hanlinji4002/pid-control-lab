/** 自动整定页：经典法给起点，数值寻优给终点，稳定裕度给底气 */
import { state, bus, toast, codeSnippet, plainCode, setGains } from '../main.js';
import { t, lang } from '../i18n.js';
import {
  OBJECTIVES, relayTune, znFromUltimate, reactionCurve, znOpenLoop,
  cohenCoon, simc, optimize, evaluate, stabilityMargins, defaultBounds, toControllerForm,
} from '../core/autotune.js';
import { fmt, fmtMs } from '../core/metrics.js';
import { Chart } from '../chart.js';

const $ = (s) => document.querySelector(s);
let chart, rows = [];

export function init() {
  chart = new Chart($('#chart-tune'), { xLabel: 't (s)', unitX: 't' });
  chart.enableLegendToggle();

  const sel = $('#tune-obj');
  sel.innerHTML = Object.entries(OBJECTIVES)
    .map(([k, o]) => `<option value="${k}">${lang === 'en' ? o.labelEn : o.label}</option>`).join('');
  sel.value = 'balanced';

  $('#btn-tune').addEventListener('click', run);
  $('#btn-copy').addEventListener('click', () => {
    navigator.clipboard.writeText(plainCode(state.gains, state.ctrl))
      .then(() => toast(t('msg.copied')));
  });
  bus.on('change', (w) => { if (w !== 'tune') paintCode(); });
  bus.on('lang', () => {
    sel.innerHTML = Object.entries(OBJECTIVES)
      .map(([k, o]) => `<option value="${k}">${lang === 'en' ? o.labelEn : o.label}</option>`).join('');
    paintCode(); if (rows.length) paintTable();
  });
  paintCode();
}

function paintCode() { $('#code-out').innerHTML = codeSnippet(state.gains, state.ctrl); }

function note(msg) { $('#tune-note').innerHTML = msg ? `<div class="warnbox">${msg}</div>` : ''; }

function status(msg) { $('#tune-status').textContent = msg; }

async function run() {
  $('#btn-tune').disabled = true;
  status(t('msg.tuning'));
  await new Promise((r) => setTimeout(r, 20));
  try {
    if ($('#tune-target').value === 'line') await runLine();
    else await runPlant();
  } catch (err) {
    toast('整定失败：' + err.message, 4000);
    console.error(err);
  }
  $('#btn-tune').disabled = false;
  status('');
}

// ───────────────── 实验台被控对象 ─────────────────

async function runPlant() {
  const objKey = $('#tune-obj').value;
  const plant = state.plant, pidCfg = state.pidCfg, simOpts = state.simOpts;
  const cands = [];
  /**
   * @param {boolean} classical true = gains 来自经典整定表（标准型），
   *   需要按当前控制器的书写形式折算。当前参数和寻优结果本来就是当前形式，不折算。
   */
  const add = (name, note, gains, classical = true) => {
    if (!gains || !Number.isFinite(gains.kp)) return;
    const conv = classical ? toControllerForm(gains, state.ctrl.mode, state.ctrl.Ts) : gains;
    const g = { kp: Math.max(0, conv.kp), ki: Math.max(0, conv.ki), kd: Math.max(0, conv.kd) };
    cands.push({ name, note, gains: g });
    return g;
  };

  add(lang === 'en' ? 'Current' : '当前参数', lang === 'en' ? 'your gains' : '侧栏里的值', { ...state.gains }, false);

  status(lang === 'en' ? 'relay feedback…' : '继电反馈激励…');
  await tick();
  const h = Math.max(state.ctrl.uMax * 0.25, 1);
  const relay = relayTune(plant, { h, tEnd: Math.max(12, state.sim.tEnd * 4), setpoint: state.sim.setpoint });
  if (relay) {
    const note = `Ku=${fmt(relay.Ku, 3)} Tu=${fmt(relay.Tu, 3)}s`;
    add('Ziegler–Nichols', note, znFromUltimate(relay.Ku, relay.Tu, 'classic'));
    add('Z-N ' + (lang === 'en' ? 'no overshoot' : '无超调型'), note, znFromUltimate(relay.Ku, relay.Tu, 'noOver'));
    add('Pessen ' + (lang === 'en' ? 'aggressive' : '激进型'), note, znFromUltimate(relay.Ku, relay.Tu, 'pessen'));
  }

  status(lang === 'en' ? 'open-loop reaction curve…' : '开环反应曲线…');
  await tick();
  const rc = reactionCurve(plant, { uStep: state.ctrl.uMax * 0.3, tEnd: Math.max(6, state.sim.tEnd * 2) });
  if (rc && !rc.integrating) {
    const note = `K=${fmt(rc.K, 3)} L=${fmt(rc.L, 3)}s T=${fmt(rc.T, 3)}s`;
    add('Cohen–Coon', note, cohenCoon(rc));
    add('Z-N ' + (lang === 'en' ? 'open loop' : '开环法'), note, znOpenLoop(rc));
    add('SIMC (λ=L)', note, simc(rc));
  } else if (rc?.integrating) {
    status(lang === 'en' ? 'plant is integrating — reaction-curve rules skipped'
      : '对象含积分环节，开环反应曲线法不适用，已跳过');
    await new Promise((r) => setTimeout(r, 900));
  }

  status(lang === 'en' ? 'numerical optimisation…' : '数值寻优…');
  await tick();
  const starts = cands.map((c) => c.gains);
  const bounds = defaultBounds([...starts, { kp: state.ctrl.uMax / Math.max(Math.abs(state.sim.setpoint), 1e-6) }]);
  const opt = optimize(starts, plant, pidCfg, simOpts, objKey, bounds);
  if (opt) {
    add(`★ ${lang === 'en' ? 'Optimised' : '数值寻优'}`,
      `${OBJECTIVES[objKey][lang === 'en' ? 'labelEn' : 'label']} · ${opt.evals} evals`, opt.gains, false);
  }

  status(lang === 'en' ? 'evaluating…' : '评估中…');
  await tick();
  rows = cands.map((c) => {
    const { sim, m } = evaluate(c.gains, plant, pidCfg, simOpts);
    const mg = stabilityMargins(c.gains, plant, { mode: state.ctrl.mode, Ts: state.ctrl.Ts });
    return { ...c, sim, m, mg, mode: 'plant' };
  });
  paintTable(); paintChart();
  note(lang === 'en'
    ? 'Bench tuning optimises setpoint tracking. A line follower is mostly rejecting track curvature — verify on the Line-follower tab, or switch "Tune against" to the full lap.'
    : '台架整定优化的是"跟设定值"。而循线小车整天在抑制赛道曲率这个持续扰动 —— '
      + '这两件事要的带宽不一样。拿到参数后回"循线小车"页跑一圈，或者把「整定对象」换成整圈跑分。');
  const best = rows.find((r) => r.name.startsWith('★'));
  if (best) applyRow(rows.indexOf(best), false);
}

// ───────────────── 循线小车整圈跑分 ─────────────────

async function runLine() {
  const { scoreGains } = await import('./line.js');
  const SEC = 24;   // 够跑完一圈，圈速才有数
  const score = (g) => {
    if (g.kp < 0 || g.ki < 0 || g.kd < 0) return { J: 1e9 };
    const s = scoreGains(g, SEC);
    const m = s.metrics;
    if (m.offTrack) return { J: 1e9, m };
    // 目标：跟得准（平均偏差）+ 不丢线 + 跑得快
    const J = m.meanAbsCte * 1000
      + m.maxAbsCte * 400
      + m.lostCount * 4
      + m.lostTime * 8
      - m.distance * 0.6;
    return { J, m };
  };

  const base = { ...state.gains };
  const cands = [{ name: lang === 'en' ? 'Current' : '当前参数', note: '', gains: base }];
  const mul = [[0.5, 1, 1], [2, 1, 1], [1, 1, 0.3], [1, 1, 2.5]];
  const label = lang === 'en' ? ['½ Kp', '2× Kp', '⅓ Kd', '2.5× Kd'] : ['Kp 减半', 'Kp 加倍', 'Kd 三分之一', 'Kd 加倍'];
  mul.forEach((mm, i) => cands.push({
    name: label[i], note: lang === 'en' ? 'sweep' : '单参数扫描',
    gains: { kp: base.kp * mm[0], ki: base.ki * mm[1], kd: base.kd * mm[2] },
  }));

  status(lang === 'en' ? 'searching on the track…' : '在赛道上寻优…');
  await tick();
  const b = {
    kp: [0, base.kp * 6 + 1e-3], ki: [0, base.kp * 2 + 1e-3], kd: [0, base.kd * 6 + base.kp + 1e-3],
  };
  let best = null;
  for (const s0 of [base, { kp: base.kp * 1.8, ki: base.ki, kd: base.kd * 1.5 }]) {
    const r = coordSearch((g) => score(g).J, s0, b, 16);
    if (!best || r.cost < best.cost) best = r;
    status(`${lang === 'en' ? 'searching…' : '寻优中…'} J=${best.cost.toFixed(2)}`);
    await tick();
  }
  cands.push({ name: `★ ${lang === 'en' ? 'Optimised' : '数值寻优'}`,
    note: lang === 'en' ? `${SEC}s run` : `整圈跑分 ${SEC}s`, gains: best.gains });

  rows = cands.map((c) => {
    const s = scoreGains(c.gains, SEC);
    return { ...c, lineSim: s, m: s.metrics, mode: 'line' };
  });
  paintTable(); paintLineChart();
  note(lang === 'en'
    ? `Scored on the ${$('#line-track').value} track over ${SEC}s. Switch tracks and re-run: gains that win on an oval are not the gains that survive right angles.`
    : `在「${$('#line-track').selectedOptions[0].textContent}」上跑 ${SEC} 秒评分。换条赛道再跑一次 —— `
      + `椭圆上最优的参数，未必扛得住直角弯。`);
  const bi = rows.findIndex((r) => r.name.startsWith('★'));
  if (bi >= 0) applyRow(bi, false);
}

function coordSearch(f, x0, b, rounds) {
  let g = { ...x0 };
  let best = f(g);
  let step = { kp: b.kp[1] * 0.18, ki: b.ki[1] * 0.18 || 1e-4, kd: b.kd[1] * 0.18 };
  for (let r = 0; r < rounds; r++) {
    let improved = false;
    for (const k of ['kp', 'ki', 'kd']) {
      for (const sgn of [1, -1]) {
        const c = { ...g, [k]: Math.min(b[k][1], Math.max(b[k][0], g[k] + sgn * step[k])) };
        const v = f(c);
        if (v < best - 1e-9) { best = v; g = c; improved = true; }
      }
    }
    if (!improved) for (const k of ['kp', 'ki', 'kd']) step[k] *= 0.55;
  }
  return { gains: g, cost: best };
}

const tick = () => new Promise((r) => setTimeout(r, 0));

// ───────────────── 表格 / 图表 ─────────────────

function paintTable() {
  if (!rows.length) return;
  const isLine = rows[0].mode === 'line';
  const g = (v) => (Math.abs(v) >= 1000 || (v !== 0 && Math.abs(v) < 1e-3) ? v.toExponential(1) : +v.toPrecision(4));
  const en = lang === 'en';

  const head = isLine
    ? [en ? 'Method' : '方法', 'Kp', 'Ki', 'Kd', en ? 'Mean |CTE|' : '平均偏差',
       en ? 'Max' : '最大', en ? 'Lost' : '丢线', en ? 'Best lap' : '最快圈', en ? 'Dist' : '里程', '']
    : [en ? 'Method' : '方法', 'Kp', 'Ki', 'Kd', en ? 'Overshoot' : '超调',
       en ? 'Rise' : '上升', en ? 'Settle' : '调节', 'ITAE',
       en ? 'Sat' : '饱和', en ? 'Chatter' : '抖动', 'GM', 'PM', ''];

  const bestJ = isLine
    ? Math.min(...rows.map((r) => (r.m.offTrack ? Infinity : r.m.meanAbsCte)))
    : Math.min(...rows.map((r) => (r.m.stable ? r.m.itae : Infinity)));

  const body = rows.map((r, i) => {
    const mk = r.name.startsWith('★') ? ' best' : '';
    const cells = isLine
      ? [`${(r.m.meanAbsCte * 1000).toFixed(1)} mm`,
         `${(r.m.maxAbsCte * 1000).toFixed(0)} mm`,
         String(r.m.lostCount),
         r.m.bestLap ? r.m.bestLap.toFixed(2) + ' s' : '—',
         r.m.distance.toFixed(2) + ' m']
      : [fmt(r.m.overshoot, 1, '%'),
         fmtMs(r.m.riseTime, '—'), fmtMs(r.m.settleTime, en ? 'not settled' : '未收敛'), fmt(r.m.itae, 3),
         r.m.satPct.toFixed(0) + '%', fmt(r.m.uEffort, 0),
         r.mg?.gm != null ? fmt(r.mg.gm, 1, ' dB') : '—',
         r.mg?.pm != null ? fmt(r.mg.pm, 0, '°') : '—'];
    const isBest = isLine ? (!r.m.offTrack && r.m.meanAbsCte === bestJ) : (r.m.stable && r.m.itae === bestJ);
    const bad = isLine ? r.m.offTrack : !r.m.stable;
    return `<tr class="${mk}">
      <td>${r.name}${isBest ? `<span class="tag win">${en ? 'best' : '最优'}</span>` : ''}
        ${bad ? `<span class="tag" style="color:var(--danger)">${isLine ? t('msg.offtrack') : t('msg.unstable')}</span>` : ''}
        <div class="hint">${r.note || ''}</div></td>
      <td>${g(r.gains.kp)}</td><td>${g(r.gains.ki)}</td><td>${g(r.gains.kd)}</td>
      ${cells.map((c) => `<td>${c}</td>`).join('')}
      <td><button class="btn sm" data-apply="${i}">${en ? 'Use' : '采用'}</button></td>
    </tr>`;
  }).join('');

  $('#tune-table').innerHTML =
    `<thead><tr>${head.map((h) => `<th>${h}</th>`).join('')}</tr></thead><tbody>${body}</tbody>`;
  $('#tune-table').querySelectorAll('[data-apply]').forEach((b) =>
    b.addEventListener('click', () => applyRow(+b.dataset.apply, true)));
}

function applyRow(i, announce) {
  const r = rows[i];
  if (!r) return;
  setGains(r.gains, 'tune');
  paintCode();
  if (announce) toast(`${lang === 'en' ? 'Applied' : '已采用'} ${r.name}`);
}

const PALETTE = ['#7f8fa6', '#4da3ff', '#b07cff', '#f0b429', '#35d0d8', '#ff7ab6', '#3fd68c', '#ff5f5f'];

function paintChart() {
  const s0 = rows[0]?.sim;
  if (!s0) return;
  chart.opts.yLabel = lang === 'en' ? state.preset.unitEn : state.preset.unit;
  chart.setSeries([
    { name: lang === 'en' ? 'setpoint' : '设定值', x: s0.t, y: s0.r, color: '#4b5768', dash: [5, 4], width: 1.2 },
    ...rows.map((r, i) => ({
      name: r.name.replace('★ ', ''), x: r.sim.t, y: r.sim.y,
      color: PALETTE[i % PALETTE.length],
      width: r.name.startsWith('★') ? 2.4 : 1.4,
      hidden: rows.length > 4 && i > 0 && !r.name.startsWith('★') && i > 3,
    })),
  ]).setBands([{
    y0: state.sim.setpoint * 0.98, y1: state.sim.setpoint * 1.02, color: 'rgba(63,214,140,.06)',
  }]).draw();
}

function paintLineChart() {
  chart.opts.yLabel = 'mm';
  chart.setBands([{ y0: -10, y1: 10, color: 'rgba(63,214,140,.06)' }])
    .setSeries(rows.map((r, i) => ({
      name: r.name.replace('★ ', ''),
      x: r.lineSim.trace.t, y: r.lineSim.trace.cte.map((v) => v * 1000),
      color: PALETTE[i % PALETTE.length],
      width: r.name.startsWith('★') ? 2.2 : 1.3,
    }))).draw();
}
