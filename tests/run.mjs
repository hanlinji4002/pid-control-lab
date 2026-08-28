/**
 * 数值内核自检 —— 不测 UI，只测"算得对不对"。
 * 能对上解析解的就跟解析解比，不能的就比行为（该振荡的振荡，该稳的稳）。
 *
 *   node tests/run.mjs
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

import { PID, arduinoToStandard, standardToArduino } from '../js/core/pid.js';
import { Plant } from '../js/core/plant.js';
import { stepMetrics } from '../js/core/metrics.js';
import { identifyARX, DiscreteModel } from '../js/core/ident.js';
import { parseLog } from '../js/core/logparse.js';
import {
  relayTune, znFromUltimate, reactionCurve, cohenCoon, simc,
  optimize, evaluate, cost, stabilityMargins, OBJECTIVES, toControllerForm,
} from '../js/core/autotune.js';
import { LineSim } from '../js/sim/car.js';
import { Track, LapTimer } from '../js/sim/track.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');

let pass = 0, fail = 0;
const results = [];

function check(name, fn) {
  try {
    const detail = fn();
    pass++; results.push(['✓', name, detail || '']);
  } catch (e) {
    fail++; results.push(['✗', name, e.message]);
  }
}
function near(actual, expected, tol, what) {
  const d = Math.abs(actual - expected);
  const lim = Math.abs(expected) * tol + 1e-12;
  if (!(d <= lim)) {
    throw new Error(`${what || ''} 期望 ${expected.toPrecision(5)} ±${(tol * 100).toFixed(0)}%，实得 ${Number(actual).toPrecision(5)}`);
  }
}
function ok(cond, msg) { if (!cond) throw new Error(msg); }

/** 开环喂恒定输入，用于和解析解对照 */
function openLoop(cfg, u, tEnd, dt = 0.001) {
  const p = new Plant(cfg);
  const t = [], y = [];
  for (let k = 0; k * dt < tEnd; k++) {
    const r = p.step(u, dt);
    t.push(k * dt); y.push(r.yTrue);
  }
  return { t, y };
}

// ───────────── PID ─────────────

check('两种离散化经增益换算后等价', () => {
  const Ts = 0.02, N = 400;
  const gStd = { kp: 1.4, ki: 3.0, kd: 0.05 };
  const gIno = standardToArduino(gStd, Ts);
  const a = new PID({ ...gStd, mode: 'standard', uMin: -1e9, uMax: 1e9, iMax: 1e9 });
  const b = new PID({ ...gIno, mode: 'arduino', uMin: -1e9, uMax: 1e9, iMax: 1e9 });
  let maxDiff = 0;
  for (let k = 0; k < N; k++) {
    const e = Math.sin(k * 0.05) + 0.3 * Math.sin(k * 0.31);
    const ua = a.update(e, 0, Ts).u;
    const ub = b.update(e, 0, Ts).u;
    maxDiff = Math.max(maxDiff, Math.abs(ua - ub));
  }
  ok(maxDiff < 1e-9, `最大偏差 ${maxDiff}`);
  const back = arduinoToStandard(gIno, Ts);
  near(back.ki, gStd.ki, 1e-9, 'Ki 往返');
  near(back.kd, gStd.kd, 1e-9, 'Kd 往返');
  return `最大逐拍偏差 ${maxDiff.toExponential(1)}`;
});

check('积分限幅确实拦住了饱和累积', () => {
  const mk = (aw) => {
    const p = new PID({ kp: 0, ki: 10, kd: 0, mode: 'standard', iMax: 5, uMin: -1, uMax: 1, antiWindup: aw });
    for (let k = 0; k < 2000; k++) p.update(1, 0, 0.01);
    return p.integral;
  };
  const clamped = mk('clamp'), none = mk('none');
  near(clamped, 5, 1e-9, '限幅后积分器');
  ok(none > 4 * clamped, `不限幅应远大于限幅：${none} vs ${clamped}`);
  return `clamp=${clamped.toFixed(2)}  none=${none.toFixed(2)}`;
});

// ───────────── 被控对象 ─────────────

check('一阶惯性：y(τ) = K·u·(1 − 1/e)', () => {
  const K = 2, tau = 0.2, u = 1;
  const { t, y } = openLoop({ type: 'speed', K, tau, deadTime: 0, measNoise: 0, quantLSB: 0 }, u, 2);
  const i = t.findIndex((v) => v >= tau);
  near(y[i], K * u * (1 - Math.exp(-1)), 0.01, `y(τ)`);
  near(y[y.length - 1], K * u, 0.005, '稳态值');
  return `y(τ)=${y[i].toFixed(4)}  y(∞)=${y[y.length - 1].toFixed(4)}`;
});

check('二阶超调量对上解析解 exp(−πζ/√(1−ζ²))', () => {
  for (const zeta of [0.2, 0.4, 0.6]) {
    const { y } = openLoop({ type: 'generic', K: 1, wn: 6, zeta, deadTime: 0, measNoise: 0, quantLSB: 0 }, 1, 6);
    const peak = Math.max(...y);
    const analytic = 1 + Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta));
    near(peak, analytic, 0.01, `ζ=${zeta} 峰值`);
  }
  return 'ζ = 0.2 / 0.4 / 0.6 三点均在 1% 内';
});

check('纯延迟把响应整体推后 L', () => {
  const L = 0.15;
  const a = openLoop({ type: 'speed', K: 1, tau: 0.1, deadTime: 0, measNoise: 0, quantLSB: 0 }, 1, 2);
  const b = openLoop({ type: 'speed', K: 1, tau: 0.1, deadTime: L, measNoise: 0, quantLSB: 0 }, 1, 2);
  const cross = (r) => r.t[r.y.findIndex((v) => v >= 0.5)];
  near(cross(b) - cross(a), L, 0.05, '延迟量');
  return `实测滞后 ${(cross(b) - cross(a)).toFixed(4)} s`;
});

// ───────────── 指标 ─────────────

check('指标算子：合成曲线上的超调 / 上升 / 调节时间', () => {
  const Ts = 0.001, T = 4, sp = 1, zeta = 0.3, wn = 8;
  const wd = wn * Math.sqrt(1 - zeta * zeta);
  const N = Math.round(T / Ts);
  const t = new Float64Array(N), y = new Float64Array(N), r = new Float64Array(N), u = new Float64Array(N);
  for (let k = 0; k < N; k++) {
    const tt = k * Ts;
    t[k] = tt; r[k] = tt < 0.1 ? 0 : sp;
    const s = Math.max(0, tt - 0.1);
    y[k] = tt < 0.1 ? 0
      : sp * (1 - Math.exp(-zeta * wn * s) * (Math.cos(wd * s) + (zeta * wn / wd) * Math.sin(wd * s)));
  }
  const m = stepMetrics({ t, y, r, u, Ts, preTime: 0.1, setpoint: sp, satPct: 0 });
  near(m.overshoot, 100 * Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)), 0.02, '超调量');
  near(m.peakTime, Math.PI / wd, 0.02, '峰值时间');
  ok(Math.abs(m.ess) < 1e-3, `稳态误差应≈0，实得 ${m.ess}`);
  const tsAnalytic = -Math.log(0.02 * Math.sqrt(1 - zeta * zeta)) / (zeta * wn);
  near(m.settleTime, tsAnalytic, 0.15, '调节时间');
  return `Mp=${m.overshoot.toFixed(2)}%  tp=${m.peakTime.toFixed(4)}s  ts=${m.settleTime.toFixed(3)}s`;
});

check('超调量不被噪声尖峰骗到', () => {
  const Ts = 0.005, N = 600;
  const t = new Float64Array(N), y = new Float64Array(N), r = new Float64Array(N), u = new Float64Array(N);
  let seed = 7;
  const rnd = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648 - 0.5;
  for (let k = 0; k < N; k++) {
    t[k] = k * Ts; r[k] = k * Ts < 0.1 ? 0 : 1;
    y[k] = (k * Ts < 0.1 ? 0 : 1 - Math.exp(-(k * Ts - 0.1) * 12)) + rnd() * 0.06;
  }
  const m = stepMetrics({ t, y, r, u, Ts, preTime: 0.1, setpoint: 1, satPct: 0 });
  ok(m.overshoot === 0, `无超调的曲线不该报出 ${m.overshoot.toFixed(2)}% 超调`);
  ok(m.peakTime === null, '无超调时峰值时间应为空');
  return `噪声 σ≈${m.noiseStd.toFixed(3)}，超调判定为 0`;
});

// ───────────── 系统辨识 ─────────────

check('ARX 最小二乘还原已知离散系统', () => {
  const truth = { a1: -1.6, a2: 0.68, b1: 0.05, b2: 0.03 };
  let y1 = 0, y2 = 0, u1 = 0, u2 = 0;
  const U = [], Y = [];
  for (let k = 0; k < 800; k++) {
    const u = Math.sin(k * 0.07) + 0.4 * Math.sin(k * 0.31) + 0.2 * Math.sin(k * 0.013);
    const y = -truth.a1 * y1 - truth.a2 * y2 + truth.b1 * u1 + truth.b2 * u2;
    U.push(u); Y.push(y);
    y2 = y1; y1 = y; u2 = u1; u1 = u;
  }
  const m = identifyARX(U, Y, 0.01);
  for (const k of ['a1', 'a2', 'b1', 'b2']) near(m[k], truth[k], 0.05, k);
  ok(m.r2 > 0.999, `R²=${m.r2}`);
  const dm = new DiscreteModel(m);
  let err = 0;
  for (let k = 0; k < 200; k++) err = Math.max(err, Math.abs(dm.step(U[k]) - Y[k + 2]));
  return `R²=${m.r2.toFixed(6)}`;
});

check('含积分约束时极点严格落在 z = 1', () => {
  const U = [], Y = [];
  let y1 = 0, y2 = 0, u1 = 0, u2 = 0;
  for (let k = 0; k < 500; k++) {
    const u = Math.sin(k * 0.05);
    const y = 1.7 * y1 - 0.7 * y2 + 0.02 * u1 + 0.01 * u2;   // 含积分器的真系统
    U.push(u); Y.push(y); y2 = y1; y1 = y; u2 = u1; u1 = u;
  }
  const m = identifyARX(U, Y, 0.01, { integrator: true });
  near(1 + m.a1 + m.a2, 0, 1e-9, 'A(1) 应为 0（z=1 处有极点）');
  ok(m.integrator === true && m.K === null && m.Kv !== null, '应报速度增益而非静态增益');
  ok(m.r2 > 0.99, `R²=${m.r2}`);
  return `A(1)=${(1 + m.a1 + m.a2).toExponential(1)}  R²=${m.r2.toFixed(5)}  Kv=${m.Kv.toFixed(4)}`;
});

// ───────────── 日志解析 ─────────────

check('三种示例日志都能解析出必需通道', () => {
  const expect = {
    'arduino_line_serial.log': { format: 'arduino-serial', need: ['nL', 'nM', 'nR', 'error', 'u'], synth: true },
    'arduino_line_fast.csv': { format: 'csv', need: ['nL', 'nM', 'nR', 'error', 'u'], synth: false },
    'robomaster_line.csv': { format: 'csv', need: ['linePos', 'error', 'u', 'v'], synth: false },
  };
  const out = [];
  for (const [file, spec] of Object.entries(expect)) {
    const log = parseLog(readFileSync(join(ROOT, 'data', file), 'utf8'), { defaultTs: 0.2 });
    ok(log.format === spec.format, `${file} 格式识别为 ${log.format}`);
    for (const c of spec.need) ok(log.ch[c], `${file} 缺通道 ${c}`);
    ok(!!log.meta.syntheticTime === spec.synth, `${file} 时间戳判定不符`);
    ok(log.meta.samples > 100, `${file} 样本太少`);
    ok(log.states.some((s) => s), `${file} 未解析出状态`);
    out.push(`${file.split('_').pop()}:${log.meta.samples}`);
  }
  return out.join(' ');
});

check('无时间戳日志按指定周期生成时间轴', () => {
  const txt = 'norm[0.10,0.90,0.05]  STRAIGHT  err=-2.0  L=90  R=70\n'.repeat(50);
  const a = parseLog(txt, { defaultTs: 0.2 });
  const b = parseLog(txt, { defaultTs: 0.05 });
  near(a.meta.Ts, 0.2, 1e-9, 'Ts=200ms');
  near(b.meta.Ts, 0.05, 1e-9, 'Ts=50ms');
  near(a.meta.duration, 49 * 0.2, 1e-9, '时长');
  return `50 行 → ${a.meta.duration.toFixed(1)}s / ${b.meta.duration.toFixed(2)}s`;
});

check('缺 error 列时按 .ino 公式补算', () => {
  const csv = 't,nL,nM,nR,pwmL,pwmR,state\n0,0.10,0.90,0.50,90,70,STRAIGHT\n0.01,0.10,0.90,0.50,90,70,STRAIGHT\n';
  const log = parseLog(csv);
  ok(log.ch.error, '未补出 error 通道');
  const tot = 0.1 + 0.9 + 0.5;
  near(log.ch.error[0], (0.5 - 0.1) * 100 / tot, 1e-9, '重算的误差');
  return `error[0] = ${log.ch.error[0].toFixed(3)}`;
});

// ───────────── 自动整定 ─────────────

check('继电反馈测出的 Ku 确实是稳定边界', () => {
  const plant = { type: 'generic', K: 1, wn: 6, zeta: 0.35, deadTime: 0.03, uSat: 1e6, measNoise: 0, quantLSB: 0 };
  const r = relayTune(plant, { h: 20, setpoint: 1 });
  ok(r && r.Ku > 0 && r.Tu > 0, '继电试验未激起等幅振荡');
  const pidCfg = { mode: 'standard', iMax: 1e6, uMin: -1e6, uMax: 1e6, antiWindup: 'clamp' };
  const so = { tEnd: 6, Ts: 0.002, setpoint: 1, preTime: 0.1 };
  // Ku 的定义就是临界稳定增益：略低于它应收敛，明显高于它应发散
  const below = evaluate({ kp: 0.6 * r.Ku, ki: 0, kd: 0 }, plant, pidCfg, so).m;
  const at = evaluate({ kp: 1.0 * r.Ku, ki: 0, kd: 0 }, plant, pidCfg, so).m;
  const above = evaluate({ kp: 1.8 * r.Ku, ki: 0, kd: 0 }, plant, pidCfg, so).m;
  ok(below.stable, '0.6·Ku 应当稳定');
  ok(!above.stable, '1.8·Ku 应当发散');
  ok(at.crossings > below.crossings, `Ku 处应持续振荡：穿越 ${at.crossings} vs ${below.crossings}`);
  ok(at.overshoot > below.overshoot + 10, `Ku 处超调应远大于 0.6Ku：${at.overshoot.toFixed(0)}% vs ${below.overshoot.toFixed(0)}%`);
  return `Ku=${r.Ku.toFixed(3)} Tu=${r.Tu.toFixed(3)}s · 0.6Ku 稳定 / 1.8Ku 发散 · 超调 ${below.overshoot.toFixed(0)}%→${at.overshoot.toFixed(0)}%`;
});

check('反应曲线两点法辨识出的 FOPDT 参数合理', () => {
  const plant = { type: 'generic', K: 1.5, wn: 6, zeta: 0.9, deadTime: 0.08, uSat: 1e6, measNoise: 0, quantLSB: 0 };
  const rc = reactionCurve(plant, { uStep: 1, tEnd: 6 });
  ok(rc && !rc.integrating, '未能得到反应曲线');
  near(rc.K, 1.5, 0.05, '静态增益');
  ok(rc.L >= 0.08 * 0.6 && rc.L < 0.6, `等效滞后 L=${rc.L.toFixed(3)} 不合理`);
  ok(rc.T > 0.1 && rc.T < 3, `时间常数 T=${rc.T.toFixed(3)} 不合理`);
  for (const [n, g] of [['Cohen-Coon', cohenCoon(rc)], ['SIMC', simc(rc)]]) {
    ok(Number.isFinite(g.kp) && g.kp > 0, `${n} 给出无效 Kp`);
  }
  return `K=${rc.K.toFixed(3)} L=${rc.L.toFixed(3)} T=${rc.T.toFixed(3)}`;
});

check('含积分对象上，开环反应曲线法被正确判定为不适用', () => {
  const rc = reactionCurve({ type: 'heading', K: 20, tau: 0.15, deadTime: 0.02, uSat: 1e6 }, { uStep: 1, tEnd: 4 });
  ok(rc && rc.integrating === true, '航向环（I 型）应被识别为含积分环节');
  return '已跳过 Cohen-Coon / SIMC';
});

check('稳定裕度对得上解析解', () => {
  // L(s) = 5/(0.2s+1)：|L|=1 处 w=√((25−1)/0.04)=24.49，PM=180−atan(0.2w)
  const plant = { type: 'speed', K: 1, tau: 0.2, deadTime: 0, uSat: 1e9, measNoise: 0, quantLSB: 0 };
  const r = stabilityMargins({ kp: 5, ki: 0, kd: 0 }, plant, { wLo: 0.5, wHi: 400 });
  const wGc = Math.sqrt((25 - 1) / 0.04);
  near(r.wGc, wGc, 0.06, '幅值穿越频率');
  near(r.pm, 180 - Math.atan(0.2 * wGc) * 180 / Math.PI, 0.06, '相位裕度');
  ok(r.gm === null, '一阶对象相位到不了 −180°，增益裕度应为无穷（null）');
  return `wGc=${r.wGc.toFixed(2)} (解析 ${wGc.toFixed(2)})  PM=${r.pm.toFixed(1)}°`;
});

check('稳定裕度按控制器书写形式折算', () => {
  const plant = { type: 'heading', K: 55, tau: 0.12, deadTime: 0.02, uSat: 1e9, measNoise: 0, quantLSB: 0 };
  const Ts = 0.01;
  const ino = { kp: 0.6, ki: 0, kd: 6 };
  const a = stabilityMargins(ino, plant, { mode: 'arduino', Ts });
  const b = stabilityMargins(arduinoToStandard(ino, Ts), plant, { mode: 'standard', Ts });
  ok(a.pm !== null && Number.isFinite(a.pm), '未算出相位裕度');
  near(a.pm, b.pm, 1e-6, '两种写法应得到同一裕度');
  return `PM=${a.pm.toFixed(1)}°  GM=${a.gm.toFixed(1)}dB`;
});

check('经典整定表的结果被折算到当前控制器写法', () => {
  const Ts = 0.01;
  const std = znFromUltimate(0.75, 0.36, 'classic');
  const ino = toControllerForm(std, 'arduino', Ts);
  near(ino.ki, std.ki * Ts, 1e-12, 'Ki 折算');
  near(ino.kd, std.kd / Ts, 1e-12, 'Kd 折算');
  near(toControllerForm(std, 'standard', Ts).ki, std.ki, 1e-12, '标准型不应改动');
  return `Ki ${std.ki.toFixed(3)} → ${ino.ki.toFixed(5)}   Kd ${std.kd.toFixed(4)} → ${ino.kd.toFixed(3)}`;
});

check('数值寻优不会输给自己的起点', () => {
  const plant = { type: 'heading', K: 55, tau: 0.12, deadTime: 0.02, deadband: 0, uSat: 80, measNoise: 0.6, quantLSB: 0.4 };
  const pidCfg = { mode: 'arduino', iMax: 100, uMin: -80, uMax: 80, antiWindup: 'clamp' };
  const so = { tEnd: 3, Ts: 0.01, setpoint: 60, preTime: 0.15 };
  const start = { kp: 0.6, ki: 0, kd: 6 };
  const j0 = cost(start, plant, pidCfg, so, OBJECTIVES.balanced);
  const opt = optimize([start], plant, pidCfg, so, 'balanced');
  ok(opt.cost <= j0 + 1e-9, `寻优结果 ${opt.cost} 反而差于起点 ${j0}`);
  const m = evaluate(opt.gains, plant, pidCfg, so).m;
  ok(m.stable, '寻优结果不稳定');
  return `J ${j0.toFixed(4)} → ${opt.cost.toFixed(4)}  (ITAE ${m.itae.toFixed(3)}, 超调 ${m.overshoot.toFixed(1)}%)`;
});

// ───────────── 循线仿真 ─────────────

check('赛道最近点查询与解析距离一致', () => {
  const tr = new Track('oval');
  for (const [x, y, exp] of [[1.05, 0, 0], [0, 0.62, 0], [0, 0, null]]) {
    const n = tr.nearest(x, y);
    if (exp !== null) ok(n.dist < 0.01, `(${x},${y}) 到赛道距离应≈0，实得 ${n.dist.toFixed(4)}`);
    else ok(n.dist > 0.5, '圆心到椭圆的距离应较大');
  }
  ok(tr.length > 4 && tr.length < 7, `椭圆周长 ${tr.length.toFixed(2)} m 不合理`);
  return `周长 ${tr.length.toFixed(3)} m`;
});

check('圈速计时不重不漏', () => {
  const tr = new Track('oval');
  const lap = new LapTimer(tr);
  // 沿赛道匀速走三圈
  const v = 0.5, dt = 0.01;
  let dist = 0, t = 0;
  for (let k = 0; k < Math.ceil(3.02 * tr.length / v / dt); k++) {
    dist += v * dt; t += dt;
    const s = dist % tr.length;
    let i = 0;
    while (i < tr.s.length - 1 && tr.s[i] < s) i++;
    lap.update(tr.pts[i].x, tr.pts[i].y, t, dist);
  }
  ok(lap.laps.length === 3, `应记到 3 圈，实得 ${lap.laps.length}`);
  near(lap.best, tr.length / v, 0.02, '单圈时间');
  return `3 圈，最快 ${lap.best.toFixed(2)}s（解析 ${(tr.length / v).toFixed(2)}s）`;
});

check('Arduino 真机增益能在椭圆上跑完整圈', () => {
  const sim = new LineSim({ profile: 'arduino', trackType: 'oval', gains: { kp: 0.6, ki: 0, kd: 6 }, seed: 4002 });
  for (let k = 0; k < Math.floor(40 / sim.Ts) && !sim.finished; k++) sim.step();
  const m = sim.metrics;
  ok(!m.offTrack, '冲出赛道');
  ok(m.laps >= 2, `40 秒应跑完至少 2 圈，实得 ${m.laps}`);
  ok(m.meanAbsCte < 0.005, `平均横向偏差 ${(m.meanAbsCte * 1000).toFixed(1)} mm 偏大`);
  ok(m.lostCount === 0, `不该丢线，实得 ${m.lostCount} 次`);
  return `${m.laps} 圈，最快 ${m.bestLap.toFixed(2)}s，平均偏差 ${(m.meanAbsCte * 1000).toFixed(1)} mm`;
});

check('RoboMaster 真机增益能在椭圆上跑完整圈', () => {
  const sim = new LineSim({ profile: 'robomaster', trackType: 'oval', gains: { kp: 280, ki: 0, kd: 30 }, seed: 12022 });
  for (let k = 0; k < Math.floor(50 / sim.Ts) && !sim.finished; k++) sim.step();
  const m = sim.metrics;
  ok(!m.offTrack, '冲出赛道');
  ok(m.laps >= 2, `50 秒应跑完至少 2 圈，实得 ${m.laps}`);
  ok(m.meanAbsCte < 0.03, `平均横向偏差 ${(m.meanAbsCte * 1000).toFixed(1)} mm 偏大`);
  return `${m.laps} 圈，最快 ${m.bestLap.toFixed(2)}s，平均偏差 ${(m.meanAbsCte * 1000).toFixed(1)} mm`;
});

check('去掉 Kd：台架上超调暴涨，赛道上偏差变大', () => {
  // 台架：航向环是 I 型对象，纯 P 必然大幅振荡，加 D 才压得住
  const plant = { type: 'heading', K: 55, tau: 0.12, deadTime: 0.02, deadband: 0, uSat: 80, measNoise: 0, quantLSB: 0 };
  const pidCfg = { mode: 'arduino', iMax: 100, uMin: -80, uMax: 80, antiWindup: 'clamp' };
  const so = { tEnd: 3, Ts: 0.01, setpoint: 60, preTime: 0.15 };
  const withD = evaluate({ kp: 0.6, ki: 0, kd: 6 }, plant, pidCfg, so).m;
  const noD = evaluate({ kp: 0.6, ki: 0, kd: 0 }, plant, pidCfg, so).m;
  ok(noD.overshoot > 40 && withD.overshoot < 5,
    `超调应从 ${noD.overshoot.toFixed(0)}% 降到 ${withD.overshoot.toFixed(0)}%`);

  // 赛道：直角赛道上差别最明显（传感器前视本身有阻尼作用，缓弯上 D 的贡献没那么大）
  const run = (g) => {
    const s = new LineSim({ profile: 'arduino', trackType: 'rightAngle', gains: g, seed: 4002 });
    for (let k = 0; k < Math.floor(24 / s.Ts) && !s.finished; k++) s.step();
    return s.metrics;
  };
  const trackD = run({ kp: 0.6, ki: 0, kd: 6 });
  const trackNo = run({ kp: 0.6, ki: 0, kd: 0 });
  ok(trackNo.maxAbsCte > trackD.maxAbsCte * 1.25,
    `最大偏差应明显变差：${(trackNo.maxAbsCte * 1000).toFixed(0)} vs ${(trackD.maxAbsCte * 1000).toFixed(0)} mm`);
  return `台架超调 ${withD.overshoot.toFixed(0)}% → ${noD.overshoot.toFixed(0)}%；`
    + `直角赛道最大偏差 ${(trackD.maxAbsCte * 1000).toFixed(0)} → ${(trackNo.maxAbsCte * 1000).toFixed(0)} mm`;
});

check('ADS1115 的量化噪声确实低于 UNO 内置 ADC', () => {
  const noise = (adc) => {
    const s = new LineSim({ profile: 'arduino', trackType: 'oval', gains: { kp: 0.6, ki: 0, kd: 6 }, params: { adc }, seed: 99 });
    for (let k = 0; k < 1500; k++) s.step();
    // 控制量的逐拍抖动 ≈ 微分项被噪声放大的程度
    let d = 0;
    for (let i = 1; i < s.trace.u.length; i++) d += Math.abs(s.trace.u[i] - s.trace.u[i - 1]);
    return d / s.trace.u.length;
  };
  const uno = noise('uno'), ads = noise('ads1115');
  ok(ads < uno * 0.7, `ADS1115 抖动 ${ads.toFixed(2)} 应明显低于 UNO ${uno.toFixed(2)}`);
  return `平均 |Δu|：UNO ${uno.toFixed(2)} → ADS1115 ${ads.toFixed(2)}`;
});

check('仿真可复现：同种子两次结果逐位相同', () => {
  const run = () => {
    const s = new LineSim({ profile: 'arduino', trackType: 'slalom', gains: { kp: 0.6, ki: 0, kd: 6 }, seed: 1234 });
    for (let k = 0; k < 800; k++) s.step();
    return [s.x, s.y, s.theta, s.metrics.meanAbsCte];
  };
  const a = run(), b = run();
  a.forEach((v, i) => ok(v === b[i], `第 ${i} 项不一致：${v} vs ${b[i]}`));
  return '位置 / 航向 / 指标全部一致';
});

// ───────────── 输出 ─────────────

const w = Math.max(...results.map((r) => r[1].length));
for (const [mark, name, detail] of results) {
  console.log(`${mark} ${name.padEnd(w)}  ${detail}`);
}
console.log(`\n${pass} 通过, ${fail} 失败`);
process.exit(fail ? 1 : 0);
