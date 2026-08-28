/**
 * 自动整定 —— 四条互补的路线，而不是只给一个数字
 *
 *  1. 继电反馈 (Åström–Hägglund)  闭环激起等幅振荡 → Ku, Tu → Z-N 表
 *  2. 开环反应曲线 (两点法)        → FOPDT 的 K/L/T → Z-N开环 / Cohen-Coon / SIMC
 *  3. 数值寻优 (Nelder–Mead)       直接最小化你选的目标函数，天然考虑饱和/死区/延迟
 *  4. 稳定裕度校核                 给出增益裕度，别让"最优"参数擦着不稳定边界跑
 *
 * 前三种给"起点"，第四种告诉你这个起点安不安全。真机调参就该这么走。
 */
import { PID, arduinoToStandard, standardToArduino } from './pid.js';
import { Plant, simulateStep } from './plant.js';
import { stepMetrics } from './metrics.js';

// ───────────────────────── 目标函数 ─────────────────────────

export const OBJECTIVES = {
  balanced: { label: '平衡', labelEn: 'Balanced', wOS: 1.0, osTarget: 10, wEffort: 0.15, wSat: 0.5 },
  fast:     { label: '快速响应', labelEn: 'Fast', wOS: 0.25, osTarget: 20, wEffort: 0.05, wSat: 0.3 },
  smooth:   { label: '无超调', labelEn: 'No overshoot', wOS: 6.0, osTarget: 1, wEffort: 0.3, wSat: 1.0 },
  gentle:   { label: '省电护电机', labelEn: 'Gentle on actuator', wOS: 1.5, osTarget: 5, wEffort: 1.2, wSat: 2.0 },
};

export function evaluate(gains, plantCfg, pidCfg, simOpts) {
  const pid = new PID({ ...pidCfg, ...gains });
  const sim = simulateStep(pid, plantCfg, simOpts);
  const m = stepMetrics(sim);
  return { sim, m };
}

export function cost(gains, plantCfg, pidCfg, simOpts, obj) {
  const { m } = evaluate(gains, plantCfg, pidCfg, simOpts);
  if (!m.stable || !Number.isFinite(m.itae)) return 1e9;

  // 各项都归一到 O(0.1~1)，否则某一项会悄悄主导整个目标函数。
  // itaeMax = 误差一直等于设定值时的 ITAE，即"完全不控制"的上界。
  const T = simOpts.tEnd || 4;
  const sp = Math.abs(simOpts.setpoint || 1);
  const itaeMax = sp * T * T / 2;
  const itaeN = (m.itae / itaeMax) * 50;

  const osPen = (Math.max(0, m.overshoot - obj.osTarget) / 10) ** 2;
  const essPen = (Math.abs(m.essPct) / 5) ** 2;
  // 抖动：平均每拍的控制量变化，相对于执行器量程
  const nSteps = Math.max(1, T / (simOpts.Ts || 0.01));
  const effort = m.uEffort / (Math.abs(pidCfg.uMax || 255) * nSteps) * 10;
  const satPen = (m.satPct / 100) ** 2;
  const noSettle = m.settleTime === null ? 1.5 : 0;

  return itaeN + obj.wOS * osPen + essPen + obj.wEffort * effort + obj.wSat * satPen + noSettle;
}

// ───────────────────── 1. 继电反馈整定 ─────────────────────

/**
 * @returns {{Ku, Tu, amplitude, trace}|null}
 */
export function relayTune(plantCfg, { h = 50, eps = 0.01, tEnd = 12, Ts = 0.005, setpoint = 1 } = {}) {
  const plant = new Plant({ ...plantCfg, measNoise: 0, procNoise: 0, quantLSB: 0 });
  const N = Math.floor(tEnd / Ts);
  const t = [], y = [], u = [];
  let meas = 0, out = h;

  for (let k = 0; k < N; k++) {
    const e = setpoint - meas;
    if (e > eps) out = h; else if (e < -eps) out = -h;
    const res = plant.step(out, Ts);
    t.push(k * Ts); y.push(res.yTrue); u.push(out);
    meas = res.y;
  }

  // 只看后半段（等幅段），找极大/极小
  const start = Math.floor(N * 0.5);
  const peaks = [], troughs = [];
  for (let k = start + 1; k < N - 1; k++) {
    if (y[k] > y[k - 1] && y[k] >= y[k + 1]) peaks.push({ t: t[k], v: y[k] });
    if (y[k] < y[k - 1] && y[k] <= y[k + 1]) troughs.push({ t: t[k], v: y[k] });
  }
  if (peaks.length < 3 || troughs.length < 3) return null;

  const lastP = peaks.slice(-4), lastT = troughs.slice(-4);
  const pMean = lastP.reduce((s, p) => s + p.v, 0) / lastP.length;
  const tMean = lastT.reduce((s, p) => s + p.v, 0) / lastT.length;
  const a = (pMean - tMean) / 2;
  if (!(a > 1e-6)) return null;

  let Tu = 0;
  for (let i = 1; i < lastP.length; i++) Tu += lastP[i].t - lastP[i - 1].t;
  Tu /= (lastP.length - 1);
  if (!(Tu > 0)) return null;

  const Ku = (4 * h) / (Math.PI * a);
  return { Ku, Tu, amplitude: a, trace: { t, y, u } };
}

/** Ziegler–Nichols 闭环整定表 */
export function znFromUltimate(Ku, Tu, form = 'classic') {
  const tables = {
    P:          [0.5 * Ku, 0, 0],
    PI:         [0.45 * Ku, 0.45 * Ku / (Tu / 1.2), 0],
    classic:    [0.6 * Ku, 0.6 * Ku / (0.5 * Tu), 0.6 * Ku * 0.125 * Tu],
    pessen:     [0.7 * Ku, 0.7 * Ku / (0.4 * Tu), 0.7 * Ku * 0.15 * Tu],
    someOver:   [0.33 * Ku, 0.33 * Ku / (0.5 * Tu), 0.33 * Ku * (Tu / 3)],
    noOver:     [0.2 * Ku, 0.2 * Ku / (0.5 * Tu), 0.2 * Ku * (Tu / 3)],
  };
  const [kp, ki, kd] = tables[form] || tables.classic;
  return { kp, ki, kd };
}

// ─────────────── 2. 开环反应曲线 → FOPDT ───────────────

/** 两点法（Smith）辨识一阶纯滞后模型，比切线法抗噪 */
export function reactionCurve(plantCfg, { uStep = 50, tEnd = 6, Ts = 0.005 } = {}) {
  const plant = new Plant({ ...plantCfg, measNoise: 0, procNoise: 0, quantLSB: 0 });
  const N = Math.floor(tEnd / Ts);
  const t = [], y = [];
  for (let k = 0; k < N; k++) {
    const res = plant.step(uStep, Ts);
    t.push(k * Ts); y.push(res.yTrue);
  }
  const yInf = y[N - 1];
  if (!(Math.abs(yInf) > 1e-9)) return null;
  // I 型对象（航向环）阶跃不收敛，两点法不适用
  const yMid = y[Math.floor(N / 2)];
  if (Math.abs(yInf) > 1e-9 && Math.abs(yInf / yMid - 1) > 0.35) {
    return { integrating: true, t, y };
  }

  const find = (frac) => {
    const target = frac * yInf;
    for (let k = 1; k < N; k++) {
      if ((yInf > 0 && y[k] >= target) || (yInf < 0 && y[k] <= target)) {
        const y0 = y[k - 1], y1 = y[k];
        const f = (target - y0) / (y1 - y0 || 1);
        return t[k - 1] + f * Ts;
      }
    }
    return null;
  };
  const t28 = find(0.283), t63 = find(0.632);
  if (t28 === null || t63 === null) return null;

  const T = Math.max(1.5 * (t63 - t28), 1e-4);
  const L = Math.max(t63 - T, 1e-4);
  const K = yInf / uStep;
  return { K, L, T, t, y, integrating: false };
}

export function znOpenLoop({ K, L, T }) {
  const kp = 1.2 * T / (K * L);
  const Ti = 2 * L, Td = 0.5 * L;
  return { kp, ki: kp / Ti, kd: kp * Td };
}

export function cohenCoon({ K, L, T }) {
  const r = L / T;
  const kp = (1 / K) * (1 / r) * (1.33 + r / 4);
  const Ti = L * (32 + 6 * r) / (13 + 8 * r);
  const Td = 4 * L / (11 + 2 * r);
  return { kp, ki: kp / Ti, kd: kp * Td };
}

/** SIMC / λ 整定（Skogestad）—— 保守、鲁棒，适合真机第一次上电 */
export function simc({ K, L, T }, lambda = null) {
  const lam = lambda === null ? L : lambda;
  const kp = T / (K * (lam + L));
  const Ti = Math.min(T, 4 * (lam + L));
  return { kp, ki: kp / Ti, kd: 0 };
}

// ─────────────── 3. Nelder–Mead 数值寻优 ───────────────

const clampG = (g, b) => ({
  kp: Math.min(Math.max(g.kp, b.kp[0]), b.kp[1]),
  ki: Math.min(Math.max(g.ki, b.ki[0]), b.ki[1]),
  kd: Math.min(Math.max(g.kd, b.kd[0]), b.kd[1]),
});

function nelderMead(f, x0, bounds, { maxEval = 240, tol = 1e-7 } = {}) {
  const toArr = (g) => [g.kp, g.ki, g.kd];
  const toObj = (a) => clampG({ kp: a[0], ki: a[1], kd: a[2] }, bounds);
  const span = [
    (bounds.kp[1] - bounds.kp[0]) * 0.08,
    (bounds.ki[1] - bounds.ki[0]) * 0.08,
    (bounds.kd[1] - bounds.kd[0]) * 0.08,
  ];

  let simplex = [toArr(x0)];
  for (let i = 0; i < 3; i++) {
    const p = [...simplex[0]];
    p[i] += span[i] || 0.1;
    simplex.push(p);
  }
  let vals = simplex.map((p) => f(toObj(p)));
  let evals = simplex.length;

  while (evals < maxEval) {
    const order = vals.map((v, i) => i).sort((a, b) => vals[a] - vals[b]);
    simplex = order.map((i) => simplex[i]);
    vals = order.map((i) => vals[i]);
    if (Math.abs(vals[3] - vals[0]) < tol * (Math.abs(vals[0]) + tol)) break;

    const centroid = [0, 1, 2].map((d) =>
      (simplex[0][d] + simplex[1][d] + simplex[2][d]) / 3);
    const worst = simplex[3];
    const refl = centroid.map((c, d) => c + 1.0 * (c - worst[d]));
    const fr = f(toObj(refl)); evals++;

    if (fr < vals[0]) {
      const exp = centroid.map((c, d) => c + 2.0 * (c - worst[d]));
      const fe = f(toObj(exp)); evals++;
      if (fe < fr) { simplex[3] = exp; vals[3] = fe; } else { simplex[3] = refl; vals[3] = fr; }
    } else if (fr < vals[2]) {
      simplex[3] = refl; vals[3] = fr;
    } else {
      const contr = centroid.map((c, d) => c + 0.5 * (worst[d] - c));
      const fc = f(toObj(contr)); evals++;
      if (fc < vals[3]) { simplex[3] = contr; vals[3] = fc; }
      else {
        for (let i = 1; i < 4; i++) {
          simplex[i] = simplex[i].map((v, d) => simplex[0][d] + 0.5 * (v - simplex[0][d]));
          vals[i] = f(toObj(simplex[i])); evals++;
        }
      }
    }
  }
  const best = vals.indexOf(Math.min(...vals));
  return { gains: toObj(simplex[best]), cost: vals[best], evals };
}

/**
 * 多起点寻优。starts 里放 Z-N / Cohen-Coon / 当前参数，
 * 让优化器从几个"物理上讲得通"的地方出发，而不是从零瞎撞。
 */
export function optimize(starts, plantCfg, pidCfg, simOpts, objKey = 'balanced', bounds = null) {
  const obj = OBJECTIVES[objKey] || OBJECTIVES.balanced;
  const b = bounds || defaultBounds(starts);
  const f = (g) => cost(g, plantCfg, pidCfg, simOpts, obj);

  let best = null;
  for (const s of starts) {
    if (!s || !Number.isFinite(s.kp)) continue;
    const r = nelderMead(f, clampG(s, b), b);
    if (!best || r.cost < best.cost) best = r;
  }
  return best;
}

/** 经典整定表输出的是标准型增益；控制器若是 Arduino 型，必须先折算 */
export function toControllerForm(gains, mode, Ts) {
  if (!gains) return gains;
  return mode === 'arduino' ? standardToArduino(gains, Ts) : { ...gains };
}

export function defaultBounds(starts) {
  const kps = starts.filter(Boolean).map((s) => Math.abs(s.kp)).filter(Number.isFinite);
  const scale = Math.max(1, ...kps) * 4;
  return { kp: [0, scale], ki: [0, scale * 8], kd: [0, scale * 2] };
}

// ─────────────── 4. 稳定裕度（数值扫频） ───────────────

/**
 * 对被控对象做数值频率响应，配上 PID 求增益裕度 / 相位裕度。
 * 用正弦注入 + 相关法（比解析传函更通用：死区、饱和都能反映出来）。
 *
 * 注意 gains 必须是**标准型**。Arduino 型的 Ki/Kd 与 Ts 耦合，
 * 直接拿来算 C(jω) = Kp + Ki/(jω) + Kd·jω 会得到完全离谱的裕度。
 * 调用方用 opts.mode/'arduino' + opts.Ts 让本函数自己折算。
 */
export function stabilityMargins(gains, plantCfg, { wLo = 0.2, wHi = 200, n = 60, mode = 'standard', Ts = 0.01 } = {}) {
  const C = mode === 'arduino' ? arduinoToStandard(gains, Ts) : gains;
  const lin = { ...plantCfg, deadband: 0, uSat: 1e9, measNoise: 0, procNoise: 0, quantLSB: 0 };
  const resp = [];
  for (let i = 0; i < n; i++) {
    const w = wLo * Math.pow(wHi / wLo, i / (n - 1));
    const P = frequencyPoint(lin, w);
    // PID 频率响应 C(jw) = Kp + Ki/(jw) + Kd·jw
    const cRe = C.kp, cIm = C.kd * w - C.ki / w;
    const re = cRe * P.re - cIm * P.im;
    const im = cRe * P.im + cIm * P.re;
    resp.push({ w, mag: Math.hypot(re, im), phase: Math.atan2(im, re) * 180 / Math.PI });
  }

  let gm = null, pm = null, wPc = null, wGc = null;
  for (let i = 1; i < resp.length; i++) {
    const a = resp[i - 1], b = resp[i];
    const pa = unwrap(a.phase), pb = unwrap(b.phase);
    if (gm === null && ((pa > -180 && pb <= -180) || (pa < -180 && pb >= -180))) {
      const f = (-180 - pa) / (pb - pa);
      const mag = a.mag + f * (b.mag - a.mag);
      gm = mag > 0 ? -20 * Math.log10(mag) : null;
      wPc = a.w + f * (b.w - a.w);
    }
    if (pm === null && ((a.mag > 1 && b.mag <= 1) || (a.mag < 1 && b.mag >= 1))) {
      const f = (1 - a.mag) / (b.mag - a.mag);
      pm = 180 + (pa + f * (pb - pa));
      wGc = a.w + f * (b.w - a.w);
    }
  }
  return { gm, pm, wPc, wGc, resp };
  function unwrap(p) { return p > 0 ? p - 360 : p; }
}

/** 单点频率响应：注入正弦，用相关法提取幅值/相位 */
function frequencyPoint(plantCfg, w) {
  const period = 2 * Math.PI / w;
  const Ts = Math.min(period / 60, 0.002);
  // 暂态衰减时间按频率自适应：高频点没必要每次都空跑 6 秒
  const settle = Math.min(6, 4 * period + 8 * (plantCfg.tau || 0.2) + 0.2);
  const nSettle = Math.ceil(settle / Ts);
  const nMeas = Math.ceil(6 * period / Ts);
  const plant = new Plant(plantCfg);
  let sRe = 0, sIm = 0;
  for (let k = 0; k < nSettle + nMeas; k++) {
    const t = k * Ts;
    const u = Math.sin(w * t);
    const { yTrue } = plant.step(u, Ts);
    if (k >= nSettle) {
      sRe += yTrue * Math.sin(w * t) * Ts;
      sIm += yTrue * Math.cos(w * t) * Ts;
    }
  }
  const dur = nMeas * Ts;
  return { re: 2 * sRe / dur, im: 2 * sIm / dur };
}
