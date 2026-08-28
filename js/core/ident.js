/**
 * 从真机日志辨识被控对象（ARX 最小二乘）
 *
 * 模型:  y[k] = −a1·y[k−1] − a2·y[k−2] + b1·u[k−1] + b2·u[k−2]
 * 有了它，导入的日志就不只是"回放录像"，而是能拿来做**闭环重仿真**：
 * 换一组 Kp/Ki/Kd，看当时那台车会跑成什么样。
 */

/** 解 n×n 线性方程组（带部分主元的高斯消元） */
function solve(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-12) return null;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}

/**
 * @param {number[]} u 控制量序列
 * @param {number[]} y 输出（或误差取负）序列
 * @param {number} Ts  采样周期
 * @param {{lambda?:number, integrator?:boolean}} opts
 *
 * integrator = true 时把 A(z) 约束成 (1 − z⁻¹)(1 + α z⁻¹)，即强制对象含一个积分环节。
 *
 * 为什么需要这个约束：日志是**闭环**采的，u 基本是 y 的函数，两者高度相关。
 * 无约束最小二乘会把控制器的镇定效果算进对象里，拟合出一个"自己就稳"的模型 ——
 * 于是重仿真里 Kp=Ki=Kd=0 反而误差最小，这是纯粹的辨识偏差，不是物理。
 * 而循线的被控对象（PWM 差 → 航向 → 横向偏差）物理上必然含积分，
 * 把这一点作为先验加回去，模型才有预测能力。
 */
export function identifyARX(u, y, Ts, opts = {}) {
  const { lambda = 1e-6, integrator = false } = (typeof opts === 'number' ? { lambda: opts } : opts);
  const N = Math.min(u.length, y.length);
  if (N < 20) return null;

  const rows = [];
  const targ = [];
  for (let k = 2; k < N; k++) {
    if (integrator) {
      // y[k] − y[k−1] = −α (y[k−1] − y[k−2]) + b1 u[k−1] + b2 u[k−2]
      rows.push([-(y[k - 1] - y[k - 2]), u[k - 1], u[k - 2]]);
      targ.push(y[k] - y[k - 1]);
    } else {
      rows.push([-y[k - 1], -y[k - 2], u[k - 1], u[k - 2]]);
      targ.push(y[k]);
    }
  }
  const p = integrator ? 3 : 4;
  const A = Array.from({ length: p }, () => new Array(p).fill(0));
  const b = new Array(p).fill(0);
  for (let r = 0; r < rows.length; r++) {
    for (let i = 0; i < p; i++) {
      b[i] += rows[r][i] * targ[r];
      for (let j = 0; j < p; j++) A[i][j] += rows[r][i] * rows[r][j];
    }
  }
  const scale = A.reduce((s, row, i) => s + row[i], 0) / p || 1;
  for (let i = 0; i < p; i++) A[i][i] += lambda * scale;

  const th = solve(A, b);
  if (!th || th.some((v) => !Number.isFinite(v))) return null;

  let a1, a2, b1, b2, alpha = null;
  if (integrator) {
    [alpha, b1, b2] = th;
    a1 = alpha - 1; a2 = -alpha;      // A(z) = (1 − z⁻¹)(1 + α z⁻¹)
  } else {
    [a1, a2, b1, b2] = th;
  }

  // 拟合优度：不管用哪种结构，R² 一律在 y[k] 本身上算，两种模型才可比。
  // （若在差分目标 Δy 上算，含积分模型的 R² 会天然偏低，看着像拟合很差）
  let ssRes = 0, ssTot = 0, sy = 0, ny = 0;
  for (let k = 2; k < N; k++) { sy += y[k]; ny++; }
  const yMean = sy / ny;
  for (let k = 2; k < N; k++) {
    const pred = -a1 * y[k - 1] - a2 * y[k - 2] + b1 * u[k - 1] + b2 * u[k - 2];
    ssRes += (y[k] - pred) ** 2;
    ssTot += (y[k] - yMean) ** 2;
  }
  const r2 = ssTot > 0 ? 1 - ssRes / ssTot : 0;

  // 离散极点 → 等效连续二阶参数（便于和"实验台"的模型对照）
  const disc = a1 * a1 - 4 * a2;
  let wn = null, zeta = null, tau = null, oscillatory = false;
  if (disc < 0) {
    oscillatory = true;
    const re = -a1 / 2, im = Math.sqrt(-disc) / 2;
    const mag = Math.hypot(re, im), ang = Math.atan2(im, re);
    const sRe = Math.log(mag) / Ts, sIm = ang / Ts;
    wn = Math.hypot(sRe, sIm);
    zeta = wn > 0 ? -sRe / wn : null;
  } else {
    const z1 = (-a1 + Math.sqrt(disc)) / 2, z2 = (-a1 - Math.sqrt(disc)) / 2;
    const s1 = z1 > 0 ? Math.log(z1) / Ts : null;
    const s2 = z2 > 0 ? Math.log(z2) / Ts : null;
    const slowest = [s1, s2].filter((v) => v !== null && v < 0).sort((a, b) => b - a)[0];
    if (slowest) tau = -1 / slowest;
  }
  const denom = 1 + a1 + a2;
  const K = (!integrator && Math.abs(denom) > 1e-9) ? (b1 + b2) / denom : null;
  // 含积分环节时静态增益无穷大，改报速度增益 Kv（单位输入产生的输出变化率）
  const Kv = integrator && Math.abs(1 + alpha) > 1e-9
    ? (b1 + b2) / ((1 + alpha) * Ts) : null;
  if (integrator) {
    const pole = -alpha;                       // 除积分器外的那个极点
    tau = (pole > 1e-6 && pole < 1) ? -Ts / Math.log(pole) : null;
    wn = null; zeta = null; oscillatory = false;
  }

  return {
    a1, a2, b1, b2, alpha, Ts, r2, integrator,
    K, Kv, wn, zeta, tau, oscillatory,
    stable: integrator ? Math.abs(alpha) < 1 : (Math.abs(a2) < 1 && Math.abs(a1) < 1 + a2),
  };
}

/** 用辨识出来的离散模型做闭环重仿真 */
export class DiscreteModel {
  constructor(m) { this.m = m; this.reset(); }
  reset() { this.y1 = 0; this.y2 = 0; this.u1 = 0; this.u2 = 0; }
  step(u) {
    const { a1, a2, b1, b2 } = this.m;
    const y = -a1 * this.y1 - a2 * this.y2 + b1 * this.u1 + b2 * this.u2;
    this.y2 = this.y1; this.y1 = y;
    this.u2 = this.u1; this.u1 = u;
    return y;
  }
}
