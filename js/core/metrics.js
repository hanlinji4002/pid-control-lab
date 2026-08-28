/** 阶跃响应性能指标 */

/**
 * @param {{t,y,r,u,Ts,preTime,setpoint,satPct}} sim
 * @param {number} band 调节时间误差带（默认 ±2%）
 */
export function stepMetrics(sim, band = 0.02) {
  const { t, y, r, u, Ts, setpoint } = sim;
  const N = t.length;

  // 阶跃发生的下标
  let k0 = 0;
  for (let k = 1; k < N; k++) { if (r[k] !== r[k - 1]) { k0 = k; break; } }
  const t0 = t[k0];
  const sp = setpoint;

  // 稳态值：末尾 5% 平均
  const tailStart = Math.max(k0, N - Math.max(3, Math.floor(N * 0.05)));
  let yFinal = 0, cnt = 0;
  for (let k = tailStart; k < N; k++) { yFinal += y[k]; cnt++; }
  yFinal /= cnt;

  // 噪声底：稳态段的标准差
  let varSum = 0;
  for (let k = tailStart; k < N; k++) varSum += (y[k] - yFinal) ** 2;
  const noiseStd = Math.sqrt(varSum / Math.max(1, cnt - 1));

  // 峰值检测的滑动平均窗按噪声自适应：干净信号不平滑（否则会系统性低估超调），
  // 有噪声时才平滑，免得单点尖峰被当成超调
  const noisy = noiseStd > Math.abs(sp) * 1e-3;
  const half = noisy ? Math.max(1, Math.round(N * 0.015)) : 0;
  let yPeak = -Infinity, kPeak = k0;
  for (let k = k0; k < N; k++) {
    let sm = y[k];
    if (half > 0) {
      let s2 = 0, c2 = 0;
      for (let j = Math.max(k0, k - half); j <= Math.min(N - 1, k + half); j++) { s2 += y[j]; c2++; }
      sm = s2 / c2;
    }
    if (sm > yPeak) { yPeak = sm; kPeak = k; }
  }

  // 峰值高出设定值不到 2σ 就不算超调 ——
  // 否则一条明明没有超调的曲线，会因为几个噪声尖峰报出"超调 1%，峰值时间 2.7 s"
  const realPeak = yPeak - sp > 2 * noiseStd;
  const overshoot = (sp !== 0 && realPeak) ? ((yPeak - sp) / Math.abs(sp)) * 100 : 0;
  const ess = sp - yFinal;
  const essPct = sp !== 0 ? (ess / Math.abs(sp)) * 100 : 0;

  // 上升时间 10% → 90%（相对设定值）
  let t10 = null, t90 = null;
  for (let k = k0; k < N; k++) {
    if (t10 === null && y[k] >= 0.1 * sp) t10 = t[k];
    if (t90 === null && y[k] >= 0.9 * sp) { t90 = t[k]; break; }
  }
  const riseTime = (t10 !== null && t90 !== null) ? t90 - t10 : null;

  // 调节时间：最后一次离开误差带之后
  const tol = Math.abs(sp) * band;
  let kSettle = null;
  for (let k = N - 1; k >= k0; k--) {
    if (Math.abs(y[k] - sp) > tol) { kSettle = k + 1; break; }
  }
  if (kSettle === null) kSettle = k0;
  const settleTime = kSettle < N ? t[kSettle] - t0 : null;

  // 误差积分与控制代价
  let iae = 0, ise = 0, itae = 0, u2 = 0, du2 = 0;
  for (let k = k0; k < N; k++) {
    const e = Math.abs(r[k] - y[k]);
    const w = (k === k0 || k === N - 1) ? 0.5 : 1;
    iae += w * e * Ts;
    ise += w * e * e * Ts;
    itae += w * (t[k] - t0) * e * Ts;
    u2 += w * u[k] * u[k] * Ts;
    if (k > k0) du2 += Math.abs(u[k] - u[k - 1]);
  }
  const dur = t[N - 1] - t0;
  const uRms = Math.sqrt(u2 / Math.max(dur, 1e-9));

  // 振荡次数：带 2% 回差，免得测量噪声在设定值附近抖出几十次假穿越
  let cross = 0, armed = false, side = 0;
  const hyst = Math.abs(sp) * 0.02 + 1e-12;
  for (let k = k0; k < N; k++) {
    const dev = y[k] - sp;
    if (Math.abs(dev) > hyst) {
      const s2 = Math.sign(dev);
      if (armed && s2 !== side) cross++;
      side = s2; armed = true;
    }
  }

  function isStable() {
    if (!Number.isFinite(yFinal) || Math.abs(yFinal) > Math.abs(sp) * 50 + 1e3) return false;
    const span = N - k0;
    if (span < 20) return true;
    const mid = rmsRange(y, k0 + Math.floor(span * 0.3), k0 + Math.floor(span * 0.5), sp);
    const end = rmsRange(y, k0 + Math.floor(span * 0.8), N, sp);
    return !(end > mid * 2 + Math.abs(sp) * 0.05);
  }

  return {
    overshoot, yPeak, peakTime: realPeak ? t[kPeak] - t0 : null, noiseStd,
    ess, essPct, yFinal,
    riseTime, settleTime, band,
    iae, ise, itae, uRms, uEffort: du2, crossings: cross,
    satPct: sim.satPct ?? 0,
    stable: isStable(),
  };
}

/** 判稳：末值有界，且后段幅值没有相对中段明显增长（纯看末值会漏掉缓慢发散） */
function rmsRange(y, a, b, ref) {
  let s = 0, n = 0;
  for (let k = a; k < b; k++) { s += (y[k] - ref) ** 2; n++; }
  return n ? Math.sqrt(s / n) : 0;
}

export function fmt(v, digits = 3, unit = '') {
  if (v === null || v === undefined || !Number.isFinite(v)) return '—';
  const a = Math.abs(v);
  let s;
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) s = v.toExponential(2);
  else s = v.toFixed(digits);
  return s + unit;
}

export function fmtMs(v, fallback = '未收敛') {
  if (v === null || v === undefined || !Number.isFinite(v)) return fallback;
  return (v * 1000).toFixed(0) + ' ms';
}
