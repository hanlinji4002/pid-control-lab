/**
 * 被控对象模型
 *
 * 三个模型都来自真实小车的不同环路：
 *   speed   一阶惯性     PWM → 轮速         （电机时间常数主导）
 *   heading 惯性 + 积分  PWM差 → 航向角      （循线实际控制的对象，I 型系统）
 *   generic 标准二阶     教科书 ζ / ωn 模型
 *
 * 非理想项全部照着真机加：
 *   死区      PWM < ~40 电机不转（L298N + 减速箱静摩擦）
 *   饱和      MAX_SPEED = 150
 *   纯延迟    RoboMaster 相机 2~3 帧
 *   量化      Arduino 10 bit ADC vs ADS1115 16 bit
 *   噪声      传感器噪声 + 地面不平引起的过程噪声
 *   负载扰动  上坡 / 电池掉压
 */
import { mulberry32, gaussian } from './rng.js';

export const PLANT_TYPES = ['speed', 'heading', 'generic'];

export const DEFAULT_PLANT = {
  type: 'heading',
  K: 1.0,          // 静态增益
  tau: 0.18,       // 一阶时间常数 (s) —— 直流电机 + 减速箱典型值
  wn: 6.0,         // 二阶自然频率 (rad/s)
  zeta: 0.35,      // 二阶阻尼比
  deadTime: 0.03,  // 纯延迟 (s)
  deadband: 0.0,   // 输入死区（与 u 同量纲）
  uSat: 255,       // 执行器饱和
  measNoise: 0.0,  // 测量噪声标准差
  procNoise: 0.0,  // 过程噪声标准差
  quantLSB: 0,     // 测量量化步长，0 = 不量化
  disturbance: 0,  // 负载扰动幅值（在 tDist 之后作用于输出）
  tDist: 1e9,
  seed: 20260828,
};

export class Plant {
  constructor(cfg = {}) {
    this.cfg = { ...DEFAULT_PLANT, ...cfg };
    this.reset();
  }

  set(cfg) { Object.assign(this.cfg, cfg); this.reset(); }

  reset() {
    this.x = [0, 0];      // [输出, 输出导数]
    this.t = 0;
    this.rand = mulberry32(this.cfg.seed);
    this.delayBuf = [];
    this.delayDt = null;
  }

  /** 状态导数 —— u 已完成死区/饱和处理 */
  _deriv(x, u) {
    const c = this.cfg;
    switch (c.type) {
      case 'speed':   // τ ẏ = −y + K u
        return [(-x[0] + c.K * u) / c.tau, 0];
      case 'heading': // ω̇ = (K u − ω)/τ ,  θ̇ = ω   →  x = [θ, ω]
        return [x[1], (c.K * u - x[1]) / c.tau];
      default:        // ÿ + 2ζωn ẏ + ωn² y = K ωn² u
        return [x[1], c.K * c.wn * c.wn * u - 2 * c.zeta * c.wn * x[1] - c.wn * c.wn * x[0]];
    }
  }

  _shape(u) {
    const c = this.cfg;
    let v = Math.min(Math.max(u, -c.uSat), c.uSat);
    if (c.deadband > 0) {
      if (Math.abs(v) <= c.deadband) v = 0;
      else v = Math.sign(v) * (Math.abs(v) - c.deadband);
    }
    return v;
  }

  /**
   * 推进一个控制周期。内部用 RK4 细分，保证大 dt 下也稳定。
   * @returns {{y:number, yTrue:number, uEff:number}}
   */
  step(u, dt) {
    const c = this.cfg;
    const uEff = this._shape(u);
    const n = Math.max(1, Math.ceil(dt / 0.002));
    const h = dt / n;

    for (let k = 0; k < n; k++) {
      const k1 = this._deriv(this.x, uEff);
      const x2 = [this.x[0] + h / 2 * k1[0], this.x[1] + h / 2 * k1[1]];
      const k2 = this._deriv(x2, uEff);
      const x3 = [this.x[0] + h / 2 * k2[0], this.x[1] + h / 2 * k2[1]];
      const k3 = this._deriv(x3, uEff);
      const x4 = [this.x[0] + h * k3[0], this.x[1] + h * k3[1]];
      const k4 = this._deriv(x4, uEff);
      this.x[0] += h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
      this.x[1] += h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
      if (c.procNoise > 0) this.x[0] += gaussian(this.rand) * c.procNoise * Math.sqrt(h);
    }
    this.t += dt;

    let y = this.x[0];
    if (this.t >= c.tDist) y += c.disturbance;

    // 纯延迟：按控制周期存环形缓冲
    if (c.deadTime > 0) {
      this.delayBuf.push(y);
      const nDelay = Math.round(c.deadTime / dt);
      y = this.delayBuf.length > nDelay
        ? this.delayBuf[this.delayBuf.length - 1 - nDelay]
        : 0;
      if (this.delayBuf.length > nDelay + 4) this.delayBuf.shift();
    }

    const yTrue = y;
    if (c.measNoise > 0) y += gaussian(this.rand) * c.measNoise;
    if (c.quantLSB > 0) y = Math.round(y / c.quantLSB) * c.quantLSB;

    return { y, yTrue, uEff };
  }
}

/**
 * 闭环阶跃仿真
 * @param {PID}    pid
 * @param {object} plantCfg
 * @param {object} opts {tEnd, Ts, setpoint, rampTime}
 */
export function simulateStep(pid, plantCfg, opts = {}) {
  const { tEnd = 4, Ts = 0.01, setpoint = 1, preTime = 0.1 } = opts;
  const plant = new Plant(plantCfg);
  pid.reset();

  const N = Math.floor(tEnd / Ts) + 1;
  const t = new Float64Array(N), y = new Float64Array(N), r = new Float64Array(N);
  const u = new Float64Array(N), pT = new Float64Array(N), iT = new Float64Array(N), dT = new Float64Array(N);
  let sat = 0, meas = 0;

  for (let k = 0; k < N; k++) {
    const time = k * Ts;
    const sp = time < preTime ? 0 : setpoint;
    const out = pid.update(sp, meas, Ts);
    const res = plant.step(out.u, Ts);
    t[k] = time; r[k] = sp; y[k] = res.yTrue; u[k] = out.u;
    pT[k] = out.p; iT[k] = out.i; dT[k] = out.d;
    if (out.saturated) sat++;
    meas = res.y;
  }
  return { t, y, r, u, p: pT, i: iT, d: dT, satPct: (sat / N) * 100, Ts, preTime, setpoint };
}
