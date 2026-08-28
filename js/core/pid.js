/**
 * PID 控制器 —— 两种离散化实现
 *
 * 'standard'  u = Kp·e + Ki·∫e dt + Kd·de/dt        （RoboMaster EP 的 PID.calculate）
 * 'arduino'   u = Kp·e + Ki·Σe    + Kd·(e − e_prev) （line_follower_pid.ino 的 computePID）
 *
 * 两者数学上等价，只差一个 dt：
 *   Ki_std = Ki_ino / Ts        Kd_std = Kd_ino · Ts
 * 这正是同一台循线小车 Kp=0.6 而 RoboMaster Kp=280 的原因之一。
 * 'arduino' 模式下增益与循环周期强耦合 —— 换个板子、加几行 Serial.print 就得重调。
 */

export const PID_MODES = ['standard', 'arduino'];
export const ANTIWINDUP = ['clamp', 'conditional', 'backcalc', 'none'];

export const DEFAULT_PID = {
  kp: 1.0,
  ki: 0.0,
  kd: 0.0,
  mode: 'standard',
  iMax: 100,            // 积分累加器限幅（对应 .ino 里的 I_MAX）
  uMin: -255,
  uMax: 255,
  antiWindup: 'clamp',
  kBackCalc: 1.0,       // backcalc 反算系数
  dOnMeasurement: false, // 微分先行：对测量值求导，避免设定值跳变时的微分冲击
  dSmooth: 0.0,         // 微分低通 0..0.95（0 = 不滤波）
  setpointWeight: 1.0,  // b 值：P 项用 (b·r − y)，b<1 可降超调
};

export class PID {
  constructor(cfg = {}) {
    this.cfg = { ...DEFAULT_PID, ...cfg };
    this.reset();
  }

  set(cfg) { Object.assign(this.cfg, cfg); }

  reset() {
    this.integral = 0;
    this.prevError = null;
    this.prevMeas = null;
    this.dFiltered = 0;
    this.last = { p: 0, i: 0, d: 0, u: 0, uRaw: 0, saturated: false };
  }

  /**
   * @param {number} r  设定值
   * @param {number} y  测量值
   * @param {number} dt 采样周期（秒）
   */
  update(r, y, dt) {
    const c = this.cfg;
    const error = r - y;
    const arduino = c.mode === 'arduino';

    // ── P（可带设定值加权 b）
    const pError = c.setpointWeight * r - y;
    const p = c.kp * pError;

    // ── D
    let dRaw;
    if (c.dOnMeasurement) {
      const prev = this.prevMeas === null ? y : this.prevMeas;
      dRaw = arduino ? -c.kd * (y - prev) : -c.kd * (y - prev) / dt;
    } else {
      const prev = this.prevError === null ? error : this.prevError;
      dRaw = arduino ? c.kd * (error - prev) : c.kd * (error - prev) / dt;
    }
    const a = Math.min(Math.max(c.dSmooth, 0), 0.95);
    this.dFiltered = a * this.dFiltered + (1 - a) * dRaw;
    const d = this.dFiltered;

    // ── I（先试算，之后按抗饱和策略修正）
    const inc = arduino ? error : error * dt;
    let integral = this.integral;
    const willSaturate = (u) => u > c.uMax || u < c.uMin;

    if (c.antiWindup === 'conditional') {
      // 条件积分：输出已饱和且误差还在把它往更深处推 → 本拍不积分
      const trial = p + c.ki * (integral + inc) + d;
      const uNow = p + c.ki * integral + d;
      const pushingOut =
        (uNow >= c.uMax && trial > uNow) || (uNow <= c.uMin && trial < uNow);
      if (!(willSaturate(uNow) && pushingOut)) integral += inc;
    } else {
      integral += inc;
    }

    if (c.antiWindup === 'clamp') {
      integral = Math.min(Math.max(integral, -c.iMax), c.iMax);
    }

    let i = c.ki * integral;
    let uRaw = p + i + d;
    let u = Math.min(Math.max(uRaw, c.uMin), c.uMax);

    if (c.antiWindup === 'backcalc' && uRaw !== u) {
      // 反算：把饱和掉的那部分按 kBackCalc 退回积分器
      integral += c.kBackCalc * (u - uRaw) * (arduino ? 1 : dt) / (c.ki || 1e-9);
      integral = Math.min(Math.max(integral, -c.iMax * 10), c.iMax * 10);
      i = c.ki * integral;
      uRaw = p + i + d;
      u = Math.min(Math.max(uRaw, c.uMin), c.uMax);
    }

    this.integral = integral;
    this.prevError = error;
    this.prevMeas = y;
    this.last = { p, i, d, u, uRaw, error, saturated: uRaw !== u };
    return this.last;
  }

  /** 直接喂误差（日志回放用：误差已由真机传感器算好） */
  updateFromError(error, dt) {
    return this.update(error, 0, dt);
  }
}

/** .ino 增益 → 标准增益（给定循环周期 Ts 秒） */
export function arduinoToStandard({ kp, ki, kd }, Ts) {
  return { kp, ki: ki / Ts, kd: kd * Ts };
}

/** 标准增益 → .ino 增益 */
export function standardToArduino({ kp, ki, kd }, Ts) {
  return { kp, ki: ki * Ts, kd: kd / Ts };
}
