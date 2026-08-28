/**
 * 循线小车仿真 —— 两套真机配置
 *
 *  profile 'arduino'    2WD + L298N + 3× TCRT5000
 *      误差 = (normR − normL)·100 / total          （line_follower_pid.ino）
 *      PID  = 'arduino' 离散化（积分不乘 dt、微分不除 dt）
 *      执行 = leftPWM = base + out, rightPWM = base − out，再乘左右补偿百分比、限幅 MAX_SPEED
 *      丢线 = 按最后已知偏向以 ±80 全力搜索
 *
 *  profile 'robomaster' EP + 摄像头 HSV 扫描行
 *      误差 = 蓝线归一化中心 − 0.5                  （robot_finalproject.py）
 *      PID  = 'standard' 离散化，30 Hz，相机 2 帧延迟
 *      执行 = z 限幅 Z_MAX，前进速度按 |误差| 在 V_MIN..V_MAX 之间降速
 *
 * 车体统一用差速运动学：v=(vL+vR)/2, ω=(vR−vL)/W（θ 逆时针为正）。
 */
import { PID } from '../core/pid.js';
import { Track, LapTimer } from './track.js';
import { mulberry32, gaussian } from '../core/rng.js';

export const PROFILES = {
  arduino: {
    label: 'Arduino 2WD · TCRT5000',
    pidMode: 'arduino',
    gains: { kp: 0.6, ki: 0.0, kd: 6.0 },
    Ts: 0.01,
    params: {
      baseSpeed: 80, maxSpeed: 150, lostSpeed: 50, iMax: 100,
      lineThreshold: 0.15, lostError: 80,
      leftPercent: 150, rightPercent: 180,
      sensorSpacing: 0.030, lookahead: 0.055, spotRadius: 0.006,
      wheelBase: 0.130, kv: 0.00325, motorTau: 0.12, deadbandPWM: 28,
      motorImbalance: 0.10, adc: 'uno',
    },
  },
  robomaster: {
    label: 'RoboMaster EP · 摄像头',
    pidMode: 'standard',
    gains: { kp: 280, ki: 0.0, kd: 30.0 },
    Ts: 1 / 30,
    params: {
      vBase: 0.25, vMin: 0.10, vMax: 0.35, zMax: 250, iMax: 100,
      slowGain: 1.6, camLookahead: 0.25, camHalfWidth: 0.22,
      camDelayFrames: 2, camNoise: 0.004, detectFail: 0.0,
      yawTau: 0.15, wheelBase: 0.30,
      noseenRotate: 0.8, searchZ: 80,
    },
  },
};

const ADC = {
  uno:     { lsb: 1 / 250, noise: 0.012, label: 'Arduino UNO 10-bit (白450~黑700)' },
  ads1115: { lsb: 1 / 16000, noise: 0.003, label: 'ADS1115 16-bit' },
};

export class LineSim {
  constructor(opts = {}) {
    this.trackType = opts.trackType || 'oval';
    this.profileKey = opts.profile || 'arduino';
    this.track = new Track(this.trackType);
    this.profile = structuredClone(PROFILES[this.profileKey]);
    if (opts.gains) Object.assign(this.profile.gains, opts.gains);
    if (opts.params) Object.assign(this.profile.params, opts.params);
    this.seed = opts.seed ?? 20260828;
    this.reset();
  }

  setTrack(type) { this.trackType = type; this.track = new Track(type); this.reset(); }
  setProfile(key) {
    this.profileKey = key;
    this.profile = structuredClone(PROFILES[key]);
    this.reset();
  }
  setGains(g) { Object.assign(this.profile.gains, g); this.pid.set(g); }
  setParams(p) { Object.assign(this.profile.params, p); this.reset(); }

  reset() {
    const p = this.profile.params;
    const t = this.track;
    // 起点：赛道起点，朝切线方向
    const tan = t.tangentAt(0);
    this.x = t.start.x; this.y = t.start.y;
    this.theta = Math.atan2(tan.y, tan.x);
    this.vL = 0; this.vR = 0; this.omega = 0;
    this.t = 0;
    this.rand = mulberry32(this.seed);
    this.camBuf = Array.from({ length: (this.profile.params.camDelayFrames || 0) + 1 },
      () => ({ seen: true, pos: 0.5 }));
    this.lostTimer = 0;
    this.seenLine = false;
    this.lastValidError = 0;
    this.state = 'WAITING';
    this.pid = new PID({
      ...this.profile.gains,
      mode: this.profile.pidMode,
      iMax: p.iMax,
      uMin: -1e6, uMax: 1e6,
      antiWindup: 'clamp',
    });
    this.lap = new LapTimer(t);
    this.trace = { t: [], cte: [], error: [], u: [], v: [], path: [] };
    this.stats = { sumAbs: 0, sumSq: 0, n: 0, maxAbs: 0, lostCount: 0, lostTime: 0, dist: 0, satTime: 0 };
    this.sensors = [0, 0, 0];
    this.pwm = { L: 0, R: 0 };
    this.linePos = 0.5;
    this.lineSeen = true;
    this.finished = false;
  }

  get Ts() { return this.profile.Ts; }

  /** 车体坐标 (fwd, lat) → 世界坐标；lat > 0 在车的左侧 */
  _toWorld(fwd, lat) {
    const c = Math.cos(this.theta), s = Math.sin(this.theta);
    return { x: this.x + fwd * c - lat * s, y: this.y + fwd * s + lat * c };
  }

  // ── 传感器 ──

  _readIR() {
    const p = this.profile.params;
    const adc = ADC[p.adc] || ADC.uno;
    const out = [];
    for (const lat of [+p.sensorSpacing, 0, -p.sensorSpacing]) {  // 左 中 右
      const w = this._toWorld(p.lookahead, lat);
      let v = this.track.darkness(w.x, w.y, p.spotRadius);
      v += gaussian(this.rand) * adc.noise;
      v = Math.min(1, Math.max(0, v));
      v = Math.round(v / adc.lsb) * adc.lsb;
      out.push(v);
    }
    return out;
  }

  _readCamera() {
    const p = this.profile.params;
    // 在扫描行上横向扫一遍，取"蓝线像素"的加权中心（对应 np.median(blue_pixels)）
    const N = 61, W = p.camHalfWidth;
    let sum = 0, wsum = 0;
    for (let i = 0; i < N; i++) {
      const lat = W - (2 * W * i) / (N - 1);
      const w = this._toWorld(p.camLookahead, lat);
      const d = this.track.darkness(w.x, w.y, 0.004);
      if (d > 0.35) { sum += lat * d; wsum += d; }
    }
    if (wsum <= 0 || this.rand() < p.detectFail) return { seen: false, pos: this.linePos };
    const lat = sum / wsum;
    // 图像 x 向右为正 → 车体左为正，符号相反
    let pos = 0.5 - lat / (2 * W) + gaussian(this.rand) * p.camNoise;
    return { seen: true, pos };
  }

  // ── 单个控制周期 ──

  step() {
    const dt = this.Ts;
    const p = this.profile.params;
    let u = 0, vCmd = 0, pwmL = 0, pwmR = 0;

    if (this.profileKey === 'arduino') {
      const [nL, nM, nR] = this._readIR();
      this.sensors = [nL, nM, nR];
      const total = nL + nM + nR;
      let error;
      if (total >= p.lineThreshold) {
        error = (nR - nL) * 100 / total;
        this.lastValidError = error;
        this.seenLine = true;
        this.lineSeen = true;
        this.state = error > 10 ? 'TURN_RIGHT' : error < -10 ? 'TURN_LEFT' : 'STRAIGHT';
      } else {
        this.lineSeen = false;
        if (!this.seenLine) { error = 0; this.state = 'WAITING'; }
        else {
          error = this.lastValidError >= 0 ? p.lostError : -p.lostError;
          if (this.state !== 'SEARCHING') this.stats.lostCount++;
          this.state = 'SEARCHING';
          this.stats.lostTime += dt;
        }
      }
      u = this.pid.updateFromError(error, dt).u;
      const base = this.state === 'WAITING' ? 0 : this.state === 'SEARCHING' ? p.lostSpeed : p.baseSpeed;
      let L = base + u, R = base - u;
      const clampPWM = (v, pct) => {
        const sign = Math.sign(v) || 1;
        let m = Math.abs(v) * pct / 100;
        if (m > p.maxSpeed) { m = p.maxSpeed; this.stats.satTime += dt / 2; }
        return sign * m;
      };
      pwmL = clampPWM(L, p.leftPercent);
      pwmR = clampPWM(R, p.rightPercent);
      this.pwm = { L: pwmL, R: pwmR };

      // PWM → 轮速（死区 + 左右不一致 + 电机一阶滞后）
      const wheel = (pwm, gain) => {
        const s = Math.sign(pwm);
        const m = Math.abs(pwm);
        return m <= p.deadbandPWM ? 0 : s * (m - p.deadbandPWM) * p.kv * gain;
      };
      const tgtL = wheel(pwmL, 1 + p.motorImbalance / 2);
      const tgtR = wheel(pwmR, 1 - p.motorImbalance / 2);
      this.vL += (tgtL - this.vL) * Math.min(1, dt / p.motorTau);
      this.vR += (tgtR - this.vR) * Math.min(1, dt / p.motorTau);
      vCmd = (this.vL + this.vR) / 2;
      this.omega = (this.vR - this.vL) / p.wheelBase;
      this._integrate(dt, vCmd, this.omega);
      this._record(dt, error, u, vCmd);
    } else {
      const cam = this._readCamera();
      this.camBuf.push(cam);
      const frame = this.camBuf.length > p.camDelayFrames
        ? this.camBuf[this.camBuf.length - 1 - p.camDelayFrames] : { seen: false, pos: 0.5 };
      if (this.camBuf.length > p.camDelayFrames + 3) this.camBuf.shift();

      let error;
      if (frame.seen) {
        this.linePos = frame.pos;
        this.lineSeen = true;
        this.lostTimer = 0;
        this.seenLine = true;
        error = frame.pos - 0.5;
        this.lastValidError = error;
        this.state = 'FOLLOW';
      } else {
        this.lineSeen = false;
        this.lostTimer += dt;
        if (this.state !== 'SEARCHING') this.stats.lostCount++;
        this.state = 'SEARCHING';
        this.stats.lostTime += dt;
        error = Math.sign(this.lastValidError || 1) * 0.5;
      }
      const raw = this.pid.updateFromError(error, dt).u;
      let z = Math.min(Math.max(raw, -p.zMax), p.zMax);
      if (z !== raw) this.stats.satTime += dt;
      if (this.state === 'SEARCHING') z = Math.sign(this.lastValidError || 1) * p.searchZ;
      u = z;
      vCmd = this.state === 'SEARCHING'
        ? 0.05
        : Math.min(p.vMax, Math.max(p.vMin, p.vBase * (1 - p.slowGain * Math.abs(error))));
      // z 为 deg/s，SDK 里正值 = 顺时针；世界坐标 θ 逆时针为正
      const targetOmega = -z * Math.PI / 180;
      this.omega += (targetOmega - this.omega) * Math.min(1, dt / p.yawTau);
      this.vL = vCmd - this.omega * p.wheelBase / 2;
      this.vR = vCmd + this.omega * p.wheelBase / 2;
      this._integrate(dt, vCmd, this.omega);
      this._record(dt, error, u, vCmd);
    }
    return this.state;
  }

  _integrate(dt, v, omega) {
    const n = Math.max(1, Math.ceil(dt / 0.002));
    const h = dt / n;
    for (let k = 0; k < n; k++) {
      this.theta += omega * h;
      this.x += v * Math.cos(this.theta) * h;
      this.y += v * Math.sin(this.theta) * h;
    }
    this.t += dt;
    this.stats.dist += Math.abs(v) * dt;
  }

  _record(dt, error, u, v) {
    const near = this.track.nearest(this.x, this.y);
    const cte = near.signed;
    const a = Math.abs(cte);
    if (this.state !== 'WAITING') {
      this.stats.sumAbs += a * dt;
      this.stats.sumSq += a * a * dt;
      this.stats.n += dt;
      if (a > this.stats.maxAbs) this.stats.maxAbs = a;
    }
    this.lap.update(this.x, this.y, this.t, this.stats.dist);
    const tr = this.trace;
    tr.t.push(this.t); tr.cte.push(cte); tr.error.push(error); tr.u.push(u); tr.v.push(v);
    tr.path.push({ x: this.x, y: this.y, lost: !this.lineSeen });
    const LIM = 20000;
    if (tr.t.length > LIM) {
      for (const k of ['t', 'cte', 'error', 'u', 'v', 'path']) tr[k].splice(0, tr[k].length - LIM);
    }
    // 冲出赛道太远视为失败
    if (a > 0.5) this.finished = true;
  }

  get metrics() {
    const s = this.stats;
    const n = Math.max(s.n, 1e-9);
    return {
      meanAbsCte: s.sumAbs / n,
      rmsCte: Math.sqrt(s.sumSq / n),
      maxAbsCte: s.maxAbs,
      lostCount: s.lostCount,
      lostTime: s.lostTime,
      distance: s.dist,
      satPct: (s.satTime / Math.max(this.t, 1e-9)) * 100,
      laps: this.lap.laps.length,
      bestLap: this.lap.best,
      lastLap: this.lap.last,
      time: this.t,
      offTrack: this.finished,
    };
  }
}

/** 不画图、纯跑数：自动整定和参数对比用 */
export function runHeadless(opts, seconds = 30) {
  const sim = new LineSim(opts);
  const N = Math.floor(seconds / sim.Ts);
  for (let k = 0; k < N && !sim.finished; k++) sim.step();
  return sim;
}

export { ADC };
