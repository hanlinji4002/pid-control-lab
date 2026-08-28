/** 赛道：中心线折线 + 空间网格最近点查询 + 圈速计时 */

export const TRACK_TYPES = {
  oval:      { label: '椭圆跑道',   labelEn: 'Oval' },
  figure8:   { label: '8 字',       labelEn: 'Figure-8' },
  slalom:    { label: 'S 弯',       labelEn: 'Slalom' },
  rightAngle:{ label: '直角赛道',   labelEn: 'Right angles' },
  hairpin:   { label: '发夹弯',     labelEn: 'Hairpin' },
};

function sample(fn, n) {
  const pts = [];
  for (let i = 0; i < n; i++) pts.push(fn((i / n) * Math.PI * 2, i / n));
  return pts;
}

function buildPolyline(type) {
  switch (type) {
    case 'oval':
      return sample((a) => ({ x: 1.05 * Math.cos(a), y: 0.62 * Math.sin(a) }), 900);
    case 'figure8':
      return sample((a) => ({ x: 1.05 * Math.sin(a), y: 0.62 * Math.sin(2 * a) }), 1100);
    case 'slalom':
      return sample((a) => ({
        x: 1.10 * Math.cos(a),
        y: 0.55 * Math.sin(a) + 0.20 * Math.sin(5 * a),
      }), 1400);
    case 'hairpin':
      return sample((a) => {
        const r = 0.75 + 0.42 * Math.cos(3 * a);
        return { x: r * Math.cos(a) * 1.15, y: r * Math.sin(a) * 0.8 };
      }, 1400);
    case 'rightAngle': {
      // 直角赛道：正是 02_line_follower_ads1115 那一版要对付的场景
      const corners = [
        [-1.1, -0.6], [0.2, -0.6], [0.2, 0.1], [-0.4, 0.1],
        [-0.4, 0.6], [1.1, 0.6], [1.1, -0.15], [0.7, -0.15],
        [0.7, -0.95], [-1.1, -0.95],
      ];
      const pts = [];
      for (let i = 0; i < corners.length; i++) {
        const [x0, y0] = corners[i];
        const [x1, y1] = corners[(i + 1) % corners.length];
        const len = Math.hypot(x1 - x0, y1 - y0);
        const n = Math.max(2, Math.round(len / 0.006));
        for (let k = 0; k < n; k++) {
          pts.push({ x: x0 + (x1 - x0) * k / n, y: y0 + (y1 - y0) * k / n });
        }
      }
      return pts;
    }
    default:
      return buildPolyline('oval');
  }
}

export class Track {
  constructor(type = 'oval', lineWidth = 0.019 /* 电工胶带宽度 */) {
    this.type = type;
    this.lineWidth = lineWidth;
    this.pts = buildPolyline(type);
    this._prepare();
  }

  _prepare() {
    const p = this.pts;
    const n = p.length;
    // 弧长
    this.s = new Float64Array(n);
    for (let i = 1; i < n; i++) {
      this.s[i] = this.s[i - 1] + Math.hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    }
    this.length = this.s[n - 1] + Math.hypot(p[0].x - p[n - 1].x, p[0].y - p[n - 1].y);

    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const q of p) {
      minX = Math.min(minX, q.x); maxX = Math.max(maxX, q.x);
      minY = Math.min(minY, q.y); maxY = Math.max(maxY, q.y);
    }
    this.bounds = { minX, minY, maxX, maxY };

    // 均匀网格加速最近点查询（每步要查 3~4 个传感器点）
    this.cell = 0.06;
    this.grid = new Map();
    const key = (i, j) => i + ',' + j;
    this._key = key;
    for (let i = 0; i < n; i++) {
      const ci = Math.floor(p[i].x / this.cell), cj = Math.floor(p[i].y / this.cell);
      for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
        const k = key(ci + di, cj + dj);
        if (!this.grid.has(k)) this.grid.set(k, []);
        this.grid.get(k).push(i);
      }
    }
    this.start = { x: p[0].x, y: p[0].y };
    const t0 = this.tangentAt(0);
    this.startNormal = { x: -t0.y, y: t0.x };
  }

  tangentAt(i) {
    const n = this.pts.length;
    const a = this.pts[(i - 1 + n) % n], b = this.pts[(i + 1) % n];
    const dx = b.x - a.x, dy = b.y - a.y;
    const L = Math.hypot(dx, dy) || 1;
    return { x: dx / L, y: dy / L };
  }

  /** @returns {{dist, idx, s, signed}} signed>0 表示点在赛道前进方向的左侧 */
  nearest(x, y) {
    const ci = Math.floor(x / this.cell), cj = Math.floor(y / this.cell);
    let cand = this.grid.get(this._key(ci, cj));
    if (!cand) {  // 落在网格外 → 退化为全量扫描（步长 5，够用）
      let best = { dist: Infinity, idx: 0 };
      for (let i = 0; i < this.pts.length; i += 5) {
        const d = Math.hypot(this.pts[i].x - x, this.pts[i].y - y);
        if (d < best.dist) best = { dist: d, idx: i };
      }
      cand = [best.idx];
    }
    let bd = Infinity, bi = 0;
    for (const i of cand) {
      const d = (this.pts[i].x - x) ** 2 + (this.pts[i].y - y) ** 2;
      if (d < bd) { bd = d; bi = i; }
    }
    const dist = Math.sqrt(bd);
    const t = this.tangentAt(bi);
    const signed = -((x - this.pts[bi].x) * -t.y + (y - this.pts[bi].y) * t.x);
    return { dist, idx: bi, s: this.s[bi], signed };
  }

  /** 传感器读数：0 = 白地，1 = 黑线中心（高斯响应，近似 TCRT5000 光斑） */
  darkness(x, y, spotRadius = 0.006) {
    const { dist } = this.nearest(x, y);
    const sigma = (this.lineWidth / 2 + spotRadius) / 1.4;
    return Math.exp(-(dist * dist) / (2 * sigma * sigma));
  }
}

/**
 * 圈速计时：几何检测穿越起跑线
 *
 * 不用"弧长绕回 0"来判圈 —— 8 字赛道自交，最近点会在两条支路之间跳变，
 * 弧长法会误判。改成检测车体沿起点切线方向穿过起跑线，并要求两次判圈之间
 * 至少跑够半圈路程，对所有赛道都成立。
 */
export class LapTimer {
  constructor(track) {
    this.track = track;
    this.reset();
  }
  reset() {
    this.laps = [];
    this.prev = null;
    this.lapStart = 0;
    this.distAtLap = 0;
    this.dist = 0;
  }
  /** @param {number} dist 车辆累计行驶里程（m） */
  update(x, y, t, dist) {
    const T = this.track;
    const p0 = T.start;
    const tan = T.tangentAt(0);
    const nrm = T.startNormal;
    const along = (x - p0.x) * tan.x + (y - p0.y) * tan.y;
    const lateral = (x - p0.x) * nrm.x + (y - p0.y) * nrm.y;
    this.dist = dist;

    let lap = null;
    if (this.prev !== null &&
        this.prev.along < 0 && along >= 0 &&
        Math.abs(lateral) < 0.25 &&
        dist - this.distAtLap > T.length * 0.5) {
      lap = t - this.lapStart;
      this.lapStart = t;
      this.distAtLap = dist;
      this.laps.push(lap);
    }
    this.prev = { along, lateral };
    return lap;
  }
  get best() { return this.laps.length ? Math.min(...this.laps) : null; }
  get last() { return this.laps.length ? this.laps[this.laps.length - 1] : null; }
  get progress() {
    return Math.min(1, (this.dist - this.distAtLap) / this.track.length);
  }
}
