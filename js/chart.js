/** 轻量 Canvas 折线图（无依赖）：多曲线 + 误差带 + 标记 + 十字光标 */

const CSS = (name, fallback) => {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback;
};

function niceTicks(min, max, count = 6) {
  if (!Number.isFinite(min) || !Number.isFinite(max) || min === max) {
    const c = Number.isFinite(min) ? min : 0;
    return [c - 1, c, c + 1];
  }
  const raw = (max - min) / count;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  const step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
  const start = Math.ceil(min / step) * step;
  const out = [];
  for (let v = start; v <= max + step * 1e-6; v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  return out;
}

function fmtTick(v, step) {
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (a >= 1e5 || a < 1e-3) return v.toExponential(1);
  const dec = Math.max(0, Math.min(4, Math.ceil(-Math.log10(step || a)) + 1));
  return v.toFixed(dec).replace(/\.?0+$/, '') || '0';
}

export class Chart {
  constructor(canvas, opts = {}) {
    this.cv = canvas;
    this.ctx = canvas.getContext('2d');
    this.opts = {
      xLabel: '', yLabel: '', yRight: null,
      padding: { l: 56, r: 16, t: 14, b: 34 },
      legend: true, crosshair: true, xFixed: null, yFixed: null,
      unitX: '', unitY: '',
      ...opts,
    };
    this.series = [];
    this.bands = [];
    this.vlines = [];
    this.markers = [];
    this.hover = null;
    this._bind();
    this._ro = new ResizeObserver(() => this.draw());
    this._ro.observe(canvas.parentElement || canvas);
  }

  _bind() {
    if (!this.opts.crosshair) return;
    this.cv.addEventListener('mousemove', (e) => {
      const r = this.cv.getBoundingClientRect();
      this.hover = { x: e.clientX - r.left, y: e.clientY - r.top };
      this.draw();
    });
    this.cv.addEventListener('mouseleave', () => { this.hover = null; this.draw(); });
  }

  setSeries(series) { this.series = series.filter(Boolean); return this; }
  setBands(bands) { this.bands = bands || []; return this; }
  setVLines(v) { this.vlines = v || []; return this; }
  setMarkers(m) { this.markers = m || []; return this; }
  destroy() { this._ro.disconnect(); }

  _domain() {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const s of this.series) {
      if (s.hidden) continue;
      const n = Math.min(s.x.length, s.y.length);
      for (let i = 0; i < n; i++) {
        const xv = s.x[i], yv = s.y[i];
        if (!Number.isFinite(xv) || !Number.isFinite(yv)) continue;
        if (xv < x0) x0 = xv; if (xv > x1) x1 = xv;
        if (yv < y0) y0 = yv; if (yv > y1) y1 = yv;
      }
    }
    for (const b of this.bands) { y0 = Math.min(y0, b.y0); y1 = Math.max(y1, b.y1); }
    if (!Number.isFinite(x0)) { x0 = 0; x1 = 1; }
    if (!Number.isFinite(y0)) { y0 = 0; y1 = 1; }
    if (this.opts.xFixed) [x0, x1] = this.opts.xFixed;
    if (this.opts.yFixed) [y0, y1] = this.opts.yFixed;
    if (x1 - x0 < 1e-12) x1 = x0 + 1;
    const pad = (y1 - y0) * 0.08 || 0.5;
    if (!this.opts.yFixed) { y0 -= pad; y1 += pad; }
    return { x0, x1, y0, y1 };
  }

  draw() {
    const cv = this.cv, ctx = this.ctx;
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth || 600, h = cv.clientHeight || 260;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
      cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const P = this.opts.padding;
    const pw = Math.max(10, w - P.l - P.r), ph = Math.max(10, h - P.t - P.b);
    const d = this._domain();
    const X = (v) => P.l + ((v - d.x0) / (d.x1 - d.x0)) * pw;
    const Y = (v) => P.t + ph - ((v - d.y0) / (d.y1 - d.y0)) * ph;
    this._map = { X, Y, d, P, pw, ph };

    const cGrid = CSS('--grid', '#22272e');
    const cAxis = CSS('--muted', '#8b949e');
    const cText = CSS('--text', '#e6edf3');

    // 误差带
    for (const b of this.bands) {
      ctx.fillStyle = b.color || 'rgba(88,166,255,0.08)';
      ctx.fillRect(P.l, Y(b.y1), pw, Math.max(1, Y(b.y0) - Y(b.y1)));
    }

    // 网格 + 刻度
    ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace';
    ctx.strokeStyle = cGrid; ctx.lineWidth = 1;
    const yt = niceTicks(d.y0, d.y1, 5), ystep = yt.length > 1 ? yt[1] - yt[0] : 1;
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (const v of yt) {
      const y = Math.round(Y(v)) + 0.5;
      if (y < P.t - 1 || y > P.t + ph + 1) continue;
      ctx.beginPath(); ctx.moveTo(P.l, y); ctx.lineTo(P.l + pw, y); ctx.stroke();
      ctx.fillStyle = cAxis; ctx.fillText(fmtTick(v, ystep), P.l - 8, y);
    }
    const xt = niceTicks(d.x0, d.x1, 6), xstep = xt.length > 1 ? xt[1] - xt[0] : 1;
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (const v of xt) {
      const x = Math.round(X(v)) + 0.5;
      if (x < P.l - 1 || x > P.l + pw + 1) continue;
      ctx.beginPath(); ctx.moveTo(x, P.t); ctx.lineTo(x, P.t + ph); ctx.stroke();
      ctx.fillStyle = cAxis; ctx.fillText(fmtTick(v, xstep), x, P.t + ph + 6);
    }

    // 竖直参考线
    for (const v of this.vlines) {
      const x = Math.round(X(v.x)) + 0.5;
      if (x < P.l || x > P.l + pw) continue;
      ctx.save();
      ctx.strokeStyle = v.color || cAxis;
      ctx.setLineDash(v.dash || [4, 4]);
      ctx.beginPath(); ctx.moveTo(x, P.t); ctx.lineTo(x, P.t + ph); ctx.stroke();
      ctx.restore();
      if (v.label) {
        ctx.fillStyle = v.color || cAxis; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
        ctx.fillText(v.label, x + 4, P.t + 2);
      }
    }

    // 曲线
    ctx.save();
    ctx.beginPath(); ctx.rect(P.l, P.t, pw, ph); ctx.clip();
    for (const s of this.series) {
      if (s.hidden) continue;
      ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 1.8;
      ctx.setLineDash(s.dash || []);
      ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath();
      const n = Math.min(s.x.length, s.y.length);
      const stride = Math.max(1, Math.floor(n / (pw * 3)));
      let started = false;
      for (let i = 0; i < n; i += stride) {
        const xv = s.x[i], yv = s.y[i];
        if (!Number.isFinite(xv) || !Number.isFinite(yv)) { started = false; continue; }
        const px = X(xv), py = Y(yv);
        if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }
    for (const m of this.markers) {
      ctx.fillStyle = m.color || cText;
      ctx.beginPath(); ctx.arc(X(m.x), Y(m.y), m.r || 3.5, 0, Math.PI * 2); ctx.fill();
      if (m.label) {
        ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
        ctx.fillText(m.label, X(m.x) + 6, Y(m.y) - 4);
      }
    }
    ctx.restore();

    // 边框
    ctx.strokeStyle = cGrid; ctx.lineWidth = 1;
    ctx.strokeRect(P.l + 0.5, P.t + 0.5, pw, ph);

    // 轴标题
    ctx.fillStyle = cAxis; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
    if (this.opts.xLabel) ctx.fillText(this.opts.xLabel, P.l + pw, h - 2);
    if (this.opts.yLabel) {
      ctx.save(); ctx.translate(11, P.t); ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'right'; ctx.textBaseline = 'top';
      ctx.fillText(this.opts.yLabel, 0, 0); ctx.restore();
    }

    if (this.opts.legend) this._legend(ctx, P, pw);
    if (this.hover) this._crosshair(ctx, P, pw, ph, X, Y, d);
  }

  _legend(ctx, P, pw) {
    const items = this.series.filter((s) => s.name && !s.noLegend);
    if (!items.length) return;
    ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace';
    ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
    let x = P.l + 6;
    const y = P.t + 10;
    for (const s of items) {
      const tw = ctx.measureText(s.name).width;
      ctx.globalAlpha = s.hidden ? 0.35 : 1;
      ctx.strokeStyle = s.color; ctx.lineWidth = 2.4;
      ctx.setLineDash(s.dash || []);
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + 16, y); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = CSS('--text', '#e6edf3');
      ctx.fillText(s.name, x + 21, y + 0.5);
      ctx.globalAlpha = 1;
      s._hit = { x0: x, x1: x + 21 + tw, y0: y - 7, y1: y + 7 };
      x += 21 + tw + 14;
      if (x > P.l + pw - 40) break;
    }
  }

  /** 点击图例可以隐藏/显示曲线 */
  enableLegendToggle(onChange) {
    this.cv.addEventListener('click', (e) => {
      const r = this.cv.getBoundingClientRect();
      const mx = e.clientX - r.left, my = e.clientY - r.top;
      for (const s of this.series) {
        const h = s._hit;
        if (h && mx >= h.x0 && mx <= h.x1 && my >= h.y0 && my <= h.y1) {
          s.hidden = !s.hidden; this.draw(); onChange?.(s); return;
        }
      }
    });
    this.cv.style.cursor = 'crosshair';
  }

  _crosshair(ctx, P, pw, ph, X, Y, d) {
    const { x, y } = this.hover;
    if (x < P.l || x > P.l + pw || y < P.t || y > P.t + ph) return;
    const xv = d.x0 + ((x - P.l) / pw) * (d.x1 - d.x0);

    ctx.save();
    ctx.strokeStyle = CSS('--muted', '#8b949e'); ctx.globalAlpha = 0.5;
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(x, P.t); ctx.lineTo(x, P.t + ph); ctx.stroke();
    ctx.restore();

    const rows = [];
    for (const s of this.series) {
      if (s.hidden || s.noLegend) continue;
      const n = Math.min(s.x.length, s.y.length);
      if (!n) continue;
      let lo = 0, hi = n - 1;
      while (lo < hi) { const m = (lo + hi) >> 1; if (s.x[m] < xv) lo = m + 1; else hi = m; }
      const yv = s.y[lo];
      if (!Number.isFinite(yv)) continue;
      rows.push({ name: s.name || '', color: s.color, v: yv, px: X(s.x[lo]), py: Y(yv) });
    }
    for (const r of rows) {
      ctx.fillStyle = r.color;
      ctx.beginPath(); ctx.arc(r.px, r.py, 3, 0, Math.PI * 2); ctx.fill();
    }
    if (!rows.length) return;

    ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace';
    const lines = [`${this.opts.unitX || 'x'} = ${xv.toFixed(3)}`,
      ...rows.map((r) => `${r.name}  ${r.v.toFixed(3)}`)];
    const wBox = Math.max(...lines.map((l) => ctx.measureText(l).width)) + 16;
    const hBox = lines.length * 15 + 10;
    let bx = x + 12, by = P.t + 8;
    if (bx + wBox > P.l + pw) bx = x - wBox - 12;
    ctx.fillStyle = CSS('--tooltip', 'rgba(13,17,23,0.94)');
    ctx.strokeStyle = CSS('--grid', '#30363d');
    ctx.beginPath(); ctx.roundRect(bx, by, wBox, hBox, 6); ctx.fill(); ctx.stroke();
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    lines.forEach((l, i) => {
      ctx.fillStyle = i === 0 ? CSS('--muted', '#8b949e') : rows[i - 1].color;
      ctx.fillText(l, bx + 8, by + 6 + i * 15);
    });
  }
}
