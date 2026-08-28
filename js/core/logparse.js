/**
 * 真机日志解析
 *
 * 直接吃三种格式，不需要你先手工整理：
 *  A. Arduino 串口原样输出 —— 就是 line_follower_pid.ino 里 printDebug() 打出来的那行
 *       norm[0.12,0.98,0.05]  STRAIGHT  err=-1.2  L=95  R=65
 *  B. CSV（带表头，列名自动匹配）        t,error,u,y,...
 *  C. RoboMaster 巡线日志 / 通用 key=value
 *       [12.480] line=0.531 err=0.031 z=8.7 v=0.25 state=FOLLOW
 *
 * A 格式没有时间戳（printDebug 每 200 ms 打一次），解析后可在界面上改采样周期。
 */

const NUM = String.raw`-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?`;

const ARDUINO_RE = new RegExp(
  String.raw`norm\[\s*(${NUM})\s*,\s*(${NUM})\s*,\s*(${NUM})\s*\]\s*` +
  String.raw`(\w+)?\*?\s*` +
  String.raw`err\s*=\s*(${NUM})\s+L\s*=\s*(${NUM})\s+R\s*=\s*(${NUM})`, 'i');

const KV_RE = new RegExp(String.raw`([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(${NUM})`, 'g');
const TS_RE = new RegExp(String.raw`^\s*[\[(]?\s*(${NUM})\s*[\])]?\s*[,\s]`);

/** 列名 → 标准通道名 */
const ALIAS = {
  t: 't', time: 't', timestamp: 't', ms: 't_ms', millis: 't_ms', sec: 't', seconds: 't',
  err: 'error', error: 'error', e: 'error', line_error: 'error', cte: 'error',
  u: 'u', out: 'u', output: 'u', pid: 'u', pidout: 'u', z: 'u', z_speed: 'u', omega: 'u',
  y: 'y', meas: 'y', measured: 'y', pv: 'y', pos: 'y', heading: 'y', yaw: 'y',
  r: 'r', sp: 'r', setpoint: 'r', target: 'r', ref: 'r',
  l: 'pwmL', left: 'pwmL', pwml: 'pwmL', lpwm: 'pwmL',
  rpwm: 'pwmR', right: 'pwmR', pwmr: 'pwmR',
  nl: 'nL', norm_left: 'nL', left_norm: 'nL',
  nm: 'nM', norm_mid: 'nM', mid_norm: 'nM',
  nr: 'nR', norm_right: 'nR', right_norm: 'nR',
  v: 'v', speed: 'v', vx: 'v',
  line: 'linePos', line_center: 'linePos', center: 'linePos', linepos: 'linePos',
  markerid: 'markerId', marker: 'markerId', marker_id: 'markerId',
  lap: 'lap', laptime: 'lapTime', cte: 'error', dist: 'dist',
  kp: 'kp', ki: 'ki', kd: 'kd',
  state: 'state', action: 'state',
};

function normKey(k) {
  const s = String(k).trim().toLowerCase().replace(/[^a-z0-9_]/g, '');
  return ALIAS[s] || (s ? s : null);
}

function emptyLog() {
  return { t: [], ch: {}, states: [], format: null, warnings: [], meta: {} };
}
function push(log, key, val) {
  if (key === null) return;
  (log.ch[key] ||= []).push(Number.isFinite(val) ? val : NaN);
}

/**
 * @param {string} text 日志全文
 * @param {{defaultTs:number}} opts 无时间戳时使用的采样周期（秒）
 */
export function parseLog(text, opts = {}) {
  const defaultTs = opts.defaultTs ?? 0.2;
  const raw = String(text).replace(/\r/g, '');
  const trimmed = raw.trim();

  if (trimmed.startsWith('[') || trimmed.startsWith('{')) {
    try { return fromJSON(JSON.parse(trimmed), defaultTs); } catch (_) { /* 落到文本解析 */ }
  }

  const lines = raw.split('\n');
  const dataLines = lines.filter((l) => l.trim() && !l.trim().startsWith('#'));
  if (!dataLines.length) throw new Error('日志为空');

  if (ARDUINO_RE.test(dataLines.find((l) => ARDUINO_RE.test(l)) || '')) {
    return fromArduino(dataLines, defaultTs);
  }
  if (looksLikeCSV(dataLines)) return fromCSV(dataLines, defaultTs);
  return fromKeyValue(dataLines, defaultTs);
}

function looksLikeCSV(lines) {
  const head = lines[0];
  if (!/[,;\t]/.test(head)) return false;
  const n = head.split(/[,;\t]/).length;
  const second = lines[1] || '';
  return n >= 2 && second.split(/[,;\t]/).length === n;
}

// ── A. Arduino printDebug ──
function fromArduino(lines, defaultTs) {
  const log = emptyLog();
  log.format = 'arduino-serial';
  let k = 0;
  for (const line of lines) {
    const m = ARDUINO_RE.exec(line);
    if (!m) continue;
    const [, nL, nM, nR, state, err, L, R] = m;
    const ts = TS_RE.exec(line);
    log.t.push(ts ? parseFloat(ts[1]) : k * defaultTs);
    push(log, 'nL', +nL); push(log, 'nM', +nM); push(log, 'nR', +nR);
    push(log, 'error', +err);
    push(log, 'pwmL', +L); push(log, 'pwmR', +R);
    push(log, 'u', (+L - +R) / 2);           // .ino 里 L=base+pid, R=base−pid
    log.states.push((state || '').trim().toUpperCase());
    k++;
  }
  if (!log.t.length) throw new Error('未匹配到 Arduino 串口格式');
  if (!TS_RE.test(lines.find((l) => ARDUINO_RE.test(l)))) {
    log.warnings.push(`该格式不含时间戳，已按 ${(defaultTs * 1000).toFixed(0)} ms/行 生成时间轴（printDebug 默认 200 ms）`);
    log.meta.syntheticTime = true;
  }
  log.meta.source = 'line_follower_pid.ino · printDebug()';
  return finish(log);
}

// ── B. CSV ──
function fromCSV(lines, defaultTs) {
  const log = emptyLog();
  log.format = 'csv';
  const sep = /\t/.test(lines[0]) ? '\t' : (/;/.test(lines[0]) ? ';' : ',');
  const header = lines[0].split(sep).map(normKey);
  const numericHeader = header.every((h) => h && /^-?[\d.]+$/.test(h));
  const cols = numericHeader ? header.map((_, i) => (i === 0 ? 't' : `col${i}`)) : header;
  const body = numericHeader ? lines : lines.slice(1);

  let k = 0;
  for (const line of body) {
    const parts = line.split(sep);
    if (parts.length < 2) continue;
    let time = null;
    cols.forEach((key, i) => {
      const v = parseFloat(parts[i]);
      if (key === 't') { time = v; return; }
      if (key === 't_ms') { time = v / 1000; return; }
      if (key === 'state') { return; }
      push(log, key, v);
    });
    const si = cols.indexOf('state');
    log.states.push(si >= 0 ? (parts[si] || '').trim().toUpperCase() : '');
    log.t.push(Number.isFinite(time) ? time : k * defaultTs);
    k++;
  }
  if (!cols.includes('t') && !cols.includes('t_ms')) {
    log.warnings.push(`CSV 无时间列，已按 ${(defaultTs * 1000).toFixed(0)} ms/行 生成时间轴`);
    log.meta.syntheticTime = true;
  }
  return finish(log);
}

// ── C. key=value ──
function fromKeyValue(lines, defaultTs) {
  const log = emptyLog();
  log.format = 'key-value';
  let k = 0, hasTime = false;
  for (const line of lines) {
    KV_RE.lastIndex = 0;
    const pairs = [...line.matchAll(KV_RE)];
    if (!pairs.length) continue;
    const ts = TS_RE.exec(line);
    if (ts) hasTime = true;
    log.t.push(ts ? parseFloat(ts[1]) : k * defaultTs);
    for (const [, key, val] of pairs) push(log, normKey(key), parseFloat(val));
    const st = /state\s*=\s*(\w+)/i.exec(line) || /\b(FOLLOW|SEARCH\w*|LOST|STRAIGHT|LEFT|RIGHT|WAIT\w*|ALIGN\w*)\b/i.exec(line);
    log.states.push(st ? st[1].toUpperCase() : '');
    k++;
  }
  if (!log.t.length) throw new Error('无法识别日志格式：既不是 Arduino 串口、CSV，也不是 key=value');
  if (!hasTime) {
    log.warnings.push(`未找到时间戳，已按 ${(defaultTs * 1000).toFixed(0)} ms/行 生成时间轴`);
    log.meta.syntheticTime = true;
  }
  return finish(log);
}

function fromJSON(obj, defaultTs) {
  const log = emptyLog();
  log.format = 'json';
  const arr = Array.isArray(obj) ? obj : (obj.samples || obj.data || []);
  arr.forEach((row, k) => {
    let time = null;
    for (const [rawK, v] of Object.entries(row)) {
      const key = normKey(rawK);
      if (key === 't') { time = +v; continue; }
      if (key === 't_ms') { time = +v / 1000; continue; }
      if (key === 'state') continue;
      if (typeof v === 'number') push(log, key, v);
    }
    log.t.push(Number.isFinite(time) ? time : k * defaultTs);
    log.states.push(String(row.state || row.action || '').toUpperCase());
  });
  if (!log.t.length) throw new Error('JSON 中没有可用样本');
  return finish(log);
}

/** 补齐长度、统一起点、算平均采样周期、补 error 通道 */
function finish(log) {
  const N = log.t.length;
  for (const key of Object.keys(log.ch)) {
    const a = log.ch[key];
    while (a.length < N) a.push(NaN);
    if (a.length > N) a.length = N;
    if (a.every((v) => !Number.isFinite(v))) delete log.ch[key];
  }
  while (log.states.length < N) log.states.push('');

  const t0 = log.t[0];
  log.t = log.t.map((v) => v - t0);

  // Arduino 三路归一化 → 按 .ino 的公式补出误差
  if (!log.ch.error && log.ch.nL && log.ch.nM && log.ch.nR) {
    log.ch.error = log.ch.nL.map((l, i) => {
      const tot = l + log.ch.nM[i] + log.ch.nR[i];
      return tot > 0.15 ? (log.ch.nR[i] - l) * 100 / tot : NaN;
    });
    log.warnings.push('误差列缺失，已按 .ino 中 (normR − normL)·100 / total 现场重算');
  }
  if (!log.ch.u && log.ch.pwmL && log.ch.pwmR) {
    log.ch.u = log.ch.pwmL.map((l, i) => (l - log.ch.pwmR[i]) / 2);
  }

  const dts = [];
  for (let i = 1; i < N; i++) { const d = log.t[i] - log.t[i - 1]; if (d > 0) dts.push(d); }
  dts.sort((a, b) => a - b);
  log.meta.Ts = dts.length ? dts[Math.floor(dts.length / 2)] : 0.2;
  log.meta.duration = log.t[N - 1];
  log.meta.samples = N;
  log.meta.channels = Object.keys(log.ch);
  const jitter = dts.length ? (dts[dts.length - 1] - dts[0]) / (log.meta.Ts || 1) : 0;
  if (jitter > 1.0 && !log.meta.syntheticTime) {
    log.warnings.push(`采样间隔抖动较大（${(jitter * 100).toFixed(0)}%），辨识与重仿真结果仅供参考`);
  }
  return log;
}
