/**
 * 生成 data/ 下的示例日志。
 *
 * 用的就是实验室自己的物理引擎（js/sim/car.js）——不在这里另写一份简化模型，
 * 否则示例日志和页面里的仿真会对不上，辨识出来的东西也不可信。
 *
 * 这些是仿真样本，格式与真机逐字一致。要真数据用同目录的：
 *   capture_serial.py      抓 Arduino 串口
 *   robomaster_logger.py   抓 RoboMaster EP 巡线
 *
 * 运行： node tools/make_sample_logs.mjs
 */
import { writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { LineSim } from '../js/sim/car.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const OUT = join(ROOT, 'data');
mkdirSync(OUT, { recursive: true });

const f = (v, n) => v.toFixed(n);

// ── 1) Arduino 2WD：S 弯赛道，用仓库里的真实增益 Kp .6 / Kd 6 ──
// 这组参数跑得住，但过弯会甩、偶尔丢线 —— 正好留给"自动整定"去改进。
{
  const sim = new LineSim({ profile: 'arduino', trackType: 'slalom', gains: { kp: 0.6, ki: 0, kd: 6 }, seed: 4002 });
  const rows = [];
  const N = Math.floor(26 / sim.Ts);
  for (let k = 0; k < N && !sim.finished; k++) {
    sim.step();
    rows.push({
      t: sim.t, n: [...sim.sensors], err: sim.trace.error.at(-1),
      L: sim.pwm.L, R: sim.pwm.R, u: sim.trace.u.at(-1), state: sim.state,
    });
  }

  // printDebug() 原样输出：每 200 ms 一行，无时间戳
  const NAME = { STRAIGHT: 'STRAIGHT', TURN_LEFT: 'LEFT    ', TURN_RIGHT: 'RIGHT   ', SEARCHING: 'SEARCH* ', WAITING: 'WAIT    ' };
  const every = Math.round(0.2 / sim.Ts);
  let out = 'Starting in 3s...\nStarting in 2s...\nStarting in 1s...\nGO!\n';
  for (let i = 0; i < rows.length; i += every) {
    const r = rows[i];
    out += `norm[${f(r.n[0], 2)},${f(r.n[1], 2)},${f(r.n[2], 2)}]  ${NAME[r.state] || r.state}  ` +
           `err=${f(r.err, 1)}  L=${Math.round(r.L)}  R=${Math.round(r.R)}\n`;
  }
  writeFileSync(join(OUT, 'arduino_line_serial.log'), out);

  // 加了 CSV 打点的高速版（见 tools/arduino_csv_logging.md），够做系统辨识
  let csv = '# 2WD PID 循线 · S 弯赛道 · Kp=0.6 Ki=0 Kd=6 · 100 Hz\n';
  csv += 't,nL,nM,nR,error,u,pwmL,pwmR,state\n';
  for (const r of rows) {
    csv += `${f(r.t, 3)},${f(r.n[0], 4)},${f(r.n[1], 4)},${f(r.n[2], 4)},` +
           `${f(r.err, 2)},${f(r.u, 2)},${Math.round(r.L)},${Math.round(r.R)},${r.state}\n`;
  }
  writeFileSync(join(OUT, 'arduino_line_fast.csv'), csv);
  const m = sim.metrics;
  console.log(`arduino_line_serial.log  ${rows.length / every | 0} 行 (200 ms)`);
  console.log(`arduino_line_fast.csv    ${rows.length} 行 · 平均偏差 ${(m.meanAbsCte * 1000).toFixed(1)} mm · 丢线 ${m.lostCount} 次`);
}

// ── 2) RoboMaster EP：30 Hz 巡线 + 途中两次 Marker 对准停车 ──
{
  const sim = new LineSim({ profile: 'robomaster', trackType: 'oval', gains: { kp: 280, ki: 0, kd: 30 }, seed: 12022 });
  let csv = '# RoboMaster EP 蓝线巡航 · KP_LINE=280 KI_LINE=0 KD_LINE=30 · LOOP_HZ=30\n';
  csv += 't,linePos,error,u,v,state,markerId\n';
  const N = Math.floor(32 / sim.Ts);
  for (let k = 0; k < N && !sim.finished; k++) {
    sim.step();
    const t = sim.t;
    const marker = (t >= 11.0 && t < 12.4) ? 12 : (t >= 22.0 && t < 23.3) ? 22 : '';
    const state = marker ? 'ALIGN' : sim.state === 'SEARCHING' ? 'SEARCHING' : 'FOLLOW';
    csv += `${f(t, 3)},${f(sim.linePos, 4)},${f(sim.trace.error.at(-1), 4)},` +
           `${f(sim.trace.u.at(-1), 2)},${f(sim.trace.v.at(-1), 3)},${state},${marker}\n`;
  }
  writeFileSync(join(OUT, 'robomaster_line.csv'), csv);
  const m = sim.metrics;
  console.log(`robomaster_line.csv      ${N} 行 (30 Hz) · 平均偏差 ${(m.meanAbsCte * 1000).toFixed(1)} mm · 丢线 ${m.lostCount} 次`);
}
