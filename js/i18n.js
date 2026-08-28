/** 中英双语。data-i18n 标记静态文案，t() 供 JS 动态取用。 */

const ZH = {
  'app.sub': '交互式 PID 仿真实验室 · 循线小车 / 真机日志 / 自动整定',
  'preset.label': '工况预设', 'nav.help': '说明',
  'gains.title': 'PID 增益', 'gains.reset': '恢复预设', 'gains.zeroID': '只留 P',
  'gains.kpHint': '比例：现在差多少 → 立刻纠多少。太大→震荡，太小→跟不上',
  'gains.kiHint': '积分：一直差一点 → 慢慢补上。消稳态误差，代价是超调与积分饱和',
  'gains.kdHint': '微分：正在快速逼近 → 提前松劲。抑制超调，但会放大传感器噪声',
  'ctrl.title': '控制器实现', 'ctrl.mode': '离散化方式',
  'ctrl.modeStd': '标准（∫e·dt，de/dt）', 'ctrl.modeIno': 'Arduino（Σe，Δe）',
  'ctrl.ts': '采样周期 Ts', 'ctrl.imax': '积分限幅 I_MAX', 'ctrl.aw': '抗积分饱和',
  'ctrl.awClamp': '累加器限幅（.ino 的做法）', 'ctrl.awCond': '条件积分',
  'ctrl.awBack': '反算 back-calculation', 'ctrl.awNone': '不处理（看饱和现象）',
  'ctrl.usat': '输出限幅 ±umax', 'ctrl.dmeas': '微分先行（对测量值求导）',
  'ctrl.dsmooth': '微分低通', 'ctrl.spw': '设定值加权 b',
  'ctrl.noteStd': '标准型：增益与采样率无关，换 30 Hz / 100 Hz 都不用重调。',
  'ctrl.noteIno': '⚠ Arduino 型：Ki 随 Ts 变、Kd 随 1/Ts 变。多打两行 Serial.print 循环变慢，参数就得重调 —— 这是 .ino 里最容易踩的坑。',
  'conv.title': '增益换算',
  'conv.note': '同一个控制律，两种写法差一个 Ts。换板子、改循环频率之前先看这里。',
  'tab.step': '阶跃响应', 'tab.line': '循线小车', 'tab.replay': '日志回放', 'tab.tune': '自动整定',
  'plant.type': '被控对象',
  'plant.heading': '航向环（惯性+积分 · 循线实际对象）', 'plant.speed': '转速环（一阶惯性）',
  'plant.generic': '通用二阶（ζ / ωn）', 'plant.delay': '纯延迟 (s)', 'plant.deadband': '执行死区',
  'plant.noise': '测量噪声 σ', 'plant.tend': '仿真时长 (s)', 'plant.dist': '中途加负载扰动',
  'plant.hold': '叠加对比曲线', 'plant.clearRef': '清除对比',
  'chart.resp': '响应曲线', 'chart.u': '控制量与 P / I / D 分解', 'chart.err': '误差',
  'chart.hint': '点图例可隐藏曲线 · 鼠标悬停读数',
  'line.robot': '机器人', 'line.track': '赛道', 'line.adc': '采样芯片', 'line.speed': '倍速',
  'line.run': '▶ 开始', 'line.pause': '⏸ 暂停', 'line.reset': '↺ 重置',
  'line.showSensors': '显示传感器', 'line.showPath': '显示轨迹',
  'line.view': '赛道', 'line.sensors': '传感器', 'line.cte': '横向偏差 (mm)',
  'line.ctrl': '误差与控制量', 'line.hw': '车体与传感器参数（对着真机改）',
  'replay.drop': '把真机日志拖进来，或点击选择文件',
  'replay.formats': '支持 Arduino 串口原样输出 · CSV · key=value · JSON',
  'replay.play': '▶ 播放', 'replay.pause': '⏸ 暂停', 'replay.ts': '无时间戳时按',
  'replay.reparse': '重新解析', 'replay.rec': '记录通道',
  'replay.ident': '系统辨识 → 换参数重仿真',
  'replay.identHint': '从日志里拟合被控对象，再问"如果当时用另一组 PID 会怎样"',
  'replay.doIdent': '辨识被控对象', 'replay.identNone': '未辨识',
  'replay.integrator': '对象含积分环节',
  'replay.resim': '用当前 PID 重仿真', 'replay.resimTune': '在这台车上自动整定',
  'tune.obj': '调优目标', 'tune.on': '整定对象', 'tune.onPlant': '实验台被控对象',
  'tune.onLine': '循线小车（整圈跑分）', 'tune.run': '开始整定',
  'tune.cands': '候选参数对比',
  'tune.candsHint': '经典整定法给起点，数值寻优给终点，稳定裕度告诉你敢不敢用',
  'tune.cmp': '响应对比', 'tune.code': '贴回代码', 'tune.copy': '复制',
  'help.title': '这个实验室在仿什么',
  // 指标
  'm.overshoot': '超调量', 'm.ess': '稳态误差', 'm.rise': '上升时间', 'm.settle': '调节时间',
  'm.peak': '峰值时间', 'm.itae': 'ITAE', 'm.iae': 'IAE', 'm.urms': '控制量 RMS',
  'm.sat': '饱和占比', 'm.cross': '穿越次数', 'm.gm': '增益裕度', 'm.pm': '相位裕度',
  'm.meanCte': '平均偏差', 'm.maxCte': '最大偏差', 'm.rmsCte': '偏差 RMS',
  'm.lost': '丢线次数', 'm.lostTime': '丢线时长', 'm.laps': '完成圈数',
  'm.best': '最快单圈', 'm.last': '上一圈', 'm.dist': '行驶里程', 'm.simtime': '仿真时长',
  'm.samples': '样本数', 'm.duration': '时长', 'm.ts': '采样周期', 'm.channels': '通道',
  'm.format': '格式', 'm.progress': '本圈进度',
  // 提示
  'msg.copied': '已复制到剪贴板', 'msg.tuning': '整定中…',
  'msg.parseFail': '解析失败：', 'msg.loaded': '已载入 ',
  'msg.identFail': '辨识失败：数据太短或激励不足（试试有明显转弯的日志段）',
  'msg.needU': '日志里缺少控制量通道（u / z / pwmL+pwmR），无法辨识',
  'msg.offtrack': '冲出赛道', 'msg.unstable': '发散',
};

const EN = {
  'app.sub': 'Interactive PID lab · line-follower / real logs / auto-tuning',
  'preset.label': 'Preset', 'nav.help': 'About',
  'gains.title': 'PID gains', 'gains.reset': 'Reset preset', 'gains.zeroID': 'P only',
  'gains.kpHint': 'Proportional: react to the error right now. Too high → oscillation.',
  'gains.kiHint': 'Integral: erase persistent offset. Costs overshoot and windup.',
  'gains.kdHint': 'Derivative: ease off as you close in. Damps overshoot, amplifies noise.',
  'ctrl.title': 'Controller', 'ctrl.mode': 'Discretisation',
  'ctrl.modeStd': 'Standard (∫e·dt, de/dt)', 'ctrl.modeIno': 'Arduino (Σe, Δe)',
  'ctrl.ts': 'Sample period Ts', 'ctrl.imax': 'Integral clamp I_MAX', 'ctrl.aw': 'Anti-windup',
  'ctrl.awClamp': 'Clamp accumulator (.ino style)', 'ctrl.awCond': 'Conditional integration',
  'ctrl.awBack': 'Back-calculation', 'ctrl.awNone': 'None (watch it wind up)',
  'ctrl.usat': 'Output limit ±umax', 'ctrl.dmeas': 'Derivative on measurement',
  'ctrl.dsmooth': 'Derivative low-pass', 'ctrl.spw': 'Setpoint weight b',
  'ctrl.noteStd': 'Standard form: gains are sample-rate independent.',
  'ctrl.noteIno': '⚠ Arduino form: Ki scales with Ts and Kd with 1/Ts. Add two Serial.print calls and your tuning is gone.',
  'conv.title': 'Gain conversion',
  'conv.note': 'Same control law, two spellings, one Ts apart. Check here before changing loop rate.',
  'tab.step': 'Step response', 'tab.line': 'Line follower', 'tab.replay': 'Log replay', 'tab.tune': 'Auto-tune',
  'plant.type': 'Plant',
  'plant.heading': 'Heading loop (lag + integrator)', 'plant.speed': 'Speed loop (first order)',
  'plant.generic': 'Generic 2nd order (ζ / ωn)', 'plant.delay': 'Dead time (s)', 'plant.deadband': 'Actuator deadband',
  'plant.noise': 'Measurement noise σ', 'plant.tend': 'Sim length (s)', 'plant.dist': 'Load disturbance',
  'plant.hold': 'Overlay reference', 'plant.clearRef': 'Clear overlay',
  'chart.resp': 'Response', 'chart.u': 'Control effort & P / I / D split', 'chart.err': 'Error',
  'chart.hint': 'Click legend to hide · hover to read values',
  'line.robot': 'Robot', 'line.track': 'Track', 'line.adc': 'ADC', 'line.speed': 'Speed',
  'line.run': '▶ Run', 'line.pause': '⏸ Pause', 'line.reset': '↺ Reset',
  'line.showSensors': 'Sensors', 'line.showPath': 'Path',
  'line.view': 'Track', 'line.sensors': 'Sensors', 'line.cte': 'Cross-track error (mm)',
  'line.ctrl': 'Error & control', 'line.hw': 'Chassis & sensor parameters',
  'replay.drop': 'Drop a robot log here, or click to choose a file',
  'replay.formats': 'Arduino serial · CSV · key=value · JSON',
  'replay.play': '▶ Play', 'replay.pause': '⏸ Pause', 'replay.ts': 'If no timestamps, assume',
  'replay.reparse': 'Re-parse', 'replay.rec': 'Recorded channels',
  'replay.ident': 'System ID → re-simulate with new gains',
  'replay.identHint': 'Fit the plant from the log, then ask what other gains would have done',
  'replay.doIdent': 'Identify plant', 'replay.identNone': 'not identified',
  'replay.integrator': 'Plant has an integrator',
  'replay.resim': 'Re-simulate with current PID', 'replay.resimTune': 'Auto-tune on this robot',
  'tune.obj': 'Objective', 'tune.on': 'Tune against', 'tune.onPlant': 'Bench plant',
  'tune.onLine': 'Line follower (full lap)', 'tune.run': 'Run auto-tune',
  'tune.cands': 'Candidates',
  'tune.candsHint': 'Classical rules give the starting point, optimisation the finish, margins the confidence',
  'tune.cmp': 'Response comparison', 'tune.code': 'Paste back into your code', 'tune.copy': 'Copy',
  'help.title': 'What this lab simulates',
  'm.overshoot': 'Overshoot', 'm.ess': 'Steady-state error', 'm.rise': 'Rise time', 'm.settle': 'Settling time',
  'm.peak': 'Peak time', 'm.itae': 'ITAE', 'm.iae': 'IAE', 'm.urms': 'Control RMS',
  'm.sat': 'Saturated', 'm.cross': 'Crossings', 'm.gm': 'Gain margin', 'm.pm': 'Phase margin',
  'm.meanCte': 'Mean |CTE|', 'm.maxCte': 'Max |CTE|', 'm.rmsCte': 'RMS CTE',
  'm.lost': 'Line lost', 'm.lostTime': 'Time lost', 'm.laps': 'Laps',
  'm.best': 'Best lap', 'm.last': 'Last lap', 'm.dist': 'Distance', 'm.simtime': 'Sim time',
  'm.samples': 'Samples', 'm.duration': 'Duration', 'm.ts': 'Sample period', 'm.channels': 'Channels',
  'm.format': 'Format', 'm.progress': 'Lap progress',
  'msg.copied': 'Copied', 'msg.tuning': 'Tuning…',
  'msg.parseFail': 'Parse failed: ', 'msg.loaded': 'Loaded ',
  'msg.identFail': 'Identification failed: too short or poorly excited',
  'msg.needU': 'Log has no control channel (u / z / pwmL+pwmR)',
  'msg.offtrack': 'off track', 'msg.unstable': 'diverged',
};

export const DICT = { zh: ZH, en: EN };
export let lang = (localStorage.getItem('pidlab.lang') || 'zh');

export function t(key, fallback) {
  return DICT[lang][key] ?? DICT.zh[key] ?? fallback ?? key;
}

export function setLang(l) {
  lang = l;
  localStorage.setItem('pidlab.lang', l);
  document.documentElement.lang = l === 'zh' ? 'zh-CN' : 'en';
  applyStatic();
}

export function applyStatic(root = document) {
  root.querySelectorAll('[data-i18n]').forEach((el) => {
    const k = el.getAttribute('data-i18n');
    const v = DICT[lang][k];
    if (v !== undefined) el.innerHTML = v;
  });
}
