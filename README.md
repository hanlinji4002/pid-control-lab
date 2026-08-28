# PID Control Lab

[![Deploy to GitHub Pages](https://github.com/hanlinji4002/pid-control-lab/actions/workflows/pages.yml/badge.svg)](https://github.com/hanlinji4002/pid-control-lab/actions/workflows/pages.yml)
[![在线体验](https://img.shields.io/badge/在线体验-hanlinji4002.github.io-4da3ff)](https://hanlinji4002.github.io/pid-control-lab/)
[![License: MIT](https://img.shields.io/badge/License-MIT-3fd68c.svg)](LICENSE)

**交互式 PID 仿真实验室** —— 调 Kp/Ki/Kd，看超调量与稳态误差，仿真循线小车，
导入真机日志做系统辨识与回放，再让它自动推荐一组参数。

纯前端，无依赖，无构建步骤。

> 🔗 **在线体验**：<https://hanlinji4002.github.io/pid-control-lab/>
> 本地跑见下方[快速开始](#快速开始)。

---

## 这不是又一个 PID 演示玩具

里面每个模型都对着两个真实机器人项目写，参数不是编出来的：

| | [2WD-Arduino-project-for-B37VR](https://github.com/hanlinji4002/2WD-Arduino-project-for-B37VR) | [Summer-Semester-Project-for-Robomaster-S1](https://github.com/hanlinji4002/Summer-Semester-Project-for-Robomaster-S1) |
|---|---|---|
| 硬件 | Arduino UNO + L298N + 3× TCRT5000 | RoboMaster EP + 摄像头 |
| 误差 | `(normR − normL)·100/total`，满量程 ±100 | `blue_center/w − 0.5`，满量程 ±0.5 |
| 积分 | `integral += error`（**不乘 dt**） | `integral += error*dt` |
| 微分 | `Kd*(e − e_prev)`（**不除 dt**） | `(e − e_prev)/dt` |
| 增益 | Kp 0.6 · Kd 6 | Kp 280 · Kd 30 |
| 环路 | `loop()` 自由跑 | 30 Hz + 相机 2 帧延迟 |
| 执行器 | PWM 死区 + `MAX_SPEED=150` + 左右补偿 150%/180% | `Z_MAX=250`，按 \|误差\| 降速 |

同一个控制律，增益差 400 倍 —— 一半来自误差量纲（±100 vs ±0.5），
一半来自离散化少了一个 `dt`。侧栏的**增益换算**面板实时把两种写法互译，
这也是整个实验室的教学主线。

---

## 四个页面

### 1 · 阶跃响应

拖动 Kp/Ki/Kd，实时给出**超调量、稳态误差、上升时间、调节时间、峰值时间、
ITAE/IAE、控制量 RMS、饱和占比、振荡穿越次数**，并把控制量拆成 P / I / D 三条曲线。

被控对象可选一阶惯性（转速环）、惯性+积分（**航向环，循线真正控制的对象**）、
通用二阶。非理想项按真机加：执行器饱和、电机死区、纯延迟、测量噪声、
10 bit ADC 量化、中途负载扰动。

控制器实现细节也全部可调：离散化方式、采样周期、积分限幅、
四种抗积分饱和策略、微分先行、微分低通、设定值加权 b。

### 2 · 循线小车

俯视赛道 + 差速小车实时仿真，五种赛道（椭圆 / 8 字 / S 弯 / 直角 / 发夹弯）。

- **Arduino 侧**：三个 TCRT5000 按高斯光斑响应建模（19 mm 胶带 + 6 mm 光斑），
  读数经 ADC 量化和噪声后，原样套用 `.ino` 里的加权公式、丢线搜索、
  左右电机补偿百分比和 PWM 限幅。
  采样芯片可以在 **UNO 10-bit** 和 **ADS1115 16-bit** 之间切换 ——
  能直接看到量化噪声怎么被 D 项放大，这正是仓库里要单独做一版 ADS1115 的原因。
- **RoboMaster 侧**：按扫描行建模，云台俯角决定扫描行落点，视野半宽决定横向量程，
  外加 2 帧相机延迟和 HSV 掩码抖动。

实时指标：平均/最大横向偏差、丢线次数与时长、**圈速与最快单圈**、行驶里程、饱和占比。

### 3 · 日志回放 —— 不只是看录像

导入真机日志（Arduino 串口原样输出 / CSV / key=value / JSON，格式自动识别），
除了回放，还能：

1. **系统辨识**：自动挑出连续跟线的最长一段（丢线搜索是开环打舵，会把模型带歪），
   用 ARX 最小二乘拟合被控对象并给出 R²；
2. **换参数重仿真**：把一步预测残差存成"那天那条赛道的外部扰动"，
   换一组 Kp/Ki/Kd 重新闭环跑一遍 —— 回答"如果当时用另一组参数会怎样"；
3. **在这台车上整定**：直接在辨识出来的模型上做数值寻优。

> **闭环辨识的坑**：日志是闭环采的，`u` 基本是 `y` 的函数，无约束最小二乘会把
> 控制器的镇定效果算进对象里，拟合出一个"自己就稳"的模型 —— 于是重仿真里
> `Kp=Ki=Kd=0` 反而误差最小。这是纯粹的辨识偏差。
> 页面上的**「对象含积分环节」**默认勾选，把 A(z) 约束成 `(1−z⁻¹)(1+αz⁻¹)`，
> 强制模型带一个积分器（循线对象物理上必然如此），退化解就消失了。

### 4 · 自动整定 —— 给四个答案，不是一个数字

| 方法 | 做什么 | 什么时候信它 |
|---|---|---|
| **继电反馈** (Åström–Hägglund) | 闭环激起等幅振荡测 Ku/Tu，再查 Ziegler–Nichols 表 | 想快速拿到量级 |
| **开环反应曲线**（两点法） | 辨识 FOPDT 的 K/L/T，给 Cohen–Coon 与 SIMC | 对象无积分环节时 |
| **数值寻优** (Nelder–Mead) | 直接最小化你选的目标函数，天然含饱和/死区/延迟 | 想要终点 |
| **稳定裕度**（数值扫频） | 给出增益裕度 / 相位裕度 | 决定敢不敢烧进车里 |

调优目标可选平衡 / 快速响应 / 无超调 / 省电护电机。
**整定对象**可以选"实验台阶跃响应"或"循线小车整圈跑分"。
结果一键采用，并生成能直接贴回 `.ino` 调参区和 `robot_finalproject.py`
参数配置区的代码片段（自动按对应写法换算增益）。

---

## 一个反直觉的结论

用 Arduino 预设，分别按两个整定对象各跑一次自动整定，再回循线小车页对比：

| 参数来源 | Kp / Ki / Kd | 阶跃调节时间 | 相位裕度 | S 弯平均偏差 | 直角赛道 | 丢线次数 |
|---|---|---|---|---|---|---|
| 仓库里的真机值 | 0.6 / 0 / 6 | 370 ms | 56° | 6.6 mm | 9.5 mm | 8 / 14 |
| 台架整定（阶跃） | 0.29 / 0 / 2.3 | **220 ms** | **65°** | 🔴 19.9 mm | 🔴 28.4 mm | 11 / 15 |
| 赛道整定（整圈） | 1.06 / 0.03 / 6.5 | — | — | 🟢 **1.7 mm** | 🟢 **3.0 mm** | **0 / 2** |

台架整定那组：阶跃更快、超调更小、增益裕度和相位裕度都更好看 —— **放到赛道上却烂了三倍**。

因为它们优化的根本不是同一件事。阶跃响应考的是"跟上设定值的突变"；
而循线小车一整圈都在抑制赛道曲率这个**持续扰动**。抑制扰动要低频环路增益，
所以赛道整定把 Kp 往上推（0.6 → 1.06），台架整定为了压控制量抖动把 Kp 往下拉。

经典整定表和裕度指标都建立在设定值跟踪上。真机调参该信哪个，
得看机器人整天在干什么 —— 这个实验室存在的意义就是让这件事**能被看见**。

---

## 快速开始

需要一个静态服务器（ES 模块不能从 `file://` 加载）：

```bash
git clone https://github.com/hanlinji4002/pid-control-lab.git
cd pid-control-lab
python3 -m http.server 8000
```

打开 <http://localhost:8000>。没有依赖，没有构建。

> **改代码时注意**：浏览器会缓存 ES 模块，`Cmd/Ctrl+R` 未必能刷新到最新的 `js/*.js`。
> 开发时用硬刷新（`Cmd/Shift+R`），或者起一个带 `Cache-Control: no-store` 的服务器。

跑数值自检（需要 Node 18+）：

```bash
node tests/run.mjs
```

重新生成示例日志：

```bash
node tools/make_sample_logs.mjs
```

---

## 导入你自己的机器人数据

`data/` 下的示例日志是**用本项目的仿真引擎生成**的，格式与真机逐字一致。
换成真车数据：

**Arduino** —— 先按 [`tools/arduino_csv_logging.md`](tools/arduino_csv_logging.md)
把 `printDebug()` 改成按循环频率吐 CSV（原来 200 ms 一行，对辨识来说太稀），
然后：

```bash
python3 tools/capture_serial.py -b 115200 -t 40 -o data/my_run
```

**RoboMaster** —— 把 [`tools/robomaster_logger.py`](tools/robomaster_logger.py)
里的 `LineLogger` import 进 `robot_finalproject.py`，控制循环里每拍调一次 `log()`：

```python
from tools.robomaster_logger import LineLogger

logger = LineLogger("data/rm_run.csv", kp=KP_LINE, ki=KI_LINE, kd=KD_LINE)
...
    z_speed = line_pid.calculate(line_error, dt)
    ep_chassis.drive_speed(x=v, y=0, z=z_speed)
    logger.log(line_error=line_error, z=z_speed, v=v,
               state="FOLLOW" if seen else "SEARCHING")
```

两种日志都直接拖进"日志回放"页。

---

## 项目结构

```
index.html              单页应用
css/style.css           深色主题
js/
  main.js               状态、工况预设、侧栏联动、代码生成
  i18n.js               中英双语
  chart.js              Canvas 折线图（多曲线 / 误差带 / 十字光标）
  core/
    pid.js              两种离散化 + 四种抗积分饱和 + 微分先行 + 增益互换
    plant.js            一阶 / 惯性+积分 / 二阶，RK4 + 死区 / 饱和 / 延迟 / 量化 / 噪声
    metrics.js          超调、稳态误差、上升/调节/峰值时间、ITAE、发散判据
    ident.js            ARX 最小二乘（可约束含积分环节）+ 离散模型重仿真
    autotune.js         继电反馈 / 反应曲线 / Z-N / Cohen-Coon / SIMC / Nelder–Mead / 稳定裕度
    logparse.js         四种日志格式自动识别
    rng.js              可复现伪随机（自动整定要求结果确定）
  sim/
    track.js            五种赛道 + 网格加速最近点查询 + 圈速计时
    car.js              差速运动学 + TCRT5000/摄像头传感器模型 + 两套真机控制逻辑
  ui/                   四个页签 + 说明页
tests/run.mjs           26 项数值自检（能对上解析解的都跟解析解比）
tools/                  真机日志采集脚本 + 示例日志生成
data/                   示例日志
```

数值内核（`js/core`、`js/sim`）不碰 DOM，可以直接在 Node 里 import ——
`tests/run.mjs` 和 `tools/make_sample_logs.mjs` 就是这么用的。

---

## 验证方式

`tests/run.mjs` 里 26 项检查，能对上解析解的都跟解析解比：

- 二阶超调量 vs `exp(−πζ/√(1−ζ²))`，ζ = 0.2/0.4/0.6 三点均在 1% 内
- 一阶阶跃 `y(τ) = K·u·(1−1/e)`
- 稳定裕度 vs `L(s)=5/(0.2s+1)` 的解析穿越频率与相位裕度
- ARX 还原已知离散系统的四个系数
- 继电反馈测出的 Ku 确实是稳定边界（0.6Ku 收敛，1.8Ku 发散）
- 两种离散化经增益换算后逐拍输出偏差 < 1e-15
- 圈速计时与解析圈长一致，同种子仿真逐位可复现

CI 在部署到 GitHub Pages 前会先跑这套自检。

---

## 部署到 GitHub Pages

`.github/workflows/pages.yml` 已配好：推到 `main` 会先跑自检，通过后自动部署。
在仓库 **Settings → Pages → Source** 里选 **GitHub Actions** 即可。

站点是纯静态的，也可以直接扔到任何静态托管上。

---

## 哪里不能当真

- 轮子不打滑、地面平整、电池电压恒定 —— 真车这三条都不成立，实测超调通常比仿真大。
- 传感器用高斯响应近似，没建 TCRT5000 的非线性和环境光干扰。
- 8 字赛道自交，圈速按穿越起跑线判定，中心交叉点附近传感器可能跟错支路。
- 日志辨识出的是**单入单出航向模型**，它看不见"执行器打满 → 前进速度垮掉"。
  日志页整定完，记得回循线小车页跑一圈验证。
- 示例日志是仿真生成的（格式与真机一致）。要真数据，用 `tools/` 里的采集脚本。

---

## License

MIT
