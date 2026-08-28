/** 说明页：讲清楚仿的是什么、公式从哪来、哪里不能当真 */
import { lang } from '../i18n.js';
import { bus } from '../main.js';

const ZH = `
<p>这不是一个通用 PID 玩具。里面每个模型都对着两个真实机器人项目写：
<a href="https://github.com/hanlinji4002/2WD-Arduino-project-for-B37VR" target="_blank" rel="noopener">2WD-Arduino-project-for-B37VR</a> 和
<a href="https://github.com/hanlinji4002/Summer-Semester-Project-for-Robomaster-S1" target="_blank" rel="noopener">Summer-Semester-Project-for-Robomaster-S1</a>。</p>

<h4>同一个 PID，为什么增益差 400 倍</h4>
<table>
<tr><th></th><th>Arduino 2WD</th><th>RoboMaster EP</th></tr>
<tr><td>误差</td><td><code>(normR − normL)·100/total</code>，满量程 ±100</td><td><code>blue_center/w − 0.5</code>，满量程 ±0.5</td></tr>
<tr><td>积分</td><td><code>integral += error</code>（不乘 dt）</td><td><code>integral += error*dt</code></td></tr>
<tr><td>微分</td><td><code>Kd*(e − e_prev)</code>（不除 dt）</td><td><code>(e − e_prev)/dt</code></td></tr>
<tr><td>增益</td><td>Kp 0.6 · Kd 6</td><td>Kp 280 · Kd 30</td></tr>
</table>
<p>误差量纲差 200 倍，离散化又差一个 T<sub>s</sub>。侧栏的<b>增益换算</b>随时把两种写法互相翻译。
Arduino 写法的坑在于：Ki 正比于 T<sub>s</sub>、Kd 反比于 T<sub>s</sub> —— 多打两行
<code>Serial.print</code> 让 <code>loop()</code> 变慢，调好的参数就作废了。</p>

<h4>被控对象</h4>
<p>循线真正控制的是<b>航向</b>，不是位置：PWM 差 → 角速度（一阶惯性，电机 + 减速箱）→ 航向角（积分）。
所以它天生是 I 型系统，P 控制就能消掉稳态误差，但相位滞后 90°，不加 D 必然振荡。
在"阶跃响应"页把 K<sub>d</sub> 拉到 0 就能看到这一点。</p>
<p>非理想项都按真机加了：执行器饱和（<code>MAX_SPEED</code>）、电机死区、
纯延迟（RoboMaster 相机 2 帧 ≈ 66 ms）、测量噪声、10 bit ADC 量化、负载扰动。</p>

<h4>循线小车</h4>
<p>三个 TCRT5000 用高斯光斑响应建模（19 mm 电工胶带 + 6 mm 光斑），
读数经 ADC 量化和噪声后，原样套用 <code>.ino</code> 里的加权公式、丢线搜索、
左右电机补偿百分比和 PWM 限幅。把"采样芯片"从 UNO 10-bit 换成 ADS1115 16-bit，
就能看到量化噪声怎么进到 D 项里 —— 这正是仓库里要单独做一版 ADS1115 的原因。</p>
<p>RoboMaster 侧按扫描行建模：云台俯角决定扫描行落在车前多远，
视野半宽决定横向量程，再加 2 帧延迟和 HSV 掩码抖动。</p>

<h4>日志回放能做的，不只是看录像</h4>
<p>导入日志后点<b>辨识被控对象</b>：程序会挑出连续跟线的最长一段（丢线搜索段是开环打舵，会把模型带歪），
用 ARX 最小二乘拟合二阶模型，并给出 R²。然后把一步预测残差当作<b>那天那条赛道的外部扰动</b>存下来，
换一组 K<sub>p</sub>/K<sub>i</sub>/K<sub>d</sub> 重新闭环跑一遍 —— 回答的是"如果当时用另一组参数会怎样"。</p>

<h4>自动整定给的是四个答案，不是一个数字</h4>
<ul>
<li><b>继电反馈</b>激起等幅振荡测 K<sub>u</sub>/T<sub>u</sub>，再查 Ziegler–Nichols 表；</li>
<li><b>开环反应曲线</b>两点法辨识 FOPDT，给 Cohen–Coon 和 SIMC（保守，适合第一次上电）；</li>
<li><b>数值寻优</b>直接最小化你选的目标函数，天然把饱和、死区、延迟算进去；</li>
<li><b>稳定裕度</b>用数值扫频给增益裕度/相位裕度 —— 告诉你这组"最优"参数离不稳定还有多远。</li>
</ul>
<p>经典法给起点，寻优给终点，裕度决定敢不敢直接烧进车里。</p>

<h4>一个反直觉的结论，值得自己跑一遍</h4>
<p>拿 Arduino 预设，在"自动整定"页分别用两个整定对象各跑一次，然后回"循线小车"页对比：</p>
<table>
<tr><th>参数来源</th><th>Kp / Ki / Kd</th><th>S 弯平均偏差</th><th>直角赛道</th><th>丢线</th></tr>
<tr><td>仓库里的真机值</td><td>0.6 / 0 / 6</td><td>6.6 mm</td><td>9.5 mm</td><td>8 / 14</td></tr>
<tr><td>台架整定（阶跃）</td><td>0.29 / 0 / 2.3</td><td><b style="color:var(--danger)">19.9 mm</b></td><td><b style="color:var(--danger)">28.4 mm</b></td><td>11 / 15</td></tr>
<tr><td>赛道整定（整圈）</td><td>1.06 / 0.03 / 6.5</td><td><b style="color:var(--ok)">1.7 mm</b></td><td><b style="color:var(--ok)">3.0 mm</b></td><td>0 / 2</td></tr>
</table>
<p>台架整定的那组，阶跃响应更快（调节时间 370 → 220 ms）、超调更小、增益裕度和相位裕度都更好看 ——
放到赛道上却烂了三倍。原因是<b>它们优化的根本不是同一件事</b>：阶跃响应考的是"跟上设定值的突变"，
而循线小车一整圈都在抑制赛道曲率这个**持续扰动**。抑制扰动要的是低频环路增益，
所以赛道整定的结论是把 Kp 往上推（0.6 → 1.06），而台架整定为了压控制量抖动把 Kp 往下拉。</p>
<p>这就是"阶跃响应漂亮"和"车跑得稳"之间的落差。经典整定表和裕度指标都建立在设定值跟踪上，
真机调参该以哪个为准，得看你的机器人整天在干什么。</p>

<h4>哪里不能当真</h4>
<ul>
<li>轮子不打滑、地面平整、电池电压恒定 —— 真车这三条都不成立，实测超调通常比仿真大。</li>
<li>传感器响应用高斯近似，没建 TCRT5000 的非线性和环境光干扰。</li>
<li>8 字赛道自交，圈速按穿越起跑线判定，中心交叉点附近可能跟错支路。</li>
<li>示例日志是仿真生成的（格式与真机一致）。要真数据，用 <code>tools/</code> 里的抓取脚本。</li>
</ul>
`;

const EN = `
<p>This isn't a generic PID toy. Every model here is written against two real robot projects:
<a href="https://github.com/hanlinji4002/2WD-Arduino-project-for-B37VR" target="_blank" rel="noopener">2WD-Arduino-project-for-B37VR</a> and
<a href="https://github.com/hanlinji4002/Summer-Semester-Project-for-Robomaster-S1" target="_blank" rel="noopener">Summer-Semester-Project-for-Robomaster-S1</a>.</p>

<h4>Same PID, gains 400× apart</h4>
<table>
<tr><th></th><th>Arduino 2WD</th><th>RoboMaster EP</th></tr>
<tr><td>Error</td><td><code>(normR − normL)·100/total</code>, ±100 full scale</td><td><code>blue_center/w − 0.5</code>, ±0.5 full scale</td></tr>
<tr><td>Integral</td><td><code>integral += error</code> (no dt)</td><td><code>integral += error*dt</code></td></tr>
<tr><td>Derivative</td><td><code>Kd*(e − e_prev)</code> (no /dt)</td><td><code>(e − e_prev)/dt</code></td></tr>
<tr><td>Gains</td><td>Kp 0.6 · Kd 6</td><td>Kp 280 · Kd 30</td></tr>
</table>
<p>The error scales differ by 200×, and the discretisation differs by one T<sub>s</sub>. The sidebar
converts between the two forms live. The trap in the Arduino form: Ki scales with T<sub>s</sub> and Kd
with 1/T<sub>s</sub> — add two <code>Serial.print</code> calls, slow down <code>loop()</code>, and your tuning is gone.</p>

<h4>The plant</h4>
<p>Line following controls <b>heading</b>, not position: PWM difference → angular rate (first-order
motor lag) → heading (integrator). That makes it a type-1 system: P control alone kills steady-state
error but adds 90° of phase lag, so it will ring without D. Drag K<sub>d</sub> to zero on the step-response
tab and watch it happen.</p>
<p>The non-idealities are modelled from the real hardware: actuator saturation (<code>MAX_SPEED</code>),
motor deadband, dead time (RoboMaster camera, 2 frames ≈ 66 ms), measurement noise, 10-bit ADC
quantisation and load disturbance.</p>

<h4>Line follower</h4>
<p>Three TCRT5000 sensors are modelled with a Gaussian spot response (19 mm tape, 6 mm spot). Readings
go through ADC quantisation and noise, then straight into the weighting formula, line-lost search,
per-side motor compensation and PWM clamp taken verbatim from the <code>.ino</code>. Switch the ADC from
UNO 10-bit to ADS1115 16-bit and watch quantisation noise leak into the D term — which is exactly why
the repository has a separate ADS1115 build.</p>

<h4>Log replay does more than play back a recording</h4>
<p>After importing a log, hit <b>Identify plant</b>: the longest continuously-tracking segment is picked
(line-lost search is open-loop and would corrupt the fit), a second-order ARX model is fitted by least
squares, and R² is reported. The one-step residual is then kept as <b>that day's track disturbance</b>, so
you can re-run the closed loop with different gains and answer "what would other gains have done on that
same lap?"</p>

<h4>Auto-tune returns four answers, not one number</h4>
<ul>
<li><b>Relay feedback</b> excites a limit cycle to measure K<sub>u</sub>/T<sub>u</sub>, then applies the Ziegler–Nichols table;</li>
<li><b>Open-loop reaction curve</b> (two-point method) fits a FOPDT model for Cohen–Coon and SIMC;</li>
<li><b>Numerical optimisation</b> minimises your chosen objective, saturation and dead time included;</li>
<li><b>Stability margins</b> from a numerical frequency sweep tell you how far the "optimal" gains sit from instability.</li>
</ul>

<h4>A counter-intuitive result worth reproducing</h4>
<p>With the Arduino preset, run auto-tune once against the bench plant and once against the full lap, then compare on the Line-follower tab:</p>
<table>
<tr><th>Gains from</th><th>Kp / Ki / Kd</th><th>Slalom mean |CTE|</th><th>Right angles</th><th>Line lost</th></tr>
<tr><td>The repo's real robot</td><td>0.6 / 0 / 6</td><td>6.6 mm</td><td>9.5 mm</td><td>8 / 14</td></tr>
<tr><td>Bench (step response)</td><td>0.29 / 0 / 2.3</td><td><b style="color:var(--danger)">19.9 mm</b></td><td><b style="color:var(--danger)">28.4 mm</b></td><td>11 / 15</td></tr>
<tr><td>Track (full lap)</td><td>1.06 / 0.03 / 6.5</td><td><b style="color:var(--ok)">1.7 mm</b></td><td><b style="color:var(--ok)">3.0 mm</b></td><td>0 / 2</td></tr>
</table>
<p>The bench-tuned gains settle faster (370 → 220 ms), overshoot less, and have better gain and phase
margins — and they are three times worse on the track. They optimise a different thing: step response
measures how well you follow a <i>setpoint change</i>, while a line follower spends every lap rejecting
track curvature, a <i>persistent disturbance</i>. Disturbance rejection wants low-frequency loop gain, so
the track tuner pushes Kp up (0.6 → 1.06) while the bench tuner pulls it down to quiet the actuator.</p>
<p>Classical tuning rules and stability margins are all defined on setpoint tracking. Which one you should
believe depends on what your robot actually does all day.</p>

<h4>Where not to trust it</h4>
<ul>
<li>No wheel slip, flat ground, constant battery voltage. Real overshoot is usually worse.</li>
<li>Gaussian sensor response; TCRT5000 nonlinearity and ambient light are not modelled.</li>
<li>The figure-8 track self-intersects; lap timing uses start-line crossing but the sensor may follow the wrong branch at the centre.</li>
<li>The sample logs are simulator-generated in the real format. For real data use the capture scripts in <code>tools/</code>.</li>
</ul>
`;

export function init() {
  const paint = () => {
    const back = lang === 'en'
      ? '<p class="hint">Pick any tab above to go back.</p>'
      : '<p class="hint">点上面任意一个页签即可返回。</p>';
    document.querySelector('#help-body').innerHTML = (lang === 'en' ? EN : ZH) + back;
  };
  paint();
  bus.on('lang', paint);
}
