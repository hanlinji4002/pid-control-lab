# 让 .ino 吐出适合系统辨识的日志

`01_line_follower_pid.ino` 原本的 `printDebug()` 每 200 ms 打一行。
人看刚好，但做**系统辨识**太稀了 —— 电机时间常数只有 ~120 ms，200 ms 采样连
上升沿都抓不住，拟合出来的模型没有意义。

下面把打印改成按循环频率输出 CSV。控制逻辑一行不动。

## 1. 提高波特率

串口打印是阻塞的。9600 baud 下一行 40 字节要花 **≈ 42 ms** —— 这会直接
把 `loop()` 拖慢，而 Arduino 型 PID 的 Kd 与循环周期成反比，等于把你调好的
参数改掉了。先提速：

```cpp
Serial.begin(115200);   // 原来是 9600
```

抓日志时 `capture_serial.py -b 115200`。

## 2. 换掉 printDebug()

```cpp
// ── CSV 打点（替换原 printDebug）────────────────────────────────
// 表头只在开头打一次；之后每个控制周期一行。
static bool csvHeaderDone = false;

void printDebug() {
    if (!csvHeaderDone) {
        Serial.println(F("t,nL,nM,nR,error,u,pwmL,pwmR,state"));
        csvHeaderDone = true;
    }
    const char* names[] = {"STRAIGHT","TURN_LEFT","TURN_RIGHT","SEARCHING","WAITING"};

    Serial.print(millis() * 0.001f, 3); Serial.print(',');
    Serial.print(normLeft, 4);  Serial.print(',');
    Serial.print(normMid, 4);   Serial.print(',');
    Serial.print(normRight, 4); Serial.print(',');
    Serial.print(error, 3);     Serial.print(',');
    Serial.print(pTerm + iTerm + dTerm, 3); Serial.print(',');   // PID 输出 u
    Serial.print(leftPWM);      Serial.print(',');
    Serial.print(rightPWM);     Serial.print(',');
    Serial.println(names[action]);
}
```

`computePID()` 里的 `pTerm / iTerm / dTerm` 是局部变量，要么把它们提成全局，
要么在函数末尾存一个 `pidOutLast`：

```cpp
float pidOutLast = 0;          // 放到运行状态区
...
    float pidOut = pTerm + iTerm + dTerm;
    pidOutLast = pidOut;       // 加这一行
```

然后上面打印处用 `pidOutLast`。

## 3. 确认循环频率

CSV 第一列是 `millis()`。抓完看一眼相邻行的时间差：抖动应该在 ±1 ms 以内。
如果时间差忽大忽小，说明串口还在拖慢 `loop()` —— 再提高波特率，
或者改成每 2 个周期打一行（**别**打得比控制周期更稀）。

实验室会把中位数采样间隔当作 `Ts`，抖动过大时会在页面上告警。

## 4. 抓一段有信息量的日志

最小二乘要"激励充分"才拟合得出模型。一条笔直的白线上匀速跑 30 秒，
误差恒等于 0，什么也辨识不出来。想要好数据：

- 跑**有弯**的赛道（S 弯、直角都行），让误差反复变号；
- 至少 **15–20 秒连续跟线**，中途别抱起来；
- 丢线搜索段没关系，如实记 `state` 即可 —— 实验室会自动排除那一段，
  因为搜索是开环打舵，会把模型带歪；
- 想要最干净的辨识数据，可以临时把 `Ki` 设为 0，减少一个耦合因素。

## 5. 导入

```bash
python3 tools/capture_serial.py -b 115200 -t 40 -o data/my_run
```

打开 pid-control-lab → **日志回放** → 把 `data/my_run_*.csv` 拖进去 →
**辨识被控对象** → **在这台车上自动整定**。
