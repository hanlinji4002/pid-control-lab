# 示例日志

这三个文件是**用本项目自己的仿真引擎**（`js/sim/car.js`）生成的，格式与真机逐字一致，
方便手边没有硬件时也能把"日志回放"整条链路跑通。

| 文件 | 来源格式 | 采样 | 说明 |
|---|---|---|---|
| `arduino_line_serial.log` | `line_follower_pid.ino` 的 `printDebug()` 原样输出 | 200 ms，**无时间戳** | 导入后可在页面上改"无时间戳时按 N ms"重新解析 |
| `arduino_line_fast.csv` | 按 [`../tools/arduino_csv_logging.md`](../tools/arduino_csv_logging.md) 改造后的 CSV | 10 ms | 采样够密，能做系统辨识 |
| `robomaster_line.csv` | [`../tools/robomaster_logger.py`](../tools/robomaster_logger.py) 的输出 | 33 ms (30 Hz) | 含 `FOLLOW` / `ALIGN` 状态与 Marker ID |

重新生成：

```bash
node tools/make_sample_logs.mjs
```

## 换成真机数据

```bash
# Arduino：先按 tools/arduino_csv_logging.md 改 .ino，再抓
python3 tools/capture_serial.py -b 115200 -t 40 -o data/my_run
```

RoboMaster 侧把 `tools/robomaster_logger.py` 里的 `LineLogger` import 进
`robot_finalproject.py`，控制循环里每拍调一次 `log()`。

两种日志都直接拖进"日志回放"页即可。
