#!/usr/bin/env python3
"""
给 RoboMaster EP 巡线程序加日志 —— 一个可以直接 import 的小类。

设计成不动原有控制逻辑：在循环里多调一次 log()，就能得到
pid-control-lab 可以直接导入并做系统辨识的 CSV。

在 robot_finalproject.py 里这样用：

    from tools.robomaster_logger import LineLogger

    logger = LineLogger("data/rm_run.csv", kp=KP_LINE, ki=KI_LINE, kd=KD_LINE)
    ...
    while True:
        seen, line_error, dbg = detect_blue_line(frame)
        z_speed = line_pid.calculate(line_error, dt)
        v = clamp(V_BASE * (1 - 1.6 * abs(line_error)), V_MIN, V_MAX)
        ep_chassis.drive_speed(x=v, y=0, z=z_speed)

        logger.log(line_error=line_error, z=z_speed, v=v,
                   line_pos=0.5 + line_error,
                   state="FOLLOW" if seen else "SEARCHING",
                   marker=current_marker_id)
    ...
    logger.close()

辨识需要连续、等间隔的采样。别在丢线搜索或 Marker 对准期间关掉记录 ——
把 state 如实写进去，实验室会自己挑出连续跟线的那一段。
"""
import time
from pathlib import Path


class LineLogger:
    COLUMNS = ["t", "linePos", "error", "u", "v", "state", "markerId"]

    def __init__(self, path, kp=None, ki=None, kd=None, loop_hz=None, note=""):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, "w", encoding="utf-8", buffering=1)
        head = f"# RoboMaster EP 巡线日志  {time.strftime('%Y-%m-%d %H:%M:%S')}"
        if kp is not None:
            head += f"  KP_LINE={kp} KI_LINE={ki} KD_LINE={kd}"
        if loop_hz:
            head += f" LOOP_HZ={loop_hz}"
        if note:
            head += f"  {note}"
        self.f.write(head + "\n")
        self.f.write(",".join(self.COLUMNS) + "\n")
        self.t0 = time.monotonic()
        self.n = 0

    def log(self, line_error, z, v=None, line_pos=None, state="FOLLOW", marker=""):
        """在控制循环里每拍调一次。line_error 就是喂给 PID 的那个误差。"""
        t = time.monotonic() - self.t0
        if line_pos is None:
            line_pos = 0.5 + line_error
        row = [f"{t:.4f}", f"{line_pos:.5f}", f"{line_error:.5f}",
               f"{z:.3f}", "" if v is None else f"{v:.4f}",
               str(state).upper(), "" if marker in (None, "") else str(marker)]
        self.f.write(",".join(row) + "\n")
        self.n += 1

    def close(self):
        if not self.f.closed:
            self.f.close()
            dur = time.monotonic() - self.t0
            print(f"[日志] {self.n} 行 / {dur:.1f}s "
                  f"({self.n / max(dur, 1e-9):.1f} Hz) → {self.path}")
            print("[日志] 打开 pid-control-lab → 日志回放 → 拖进去 → 辨识被控对象")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


if __name__ == "__main__":
    # 冒烟测试：不接机器人也能跑，确认格式对得上
    import math
    with LineLogger("data/_logger_smoke_test.csv", kp=280, ki=0, kd=30, loop_hz=30) as lg:
        for k in range(90):
            e = 0.12 * math.sin(k * 0.2)
            lg.log(line_error=e, z=280 * e, v=0.25, state="FOLLOW")
            time.sleep(1 / 90)
