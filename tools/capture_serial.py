#!/usr/bin/env python3
"""
抓 Arduino 串口日志，存成 pid-control-lab 能直接导入的文件。

printDebug() 的原样输出本身就能导入，但它每 200 ms 才打一行，
对系统辨识来说太稀了。所以本脚本同时存两份：
  *.log  原样保存（带时间戳，方便对照现场）
  *.csv  加上主机侧时间戳，通道拆开

要更高的采样率，先按 arduino_csv_logging.md 改一下 .ino。

用法：
    python3 tools/capture_serial.py                       # 自动找串口
    python3 tools/capture_serial.py -p /dev/cu.usbmodem14101 -b 115200 -t 60
    python3 tools/capture_serial.py -o data/my_run        # 指定输出前缀

依赖： pip install pyserial
"""
import argparse
import re
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("需要 pyserial：  pip install pyserial")

NUM = r"-?\d+(?:\.\d+)?"
ARDUINO_RE = re.compile(
    rf"norm\[\s*({NUM})\s*,\s*({NUM})\s*,\s*({NUM})\s*\]\s*(\w+)\*?\s*"
    rf"err\s*=\s*({NUM})\s+L\s*=\s*({NUM})\s+R\s*=\s*({NUM})", re.I)
CSV_HINT_RE = re.compile(r"^\s*-?[\d.]+\s*(,\s*-?[\d.]+\s*)+")


def autodetect():
    """挑一个看起来像 Arduino 的口"""
    ports = list(list_ports.comports())
    if not ports:
        return None
    for p in ports:
        blob = f"{p.device} {p.description} {p.manufacturer}".lower()
        if any(k in blob for k in ("arduino", "usbmodem", "wchusb", "ch340", "usbserial", "ttyacm")):
            return p.device
    return ports[0].device


def main():
    ap = argparse.ArgumentParser(description="抓 Arduino 循线日志")
    ap.add_argument("-p", "--port", help="串口设备，缺省自动探测")
    ap.add_argument("-b", "--baud", type=int, default=9600,
                    help="波特率，要和 Serial.begin() 一致（.ino 里是 9600）")
    ap.add_argument("-t", "--seconds", type=float, default=30.0, help="采集时长")
    ap.add_argument("-o", "--out", default="data/run", help="输出文件前缀")
    args = ap.parse_args()

    port = args.port or autodetect()
    if not port:
        sys.exit("没找到串口。插上板子，或用 -p 指定。")

    prefix = Path(args.out)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    raw_path = prefix.with_name(f"{prefix.name}_{stamp}.log")
    csv_path = prefix.with_name(f"{prefix.name}_{stamp}.csv")

    print(f"[串口] {port} @ {args.baud}  采集 {args.seconds:.0f}s")
    print(f"[输出] {raw_path}\n       {csv_path}")

    n_parsed = 0
    with serial.Serial(port, args.baud, timeout=1) as ser, \
         open(raw_path, "w", encoding="utf-8") as raw, \
         open(csv_path, "w", encoding="utf-8") as csv:
        csv.write("t,nL,nM,nR,error,pwmL,pwmR,state\n")
        t0 = time.monotonic()
        passthrough = False
        while time.monotonic() - t0 < args.seconds:
            try:
                line = ser.readline().decode("utf-8", "replace").strip()
            except serial.SerialException as e:
                print(f"\n[串口断开] {e}")
                break
            if not line:
                continue
            t = time.monotonic() - t0
            raw.write(f"{line}\n")

            m = ARDUINO_RE.search(line)
            if m:
                nl, nm, nr, state, err, L, R = m.groups()
                csv.write(f"{t:.3f},{nl},{nm},{nr},{err},{L},{R},{state.upper()}\n")
                n_parsed += 1
            elif CSV_HINT_RE.match(line):
                # .ino 已经按 arduino_csv_logging.md 改成直接吐 CSV
                if not passthrough:
                    print("[提示] 检测到 CSV 输出，直接透传（时间列用板子自己的 millis）")
                    passthrough = True
                csv.write(line + "\n")
                n_parsed += 1
            print(f"\r[{t:5.1f}s] 已解析 {n_parsed} 行  {line[:64]:<64}", end="", flush=True)

    print(f"\n[完成] 解析 {n_parsed} 行 → {csv_path}")
    print("       打开 pid-control-lab → 日志回放 → 把文件拖进去")
    if n_parsed == 0:
        print("       ⚠ 一行都没解析出来：确认波特率对不对，以及 printDebug() 有没有被注释掉")


if __name__ == "__main__":
    main()
