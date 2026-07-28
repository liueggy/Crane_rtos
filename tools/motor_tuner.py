#!/usr/bin/env python3
"""STM32 四轮底盘串口监视与闭环调参工具。"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import serial
except ImportError:
    print("缺少 pyserial，请执行: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


TELEMETRY_COLUMNS = [
    "kind", "tick", "closed", "run", "dir", "target",
    "r1", "r2", "r3", "r4", "p1", "p2", "p3", "p4",
    "c1", "c2", "c3", "c4",
]


@dataclass
class Sample:
    received_s: float
    values: dict[str, int | str]

    @property
    def rpms(self) -> list[int]:
        return [int(self.values[f"r{i}"]) for i in range(1, 5)]


class MotorLink:
    def __init__(self, port: str, baud: int) -> None:
        self.serial = serial.Serial(port, baud, timeout=0.1)
        self.serial.reset_input_buffer()

    def close(self) -> None:
        self.serial.close()

    def send(self, command: str) -> None:
        self.serial.write((command.strip() + "\r\n").encode("ascii"))
        self.serial.flush()

    def read_line(self) -> str | None:
        raw = self.serial.readline()
        if not raw:
            return None
        return raw.decode("ascii", errors="replace").strip()

    def wait_response(self, prefix: tuple[str, ...], timeout_s: float = 1.5) -> str:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = self.read_line()
            if line and line.startswith(prefix):
                return line
        raise TimeoutError(f"等待响应超时: {prefix}")

    def command(self, command: str, prefix: tuple[str, ...] = ("OK", "ERR", "CFG")) -> str:
        self.send(command)
        return self.wait_response(prefix)


def parse_sample(line: str, received_s: float) -> Sample | None:
    parts = line.split(",")
    if len(parts) != len(TELEMETRY_COLUMNS) or parts[0] != "M":
        return None
    try:
        values: dict[str, int | str] = {"kind": "M"}
        values.update({name: int(value) for name, value in zip(TELEMETRY_COLUMNS[1:], parts[1:])})
    except ValueError:
        return None
    return Sample(received_s, values)


def collect(link: MotorLink, duration_s: float, echo: bool = True) -> list[Sample]:
    samples: list[Sample] = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        line = link.read_line()
        if not line:
            continue
        sample = parse_sample(line, time.time())
        if sample is not None:
            samples.append(sample)
            if echo:
                rpm = " ".join(f"M{i + 1}:{value:4d}" for i, value in enumerate(sample.rpms))
                print(f"\r{rpm} RPM  目标:{sample.values['target']:4d}", end="", flush=True)
        elif echo and not line.startswith("CSV:"):
            print(f"\n{line}")
    if echo:
        print()
    return samples


def save_csv(samples: list[Sample], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=["received_s", *TELEMETRY_COLUMNS])
        writer.writeheader()
        for sample in samples:
            writer.writerow({"received_s": f"{sample.received_s:.3f}", **sample.values})


def report(samples: list[Sample], settle_s: float) -> None:
    if not samples:
        print("未收到有效遥测数据。")
        return
    start = samples[0].received_s
    steady = [sample for sample in samples if sample.received_s - start >= settle_s]
    if len(steady) < 3:
        steady = samples
        print("警告：稳态样本不足，改用全部样本计算。")

    target = abs(int(steady[-1].values["target"]))
    means = [statistics.fmean(abs(sample.rpms[i]) for sample in steady) for i in range(4)]
    deviations = [statistics.pstdev(abs(sample.rpms[i]) for sample in steady) for i in range(4)]
    spreads = [max(map(abs, sample.rpms)) - min(map(abs, sample.rpms)) for sample in steady]

    print(f"样本数: {len(samples)}，稳态样本: {len(steady)}，目标: {target} RPM")
    for index, (mean, deviation) in enumerate(zip(means, deviations), start=1):
        print(
            f"M{index}: 均值 {mean:6.2f} RPM，标准差 {deviation:5.2f}，"
            f"稳态误差 {mean - target:+6.2f}"
        )
    print(
        f"四轮瞬时离散度: 平均 {statistics.fmean(spreads):.2f} RPM，"
        f"最大 {max(spreads):.2f} RPM"
    )


def run_step(link: MotorLink, args: argparse.Namespace) -> None:
    samples: list[Sample] = []
    try:
        print(link.command("MRUN 0"))
        print(link.command(f"MMODE {args.mode}"))
        print(link.command(f"MDIR {args.direction}"))
        print(link.command(f"MTARGET {args.target}"))
        print(link.command("MGET"))
        print("电机即将启动；Ctrl+C 可立即请求停止。")
        print(link.command("MRUN 1"))
        samples = collect(link, args.duration)
    finally:
        try:
            print(link.command("MRUN 0"))
        except Exception as error:
            print(f"警告：停止命令未确认，请人工断开电机电源：{error}", file=sys.stderr)

    if args.output:
        save_csv(samples, args.output)
        print(f"数据已保存到 {args.output.resolve()}")
    report(samples, args.settle)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM14", help="串口名称，默认 COM14")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    subparsers = parser.add_subparsers(dest="action", required=True)

    monitor = subparsers.add_parser("monitor", help="只读监视并分析遥测")
    monitor.add_argument("--duration", type=float, default=10.0)
    monitor.add_argument("--settle", type=float, default=2.0)
    monitor.add_argument("--output", type=Path)

    send = subparsers.add_parser("send", help="发送一条调参命令")
    send.add_argument("command", help='例如 "MSET KP ALL 0.8"')

    step = subparsers.add_parser("step", help="明确启动一次有自动停机保护的测试")
    step.add_argument("--target", type=int, default=40, choices=range(20, 131))
    step.add_argument("--duration", type=float, default=10.0)
    step.add_argument("--settle", type=float, default=3.0)
    step.add_argument("--mode", choices=("OPEN", "CLOSED"), default="CLOSED")
    step.add_argument("--direction", choices=("FWD", "REV"), default="FWD")
    step.add_argument("--output", type=Path, default=Path("motor_test.csv"))
    return parser


def main() -> int:
    args = build_parser().parse_args()
    link = MotorLink(args.port, args.baud)
    try:
        if args.action == "send":
            print(link.command(args.command))
        elif args.action == "monitor":
            samples = collect(link, args.duration)
            if args.output:
                save_csv(samples, args.output)
                print(f"数据已保存到 {args.output.resolve()}")
            report(samples, args.settle)
        elif args.action == "step":
            run_step(link, args)
    except KeyboardInterrupt:
        print("\n已中断。")
        if args.action == "step":
            try:
                link.send("MRUN 0")
            except serial.SerialException:
                pass
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
