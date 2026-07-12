from __future__ import annotations

import csv
import math
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Optional, Union

from lifted_sim import LiftedCarSimulator
from simulator import PathSimulator
from telemetry import TelemetryFrame, csv_header, frame_to_csv_row, parse_telemetry_line

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover - depends on local optional package
    serial = None


SerialEvent = Union[TelemetryFrame, tuple[str, str]]


def format_map_overlay_lines(
    frame: TelemetryFrame,
    raw_count: int,
    tel_count: int,
    lifted_enabled: bool,
    recorded_count: int,
    actual_count: int,
) -> list[str]:
    return [
        f"MODE {frame.mode}   TEL {tel_count} RAW {raw_count}   LIFT {'ON' if lifted_enabled else 'OFF'}",
        f"x={frame.x:.1f} y={frame.y:.1f} h={frame.h:.1f} d={frame.d:.1f}cm",
        f"v={frame.v:.1f} st={frame.st:.1f} e={frame.e:.1f} p={frame.p}/{frame.n}",
        f"adc={frame.adc} se={frame.se} so={frame.so}   L={frame.l} R={frame.r}",
        f"wh={frame.wh:.1f} sa={frame.sa:.1f} th={frame.th:.1f} hc={frame.hc:.2f} co={frame.co}",
        f"rec={recorded_count} run={actual_count}",
    ]


class SerialReader:
    def __init__(self, event_queue: queue.Queue[SerialEvent]) -> None:
        self._event_queue = event_queue
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._port = None

    def start(self, port_name: str, baudrate: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.stop()
        self._stop_event.clear()
        self._port = serial.Serial(port_name, baudrate, timeout=0.2)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._port is not None:
            try:
                self._port.close()
            finally:
                self._port = None

    def _run(self) -> None:
        while not self._stop_event.is_set() and self._port is not None:
            try:
                raw_line = self._port.readline()
            except Exception:
                break
            if not raw_line:
                continue
            line = raw_line.decode("utf-8", errors="ignore")
            self._event_queue.put(("raw", line.strip()))
            frame = parse_telemetry_line(line)
            if frame is not None:
                self._event_queue.put(frame)


class CarfastHostApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Carfast 上位机测试 App")
        self.geometry("1180x760")
        self.minsize(980, 640)

        self._event_queue: queue.Queue[SerialEvent] = queue.Queue()
        self._serial_reader = SerialReader(self._event_queue)
        self._latest = TelemetryFrame()
        self._recorded_points: list[tuple[float, float]] = []
        self._actual_points: list[tuple[float, float]] = []
        self._frames: list[TelemetryFrame] = []
        self._sim_enabled = False
        self._simulator = PathSimulator()
        self._lifted_sim_enabled = False
        self._lifted_simulator = LiftedCarSimulator()
        self._raw_line_count = 0
        self._tel_frame_count = 0
        self._lifted_debug_count = 0

        self._metric_vars: dict[str, tk.StringVar] = {}
        self._status_var = tk.StringVar(value="未连接")
        self._port_var = tk.StringVar()
        self._baud_var = tk.StringVar(value="115200")

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(self, padding=(10, 8))
        toolbar.grid(row=0, column=0, sticky="ew")
        toolbar.columnconfigure(10, weight=1)

        ttk.Label(toolbar, text="串口").grid(row=0, column=0, padx=(0, 6))
        self._port_combo = ttk.Combobox(toolbar, textvariable=self._port_var, width=18)
        self._port_combo.grid(row=0, column=1, padx=(0, 8))
        ttk.Button(toolbar, text="刷新", command=self._refresh_ports).grid(row=0, column=2, padx=(0, 8))

        ttk.Label(toolbar, text="波特率").grid(row=0, column=3, padx=(0, 6))
        ttk.Entry(toolbar, textvariable=self._baud_var, width=10).grid(row=0, column=4, padx=(0, 8))
        ttk.Button(toolbar, text="连接", command=self._connect).grid(row=0, column=5, padx=(0, 6))
        ttk.Button(toolbar, text="断开", command=self._disconnect).grid(row=0, column=6, padx=(0, 12))
        ttk.Button(toolbar, text="模拟数据", command=self._toggle_sim).grid(row=0, column=7, padx=(0, 6))
        ttk.Button(toolbar, text="架车仿真", command=self._toggle_lifted_sim).grid(row=0, column=8, padx=(0, 6))
        ttk.Button(toolbar, text="清空轨迹", command=self._clear_trace).grid(row=0, column=9, padx=(0, 6))
        ttk.Button(toolbar, text="保存CSV", command=self._save_csv).grid(row=0, column=10, padx=(0, 12))
        ttk.Label(toolbar, textvariable=self._status_var).grid(row=0, column=11, sticky="e")

        main = ttk.Frame(self, padding=(10, 0, 10, 10))
        main.grid(row=1, column=0, sticky="nsew")
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        self._canvas = tk.Canvas(main, background="#111827", highlightthickness=0)
        self._canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        side = ttk.Frame(main)
        side.grid(row=0, column=1, sticky="nsew")
        side.columnconfigure(0, weight=1)

        self._build_metrics(side)
        self._build_log(side)

    def _build_metrics(self, parent: ttk.Frame) -> None:
        panel = ttk.LabelFrame(parent, text="实时数据", padding=10)
        panel.grid(row=0, column=0, sticky="ew")
        panel.columnconfigure(1, weight=1)

        rows = [
            ("mode", "模式"),
            ("t", "时间"),
            ("x", "X cm"),
            ("y", "Y cm"),
            ("h", "航向 deg"),
            ("d", "距离 cm"),
            ("v", "速度 V"),
            ("st", "转向 ST"),
            ("p", "目标 P"),
            ("e", "航向误差 E"),
            ("adc", "方向 ADC"),
            ("se", "舵机误差"),
            ("so", "舵机输出"),
            ("l", "左编码器"),
            ("r", "右编码器"),
            ("wh", "轮差航向"),
            ("sa", "前轮角"),
            ("th", "转向航向"),
            ("hc", "航向修正"),
            ("co", "一致性"),
        ]
        for row, (key, label) in enumerate(rows):
            ttk.Label(panel, text=label).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="--")
            self._metric_vars[key] = var
            ttk.Label(panel, textvariable=var, anchor="e").grid(row=row, column=1, sticky="ew", pady=2)

    def _build_log(self, parent: ttk.Frame) -> None:
        panel = ttk.LabelFrame(parent, text="提示", padding=10)
        panel.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        parent.rowconfigure(1, weight=1)

        self._log = tk.Text(panel, height=10, wrap="word")
        self._log.pack(fill="both", expand=True)
        self._append_log("先点“模拟数据”可以不接车看界面。")
        self._append_log("车端串口输出 TEL 行后，连接串口即可实时画图。")
        self._append_log("模拟数据使用轴距 62.5cm 的简化车辆模型。")
        self._append_log("架车测试时点“架车仿真”，用编码器+方向ADC计算虚拟转弯。")

    def _refresh_ports(self) -> None:
        if serial is None:
            self._port_combo["values"] = []
            self._status_var.set("未安装 pyserial，可用模拟数据")
            return

        ports = [port.device for port in serial.tools.list_ports.comports()]
        self._port_combo["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _connect(self) -> None:
        port_name = self._port_var.get().strip()
        if not port_name:
            messagebox.showwarning("缺少串口", "请先选择串口。")
            return
        try:
            baudrate = int(self._baud_var.get())
            self._serial_reader.start(port_name, baudrate)
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))
            return
        self._sim_enabled = False
        self._status_var.set(f"已连接 {port_name}")
        self._append_log(f"串口已连接：{port_name}")

    def _disconnect(self) -> None:
        self._serial_reader.stop()
        self._status_var.set("已断开")
        self._append_log("串口已断开。")

    def _toggle_sim(self) -> None:
        self._sim_enabled = not self._sim_enabled
        if self._sim_enabled:
            self._simulator = PathSimulator()
            self._clear_trace()
            self._status_var.set("模拟数据运行中")
            self._append_log("模拟数据已开启。")
        else:
            self._status_var.set("模拟数据已停止")
            self._append_log("模拟数据已停止。")

    def _clear_trace(self) -> None:
        self._recorded_points.clear()
        self._actual_points.clear()
        self._frames.clear()
        self._lifted_simulator.reset()
        self._draw_map()
        self._append_log("轨迹已清空。")

    def _toggle_lifted_sim(self) -> None:
        self._lifted_sim_enabled = not self._lifted_sim_enabled
        self._lifted_simulator.reset()
        self._recorded_points.clear()
        self._actual_points.clear()
        self._lifted_debug_count = 0
        state = "开启" if self._lifted_sim_enabled else "关闭"
        self._append_log(f"架车仿真已{state}。")
        self._append_log(f"LIFT mode {'ON' if self._lifted_sim_enabled else 'OFF'}")

    def _save_csv(self) -> None:
        if not self._frames:
            messagebox.showinfo("没有数据", "当前没有遥测数据可保存。")
            return
        default_name = time.strftime("carfast_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            title="保存遥测 CSV",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV 文件", "*.csv")],
        )
        if not path:
            return

        with Path(path).open("w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.writer(fp)
            writer.writerow(csv_header())
            for frame in self._frames:
                writer.writerow(frame_to_csv_row(frame))
        self._append_log(f"已保存：{path}")

    def _tick(self) -> None:
        if self._sim_enabled:
            self._event_queue.put(self._simulator.step())

        updated = False
        while True:
            try:
                event = self._event_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(event, TelemetryFrame):
                self._tel_frame_count += 1
                if self._lifted_sim_enabled and not self._sim_enabled:
                    event = self._lifted_simulator.apply(event)
                    event.mode = f"LIFT-{event.mode}"
                    self._lifted_debug_count += 1
                    if self._lifted_debug_count <= 5 or self._lifted_debug_count % 20 == 0:
                        self._append_log(
                            "LIFT "
                            f"src={self._lifted_simulator.last_distance_source} "
                            f"ds={self._lifted_simulator.last_distance_delta_cm:.1f} "
                            f"sa={self._lifted_simulator.last_steer_deg:.1f}"
                        )
                self._accept_frame(event)
                updated = True
            else:
                self._raw_line_count += 1
                if self._raw_line_count <= 5 or self._raw_line_count % 20 == 0:
                    self._append_log(f"RAW#{self._raw_line_count}: {event[1]}")

        if updated:
            self._update_metrics()
            self._draw_map()

        if self._raw_line_count or self._tel_frame_count:
            lifted = " LIFT:ON" if self._lifted_sim_enabled else ""
            self._status_var.set(
                f"RAW:{self._raw_line_count} TEL:{self._tel_frame_count}{lifted}"
            )

        self.after(50, self._tick)

    def _accept_frame(self, frame: TelemetryFrame) -> None:
        self._latest = frame
        self._frames.append(frame)
        point = (frame.x, frame.y)
        mode = frame.mode.upper()
        if mode in {"REC", "RECORD", "RECORDING"} or mode.startswith("LIFT-REC"):
            self._recorded_points.append(point)
        else:
            self._actual_points.append(point)

        if len(self._frames) > 20000:
            self._frames = self._frames[-10000:]
        if len(self._recorded_points) > 2000:
            self._recorded_points = self._recorded_points[-2000:]
        if len(self._actual_points) > 2000:
            self._actual_points = self._actual_points[-2000:]

    def _update_metrics(self) -> None:
        f = self._latest
        values = {
            "mode": f.mode,
            "t": str(f.t),
            "x": f"{f.x:.1f}",
            "y": f"{f.y:.1f}",
            "h": f"{f.h:.1f}",
            "d": f"{f.d:.1f}",
            "v": f"{f.v:.1f}",
            "st": f"{f.st:.1f}",
            "p": f"{f.p}/{f.n}",
            "e": f"{f.e:.1f}",
            "adc": str(f.adc),
            "se": str(f.se),
            "so": str(f.so),
            "l": str(f.l),
            "r": str(f.r),
            "wh": f"{f.wh:.1f}",
            "sa": f"{f.sa:.1f}",
            "th": f"{f.th:.1f}",
            "hc": f"{f.hc:.2f}",
            "co": str(f.co),
        }
        for key, value in values.items():
            self._metric_vars[key].set(value)

    def _draw_map(self) -> None:
        canvas = self._canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 10)
        height = max(canvas.winfo_height(), 10)

        points = self._recorded_points + self._actual_points
        if not points:
            canvas.create_text(
                width / 2,
                height / 2,
                text="等待 TEL 遥测数据",
                fill="#9ca3af",
                font=("Microsoft YaHei UI", 16),
            )
            return

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(max_x - min_x, 100.0)
        span_y = max(max_y - min_y, 100.0)
        margin = 48
        scale = min((width - margin * 2) / span_x, (height - margin * 2) / span_y)

        def project(point: tuple[float, float]) -> tuple[float, float]:
            x, y = point
            sx = margin + (x - min_x) * scale
            sy = height - margin - (y - min_y) * scale
            return sx, sy

        self._draw_grid(canvas, width, height)
        self._draw_polyline(canvas, self._recorded_points, project, "#9ca3af", 2)
        self._draw_polyline(canvas, self._actual_points, project, "#22d3ee", 3)
        self._draw_endpoint_markers(canvas, project)
        self._draw_car_marker(canvas, project)
        self._draw_scale_bar(canvas, scale, width, height)
        self._draw_map_overlay(canvas)

        canvas.create_text(14, 14, anchor="nw", text="灰色=录制路线  青色=实际路线", fill="#e5e7eb")

    def _draw_grid(self, canvas: tk.Canvas, width: int, height: int) -> None:
        for x in range(0, width, 60):
            canvas.create_line(x, 0, x, height, fill="#1f2937")
        for y in range(0, height, 60):
            canvas.create_line(0, y, width, y, fill="#1f2937")

    def _draw_polyline(self, canvas, points, project, color: str, width: int) -> None:
        if len(points) < 2:
            return
        coords: list[float] = []
        for point in points:
            coords.extend(project(point))
        canvas.create_line(*coords, fill=color, width=width, smooth=True)

    def _draw_endpoint_markers(self, canvas, project) -> None:
        if self._recorded_points:
            sx, sy = project(self._recorded_points[0])
            canvas.create_oval(sx - 6, sy - 6, sx + 6, sy + 6, fill="#22c55e", outline="#dcfce7")
            canvas.create_text(
                sx + 10,
                sy - 10,
                anchor="sw",
                text="START",
                fill="#dcfce7",
                font=("Consolas", 10, "bold"),
            )
            ex, ey = project(self._recorded_points[-1])
            canvas.create_rectangle(ex - 6, ey - 6, ex + 6, ey + 6, fill="#f43f5e", outline="#ffe4e6")
            canvas.create_text(
                ex + 10,
                ey + 10,
                anchor="nw",
                text="REC END",
                fill="#ffe4e6",
                font=("Consolas", 10, "bold"),
            )
        if self._actual_points:
            ax, ay = project(self._actual_points[-1])
            canvas.create_oval(ax - 5, ay - 5, ax + 5, ay + 5, fill="#38bdf8", outline="#e0f2fe")

    def _draw_car_marker(self, canvas, project) -> None:
        f = self._latest
        x, y = project((f.x, f.y))
        heading = math.radians(f.h)
        tip = (x + math.cos(heading) * 18, y - math.sin(heading) * 18)
        left = (x + math.cos(heading + 2.5) * 12, y - math.sin(heading + 2.5) * 12)
        right = (x + math.cos(heading - 2.5) * 12, y - math.sin(heading - 2.5) * 12)
        canvas.create_polygon(tip, left, right, fill="#facc15", outline="#fef3c7")
        canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#f97316", outline="")

    def _draw_scale_bar(self, canvas: tk.Canvas, scale: float, width: int, height: int) -> None:
        pixels = max(10.0, 100.0 * scale)
        x0 = 14
        y0 = height - 18
        x1 = min(width - 14, x0 + pixels)
        canvas.create_line(x0, y0, x1, y0, fill="#f9fafb", width=3)
        canvas.create_line(x0, y0 - 5, x0, y0 + 5, fill="#f9fafb", width=2)
        canvas.create_line(x1, y0 - 5, x1, y0 + 5, fill="#f9fafb", width=2)
        canvas.create_text(
            x0,
            y0 - 8,
            anchor="sw",
            text="100 cm",
            fill="#f9fafb",
            font=("Consolas", 10, "bold"),
        )

    def _draw_map_overlay(self, canvas: tk.Canvas) -> None:
        lines = format_map_overlay_lines(
            self._latest,
            raw_count=self._raw_line_count,
            tel_count=self._tel_frame_count,
            lifted_enabled=self._lifted_sim_enabled,
            recorded_count=len(self._recorded_points),
            actual_count=len(self._actual_points),
        )
        x0 = 12
        y0 = 12
        line_height = 20
        panel_width = 430
        panel_height = 18 + line_height * len(lines)
        canvas.create_rectangle(
            x0,
            y0,
            x0 + panel_width,
            y0 + panel_height,
            fill="#020617",
            outline="#334155",
        )
        for index, line in enumerate(lines):
            canvas.create_text(
                x0 + 10,
                y0 + 10 + index * line_height,
                anchor="nw",
                text=line,
                fill="#f8fafc",
                font=("Consolas", 12, "bold"),
            )

    def _append_log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self._log.insert("end", f"[{timestamp}] {text}\n")
        self._log.see("end")

    def _on_close(self) -> None:
        self._serial_reader.stop()
        self.destroy()


def main() -> None:
    app = CarfastHostApp()
    app.mainloop()


if __name__ == "__main__":
    main()
