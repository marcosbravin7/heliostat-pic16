"""
Custom telemetry and visualization widgets for the 2-Axis Light Tracker GUI:
1. Radar2D: 2D Pan/Tilt Compass & Crosshair with motion trail and virtual sun drag
2. TimeSeriesChart: Real-time scrolling oscilloscope chart (PWM µs / Angle ° / Counts)
3. ProtocolInspector: Live [0xFF, pulso_x, pulso_y] byte-frame breakdown & UART echo log
"""

from __future__ import annotations

import collections
import time
import tkinter as tk
from typing import Callable, Deque, Optional, Tuple

import customtkinter as ctk

from serial_protocol import (
    PULSE_CENTER,
    PULSE_MAX,
    PULSE_MIN,
    EchoEvent,
    TelemetryEvent,
    TxEvent,
    clamp_counts,
    counts_to_deg,
    counts_to_us,
)


class Radar2D(tk.Frame):
    """
    2D Pan/Tilt Compass & Reticle canvas.
    Maps X-axis [1..251] -> West (0°) to East (180°)
         Y-axis [1..251] -> South (0°) to North (180°)
    In Simulator mode, clicking/dragging moves the virtual light source.
    """

    def __init__(
        self,
        master: tk.Misc,
        on_sun_move: Optional[Callable[[float, float], None]] = None,
        **kwargs,
    ) -> None:
        super().__init__(master, bg="#0F172A", **kwargs)
        self.on_sun_move = on_sun_move

        self.pulse_x: int = PULSE_CENTER
        self.pulse_y: int = PULSE_CENTER
        self.sun_counts: Optional[Tuple[float, float]] = None
        self._trail: Deque[Tuple[int, int]] = collections.deque(maxlen=45)
        self._trail.append((PULSE_CENTER, PULSE_CENTER))
        self._last_size: Tuple[int, int] = (0, 0)

        self.canvas = tk.Canvas(
            self,
            bg="#0F172A",
            highlightthickness=1,
            highlightbackground="#1E293B",
            cursor="crosshair",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<Configure>", self._on_resize)
        self.canvas.bind("<ButtonPress-1>", self._on_mouse_action)
        self.canvas.bind("<B1-Motion>", self._on_mouse_action)

    def _on_resize(self, event: tk.Event) -> None:
        new_size = (event.width, event.height)
        if new_size != self._last_size and event.width >= 40 and event.height >= 40:
            self._last_size = new_size
            self.render()

    def update_state(
        self,
        pulse_x: int,
        pulse_y: int,
        sun_counts: Optional[Tuple[float, float]] = None,
    ) -> None:
        if not self._trail or self._trail[-1] != (pulse_x, pulse_y):
            self._trail.append((pulse_x, pulse_y))
        self.pulse_x = pulse_x
        self.pulse_y = pulse_y
        self.sun_counts = sun_counts
        self.render()

    def _counts_to_canvas(self, val_x: float, val_y: float, cx: float, cy: float, radius: float) -> Tuple[float, float]:
        nx = (val_x - PULSE_CENTER) / 125.0  # -1.0 .. +1.0
        ny = (val_y - PULSE_CENTER) / 125.0  # -1.0 .. +1.0
        return cx + nx * radius, cy - ny * radius

    def _canvas_to_counts(self, sx: float, sy: float, cx: float, cy: float, radius: float) -> Tuple[float, float]:
        if radius <= 1.0:
            return float(PULSE_CENTER), float(PULSE_CENTER)
        nx = max(-1.0, min(1.0, (sx - cx) / radius))
        ny = max(-1.0, min(1.0, (cy - sy) / radius))
        val_x = clamp_counts(round(PULSE_CENTER + nx * 125.0))
        val_y = clamp_counts(round(PULSE_CENTER + ny * 125.0))
        return float(val_x), float(val_y)

    def _on_mouse_action(self, event: tk.Event) -> None:
        if self.on_sun_move is None:
            return
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        cx = w * 0.5
        cy = h * 0.53
        radius = max(20.0, min(w, h) * 0.37)
        sx, sy = self._canvas_to_counts(event.x, event.y, cx, cy, radius)
        self.on_sun_move(sx, sy)

    def render(self) -> None:
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 40 or h < 40:
            return
        self.canvas.delete("all")

        cx = w * 0.5
        cy = h * 0.53
        radius = max(20.0, min(w, h) * 0.37)

        # Header
        self.canvas.create_text(
            12,
            10,
            anchor="nw",
            text="2D Pan/Tilt Compass Radar (N/S/E/W)",
            fill="#E2E8F0",
            font=("Segoe UI", 10, "bold"),
        )

        # Outer bounding square (full servo workspace [1..251] x [1..251])
        self.canvas.create_rectangle(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            outline="#1E293B",
            fill="#0B1120",
            width=1,
        )

        # Concentric angular rings (30°, 60°, 90° from center)
        for frac, deg_lbl in ((1.0 / 3.0, "±30°"), (2.0 / 3.0, "±60°"), (1.0, "±90°")):
            r = radius * frac
            self.canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                outline="#1E293B",
                dash=(3, 3) if frac < 1.0 else (),
                width=1,
            )
            self.canvas.create_text(
                cx + 4,
                cy - r + 8,
                anchor="nw",
                text=deg_lbl,
                fill="#475569",
                font=("Consolas", 8),
            )

        # Center axes
        self.canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#334155", width=1)
        self.canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#334155", width=1)

        # Cardinal labels
        self.canvas.create_text(cx, cy - radius - 12, text="N — North (Y=251 / 180°)", fill="#34D399", font=("Segoe UI", 8, "bold"))
        self.canvas.create_text(cx, cy + radius + 12, text="S — South (Y=1 / 0°)", fill="#F87171", font=("Segoe UI", 8, "bold"))
        self.canvas.create_text(cx + radius + 8, cy, anchor="w", text="E\n180°", fill="#38BDF8", font=("Segoe UI", 8, "bold"))
        self.canvas.create_text(cx - radius - 8, cy, anchor="e", text="W\n0°", fill="#A78BFA", font=("Segoe UI", 8, "bold"))

        # Motion trail
        if len(self._trail) > 1:
            pts = [self._counts_to_canvas(px, py, cx, cy, radius) for px, py in self._trail]
            for i in range(len(pts) - 1):
                alpha = (i + 1) / len(pts)
                col = "#1D4ED8" if alpha < 0.6 else "#38BDF8"
                self.canvas.create_line(
                    pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1],
                    fill=col, width=1.5 if alpha < 0.6 else 2.2,
                )

        # Virtual Sun indicator (when simulator mode is active)
        if self.sun_counts is not None:
            sun_sx, sun_sy = self._counts_to_canvas(self.sun_counts[0], self.sun_counts[1], cx, cy, radius)
            self.canvas.create_oval(
                sun_sx - 10, sun_sy - 10, sun_sx + 10, sun_sy + 10,
                outline="#FBBF24", fill="#78350F", width=2,
            )
            self.canvas.create_oval(
                sun_sx - 4, sun_sy - 4, sun_sx + 4, sun_sy + 4,
                outline="", fill="#FDE047",
            )
            self.canvas.create_text(
                sun_sx + 13, sun_sy - 8,
                anchor="w",
                text="Virtual Light",
                fill="#FBBF24",
                font=("Segoe UI", 8, "bold"),
            )

        # Current Servo Target Reticle
        tx, ty = self._counts_to_canvas(self.pulse_x, self.pulse_y, cx, cy, radius)
        self.canvas.create_line(tx, cy, tx, ty, fill="#0284C7", dash=(2, 2))
        self.canvas.create_line(cx, ty, tx, ty, fill="#D97706", dash=(2, 2))

        self.canvas.create_oval(tx - 9, ty - 9, tx + 9, ty + 9, outline="#38BDF8", width=2)
        self.canvas.create_oval(tx - 3, ty - 3, tx + 3, ty + 3, fill="#F8FAFC", outline="")
        self.canvas.create_line(tx - 14, ty, tx + 14, ty, fill="#38BDF8", width=1.5)
        self.canvas.create_line(tx, ty - 14, tx, ty + 14, fill="#38BDF8", width=1.5)

        deg_x = counts_to_deg(self.pulse_x)
        deg_y = counts_to_deg(self.pulse_y)
        self.canvas.create_text(
            12,
            28,
            anchor="nw",
            text=f"X: {self.pulse_x:3d} ({deg_x:5.1f}°)   Y: {self.pulse_y:3d} ({deg_y:5.1f}°)",
            fill="#94A3B8",
            font=("Consolas", 9),
        )
        if self.sun_counts is not None:
            self.canvas.create_text(
                w - 10,
                h - 8,
                anchor="se",
                text="Click/Drag on Radar to move Virtual Light",
                fill="#FBBF24",
                font=("Segoe UI", 8),
            )


class TimeSeriesChart(ctk.CTkFrame):
    """
    Real-time scrolling dual-trace oscilloscope chart for X and Y servo telemetry.
    Supports switching between Pulse Width (µs), Servo Angle (°), and Raw Counts.
    """

    WINDOW_SECONDS: float = 15.0

    def __init__(self, master: tk.Misc, **kwargs) -> None:
        super().__init__(master, fg_color="#0F172A", corner_radius=8, **kwargs)
        self.pack_propagate(False)

        self._history: Deque[Tuple[float, int, int]] = collections.deque(maxlen=300)
        self._unit_mode: str = "Pulse Width (µs)"
        self._last_size: Tuple[int, int] = (0, 0)

        # Header bar
        top_bar = ctk.CTkFrame(self, fg_color="transparent", height=28)
        top_bar.pack(fill=tk.X, padx=10, pady=(6, 2))
        top_bar.pack_propagate(False)

        ctk.CTkLabel(
            top_bar,
            text="Real-Time Telemetry Chart (15s Rolling Window)",
            font=ctk.CTkFont(family="Segoe UI", size=12, weight="bold"),
            text_color="#E2E8F0",
        ).pack(side=tk.LEFT)

        self.unit_selector = ctk.CTkSegmentedButton(
            top_bar,
            values=["Pulse Width (µs)", "Angle (°)", "Counts (1-251)"],
            command=self._on_unit_change,
            height=24,
            font=ctk.CTkFont(size=11),
        )
        self.unit_selector.set("Pulse Width (µs)")
        self.unit_selector.pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(
            self,
            bg="#0B1120",
            highlightthickness=1,
            highlightbackground="#1E293B",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 8))
        self.canvas.bind("<Configure>", self._on_resize)

    def _on_resize(self, event: tk.Event) -> None:
        new_size = (event.width, event.height)
        if new_size != self._last_size and event.width >= 60 and event.height >= 40:
            self._last_size = new_size
            self.render()

    def _on_unit_change(self, value: str) -> None:
        self._unit_mode = value
        self.render()

    def record_sample(self, timestamp: float, pulse_x: int, pulse_y: int) -> None:
        """Append sample without triggering a redraw yet."""
        self._history.append((timestamp, pulse_x, pulse_y))

    def add_sample(self, timestamp: float, pulse_x: int, pulse_y: int) -> None:
        self._history.append((timestamp, pulse_x, pulse_y))
        self.render()

    def clear(self) -> None:
        self._history.clear()
        self.render()

    def _convert(self, counts: int) -> float:
        if self._unit_mode == "Angle (°)":
            return counts_to_deg(counts)
        if self._unit_mode == "Counts (1-251)":
            return float(counts)
        return float(counts_to_us(counts))

    def render(self) -> None:
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 60 or h < 40:
            return
        self.canvas.delete("all")

        pad_left = 54.0
        pad_right = 16.0
        pad_top = 14.0
        pad_bottom = 22.0
        plot_w = max(10.0, w - pad_left - pad_right)
        plot_h = max(10.0, h - pad_top - pad_bottom)

        if self._unit_mode == "Angle (°)":
            y_min, y_mid, y_max, fmt = 0.0, 90.0, 180.0, "{:.0f}°"
        elif self._unit_mode == "Counts (1-251)":
            y_min, y_mid, y_max, fmt = float(PULSE_MIN), float(PULSE_CENTER), float(PULSE_MAX), "{:.0f}"
        else:
            y_min, y_mid, y_max, fmt = 500.0, 1500.0, 2500.0, "{:.0f} µs"

        # Grid lines & Y-axis labels
        for val in (y_min, 0.5 * (y_min + y_mid), y_mid, 0.5 * (y_mid + y_max), y_max):
            norm = (val - y_min) / (y_max - y_min)
            sy = pad_top + plot_h * (1.0 - norm)
            is_center = abs(val - y_mid) < 1e-3
            self.canvas.create_line(
                pad_left,
                sy,
                pad_left + plot_w,
                sy,
                fill="#334155" if is_center else "#1E293B",
                dash=() if is_center else (3, 3),
            )
            self.canvas.create_text(
                pad_left - 6,
                sy,
                anchor="e",
                text=fmt.format(val),
                fill="#94A3B8",
                font=("Consolas", 8),
            )

        # Legend
        self.canvas.create_text(
            pad_left + 8,
            pad_top + 2,
            anchor="nw",
            text="━ X-Axis (East/West)",
            fill="#38BDF8",
            font=("Segoe UI", 9, "bold"),
        )
        self.canvas.create_text(
            pad_left + 150,
            pad_top + 2,
            anchor="nw",
            text="━ Y-Axis (North/South)",
            fill="#F59E0B",
            font=("Segoe UI", 9, "bold"),
        )

        if len(self._history) < 2:
            self.canvas.create_text(
                pad_left + plot_w * 0.5,
                pad_top + plot_h * 0.5,
                text="Waiting for [0xFF, pulso_x, pulso_y] telemetry stream...",
                fill="#64748B",
                font=("Segoe UI", 9),
            )
            return

        now = self._history[-1][0]
        t_start = now - self.WINDOW_SECONDS

        pts_x = []
        pts_y = []
        for ts, px, py in self._history:
            if ts < t_start:
                continue
            tx = pad_left + ((ts - t_start) / self.WINDOW_SECONDS) * plot_w
            vx = self._convert(px)
            vy = self._convert(py)
            sy_x = pad_top + plot_h * (1.0 - (vx - y_min) / (y_max - y_min))
            sy_y = pad_top + plot_h * (1.0 - (vy - y_min) / (y_max - y_min))
            pts_x.extend([tx, sy_x])
            pts_y.extend([tx, sy_y])

        if len(pts_x) >= 4:
            self.canvas.create_line(*pts_x, fill="#38BDF8", width=2, smooth=False)
        if len(pts_y) >= 4:
            self.canvas.create_line(*pts_y, fill="#F59E0B", width=2, smooth=False)

        # Time axis ticks
        for sec_ago in (15, 10, 5, 0):
            tx = pad_left + ((self.WINDOW_SECONDS - sec_ago) / self.WINDOW_SECONDS) * plot_w
            self.canvas.create_text(
                tx,
                h - 8,
                text=f"-{sec_ago}s" if sec_ago > 0 else "Now",
                fill="#64748B",
                font=("Consolas", 8),
            )


class ProtocolInspector(ctk.CTkFrame):
    """
    Displays live byte-level breakdown of `[0xFF, pulso_x, pulso_y]` frames,
    packet rate (Hz), the latest PIC keyboard echo (`RX Echo`), and a serial log.
    Uses a lightweight tk.Canvas banner for the 10 Hz byte boxes to avoid
    CustomTkinter label relayout overhead.
    """

    def __init__(self, master: tk.Misc, **kwargs) -> None:
        super().__init__(master, fg_color="#0F172A", corner_radius=8, **kwargs)
        self.pack_propagate(False)

        self._packet_timestamps: Deque[float] = collections.deque(maxlen=25)
        self._log_telemetry_var = tk.BooleanVar(value=False)
        self._last_echo_str: str = "—"
        self._last_b1_str: str = "0x7E (126) → 1500 µs"
        self._last_b2_str: str = "0x7E (126) → 1500 µs"
        self._last_rate_str: str = "Rate: 0.0 Hz  |  Frames: 0  |  Echoes: 0"

        # Top header + 3-Byte Packet Breakdown Canvas (zero layout overhead on 10 Hz updates)
        self.banner_canvas = tk.Canvas(
            self,
            bg="#0F172A",
            height=68,
            highlightthickness=0,
        )
        self.banner_canvas.pack(fill=tk.X, padx=10, pady=(6, 2))
        self.banner_canvas.bind("<Configure>", lambda _e: self._draw_banner())

        # Log controls + Scrollable Textbox
        ctrl_row = ctk.CTkFrame(self, fg_color="transparent", height=24)
        ctrl_row.pack(fill=tk.X, padx=10, pady=(2, 2))
        ctrl_row.pack_propagate(False)

        ctk.CTkCheckBox(
            ctrl_row,
            text="Log 10 Hz [0xFF, X, Y] Frames",
            variable=self._log_telemetry_var,
            font=ctk.CTkFont(size=11),
            checkbox_width=16,
            checkbox_height=16,
        ).pack(side=tk.LEFT)

        ctk.CTkButton(
            ctrl_row,
            text="Clear Log",
            width=70,
            height=22,
            fg_color="#334155",
            hover_color="#475569",
            font=ctk.CTkFont(size=11),
            command=self.clear_log,
        ).pack(side=tk.RIGHT)

        self.log_box = ctk.CTkTextbox(
            self,
            height=80,
            fg_color="#0B1120",
            text_color="#CBD5E1",
            font=ctk.CTkFont(family="Consolas", size=11),
            activate_scrollbars=True,
        )
        self.log_box.pack(fill=tk.BOTH, expand=True, padx=10, pady=(2, 8))
        self.log_box.configure(state="disabled")

    def _draw_banner(self) -> None:
        w = self.banner_canvas.winfo_width()
        if w < 100:
            return
        self.banner_canvas.delete("all")

        # Title & Rate
        self.banner_canvas.create_text(
            2,
            10,
            anchor="w",
            text="UART Protocol Inspector & Echo Monitor (9600 8N1)",
            fill="#E2E8F0",
            font=("Segoe UI", 10, "bold"),
        )
        self._rate_item = self.banner_canvas.create_text(
            w - 2,
            10,
            anchor="e",
            text=self._last_rate_str,
            fill="#38BDF8",
            font=("Consolas", 9, "bold"),
        )

        # 4 Byte-breakdown boxes
        y0, y1 = 24, 64
        box_w = (w - 18) / 4.0
        specs = [
            ("Byte 0 (Sync Header)", "0xFF (255)", "#F43F5E", "b0"),
            ("Byte 1 (pulso_x)", self._last_b1_str, "#38BDF8", "b1"),
            ("Byte 2 (pulso_y)", self._last_b2_str, "#F59E0B", "b2"),
            ("Last PIC Echo (RX)", self._last_echo_str, "#34D399", "echo"),
        ]
        for idx, (title, val, col, tag) in enumerate(specs):
            x0 = idx * (box_w + 6)
            x1 = x0 + box_w
            cx = 0.5 * (x0 + x1)
            self.banner_canvas.create_rectangle(x0, y0, x1, y1, fill="#1E293B", outline="#334155")
            self.banner_canvas.create_text(cx, y0 + 11, text=title, fill="#94A3B8", font=("Segoe UI", 8))
            self.banner_canvas.create_text(
                cx, y0 + 28, text=val, fill=col, font=("Consolas", 9, "bold"), tags=(tag,)
            )

    def on_telemetry(self, ev: TelemetryEvent, total_packets: int, total_echoes: int) -> None:
        self._packet_timestamps.append(ev.timestamp)
        rate = 0.0
        if len(self._packet_timestamps) >= 2:
            dt = self._packet_timestamps[-1] - self._packet_timestamps[0]
            if dt > 0.01:
                rate = (len(self._packet_timestamps) - 1) / dt

        self._last_rate_str = f"Rate: {rate:4.1f} Hz  |  Frames: {total_packets}  |  Echoes: {total_echoes}"
        self._last_b1_str = f"0x{ev.pulse_x:02X} ({ev.pulse_x:3d}) → {ev.us_x} µs"
        self._last_b2_str = f"0x{ev.pulse_y:02X} ({ev.pulse_y:3d}) → {ev.us_y} µs"

        if hasattr(self, "_rate_item"):
            self.banner_canvas.itemconfigure(self._rate_item, text=self._last_rate_str)
            self.banner_canvas.itemconfigure("b1", text=self._last_b1_str)
            self.banner_canvas.itemconfigure("b2", text=self._last_b2_str)

        if self._log_telemetry_var.get():
            self.append_log(
                f"RX FRAME  {ev.hex_str}  |  X={ev.pulse_x:3d} ({ev.us_x} µs, {ev.deg_x:5.1f}°)  "
                f"Y={ev.pulse_y:3d} ({ev.us_y} µs, {ev.deg_y:5.1f}°)"
            )

    def on_tx(self, ev: TxEvent) -> None:
        self.append_log(f"TX CMD -> '{ev.char}' (0x{ev.raw_byte:02X}) — {ev.description}")

    def on_echo(self, ev: EchoEvent, total_packets: int, total_echoes: int) -> None:
        self._last_echo_str = f"'{ev.char}' (0x{ev.raw_byte:02X})"
        self._last_rate_str = f"Frames: {total_packets}  |  Echoes: {total_echoes}"
        if hasattr(self, "_rate_item"):
            self.banner_canvas.itemconfigure("echo", text=self._last_echo_str)
            self.banner_canvas.itemconfigure(self._rate_item, text=self._last_rate_str)
        self.append_log(f"RX ECHO <- '{ev.char}' (0x{ev.raw_byte:02X}) Confirmed by PIC ({ev.description})")

    def append_log(self, line: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{ts}] {line}\n")
        lines = int(self.log_box.index("end-1c").split(".")[0])
        if lines > 200:
            self.log_box.delete("1.0", "40.0")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def clear_log(self) -> None:
        self.log_box.configure(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.configure(state="disabled")
