"""
Custom monochrome telemetry and visualization widgets for the 2-Axis Light Tracker GUI:
1. Radar2D: 2D Pan/Tilt Compass & Crosshair in pure black, white & layered greys
2. TimeSeriesChart: Real-time scrolling oscilloscope chart (White X trace, Grey Y trace)
3. ProtocolInspector: Live [0xFF, pulso_x, pulso_y] byte-frame breakdown & UART echo log

Typography:
- UI Headers & Labels: Bahnschrift (DIN-1451 industrial sans-serif)
- Numeric Telemetry, Hex Frames & Terminal Log: Cascadia Mono
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

FONT_UI = "Bahnschrift"
FONT_MONO = "Cascadia Mono"


class Radar2D(tk.Frame):
    """
    Monochrome 2D Pan/Tilt Compass & Reticle canvas.
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
        super().__init__(master, bg="#000000", **kwargs)
        self.on_sun_move = on_sun_move

        self.pulse_x: int = PULSE_CENTER
        self.pulse_y: int = PULSE_CENTER
        self.sun_counts: Optional[Tuple[float, float]] = None
        self._trail: Deque[Tuple[int, int]] = collections.deque(maxlen=45)
        self._trail.append((PULSE_CENTER, PULSE_CENTER))
        self._last_size: Tuple[int, int] = (0, 0)

        self.canvas = tk.Canvas(
            self,
            bg="#000000",
            highlightthickness=1,
            highlightbackground="#26262B",
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
        cy = h * 0.55
        radius = max(20.0, min(w, h) * 0.36)
        sx, sy = self._canvas_to_counts(event.x, event.y, cx, cy, radius)
        self.on_sun_move(sx, sy)

    def render(self) -> None:
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 40 or h < 40:
            return
        self.canvas.delete("all")

        cx = w * 0.5
        cy = h * 0.55
        radius = max(20.0, min(w, h) * 0.36)

        # Outer bounding square (full servo workspace [1..251] x [1..251])
        self.canvas.create_rectangle(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            outline="#1F1F24",
            fill="#050507",
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
                outline="#27272A" if frac == 1.0 else "#18181C",
                dash=(3, 3) if frac < 1.0 else (),
                width=1,
            )
            self.canvas.create_text(
                cx + 5,
                cy - r + 8,
                anchor="nw",
                text=deg_lbl,
                fill="#52525B",
                font=(FONT_MONO, 8),
            )

        # Center axes
        self.canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#27272A", width=1)
        self.canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#27272A", width=1)

        # Cardinal labels
        self.canvas.create_text(cx, cy - radius - 12, text="N  ·  NORTH (180°)", fill="#FFFFFF", font=(FONT_UI, 9, "bold"))
        self.canvas.create_text(cx, cy + radius + 12, text="S  ·  SOUTH (0°)", fill="#71717A", font=(FONT_UI, 9, "bold"))
        self.canvas.create_text(cx + radius + 8, cy, anchor="w", text="E\n180°", fill="#D4D4D8", font=(FONT_UI, 9, "bold"))
        self.canvas.create_text(cx - radius - 8, cy, anchor="e", text="W\n0°", fill="#71717A", font=(FONT_UI, 9, "bold"))

        # Monochrome gradient motion trail
        if len(self._trail) > 1:
            pts = [self._counts_to_canvas(px, py, cx, cy, radius) for px, py in self._trail]
            n_seg = len(pts) - 1
            for i in range(n_seg):
                alpha = (i + 1) / float(n_seg)
                if alpha < 0.4:
                    col, lw = "#27272A", 1.2
                elif alpha < 0.75:
                    col, lw = "#52525B", 1.6
                else:
                    col, lw = "#A1A1AA", 2.0
                self.canvas.create_line(
                    pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1],
                    fill=col, width=lw,
                )

        # Virtual Sun indicator (when simulator mode is active)
        if self.sun_counts is not None:
            sun_sx, sun_sy = self._counts_to_canvas(self.sun_counts[0], self.sun_counts[1], cx, cy, radius)
            self.canvas.create_oval(
                sun_sx - 11, sun_sy - 11, sun_sx + 11, sun_sy + 11,
                outline="#52525B", fill="#141418", width=1.5,
            )
            self.canvas.create_oval(
                sun_sx - 4, sun_sy - 4, sun_sx + 4, sun_sy + 4,
                outline="", fill="#FFFFFF",
            )
            self.canvas.create_text(
                sun_sx + 14, sun_sy - 7,
                anchor="w",
                text="LIGHT SOURCE",
                fill="#A1A1AA",
                font=(FONT_UI, 9, "bold"),
            )

        # Current Servo Target Reticle (Crisp White)
        tx, ty = self._counts_to_canvas(self.pulse_x, self.pulse_y, cx, cy, radius)
        self.canvas.create_line(tx, cy, tx, ty, fill="#3F3F46", dash=(2, 2))
        self.canvas.create_line(cx, ty, tx, ty, fill="#3F3F46", dash=(2, 2))

        self.canvas.create_oval(tx - 9, ty - 9, tx + 9, ty + 9, outline="#FFFFFF", width=1.8)
        self.canvas.create_oval(tx - 2.5, ty - 2.5, tx + 2.5, ty + 2.5, fill="#FFFFFF", outline="")
        self.canvas.create_line(tx - 15, ty, tx - 5, ty, fill="#FFFFFF", width=1.5)
        self.canvas.create_line(tx + 5, ty, tx + 15, ty, fill="#FFFFFF", width=1.5)
        self.canvas.create_line(tx, ty - 15, tx, ty - 5, fill="#FFFFFF", width=1.5)
        self.canvas.create_line(tx, ty + 5, tx, ty + 15, fill="#FFFFFF", width=1.5)

        # Top-Left Translucent-Style HUD Pill
        deg_x = counts_to_deg(self.pulse_x)
        deg_y = counts_to_deg(self.pulse_y)
        self.canvas.create_rectangle(
            10, 10, 290, 54,
            fill="#0C0C0E", outline="#26262B", width=1,
        )
        self.canvas.create_text(
            20,
            15,
            anchor="nw",
            text="2D COMPASS RADAR  ·  N / S / E / W",
            fill="#FFFFFF",
            font=(FONT_UI, 10, "bold"),
        )
        self.canvas.create_text(
            20,
            33,
            anchor="nw",
            text=f"X: {self.pulse_x:3d} ({deg_x:5.1f}°)   ·   Y: {self.pulse_y:3d} ({deg_y:5.1f}°)",
            fill="#A1A1AA",
            font=(FONT_MONO, 9),
        )
        if self.sun_counts is not None:
            self.canvas.create_text(
                w - 12,
                h - 10,
                anchor="se",
                text="Click/Drag to position virtual light",
                fill="#52525B",
                font=(FONT_UI, 9),
            )


class TimeSeriesChart(ctk.CTkFrame):
    """
    Monochrome real-time scrolling dual-trace oscilloscope chart:
    - X-Axis rendered in solid White (#FFFFFF)
    - Y-Axis rendered in medium Grey (#71717A)
    """

    WINDOW_SECONDS: float = 15.0

    def __init__(self, master: tk.Misc, **kwargs) -> None:
        super().__init__(
            master,
            fg_color="#0C0C0E",
            border_width=1,
            border_color="#26262B",
            corner_radius=10,
            **kwargs,
        )
        self.pack_propagate(False)

        self._history: Deque[Tuple[float, int, int]] = collections.deque(maxlen=300)
        self._unit_mode: str = "Pulse Width (µs)"
        self._last_size: Tuple[int, int] = (0, 0)

        # Header bar
        top_bar = ctk.CTkFrame(self, fg_color="transparent", height=28)
        top_bar.pack(fill=tk.X, padx=12, pady=(8, 4))
        top_bar.pack_propagate(False)

        ctk.CTkLabel(
            top_bar,
            text="REAL-TIME TELEMETRY WAVEFORM  ·  15s WINDOW",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            text_color="#FFFFFF",
        ).pack(side=tk.LEFT)

        self.unit_selector = ctk.CTkSegmentedButton(
            top_bar,
            values=["Pulse Width (µs)", "Angle (°)", "Counts (1-251)"],
            command=self._on_unit_change,
            height=24,
            fg_color="#141418",
            selected_color="#27272A",
            selected_hover_color="#3F3F46",
            unselected_color="#0C0C0E",
            unselected_hover_color="#18181C",
            text_color="#F4F4F5",
            font=ctk.CTkFont(family=FONT_UI, size=11),
        )
        self.unit_selector.set("Pulse Width (µs)")
        self.unit_selector.pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(
            self,
            bg="#000000",
            highlightthickness=1,
            highlightbackground="#1F1F24",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=(2, 10))
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

        pad_left = 60.0
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
                fill="#27272A" if is_center else "#141418",
                dash=() if is_center else (3, 3),
            )
            self.canvas.create_text(
                pad_left - 8,
                sy,
                anchor="e",
                text=fmt.format(val),
                fill="#71717A",
                font=(FONT_MONO, 8),
            )

        # Monochrome Legend
        self.canvas.create_text(
            pad_left + 10,
            pad_top + 2,
            anchor="nw",
            text="━━ X-AXIS (WHITE)",
            fill="#FFFFFF",
            font=(FONT_UI, 9, "bold"),
        )
        self.canvas.create_text(
            pad_left + 140,
            pad_top + 2,
            anchor="nw",
            text="━━ Y-AXIS (GREY)",
            fill="#71717A",
            font=(FONT_UI, 9, "bold"),
        )

        if len(self._history) < 2:
            self.canvas.create_text(
                pad_left + plot_w * 0.5,
                pad_top + plot_h * 0.5,
                text="Waiting for [0xFF, pulso_x, pulso_y] telemetry stream...",
                fill="#52525B",
                font=(FONT_UI, 10),
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

        if len(pts_y) >= 4:
            self.canvas.create_line(*pts_y, fill="#71717A", width=2, smooth=False)
        if len(pts_x) >= 4:
            self.canvas.create_line(*pts_x, fill="#FFFFFF", width=2, smooth=False)

        # Time axis ticks
        for sec_ago in (15, 10, 5, 0):
            tx = pad_left + ((self.WINDOW_SECONDS - sec_ago) / self.WINDOW_SECONDS) * plot_w
            self.canvas.create_text(
                tx,
                h - 8,
                text=f"-{sec_ago}s" if sec_ago > 0 else "NOW",
                fill="#52525B",
                font=(FONT_MONO, 8),
            )


class ProtocolInspector(ctk.CTkFrame):
    """
    Monochrome UART Protocol Inspector & Echo Monitor.
    Uses a lightweight tk.Canvas banner for the 10 Hz byte cards to avoid
    CustomTkinter label relayout overhead.
    """

    def __init__(self, master: tk.Misc, **kwargs) -> None:
        super().__init__(
            master,
            fg_color="#0C0C0E",
            border_width=1,
            border_color="#26262B",
            corner_radius=10,
            **kwargs,
        )
        self.pack_propagate(False)

        self._packet_timestamps: Deque[float] = collections.deque(maxlen=25)
        self._log_telemetry_var = tk.BooleanVar(value=False)
        self._last_echo_str: str = "—"
        self._last_b1_str: str = "0x7E (126) → 1500 µs"
        self._last_b2_str: str = "0x7E (126) → 1500 µs"
        self._last_rate_str: str = "RATE: 0.0 Hz   ·   FRAMES: 0   ·   ECHOES: 0"

        # Top header + 3-Byte Packet Breakdown Canvas
        self.banner_canvas = tk.Canvas(
            self,
            bg="#0C0C0E",
            height=68,
            highlightthickness=0,
        )
        self.banner_canvas.pack(fill=tk.X, padx=12, pady=(8, 2))
        self.banner_canvas.bind("<Configure>", lambda _e: self._draw_banner())

        # Log controls + Scrollable Textbox
        ctrl_row = ctk.CTkFrame(self, fg_color="transparent", height=24)
        ctrl_row.pack(fill=tk.X, padx=12, pady=(2, 2))
        ctrl_row.pack_propagate(False)

        ctk.CTkCheckBox(
            ctrl_row,
            text="Stream 10 Hz [0xFF, X, Y] packets to log",
            variable=self._log_telemetry_var,
            font=ctk.CTkFont(family=FONT_UI, size=11),
            text_color="#A1A1AA",
            fg_color="#FFFFFF",
            checkmark_color="#000000",
            hover_color="#D4D4D8",
            border_color="#3F3F46",
            checkbox_width=16,
            checkbox_height=16,
        ).pack(side=tk.LEFT)

        ctk.CTkButton(
            ctrl_row,
            text="Clear",
            width=64,
            height=22,
            fg_color="#141418",
            hover_color="#27272A",
            border_width=1,
            border_color="#26262B",
            text_color="#D4D4D8",
            font=ctk.CTkFont(family=FONT_UI, size=11),
            command=self.clear_log,
        ).pack(side=tk.RIGHT)

        self.log_box = ctk.CTkTextbox(
            self,
            height=78,
            fg_color="#000000",
            border_width=1,
            border_color="#1F1F24",
            text_color="#D4D4D8",
            font=ctk.CTkFont(family=FONT_MONO, size=11),
            activate_scrollbars=True,
        )
        self.log_box.pack(fill=tk.BOTH, expand=True, padx=12, pady=(2, 10))
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
            text="UART PROTOCOL INSPECTOR  ·  9600 8N1",
            fill="#FFFFFF",
            font=(FONT_UI, 11, "bold"),
        )
        self._rate_item = self.banner_canvas.create_text(
            w - 2,
            10,
            anchor="e",
            text=self._last_rate_str,
            fill="#A1A1AA",
            font=(FONT_MONO, 9, "bold"),
        )

        # 4 Monochrome Translucent-Style Byte Cards
        y0, y1 = 24, 64
        box_w = (w - 18) / 4.0
        specs = [
            ("BYTE 0 · SYNC HEADER", "0xFF (255)", "#FFFFFF", "b0"),
            ("BYTE 1 · PULSO_X", self._last_b1_str, "#F4F4F5", "b1"),
            ("BYTE 2 · PULSO_Y", self._last_b2_str, "#A1A1AA", "b2"),
            ("LAST PIC ECHO (RX)", self._last_echo_str, "#FFFFFF", "echo"),
        ]
        for idx, (title, val, col, tag) in enumerate(specs):
            x0 = idx * (box_w + 6)
            x1 = x0 + box_w
            cx = 0.5 * (x0 + x1)
            self.banner_canvas.create_rectangle(x0, y0, x1, y1, fill="#141418", outline="#26262B")
            self.banner_canvas.create_text(cx, y0 + 11, text=title, fill="#71717A", font=(FONT_UI, 9, "bold"))
            self.banner_canvas.create_text(
                cx, y0 + 28, text=val, fill=col, font=(FONT_MONO, 9, "bold"), tags=(tag,)
            )

    def on_telemetry(self, ev: TelemetryEvent, total_packets: int, total_echoes: int) -> None:
        self._packet_timestamps.append(ev.timestamp)
        rate = 0.0
        if len(self._packet_timestamps) >= 2:
            dt = self._packet_timestamps[-1] - self._packet_timestamps[0]
            if dt > 0.01:
                rate = (len(self._packet_timestamps) - 1) / dt

        self._last_rate_str = f"RATE: {rate:4.1f} Hz   ·   FRAMES: {total_packets}   ·   ECHOES: {total_echoes}"
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
        self._last_rate_str = f"FRAMES: {total_packets}   ·   ECHOES: {total_echoes}"
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
