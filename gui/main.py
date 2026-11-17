"""
Main CustomTkinter GUI application for the PIC16F887 2-Axis Light Tracker.

Monochrome Instrumentation Design:
- Pure black (#000000) background with layered translucent-style dark grey panels (#0C0C0E / #121216)
- Crisp 1px borders (#26262B) and high-contrast white (#FFFFFF) active states
- Typography: Bahnschrift (DIN-1451 industrial UI) + Cascadia Mono (technical telemetry)
"""

from __future__ import annotations

import sys
import tkinter as tk
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import customtkinter as ctk

# Ensure local imports resolve regardless of working directory
GUI_DIR = Path(__file__).resolve().parent
if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))

from pic_simulator import PICSimulator
from serial_protocol import (
    BAUD_RATE,
    PULSE_CENTER,
    PULSE_MAX,
    PULSE_MIN,
    EchoEvent,
    SerialEvent,
    SerialManager,
    StatusEvent,
    TelemetryEvent,
    TxEvent,
)
from telemetry_widgets import FONT_MONO, FONT_UI, ProtocolInspector, Radar2D, TimeSeriesChart
from viewer_3d import Enclosure3DViewer


ctk.set_appearance_mode("dark")


class LightTrackerApp(ctk.CTk):
    """Main desktop window for the PIC16F887 2-Axis Light Tracker."""

    POLL_INTERVAL_MS: int = 30  # ~33 Hz UI event poll loop

    def __init__(self) -> None:
        super().__init__()

        self.title("PIC16F887 2-Axis Light Tracker — Serial Interpreter & Telemetry")
        self.geometry("1360x850")
        self.minsize(1120, 720)
        self.configure(fg_color="#000000")

        # Subtle desktop translucency for modern glass feel
        self._glass_var = tk.BooleanVar(value=True)
        try:
            self.attributes("-alpha", 0.96)
        except Exception:
            pass

        # Communication backends
        self.serial_mgr = SerialManager()
        self.simulator = PICSimulator()
        self.use_simulator: bool = False

        # Current state
        self.pulse_x: int = -1
        self.pulse_y: int = -1
        self.is_manual_mode: bool = False
        self._closing: bool = False
        self._poll_job: Optional[str] = None
        self._flash_job: Optional[str] = None
        self._port_map: Dict[str, str] = {}
        self._dpad_buttons: Dict[str, ctk.CTkButton] = {}

        self._build_ui()
        self._bind_keyboard()
        self.refresh_ports()

        # Start in Simulator Mode automatically so the interface works out-of-the-box
        self.sim_switch.select()
        self._on_toggle_simulator()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll_job = self.after(self.POLL_INTERVAL_MS, self._poll_loop)

    # ------------------------------------------------------------------
    # UI Construction
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        # 1. Top Connection & Mode Toolbar
        toolbar = ctk.CTkFrame(
            self,
            fg_color="#0C0C0E",
            border_width=1,
            border_color="#26262B",
            corner_radius=10,
            height=54,
        )
        toolbar.pack(fill=tk.X, padx=14, pady=(12, 8))
        toolbar.pack_propagate(False)

        title_lbl = ctk.CTkLabel(
            toolbar,
            text="2-AXIS LIGHT TRACKER  ·  PIC16F887",
            font=ctk.CTkFont(family=FONT_UI, size=14, weight="bold"),
            text_color="#FFFFFF",
        )
        title_lbl.pack(side=tk.LEFT, padx=(16, 18), pady=10)

        ctk.CTkLabel(
            toolbar,
            text="PORT",
            font=ctk.CTkFont(family=FONT_UI, size=11, weight="bold"),
            text_color="#71717A",
        ).pack(side=tk.LEFT, padx=(2, 6))

        self.port_combo = ctk.CTkComboBox(
            toolbar,
            values=["No COM ports found"],
            width=210,
            height=30,
            fg_color="#141418",
            border_width=1,
            border_color="#2A2A30",
            button_color="#1F1F24",
            button_hover_color="#2E2E36",
            dropdown_fg_color="#141418",
            dropdown_hover_color="#27272A",
            dropdown_text_color="#F4F4F5",
            text_color="#F4F4F5",
            font=ctk.CTkFont(family=FONT_MONO, size=11),
            dropdown_font=ctk.CTkFont(family=FONT_MONO, size=11),
            state="readonly",
        )
        self.port_combo.pack(side=tk.LEFT, padx=4)

        self.refresh_btn = ctk.CTkButton(
            toolbar,
            text="↻",
            width=34,
            height=30,
            fg_color="#141418",
            hover_color="#27272A",
            border_width=1,
            border_color="#2A2A30",
            text_color="#D4D4D8",
            font=ctk.CTkFont(family=FONT_UI, size=13, weight="bold"),
            command=self.refresh_ports,
        )
        self.refresh_btn.pack(side=tk.LEFT, padx=3)

        self.connect_btn = ctk.CTkButton(
            toolbar,
            text="Connect (9600 8N1)",
            width=146,
            height=30,
            fg_color="#FFFFFF",
            hover_color="#E4E4E7",
            text_color="#000000",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            command=self.toggle_serial_connection,
        )
        self.connect_btn.pack(side=tk.LEFT, padx=(4, 14))

        # Simulator controls
        self.sim_switch = ctk.CTkSwitch(
            toolbar,
            text="Simulator",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            text_color="#D4D4D8",
            fg_color="#27272A",
            progress_color="#FFFFFF",
            button_color="#71717A",
            button_hover_color="#A1A1AA",
            command=self._on_toggle_simulator,
        )
        self.sim_switch.pack(side=tk.LEFT, padx=8)

        self.orbit_chk_var = tk.BooleanVar(value=True)
        self.orbit_chk = ctk.CTkCheckBox(
            toolbar,
            text="Orbit Light",
            variable=self.orbit_chk_var,
            font=ctk.CTkFont(family=FONT_UI, size=12),
            text_color="#A1A1AA",
            fg_color="#FFFFFF",
            checkmark_color="#000000",
            hover_color="#D4D4D8",
            border_color="#3F3F46",
            checkbox_width=16,
            checkbox_height=16,
            command=self._on_toggle_orbit,
        )
        self.orbit_chk.pack(side=tk.LEFT, padx=8)

        self.glass_chk = ctk.CTkCheckBox(
            toolbar,
            text="Glass Alpha",
            variable=self._glass_var,
            font=ctk.CTkFont(family=FONT_UI, size=12),
            text_color="#A1A1AA",
            fg_color="#FFFFFF",
            checkmark_color="#000000",
            hover_color="#D4D4D8",
            border_color="#3F3F46",
            checkbox_width=16,
            checkbox_height=16,
            command=self._on_toggle_glass,
        )
        self.glass_chk.pack(side=tk.LEFT, padx=8)

        self.status_badge = ctk.CTkLabel(
            toolbar,
            text="○ DISCONNECTED",
            font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
            text_color="#71717A",
        )
        self.status_badge.pack(side=tk.RIGHT, padx=16)

        # 2. Main Body Container (Left Sidebar + Right Visualization Workspace)
        body = ctk.CTkFrame(self, fg_color="transparent")
        body.pack(fill=tk.BOTH, expand=True, padx=14, pady=(0, 12))

        self._build_sidebar(body)
        self._build_workspace(body)

    def _build_sidebar(self, parent: ctk.CTkFrame) -> None:
        sidebar = ctk.CTkFrame(
            parent,
            fg_color="#0C0C0E",
            border_width=1,
            border_color="#26262B",
            corner_radius=10,
            width=320,
        )
        sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        sidebar.pack_propagate(False)

        # --- Operating Mode Section ---
        mode_card = ctk.CTkFrame(
            sidebar,
            fg_color="#121216",
            border_width=1,
            border_color="#222228",
            corner_radius=8,
        )
        mode_card.pack(fill=tk.X, padx=12, pady=(12, 8))

        ctk.CTkLabel(
            mode_card,
            text="OPERATING MODE",
            font=ctk.CTkFont(family=FONT_UI, size=11, weight="bold"),
            text_color="#71717A",
        ).pack(anchor="w", padx=12, pady=(8, 2))

        self.mode_badge = ctk.CTkLabel(
            mode_card,
            text="AUTOMATIC  ·  LDR TRACKING",
            font=ctk.CTkFont(family=FONT_UI, size=13, weight="bold"),
            text_color="#FFFFFF",
        )
        self.mode_badge.pack(anchor="w", padx=12, pady=(0, 8))

        btn_row = ctk.CTkFrame(mode_card, fg_color="transparent")
        btn_row.pack(fill=tk.X, padx=10, pady=(0, 10))

        self.btn_auto = ctk.CTkButton(
            btn_row,
            text="[1] Auto Mode",
            height=34,
            fg_color="#FFFFFF",
            hover_color="#E4E4E7",
            text_color="#000000",
            border_width=1,
            border_color="#FFFFFF",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            command=lambda: self.send_pic_command("1"),
        )
        self.btn_auto.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self._dpad_buttons["1"] = self.btn_auto

        self.btn_manual = ctk.CTkButton(
            btn_row,
            text="[2] Manual Mode",
            height=34,
            fg_color="#18181D",
            hover_color="#27272A",
            text_color="#A1A1AA",
            border_width=1,
            border_color="#2A2A30",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            command=lambda: self.send_pic_command("2"),
        )
        self.btn_manual.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))
        self._dpad_buttons["2"] = self.btn_manual

        # --- Manual D-Pad Control Section ---
        dpad_card = ctk.CTkFrame(
            sidebar,
            fg_color="#121216",
            border_width=1,
            border_color="#222228",
            corner_radius=8,
        )
        dpad_card.pack(fill=tk.X, padx=12, pady=6)

        ctk.CTkLabel(
            dpad_card,
            text="MANUAL POSITIONING  ·  W / A / S / D / C",
            font=ctk.CTkFont(family=FONT_UI, size=11, weight="bold"),
            text_color="#71717A",
        ).pack(anchor="w", padx=12, pady=(8, 4))

        grid = ctk.CTkFrame(dpad_card, fg_color="transparent")
        grid.pack(padx=12, pady=(2, 8))
        for col in range(3):
            grid.grid_columnconfigure(col, weight=1, minsize=84)

        btn_w = ctk.CTkButton(
            grid,
            text="▲ W\nNorth (+Y)",
            width=84,
            height=46,
            fg_color="#18181D",
            hover_color="#27272A",
            border_width=1,
            border_color="#2E2E36",
            text_color="#F4F4F5",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            command=lambda: self.send_pic_command("w"),
        )
        btn_w.grid(row=0, column=1, padx=4, pady=4)
        self._dpad_buttons["w"] = btn_w

        btn_a = ctk.CTkButton(
            grid,
            text="◀ A\nWest (-X)",
            width=84,
            height=46,
            fg_color="#18181D",
            hover_color="#27272A",
            border_width=1,
            border_color="#2E2E36",
            text_color="#F4F4F5",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            command=lambda: self.send_pic_command("a"),
        )
        btn_a.grid(row=1, column=0, padx=4, pady=4)
        self._dpad_buttons["a"] = btn_a

        btn_c = ctk.CTkButton(
            grid,
            text="● C\nCenter 90°",
            width=84,
            height=46,
            fg_color="#24242C",
            hover_color="#3F3F46",
            border_width=1,
            border_color="#52525B",
            text_color="#FFFFFF",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            command=lambda: self.send_pic_command("c"),
        )
        btn_c.grid(row=1, column=1, padx=4, pady=4)
        self._dpad_buttons["c"] = btn_c

        btn_d = ctk.CTkButton(
            grid,
            text="D ▶\nEast (+X)",
            width=84,
            height=46,
            fg_color="#18181D",
            hover_color="#27272A",
            border_width=1,
            border_color="#2E2E36",
            text_color="#F4F4F5",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            command=lambda: self.send_pic_command("d"),
        )
        btn_d.grid(row=1, column=2, padx=4, pady=4)
        self._dpad_buttons["d"] = btn_d

        btn_s = ctk.CTkButton(
            grid,
            text="▼ S\nSouth (-Y)",
            width=84,
            height=46,
            fg_color="#18181D",
            hover_color="#27272A",
            border_width=1,
            border_color="#2E2E36",
            text_color="#F4F4F5",
            font=ctk.CTkFont(family=FONT_UI, size=12, weight="bold"),
            command=lambda: self.send_pic_command("s"),
        )
        btn_s.grid(row=2, column=1, padx=4, pady=4)
        self._dpad_buttons["s"] = btn_s

        ctk.CTkLabel(
            dpad_card,
            text="Step: ±4 counts (±32 µs / ±2.88°)",
            font=ctk.CTkFont(family=FONT_MONO, size=10),
            text_color="#52525B",
        ).pack(pady=(0, 8))

        # --- X-Axis Telemetry Card ---
        x_card = ctk.CTkFrame(
            sidebar,
            fg_color="#121216",
            border_width=1,
            border_color="#222228",
            corner_radius=8,
        )
        x_card.pack(fill=tk.X, padx=12, pady=6)

        ctk.CTkLabel(
            x_card,
            text="X-AXIS SERVO  ·  RD0 (WEST ↔ EAST)",
            font=ctk.CTkFont(family=FONT_UI, size=11, weight="bold"),
            text_color="#A1A1AA",
        ).pack(anchor="w", padx=12, pady=(8, 2))

        self.x_deg_lbl = ctk.CTkLabel(
            x_card,
            text="90.0°  |  1500 µs",
            font=ctk.CTkFont(family=FONT_MONO, size=17, weight="bold"),
            text_color="#FFFFFF",
        )
        self.x_deg_lbl.pack(anchor="w", padx=12, pady=(0, 2))

        self.x_cnt_lbl = ctk.CTkLabel(
            x_card,
            text="Timer1 Setpoint: 126 / 251 (0x7E)",
            font=ctk.CTkFont(family=FONT_MONO, size=10),
            text_color="#71717A",
        )
        self.x_cnt_lbl.pack(anchor="w", padx=12, pady=(0, 6))

        self.x_bar = ctk.CTkProgressBar(
            x_card,
            fg_color="#1F1F26",
            progress_color="#FFFFFF",
            height=6,
        )
        self.x_bar.pack(fill=tk.X, padx=12, pady=(0, 10))
        self.x_bar.set(0.5)

        # --- Y-Axis Telemetry Card ---
        y_card = ctk.CTkFrame(
            sidebar,
            fg_color="#121216",
            border_width=1,
            border_color="#222228",
            corner_radius=8,
        )
        y_card.pack(fill=tk.X, padx=12, pady=6)

        ctk.CTkLabel(
            y_card,
            text="Y-AXIS SERVO  ·  RD1 (SOUTH ↔ NORTH)",
            font=ctk.CTkFont(family=FONT_UI, size=11, weight="bold"),
            text_color="#71717A",
        ).pack(anchor="w", padx=12, pady=(8, 2))

        self.y_deg_lbl = ctk.CTkLabel(
            y_card,
            text="90.0°  |  1500 µs",
            font=ctk.CTkFont(family=FONT_MONO, size=17, weight="bold"),
            text_color="#D4D4D8",
        )
        self.y_deg_lbl.pack(anchor="w", padx=12, pady=(0, 2))

        self.y_cnt_lbl = ctk.CTkLabel(
            y_card,
            text="Timer1 Setpoint: 126 / 251 (0x7E)",
            font=ctk.CTkFont(family=FONT_MONO, size=10),
            text_color="#71717A",
        )
        self.y_cnt_lbl.pack(anchor="w", padx=12, pady=(0, 6))

        self.y_bar = ctk.CTkProgressBar(
            y_card,
            fg_color="#1F1F26",
            progress_color="#71717A",
            height=6,
        )
        self.y_bar.pack(fill=tk.X, padx=12, pady=(0, 10))
        self.y_bar.set(0.5)

    def _build_workspace(self, parent: ctk.CTkFrame) -> None:
        workspace = ctk.CTkFrame(parent, fg_color="transparent")
        workspace.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Top row: 3D STL Viewer (left) + 2D Pan/Tilt Radar (right)
        top_pane = ctk.CTkFrame(workspace, fg_color="transparent")
        top_pane.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        top_pane.grid_columnconfigure(0, weight=6)
        top_pane.grid_columnconfigure(1, weight=5)
        top_pane.grid_rowconfigure(0, weight=1)

        self.viewer_3d = Enclosure3DViewer(top_pane)
        self.viewer_3d.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        self.radar_2d = Radar2D(top_pane, on_sun_move=self._on_radar_sun_drag)
        self.radar_2d.grid(row=0, column=1, sticky="nsew", padx=(4, 0))

        # Middle row: Real-time scrolling chart
        self.chart = TimeSeriesChart(workspace, height=185)
        self.chart.pack(fill=tk.X, pady=(0, 8))

        # Bottom row: UART Protocol Inspector & Echo Monitor
        self.inspector = ProtocolInspector(workspace, height=195)
        self.inspector.pack(fill=tk.X)

    # ------------------------------------------------------------------
    # Keyboard & Control Logic
    # ------------------------------------------------------------------
    def _bind_keyboard(self) -> None:
        self.bind_all("<KeyPress>", self._on_key_press)

    def _on_key_press(self, event: tk.Event) -> None:
        if isinstance(event.widget, (tk.Entry, tk.Text)):
            return

        keysym = event.keysym.lower()
        key_map = {
            "1": "1",
            "2": "2",
            "w": "w",
            "a": "a",
            "s": "s",
            "d": "d",
            "c": "c",
            "up": "w",
            "down": "s",
            "left": "a",
            "right": "d",
            "space": "c",
        }
        cmd = key_map.get(keysym)
        if cmd is not None:
            self.send_pic_command(cmd)

    def _flash_button(self, cmd: str) -> None:
        btn = self._dpad_buttons.get(cmd)
        if btn is None:
            return
        if self._flash_job is not None:
            try:
                self.after_cancel(self._flash_job)
            except Exception:
                pass
            self._flash_job = None
        btn.configure(fg_color="#FFFFFF", text_color="#000000")
        self._flash_job = self.after(100, self._restore_button_colors)

    def _restore_button_colors(self) -> None:
        self._flash_job = None
        if self._closing:
            return
        if self.is_manual_mode:
            self.btn_auto.configure(
                fg_color="#18181D", text_color="#A1A1AA", border_color="#2A2A30"
            )
            self.btn_manual.configure(
                fg_color="#FFFFFF", text_color="#000000", border_color="#FFFFFF"
            )
        else:
            self.btn_auto.configure(
                fg_color="#FFFFFF", text_color="#000000", border_color="#FFFFFF"
            )
            self.btn_manual.configure(
                fg_color="#18181D", text_color="#A1A1AA", border_color="#2A2A30"
            )

        for k in ("w", "a", "s", "d"):
            self._dpad_buttons[k].configure(fg_color="#18181D", text_color="#F4F4F5")
        self._dpad_buttons["c"].configure(fg_color="#24242C", text_color="#FFFFFF")

    def send_pic_command(self, cmd: str) -> None:
        """Send a 1-byte command ('1', '2', 'w', 'a', 's', 'd', 'c') to the active backend."""
        ch = cmd[0].lower()
        self._flash_button(ch)

        if ch == "1":
            self._set_mode_ui(manual=False)
        elif ch == "2":
            self._set_mode_ui(manual=True)

        if self.use_simulator:
            self.simulator.send_command(ch)
        elif self.serial_mgr.is_connected:
            self.serial_mgr.send_command(ch)
        else:
            self.inspector.append_log(
                f"WARN: Cannot send '{ch}' — Not connected to a COM port or Simulator."
            )

    def _set_mode_ui(self, manual: bool) -> None:
        if self.is_manual_mode == manual:
            return
        self.is_manual_mode = manual
        if manual:
            self.mode_badge.configure(
                text="MANUAL  ·  KEYBOARD CONTROL", text_color="#E4E4E7"
            )
        else:
            self.mode_badge.configure(
                text="AUTOMATIC  ·  LDR TRACKING", text_color="#FFFFFF"
            )
        self._restore_button_colors()

    # ------------------------------------------------------------------
    # Connection & Simulator Handlers
    # ------------------------------------------------------------------
    def _on_toggle_glass(self) -> None:
        try:
            self.attributes("-alpha", 0.95 if self._glass_var.get() else 1.0)
        except Exception:
            pass

    def refresh_ports(self) -> None:
        ports = self.serial_mgr.list_ports()
        self._port_map.clear()
        if not ports:
            self.port_combo.configure(values=["No COM ports found"])
            self.port_combo.set("No COM ports found")
        else:
            labels = []
            for dev, label in ports:
                self._port_map[label] = dev
                labels.append(label)
            self.port_combo.configure(values=labels)
            self.port_combo.set(labels[0])

    def toggle_serial_connection(self) -> None:
        if self.serial_mgr.is_connected:
            self.serial_mgr.disconnect()
            self.connect_btn.configure(
                text="Connect (9600 8N1)",
                fg_color="#FFFFFF",
                hover_color="#E4E4E7",
                text_color="#000000",
            )
            return

        selected_label = self.port_combo.get()
        device = self._port_map.get(selected_label, selected_label.split()[0] if selected_label else "")
        if not device or device == "No":
            self.inspector.append_log("ERROR: No valid COM port selected.")
            return

        if self.use_simulator:
            self.sim_switch.deselect()
            self._on_toggle_simulator()

        if self.serial_mgr.connect(device, BAUD_RATE):
            self.connect_btn.configure(
                text=f"Disconnect ({device})",
                fg_color="#1F1F24",
                hover_color="#2E2E36",
                text_color="#FFFFFF",
            )

    def _on_toggle_simulator(self) -> None:
        enabled = bool(self.sim_switch.get())
        self.use_simulator = enabled
        if enabled:
            if self.serial_mgr.is_connected:
                self.serial_mgr.disconnect()
                self.connect_btn.configure(
                    text="Connect (9600 8N1)",
                    fg_color="#FFFFFF",
                    hover_color="#E4E4E7",
                    text_color="#000000",
                )
            self.simulator.orbit_sun = bool(self.orbit_chk_var.get())
            self.simulator.start()
        else:
            self.simulator.stop()
            px = PULSE_CENTER if self.pulse_x < 0 else self.pulse_x
            py = PULSE_CENTER if self.pulse_y < 0 else self.pulse_y
            self.radar_2d.update_state(px, py, sun_counts=None)
            self.viewer_3d.set_pose(px, py, sun_counts=None)

    def _on_toggle_orbit(self) -> None:
        self.simulator.orbit_sun = bool(self.orbit_chk_var.get())

    def _on_radar_sun_drag(self, sun_x: float, sun_y: float) -> None:
        if not self.use_simulator:
            return
        self.orbit_chk_var.set(False)
        self.simulator.set_sun_position(sun_x, sun_y, disable_orbit=True)
        px = PULSE_CENTER if self.pulse_x < 0 else self.pulse_x
        py = PULSE_CENTER if self.pulse_y < 0 else self.pulse_y
        self.radar_2d.update_state(px, py, sun_counts=(sun_x, sun_y))

    # ------------------------------------------------------------------
    # Event Polling & Telemetry Updates
    # ------------------------------------------------------------------
    def _poll_loop(self) -> None:
        if self._closing:
            return
        try:
            events: List[SerialEvent] = []
            events.extend(self.serial_mgr.poll_events())
            events.extend(self.simulator.poll_events())

            active_decoder = self.simulator.decoder if self.use_simulator else self.serial_mgr.decoder
            latest_telemetry: Optional[TelemetryEvent] = None

            for ev in events:
                if isinstance(ev, StatusEvent):
                    self._handle_status_event(ev)
                elif isinstance(ev, TxEvent):
                    self.inspector.on_tx(ev)
                elif isinstance(ev, EchoEvent):
                    if ev.char == "1":
                        self._set_mode_ui(manual=False)
                    elif ev.char == "2":
                        self._set_mode_ui(manual=True)
                    self.inspector.on_echo(
                        ev,
                        total_packets=active_decoder.packets_decoded,
                        total_echoes=active_decoder.echoes_decoded,
                    )
                elif isinstance(ev, TelemetryEvent):
                    self.chart.record_sample(ev.timestamp, ev.pulse_x, ev.pulse_y)
                    self.inspector.on_telemetry(
                        ev,
                        total_packets=active_decoder.packets_decoded,
                        total_echoes=active_decoder.echoes_decoded,
                    )
                    latest_telemetry = ev

            if latest_telemetry is not None:
                self._handle_telemetry_event(latest_telemetry)
        finally:
            if not self._closing:
                self._poll_job = self.after(self.POLL_INTERVAL_MS, self._poll_loop)

    def _handle_status_event(self, ev: StatusEvent) -> None:
        self.inspector.append_log(ev.message)
        if ev.connected:
            self.status_badge.configure(
                text=f"● CONNECTED · {ev.port}",
                text_color="#FFFFFF",
            )
        else:
            self.status_badge.configure(
                text="○ DISCONNECTED",
                text_color="#71717A",
            )
            if not self.use_simulator:
                self.connect_btn.configure(
                    text="Connect (9600 8N1)",
                    fg_color="#FFFFFF",
                    hover_color="#E4E4E7",
                    text_color="#000000",
                )

    def _handle_telemetry_event(self, ev: TelemetryEvent) -> None:
        if ev.pulse_x != self.pulse_x:
            self.pulse_x = ev.pulse_x
            self.x_deg_lbl.configure(text=f"{ev.deg_x:5.1f}°  |  {ev.us_x:4d} µs")
            self.x_cnt_lbl.configure(
                text=f"Timer1 Setpoint: {ev.pulse_x:3d} / 251 (0x{ev.pulse_x:02X})"
            )
            self.x_bar.set((ev.pulse_x - PULSE_MIN) / float(PULSE_MAX - PULSE_MIN))

        if ev.pulse_y != self.pulse_y:
            self.pulse_y = ev.pulse_y
            self.y_deg_lbl.configure(text=f"{ev.deg_y:5.1f}°  |  {ev.us_y:4d} µs")
            self.y_cnt_lbl.configure(
                text=f"Timer1 Setpoint: {ev.pulse_y:3d} / 251 (0x{ev.pulse_y:02X})"
            )
            self.y_bar.set((ev.pulse_y - PULSE_MIN) / float(PULSE_MAX - PULSE_MIN))

        sun_pos: Optional[Tuple[float, float]] = (
            self.simulator.get_sun_position() if self.use_simulator else None
        )
        self.viewer_3d.set_pose(ev.pulse_x, ev.pulse_y, sun_counts=sun_pos)
        self.radar_2d.update_state(ev.pulse_x, ev.pulse_y, sun_counts=sun_pos)
        self.chart.render()

    def _on_close(self) -> None:
        self._closing = True
        if self._poll_job is not None:
            try:
                self.after_cancel(self._poll_job)
            except Exception:
                pass
            self._poll_job = None
        if self._flash_job is not None:
            try:
                self.after_cancel(self._flash_job)
            except Exception:
                pass
            self._flash_job = None
        self.simulator.stop()
        self.serial_mgr.disconnect()
        self.destroy()


def main() -> None:
    app = LightTrackerApp()
    app.mainloop()


if __name__ == "__main__":
    main()
