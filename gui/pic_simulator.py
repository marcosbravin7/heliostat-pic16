"""
Virtual PIC16F887 simulator for offline testing and demonstration of the GUI.

Replicates the exact timing and state machine of `firmware/src/Seguidor_de_Luz.asm`:
- 20 ms Timer0 frame period (50 Hz)
- Automatic mode ('1'): steps pulse_x / pulse_y by +/-1 count per 20 ms frame
  toward a virtual light source with a +/-2 count deadband (ZONA_MUERTA = .2)
- Manual mode ('2'): adjusts pulse_x / pulse_y by +/-4 counts on 'a'/'d'/'w'/'s'
  or centers both to 126 on 'c'
- Immediate 1-byte UART echo on every received byte
- Periodic 3-byte telemetry packet [0xFF, pulso_x, pulso_y] every 5 frames (~100 ms / 10 Hz)
"""

from __future__ import annotations

import math
import queue
import threading
import time
from typing import List, Optional, Tuple

from serial_protocol import (
    AUTO_STEP_COUNTS,
    MANUAL_STEP_COUNTS,
    PULSE_CENTER,
    PULSE_MAX,
    PULSE_MIN,
    TELEMETRY_HEADER,
    SerialDecoder,
    SerialEvent,
    StatusEvent,
    TxEvent,
    clamp_counts,
)


class PICSimulator:
    """Thread-safe virtual PIC16F887 that feeds raw UART bytes into SerialDecoder."""

    FRAME_PERIOD_S: float = 0.020  # 19.968 ms Timer0 period
    TELEMETRY_DIVIDER: int = 5     # Every 5 frames (~100 ms / 10 Hz)
    DEADBAND_COUNTS: int = 2       # ZONA_MUERTA = .2

    def __init__(self) -> None:
        self.pulse_x: int = PULSE_CENTER
        self.pulse_y: int = PULSE_CENTER
        self.manual_mode: bool = False

        # Virtual sun target in Timer1 counts [1..251]
        self.sun_x: float = 175.0
        self.sun_y: float = 160.0
        self.orbit_sun: bool = True

        self._tx_divider: int = self.TELEMETRY_DIVIDER
        self._rx_from_pc: "queue.Queue[bytes]" = queue.Queue()
        self._event_queue: "queue.Queue[SerialEvent]" = queue.Queue()
        self._decoder = SerialDecoder()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._running: bool = False
        self._start_time: float = time.time()

    @property
    def is_connected(self) -> bool:
        return self._running

    @property
    def port_name(self) -> str:
        return "SIM-PIC16F887"

    @property
    def decoder(self) -> SerialDecoder:
        return self._decoder

    def start(self) -> None:
        """Start the virtual PIC16F887 50 Hz loop."""
        self.stop()
        self._decoder.reset()
        self._stop_event.clear()
        self._running = True
        self._start_time = time.time()
        self._tx_divider = 1  # Emit first telemetry frame immediately

        self._event_queue.put(
            StatusEvent(
                timestamp=time.time(),
                connected=True,
                port=self.port_name,
                message="Connected to Virtual PIC16F887 Simulator (9600 8N1 emulated)",
                is_error=False,
            )
        )

        self._thread = threading.Thread(target=self._sim_loop, name="PICSimulator", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the virtual PIC16F887 loop."""
        was_running = self._running
        self._running = False
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self._thread = None

        if was_running:
            self._event_queue.put(
                StatusEvent(
                    timestamp=time.time(),
                    connected=False,
                    port=self.port_name,
                    message="Stopped Virtual PIC16F887 Simulator",
                    is_error=False,
                )
            )

    def set_sun_position(self, sun_x: float, sun_y: float, disable_orbit: bool = True) -> None:
        """Set virtual light source position in Timer1 counts [1..251]."""
        self.sun_x = float(max(PULSE_MIN, min(PULSE_MAX, round(sun_x))))
        self.sun_y = float(max(PULSE_MIN, min(PULSE_MAX, round(sun_y))))
        if disable_orbit:
            self.orbit_sun = False

    def get_sun_position(self) -> Tuple[float, float]:
        """Return current (sun_x, sun_y) in Timer1 counts [1..251]."""
        return self.sun_x, self.sun_y

    def send_command(self, char: str) -> bool:
        """Send a 1-byte ASCII command from the PC to the simulated PIC."""
        if not char or not self._running:
            return False
        ch = char[0].lower()
        raw = ch.encode("ascii", errors="ignore")
        if not raw:
            return False
        self._rx_from_pc.put(raw[:1])
        self._event_queue.put(TxEvent(timestamp=time.time(), char=ch, raw_byte=raw[0]))
        return True

    def poll_events(self) -> List[SerialEvent]:
        """Drain decoded serial events for the GUI thread."""
        events: List[SerialEvent] = []
        while True:
            try:
                events.append(self._event_queue.get_nowait())
            except queue.Empty:
                break
        return events

    def _sim_loop(self) -> None:
        """Emulate Timer0 20 ms ISR + main loop UART telemetry and command reception."""
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(max(0.001, next_tick - now))
                continue
            next_tick += self.FRAME_PERIOD_S

            # 1. Timer0 ISR: Automatic tracking (ajustar_por_luz) when MODO_MANUAL == 0
            if not self.manual_mode:
                if self.orbit_sun:
                    elapsed = time.time() - self._start_time
                    # Smooth Lissajous / solar path across the sky
                    self.sun_x = PULSE_CENTER + 85.0 * math.sin(elapsed * 0.45)
                    self.sun_y = PULSE_CENTER + 65.0 * math.cos(elapsed * 0.30)

                err_x = self.sun_x - self.pulse_x
                if abs(err_x) > self.DEADBAND_COUNTS:
                    step_x = AUTO_STEP_COUNTS if err_x > 0 else -AUTO_STEP_COUNTS
                    self.pulse_x = clamp_counts(self.pulse_x + step_x)

                err_y = self.sun_y - self.pulse_y
                if abs(err_y) > self.DEADBAND_COUNTS:
                    step_y = AUTO_STEP_COUNTS if err_y > 0 else -AUTO_STEP_COUNTS
                    self.pulse_y = clamp_counts(self.pulse_y + step_y)

            # 2. Main loop after emitir_pwm: Telemetry every 5 frames (~100 ms)
            self._tx_divider -= 1
            if self._tx_divider <= 0:
                self._tx_divider = self.TELEMETRY_DIVIDER
                packet = bytes([TELEMETRY_HEADER, self.pulse_x, self.pulse_y])
                for ev in self._decoder.feed(packet):
                    self._event_queue.put(ev)

            # 3. Main loop: recibir_uart (immediate echo + command execution)
            while not self._rx_from_pc.empty():
                try:
                    byte_in = self._rx_from_pc.get_nowait()
                except queue.Empty:
                    break
                if not byte_in:
                    continue

                # Immediate 1-byte keyboard echo back to PC (CALL enviar_uart)
                for ev in self._decoder.feed(byte_in):
                    self._event_queue.put(ev)

                ch = chr(byte_in[0])
                if ch == "1":
                    self.manual_mode = False
                elif ch == "2":
                    self.manual_mode = True
                elif self.manual_mode:
                    if ch == "a":
                        self.pulse_x = clamp_counts(self.pulse_x - MANUAL_STEP_COUNTS)
                    elif ch == "d":
                        self.pulse_x = clamp_counts(self.pulse_x + MANUAL_STEP_COUNTS)
                    elif ch == "w":
                        self.pulse_y = clamp_counts(self.pulse_y + MANUAL_STEP_COUNTS)
                    elif ch == "s":
                        self.pulse_y = clamp_counts(self.pulse_y - MANUAL_STEP_COUNTS)
                    elif ch == "c":
                        self.pulse_x = PULSE_CENTER
                        self.pulse_y = PULSE_CENTER
