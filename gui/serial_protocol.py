"""
Serial protocol engine for the PIC16F887 2-Axis Light Tracker.

Implements the exact binary telemetry and ASCII echo specification defined in
`firmware/src/Seguidor_de_Luz.asm` and `AGENTS.md`:
- UART configuration: 9600 baud, 8 data bits, no parity, 1 stop bit (8N1)
- PC -> PIC commands (1 byte ASCII): '1', '2', 'w', 'a', 's', 'd', 'c'
- PIC -> PC messages:
    1. 3-byte telemetry packet @ ~10 Hz: [0xFF, pulso_x, pulso_y]
       where pulso_x, pulso_y in [1..251]
    2. 1-byte immediate keyboard echo (< 128): '1', '2', 'w', 'a', 's', 'd', 'c'
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple, Union

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover
    serial = None  # type: ignore


# --- Protocol & Firmware Constants ---
BAUD_RATE: int = 9600
TELEMETRY_HEADER: int = 0xFF

PULSE_MIN: int = 1       # 500 us  -> 0.0 deg
PULSE_CENTER: int = 126  # 1500 us -> 90.0 deg
PULSE_MAX: int = 251     # 2500 us -> 180.0 deg

MANUAL_STEP_COUNTS: int = 4   # +/- 32 us (+/- 2.88 deg) per manual keypress
AUTO_STEP_COUNTS: int = 1     # +/- 8 us (+/- 0.72 deg) per 20 ms frame

VALID_COMMANDS: dict[str, str] = {
    "1": "Switch to Automatic Mode (LDR Tracking)",
    "2": "Switch to Manual Mode (Keyboard Control)",
    "w": "Tilt Up / North (+4 counts, +32 µs)",
    "s": "Tilt Down / South (-4 counts, -32 µs)",
    "d": "Pan Right / East (+4 counts, +32 µs)",
    "a": "Pan Left / West (-4 counts, -32 µs)",
    "c": "Center Both Axes (126 counts, 1500 µs, 90°)",
}


# --- Value Conversion Helpers ---
def clamp_counts(val: int) -> int:
    """Clamp a Timer1 setpoint to the valid [1, 251] firmware range."""
    return max(PULSE_MIN, min(PULSE_MAX, int(val)))


def counts_to_us(val: int) -> int:
    """
    Convert Timer1 setpoint counts [1..251] to pulse width in microseconds.
    Formula from Seguidor_de_Luz.asm:
        PWM_us = (61 + val) * 8 + 4 = 492 + 8 * val
        1 -> 500 us | 126 -> 1500 us | 251 -> 2500 us
    """
    return 492 + 8 * int(val)


def counts_to_deg(val: int) -> float:
    """
    Convert Timer1 setpoint counts [1..251] to physical servo angle [0.0..180.0] deg.
    Formula:
        theta = (val - 1) * 0.72
        1 -> 0.0 deg | 126 -> 90.0 deg | 251 -> 180.0 deg
    """
    return (int(val) - 1) * 0.72


def deg_to_counts(deg: float) -> int:
    """Convert servo angle [0.0..180.0] deg to nearest Timer1 count [1..251]."""
    return clamp_counts(round(float(deg) / 0.72) + 1)


def us_to_counts(us: float) -> int:
    """Convert pulse width [500..2500] us to nearest Timer1 count [1..251]."""
    return clamp_counts(round((float(us) - 492.0) / 8.0))


# --- Event Dataclasses ---
@dataclass(frozen=True)
class TelemetryEvent:
    """Represents a decoded 3-byte [0xFF, pulso_x, pulso_y] packet from the PIC."""
    timestamp: float
    pulse_x: int
    pulse_y: int
    raw_bytes: bytes

    @property
    def us_x(self) -> int:
        return counts_to_us(self.pulse_x)

    @property
    def us_y(self) -> int:
        return counts_to_us(self.pulse_y)

    @property
    def deg_x(self) -> float:
        return counts_to_deg(self.pulse_x)

    @property
    def deg_y(self) -> float:
        return counts_to_deg(self.pulse_y)

    @property
    def hex_str(self) -> str:
        return f"0xFF 0x{self.pulse_x:02X} 0x{self.pulse_y:02X}"

    @property
    def dec_str(self) -> str:
        return f"[255, {self.pulse_x:3d}, {self.pulse_y:3d}]"


@dataclass(frozen=True)
class EchoEvent:
    """Represents a 1-byte ASCII keyboard echo received back from the PIC."""
    timestamp: float
    char: str
    raw_byte: int

    @property
    def description(self) -> str:
        return VALID_COMMANDS.get(self.char, f"ASCII '{self.char}' (0x{self.raw_byte:02X})")


@dataclass(frozen=True)
class TxEvent:
    """Represents a 1-byte ASCII command transmitted from the PC to the PIC."""
    timestamp: float
    char: str
    raw_byte: int

    @property
    def description(self) -> str:
        return VALID_COMMANDS.get(self.char, f"ASCII '{self.char}' (0x{self.raw_byte:02X})")


@dataclass(frozen=True)
class StatusEvent:
    """Represents a serial connection status update or error."""
    timestamp: float
    connected: bool
    port: str
    message: str
    is_error: bool = False


SerialEvent = Union[TelemetryEvent, EchoEvent, TxEvent, StatusEvent]


# --- Stateful Stream Decoder ---
class SerialDecoder:
    """
    Byte-by-byte state machine that separates 3-byte binary telemetry packets
    [0xFF, pulso_x, pulso_y] from 1-byte ASCII keyboard echoes (< 128).
    """

    _WAIT_BYTE = 0
    _READ_X = 1
    _READ_Y = 2

    def __init__(self) -> None:
        self._state: int = self._WAIT_BYTE
        self._pending_x: int = PULSE_CENTER
        self.packets_decoded: int = 0
        self.echoes_decoded: int = 0
        self.framing_errors: int = 0

    def reset(self) -> None:
        """Reset parser state machine."""
        self._state = self._WAIT_BYTE
        self._pending_x = PULSE_CENTER

    def feed(self, data: bytes, timestamp: Optional[float] = None) -> List[Union[TelemetryEvent, EchoEvent]]:
        """Feed raw bytes from UART RX and return decoded events."""
        now = time.time() if timestamp is None else timestamp
        events: List[Union[TelemetryEvent, EchoEvent]] = []

        for b in data:
            if self._state == self._WAIT_BYTE:
                if b == TELEMETRY_HEADER:
                    self._state = self._READ_X
                elif 0 < b < 128:
                    self.echoes_decoded += 1
                    events.append(EchoEvent(timestamp=now, char=chr(b), raw_byte=b))
                else:
                    # Out-of-range byte when not in telemetry frame
                    self.framing_errors += 1

            elif self._state == self._READ_X:
                if b == TELEMETRY_HEADER:
                    # Re-synchronize on consecutive 0xFF
                    self.framing_errors += 1
                    self._state = self._READ_X
                elif PULSE_MIN <= b <= PULSE_MAX:
                    self._pending_x = b
                    self._state = self._READ_Y
                else:
                    self.framing_errors += 1
                    self._state = self._WAIT_BYTE

            elif self._state == self._READ_Y:
                if b == TELEMETRY_HEADER:
                    # Aborted frame; treat 0xFF as start of a new frame
                    self.framing_errors += 1
                    self._state = self._READ_X
                elif PULSE_MIN <= b <= PULSE_MAX:
                    self.packets_decoded += 1
                    raw = bytes([TELEMETRY_HEADER, self._pending_x, b])
                    events.append(
                        TelemetryEvent(
                            timestamp=now,
                            pulse_x=self._pending_x,
                            pulse_y=b,
                            raw_bytes=raw,
                        )
                    )
                    self._state = self._WAIT_BYTE
                else:
                    self.framing_errors += 1
                    self._state = self._WAIT_BYTE

        return events


# --- Thread-Safe Hardware Serial Manager ---
class SerialManager:
    """Manages non-blocking pyserial communication on a background daemon thread."""

    def __init__(self) -> None:
        self._ser: Optional["serial.Serial"] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._tx_queue: "queue.Queue[bytes]" = queue.Queue()
        self._event_queue: "queue.Queue[SerialEvent]" = queue.Queue()
        self._decoder = SerialDecoder()
        self._port_name: str = ""
        self._lock = threading.Lock()

    @staticmethod
    def list_ports() -> List[Tuple[str, str]]:
        """Return available COM ports as [(device, description), ...]."""
        if serial is None:
            return []
        ports = serial.tools.list_ports.comports()
        result: List[Tuple[str, str]] = []
        for p in sorted(ports, key=lambda item: item.device):
            desc = p.description or p.device
            result.append((p.device, f"{p.device} — {desc}"))
        return result

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._ser is not None and self._ser.is_open

    @property
    def port_name(self) -> str:
        return self._port_name

    @property
    def decoder(self) -> SerialDecoder:
        return self._decoder

    def connect(self, port: str, baudrate: int = BAUD_RATE) -> bool:
        """Open the specified COM port at 9600 8N1 and start the reader/writer thread."""
        self.disconnect()
        if serial is None:
            self._event_queue.put(
                StatusEvent(
                    timestamp=time.time(),
                    connected=False,
                    port=port,
                    message="pyserial is not installed.",
                    is_error=True,
                )
            )
            return False

        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02,
                write_timeout=0.2,
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception as exc:
            self._event_queue.put(
                StatusEvent(
                    timestamp=time.time(),
                    connected=False,
                    port=port,
                    message=f"Could not open {port}: {exc}",
                    is_error=True,
                )
            )
            return False

        with self._lock:
            self._ser = ser
            self._port_name = port
            self._decoder.reset()
            self._stop_event.clear()
            while not self._tx_queue.empty():
                try:
                    self._tx_queue.get_nowait()
                except queue.Empty:
                    break

        self._event_queue.put(
            StatusEvent(
                timestamp=time.time(),
                connected=True,
                port=port,
                message=f"Connected to {port} ({baudrate} 8N1)",
                is_error=False,
            )
        )

        self._thread = threading.Thread(target=self._io_loop, name="PICSerialIO", daemon=True)
        self._thread.start()
        return True

    def disconnect(self) -> None:
        """Stop the background serial thread and close the COM port."""
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self._thread = None

        closed_port = self._port_name
        with self._lock:
            if self._ser is not None:
                try:
                    if self._ser.is_open:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
            self._port_name = ""

        if closed_port:
            self._event_queue.put(
                StatusEvent(
                    timestamp=time.time(),
                    connected=False,
                    port=closed_port,
                    message=f"Disconnected from {closed_port}",
                    is_error=False,
                )
            )

    def send_command(self, char: str) -> bool:
        """Queue a 1-byte ASCII command to be transmitted to the PIC."""
        if not char:
            return False
        ch = char[0].lower()
        if not self.is_connected:
            return False
        raw = ch.encode("ascii", errors="ignore")
        if not raw:
            return False
        self._tx_queue.put(raw[:1])
        self._event_queue.put(TxEvent(timestamp=time.time(), char=ch, raw_byte=raw[0]))
        return True

    def poll_events(self) -> List[SerialEvent]:
        """Drain all pending serial events for the GUI thread."""
        events: List[SerialEvent] = []
        while True:
            try:
                events.append(self._event_queue.get_nowait())
            except queue.Empty:
                break
        return events

    def _io_loop(self) -> None:
        """Background thread loop performing non-blocking UART TX and RX."""
        while not self._stop_event.is_set():
            with self._lock:
                ser = self._ser
            if ser is None or not ser.is_open:
                break

            # 1. Drain outgoing TX commands
            while not self._tx_queue.empty():
                try:
                    payload = self._tx_queue.get_nowait()
                    ser.write(payload)
                    ser.flush()
                except queue.Empty:
                    break
                except Exception as exc:
                    self._event_queue.put(
                        StatusEvent(
                            timestamp=time.time(),
                            connected=False,
                            port=self._port_name,
                            message=f"UART TX error on {self._port_name}: {exc}",
                            is_error=True,
                        )
                    )
                    self._stop_event.set()
                    break

            if self._stop_event.is_set():
                break

            # 2. Read incoming RX bytes
            try:
                waiting = ser.in_waiting
                chunk = ser.read(waiting if waiting > 0 else 1)
                if chunk:
                    decoded = self._decoder.feed(chunk)
                    for ev in decoded:
                        self._event_queue.put(ev)
            except Exception as exc:
                self._event_queue.put(
                    StatusEvent(
                        timestamp=time.time(),
                        connected=False,
                        port=self._port_name,
                        message=f"UART RX error on {self._port_name}: {exc}",
                        is_error=True,
                    )
                )
                self._stop_event.set()
                break
