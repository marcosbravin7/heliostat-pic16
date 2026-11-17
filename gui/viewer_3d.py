"""
Interactive 3D STL kinematic viewer for the 2-Axis Light Tracker enclosure.

Monochrome Edition:
- Pure black (#000000) viewport with subtle #16161A perspective floor grid
- Brushed silver-grey U-bracket turret (`Base caja y brazos.stl`)
- Matte titanium-white 4-LDR sensor head (`Caja fotorresistencias.stl`)
- Floating translucent-style dark grey HUD pills with 1px #26262B borders
"""

from __future__ import annotations

import math
import struct
import tkinter as tk
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from serial_protocol import PULSE_CENTER, counts_to_deg, counts_to_us


def load_binary_stl(path: Path, min_area: float = 0.0) -> np.ndarray:
    """
    Load a binary STL file and return an (N, 3, 3) float32 array of triangle vertices.
    Filters out microscopic screw-hole sliver facets (< min_area mm^2)
    to keep real-time canvas rendering fast and responsive.
    """
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 84:
        raise ValueError(f"STL file too short: {path}")
    n_tris = struct.unpack("<I", data[80:84])[0]
    expected_bytes = 84 + n_tris * 50
    if len(data) >= expected_bytes:
        dt = np.dtype([("normal", "<f4", (3,)), ("verts", "<f4", (3, 3)), ("attr", "<u2")])
        records = np.frombuffer(data, dtype=dt, offset=84, count=n_tris)
        verts = records["verts"].astype(np.float32, copy=True)
    else:
        raw_verts: List[List[float]] = []
        for line in data.decode("latin1", errors="ignore").splitlines():
            parts = line.strip().split()
            if len(parts) == 4 and parts[0].lower() == "vertex":
                raw_verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        verts = np.array(raw_verts, dtype=np.float32).reshape(-1, 3, 3)

    if min_area > 0.0 and len(verts) > 0:
        areas = 0.5 * np.linalg.norm(
            np.cross(verts[:, 1] - verts[:, 0], verts[:, 2] - verts[:, 0]),
            axis=1,
        )
        verts = verts[areas >= min_area]
    return verts


def _rotation_x(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float32)


def _rotation_z(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)


class Enclosure3DViewer(tk.Frame):
    """
    Real-time monochrome 3D viewport rendering the articulated STL assembly:
    - U-Bracket Turret (`Base caja y brazos.stl`) -> Yaw (pulso_x)
    - LDR Sensor Box (`Caja fotorresistencias.stl`) -> Pitch (pulso_y) + Yaw (pulso_x)
    """

    def __init__(self, master: tk.Misc, enclosure_dir: Optional[Path] = None, **kwargs) -> None:
        super().__init__(master, bg="#000000", **kwargs)

        if enclosure_dir is None:
            enclosure_dir = Path(__file__).resolve().parent.parent / "enclosure"
        self.enclosure_dir = enclosure_dir

        # Current servo state (Timer1 counts [1..251])
        self.pulse_x: int = PULSE_CENTER
        self.pulse_y: int = PULSE_CENTER

        # Camera orbit state (+Z world is always UP)
        self.cam_azimuth_deg: float = -35.0
        self.cam_elevation_deg: float = 28.0
        self.cam_zoom: float = 3.2
        self._drag_last: Optional[Tuple[int, int]] = None
        self._dirty: bool = True
        self._render_scheduled: bool = False
        self._last_size: Tuple[int, int] = (0, 0)

        # Load and center STL meshes in monochrome silver & titanium white
        self._turret_tris, self._turret_rgb = self._load_turret_mesh()
        self._head_tris, self._head_rgb = self._load_head_mesh()

        # Height where the LDR sensor head pivots between the two vertical arms
        self._hinge_z: float = 31.0

        self.canvas = tk.Canvas(
            self,
            bg="#000000",
            highlightthickness=1,
            highlightbackground="#26262B",
            cursor="fleur",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Mouse interactions for camera orbit and zoom
        self.canvas.bind("<ButtonPress-1>", self._on_mouse_down)
        self.canvas.bind("<B1-Motion>", self._on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_mouse_up)
        self.canvas.bind("<Double-Button-1>", self._on_reset_camera)
        self.canvas.bind("<MouseWheel>", self._on_mouse_wheel)
        self.canvas.bind("<Configure>", self._on_resize)

    def _load_turret_mesh(self) -> Tuple[np.ndarray, Tuple[int, int, int]]:
        """Load and center `Base caja y brazos.stl` in brushed silver-grey."""
        stl_path = self.enclosure_dir / "Base caja y brazos.stl"
        if stl_path.is_file():
            try:
                tris = load_binary_stl(stl_path, min_area=3.5)
                pts = tris.reshape(-1, 3)
                cx = 0.5 * (pts[:, 0].min() + pts[:, 0].max())
                cy = 0.5 * (pts[:, 1].min() + pts[:, 1].max())
                z_min = pts[:, 2].min()
                tris[:, :, 0] -= cx
                tris[:, :, 1] -= cy
                tris[:, :, 2] -= z_min
                return tris, (158, 158, 168)  # Brushed silver-grey
            except Exception:
                pass
        return np.zeros((0, 3, 3), dtype=np.float32), (158, 158, 168)

    def _load_head_mesh(self) -> Tuple[np.ndarray, Tuple[int, int, int]]:
        """Load and center `Caja fotorresistencias.stl` in matte titanium white."""
        stl_path = self.enclosure_dir / "Caja fotorresistencias.stl"
        if stl_path.is_file():
            try:
                tris = load_binary_stl(stl_path, min_area=3.5)
                pts = tris.reshape(-1, 3)
                cx = 0.5 * (pts[:, 0].min() + pts[:, 0].max())
                cy = 0.5 * (pts[:, 1].min() + pts[:, 1].max())
                z_min = pts[:, 2].min()
                tris[:, :, 0] -= cx
                tris[:, :, 1] -= cy
                tris[:, :, 2] -= (z_min + 6.0)
                tris[:, :, 0] *= 0.74
                tris[:, :, 1] *= 0.78
                tris[:, :, 2] *= 0.78
                return tris, (242, 242, 247)  # Titanium white
            except Exception:
                pass
        return np.zeros((0, 3, 3), dtype=np.float32), (242, 242, 247)

    def set_pose(
        self,
        pulse_x: int,
        pulse_y: int,
        sun_counts: Optional[Tuple[float, float]] = None,
    ) -> None:
        """Update target X/Y counts and schedule a redraw only if servo counts changed."""
        _ = sun_counts
        if pulse_x != self.pulse_x or pulse_y != self.pulse_y:
            self.pulse_x = pulse_x
            self.pulse_y = pulse_y
            self.request_render()

    def request_render(self) -> None:
        """Coalesce multiple redraw requests into a single frame."""
        self._dirty = True
        if not self._render_scheduled:
            self._render_scheduled = True
            self.after(25, self._do_scheduled_render)

    def _do_scheduled_render(self) -> None:
        self._render_scheduled = False
        if self._dirty:
            self.render()

    def reset_camera(self) -> None:
        self.cam_azimuth_deg = -35.0
        self.cam_elevation_deg = 28.0
        self.cam_zoom = 3.2
        self.request_render()

    def _on_mouse_down(self, event: tk.Event) -> None:
        self._drag_last = (event.x, event.y)

    def _on_mouse_drag(self, event: tk.Event) -> None:
        if self._drag_last is None:
            return
        dx = event.x - self._drag_last[0]
        dy = event.y - self._drag_last[1]
        self._drag_last = (event.x, event.y)
        self.cam_azimuth_deg = (self.cam_azimuth_deg + dx * 0.6) % 360.0
        self.cam_elevation_deg = max(5.0, min(85.0, self.cam_elevation_deg + dy * 0.5))
        self.request_render()

    def _on_mouse_up(self, _event: tk.Event) -> None:
        self._drag_last = None

    def _on_reset_camera(self, _event: tk.Event) -> None:
        self.reset_camera()

    def _on_mouse_wheel(self, event: tk.Event) -> None:
        factor = 1.1 if event.delta > 0 else 0.9
        self.cam_zoom = max(1.6, min(6.0, self.cam_zoom * factor))
        self.request_render()

    def _on_resize(self, event: tk.Event) -> None:
        new_size = (event.width, event.height)
        if new_size != self._last_size and event.width >= 40 and event.height >= 40:
            self._last_size = new_size
            self.request_render()

    def _build_camera_matrix(self) -> np.ndarray:
        """
        Build right-handed orthonormal world-to-camera matrix:
        - Row 0 (X_cam): Right on screen
        - Row 1 (Y_cam): Up on screen (+Z_world has +cos(el) > 0, so +Z is always UP)
        - Row 2 (Z_cam): Depth toward viewer
        """
        az = math.radians(self.cam_azimuth_deg)
        el = math.radians(self.cam_elevation_deg)
        ca, sa = math.cos(az), math.sin(az)
        ce, se = math.cos(el), math.sin(el)
        return np.array(
            [
                [ca, sa, 0.0],
                [-sa * se, ca * se, ce],
                [sa * ce, -ca * ce, se],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _project_point(
        pt: np.ndarray,
        cam_mat: np.ndarray,
        center_x: float,
        center_y: float,
        scale: float,
    ) -> Tuple[float, float, float]:
        """Project a single 3D world point [x, y, z] to 2D canvas coordinates."""
        p = cam_mat @ pt
        depth = float(p[2])
        persp = 320.0 / max(120.0, 320.0 - depth)
        sx = center_x + float(p[0]) * scale * persp
        sy = center_y - float(p[1]) * scale * persp
        return sx, sy, depth

    def render(self) -> None:
        """Render the monochrome 3D scene onto the Tkinter Canvas."""
        if not self._dirty:
            return
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 40 or h < 40:
            return
        self._dirty = False

        self.canvas.delete("all")

        cx = w * 0.5
        cy = h * 0.68
        scale = min(w, h) * 0.0031 * self.cam_zoom
        cam_mat = self._build_camera_matrix()

        # 1. Draw 3D Monochrome Cardinal Ground Grid (Z = 0)
        grid_radius = 48.0
        for offset in (-40.0, -20.0, 0.0, 20.0, 40.0):
            col = "#27272A" if abs(offset) < 1e-3 else "#141418"
            p1 = self._project_point(np.array([-grid_radius, offset, 0.0], dtype=np.float32), cam_mat, cx, cy, scale)
            p2 = self._project_point(np.array([grid_radius, offset, 0.0], dtype=np.float32), cam_mat, cx, cy, scale)
            self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=col, width=1)

            q1 = self._project_point(np.array([offset, -grid_radius, 0.0], dtype=np.float32), cam_mat, cx, cy, scale)
            q2 = self._project_point(np.array([offset, grid_radius, 0.0], dtype=np.float32), cam_mat, cx, cy, scale)
            self.canvas.create_line(q1[0], q1[1], q2[0], q2[1], fill=col, width=1)

        # Cardinal labels on the ground plane
        cardinals = [
            ("N · 180°", np.array([0.0, 56.0, 0.0], dtype=np.float32), "#FFFFFF"),
            ("S · 0°", np.array([0.0, -56.0, 0.0], dtype=np.float32), "#71717A"),
            ("E · 180°", np.array([56.0, 0.0, 0.0], dtype=np.float32), "#D4D4D8"),
            ("W · 0°", np.array([-56.0, 0.0, 0.0], dtype=np.float32), "#71717A"),
        ]
        for label, pos, color in cardinals:
            sx, sy, _ = self._project_point(pos, cam_mat, cx, cy, scale)
            self.canvas.create_text(sx, sy, text=label, fill=color, font=("Bahnschrift", 9, "bold"))

        # 2. Compute Kinematic Transformations from pulse_x and pulse_y
        deg_x = counts_to_deg(self.pulse_x)  # 0..180 (90 = center)
        deg_y = counts_to_deg(self.pulse_y)  # 0..180 (90 = center)

        yaw_rad = math.radians(deg_x - 90.0)
        pitch_rad = math.radians((deg_y - 90.0) * 0.68)

        rot_yaw = _rotation_z(-yaw_rad)
        rot_pitch = _rotation_x(-pitch_rad)

        mesh_parts: List[Tuple[np.ndarray, Tuple[int, int, int]]] = []

        if len(self._turret_tris) > 0:
            turret_world = self._turret_tris @ rot_yaw.T
            mesh_parts.append((turret_world, self._turret_rgb))

        if len(self._head_tris) > 0:
            head_pitched = self._head_tris @ rot_pitch.T
            head_pitched[:, :, 2] += self._hinge_z
            head_world = head_pitched @ rot_yaw.T
            mesh_parts.append((head_world, self._head_rgb))

        # 3. Vectorized Projection, High-Contrast Studio Shading & Painter's Sort
        if mesh_parts:
            all_tris = np.concatenate([m[0] for m in mesh_parts], axis=0)
            base_colors = np.concatenate(
                [np.full((len(m[0]), 3), m[1], dtype=np.float32) for m in mesh_parts],
                axis=0,
            )

            v1 = all_tris[:, 1] - all_tris[:, 0]
            v2 = all_tris[:, 2] - all_tris[:, 0]
            normals = np.cross(v1, v2)
            norms = np.linalg.norm(normals, axis=1, keepdims=True)
            normals = normals / np.maximum(norms, 1e-6)

            cam_tris = all_tris @ cam_mat.T
            depths = cam_tris[:, :, 2].mean(axis=1)

            persp = 320.0 / np.maximum(120.0, 320.0 - cam_tris[:, :, 2])
            screen_x = cx + cam_tris[:, :, 0] * (scale * persp)
            screen_y = cy - cam_tris[:, :, 1] * (scale * persp)

            light_dir = np.array([0.42, -0.35, 0.84], dtype=np.float32)
            diffuse = np.abs(normals @ light_dir)
            shade = 0.26 + 0.74 * diffuse
            rgb = np.clip(base_colors * shade[:, None], 0, 255).astype(np.uint8)

            order = np.argsort(depths)
            sx_sorted = screen_x[order]
            sy_sorted = screen_y[order]
            rgb_sorted = rgb[order]

            create_poly = self.canvas.create_polygon
            for i in range(len(order)):
                r, g, b = int(rgb_sorted[i, 0]), int(rgb_sorted[i, 1]), int(rgb_sorted[i, 2])
                hex_col = f"#{r:02x}{g:02x}{b:02x}"
                # Subtle slightly darker edge on the largest faces for crisp architectural definition
                er, eg, eb = max(0, r - 12), max(0, g - 12), max(0, b - 12)
                edge_col = f"#{er:02x}{eg:02x}{eb:02x}"
                create_poly(
                    float(sx_sorted[i, 0]),
                    float(sy_sorted[i, 0]),
                    float(sx_sorted[i, 1]),
                    float(sy_sorted[i, 1]),
                    float(sx_sorted[i, 2]),
                    float(sy_sorted[i, 2]),
                    fill=hex_col,
                    outline=edge_col,
                )

        # 4. Draw Crisp White Optical Pointing Vector from the Sensor Head (+Z normal)
        local_normal = np.array([0.0, 0.0, 36.0], dtype=np.float32)
        world_dir = rot_yaw @ (rot_pitch @ local_normal)
        origin_world = np.array([0.0, 0.0, self._hinge_z + 10.0], dtype=np.float32)
        tip_world = origin_world + world_dir

        p0 = self._project_point(origin_world, cam_mat, cx, cy, scale)
        p1 = self._project_point(tip_world, cam_mat, cx, cy, scale)
        self.canvas.create_line(
            p0[0], p0[1], p1[0], p1[1],
            fill="#FFFFFF", width=2.2, arrow=tk.LAST, arrowshape=(10, 12, 5),
        )
        self.canvas.create_oval(
            p0[0] - 3, p0[1] - 3, p0[0] + 3, p0[1] + 3,
            fill="#FFFFFF", outline="#000000", width=1,
        )

        # 5. Translucent-Style Dark Grey HUD Pill Overlay
        us_x = counts_to_us(self.pulse_x)
        us_y = counts_to_us(self.pulse_y)
        self.canvas.create_rectangle(
            10, 10, 326, 54,
            fill="#0C0C0E", outline="#26262B", width=1,
        )
        self.canvas.create_text(
            20,
            15,
            anchor="nw",
            text="3D ENCLOSURE MODEL  ·  STL KINEMATICS",
            fill="#FFFFFF",
            font=("Bahnschrift", 10, "bold"),
        )
        self.canvas.create_text(
            20,
            33,
            anchor="nw",
            text=f"YAW {deg_x:5.1f}° ({us_x} µs)   ·   PITCH {deg_y:5.1f}° ({us_y} µs)",
            fill="#A1A1AA",
            font=("Cascadia Mono", 9),
        )
        self.canvas.create_text(
            w - 12,
            h - 10,
            anchor="se",
            text="Drag: Orbit  ·  Wheel: Zoom  ·  Double-Click: Reset",
            fill="#52525B",
            font=("Bahnschrift", 9),
        )
