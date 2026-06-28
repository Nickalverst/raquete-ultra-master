"""
Dashboard de telemetria da raquete instrumentada — PyQtGraph edition.
Protocolo: FlatBuffers binário via racket_fb.py

Wire format:
    [type:1][len_lo:1][len_hi:1][FlatBuffer payload]
    type 0x01 = ImuPacket   (42 bytes)
    type 0x02 = HitPacket   (54 bytes)

Uso:
    python telemetria_raquete.py --port COM3
    python telemetria_raquete.py --port /dev/ttyUSB0 --baud 115200
    python telemetria_raquete.py --mock
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Union

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt.QtCore import Qt, QTimer
from pyqtgraph.Qt.QtGui import QFont
from pyqtgraph.Qt.QtWidgets import (
    QApplication, QHBoxLayout, QLabel,
    QMainWindow, QVBoxLayout, QWidget,
)

try:
    import pyqtgraph.opengl as gl
    _GL = True
except ImportError:
    _GL = False
    print("[WARN] PyOpenGL não encontrado — view 3D desativada.\n"
          "       pip install PyOpenGL PyOpenGL_accelerate")

try:
    import serial
except ImportError:
    serial = None

from racket_fb import FB_TYPE_HIT, FB_TYPE_IMU, FrameReader, decode_hit, decode_imu


# ── configuração global do pyqtgraph ──────────────────────────────────────────
pg.setConfigOptions(antialias=True, foreground="#e8eaf0", background="#0e1117")


# ── helpers de geometria ───────────────────────────────────────────────────────

def _rot_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    Rx = np.array([[1, 0,           0           ],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [ 0,           1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,            1]])
    return Rz @ Ry @ Rx


# ── modelos de dados ───────────────────────────────────────────────────────────

@dataclass
class ImuSample:
    t_pc:      float        = 0.0
    yaw_deg:   Optional[int] = None
    roll_deg:  Optional[int] = None
    pitch_deg: Optional[int] = None
    ax_mg:     Optional[int] = None
    ay_mg:     Optional[int] = None
    az_mg:     Optional[int] = None

    @property
    def estado(self) -> str:
        """Orientation label derived from roll/pitch on the PC side."""
        if self.pitch_deg is None or self.roll_deg is None:
            return ""
        if self.pitch_deg >  45: return "FRENTE  >>>"
        if self.pitch_deg < -45: return "<<< ATRAS"
        if self.roll_deg  >  45: return "DIREITA  v"
        if self.roll_deg  < -45: return "^  ESQUERDA"
        return "PLANO  ==="


@dataclass
class HitSample:
    t_pc:         float     = 0.0
    timestamp_ms: int       = 0
    region:       int       = 0
    peak_raw:     int       = 0
    heatmap:      List[int] = field(default_factory=lambda: [0] * 9)


Sample = Union[ImuSample, HitSample]


# ── fontes de dados ────────────────────────────────────────────────────────────

class SerialSource:
    def __init__(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial nao instalado: pip install pyserial")
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
        self.ser.reset_input_buffer()
        self._reader = FrameReader(self.ser)
        print(f"[INFO] Conectado em {port} @ {baud} bps")

    def read_samples(self) -> list[Sample]:
        samples: list[Sample] = []
        for t, payload in self._reader.read_frames(timeout_s=0.04):
            now = time.time()
            if t == FB_TYPE_IMU:
                d = decode_imu(payload)
                samples.append(ImuSample(
                    t_pc      = now,
                    yaw_deg   = d["yaw_deg"],
                    roll_deg  = d["roll_deg"],
                    pitch_deg = d["pitch_deg"],
                    ax_mg     = d["ax_mg"],
                    ay_mg     = d["ay_mg"],
                    az_mg     = d["az_mg"],
                ))
            elif t == FB_TYPE_HIT:
                d = decode_hit(payload)
                samples.append(HitSample(
                    t_pc         = now,
                    timestamp_ms = d["timestamp_ms"],
                    region       = d["region"],
                    peak_raw     = d["peak_raw"],
                    heatmap      = d["heatmap"],
                ))
        return samples


class MockSource:
    """Generates synthetic IMU + HIT samples without hardware."""

    def __init__(self) -> None:
        self.t0       = time.time()
        self.last_imu = -1.0
        self.last_hit = -1.0
        self._heatmap = [0] * 9

    def read_samples(self) -> list[Sample]:
        samples: list[Sample] = []
        now = time.time() - self.t0

        # IMU at 20 Hz (matching vTaskIMU)
        while self.last_imu + 0.05 <= now:
            self.last_imu += 0.05
            t = self.last_imu
            samples.append(ImuSample(
                t_pc      = time.time(),
                yaw_deg   = int(35  * math.sin(0.45 * t)),
                roll_deg  = int(28  * math.sin(1.20 * t)),
                pitch_deg = int(22  * math.cos(0.90 * t)),
                ax_mg     = int(450 * math.sin(2.10 * t)),
                ay_mg     = int(280 * math.cos(1.60 * t)),
                az_mg     = int(1000 + 100 * math.sin(3.30 * t)),
            ))

        # Sporadic hits ~once per second
        if now > 1.0 and now - self.last_hit >= 1.0 and random.random() < 0.4:
            self.last_hit = now
            region = random.randint(0, 8)
            self._heatmap[region] += 1
            samples.append(HitSample(
                t_pc         = time.time(),
                timestamp_ms = int(now * 1000),
                region       = region,
                peak_raw     = random.randint(500, 4000),
                heatmap      = list(self._heatmap),
            ))

        return samples


# ── dashboard ──────────────────────────────────────────────────────────────────

_C_BG     = "#0e1117"
_C_PANEL  = "#161b22"
_C_BORDER = "#30363d"
_C_TICK   = "#6e7681"
_C_TEXT   = "#e8eaf0"
_C_ANGLE  = "#58a6ff"
_C_ACC    = "#f78166"
_C_GREEN  = "#3fb950"
_C_ORANGE = "#f0883e"

_ESTADO_STYLE: dict[str, tuple[str, str]] = {
    "FRENTE":   ("FRENTE  >>>", _C_ORANGE),
    "ATRAS":    ("<<< ATRAS",   _C_ORANGE),
    "DIREITA":  ("DIREITA  v",  _C_ANGLE),
    "ESQUERDA": ("^  ESQUERDA", _C_ANGLE),
    "PLANO":    ("PLANO  ===",  _C_GREEN),
}


class Dashboard(QMainWindow):

    WINDOW = 400  # 400 samples * 50 ms = 20 s rolling window

    def __init__(self, source: SerialSource | MockSource) -> None:
        super().__init__()
        self.source   = source
        self.samples: Deque[ImuSample] = deque(maxlen=self.WINDOW)
        self._heatmap = [0] * 9

        self.setWindowTitle("Raquete instrumentada — telemetria IMU")
        self.resize(1440, 820)
        self.setStyleSheet(f"background: {_C_BG};")

        # ── central widget ───────────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        outer = QHBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        # Left: plots + heatmap in a GraphicsLayoutWidget, plus estado label
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        self._glw = pg.GraphicsLayoutWidget()
        self._glw.setBackground(_C_BG)
        left_layout.addWidget(self._glw, stretch=1)

        self._lbl_estado = QLabel("Aguardando dados...")
        self._lbl_estado.setAlignment(Qt.AlignCenter)
        self._lbl_estado.setStyleSheet(
            f"color: {_C_TEXT}; font-size: 13px; font-weight: bold;"
            f"background: {_C_PANEL}; border-radius: 4px; padding: 5px;")
        left_layout.addWidget(self._lbl_estado)

        outer.addWidget(left_panel, stretch=3)

        # Right: 3D GL view
        if _GL:
            self.view3d = gl.GLViewWidget()
            self.view3d.setBackgroundColor(pg.mkColor(_C_BG))
            self.view3d.setCameraPosition(distance=3.5, elevation=20, azimuth=45)
            outer.addWidget(self.view3d, stretch=2)

        # ── build sub-widgets ────────────────────────────────────────────
        self._setup_plots()
        self._setup_heatmap()
        if _GL:
            self._setup_3d()

        # ── 50 ms timer — 20 Hz, matching vTaskIMU ───────────────────────
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    # ── plots ──────────────────────────────────────────────────────────────────

    def _make_plot(self, row: int, col: int, title: str,
                   pen: pg.mkPen) -> tuple[pg.PlotItem, pg.PlotDataItem]:
        p = self._glw.addPlot(row=row, col=col)
        p.setTitle(title, color=_C_TICK, size="9pt")
        for axis in ("left", "bottom"):
            ax = p.getAxis(axis)
            ax.setPen(_C_BORDER)
            ax.setTextPen(_C_TICK)
            ax.setStyle(tickFont=QFont("monospace", 7))
        p.showGrid(x=True, y=True, alpha=0.25)
        p.setMenuEnabled(False)
        p.setMouseEnabled(x=False, y=True)
        p.setDownsampling(auto=True, mode="peak")
        curve = p.plot(pen=pen)
        return p, curve

    def _setup_plots(self) -> None:
        a_pen = pg.mkPen(_C_ANGLE, width=1.5)
        c_pen = pg.mkPen(_C_ACC,   width=1.5)

        defs = [
            (0, 0, "Yaw (graus)",       a_pen),
            (0, 1, "Roll (graus)",      a_pen),
            (1, 0, "Pitch (graus)",     a_pen),
            (1, 1, "Aceleracao X (mg)", c_pen),
            (2, 0, "Aceleracao Y (mg)", c_pen),
        ]

        self._plots:  dict[str, pg.PlotItem]     = {}
        self._curves: dict[str, pg.PlotDataItem] = {}

        for row, col, title, pen in defs:
            plot, curve = self._make_plot(row, col, title, pen)
            self._plots[title]  = plot
            self._curves[title] = curve

        for c in (0, 1):
            self._glw.ci.layout.setColumnStretchFactor(c, 1)

    # ── heatmap ────────────────────────────────────────────────────────────────

    def _setup_heatmap(self) -> None:
        hm = self._glw.addPlot(row=2, col=1)
        hm.setTitle("Heatmap de impactos", color=_C_TICK, size="9pt")
        hm.hideAxis("left")
        hm.hideAxis("bottom")
        hm.setAspectLocked(True)
        hm.setRange(xRange=[0, 3], yRange=[0, 3], padding=0.02)
        hm.getViewBox().invertY(True)
        hm.setMenuEnabled(False)
        hm.setMouseEnabled(x=False, y=False)

        cmap = pg.colormap.get("inferno")

        self._hm_img = pg.ImageItem(np.zeros((3, 3)))
        self._hm_img.setColorMap(cmap)
        self._hm_img.setLevels((0, 1))
        self._hm_img.setRect(0, 0, 3, 3)
        hm.addItem(self._hm_img)

        div_pen = pg.mkPen(_C_BORDER, width=1)
        for v in (1, 2):
            hm.addItem(pg.InfiniteLine(pos=v, angle=90, pen=div_pen))
            hm.addItem(pg.InfiniteLine(pos=v, angle=0,  pen=div_pen))

        cell_font = QFont("monospace")
        cell_font.setPointSize(11)
        cell_font.setBold(True)

        self._hm_texts: list[pg.TextItem] = []
        for idx in range(9):
            r, c = divmod(idx, 3)
            t = pg.TextItem("0", anchor=(0.5, 0.5), color="w")
            t.setFont(cell_font)
            t.setPos(c + 0.5, r + 0.5)
            hm.addItem(t)
            self._hm_texts.append(t)

    # ── 3D view ────────────────────────────────────────────────────────────────

    _HEAD_R  = 0.42
    _HEAD_CY = 0.28
    _STR_R   = 0.42 * 0.85
    _STR_OFF = 0.42 * 0.85 / 2.5

    def _setup_3d(self) -> None:
        grid = gl.GLGridItem()
        grid.setSize(3, 3, 0)
        grid.setSpacing(0.5, 0.5, 0.5)
        grid.setColor(pg.mkColor(_C_BORDER))
        self.view3d.addItem(grid)

        # Base geometry in the identity orientation (z=0 plane)
        theta = np.linspace(0, 2 * math.pi, 65, dtype=np.float32)
        self._base_circle = np.column_stack([
            self._HEAD_R * np.cos(theta),
            self._HEAD_CY + self._HEAD_R * np.sin(theta),
            np.zeros(65, dtype=np.float32),
        ])

        self._base_handle = np.array([
            [-0.10, -1.00, 0.0],
            [ 0.10, -1.00, 0.0],
            [ 0.13, -0.05, 0.0],
            [-0.13, -0.05, 0.0],
            [-0.10, -1.00, 0.0],
        ], dtype=np.float32)

        sr, so, cy = self._STR_R, self._STR_OFF, self._HEAD_CY
        self._base_strings = [
            np.array([[-so, cy - sr, 0], [-so, cy + sr, 0]], dtype=np.float32),
            np.array([[ so, cy - sr, 0], [ so, cy + sr, 0]], dtype=np.float32),
            np.array([[-sr, cy - so, 0], [ sr, cy - so, 0]], dtype=np.float32),
            np.array([[-sr, cy + so, 0], [ sr, cy + so, 0]], dtype=np.float32),
        ]

        self._gl_head = gl.GLLinePlotItem(
            pos=self._base_circle,
            color=(0.14, 0.52, 0.21, 1.0), width=2.5, antialias=True)
        self.view3d.addItem(self._gl_head)

        self._gl_handle = gl.GLLinePlotItem(
            pos=self._base_handle,
            color=(0.55, 0.38, 0.16, 1.0), width=2.5, antialias=True)
        self.view3d.addItem(self._gl_handle)

        str_color = (0.34, 0.63, 0.83, 0.55)
        self._gl_strings: list[gl.GLLinePlotItem] = []
        for seg in self._base_strings:
            item = gl.GLLinePlotItem(pos=seg, color=str_color,
                                     width=1.2, antialias=True)
            self.view3d.addItem(item)
            self._gl_strings.append(item)

        # X=red  Y=green  Z=blue
        axis_colors = [
            (0.97, 0.49, 0.40, 1.0),
            (0.24, 0.71, 0.31, 1.0),
            (0.34, 0.65, 1.00, 1.0),
        ]
        self._gl_axes: list[gl.GLLinePlotItem] = []
        for c in axis_colors:
            item = gl.GLLinePlotItem(
                pos=np.zeros((2, 3), dtype=np.float32),
                color=c, width=2.0, antialias=True)
            self.view3d.addItem(item)
            self._gl_axes.append(item)

    def _update_3d(self, roll: float, pitch: float, yaw: float) -> None:
        R = _rot_matrix(roll, pitch, yaw).astype(np.float32)

        self._gl_head.setData(pos=(self._base_circle @ R.T))
        self._gl_handle.setData(pos=(self._base_handle @ R.T))

        for item, seg in zip(self._gl_strings, self._base_strings):
            item.setData(pos=(seg @ R.T))

        origin = np.zeros(3, dtype=np.float32)
        for i, ax_item in enumerate(self._gl_axes):
            tip = np.zeros(3, dtype=np.float32)
            tip[i] = 0.65
            ax_item.setData(
                pos=np.array([origin, tip @ R.T], dtype=np.float32))

    # ── tick ───────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        for s in self.source.read_samples():
            if isinstance(s, ImuSample):
                self.samples.append(s)
            elif isinstance(s, HitSample):
                self._heatmap = s.heatmap[:]

        if not self.samples:
            return

        self._update_plots()
        self._update_heatmap()

        last = self.samples[-1]
        if _GL:
            self._update_3d(
                last.roll_deg  or 0.0,
                last.pitch_deg or 0.0,
                last.yaw_deg   or 0.0,
            )
        self._update_estado(last)

    def _update_plots(self) -> None:
        data  = list(self.samples)
        t0    = data[0].t_pc
        t_arr = np.fromiter(
            (s.t_pc - t0 for s in data), dtype=np.float64, count=len(data))

        t_end  = t_arr[-1]
        t_from = max(0.0, t_end - 20.0)

        attrs = {
            "Yaw (graus)":       "yaw_deg",
            "Roll (graus)":      "roll_deg",
            "Pitch (graus)":     "pitch_deg",
            "Aceleracao X (mg)": "ax_mg",
            "Aceleracao Y (mg)": "ay_mg",
        }
        for title, attr in attrs.items():
            y = np.fromiter(
                (getattr(s, attr) or 0 for s in data),
                dtype=np.float32, count=len(data))
            self._curves[title].setData(t_arr, y)
            self._plots[title].setXRange(
                t_from, max(20.0, t_end + 0.5), padding=0)

    def _update_heatmap(self) -> None:
        data = np.array(self._heatmap, dtype=np.float32).reshape(3, 3)
        mx   = float(data.max())
        self._hm_img.setImage(data)
        self._hm_img.setLevels((0.0, max(mx, 1.0)))
        for idx, txt in enumerate(self._hm_texts):
            txt.setText(str(self._heatmap[idx]))

    def _update_estado(self, s: ImuSample) -> None:
        texto = s.estado or "Aguardando dados..."
        cor   = _C_TEXT
        for key, (label, c) in _ESTADO_STYLE.items():
            if key in texto.upper():
                texto, cor = label, c
                break
        self._lbl_estado.setText(texto)
        self._lbl_estado.setStyleSheet(
            f"color: {cor}; font-size: 13px; font-weight: bold;"
            f"background: {_C_PANEL}; border-radius: 4px; padding: 5px;")


# ── CLI ────────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="Dashboard telemetria raquete")
    ap.add_argument("--port", help="Porta serial (ex.: COM3, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mock", "--simulate", action="store_true",
                    help="Dados simulados sem hardware")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName("Telemetria Raquete")

    if args.mock:
        print("[INFO] Modo mock ativo\n")
        source: SerialSource | MockSource = MockSource()
    elif args.port:
        source = SerialSource(args.port, args.baud)
    else:
        print("Erro: passe --port COMx ou use --mock", file=sys.stderr)
        return 2

    dash = Dashboard(source)
    dash.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
