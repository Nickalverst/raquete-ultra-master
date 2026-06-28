"""
Dashboard de telemetria da raquete instrumentada.
Protocolo: FlatBuffers binário via racket_fb.py

Wire format:
    [type:1][len_lo:1][len_hi:1][FlatBuffer payload]
    type 0x01 = ImuPacket   (42 bytes)
    type 0x02 = HitPacket   (54 bytes)

Uso:
    python telemetria_raquete.py --port COM3
    python telemetria_raquete.py --port /dev/ttyUSB0
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

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import gridspec
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    import serial
except ImportError:
    serial = None

from racket_fb import FB_TYPE_IMU, FB_TYPE_HIT, FrameReader, decode_imu, decode_hit


# ── Dados ──────────────────────────────────────────────────────────────────────

@dataclass
class ImuSample:
    t_pc:      float = 0.0
    ax_mg:     Optional[int] = None
    ay_mg:     Optional[int] = None
    az_mg:     Optional[int] = None
    roll_deg:  Optional[int] = None
    pitch_deg: Optional[int] = None
    yaw_deg:   Optional[int] = None

    @property
    def estado(self) -> str:
        """Derive orientation state from roll/pitch (computed on the PC side)."""
        if self.pitch_deg is None or self.roll_deg is None:
            return ""
        if self.pitch_deg > 45:   return "FRENTE  >>>"
        if self.pitch_deg < -45:  return "<<< ATRAS"
        if self.roll_deg  > 45:   return "DIREITA vvv"
        if self.roll_deg  < -45:  return "^^^ ESQUERDA"
        return "PLANO  [===]"


@dataclass
class HitSample:
    t_pc:         float = 0.0
    timestamp_ms: int   = 0
    region:       int   = 0
    peak_raw:     int   = 0
    heatmap:      List[int] = field(default_factory=lambda: [0] * 9)


Sample = Union[ImuSample, HitSample]


# ── Fontes de dados ────────────────────────────────────────────────────────────

class SerialSource:
    def __init__(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial não instalado: pip install pyserial")
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
        self.ser.reset_input_buffer()
        self._reader = FrameReader(self.ser)
        print(f"[INFO] Conectado em {port} @ {baud} bps")

    def read_samples(self) -> list[Sample]:
        samples: list[Sample] = []
        for t, payload in self._reader.read_frames(timeout_s=0.08):
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
    """Generates synthetic IMU and HIT samples without any hardware."""

    def __init__(self) -> None:
        self.t0         = time.time()
        self.last_imu   = -1.0
        self.last_hit   = -1.0
        self._heatmap   = [0] * 9

    def read_samples(self) -> list[Sample]:
        samples: list[Sample] = []
        now = time.time() - self.t0

        # ── IMU samples at 4 Hz ──────────────────────────────────────────
        while self.last_imu + 0.25 <= now:
            self.last_imu += 0.25
            t = self.last_imu
            samples.append(ImuSample(
                t_pc      = time.time(),
                ax_mg     = int(450 * math.sin(2.1 * t)),
                ay_mg     = int(280 * math.cos(1.6 * t)),
                az_mg     = int(1000 + 100 * math.sin(3.3 * t)),
                roll_deg  = int(28  * math.sin(1.2 * t)),
                pitch_deg = int(22  * math.cos(0.9 * t)),
                yaw_deg   = int(35  * math.sin(0.45 * t)),
            ))

        # ── Sporadic HIT samples ~once per second ────────────────────────
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


# ── Geometria 3D ───────────────────────────────────────────────────────────────

def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

def transform(pts: np.ndarray, roll: float, pitch: float, yaw: float) -> np.ndarray:
    return pts @ rotation_matrix(roll, pitch, yaw).T


# ── Dashboard ──────────────────────────────────────────────────────────────────

class Dashboard:
    WINDOW = 200

    def __init__(self, source) -> None:
        self.source      = source
        self.samples:   Deque[ImuSample] = deque(maxlen=self.WINDOW)
        self._heatmap   = [0] * 9

        # ── figura ──────────────────────────────────────────────────────────
        self.fig = plt.figure(figsize=(16, 8), facecolor="#0e1117")
        self.fig.suptitle("Raquete instrumentada — telemetria IMU",
                          color="#e8eaf0", fontsize=13, fontweight="bold", y=0.98)

        gs = gridspec.GridSpec(3, 3, figure=self.fig,
                               width_ratios=[1.1, 1.1, 1.2],
                               hspace=0.55, wspace=0.35)

        def make_ax(row, col, title, color):
            ax = self.fig.add_subplot(gs[row, col])
            ax.set_facecolor("#161b22")
            ax.set_title(title, color=color, fontsize=9, pad=4)
            ax.tick_params(colors="#6e7681", labelsize=8)
            for sp in ax.spines.values():
                sp.set_edgecolor("#30363d")
            ax.grid(True, color="#21262d", linewidth=0.5, linestyle="--")
            line, = ax.plot([], [], color=color, linewidth=1.4)
            return ax, line

        angle_color = "#58a6ff"
        acc_color   = "#f78166"

        self.ax_yaw,   self.ln_yaw   = make_ax(0, 0, "Yaw (°)",           angle_color)
        self.ax_roll,  self.ln_roll  = make_ax(0, 1, "Roll (°)",           angle_color)
        self.ax_pitch, self.ln_pitch = make_ax(1, 0, "Pitch (°)",          angle_color)
        self.ax_ax,    self.ln_ax    = make_ax(1, 1, "Aceleração X (mg)",  acc_color)
        self.ax_ay,    self.ln_ay    = make_ax(2, 0, "Aceleração Y (mg)",  acc_color)

        self.all_axes  = [self.ax_yaw, self.ax_roll, self.ax_pitch,
                          self.ax_ax,  self.ax_ay]
        self.all_lines = [self.ln_yaw, self.ln_roll, self.ln_pitch,
                          self.ln_ax,  self.ln_ay]

        # ── heatmap de impactos (gs[2,1]) ────────────────────────────────
        self.ax_hm = self.fig.add_subplot(gs[2, 1])
        self.ax_hm.set_facecolor("#161b22")
        self.ax_hm.set_title("Heatmap de impactos", color="#e8eaf0",
                              fontsize=9, pad=4)
        for sp in self.ax_hm.spines.values():
            sp.set_edgecolor("#30363d")
        self.ax_hm.set_xticks([])
        self.ax_hm.set_yticks([])

        self._hm_img = self.ax_hm.imshow(
            np.zeros((3, 3)), cmap="inferno", aspect="auto",
            vmin=0, vmax=1, origin="upper")
        self._hm_texts = [
            self.ax_hm.text(c, r, "0",
                            ha="center", va="center",
                            fontsize=11, color="white", fontweight="bold")
            for r in range(3) for c in range(3)
        ]
        self.txt_estado = self.ax_hm.text(
            0.5, -0.14, "Aguardando dados...",
            ha="center", va="top", fontsize=9, fontweight="bold",
            color="#e8eaf0", transform=self.ax_hm.transAxes)

        # ── 3D ──────────────────────────────────────────────────────────
        self.ax3d = self.fig.add_subplot(gs[:, 2], projection="3d")
        self.ax3d.set_facecolor("#0e1117")
        self.ax3d.set_title("Orientação 3D", color="#e8eaf0", fontsize=9, pad=6)

        plt.tight_layout(rect=[0, 0, 1, 0.97])

    # ── atualização ───────────────────────────────────────────────────────────

    def update(self, _frame):
        for s in self.source.read_samples():
            if isinstance(s, ImuSample):
                self.samples.append(s)
            elif isinstance(s, HitSample):
                self._heatmap = s.heatmap[:]

        if self.samples:
            self._update_graphs()
            self._update_heatmap(self.samples[-1])
            self._update_3d(self.samples[-1])
        else:
            self._update_3d(None)

        return self.all_lines

    def _update_graphs(self) -> None:
        data = list(self.samples)
        t0   = data[0].t_pc
        t    = np.array([(s.t_pc - t0) for s in data])

        series = [
            [s.yaw_deg   for s in data],
            [s.roll_deg  for s in data],
            [s.pitch_deg for s in data],
            [s.ax_mg     for s in data],
            [s.ay_mg     for s in data],
        ]

        t_end  = t[-1]
        t_from = max(0.0, t_end - 15.0)

        for ax, line, y in zip(self.all_axes, self.all_lines, series):
            line.set_data(t, y)
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(t_from, max(15.0, t_end + 0.5))

    def _update_heatmap(self, s: ImuSample) -> None:
        # Colour grid
        data = np.array(self._heatmap, dtype=float).reshape(3, 3)
        self._hm_img.set_data(data)
        mx = data.max()
        self._hm_img.set_clim(0, max(mx, 1))
        for idx, txt in enumerate(self._hm_texts):
            txt.set_text(str(self._heatmap[idx]))

        # Orientation label below the grid
        estado_map = {
            "FRENTE":   ("FRENTE  >>>", "#f0883e"),
            "ATRAS":    ("<<< ATRÁS",   "#f0883e"),
            "DIREITA":  ("DIREITA  ↓",  "#58a6ff"),
            "ESQUERDA": ("↑  ESQUERDA", "#58a6ff"),
            "PLANO":    ("PLANO  ═══",  "#3fb950"),
        }
        texto, cor = s.estado, "#e8eaf0"
        for key, (label, c) in estado_map.items():
            if key in s.estado.upper():
                texto, cor = label, c
                break
        self.txt_estado.set_text(texto)
        self.txt_estado.set_color(cor)

    def _update_3d(self, s: Optional[ImuSample]) -> None:
        ax = self.ax3d
        ax.cla()
        ax.set_facecolor("#0e1117")
        ax.set_title("Orientação 3D", color="#e8eaf0", fontsize=9, pad=6)
        ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2); ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel("X", color="#6e7681", fontsize=8)
        ax.set_ylabel("Y", color="#6e7681", fontsize=8)
        ax.set_zlabel("Z", color="#6e7681", fontsize=8)
        ax.tick_params(colors="#6e7681", labelsize=7)
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#21262d")

        roll  = s.roll_deg  if s else 0.0
        pitch = s.pitch_deg if s else 0.0
        yaw   = s.yaw_deg   if s else 0.0

        theta  = np.linspace(0, 2 * math.pi, 64)
        radius = 0.42
        cy     = 0.28
        circle = np.column_stack([radius * np.cos(theta),
                                  cy + radius * np.sin(theta),
                                  np.zeros_like(theta)])
        handle = np.array([[-0.10, -1.00, 0.0], [ 0.10, -1.00, 0.0],
                           [ 0.13, -0.05, 0.0], [-0.13, -0.05, 0.0]])
        dz = 0.035

        cf = transform(circle + [0, 0,  dz], roll, pitch, yaw)
        cb = transform(circle + [0, 0, -dz], roll, pitch, yaw)
        hf = transform(handle + [0, 0,  dz], roll, pitch, yaw)
        hb = transform(handle + [0, 0, -dz], roll, pitch, yaw)

        ax.add_collection3d(Poly3DCollection([cf], alpha=0.55,
                            facecolor="#238636", edgecolor="#3fb950"))
        ax.add_collection3d(Poly3DCollection([cb], alpha=0.20,
                            facecolor="#238636", edgecolor="none"))
        ax.add_collection3d(Poly3DCollection([hf], alpha=0.65,
                            facecolor="#6e4c1e", edgecolor="#9e6b2e"))
        ax.add_collection3d(Poly3DCollection([hb], alpha=0.25,
                            facecolor="#6e4c1e", edgecolor="none"))

        ax.plot(cf[:, 0], cf[:, 1], cf[:, 2], color="#3fb950", linewidth=1.2)
        ch = np.vstack([hf, hf[0]])
        ax.plot(ch[:, 0], ch[:, 1], ch[:, 2], color="#9e6b2e", linewidth=1.2)

        for x in [-radius / 3, radius / 3]:
            ln = np.array([[x, cy - radius, dz * 1.5], [x, cy + radius, dz * 1.5]])
            r  = transform(ln, roll, pitch, yaw)
            ax.plot(r[:, 0], r[:, 1], r[:, 2], color="#58a6ff", linewidth=0.6, alpha=0.6)
        for yl in [cy - radius / 3, cy + radius / 3]:
            ln = np.array([[-radius, yl, dz * 1.5], [radius, yl, dz * 1.5]])
            r  = transform(ln, roll, pitch, yaw)
            ax.plot(r[:, 0], r[:, 1], r[:, 2], color="#58a6ff", linewidth=0.6, alpha=0.6)

        R = rotation_matrix(roll, pitch, yaw)
        origin = np.zeros(3)
        for i, c in enumerate(["#f78166", "#3fb950", "#58a6ff"]):
            v = R[i] * 0.6
            ax.quiver(*origin, *v, color=c, linewidth=1.2, arrow_length_ratio=0.2)

        label = ("sem dados" if s is None
                 else f"yaw={yaw:.0f}°  roll={roll:.0f}°  pitch={pitch:.0f}°")
        ax.text2D(0.02, 0.02, label, transform=ax.transAxes,
                  color="#8b949e", fontsize=7)

    # ── loop principal ────────────────────────────────────────────────────────

    def run(self) -> None:
        self.anim = animation.FuncAnimation(
            self.fig, self.update,
            interval=80, blit=False, cache_frame_data=False)
        plt.show()


# ── CLI ────────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="Dashboard telemetria raquete")
    ap.add_argument("--port", help="Ex.: COM3 ou /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mock", "--simulate", action="store_true",
                    help="Dados falsos sem hardware")
    args = ap.parse_args()

    if args.mock:
        print("[INFO] Modo mock ativo\n")
        source = MockSource()
    elif args.port:
        source = SerialSource(args.port, args.baud)
    else:
        print("Erro: passe --port COMx ou use --mock", file=sys.stderr)
        return 2

    Dashboard(source).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
