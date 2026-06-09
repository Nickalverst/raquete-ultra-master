"""
Dashboard de telemetria da raquete instrumentada.
Lê o formato de texto atual do firmware:

    Accel : X=    3mg  Y=    5mg  Z= 1106mg
    Roll  :    0deg   Pitch:    0deg   Yaw:   65deg
    Estado: PLANO  [===]
    ----------------------------------

Uso:
    python telemetria_raquete.py --port COM3
    python telemetria_raquete.py --port /dev/ttyUSB0
    python telemetria_raquete.py --mock
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import gridspec
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    import serial
except ImportError:
    serial = None


# ── Regex ──────────────────────────────────────────────────────────────────────

ACCEL_RE = re.compile(
    r"Accel\s*:\s*X=\s*(?P<ax>-?\d+)mg\s+Y=\s*(?P<ay>-?\d+)mg\s+Z=\s*(?P<az>-?\d+)mg"
)
ANGLE_RE = re.compile(
    r"Roll\s*:\s*(?P<roll>-?\d+)deg\s+Pitch\s*:\s*(?P<pitch>-?\d+)deg\s+Yaw\s*:\s*(?P<yaw>-?\d+)deg"
)
ESTADO_RE = re.compile(r"Estado\s*:\s*(?P<estado>.+)")
SEP_RE    = re.compile(r"^-{5,}$")


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
    estado:    str = ""

    def complete(self) -> bool:
        return None not in (self.ax_mg, self.ay_mg, self.az_mg,
                            self.roll_deg, self.pitch_deg, self.yaw_deg)


# ── Parser de blocos ───────────────────────────────────────────────────────────

class BlockParser:
    def __init__(self) -> None:
        self._cur = ImuSample()

    def feed(self, line: str) -> Optional[ImuSample]:
        line = line.strip()

        m = ACCEL_RE.search(line)
        if m:
            self._cur.ax_mg = int(m.group("ax"))
            self._cur.ay_mg = int(m.group("ay"))
            self._cur.az_mg = int(m.group("az"))
            self._cur.t_pc  = time.time()
            return None

        m = ANGLE_RE.search(line)
        if m:
            self._cur.roll_deg  = int(m.group("roll"))
            self._cur.pitch_deg = int(m.group("pitch"))
            self._cur.yaw_deg   = int(m.group("yaw"))
            return None

        m = ESTADO_RE.search(line)
        if m:
            self._cur.estado = m.group("estado").strip()
            return None

        if SEP_RE.match(line):
            if self._cur.complete():
                sample, self._cur = self._cur, ImuSample()
                return sample
            else:
                self._cur = ImuSample()

        return None


# ── Fontes de dados ────────────────────────────────────────────────────────────

class SerialSource:
    def __init__(self, port: str, baud: int, parser: BlockParser) -> None:
        if serial is None:
            raise RuntimeError("pyserial não instalado: pip install pyserial")
        self.ser    = serial.Serial(port=port, baudrate=baud, timeout=1.0)
        self.parser = parser
        self.ser.reset_input_buffer()
        print(f"[INFO] Conectado em {port} @ {baud} bps")

    def read_samples(self) -> list[ImuSample]:
        samples = []
        deadline = time.time() + 0.08
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            s = self.parser.feed(raw.decode("utf-8", errors="ignore"))
            if s:
                samples.append(s)
        return samples


class MockSource:
    def __init__(self) -> None:
        self.t0        = time.time()
        self.last_sent = -1.0
        self.parser    = BlockParser()

    def read_samples(self) -> list[ImuSample]:
        samples = []
        now = time.time() - self.t0
        while self.last_sent + 0.25 <= now:
            self.last_sent += 0.25
            t = self.last_sent
            ax    = int(450 * math.sin(2.1 * t))
            ay    = int(280 * math.cos(1.6 * t))
            az    = int(1000 + 100 * math.sin(3.3 * t))
            roll  = int(28  * math.sin(1.2 * t))
            pitch = int(22  * math.cos(0.9 * t))
            yaw   = int(35  * math.sin(0.45 * t))
            if   pitch >  45: estado = "FRENTE  >>>"
            elif pitch < -45: estado = "<<< ATRAS"
            elif roll  >  45: estado = "DIREITA vvv"
            elif roll  < -45: estado = "^^^ ESQUERDA"
            else:             estado = "PLANO  [===]"
            lines = [
                f"Accel : X={ax:5d}mg  Y={ay:5d}mg  Z={az:5d}mg",
                f"Roll  : {roll:4d}deg   Pitch: {pitch:4d}deg   Yaw: {yaw:4d}deg",
                f"Estado: {estado}",
                "----------------------------------",
            ]
            for ln in lines:
                s = self.parser.feed(ln)
                if s:
                    samples.append(s)
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
    WINDOW = 200   # amostras no histórico

    def __init__(self, source) -> None:
        self.source  = source
        self.samples: Deque[ImuSample] = deque(maxlen=self.WINDOW)

        # ── figura ──────────────────────────────────────────────────────────
        self.fig = plt.figure(figsize=(16, 8), facecolor="#0e1117")
        self.fig.suptitle("Raquete instrumentada — telemetria IMU",
                          color="#e8eaf0", fontsize=13, fontweight="bold", y=0.98)

        gs = gridspec.GridSpec(3, 3, figure=self.fig,
                               width_ratios=[1.1, 1.1, 1.2],
                               hspace=0.55, wspace=0.35)

        ax_cfg = dict(facecolor="#161b22",
                      tick_params=dict(colors="#6e7681", labelsize=8),
                      spine_color="#30363d")

        def make_ax(row, col, title, color):
            ax = self.fig.add_subplot(gs[row, col])
            ax.set_facecolor(ax_cfg["facecolor"])
            ax.set_title(title, color=color, fontsize=9, pad=4)
            ax.tick_params(colors=ax_cfg["tick_params"]["colors"],
                           labelsize=ax_cfg["tick_params"]["labelsize"])
            for sp in ax.spines.values():
                sp.set_edgecolor(ax_cfg["spine_color"])
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
        self.ax_az,    self.ln_az    = make_ax(2, 1, "Aceleração Z (mg)",  acc_color)

        self.all_axes  = [self.ax_yaw, self.ax_roll, self.ax_pitch,
                          self.ax_ax,  self.ax_ay,   self.ax_az]
        self.all_lines = [self.ln_yaw, self.ln_roll, self.ln_pitch,
                          self.ln_ax,  self.ln_ay,   self.ln_az]

        # ── painel de estado (centro-baixo) ─────────────────────────────────
        self.ax_status = self.fig.add_subplot(gs[2, 1])
        self.ax_status.set_facecolor("#161b22")
        self.ax_status.axis("off")
        for sp in self.ax_status.spines.values():
            sp.set_edgecolor("#30363d")
        self.txt_status = self.ax_status.text(
            0.5, 0.55, "Aguardando dados...",
            ha="center", va="center", fontsize=14, fontweight="bold",
            color="#e8eaf0", transform=self.ax_status.transAxes)
        self.txt_vals = self.ax_status.text(
            0.5, 0.15, "",
            ha="center", va="center", fontsize=8,
            color="#8b949e", transform=self.ax_status.transAxes,
            fontfamily="monospace")

        # ── 3D ──────────────────────────────────────────────────────────────
        self.ax3d = self.fig.add_subplot(gs[:, 2], projection="3d")
        self.ax3d.set_facecolor("#0e1117")
        self.ax3d.set_title("Orientação 3D", color="#e8eaf0", fontsize=9, pad=6)

        plt.tight_layout(rect=[0, 0, 1, 0.97])

    # ── atualização ───────────────────────────────────────────────────────────

    def update(self, _frame):
        for s in self.source.read_samples():
            self.samples.append(s)

        if self.samples:
            self._update_graphs()
            self._update_status(self.samples[-1])
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
            [s.az_mg     for s in data],
        ]

        t_end  = t[-1]
        t_from = max(0.0, t_end - 15.0)

        for ax, line, y in zip(self.all_axes, self.all_lines, series):
            line.set_data(t, y)
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(t_from, max(15.0, t_end + 0.5))

    def _update_status(self, s: ImuSample) -> None:
        estado_map = {
            "FRENTE":   ("FRENTE  >>>",  "#f0883e"),
            "ATRAS":    ("<<< ATRÁS",    "#f0883e"),
            "DIREITA":  ("DIREITA  ↓",   "#58a6ff"),
            "ESQUERDA": ("↑  ESQUERDA",  "#58a6ff"),
            "PLANO":    ("PLANO  ═══",   "#3fb950"),
        }
        cor   = "#e8eaf0"
        texto = s.estado
        for key, (label, c) in estado_map.items():
            if key in s.estado.upper():
                texto, cor = label, c
                break

        self.txt_status.set_text(texto)
        self.txt_status.set_color(cor)
        self.txt_vals.set_text(
            f"yaw {s.yaw_deg:+4d}°   roll {s.roll_deg:+4d}°   pitch {s.pitch_deg:+4d}°\n"
            f"ax {s.ax_mg:+5d} mg   ay {s.ay_mg:+5d} mg   az {s.az_mg:+5d} mg"
        )

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

        # Face circular da raquete
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

        # Grid 3x3 sobre a face
        for x in [-radius / 3, radius / 3]:
            ln = np.array([[x, cy - radius, dz * 1.5], [x, cy + radius, dz * 1.5]])
            r  = transform(ln, roll, pitch, yaw)
            ax.plot(r[:, 0], r[:, 1], r[:, 2], color="#58a6ff", linewidth=0.6, alpha=0.6)
        for yl in [cy - radius / 3, cy + radius / 3]:
            ln = np.array([[-radius, yl, dz * 1.5], [radius, yl, dz * 1.5]])
            r  = transform(ln, roll, pitch, yaw)
            ax.plot(r[:, 0], r[:, 1], r[:, 2], color="#58a6ff", linewidth=0.6, alpha=0.6)

        # Eixos locais
        R = rotation_matrix(roll, pitch, yaw)
        origin = np.zeros(3)
        colors_ax = ["#f78166", "#3fb950", "#58a6ff"]
        for i, c in enumerate(colors_ax):
            v = R[i] * 0.6
            ax.quiver(*origin, *v, color=c, linewidth=1.2, arrow_length_ratio=0.2)

        label = "sem dados" if s is None else f"yaw={yaw:.0f}°  roll={roll:.0f}°  pitch={pitch:.0f}°"
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
    ap.add_argument("--mock", action="store_true", help="Dados falsos sem hardware")
    args = ap.parse_args()

    if args.mock:
        print("[INFO] Modo mock ativo\n")
        source = MockSource()
    elif args.port:
        source = SerialSource(args.port, args.baud, BlockParser())
    else:
        print("Erro: passe --port COMx ou use --mock", file=sys.stderr)
        return 2

    Dashboard(source).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())