"""
Recebedor de telemetria da raquete instrumentada.

Este script roda NO COMPUTADOR, não no STM32.
Ele lê uma porta serial criada pelo rádio/USB/Bluetooth e mostra:
- 6 gráficos em tempo real: yaw, ang_x/roll, ang_y/pitch, acc_x, acc_y, acc_z
- uma visualização 3D simples da orientação da raquete

Formato recomendado vindo do firmware:
TEL,t_ms,yaw_deg,ang_x_deg,ang_y_deg,acc_x_mg,acc_y_mg,acc_z_mg

Exemplo:
TEL,1250,3,12,-4,30,-21,985

Também tenta ler o formato antigo do firmware, em linhas como:
Accel : X=   30mg  Y=  -21mg  Z=  985mg
Roll  :   12deg   Pitch:   -4deg   Yaw:    3deg
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Iterable, Optional

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    import serial
except ImportError:  # permite usar --simulate sem pyserial instalado
    serial = None


CSV_RE = re.compile(
    r"^TEL\s*,\s*"
    r"(?P<t_ms>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<yaw>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<ang_x>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<ang_y>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<ax>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<ay>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<az>-?\d+(?:\.\d+)?)\s*$"
)

ACCEL_RE = re.compile(
    r"Accel\s*:\s*X=\s*(?P<ax>-?\d+)\s*mg\s*"
    r"Y=\s*(?P<ay>-?\d+)\s*mg\s*"
    r"Z=\s*(?P<az>-?\d+)\s*mg",
    re.IGNORECASE,
)

ORIENT_RE = re.compile(
    r"Roll\s*:\s*(?P<roll>-?\d+)\s*deg\s*"
    r"Pitch\s*:\s*(?P<pitch>-?\d+)\s*deg\s*"
    r"Yaw\s*:\s*(?P<yaw>-?\d+)\s*deg",
    re.IGNORECASE,
)


@dataclass
class Sample:
    t_pc: float
    t_ms: float
    yaw_deg: float
    ang_x_deg: float  # no firmware atual, equivale ao roll
    ang_y_deg: float  # no firmware atual, equivale ao pitch
    acc_x_mg: float
    acc_y_mg: float
    acc_z_mg: float


class TelemetryParser:
    """Converte linhas recebidas da serial em amostras completas."""

    def __init__(self) -> None:
        self._last_accel: Optional[tuple[float, float, float]] = None
        self._t0 = time.time()

    def parse_line(self, line: str) -> Optional[Sample]:
        line = line.strip()
        if not line:
            return None

        # Formato recomendado: uma linha CSV por amostra.
        m = CSV_RE.match(line)
        if m:
            return Sample(
                t_pc=time.time(),
                t_ms=float(m.group("t_ms")),
                yaw_deg=float(m.group("yaw")),
                ang_x_deg=float(m.group("ang_x")),
                ang_y_deg=float(m.group("ang_y")),
                acc_x_mg=float(m.group("ax")),
                acc_y_mg=float(m.group("ay")),
                acc_z_mg=float(m.group("az")),
            )

        # Compatibilidade com o formato atual, em duas linhas separadas.
        m = ACCEL_RE.search(line)
        if m:
            self._last_accel = (
                float(m.group("ax")),
                float(m.group("ay")),
                float(m.group("az")),
            )
            return None

        m = ORIENT_RE.search(line)
        if m and self._last_accel is not None:
            ax, ay, az = self._last_accel
            return Sample(
                t_pc=time.time(),
                t_ms=(time.time() - self._t0) * 1000.0,
                yaw_deg=float(m.group("yaw")),
                ang_x_deg=float(m.group("roll")),
                ang_y_deg=float(m.group("pitch")),
                acc_x_mg=ax,
                acc_y_mg=ay,
                acc_z_mg=az,
            )

        return None


class SerialSource:
    def __init__(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial não está instalado. Rode: pip install pyserial")
        self.parser = TelemetryParser()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.01)
        self.ser.reset_input_buffer()
        print(f"Conectado em {port} a {baud} bps. Aguardando telemetria...")

    def read_samples(self) -> list[Sample]:
        samples: list[Sample] = []
        start = time.time()

        # Lê tudo que já chegou, mas sem travar a interface gráfica.
        while time.time() - start < 0.03:
            raw = self.ser.readline()
            if not raw:
                break
            line = raw.decode("utf-8", errors="ignore").strip()
            sample = self.parser.parse_line(line)
            if sample is not None:
                samples.append(sample)

        return samples


class SimulatedSource:
    """Fonte falsa para testar os gráficos sem a raquete conectada."""

    def __init__(self) -> None:
        self.t0 = time.time()
        self.last = 0.0

    def read_samples(self) -> list[Sample]:
        now = time.time() - self.t0
        samples: list[Sample] = []

        # Gera aproximadamente 20 Hz no modo simulado.
        while self.last + 0.05 <= now:
            self.last += 0.05
            t = self.last
            samples.append(
                Sample(
                    t_pc=time.time(),
                    t_ms=t * 1000.0,
                    yaw_deg=45.0 * math.sin(0.4 * t),
                    ang_x_deg=35.0 * math.sin(0.9 * t),
                    ang_y_deg=25.0 * math.cos(0.7 * t),
                    acc_x_mg=250.0 * math.sin(2.0 * t),
                    acc_y_mg=200.0 * math.cos(1.7 * t),
                    acc_z_mg=1000.0 + 120.0 * math.sin(1.2 * t),
                )
            )

        return samples


def rotation_matrix_from_euler(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Matriz de rotação usando roll X, pitch Y e yaw Z, em graus."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    rx = np.array(
        [[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]],
        dtype=float,
    )
    ry = np.array(
        [[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]],
        dtype=float,
    )
    rz = np.array(
        [[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]],
        dtype=float,
    )
    return rz @ ry @ rx


class Dashboard:
    def __init__(self, source, window_s: float, csv_path: Optional[Path]) -> None:
        self.source = source
        self.window_s = window_s
        self.samples: Deque[Sample] = deque(maxlen=5000)
        self.start_pc: Optional[float] = None
        self.last_message_time = 0.0

        self.csv_file = None
        self.csv_writer = None
        if csv_path is not None:
            self.csv_file = csv_path.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "t_pc",
                    "t_ms",
                    "yaw_deg",
                    "ang_x_roll_deg",
                    "ang_y_pitch_deg",
                    "acc_x_mg",
                    "acc_y_mg",
                    "acc_z_mg",
                ]
            )

        self.fig = plt.figure(figsize=(15, 8))
        self.fig.canvas.manager.set_window_title("Raquete instrumentada - telemetria em tempo real")
        grid = self.fig.add_gridspec(3, 3)

        self.axes = [
            self.fig.add_subplot(grid[0, 0]),
            self.fig.add_subplot(grid[0, 1]),
            self.fig.add_subplot(grid[1, 0]),
            self.fig.add_subplot(grid[1, 1]),
            self.fig.add_subplot(grid[2, 0]),
            self.fig.add_subplot(grid[2, 1]),
        ]
        self.ax3d = self.fig.add_subplot(grid[:, 2], projection="3d")

        self.labels = [
            ("Yaw", "graus"),
            ("Ângulo X / Roll", "graus"),
            ("Ângulo Y / Pitch", "graus"),
            ("Aceleração X", "mg"),
            ("Aceleração Y", "mg"),
            ("Aceleração Z", "mg"),
        ]
        self.lines = []
        for ax, (title, ylabel) in zip(self.axes, self.labels):
            (line,) = ax.plot([], [])
            self.lines.append(line)
            ax.set_title(title)
            ax.set_xlabel("tempo (s)")
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)

        self.status_text = self.fig.text(0.01, 0.01, "Sem dados ainda", fontsize=10)
        self.fig.tight_layout(rect=(0, 0.03, 1, 1))
        self._setup_3d_axes()

    def _setup_3d_axes(self) -> None:
        self.ax3d.set_title("Orientação 3D da raquete")
        self.ax3d.set_xlim(-1.2, 1.2)
        self.ax3d.set_ylim(-1.2, 1.2)
        self.ax3d.set_zlim(-1.2, 1.2)
        self.ax3d.set_xlabel("X")
        self.ax3d.set_ylabel("Y")
        self.ax3d.set_zlabel("Z")
        try:
            self.ax3d.set_box_aspect((1, 1, 1))
        except Exception:
            pass

    def _append_sample(self, sample: Sample) -> None:
        if self.start_pc is None:
            self.start_pc = sample.t_pc
        self.samples.append(sample)
        self.last_message_time = time.time()

        if self.csv_writer is not None:
            self.csv_writer.writerow(
                [
                    f"{sample.t_pc:.3f}",
                    f"{sample.t_ms:.1f}",
                    f"{sample.yaw_deg:.3f}",
                    f"{sample.ang_x_deg:.3f}",
                    f"{sample.ang_y_deg:.3f}",
                    f"{sample.acc_x_mg:.3f}",
                    f"{sample.acc_y_mg:.3f}",
                    f"{sample.acc_z_mg:.3f}",
                ]
            )
            self.csv_file.flush()

    def _current_arrays(self):
        if not self.samples:
            return None

        data = list(self.samples)
        t0 = data[0].t_pc if self.start_pc is None else self.start_pc
        t = np.array([s.t_pc - t0 for s in data], dtype=float)
        values = [
            np.array([s.yaw_deg for s in data], dtype=float),
            np.array([s.ang_x_deg for s in data], dtype=float),
            np.array([s.ang_y_deg for s in data], dtype=float),
            np.array([s.acc_x_mg for s in data], dtype=float),
            np.array([s.acc_y_mg for s in data], dtype=float),
            np.array([s.acc_z_mg for s in data], dtype=float),
        ]
        return t, values, data[-1]

    def _redraw_3d(self, sample: Sample) -> None:
        self.ax3d.cla()
        self._setup_3d_axes()

        # Modelo simples: uma placa retangular representando a face da raquete.
        racket = np.array(
            [
                [-0.45, -0.75, 0.0],
                [0.45, -0.75, 0.0],
                [0.45, 0.75, 0.0],
                [-0.45, 0.75, 0.0],
            ]
        )
        handle = np.array([[0.0, -0.75, 0.0], [0.0, -1.15, 0.0]])

        r = rotation_matrix_from_euler(
            roll_deg=sample.ang_x_deg,
            pitch_deg=sample.ang_y_deg,
            yaw_deg=sample.yaw_deg,
        )
        racket_rot = racket @ r.T
        handle_rot = handle @ r.T

        poly = Poly3DCollection([racket_rot], alpha=0.35)
        self.ax3d.add_collection3d(poly)
        self.ax3d.plot(handle_rot[:, 0], handle_rot[:, 1], handle_rot[:, 2], linewidth=4)

        # Eixos locais da raquete.
        origin = np.array([0.0, 0.0, 0.0])
        local_axes = np.eye(3) * 0.7
        rotated_axes = local_axes @ r.T
        for vec, label in zip(rotated_axes, ["X", "Y", "Z"]):
            self.ax3d.quiver(origin[0], origin[1], origin[2], vec[0], vec[1], vec[2], length=1.0)
            self.ax3d.text(vec[0], vec[1], vec[2], label)

        self.ax3d.text2D(
            0.02,
            0.95,
            f"Yaw={sample.yaw_deg:.1f}°\nX/Roll={sample.ang_x_deg:.1f}°\nY/Pitch={sample.ang_y_deg:.1f}°",
            transform=self.ax3d.transAxes,
        )

    def update(self, _frame):
        for sample in self.source.read_samples():
            self._append_sample(sample)

        arrays = self._current_arrays()
        if arrays is None:
            return self.lines

        t, values, last = arrays
        max_t = float(t[-1])
        min_t = max(0.0, max_t - self.window_s)

        for ax, line, y in zip(self.axes, self.lines, values):
            line.set_data(t, y)
            ax.set_xlim(min_t, max(self.window_s, max_t))

            mask = t >= min_t
            visible_y = y[mask] if mask.any() else y
            y_min = float(np.nanmin(visible_y))
            y_max = float(np.nanmax(visible_y))
            if y_min == y_max:
                y_min -= 1.0
                y_max += 1.0
            margin = max(1.0, 0.12 * (y_max - y_min))
            ax.set_ylim(y_min - margin, y_max + margin)

        self._redraw_3d(last)

        age = time.time() - self.last_message_time
        self.status_text.set_text(
            f"Amostras: {len(self.samples)} | último pacote há {age:.2f}s | "
            f"formato esperado: TEL,t_ms,yaw,ang_x,ang_y,acc_x,acc_y,acc_z"
        )
        return self.lines

    def run(self) -> None:
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=50, blit=False)
        try:
            plt.show()
        finally:
            if self.csv_file is not None:
                self.csv_file.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Dashboard de telemetria da raquete instrumentada")
    parser.add_argument("--port", help="Porta serial do rádio. Ex.: COM5 no Windows ou /dev/ttyUSB0 no Linux")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate. Padrão: 115200")
    parser.add_argument("--window", type=float, default=20.0, help="Janela de tempo exibida nos gráficos, em segundos")
    parser.add_argument("--csv", type=Path, help="Arquivo CSV para gravar os dados recebidos")
    parser.add_argument("--simulate", action="store_true", help="Roda com dados simulados, sem conectar na serial")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    if args.simulate:
        source = SimulatedSource()
    else:
        if not args.port:
            print(
                "Erro: informe a porta serial com --port ou use --simulate.\n"
                "Dica Windows: python -m serial.tools.list_ports",
                file=sys.stderr,
            )
            return 2
        source = SerialSource(args.port, args.baud)

    dashboard = Dashboard(source=source, window_s=args.window, csv_path=args.csv)
    dashboard.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
