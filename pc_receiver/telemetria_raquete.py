"""
Dashboard de telemetria da raquete instrumentada.

Roda no computador, não no STM32.

Recebe dois tipos de mensagem pela serial/radio:

1) IMU/orientação/aceleração:
$RAQ,RAQ01,t_ms,yaw_deg,roll_deg,pitch_deg,acc_x_mg,acc_y_mg,acc_z_mg

2) Impacto/heatmap dos piezos:
$HIT,RAQ01,t_ms,regiao,valor_pico,h0,h1,h2,h3,h4,h5,h6,h7,h8

O Python ignora qualquer linha que não comece com $RAQ ou $HIT.

Teste sem raquete:
python telemetria_raquete.py --mock

Também aceita o alias antigo:
python telemetria_raquete.py --simulate
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
from typing import Deque, Optional, Union

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import gridspec
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    import serial
except ImportError:
    serial = None


RAQ_RE = re.compile(
    r"^\$RAQ\s*,\s*"
    r"(?P<device>[A-Za-z0-9_-]+)\s*,\s*"
    r"(?P<t_ms>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<yaw>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<roll>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<pitch>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<ax>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<ay>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<az>-?\d+(?:\.\d+)?)\s*$"
)

# Aceita os dois formatos:
# curto:   $HIT,RAQ01,t_ms,regiao,valor_pico
# completo:$HIT,RAQ01,t_ms,regiao,valor_pico,h0,h1,h2,h3,h4,h5,h6,h7,h8
HIT_RE = re.compile(
    r"^\$HIT\s*,\s*"
    r"(?P<device>[A-Za-z0-9_-]+)\s*,\s*"
    r"(?P<t_ms>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<region>[0-8])\s*,\s*"
    r"(?P<peak>-?\d+(?:\.\d+)?)"
    r"(?P<counts>(?:\s*,\s*-?\d+(?:\.\d+)?){0,9})\s*$"
)

# Compatibilidade com o formato inicial, caso alguém ainda use TEL.
OLD_TEL_RE = re.compile(
    r"^TEL\s*,\s*"
    r"(?P<t_ms>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<yaw>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<roll>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<pitch>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<ax>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<ay>-?\d+(?:\.\d+)?)\s*,\s*"
    r"(?P<az>-?\d+(?:\.\d+)?)\s*$"
)


@dataclass
class ImuSample:
    device_id: str
    t_pc: float
    t_ms: float
    yaw_deg: float
    roll_deg: float
    pitch_deg: float
    acc_x_mg: float
    acc_y_mg: float
    acc_z_mg: float


@dataclass
class HitEvent:
    device_id: str
    t_pc: float
    t_ms: float
    region: int
    peak_value: float
    counts: Optional[list[float]] = None


TelemetryMessage = Union[ImuSample, HitEvent]


class TelemetryParser:
    def __init__(self, expected_id: Optional[str] = "RAQ01", accept_old_tel: bool = True) -> None:
        self.expected_id = expected_id
        self.accept_old_tel = accept_old_tel

    def _id_ok(self, device_id: str) -> bool:
        return (self.expected_id is None) or (device_id == self.expected_id)

    def parse_line(self, line: str) -> Optional[TelemetryMessage]:
        line = line.strip()
        if not line:
            return None

        match = RAQ_RE.match(line)
        if match:
            device_id = match.group("device")
            if not self._id_ok(device_id):
                return None
            return ImuSample(
                device_id=device_id,
                t_pc=time.time(),
                t_ms=float(match.group("t_ms")),
                yaw_deg=float(match.group("yaw")),
                roll_deg=float(match.group("roll")),
                pitch_deg=float(match.group("pitch")),
                acc_x_mg=float(match.group("ax")),
                acc_y_mg=float(match.group("ay")),
                acc_z_mg=float(match.group("az")),
            )

        match = HIT_RE.match(line)
        if match:
            device_id = match.group("device")
            if not self._id_ok(device_id):
                return None

            counts_raw = match.group("counts") or ""
            counts: Optional[list[float]] = None
            if counts_raw:
                # Remove a vírgula inicial e converte. Só usa se vierem exatamente 9 campos.
                parts = [p.strip() for p in counts_raw.split(",") if p.strip()]
                if len(parts) == 9:
                    counts = [float(p) for p in parts]

            return HitEvent(
                device_id=device_id,
                t_pc=time.time(),
                t_ms=float(match.group("t_ms")),
                region=int(match.group("region")),
                peak_value=float(match.group("peak")),
                counts=counts,
            )

        if self.accept_old_tel:
            match = OLD_TEL_RE.match(line)
            if match:
                return ImuSample(
                    device_id="TEL",
                    t_pc=time.time(),
                    t_ms=float(match.group("t_ms")),
                    yaw_deg=float(match.group("yaw")),
                    roll_deg=float(match.group("roll")),
                    pitch_deg=float(match.group("pitch")),
                    acc_x_mg=float(match.group("ax")),
                    acc_y_mg=float(match.group("ay")),
                    acc_z_mg=float(match.group("az")),
                )

        # Qualquer printf humano cai aqui e é ignorado.
        return None


class SerialSource:
    def __init__(self, port: str, baud: int, parser: TelemetryParser) -> None:
        if serial is None:
            raise RuntimeError("pyserial não está instalado. Rode: pip install pyserial")
        self.parser = parser
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.ser.reset_input_buffer()
        print(f"Conectado em {port} a {baud} bps. Aguardando mensagens $RAQ/$HIT...")

    def read_messages(self) -> list[TelemetryMessage]:
        messages: list[TelemetryMessage] = []
        start = time.time()
        while time.time() - start < 0.08:
            raw = self.ser.readline()
            if not raw:
                break
            line = raw.decode("utf-8", errors="ignore").strip()
            message = self.parser.parse_line(line)
            if message:
                messages.append(message)
        return messages


class MockSource:
    """Fonte falsa para testar sem raquete e sem rádio."""

    def __init__(self, device_id: str = "RAQ01", imu_hz: float = 20.0) -> None:
        self.device_id = device_id
        self.imu_period_s = 1.0 / imu_hz
        self.t0 = time.time()
        self.last_imu = 0.0
        self.last_hit_bucket = -1
        self.parser = TelemetryParser(expected_id=device_id)
        self.hit_counts = [0] * 9

    def make_raq_line(self, now: float) -> str:
        t_ms = int(now * 1000)

        yaw = int(35.0 * math.sin(0.45 * now))
        roll = int(28.0 * math.sin(1.20 * now))
        pitch = int(22.0 * math.cos(0.90 * now))

        ax = int(450.0 * math.sin(2.10 * now))
        ay = int(280.0 * math.cos(1.60 * now))
        az = int(980.0 + 100.0 * math.sin(3.30 * now))

        # Simula uma batida de vez em quando para dar picos nos gráficos.
        hit_phase = now % 4.0
        if 1.15 < hit_phase < 1.30:
            ax += 900
            ay -= 500
            az += 350

        return f"$RAQ,{self.device_id},{t_ms},{yaw},{roll},{pitch},{ax},{ay},{az}"

    def maybe_make_hit_line(self, now: float) -> Optional[str]:
        # Gera um impacto falso a cada ~2,2 s, mudando a região para preencher o heatmap.
        bucket = int(now / 2.2)
        if bucket == self.last_hit_bucket:
            return None
        self.last_hit_bucket = bucket

        # Região pseudoaleatória, mas reprodutível e espalhada pela matriz 3x3.
        region = (bucket * 4 + bucket // 2) % 9
        peak = int(1300 + 850 * abs(math.sin(1.7 * now)))
        self.hit_counts[region] += 1
        t_ms = int(now * 1000)
        counts = ",".join(str(v) for v in self.hit_counts)
        return f"$HIT,{self.device_id},{t_ms},{region},{peak},{counts}"

    def read_messages(self) -> list[TelemetryMessage]:
        messages: list[TelemetryMessage] = []
        now_abs = time.time()
        now = now_abs - self.t0

        while self.last_imu + self.imu_period_s <= now:
            self.last_imu += self.imu_period_s
            line = self.make_raq_line(self.last_imu)
            message = self.parser.parse_line(line)
            if message:
                messages.append(message)

        hit_line = self.maybe_make_hit_line(now)
        if hit_line:
            message = self.parser.parse_line(hit_line)
            if message:
                messages.append(message)

        return messages


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Matriz Rz(yaw) * Ry(pitch) * Rx(roll), ângulos em graus."""
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


def transform_points(points: np.ndarray, roll: float, pitch: float, yaw: float) -> np.ndarray:
    R = rotation_matrix(roll, pitch, yaw)
    return points @ R.T


class Dashboard:
    def __init__(self, source, csv_path: Optional[Path] = None, window: int = 300) -> None:
        self.source = source
        self.samples: Deque[ImuSample] = deque(maxlen=window)
        self.hit_events: Deque[HitEvent] = deque(maxlen=200)
        self.heatmap_counts = np.zeros((3, 3), dtype=float)
        self.last_hit_text = "Nenhum impacto recebido ainda"

        self.csv_file = None
        self.csv_writer = None
        if csv_path:
            self.csv_file = csv_path.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "tipo",
                    "device_id",
                    "t_ms",
                    "yaw_deg",
                    "roll_deg",
                    "pitch_deg",
                    "acc_x_mg",
                    "acc_y_mg",
                    "acc_z_mg",
                    "hit_region",
                    "hit_peak",
                    "heat_h0",
                    "heat_h1",
                    "heat_h2",
                    "heat_h3",
                    "heat_h4",
                    "heat_h5",
                    "heat_h6",
                    "heat_h7",
                    "heat_h8",
                ]
            )

        self.fig = plt.figure(figsize=(16, 8))
        self.fig.suptitle("Raquete instrumentada - telemetria + heatmap")
        gs = gridspec.GridSpec(3, 4, figure=self.fig, width_ratios=[1.1, 1.1, 0.9, 1.2])

        labels = [
            "Yaw (°)",
            "Roll / Ângulo X (°)",
            "Pitch / Ângulo Y (°)",
            "Aceleração X (mg)",
            "Aceleração Y (mg)",
            "Aceleração Z (mg)",
        ]

        positions = [(0, 0), (0, 1), (1, 0), (1, 1), (2, 0), (2, 1)]
        self.axes = []
        self.lines = []
        for label, (row, col) in zip(labels, positions):
            ax = self.fig.add_subplot(gs[row, col])
            ax.set_title(label)
            ax.grid(True, alpha=0.3)
            line, = ax.plot([], [], linewidth=1.5)
            self.axes.append(ax)
            self.lines.append(line)

        self.ax_heat = self.fig.add_subplot(gs[0:2, 2])
        self.ax_heat.set_title("Heatmap dos piezos")
        self.heat_img = self.ax_heat.imshow(self.heatmap_counts, vmin=0, vmax=1, origin="upper")
        self.ax_heat.set_xticks([0, 1, 2])
        self.ax_heat.set_yticks([0, 1, 2])
        self.ax_heat.set_xticklabels(["0", "1", "2"])
        self.ax_heat.set_yticklabels(["0", "1", "2"])
        self.heat_texts = []
        for r in range(3):
            row_texts = []
            for c in range(3):
                txt = self.ax_heat.text(c, r, "0", ha="center", va="center")
                row_texts.append(txt)
            self.heat_texts.append(row_texts)

        self.ax_hit_info = self.fig.add_subplot(gs[2, 2])
        self.ax_hit_info.axis("off")
        self.hit_info_artist = self.ax_hit_info.text(0.0, 0.75, self.last_hit_text, va="top")

        self.ax3d = self.fig.add_subplot(gs[:, 3], projection="3d")
        self.ax3d.set_title("Orientação 3D - raquete")

        plt.tight_layout()

    def add_message(self, message: TelemetryMessage) -> None:
        if isinstance(message, ImuSample):
            self.samples.append(message)
            if self.csv_writer:
                self.csv_writer.writerow(
                    [
                        "RAQ",
                        message.device_id,
                        message.t_ms,
                        message.yaw_deg,
                        message.roll_deg,
                        message.pitch_deg,
                        message.acc_x_mg,
                        message.acc_y_mg,
                        message.acc_z_mg,
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                    ]
                )
            return

        if isinstance(message, HitEvent):
            self.hit_events.append(message)
            if message.counts and len(message.counts) == 9:
                self.heatmap_counts = np.array(message.counts, dtype=float).reshape(3, 3)
            else:
                row = message.region // 3
                col = message.region % 3
                self.heatmap_counts[row, col] += 1

            self.last_hit_text = (
                f"Último impacto\n"
                f"ID: {message.device_id}\n"
                f"t = {message.t_ms:.0f} ms\n"
                f"região = {message.region}\n"
                f"pico = {message.peak_value:.0f}"
            )

            if self.csv_writer:
                flat = self.heatmap_counts.reshape(-1).tolist()
                self.csv_writer.writerow(
                    [
                        "HIT",
                        message.device_id,
                        message.t_ms,
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        message.region,
                        message.peak_value,
                        *flat,
                    ]
                )

    def update(self, _frame):
        for message in self.source.read_messages():
            self.add_message(message)

        if self.samples:
            self.update_graphs()
            self.update_3d(self.samples[-1])
        else:
            self.update_3d(None)

        self.update_heatmap()
        return self.lines

    def update_graphs(self) -> None:
        data = list(self.samples)
        t = np.array([(s.t_ms - data[0].t_ms) / 1000.0 for s in data])
        values = [
            [s.yaw_deg for s in data],
            [s.roll_deg for s in data],
            [s.pitch_deg for s in data],
            [s.acc_x_mg for s in data],
            [s.acc_y_mg for s in data],
            [s.acc_z_mg for s in data],
        ]

        for ax, line, y in zip(self.axes, self.lines, values):
            line.set_data(t, y)
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(max(0, t[-1] - 15), max(15, t[-1] + 0.5))

    def update_heatmap(self) -> None:
        max_count = max(1.0, float(np.max(self.heatmap_counts)))
        self.heat_img.set_data(self.heatmap_counts)
        self.heat_img.set_clim(vmin=0, vmax=max_count)

        for r in range(3):
            for c in range(3):
                value = int(self.heatmap_counts[r, c])
                self.heat_texts[r][c].set_text(str(value))

        self.hit_info_artist.set_text(self.last_hit_text)

    def update_3d(self, sample: Optional[ImuSample]) -> None:
        self.ax3d.cla()
        self.ax3d.set_title("Orientação 3D - raquete de ping-pong")
        self.ax3d.set_xlim(-1.2, 1.2)
        self.ax3d.set_ylim(-1.2, 1.2)
        self.ax3d.set_zlim(-1.2, 1.2)
        self.ax3d.set_xlabel("X")
        self.ax3d.set_ylabel("Y")
        self.ax3d.set_zlabel("Z")

        if sample is None:
            roll = pitch = yaw = 0.0
            device_id = "sem dados"
        else:
            roll = sample.roll_deg
            pitch = sample.pitch_deg
            yaw = sample.yaw_deg
            device_id = sample.device_id

        # Face circular da raquete de ping-pong no plano XY local.
        theta = np.linspace(0, 2 * math.pi, 64)
        radius = 0.42
        center_y = 0.28
        circle = np.column_stack(
            [
                radius * np.cos(theta),
                center_y + radius * np.sin(theta),
                np.zeros_like(theta),
            ]
        )

        # Cabo da raquete, também no plano XY local.
        handle = np.array(
            [
                [-0.10, -1.00, 0.0],
                [0.10, -1.00, 0.0],
                [0.13, -0.05, 0.0],
                [-0.13, -0.05, 0.0],
            ],
            dtype=float,
        )

        # Pequena espessura visual.
        z_offset = 0.035
        circle_front = transform_points(circle + np.array([0, 0, z_offset]), roll, pitch, yaw)
        circle_back = transform_points(circle - np.array([0, 0, z_offset]), roll, pitch, yaw)
        handle_front = transform_points(handle + np.array([0, 0, z_offset]), roll, pitch, yaw)
        handle_back = transform_points(handle - np.array([0, 0, z_offset]), roll, pitch, yaw)

        self.ax3d.add_collection3d(Poly3DCollection([circle_front], alpha=0.45))
        self.ax3d.add_collection3d(Poly3DCollection([circle_back], alpha=0.20))
        self.ax3d.add_collection3d(Poly3DCollection([handle_front], alpha=0.55))
        self.ax3d.add_collection3d(Poly3DCollection([handle_back], alpha=0.25))

        # Contorno da face e do cabo para ficar mais reconhecível.
        self.ax3d.plot(circle_front[:, 0], circle_front[:, 1], circle_front[:, 2], linewidth=1.5)
        closed_handle = np.vstack([handle_front, handle_front[0]])
        self.ax3d.plot(closed_handle[:, 0], closed_handle[:, 1], closed_handle[:, 2], linewidth=1.5)

        # Linhas 3x3 sobre a face, alinhadas com a ideia dos 9 piezos.
        for x in [-radius / 3, radius / 3]:
            line = np.array([[x, center_y - radius, z_offset * 1.3], [x, center_y + radius, z_offset * 1.3]])
            rot = transform_points(line, roll, pitch, yaw)
            self.ax3d.plot(rot[:, 0], rot[:, 1], rot[:, 2], linewidth=0.8)
        for yline in [center_y - radius / 3, center_y + radius / 3]:
            line = np.array([[-radius, yline, z_offset * 1.3], [radius, yline, z_offset * 1.3]])
            rot = transform_points(line, roll, pitch, yaw)
            self.ax3d.plot(rot[:, 0], rot[:, 1], rot[:, 2], linewidth=0.8)

        # Eixos locais da raquete.
        origin = np.array([0.0, 0.0, 0.0])
        R = rotation_matrix(roll, pitch, yaw)
        local_axes = np.eye(3) @ R.T
        self.ax3d.quiver(*origin, *local_axes[0], length=0.55, normalize=True)
        self.ax3d.quiver(*origin, *local_axes[1], length=0.55, normalize=True)
        self.ax3d.quiver(*origin, *local_axes[2], length=0.55, normalize=True)

        self.ax3d.text2D(
            0.02,
            0.02,
            f"{device_id}\nyaw={yaw:.0f}°  roll={roll:.0f}°  pitch={pitch:.0f}°",
            transform=self.ax3d.transAxes,
        )

    def run(self) -> None:
        # IMPORTANTE:
        # O objeto da animação precisa ficar guardado em uma variável viva.
        # Se criarmos FuncAnimation sem salvar a referência, o Python pode apagar
        # a animação da memória e a janela abre sem atualizar os dados.
        self.anim = animation.FuncAnimation(
            self.fig,
            self.update,
            interval=50,
            blit=False,
            cache_frame_data=False,
        )

        try:
            plt.show()
        finally:
            if self.csv_file:
                self.csv_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Recebedor de telemetria da raquete instrumentada")
    parser.add_argument("--port", help="Porta serial. Ex.: COM5 ou /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate da serial")
    parser.add_argument("--id", default="RAQ01", help="ID esperado da raquete. Use --id '' para aceitar qualquer ID")
    parser.add_argument(
        "--mock",
        "--simulate",
        dest="mock",
        action="store_true",
        help="Usa dados falsos, sem raquete e sem rádio",
    )
    parser.add_argument("--csv", type=Path, help="Salva as amostras e impactos recebidos em CSV")
    args = parser.parse_args()

    expected_id = args.id if args.id else None

    if args.mock:
        source = MockSource(device_id=expected_id or "RAQ01")
        print("Modo mock ligado: gerando $RAQ e $HIT falsos.")
    else:
        if not args.port:
            print("Erro: informe --port COMx ou use --mock para testar sem raquete.", file=sys.stderr)
            return 2
        source = SerialSource(args.port, args.baud, TelemetryParser(expected_id=expected_id))

    Dashboard(source, csv_path=args.csv).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
