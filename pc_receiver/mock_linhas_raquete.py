"""Imprime no terminal linhas falsas no formato da telemetria da raquete.

Gera mensagens $RAQ continuamente e mensagens $HIT de tempos em tempos.
"""

from __future__ import annotations

import argparse
import math
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", default="RAQ01")
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    t0 = time.time()
    period = 1.0 / args.hz
    hit_counts = [0] * 9
    last_hit_bucket = -1

    while True:
        now = time.time() - t0
        t_ms = int(now * 1000)
        yaw = int(35.0 * math.sin(0.45 * now))
        roll = int(28.0 * math.sin(1.20 * now))
        pitch = int(22.0 * math.cos(0.90 * now))
        ax = int(450.0 * math.sin(2.10 * now))
        ay = int(280.0 * math.cos(1.60 * now))
        az = int(980.0 + 100.0 * math.sin(3.30 * now))

        print(f"$RAQ,{args.id},{t_ms},{yaw},{roll},{pitch},{ax},{ay},{az}", flush=True)

        bucket = int(now / 2.2)
        if bucket != last_hit_bucket:
            last_hit_bucket = bucket
            region = (bucket * 4 + bucket // 2) % 9
            peak = int(1300 + 850 * abs(math.sin(1.7 * now)))
            hit_counts[region] += 1
            counts = ",".join(str(v) for v in hit_counts)
            print(f"$HIT,{args.id},{t_ms},{region},{peak},{counts}", flush=True)

        time.sleep(period)


if __name__ == "__main__":
    raise SystemExit(main())
