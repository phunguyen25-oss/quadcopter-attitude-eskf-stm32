#!/usr/bin/env python3
"""Overlapping Allan deviation tối giản cho log gyro tĩnh."""

from __future__ import annotations
import argparse
import csv
from pathlib import Path
import numpy as np


def load(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8-sig")))
    t = np.array([float(r["t_s"]) for r in rows])
    w = np.array([[float(r[k]) for k in ("gx", "gy", "gz")] for r in rows])
    return t, w


def overlapping_adev(x: np.ndarray, dt: float, m_values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    # Allan variance cho rate data thông qua phase tích phân.
    phase = np.concatenate(([0.0], np.cumsum(x) * dt))
    taus, adev = [], []
    n = len(x)
    for m in m_values:
        if 2 * m >= n:
            continue
        d2 = phase[2*m:] - 2.0 * phase[m:-m] + phase[:-2*m]
        avar = np.mean(d2 * d2) / (2.0 * (m * dt) ** 2)
        taus.append(m * dt)
        adev.append(np.sqrt(avar))
    return np.asarray(taus), np.asarray(adev)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_file", type=Path)
    ap.add_argument("--plot", type=Path, default=Path("allan.png"))
    args = ap.parse_args()
    t, gyro = load(args.csv_file)
    dt_all = np.diff(t)
    dt = float(np.median(dt_all))
    jitter = float(np.std(dt_all) / dt)
    if jitter > 0.05:
        raise ValueError(f"Timestamp jitter {100*jitter:.2f}% > 5%; resample trước khi Allan")
    max_m = max(2, len(t) // 20)
    m_values = np.unique(np.logspace(0, np.log10(max_m), 60).astype(int))

    import matplotlib.pyplot as plt
    for axis, label in enumerate("XYZ"):
        tau, adev = overlapping_adev(gyro[:, axis], dt, m_values)
        plt.loglog(tau, adev, label=label)
        i1 = int(np.argmin(np.abs(tau - 1.0)))
        imin = int(np.argmin(adev))
        print(f"{label}: ADEV@1s={adev[i1]:.6g} rad/s, min={adev[imin]:.6g} tại tau={tau[imin]:.3g}s")
    plt.xlabel("tau [s]")
    plt.ylabel("Allan deviation [rad/s]")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.plot, dpi=160)
    print(f"dt={dt:.6f}s, jitter={100*jitter:.3f}%, plot={args.plot}")
    print("Lưu ý: ADEV không tự động biến thành Q; cần xác nhận quy ước noise-density và đơn vị.")


if __name__ == "__main__":
    main()

