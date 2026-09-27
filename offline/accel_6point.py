#!/usr/bin/env python3
"""Hiệu chuẩn accel 6 mặt, mô hình độc lập từng trục.

calibrated = (raw_si - bias) * scale

Mô hình này sửa bias và scale nhưng không sửa misalignment chéo trục.
"""

from __future__ import annotations
import argparse
import csv
from pathlib import Path
import numpy as np

G = 9.80665
POSES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")


def load_csv(path: Path) -> dict[str, list[np.ndarray]]:
    data = {p: [] for p in POSES}
    with path.open(newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            pose = row["pose"].strip().upper()
            if pose not in data:
                continue
            data[pose].append(np.array([float(row["ax"]), float(row["ay"]), float(row["az"])]))
    missing = [p for p, samples in data.items() if not samples]
    if missing:
        raise ValueError(f"Thiếu tư thế: {', '.join(missing)}")
    return data


def solve(data: dict[str, list[np.ndarray]]) -> tuple[np.ndarray, np.ndarray, float]:
    means = {p: np.mean(np.vstack(v), axis=0) for p, v in data.items()}
    bias = np.zeros(3)
    scale = np.zeros(3)
    for i, axis in enumerate("XYZ"):
        plus, minus = means[f"+{axis}"][i], means[f"-{axis}"][i]
        bias[i] = 0.5 * (plus + minus)
        span = plus - minus
        if abs(span) < 0.5 * G:
            raise ValueError(f"Span trục {axis} quá nhỏ; kiểm tra nhãn tư thế/đơn vị")
        scale[i] = (2.0 * G) / span

    errors = []
    expected = {
        "+X": np.array([G, 0, 0]), "-X": np.array([-G, 0, 0]),
        "+Y": np.array([0, G, 0]), "-Y": np.array([0, -G, 0]),
        "+Z": np.array([0, 0, G]), "-Z": np.array([0, 0, -G]),
    }
    for pose, samples in data.items():
        calibrated = (np.vstack(samples) - bias) * scale
        errors.extend((calibrated - expected[pose]).ravel())
    return bias, scale, float(np.sqrt(np.mean(np.square(errors))))


def write_header(path: Path, bias: np.ndarray, scale: np.ndarray) -> None:
    text = f"""#ifndef GENERATED_IMU_CALIBRATION_H
#define GENERATED_IMU_CALIBRATION_H

/* Sinh tự động bởi offline/accel_6point.py. Đơn vị bias: m/s^2. */
#define IMU_ACC_BIAS_X ({bias[0]:.9g}f)
#define IMU_ACC_BIAS_Y ({bias[1]:.9g}f)
#define IMU_ACC_BIAS_Z ({bias[2]:.9g}f)
#define IMU_ACC_SCALE_X ({scale[0]:.9g}f)
#define IMU_ACC_SCALE_Y ({scale[1]:.9g}f)
#define IMU_ACC_SCALE_Z ({scale[2]:.9g}f)

#endif
"""
    path.write_text(text, encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_file", type=Path)
    ap.add_argument("--output", type=Path, default=Path("generated_imu_calibration.h"))
    args = ap.parse_args()
    bias, scale, rms = solve(load_csv(args.csv_file))
    write_header(args.output, bias, scale)
    print("bias [m/s^2] :", bias)
    print("scale         :", scale)
    print(f"RMS residual  : {rms:.6f} m/s^2")
    print("PASS" if rms < 0.02 else "CHƯA ĐẠT: kiểm tra mặt phẳng, nhãn pose và rung")
    print("Đã ghi:", args.output)


if __name__ == "__main__":
    main()

