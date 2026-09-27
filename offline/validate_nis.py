#!/usr/bin/env python3
"""Kiểm tra NIS accel và tính trắng của innovation."""

from __future__ import annotations
import argparse
import csv
from pathlib import Path
import numpy as np

CHI2_DOF3_999 = 16.266


def autocorrelation(x: np.ndarray, max_lag: int) -> np.ndarray:
    x = x - np.mean(x)
    den = np.dot(x, x)
    if den <= 0:
        return np.zeros(max_lag + 1)
    return np.array([np.dot(x[:-k] if k else x, x[k:]) / den for k in range(max_lag + 1)])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_file", type=Path)
    ap.add_argument("--gate", type=float, default=CHI2_DOF3_999)
    args = ap.parse_args()
    rows = list(csv.DictReader(args.csv_file.open(newline="", encoding="utf-8-sig")))
    nis = np.array([float(r["nis_acc"]) for r in rows if r.get("nis_acc", "")])
    if len(nis) < 100:
        raise ValueError("Cần ít nhất 100 mẫu NIS")
    inside = float(np.mean(nis <= args.gate))
    print(f"N={len(nis)}, mean NIS={np.mean(nis):.3f} (kỳ vọng xấp xỉ dof=3)")
    print(f"NIS <= {args.gate:.3f}: {100*inside:.2f}% (mục tiêu gần 99.9%, không dùng gate để che Q/R sai)")

    keys = [k for k in ("innov_x", "innov_y", "innov_z") if rows[0].get(k, "") != ""]
    bound = 1.96 / np.sqrt(len(rows))
    for key in keys:
        x = np.array([float(r[key]) for r in rows])
        ac = autocorrelation(x, 50)
        bad = int(np.sum(np.abs(ac[1:]) > bound))
        print(f"{key}: mean={np.mean(x):+.4g}, lag vượt biên 95%={bad}/50")


if __name__ == "__main__":
    main()
