"""Estimate a constant time offset between EuRoC IMU and ground truth.

The idea is simple:
- derive angular velocity from successive ground-truth orientations;
- interpolate that derived gyro signal onto a uniform time grid;
- interpolate the IMU gyro onto the same grid;
- estimate the constant offset by maximizing cross-correlation.

By default, the script correlates the gyro norm, which is robust to small frame
convention mismatches. Optionally, it can correlate the three gyro components.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import gtsam

import euroc


def _uniform_time_grid(t_min: float, t_max: float, dt: float) -> np.ndarray:
    """Create a uniform time grid on [t_min, t_max]."""
    n = int(np.floor((t_max - t_min) / dt)) + 1
    return t_min + dt * np.arange(n, dtype=float)


def _derive_gt_gyro(
    gt_t: np.ndarray, gt_states: list[gtsam.NavState]
) -> tuple[np.ndarray, np.ndarray]:
    """Derive body-frame angular velocity from successive GT orientations.

    Returns:
        t_mid: midpoint timestamps, shape (N-1,)
        omega: angular velocity in rad/s, shape (N-1, 3)
    """
    if len(gt_t) < 2:
        raise ValueError("Need at least two ground-truth states.")

    t_mid = []
    omega = []
    for i in range(len(gt_t) - 1):
        dt = gt_t[i + 1] - gt_t[i]
        if dt <= 0.0:
            continue
        R_i = gt_states[i].pose().rotation()
        R_j = gt_states[i + 1].pose().rotation()
        dtheta = gtsam.Rot3.Logmap(R_i.between(R_j))
        t_mid.append(0.5 * (gt_t[i] + gt_t[i + 1]))
        omega.append(dtheta / dt)

    if not t_mid:
        raise ValueError("No valid GT intervals with positive dt.")

    return np.asarray(t_mid), np.asarray(omega)


def _interp_vec3(t_src: np.ndarray, y_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """Linearly interpolate a 3D vector-valued signal."""
    return np.column_stack([np.interp(t_dst, t_src, y_src[:, k]) for k in range(3)])


def _interp_scalar(
    t_src: np.ndarray, y_src: np.ndarray, t_dst: np.ndarray
) -> np.ndarray:
    """Linearly interpolate a scalar signal."""
    return np.interp(t_dst, t_src, y_src)


def _zscore(x: np.ndarray) -> np.ndarray:
    """Return a standardized copy of x."""
    x = np.asarray(x, dtype=float)
    s = x.std()
    if s == 0.0:
        return x - x.mean()
    return (x - x.mean()) / s


def _best_offset_from_correlation(
    ref: np.ndarray, sig: np.ndarray, dt: float, max_offset: float
) -> tuple[float, float]:
    """Find the offset that maximizes discrete cross-correlation.

    Args:
        ref: reference signal sampled on a uniform grid
        sig: signal to shift relative to ref, same shape as ref
        dt: sample interval in seconds
        max_offset: search radius in seconds

    Returns:
        best_offset: time shift in seconds such that sig(t + best_offset)
            best aligns with ref(t)
        best_score: normalized correlation score at the best lag
    """
    if ref.shape != sig.shape:
        raise ValueError("ref and sig must have the same shape")

    max_lag = int(round(max_offset / dt))
    if ref.ndim == 1:
        corr = np.correlate(sig, ref, mode="full")
    else:
        corr = sum(
            np.correlate(sig[:, k], ref[:, k], mode="full") for k in range(ref.shape[1])
        )

    lags = np.arange(-len(ref) + 1, len(ref))
    keep = np.abs(lags) <= max_lag
    corr = corr[keep]
    lags = lags[keep]

    i_best = int(np.argmax(corr))
    best_lag = int(lags[i_best])
    best_offset = -best_lag * dt

    denom = float(ref.shape[0] * (ref.shape[1] if ref.ndim == 2 else 1))
    best_score = float(corr[i_best] / denom)
    return best_offset, best_score


def estimate_euroc_offset(
    gt_csv: str | Path,
    imu_csv: str | Path,
    dt: float = 0.002,
    max_offset: float = 0.2,
    use_components: bool = False,
    t_start: float | None = None,
    t_end: float | None = None,
) -> tuple[float, float]:
    """Estimate the constant IMU-vs-GT time offset.

    Args:
        gt_csv: EuRoC ground-truth CSV path
        imu_csv: EuRoC IMU CSV path
        dt: uniform resampling interval in seconds
        max_offset: search window in seconds on either side of zero
        use_components: if True, correlate 3-axis gyro directly; otherwise use norms
        t_start: optional start time in seconds for the fitting window
        t_end: optional end time in seconds for the fitting window

    Returns:
        offset_s: estimated offset in seconds to add to IMU timestamps
        score: correlation score at the optimum
    """
    gt_t, gt_states, _, _ = euroc.read_ground_truth(gt_csv)
    imu_df, imu_t = euroc.read_imu(imu_csv)

    gt_gyro_t, gt_gyro = _derive_gt_gyro(gt_t, gt_states)
    imu_gyro = imu_df[[euroc.IMU_COLS["w_x"], euroc.IMU_COLS["w_y"], euroc.IMU_COLS["w_z"]]].to_numpy(
        dtype=float
    )

    t_lo = max(gt_gyro_t[0], imu_t[0])
    t_hi = min(gt_gyro_t[-1], imu_t[-1])
    if t_start is not None:
        t_lo = max(t_lo, t_start)
    if t_end is not None:
        t_hi = min(t_hi, t_end)
    if t_hi <= t_lo:
        raise ValueError("Empty overlap between GT and IMU time ranges.")

    grid = _uniform_time_grid(t_lo, t_hi, dt)
    gt_gyro_u = _interp_vec3(gt_gyro_t, gt_gyro, grid)
    imu_gyro_u = _interp_vec3(imu_t, imu_gyro, grid)

    if use_components:
        ref = np.column_stack([_zscore(gt_gyro_u[:, k]) for k in range(3)])
        sig = np.column_stack([_zscore(imu_gyro_u[:, k]) for k in range(3)])
    else:
        ref = _zscore(np.linalg.norm(gt_gyro_u, axis=1))
        sig = _zscore(np.linalg.norm(imu_gyro_u, axis=1))

    return _best_offset_from_correlation(ref, sig, dt, max_offset)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Estimate a constant EuRoC IMU/Ground-Truth time offset from gyro."
    )
    euroc.add_dataset_args(parser)
    parser.add_argument(
        "--dt",
        type=float,
        default=0.002,
        help="Uniform resampling interval in seconds (default: 0.002)",
    )
    parser.add_argument(
        "--max_offset",
        type=float,
        default=0.2,
        help="Search radius in seconds around zero (default: 0.2)",
    )
    parser.add_argument(
        "--use_components",
        action="store_true",
        help="Correlate 3-axis gyro directly instead of gyro norm",
    )
    parser.add_argument(
        "--t_start",
        type=float,
        default=None,
        help="Optional start time in seconds for the fitting window",
    )
    parser.add_argument(
        "--t_end",
        type=float,
        default=None,
        help="Optional end time in seconds for the fitting window",
    )

    args = parser.parse_args()

    _, gt_csv, imu_csv = euroc.csv_paths(args.root, args.seq, args.run)

    offset_s, score = estimate_euroc_offset(
        gt_csv=gt_csv,
        imu_csv=imu_csv,
        dt=args.dt,
        max_offset=args.max_offset,
        use_components=args.use_components,
        t_start=args.t_start,
        t_end=args.t_end,
    )

    print(f"estimated offset to add to IMU timestamps: {offset_s:.6f} s")
    print(f"correlation score: {score:.6f}")


if __name__ == "__main__":
    main()
