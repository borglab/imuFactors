"""\
Combine EuRoC IMU and ground-truth CSV files into a single, time-aligned dataset.

This script:
- Loads EuRoC ground-truth and IMU CSV files.
- Constructs NavState objects from ground truth.
- Interpolates ground truth onto IMU timestamps using:
      X = X0 * Exp(alpha * log(X0^{-1} X1))
  implemented via:
      xi = X0.localCoordinates(X1)
      X  = X0.compose(Exp(alpha * xi))
- Linearly interpolates IMU biases.
- Writes a combined CSV with state, biases, and raw IMU.

Usage (CLI):
    python combine_euroc_files.py --root euroc --seq V2 --run 03

This will read:
    <root>/<seq>/<run>/state_groundtruth_estimate0/data.csv
    <root>/<seq>/<run>/imu0/data.csv

and write:
    <root>_<seq><run>.csv
"""

from __future__ import annotations

from pathlib import Path
import pandas as pd
import argparse

import euroc
import gtsam


def load_and_interpolate_euroc(
    gt_csv: str | Path,
    imu_csv: str | Path,
    out_csv: str | Path,
    t_start: float | None = None,
    t_end: float | None = None,
    t_offset: float = 0.0,
) -> pd.DataFrame:
    """
    Load EuRoC GT + IMU, interpolate GT onto IMU timestamps, and save combined CSV.
    """
    gt_t, gt_states, gt_bw, gt_ba = euroc.read_ground_truth(gt_csv)
    imu, _ = euroc.read_imu(imu_csv)

    # Apply timestamp offset (seconds)
    if t_offset != 0.0:
        imu[euroc.IMU_COLS["t"]] = imu[euroc.IMU_COLS["t"]] + t_offset * 1e9

    # Optional cropping based on IMU timestamps
    if t_start is not None or t_end is not None:
        t_ns = imu[euroc.IMU_COLS["t"]]
        mask = pd.Series(True, index=imu.index)
        if t_start is not None:
            mask &= (t_ns * 1e-9) >= t_start
        if t_end is not None:
            mask &= (t_ns * 1e-9) <= t_end
        imu = imu[mask]

    rows = []
    i = 0
    n = len(gt_t)

    for _, r in imu.iterrows():
        t = float(r[euroc.IMU_COLS["t"]]) * 1e-9

        # advance lower bound i until gt_t[i] <= t <= gt_t[i+1]
        while i + 1 < n and gt_t[i + 1] < t:
            i += 1

        # clamp to ends
        if t <= gt_t[0]:
            X = gt_states[0]
            bw = gt_bw[0]
            ba = gt_ba[0]
        elif t >= gt_t[-1]:
            X = gt_states[-1]
            bw = gt_bw[-1]
            ba = gt_ba[-1]
        else:
            X0, X1 = gt_states[i], gt_states[i + 1]
            t0, t1 = gt_t[i], gt_t[i + 1]
            alpha = (t - t0) / (t1 - t0)

            # Explicit NavState interpolation:
            # X = X0 * Exp(alpha * localCoordinates(X0, X1))
            xi = X0.logmap(X1)
            X = X0.expmap(alpha * xi)

            # Euclidean interpolation for biases
            bw = (1.0 - alpha) * gt_bw[i] + alpha * gt_bw[i + 1]
            ba = (1.0 - alpha) * gt_ba[i] + alpha * gt_ba[i + 1]

        pose = X.pose()
        q = pose.rotation().toQuaternion()  # returns quaternion object
        p = pose.translation()
        v = X.velocity()

        rows.append(
            {
                "t": t,
                "q_w": q.w(),
                "q_x": q.x(),
                "q_y": q.y(),
                "q_z": q.z(),
                "v_x": v[0],
                "v_y": v[1],
                "v_z": v[2],
                "p_x": p[0],
                "p_y": p[1],
                "p_z": p[2],
                "b_w_x": bw[0],
                "b_w_y": bw[1],
                "b_w_z": bw[2],
                "b_a_x": ba[0],
                "b_a_y": ba[1],
                "b_a_z": ba[2],
                "w_x": float(r[euroc.IMU_COLS["w_x"]]),
                "w_y": float(r[euroc.IMU_COLS["w_y"]]),
                "w_z": float(r[euroc.IMU_COLS["w_z"]]),
                "a_x": float(r[euroc.IMU_COLS["a_x"]]),
                "a_y": float(r[euroc.IMU_COLS["a_y"]]),
                "a_z": float(r[euroc.IMU_COLS["a_z"]]),
            }
        )

    out = pd.DataFrame(rows)
    out.to_csv(out_csv, index=False)
    return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Combine EuRoC IMU and ground truth files with NavState interpolation."
    )
    euroc.add_dataset_args(parser)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output CSV filename (default: <root>_<seq><run>.csv)",
    )
    parser.add_argument(
        "--t_start",
        type=float,
        default=None,
        help="Start time (seconds) for IMU cropping",
    )
    parser.add_argument(
        "--t_end",
        type=float,
        default=None,
        help="End time (seconds) for IMU cropping",
    )
    parser.add_argument(
        "--t_offset",
        type=float,
        default=0.0,
        help="Constant time offset (seconds) added to IMU timestamps",
    )

    args = parser.parse_args()

    _, gt_csv, imu_csv = euroc.csv_paths(args.root, args.seq, args.run)

    if args.output is None:
        out_csv = Path(f"{args.seq}_{args.run}.csv")
    else:
        out_csv = args.output

    df = load_and_interpolate_euroc(
        gt_csv,
        imu_csv,
        out_csv,
        t_start=args.t_start,
        t_end=args.t_end,
        t_offset=args.t_offset,
    )
    print(f"wrote {len(df)} rows to {out_csv}")
