"""Helpers for reading EuRoC ground-truth and IMU CSV files."""

from __future__ import annotations

import argparse

from pathlib import Path

import numpy as np
import pandas as pd
import gtsam


GT_COLS = {
    "t": "#timestamp",
    "p_x": " p_RS_R_x [m]",
    "p_y": " p_RS_R_y [m]",
    "p_z": " p_RS_R_z [m]",
    "q_w": " q_RS_w []",
    "q_x": " q_RS_x []",
    "q_y": " q_RS_y []",
    "q_z": " q_RS_z []",
    "v_x": " v_RS_R_x [m s^-1]",
    "v_y": " v_RS_R_y [m s^-1]",
    "v_z": " v_RS_R_z [m s^-1]",
    "b_w_x": " b_w_RS_S_x [rad s^-1]",
    "b_w_y": " b_w_RS_S_y [rad s^-1]",
    "b_w_z": " b_w_RS_S_z [rad s^-1]",
    "b_a_x": " b_a_RS_S_x [m s^-2]",
    "b_a_y": " b_a_RS_S_y [m s^-2]",
    "b_a_z": " b_a_RS_S_z [m s^-2]",
}

IMU_COLS = {
    "t": "#timestamp [ns]",
    "w_x": "w_RS_S_x [rad s^-1]",
    "w_y": "w_RS_S_y [rad s^-1]",
    "w_z": "w_RS_S_z [rad s^-1]",
    "a_x": "a_RS_S_x [m s^-2]",
    "a_y": "a_RS_S_y [m s^-2]",
    "a_z": "a_RS_S_z [m s^-2]",
}


def add_dataset_args(parser: argparse.ArgumentParser) -> None:
    """Add shared EuRoC dataset path arguments to a CLI parser."""
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("euroc"),
        help="Root directory containing EuRoC dataset (default: euroc)",
    )
    parser.add_argument(
        "--seq",
        type=str,
        default="vicon_room1",
        help="Sequence name (e.g., vicon_room1)",
    )
    parser.add_argument(
        "--run",
        type=str,
        default="V1_01_easy",
        help="Run identifier within sequence (e.g., V1_01_easy)",
    )


def csv_paths(root: Path, seq: str, run: str) -> tuple[Path, Path, Path]:
    """Return dataset root, ground-truth CSV path, and IMU CSV path."""
    dataset_root = root / seq / run
    gt_csv = dataset_root / "state_groundtruth_estimate0" / "data.csv"
    imu_csv = dataset_root / "imu0" / "data.csv"
    return dataset_root, gt_csv, imu_csv


def read_ground_truth(
    gt_csv: str | Path,
) -> tuple[np.ndarray, list[gtsam.NavState], np.ndarray, np.ndarray]:
    """
    Read EuRoC ground-truth CSV and return timestamps, NavStates, gyro biases, and accel biases.

    Returns:
        gt_t: timestamps in seconds, shape (N,)
        gt_states: list of NavState objects, length N
        gt_bw: gyro biases, shape (N, 3)
        gt_ba: accel biases, shape (N, 3)
    """
    gt = pd.read_csv(gt_csv)
    gt_t = gt[GT_COLS["t"]].to_numpy(dtype=float) * 1e-9

    gt_states = []
    gt_bw = []
    gt_ba = []

    for _, r in gt.iterrows():
        R = gtsam.Rot3.Quaternion(
            float(r[GT_COLS["q_w"]]),
            float(r[GT_COLS["q_x"]]),
            float(r[GT_COLS["q_y"]]),
            float(r[GT_COLS["q_z"]]),
        )
        p = np.array(
            [r[GT_COLS["p_x"]], r[GT_COLS["p_y"]], r[GT_COLS["p_z"]]],
            dtype=float,
        )
        v = np.array(
            [r[GT_COLS["v_x"]], r[GT_COLS["v_y"]], r[GT_COLS["v_z"]]],
            dtype=float,
        )

        gt_states.append(gtsam.NavState(gtsam.Pose3(R, p), v))
        gt_bw.append(
            np.array(
                [
                    r[GT_COLS["b_w_x"]],
                    r[GT_COLS["b_w_y"]],
                    r[GT_COLS["b_w_z"]],
                ],
                dtype=float,
            )
        )
        gt_ba.append(
            np.array(
                [
                    r[GT_COLS["b_a_x"]],
                    r[GT_COLS["b_a_y"]],
                    r[GT_COLS["b_a_z"]],
                ],
                dtype=float,
            )
        )

    return gt_t, gt_states, np.asarray(gt_bw), np.asarray(gt_ba)


def read_imu(imu_csv: str | Path) -> tuple[pd.DataFrame, np.ndarray]:
    """
    Read EuRoC IMU CSV and return the dataframe and timestamps in seconds.

    Returns:
        imu: raw IMU dataframe
        imu_t: timestamps in seconds, shape (M,)
    """
    imu = pd.read_csv(imu_csv)
    imu_t = imu[IMU_COLS["t"]].to_numpy(dtype=float) * 1e-9
    return imu, imu_t
