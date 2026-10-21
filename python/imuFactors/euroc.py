"""Helpers for reading, deriving, and discovering EuRoC CSV signals."""

from __future__ import annotations

import argparse
from pathlib import Path

import gtsam
import numpy as np
import pandas as pd


GRAVITY = 9.81
DEFAULT_SIGNAL_GROUP = "imu"

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

SIGNAL_GROUPS = {
    "imu": ["w_x", "w_y", "w_z", "a_x", "a_y", "a_z"],
    "imu_norms": ["gyro_norm", "accel_norm"],
    "gyro": ["w_x", "w_y", "w_z"],
    "accel": ["a_x", "a_y", "a_z"],
    "accel_gravity_compensated": ["a_gc_x", "a_gc_y", "a_gc_z"],
    "position": ["p_x", "p_y", "p_z"],
    "velocity": ["v_x", "v_y", "v_z"],
    "gyro_bias": ["b_w_x", "b_w_y", "b_w_z"],
    "accel_bias": ["b_a_x", "b_a_y", "b_a_z"],
    "quaternion_components": ["q_w", "q_x", "q_y", "q_z"],
    "state_plus_bias": [
        "q_w",
        "q_x",
        "q_y",
        "q_z",
        "v_x",
        "v_y",
        "v_z",
        "p_x",
        "p_y",
        "p_z",
        "b_w_x",
        "b_w_y",
        "b_w_z",
        "b_a_x",
        "b_a_y",
        "b_a_z",
    ],
    "all_numeric": [],
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
    """Read a raw EuRoC ground-truth CSV."""
    gt = pd.read_csv(gt_csv)
    gt_t = gt[GT_COLS["t"]].to_numpy(dtype=float) * 1e-9

    gt_states = []
    gt_bw = []
    gt_ba = []

    for _, row in gt.iterrows():
        rotation = gtsam.Rot3.Quaternion(
            float(row[GT_COLS["q_w"]]),
            float(row[GT_COLS["q_x"]]),
            float(row[GT_COLS["q_y"]]),
            float(row[GT_COLS["q_z"]]),
        )
        position = np.array(
            [row[GT_COLS["p_x"]], row[GT_COLS["p_y"]], row[GT_COLS["p_z"]]],
            dtype=float,
        )
        velocity = np.array(
            [row[GT_COLS["v_x"]], row[GT_COLS["v_y"]], row[GT_COLS["v_z"]]],
            dtype=float,
        )

        gt_states.append(gtsam.NavState(gtsam.Pose3(rotation, position), velocity))
        gt_bw.append(
            np.array(
                [
                    row[GT_COLS["b_w_x"]],
                    row[GT_COLS["b_w_y"]],
                    row[GT_COLS["b_w_z"]],
                ],
                dtype=float,
            )
        )
        gt_ba.append(
            np.array(
                [
                    row[GT_COLS["b_a_x"]],
                    row[GT_COLS["b_a_y"]],
                    row[GT_COLS["b_a_z"]],
                ],
                dtype=float,
            )
        )

    return gt_t, gt_states, np.asarray(gt_bw), np.asarray(gt_ba)


def read_imu(imu_csv: str | Path) -> tuple[pd.DataFrame, np.ndarray]:
    """Read a raw EuRoC IMU CSV and return the dataframe plus seconds."""
    imu = pd.read_csv(imu_csv)
    imu_t = imu[IMU_COLS["t"]].to_numpy(dtype=float) * 1e-9
    return imu, imu_t


def discover_euroc_files(data_dir: str | Path) -> list[Path]:
    """Return sorted merged EuRoC CSV files under ``data_dir``."""
    return sorted(Path(data_dir).glob("euroc_*.csv"))


def load_euroc_csv(
    path: str | Path,
    *,
    continuous_quaternions: bool = True,
    include_imu_norms: bool = True,
    include_gravity_compensated_accel: bool = True,
    gravity: float = GRAVITY,
) -> pd.DataFrame:
    """Load a merged EuRoC CSV and add useful derived signal columns."""
    dataframe = pd.read_csv(path)
    quat_cols = ["q_w", "q_x", "q_y", "q_z"]
    if continuous_quaternions and all(
        column in dataframe.columns for column in quat_cols
    ):
        quat = dataframe[quat_cols].to_numpy(dtype=float, copy=True)
        for index in range(1, len(quat)):
            if float(np.dot(quat[index - 1], quat[index])) < 0.0:
                quat[index] *= -1.0
        dataframe.loc[:, quat_cols] = quat
    if include_imu_norms:
        add_imu_norm_columns(dataframe)
    if include_gravity_compensated_accel:
        add_gravity_compensated_accel_columns(dataframe, gravity=gravity)
    return dataframe


def add_imu_norm_columns(dataframe: pd.DataFrame) -> pd.DataFrame:
    """Add ``gyro_norm`` and ``accel_norm`` columns when IMU axes are present."""
    gyro_columns = ["w_x", "w_y", "w_z"]
    accel_columns = ["a_x", "a_y", "a_z"]
    if all(column in dataframe.columns for column in gyro_columns):
        gyro = dataframe[gyro_columns].to_numpy(dtype=float)
        dataframe["gyro_norm"] = np.linalg.norm(gyro, axis=1)
    if all(column in dataframe.columns for column in accel_columns):
        accel = dataframe[accel_columns].to_numpy(dtype=float)
        dataframe["accel_norm"] = np.linalg.norm(accel, axis=1)
    return dataframe


def add_gravity_compensated_accel_columns(
    dataframe: pd.DataFrame,
    *,
    gravity: float = GRAVITY,
) -> pd.DataFrame:
    """Add body-frame accelerometer columns with static gravity removed."""
    accel_columns = ["a_x", "a_y", "a_z"]
    quat_columns = ["q_w", "q_x", "q_y", "q_z"]
    if not all(column in dataframe.columns for column in accel_columns + quat_columns):
        return dataframe

    accel = dataframe[accel_columns].to_numpy(dtype=float)
    quaternions = dataframe[quat_columns].to_numpy(dtype=float)
    expected_static_accel = np.empty_like(accel)
    n_gravity_opposite = np.array([0.0, 0.0, float(gravity)])

    for index, quaternion in enumerate(quaternions):
        nRb = gtsam.Rot3.Quaternion(
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        )
        expected_static_accel[index, :] = nRb.unrotate(n_gravity_opposite)

    compensated = accel - expected_static_accel
    dataframe["a_gc_x"] = compensated[:, 0]
    dataframe["a_gc_y"] = compensated[:, 1]
    dataframe["a_gc_z"] = compensated[:, 2]
    return dataframe


def available_signal_groups(dataframe: pd.DataFrame) -> dict[str, list[str]]:
    """Return signal groups whose columns are present in ``dataframe``."""
    groups: dict[str, list[str]] = {}
    for name, columns in SIGNAL_GROUPS.items():
        if name == "all_numeric":
            continue
        present = [column for column in columns if column in dataframe.columns]
        if present:
            groups[name] = present
    groups["all_numeric"] = [
        column
        for column in dataframe.columns
        if column != "t" and pd.api.types.is_numeric_dtype(dataframe[column])
    ]
    return groups
