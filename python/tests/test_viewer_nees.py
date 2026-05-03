"""Tests for the NEES diagnostics Dash viewer and helper math."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from python.viewer.discovery import discover_runs
from python.viewer.loading import load_run_data, load_trajectory_samples
from python.viewer.nees_app import (
    _build_outlier_rows,
    _load_active_window_frame,
    _resolve_active_window,
    _selection_has_trajectory_data,
    create_dash_app,
)
from python.viewer.nees_diagnostics import (
    COVARIANCE_COLUMNS,
    ERROR_COMPONENT_COLUMNS,
    NORMALIZED_COMPONENT_COLUMNS,
    compute_row_diagnostics,
    whiten_residual,
)


def write_text(path: Path, contents: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf-8")


def make_run(root: Path, app_name: str, run_id: str) -> Path:
    run_dir = root / app_name / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def make_window_metrics_csv() -> str:
    return (
        "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,quadrature_nodes,"
        "window_index,window_start_sample,window_end_sample,window_start_time,window_end_time,normalized_nees,"
        "rot_error_norm,rot_pred_sigma,pos_error_norm,pos_pred_sigma,vel_error_norm,vel_pred_sigma\n"
        "20260420T000000000Z,evalEkf,MH01,gal3_imu_ekf,default,0.2,40,0,0,0,40,0.0,0.1,1.4,0.1,0.2,0.2,0.3,0.3,0.4\n"
        "20260420T000000000Z,evalEkf,MH01,gal3_imu_ekf,default,0.2,40,0,1,40,80,0.1,0.2,2.4,0.1,0.2,0.2,0.3,0.3,0.4\n"
        "20260420T000000000Z,evalEkf,MH01,navstate_imu_ekf,default,0.2,40,0,0,0,40,0.0,0.1,0.9,0.1,0.2,0.2,0.3,0.3,0.4\n"
    )


def make_trajectory_row(timestamp: float, *, method: str = "gal3_imu_ekf", scale: float = 1.0) -> str:
    covariance_values: list[str] = []
    for row in range(9):
        for column in range(9):
            covariance_values.append("1.0" if row == column else "0.0")
    values = [
        "20260420T000000000Z",
        "evalEkf",
        "MH01",
        method,
        "default",
        "0.2",
        "40",
        str(timestamp),
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        "0.0",
        str(scale),
        str(scale),
        str(scale),
        str(scale),
        str(scale),
        str(scale),
        str(scale),
        str(scale),
        str(scale),
        "1.0",
        "1.0",
        "1.0",
        *covariance_values,
    ]
    return ",".join(values)


def make_trajectory_csv(*rows: str) -> str:
    header = (
        "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,timestamp,"
        "gt_x,gt_y,gt_z,gt_vx,gt_vy,gt_vz,gt_roll,gt_pitch,gt_yaw,pred_x,pred_y,pred_z,pred_vx,pred_vy,"
        "pred_vz,pred_roll,pred_pitch,pred_yaw,err_rot_x,err_rot_y,err_rot_z,err_pos_x,err_pos_y,err_pos_z,"
        "err_vel_x,err_vel_y,err_vel_z,rot_pred_sigma,pos_pred_sigma,vel_pred_sigma,"
        + ",".join(COVARIANCE_COLUMNS)
        + "\n"
    )
    return header + "\n".join(rows) + "\n"


def make_identity_covariance_row() -> dict[str, float]:
    row = {column: 1.0 for column in ERROR_COMPONENT_COLUMNS}
    for index, column in enumerate(COVARIANCE_COLUMNS):
        row[column] = 1.0 if index // 9 == index % 9 else 0.0
    return row


class ViewerNeesTests(unittest.TestCase):
    def test_load_run_data_includes_window_metrics_and_trajectory_index(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalEkf", "20260420T000000000Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260420T000000000Z,evalEkf,2026-04-20T00:00:00.000Z,./evalEkf,./results,\n",
            )
            write_text(run_dir / "window_metrics.csv", make_window_metrics_csv())
            write_text(
                run_dir / "trajectory_samples.csv",
                make_trajectory_csv(make_trajectory_row(0.0), make_trajectory_row(0.05), make_trajectory_row(0.15)),
            )

            entry = discover_runs(root)[0]
            run_data = load_run_data(entry)
            self.assertEqual(len(run_data.window_metrics), 3)
            self.assertEqual(len(run_data.trajectory_index), 1)
            self.assertEqual(run_data.trajectory_index.iloc[0]["sample_count"], 3)
            self.assertTrue(entry.has_window_metrics)
            self.assertTrue(entry.has_trajectory_samples)

    def test_load_trajectory_samples_respects_selected_columns_and_interval_filter(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalEkf", "20260420T000000001Z")
            write_text(
                run_dir / "trajectory_samples.csv",
                make_trajectory_csv(
                    make_trajectory_row(0.0),
                    make_trajectory_row(0.05),
                    make_trajectory_row(0.1),
                    make_trajectory_row(0.25),
                ),
            )

            frame = load_trajectory_samples(
                run_dir,
                ("method", "timestamp"),
                method="gal3_imu_ekf",
                start_time=0.0,
                end_time=0.1,
            )
            self.assertEqual(list(frame.columns), ["method", "timestamp"])
            self.assertEqual(frame["timestamp"].tolist(), [0.0, 0.05, 0.1])

    def test_compute_row_diagnostics_identity_covariance(self) -> None:
        diagnostics = compute_row_diagnostics(make_identity_covariance_row())
        self.assertTrue(diagnostics.whitening_ok)
        self.assertAlmostEqual(diagnostics.sample_nees, 1.0)
        self.assertTrue(np.allclose(diagnostics.normalized_components, np.ones(9)))

    def test_whiten_residual_uses_jitter_for_semidefinite_covariance(self) -> None:
        error = np.ones(9, dtype=float)
        covariance = np.eye(9, dtype=float)
        covariance[0, 0] = 0.0
        whitened, jitter = whiten_residual(error, covariance)
        self.assertIsNotNone(whitened)
        self.assertGreater(jitter, 0.0)

    def test_augment_trajectory_samples_reports_skipped_rows_for_singular_covariance(self) -> None:
        singular_row = {column: 1.0 for column in ERROR_COMPONENT_COLUMNS}
        singular_row.update({column: 0.0 for column in COVARIANCE_COLUMNS})
        singular_row["cov_0_0"] = -1.0
        diagnostics = compute_row_diagnostics(singular_row)
        self.assertFalse(diagnostics.whitening_ok)
        self.assertTrue(np.isnan(diagnostics.sample_nees))
        self.assertTrue(np.isnan(diagnostics.normalized_components[0]))

    def test_create_dash_app_for_nees_viewer(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalEkf", "20260420T000000002Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260420T000000002Z,evalEkf,2026-04-20T00:00:02.000Z,./evalEkf,./results,\n",
            )
            write_text(run_dir / "window_metrics.csv", make_window_metrics_csv())
            app = create_dash_app(root)
            self.assertEqual(app.title, "imuFactors NEES Diagnostics")

    def test_selection_has_trajectory_data_for_matching_method(self) -> None:
        active_window = {
            "dataset": "MH01",
            "method": "gal3_imu_ekf",
            "config_label": "default",
            "interval_seconds": 0.2,
        }
        index_rows = [
            {
                "dataset": "MH01",
                "method": "gal3_imu_ekf",
                "config_label": "default",
                "interval_seconds": 0.2,
            }
        ]
        self.assertTrue(_selection_has_trajectory_data(index_rows, active_window))
        self.assertFalse(
            _selection_has_trajectory_data(
                index_rows,
                {**active_window, "method": "quadrature"},
            )
        )

    def test_resolve_active_window_prefers_click_then_selection_then_default(self) -> None:
        table_rows = [
            {
                "dataset": "MH01",
                "method": "gal3_imu_ekf",
                "config_label": "default",
                "interval_seconds": 0.2,
                "window_index": 0,
                "window_start_time": 0.0,
                "window_end_time": 0.1,
            },
            {
                "dataset": "MH01",
                "method": "navstate_imu_ekf",
                "config_label": "default",
                "interval_seconds": 0.2,
                "window_index": 1,
                "window_start_time": 0.1,
                "window_end_time": 0.2,
            },
        ]
        clicked = _resolve_active_window(
            table_rows,
            current_active=None,
            selected_rows=None,
            click_payload={"points": [{"customdata": ["MH01", "navstate_imu_ekf", "default", 0.2, 1, 0.1, 0.2]}]},
            trigger_id="overview-nees-graph",
        )
        self.assertEqual(clicked["method"], "navstate_imu_ekf")

        selected = _resolve_active_window(
            table_rows,
            current_active=None,
            selected_rows=[0],
            click_payload=None,
            trigger_id="outlier-table",
        )
        self.assertEqual(selected["window_index"], 0)

    def test_load_active_window_frame_uses_window_time_bounds(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalEkf", "20260420T000000003Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260420T000000003Z,evalEkf,2026-04-20T00:00:03.000Z,./evalEkf,./results,\n",
            )
            write_text(run_dir / "window_metrics.csv", make_window_metrics_csv())
            write_text(
                run_dir / "trajectory_samples.csv",
                make_trajectory_csv(
                    make_trajectory_row(0.0),
                    make_trajectory_row(0.05),
                    make_trajectory_row(0.1),
                    make_trajectory_row(0.25),
                ),
            )
            entry = discover_runs(root)[0]
            frame, skipped_count = _load_active_window_frame(
                entry,
                {
                    "dataset": "MH01",
                    "method": "gal3_imu_ekf",
                    "config_label": "default",
                    "interval_seconds": 0.2,
                    "window_index": 0,
                    "window_start_time": 0.0,
                    "window_end_time": 0.1,
                },
            )
            self.assertEqual(frame["timestamp"].tolist(), [0.0, 0.05, 0.1])
            self.assertEqual(skipped_count, 0)
            self.assertIn("sample_nees", frame.columns)
            self.assertTrue(set(NORMALIZED_COMPONENT_COLUMNS).issubset(frame.columns))

    def test_outlier_rows_display_rotation_error_in_degrees(self) -> None:
        rows = [
            {"window_index": 0, "normalized_nees": 1.0, "rot_error_norm": 0.1},
            {"window_index": 1, "normalized_nees": 2.0, "rot_error_norm": 0.2},
        ]

        display_rows = _build_outlier_rows(rows)

        self.assertEqual([row["window_index"] for row in display_rows], [1, 0])
        self.assertAlmostEqual(display_rows[0]["rot_error_norm"], 11.4591559026)
        self.assertAlmostEqual(display_rows[1]["rot_error_norm"], 5.7295779513)
        self.assertEqual(rows[0]["rot_error_norm"], 0.1)


if __name__ == "__main__":
    unittest.main()
