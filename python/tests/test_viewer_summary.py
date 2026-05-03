"""Smoke tests for summary-viewer discovery and loading."""

from __future__ import annotations

import os
import tempfile
import time
import unittest
from pathlib import Path

from python.viewer.app import (
    _build_window_method_comparison,
    _comparison_rows,
    _display_window_summary_rows,
    _load_payload_for_entry,
    _resolve_compare_paths,
    create_dash_app,
)
from python.viewer.discovery import discover_runs
from python.viewer.layout import build_data_table, format_local_timestamp, format_run_option
from python.viewer.loading import load_run_data
from python.viewer.models import RunCatalogEntry


def write_text(path: Path, contents: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf-8")


def make_run(root: Path, app_name: str, run_id: str) -> Path:
    run_dir = root / app_name / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


class ViewerSummaryTests(unittest.TestCase):
    def test_shared_data_table_uses_scroll_instead_of_pagination(self) -> None:
        table = build_data_table("summary-table")

        self.assertEqual(table.page_action, "none")
        self.assertEqual(table.filter_action, "none")
        self.assertTrue(table.virtualization)
        self.assertEqual(table.fixed_rows, {"headers": True})
        self.assertEqual(table.style_table["overflowY"], "auto")

    def test_format_local_timestamp_uses_process_timezone(self) -> None:
        previous_timezone = os.environ.get("TZ")
        try:
            os.environ["TZ"] = "America/New_York"
            time.tzset()
            self.assertEqual(
                format_local_timestamp("2026-04-17T00:00:00.000Z"),
                "2026-04-16 20:00:00 EDT",
            )
        finally:
            if previous_timezone is None:
                os.environ.pop("TZ", None)
            else:
                os.environ["TZ"] = previous_timezone
            time.tzset()

    def test_format_run_option_displays_local_timestamp(self) -> None:
        previous_timezone = os.environ.get("TZ")
        try:
            os.environ["TZ"] = "America/New_York"
            time.tzset()
            option = format_run_option(
                RunCatalogEntry(
                    app_name="evalFoo",
                    run_id="run-a",
                    path=Path("/tmp/evalFoo/run-a"),
                    status="ready",
                    timestamp_utc="2026-04-17T00:00:00.000Z",
                )
            )
            self.assertIn("2026-04-16 20:00:00 EDT", option["label"])
        finally:
            if previous_timezone is None:
                os.environ.pop("TZ", None)
            else:
                os.environ["TZ"] = previous_timezone
            time.tzset()

    def test_summary_payload_skips_trajectory_index(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalFoo", "20260417T000000000Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260417T000000000Z,evalFoo,2026-04-17T00:00:00.000Z,./evalFoo,./results,\n",
            )
            write_text(
                run_dir / "trajectory_samples.csv",
                "dataset,method,config_label,interval_seconds,timestamp\n"
                "MH01,quadrature,default,0.2,0.0\n",
            )
            entry = RunCatalogEntry(
                app_name="evalFoo",
                run_id="20260417T000000000Z",
                path=run_dir,
                status="ready",
                has_trajectory_samples=True,
            )

            payload = _load_payload_for_entry(entry)

            self.assertEqual(payload["trajectory_index_rows"], [])

    def test_discover_runs_complete_and_header_only(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalFoo", "20260417T000000000Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260417T000000000Z,evalFoo,2026-04-17T00:00:00.000Z,./evalFoo,./results,\n",
            )
            write_text(
                run_dir / "datasets.csv",
                "run_id,app_name,dataset,source_path,dataset_group\n"
                "20260417T000000000Z,evalFoo,MH01,../data/euroc/euroc_MH01.csv,machine_hall\n",
            )
            write_text(
                run_dir / "window_summaries.csv",
                "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,"
                "quadrature_nodes,sample_count,normalized_nees_mean,normalized_nees_median,"
                "normalized_nees_p95,normalized_nees_variance,rot_error_median,rot_pred_sigma_median,"
                "pos_error_median,pos_pred_sigma_median,vel_error_median,vel_pred_sigma_median\n",
            )
            write_text(
                run_dir / "calibration_summaries.csv",
                "run_id,app_name,study_name,result_label,alpha_gyro,alpha_acc,mean_nees,sum_deviations\n",
            )

            catalog = discover_runs(root)
            self.assertEqual(len(catalog), 1)
            self.assertEqual(catalog[0].status, "ready")
            self.assertEqual(catalog[0].dataset_group_label, "machine_hall")

    def test_discover_runs_partial_for_zero_byte_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalFoo", "20260417T000000001Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260417T000000001Z,evalFoo,2026-04-17T00:00:01.000Z,./evalFoo,./results,\n",
            )
            (run_dir / "window_summaries.csv").touch()

            catalog = discover_runs(root)
            self.assertEqual(len(catalog), 1)
            self.assertEqual(catalog[0].status, "partial")

    def test_discover_runs_fallback_to_path_when_metadata_missing(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalFallback", "20260417T000000002Z")
            write_text(
                run_dir / "window_summaries.csv",
                "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,"
                "quadrature_nodes,sample_count,normalized_nees_mean,normalized_nees_median,"
                "normalized_nees_p95,normalized_nees_variance,rot_error_median,rot_pred_sigma_median,"
                "pos_error_median,pos_pred_sigma_median,vel_error_median,vel_pred_sigma_median\n"
                "20260417T000000002Z,evalFallback,MH01,quadrature,default,0.2,40,6,720,1.0,0.5,1.5,0.2,0.1,0.1,0.2,0.2,0.3,0.3\n",
            )

            catalog = discover_runs(root)
            self.assertEqual(len(catalog), 1)
            self.assertEqual(catalog[0].app_name, "evalFallback")
            self.assertEqual(catalog[0].run_id, "20260417T000000002Z")
            self.assertEqual(catalog[0].status, "partial")

    def test_discover_runs_orders_missing_timestamp_by_run_id_recency(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)

            older_run = make_run(root, "evalReduced", "20260424T165239915Z")
            write_text(
                older_run / "window_summaries.csv",
                "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,"
                "quadrature_nodes,sample_count,normalized_nees_mean,normalized_nees_median,"
                "normalized_nees_p95,normalized_nees_variance,rot_error_median,rot_pred_sigma_median,"
                "pos_error_median,pos_pred_sigma_median,vel_error_median,vel_pred_sigma_median\n"
                "20260424T165239915Z,evalReduced,MH01,quadrature,default,0.2,40,6,720,1.0,0.5,1.5,0.2,0.1,0.1,0.2,0.2,0.3,0.3\n",
            )

            newer_run = make_run(root, "evalReduced", "20260503T185304287Z")
            write_text(
                newer_run / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260503T185304287Z,evalReduced,2026-05-03T18:53:04.287Z,./evalReduced,./results,\n",
            )
            write_text(
                newer_run / "window_summaries.csv",
                "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,"
                "quadrature_nodes,sample_count,normalized_nees_mean,normalized_nees_median,"
                "normalized_nees_p95,normalized_nees_variance,rot_error_median,rot_pred_sigma_median,"
                "pos_error_median,pos_pred_sigma_median,vel_error_median,vel_pred_sigma_median\n"
                "20260503T185304287Z,evalReduced,MH01,quadrature,default,0.2,40,6,720,1.0,0.5,1.5,0.2,0.1,0.1,0.2,0.2,0.3,0.3\n",
            )

            catalog = discover_runs(root)
            self.assertEqual(len(catalog), 2)
            self.assertEqual(catalog[0].run_id, "20260503T185304287Z")
            self.assertEqual(catalog[1].run_id, "20260424T165239915Z")

    def test_load_run_data_window_summary_only(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalWindow", "20260417T000000003Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260417T000000003Z,evalWindow,2026-04-17T00:00:03.000Z,./evalWindow,./results,\n",
            )
            write_text(
                run_dir / "window_summaries.csv",
                "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,"
                "quadrature_nodes,sample_count,normalized_nees_mean,normalized_nees_median,"
                "normalized_nees_p95,normalized_nees_variance,rot_error_median,rot_pred_sigma_median,"
                "pos_error_median,pos_pred_sigma_median,vel_error_median,vel_pred_sigma_median\n"
                "20260417T000000003Z,evalWindow,MH01,quadrature,default,0.2,40,6,720,1.0,0.5,1.5,0.2,0.1,0.1,0.2,0.2,0.3,0.3\n",
            )
            entry = discover_runs(root)[0]
            run_data = load_run_data(entry)
            self.assertEqual(len(run_data.window_summaries), 1)
            self.assertTrue(run_data.calibration_summaries.empty)

    def test_load_run_data_calibration_summary_only(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalCalibration", "20260417T000000004Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260417T000000004Z,evalCalibration,2026-04-17T00:00:04.000Z,./evalCalibration,./results,\n",
            )
            write_text(
                run_dir / "calibration_summaries.csv",
                "run_id,app_name,study_name,result_label,alpha_gyro,alpha_acc,mean_nees,sum_deviations\n"
                "20260417T000000004Z,evalCalibration,all_coarse,best,1.0,2.0,0.8,0.2\n",
            )
            entry = discover_runs(root)[0]
            run_data = load_run_data(entry)
            self.assertTrue(run_data.window_summaries.empty)
            self.assertEqual(len(run_data.calibration_summaries), 1)

    def test_load_run_data_no_populated_summaries(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalEmpty", "20260417T000000005Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260417T000000005Z,evalEmpty,2026-04-17T00:00:05.000Z,./evalEmpty,./results,\n",
            )
            write_text(
                run_dir / "window_summaries.csv",
                "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,"
                "quadrature_nodes,sample_count,normalized_nees_mean,normalized_nees_median,"
                "normalized_nees_p95,normalized_nees_variance,rot_error_median,rot_pred_sigma_median,"
                "pos_error_median,pos_pred_sigma_median,vel_error_median,vel_pred_sigma_median\n",
            )
            write_text(
                run_dir / "calibration_summaries.csv",
                "run_id,app_name,study_name,result_label,alpha_gyro,alpha_acc,mean_nees,sum_deviations\n",
            )
            entry = discover_runs(root)[0]
            run_data = load_run_data(entry)
            self.assertTrue(run_data.window_summaries.empty)
            self.assertTrue(run_data.calibration_summaries.empty)

    def test_create_dash_app_uses_discovered_runs(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            run_dir = make_run(root, "evalDash", "20260417T000000006Z")
            write_text(
                run_dir / "run_metadata.csv",
                "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version\n"
                "20260417T000000006Z,evalDash,2026-04-17T00:00:06.000Z,./evalDash,./results,\n",
            )
            write_text(
                run_dir / "window_summaries.csv",
                "run_id,app_name,dataset,method,config_label,interval_seconds,samples_per_window,"
                "quadrature_nodes,sample_count,normalized_nees_mean,normalized_nees_median,"
                "normalized_nees_p95,normalized_nees_variance,rot_error_median,rot_pred_sigma_median,"
                "pos_error_median,pos_pred_sigma_median,vel_error_median,vel_pred_sigma_median\n"
                "20260417T000000006Z,evalDash,MH01,quadrature,default,0.2,40,6,720,1.0,0.5,1.5,0.2,0.1,0.1,0.2,0.2,0.3,0.3\n",
            )

            app = create_dash_app(root)
            self.assertEqual(app.title, "imuFactors Summary Viewer")

    def test_resolve_compare_paths_includes_selected_once(self) -> None:
        resolved = _resolve_compare_paths("run-a", ["run-b", "run-a", "run-c"])
        self.assertEqual(resolved, ["run-a", "run-b", "run-c"])

    def test_comparison_rows_merge_multiple_runs(self) -> None:
        payloads = [
            {
                "entry": {
                    "app_name": "evalA",
                    "run_id": "run-a",
                    "path": "/tmp/evalA/run-a",
                    "status": "ready",
                    "timestamp_utc": "2026-04-17T00:00:00.000Z",
                },
                "window_rows": [
                    {
                        "dataset": "MH01",
                        "method": "quadrature",
                        "config_label": "default",
                        "interval_seconds": 0.2,
                    }
                ],
            },
            {
                "entry": {
                    "app_name": "evalB",
                    "run_id": "run-b",
                    "path": "/tmp/evalB/run-b",
                    "status": "partial",
                    "timestamp_utc": "2026-04-17T00:00:01.000Z",
                },
                "window_rows": [
                    {
                        "dataset": "MH02",
                        "method": "manifold",
                        "config_label": "best",
                        "interval_seconds": 0.5,
                    }
                ],
            },
        ]

        columns, rows = _comparison_rows(payloads, "window_rows")
        self.assertEqual(columns[:5], ["run_label", "app_name", "run_id", "timestamp_utc", "status"])
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["run_label"], "evalA | run-a")
        self.assertEqual(rows[1]["dataset"], "MH02")

    def test_build_window_method_comparison_pivots_methods_side_by_side(self) -> None:
        rows = [
            {
                "dataset": "V102",
                "interval_seconds": 0.2,
                "config_label": "default",
                "samples_per_window": 40,
                "sample_count": 400,
                "method": "quadrature",
                "normalized_nees_mean": 0.0240659,
                "normalized_nees_median": 0.0187926,
                "rot_error_median": 0.0011,
                "pos_error_median": 0.0022,
                "vel_error_median": 0.0033,
            },
            {
                "dataset": "V102",
                "interval_seconds": 0.2,
                "config_label": "default",
                "samples_per_window": 40,
                "sample_count": 400,
                "method": "manifold",
                "normalized_nees_mean": 0.0189061,
                "normalized_nees_median": 0.0151347,
                "rot_error_median": 0.0010,
                "pos_error_median": 0.0021,
                "vel_error_median": 0.0031,
            },
            {
                "dataset": "V102",
                "interval_seconds": 0.2,
                "config_label": "default",
                "samples_per_window": 40,
                "sample_count": 400,
                "method": "tangent",
                "normalized_nees_mean": 0.0189061,
                "normalized_nees_median": 0.0151347,
                "rot_error_median": 0.0010,
                "pos_error_median": 0.0021,
                "vel_error_median": 0.0031,
            },
        ]

        columns, comparison_rows, style_rules = _build_window_method_comparison(rows, "core")
        self.assertEqual(comparison_rows[0]["dataset"], "V102")
        self.assertIn("normalized_nees_mean__quadrature", comparison_rows[0])
        self.assertIn("normalized_nees_mean__manifold", comparison_rows[0])
        self.assertIn("normalized_nees_mean__tangent", comparison_rows[0])
        self.assertEqual(columns[0]["name"], ["Dataset", ""])
        self.assertEqual(columns[5]["name"], ["Normalized NEES Mean", "Quadrature"])
        self.assertEqual(comparison_rows[0]["normalized_nees_mean__quadrature"], "0.02407")
        self.assertEqual(columns[11]["name"], ["Rotation Error Median (deg)", "Quadrature"])
        self.assertEqual(comparison_rows[0]["rot_error_median__manifold"], "0.0573")
        self.assertEqual(comparison_rows[0]["pos_error_median__manifold"], "0.0021")
        self.assertIn(
            {"if": {"row_index": 0, "column_id": "rot_error_median__manifold"}, "fontWeight": "700"},
            style_rules,
        )
        self.assertIn(
            {"if": {"row_index": 0, "column_id": "normalized_nees_mean__quadrature"}, "fontWeight": "700"},
            style_rules,
        )

    def test_display_window_summary_rows_converts_rotation_errors_only(self) -> None:
        rows = [{"rot_error_median": 0.1, "pos_error_median": 0.2, "vel_error_median": 0.3}]

        display_rows = _display_window_summary_rows(rows)

        self.assertAlmostEqual(display_rows[0]["rot_error_median"], 5.7295779513)
        self.assertEqual(display_rows[0]["pos_error_median"], 0.2)
        self.assertEqual(display_rows[0]["vel_error_median"], 0.3)
        self.assertEqual(rows[0]["rot_error_median"], 0.1)


if __name__ == "__main__":
    unittest.main()
