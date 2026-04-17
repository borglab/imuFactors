"""Smoke tests for summary-viewer discovery and loading."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from python.viewer.app import create_dash_app
from python.viewer.discovery import discover_runs
from python.viewer.loading import load_run_data


def write_text(path: Path, contents: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf-8")


def make_run(root: Path, app_name: str, run_id: str) -> Path:
    run_dir = root / app_name / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


class ViewerSummaryTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
