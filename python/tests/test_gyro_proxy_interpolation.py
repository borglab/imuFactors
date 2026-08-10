"""Tests for gyro proxy interpolation experiments."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import gtsam
import numpy as np
import pandas as pd

PYTHON_DIR = Path(__file__).resolve().parents[1]
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

from imuFactors import spectral
from imuFactors.gyro_proxy_interpolation import (
    run_gyro_proxy_interpolation_experiments,
    scaled_tikhonov_lambda,
)
from imuFactors.gyro_proxy_monte_carlo import GyroProxySnippet


class GyroProxyInterpolationTests(unittest.TestCase):
    def test_scaled_lambda_is_zero_and_quadratic_in_noise(self) -> None:
        self.assertEqual(
            scaled_tikhonov_lambda(m=20, noise_std=0.0, roughness=10.0),
            0.0,
        )
        self.assertEqual(
            scaled_tikhonov_lambda(m=20, noise_std=1.0, roughness=0.0),
            0.0,
        )
        base = scaled_tikhonov_lambda(m=20, noise_std=0.1, roughness=10.0)
        doubled = scaled_tikhonov_lambda(m=20, noise_std=0.2, roughness=10.0)
        self.assertAlmostEqual(doubled, 4.0 * base, places=14)

    def test_constant_omega_has_near_zero_interpolation_and_trajectory_error(self) -> None:
        snippet = constant_snippet(np.array([0.1, -0.2, 0.3]))
        result = run_gyro_proxy_interpolation_experiments(
            [snippet],
            m_values=[5],
            noise_fractions=[0.0],
            num_seeds=1,
            seed=20260523,
            dense_count=51,
            subintervals_per_duration_bin=3,
        )

        self.assertLess(result.signal_metrics["gyro_rmse_rad_s"].max(), 1e-10)
        self.assertLess(
            result.trajectory_metrics["trajectory_rms_error_deg"].max(),
            1e-5,
        )
        self.assertLess(result.subinterval_metrics["rms_error_deg"].max(), 1e-5)

    def test_small_run_is_deterministic_and_finite(self) -> None:
        snippet = smooth_snippet()
        kwargs = dict(
            m_values=[5],
            noise_fractions=[0.0, 0.1],
            num_seeds=2,
            seed=20260523,
            dense_count=51,
            subintervals_per_duration_bin=3,
        )

        first = run_gyro_proxy_interpolation_experiments([snippet], **kwargs)
        second = run_gyro_proxy_interpolation_experiments([snippet], **kwargs)

        pd.testing.assert_frame_equal(first.signal_metrics, second.signal_metrics)
        pd.testing.assert_frame_equal(first.trajectory_metrics, second.trajectory_metrics)
        pd.testing.assert_frame_equal(first.subinterval_metrics, second.subinterval_metrics)
        self.assertTrue(np.isfinite(first.signal_metrics["gyro_nrmse"]).all())
        self.assertTrue(
            np.isfinite(first.trajectory_metrics["trajectory_rms_error_deg"]).all()
        )
        self.assertTrue(np.isfinite(first.subinterval_metrics["rms_error_deg"]).all())
        self.assertFalse(first.signal_summary.empty)
        self.assertFalse(first.trajectory_summary.empty)
        self.assertFalse(first.subinterval_summary.empty)


def constant_snippet(omega: np.ndarray) -> GyroProxySnippet:
    sample_seconds = np.linspace(0.0, 1.0, 101)
    cgl_node_values = np.tile(np.asarray(omega, dtype=float), (6, 1))
    raw = np.tile(np.asarray(omega, dtype=float), (sample_seconds.size, 1))
    return GyroProxySnippet(
        dataset="constant",
        csv_path=Path("constant.csv"),
        window_index=0,
        window_start_s=0.0,
        aggressive_score=0.0,
        center_activity=0.0,
        center_high_order_ratio=0.0,
        center_normalized_rmse=0.0,
        sample_seconds=sample_seconds,
        raw_bias_corrected_gyro=raw,
        cgl_node_seconds=spectral.chebyshev2_points(6, (0.0, 1.0)),
        cgl_node_values=cgl_node_values,
        reconstructed_samples=raw,
        fit_relative_error=0.0,
        middle_interval=(0.4, 0.6),
        reference_rotation=gtsam.Rot3.Expmap(np.asarray(omega, dtype=float) * 0.2),
        noise_scale=1.0,
    )


def smooth_snippet() -> GyroProxySnippet:
    sample_seconds = np.linspace(0.0, 1.0, 101)
    cgl_times = spectral.chebyshev2_points(8, (0.0, 1.0))
    cgl_node_values = smooth_omega(cgl_times)
    raw = smooth_omega(sample_seconds)
    return GyroProxySnippet(
        dataset="smooth",
        csv_path=Path("smooth.csv"),
        window_index=0,
        window_start_s=0.0,
        aggressive_score=1.0,
        center_activity=1.0,
        center_high_order_ratio=1.0,
        center_normalized_rmse=0.0,
        sample_seconds=sample_seconds,
        raw_bias_corrected_gyro=raw,
        cgl_node_seconds=cgl_times,
        cgl_node_values=cgl_node_values,
        reconstructed_samples=raw,
        fit_relative_error=0.0,
        middle_interval=(0.4, 0.6),
        reference_rotation=gtsam.Rot3(),
        noise_scale=0.5,
    )


def smooth_omega(times: np.ndarray) -> np.ndarray:
    t = np.asarray(times, dtype=float)
    return np.column_stack(
        [
            0.1 + 0.2 * t,
            -0.2 + 0.1 * np.sin(2.0 * np.pi * t),
            0.3 + 0.05 * np.cos(2.0 * np.pi * t),
        ]
    )


if __name__ == "__main__":
    unittest.main()
