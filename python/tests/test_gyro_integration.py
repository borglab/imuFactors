"""Tests for SO(3) gyro integration helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import gtsam
import numpy as np

PYTHON_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PYTHON_DIR.parent
REPORT_DIR = REPO_ROOT / "reports"
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))
if str(REPORT_DIR) not in sys.path:
    sys.path.insert(0, str(REPORT_DIR))

from imuFactors.gyro_integration import (
    fit_vector_samples_to_chebyshev_nodes,
    integrate_d3_single_speed_rot3,
    integrate_magnus2_rot3,
    integrate_psychmagpi_rot3,
    integrate_rk2_rot3,
    rotation_error_degrees,
)
from imuFactors.gyro_proxy_monte_carlo import (
    integrate_gyro_function_reference,
    load_proxy_snippets,
)


class GyroIntegrationTests(unittest.TestCase):
    def test_constant_omega_matches_expmap_reference(self) -> None:
        times = np.linspace(0.0, 1.0, 21)
        omega = np.tile(np.array([0.1, -0.2, 0.3]), (times.size, 1))
        reference = gtsam.Rot3.Expmap(omega[0] * (times[-1] - times[0]))
        solve_ivp_reference = integrate_gyro_function_reference(
            lambda query_times: np.tile(omega[0], (np.asarray(query_times).size, 1)),
            (float(times[0]), float(times[-1])),
        )
        initial = gtsam.Rot3()

        rk2 = integrate_rk2_rot3(initial, times, omega)
        d3 = integrate_d3_single_speed_rot3(initial, times, omega)
        magnus2 = integrate_magnus2_rot3(initial, times, omega)
        psychmagpi = integrate_psychmagpi_rot3(
            initial,
            times,
            omega,
            N=5,
            tolerance=1e-12,
            max_iterations=50,
        )
        regularized_psychmagpi = integrate_psychmagpi_rot3(
            initial,
            times,
            omega,
            N=5,
            tolerance=1e-12,
            max_iterations=50,
            lambda1=0.05,
        )
        approximate_psychmagpi = integrate_psychmagpi_rot3(
            initial,
            times,
            omega,
            N=5,
            tolerance=1e-12,
            max_iterations=50,
            use_approximate_integration_matrix=True,
        )

        self.assertLess(rotation_error_degrees(reference, solve_ivp_reference), 1e-10)
        self.assertLess(rotation_error_degrees(reference, rk2), 1e-9)
        self.assertLess(rotation_error_degrees(reference, d3), 1e-9)
        self.assertLess(rotation_error_degrees(reference, magnus2), 1e-9)
        self.assertLess(rotation_error_degrees(reference, psychmagpi.rotation), 1e-9)
        self.assertLess(
            rotation_error_degrees(reference, regularized_psychmagpi.rotation),
            1e-9,
        )
        self.assertLess(
            rotation_error_degrees(reference, approximate_psychmagpi.rotation),
            1e-9,
        )
        self.assertTrue(psychmagpi.converged)
        self.assertTrue(regularized_psychmagpi.converged)
        self.assertTrue(approximate_psychmagpi.converged)
        self.assertTrue(approximate_psychmagpi.use_approximate_integration_matrix)

    def test_time_varying_omega_returns_valid_rotations(self) -> None:
        times = np.linspace(0.0, 0.5, 51)
        omega = np.column_stack(
            [
                0.25 + 0.1 * np.sin(2.0 * np.pi * times),
                -0.15 + 0.05 * np.cos(2.0 * np.pi * times),
                0.2 * times,
            ]
        )
        initial = gtsam.Rot3()

        rotations = [
            integrate_rk2_rot3(initial, times, omega),
            integrate_d3_single_speed_rot3(initial, times, omega),
            integrate_magnus2_rot3(initial, times, omega),
            integrate_psychmagpi_rot3(initial, times, omega, N=8).rotation,
        ]

        for rotation in rotations:
            matrix = np.asarray(rotation.matrix(), dtype=float)
            self.assertTrue(np.all(np.isfinite(matrix)))
            np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(float(np.linalg.det(matrix)), 1.0, places=12)

    def test_chebyshev_node_fit_recovers_linear_vector_signal(self) -> None:
        times = np.linspace(0.0, 1.0, 51)
        values = np.column_stack(
            [
                1.0 + 2.0 * times,
                -0.3 * times,
                0.5 - 0.7 * times,
            ]
        )

        nodes, nodal_values, fitted_samples, fit_error = (
            fit_vector_samples_to_chebyshev_nodes(times, values, N=3)
        )

        self.assertEqual(nodes.shape, (3,))
        self.assertEqual(nodal_values.shape, (3, 3))
        np.testing.assert_allclose(fitted_samples, values, atol=1e-12)
        self.assertLess(fit_error, 1e-12)

        regularized_nodes, regularized_values, regularized_fit, regularized_error = (
            fit_vector_samples_to_chebyshev_nodes(
                times,
                values,
                N=3,
                lambda1=0.05,
            )
        )
        self.assertEqual(regularized_nodes.shape, (3,))
        self.assertEqual(regularized_values.shape, (3, 3))
        self.assertEqual(regularized_fit.shape, values.shape)
        self.assertTrue(np.isfinite(regularized_error))

    def test_proxy_loader_matches_cached_euroc_selection(self) -> None:
        import generate_chebyshev_quadrature_report as chebyshev_report

        snippets = load_proxy_snippets(
            chebyshev_report.load_or_build_euroc_cache(),
            data_dir=REPO_ROOT / "data" / "euroc",
        )

        self.assertEqual(len(snippets), 11)
        self.assertEqual(snippets[0].dataset, "MH01")
        self.assertEqual(snippets[-1].dataset, "V203")
        self.assertTrue(all(snippet.csv_path.exists() for snippet in snippets))
        self.assertTrue(all(snippet.window_start_s >= 0.0 for snippet in snippets))

    def test_proxy_loader_uses_one_second_fit_and_center_200ms_window(self) -> None:
        import generate_chebyshev_quadrature_report as chebyshev_report

        snippet = load_proxy_snippets(
            chebyshev_report.load_or_build_euroc_cache(),
            data_dir=REPO_ROOT / "data" / "euroc",
        )[0]
        timestamp_tolerance = 1e-6
        center_mask = (
            (snippet.sample_seconds >= snippet.middle_interval[0] - timestamp_tolerance)
            & (snippet.sample_seconds <= snippet.middle_interval[1] + timestamp_tolerance)
        )

        self.assertEqual(snippet.sample_seconds.size, 201)
        self.assertEqual(int(np.count_nonzero(center_mask)), 41)
        self.assertEqual(snippet.raw_bias_corrected_gyro.shape, (201, 3))
        self.assertEqual(snippet.cgl_node_values.shape, (50, 3))
        self.assertEqual(snippet.reconstructed_samples.shape, (201, 3))
        self.assertAlmostEqual(float(snippet.middle_interval[0]), 0.4, places=12)
        self.assertAlmostEqual(float(snippet.middle_interval[1]), 0.6, places=12)
        self.assertGreaterEqual(snippet.noise_scale, 0.0)


if __name__ == "__main__":
    unittest.main()
