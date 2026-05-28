"""Tests for scalar Chebyshev2 and trapezoid quadrature experiments."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pandas as pd

PYTHON_DIR = Path(__file__).resolve().parents[1]
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

import imuFactors.scalar_quadrature as scalar_quadrature
import imuFactors.spectral as spectral


class ScalarQuadratureTests(unittest.TestCase):
    def test_chebyshev_integrates_quadratic_polynomial_exactly(self) -> None:
        sample_times = np.linspace(0.0, 1.0, 7)
        sample_values = sample_times**2
        evaluation_times = np.linspace(0.0, 1.0, 11)

        final, curve = scalar_quadrature.chebyshev_trial_curves(
            sample_times, sample_values, evaluation_times, N=3
        )

        np.testing.assert_allclose(final, 1.0 / 3.0, atol=1e-12)
        np.testing.assert_allclose(curve, evaluation_times**3 / 3.0, atol=1e-12)

    def test_chebyshev_lambda1_regularizes_trial_curve(self) -> None:
        sample_times = np.linspace(0.0, 1.0, 8)
        sample_values = np.array([0.0, 1.0, -0.8, 0.7, -0.6, 0.5, -0.4, 0.0])
        evaluation_times = np.linspace(0.0, 1.0, 21)

        _, unregularized = scalar_quadrature.chebyshev_trial_curves(
            sample_times, sample_values, evaluation_times, N=6
        )
        _, regularized = scalar_quadrature.chebyshev_trial_curves(
            sample_times,
            sample_values,
            evaluation_times,
            N=6,
            lambda1=10.0,
        )

        self.assertGreater(np.linalg.norm(unregularized - regularized), 1e-6)

    def test_chebyshev_path_calls_gtsam_directly(self) -> None:
        calls: list[str] = []

        class FakeChebyshev2:
            @staticmethod
            def WeightMatrix(N, times, a, b):
                calls.append("WeightMatrix")
                self = FakeChebyshev2
                self.assert_interval(a, b)
                if N == 2:
                    return np.eye(2)
                return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

            @staticmethod
            def IntegrationWeights(N, a, b):
                calls.append("IntegrationWeights")
                return np.array([0.25, 0.75])

            @staticmethod
            def IntegrationMatrix(N, a, b):
                calls.append("IntegrationMatrix")
                return np.array([[0.0, 0.0], [0.0, 0.0], [0.25, 0.75]])

            @staticmethod
            def assert_interval(a, b):
                assert a == 0.0
                assert b == 1.0

        with patch.object(spectral.gtsam, "Chebyshev2", FakeChebyshev2):
            final, curve = scalar_quadrature.chebyshev_trial_curves(
                sample_times=[0.0, 1.0],
                sample_values=[2.0, 4.0],
                evaluation_times=[0.0, 1.0],
                N=2,
            )

        self.assertEqual(final, 3.5)
        np.testing.assert_allclose(curve, [0.0, 3.5])
        self.assertIn("WeightMatrix", calls)
        self.assertIn("IntegrationWeights", calls)
        self.assertIn("IntegrationMatrix", calls)

    def test_trapezoid_integral_curve_is_exact_for_linear_interpolant(self) -> None:
        sample_times = np.array([0.0, 0.25, 0.75, 1.0])
        sample_values = 2.0 * sample_times + 1.0
        evaluation_times = np.linspace(0.0, 1.0, 9)

        curve = scalar_quadrature.trapezoid_integral_curve(
            sample_times, sample_values, evaluation_times
        )

        expected = evaluation_times**2 + evaluation_times
        np.testing.assert_allclose(curve, expected, atol=1e-12)

    def test_scalar_function_from_chebyshev2_nodes_matches_node_values(self) -> None:
        node_values = np.array([-0.25, 0.5, -0.1, 0.2, 0.0])
        scalar_function = scalar_quadrature.scalar_function_from_chebyshev2_nodes(
            "fixed-poly", node_values
        )
        nodes = spectral.chebyshev2_points(5)

        np.testing.assert_allclose(scalar_function.value(nodes), node_values)
        np.testing.assert_allclose(
            scalar_function.antiderivative(np.array([0.0])), [0.0], atol=1e-12
        )

    def test_run_scalar_monte_carlo_masks_Ns_above_m(self) -> None:
        linear = scalar_quadrature.ScalarFunction(
            name="linear",
            value=lambda t: t,
            antiderivative=lambda t: 0.5 * t**2,
        )

        result = scalar_quadrature.run_scalar_monte_carlo(
            [linear],
            m_values=[2],
            N_values=[2, 3],
            noise_fractions=[0.0],
            num_seeds=3,
            seed=42,
            evaluation_count=9,
            lambda1=0.25,
        )

        self.assertEqual(set(result.comparisons["N"]), {2})
        self.assertNotIn(3, set(result.method_metrics["N"].dropna()))
        self.assertEqual(set(result.comparisons["n"]), {1})
        self.assertEqual(set(result.comparisons["lambda1"]), {0.25})

    def test_run_scalar_monte_carlo_is_deterministic(self) -> None:
        quadratic = scalar_quadrature.ScalarFunction(
            name="quadratic",
            value=lambda t: t**2,
            antiderivative=lambda t: t**3 / 3.0,
        )
        kwargs = dict(
            functions=[quadratic],
            m_values=[4, 5],
            N_values=[2, 3],
            noise_fractions=[0.0, 0.05],
            num_seeds=4,
            seed=7,
            evaluation_count=7,
        )

        first = scalar_quadrature.run_scalar_monte_carlo(**kwargs)
        second = scalar_quadrature.run_scalar_monte_carlo(**kwargs)

        pd.testing.assert_frame_equal(first.method_metrics, second.method_metrics)
        pd.testing.assert_frame_equal(first.comparisons, second.comparisons)

    def test_plot_fixed_m_comparison_returns_figure(self) -> None:
        comparisons = pd.DataFrame(
            {
                "function": ["linear", "linear", "linear", "linear"],
                "noise_std": [0.0, 0.1, 0.0, 0.1],
                "m": [10, 10, 10, 10],
                "N": [4, 4, 5, 5],
                "end_error": [0.0, 0.1, -0.1, 0.2],
                "rmse_error": [0.0, -0.1, -0.2, 0.1],
                "max_error": [0.0, 0.2, -0.1, 0.3],
            }
        )

        figure = scalar_quadrature.plot_fixed_m_comparison(
            comparisons,
            "linear",
            selected_m_values=[10],
            metrics=["end_error"],
        )

        self.assertEqual(len(figure.axes), 2)

    def test_robust_N_summary_prefers_highest_median_advantage(self) -> None:
        comparisons = pd.DataFrame(
            {
                "function": ["linear"] * 4,
                "noise_fraction": [0.0, 0.1, 0.0, 0.1],
                "noise_std": [0.0, 0.1, 0.0, 0.1],
                "m": [10, 10, 10, 10],
                "N": [3, 3, 5, 5],
                "end_error": [0.1, 0.2, -0.3, -0.4],
                "rmse_error": [0.2, 0.3, -0.4, -0.5],
                "max_error": [0.3, 0.4, -0.5, -0.6],
            }
        )

        summary = scalar_quadrature.robust_N_summary(
            comparisons,
            "linear",
            selected_m_values=[10],
        )

        self.assertEqual(summary.loc[0, "best_end_error_N"], 5)
        self.assertEqual(summary.loc[0, "best_rmse_error_N"], 5)
        self.assertEqual(summary.loc[0, "best_max_error_N"], 5)
        self.assertEqual(summary.loc[0, "robust_N"], 5)
        self.assertEqual(summary.loc[0, "win_rate"], 1.0)

    def test_plotly_advantage_views_return_figures(self) -> None:
        comparisons = pd.DataFrame(
            {
                "function": ["linear", "linear", "linear", "linear"],
                "noise_fraction": [0.0, 0.1, 0.0, 0.1],
                "noise_std": [0.0, 0.1, 0.0, 0.1],
                "m": [10, 10, 10, 10],
                "N": [4, 4, 5, 5],
                "end_error": [0.0, 0.1, -0.1, 0.2],
                "rmse_error": [0.0, -0.1, -0.2, 0.1],
                "max_error": [0.0, 0.2, -0.1, 0.3],
            }
        )

        curve_figure = scalar_quadrature.plot_advantage_curves_by_m(
            comparisons,
            "linear",
            selected_m_values=[10],
            metric="rmse_error",
        )
        table_figure = scalar_quadrature.plot_robust_N_table(
            comparisons,
            "linear",
            selected_m_values=[10],
        )

        self.assertGreater(len(curve_figure.data), 0)
        self.assertEqual(len(table_figure.data), 1)

    def test_advantage_curve_uses_robust_y_range(self) -> None:
        rows = []
        for index, N in enumerate(range(2, 22)):
            rows.append(
                {
                    "function": "linear",
                    "noise_fraction": 0.01 * index,
                    "noise_std": 0.01 * index,
                    "m": 30,
                    "N": N,
                    "end_error": -float(index),
                    "rmse_error": -1000.0 if N == 21 else -float(index),
                    "max_error": -float(index),
                }
            )
        comparisons = pd.DataFrame(rows)

        figure = scalar_quadrature.plot_advantage_curves_by_m(
            comparisons,
            "linear",
            selected_m_values=[30],
            metric="rmse_error",
        )

        y_range = figure.layout.yaxis.range
        self.assertLess(y_range[1], 1000.0)
        self.assertLessEqual(y_range[0], 0.0)

    def test_advantage_curve_y_range_can_ignore_small_m_values(self) -> None:
        comparisons = pd.DataFrame(
            {
                "function": ["linear"] * 9,
                "noise_fraction": [0.01] * 9,
                "noise_std": [0.01] * 9,
                "m": [20] * 9,
                "N": list(range(2, 11)),
                "end_error": [0.0] * 9,
                "rmse_error": [
                    20.0,
                    18.0,
                    16.0,
                    -0.01,
                    -0.02,
                    -0.01,
                    0.0,
                    -0.01,
                    -0.02,
                ],
                "max_error": [0.0] * 9,
            }
        )

        figure = scalar_quadrature.plot_advantage_curves_by_m(
            comparisons,
            "linear",
            selected_m_values=[20],
            metric="rmse_error",
            y_range_min_N=5,
        )

        y_range = figure.layout.yaxis.range
        self.assertGreater(y_range[0], -1.0)


if __name__ == "__main__":
    unittest.main()
