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

from imuFactors.quadrature import scalar
from imuFactors.quadrature.scalar import (
    ScalarFunction,
    chebyshev_trial_curves,
    plot_fixed_sample_comparison,
    run_scalar_monte_carlo,
    scalar_function_from_chebyshev2_nodes,
    trapezoid_integral_curve,
)


class ScalarQuadratureTests(unittest.TestCase):
    def test_chebyshev_integrates_quadratic_polynomial_exactly(self) -> None:
        sample_times = np.linspace(0.0, 1.0, 7)
        sample_values = sample_times**2
        evaluation_times = np.linspace(0.0, 1.0, 11)

        final, curve = chebyshev_trial_curves(
            sample_times, sample_values, evaluation_times, node_count=3
        )

        np.testing.assert_allclose(final, 1.0 / 3.0, atol=1e-12)
        np.testing.assert_allclose(curve, evaluation_times**3 / 3.0, atol=1e-12)

    def test_chebyshev_path_calls_gtsam_directly(self) -> None:
        calls: list[str] = []

        class FakeChebyshev2:
            @staticmethod
            def WeightMatrix(node_count, times, a, b):
                calls.append("WeightMatrix")
                self = FakeChebyshev2
                self.assert_interval(a, b)
                if node_count == 2:
                    return np.eye(2)
                return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

            @staticmethod
            def IntegrationWeights(node_count, a, b):
                calls.append("IntegrationWeights")
                return np.array([0.25, 0.75])

            @staticmethod
            def IntegrationMatrix(node_count, a, b):
                calls.append("IntegrationMatrix")
                return np.array(
                    [[0.0, 0.0], [0.0, 0.0], [0.25, 0.75]]
                )

            @staticmethod
            def assert_interval(a, b):
                assert a == 0.0
                assert b == 1.0

        with patch.object(scalar.gtsam, "Chebyshev2", FakeChebyshev2):
            final, curve = chebyshev_trial_curves(
                sample_times=[0.0, 1.0],
                sample_values=[2.0, 4.0],
                evaluation_times=[0.0, 1.0],
                node_count=2,
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

        curve = trapezoid_integral_curve(
            sample_times, sample_values, evaluation_times
        )

        expected = evaluation_times**2 + evaluation_times
        np.testing.assert_allclose(curve, expected, atol=1e-12)

    def test_scalar_function_from_chebyshev2_nodes_matches_node_values(self) -> None:
        node_values = np.array([-0.25, 0.5, -0.1, 0.2, 0.0])
        scalar_function = scalar_function_from_chebyshev2_nodes(
            "fixed-poly", node_values
        )
        nodes = scalar.gtsam.Chebyshev2.Points(5, 0.0, 1.0)

        np.testing.assert_allclose(scalar_function.value(nodes), node_values)
        np.testing.assert_allclose(
            scalar_function.antiderivative(np.array([0.0])), [0.0], atol=1e-12
        )

    def test_run_scalar_monte_carlo_masks_node_counts_above_sample_count(self) -> None:
        linear = ScalarFunction(
            name="linear",
            value=lambda t: t,
            antiderivative=lambda t: 0.5 * t**2,
        )

        result = run_scalar_monte_carlo(
            [linear],
            sample_counts=[2],
            chebyshev_node_counts=[2, 3],
            noise_fractions=[0.0],
            num_seeds=3,
            seed=42,
            evaluation_count=9,
        )

        self.assertEqual(set(result.comparisons["chebyshev_nodes"]), {2})
        self.assertNotIn(3, set(result.method_metrics["chebyshev_nodes"].dropna()))

    def test_run_scalar_monte_carlo_is_deterministic(self) -> None:
        quadratic = ScalarFunction(
            name="quadratic",
            value=lambda t: t**2,
            antiderivative=lambda t: t**3 / 3.0,
        )
        kwargs = dict(
            functions=[quadratic],
            sample_counts=[4, 5],
            chebyshev_node_counts=[2, 3],
            noise_fractions=[0.0, 0.05],
            num_seeds=4,
            seed=7,
            evaluation_count=7,
        )

        first = run_scalar_monte_carlo(**kwargs)
        second = run_scalar_monte_carlo(**kwargs)

        pd.testing.assert_frame_equal(first.method_metrics, second.method_metrics)
        pd.testing.assert_frame_equal(first.comparisons, second.comparisons)

    def test_plot_fixed_sample_comparison_returns_figure(self) -> None:
        comparisons = pd.DataFrame(
            {
                "function": ["linear", "linear", "linear", "linear"],
                "noise_std": [0.0, 0.1, 0.0, 0.1],
                "sample_count": [10, 10, 10, 10],
                "chebyshev_nodes": [4, 4, 5, 5],
                "end_error": [0.0, 0.1, -0.1, 0.2],
                "rmse_error": [0.0, -0.1, -0.2, 0.1],
                "max_error": [0.0, 0.2, -0.1, 0.3],
            }
        )

        figure = plot_fixed_sample_comparison(
            comparisons,
            "linear",
            selected_sample_counts=[10],
            metrics=["end_error"],
        )

        self.assertEqual(len(figure.axes), 2)


if __name__ == "__main__":
    unittest.main()
