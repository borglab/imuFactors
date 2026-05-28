"""Tests for shared spectral basis helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

PYTHON_DIR = Path(__file__).resolve().parents[1]
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

import imuFactors.spectral as spectral


class SpectralTests(unittest.TestCase):
    def test_builds_chebyshev1_chebyshev2_and_fourier_plans(self) -> None:
        for basis, N in [
            ("chebyshev", 4),
            ("chebyshev2", 4),
            ("fourier", 5),
        ]:
            with self.subTest(basis=basis):
                plan = spectral.basis_plan(N, m=11, basis=basis)
                coeffs = np.arange(N, dtype=float)
                values = plan.reconstruct(coeffs)
                recovered = plan.fit(values)

                self.assertEqual(plan.basis, basis)
                self.assertEqual(plan.N, N)
                self.assertEqual(plan.m, 11)
                self.assertEqual(plan.n, N - 1)
                self.assertEqual(plan.weight_matrix.shape, (11, N))
                self.assertEqual(plan.solver.shape, (N, 11))
                np.testing.assert_allclose(recovered, coeffs, atol=1e-10)

    def test_chebyshev2_recovers_polynomial_node_values(self) -> None:
        def polynomial(t: np.ndarray) -> np.ndarray:
            return 1.0 - 2.0 * t + 0.5 * t**2 + 3.0 * t**3

        sample_times = np.linspace(0.0, 1.0, 12)
        plan = spectral.basis_plan_from_coordinates(4, sample_times, basis="chebyshev2")
        coeffs = plan.fit(polynomial(sample_times))
        nodes = spectral.chebyshev2_points(4)

        np.testing.assert_allclose(coeffs, polynomial(nodes), atol=1e-12)
        np.testing.assert_allclose(
            plan.reconstruct(coeffs), polynomial(sample_times), atol=1e-12
        )

    def test_chebyshev2_integration_matrix_matches_antiderivative(self) -> None:
        N = 3
        nodes = spectral.chebyshev2_points(N)
        values = 2.0 * nodes
        integration_matrix = spectral.chebyshev2_integration_matrix(N)
        integral_nodes = spectral.chebyshev2_points(N + 1)

        np.testing.assert_allclose(
            integration_matrix @ values, integral_nodes**2, atol=1e-12
        )
        np.testing.assert_allclose(
            spectral.chebyshev2_integration_weights(N) @ values,
            1.0,
            atol=1e-12,
        )

    def test_lambda1_first_derivative_penalty_is_sobolev_energy(self) -> None:
        nodes = spectral.chebyshev2_points(5)
        penalty = spectral.chebyshev2_first_derivative_penalty(5)
        constant = np.ones(5)

        self.assertGreater(np.linalg.norm(penalty), 0.0)
        self.assertAlmostEqual(float(constant @ penalty @ constant), 0.0)
        self.assertAlmostEqual(float(nodes @ penalty @ nodes), 1.0)

    def test_invalid_basis_and_regularization_errors(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unknown basis"):
            spectral.basis_plan(3, 10, basis="not-a-basis")
        with self.assertRaisesRegex(ValueError, "lambda1"):
            spectral.basis_plan(3, 10, basis="chebyshev2", lambda1=-1.0)
        with self.assertRaisesRegex(ValueError, "Chebyshev2"):
            spectral.basis_plan(3, 10, basis="fourier", lambda1=1.0)


if __name__ == "__main__":
    unittest.main()
