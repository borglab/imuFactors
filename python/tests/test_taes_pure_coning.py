"""Tests for the TAES pure-coning reproduction helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

PYTHON_DIR = Path(__file__).resolve().parents[1]
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

from imuFactors.gyro_integration import rotation_error_degrees
from imuFactors.gyro_proxy_monte_carlo import integrate_gyro_function_reference
from imuFactors.taes_pure_coning import (
    METHOD_ORDER,
    TAES_INTERVAL_OBSERVATIONS,
    TAES_N_VALUES,
    TAES_NOISE_SIGMAS,
    pure_coning_omega,
    pure_coning_reference_rotation,
    run_taes_pure_coning,
    run_taes_table2_long_integration_monte_carlo,
)


class TaesPureConingTests(unittest.TestCase):
    def test_analytic_reference_matches_ode_integration(self) -> None:
        for time_s in (0.08, 0.4, 1.2, 2.0):
            analytic = pure_coning_reference_rotation(time_s)
            numeric = integrate_gyro_function_reference(
                pure_coning_omega,
                (0.0, time_s),
            )
            self.assertLess(rotation_error_degrees(analytic, numeric), 1e-8)

    def test_default_grid_is_finite_and_has_expected_shape(self) -> None:
        result = run_taes_pure_coning()
        expected_blocks = int(200 / TAES_INTERVAL_OBSERVATIONS)
        expected_rows = (
            expected_blocks
            * len(TAES_N_VALUES)
            * len(TAES_NOISE_SIGMAS)
            * len(METHOD_ORDER)
        )

        self.assertEqual(len(result.errors), expected_rows)
        self.assertEqual(
            len(result.summary),
            len(TAES_N_VALUES) * len(TAES_NOISE_SIGMAS) * len(METHOD_ORDER),
        )
        self.assertFalse(result.errors["error_rad"].isna().any())
        self.assertTrue(np.isfinite(result.errors["error_rad"]).all())
        self.assertTrue(result.errors[result.errors["method"] == "PsyChMagPI"]["converged"].all())

    def test_noiseless_final_errors_have_expected_classical_ordering(self) -> None:
        result = run_taes_pure_coning(N_values=(6,), noise_sigmas=(0.0,))
        final = result.summary.set_index("method")["final_error_rad"]

        self.assertLess(final["D3 single-speed"], final["D2 sequential rotations"])
        self.assertLess(final["D2 sequential rotations"], final["D1 no coning"])
        self.assertLess(final["PsyChMagPI"], final["D3 single-speed"])

    def test_table_monte_carlo_summary_is_finite(self) -> None:
        result = run_taes_table2_long_integration_monte_carlo(
            noise_sigmas=(0.0, 1e-4),
            lambda_values=(0.0, 0.01),
            num_trials=3,
        )

        self.assertEqual(len(result.trial_errors), 20)
        self.assertEqual(len(result.summary), 10)
        self.assertTrue(np.isfinite(result.summary["rms_error_mrad"]).all())
        self.assertTrue(
            result.summary[
                result.summary["method"].astype(str).str.startswith("PsyChMagPI")
            ]["converged_rate"].eq(1.0).all()
        )

    def test_table2_noiseless_common_methods_match_draft(self) -> None:
        result = run_taes_table2_long_integration_monte_carlo(
            noise_sigmas=(0.0,),
            lambda_values=(0.0,),
            num_trials=3,
        )
        summary = result.summary.set_index("method")["rms_error_mrad"]

        self.assertAlmostEqual(summary["D1 no coning"], 16.689, places=3)
        self.assertAlmostEqual(summary["D2 sequential rotations"], 0.719, places=3)
        self.assertAlmostEqual(summary["D3 single-speed"], 0.696, places=3)
        self.assertAlmostEqual(summary["PsyChMagPI lambda=0"], 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
