"""Tests for the EuRoC gyro proxy Monte Carlo helper."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

PYTHON_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PYTHON_DIR.parent
REPORT_DIR = REPO_ROOT / "reports"
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))
if str(REPORT_DIR) not in sys.path:
    sys.path.insert(0, str(REPORT_DIR))

import generate_chebyshev_quadrature_report as chebyshev_report
from imuFactors.gyro_proxy_monte_carlo import (
    ALGORITHM_ORDER,
    load_proxy_snippets,
    run_gyro_proxy_monte_carlo,
    standard_noise,
)


class GyroProxyMonteCarloTests(unittest.TestCase):
    def test_standard_noise_is_reproducible_and_keyed_by_trial_context(self) -> None:
        first = standard_noise(20260523, snippet_index=0, m=10, num_seeds=3)
        repeat = standard_noise(20260523, snippet_index=0, m=10, num_seeds=3)
        different_snippet = standard_noise(20260523, snippet_index=1, m=10, num_seeds=3)
        different_count = standard_noise(20260523, snippet_index=0, m=20, num_seeds=3)

        self.assertEqual(first.shape, (3, 10, 3))
        np.testing.assert_allclose(first, repeat)
        self.assertFalse(np.allclose(first, different_snippet))
        self.assertFalse(np.allclose(first[:, :10], different_count[:, :10]))

    def test_small_proxy_monte_carlo_is_finite_and_skips_invalid_N(self) -> None:
        snippets = load_proxy_snippets(
            chebyshev_report.load_or_build_euroc_cache(),
            data_dir=REPO_ROOT / "data" / "euroc",
        )[:1]

        result = run_gyro_proxy_monte_carlo(
            snippets,
            m_values=[3],
            N_values=[2, 3, 5],
            lambda_values=[0.0, 0.05],
            noise_fractions=[0.0, 0.1],
            num_seeds=2,
            seed=20260523,
        )
        metrics = result.method_metrics
        psych_metrics = metrics[metrics["method"] == "PsyChMagPI"]

        self.assertEqual(len(metrics), 24)
        self.assertFalse((psych_metrics["N"] > psych_metrics["m"]).any())
        self.assertTrue(np.all(np.isfinite(metrics["error_deg"])))
        self.assertTrue(np.all(np.isfinite(result.algorithm_trials["error_deg"])))
        self.assertEqual(
            set(result.algorithm_trials["algorithm"].astype(str).unique()),
            set(ALGORITHM_ORDER),
        )
        self.assertTrue(result.best_settings_by_band["N"].isin([2, 3]).all())


if __name__ == "__main__":
    unittest.main()
