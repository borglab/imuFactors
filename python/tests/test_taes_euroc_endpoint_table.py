"""Tests for the cached EuRoC endpoint RMS aggregation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

REPORT_DIR = Path(__file__).resolve().parents[2] / "reports"
if str(REPORT_DIR) not in sys.path:
    sys.path.insert(0, str(REPORT_DIR))

from taes_euroc_endpoint_table import (
    EUROC_RMS_METHOD_COLUMNS,
    endpoint_metrics_for_sqrt_rule,
    endpoint_rms_by_exact_noise,
    euroc_rms_method_values,
)


class TaesEurocEndpointTableTests(unittest.TestCase):
    def test_rms_conversion_order_and_unavailable_columns(self) -> None:
        methods = ("D2/RK2", "D3 single-speed", "PsyChMagPI")
        errors_by_method = {
            "D2/RK2": (1.0, 3.0),
            "D3 single-speed": (2.0, 4.0),
            "PsyChMagPI": (0.5, 1.5),
        }
        rows = []
        for noise_fraction in (0.0, 0.1):
            for method in methods:
                for error_deg in errors_by_method[method]:
                    rows.append(
                        {
                            "m": 41,
                            "N_paper": 13,
                            "benchmark_version": 5,
                            "noise_fraction": noise_fraction,
                            "method": method,
                            "error_deg": error_deg,
                            "converged": True,
                        }
                    )

        # Old M=40 trials must not enter the new 200 Hz table.
        old_rows = [dict(row, m=40, error_deg=1000.0) for row in rows]
        summary = endpoint_rms_by_exact_noise(
            pd.DataFrame(rows + old_rows).sample(frac=1, random_state=7),
            noise_fractions=(0.0, 0.1),
            trials_per_cell=2,
        )

        self.assertEqual(summary["Noise fraction alpha"].tolist(), [0.0, 0.1])
        expected_d2 = np.sqrt((1.0**2 + 3.0**2) / 2.0) * np.pi * 1000.0 / 180.0
        self.assertAlmostEqual(float(summary.iloc[0]["D2/RK2"]), expected_d2)
        values = euroc_rms_method_values(summary.iloc[0])
        self.assertEqual(
            [label for label, _ in EUROC_RMS_METHOD_COLUMNS],
            [
                "No Coning",
                "Two-Speed",
                "Seq. Rot.",
                "One-Speed",
                "PS-CPI (1 iter.)",
                "PS-CPI (convg.)",
            ],
        )
        self.assertIsNone(values[0])
        self.assertIsNone(values[1])
        self.assertAlmostEqual(float(values[2]), expected_d2)
        self.assertIsNone(values[4])

        for column, value, message in (
            ("N_paper", 7, "N_paper=13"),
            ("benchmark_version", 4, "benchmark_version=5"),
            ("converged", False, "non-converged"),
        ):
            with self.subTest(column=column), self.assertRaisesRegex(ValueError, message):
                endpoint_rms_by_exact_noise(
                    pd.DataFrame(rows).assign(**{column: value}),
                    noise_fractions=(0.0, 0.1), trials_per_cell=2,
                )

    def test_rejects_incomplete_cache_cells(self) -> None:
        rows = [
            {
                "m": 41,
                "N_paper": 13,
                "benchmark_version": 5,
                "noise_fraction": 0.0,
                "method": method,
                "error_deg": 1.0,
                "converged": True,
            }
            for method in ("D2/RK2", "D3 single-speed", "PsyChMagPI")
        ]
        rows.pop()
        with self.assertRaisesRegex(ValueError, "methods do not match"):
            endpoint_rms_by_exact_noise(
                pd.DataFrame(rows),
                noise_fractions=(0.0,),
                trials_per_cell=1,
            )

    def test_combines_sqrt_rule_psych_with_invariant_baselines(self) -> None:
        baseline_rows = []
        for method, error_deg in (("D2/RK2", 2.0), ("D3 single-speed", 1.0)):
            baseline_rows.append(
                {
                    "dataset": "MH01",
                    "m": 40,
                    "N_paper": 13,
                    "benchmark_version": 5,
                    "noise_fraction": 0.0,
                    "noise_band": "low",
                    "seed_index": 0,
                    "method": method,
                    "error_deg": error_deg,
                    "converged": True,
                }
            )
        psych_rows = pd.DataFrame(
            [
                {
                    "dataset": "MH01",
                    "m": 40,
                    "N_sqrt": 7,
                    "noise_fraction": 0.0,
                    "noise_band": "low",
                    "seed_index": 0,
                    "method": "PsyChMagPI",
                    "N": 7,
                    "lambda1": 0.0,
                    "error_deg": 0.5,
                    "converged": True,
                },
                {
                    "dataset": "MH01",
                    "m": 40,
                    "N_sqrt": 7,
                    "noise_fraction": 0.0,
                    "noise_band": "low",
                    "seed_index": 0,
                    "method": "PsyChMagPI",
                    "N": 8,
                    "lambda1": 0.0,
                    "error_deg": 0.1,
                    "converged": True,
                },
            ]
        )

        combined = endpoint_metrics_for_sqrt_rule(
            pd.DataFrame(baseline_rows), psych_rows
        )

        self.assertEqual(
            set(combined["method"]),
            {
                "D2/RK2",
                "D3 single-speed",
                "PsyChMagPI",
            },
        )
        self.assertEqual(set(combined["N_paper"]), {7})
        psych = combined[combined["method"].eq("PsyChMagPI")].iloc[0]
        self.assertEqual(float(psych["win_rate"]), 1.0)


if __name__ == "__main__":
    unittest.main()
