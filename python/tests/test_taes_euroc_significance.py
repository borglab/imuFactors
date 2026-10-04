"""Synthetic tests for clustered significance and manuscript formatting."""

import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "reports"))
from taes_euroc_significance import (
    holm_adjust, paired_cluster_median_test, select_noise_group,
)
from generate_taes_paper_euroc_section import highlight_lower


class EurocSignificanceTests(unittest.TestCase):
    def test_exact_paired_cluster_test_and_repeated_seeds(self):
        rows = [
            dict(dataset=f"snippet{i}", m=41, noise_fraction=0.0,
                 seed_index=seed, method=method, error=value)
            for i in range(3) for seed in range(100)
            for method, value in (("baseline", 2.0), ("PsyChMagPI", 1.0))
        ]
        frame = pd.DataFrame(rows)
        # For identical clustered effects, the pooled median can be determined
        # by either majority: all eight permutations are equally extreme.
        self.assertEqual(paired_cluster_median_test(frame, "baseline", "error"), 1.0)
        frame["error"] += frame["dataset"].str[-1].astype(int) * 10.0
        p_value = paired_cluster_median_test(frame, "baseline", "error")
        single_seed = frame[frame["seed_index"].eq(0)]
        self.assertEqual(p_value, paired_cluster_median_test(single_seed, "baseline", "error"))
        # Ties are not significant, and missing matched trials are rejected.
        frame["error"] = 1.0
        self.assertEqual(paired_cluster_median_test(frame, "baseline", "error"), 1.0)
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            paired_cluster_median_test(frame.iloc[1:], "baseline", "error")

    def test_holm_and_noise_groups(self):
        np.testing.assert_allclose(holm_adjust(np.array([0.04, 0.01, 0.03])), [.06, .03, .06])
        frame = pd.DataFrame({"noise_fraction": [0, .025, .12], "noise_band": ["low", "low", "mid"]})
        self.assertEqual(select_noise_group(frame, "none").index.tolist(), [0])
        self.assertEqual(select_noise_group(frame, "low").index.tolist(), [1])

    def test_only_significant_strict_winner_is_bold(self):
        self.assertFalse(any("textbf" in x for x in highlight_lower(1, 2, False)))
        self.assertFalse(any("textbf" in x for x in highlight_lower(1, 1, True)))
        self.assertIn("textbf", highlight_lower(2, 1, True)[1])
        self.assertTrue(all("SecondRevision" in x for x in highlight_lower(2, 1, True)))


if __name__ == "__main__":
    unittest.main()
