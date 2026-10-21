"""Mode-selection and reporting regression checks; run in conda lidar."""

from pathlib import Path
import sys
import unittest

import gtsam
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "reports"))
from run_taes_nodal_monte_carlo import (
    DEG_TO_MRAD, exact_noise_summary, make_integrator, paired_trials,
)


class NodalMonteCarloTests(unittest.TestCase):
    def test_square_modes_and_legacy_reproduction(self):
        nodes = np.asarray(gtsam.Chebyshev2.Points(7, 0, .2))
        gyro = np.array([1 + nodes**2, 2 * nodes, np.sin(nodes * 4)])
        pseudo = make_integrator(7, (0, .2), "PSEUDOINVERSE").integrateWithHistory(gyro)
        nodal = make_integrator(7, (0, .2), "EXACT_NODAL").integrateWithHistory(gyro)
        legacy = gtsam.PsyChMagPIRot3(7, 0, .2, gtsam.PseudoSpectralPiParams(
            gtsam.PseudoSpectralPiInit.RK2, 1e-12, 50, True
        )).integrateWithHistory(gyro)
        self.assertEqual(pseudo.finalZ.shape, (3, 7))
        self.assertEqual(nodal.finalZ.shape, (3, 7))
        self.assertTrue(pseudo.converged and nodal.converged)
        np.testing.assert_allclose(pseudo.finalZ, legacy.finalZ, atol=1e-15)
        self.assertGreater(np.linalg.norm(pseudo.finalZ - nodal.finalZ), 1e-10)

    def test_nodal_integrates_quadratic_exactly(self):
        nodes = np.asarray(gtsam.Chebyshev2.Points(3))
        exact = gtsam.Chebyshev2.NodalIntegrationMatrix(3) @ nodes**2
        pseudo = gtsam.Chebyshev2.ApproximateIntegrationMatrix(3) @ nodes**2
        np.testing.assert_allclose(exact, (nodes**3 + 1) / 3, atol=1e-14)
        self.assertAlmostEqual(pseudo[-1], 4 / 3)
        # Collinear angular velocity eliminates nonlinear coning terms.
        gyro = np.array([nodes**2, np.zeros(3), np.zeros(3)])
        result = make_integrator(3, (-1, 1), "EXACT_NODAL").integrateWithHistory(gyro)
        self.assertAlmostEqual(result.finalZ[0, -1], 2 / 3)

    def test_rms_pairing_units_order_and_rejection(self):
        rows, baselines = [], []
        for count in (41, 10):
            for fraction in (.1, 0):
                for seed, (pseudo, nodal) in enumerate(((3., 1.), (4., 4.))):
                    key = dict(dataset="synthetic", m=count, noise_fraction=fraction, seed_index=seed)
                    for mode, error in (("PSEUDOINVERSE", pseudo), ("EXACT_NODAL", nodal)):
                        rows.append(dict(key, mode=mode, error_deg=error, N=7))
                    baselines.append(dict(key, method="D3 single-speed", error_deg=2))
        data = pd.DataFrame(rows)
        summary = exact_noise_summary(data, pd.DataFrame(baselines))
        self.assertEqual(summary.M.tolist(), [10, 10, 41, 41])
        self.assertAlmostEqual(summary.iloc[0].pseudoinverse_rms_mrad, np.sqrt(12.5) * DEG_TO_MRAD)
        self.assertAlmostEqual(summary.iloc[0].nodal_rms_mrad, np.sqrt(8.5) * DEG_TO_MRAD)
        self.assertEqual(summary.iloc[0].nodal_wins_pct, 50)
        self.assertEqual(summary.iloc[0].ties_pct, 50)
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            paired_trials(pd.concat([data, data.iloc[:1]]))
        with self.assertRaisesRegex(ValueError, "Missing"):
            paired_trials(data.iloc[1:])


if __name__ == "__main__":
    unittest.main()
