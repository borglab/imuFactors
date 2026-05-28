"""Tests for spectral Chebyshev EuRoC spectrogram helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go

PYTHON_DIR = Path(__file__).resolve().parents[1]
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

from imuFactors.chebyshev_spectrogram import (
    available_signal_groups,
    characteristic_windows,
    fit_spectral_chebyshev_windows,
    interval_window_starts,
    load_euroc_csv,
    plot_average_spectra,
    plot_average_spectrogram_parts,
    plot_coefficient_spectrogram,
    plot_interval_coefficients,
    plot_interval_fit,
    plot_window_characteristics,
)


class ChebyshevSpectrogramTests(unittest.TestCase):
    def test_one_second_window_starts_uses_closed_windows(self) -> None:
        time = np.arange(0.0, 2.5, 0.005)

        starts, steps, dt = interval_window_starts(time)

        np.testing.assert_array_equal(starts, [0, 200])
        self.assertEqual(steps, 200)
        self.assertAlmostEqual(dt, 0.005)

    def test_half_second_window_starts_uses_requested_interval(self) -> None:
        time = np.arange(0.0, 2.0, 0.005)

        starts, steps, _ = interval_window_starts(time, window_seconds=0.5)

        np.testing.assert_array_equal(starts, [0, 100, 200])
        self.assertEqual(steps, 100)

    def test_load_adds_imu_norm_signal_group(self) -> None:
        dataframe = load_euroc_csv(_write_synthetic_csv())

        self.assertIn("gyro_norm", dataframe.columns)
        self.assertIn("accel_norm", dataframe.columns)
        groups = available_signal_groups(dataframe)
        self.assertEqual(groups["imu_norms"], ["gyro_norm", "accel_norm"])
        np.testing.assert_allclose(
            dataframe.loc[0, ["gyro_norm", "accel_norm"]],
            [0.0, np.sqrt(5.0)],
        )

    def test_fit_spectral_chebyshev_windows_recovers_linear_signal(self) -> None:
        with self.subTest("fit"):
            path = _write_synthetic_csv()
            result = fit_spectral_chebyshev_windows(
                path,
                coefficient_count=3,
                columns=["w_x", "a_x"],
            )

            self.assertEqual(result.coefficient_count, 3)
            self.assertEqual(result.degree, 2)
            self.assertEqual(result.window_count, 2)
            self.assertEqual(result.samples.shape, (2, 201, 2))
            self.assertEqual(result.coeffs.shape, (2, 3, 2))
            self.assertEqual(result.coeff_energy.shape, (2, 3))
            np.testing.assert_allclose(result.reconstructed, result.samples, atol=1e-12)

    def test_fit_can_use_imu_norms_and_custom_interval(self) -> None:
        result = fit_spectral_chebyshev_windows(
            _write_synthetic_csv(),
            coefficient_count=4,
            signal_group="imu_norms",
            window_seconds=0.5,
        )

        self.assertEqual(result.columns, ["gyro_norm", "accel_norm"])
        self.assertEqual(result.window_seconds, 0.5)
        self.assertEqual(result.sample_count, 101)
        self.assertEqual(result.window_count, 4)
        self.assertEqual(result.coeffs.shape, (4, 4, 2))

    def test_signal_groups_and_plots_return_figures(self) -> None:
        path = _write_synthetic_csv()
        dataframe = pd.read_csv(path)
        groups = available_signal_groups(dataframe)

        self.assertIn("imu", groups)
        self.assertIn("gyro", groups)

        result = fit_spectral_chebyshev_windows(
            path,
            coefficient_count=4,
            signal_group="imu",
        )
        selected = characteristic_windows(result)

        figures = [
            plot_window_characteristics(result, selected),
            plot_interval_fit(result, selected["easy"]),
            plot_interval_coefficients(result, selected["easy"]),
            plot_coefficient_spectrogram(result),
            plot_average_spectra(result),
            plot_average_spectrogram_parts(result),
        ]

        for figure in figures:
            self.assertIsInstance(figure, go.Figure)


def _write_synthetic_csv() -> Path:
    path = Path("/tmp/imu_factors_chebyshev_synthetic.csv")
    time = np.arange(0.0, 2.005, 0.005)
    rows = pd.DataFrame(
        {
            "t": time,
            "q_w": 1.0,
            "q_x": 0.0,
            "q_y": 0.0,
            "q_z": 0.0,
            "v_x": time,
            "v_y": 2.0 * time,
            "v_z": -time,
            "p_x": time**2,
            "p_y": 0.5 * time,
            "p_z": -0.25 * time,
            "b_w_x": 0.01,
            "b_w_y": 0.02,
            "b_w_z": 0.03,
            "b_a_x": -0.01,
            "b_a_y": -0.02,
            "b_a_z": -0.03,
            "w_x": time,
            "w_y": 2.0 * time,
            "w_z": 3.0 * time,
            "a_x": 1.0 + time,
            "a_y": 2.0 - time,
            "a_z": 0.5 * time,
        }
    )
    rows.to_csv(path, index=False)
    return path


if __name__ == "__main__":
    unittest.main()
