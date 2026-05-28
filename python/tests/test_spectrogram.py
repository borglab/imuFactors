"""Tests for EuRoC spectral basis spectrogram helpers."""

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

import imuFactors.euroc as euroc
import imuFactors.spectral as spectral
import imuFactors.spectrogram as spectrogram


class SpectrogramTests(unittest.TestCase):
    def test_one_second_window_starts_uses_closed_windows(self) -> None:
        time = np.arange(0.0, 2.5, 0.005)

        starts, steps, dt = spectrogram.interval_window_starts(time)

        np.testing.assert_array_equal(starts, [0, 200])
        self.assertEqual(steps, 200)
        self.assertAlmostEqual(dt, 0.005)

    def test_half_second_window_starts_uses_requested_interval(self) -> None:
        time = np.arange(0.0, 2.0, 0.005)

        starts, steps, _ = spectrogram.interval_window_starts(
            time, window_seconds=0.5
        )

        np.testing.assert_array_equal(starts, [0, 100, 200])
        self.assertEqual(steps, 100)

    def test_load_adds_imu_norm_signal_group(self) -> None:
        dataframe = euroc.load_euroc_csv(_write_synthetic_csv())

        self.assertIn("gyro_norm", dataframe.columns)
        self.assertIn("accel_norm", dataframe.columns)
        groups = euroc.available_signal_groups(dataframe)
        self.assertEqual(groups["imu_norms"], ["gyro_norm", "accel_norm"])
        np.testing.assert_allclose(
            dataframe.loc[0, ["gyro_norm", "accel_norm"]],
            [0.0, np.sqrt(5.0)],
        )

    def test_load_adds_gravity_compensated_accel_signal_group(self) -> None:
        dataframe = euroc.load_euroc_csv(_write_synthetic_csv())

        self.assertIn("a_gc_x", dataframe.columns)
        self.assertIn("a_gc_y", dataframe.columns)
        self.assertIn("a_gc_z", dataframe.columns)
        groups = euroc.available_signal_groups(dataframe)
        self.assertEqual(
            groups["accel_gravity_compensated"],
            ["a_gc_x", "a_gc_y", "a_gc_z"],
        )
        np.testing.assert_allclose(
            dataframe.loc[0, ["a_gc_x", "a_gc_y", "a_gc_z"]],
            [1.0, 2.0, -euroc.GRAVITY],
        )

    def test_chebyshev_fit_recovers_linear_signal(self) -> None:
        with self.subTest("fit"):
            path = _write_synthetic_csv()
            result = spectrogram.fit_spectral_windows(
                path,
                coefficient_count=3,
                basis="chebyshev",
                columns=["w_x", "a_x"],
            )

            self.assertEqual(result.basis, "chebyshev")
            self.assertEqual(result.coefficient_count, 3)
            self.assertEqual(result.degree, 2)
            self.assertEqual(result.window_count, 2)
            self.assertEqual(result.samples.shape, (2, 201, 2))
            self.assertEqual(result.coeffs.shape, (2, 3, 2))
            self.assertEqual(result.coeff_energy.shape, (2, 3))
            np.testing.assert_allclose(result.reconstructed, result.samples, atol=1e-12)

    def test_fit_can_use_imu_norms_and_custom_interval(self) -> None:
        result = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=4,
            basis="chebyshev",
            signal_group="imu_norms",
            window_seconds=0.5,
        )

        self.assertEqual(result.columns, ["gyro_norm", "accel_norm"])
        self.assertEqual(result.window_seconds, 0.5)
        self.assertEqual(result.sample_count, 101)
        self.assertEqual(result.window_count, 4)
        self.assertEqual(result.coeffs.shape, (4, 4, 2))

    def test_fit_can_use_gravity_compensated_accel(self) -> None:
        result = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=4,
            basis="chebyshev",
            signal_group="accel_gravity_compensated",
        )

        self.assertEqual(result.columns, ["a_gc_x", "a_gc_y", "a_gc_z"])
        self.assertEqual(result.coeffs.shape, (2, 4, 3))

    def test_fourier_fit_uses_gtsam_fourier_basis(self) -> None:
        result = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=5,
            basis="fourier",
            columns=["w_x", "a_x"],
        )

        self.assertEqual(result.basis, "fourier")
        self.assertEqual(result.max_harmonic, 2)
        self.assertEqual(result.coeffs.shape, (2, 5, 2))
        self.assertEqual(
            spectrogram.basis_tick_labels(result),
            ["1", "cos1", "sin1", "cos2", "sin2"],
        )

    def test_chebyshev2_fit_uses_pseudo_spectral_basis(self) -> None:
        result = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=5,
            basis="chebyshev2",
            columns=["w_x", "a_x"],
        )

        self.assertEqual(result.basis, "chebyshev2")
        self.assertEqual(result.coeffs.shape, (2, 5, 2))
        self.assertEqual(
            spectrogram.basis_tick_labels(result),
            ["cgl0", "cgl1", "cgl2", "cgl3", "cgl4"],
        )
        self.assertEqual(
            spectrogram.basis_order_summary(result),
            "5 CGL node values",
        )

    def test_fit_uses_shared_spectral_plan(self) -> None:
        result = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=5,
            basis="chebyshev2",
            columns=["w_x"],
            window_seconds=0.5,
            lambda1=0.1,
        )
        plan = spectral.basis_plan(
            5,
            result.sample_count,
            basis="chebyshev2",
            window_seconds=0.5,
            lambda1=0.1,
        )

        np.testing.assert_allclose(result.basis_coordinates, plan.coordinates)
        np.testing.assert_allclose(result.weight_matrix, plan.weight_matrix)
        self.assertEqual(
            spectrogram.basis_tick_labels(result),
            spectral.basis_tick_labels(result.basis, result.coefficient_count),
        )

    def test_chebyshev2_lambda1_penalizes_first_derivative(self) -> None:
        unregularized = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=6,
            basis="chebyshev2",
            columns=["w_x"],
        )
        regularized = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=6,
            basis="chebyshev2",
            columns=["w_x"],
            lambda1=100.0,
        )
        penalty = spectral.chebyshev2_first_derivative_penalty(
            unregularized.coefficient_count,
            window_seconds=unregularized.window_seconds,
        )

        unregularized_energy = _mean_quadratic_energy(
            unregularized.coeffs[:, :, 0], penalty
        )
        regularized_energy = _mean_quadratic_energy(
            regularized.coeffs[:, :, 0], penalty
        )

        self.assertEqual(regularized.lambda1, 100.0)
        self.assertLess(regularized_energy, unregularized_energy)
        self.assertGreater(np.mean(regularized.rmse), np.mean(unregularized.rmse))

    def test_lambda1_requires_chebyshev2_basis(self) -> None:
        with self.assertRaisesRegex(ValueError, "Chebyshev2"):
            spectrogram.fit_spectral_windows(
                _write_synthetic_csv(),
                coefficient_count=5,
                basis="chebyshev",
                columns=["w_x"],
                lambda1=1.0,
            )

    def test_signal_groups_and_plots_return_figures(self) -> None:
        path = _write_synthetic_csv()
        dataframe = pd.read_csv(path)
        groups = euroc.available_signal_groups(dataframe)

        self.assertIn("imu", groups)
        self.assertIn("gyro", groups)

        result = spectrogram.fit_spectral_windows(
            path,
            coefficient_count=4,
            basis="fourier",
            signal_group="imu",
        )
        selected = spectrogram.characteristic_windows(result)

        figures = [
            spectrogram.plot_window_characteristics(result, selected),
            spectrogram.plot_interval_fit(result, selected["easy"]),
            spectrogram.plot_interval_coefficients(result, selected["easy"]),
            spectrogram.plot_coefficient_spectrogram(result),
            spectrogram.plot_average_spectra(result),
            spectrogram.plot_average_spectrogram_parts(result),
            spectrogram.plot_dataset_average_spectrograms({"synthetic": result}),
        ]

        for figure in figures:
            self.assertIsInstance(figure, go.Figure)

    def test_coefficient_spectrogram_reverses_time_axis(self) -> None:
        result = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=4,
            basis="chebyshev",
            signal_group="gyro",
        )

        figure = spectrogram.plot_coefficient_spectrogram(result)

        self.assertEqual(figure.layout.yaxis.autorange, "reversed")

    def test_dataset_average_spectrogram_compares_whole_file_means(self) -> None:
        first = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=4,
            basis="chebyshev",
            signal_group="gyro",
        )
        second = spectrogram.fit_spectral_windows(
            _write_synthetic_csv(),
            coefficient_count=4,
            basis="chebyshev",
            signal_group="gyro",
        )

        figure = spectrogram.plot_dataset_average_spectrograms(
            {"first": first, "second": second}
        )

        self.assertIsInstance(figure, go.Figure)
        self.assertEqual(list(figure.data[0].y), ["first", "second"])
        self.assertEqual(list(figure.data[0].x), ["T0", "T1", "T2", "T3"])
        self.assertEqual(np.asarray(figure.data[0].z).shape, (2, 4))


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


def _mean_quadratic_energy(values: np.ndarray, matrix: np.ndarray) -> float:
    return float(np.mean(np.einsum("wi,ij,wj->w", values, matrix, values)))


if __name__ == "__main__":
    unittest.main()
