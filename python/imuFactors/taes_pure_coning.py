"""Pure-coning reproduction helpers for the PsyChMagPI report."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import gtsam
import numpy as np
import pandas as pd

from imuFactors.gyro_integration import (
    integrate_psychmagpi_rot3,
    rotation_error_degrees,
)


TAES_ALPHA_DEG = 10.0
TAES_OMEGA = 0.74 * np.pi
TAES_SAMPLE_RATE_HZ = 100.0
TAES_DURATION_S = 2.0
TAES_INTERVAL_OBSERVATIONS = 8
TAES_PURE_CONING_NOISE_SIGMAS = (0.0, 1e-12, 1e-6)
TAES_TABLE2_NOISE_SIGMAS = (0.0, 1e-4, 0.025, 0.075, 0.125)
TAES_NOISE_SIGMAS = TAES_PURE_CONING_NOISE_SIGMAS
TAES_N_VALUES = (6, 9)
TAES_TABLE_MONTE_CARLO_TRIALS = 5000
TAES_TABLE2_SAMPLE_RATE_HZ = 200.0
TAES_TABLE2_DURATION_S = 0.1
TAES_TABLE2_SIGNAL_FREQUENCY_HZ = 7.0
TAES_TABLE2_N = 11

METHOD_ORDER = (
    "D1 no coning",
    "D2 sequential rotations",
    "D3 single-speed",
    "PsyChMagPI",
)


@dataclass(frozen=True)
class PureConingResult:
    """Pure-coning attitude errors and final summaries."""

    errors: pd.DataFrame
    summary: pd.DataFrame
    parameters: dict[str, float | int | tuple[float, ...] | tuple[int, ...]]


@dataclass(frozen=True)
class PureConingMonteCarloResult:
    """Table-II-style pure-coning Monte Carlo metrics."""

    trial_errors: pd.DataFrame
    summary: pd.DataFrame
    parameters: dict[str, float | int | tuple[float, ...] | tuple[int, ...]]


def run_taes_pure_coning(
    *,
    N_values: Iterable[int] = TAES_N_VALUES,
    noise_sigmas: Iterable[float] = TAES_NOISE_SIGMAS,
    sample_rate_hz: float = TAES_SAMPLE_RATE_HZ,
    duration_s: float = TAES_DURATION_S,
    interval_observations: int = TAES_INTERVAL_OBSERVATIONS,
    alpha_deg: float = TAES_ALPHA_DEG,
    coning_rate: float = TAES_OMEGA,
    seed: int = 20260523,
    psych_tolerance: float = 1e-12,
    psych_max_iterations: int = 50,
    use_approximate_integration_matrix: bool = False,
) -> PureConingResult:
    """Run the TAES pure-coning reproduction on a deterministic noise draw."""
    N_values = tuple(int(value) for value in N_values)
    noise_sigmas = tuple(float(value) for value in noise_sigmas)
    interval_observations = int(interval_observations)
    sample_rate_hz = float(sample_rate_hz)
    duration_s = float(duration_s)
    sensor_dt = 1.0 / sample_rate_hz
    interval_count = int(round(duration_s * sample_rate_hz))
    if interval_count % interval_observations != 0:
        raise ValueError("duration*sample_rate must be divisible by interval_observations")
    times = np.linspace(0.0, duration_s, interval_count + 1)
    clean_omega = pure_coning_omega(times, alpha_deg=alpha_deg, coning_rate=coning_rate)

    rng = np.random.default_rng(int(seed))
    standard_noises = {
        sigma: np.zeros_like(clean_omega)
        if sigma == 0.0
        else rng.normal(size=clean_omega.shape)
        for sigma in noise_sigmas
    }

    rows: list[dict[str, float | int | str]] = []
    for sigma in noise_sigmas:
        noisy_omega = clean_omega + sigma * standard_noises[sigma]
        delta_theta = trapezoid_delta_theta(times, noisy_omega)
        for N in N_values:
            rows.extend(
                _evaluate_methods_for_grid(
                    times=times,
                    noisy_omega=noisy_omega,
                    delta_theta=delta_theta,
                    N=N,
                    sigma=sigma,
                    interval_observations=interval_observations,
                    alpha_deg=alpha_deg,
                    coning_rate=coning_rate,
                    psych_tolerance=psych_tolerance,
                    psych_max_iterations=psych_max_iterations,
                    use_approximate_integration_matrix=use_approximate_integration_matrix,
                )
            )
    errors = pd.DataFrame(rows)
    summary = (
        errors.sort_values("time_s")
        .groupby(["N", "sigma_rad_s", "method"], observed=True)
        .tail(1)
        .rename(columns={"error_rad": "final_error_rad"})
        [
            [
                "N",
                "sigma_rad_s",
                "method",
                "time_s",
                "final_error_rad",
                "error_deg",
                "converged",
                "iterations",
            ]
        ]
        .sort_values(["N", "sigma_rad_s", "method"])
        .reset_index(drop=True)
    )
    parameters = {
        "alpha_deg": float(alpha_deg),
        "coning_rate_rad_s": float(coning_rate),
        "sample_rate_hz": float(sample_rate_hz),
        "duration_s": float(duration_s),
        "interval_observations": int(interval_observations),
        "sensor_dt_s": float(sensor_dt),
        "N_values": N_values,
        "noise_sigmas_rad_s": noise_sigmas,
        "seed": int(seed),
        "use_approximate_integration_matrix": bool(use_approximate_integration_matrix),
    }
    return PureConingResult(errors=errors, summary=summary, parameters=parameters)


def run_taes_pure_coning_table_monte_carlo(
    *,
    N_values: Iterable[int] = TAES_N_VALUES,
    noise_sigmas: Iterable[float] = TAES_NOISE_SIGMAS,
    num_trials: int = TAES_TABLE_MONTE_CARLO_TRIALS,
    sample_rate_hz: float = TAES_SAMPLE_RATE_HZ,
    duration_s: float = TAES_DURATION_S,
    interval_observations: int = TAES_INTERVAL_OBSERVATIONS,
    alpha_deg: float = TAES_ALPHA_DEG,
    coning_rate: float = TAES_OMEGA,
    seed: int = 20260523,
    psych_tolerance: float = 1e-12,
    psych_max_iterations: int = 50,
    use_approximate_integration_matrix: bool = False,
) -> PureConingMonteCarloResult:
    """Run a Table-II-style RMS final-error Monte Carlo for pure coning."""
    N_values = tuple(int(value) for value in N_values)
    noise_sigmas = tuple(float(value) for value in noise_sigmas)
    num_trials = int(num_trials)
    if num_trials < 1:
        raise ValueError("num_trials must be positive")
    interval_observations = int(interval_observations)
    sample_rate_hz = float(sample_rate_hz)
    duration_s = float(duration_s)
    interval_count = int(round(duration_s * sample_rate_hz))
    if interval_count % interval_observations != 0:
        raise ValueError("duration*sample_rate must be divisible by interval_observations")

    times = np.linspace(0.0, duration_s, interval_count + 1)
    clean_omega = pure_coning_omega(times, alpha_deg=alpha_deg, coning_rate=coning_rate)
    reference = pure_coning_reference_rotation(
        duration_s,
        alpha_deg=alpha_deg,
        coning_rate=coning_rate,
    )

    rows: list[dict[str, float | int | str | bool]] = []
    for sigma_index, sigma in enumerate(noise_sigmas):
        trial_count = 1 if sigma == 0.0 else num_trials
        rng = np.random.default_rng(int(seed) + 1000003 * sigma_index)
        for trial_index in range(trial_count):
            if sigma == 0.0:
                noisy_omega = clean_omega
            else:
                noisy_omega = clean_omega + float(sigma) * rng.normal(
                    size=clean_omega.shape
                )
            delta_theta = trapezoid_delta_theta(times, noisy_omega)
            rotations, psych_metadata = integrate_all_pure_coning_methods(
                times=times,
                noisy_omega=noisy_omega,
                delta_theta=delta_theta,
                N_values=N_values,
                interval_observations=interval_observations,
                psych_tolerance=psych_tolerance,
                psych_max_iterations=psych_max_iterations,
                use_approximate_integration_matrix=use_approximate_integration_matrix,
            )
            for method, rotation in rotations.items():
                error_deg = rotation_error_degrees(reference, rotation)
                metadata = psych_metadata.get(method, {})
                rows.append(
                    {
                        "sigma_rad_s": float(sigma),
                        "trial_index": int(trial_index),
                        "method": method,
                        "N": int(metadata.get("N", 0)),
                        "error_rad": float(np.deg2rad(error_deg)),
                        "error_mrad": float(1000.0 * np.deg2rad(error_deg)),
                        "error_deg": float(error_deg),
                        "converged": bool(metadata.get("converged", True)),
                        "iterations": int(metadata.get("iterations", 0)),
                        "nominal_trials": int(num_trials),
                    }
                )

    trial_errors = pd.DataFrame(rows)
    summary = (
        trial_errors.groupby(["sigma_rad_s", "method", "N"], observed=True)
        .agg(
            trials=("error_rad", "size"),
            nominal_trials=("nominal_trials", "first"),
            rms_error_mrad=("error_rad", lambda values: 1000.0 * float(np.sqrt(np.mean(np.square(values))))),
            mean_error_mrad=("error_mrad", "mean"),
            median_error_mrad=("error_mrad", "median"),
            converged_rate=("converged", "mean"),
            median_iterations=("iterations", "median"),
        )
        .reset_index()
        .sort_values(["sigma_rad_s", "method", "N"])
    )
    parameters = {
        "alpha_deg": float(alpha_deg),
        "coning_rate_rad_s": float(coning_rate),
        "sample_rate_hz": float(sample_rate_hz),
        "duration_s": float(duration_s),
        "interval_observations": int(interval_observations),
        "N_values": N_values,
        "noise_sigmas_rad_s": noise_sigmas,
        "num_trials": int(num_trials),
        "seed": int(seed),
        "use_approximate_integration_matrix": bool(use_approximate_integration_matrix),
    }
    return PureConingMonteCarloResult(
        trial_errors=trial_errors,
        summary=summary,
        parameters=parameters,
    )


def run_taes_table2_long_integration_monte_carlo(
    *,
    noise_sigmas: Iterable[float] = TAES_TABLE2_NOISE_SIGMAS,
    lambda_values: Iterable[float] = (0.0,),
    num_trials: int = TAES_TABLE_MONTE_CARLO_TRIALS,
    sample_rate_hz: float = TAES_TABLE2_SAMPLE_RATE_HZ,
    duration_s: float = TAES_TABLE2_DURATION_S,
    signal_frequency_hz: float = TAES_TABLE2_SIGNAL_FREQUENCY_HZ,
    N: int = TAES_TABLE2_N,
    seed: int = 20260523,
    psych_tolerance: float = 1e-12,
    psych_max_iterations: int = 50,
    use_approximate_integration_matrix: bool = False,
) -> PureConingMonteCarloResult:
    """Reproduce our TAES draft Table II long-integration Monte Carlo."""
    noise_sigmas = tuple(float(value) for value in noise_sigmas)
    lambda_values = tuple(float(value) for value in lambda_values)
    num_trials = int(num_trials)
    if num_trials < 1:
        raise ValueError("num_trials must be positive")
    interval_count = int(round(float(duration_s) * float(sample_rate_hz)))
    times = np.linspace(0.0, float(duration_s), interval_count + 1)
    clean_omega = taes_table2_omega(
        times,
        signal_frequency_hz=signal_frequency_hz,
    )
    reference = gtsam.Rot3.Expmap(
        taes_table2_phi(float(duration_s), signal_frequency_hz=signal_frequency_hz)
    )

    rows: list[dict[str, float | int | str | bool]] = []
    for sigma_index, sigma in enumerate(noise_sigmas):
        trial_count = 1 if sigma == 0.0 else num_trials
        rng = np.random.default_rng(int(seed) + 1000003 * sigma_index)
        for trial_index in range(trial_count):
            if sigma == 0.0:
                noisy_omega = clean_omega
            else:
                noisy_omega = clean_omega + float(sigma) * rng.normal(
                    size=clean_omega.shape
                )
            delta_theta = trapezoid_delta_theta(times, noisy_omega)
            rotations: dict[str, gtsam.Rot3] = {
                "D1 no coning": integrate_no_coning_delta(gtsam.Rot3(), delta_theta),
                "D2 sequential rotations": integrate_sequential_delta(
                    gtsam.Rot3(),
                    delta_theta,
                ),
            }
            rotations["D3 single-speed"], _ = integrate_single_speed_delta(
                gtsam.Rot3(),
                delta_theta,
            )
            metadata = {}
            for lambda1 in lambda_values:
                psych = integrate_psychmagpi_rot3(
                    gtsam.Rot3(),
                    times,
                    noisy_omega,
                    N=int(N),
                    tolerance=float(psych_tolerance),
                    max_iterations=int(psych_max_iterations),
                    lambda1=float(lambda1),
                    use_approximate_integration_matrix=bool(
                        use_approximate_integration_matrix
                    ),
                )
                method = f"PsyChMagPI lambda={lambda1:g}"
                rotations[method] = psych.rotation
                metadata[method] = psych
            for method, rotation in rotations.items():
                error_deg = rotation_error_degrees(reference, rotation)
                psych = metadata.get(method)
                rows.append(
                    {
                        "sigma_rad_s": float(sigma),
                        "trial_index": int(trial_index),
                        "method": method,
                        "N": int(N) if psych is not None else 0,
                        "lambda1": float(psych.lambda1) if psych is not None else -1.0,
                        "error_rad": float(np.deg2rad(error_deg)),
                        "error_mrad": float(1000.0 * np.deg2rad(error_deg)),
                        "error_deg": float(error_deg),
                        "converged": bool(psych.converged) if psych is not None else True,
                        "iterations": int(psych.iterations) if psych is not None else 0,
                        "nominal_trials": int(num_trials),
                    }
                )
    trial_errors = pd.DataFrame(rows)
    summary = (
        trial_errors.groupby(["sigma_rad_s", "method", "N", "lambda1"], observed=True)
        .agg(
            trials=("error_rad", "size"),
            nominal_trials=("nominal_trials", "first"),
            rms_error_mrad=("error_rad", lambda values: 1000.0 * float(np.sqrt(np.mean(np.square(values))))),
            mean_error_mrad=("error_mrad", "mean"),
            median_error_mrad=("error_mrad", "median"),
            converged_rate=("converged", "mean"),
            median_iterations=("iterations", "median"),
        )
        .reset_index()
        .sort_values(["sigma_rad_s", "method", "N"])
    )
    return PureConingMonteCarloResult(
        trial_errors=trial_errors,
        summary=summary,
        parameters={
            "sample_rate_hz": float(sample_rate_hz),
            "duration_s": float(duration_s),
            "signal_frequency_hz": float(signal_frequency_hz),
            "N": int(N),
            "lambda_values": lambda_values,
            "noise_sigmas_rad_s": noise_sigmas,
            "num_trials": int(num_trials),
            "seed": int(seed),
            "use_approximate_integration_matrix": bool(
                use_approximate_integration_matrix
            ),
        },
    )


def pure_coning_omega(
    times: np.ndarray,
    *,
    alpha_deg: float = TAES_ALPHA_DEG,
    coning_rate: float = TAES_OMEGA,
) -> np.ndarray:
    """Return the TAES Section VI.A pure-coning angular velocity."""
    t = np.asarray(times, dtype=float).reshape(-1)
    alpha = np.deg2rad(float(alpha_deg))
    return float(coning_rate) * np.column_stack(
        [
            np.full_like(t, -2.0 * np.sin(alpha / 2.0) ** 2),
            -np.sin(alpha) * np.sin(float(coning_rate) * t),
            np.sin(alpha) * np.cos(float(coning_rate) * t),
        ]
    )


def taes_table2_phi(
    times: float | np.ndarray,
    *,
    signal_frequency_hz: float = TAES_TABLE2_SIGNAL_FREQUENCY_HZ,
) -> np.ndarray:
    """Return the Eq. 72 rotation-vector signal from our TAES draft."""
    t = np.asarray(times, dtype=float)
    phase = 2.0 * np.pi * float(signal_frequency_hz) * t
    values = np.stack(
        [
            0.100 * (np.cos(phase) - 1.0),
            0.050 * np.sin(phase),
            0.075 * (np.cos(phase) - 1.0),
        ],
        axis=-1,
    )
    return values


def taes_table2_omega(
    times: np.ndarray,
    *,
    signal_frequency_hz: float = TAES_TABLE2_SIGNAL_FREQUENCY_HZ,
) -> np.ndarray:
    """Return body angular velocity for the Eq. 72 rotation-vector signal."""
    t = np.asarray(times, dtype=float).reshape(-1)
    phase = 2.0 * np.pi * float(signal_frequency_hz) * t
    phase_dot = 2.0 * np.pi * float(signal_frequency_hz)
    phi_values = taes_table2_phi(t, signal_frequency_hz=signal_frequency_hz)
    phi_dot_values = np.stack(
        [
            -0.100 * phase_dot * np.sin(phase),
            0.050 * phase_dot * np.cos(phase),
            -0.075 * phase_dot * np.sin(phase),
        ],
        axis=-1,
    )
    return np.vstack(
        [
            _so3_right_jacobian(phi_value) @ phi_dot
            for phi_value, phi_dot in zip(phi_values, phi_dot_values, strict=True)
        ]
    )


def pure_coning_reference_rotation(
    time_s: float,
    *,
    alpha_deg: float = TAES_ALPHA_DEG,
    coning_rate: float = TAES_OMEGA,
) -> gtsam.Rot3:
    """Return the analytic relative rotation from t=0 to ``time_s``."""
    initial = _pure_coning_absolute_rotation(
        0.0, alpha_deg=alpha_deg, coning_rate=coning_rate
    )
    current = _pure_coning_absolute_rotation(
        float(time_s), alpha_deg=alpha_deg, coning_rate=coning_rate
    )
    return initial.inverse().compose(current)


def trapezoid_delta_theta(times: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """Approximate rate-gyro increments using our TAES draft's trapezoidal rule."""
    t = np.asarray(times, dtype=float).reshape(-1)
    w = np.asarray(omega, dtype=float)
    if w.shape != (t.size, 3):
        raise ValueError("omega must have shape (len(times), 3)")
    dt = np.diff(t).reshape(-1, 1)
    return 0.5 * dt * (w[:-1] + w[1:])


def integrate_no_coning_delta(
    initial_rotation: gtsam.Rot3,
    delta_theta: np.ndarray,
) -> gtsam.Rot3:
    """D1: ignore coning inside the update and exponentiate summed theta."""
    return initial_rotation.compose(gtsam.Rot3.Expmap(np.sum(delta_theta, axis=0)))


def integrate_sequential_delta(
    initial_rotation: gtsam.Rot3,
    delta_theta: np.ndarray,
) -> gtsam.Rot3:
    """D2: compose the per-sample delta-theta rotations sequentially."""
    rotation = initial_rotation
    for delta in np.asarray(delta_theta, dtype=float):
        rotation = rotation.compose(gtsam.Rot3.Expmap(delta))
    return rotation


def integrate_single_speed_delta(
    initial_rotation: gtsam.Rot3,
    delta_theta: np.ndarray,
    previous_delta: np.ndarray | None = None,
) -> tuple[gtsam.Rot3, np.ndarray]:
    """D3: single-speed constant-angular-acceleration coning correction."""
    rotation = initial_rotation
    previous = (
        np.zeros(3, dtype=float)
        if previous_delta is None
        else np.asarray(previous_delta, dtype=float).reshape(3)
    )
    for delta in np.asarray(delta_theta, dtype=float):
        corrected = delta + (1.0 / 12.0) * np.cross(previous, delta)
        rotation = rotation.compose(gtsam.Rot3.Expmap(corrected))
        previous = delta
    return rotation, previous


def integrate_all_pure_coning_methods(
    *,
    times: np.ndarray,
    noisy_omega: np.ndarray,
    delta_theta: np.ndarray,
    N_values: Iterable[int],
    interval_observations: int,
    psych_tolerance: float,
    psych_max_iterations: int,
    use_approximate_integration_matrix: bool = False,
) -> tuple[dict[str, gtsam.Rot3], dict[str, dict[str, float | int | bool]]]:
    """Integrate all pure-coning methods through the full time interval."""
    rotations: dict[str, gtsam.Rot3] = {
        "D1 no coning": gtsam.Rot3(),
        "D2 sequential rotations": gtsam.Rot3(),
        "D3 single-speed": gtsam.Rot3(),
    }
    for N in N_values:
        rotations[f"PsyChMagPI N={int(N)}"] = gtsam.Rot3()

    previous_delta = np.zeros(3, dtype=float)
    psych_metadata: dict[str, dict[str, float | int | bool]] = {}
    for start in range(0, delta_theta.shape[0], int(interval_observations)):
        stop = start + int(interval_observations)
        block_dtheta = delta_theta[start:stop]
        block_times = times[start : stop + 1]
        block_omega = noisy_omega[start : stop + 1]

        rotations["D1 no coning"] = integrate_no_coning_delta(
            rotations["D1 no coning"], block_dtheta
        )
        rotations["D2 sequential rotations"] = integrate_sequential_delta(
            rotations["D2 sequential rotations"], block_dtheta
        )
        rotations["D3 single-speed"], previous_delta = integrate_single_speed_delta(
            rotations["D3 single-speed"], block_dtheta, previous_delta
        )

        for N in N_values:
            method = f"PsyChMagPI N={int(N)}"
            psych = integrate_psychmagpi_rot3(
                rotations[method],
                block_times - block_times[0],
                block_omega,
                N=int(N),
                tolerance=float(psych_tolerance),
                max_iterations=int(psych_max_iterations),
                lambda1=0.0,
                use_approximate_integration_matrix=bool(
                    use_approximate_integration_matrix
                ),
            )
            rotations[method] = psych.rotation
            psych_metadata[method] = {
                "N": int(N),
                "converged": bool(psych.converged),
                "iterations": int(psych.iterations),
                "final_residual": float(psych.final_residual),
            }
    return rotations, psych_metadata


def _evaluate_methods_for_grid(
    *,
    times: np.ndarray,
    noisy_omega: np.ndarray,
    delta_theta: np.ndarray,
    N: int,
    sigma: float,
    interval_observations: int,
    alpha_deg: float,
    coning_rate: float,
    psych_tolerance: float,
    psych_max_iterations: int,
    use_approximate_integration_matrix: bool = False,
) -> list[dict[str, float | int | str]]:
    rotations = {
        "D1 no coning": gtsam.Rot3(),
        "D2 sequential rotations": gtsam.Rot3(),
        "D3 single-speed": gtsam.Rot3(),
        "PsyChMagPI": gtsam.Rot3(),
    }
    previous_delta = np.zeros(3, dtype=float)
    rows: list[dict[str, float | int | str]] = []
    for start in range(0, delta_theta.shape[0], interval_observations):
        stop = start + interval_observations
        block_dtheta = delta_theta[start:stop]
        block_times = times[start : stop + 1]
        block_omega = noisy_omega[start : stop + 1]

        rotations["D1 no coning"] = integrate_no_coning_delta(
            rotations["D1 no coning"], block_dtheta
        )
        rotations["D2 sequential rotations"] = integrate_sequential_delta(
            rotations["D2 sequential rotations"], block_dtheta
        )
        rotations["D3 single-speed"], previous_delta = integrate_single_speed_delta(
            rotations["D3 single-speed"], block_dtheta, previous_delta
        )
        psych = integrate_psychmagpi_rot3(
            rotations["PsyChMagPI"],
            block_times - block_times[0],
            block_omega,
            N=int(N),
            tolerance=float(psych_tolerance),
            max_iterations=int(psych_max_iterations),
            lambda1=0.0,
            use_approximate_integration_matrix=bool(
                use_approximate_integration_matrix
            ),
        )
        rotations["PsyChMagPI"] = psych.rotation

        reference = pure_coning_reference_rotation(
            float(block_times[-1]), alpha_deg=alpha_deg, coning_rate=coning_rate
        )
        for method in METHOD_ORDER:
            error_deg = rotation_error_degrees(reference, rotations[method])
            rows.append(
                {
                    "N": int(N),
                    "sigma_rad_s": float(sigma),
                    "time_s": float(block_times[-1]),
                    "method": method,
                    "error_deg": float(error_deg),
                    "error_rad": float(np.deg2rad(error_deg)),
                    "converged": bool(psych.converged) if method == "PsyChMagPI" else True,
                    "iterations": int(psych.iterations) if method == "PsyChMagPI" else 0,
                }
            )
    return rows


def _pure_coning_absolute_rotation(
    time_s: float,
    *,
    alpha_deg: float,
    coning_rate: float,
) -> gtsam.Rot3:
    alpha = np.deg2rad(float(alpha_deg))
    gibbs = np.array(
        [
            0.0,
            np.tan(alpha / 2.0) * np.cos(float(coning_rate) * float(time_s)),
            np.tan(alpha / 2.0) * np.sin(float(coning_rate) * float(time_s)),
        ],
        dtype=float,
    )
    norm = float(np.linalg.norm(gibbs))
    if norm == 0.0:
        return gtsam.Rot3()
    rotation_vector = 2.0 * np.arctan(norm) * gibbs / norm
    return gtsam.Rot3.Expmap(rotation_vector)


def _so3_right_jacobian(phi: np.ndarray) -> np.ndarray:
    vector = np.asarray(phi, dtype=float).reshape(3)
    theta = float(np.linalg.norm(vector))
    skew = _skew(vector)
    if theta < 1e-8:
        return np.eye(3) - 0.5 * skew + (1.0 / 6.0) * (skew @ skew)
    return (
        np.eye(3)
        - ((1.0 - np.cos(theta)) / (theta * theta)) * skew
        + ((theta - np.sin(theta)) / (theta**3)) * (skew @ skew)
    )


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )
