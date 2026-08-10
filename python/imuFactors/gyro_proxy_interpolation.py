"""Interpolation and trajectory-fidelity experiments for gyro proxy snippets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import gtsam
import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp
from scipy.interpolate import CubicSpline

from imuFactors import spectral
from imuFactors.gyro_proxy_monte_carlo import (
    GyroProxySnippet,
    evaluate_proxy_gyro,
    noise_band_labels,
    standard_noise,
)


SIGNAL_METHOD_ORDER = (
    "Linear",
    "Cubic spline",
    "PsyChMagPI lambda0",
    "PsyChMagPI scaled lambda",
)
TRAJECTORY_METHOD_ORDER = (
    "RK2",
    "Magnus-2",
    "Linear",
    "Cubic spline",
    "PsyChMagPI lambda0",
    "PsyChMagPI scaled lambda",
)
DURATION_BINS = (
    ("short", 0.0, 0.05),
    ("mid", 0.05, 0.10),
    ("long", 0.10, np.inf),
)


@dataclass(frozen=True)
class GyroInterpolationResult:
    """Metrics and summaries for proxy interpolation experiments."""

    signal_metrics: pd.DataFrame
    trajectory_metrics: pd.DataFrame
    subinterval_metrics: pd.DataFrame
    signal_summary: pd.DataFrame
    trajectory_summary: pd.DataFrame
    subinterval_summary: pd.DataFrame


def scaled_tikhonov_lambda(
    *,
    m: int,
    noise_std: float,
    roughness: float,
    dimension: int = 3,
    c_lambda: float = 1.0,
) -> float:
    """Return noise-energy-scaled first-derivative Tikhonov strength."""
    if roughness <= 0.0 or noise_std <= 0.0:
        return 0.0
    return (
        float(c_lambda)
        * float(dimension)
        * float(m)
        * float(noise_std) ** 2
        / float(roughness)
    )


def run_gyro_proxy_interpolation_experiments(
    snippets: Sequence[GyroProxySnippet],
    *,
    m_values: Iterable[int],
    noise_fractions: Iterable[float],
    num_seeds: int,
    seed: int,
    dense_count: int = 1001,
    subintervals_per_duration_bin: int = 24,
    c_lambda: float = 1.0,
) -> GyroInterpolationResult:
    """Run interpolation, trajectory, and subinterval experiments on snippets."""
    snippets = list(snippets)
    m_values = [int(value) for value in m_values]
    noise_fractions = [float(value) for value in noise_fractions]
    label_map = noise_band_labels(noise_fractions)

    signal_rows: list[dict[str, float | int | str]] = []
    trajectory_rows: list[dict[str, float | int | str]] = []
    subinterval_rows: list[dict[str, float | int | str]] = []

    for snippet_index, snippet in enumerate(snippets):
        dense_times = np.linspace(
            snippet.middle_interval[0],
            snippet.middle_interval[1],
            int(dense_count),
        )
        clean_dense = evaluate_proxy_gyro(snippet, dense_times)
        reference_trajectory = reference_rotation_trajectory(
            lambda times: evaluate_proxy_gyro(snippet, times),
            dense_times,
        )
        roughness = angular_velocity_roughness(dense_times, clean_dense)
        normalization = max(float(snippet.noise_scale), 1e-12)

        for m in m_values:
            sample_times = np.linspace(
                snippet.middle_interval[0],
                snippet.middle_interval[1],
                int(m),
            )
            clean_samples = evaluate_proxy_gyro(snippet, sample_times)
            linear_matrix = linear_interpolation_matrix(sample_times, dense_times)
            cubic_matrix = cubic_interpolation_matrix(sample_times, dense_times)
            sample_subintervals = deterministic_subintervals(
                dense_times,
                seed=seed,
                snippet_index=snippet_index,
                m=m,
                count_per_bin=subintervals_per_duration_bin,
            )
            noises = standard_noise(seed, snippet_index, m, num_seeds)

            for noise_fraction in noise_fractions:
                noise_std = float(noise_fraction) * float(snippet.noise_scale)
                sample_batch = clean_samples[None, :, :] + noise_std * noises
                n_sqrt = int(np.ceil(np.sqrt(float(m))))
                scaled_lambda = scaled_tikhonov_lambda(
                    m=m,
                    noise_std=noise_std,
                    roughness=roughness,
                    c_lambda=c_lambda,
                )
                predictions = {
                    "Linear": np.einsum("dm,smc->sdc", linear_matrix, sample_batch),
                    "Cubic spline": np.einsum("dm,smc->sdc", cubic_matrix, sample_batch),
                }
                sample_predictions = {
                    "Linear": sample_batch,
                    "Cubic spline": sample_batch,
                }
                for method, lambda1 in (
                    ("PsyChMagPI lambda0", 0.0),
                    ("PsyChMagPI scaled lambda", scaled_lambda),
                ):
                    dense_prediction, sample_prediction = psychmagpi_fit_predictions(
                        sample_times,
                        sample_batch,
                        dense_times,
                        n_sqrt,
                        lambda1,
                    )
                    predictions[method] = dense_prediction
                    sample_predictions[method] = sample_prediction

                signal_rows.extend(
                    signal_metric_rows(
                        snippet=snippet,
                        m=m,
                        n_sqrt=n_sqrt,
                        noise_fraction=noise_fraction,
                        noise_band=label_map[float(noise_fraction)],
                        noise_std=noise_std,
                        roughness=roughness,
                        scaled_lambda=scaled_lambda,
                        clean_dense=clean_dense,
                        clean_samples=clean_samples,
                        sample_batch=sample_batch,
                        predictions=predictions,
                        sample_predictions=sample_predictions,
                        normalization=normalization,
                    )
                )

                trajectories = {
                    "RK2": sample_piecewise_trajectory_batch(
                        sample_times,
                        sample_batch,
                        dense_times,
                        method="RK2",
                    ),
                    "Magnus-2": sample_piecewise_trajectory_batch(
                        sample_times,
                        sample_batch,
                        dense_times,
                        method="Magnus-2",
                    ),
                }
                for method, dense_prediction in predictions.items():
                    trajectories[method] = integrate_dense_trajectory_batch(
                        dense_times,
                        dense_prediction,
                    )

                trajectory_rows.extend(
                    trajectory_metric_rows(
                        snippet=snippet,
                        m=m,
                        n_sqrt=n_sqrt,
                        noise_fraction=noise_fraction,
                        noise_band=label_map[float(noise_fraction)],
                        noise_std=noise_std,
                        scaled_lambda=scaled_lambda,
                        roughness=roughness,
                        reference_trajectory=reference_trajectory,
                        trajectories=trajectories,
                    )
                )
                subinterval_rows.extend(
                    subinterval_metric_rows(
                        snippet=snippet,
                        m=m,
                        n_sqrt=n_sqrt,
                        noise_fraction=noise_fraction,
                        noise_band=label_map[float(noise_fraction)],
                        noise_std=noise_std,
                        scaled_lambda=scaled_lambda,
                        roughness=roughness,
                        reference_trajectory=reference_trajectory,
                        trajectories=trajectories,
                        subintervals=sample_subintervals,
                    )
                )

    signal_metrics = pd.DataFrame(signal_rows)
    trajectory_metrics = pd.DataFrame(trajectory_rows)
    subinterval_metrics = pd.DataFrame(subinterval_rows)
    signal_summary = summarize_signal_metrics(signal_metrics)
    trajectory_summary = summarize_trajectory_metrics(trajectory_metrics)
    subinterval_summary = summarize_subinterval_metrics(subinterval_metrics)
    return GyroInterpolationResult(
        signal_metrics=signal_metrics,
        trajectory_metrics=trajectory_metrics,
        subinterval_metrics=subinterval_metrics,
        signal_summary=signal_summary,
        trajectory_summary=trajectory_summary,
        subinterval_summary=subinterval_summary,
    )


def angular_velocity_roughness(times: np.ndarray, values: np.ndarray) -> float:
    """Approximate ``integral ||omega_dot(t)||^2 dt`` on a dense grid."""
    derivatives = np.gradient(values, times, axis=0, edge_order=2)
    return float(np.trapz(np.sum(derivatives * derivatives, axis=1), times))


def reference_rotation_trajectory(
    omega_function,
    dense_times: np.ndarray,
    *,
    rtol: float = 1e-11,
    atol: float = 1e-13,
) -> np.ndarray:
    """Return clean reference rotation matrices at ``dense_times``."""
    dense_times = np.asarray(dense_times, dtype=float)

    def rhs(t: float, state: np.ndarray) -> np.ndarray:
        rotation = state.reshape(3, 3)
        omega = np.asarray(omega_function(np.asarray([t])), dtype=float).reshape(3)
        return (rotation @ skew_matrix(omega)).reshape(-1)

    solution = solve_ivp(
        rhs,
        (float(dense_times[0]), float(dense_times[-1])),
        np.eye(3).reshape(-1),
        t_eval=dense_times,
        method="DOP853",
        rtol=rtol,
        atol=atol,
    )
    if not solution.success:
        raise RuntimeError(f"Reference trajectory integration failed: {solution.message}")
    matrices = solution.y.T.reshape(-1, 3, 3)
    return orthonormalize_rotations(matrices)


def linear_interpolation_matrix(sample_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    """Return a matrix mapping sample values to piecewise-linear values."""
    sample_times = np.asarray(sample_times, dtype=float)
    query_times = np.asarray(query_times, dtype=float)
    matrix = np.zeros((query_times.size, sample_times.size), dtype=float)
    indices = np.searchsorted(sample_times, query_times, side="right") - 1
    indices = np.clip(indices, 0, sample_times.size - 2)
    right = indices + 1
    dt = sample_times[right] - sample_times[indices]
    fraction = np.divide(
        query_times - sample_times[indices],
        dt,
        out=np.zeros_like(query_times),
        where=dt > 0.0,
    )
    matrix[np.arange(query_times.size), indices] = 1.0 - fraction
    matrix[np.arange(query_times.size), right] += fraction
    matrix[query_times <= sample_times[0], :] = 0.0
    matrix[query_times <= sample_times[0], 0] = 1.0
    matrix[query_times >= sample_times[-1], :] = 0.0
    matrix[query_times >= sample_times[-1], -1] = 1.0
    return matrix


def cubic_interpolation_matrix(sample_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    """Return a natural-cubic-spline matrix from samples to query values."""
    identity = np.eye(np.asarray(sample_times).size, dtype=float)
    spline = CubicSpline(sample_times, identity, axis=0, bc_type="natural")
    return np.asarray(spline(query_times), dtype=float)


def psychmagpi_fit_predictions(
    sample_times: np.ndarray,
    sample_batch: np.ndarray,
    dense_times: np.ndarray,
    N: int,
    lambda1: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return dense and sample-grid PsyChMagPI input-fit predictions."""
    interval = (float(sample_times[0]), float(sample_times[-1]))
    plan = spectral.basis_plan_from_coordinates(
        int(N),
        sample_times,
        basis="chebyshev2",
        interval=interval,
        window_seconds=interval[1] - interval[0],
        lambda1=float(lambda1),
    )
    dense_weights = spectral.chebyshev2_weight_matrix(int(N), dense_times, interval)
    cgl_batch = np.einsum("nm,smc->snc", plan.solver, sample_batch)
    dense_prediction = np.einsum("dn,snc->sdc", dense_weights, cgl_batch)
    sample_prediction = np.einsum("mn,snc->smc", plan.weight_matrix, cgl_batch)
    return dense_prediction, sample_prediction


def integrate_dense_trajectory_batch(times: np.ndarray, omega_batch: np.ndarray) -> np.ndarray:
    """Integrate dense gyro curves with Lie midpoint updates."""
    times = np.asarray(times, dtype=float)
    omega = np.asarray(omega_batch, dtype=float)
    seed_count = omega.shape[0]
    rotations = np.empty((seed_count, times.size, 3, 3), dtype=float)
    current = np.broadcast_to(np.eye(3), (seed_count, 3, 3)).copy()
    rotations[:, 0] = current
    for index in range(times.size - 1):
        dt = float(times[index + 1] - times[index])
        delta = 0.5 * dt * (omega[:, index] + omega[:, index + 1])
        current = np.matmul(current, so3_exp_batch(delta))
        rotations[:, index + 1] = current
    return rotations


def sample_piecewise_trajectory_batch(
    sample_times: np.ndarray,
    sample_batch: np.ndarray,
    dense_times: np.ndarray,
    *,
    method: str,
) -> np.ndarray:
    """Return piecewise-geodesic RK2 or Magnus-2 sample trajectories."""
    sample_times = np.asarray(sample_times, dtype=float)
    dense_times = np.asarray(dense_times, dtype=float)
    samples = np.asarray(sample_batch, dtype=float)
    seed_count = samples.shape[0]
    rotations = np.empty((seed_count, dense_times.size, 3, 3), dtype=float)
    current = np.broadcast_to(np.eye(3), (seed_count, 3, 3)).copy()
    rotations[:, 0] = current

    for index in range(sample_times.size - 1):
        dt = float(sample_times[index + 1] - sample_times[index])
        if method == "RK2":
            delta = 0.5 * dt * (samples[:, index] + samples[:, index + 1])
        elif method == "Magnus-2":
            delta = (
                0.5 * dt * (samples[:, index] + samples[:, index + 1])
                + (dt * dt / 12.0) * np.cross(samples[:, index + 1], samples[:, index])
            )
        else:
            raise ValueError(f"Unknown piecewise trajectory method {method}")

        start = int(np.searchsorted(dense_times, sample_times[index], side="right"))
        stop = int(np.searchsorted(dense_times, sample_times[index + 1], side="right"))
        if stop > start:
            fractions = (
                (dense_times[start:stop] - sample_times[index]) / dt
            ).reshape(-1, 1)
            increments = so3_exp_batch(
                fractions[:, None, :] * delta[None, :, :]
            ).transpose(1, 0, 2, 3)
            rotations[:, start:stop] = np.matmul(current[:, None, :, :], increments)
        current = np.matmul(current, so3_exp_batch(delta))
    rotations[:, -1] = current
    return rotations


def deterministic_subintervals(
    dense_times: np.ndarray,
    *,
    seed: int,
    snippet_index: int,
    m: int,
    count_per_bin: int,
) -> pd.DataFrame:
    """Draw deterministic dense-grid subinterval index pairs by duration bin."""
    dense_times = np.asarray(dense_times, dtype=float)
    start_indices, end_indices = np.triu_indices(dense_times.size, k=1)
    durations = dense_times[end_indices] - dense_times[start_indices]
    rng = np.random.default_rng(int(seed) + 9176 * int(snippet_index) + 37 * int(m))
    rows = []
    for label, lower, upper in DURATION_BINS:
        if np.isinf(upper):
            mask = durations > float(lower)
        else:
            mask = (durations > float(lower)) & (durations <= float(upper))
        candidates = np.flatnonzero(mask)
        if candidates.size == 0:
            continue
        chosen = rng.choice(
            candidates,
            size=int(count_per_bin),
            replace=candidates.size < int(count_per_bin),
        )
        for candidate in chosen:
            rows.append(
                {
                    "duration_bin": label,
                    "start_index": int(start_indices[candidate]),
                    "end_index": int(end_indices[candidate]),
                    "duration_s": float(durations[candidate]),
                }
            )
    return pd.DataFrame(rows)


def signal_metric_rows(
    *,
    snippet: GyroProxySnippet,
    m: int,
    n_sqrt: int,
    noise_fraction: float,
    noise_band: str,
    noise_std: float,
    roughness: float,
    scaled_lambda: float,
    clean_dense: np.ndarray,
    clean_samples: np.ndarray,
    sample_batch: np.ndarray,
    predictions: dict[str, np.ndarray],
    sample_predictions: dict[str, np.ndarray],
    normalization: float,
) -> list[dict[str, float | int | str]]:
    raw_errors = np.sqrt(np.mean(np.sum((sample_batch - clean_samples[None]) ** 2, axis=2), axis=1))
    rows = []
    for method in SIGNAL_METHOD_ORDER:
        dense_error = predictions[method] - clean_dense[None]
        dense_norm = np.linalg.norm(dense_error, axis=2)
        sample_error = sample_predictions[method] - clean_samples[None]
        fit_errors = np.sqrt(np.mean(np.sum(sample_error * sample_error, axis=2), axis=1))
        gyro_rmse = np.sqrt(np.mean(dense_norm * dense_norm, axis=1))
        max_error = np.max(dense_norm, axis=1)
        denoising_gain = denoising_gain_db(raw_errors, fit_errors)
        lambda1 = method_lambda(method, scaled_lambda)
        for seed_index in range(sample_batch.shape[0]):
            rows.append(
                {
                    "dataset": snippet.dataset,
                    "m": int(m),
                    "N_sqrt": int(n_sqrt),
                    "noise_fraction": float(noise_fraction),
                    "noise_band": noise_band,
                    "noise_std": float(noise_std),
                    "seed_index": int(seed_index),
                    "method": method,
                    "N": int(n_sqrt) if method.startswith("PsyChMagPI") else np.nan,
                    "lambda1": lambda1,
                    "roughness": float(roughness),
                    "gyro_rmse_rad_s": float(gyro_rmse[seed_index]),
                    "gyro_nrmse": float(gyro_rmse[seed_index] / normalization),
                    "max_gyro_error_rad_s": float(max_error[seed_index]),
                    "raw_sample_rmse_rad_s": float(raw_errors[seed_index]),
                    "fit_sample_rmse_rad_s": float(fit_errors[seed_index]),
                    "denoising_gain_db": float(denoising_gain[seed_index]),
                }
            )
    return rows


def trajectory_metric_rows(
    *,
    snippet: GyroProxySnippet,
    m: int,
    n_sqrt: int,
    noise_fraction: float,
    noise_band: str,
    noise_std: float,
    scaled_lambda: float,
    roughness: float,
    reference_trajectory: np.ndarray,
    trajectories: dict[str, np.ndarray],
) -> list[dict[str, float | int | str]]:
    rows = []
    for method in TRAJECTORY_METHOD_ORDER:
        errors = rotation_error_degrees_batch(reference_trajectory[None], trajectories[method])
        rms_error = np.sqrt(np.mean(errors * errors, axis=1))
        median_error = np.median(errors, axis=1)
        max_error = np.max(errors, axis=1)
        final_error = errors[:, -1]
        lambda1 = method_lambda(method, scaled_lambda)
        for seed_index in range(errors.shape[0]):
            rows.append(
                {
                    "dataset": snippet.dataset,
                    "m": int(m),
                    "N_sqrt": int(n_sqrt),
                    "noise_fraction": float(noise_fraction),
                    "noise_band": noise_band,
                    "noise_std": float(noise_std),
                    "seed_index": int(seed_index),
                    "method": method,
                    "N": int(n_sqrt) if method.startswith("PsyChMagPI") else np.nan,
                    "lambda1": lambda1,
                    "roughness": float(roughness),
                    "trajectory_rms_error_deg": float(rms_error[seed_index]),
                    "trajectory_median_error_deg": float(median_error[seed_index]),
                    "trajectory_max_error_deg": float(max_error[seed_index]),
                    "final_error_deg": float(final_error[seed_index]),
                }
            )
    return rows


def subinterval_metric_rows(
    *,
    snippet: GyroProxySnippet,
    m: int,
    n_sqrt: int,
    noise_fraction: float,
    noise_band: str,
    noise_std: float,
    scaled_lambda: float,
    roughness: float,
    reference_trajectory: np.ndarray,
    trajectories: dict[str, np.ndarray],
    subintervals: pd.DataFrame,
) -> list[dict[str, float | int | str]]:
    rows = []
    for duration_bin, group in subintervals.groupby("duration_bin", observed=True):
        starts = group["start_index"].to_numpy(dtype=int)
        ends = group["end_index"].to_numpy(dtype=int)
        method_errors = {}
        for method in TRAJECTORY_METHOD_ORDER:
            method_errors[method] = relative_rotation_errors(
                reference_trajectory,
                trajectories[method],
                starts,
                ends,
            )
        stacked = np.stack([method_errors[method] for method in TRAJECTORY_METHOD_ORDER])
        winners = stacked <= np.min(stacked, axis=0, keepdims=True) + 1e-12
        for method_index, method in enumerate(TRAJECTORY_METHOD_ORDER):
            errors = method_errors[method]
            rows.append(
                {
                    "dataset": snippet.dataset,
                    "m": int(m),
                    "N_sqrt": int(n_sqrt),
                    "noise_fraction": float(noise_fraction),
                    "noise_band": noise_band,
                    "noise_std": float(noise_std),
                    "duration_bin": str(duration_bin),
                    "method": method,
                    "N": int(n_sqrt) if method.startswith("PsyChMagPI") else np.nan,
                    "lambda1": method_lambda(method, scaled_lambda),
                    "roughness": float(roughness),
                    "subinterval_count": int(errors.size),
                    "mean_duration_s": float(group["duration_s"].mean()),
                    "rms_error_deg": float(np.sqrt(np.mean(errors * errors))),
                    "median_error_deg": float(np.median(errors)),
                    "max_error_deg": float(np.max(errors)),
                    "win_rate": float(np.mean(winners[method_index])),
                }
            )
    return rows


def summarize_signal_metrics(signal_metrics: pd.DataFrame) -> pd.DataFrame:
    """Summarize signal interpolation metrics and per-trial winners."""
    metrics = signal_metrics.copy()
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    metrics["winner"] = metrics["gyro_nrmse"] <= metrics.groupby(key, observed=True)[
        "gyro_nrmse"
    ].transform("min") + 1e-15
    return (
        metrics.groupby(["method", "m", "noise_band"], observed=True)
        .agg(
            trials=("gyro_nrmse", "size"),
            win_rate=("winner", "mean"),
            median_gyro_nrmse=("gyro_nrmse", "median"),
            mean_gyro_nrmse=("gyro_nrmse", "mean"),
            median_denoising_gain_db=("denoising_gain_db", "median"),
            median_lambda1=("lambda1", "median"),
        )
        .reset_index()
        .sort_values(["noise_band", "m", "method"])
    )


def summarize_trajectory_metrics(trajectory_metrics: pd.DataFrame) -> pd.DataFrame:
    """Summarize trajectory RMS attitude metrics and per-trial winners."""
    metrics = trajectory_metrics.copy()
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    metrics["winner"] = metrics["trajectory_rms_error_deg"] <= metrics.groupby(
        key, observed=True
    )["trajectory_rms_error_deg"].transform("min") + 1e-15
    return (
        metrics.groupby(["method", "m", "noise_band"], observed=True)
        .agg(
            trials=("trajectory_rms_error_deg", "size"),
            win_rate=("winner", "mean"),
            median_trajectory_rms_error_deg=("trajectory_rms_error_deg", "median"),
            mean_trajectory_rms_error_deg=("trajectory_rms_error_deg", "mean"),
            median_final_error_deg=("final_error_deg", "median"),
            median_lambda1=("lambda1", "median"),
        )
        .reset_index()
        .sort_values(["noise_band", "m", "method"])
    )


def summarize_subinterval_metrics(subinterval_metrics: pd.DataFrame) -> pd.DataFrame:
    """Summarize subinterval metrics over snippets and noise fractions."""
    return (
        subinterval_metrics.groupby(
            ["method", "m", "noise_band", "duration_bin"], observed=True
        )
        .agg(
            groups=("rms_error_deg", "size"),
            subinterval_count=("subinterval_count", "sum"),
            win_rate=("win_rate", "mean"),
            median_rms_error_deg=("rms_error_deg", "median"),
            median_error_deg=("median_error_deg", "median"),
            mean_duration_s=("mean_duration_s", "mean"),
            median_lambda1=("lambda1", "median"),
        )
        .reset_index()
        .sort_values(["noise_band", "duration_bin", "m", "method"])
    )


def method_lambda(method: str, scaled_lambda: float) -> float:
    """Return the lambda associated with a display method."""
    if method == "PsyChMagPI lambda0":
        return 0.0
    if method == "PsyChMagPI scaled lambda":
        return float(scaled_lambda)
    return np.nan


def denoising_gain_db(raw_errors: np.ndarray, fit_errors: np.ndarray) -> np.ndarray:
    """Return finite denoising gains in dB for per-seed errors."""
    raw = np.asarray(raw_errors, dtype=float)
    fit = np.asarray(fit_errors, dtype=float)
    gain = np.zeros_like(raw)
    useful = raw > 1e-15
    gain[useful] = 20.0 * np.log10(raw[useful] / np.maximum(fit[useful], 1e-15))
    return gain


def relative_rotation_errors(
    reference_trajectory: np.ndarray,
    estimated_trajectory: np.ndarray,
    starts: np.ndarray,
    ends: np.ndarray,
) -> np.ndarray:
    """Return subinterval relative-rotation errors with shape ``(seeds, pairs)``."""
    reference_rel = np.matmul(
        np.swapaxes(reference_trajectory[starts], -1, -2),
        reference_trajectory[ends],
    )
    estimated_rel = np.matmul(
        np.swapaxes(estimated_trajectory[:, starts], -1, -2),
        estimated_trajectory[:, ends],
    )
    error_matrices = np.matmul(
        np.swapaxes(reference_rel, -1, -2)[None, :, :, :],
        estimated_rel,
    )
    return rotation_angles_degrees(error_matrices)


def rotation_error_degrees_batch(reference: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    """Return SO(3) angle errors for broadcastable rotation trajectories."""
    error_matrices = np.matmul(np.swapaxes(reference, -1, -2), estimate)
    return rotation_angles_degrees(error_matrices)


def rotation_angles_degrees(rotation_matrices: np.ndarray) -> np.ndarray:
    """Return rotation angles in degrees for a batch of rotation matrices."""
    traces = np.trace(rotation_matrices, axis1=-2, axis2=-1)
    cos_angles = np.clip(0.5 * (traces - 1.0), -1.0, 1.0)
    return np.degrees(np.arccos(cos_angles))


def so3_exp_batch(vectors: np.ndarray) -> np.ndarray:
    """Vectorized SO(3) exponential map."""
    vectors = np.asarray(vectors, dtype=float)
    original_shape = vectors.shape[:-1]
    flat = vectors.reshape(-1, 3)
    theta = np.linalg.norm(flat, axis=1)
    skew = skew_matrices(flat)
    skew_sq = np.matmul(skew, skew)
    eye = np.broadcast_to(np.eye(3), skew.shape).copy()
    theta_sq = theta * theta
    small = theta < 1e-8
    A = np.empty_like(theta)
    B = np.empty_like(theta)
    A[~small] = np.sin(theta[~small]) / theta[~small]
    B[~small] = (1.0 - np.cos(theta[~small])) / theta_sq[~small]
    A[small] = 1.0 - theta_sq[small] / 6.0
    B[small] = 0.5 - theta_sq[small] / 24.0
    rotations = eye + A[:, None, None] * skew + B[:, None, None] * skew_sq
    return rotations.reshape(*original_shape, 3, 3)


def skew_matrix(vector: np.ndarray) -> np.ndarray:
    """Return the 3x3 skew matrix for one vector."""
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def skew_matrices(vectors: np.ndarray) -> np.ndarray:
    """Return skew matrices for a vector batch."""
    vectors = np.asarray(vectors, dtype=float).reshape(-1, 3)
    matrices = np.zeros((vectors.shape[0], 3, 3), dtype=float)
    matrices[:, 0, 1] = -vectors[:, 2]
    matrices[:, 0, 2] = vectors[:, 1]
    matrices[:, 1, 0] = vectors[:, 2]
    matrices[:, 1, 2] = -vectors[:, 0]
    matrices[:, 2, 0] = -vectors[:, 1]
    matrices[:, 2, 1] = vectors[:, 0]
    return matrices


def orthonormalize_rotations(matrices: np.ndarray) -> np.ndarray:
    """Project matrices to SO(3) with an SVD polar factor."""
    matrices = np.asarray(matrices, dtype=float)
    projected = np.empty_like(matrices)
    for index, matrix in enumerate(matrices):
        u, _, vh = np.linalg.svd(matrix)
        rotation = u @ vh
        if np.linalg.det(rotation) < 0.0:
            u[:, -1] *= -1.0
            rotation = u @ vh
        projected[index] = rotation
    return projected
