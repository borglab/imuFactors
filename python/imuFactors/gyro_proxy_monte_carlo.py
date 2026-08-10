"""SO(3) gyro integration Monte Carlo experiments on smooth EuRoC proxies."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Callable, Iterable, Sequence

import gtsam
import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp

from imuFactors import euroc, spectral, spectrogram
from imuFactors.gyro_integration import (
    integrate_magnus2_rot3,
    integrate_rk2_rot3,
    make_pseudospectral_pi_params,
    rotation_error_degrees,
)


METHOD_NAMES = ("RK2", "Magnus-2", "PsyChMagPI")
ALGORITHM_ORDER = (
    "RK2",
    "Magnus-2",
    "PsyChMagPI N_sqrt",
    "PsyChMagPI best fixed",
)
GYRO_COLUMNS = ("w_x", "w_y", "w_z")
BIAS_COLUMNS = ("b_w_x", "b_w_y", "b_w_z")


@dataclass(frozen=True)
class GyroProxySnippet:
    """Smooth 3-axis gyro proxy for one selected EuRoC window."""

    dataset: str
    csv_path: Path
    window_index: int
    window_start_s: float
    aggressive_score: float
    center_activity: float
    center_high_order_ratio: float
    center_normalized_rmse: float
    sample_seconds: np.ndarray
    raw_bias_corrected_gyro: np.ndarray
    cgl_node_seconds: np.ndarray
    cgl_node_values: np.ndarray
    reconstructed_samples: np.ndarray
    fit_relative_error: float
    middle_interval: tuple[float, float]
    reference_rotation: gtsam.Rot3
    noise_scale: float


@dataclass(frozen=True)
class GyroMonteCarloResult:
    """Per-trial metrics and grouped summaries for the proxy experiment."""

    method_metrics: pd.DataFrame
    algorithm_trials: pd.DataFrame
    algorithm_summary: pd.DataFrame
    psych_setting_summary: pd.DataFrame
    sqrt_lambda_summary: pd.DataFrame
    best_settings_by_band: pd.DataFrame


def load_proxy_snippets(
    cache: dict[str, np.ndarray],
    *,
    data_dir: Path,
    reference_N: int = 50,
    full_interval: tuple[float, float] = (0.0, 1.0),
    middle_interval: tuple[float, float] = (0.4, 0.6),
) -> list[GyroProxySnippet]:
    """Load selected windows and fit 3D Chebyshev2 proxy gyros."""
    snippets: list[GyroProxySnippet] = []
    for index, dataset in enumerate(cache["datasets"]):
        csv_path = _resolve_csv_path(Path(str(cache["csv_paths"][index])), data_dir)
        dataframe = euroc.load_euroc_csv(csv_path)
        time_values = dataframe["t"].to_numpy(dtype=float)
        starts, steps_per_window, _ = spectrogram.interval_window_starts(
            time_values,
            window_seconds=full_interval[1] - full_interval[0],
        )
        window_index = int(cache["window_indices"][index])
        if window_index >= starts.size:
            raise IndexError(f"{dataset} window {window_index} exceeds available windows")
        start = int(starts[window_index])
        stop = start + int(steps_per_window) + 1
        window = dataframe.iloc[start:stop]
        sample_seconds = window["t"].to_numpy(dtype=float) - float(window["t"].iloc[0])
        raw_gyro = window[list(GYRO_COLUMNS)].to_numpy(dtype=float)
        gyro_bias = window[list(BIAS_COLUMNS)].to_numpy(dtype=float)
        bias_corrected_gyro = raw_gyro - gyro_bias

        plan = spectral.basis_plan_from_coordinates(
            int(reference_N),
            sample_seconds,
            basis="chebyshev2",
            interval=full_interval,
            window_seconds=full_interval[1] - full_interval[0],
            lambda1=0.0,
        )
        cgl_node_values = plan.fit(bias_corrected_gyro)
        reconstructed = plan.reconstruct(cgl_node_values)
        denominator = max(float(np.linalg.norm(bias_corrected_gyro, "fro")), 1e-15)
        fit_relative_error = float(
            np.linalg.norm(reconstructed - bias_corrected_gyro, "fro") / denominator
        )

        cgl_node_seconds = spectral.chebyshev2_points(int(reference_N), full_interval)
        proxy_without_reference = _ProxyForReference(
            cgl_node_values=cgl_node_values,
            full_interval=full_interval,
        )
        reference_rotation = integrate_gyro_function_reference(
            proxy_without_reference.evaluate,
            middle_interval,
        )
        noise_scale = proxy_gyro_norm_range(
            proxy_without_reference.evaluate,
            middle_interval,
        )

        snippets.append(
            GyroProxySnippet(
                dataset=str(dataset),
                csv_path=csv_path,
                window_index=window_index,
                window_start_s=float(cache["start_seconds"][index]),
                aggressive_score=float(cache["aggressive_scores"][index]),
                center_activity=float(cache["selection_activity"][index]),
                center_high_order_ratio=float(cache["selection_high_order_ratio"][index]),
                center_normalized_rmse=float(cache["selection_normalized_rmse"][index]),
                sample_seconds=sample_seconds,
                raw_bias_corrected_gyro=bias_corrected_gyro,
                cgl_node_seconds=cgl_node_seconds,
                cgl_node_values=cgl_node_values,
                reconstructed_samples=reconstructed,
                fit_relative_error=fit_relative_error,
                middle_interval=middle_interval,
                reference_rotation=reference_rotation,
                noise_scale=noise_scale,
            )
        )
    return snippets


def evaluate_proxy_gyro(
    snippet: GyroProxySnippet,
    times: Sequence[float] | np.ndarray,
) -> np.ndarray:
    """Evaluate one snippet's fitted 3-axis gyro proxy."""
    return _evaluate_chebyshev_proxy(
        snippet.cgl_node_values,
        np.asarray(times, dtype=float),
        (float(snippet.sample_seconds[0]), float(snippet.sample_seconds[-1])),
    )


def integrate_gyro_function_reference(
    omega_function: Callable[[np.ndarray], np.ndarray],
    interval: tuple[float, float],
    *,
    rtol: float = 1e-11,
    atol: float = 1e-13,
) -> gtsam.Rot3:
    """Integrate ``Rdot = R hat(omega(t))`` with an independent ODE solver."""
    a, b = float(interval[0]), float(interval[1])

    def right_hand_side(t: float, state: np.ndarray) -> np.ndarray:
        rotation = state.reshape(3, 3)
        omega = np.asarray(omega_function(np.asarray([t])), dtype=float).reshape(1, 3)[0]
        return (rotation @ _skew(omega)).reshape(-1)

    solution = solve_ivp(
        right_hand_side,
        (a, b),
        np.eye(3).reshape(-1),
        method="DOP853",
        rtol=rtol,
        atol=atol,
    )
    if not solution.success:
        raise RuntimeError(f"Reference SO(3) integration failed: {solution.message}")
    matrix = solution.y[:, -1].reshape(3, 3)
    return gtsam.Rot3.ClosestTo(matrix)


def proxy_gyro_norm_range(
    omega_function: Callable[[np.ndarray], np.ndarray],
    interval: tuple[float, float],
    *,
    grid_size: int = 1001,
) -> float:
    """Return ``range(||omega(t)||)`` over the comparison interval."""
    times = np.linspace(float(interval[0]), float(interval[1]), int(grid_size))
    values = np.asarray(omega_function(times), dtype=float)
    norms = np.linalg.norm(values, axis=1)
    return float(np.max(norms) - np.min(norms))


def standard_noise(
    seed: int,
    snippet_index: int,
    m: int,
    num_seeds: int,
) -> np.ndarray:
    """Return deterministic standard normal component noise."""
    rng = np.random.default_rng(
        int(seed) + 1000003 * int(snippet_index) + 1009 * int(m)
    )
    return rng.normal(size=(int(num_seeds), int(m), 3))


def run_gyro_proxy_monte_carlo(
    snippets: Sequence[GyroProxySnippet],
    *,
    m_values: Iterable[int],
    N_values: Iterable[int],
    lambda_values: Iterable[float],
    noise_fractions: Iterable[float],
    num_seeds: int,
    seed: int,
    tolerance: float = 1e-12,
    max_iterations: int = 50,
    use_approximate_integration_matrix: bool = False,
) -> GyroMonteCarloResult:
    """Run SO(3) Monte Carlo comparisons on smooth gyro proxies."""
    snippets = list(snippets)
    m_values = [int(value) for value in m_values]
    N_values = [int(value) for value in N_values]
    lambda_values = [float(value) for value in lambda_values]
    noise_fractions = [float(value) for value in noise_fractions]
    row_count = _metric_row_count(
        len(snippets),
        m_values,
        N_values,
        lambda_values,
        noise_fractions,
        int(num_seeds),
    )
    arrays = _MetricArrays(row_count)
    row_index = 0

    for snippet_index, snippet in enumerate(snippets):
        for m in m_values:
            sample_times = np.linspace(
                snippet.middle_interval[0],
                snippet.middle_interval[1],
                int(m),
            )
            clean_samples = evaluate_proxy_gyro(snippet, sample_times)
            noises = standard_noise(seed, snippet_index, m, num_seeds)
            valid_N_values = [N for N in N_values if N <= m]
            plan_cache = {
                (N, lambda1): spectral.basis_plan_from_coordinates(
                    N,
                    sample_times,
                    basis="chebyshev2",
                    interval=snippet.middle_interval,
                    window_seconds=snippet.middle_interval[1] - snippet.middle_interval[0],
                    lambda1=lambda1,
                )
                for N in valid_N_values
                for lambda1 in lambda_values
            }
            integrator_cache = {
                N: gtsam.PsyChMagPIRot3(
                    N,
                    snippet.middle_interval[0],
                    snippet.middle_interval[1],
                    make_pseudospectral_pi_params(
                        gtsam.PseudoSpectralPiInit.RK2,
                        tolerance=float(tolerance),
                        max_iterations=int(max_iterations),
                        use_approximate_integration_matrix=bool(
                            use_approximate_integration_matrix
                        ),
                    ),
                )
                for N in valid_N_values
            }

            for noise_fraction in noise_fractions:
                noise_std = float(noise_fraction) * snippet.noise_scale
                sample_batch = clean_samples[None, :, :] + noise_std * noises
                for seed_index in range(int(num_seeds)):
                    row_index = _append_baseline_row(
                        arrays,
                        row_index,
                        snippet_index,
                        m,
                        noise_fraction,
                        noise_std,
                        seed_index,
                        method_code=0,
                        reference_rotation=snippet.reference_rotation,
                        sample_times=sample_times,
                        sample_values=sample_batch[seed_index],
                    )
                    row_index = _append_baseline_row(
                        arrays,
                        row_index,
                        snippet_index,
                        m,
                        noise_fraction,
                        noise_std,
                        seed_index,
                        method_code=1,
                        reference_rotation=snippet.reference_rotation,
                        sample_times=sample_times,
                        sample_values=sample_batch[seed_index],
                    )

                for N in valid_N_values:
                    integrator = integrator_cache[N]
                    for lambda1 in lambda_values:
                        plan = plan_cache[(N, lambda1)]
                        cgl_batch = np.einsum(
                            "nm,smc->snc",
                            plan.solver,
                            sample_batch,
                        )
                        for seed_index, cgl_values in enumerate(cgl_batch):
                            row_index = _append_psychmagpi_row(
                                arrays,
                                row_index,
                                snippet_index,
                                m,
                                noise_fraction,
                                noise_std,
                                seed_index,
                                N,
                                lambda1,
                                snippet.reference_rotation,
                                integrator,
                                cgl_values,
                            )

    if row_index != row_count:
        raise RuntimeError(f"Filled {row_index} rows, expected {row_count}")

    method_metrics = arrays.to_frame(
        datasets=[snippet.dataset for snippet in snippets],
        noise_band_labels=noise_band_labels(noise_fractions),
    )
    psych_setting_summary = summarize_psychmagpi_settings(method_metrics)
    sqrt_lambda_summary = summarize_sqrt_lambda(method_metrics)
    best_settings = best_psychmagpi_settings_by_band(psych_setting_summary)
    sqrt_settings = best_psychmagpi_settings_by_band(
        psych_setting_summary[
            np.isclose(
                psych_setting_summary["N"],
                np.ceil(np.sqrt(psych_setting_summary["m"])),
            )
        ]
    )
    algorithm_trials = collapse_algorithm_trials(
        method_metrics,
        best_settings=best_settings,
        sqrt_settings=sqrt_settings,
    )
    algorithm_summary = summarize_algorithm_trials(algorithm_trials)
    return GyroMonteCarloResult(
        method_metrics=method_metrics,
        algorithm_trials=algorithm_trials,
        algorithm_summary=algorithm_summary,
        psych_setting_summary=psych_setting_summary,
        sqrt_lambda_summary=sqrt_lambda_summary,
        best_settings_by_band=best_settings,
    )


def collapse_algorithm_trials(
    method_metrics: pd.DataFrame,
    *,
    best_settings: pd.DataFrame | None = None,
    sqrt_settings: pd.DataFrame | None = None,
) -> pd.DataFrame:
    """Collapse PsyChMagPI settings into fixed-setting algorithm rows."""
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    baselines = method_metrics[method_metrics["method"].isin(["RK2", "Magnus-2"])].copy()
    baselines["algorithm"] = baselines["method"].astype(str)

    if best_settings is None or sqrt_settings is None:
        setting_summary = summarize_psychmagpi_settings(method_metrics)
        if best_settings is None:
            best_settings = best_psychmagpi_settings_by_band(setting_summary)
        if sqrt_settings is None:
            sqrt_settings = best_psychmagpi_settings_by_band(
                setting_summary[
                    np.isclose(
                        setting_summary["N"],
                        np.ceil(np.sqrt(setting_summary["m"])),
                    )
                ]
            )

    sqrt_best = _select_fixed_psychmagpi_rows(
        method_metrics,
        sqrt_settings,
        "PsyChMagPI N_sqrt",
    )
    best_fixed = _select_fixed_psychmagpi_rows(
        method_metrics,
        best_settings,
        "PsyChMagPI best fixed",
    )

    columns = [
        "dataset",
        "m",
        "N_sqrt",
        "noise_fraction",
        "noise_band",
        "noise_std",
        "seed_index",
        "algorithm",
        "method",
        "N",
        "lambda1",
        "error_deg",
        "converged",
        "iterations",
        "final_residual",
    ]
    output = pd.concat([baselines, sqrt_best, best_fixed], ignore_index=True)[columns]
    group_min = output.groupby(key, observed=True)["error_deg"].transform("min")
    output["winner"] = output["error_deg"] <= group_min + 1e-15
    output["algorithm"] = pd.Categorical(
        output["algorithm"],
        categories=list(ALGORITHM_ORDER),
        ordered=True,
    )
    return output.sort_values(key + ["algorithm"]).reset_index(drop=True)


def _select_fixed_psychmagpi_rows(
    method_metrics: pd.DataFrame,
    settings: pd.DataFrame,
    algorithm: str,
) -> pd.DataFrame:
    psych = method_metrics[method_metrics["method"] == "PsyChMagPI"].copy()
    selected_settings = settings[["m", "noise_band", "N", "lambda1"]].copy()
    selected_settings["N"] = selected_settings["N"].astype(float)
    selected_settings["lambda1"] = selected_settings["lambda1"].astype(float)
    merged = psych.merge(
        selected_settings,
        on=["m", "noise_band"],
        how="inner",
        suffixes=("", "_selected"),
    )
    selected = merged[
        np.isclose(merged["N"], merged["N_selected"])
        & np.isclose(merged["lambda1"], merged["lambda1_selected"])
    ].copy()
    selected["algorithm"] = algorithm
    return selected.drop(columns=["N_selected", "lambda1_selected"])


def summarize_algorithm_trials(algorithm_trials: pd.DataFrame) -> pd.DataFrame:
    """Summarize algorithm-level Monte Carlo outcomes."""
    return (
        algorithm_trials.groupby(["algorithm", "m", "noise_band"], observed=True)
        .agg(
            trials=("error_deg", "size"),
            win_rate=("winner", "mean"),
            median_error_deg=("error_deg", "median"),
            mean_error_deg=("error_deg", "mean"),
            max_error_deg=("error_deg", "max"),
        )
        .reset_index()
        .sort_values(["noise_band", "m", "algorithm"])
    )


def summarize_psychmagpi_settings(method_metrics: pd.DataFrame) -> pd.DataFrame:
    """Summarize every PsyChMagPI N/lambda setting against RK2/Magnus-2."""
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    baselines = method_metrics[method_metrics["method"].isin(["RK2", "Magnus-2"])]
    baseline_best = (
        baselines.groupby(key, observed=True)["error_deg"]
        .min()
        .rename("best_baseline_error_deg")
        .reset_index()
    )
    psych = method_metrics[method_metrics["method"] == "PsyChMagPI"].merge(
        baseline_best,
        on=key,
        how="left",
    )
    psych["win_vs_baselines"] = psych["error_deg"] < psych["best_baseline_error_deg"]
    return (
        psych.groupby(["N", "lambda1", "m", "noise_band"], observed=True)
        .agg(
            trials=("error_deg", "size"),
            win_rate_vs_baselines=("win_vs_baselines", "mean"),
            median_error_deg=("error_deg", "median"),
            mean_error_deg=("error_deg", "mean"),
            converged_rate=("converged", "mean"),
            median_iterations=("iterations", "median"),
            max_final_residual=("final_residual", "max"),
        )
        .reset_index()
        .sort_values(["noise_band", "m", "N", "lambda1"])
    )


def summarize_sqrt_lambda(method_metrics: pd.DataFrame) -> pd.DataFrame:
    """Summarize PsyChMagPI lambda choices at ``N=ceil(sqrt(m))``."""
    psych = method_metrics[
        (method_metrics["method"] == "PsyChMagPI")
        & np.isclose(method_metrics["N"], method_metrics["N_sqrt"])
    ].copy()
    return (
        psych.groupby(["lambda1", "m", "noise_band"], observed=True)
        .agg(
            trials=("error_deg", "size"),
            median_error_deg=("error_deg", "median"),
            mean_error_deg=("error_deg", "mean"),
            converged_rate=("converged", "mean"),
        )
        .reset_index()
        .sort_values(["noise_band", "m", "lambda1"])
    )


def best_psychmagpi_settings_by_band(
    psych_setting_summary: pd.DataFrame,
) -> pd.DataFrame:
    """Return best PsyChMagPI N/lambda per noise band and sample count."""
    rows = []
    for (noise_band, m), group in psych_setting_summary.groupby(
        ["noise_band", "m"], observed=True
    ):
        best = group.sort_values(
            ["win_rate_vs_baselines", "median_error_deg", "N", "lambda1"],
            ascending=[False, True, True, True],
        ).iloc[0]
        rows.append(best.to_dict())
    return pd.DataFrame(rows).sort_values(["noise_band", "m"]).reset_index(drop=True)


def proxy_metadata(snippets: Sequence[GyroProxySnippet]) -> pd.DataFrame:
    """Return one metadata row per proxy snippet."""
    rows = []
    for snippet in snippets:
        rows.append(
            {
                "dataset": snippet.dataset,
                "window_index": snippet.window_index,
                "window_start_s": snippet.window_start_s,
                "middle_interval_s": (
                    f"{snippet.middle_interval[0]:.1f}-{snippet.middle_interval[1]:.1f}"
                ),
                "reference_N": int(snippet.cgl_node_values.shape[0]),
                "window_samples": int(snippet.sample_seconds.size),
                "fit_relative_error": snippet.fit_relative_error,
                "noise_scale_rad_s": snippet.noise_scale,
                "aggressive_score": snippet.aggressive_score,
                "center_activity": snippet.center_activity,
                "center_high_order_ratio": snippet.center_high_order_ratio,
                "center_normalized_rmse": snippet.center_normalized_rmse,
            }
        )
    return pd.DataFrame(rows)


def noise_band_labels(noise_fractions: Sequence[float]) -> dict[float, str]:
    """Split sorted noise fractions into low/mid/high bands."""
    unique_values = np.asarray(sorted(set(float(value) for value in noise_fractions)))
    names = ("low", "mid", "high")
    groups = np.array_split(unique_values, min(len(names), unique_values.size))
    labels: dict[float, str] = {}
    for name, group in zip(names, groups):
        for value in group:
            labels[float(value)] = name
    return labels


@dataclass(frozen=True)
class _ProxyForReference:
    cgl_node_values: np.ndarray
    full_interval: tuple[float, float]

    def evaluate(self, times: np.ndarray) -> np.ndarray:
        return _evaluate_chebyshev_proxy(
            self.cgl_node_values,
            times,
            self.full_interval,
        )


@dataclass
class _MetricArrays:
    size: int

    def __post_init__(self) -> None:
        self.dataset_code = np.empty(self.size, dtype=np.int16)
        self.m = np.empty(self.size, dtype=np.int16)
        self.N_sqrt = np.empty(self.size, dtype=np.int16)
        self.noise_fraction = np.empty(self.size, dtype=np.float64)
        self.noise_std = np.empty(self.size, dtype=np.float64)
        self.seed_index = np.empty(self.size, dtype=np.int16)
        self.method_code = np.empty(self.size, dtype=np.int8)
        self.N = np.empty(self.size, dtype=np.float64)
        self.lambda1 = np.empty(self.size, dtype=np.float64)
        self.error_deg = np.empty(self.size, dtype=np.float64)
        self.converged = np.empty(self.size, dtype=bool)
        self.iterations = np.empty(self.size, dtype=np.int16)
        self.final_residual = np.empty(self.size, dtype=np.float64)

    def to_frame(
        self,
        *,
        datasets: Sequence[str],
        noise_band_labels: dict[float, str],
    ) -> pd.DataFrame:
        frame = pd.DataFrame(
            {
                "dataset": pd.Categorical.from_codes(self.dataset_code, list(datasets)),
                "m": self.m,
                "N_sqrt": self.N_sqrt,
                "noise_fraction": self.noise_fraction,
                "noise_std": self.noise_std,
                "noise_band": [
                    noise_band_labels[float(value)] for value in self.noise_fraction
                ],
                "seed_index": self.seed_index,
                "method": pd.Categorical.from_codes(
                    self.method_code,
                    list(METHOD_NAMES),
                    ordered=True,
                ),
                "N": self.N,
                "lambda1": self.lambda1,
                "error_deg": self.error_deg,
                "converged": self.converged,
                "iterations": self.iterations,
                "final_residual": self.final_residual,
            }
        )
        frame["noise_band"] = pd.Categorical(
            frame["noise_band"],
            categories=["low", "mid", "high"],
            ordered=True,
        )
        return frame


def _append_baseline_row(
    arrays: _MetricArrays,
    row_index: int,
    snippet_index: int,
    m: int,
    noise_fraction: float,
    noise_std: float,
    seed_index: int,
    *,
    method_code: int,
    reference_rotation: gtsam.Rot3,
    sample_times: np.ndarray,
    sample_values: np.ndarray,
) -> int:
    start = time.perf_counter()
    if method_code == 0:
        rotation = integrate_rk2_rot3(gtsam.Rot3(), sample_times, sample_values)
    elif method_code == 1:
        rotation = integrate_magnus2_rot3(gtsam.Rot3(), sample_times, sample_values)
    else:
        raise ValueError(f"Unknown baseline method code {method_code}")
    _ = time.perf_counter() - start
    arrays.dataset_code[row_index] = snippet_index
    arrays.m[row_index] = int(m)
    arrays.N_sqrt[row_index] = int(np.ceil(np.sqrt(float(m))))
    arrays.noise_fraction[row_index] = float(noise_fraction)
    arrays.noise_std[row_index] = float(noise_std)
    arrays.seed_index[row_index] = int(seed_index)
    arrays.method_code[row_index] = int(method_code)
    arrays.N[row_index] = np.nan
    arrays.lambda1[row_index] = np.nan
    arrays.error_deg[row_index] = rotation_error_degrees(reference_rotation, rotation)
    arrays.converged[row_index] = True
    arrays.iterations[row_index] = 0
    arrays.final_residual[row_index] = np.nan
    return row_index + 1


def _append_psychmagpi_row(
    arrays: _MetricArrays,
    row_index: int,
    snippet_index: int,
    m: int,
    noise_fraction: float,
    noise_std: float,
    seed_index: int,
    N: int,
    lambda1: float,
    reference_rotation: gtsam.Rot3,
    integrator: gtsam.PsyChMagPIRot3,
    cgl_values: np.ndarray,
) -> int:
    history = integrator.integrateWithHistory(cgl_values.T)
    error_norm = np.asarray(history.errorNorm, dtype=float).reshape(-1)
    final_residual = float(error_norm[-1]) if error_norm.size else np.nan
    final_z = np.asarray(history.finalZ, dtype=float)[:, -1]
    rotation = gtsam.Rot3.Expmap(final_z)
    arrays.dataset_code[row_index] = snippet_index
    arrays.m[row_index] = int(m)
    arrays.N_sqrt[row_index] = int(np.ceil(np.sqrt(float(m))))
    arrays.noise_fraction[row_index] = float(noise_fraction)
    arrays.noise_std[row_index] = float(noise_std)
    arrays.seed_index[row_index] = int(seed_index)
    arrays.method_code[row_index] = 2
    arrays.N[row_index] = float(N)
    arrays.lambda1[row_index] = float(lambda1)
    arrays.error_deg[row_index] = rotation_error_degrees(reference_rotation, rotation)
    arrays.converged[row_index] = bool(history.converged)
    arrays.iterations[row_index] = int(error_norm.size)
    arrays.final_residual[row_index] = final_residual
    return row_index + 1


def _metric_row_count(
    snippet_count: int,
    m_values: Sequence[int],
    N_values: Sequence[int],
    lambda_values: Sequence[float],
    noise_fractions: Sequence[float],
    num_seeds: int,
) -> int:
    rows = 0
    for m in m_values:
        valid_N_count = sum(1 for N in N_values if int(N) <= int(m))
        settings = 2 + valid_N_count * len(lambda_values)
        rows += snippet_count * len(noise_fractions) * int(num_seeds) * settings
    return rows


def _evaluate_chebyshev_proxy(
    cgl_node_values: np.ndarray,
    times: np.ndarray,
    interval: tuple[float, float],
) -> np.ndarray:
    time_array = np.asarray(times, dtype=float).reshape(-1)
    weights = spectral.chebyshev2_weight_matrix(
        int(cgl_node_values.shape[0]),
        time_array,
        interval,
    )
    return weights @ np.asarray(cgl_node_values, dtype=float)


def _resolve_csv_path(path: Path, data_dir: Path) -> Path:
    if path.exists():
        return path
    candidate = data_dir / path.name
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"Could not find EuRoC CSV {path}")


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
