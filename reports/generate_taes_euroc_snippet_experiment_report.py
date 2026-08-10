"""Generate a paper-facing EuRoC snippet experiment report for the TAES draft."""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import Sequence

import gtsam
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots


REPORT_DIR = Path(__file__).resolve().parent
REPO_ROOT = REPORT_DIR.parent
PYTHON_DIR = REPO_ROOT / "python"
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))
if str(REPORT_DIR) not in sys.path:
    sys.path.insert(0, str(REPORT_DIR))

import generate_chebyshev_quadrature_report as chebyshev_report
import generate_psychmagpi_gyro_report as psych_report
from imuFactors.gyro_integration import (
    integrate_psychmagpi_rot3,
    rotation_error_degrees,
)
from imuFactors.gyro_proxy_interpolation import (
    GyroInterpolationResult,
    angular_velocity_roughness,
)
from imuFactors.gyro_proxy_monte_carlo import (
    GyroProxySnippet,
    evaluate_proxy_gyro,
    load_proxy_snippets,
    noise_band_labels,
    proxy_metadata,
    standard_noise,
)


ASSET_DIR = REPORT_DIR / "taes_euroc_snippet_experiment_assets"
QMD_PATH = REPORT_DIR / "taes_euroc_snippet_experiment_report.qmd"
PDF_PATH = REPORT_DIR / "taes_euroc_snippet_experiment_report.pdf"
DATA_DIR = REPO_ROOT / "data" / "euroc"

ENDPOINT_METHOD_ORDER = (
    "D2/RK2",
    "D3 single-speed",
    "PsyChMagPI",
)
ENDPOINT_COLORS = {
    "D2/RK2": "#222222",
    "D3 single-speed": "#9c4a1a",
    "PsyChMagPI": "#1f4e79",
}
INTERPOLATION_SIGNAL_METHOD_ORDER = ("Linear", "Cubic spline", "PsyChMagPI")
INTERPOLATION_TRAJECTORY_METHOD_ORDER = (
    "RK2",
    "Magnus-2",
    "Linear",
    "Cubic spline",
    "PsyChMagPI",
)
INTERPOLATION_COLORS = {
    "RK2": "#222222",
    "Magnus-2": "#9c4a1a",
    "Linear": "#777777",
    "Cubic spline": "#b55d60",
    "PsyChMagPI": "#1f4e79",
}
RAW_PSYCH_METHOD = "PsyChMagPI lambda0"
REGULARIZED_PSYCH_METHOD = "PsyChMagPI scaled lambda"
NOISE_BANDS = ("low", "mid", "high")
SPEED_NUM_SEEDS = 5
PSYCHMAGPI_USE_APPROXIMATE_INTEGRATION_MATRIX = True
ENDPOINT_BENCHMARK_VERSION = 4
SPEED_BENCHMARK_VERSION = 5


def main() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    snippets = load_proxy_snippets(
        chebyshev_report.load_or_build_euroc_cache(),
        data_dir=DATA_DIR,
        reference_N=psych_report.REFERENCE_N,
        middle_interval=psych_report.MIDDLE_INTERVAL,
    )
    metadata = proxy_metadata(snippets)
    metadata.to_csv(ASSET_DIR / "proxy_snippet_metadata.csv", index=False)

    endpoint_metrics, endpoint_summary, endpoint_overall = load_or_run_endpoint_results(
        snippets
    )
    speed_metrics, speed_summary, speed_overall = load_or_run_speed_results(snippets)
    interpolation_result = load_interpolation_result(snippets)
    write_interpolation_csvs(interpolation_result)

    figure_paths = {
        "snippet_overview": psych_report.write_plotly_image(
            plot_snippet_overview(snippets),
            ASSET_DIR / "snippet_overview.png",
            width=1500,
            height=1100,
        ),
        "proxy_previews": psych_report.write_plotly_image(
            psych_report.plot_proxy_previews(snippets),
            ASSET_DIR / "proxy_gyro_previews.png",
            width=1500,
            height=1250,
        ),
        "endpoint_median": psych_report.write_plotly_image(
            plot_endpoint_median_errors(endpoint_summary),
            ASSET_DIR / "endpoint_median_error.png",
            width=1500,
            height=520,
        ),
        "endpoint_win_rate": psych_report.write_plotly_image(
            plot_endpoint_win_rates(endpoint_summary),
            ASSET_DIR / "endpoint_win_rate.png",
            width=1500,
            height=520,
        ),
        "endpoint_snippet": psych_report.write_plotly_image(
            plot_endpoint_snippet_outcomes(endpoint_metrics),
            ASSET_DIR / "endpoint_by_snippet.png",
            width=1500,
            height=620,
        ),
        "speed_by_m": psych_report.write_plotly_image(
            plot_speed_by_m(speed_summary),
            ASSET_DIR / "runtime_by_sample_count.png",
            width=1500,
            height=520,
        ),
        "interpolation_signal": psych_report.write_plotly_image(
            plot_interpolation_signal_nrmse(interpolation_result.signal_summary),
            ASSET_DIR / "interpolation_signal_nrmse.png",
            width=1500,
            height=520,
        ),
        "interpolation_trajectory": psych_report.write_plotly_image(
            plot_interpolation_trajectory_rms(interpolation_result.trajectory_summary),
            ASSET_DIR / "interpolation_trajectory_rms.png",
            width=1500,
            height=520,
        ),
        "interpolation_subintervals": psych_report.write_plotly_image(
            plot_interpolation_subinterval_rms(interpolation_result.subinterval_summary),
            ASSET_DIR / "interpolation_subinterval_rms.png",
            width=1500,
            height=520,
        ),
    }
    write_report_qmd(
        metadata=metadata,
        endpoint_metrics=endpoint_metrics,
        endpoint_summary=endpoint_summary,
        endpoint_overall=endpoint_overall,
        speed_summary=speed_summary,
        speed_overall=speed_overall,
        interpolation_result=interpolation_result,
        figure_paths=figure_paths,
    )


def load_or_run_endpoint_results(
    snippets: Sequence[GyroProxySnippet],
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    metrics_path = ASSET_DIR / "endpoint_trial_metrics.csv.gz"
    summary_path = ASSET_DIR / "endpoint_summary.csv"
    overall_path = ASSET_DIR / "endpoint_overall_summary.csv"
    if metrics_path.exists() and summary_path.exists() and overall_path.exists():
        metrics = pd.read_csv(metrics_path)
        if (
            "benchmark_version" in metrics
            and metrics["benchmark_version"].eq(ENDPOINT_BENCHMARK_VERSION).all()
        ):
            return (
                metrics,
                pd.read_csv(summary_path),
                pd.read_csv(overall_path),
            )

    metrics = run_endpoint_results(snippets)
    summary = summarize_endpoint_metrics(metrics)
    overall = summarize_endpoint_overall(metrics)
    metrics.to_csv(metrics_path, index=False, compression="gzip")
    summary.to_csv(summary_path, index=False)
    overall.to_csv(overall_path, index=False)
    return metrics, summary, overall


def run_endpoint_results(snippets: Sequence[GyroProxySnippet]) -> pd.DataFrame:
    label_map = noise_band_labels(psych_report.NOISE_FRACTIONS)
    rows: list[dict[str, float | int | str | bool]] = []
    for snippet_index, snippet in enumerate(snippets):
        dense_times = np.linspace(
            snippet.middle_interval[0],
            snippet.middle_interval[1],
            psych_report.INTERPOLATION_DENSE_COUNT,
        )
        clean_dense = evaluate_proxy_gyro(snippet, dense_times)
        roughness = angular_velocity_roughness(dense_times, clean_dense)
        for m in psych_report.SAMPLE_COUNTS:
            sample_times = np.linspace(
                snippet.middle_interval[0],
                snippet.middle_interval[1],
                int(m),
            )
            clean_samples = evaluate_proxy_gyro(snippet, sample_times)
            noises = standard_noise(
                psych_report.RANDOM_SEED,
                snippet_index,
                int(m),
                psych_report.NUM_SEEDS,
            )
            n_sqrt = int(np.ceil(np.sqrt(float(m))))
            for noise_fraction in psych_report.NOISE_FRACTIONS:
                noise_std = float(noise_fraction) * float(snippet.noise_scale)
                sample_batch = clean_samples[None, :, :] + noise_std * noises
                for seed_index, sample_values in enumerate(sample_batch):
                    rows.extend(
                        endpoint_trial_rows(
                            snippet=snippet,
                            m=int(m),
                            n_sqrt=n_sqrt,
                            noise_fraction=float(noise_fraction),
                            noise_band=label_map[float(noise_fraction)],
                            noise_std=noise_std,
                            seed_index=seed_index,
                            sample_times=sample_times,
                            sample_values=sample_values,
                            roughness=roughness,
                        )
                    )
    metrics = pd.DataFrame(rows)
    metrics = add_psych_vs_d3_wins(metrics, error_column="error_deg")
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    metrics["method"] = pd.Categorical(
        metrics["method"],
        categories=list(ENDPOINT_METHOD_ORDER),
        ordered=True,
    )
    return metrics.sort_values(key + ["method"]).reset_index(drop=True)


def endpoint_trial_rows(
    *,
    snippet: GyroProxySnippet,
    m: int,
    n_sqrt: int,
    noise_fraction: float,
    noise_band: str,
    noise_std: float,
    seed_index: int,
    sample_times: np.ndarray,
    sample_values: np.ndarray,
    roughness: float,
) -> list[dict[str, float | int | str | bool]]:
    rows: list[dict[str, float | int | str | bool]] = []
    reference = snippet.reference_rotation
    baseline_rotations = {
        "D2/RK2": integrate_sequential_rotations_cpp(sample_times, sample_values),
        "D3 single-speed": integrate_single_speed_coning_cpp(
            sample_times,
            sample_values,
        ),
    }
    for method, rotation in baseline_rotations.items():
        rows.append(
            make_endpoint_row(
                snippet=snippet,
                m=m,
                n_sqrt=n_sqrt,
                noise_fraction=noise_fraction,
                noise_band=noise_band,
                noise_std=noise_std,
                seed_index=seed_index,
                method=method,
                error_deg=rotation_error_degrees(reference, rotation),
                converged=True,
                iterations=0,
                final_residual=np.nan,
                roughness=roughness,
            )
        )
    psych = integrate_psychmagpi_rot3(
        gtsam.Rot3(),
        sample_times,
        sample_values,
        N=n_sqrt,
        tolerance=psych_report.PSYCHMAGPI_TOLERANCE,
        max_iterations=psych_report.PSYCHMAGPI_MAX_ITERATIONS,
        lambda1=0.0,
        use_approximate_integration_matrix=PSYCHMAGPI_USE_APPROXIMATE_INTEGRATION_MATRIX,
    )
    rows.append(
        make_endpoint_row(
            snippet=snippet,
            m=m,
            n_sqrt=n_sqrt,
            noise_fraction=noise_fraction,
            noise_band=noise_band,
            noise_std=noise_std,
            seed_index=seed_index,
            method="PsyChMagPI",
            error_deg=rotation_error_degrees(reference, psych.rotation),
            converged=bool(psych.converged),
            iterations=int(psych.iterations),
            final_residual=float(psych.final_residual),
            roughness=roughness,
        )
    )
    return rows


def make_endpoint_row(
    *,
    snippet: GyroProxySnippet,
    m: int,
    n_sqrt: int,
    noise_fraction: float,
    noise_band: str,
    noise_std: float,
    seed_index: int,
    method: str,
    error_deg: float,
    converged: bool,
    iterations: int,
    final_residual: float,
    roughness: float,
) -> dict[str, float | int | str | bool]:
    return {
        "dataset": snippet.dataset,
        "window_index": int(snippet.window_index),
        "m": int(m),
        "N_sqrt": int(n_sqrt),
        "noise_fraction": float(noise_fraction),
        "noise_band": str(noise_band),
        "noise_std": float(noise_std),
        "seed_index": int(seed_index),
        "method": method,
        "roughness": float(roughness),
        "error_deg": float(error_deg),
        "converged": bool(converged),
        "iterations": int(iterations),
        "final_residual": float(final_residual),
        "psychmagpi_integration_matrix": psychmagpi_integration_matrix_label(),
        "benchmark_version": int(ENDPOINT_BENCHMARK_VERSION),
    }


def add_psych_vs_d3_wins(metrics: pd.DataFrame, *, error_column: str) -> pd.DataFrame:
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    psych = metrics[metrics["method"].astype(str) == "PsyChMagPI"][
        key + [error_column]
    ].rename(columns={error_column: "psych_error"})
    d3 = metrics[metrics["method"].astype(str) == "D3 single-speed"][
        key + [error_column]
    ].rename(columns={error_column: "d3_error"})
    comparison = psych.merge(d3, on=key, how="inner")
    comparison["win_rate"] = (
        comparison["psych_error"] <= comparison["d3_error"] + 1e-15
    ).astype(float)
    comparison = comparison[key + ["win_rate"]]

    result = metrics.merge(comparison, on=key, how="left")
    result.loc[result["method"].astype(str) != "PsyChMagPI", "win_rate"] = np.nan
    return result


def summarize_endpoint_metrics(metrics: pd.DataFrame) -> pd.DataFrame:
    return (
        metrics.groupby(["method", "m", "noise_band"], observed=True)
        .agg(
            trials=("error_deg", "size"),
            win_rate=("win_rate", "mean"),
            median_error_deg=("error_deg", "median"),
            mean_error_deg=("error_deg", "mean"),
            max_error_deg=("error_deg", "max"),
            converged_rate=("converged", "mean"),
        )
        .reset_index()
        .sort_values(["noise_band", "m", "method"])
    )


def summarize_endpoint_overall(metrics: pd.DataFrame) -> pd.DataFrame:
    return (
        metrics.groupby("method", observed=True)
        .agg(
            trials=("error_deg", "size"),
            win_rate=("win_rate", "mean"),
            median_error_deg=("error_deg", "median"),
            mean_error_deg=("error_deg", "mean"),
            max_error_deg=("error_deg", "max"),
            converged_rate=("converged", "mean"),
        )
        .reset_index()
        .sort_values("method")
    )


def load_or_run_speed_results(
    snippets: Sequence[GyroProxySnippet],
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    metrics_path = ASSET_DIR / "runtime_trial_metrics.csv"
    summary_path = ASSET_DIR / "runtime_summary.csv"
    overall_path = ASSET_DIR / "runtime_overall_summary.csv"
    if metrics_path.exists() and summary_path.exists() and overall_path.exists():
        metrics = pd.read_csv(metrics_path)
        if (
            "benchmark_version" in metrics
            and metrics["benchmark_version"].eq(SPEED_BENCHMARK_VERSION).all()
        ):
            return (
                metrics,
                pd.read_csv(summary_path),
                pd.read_csv(overall_path),
            )

    metrics = run_speed_results(snippets)
    summary = summarize_speed_metrics(metrics)
    overall = summarize_speed_overall(metrics)
    metrics.to_csv(metrics_path, index=False)
    summary.to_csv(summary_path, index=False)
    overall.to_csv(overall_path, index=False)
    return metrics, summary, overall


def run_speed_results(snippets: Sequence[GyroProxySnippet]) -> pd.DataFrame:
    label_map = noise_band_labels(psych_report.NOISE_FRACTIONS)
    representative_fractions = representative_noise_fractions(label_map)
    rows: list[dict[str, float | int | str | bool]] = []
    for snippet_index, snippet in enumerate(snippets):
        for m in psych_report.SAMPLE_COUNTS:
            sample_times = np.linspace(
                snippet.middle_interval[0],
                snippet.middle_interval[1],
                int(m),
            )
            clean_samples = evaluate_proxy_gyro(snippet, sample_times)
            noises = standard_noise(
                psych_report.RANDOM_SEED + 7919,
                snippet_index,
                int(m),
                SPEED_NUM_SEEDS,
            )
            n_sqrt = int(np.ceil(np.sqrt(float(m))))
            for noise_fraction in representative_fractions:
                noise_std = float(noise_fraction) * float(snippet.noise_scale)
                sample_batch = clean_samples[None, :, :] + noise_std * noises
                for seed_index, sample_values in enumerate(sample_batch):
                    rows.extend(
                        speed_trial_rows(
                            snippet=snippet,
                            m=int(m),
                            n_sqrt=n_sqrt,
                            noise_fraction=float(noise_fraction),
                            noise_band=label_map[float(noise_fraction)],
                            noise_std=noise_std,
                            seed_index=seed_index,
                            sample_times=sample_times,
                            sample_values=sample_values,
                        )
                    )
    metrics = pd.DataFrame(rows)
    metrics["method"] = pd.Categorical(
        metrics["method"],
        categories=list(ENDPOINT_METHOD_ORDER),
        ordered=True,
    )
    return metrics.sort_values(
        ["dataset", "m", "noise_band", "seed_index", "method"]
    ).reset_index(drop=True)


def representative_noise_fractions(label_map: dict[float, str]) -> tuple[float, ...]:
    values: list[float] = []
    for band in NOISE_BANDS:
        band_values = sorted(
            float(value)
            for value, label in label_map.items()
            if str(label) == band
        )
        if not band_values:
            raise ValueError(f"No noise fractions found for band {band}")
        values.append(band_values[len(band_values) // 2])
    return tuple(values)


def speed_trial_rows(
    *,
    snippet: GyroProxySnippet,
    m: int,
    n_sqrt: int,
    noise_fraction: float,
    noise_band: str,
    noise_std: float,
    seed_index: int,
    sample_times: np.ndarray,
    sample_values: np.ndarray,
) -> list[dict[str, float | int | str | bool]]:
    rows: list[dict[str, float | int | str | bool]] = []
    d2_runtime_s, _ = time_integration_call(
        lambda: integrate_sequential_rotations_cpp(sample_times, sample_values)
    )
    d3_runtime_s, _ = time_integration_call(
        lambda: integrate_single_speed_coning_cpp(sample_times, sample_values)
    )
    rows.append(
        make_speed_row(
            snippet=snippet,
            m=m,
            n_sqrt=n_sqrt,
            noise_fraction=noise_fraction,
            noise_band=noise_band,
            noise_std=noise_std,
            seed_index=seed_index,
            method="D2/RK2",
            runtime_s=d2_runtime_s,
            converged=True,
            iterations=0,
        )
    )
    rows.append(
        make_speed_row(
            snippet=snippet,
            m=m,
            n_sqrt=n_sqrt,
            noise_fraction=noise_fraction,
            noise_band=noise_band,
            noise_std=noise_std,
            seed_index=seed_index,
            method="D3 single-speed",
            runtime_s=d3_runtime_s,
            converged=True,
            iterations=0,
        )
    )
    runtime_s, psych = time_integration_call(
        lambda: integrate_psychmagpi_rot3(
            gtsam.Rot3(),
            sample_times,
            sample_values,
            N=n_sqrt,
            tolerance=psych_report.PSYCHMAGPI_TOLERANCE,
            max_iterations=psych_report.PSYCHMAGPI_MAX_ITERATIONS,
            lambda1=0.0,
            use_approximate_integration_matrix=PSYCHMAGPI_USE_APPROXIMATE_INTEGRATION_MATRIX,
        )
    )
    rows.append(
        make_speed_row(
            snippet=snippet,
            m=m,
            n_sqrt=n_sqrt,
            noise_fraction=noise_fraction,
            noise_band=noise_band,
            noise_std=noise_std,
            seed_index=seed_index,
            method="PsyChMagPI",
            runtime_s=runtime_s,
            converged=bool(psych.converged),
            iterations=int(psych.iterations),
        )
    )
    return rows


def time_integration_call(function):
    start = time.perf_counter()
    result = function()
    return time.perf_counter() - start, result


def integrate_sequential_rotations_cpp(
    sample_times: np.ndarray,
    sample_values: np.ndarray,
) -> gtsam.Rot3:
    return gtsam.integrateSequentialRotations(
        np.asarray(sample_times, dtype=float),
        np.ascontiguousarray(sample_values, dtype=float),
    )


def integrate_single_speed_coning_cpp(
    sample_times: np.ndarray,
    sample_values: np.ndarray,
) -> gtsam.Rot3:
    return gtsam.integrateSingleSpeedConing(
        np.asarray(sample_times, dtype=float),
        np.ascontiguousarray(sample_values, dtype=float),
    )


def make_speed_row(
    *,
    snippet: GyroProxySnippet,
    m: int,
    n_sqrt: int,
    noise_fraction: float,
    noise_band: str,
    noise_std: float,
    seed_index: int,
    method: str,
    runtime_s: float,
    converged: bool,
    iterations: int,
) -> dict[str, float | int | str | bool]:
    return {
        "dataset": snippet.dataset,
        "window_index": int(snippet.window_index),
        "m": int(m),
        "N_sqrt": int(n_sqrt),
        "noise_fraction": float(noise_fraction),
        "noise_band": str(noise_band),
        "noise_std": float(noise_std),
        "seed_index": int(seed_index),
        "method": method,
        "runtime_s": float(runtime_s),
        "runtime_us": 1.0e6 * float(runtime_s),
        "converged": bool(converged),
        "iterations": int(iterations),
        "psychmagpi_integration_matrix": psychmagpi_integration_matrix_label(),
        "benchmark_version": int(SPEED_BENCHMARK_VERSION),
    }


def psychmagpi_integration_matrix_label() -> str:
    if PSYCHMAGPI_USE_APPROXIMATE_INTEGRATION_MATRIX:
        return "approximate_N_to_N"
    return "exact_N_to_N_plus_1"


def summarize_speed_metrics(metrics: pd.DataFrame) -> pd.DataFrame:
    summary = (
        metrics.groupby(["method", "m", "noise_band"], observed=True)
        .agg(
            trials=("runtime_s", "size"),
            median_runtime_us=("runtime_us", "median"),
            mean_runtime_us=("runtime_us", "mean"),
            p90_runtime_us=("runtime_us", lambda values: float(np.percentile(values, 90))),
            converged_rate=("converged", "mean"),
            median_iterations=("iterations", "median"),
        )
        .reset_index()
        .sort_values(["noise_band", "m", "method"])
    )
    baseline = summary[summary["method"].astype(str) == "D2/RK2"][
        ["m", "noise_band", "median_runtime_us"]
    ].rename(columns={"median_runtime_us": "d2_median_runtime_us"})
    summary = summary.merge(baseline, on=["m", "noise_band"], how="left")
    summary["slowdown_vs_d2"] = summary["median_runtime_us"] / summary[
        "d2_median_runtime_us"
    ]
    return summary


def summarize_speed_overall(metrics: pd.DataFrame) -> pd.DataFrame:
    overall = (
        metrics.groupby("method", observed=True)
        .agg(
            trials=("runtime_s", "size"),
            median_runtime_us=("runtime_us", "median"),
            mean_runtime_us=("runtime_us", "mean"),
            p90_runtime_us=("runtime_us", lambda values: float(np.percentile(values, 90))),
            converged_rate=("converged", "mean"),
            median_iterations=("iterations", "median"),
        )
        .reset_index()
        .sort_values("method")
    )
    d2_median = float(
        overall[overall["method"].astype(str) == "D2/RK2"]["median_runtime_us"].iloc[0]
    )
    overall["slowdown_vs_d2"] = overall["median_runtime_us"] / d2_median
    return overall.sort_values("slowdown_vs_d2").reset_index(drop=True)


def load_interpolation_result(
    snippets: Sequence[GyroProxySnippet],
) -> GyroInterpolationResult:
    result = psych_report.load_or_run_interpolation_experiments(
        snippets,
        asset_dir=psych_report.ASSET_DIR,
    )
    return unregularized_interpolation_result(result)


def unregularized_interpolation_result(
    result: GyroInterpolationResult,
) -> GyroInterpolationResult:
    return GyroInterpolationResult(
        signal_metrics=filter_unregularized_methods(result.signal_metrics),
        trajectory_metrics=filter_unregularized_methods(result.trajectory_metrics),
        subinterval_metrics=filter_unregularized_methods(result.subinterval_metrics),
        signal_summary=filter_unregularized_methods(result.signal_summary),
        trajectory_summary=filter_unregularized_methods(result.trajectory_summary),
        subinterval_summary=filter_unregularized_methods(result.subinterval_summary),
    )


def filter_unregularized_methods(frame: pd.DataFrame) -> pd.DataFrame:
    if "method" not in frame:
        return frame.copy()
    filtered = frame[frame["method"].astype(str) != REGULARIZED_PSYCH_METHOD].copy()
    filtered["method"] = filtered["method"].replace({RAW_PSYCH_METHOD: "PsyChMagPI"})
    drop_columns = [
        column
        for column in filtered.columns
        if "lambda" in column.lower() or column in {"win_rate", "winner"}
    ]
    if drop_columns:
        filtered = filtered.drop(columns=drop_columns)
    return filtered.reset_index(drop=True)


def write_interpolation_csvs(result: GyroInterpolationResult) -> None:
    result.signal_metrics.to_csv(
        ASSET_DIR / "interpolation_signal_metrics.csv.gz",
        index=False,
        compression="gzip",
    )
    result.trajectory_metrics.to_csv(
        ASSET_DIR / "interpolation_trajectory_metrics.csv.gz",
        index=False,
        compression="gzip",
    )
    result.subinterval_metrics.to_csv(
        ASSET_DIR / "interpolation_subinterval_metrics.csv.gz",
        index=False,
        compression="gzip",
    )
    result.signal_summary.to_csv(ASSET_DIR / "interpolation_signal_summary.csv", index=False)
    result.trajectory_summary.to_csv(
        ASSET_DIR / "interpolation_trajectory_summary.csv",
        index=False,
    )
    result.subinterval_summary.to_csv(
        ASSET_DIR / "interpolation_subinterval_summary.csv",
        index=False,
    )


def plot_snippet_overview(snippets: Sequence[GyroProxySnippet]) -> go.Figure:
    cols = 3
    rows = int(np.ceil(len(snippets) / cols))
    fig = make_subplots(
        rows=rows,
        cols=cols,
        subplot_titles=[snippet.dataset for snippet in snippets],
        horizontal_spacing=0.045,
        vertical_spacing=0.075,
    )
    for index, snippet in enumerate(snippets):
        row = index // cols + 1
        col = index % cols + 1
        dense_times = np.linspace(
            float(snippet.sample_seconds[0]),
            float(snippet.sample_seconds[-1]),
            501,
        )
        raw_norm = np.linalg.norm(snippet.raw_bias_corrected_gyro, axis=1)
        proxy_norm = np.linalg.norm(evaluate_proxy_gyro(snippet, dense_times), axis=1)
        fig.add_trace(
            go.Scatter(
                x=snippet.sample_seconds,
                y=raw_norm,
                mode="markers",
                marker=dict(size=2.5, color="rgba(80,80,80,0.38)"),
                name="raw bias-corrected samples",
                showlegend=index == 0,
                hovertemplate="t=%{x:.3f}s<br>|omega|=%{y:.3g}<extra></extra>",
            ),
            row=row,
            col=col,
        )
        fig.add_trace(
            go.Scatter(
                x=dense_times,
                y=proxy_norm,
                mode="lines",
                line=dict(width=2, color="#1f4e79"),
                name="Chebyshev2 proxy",
                showlegend=index == 0,
                hovertemplate="t=%{x:.3f}s<br>|omega*|=%{y:.3g}<extra></extra>",
            ),
            row=row,
            col=col,
        )
        fig.add_vrect(
            x0=snippet.middle_interval[0],
            x1=snippet.middle_interval[1],
            fillcolor="rgba(42,127,98,0.12)",
            line_width=0,
            row=row,
            col=col,
        )
    fig.update_xaxes(showgrid=False, title_text="seconds")
    fig.update_yaxes(gridcolor="rgba(0,0,0,0.08)", title_text="rad/s")
    fig.update_layout(
        template="plotly_white",
        title="Selected EuRoC aggressive 1 s snippets; shaded band is the tested 200 ms interval",
        legend=dict(orientation="h", y=1.04, x=0.0),
        margin=dict(l=74, r=24, t=92, b=56),
        width=1500,
        height=1100,
    )
    return fig


def plot_endpoint_median_errors(summary: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[psych_report.NOISE_BAND_LABELS[band] for band in NOISE_BANDS],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(NOISE_BANDS, start=1):
        band_data = summary[summary["noise_band"].astype(str) == band]
        for method in ENDPOINT_METHOD_ORDER:
            data = band_data[band_data["method"].astype(str) == method].sort_values("m")
            fig.add_trace(
                go.Scatter(
                    x=data["m"],
                    y=data["median_error_deg"],
                    mode="lines+markers",
                    line=dict(width=2, color=ENDPOINT_COLORS[method]),
                    marker=dict(size=7, color=ENDPOINT_COLORS[method]),
                    name=method,
                    legendgroup=method,
                    showlegend=col == 1,
                    hovertemplate=(
                        f"{method}<br>m=%{{x}}<br>"
                        "median error=%{y:.3g} deg<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="median final SO(3) error (deg)",
        type="log",
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    fig.update_layout(
        template="plotly_white",
        legend=dict(orientation="h", y=1.08, x=0.0),
        margin=dict(l=82, r=34, t=78, b=58),
        height=520,
        width=1500,
    )
    return fig


def plot_endpoint_win_rates(summary: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[psych_report.NOISE_BAND_LABELS[band] for band in NOISE_BANDS],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(NOISE_BANDS, start=1):
        band_data = summary[summary["noise_band"].astype(str) == band]
        method = "PsyChMagPI"
        data = band_data[band_data["method"].astype(str) == method].sort_values("m")
        fig.add_trace(
            go.Scatter(
                x=data["m"],
                y=data["win_rate"],
                mode="lines+markers",
                line=dict(width=2, color=ENDPOINT_COLORS[method]),
                marker=dict(size=7, color=ENDPOINT_COLORS[method]),
                name="PsyChMagPI vs D3",
                legendgroup=method,
                showlegend=col == 1,
                hovertemplate=(
                    "PsyChMagPI vs D3<br>m=%{x}<br>"
                    "win rate=%{y:.1%}<extra></extra>"
                ),
            ),
            row=1,
            col=col,
        )
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="PsyChMagPI win rate vs D3",
        tickmode="array",
        tickvals=[0.0, 0.25, 0.5, 0.75, 1.0],
        ticktext=["0", "25%", "50%", "75%", "100%"],
        range=[-0.02, 1.02],
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    fig.update_layout(
        template="plotly_white",
        legend=dict(orientation="h", y=1.08, x=0.0),
        margin=dict(l=82, r=34, t=78, b=58),
        height=520,
        width=1500,
    )
    return fig


def plot_endpoint_snippet_outcomes(metrics: pd.DataFrame) -> go.Figure:
    aggregate = (
        metrics.groupby(["dataset", "method"], observed=True)
        .agg(median_error_deg=("error_deg", "median"))
        .reset_index()
    )
    fig = go.Figure()
    datasets = list(dict.fromkeys(aggregate["dataset"].astype(str)))
    for method in ENDPOINT_METHOD_ORDER:
        data = aggregate[aggregate["method"].astype(str) == method]
        fig.add_trace(
            go.Scatter(
                x=data["dataset"],
                y=data["median_error_deg"],
                mode="markers+lines",
                line=dict(width=1.6, color=ENDPOINT_COLORS[method]),
                marker=dict(size=8, color=ENDPOINT_COLORS[method]),
                name=method,
                hovertemplate=(
                    f"{method}<br>%{{x}}<br>"
                    "median error=%{y:.3g} deg<extra></extra>"
                ),
            )
        )
    fig.update_layout(
        template="plotly_white",
        title="Per-snippet median endpoint error across all sample counts, noise levels, and seeds",
        xaxis=dict(categoryorder="array", categoryarray=datasets, showgrid=False),
        yaxis=dict(
            title="median final SO(3) error (deg)",
            type="log",
            gridcolor="rgba(0,0,0,0.12)",
        ),
        legend=dict(orientation="h", y=1.08, x=0.0),
        margin=dict(l=82, r=34, t=80, b=58),
        width=1500,
        height=620,
    )
    return fig


def plot_speed_by_m(summary: pd.DataFrame) -> go.Figure:
    aggregate = (
        summary.groupby(["method", "m"], observed=True)
        .agg(median_runtime_us=("median_runtime_us", "median"))
        .reset_index()
        .sort_values(["m", "method"])
    )
    fig = go.Figure()
    for method in ENDPOINT_METHOD_ORDER:
        data = aggregate[aggregate["method"].astype(str) == method].sort_values("m")
        fig.add_trace(
            go.Scatter(
                x=data["m"],
                y=data["median_runtime_us"],
                mode="lines+markers",
                line=dict(width=2, color=ENDPOINT_COLORS[method]),
                marker=dict(size=8, color=ENDPOINT_COLORS[method]),
                name=method,
                hovertemplate=(
                    f"{method}<br>m=%{{x}}<br>"
                    "median runtime=%{y:.3g} us<extra></extra>"
                ),
            )
        )
    fig.update_layout(
        template="plotly_white",
        title="Wrapped GTSAM wall-clock runtime by sample count",
        xaxis=dict(title="samples m", showgrid=False),
        yaxis=dict(
            title="median runtime per integration (us)",
            type="log",
            gridcolor="rgba(0,0,0,0.12)",
        ),
        legend=dict(orientation="h", y=1.08, x=0.0),
        margin=dict(l=82, r=34, t=78, b=58),
        width=1500,
        height=520,
    )
    return fig


def plot_interpolation_signal_nrmse(summary: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[psych_report.NOISE_BAND_LABELS[band] for band in NOISE_BANDS],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(NOISE_BANDS, start=1):
        band_data = summary[summary["noise_band"].astype(str) == band]
        for method in INTERPOLATION_SIGNAL_METHOD_ORDER:
            data = band_data[band_data["method"].astype(str) == method].sort_values("m")
            if data.empty:
                continue
            fig.add_trace(
                go.Scatter(
                    x=data["m"],
                    y=np.maximum(data["median_gyro_nrmse"].to_numpy(dtype=float), 1e-12),
                    mode="lines+markers",
                    line=dict(width=2, color=INTERPOLATION_COLORS[method]),
                    marker=dict(size=7, color=INTERPOLATION_COLORS[method]),
                    name=method,
                    legendgroup=method,
                    showlegend=col == 1,
                    hovertemplate=(
                        f"{method}<br>m=%{{x}}<br>"
                        "median NRMSE=%{y:.4g}<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="median gyro NRMSE",
        type="log",
        tickmode="array",
        tickvals=[0.03, 0.1, 0.3, 1.0, 3.0],
        ticktext=["0.03", "0.1", "0.3", "1", "3"],
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    for col in (2, 3):
        fig.update_yaxes(type="log", showticklabels=True, row=1, col=col)
    fig.update_layout(
        template="plotly_white",
        title="Gyro interpolation error against the clean proxy signal",
        legend=dict(orientation="h", y=1.16, x=0.0),
        margin=dict(l=82, r=32, t=116, b=62),
        width=1500,
        height=520,
    )
    return fig


def plot_interpolation_trajectory_rms(summary: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[psych_report.NOISE_BAND_LABELS[band] for band in NOISE_BANDS],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(NOISE_BANDS, start=1):
        band_data = summary[summary["noise_band"].astype(str) == band]
        for method in INTERPOLATION_TRAJECTORY_METHOD_ORDER:
            data = band_data[band_data["method"].astype(str) == method].sort_values("m")
            if data.empty:
                continue
            fig.add_trace(
                go.Scatter(
                    x=data["m"],
                    y=np.maximum(
                        data["median_trajectory_rms_error_deg"].to_numpy(dtype=float),
                        1e-12,
                    ),
                    mode="lines+markers",
                    line=dict(
                        width=2,
                        color=INTERPOLATION_COLORS[method],
                        dash="dash" if method in ("RK2", "Magnus-2") else "solid",
                    ),
                    marker=dict(size=7, color=INTERPOLATION_COLORS[method]),
                    name=method,
                    legendgroup=method,
                    showlegend=col == 1,
                    hovertemplate=(
                        f"{method}<br>m=%{{x}}<br>"
                        "median RMS=%{y:.4g} deg<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="median trajectory RMS (deg)",
        type="log",
        tickmode="array",
        tickvals=[0.01, 0.03, 0.1, 0.3, 1.0],
        ticktext=["0.01", "0.03", "0.1", "0.3", "1"],
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    for col in (2, 3):
        fig.update_yaxes(type="log", showticklabels=True, row=1, col=col)
    fig.update_layout(
        template="plotly_white",
        title="Attitude trajectory RMS error against the clean proxy trajectory",
        legend=dict(orientation="h", y=1.16, x=0.0),
        margin=dict(l=86, r=32, t=116, b=62),
        width=1500,
        height=520,
    )
    return fig


def plot_interpolation_subinterval_rms(summary: pd.DataFrame) -> go.Figure:
    aggregate = interpolation_subinterval_aggregate(summary)
    duration_order = ["short", "mid", "long"]
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[psych_report.NOISE_BAND_LABELS[band] for band in NOISE_BANDS],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(NOISE_BANDS, start=1):
        band_data = aggregate[aggregate["noise_band"].astype(str) == band]
        for method in INTERPOLATION_TRAJECTORY_METHOD_ORDER:
            data = band_data[band_data["method"].astype(str) == method].copy()
            if data.empty:
                continue
            data["duration_order"] = data["duration_bin"].map(
                {label: index for index, label in enumerate(duration_order)}
            )
            data = data.sort_values("duration_order")
            fig.add_trace(
                go.Scatter(
                    x=data["duration_bin"],
                    y=np.maximum(data["median_rms_error_deg"].to_numpy(dtype=float), 1e-12),
                    mode="lines+markers",
                    line=dict(
                        width=2,
                        color=INTERPOLATION_COLORS[method],
                        dash="dash" if method in ("RK2", "Magnus-2") else "solid",
                    ),
                    marker=dict(size=7, color=INTERPOLATION_COLORS[method]),
                    name=method,
                    legendgroup=method,
                    showlegend=col == 1,
                    hovertemplate=(
                        f"{method}<br>%{{x}} subintervals<br>"
                        "median RMS=%{y:.4g} deg<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.update_xaxes(title_text="duration bin", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="median subinterval RMS (deg)",
        type="log",
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    for col in (2, 3):
        fig.update_yaxes(type="log", showticklabels=True, row=1, col=col)
    fig.update_layout(
        template="plotly_white",
        title="Relative-rotation error over deterministic random subintervals",
        legend=dict(orientation="h", y=1.16, x=0.0),
        margin=dict(l=82, r=32, t=116, b=62),
        width=1500,
        height=520,
    )
    return fig


def interpolation_subinterval_aggregate(summary: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (method, band, duration), group in summary.groupby(
        ["method", "noise_band", "duration_bin"], observed=True
    ):
        weights = group["subinterval_count"].to_numpy(dtype=float)
        if np.sum(weights) <= 0.0:
            median_rms = np.nan
            median_error = np.nan
        else:
            median_rms = float(np.average(group["median_rms_error_deg"], weights=weights))
            median_error = float(np.average(group["median_error_deg"], weights=weights))
        rows.append(
            {
                "method": method,
                "noise_band": band,
                "duration_bin": duration,
                "subinterval_count": int(np.sum(weights)),
                "median_rms_error_deg": median_rms,
                "median_error_deg": median_error,
            }
        )
    return pd.DataFrame(rows)


def write_report_qmd(
    *,
    metadata: pd.DataFrame,
    endpoint_metrics: pd.DataFrame,
    endpoint_summary: pd.DataFrame,
    endpoint_overall: pd.DataFrame,
    speed_summary: pd.DataFrame,
    speed_overall: pd.DataFrame,
    interpolation_result: GyroInterpolationResult,
    figure_paths: dict[str, Path],
) -> None:
    sections = [
        report_header(),
        executive_summary(endpoint_overall, speed_overall, interpolation_result),
        snippet_section(metadata, figure_paths),
        endpoint_section(endpoint_overall, endpoint_summary, figure_paths),
        speed_section(speed_summary, speed_overall, figure_paths),
        interpolation_section(interpolation_result, figure_paths),
        assets_section(),
    ]
    QMD_PATH.write_text(
        psych_report.number_display_equations(
            "\n\n".join(section.rstrip() for section in sections) + "\n"
        ),
        encoding="utf-8",
    )
    print(f"wrote {QMD_PATH}")


def report_header() -> str:
    return """---
title: "EuRoC Snippet Experiment for TAES Draft"
subtitle: "Endpoint integration and interpolation results on smooth aggressive gyro proxies"
author: "Frank Dellaert"
date: "2026-06-14"
format:
  pdf:
    number-sections: true
    colorlinks: true
    fig-pos: H
    geometry:
      - top=0.65in
      - bottom=0.65in
      - left=0.65in
      - right=0.65in
fontsize: 10pt
---
"""


def executive_summary(
    endpoint_overall: pd.DataFrame,
    speed_overall: pd.DataFrame,
    interpolation_result: GyroInterpolationResult,
) -> str:
    endpoint_table = endpoint_overall_table(endpoint_overall)
    speed_table = speed_overall_table(speed_overall)
    d3_speed = speed_overall[
        speed_overall["method"].astype(str) == "D3 single-speed"
    ].iloc[0]
    psych_speed = speed_overall[
        speed_overall["method"].astype(str) == "PsyChMagPI"
    ].iloc[0]
    psych_endpoint = endpoint_overall[
        endpoint_overall["method"].astype(str) == "PsyChMagPI"
    ].iloc[0]
    d3_endpoint = endpoint_overall[
        endpoint_overall["method"].astype(str) == "D3 single-speed"
    ].iloc[0]
    interpolation_sentence = interpolation_executive_sentence(interpolation_result)
    return f"""# Executive Summary

This report stages the EuRoC snippet material used for the compact TAES experiment assets. It summarizes the endpoint, runtime, and interpolation evidence behind the paper-side figure and tables.

Main conclusions:

- I used the same eleven aggressive EuRoC snippets as the PsyChMagPI gyro report, with smooth Chebyshev2 vector proxies fitted on 1 s windows and measurements restricted to the center 200 ms interval.
- For final endpoint integration, PsyChMagPI used the historical approximate `N -> N` integration matrix and beat D3 single-speed in {format_percent(float(psych_endpoint["win_rate"]))} of matched trials, with median final SO(3) error {psych_report.format_error(float(psych_endpoint["median_error_deg"]))} degrees versus D3's {psych_report.format_error(float(d3_endpoint["median_error_deg"]))} degrees.
- D3 single-speed was included as the classical coning baseline beyond D2/RK2, so the endpoint comparison is now aligned with the TAES draft's classical-method vocabulary.
- In wrapped GTSAM timing, D3 was {psych_report.format_ratio(float(d3_speed["slowdown_vs_d2"]))} the runtime of D2/RK2, while PsyChMagPI was {psych_report.format_ratio(float(psych_speed["slowdown_vs_d2"]))} the runtime of D2/RK2.
- I evaluated all interpolation evidence already developed for the EuRoC proxy study: gyro interpolation error, dense attitude trajectory error, and random-subinterval rotation error.
- {interpolation_sentence}

Endpoint aggregate summary:

{endpoint_table}

Runtime aggregate summary:

{speed_table}
"""


def snippet_section(metadata: pd.DataFrame, figure_paths: dict[str, Path]) -> str:
    table = snippet_metadata_table(metadata)
    return f"""# EuRoC Snippet Construction

I used selected EuRoC windows only to define smooth aggressive gyro proxies, not as ground-truth attitude trajectories. Each snippet is a 1 s bias-corrected gyro window fitted by a Chebyshev2 vector proxy with `N={psych_report.REFERENCE_N}`; every endpoint and interpolation metric below uses only the shaded 200 ms center interval.

![EuRoC aggressive snippet overview.]({psych_report.relative_asset(figure_paths["snippet_overview"])}){{width=100%}}

The proxy preview below shows the three bias-corrected gyro components and the smooth 1 s vector fit. This figure is useful for paper figure selection because it exposes both the raw IMU contamination and the smooth proxy used for controlled Monte Carlo evaluation.

![Raw bias-corrected gyro samples and Chebyshev2 vector proxies.]({psych_report.relative_asset(figure_paths["proxy_previews"])}){{width=100%}}

Snippet metadata:

{table}
"""


def endpoint_section(
    endpoint_overall: pd.DataFrame,
    endpoint_summary: pd.DataFrame,
    figure_paths: dict[str, Path],
) -> str:
    band_table = endpoint_band_table(endpoint_summary)
    overall_table = endpoint_overall_table(endpoint_overall)
    return f"""# Final Integration on the Middle 200 ms Window

I compared final rotation accuracy on identical noisy samples using D2/RK2, D3 single-speed, and PsyChMagPI with `N=ceil(sqrt(m))`. D2/RK2 composes trapezoidal per-sample Lie increments; D3 adds the adjacent-increment single-speed coning correction; PsyChMagPI fits a global CGL representation before SO(3) integration and uses the historical approximate `N -> N` integration matrix.

![Endpoint median final attitude error.]({psych_report.relative_asset(figure_paths["endpoint_median"])}){{width=100%}}

![PsyChMagPI matched-trial win rate against D3 single-speed.]({psych_report.relative_asset(figure_paths["endpoint_win_rate"])}){{width=100%}}

Endpoint aggregate summary:

{overall_table}

Endpoint summary by noise band and method:

{band_table}

The per-snippet view checks whether the aggregate result is dominated by one EuRoC sequence. I aggregated across all sample counts, noise levels, and seeds for each selected snippet.

![Endpoint median error by snippet.]({psych_report.relative_asset(figure_paths["endpoint_snippet"])}){{width=100%}}
"""


def speed_section(
    speed_summary: pd.DataFrame,
    speed_overall: pd.DataFrame,
    figure_paths: dict[str, Path],
) -> str:
    overall_table = speed_overall_table(speed_overall)
    by_m_table = speed_by_m_table(speed_summary)
    return f"""# Runtime Comparison

I timed the endpoint methods to estimate the computational cost that would accompany the accuracy results. This is a Python-level wall-clock benchmark of wrapped GTSAM calls, not a final optimized C++ timing claim: D2/RK2 calls `gtsam.integrateSequentialRotations`, D3 calls `gtsam.integrateSingleSpeedConing`, and PsyChMagPI calls the current GTSAM PsyChMagPI wrapper path using the historical approximate `N -> N` integration matrix and including the sample-to-CGL fit used by the report.

The benchmark used all eleven snippets, all sample counts, representative low/mid/high noise levels, and {SPEED_NUM_SEEDS} deterministic noisy draws per snippet/sample/noise combination. Each row timed one final-rotation integration on the middle 200 ms window.

![Runtime by sample count.]({psych_report.relative_asset(figure_paths["speed_by_m"])}){{width=100%}}

Runtime aggregate summary:

{overall_table}

Runtime by sample count:

{by_m_table}
"""


def interpolation_section(
    interpolation_result: GyroInterpolationResult,
    figure_paths: dict[str, Path],
) -> str:
    signal_table = interpolation_signal_table(interpolation_result.signal_summary)
    trajectory_table = interpolation_trajectory_table(
        interpolation_result.trajectory_summary
    )
    subinterval_table = interpolation_subinterval_table(
        interpolation_result.subinterval_summary
    )
    return f"""# Interpolation and Continuous-Trajectory Results

I included all three interpolation checks because the TAES paper can use them to separate endpoint integration accuracy from continuous representation quality. The signal methods are componentwise linear interpolation, componentwise natural cubic interpolation, and PsyChMagPI; the trajectory checks also include RK2 and Magnus-2 sample-trajectory baselines.

To measure signal fidelity, I compared each continuous angular-velocity estimate against the clean Chebyshev2 proxy on a dense 1001-point grid.

![Gyro interpolation normalized RMSE.]({psych_report.relative_asset(figure_paths["interpolation_signal"])}){{width=100%}}

Gyro interpolation summary:

{signal_table}

To measure dense attitude fidelity, I integrated each continuous gyro representation over the dense grid and compared the full attitude path against the clean reference trajectory.

![Dense attitude trajectory RMS error.]({psych_report.relative_asset(figure_paths["interpolation_trajectory"])}){{width=100%}}

Trajectory summary:

{trajectory_table}

To measure arbitrary interval fidelity, I compared relative rotations over deterministic random subintervals inside the 200 ms window.

![Random-subinterval rotation RMS error.]({psych_report.relative_asset(figure_paths["interpolation_subintervals"])}){{width=100%}}

Random-subinterval summary:

{subinterval_table}
"""


def assets_section() -> str:
    return f"""# Output Files

I wrote the paper-staging assets under `{psych_report.relative_asset(ASSET_DIR)}`. The endpoint trial rows are in `endpoint_trial_metrics.csv.gz`, the runtime rows are in `runtime_trial_metrics.csv`, and interpolation trial rows and summaries are copied into the same asset directory so the paper figures can be regenerated without depending on the earlier report directory.
"""


def snippet_metadata_table(metadata: pd.DataFrame) -> str:
    display = metadata.copy()
    for column in [
        "fit_relative_error",
        "noise_scale_rad_s",
        "aggressive_score",
        "center_activity",
    ]:
        display[column] = display[column].map(lambda value: f"{float(value):.4g}")
    return psych_report.markdown_table(
        display[
            [
                "dataset",
                "window_index",
                "window_start_s",
                "middle_interval_s",
                "window_samples",
                "fit_relative_error",
                "noise_scale_rad_s",
                "aggressive_score",
            ]
        ].rename(
            columns={
                "dataset": "Dataset",
                "window_index": "Window",
                "window_start_s": "Start s",
                "middle_interval_s": "Middle s",
                "window_samples": "Raw samples",
                "fit_relative_error": "Fit rel. err",
                "noise_scale_rad_s": "Noise scale",
                "aggressive_score": "Aggressive score",
            }
        )
    )


def interpolation_executive_sentence(result: GyroInterpolationResult) -> str:
    signal = aggregate_interpolation_signal_by_method(result.signal_summary)
    trajectory = aggregate_interpolation_trajectory_by_method(result.trajectory_summary)
    subinterval = aggregate_interpolation_subinterval_by_method(
        result.subinterval_summary
    )
    best_signal = signal.sort_values("median_gyro_nrmse").iloc[0]
    psych_signal = signal[signal["method"] == "PsyChMagPI"].iloc[0]
    best_trajectory = trajectory.sort_values(
        "median_trajectory_rms_error_deg"
    ).iloc[0]
    psych_trajectory = trajectory[trajectory["method"] == "PsyChMagPI"].iloc[0]
    best_subinterval = subinterval.sort_values("median_rms_error_deg").iloc[0]
    psych_subinterval = subinterval[subinterval["method"] == "PsyChMagPI"].iloc[0]
    signal_phrase = best_metric_phrase(
        best_method=str(best_signal["method"]),
        best_value=float(best_signal["median_gyro_nrmse"]),
        psych_value=float(psych_signal["median_gyro_nrmse"]),
        metric_label="gyro NRMSE",
        unit="",
    )
    trajectory_phrase = best_metric_phrase(
        best_method=str(best_trajectory["method"]),
        best_value=float(best_trajectory["median_trajectory_rms_error_deg"]),
        psych_value=float(psych_trajectory["median_trajectory_rms_error_deg"]),
        metric_label="trajectory RMS",
        unit=" degrees",
    )
    subinterval_phrase = best_metric_phrase(
        best_method=str(best_subinterval["method"]),
        best_value=float(best_subinterval["median_rms_error_deg"]),
        psych_value=float(psych_subinterval["median_rms_error_deg"]),
        metric_label="random-subinterval RMS",
        unit=" degrees",
    )
    return f"{signal_phrase} {trajectory_phrase} {subinterval_phrase}"


def best_metric_phrase(
    *,
    best_method: str,
    best_value: float,
    psych_value: float,
    metric_label: str,
    unit: str,
) -> str:
    best_text = f"{psych_report.format_error(best_value)}{unit}"
    psych_text = f"{psych_report.format_error(psych_value)}{unit}"
    if best_method == "PsyChMagPI":
        return f"PsyChMagPI had the best median {metric_label}, {best_text}."
    return (
        f"The best median {metric_label} was `{best_method}` at {best_text}; "
        f"PsyChMagPI had {psych_text}."
    )


def aggregate_interpolation_signal_by_method(summary: pd.DataFrame) -> pd.DataFrame:
    return (
        summary.groupby("method", observed=True)
        .agg(
            median_gyro_nrmse=("median_gyro_nrmse", "median"),
            median_denoising_gain_db=("median_denoising_gain_db", "median"),
        )
        .reset_index()
    )


def aggregate_interpolation_trajectory_by_method(summary: pd.DataFrame) -> pd.DataFrame:
    return (
        summary.groupby("method", observed=True)
        .agg(
            median_trajectory_rms_error_deg=(
                "median_trajectory_rms_error_deg",
                "median",
            ),
            median_final_error_deg=("median_final_error_deg", "median"),
        )
        .reset_index()
    )


def aggregate_interpolation_subinterval_by_method(summary: pd.DataFrame) -> pd.DataFrame:
    aggregate = interpolation_subinterval_aggregate(summary)
    return (
        aggregate.groupby("method", observed=True)
        .agg(
            median_rms_error_deg=("median_rms_error_deg", "median"),
            median_error_deg=("median_error_deg", "median"),
        )
        .reset_index()
    )


def interpolation_signal_table(summary: pd.DataFrame) -> str:
    aggregate = (
        summary.groupby(["method", "noise_band"], observed=True)
        .agg(
            median_gyro_nrmse=("median_gyro_nrmse", "median"),
            median_denoising_gain_db=("median_denoising_gain_db", "median"),
        )
        .reset_index()
    )
    rows = []
    for band in NOISE_BANDS:
        band_data = aggregate[aggregate["noise_band"].astype(str) == band]
        best = band_data.sort_values("median_gyro_nrmse").iloc[0]
        psych = band_data[band_data["method"] == "PsyChMagPI"].iloc[0]
        rows.append(
            {
                "Noise": band,
                "Best method": best["method"],
                "Best NRMSE": psych_report.format_error(
                    float(best["median_gyro_nrmse"])
                ),
                "PsyCh NRMSE": psych_report.format_error(
                    float(psych["median_gyro_nrmse"])
                ),
                "PsyCh denoise dB": psych_report.format_error(
                    float(psych["median_denoising_gain_db"])
                ),
            }
        )
    return psych_report.markdown_table(pd.DataFrame(rows))


def interpolation_trajectory_table(summary: pd.DataFrame) -> str:
    aggregate = (
        summary.groupby(["method", "noise_band"], observed=True)
        .agg(
            median_trajectory_rms_error_deg=(
                "median_trajectory_rms_error_deg",
                "median",
            ),
            median_final_error_deg=("median_final_error_deg", "median"),
        )
        .reset_index()
    )
    rows = []
    for band in NOISE_BANDS:
        band_data = aggregate[aggregate["noise_band"].astype(str) == band]
        best = band_data.sort_values("median_trajectory_rms_error_deg").iloc[0]
        psych = band_data[band_data["method"] == "PsyChMagPI"].iloc[0]
        rows.append(
            {
                "Noise": band,
                "Best method": best["method"],
                "Best RMS deg": psych_report.format_error(
                    float(best["median_trajectory_rms_error_deg"])
                ),
                "PsyCh RMS deg": psych_report.format_error(
                    float(psych["median_trajectory_rms_error_deg"])
                ),
                "PsyCh final deg": psych_report.format_error(
                    float(psych["median_final_error_deg"])
                ),
            }
        )
    return psych_report.markdown_table(pd.DataFrame(rows))


def interpolation_subinterval_table(summary: pd.DataFrame) -> str:
    aggregate = interpolation_subinterval_aggregate(summary)
    rows = []
    for band in NOISE_BANDS:
        for duration in ("short", "mid", "long"):
            data = aggregate[
                (aggregate["noise_band"].astype(str) == band)
                & (aggregate["duration_bin"].astype(str) == duration)
            ]
            if data.empty:
                continue
            best = data.sort_values("median_rms_error_deg").iloc[0]
            psych = data[data["method"] == "PsyChMagPI"].iloc[0]
            rows.append(
                {
                    "Noise": band,
                    "Duration": duration,
                    "Best method": best["method"],
                    "Best RMS deg": psych_report.format_error(
                        float(best["median_rms_error_deg"])
                    ),
                    "PsyCh RMS deg": psych_report.format_error(
                        float(psych["median_rms_error_deg"])
                    ),
                }
            )
    return psych_report.markdown_table(pd.DataFrame(rows))


def endpoint_overall_table(overall: pd.DataFrame) -> str:
    display = overall.copy()
    display["win_rate"] = display["win_rate"].map(format_optional_percent)
    for column in ["median_error_deg", "mean_error_deg", "max_error_deg"]:
        display[column] = display[column].map(psych_report.format_error)
    display["converged_rate"] = display["converged_rate"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    return psych_report.markdown_table(
        display[
            [
                "method",
                "trials",
                "win_rate",
                "median_error_deg",
                "mean_error_deg",
                "max_error_deg",
                "converged_rate",
            ]
        ].rename(
            columns={
                "method": "Method",
                "trials": "Trials",
                "win_rate": "PsyCh win vs D3",
                "median_error_deg": "Median err. deg",
                "mean_error_deg": "Mean err. deg",
                "max_error_deg": "Max err. deg",
                "converged_rate": "Converged",
            }
        )
    )


def endpoint_band_table(summary: pd.DataFrame) -> str:
    rows = []
    for band in NOISE_BANDS:
        band_data = summary[summary["noise_band"].astype(str) == band]
        for method in ENDPOINT_METHOD_ORDER:
            data = band_data[band_data["method"].astype(str) == method]
            if data.empty:
                continue
            weights = data["trials"].to_numpy(dtype=float)
            rows.append(
                {
                    "Noise": band,
                    "Method": method,
                    "PsyCh win vs D3": format_optional_percent(
                        np.average(data["win_rate"], weights=weights)
                    ),
                    "Median err. deg": psych_report.format_error(
                        float(data["median_error_deg"].median())
                    ),
                    "Mean err. deg": psych_report.format_error(
                        float(np.average(data["mean_error_deg"], weights=weights))
                    ),
                }
            )
    return psych_report.markdown_table(pd.DataFrame(rows))


def speed_overall_table(overall: pd.DataFrame) -> str:
    display = overall.copy()
    display["median_runtime_us"] = display["median_runtime_us"].map(format_runtime_us)
    display["p90_runtime_us"] = display["p90_runtime_us"].map(format_runtime_us)
    display["slowdown_vs_d2"] = display["slowdown_vs_d2"].map(psych_report.format_ratio)
    display["converged_rate"] = display["converged_rate"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    return psych_report.markdown_table(
        display[
            [
                "method",
                "trials",
                "median_runtime_us",
                "p90_runtime_us",
                "slowdown_vs_d2",
                "median_iterations",
                "converged_rate",
            ]
        ].rename(
            columns={
                "method": "Method",
                "trials": "Timed calls",
                "median_runtime_us": "Median runtime",
                "p90_runtime_us": "P90 runtime",
                "slowdown_vs_d2": "Runtime/D2",
                "median_iterations": "Median iters",
                "converged_rate": "Converged",
            }
        )
    )


def speed_by_m_table(summary: pd.DataFrame) -> str:
    rows = []
    aggregate = (
        summary.groupby(["method", "m"], observed=True)
        .agg(
            median_runtime_us=("median_runtime_us", "median"),
            slowdown_vs_d2=("slowdown_vs_d2", "median"),
        )
        .reset_index()
    )
    for m in psych_report.SAMPLE_COUNTS:
        data = aggregate[aggregate["m"] == int(m)]
        for method in ENDPOINT_METHOD_ORDER:
            row = data[data["method"].astype(str) == method]
            if row.empty:
                continue
            value = row.iloc[0]
            rows.append(
                {
                    "m": int(m),
                    "Method": method,
                    "Median runtime": format_runtime_us(
                        float(value["median_runtime_us"])
                    ),
                    "Runtime/D2": psych_report.format_ratio(
                        float(value["slowdown_vs_d2"])
                    ),
                }
            )
    return psych_report.markdown_table(pd.DataFrame(rows))


def format_runtime_us(value: float) -> str:
    value = float(value)
    if value >= 1000.0:
        return f"{value / 1000.0:.3g} ms"
    return f"{value:.3g} us"


def format_percent(value: float) -> str:
    return f"{100.0 * float(value):.0f}%"


def format_optional_percent(value: float) -> str:
    if pd.isna(value):
        return ""
    return format_percent(float(value))


if __name__ == "__main__":
    main()
