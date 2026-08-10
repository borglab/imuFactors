"""Generate a PsyChMagPI SO(3) gyro proxy Monte Carlo Quarto report."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Sequence

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
from imuFactors.gyro_proxy_interpolation import (
    SIGNAL_METHOD_ORDER,
    TRAJECTORY_METHOD_ORDER,
    GyroInterpolationResult,
    run_gyro_proxy_interpolation_experiments,
)
from imuFactors.gyro_proxy_monte_carlo import (
    ALGORITHM_ORDER,
    GyroMonteCarloResult,
    GyroProxySnippet,
    best_psychmagpi_settings_by_band,
    evaluate_proxy_gyro,
    load_proxy_snippets,
    proxy_metadata,
    run_gyro_proxy_monte_carlo,
)
from imuFactors.taes_pure_coning import (
    PureConingMonteCarloResult,
    TAES_TABLE_MONTE_CARLO_TRIALS,
    run_taes_table2_long_integration_monte_carlo,
)


ASSET_DIR = REPORT_DIR / "psychmagpi_gyro_report_assets"
QMD_PATH = REPORT_DIR / "psychmagpi_gyro_report.qmd"
PDF_PATH = REPORT_DIR / "psychmagpi_gyro_report.pdf"
SYSTEMATIC_QMD_PATH = REPORT_DIR / "psychmagpi_gyro_report_systematic.qmd"
SYSTEMATIC_PDF_PATH = REPORT_DIR / "psychmagpi_gyro_report_systematic.pdf"
APPROX_ASSET_DIR = REPORT_DIR / "psychmagpi_gyro_report_approximate_assets"
APPROX_QMD_PATH = REPORT_DIR / "psychmagpi_gyro_report_approximate.qmd"
APPROX_PDF_PATH = REPORT_DIR / "psychmagpi_gyro_report_approximate.pdf"
COMPARISON_ASSET_DIR = REPORT_DIR / "psychmagpi_gyro_exact_vs_approximate_assets"
COMPARISON_QMD_PATH = REPORT_DIR / "psychmagpi_gyro_exact_vs_approximate.qmd"
COMPARISON_PDF_PATH = REPORT_DIR / "psychmagpi_gyro_exact_vs_approximate.pdf"
DATA_DIR = REPO_ROOT / "data" / "euroc"

PSYCHMAGPI_TOLERANCE = 1e-12
PSYCHMAGPI_MAX_ITERATIONS = 50
PSYCHMAGPI_LAMBDA_VALUES = (0.0, 0.001, 0.005, 0.01, 0.05)
SQRT_VS_RK2_LAMBDA = 0.01
INTERPOLATION_C_LAMBDA = 1.0
INTERPOLATION_DENSE_COUNT = 1001
INTERPOLATION_SUBINTERVALS_PER_BIN = 24
METHOD_N_VALUES = tuple(int(value) for value in chebyshev_report.N_VALUES)
SAMPLE_COUNTS = tuple(int(value) for value in chebyshev_report.M_VALUES)
NOISE_FRACTIONS = tuple(float(value) for value in chebyshev_report.NOISE_FRACTIONS)
NUM_SEEDS = int(chebyshev_report.NUM_SEEDS)
RANDOM_SEED = int(chebyshev_report.RANDOM_SEED)
REFERENCE_N = int(chebyshev_report.REFERENCE_N)
MIDDLE_INTERVAL = tuple(float(value) for value in chebyshev_report.MIDDLE_INTERVAL)

ALGORITHM_COLORS = {
    "RK2": "#222222",
    "Magnus-2": "#9c4a1a",
    "PsyChMagPI N_sqrt": "#1f4e79",
    "PsyChMagPI best fixed": "#2a7f62",
}
INTERPOLATION_COLORS = {
    "RK2": "#222222",
    "Magnus-2": "#9c4a1a",
    "Linear": "#6b6b6b",
    "Cubic spline": "#7d5aa6",
    "PsyChMagPI lambda0": "#1f4e79",
    "PsyChMagPI scaled lambda": "#2a7f62",
}
COMPONENT_COLORS = {
    "w_x - b_x": "#222222",
    "w_y - b_y": "#1f4e79",
    "w_z - b_z": "#9c4a1a",
}
NOISE_BAND_LABELS = {
    "low": "low noise",
    "mid": "mid noise",
    "high": "high noise",
}


@dataclass(frozen=True)
class ReportArtifacts:
    """Data, figures, and paths for one PsyChMagPI report variant."""

    label: str
    asset_dir: Path
    qmd_path: Path
    use_approximate_integration_matrix: bool
    metadata: pd.DataFrame
    result: GyroMonteCarloResult
    psych_vs_baseline_summary: pd.DataFrame
    sqrt_lambda_rk2_trials: pd.DataFrame
    sqrt_lambda_rk2_summary: pd.DataFrame
    taes_table2_mc: PureConingMonteCarloResult
    interpolation_result: GyroInterpolationResult
    figure_paths: dict[str, Path]


def main() -> None:
    cache = chebyshev_report.load_or_build_euroc_cache()
    snippets = load_proxy_snippets(
        cache,
        data_dir=DATA_DIR,
        reference_N=REFERENCE_N,
        middle_interval=MIDDLE_INTERVAL,
    )
    metadata = proxy_metadata(snippets)
    exact = build_report_variant(
        snippets,
        metadata=metadata,
        label="Exact N->N+1 integration matrix",
        asset_dir=ASSET_DIR,
        qmd_path=QMD_PATH,
        title="PsyChMagPI Gyro Integration on EuRoC Proxy Snippets",
        subtitle=(
            "TAES Table II reproduction and Monte Carlo SO(3) comparison on "
            "smooth aggressive 200 ms gyro proxies"
        ),
        use_approximate_integration_matrix=False,
    )
    write_systematic_report_qmd(
        metadata=exact.metadata,
        algorithm_trials=exact.result.algorithm_trials,
        algorithm_summary=exact.result.algorithm_summary,
        psych_vs_baseline_summary=exact.psych_vs_baseline_summary,
        psych_setting_summary=exact.result.psych_setting_summary,
        best_settings=exact.result.best_settings_by_band,
        sqrt_lambda_rk2_trials=exact.sqrt_lambda_rk2_trials,
        sqrt_lambda_rk2_summary=exact.sqrt_lambda_rk2_summary,
        taes_table2_mc=exact.taes_table2_mc,
        interpolation_result=exact.interpolation_result,
        figure_paths=exact.figure_paths,
        output_path=SYSTEMATIC_QMD_PATH,
    )
    approximate = build_report_variant(
        snippets,
        metadata=metadata,
        label="Approximate N->N integration matrix",
        asset_dir=APPROX_ASSET_DIR,
        qmd_path=APPROX_QMD_PATH,
        title=(
            "PsyChMagPI Gyro Integration on EuRoC Proxy Snippets "
            "(Approximate Integration Matrix)"
        ),
        subtitle=(
            "Same Monte Carlo study using the historical Chebyshev2 approximate "
            "integration matrix"
        ),
        use_approximate_integration_matrix=True,
    )
    write_comparison_report_qmd(
        exact,
        approximate,
        asset_dir=COMPARISON_ASSET_DIR,
        output_path=COMPARISON_QMD_PATH,
    )


def build_report_variant(
    snippets: Sequence[GyroProxySnippet],
    *,
    metadata: pd.DataFrame,
    label: str,
    asset_dir: Path,
    qmd_path: Path,
    title: str,
    subtitle: str,
    use_approximate_integration_matrix: bool,
) -> ReportArtifacts:
    """Generate or load one report variant and write its QMD."""
    asset_dir.mkdir(parents=True, exist_ok=True)
    result = load_or_run_gyro_proxy_monte_carlo(
        snippets,
        asset_dir=asset_dir,
        use_approximate_integration_matrix=use_approximate_integration_matrix,
    )

    metadata.to_csv(asset_dir / "proxy_snippet_metadata.csv", index=False)
    psych_vs_baseline_summary = summarize_psychmagpi_vs_baselines(
        result.algorithm_trials
    )
    psych_vs_baseline_summary.to_csv(
        asset_dir / "psychmagpi_vs_baseline_summary.csv",
        index=False,
    )
    sqrt_lambda_rk2_trials = sqrt_lambda_vs_rk2_trials(
        result.method_metrics,
        lambda1=SQRT_VS_RK2_LAMBDA,
    )
    sqrt_lambda_rk2_summary = summarize_sqrt_lambda_vs_rk2(
        sqrt_lambda_rk2_trials
    )
    sqrt_lambda_rk2_trials.to_csv(
        asset_dir / "psychmagpi_sqrt_lambda001_vs_rk2_trials.csv",
        index=False,
    )
    sqrt_lambda_rk2_summary.to_csv(
        asset_dir / "psychmagpi_sqrt_lambda001_vs_rk2_summary.csv",
        index=False,
    )
    taes_table2_mc = load_or_run_taes_table2_monte_carlo(
        asset_dir=asset_dir,
        use_approximate_integration_matrix=use_approximate_integration_matrix,
    )
    taes_table2_mc.summary.to_csv(
        asset_dir / "taes_table2_long_integration_sweep_summary.csv",
        index=False,
    )
    taes_table2_mc.trial_errors.to_csv(
        asset_dir / "taes_table2_long_integration_sweep_trials.csv.gz",
        index=False,
        compression="gzip",
    )
    interpolation_result = load_or_run_interpolation_experiments(
        snippets,
        asset_dir=asset_dir,
    )

    figure_paths = {
        "win_rates": write_plotly_image(
            plot_psychmagpi_win_rates(psych_vs_baseline_summary),
            asset_dir / "executive_win_rate_by_noise_band.png",
            width=1500,
            height=520,
        ),
        "median_errors": write_plotly_image(
            plot_algorithm_median_errors(result.algorithm_summary),
            asset_dir / "executive_median_error_by_noise_band.png",
            width=1500,
            height=520,
        ),
        "psych_sweep": write_plotly_image(
            plot_psychmagpi_sweep(result.psych_setting_summary),
            asset_dir / "psychmagpi_n_lambda_sweep.png",
            width=1500,
            height=760,
        ),
        "proxy_previews": write_plotly_image(
            plot_proxy_previews(snippets),
            asset_dir / "proxy_gyro_previews.png",
            width=1500,
            height=1250,
        ),
        "sqrt_lambda_rk2": write_plotly_image(
            plot_sqrt_lambda_vs_rk2(sqrt_lambda_rk2_summary),
            asset_dir / "sqrt_lambda001_vs_rk2.png",
            width=1500,
            height=520,
        ),
        "interpolation_signal": write_plotly_image(
            plot_interpolation_signal_nrmse(interpolation_result.signal_summary),
            asset_dir / "interpolation_signal_nrmse.png",
            width=1500,
            height=520,
        ),
        "interpolation_trajectory": write_plotly_image(
            plot_interpolation_trajectory_rms(
                interpolation_result.trajectory_summary
            ),
            asset_dir / "interpolation_trajectory_rms.png",
            width=1500,
            height=520,
        ),
        "interpolation_subintervals": write_plotly_image(
            plot_interpolation_subinterval_win_rate(
                interpolation_result.subinterval_summary
            ),
            asset_dir / "interpolation_subinterval_win_rate.png",
            width=1500,
            height=520,
        ),
    }
    write_report_qmd(
        metadata=metadata,
        algorithm_trials=result.algorithm_trials,
        algorithm_summary=result.algorithm_summary,
        psych_vs_baseline_summary=psych_vs_baseline_summary,
        psych_setting_summary=result.psych_setting_summary,
        best_settings=result.best_settings_by_band,
        sqrt_lambda_rk2_trials=sqrt_lambda_rk2_trials,
        sqrt_lambda_rk2_summary=sqrt_lambda_rk2_summary,
        taes_table2_mc=taes_table2_mc,
        interpolation_result=interpolation_result,
        figure_paths=figure_paths,
        asset_dir=asset_dir,
        output_path=qmd_path,
        title=title,
        subtitle=subtitle,
        integration_matrix_label=label,
        use_approximate_integration_matrix=use_approximate_integration_matrix,
    )
    return ReportArtifacts(
        label=label,
        asset_dir=asset_dir,
        qmd_path=qmd_path,
        use_approximate_integration_matrix=use_approximate_integration_matrix,
        metadata=metadata,
        result=result,
        psych_vs_baseline_summary=psych_vs_baseline_summary,
        sqrt_lambda_rk2_trials=sqrt_lambda_rk2_trials,
        sqrt_lambda_rk2_summary=sqrt_lambda_rk2_summary,
        taes_table2_mc=taes_table2_mc,
        interpolation_result=interpolation_result,
        figure_paths=figure_paths,
    )


def load_or_run_gyro_proxy_monte_carlo(
    snippets: Sequence[GyroProxySnippet],
    *,
    asset_dir: Path,
    use_approximate_integration_matrix: bool,
) -> GyroMonteCarloResult:
    method_path = asset_dir / "method_trial_metrics.csv.gz"
    algorithm_trials_path = asset_dir / "algorithm_trial_metrics.csv"
    algorithm_summary_path = asset_dir / "algorithm_summary.csv"
    setting_summary_path = asset_dir / "psychmagpi_setting_summary.csv"
    sqrt_summary_path = asset_dir / "psychmagpi_sqrt_lambda_summary.csv"
    best_settings_path = asset_dir / "psychmagpi_best_settings_by_band.csv"
    paths = [
        method_path,
        algorithm_trials_path,
        algorithm_summary_path,
        setting_summary_path,
        sqrt_summary_path,
        best_settings_path,
    ]
    if all(path.exists() for path in paths):
        return GyroMonteCarloResult(
            method_metrics=pd.read_csv(method_path),
            algorithm_trials=pd.read_csv(algorithm_trials_path),
            algorithm_summary=pd.read_csv(algorithm_summary_path),
            psych_setting_summary=pd.read_csv(setting_summary_path),
            sqrt_lambda_summary=pd.read_csv(sqrt_summary_path),
            best_settings_by_band=pd.read_csv(best_settings_path),
        )

    result = run_gyro_proxy_monte_carlo(
        snippets,
        m_values=SAMPLE_COUNTS,
        N_values=METHOD_N_VALUES,
        lambda_values=PSYCHMAGPI_LAMBDA_VALUES,
        noise_fractions=NOISE_FRACTIONS,
        num_seeds=NUM_SEEDS,
        seed=RANDOM_SEED,
        tolerance=PSYCHMAGPI_TOLERANCE,
        max_iterations=PSYCHMAGPI_MAX_ITERATIONS,
        use_approximate_integration_matrix=use_approximate_integration_matrix,
    )
    result.method_metrics.to_csv(
        method_path,
        index=False,
        compression="gzip",
    )
    result.algorithm_trials.to_csv(
        algorithm_trials_path,
        index=False,
    )
    result.algorithm_summary.to_csv(
        algorithm_summary_path,
        index=False,
    )
    result.psych_setting_summary.to_csv(
        setting_summary_path,
        index=False,
    )
    result.sqrt_lambda_summary.to_csv(
        sqrt_summary_path,
        index=False,
    )
    result.best_settings_by_band.to_csv(
        best_settings_path,
        index=False,
    )
    return result


def load_or_run_interpolation_experiments(
    snippets: Sequence[GyroProxySnippet],
    *,
    asset_dir: Path = ASSET_DIR,
) -> GyroInterpolationResult:
    signal_path = asset_dir / "interpolation_signal_metrics.csv.gz"
    trajectory_path = asset_dir / "interpolation_trajectory_metrics.csv.gz"
    subinterval_path = asset_dir / "interpolation_subinterval_metrics.csv.gz"
    signal_summary_path = asset_dir / "interpolation_signal_summary.csv"
    trajectory_summary_path = asset_dir / "interpolation_trajectory_summary.csv"
    subinterval_summary_path = asset_dir / "interpolation_subinterval_summary.csv"
    paths = [
        signal_path,
        trajectory_path,
        subinterval_path,
        signal_summary_path,
        trajectory_summary_path,
        subinterval_summary_path,
    ]
    if all(path.exists() for path in paths):
        return GyroInterpolationResult(
            signal_metrics=pd.read_csv(signal_path),
            trajectory_metrics=pd.read_csv(trajectory_path),
            subinterval_metrics=pd.read_csv(subinterval_path),
            signal_summary=pd.read_csv(signal_summary_path),
            trajectory_summary=pd.read_csv(trajectory_summary_path),
            subinterval_summary=pd.read_csv(subinterval_summary_path),
        )

    result = run_gyro_proxy_interpolation_experiments(
        snippets,
        m_values=SAMPLE_COUNTS,
        noise_fractions=NOISE_FRACTIONS,
        num_seeds=NUM_SEEDS,
        seed=RANDOM_SEED,
        dense_count=INTERPOLATION_DENSE_COUNT,
        subintervals_per_duration_bin=INTERPOLATION_SUBINTERVALS_PER_BIN,
        c_lambda=INTERPOLATION_C_LAMBDA,
    )
    result.signal_metrics.to_csv(
        signal_path,
        index=False,
        compression="gzip",
    )
    result.trajectory_metrics.to_csv(
        trajectory_path,
        index=False,
        compression="gzip",
    )
    result.subinterval_metrics.to_csv(
        subinterval_path,
        index=False,
        compression="gzip",
    )
    result.signal_summary.to_csv(signal_summary_path, index=False)
    result.trajectory_summary.to_csv(trajectory_summary_path, index=False)
    result.subinterval_summary.to_csv(subinterval_summary_path, index=False)
    return result


def load_or_run_taes_table2_monte_carlo(
    *,
    asset_dir: Path = ASSET_DIR,
    use_approximate_integration_matrix: bool = False,
) -> PureConingMonteCarloResult:
    trials_path = asset_dir / "taes_table2_long_integration_sweep_trials.csv.gz"
    summary_path = asset_dir / "taes_table2_long_integration_sweep_summary.csv"
    if trials_path.exists() and summary_path.exists():
        trial_errors = pd.read_csv(trials_path)
        summary = pd.read_csv(summary_path)
        return PureConingMonteCarloResult(
            trial_errors=trial_errors,
            summary=summary,
            parameters={
                "num_trials": TAES_TABLE_MONTE_CARLO_TRIALS,
                "lambda_values": PSYCHMAGPI_LAMBDA_VALUES,
                "use_approximate_integration_matrix": bool(
                    use_approximate_integration_matrix
                ),
            },
        )
    return run_taes_table2_long_integration_monte_carlo(
        lambda_values=PSYCHMAGPI_LAMBDA_VALUES,
        num_trials=TAES_TABLE_MONTE_CARLO_TRIALS,
        psych_tolerance=PSYCHMAGPI_TOLERANCE,
        psych_max_iterations=PSYCHMAGPI_MAX_ITERATIONS,
        use_approximate_integration_matrix=use_approximate_integration_matrix,
    )


def plot_psychmagpi_win_rates(summary: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(("low", "mid", "high"), start=1):
        band_data = summary[summary["noise_band"].astype(str) == band]
        for algorithm in ("PsyChMagPI N_sqrt", "PsyChMagPI best fixed"):
            data = band_data[band_data["algorithm"].astype(str) == algorithm].sort_values("m")
            if data.empty:
                continue
            fig.add_trace(
                go.Scatter(
                    x=data["m"],
                    y=data["win_rate_vs_baselines"],
                    mode="lines+markers",
                    line=dict(width=2, color=ALGORITHM_COLORS[algorithm]),
                    marker=dict(size=7, color=ALGORITHM_COLORS[algorithm]),
                    name=algorithm,
                    legendgroup=algorithm,
                    showlegend=col == 1,
                    hovertemplate=(
                        f"{algorithm}<br>"
                        "m=%{x}<br>"
                        "win vs baselines=%{y:.1%}<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="win vs better RK2/Mag2",
        range=[-0.02, 1.02],
        tickmode="array",
        tickvals=[0.0, 0.25, 0.5, 0.75, 1.0],
        ticktext=["0", "25%", "50%", "75%", "100%"],
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    fig.update_layout(
        template="plotly_white",
        title="PsyChMagPI win rate against the better dense-sample baseline",
        legend=dict(orientation="h", y=1.16, x=0.0),
        margin=dict(l=76, r=32, t=116, b=62),
        width=1500,
        height=520,
    )
    return fig


def plot_algorithm_median_errors(summary: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        horizontal_spacing=0.055,
    )
    line_dashes = {
        "RK2": "solid",
        "Magnus-2": "dash",
        "PsyChMagPI N_sqrt": "solid",
        "PsyChMagPI best fixed": "solid",
    }
    line_widths = {
        "RK2": 3.4,
        "Magnus-2": 2.2,
        "PsyChMagPI N_sqrt": 2.0,
        "PsyChMagPI best fixed": 2.0,
    }
    marker_symbols = {
        "RK2": "circle",
        "Magnus-2": "circle-open",
        "PsyChMagPI N_sqrt": "diamond",
        "PsyChMagPI best fixed": "circle",
    }
    for col, band in enumerate(("low", "mid", "high"), start=1):
        band_data = summary[summary["noise_band"].astype(str) == band]
        for algorithm in ALGORITHM_ORDER:
            data = band_data[band_data["algorithm"].astype(str) == algorithm].sort_values("m")
            if data.empty:
                continue
            fig.add_trace(
                go.Scatter(
                    x=data["m"],
                    y=np.maximum(data["median_error_deg"].to_numpy(dtype=float), 1e-12),
                    mode="lines+markers",
                    line=dict(
                        width=line_widths[algorithm],
                        color=ALGORITHM_COLORS[algorithm],
                        dash=line_dashes[algorithm],
                    ),
                    marker=dict(
                        size=7,
                        color=ALGORITHM_COLORS[algorithm],
                        symbol=marker_symbols[algorithm],
                        line=dict(width=1.5, color=ALGORITHM_COLORS[algorithm]),
                    ),
                    name=algorithm,
                    legendgroup=algorithm,
                    showlegend=col == 1,
                    hovertemplate=(
                        f"{algorithm}<br>"
                        "m=%{x}<br>"
                        "median=%{y:.4g} deg<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="median final attitude error [deg]",
        type="log",
        tickmode="array",
        tickvals=[0.03, 0.1, 0.3, 1.0],
        ticktext=["0.03", "0.1", "0.3", "1.0"],
        range=[np.log10(0.025), np.log10(1.1)],
        ticks="outside",
        showticklabels=True,
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    for col in (2, 3):
        fig.update_yaxes(
            type="log",
            tickmode="array",
            tickvals=[0.03, 0.1, 0.3, 1.0],
            ticktext=["0.03", "0.1", "0.3", "1.0"],
            range=[np.log10(0.025), np.log10(1.1)],
            ticks="outside",
            showticklabels=True,
            gridcolor="rgba(0,0,0,0.12)",
            row=1,
            col=col,
        )
    fig.update_layout(
        template="plotly_white",
        title="Median final SO(3) error against the noiseless proxy truth",
        legend=dict(orientation="h", y=1.16, x=0.0),
        margin=dict(l=86, r=32, t=116, b=62),
        width=1500,
        height=520,
    )
    return fig


def plot_interpolation_signal_nrmse(summary: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(("low", "mid", "high"), start=1):
        band_data = summary[summary["noise_band"].astype(str) == band]
        for method in SIGNAL_METHOD_ORDER:
            data = band_data[band_data["method"].astype(str) == method].sort_values("m")
            if data.empty:
                continue
            fig.add_trace(
                go.Scatter(
                    x=data["m"],
                    y=np.maximum(
                        data["median_gyro_nrmse"].to_numpy(dtype=float),
                        1e-12,
                    ),
                    mode="lines+markers",
                    line=dict(width=2, color=INTERPOLATION_COLORS[method]),
                    marker=dict(size=7, color=INTERPOLATION_COLORS[method]),
                    name=method,
                    legendgroup=method,
                    showlegend=col == 1,
                    hovertemplate=(
                        f"{method}<br>"
                        "m=%{x}<br>"
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
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(("low", "mid", "high"), start=1):
        band_data = summary[summary["noise_band"].astype(str) == band]
        for method in TRAJECTORY_METHOD_ORDER:
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
                        f"{method}<br>"
                        "m=%{x}<br>"
                        "median RMS=%{y:.4g} deg<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="median trajectory RMS [deg]",
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


def plot_interpolation_subinterval_win_rate(summary: pd.DataFrame) -> go.Figure:
    aggregate = weighted_subinterval_aggregate(summary)
    duration_order = ["short", "mid", "long"]
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(("low", "mid", "high"), start=1):
        band_data = aggregate[aggregate["noise_band"].astype(str) == band]
        for method in TRAJECTORY_METHOD_ORDER:
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
                    y=data["win_rate"],
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
                        f"{method}<br>"
                        "%{x} subintervals<br>"
                        "win rate=%{y:.1%}<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.add_hline(
            y=1.0 / len(TRAJECTORY_METHOD_ORDER),
            line=dict(color="rgba(0,0,0,0.30)", width=1, dash="dot"),
            row=1,
            col=col,
        )
        fig.update_xaxes(title_text="duration bin", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="random-subinterval win rate",
        range=[0.0, 1.0],
        tickmode="array",
        tickvals=[0.0, 0.25, 0.5, 0.75, 1.0],
        ticktext=["0", "25%", "50%", "75%", "100%"],
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    fig.update_layout(
        template="plotly_white",
        title="Relative-rotation win rate over deterministic random subintervals",
        legend=dict(orientation="h", y=1.16, x=0.0),
        margin=dict(l=82, r=32, t=116, b=62),
        width=1500,
        height=520,
    )
    return fig


def weighted_subinterval_summary(group: pd.DataFrame) -> pd.Series:
    weights = group["subinterval_count"].to_numpy(dtype=float)
    if np.sum(weights) <= 0.0:
        return pd.Series({"win_rate": np.nan, "median_rms_error_deg": np.nan})
    return pd.Series(
        {
            "win_rate": float(np.average(group["win_rate"], weights=weights)),
            "median_rms_error_deg": float(group["median_rms_error_deg"].median()),
        }
    )


def weighted_subinterval_aggregate(summary: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (method, noise_band, duration_bin), group in summary.groupby(
        ["method", "noise_band", "duration_bin"],
        observed=True,
    ):
        weights = group["subinterval_count"].to_numpy(dtype=float)
        if np.sum(weights) <= 0.0:
            win_rate = np.nan
        else:
            win_rate = float(np.average(group["win_rate"], weights=weights))
        rows.append(
            {
                "method": method,
                "noise_band": noise_band,
                "duration_bin": duration_bin,
                "win_rate": win_rate,
                "median_rms_error_deg": float(
                    group["median_rms_error_deg"].median()
                ),
            }
        )
    return pd.DataFrame(rows)


def plot_sqrt_lambda_vs_rk2(summary: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(("low", "mid", "high"), start=1):
        band_data = summary[summary["noise_band"].astype(str) == band].sort_values("m")
        fig.add_trace(
            go.Scatter(
                x=band_data["m"],
                y=band_data["win_rate_vs_rk2"],
                mode="lines+markers",
                line=dict(width=2.4, color=ALGORITHM_COLORS["PsyChMagPI N_sqrt"]),
                marker=dict(size=7, color=ALGORITHM_COLORS["PsyChMagPI N_sqrt"]),
                name="PsyChMagPI N_sqrt, lambda1=0.01",
                legendgroup="psych",
                showlegend=col == 1,
                hovertemplate=(
                    "m=%{x}<br>"
                    "win vs RK2=%{y:.1%}<extra></extra>"
                ),
            ),
            row=1,
            col=col,
        )
        fig.add_hline(
            y=0.5,
            line=dict(color="rgba(0,0,0,0.35)", width=1, dash="dot"),
            row=1,
            col=col,
        )
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="win rate vs RK2",
        range=[0.0, 1.0],
        tickmode="array",
        tickvals=[0.0, 0.25, 0.5, 0.75, 1.0],
        ticktext=["0", "25%", "50%", "75%", "100%"],
        gridcolor="rgba(0,0,0,0.12)",
        row=1,
        col=1,
    )
    fig.update_layout(
        template="plotly_white",
        title="Fixed N_sqrt and lambda1=0.01 against RK2 only",
        legend=dict(orientation="h", y=1.16, x=0.0),
        margin=dict(l=76, r=32, t=116, b=62),
        width=1500,
        height=520,
    )
    return fig


def plot_psychmagpi_sweep(setting_summary: pd.DataFrame) -> go.Figure:
    aggregate = (
        setting_summary.groupby(["noise_band", "N", "lambda1"], observed=True)
        .agg(
            win_rate_vs_baselines=("win_rate_vs_baselines", "mean"),
            median_error_deg=("median_error_deg", "median"),
        )
        .reset_index()
    )
    fig = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        vertical_spacing=0.11,
    )
    lambda_values = sorted(float(value) for value in aggregate["lambda1"].unique())
    colors = {
        0.0: "#222222",
        0.001: "#6b6b6b",
        0.005: "#1f4e79",
        0.01: "#2a7f62",
        0.05: "#9c4a1a",
    }
    dashes = {0.0: "solid", 0.001: "dot", 0.005: "solid", 0.01: "dash", 0.05: "dash"}
    for row, band in enumerate(("low", "mid", "high"), start=1):
        band_data = aggregate[aggregate["noise_band"].astype(str) == band]
        for lambda1 in lambda_values:
            data = band_data[np.isclose(band_data["lambda1"], lambda1)].sort_values("N")
            if data.empty:
                continue
            label = f"lambda={lambda1:g}"
            fig.add_trace(
                go.Scatter(
                    x=data["N"],
                    y=data["win_rate_vs_baselines"],
                    mode="lines+markers",
                    line=dict(
                        width=2,
                        color=colors.get(lambda1, "#555555"),
                        dash=dashes.get(lambda1, "solid"),
                    ),
                    marker=dict(size=6, color=colors.get(lambda1, "#555555")),
                    name=label,
                    legendgroup=label,
                    showlegend=row == 1,
                    hovertemplate=(
                        f"{NOISE_BAND_LABELS[band]}<br>"
                        f"{label}<br>"
                        "N=%{x}<br>"
                        "win vs baselines=%{y:.1%}<extra></extra>"
                    ),
                ),
                row=row,
                col=1,
            )
        fig.update_yaxes(
            title_text="win vs RK2/Mag2",
            range=[-0.02, 1.02],
            tickmode="array",
            tickvals=[0.0, 0.5, 1.0],
            ticktext=["0", "50%", "100%"],
            gridcolor="rgba(0,0,0,0.12)",
            row=row,
            col=1,
        )
    fig.update_xaxes(title_text="PsyChMagPI CGL input nodes N", showgrid=False, row=3, col=1)
    fig.update_layout(
        template="plotly_white",
        title="PsyChMagPI N/lambda sweep, aggregated over snippets and sample counts",
        legend=dict(orientation="h", y=1.08, x=0.0),
        margin=dict(l=92, r=36, t=102, b=62),
        width=1500,
        height=760,
    )
    return fig


def plot_proxy_previews(snippets: Sequence[GyroProxySnippet]) -> go.Figure:
    rows = len(snippets)
    fig = make_subplots(
        rows=rows,
        cols=1,
        shared_xaxes=True,
        subplot_titles=[snippet.dataset for snippet in snippets],
        vertical_spacing=0.012,
    )
    dense_times = np.linspace(0.0, 1.0, 501)
    labels = list(COMPONENT_COLORS)
    for row, snippet in enumerate(snippets, start=1):
        proxy = evaluate_proxy_gyro(snippet, dense_times)
        for component_index, label in enumerate(labels):
            color = COMPONENT_COLORS[label]
            fig.add_trace(
                go.Scatter(
                    x=snippet.sample_seconds,
                    y=snippet.raw_bias_corrected_gyro[:, component_index],
                    mode="markers",
                    marker=dict(size=3.2, color=color, opacity=0.62),
                    name=f"{label} raw",
                    legendgroup=label,
                    showlegend=False,
                    hoverinfo="skip",
                ),
                row=row,
                col=1,
            )
            fig.add_trace(
                go.Scatter(
                    x=dense_times,
                    y=proxy[:, component_index],
                    mode="lines",
                    line=dict(width=1.5, color=color),
                    name=label,
                    legendgroup=label,
                    showlegend=row == 1,
                    hovertemplate=(
                        f"{snippet.dataset}<br>"
                        "t=%{x:.3f}s<br>"
                        f"{label}=%{{y:.4g}} rad/s<extra></extra>"
                    ),
                ),
                row=row,
                col=1,
            )
        fig.add_vrect(
            x0=snippet.middle_interval[0],
            x1=snippet.middle_interval[1],
            fillcolor="rgba(31,78,121,0.10)",
            line_width=0,
            row=row,
            col=1,
        )
        fig.update_yaxes(title_text="rad/s", row=row, col=1)
    fig.update_xaxes(title_text="seconds inside selected 1 s window", row=rows, col=1)
    fig.update_layout(
        template="plotly_white",
        title="Raw 200 Hz gyro sample dots and Chebyshev2 N=50 vector proxy",
        legend=dict(orientation="h", y=1.04, x=0.0),
        margin=dict(l=76, r=34, t=84, b=58),
        height=1250,
        width=1500,
    )
    fig.update_xaxes(showgrid=False)
    fig.update_yaxes(gridcolor="rgba(0,0,0,0.08)", zeroline=False)
    return fig


def write_report_qmd(
    *,
    metadata: pd.DataFrame,
    algorithm_trials: pd.DataFrame,
    algorithm_summary: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    psych_setting_summary: pd.DataFrame,
    best_settings: pd.DataFrame,
    sqrt_lambda_rk2_trials: pd.DataFrame,
    sqrt_lambda_rk2_summary: pd.DataFrame,
    taes_table2_mc: PureConingMonteCarloResult,
    interpolation_result: GyroInterpolationResult,
    figure_paths: dict[str, Path],
    asset_dir: Path,
    output_path: Path,
    title: str,
    subtitle: str,
    integration_matrix_label: str,
    use_approximate_integration_matrix: bool,
) -> None:
    sections = [
        report_header(title=title, subtitle=subtitle),
        executive_summary(
            algorithm_trials,
            psych_vs_baseline_summary,
            best_settings,
            taes_table2_mc,
            interpolation_result,
            figure_paths,
        ),
        data_and_methods_section(
            metadata,
            figure_paths,
            integration_matrix_label=integration_matrix_label,
            use_approximate_integration_matrix=use_approximate_integration_matrix,
        ),
        taes_table2_section(taes_table2_mc, asset_dir=asset_dir),
        monte_carlo_results_section(
            algorithm_summary,
            psych_vs_baseline_summary,
            figure_paths,
            asset_dir=asset_dir,
        ),
        psychmagpi_sweep_section(psych_setting_summary, best_settings, figure_paths),
        sqrt_lambda_vs_rk2_section(
            sqrt_lambda_rk2_trials,
            sqrt_lambda_rk2_summary,
            figure_paths,
            asset_dir=asset_dir,
        ),
        interpolation_fidelity_section(
            interpolation_result,
            figure_paths,
            asset_dir=asset_dir,
        ),
        conclusion_section(
            algorithm_trials,
            psych_vs_baseline_summary,
            best_settings,
            taes_table2_mc,
            interpolation_result,
        ),
    ]
    output_path.write_text(
        number_display_equations("\n\n".join(section.rstrip() for section in sections) + "\n"),
        encoding="utf-8",
    )
    print(f"wrote {output_path}")


def write_systematic_report_qmd(
    *,
    metadata: pd.DataFrame,
    algorithm_trials: pd.DataFrame,
    algorithm_summary: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    psych_setting_summary: pd.DataFrame,
    best_settings: pd.DataFrame,
    sqrt_lambda_rk2_trials: pd.DataFrame,
    sqrt_lambda_rk2_summary: pd.DataFrame,
    taes_table2_mc: PureConingMonteCarloResult,
    interpolation_result: GyroInterpolationResult,
    figure_paths: dict[str, Path],
    output_path: Path = SYSTEMATIC_QMD_PATH,
) -> None:
    sections = [
        systematic_report_header(),
        systematic_executive_summary(
            algorithm_trials,
            psych_vs_baseline_summary,
            taes_table2_mc,
            interpolation_result,
            figure_paths,
        ),
        systematic_experimental_design(metadata, figure_paths),
        systematic_algorithms_and_policy(),
        systematic_taes_experiment(taes_table2_mc),
        systematic_endpoint_experiment(
            algorithm_trials,
            algorithm_summary,
            psych_vs_baseline_summary,
            figure_paths,
        ),
        systematic_hyperparameter_experiment(
            sqrt_lambda_rk2_trials,
            sqrt_lambda_rk2_summary,
            figure_paths,
        ),
        systematic_interpolation_experiment(interpolation_result, figure_paths),
        systematic_discussion_and_recommendation(
            algorithm_trials,
            psych_vs_baseline_summary,
            taes_table2_mc,
            interpolation_result,
        ),
        systematic_appendix(
            metadata,
            algorithm_summary,
            psych_vs_baseline_summary,
            psych_setting_summary,
            best_settings,
            sqrt_lambda_rk2_summary,
            taes_table2_mc,
        ),
    ]
    output_path.write_text(
        number_display_equations("\n\n".join(section.rstrip() for section in sections) + "\n"),
        encoding="utf-8",
    )
    print(f"wrote {output_path}")


def write_comparison_report_qmd(
    exact: ReportArtifacts,
    approximate: ReportArtifacts,
    *,
    asset_dir: Path,
    output_path: Path,
) -> None:
    asset_dir.mkdir(parents=True, exist_ok=True)
    endpoint_delta = exact_approx_endpoint_delta(exact, approximate)
    overall_delta = exact_approx_overall_delta(exact, approximate)
    sqrt_rk2_delta = exact_approx_sqrt_rk2_delta(exact, approximate)
    taes_delta = exact_approx_taes_delta(exact, approximate)
    endpoint_delta.to_csv(asset_dir / "endpoint_exact_vs_approximate.csv", index=False)
    overall_delta.to_csv(asset_dir / "overall_exact_vs_approximate.csv", index=False)
    sqrt_rk2_delta.to_csv(asset_dir / "sqrt_lambda001_exact_vs_approximate.csv", index=False)
    taes_delta.to_csv(asset_dir / "taes_table2_exact_vs_approximate.csv", index=False)

    figure_paths = {
        "endpoint_delta": write_plotly_image(
            plot_exact_approx_endpoint_delta(endpoint_delta),
            asset_dir / "endpoint_median_error_delta.png",
            width=1500,
            height=520,
        ),
        "sqrt_rk2_delta": write_plotly_image(
            plot_exact_approx_sqrt_rk2_delta(sqrt_rk2_delta),
            asset_dir / "sqrt_lambda001_win_delta.png",
            width=1500,
            height=520,
        ),
    }

    sections = [
        comparison_report_header(),
        comparison_executive_summary(
            exact,
            approximate,
            overall_delta,
            sqrt_rk2_delta,
            taes_delta,
            figure_paths,
        ),
        comparison_endpoint_section(endpoint_delta, overall_delta, figure_paths),
        comparison_taes_section(taes_delta),
        comparison_interpolation_section(exact, approximate),
        comparison_assets_section(asset_dir),
    ]
    output_path.write_text(
        number_display_equations("\n\n".join(section.rstrip() for section in sections) + "\n"),
        encoding="utf-8",
    )
    print(f"wrote {output_path}")


def comparison_report_header() -> str:
    return """---
title: "Exact Versus Approximate PsyChMagPI Integration Matrix"
subtitle: "Same gyro proxy report inputs with only the PsyChMagPI integration matrix changed"
author: "Frank Dellaert"
date: "2026-06-10"
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


def comparison_executive_summary(
    exact: ReportArtifacts,
    approximate: ReportArtifacts,
    overall_delta: pd.DataFrame,
    sqrt_rk2_delta: pd.DataFrame,
    taes_delta: pd.DataFrame,
    figure_paths: dict[str, Path],
) -> str:
    overall_table = format_exact_approx_overall_table(overall_delta)
    sqrt_table = format_exact_approx_sqrt_rk2_overall_table(sqrt_rk2_delta)
    taes_table = format_exact_approx_taes_key_table(taes_delta)
    bullets = exact_approx_main_bullets(overall_delta, sqrt_rk2_delta, taes_delta)
    return f"""# Executive Summary

Main conclusions:

{bullets}

I compared two PsyChMagPI integration-matrix settings while keeping every other report input fixed. The exact report used the default `N -> N+1` integration matrix; the approximate report used `useApproximateIntegrationMatrix = true`, which calls the historical Chebyshev2 approximate integration matrix and returns an `N`-column tangent trajectory. In both reports I took the final state from the last returned trajectory column.

The comparison isolates PsyChMagPI only. RK2, Magnus-2, D1, D2/RK2, D3, sample selection, noise draws, Chebyshev proxy fits, `N` grids, and `lambda1` grids were identical across the two reports.

Overall EuRoC endpoint comparison:

{overall_table}

Fixed `N_sqrt, lambda1=0.01` comparison against RK2:

{sqrt_table}

TAES Table-II key rows:

{taes_table}

![Approximate minus exact median endpoint-error change.]({relative_asset(figure_paths["endpoint_delta"])}){{width=100%}}

![Approximate minus exact fixed-N_sqrt win-rate change against RK2.]({relative_asset(figure_paths["sqrt_rk2_delta"])}){{width=100%}}

The full exact report is `{relative_asset(exact.qmd_path)}`. The full approximate-mode report is `{relative_asset(approximate.qmd_path)}`.
"""


def comparison_endpoint_section(
    endpoint_delta: pd.DataFrame,
    overall_delta: pd.DataFrame,
    figure_paths: dict[str, Path],
) -> str:
    by_band = format_exact_approx_endpoint_band_table(endpoint_delta)
    return f"""# EuRoC Proxy Endpoint Differences

I used the EuRoC proxy endpoint task to measure how much the integration-matrix change affected the final SO(3) result. Positive median-error deltas mean the approximate matrix had larger error; positive win-rate deltas mean it won more often against the better RK2/Magnus-2 baseline.

![Approximate minus exact median endpoint-error change.]({relative_asset(figure_paths["endpoint_delta"])}){{width=100%}}

Endpoint differences by PsyChMagPI view, noise band, and sample count:

{by_band}
"""


def comparison_taes_section(taes_delta: pd.DataFrame) -> str:
    table = format_exact_approx_taes_full_table(taes_delta)
    return f"""# TAES Table-II Differences

I used the TAES Table-II reproduction as the clean low-noise check because PsyChMagPI should benefit most when the signal is smooth and the samples are not dominated by noise. The table reports RMS final attitude error in mrad; positive deltas mean the approximate integration matrix was worse.

{table}
"""


def comparison_interpolation_section(
    exact: ReportArtifacts,
    approximate: ReportArtifacts,
) -> str:
    return f"""# Interpolation and Trajectory Fidelity

I treated the interpolation section as a shared diagnostic rather than an integration-matrix comparison. Those metrics fit or interpolate noisy angular-velocity samples and then integrate the dense reconstructed signal with common trajectory code, so the PsyChMagPI integration matrix used by `PsyChMagPIRot3.integrateWithHistory` is not the active variable there.

I therefore interpreted the endpoint and TAES tables above as the relevant exact-versus-approximate comparison, while the interpolation section still supports the continuous-representation claim in each full report.
"""


def comparison_assets_section(asset_dir: Path) -> str:
    return f"""# Assets

I wrote the comparison CSVs and figures under `{relative_asset(asset_dir)}`.
"""


def exact_approx_endpoint_delta(
    exact: ReportArtifacts,
    approximate: ReportArtifacts,
) -> pd.DataFrame:
    keys = ["algorithm", "m", "noise_band"]
    merged = exact.psych_vs_baseline_summary.merge(
        approximate.psych_vs_baseline_summary,
        on=keys,
        suffixes=("_exact", "_approx"),
    )
    merged["win_rate_delta_pp"] = 100.0 * (
        merged["win_rate_vs_baselines_approx"]
        - merged["win_rate_vs_baselines_exact"]
    )
    merged["median_error_delta_deg"] = (
        merged["median_error_deg_approx"] - merged["median_error_deg_exact"]
    )
    merged["mean_error_delta_deg"] = (
        merged["mean_error_deg_approx"] - merged["mean_error_deg_exact"]
    )
    merged["median_error_ratio"] = merged["median_error_deg_approx"] / np.maximum(
        merged["median_error_deg_exact"],
        1e-15,
    )
    return merged.sort_values(["noise_band", "m", "algorithm"]).reset_index(drop=True)


def exact_approx_overall_delta(
    exact: ReportArtifacts,
    approximate: ReportArtifacts,
) -> pd.DataFrame:
    exact_overall = summarize_overall_psychmagpi_vs_baselines(
        exact.result.algorithm_trials
    )
    approximate_overall = summarize_overall_psychmagpi_vs_baselines(
        approximate.result.algorithm_trials
    )
    merged = exact_overall.merge(
        approximate_overall,
        on="algorithm",
        suffixes=("_exact", "_approx"),
    )
    merged["win_rate_delta_pp"] = 100.0 * (
        merged["win_rate_vs_baselines_approx"]
        - merged["win_rate_vs_baselines_exact"]
    )
    merged["median_error_delta_deg"] = (
        merged["median_error_deg_approx"] - merged["median_error_deg_exact"]
    )
    merged["mean_error_delta_deg"] = (
        merged["mean_error_deg_approx"] - merged["mean_error_deg_exact"]
    )
    merged["median_error_ratio"] = merged["median_error_deg_approx"] / np.maximum(
        merged["median_error_deg_exact"],
        1e-15,
    )
    return merged.sort_values("algorithm").reset_index(drop=True)


def exact_approx_sqrt_rk2_delta(
    exact: ReportArtifacts,
    approximate: ReportArtifacts,
) -> pd.DataFrame:
    keys = ["noise_band", "m", "N_sqrt", "lambda1"]
    merged = exact.sqrt_lambda_rk2_summary.merge(
        approximate.sqrt_lambda_rk2_summary,
        on=keys,
        suffixes=("_exact", "_approx"),
    )
    merged["win_rate_delta_pp"] = 100.0 * (
        merged["win_rate_vs_rk2_approx"] - merged["win_rate_vs_rk2_exact"]
    )
    merged["median_psych_error_delta_deg"] = (
        merged["median_psych_error_deg_approx"]
        - merged["median_psych_error_deg_exact"]
    )
    merged["median_delta_change_deg"] = (
        merged["median_delta_deg_approx"] - merged["median_delta_deg_exact"]
    )
    return merged.sort_values(["noise_band", "m"]).reset_index(drop=True)


def exact_approx_taes_delta(
    exact: ReportArtifacts,
    approximate: ReportArtifacts,
) -> pd.DataFrame:
    exact_wide = taes_table_mc_wide_frame(exact.taes_table2_mc.summary)
    approximate_wide = taes_table_mc_wide_frame(approximate.taes_table2_mc.summary)
    rows = []
    methods = [column for column in exact_wide.columns if column != "sigma_rad_s"]
    for _, exact_row in exact_wide.iterrows():
        sigma = float(exact_row["sigma_rad_s"])
        approximate_row = approximate_wide[
            np.isclose(approximate_wide["sigma_rad_s"], sigma)
        ].iloc[0]
        for method in methods:
            exact_value = float(exact_row[method])
            approximate_value = float(approximate_row[method])
            rows.append(
                {
                    "sigma_rad_s": sigma,
                    "method": method,
                    "exact_rms_error_mrad": exact_value,
                    "approximate_rms_error_mrad": approximate_value,
                    "delta_rms_error_mrad": approximate_value - exact_value,
                    "ratio": approximate_value / max(exact_value, 1e-15),
                }
            )
    return pd.DataFrame(rows).sort_values(["sigma_rad_s", "method"]).reset_index(drop=True)


def exact_approx_main_bullets(
    overall_delta: pd.DataFrame,
    sqrt_rk2_delta: pd.DataFrame,
    taes_delta: pd.DataFrame,
) -> str:
    sqrt = overall_delta[overall_delta["algorithm"] == "PsyChMagPI N_sqrt"].iloc[0]
    best = overall_delta[overall_delta["algorithm"] == "PsyChMagPI best fixed"].iloc[0]
    sqrt_weights = sqrt_rk2_delta["trials_exact"].to_numpy(dtype=float)
    sqrt_exact_win = np.average(
        sqrt_rk2_delta["win_rate_vs_rk2_exact"],
        weights=sqrt_weights,
    )
    sqrt_approx_win = np.average(
        sqrt_rk2_delta["win_rate_vs_rk2_approx"],
        weights=sqrt_weights,
    )
    taes_psych = taes_delta[taes_delta["method"] == "PsyChMagPI lambda1=0"]
    zero_taes = taes_psych[np.isclose(taes_psych["sigma_rad_s"], 0.0)].iloc[0]
    high_taes = taes_psych.sort_values("sigma_rad_s").iloc[-1]
    return "\n".join(
        [
            (
                "- For `PsyChMagPI N_sqrt`, the approximate matrix changed the "
                f"EuRoC endpoint win rate by {format_signed_pp(float(sqrt['win_rate_delta_pp']))} "
                f"and changed the median final error by "
                f"{format_signed_error(float(sqrt['median_error_delta_deg']))} degrees."
            ),
            (
                "- For `PsyChMagPI best fixed`, the approximate matrix changed the "
                f"EuRoC endpoint win rate by {format_signed_pp(float(best['win_rate_delta_pp']))} "
                f"and changed the median final error by "
                f"{format_signed_error(float(best['median_error_delta_deg']))} degrees."
            ),
            (
                "- For the deterministic `N_sqrt, lambda1=0.01` rule against RK2, "
                f"the aggregate win rate moved from {100.0 * sqrt_exact_win:.0f}% "
                f"to {100.0 * sqrt_approx_win:.0f}%."
            ),
            (
                "- On the TAES Table-II zero-noise row, unregularized PsyChMagPI "
                f"changed from {format_error(float(zero_taes['exact_rms_error_mrad']))} "
                f"mrad to {format_error(float(zero_taes['approximate_rms_error_mrad']))} "
                "mrad."
            ),
            (
                "- At the largest TAES Table-II noise level, unregularized PsyChMagPI "
                f"changed by {format_signed_error(float(high_taes['delta_rms_error_mrad']))} "
                "mrad."
            ),
        ]
    )


def plot_exact_approx_endpoint_delta(delta: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(("low", "mid", "high"), start=1):
        band_data = delta[delta["noise_band"].astype(str) == band]
        for algorithm in ("PsyChMagPI N_sqrt", "PsyChMagPI best fixed"):
            data = band_data[band_data["algorithm"].astype(str) == algorithm].sort_values("m")
            fig.add_trace(
                go.Scatter(
                    x=data["m"],
                    y=data["median_error_delta_deg"],
                    mode="lines+markers",
                    line=dict(width=2, color=ALGORITHM_COLORS[algorithm]),
                    marker=dict(size=7, color=ALGORITHM_COLORS[algorithm]),
                    name=algorithm,
                    legendgroup=algorithm,
                    showlegend=col == 1,
                    hovertemplate=(
                        f"{algorithm}<br>"
                        "m=%{x}<br>"
                        "approx-exact median=%{y:.3g} deg<extra></extra>"
                    ),
                ),
                row=1,
                col=col,
            )
        fig.add_hline(y=0.0, line_width=1, line_color="rgba(0,0,0,0.35)", row=1, col=col)
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="approx - exact median error (deg)",
        gridcolor="rgba(0,0,0,0.12)",
        zeroline=False,
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


def plot_exact_approx_sqrt_rk2_delta(delta: pd.DataFrame) -> go.Figure:
    fig = make_subplots(
        rows=1,
        cols=3,
        shared_yaxes=True,
        subplot_titles=[NOISE_BAND_LABELS[band] for band in ("low", "mid", "high")],
        horizontal_spacing=0.055,
    )
    for col, band in enumerate(("low", "mid", "high"), start=1):
        data = delta[delta["noise_band"].astype(str) == band].sort_values("m")
        fig.add_trace(
            go.Scatter(
                x=data["m"],
                y=data["win_rate_delta_pp"],
                mode="lines+markers",
                line=dict(width=2, color=ALGORITHM_COLORS["PsyChMagPI N_sqrt"]),
                marker=dict(size=7, color=ALGORITHM_COLORS["PsyChMagPI N_sqrt"]),
                name="N_sqrt lambda1=0.01",
                legendgroup="sqrt",
                showlegend=col == 1,
                hovertemplate=(
                    "m=%{x}<br>"
                    "approx-exact win=%{y:.2f} pp<extra></extra>"
                ),
            ),
            row=1,
            col=col,
        )
        fig.add_hline(y=0.0, line_width=1, line_color="rgba(0,0,0,0.35)", row=1, col=col)
        fig.update_xaxes(title_text="samples m", showgrid=False, row=1, col=col)
    fig.update_yaxes(
        title_text="approx - exact win rate (percentage points)",
        gridcolor="rgba(0,0,0,0.12)",
        zeroline=False,
        row=1,
        col=1,
    )
    fig.update_layout(
        template="plotly_white",
        legend=dict(orientation="h", y=1.08, x=0.0),
        margin=dict(l=92, r=34, t=78, b=58),
        height=520,
        width=1500,
    )
    return fig


def format_exact_approx_overall_table(delta: pd.DataFrame) -> str:
    display = delta.copy()
    display["win_rate_vs_baselines_exact"] = display[
        "win_rate_vs_baselines_exact"
    ].map(lambda value: f"{100.0 * float(value):.0f}%")
    display["win_rate_vs_baselines_approx"] = display[
        "win_rate_vs_baselines_approx"
    ].map(lambda value: f"{100.0 * float(value):.0f}%")
    display["win_rate_delta_pp"] = display["win_rate_delta_pp"].map(format_signed_pp)
    for column in [
        "median_error_deg_exact",
        "median_error_deg_approx",
        "median_error_delta_deg",
    ]:
        display[column] = display[column].map(
            format_signed_error if column.endswith("_delta_deg") else format_error
        )
    display["median_error_ratio"] = display["median_error_ratio"].map(format_ratio)
    return markdown_table(
        display[
            [
                "algorithm",
                "win_rate_vs_baselines_exact",
                "win_rate_vs_baselines_approx",
                "win_rate_delta_pp",
                "median_error_deg_exact",
                "median_error_deg_approx",
                "median_error_delta_deg",
                "median_error_ratio",
            ]
        ].rename(
            columns={
                "algorithm": "PsyCh view",
                "win_rate_vs_baselines_exact": "Exact win",
                "win_rate_vs_baselines_approx": "Approx win",
                "win_rate_delta_pp": "Delta win",
                "median_error_deg_exact": "Exact median deg",
                "median_error_deg_approx": "Approx median deg",
                "median_error_delta_deg": "Delta median deg",
                "median_error_ratio": "Approx/exact",
            }
        )
    )


def format_exact_approx_endpoint_band_table(delta: pd.DataFrame) -> str:
    display = delta.copy()
    display["win_rate_vs_baselines_exact"] = display[
        "win_rate_vs_baselines_exact"
    ].map(lambda value: f"{100.0 * float(value):.0f}%")
    display["win_rate_vs_baselines_approx"] = display[
        "win_rate_vs_baselines_approx"
    ].map(lambda value: f"{100.0 * float(value):.0f}%")
    display["win_rate_delta_pp"] = display["win_rate_delta_pp"].map(format_signed_pp)
    for column in [
        "median_error_deg_exact",
        "median_error_deg_approx",
        "median_error_delta_deg",
    ]:
        display[column] = display[column].map(
            format_signed_error if column.endswith("_delta_deg") else format_error
        )
    return markdown_table(
        display[
            [
                "noise_band",
                "m",
                "algorithm",
                "win_rate_vs_baselines_exact",
                "win_rate_vs_baselines_approx",
                "win_rate_delta_pp",
                "median_error_deg_exact",
                "median_error_deg_approx",
                "median_error_delta_deg",
            ]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "algorithm": "PsyCh view",
                "win_rate_vs_baselines_exact": "Exact win",
                "win_rate_vs_baselines_approx": "Approx win",
                "win_rate_delta_pp": "Delta win",
                "median_error_deg_exact": "Exact median",
                "median_error_deg_approx": "Approx median",
                "median_error_delta_deg": "Delta median",
            }
        )
    )


def format_exact_approx_sqrt_rk2_overall_table(delta: pd.DataFrame) -> str:
    rows = []
    for band, group in delta.groupby("noise_band", observed=True):
        weights = group["trials_exact"].to_numpy(dtype=float)
        rows.append(
            {
                "Noise": band,
                "Exact win": f"{100.0 * np.average(group['win_rate_vs_rk2_exact'], weights=weights):.0f}%",
                "Approx win": f"{100.0 * np.average(group['win_rate_vs_rk2_approx'], weights=weights):.0f}%",
                "Delta win": format_signed_pp(
                    100.0
                    * np.average(
                        group["win_rate_vs_rk2_approx"]
                        - group["win_rate_vs_rk2_exact"],
                        weights=weights,
                    )
                ),
                "Median delta change deg": format_signed_error(
                    float(group["median_delta_change_deg"].median())
                ),
            }
        )
    return markdown_table(pd.DataFrame(rows))


def format_exact_approx_taes_key_table(delta: pd.DataFrame) -> str:
    psych = delta[delta["method"].astype(str).str.startswith("PsyChMagPI")].copy()
    keep_sigmas = [0.0, float(psych["sigma_rad_s"].max())]
    display = psych[
        psych["sigma_rad_s"].map(lambda value: any(np.isclose(value, sigma) for sigma in keep_sigmas))
    ].copy()
    return format_exact_approx_taes_table(display)


def format_exact_approx_taes_full_table(delta: pd.DataFrame) -> str:
    return format_exact_approx_taes_table(delta)


def format_exact_approx_taes_table(delta: pd.DataFrame) -> str:
    display = delta.copy()
    display["sigma_rad_s"] = display["sigma_rad_s"].map(format_sigma)
    for column in [
        "exact_rms_error_mrad",
        "approximate_rms_error_mrad",
        "delta_rms_error_mrad",
    ]:
        display[column] = display[column].map(
            format_signed_error if column.startswith("delta") else format_error
        )
    display["ratio"] = display["ratio"].map(format_ratio)
    return markdown_table(
        display[
            [
                "sigma_rad_s",
                "method",
                "exact_rms_error_mrad",
                "approximate_rms_error_mrad",
                "delta_rms_error_mrad",
                "ratio",
            ]
        ].rename(
            columns={
                "sigma_rad_s": "Noise sigma rad/s",
                "method": "Method",
                "exact_rms_error_mrad": "Exact RMS mrad",
                "approximate_rms_error_mrad": "Approx RMS mrad",
                "delta_rms_error_mrad": "Delta mrad",
                "ratio": "Approx/exact",
            }
        )
    )


def systematic_report_header() -> str:
    return """---
title: "PsyChMagPI Gyro Integration Report"
subtitle: "TAES coning reproduction and EuRoC proxy Monte Carlo study"
author: "Frank Dellaert"
date: "2026-06-10"
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


def systematic_executive_summary(
    algorithm_trials: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    taes_table2_mc: PureConingMonteCarloResult,
    interpolation_result: GyroInterpolationResult,
    figure_paths: dict[str, Path],
) -> str:
    psych_overall = summarize_overall_psychmagpi_vs_baselines(algorithm_trials)
    best_fixed = psych_overall[
        psych_overall["algorithm"] == "PsyChMagPI best fixed"
    ].iloc[0]
    sqrt = psych_overall[psych_overall["algorithm"] == "PsyChMagPI N_sqrt"].iloc[0]
    band_sentence = psych_baseline_band_sentence(
        psych_vs_baseline_summary,
        "PsyChMagPI N_sqrt",
    )
    decision_table = format_systematic_decision_table(
        best_fixed=best_fixed,
        sqrt=sqrt,
        taes_table2_mc=taes_table2_mc,
        interpolation_result=interpolation_result,
    )
    return f"""# Executive Summary

PsyChMagPI is best characterized as a high-order continuous SO(3) representation whose advantage is strongest on smooth low-noise coning problems and on noisy continuous-query tasks, not as a uniformly dominant endpoint integrator on every short EuRoC proxy window.

Main conclusions:

- PsyChMagPI reproduced the expected low-noise advantage on the TAES Table-II coning benchmark.
- PsyChMagPI's coning advantage shrank at the largest TAES noise level, where the classical methods became nearly indistinguishable in RMS endpoint error.
- PsyChMagPI was competitive but not dominant on aggressive EuRoC proxy endpoint error: the best fixed setting won {100.0 * float(best_fixed["win_rate_vs_baselines"]):.0f}% against the better RK2/Magnus-2 baseline, and the practical `N_sqrt = ceil(sqrt(m))` rule won {100.0 * float(sqrt["win_rate_vs_baselines"]):.0f}% overall; {band_sentence}.
- PsyChMagPI has its clearest EuRoC value as a continuous representation for gyro interpolation, dense attitude trajectories, and random subinterval rotations.
- The practical unresolved issue is lambda policy: deterministic `N = ceil(sqrt(m))` is defensible, and regularization should be tied to a stated noise or signal-roughness model.

Decision snapshot:

{decision_table}

![PsyChMagPI endpoint win rate against the better dense-sample baseline.]({relative_asset(figure_paths["win_rates"])}){{width=100%}}
"""


def systematic_experimental_design(
    metadata: pd.DataFrame,
    figure_paths: dict[str, Path],
) -> str:
    return rf"""# Study Design

To separate implementation validation from EuRoC proxy performance, I posed two distinct questions. The TAES Table-II reproduction tested whether the PsyChMagPI implementation behaved like the method in our TAES draft on a controlled coning benchmark, while the EuRoC proxy Monte Carlo tested whether that behavior transferred to short aggressive inertial snippets.

To keep continuity with the earlier Chebyshev quadrature report, I used EuRoC to select aggressive windows and then replaced raw propeller-contaminated samples with smooth proxy signals. I fitted each selected 1 s window componentwise as a 3-axis Chebyshev2 proxy with $N={REFERENCE_N}$, and every method comparison used only the center $[0.4,0.6]$ s interval.

To make the Monte Carlo comparison fair, I controlled the sample count, noise level, and random draw so that all methods saw identical data. I used sample counts $m\in{list(SAMPLE_COUNTS)}$ with closed-interval endpoints, the same noise-fraction grid as the Chebyshev quadrature report, and {NUM_SEEDS} deterministic seeds for each snippet/sample-count/noise cell.

I used clean signals and trajectories as references, not the noisy IMU samples themselves. I measured endpoint attitude error against a tight-tolerance noiseless integration of the proxy, gyro interpolation against the clean proxy on a dense grid, and trajectory/subinterval errors against the clean dense reference attitude path.

![Raw EuRoC sample dots and smooth vector proxies.]({relative_asset(figure_paths["proxy_previews"])}){{width=100%}}
"""


def systematic_algorithms_and_policy() -> str:
    return rf"""# Algorithms and Parameter Policy

The method sequence runs from classical local coning approximations to global pseudospectral SO(3) integration. This ordering makes the comparison cumulative: D1 ignores within-block non-commutativity, D2/RK2 composes local rotations with the Lie-group trapezoidal update, D3 adds a classical coning correction, Magnus-2 adds a second-order commutator term, and PsyChMagPI fits a global angular-velocity model before solving the tangent-space attitude equation.

The Bortz equation supplies the common Lie-algebra view of the integration problem. With body-rate convention $\dot R=R\widehat{{\boldsymbol{{\omega}}}}$ and $R(t)=R(a)\operatorname{{Exp}}(\boldsymbol{{\phi}}(t))$,

$$
\dot{{\boldsymbol{{\phi}}}}
= J_r(\boldsymbol{{\phi}})^{{-1}}\boldsymbol{{\omega}}
= \boldsymbol{{\omega}}
+\frac{{1}}{{2}}\boldsymbol{{\phi}}\times\boldsymbol{{\omega}}
+c(\lVert\boldsymbol{{\phi}}\rVert)\,
\boldsymbol{{\phi}}\times\left(\boldsymbol{{\phi}}\times\boldsymbol{{\omega}}\right),
$$

where

$$
c(\theta)=\frac{{1}}{{\theta^2}}-\frac{{1+\cos\theta}}{{2\theta\sin\theta}}.
$$

The inverse right Jacobian is the modern Lie-group expression of the classical coning correction. Non-commutative terms appear because tangent increments must be corrected by $J_r^{{-1}}$ before they are integrated.

**D1 no coning.** D1 collapses all rate increments in an update block into one vector and exponentiates once:

$$
\Delta\boldsymbol{{\theta}}_k
\approx
\frac{{\Delta t_k}}{{2}}\left(\boldsymbol{{\omega}}_k+\boldsymbol{{\omega}}_{{k+1}}\right),
\qquad
\boldsymbol{{\theta}}_B=\sum_{{k\in B}}\Delta\boldsymbol{{\theta}}_k,
\qquad
R_B^+=R_B^-\operatorname{{Exp}}(\boldsymbol{{\theta}}_B).
$$

**D2/RK2 sequential rotations.** I treated D2/RK2 as one method here, not two separate baselines: it uses the Lie-group trapezoidal update and composes the rotations sequentially:

$$
R_{{k+1}}
=R_k\operatorname{{Exp}}(\Delta\boldsymbol{{\theta}}_k),
\qquad
\Delta\boldsymbol{{\theta}}_k
\approx
\frac{{\Delta t_k}}{{2}}\left(\boldsymbol{{\omega}}_k+\boldsymbol{{\omega}}_{{k+1}}\right).
$$

**D3 single-speed coning correction.** D3 adds the adjacent-increment cross product before the sequential update:

$$
\Delta\boldsymbol{{\phi}}_k
\approx
\Delta\boldsymbol{{\theta}}_k
+\frac{{1}}{{12}}\left(\Delta\boldsymbol{{\theta}}_{{k-1}}\times\Delta\boldsymbol{{\theta}}_k\right),
\qquad
R_{{k+1}}=R_k\operatorname{{Exp}}(\Delta\boldsymbol{{\phi}}_k).
$$

**Magnus-2 endpoint update.** Magnus-2 applies a second-order SO(3) commutator correction directly on each sample interval:

$$
\boldsymbol{{\Omega}}_k^{{(2)}}
=
\frac{{\Delta t_k}}{{2}}\left(\boldsymbol{{\omega}}_k+\boldsymbol{{\omega}}_{{k+1}}\right)
+ \frac{{\Delta t_k^2}}{{12}}\left(\boldsymbol{{\omega}}_{{k+1}}\times\boldsymbol{{\omega}}_k\right),
\qquad
R_{{k+1}}
=R_k\,\operatorname{{Exp}}\!\left(\boldsymbol{{\Omega}}_k^{{(2)}}\right).
$$

D3 and Magnus-2 differ in where the commutator information comes from. D3 uses the previous and current angle increments, $\frac{{1}}{{12}}\Delta\boldsymbol{{\theta}}_{{k-1}}\times\Delta\boldsymbol{{\theta}}_k$, while Magnus-2 uses the current interval's endpoint rates, $\frac{{\Delta t_k^2}}{{12}}\boldsymbol{{\omega}}_{{k+1}}\times\boldsymbol{{\omega}}_k$; the methods agree for constant angular velocity but generally differ for time-varying gyro signals.

**PsyChMagPI.** PsyChMagPI first fits a Chebyshev-Gauss-Lobatto angular-velocity field on $[a,b]$:

$$
\widehat{{\boldsymbol{{\omega}}}}_X(t)
=
\sum_{{j=0}}^{{N-1}}\ell_j(t)\,\mathbf{{x}}_j,
\qquad
s_j=\frac{{a+b}}{{2}}+\frac{{b-a}}{{2}}\cos\!\left(\frac{{j\pi}}{{N-1}}\right).
$$

The fitted input uses first-derivative Tikhonov regularization, equivalently a Sobolev $H^1$ seminorm penalty, rather than ridge shrinkage of the CGL nodal gyro values:

$$
X^\star
=
\arg\min_X
\sum_{{k=0}}^{{m-1}}
\left\|
\sum_{{j=0}}^{{N-1}} W_{{kj}}\mathbf{{x}}_j
-\boldsymbol{{\omega}}_k
\right\|_2^2
+ \lambda_1\,\operatorname{{tr}}\!\left(X^\top Q_N X\right).
$$

The penalty matrix is the same derivative matrix used in the code:

$$
Q_N
=
\frac{{1}}{{b-a}}
D_N^\top \operatorname{{diag}}(q_N)D_N,
$$

where $D_N$ is the Chebyshev2 CGL differentiation matrix on the normalized interval and $q_N$ are the corresponding CGL integration weights. Thus $\lambda_1$ penalizes fitted gyro roughness, approximating $\int_a^b\lVert\dot{{\widehat{{\boldsymbol{{\omega}}}}}}_X(t)\rVert^2\,dt$, not the magnitude of $\widehat{{\boldsymbol{{\omega}}}}_X(t)$ itself.

PsyChMagPI then performs Picard iteration on the tangent/exponent trajectory rather than directly on rotation matrices:

$$
R(t)=R(a)\operatorname{{Exp}}\!\left(\boldsymbol{{\zeta}}(t)\right),
\qquad
\dot{{\boldsymbol{{\zeta}}}}(t)
=
J_r\!\left(\boldsymbol{{\zeta}}(t)\right)^{{-1}}
\widehat{{\boldsymbol{{\omega}}}}_{{X^\star}}(t).
$$

At iteration $i$, the nodal derivative is evaluated as

$$
\mathbf{{g}}_j^{{(i)}}
=
J_r\!\left(\boldsymbol{{\zeta}}_j^{{(i)}}\right)^{{-1}}
\widehat{{\boldsymbol{{\omega}}}}_{{X^\star}}(s_j),
\qquad
Z^{{(i+1)}} = G^{{(i)}}P,
$$

and I used the stopping rule $\left\|Z^{{(i+1)}}-Z^{{(i)}}\right\|_F < 10^{{-12}}$ with at most {PSYCHMAGPI_MAX_ITERATIONS} iterations.

I fixed the parameter policy within each comparison. For the TAES reproduction, I used the TAES draft setting $N=11$ and showed $\lambda_1=0$ plus a positive-lambda sweep; for the EuRoC endpoint sweep, I showed fixed aggregate $(N,\lambda_1)$ choices by noise band and sample count; for the EuRoC continuous-fidelity comparisons, I used deterministic $N=\lceil\sqrt{{m}}\rceil$, a noise-scaled $\lambda_1$, and a $\lambda_1=0$ ablation.
"""


def systematic_taes_experiment(taes_table2_mc: PureConingMonteCarloResult) -> str:
    table = format_taes_table_mc_wide(taes_table2_mc.summary)
    sentence = taes_table_mc_sentence(taes_table2_mc)
    return rf"""# Validation: TAES Table-II Reproduction

To validate the implementation on controlled long-integration coning, I reproduced the TAES Table-II setting from our TAES draft. {sentence}

To match the TAES Table-II metric, I reported RMS final attitude error in mrad over the draft's noise grid. The zero-noise row is deterministic, while the noisy rows used {TAES_TABLE_MONTE_CARLO_TRIALS} Monte Carlo draws.

{table}

I treated the regularized PsyChMagPI sweep as a sensitivity check rather than the main claim. I moved its selected positive $\lambda_1$ values and deltas against $\lambda_1=0$ to the appendix.
"""


def systematic_endpoint_experiment(
    algorithm_trials: pd.DataFrame,
    algorithm_summary: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    figure_paths: dict[str, Path],
) -> str:
    overall = summarize_overall_algorithms(algorithm_trials)
    psych_overall = summarize_overall_psychmagpi_vs_baselines(algorithm_trials)
    algorithm_table = format_algorithm_table(overall, psych_overall)
    band_table = format_psych_baseline_band_table(psych_vs_baseline_summary)
    best = overall.sort_values(["win_rate", "median_error_deg"], ascending=[False, True]).iloc[0]
    return f"""# Endpoint Accuracy: EuRoC Proxy Integration

To test final attitude accuracy on short aggressive 200 ms proxy windows, I ran the EuRoC endpoint comparison against RK2 and Magnus-2. PsyChMagPI was competitive but not uniformly dominant: across the four algorithm views, `{best["algorithm"]}` had the highest aggregate win rate, while RK2 and Magnus-2 remained extremely strong dense-sample endpoint baselines.

Overall endpoint summary:

{algorithm_table}

To show where PsyChMagPI was most useful, I split the results by noise band against the better RK2/Magnus-2 baseline. Low noise favored dense local integration because samples already lay close to the clean proxy; mid and high noise made input smoothing more valuable.

{band_table}

![PsyChMagPI endpoint win rate against the better RK2/Magnus-2 baseline.]({relative_asset(figure_paths["win_rates"])}){{width=100%}}

![Median final SO(3) error against the noiseless proxy truth.]({relative_asset(figure_paths["median_errors"])}){{width=100%}}

I moved the exhaustive best-algorithm and PsyChMagPI-vs-baseline tables by sample count to the appendix.
"""


def systematic_hyperparameter_experiment(
    trials: pd.DataFrame,
    summary: pd.DataFrame,
    figure_paths: dict[str, Path],
) -> str:
    overall = summarize_sqrt_lambda_vs_rk2_overall(trials)
    band_table = format_sqrt_rk2_band_table(trials)
    return rf"""# Hyperparameter Sensitivity

To quantify how much PsyChMagPI depended on $N$ and $\lambda_1$, I ran both an N/lambda sweep and a fixed `N_sqrt` comparison. I interpreted the sweep as sensitivity analysis, while the fixed `N_sqrt` comparison tested a deterministic rule.

![PsyChMagPI N/lambda sweep against the better RK2/Magnus-2 baseline.]({relative_asset(figure_paths["psych_sweep"])}){{width=100%}}

The fixed practical rule `N = ceil(sqrt(m))` with `lambda1 = {SQRT_VS_RK2_LAMBDA:g}` was almost tied with RK2 overall. Across all matched trials, PsyChMagPI won {100.0 * float(overall["win_rate_vs_rk2"]):.0f}% against RK2, with median final error {format_error(float(overall["median_psych_error_deg"]))} degrees versus RK2's {format_error(float(overall["median_rk2_error_deg"]))} degrees.

{band_table}

![Fixed N_sqrt and lambda1=0.01 against RK2 only.]({relative_asset(figure_paths["sqrt_lambda_rk2"])}){{width=100%}}

I moved the exhaustive selected-setting tables and sample-count matched RK2 table to the appendix.
"""


def systematic_interpolation_experiment(
    interpolation_result: GyroInterpolationResult,
    figure_paths: dict[str, Path],
) -> str:
    signal_table = format_interpolation_signal_table(
        interpolation_result.signal_summary
    )
    trajectory_table = format_interpolation_trajectory_table(
        interpolation_result.trajectory_summary
    )
    subinterval_table = format_interpolation_subinterval_table(
        interpolation_result.subinterval_summary
    )
    sentence = interpolation_executive_sentence(interpolation_result)
    return f"""# Continuous Fidelity: Interpolation and Trajectory Queries

To test whether PsyChMagPI is more useful as a continuous representation than as an endpoint integrator, I ran gyro interpolation, dense trajectory, and random-subinterval comparisons. {sentence}

To measure gyro-signal fidelity, I compared continuous angular-velocity estimates against the clean proxy on a {INTERPOLATION_DENSE_COUNT}-point grid. Linear and cubic spline were componentwise interpolants of the noisy samples, while I evaluated PsyChMagPI as a CGL least-squares signal fit.

![Gyro interpolation NRMSE.]({relative_asset(figure_paths["interpolation_signal"])}){{width=100%}}

{signal_table}

To measure attitude-trajectory fidelity, I integrated each continuous gyro representation and compared the full attitude path to the clean reference path. I included RK2 and Magnus-2 as piecewise-geodesic sample-update trajectories so the table answered a trajectory question rather than only a final-rotation question.

![Trajectory RMS attitude error.]({relative_asset(figure_paths["interpolation_trajectory"])}){{width=100%}}

{trajectory_table}

To measure arbitrary query fidelity, I compared relative rotations over deterministic random subintervals inside the 200 ms window. This was the closest test to the continuous-representation claim.

![Random-subinterval win rate.]({relative_asset(figure_paths["interpolation_subintervals"])}){{width=100%}}

{subinterval_table}
"""


def systematic_discussion_and_recommendation(
    algorithm_trials: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    taes_table2_mc: PureConingMonteCarloResult,
    interpolation_result: GyroInterpolationResult,
) -> str:
    psych_overall = summarize_overall_psychmagpi_vs_baselines(algorithm_trials)
    sqrt = psych_overall[psych_overall["algorithm"] == "PsyChMagPI N_sqrt"].iloc[0]
    band_sentence = psych_baseline_band_sentence(
        psych_vs_baseline_summary,
        "PsyChMagPI N_sqrt",
    )
    interpolation_sentence = interpolation_executive_sentence(interpolation_result)
    taes_sentence = taes_table_mc_sentence(taes_table2_mc)
    return rf"""# Discussion and Recommendation

I interpreted PsyChMagPI as a high-order global representation whose value depends on signal smoothness, noise, and query type. {taes_sentence}

I interpreted the EuRoC endpoint results as a stress test of smoothing, not as a contradiction of the coning benchmark. The practical `N_sqrt` rule beat the better dense-sample endpoint baseline in {100.0 * float(sqrt["win_rate_vs_baselines"]):.0f}% of matched trials with median error {format_error(float(sqrt["median_error_deg"]))} degrees; {band_sentence}.

I used the interpolation results to support a broader claim than endpoint error alone. {interpolation_sentence}

For future EuRoC-style proxy studies, I recommend deterministic $N=\lceil\sqrt{{m}}\rceil$ with an explicitly stated lambda rule and a $\lambda_1=0$ ablation. Lambda should be treated as a noise-model parameter with a reproducible rule, and future reports should make the endpoint and continuous-query claims separately.
"""


def systematic_appendix(
    metadata: pd.DataFrame,
    algorithm_summary: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    psych_setting_summary: pd.DataFrame,
    best_settings: pd.DataFrame,
    sqrt_lambda_rk2_summary: pd.DataFrame,
    taes_table2_mc: PureConingMonteCarloResult,
) -> str:
    proxy_table = format_proxy_metadata_table(metadata)
    best_by_band = format_best_algorithm_by_band_sample_table(algorithm_summary)
    psych_by_band = format_psych_vs_baseline_by_band_sample_table(
        psych_vs_baseline_summary
    )
    best_settings_table = format_best_settings_table(best_settings)
    sqrt_settings_table = format_sqrt_settings_table(psych_setting_summary)
    sqrt_rk2_table = format_sqrt_rk2_sample_table(sqrt_lambda_rk2_summary)
    taes_lambda_table = format_taes_table2_lambda_sweep(taes_table2_mc.summary)
    return f"""\\appendix

# Appendix: Exhaustive Tables and Reproducibility

The appendix collects the exhaustive grids that would interrupt the main argument.

## Proxy Metadata

{proxy_table}

## Endpoint Best Algorithm by Noise Band and Sample Count

{best_by_band}

## PsyChMagPI Views Against the Better RK2/Magnus-2 Baseline

{psych_by_band}

## PsyChMagPI Best Fixed Settings

{best_settings_table}

## PsyChMagPI N_sqrt Settings

{sqrt_settings_table}

## Fixed N_sqrt, Lambda 0.01 Against RK2 by Sample Count

{sqrt_rk2_table}

## TAES Positive-Lambda Sweep

{taes_lambda_table}
"""


def report_header(*, title: str, subtitle: str) -> str:
    return f"""---
title: "{title}"
subtitle: "{subtitle}"
author: "Frank Dellaert"
date: "2026-06-10"
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
    algorithm_trials: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    best_settings: pd.DataFrame,
    taes_table2_mc: PureConingMonteCarloResult,
    interpolation_result: GyroInterpolationResult,
    figure_paths: dict[str, Path],
) -> str:
    overall = summarize_overall_algorithms(algorithm_trials)
    psych_overall = summarize_overall_psychmagpi_vs_baselines(algorithm_trials)
    table = format_algorithm_table(overall, psych_overall)
    taes_sentence = taes_table_mc_sentence(taes_table2_mc)
    taes_table = format_taes_table_mc_wide(taes_table2_mc.summary)
    best_fixed = psych_overall[
        psych_overall["algorithm"] == "PsyChMagPI best fixed"
    ].iloc[0]
    sqrt = psych_overall[psych_overall["algorithm"] == "PsyChMagPI N_sqrt"].iloc[0]
    band_sentence = psych_baseline_band_sentence(
        psych_vs_baseline_summary,
        "PsyChMagPI N_sqrt",
    )
    interpolation_sentence = interpolation_executive_sentence(interpolation_result)
    bullets = executive_conclusion_bullets(
        best_fixed=best_fixed,
        sqrt=sqrt,
        band_sentence=band_sentence,
        interpolation_sentence=interpolation_sentence,
        taes_table2_mc=taes_table2_mc,
    )
    return f"""# Executive Summary

Main conclusions:

{bullets}

I first reproduced the Table II long-integration Monte Carlo from our TAES draft. {taes_sentence} The low-noise rows showed the expected high-order advantage of tangent-space Picard integration and pseudospectral angular-velocity fitting; the high-noise rows showed the expected collapse toward the classical coning-correction baselines.

Table II long-integration Monte Carlo reproduction, including a first-derivative Tikhonov sweep:

{taes_table}

I then ran a harder EuRoC proxy Monte Carlo where performance depended strongly on noise. With fixed PsyChMagPI parameters, the best fixed PsyChMagPI setting beat the better of RK2 and Magnus-2 in {100.0 * float(best_fixed["win_rate_vs_baselines"]):.0f}% of matched trials with median final attitude error {format_error(float(best_fixed["median_error_deg"]))} degrees, while the practical `N_sqrt = ceil(sqrt(m))` rule beat the better dense-sample baseline in {100.0 * float(sqrt["win_rate_vs_baselines"]):.0f}% of matched trials; {band_sentence}.

I used only fixed PsyChMagPI `lambda1` values chosen from `{list(PSYCHMAGPI_LAMBDA_VALUES)}` at the noise-band/sample-count level. The `best fixed` view used the exact `(N, lambda1)` pair listed in Section 4 for that band and `m`; the `N_sqrt` view used `N = ceil(sqrt(m))` and the exact `lambda1` listed in Section 4 for that band and `m`. RK2 and Magnus-2 used no `lambda1`.

To keep continuity with the earlier Chebyshev quadrature report, I used the EuRoC IMU stream to select aggressive windows and construct smooth proxy signals, then measured accuracy against the clean proxy rather than the propeller-contaminated samples themselves. I converted each selected aggressive 1 s window into a smooth 3-axis Chebyshev2 `N=50` proxy, sampled only the center 200 ms interval, added controlled iid component noise, and compared final SO(3) error against a high-accuracy noiseless integration of that clean proxy.

![Monte Carlo win rate by noise band and sample count.]({relative_asset(figure_paths["win_rates"])}){{width=100%}}

![Median final SO(3) error by noise band and sample count.]({relative_asset(figure_paths["median_errors"])}){{width=100%}}

The RK2 and Magnus-2 median-error curves are nearly coincident in the second executive figure. Magnus-2 is drawn as a dashed/open-marker overlay on a thicker solid RK2 trace so both baseline entries in the legend correspond to visible ink.

To separate continuous signal quality from final endpoint accuracy, I ran interpolation and trajectory-fidelity comparisons. {interpolation_sentence}

![Gyro interpolation error by noise band and sample count.]({relative_asset(figure_paths["interpolation_signal"])}){{width=100%}}

![Attitude trajectory RMS error by noise band and sample count.]({relative_asset(figure_paths["interpolation_trajectory"])}){{width=100%}}

![Random-subinterval rotation win rate by duration and noise band.]({relative_asset(figure_paths["interpolation_subintervals"])}){{width=100%}}

Overall algorithm summary:

{table}
"""


def executive_conclusion_bullets(
    *,
    best_fixed: pd.Series,
    sqrt: pd.Series,
    band_sentence: str,
    interpolation_sentence: str,
    taes_table2_mc: PureConingMonteCarloResult,
) -> str:
    taes_summary = taes_table_mc_wide_frame(taes_table2_mc.summary)
    classical_columns = [
        "D1 no coning",
        "D2 sequential rotations",
        "D3 single-speed",
    ]
    zero_noise = taes_summary[np.isclose(taes_summary["sigma_rad_s"], 0.0)].iloc[0]
    high_noise = taes_summary.sort_values("sigma_rad_s").iloc[-1]
    zero_psych = float(zero_noise["PsyChMagPI lambda1=0"])
    zero_classical = float(
        zero_noise[classical_columns].min()
    )
    high_sigma = float(high_noise["sigma_rad_s"])
    high_psych = min(
        float(high_noise["PsyChMagPI lambda1=0"]),
        float(high_noise["PsyChMagPI best lambda1>0"]),
    )
    high_classical = float(
        high_noise[classical_columns].min()
    )
    return "\n".join(
        [
            (
                f"- On the TAES Table-II pure-coning benchmark, PsyChMagPI is the clear "
                f"low-noise winner: `lambda1=0` gives {zero_psych:.3g} mrad at "
                f"zero noise versus {zero_classical:.4g} mrad for the best classical "
                f"method."
            ),
            (
                f"- At the largest Table-II noise level, $\\sigma={high_sigma:g}$ rad/s, "
                f"the advantage mostly disappears: the best PsyChMagPI entry is "
                f"{high_psych:.4g} mrad versus {high_classical:.4g} mrad for the best "
                f"classical method."
            ),
            (
                f"- On the aggressive EuRoC proxy endpoint task, PsyChMagPI was competitive "
                f"rather than dominant: the best fixed PsyChMagPI setting beat the better "
                f"RK2/Magnus-2 baseline in {100.0 * float(best_fixed['win_rate_vs_baselines']):.0f}% "
                f"of matched trials, and `N_sqrt = ceil(sqrt(m))` won "
                f"{100.0 * float(sqrt['win_rate_vs_baselines']):.0f}% overall; {band_sentence}."
            ),
            (
                "- Hyperparameter selection matters. I used fixed `lambda1` values "
                "chosen only at the noise-band/sample-count level; RK2 and Magnus-2 have no "
                "`lambda1`, and the deterministic practical rule is `N_sqrt = ceil(sqrt(m))`."
            ),
            (
                "- PsyChMagPI makes its strongest EuRoC case as a continuous representation "
                f"of the gyro signal and attitude trajectory. {interpolation_sentence}"
            ),
        ]
    )


def data_and_methods_section(
    metadata: pd.DataFrame,
    figure_paths: dict[str, Path],
    *,
    integration_matrix_label: str,
    use_approximate_integration_matrix: bool,
) -> str:
    compact = metadata.copy()
    for column in [
        "fit_relative_error",
        "noise_scale_rad_s",
        "aggressive_score",
        "center_activity",
        "center_high_order_ratio",
        "center_normalized_rmse",
    ]:
        compact[column] = compact[column].map(lambda value: f"{float(value):.4g}")
    table = markdown_table(
        compact[
            [
                "dataset",
                "window_index",
                "window_start_s",
                "middle_interval_s",
                "reference_N",
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
                "reference_N": "Proxy N",
                "window_samples": "Raw samples",
                "fit_relative_error": "Fit rel. err",
                "noise_scale_rad_s": "Noise scale",
                "aggressive_score": "Score",
            }
        )
    )
    if use_approximate_integration_matrix:
        integration_matrix_sentence = (
            "For this report variant, I set "
            "`useApproximateIntegrationMatrix = true`, so PsyChMagPI used the "
            "historical Chebyshev2 approximate integration matrix and returned "
            "a tangent trajectory with `N` columns."
        )
    else:
        integration_matrix_sentence = (
            "For this report variant, PsyChMagPI used the default exact "
            "`N -> N+1` integration matrix and returned a tangent trajectory "
            "with `N+1` columns."
        )
    return rf"""# Data and Methods

I used the same aggressive EuRoC windows selected for the earlier Chebyshev quadrature report. Selection was based on the center $[0.4,0.6]$ interval of each 1 s $\lVert\boldsymbol{{\omega}}\rVert$ window, but here I fitted the full 3-axis bias-corrected gyro vector. The 1 s proxy was a Chebyshev2 $N={REFERENCE_N}$ least-squares fit to $\boldsymbol{{\omega}}-\mathbf{{b}}_\omega$; all accuracy comparisons used only the center 200 ms interval.

To define the Monte Carlo grid, I used $m\in{list(SAMPLE_COUNTS)}$ closed-interval samples, the Chebyshev-report noise-fraction grid, and $n_\mathrm{{seed}}={NUM_SEEDS}$ deterministic noise draws per snippet and sample count. I drew iid noise across components and samples with $\sigma=\alpha\,\operatorname{{range}}(\lVert\boldsymbol{{\omega}}^\star(t)\rVert)$ over the 200 ms interval. I computed the reference rotation from the noiseless proxy by solving $\dot R=R\,(\boldsymbol{{\omega}}^\star(t))^\wedge$ with tight tolerances.

I controlled PsyChMagPI regularization only through $\lambda_1\in{list(PSYCHMAGPI_LAMBDA_VALUES)}$. The algorithm-level PsyChMagPI best-fixed rows used the fixed $(N,\lambda_1)$ pair selected for each noise-band/$m$ aggregate and shown in the first Section 4 table. The algorithm-level PsyChMagPI $N_{{\sqrt{{m}}}}$ rows used $N_{{\sqrt{{m}}}}=\lceil\sqrt{{m}}\rceil$ and the fixed $\lambda_1$ shown in the second Section 4 table.

I fed identical noisy sample sets to every method, so differences reflected integration and input-fit choices rather than input data. Let $\boldsymbol{{\omega}}^\star(t)$ denote the clean proxy gyro signal. For one Monte Carlo trial, let $a=0.4$, $b=0.6$, $t_0=a$, $t_{{m-1}}=b$, $\Delta t_k=t_{{k+1}}-t_k$, and write the sampled gyro values as

$$
\boldsymbol{{\omega}}_k = \boldsymbol{{\omega}}^\star(t_k) + \boldsymbol{{\epsilon}}_k,
\qquad
\boldsymbol{{\epsilon}}_k \sim \mathcal{{N}}(\mathbf{{0}}, \sigma^2 I_3).
$$

From this point on, $\boldsymbol{{\omega}}_k$ denotes the sampled gyro values consumed by RK2, Magnus-2, and PsyChMagPI. The initial rotation is $R_0=I$, each estimate lies in $\mathrm{{SO}}(3)$, and $\operatorname{{Exp}}:\mathbb{{R}}^3\rightarrow\mathrm{{SO}}(3)$ denotes the Lie-group exponential.

The Bortz equation is the Lie-algebra exponent ODE written with the inverse right Jacobian. With the body-rate convention $\dot R=R\widehat{{\boldsymbol{{\omega}}}}$ and local coordinates $R(t)=R(a)\operatorname{{Exp}}(\boldsymbol{{\phi}}(t))$,

$$
\dot{{\boldsymbol{{\phi}}}}
= J_r(\boldsymbol{{\phi}})^{{-1}}\boldsymbol{{\omega}}
= \boldsymbol{{\omega}}
+\frac{{1}}{{2}}\boldsymbol{{\phi}}\times\boldsymbol{{\omega}}
+c(\lVert\boldsymbol{{\phi}}\rVert)\,
\boldsymbol{{\phi}}\times\left(\boldsymbol{{\phi}}\times\boldsymbol{{\omega}}\right),
$$

where

$$
c(\theta)=\frac{{1}}{{\theta^2}}-\frac{{1+\cos\theta}}{{2\theta\sin\theta}}.
$$

This identity is the connection between the classical Bortz form and the modern Lie-group Jacobian correction: coning terms appear because the tangent increment must be corrected by $J_r^{{-1}}$ before it is integrated.

**D1 no coning.** The simplest classical comparator collapses all rate increments in an update block into one vector and exponentiates once,

$$
\Delta\boldsymbol{{\theta}}_k
\approx
\frac{{\Delta t_k}}{{2}}\left(\boldsymbol{{\omega}}_k+\boldsymbol{{\omega}}_{{k+1}}\right),
\qquad
\boldsymbol{{\theta}}_B=\sum_{{k\in B}}\Delta\boldsymbol{{\theta}}_k,
\qquad
R_B^+=R_B^-\operatorname{{Exp}}(\boldsymbol{{\theta}}_B).
$$

D1 is useful as a lower baseline because it deliberately ignores non-commutativity inside the block.

**D2/RK2 sequential rotations.** I treated D2/RK2 as one method here, not two separate baselines: it composes the per-sample Lie-group trapezoidal increments instead of summing them first,

$$
R_{{k+1}}
=R_k\operatorname{{Exp}}(\Delta\boldsymbol{{\theta}}_k),
\qquad
\Delta\boldsymbol{{\theta}}_k
\approx
\frac{{\Delta t_k}}{{2}}\left(\boldsymbol{{\omega}}_k+\boldsymbol{{\omega}}_{{k+1}}\right).
$$

**D3 single-speed coning correction.** The single-speed correction adds the first adjacent-increment cross product before the sequential rotation update,

$$
\Delta\boldsymbol{{\phi}}_k
\approx
\Delta\boldsymbol{{\theta}}_k
+\frac{{1}}{{12}}\left(\Delta\boldsymbol{{\theta}}_{{k-1}}\times\Delta\boldsymbol{{\theta}}_k\right),
\qquad
R_{{k+1}}=R_k\operatorname{{Exp}}(\Delta\boldsymbol{{\phi}}_k).
$$

D3 is the most accurate classical coning correction included in the Table II reproduction; the two-speed correction is omitted here.

**Magnus-2 endpoint update.** For the EuRoC proxy endpoint baseline, I applied the second-order SO(3) commutator correction directly to each sample interval,

$$
\boldsymbol{{\Omega}}_k^{{(2)}}
=
\frac{{\Delta t_k}}{{2}}\left(\boldsymbol{{\omega}}_k+\boldsymbol{{\omega}}_{{k+1}}\right)
+ \frac{{\Delta t_k^2}}{{12}}\left(\boldsymbol{{\omega}}_{{k+1}}\times\boldsymbol{{\omega}}_k\right),
\qquad
R_{{k+1}}
=R_k\,\operatorname{{Exp}}\!\left(\boldsymbol{{\Omega}}_k^{{(2)}}\right).
$$

D3 and Magnus-2 differ in where the commutator information comes from. D3 uses the previous and current angle increments, $\frac{{1}}{{12}}\Delta\boldsymbol{{\theta}}_{{k-1}}\times\Delta\boldsymbol{{\theta}}_k$, while Magnus-2 uses the current interval's endpoint rates, $\frac{{\Delta t_k^2}}{{12}}\boldsymbol{{\omega}}_{{k+1}}\times\boldsymbol{{\omega}}_k$; the methods agree for constant angular velocity but generally differ for time-varying gyro signals.

**PsyChMagPI.** The method first fits a Chebyshev-Gauss-Lobatto angular-velocity field on $[a,b]$. For $N$ CGL nodes

$$
s_j=\frac{{a+b}}{{2}}+\frac{{b-a}}{{2}}\cos\!\left(\frac{{j\pi}}{{N-1}}\right),
\qquad j=0,\ldots,N-1,
$$

let $\ell_j(t)$ be the corresponding Lagrange basis and let $X=[\mathbf{{x}}_0,\ldots,\mathbf{{x}}_{{N-1}}]^\top\in\mathbb{{R}}^{{N\times 3}}$ store the fitted nodal gyro values:

$$
\widehat{{\boldsymbol{{\omega}}}}_X(t)
=
\sum_{{j=0}}^{{N-1}}\ell_j(t)\,\mathbf{{x}}_j.
$$

The fitted input uses first-derivative Tikhonov regularization, equivalently a Sobolev $H^1$ seminorm penalty, rather than ridge shrinkage of the CGL nodal gyro values:

$$
X^\star
=
\arg\min_X
\sum_{{k=0}}^{{m-1}}
\left\|
\sum_{{j=0}}^{{N-1}} W_{{kj}}\mathbf{{x}}_j
-\boldsymbol{{\omega}}_k
\right\|_2^2
+ \lambda_1\,\operatorname{{tr}}\!\left(X^\top Q_N X\right),
$$

where $W_{{kj}}=\ell_j(t_k)$. The penalty matrix is the same derivative matrix used in the code:

$$
Q_N
=
\frac{{1}}{{b-a}}
D_N^\top \operatorname{{diag}}(q_N)D_N,
$$

where $D_N$ is the Chebyshev2 CGL differentiation matrix on the normalized interval and $q_N$ are the corresponding CGL integration weights. Thus $\lambda_1$ penalizes fitted gyro roughness, approximating $\int_a^b \lVert\dot{{\widehat{{\boldsymbol{{\omega}}}}}}_X(t)\rVert^2\,dt$, not the magnitude of $\widehat{{\boldsymbol{{\omega}}}}_X(t)$ itself.

PsyChMagPI performs Picard iteration on the tangent/exponent trajectory, not directly on rotation matrices. It represents the attitude path as

$$
R(t)=R(a)\operatorname{{Exp}}\!\left(\boldsymbol{{\zeta}}(t)\right),
\qquad
\boldsymbol{{\zeta}}(a)=\mathbf{{0}},
$$

and solves the right-trivialized exponent ODE

$$
\dot{{\boldsymbol{{\zeta}}}}(t)
=
J_r\!\left(\boldsymbol{{\zeta}}(t)\right)^{{-1}}
\widehat{{\boldsymbol{{\omega}}}}_{{X^\star}}(t),
$$

where $J_r$ is the $\mathrm{{SO}}(3)$ right Jacobian. At Picard iteration $i$, the current exponent trajectory $Z^{{(i)}}$ is interpolated to the CGL input nodes, the node derivatives are evaluated as

$$
\mathbf{{g}}_j^{{(i)}}
=
J_r\!\left(\boldsymbol{{\zeta}}_j^{{(i)}}\right)^{{-1}}
\widehat{{\boldsymbol{{\omega}}}}_{{X^\star}}(s_j),
$$

and Chebyshev integration updates the nodal exponent trajectory,

$$
Z^{{(i+1)}} = G^{{(i)}}P,
\qquad
\left\|Z^{{(i+1)}}-Z^{{(i)}}\right\|_F < 10^{{-12}}.
$$

The final rotation is $R(b)=R(a)\operatorname{{Exp}}(\boldsymbol{{\zeta}}(b))$. I used $N=2,\ldots,10$, $\lambda_1\in{list(PSYCHMAGPI_LAMBDA_VALUES)}$, RK2 initialization, and $I_{{\max}}={PSYCHMAGPI_MAX_ITERATIONS}$; I presented only the fixed settings listed in Section 4.

The PsyChMagPI integration-matrix setting in this report is **{integration_matrix_label}**. {integration_matrix_sentence} In both modes I took the final tangent state from the last returned trajectory column before applying $\operatorname{{Exp}}$.

I included the proxy preview to show that the clean signal was a smooth 1 s surrogate fitted through noisy 200 Hz EuRoC samples, while method comparisons used separate Monte Carlo samples from its center interval. It did not show RK2-specific samples: RK2, Magnus-2, and PsyChMagPI received the same Monte Carlo sample values for each $(m,\mathrm{{seed}},\alpha)$ trial.

![Raw EuRoC sample dots and smooth vector proxies.]({relative_asset(figure_paths["proxy_previews"])}){{width=100%}}

Proxy metadata:

{table}
"""


def taes_table2_section(
    taes_table2_mc: PureConingMonteCarloResult,
    *,
    asset_dir: Path,
) -> str:
    mc_table = format_taes_table_mc_wide(taes_table2_mc.summary)
    lambda_table = format_taes_table2_lambda_sweep(taes_table2_mc.summary)
    return rf"""# TAES Table II Reproduction

To reproduce the Table II long-integration result from our TAES draft, I used the methods that overlapped with the EuRoC proxy comparison. The signal was the Eq. 72 rotation-vector trajectory with frequency $f=7$ Hz, sampled as rate-gyro angular velocity at $f_s=200$ Hz over $T=0.1$ s, giving $M=21$ samples and $N=11$ CGL nodes.

To match the Table II setup, I used the draft's rate-gyro noise grid $\sigma\in\{{0,10^{{-4}},0.025,0.075,0.125\}}$ rad/s and reported RMS final attitude error in mrad over {TAES_TABLE_MONTE_CARLO_TRIALS} Monte Carlo draws. The zero-noise row is deterministic; repeating it would not change the RMS value. I intentionally omitted two-speed to keep the comparison aligned with the methods I used for EuRoC.

{mc_table}

I showed PsyChMagPI twice: once with no first-derivative Tikhonov regularization, $\lambda_1=0$, and once with the positive-lambda aggregate sweep winner at each noise level from $\lambda_1\in{list(PSYCHMAGPI_LAMBDA_VALUES)}$. The aggregate regularized settings were:

{lambda_table}

I wrote the Table II sweep assets to `{relative_asset(asset_dir / "taes_table2_long_integration_sweep_trials.csv.gz")}` and `{relative_asset(asset_dir / "taes_table2_long_integration_sweep_summary.csv")}`.
"""

def monte_carlo_results_section(
    algorithm_summary: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    figure_paths: dict[str, Path],
    *,
    asset_dir: Path,
) -> str:
    best_by_band = (
        algorithm_summary.sort_values(
            ["noise_band", "m", "win_rate", "median_error_deg"],
            ascending=[True, True, False, True],
        )
        .groupby(["noise_band", "m"], observed=True)
        .head(1)
        .copy()
    )
    best_by_band["win_rate"] = best_by_band["win_rate"].map(lambda value: f"{100.0 * float(value):.0f}%")
    best_by_band["median_error_deg"] = best_by_band["median_error_deg"].map(format_error)
    table = markdown_table(
        best_by_band[
            ["noise_band", "m", "algorithm", "win_rate", "median_error_deg"]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "algorithm": "Best algorithm",
                "win_rate": "Win rate",
                "median_error_deg": "Median err. deg",
            }
        )
    )
    psych_display = psych_vs_baseline_summary.copy()
    psych_display["win_rate_vs_baselines"] = psych_display[
        "win_rate_vs_baselines"
    ].map(lambda value: f"{100.0 * float(value):.0f}%")
    psych_display["median_error_deg"] = psych_display["median_error_deg"].map(
        format_error
    )
    psych_table = markdown_table(
        psych_display[
            ["noise_band", "m", "algorithm", "win_rate_vs_baselines", "median_error_deg"]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "algorithm": "PsyChMagPI view",
                "win_rate_vs_baselines": "Win vs baselines",
                "median_error_deg": "Median err. deg",
            }
        )
    )
    return f"""# Monte Carlo SO(3) Results

PsyChMagPI gained most over dense local baselines when noise made smoothing useful. Low noise exposed approximation bias because dense local baselines remained competitive when samples were already close to the smooth proxy; as noise increased, the global CGL fit and first-derivative Tikhonov options traded local sample noise for a smoother angular-velocity input before SO(3) integration.

Best four-way algorithm by noise band and sample count:

{table}

PsyChMagPI views against the better RK2/Magnus-2 baseline:

{psych_table}

I wrote the full trial rows to `{relative_asset(asset_dir / "method_trial_metrics.csv.gz")}`. I also wrote the smaller collapsed algorithm rows to `{relative_asset(asset_dir / "algorithm_trial_metrics.csv")}` and the PsyChMagPI-vs-baseline summary to `{relative_asset(asset_dir / "psychmagpi_vs_baseline_summary.csv")}`.
"""


def psychmagpi_sweep_section(
    setting_summary: pd.DataFrame,
    best_settings: pd.DataFrame,
    figure_paths: dict[str, Path],
) -> str:
    display = best_settings.copy()
    display["N"] = display["N"].map(lambda value: int(value))
    display["lambda1"] = display["lambda1"].map(lambda value: f"{float(value):g}")
    display["win_rate_vs_baselines"] = display["win_rate_vs_baselines"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    display["median_error_deg"] = display["median_error_deg"].map(format_error)
    table = markdown_table(
        display[
            ["noise_band", "m", "N", "lambda1", "win_rate_vs_baselines", "median_error_deg"]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "N": "Best N",
                "lambda1": "Best lambda1",
                "win_rate_vs_baselines": "Win vs baselines",
                "median_error_deg": "Median err. deg",
            }
        )
    )
    sqrt_rows = setting_summary[
        np.isclose(setting_summary["N"], np.ceil(np.sqrt(setting_summary["m"])))
    ]
    sqrt_best = best_psychmagpi_settings_by_band(sqrt_rows)
    sqrt_best["lambda1"] = sqrt_best["lambda1"].map(lambda value: f"{float(value):g}")
    sqrt_best["N"] = sqrt_best["N"].map(lambda value: int(value))
    sqrt_best["win_rate_vs_baselines"] = sqrt_best["win_rate_vs_baselines"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    sqrt_best["median_error_deg"] = sqrt_best["median_error_deg"].map(format_error)
    sqrt_table = markdown_table(
        sqrt_best[
            ["noise_band", "m", "N", "lambda1", "win_rate_vs_baselines", "median_error_deg"]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "N": "N_sqrt",
                "lambda1": "Best lambda1",
                "win_rate_vs_baselines": "Win vs baselines",
                "median_error_deg": "Median err. deg",
            }
        )
    )
    return rf"""# PsyChMagPI N/Lambda Sweep

To isolate how PsyChMagPI's least-squares input fit traded approximation bias against noise suppression, I swept $N$ and $\lambda_1$. The y-axis is win rate against the better of RK2 and Magnus-2 for the same noisy trial, aggregated over snippets and sample counts within each noise band.

![PsyChMagPI N/lambda sweep.]({relative_asset(figure_paths["psych_sweep"])}){{width=100%}}

Fixed `(N, lambda1)` used by the `PsyChMagPI best fixed` rows:

{table}

Fixed `lambda1` used by the practical `N_sqrt = ceil(sqrt(m))` rows:

{sqrt_table}
"""


def sqrt_lambda_vs_rk2_section(
    trials: pd.DataFrame,
    summary: pd.DataFrame,
    figure_paths: dict[str, Path],
    *,
    asset_dir: Path,
) -> str:
    overall = summarize_sqrt_lambda_vs_rk2_overall(trials)
    by_band = (
        trials.groupby("noise_band", observed=True)
        .agg(
            trials=("psych_win_vs_rk2", "size"),
            win_rate_vs_rk2=("psych_win_vs_rk2", "mean"),
            median_psych_error_deg=("psych_error_deg", "median"),
            median_rk2_error_deg=("rk2_error_deg", "median"),
        )
        .reset_index()
        .sort_values("noise_band")
    )
    display = summary.copy()
    for column in ["win_rate_vs_rk2"]:
        display[column] = display[column].map(lambda value: f"{100.0 * float(value):.0f}%")
    for column in ["median_psych_error_deg", "median_rk2_error_deg", "median_delta_deg"]:
        display[column] = display[column].map(format_error)
    table = markdown_table(
        display[
            [
                "noise_band",
                "m",
                "trials",
                "N_sqrt",
                "lambda1",
                "win_rate_vs_rk2",
                "median_psych_error_deg",
                "median_rk2_error_deg",
                "median_delta_deg",
            ]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "trials": "Trials",
                "N_sqrt": "N_sqrt",
                "lambda1": "lambda1",
                "win_rate_vs_rk2": "Win vs RK2",
                "median_psych_error_deg": "PsyCh median",
                "median_rk2_error_deg": "RK2 median",
                "median_delta_deg": "PsyCh-RK2",
            }
        )
    )
    band_display = by_band.copy()
    band_display["win_rate_vs_rk2"] = band_display["win_rate_vs_rk2"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    band_display["median_psych_error_deg"] = band_display[
        "median_psych_error_deg"
    ].map(format_error)
    band_display["median_rk2_error_deg"] = band_display["median_rk2_error_deg"].map(
        format_error
    )
    band_table = markdown_table(
        band_display[
            [
                "noise_band",
                "trials",
                "win_rate_vs_rk2",
                "median_psych_error_deg",
                "median_rk2_error_deg",
            ]
        ].rename(
            columns={
                "noise_band": "Noise",
                "trials": "Trials",
                "win_rate_vs_rk2": "Win vs RK2",
                "median_psych_error_deg": "PsyCh median",
                "median_rk2_error_deg": "RK2 median",
            }
        )
    )
    return f"""# Fixed N_sqrt, Lambda 0.01 Versus RK2

With `N = ceil(sqrt(m))` and `lambda1 = {SQRT_VS_RK2_LAMBDA:g}`, PsyChMagPI was almost tied with RK2 overall and improved mainly in the mid/high-noise bands. Across all matched trials, PsyChMagPI won {100.0 * float(overall["win_rate_vs_rk2"]):.0f}% against RK2, with median final error {format_error(float(overall["median_psych_error_deg"]))} degrees versus RK2's {format_error(float(overall["median_rk2_error_deg"]))} degrees.

To isolate a deterministic practical rule, I fixed `lambda1 = {SQRT_VS_RK2_LAMBDA:g}` for every noise band and sample count. The only PsyChMagPI parameter that changed with sample count was `N_sqrt = ceil(sqrt(m))`; RK2 used the same noisy samples and had no `lambda1`.

![Win rate for fixed N_sqrt and lambda1=0.01 against RK2 only.]({relative_asset(figure_paths["sqrt_lambda_rk2"])}){{width=100%}}

Band-level summary:

{band_table}

Matched-trial summary by noise band and sample count:

The `PsyCh-RK2` column is the median matched-trial error difference in degrees; negative values favor PsyChMagPI.

{table}

I wrote the matched trial rows to `{relative_asset(asset_dir / "psychmagpi_sqrt_lambda001_vs_rk2_trials.csv")}` and the grouped summary to `{relative_asset(asset_dir / "psychmagpi_sqrt_lambda001_vs_rk2_summary.csv")}`.
"""


def interpolation_fidelity_section(
    interpolation_result: GyroInterpolationResult,
    figure_paths: dict[str, Path],
    *,
    asset_dir: Path,
) -> str:
    signal_table = format_interpolation_signal_table(
        interpolation_result.signal_summary
    )
    trajectory_table = format_interpolation_trajectory_table(
        interpolation_result.trajectory_summary
    )
    subinterval_table = format_interpolation_subinterval_table(
        interpolation_result.subinterval_summary
    )
    return f"""# Interpolation and Trajectory Fidelity

To test whether PsyChMagPI gave a better continuous representation of the clean EuRoC proxy, I evaluated interpolation and trajectory fidelity independently of endpoint attitude error. I used the same noisy samples as the endpoint Monte Carlo with deterministic PsyChMagPI parameters: `N = ceil(sqrt(m))` and a noise-scaled first-derivative Tikhonov value with `c_lambda = {INTERPOLATION_C_LAMBDA:g}`. The `PsyChMagPI lambda0` row kept the same `N` and disabled first-derivative Tikhonov regularization as an ablation.

To measure gyro interpolation fidelity, I compared continuous angular-velocity estimates against the clean proxy on a {INTERPOLATION_DENSE_COUNT}-point grid. Linear and cubic spline were componentwise interpolants of the noisy samples; the PsyChMagPI rows were CGL least-squares fits evaluated on the same dense grid.

![Gyro interpolation NRMSE.]({relative_asset(figure_paths["interpolation_signal"])}){{width=100%}}

Gyro interpolation summary by noise band:

{signal_table}

To measure trajectory fidelity, I integrated each continuous gyro representation over the dense grid and compared the full attitude path to the clean reference path. I included RK2 and Magnus-2 as piecewise-geodesic sample-update trajectories, so this table asked a trajectory question rather than only a final-rotation question.

![Trajectory RMS attitude error.]({relative_asset(figure_paths["interpolation_trajectory"])}){{width=100%}}

Trajectory RMS summary by noise band:

{trajectory_table}

To measure arbitrary subinterval fidelity, I compared relative rotations over deterministic random subintervals in the 200 ms window. This directly tested whether a method provided a useful continuous attitude representation for arbitrary query intervals.

![Random-subinterval win rate.]({relative_asset(figure_paths["interpolation_subintervals"])}){{width=100%}}

Random-subinterval summary by noise band and duration:

{subinterval_table}

I wrote the interpolation metric rows and summaries under `{relative_asset(asset_dir)}`.
"""


def conclusion_section(
    algorithm_trials: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
    best_settings: pd.DataFrame,
    taes_table2_mc: PureConingMonteCarloResult,
    interpolation_result: GyroInterpolationResult,
) -> str:
    overall = summarize_overall_algorithms(algorithm_trials)
    best = overall.sort_values(["win_rate", "median_error_deg"], ascending=[False, True]).iloc[0]
    psych_overall = summarize_overall_psychmagpi_vs_baselines(algorithm_trials)
    best_fixed = psych_overall[
        psych_overall["algorithm"] == "PsyChMagPI best fixed"
    ].iloc[0]
    sqrt = psych_overall[psych_overall["algorithm"] == "PsyChMagPI N_sqrt"].iloc[0]
    sqrt_band_sentence = psych_baseline_band_sentence(
        psych_vs_baseline_summary,
        "PsyChMagPI N_sqrt",
    )
    taes_sentence = taes_table_mc_sentence(taes_table2_mc)
    interpolation_sentence = interpolation_executive_sentence(interpolation_result)
    return f"""# Conclusion

I validated PsyChMagPI most strongly in the low-noise Table II reproduction from our TAES draft. {taes_sentence} The regularized sweep was useful mainly at noisy settings where the best positive `lambda1` reduced sample noise, while `lambda1=0` preserved the zero- and low-noise accuracy.

I treated the EuRoC proxy Monte Carlo as a tougher noise-smoothing benchmark, not as a contradiction of the Table II result. With fixed `lambda1` choices from the Section 4 tables, PsyChMagPI was competitive but not dominant on the smooth aggressive proxy benchmark. Against that proxy, `{best["algorithm"]}` had the best four-way aggregate win rate. The best fixed PsyChMagPI setting beat the better RK2/Magnus-2 baseline in {100.0 * float(best_fixed["win_rate_vs_baselines"]):.0f}% of matched trials with median error {format_error(float(best_fixed["median_error_deg"]))} degrees. The practical `N_sqrt` PsyChMagPI rule beat the better dense-sample baseline in {100.0 * float(sqrt["win_rate_vs_baselines"]):.0f}% of matched trials with median error {format_error(float(sqrt["median_error_deg"]))} degrees; {sqrt_band_sentence}.

I used the interpolation comparisons to make the continuous-representation claim explicit. {interpolation_sentence} The fair interpretation is that PsyChMagPI should be judged both as an endpoint integrator and as a smooth global SO(3) representation whose deterministic N/lambda fit can suppress sample noise before answering dense trajectory or arbitrary subinterval queries.
"""


def format_systematic_decision_table(
    *,
    best_fixed: pd.Series,
    sqrt: pd.Series,
    taes_table2_mc: PureConingMonteCarloResult,
    interpolation_result: GyroInterpolationResult,
) -> str:
    wide = taes_table_mc_wide_frame(taes_table2_mc.summary)
    classical_columns = [
        "D1 no coning",
        "D2 sequential rotations",
        "D3 single-speed",
    ]
    zero = wide[np.isclose(wide["sigma_rad_s"], 0.0)].iloc[0]
    high = wide.sort_values("sigma_rad_s").iloc[-1]
    signal = aggregate_interpolation_signal_by_method(
        interpolation_result.signal_summary
    )
    trajectory = aggregate_interpolation_trajectory_by_method(
        interpolation_result.trajectory_summary
    )
    subinterval = aggregate_interpolation_subinterval_by_method(
        interpolation_result.subinterval_summary
    )
    signal_best = signal.sort_values("median_gyro_nrmse").iloc[0]
    trajectory_best = trajectory.sort_values("median_trajectory_rms_error_deg").iloc[0]
    sub_best = subinterval.sort_values("win_rate", ascending=False).iloc[0]
    rows = [
        {
            "Question": "TAES low-noise coning",
            "Finding": (
                f"PsyChMagPI lambda1=0: {format_error(float(zero['PsyChMagPI lambda1=0']))} "
                f"mrad vs best classical {format_error(float(zero[classical_columns].min()))} mrad"
            ),
            "Implication": "Implementation validates the high-order method in the clean regime",
        },
        {
            "Question": "TAES high-noise coning",
            "Finding": (
                f"Best PsyChMagPI {format_error(min(float(high['PsyChMagPI lambda1=0']), float(high['PsyChMagPI best lambda1>0'])))} "
                f"mrad vs best classical {format_error(float(high[classical_columns].min()))} mrad"
            ),
            "Implication": "Large rate noise erases most high-order endpoint advantage",
        },
        {
            "Question": "EuRoC endpoint",
            "Finding": (
                f"Best fixed won {100.0 * float(best_fixed['win_rate_vs_baselines']):.0f}%; "
                f"N_sqrt won {100.0 * float(sqrt['win_rate_vs_baselines']):.0f}%"
            ),
            "Implication": "PsyChMagPI is competitive, not dominant, for final attitude only",
        },
        {
            "Question": "Continuous EuRoC queries",
            "Finding": (
                f"Best signal: {signal_best['method']}; best trajectory: "
                f"{trajectory_best['method']}; best subinterval: {sub_best['method']}"
            ),
            "Implication": "The strongest EuRoC claim is representation fidelity",
        },
    ]
    return markdown_table(pd.DataFrame(rows))


def format_proxy_metadata_table(metadata: pd.DataFrame) -> str:
    compact = metadata.copy()
    for column in [
        "fit_relative_error",
        "noise_scale_rad_s",
        "aggressive_score",
        "center_activity",
        "center_high_order_ratio",
        "center_normalized_rmse",
    ]:
        compact[column] = compact[column].map(lambda value: f"{float(value):.4g}")
    return markdown_table(
        compact[
            [
                "dataset",
                "window_index",
                "window_start_s",
                "middle_interval_s",
                "reference_N",
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
                "reference_N": "Proxy N",
                "window_samples": "Raw samples",
                "fit_relative_error": "Fit rel. err",
                "noise_scale_rad_s": "Noise scale",
                "aggressive_score": "Score",
            }
        )
    )


def format_psych_baseline_band_table(summary: pd.DataFrame) -> str:
    rows = []
    for band in ("low", "mid", "high"):
        band_data = summary[summary["noise_band"].astype(str) == band]
        for algorithm in ("PsyChMagPI N_sqrt", "PsyChMagPI best fixed"):
            data = band_data[band_data["algorithm"].astype(str) == algorithm]
            if data.empty:
                continue
            weights = data["trials"].to_numpy(dtype=float)
            rows.append(
                {
                    "Noise": band,
                    "PsyChMagPI view": algorithm,
                    "Win vs baselines": (
                        f"{100.0 * float(np.average(data['win_rate_vs_baselines'], weights=weights)):.0f}%"
                    ),
                    "Median err. deg": format_error(
                        float(data["median_error_deg"].median())
                    ),
                    "Mean err. deg": format_error(
                        float(np.average(data["mean_error_deg"], weights=weights))
                    ),
                }
            )
    return markdown_table(pd.DataFrame(rows))


def format_sqrt_rk2_band_table(trials: pd.DataFrame) -> str:
    by_band = (
        trials.groupby("noise_band", observed=True)
        .agg(
            trials=("psych_win_vs_rk2", "size"),
            win_rate_vs_rk2=("psych_win_vs_rk2", "mean"),
            median_psych_error_deg=("psych_error_deg", "median"),
            median_rk2_error_deg=("rk2_error_deg", "median"),
            median_delta_deg=("delta_deg", "median"),
        )
        .reset_index()
        .sort_values("noise_band")
    )
    display = by_band.copy()
    display["win_rate_vs_rk2"] = display["win_rate_vs_rk2"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    for column in [
        "median_psych_error_deg",
        "median_rk2_error_deg",
        "median_delta_deg",
    ]:
        display[column] = display[column].map(format_error)
    return markdown_table(
        display[
            [
                "noise_band",
                "trials",
                "win_rate_vs_rk2",
                "median_psych_error_deg",
                "median_rk2_error_deg",
                "median_delta_deg",
            ]
        ].rename(
            columns={
                "noise_band": "Noise",
                "trials": "Trials",
                "win_rate_vs_rk2": "Win vs RK2",
                "median_psych_error_deg": "PsyCh median",
                "median_rk2_error_deg": "RK2 median",
                "median_delta_deg": "PsyCh-RK2",
            }
        )
    )


def format_best_algorithm_by_band_sample_table(algorithm_summary: pd.DataFrame) -> str:
    display = (
        algorithm_summary.sort_values(
            ["noise_band", "m", "win_rate", "median_error_deg"],
            ascending=[True, True, False, True],
        )
        .groupby(["noise_band", "m"], observed=True)
        .head(1)
        .copy()
    )
    display["win_rate"] = display["win_rate"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    display["median_error_deg"] = display["median_error_deg"].map(format_error)
    return markdown_table(
        display[
            ["noise_band", "m", "algorithm", "win_rate", "median_error_deg"]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "algorithm": "Best algorithm",
                "win_rate": "Win rate",
                "median_error_deg": "Median err. deg",
            }
        )
    )


def format_psych_vs_baseline_by_band_sample_table(summary: pd.DataFrame) -> str:
    display = summary.copy()
    display["win_rate_vs_baselines"] = display["win_rate_vs_baselines"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    display["median_error_deg"] = display["median_error_deg"].map(format_error)
    return markdown_table(
        display[
            ["noise_band", "m", "algorithm", "win_rate_vs_baselines", "median_error_deg"]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "algorithm": "PsyChMagPI view",
                "win_rate_vs_baselines": "Win vs baselines",
                "median_error_deg": "Median err. deg",
            }
        )
    )


def format_best_settings_table(best_settings: pd.DataFrame) -> str:
    display = best_settings.copy()
    display["N"] = display["N"].map(lambda value: int(value))
    display["lambda1"] = display["lambda1"].map(lambda value: f"{float(value):g}")
    display["win_rate_vs_baselines"] = display["win_rate_vs_baselines"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    display["median_error_deg"] = display["median_error_deg"].map(format_error)
    return markdown_table(
        display[
            ["noise_band", "m", "N", "lambda1", "win_rate_vs_baselines", "median_error_deg"]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "N": "Best N",
                "lambda1": "Best lambda1",
                "win_rate_vs_baselines": "Win vs baselines",
                "median_error_deg": "Median err. deg",
            }
        )
    )


def format_sqrt_settings_table(setting_summary: pd.DataFrame) -> str:
    sqrt_rows = setting_summary[
        np.isclose(setting_summary["N"], np.ceil(np.sqrt(setting_summary["m"])))
    ]
    display = best_psychmagpi_settings_by_band(sqrt_rows)
    display["lambda1"] = display["lambda1"].map(lambda value: f"{float(value):g}")
    display["N"] = display["N"].map(lambda value: int(value))
    display["win_rate_vs_baselines"] = display["win_rate_vs_baselines"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    display["median_error_deg"] = display["median_error_deg"].map(format_error)
    return markdown_table(
        display[
            ["noise_band", "m", "N", "lambda1", "win_rate_vs_baselines", "median_error_deg"]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "N": "N_sqrt",
                "lambda1": "Best lambda1",
                "win_rate_vs_baselines": "Win vs baselines",
                "median_error_deg": "Median err. deg",
            }
        )
    )


def format_sqrt_rk2_sample_table(summary: pd.DataFrame) -> str:
    display = summary.copy()
    display["win_rate_vs_rk2"] = display["win_rate_vs_rk2"].map(
        lambda value: f"{100.0 * float(value):.0f}%"
    )
    for column in [
        "median_psych_error_deg",
        "median_rk2_error_deg",
        "median_delta_deg",
    ]:
        display[column] = display[column].map(format_error)
    return markdown_table(
        display[
            [
                "noise_band",
                "m",
                "trials",
                "N_sqrt",
                "lambda1",
                "win_rate_vs_rk2",
                "median_psych_error_deg",
                "median_rk2_error_deg",
                "median_delta_deg",
            ]
        ].rename(
            columns={
                "noise_band": "Noise",
                "m": "m",
                "trials": "Trials",
                "N_sqrt": "N_sqrt",
                "lambda1": "lambda1",
                "win_rate_vs_rk2": "Win vs RK2",
                "median_psych_error_deg": "PsyCh median",
                "median_rk2_error_deg": "RK2 median",
                "median_delta_deg": "PsyCh-RK2",
            }
        )
    )


def summarize_overall_algorithms(algorithm_trials: pd.DataFrame) -> pd.DataFrame:
    return (
        algorithm_trials.groupby("algorithm", observed=True)
        .agg(
            trials=("error_deg", "size"),
            win_rate=("winner", "mean"),
            median_error_deg=("error_deg", "median"),
            mean_error_deg=("error_deg", "mean"),
            max_error_deg=("error_deg", "max"),
        )
        .reset_index()
        .sort_values("algorithm")
    )


def sqrt_lambda_vs_rk2_trials(
    method_metrics: pd.DataFrame,
    *,
    lambda1: float,
) -> pd.DataFrame:
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    rk2 = method_metrics[method_metrics["method"] == "RK2"][
        key + ["error_deg"]
    ].rename(columns={"error_deg": "rk2_error_deg"})
    psych = method_metrics[
        (method_metrics["method"] == "PsyChMagPI")
        & np.isclose(method_metrics["N"], method_metrics["N_sqrt"])
        & np.isclose(method_metrics["lambda1"], float(lambda1))
    ][key + ["N_sqrt", "lambda1", "error_deg", "converged", "iterations"]].rename(
        columns={"error_deg": "psych_error_deg"}
    )
    trials = psych.merge(rk2, on=key, how="inner")
    trials["psych_win_vs_rk2"] = (
        trials["psych_error_deg"] <= trials["rk2_error_deg"] + 1e-15
    )
    trials["delta_deg"] = trials["psych_error_deg"] - trials["rk2_error_deg"]
    return trials.sort_values(key).reset_index(drop=True)


def summarize_sqrt_lambda_vs_rk2(trials: pd.DataFrame) -> pd.DataFrame:
    return (
        trials.groupby(["noise_band", "m"], observed=True)
        .agg(
            trials=("psych_win_vs_rk2", "size"),
            N_sqrt=("N_sqrt", "first"),
            lambda1=("lambda1", "first"),
            win_rate_vs_rk2=("psych_win_vs_rk2", "mean"),
            median_psych_error_deg=("psych_error_deg", "median"),
            median_rk2_error_deg=("rk2_error_deg", "median"),
            median_delta_deg=("delta_deg", "median"),
            mean_delta_deg=("delta_deg", "mean"),
            converged_rate=("converged", "mean"),
            median_iterations=("iterations", "median"),
        )
        .reset_index()
        .sort_values(["noise_band", "m"])
    )


def summarize_sqrt_lambda_vs_rk2_overall(trials: pd.DataFrame) -> pd.Series:
    return pd.Series(
        {
            "trials": int(trials.shape[0]),
            "win_rate_vs_rk2": float(trials["psych_win_vs_rk2"].mean()),
            "median_psych_error_deg": float(trials["psych_error_deg"].median()),
            "median_rk2_error_deg": float(trials["rk2_error_deg"].median()),
            "median_delta_deg": float(trials["delta_deg"].median()),
        }
    )


def summarize_psychmagpi_vs_baselines(algorithm_trials: pd.DataFrame) -> pd.DataFrame:
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    baselines = algorithm_trials[
        algorithm_trials["algorithm"].astype(str).isin(["RK2", "Magnus-2"])
    ]
    baseline_best = (
        baselines.groupby(key, observed=True)["error_deg"]
        .min()
        .rename("best_baseline_error_deg")
        .reset_index()
    )
    psych = algorithm_trials[
        algorithm_trials["algorithm"].astype(str).isin(
            ["PsyChMagPI N_sqrt", "PsyChMagPI best fixed"]
        )
    ].merge(baseline_best, on=key, how="left")
    psych["win_vs_baselines"] = (
        psych["error_deg"] <= psych["best_baseline_error_deg"] + 1e-15
    )
    return (
        psych.groupby(["algorithm", "m", "noise_band"], observed=True)
        .agg(
            trials=("error_deg", "size"),
            win_rate_vs_baselines=("win_vs_baselines", "mean"),
            median_error_deg=("error_deg", "median"),
            mean_error_deg=("error_deg", "mean"),
            max_error_deg=("error_deg", "max"),
        )
        .reset_index()
        .sort_values(["noise_band", "m", "algorithm"])
    )


def summarize_overall_psychmagpi_vs_baselines(
    algorithm_trials: pd.DataFrame,
) -> pd.DataFrame:
    key = ["dataset", "m", "noise_fraction", "noise_band", "seed_index"]
    baselines = algorithm_trials[
        algorithm_trials["algorithm"].astype(str).isin(["RK2", "Magnus-2"])
    ]
    baseline_best = (
        baselines.groupby(key, observed=True)["error_deg"]
        .min()
        .rename("best_baseline_error_deg")
        .reset_index()
    )
    psych = algorithm_trials[
        algorithm_trials["algorithm"].astype(str).isin(
            ["PsyChMagPI N_sqrt", "PsyChMagPI best fixed"]
        )
    ].merge(baseline_best, on=key, how="left")
    psych["win_vs_baselines"] = (
        psych["error_deg"] <= psych["best_baseline_error_deg"] + 1e-15
    )
    return (
        psych.groupby("algorithm", observed=True)
        .agg(
            trials=("error_deg", "size"),
            win_rate_vs_baselines=("win_vs_baselines", "mean"),
            median_error_deg=("error_deg", "median"),
            mean_error_deg=("error_deg", "mean"),
            max_error_deg=("error_deg", "max"),
        )
        .reset_index()
        .sort_values("algorithm")
    )


def format_algorithm_table(
    summary: pd.DataFrame,
    psych_vs_baseline_summary: pd.DataFrame,
) -> str:
    display = summary.copy()
    psych_rates = psych_vs_baseline_summary.set_index("algorithm")[
        "win_rate_vs_baselines"
    ].to_dict()
    display["win_rate_vs_baselines"] = display["algorithm"].map(psych_rates)
    display["win_rate"] = display["win_rate"].map(lambda value: f"{100.0 * float(value):.0f}%")
    display["win_rate_vs_baselines"] = display["win_rate_vs_baselines"].map(
        lambda value: "" if pd.isna(value) else f"{100.0 * float(value):.0f}%"
    )
    display["median_error_deg"] = display["median_error_deg"].map(format_error)
    display["mean_error_deg"] = display["mean_error_deg"].map(format_error)
    display["max_error_deg"] = display["max_error_deg"].map(format_error)
    return markdown_table(
        display[
            [
                "algorithm",
                "win_rate",
                "win_rate_vs_baselines",
                "median_error_deg",
                "mean_error_deg",
                "max_error_deg",
            ]
        ].rename(
            columns={
                "algorithm": "Algorithm",
                "win_rate": "Four-way win",
                "win_rate_vs_baselines": "Win vs baselines",
                "median_error_deg": "Median err. deg",
                "mean_error_deg": "Mean err. deg",
                "max_error_deg": "Max err. deg",
            }
        )
    )


def interpolation_executive_sentence(result: GyroInterpolationResult) -> str:
    signal = aggregate_interpolation_signal_by_method(result.signal_summary)
    trajectory = aggregate_interpolation_trajectory_by_method(result.trajectory_summary)
    subinterval = aggregate_interpolation_subinterval_by_method(
        result.subinterval_summary
    )
    signal_best = signal.sort_values("median_gyro_nrmse").iloc[0]
    trajectory_best = trajectory.sort_values("median_trajectory_rms_error_deg").iloc[0]
    sub_best = subinterval.sort_values("win_rate", ascending=False).iloc[0]
    psych_signal = signal[signal["method"] == "PsyChMagPI scaled lambda"].iloc[0]
    psych_trajectory = trajectory[
        trajectory["method"] == "PsyChMagPI scaled lambda"
    ].iloc[0]
    psych_subinterval = subinterval[
        subinterval["method"] == "PsyChMagPI scaled lambda"
    ].iloc[0]
    if str(sub_best["method"]) == "PsyChMagPI scaled lambda":
        subinterval_phrase = (
            "For random subintervals, scaled-lambda PsyChMagPI was best "
            f"with aggregate win rate {100.0 * float(psych_subinterval['win_rate']):.0f}%."
        )
    else:
        subinterval_phrase = (
            f"For random subintervals, `{sub_best['method']}` had aggregate win rate "
            f"{100.0 * float(sub_best['win_rate']):.0f}%, while scaled-lambda PsyChMagPI had "
            f"{100.0 * float(psych_subinterval['win_rate']):.0f}%."
        )
    return (
        f"The best median gyro interpolator was `{signal_best['method']}` "
        f"with NRMSE {format_error(float(signal_best['median_gyro_nrmse']))}; "
        f"deterministic scaled-lambda PsyChMagPI had NRMSE "
        f"{format_error(float(psych_signal['median_gyro_nrmse']))}. "
        f"The best median trajectory method was `{trajectory_best['method']}` "
        f"with RMS {format_error(float(trajectory_best['median_trajectory_rms_error_deg']))} degrees; "
        f"scaled-lambda PsyChMagPI had RMS "
        f"{format_error(float(psych_trajectory['median_trajectory_rms_error_deg']))} degrees. "
        f"{subinterval_phrase}"
    )


def format_interpolation_signal_table(summary: pd.DataFrame) -> str:
    aggregate = (
        summary.groupby(["method", "noise_band"], observed=True)
        .agg(
            median_gyro_nrmse=("median_gyro_nrmse", "median"),
            win_rate=("win_rate", "mean"),
            median_denoising_gain_db=("median_denoising_gain_db", "median"),
        )
        .reset_index()
    )
    rows = []
    for band in ("low", "mid", "high"):
        band_data = aggregate[aggregate["noise_band"].astype(str) == band]
        best = band_data.sort_values("median_gyro_nrmse").iloc[0]
        psych = band_data[band_data["method"] == "PsyChMagPI scaled lambda"].iloc[0]
        psych0 = band_data[band_data["method"] == "PsyChMagPI lambda0"].iloc[0]
        rows.append(
            {
                "Noise": band,
                "Best method": best["method"],
                "Best NRMSE": format_error(float(best["median_gyro_nrmse"])),
                "Scaled PsyCh NRMSE": format_error(float(psych["median_gyro_nrmse"])),
                "Lambda0 PsyCh NRMSE": format_error(float(psych0["median_gyro_nrmse"])),
                "Scaled win": f"{100.0 * float(psych['win_rate']):.0f}%",
                "Scaled denoise dB": format_error(
                    float(psych["median_denoising_gain_db"])
                ),
            }
        )
    return markdown_table(pd.DataFrame(rows))


def format_interpolation_trajectory_table(summary: pd.DataFrame) -> str:
    aggregate = (
        summary.groupby(["method", "noise_band"], observed=True)
        .agg(
            median_trajectory_rms_error_deg=(
                "median_trajectory_rms_error_deg",
                "median",
            ),
            win_rate=("win_rate", "mean"),
        )
        .reset_index()
    )
    rows = []
    for band in ("low", "mid", "high"):
        band_data = aggregate[aggregate["noise_band"].astype(str) == band]
        best = band_data.sort_values("median_trajectory_rms_error_deg").iloc[0]
        psych = band_data[band_data["method"] == "PsyChMagPI scaled lambda"].iloc[0]
        psych0 = band_data[band_data["method"] == "PsyChMagPI lambda0"].iloc[0]
        rows.append(
            {
                "Noise": band,
                "Best method": best["method"],
                "Best RMS deg": format_error(
                    float(best["median_trajectory_rms_error_deg"])
                ),
                "Scaled PsyCh RMS": format_error(
                    float(psych["median_trajectory_rms_error_deg"])
                ),
                "Lambda0 PsyCh RMS": format_error(
                    float(psych0["median_trajectory_rms_error_deg"])
                ),
                "Scaled win": f"{100.0 * float(psych['win_rate']):.0f}%",
            }
        )
    return markdown_table(pd.DataFrame(rows))


def format_interpolation_subinterval_table(summary: pd.DataFrame) -> str:
    rows = []
    for band in ("low", "mid", "high"):
        for duration in ("short", "mid", "long"):
            data = summary[
                (summary["noise_band"].astype(str) == band)
                & (summary["duration_bin"].astype(str) == duration)
            ]
            if data.empty:
                continue
            best = data.sort_values("win_rate", ascending=False).iloc[0]
            psych = data[data["method"] == "PsyChMagPI scaled lambda"].iloc[0]
            rows.append(
                {
                    "Noise": band,
                    "Duration": duration,
                    "Best method": best["method"],
                    "Best win": f"{100.0 * float(best['win_rate']):.0f}%",
                    "Scaled PsyCh win": f"{100.0 * float(psych['win_rate']):.0f}%",
                    "Scaled RMS deg": format_error(
                        float(psych["median_rms_error_deg"])
                    ),
                }
            )
    return markdown_table(pd.DataFrame(rows))


def aggregate_interpolation_signal_by_method(summary: pd.DataFrame) -> pd.DataFrame:
    return (
        summary.groupby("method", observed=True)
        .agg(
            median_gyro_nrmse=("median_gyro_nrmse", "median"),
            win_rate=("win_rate", "mean"),
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
            win_rate=("win_rate", "mean"),
        )
        .reset_index()
    )


def aggregate_interpolation_subinterval_by_method(summary: pd.DataFrame) -> pd.DataFrame:
    return (
        summary.groupby("method", observed=True)
        .agg(
            win_rate=("win_rate", "mean"),
            median_rms_error_deg=("median_rms_error_deg", "median"),
        )
        .reset_index()
    )


def format_taes_table_mc_wide(summary: pd.DataFrame) -> str:
    wide = taes_table_mc_wide_frame(summary)
    display = wide.copy()
    display["sigma_rad_s"] = display["sigma_rad_s"].map(format_sigma)
    for column in display.columns:
        if column != "sigma_rad_s":
            display[column] = display[column].map(format_error)
    return markdown_table(
        display.rename(
            columns={
                "sigma_rad_s": "Noise sigma rad/s",
                "D2 sequential rotations": "D2/RK2",
            }
        )
    )


def taes_table_mc_wide_frame(summary: pd.DataFrame) -> pd.DataFrame:
    classical_methods = [
        "D1 no coning",
        "D2 sequential rotations",
        "D3 single-speed",
    ]
    classical = (
        summary[summary["method"].astype(str).isin(classical_methods)]
        .pivot_table(
            index="sigma_rad_s",
            columns="method",
            values="rms_error_mrad",
            observed=True,
            aggfunc="first",
        )
        .reset_index()
        .sort_values("sigma_rad_s")
    )
    psych = summary[summary["method"].astype(str).str.startswith("PsyChMagPI")].copy()
    unregularized = psych[np.isclose(psych["lambda1"].astype(float), 0.0)][
        ["sigma_rad_s", "rms_error_mrad"]
    ].rename(columns={"rms_error_mrad": "PsyChMagPI lambda1=0"})
    regularized = (
        psych[psych["lambda1"].astype(float) > 0.0]
        .sort_values(["sigma_rad_s", "rms_error_mrad", "lambda1"])
        .groupby("sigma_rad_s", observed=True)
        .head(1)[["sigma_rad_s", "rms_error_mrad"]]
        .rename(columns={"rms_error_mrad": "PsyChMagPI best lambda1>0"})
    )
    wide = classical.merge(unregularized, on="sigma_rad_s", how="left").merge(
        regularized,
        on="sigma_rad_s",
        how="left",
    )
    columns = [
        "sigma_rad_s",
        *[column for column in classical_methods if column in wide],
        "PsyChMagPI lambda1=0",
        "PsyChMagPI best lambda1>0",
    ]
    return wide[columns]


def format_taes_table2_lambda_sweep(summary: pd.DataFrame) -> str:
    psych = summary[summary["method"].astype(str).str.startswith("PsyChMagPI")].copy()
    unregularized = psych[np.isclose(psych["lambda1"].astype(float), 0.0)][
        ["sigma_rad_s", "rms_error_mrad"]
    ].rename(columns={"rms_error_mrad": "unregularized_rms_mrad"})
    regularized = (
        psych[psych["lambda1"].astype(float) > 0.0]
        .sort_values(["sigma_rad_s", "rms_error_mrad", "lambda1"])
        .groupby("sigma_rad_s", observed=True)
        .head(1)[["sigma_rad_s", "lambda1", "rms_error_mrad"]]
        .rename(
            columns={
                "lambda1": "best_lambda1",
                "rms_error_mrad": "regularized_rms_mrad",
            }
        )
    )
    display = regularized.merge(unregularized, on="sigma_rad_s", how="left")
    display["delta_mrad"] = (
        display["regularized_rms_mrad"] - display["unregularized_rms_mrad"]
    )
    display["sigma_rad_s"] = display["sigma_rad_s"].map(format_sigma)
    display["best_lambda1"] = display["best_lambda1"].map(format_sigma)
    for column in (
        "regularized_rms_mrad",
        "unregularized_rms_mrad",
        "delta_mrad",
    ):
        display[column] = display[column].map(format_error)
    return markdown_table(
        display[
            [
                "sigma_rad_s",
                "best_lambda1",
                "regularized_rms_mrad",
                "unregularized_rms_mrad",
                "delta_mrad",
            ]
        ].rename(
            columns={
                "sigma_rad_s": "Noise sigma rad/s",
                "best_lambda1": "Best positive lambda1",
                "regularized_rms_mrad": "Reg. RMS mrad",
                "unregularized_rms_mrad": "lambda1=0 RMS mrad",
                "delta_mrad": "Reg - unreg mrad",
            }
        )
    )


def taes_table_mc_sentence(result: PureConingMonteCarloResult) -> str:
    wide = taes_table_mc_wide_frame(result.summary)
    classical_columns = [
        "D1 no coning",
        "D2 sequential rotations",
        "D3 single-speed",
    ]
    zero = wide[np.isclose(wide["sigma_rad_s"], 0.0)].iloc[0]
    highest_noise = wide.sort_values("sigma_rad_s").iloc[-1]
    zero_psych = float(zero["PsyChMagPI lambda1=0"])
    zero_classical = float(zero[classical_columns].min())
    high_unregularized = float(highest_noise["PsyChMagPI lambda1=0"])
    high_regularized = float(highest_noise["PsyChMagPI best lambda1>0"])
    high_classical = float(highest_noise[classical_columns].min())
    high_sigma = float(highest_noise["sigma_rad_s"])
    return (
        "Using our TAES draft's Table-II noise grid and RMS final attitude error in mrad, "
        f"PsyChMagPI with `lambda1=0` reached {format_error(zero_psych)} mrad at zero noise "
        f"versus {format_error(zero_classical)} mrad for the best classical "
        "method. At "
        rf"$\sigma={format_sigma(high_sigma)}$ rad/s, the best positive-`lambda1` "
        f"PsyChMagPI sweep entry was {format_error(high_regularized)} mrad "
        f"and the `lambda1=0` entry was {format_error(high_unregularized)} mrad versus "
        f"{format_error(high_classical)} mrad for the best classical method, "
        "showing the expected shrinkage of high-order advantage as rate noise grew."
    )


def psych_baseline_band_sentence(
    psych_vs_baseline_summary: pd.DataFrame,
    algorithm: str,
) -> str:
    pieces = []
    data = psych_vs_baseline_summary[
        psych_vs_baseline_summary["algorithm"].astype(str) == algorithm
    ]
    for band in ("low", "mid", "high"):
        band_data = data[data["noise_band"].astype(str) == band]
        if band_data.empty:
            continue
        weighted = np.average(
            band_data["win_rate_vs_baselines"].to_numpy(dtype=float),
            weights=band_data["trials"].to_numpy(dtype=float),
        )
        pieces.append(f"{band} noise {100.0 * float(weighted):.0f}%")
    return "by band: " + ", ".join(pieces)


def write_plotly_image(
    fig: go.Figure,
    path: Path,
    *,
    width: int | None = None,
    height: int | None = None,
) -> Path:
    layout_width = int(fig.layout.width) if fig.layout.width else 1200
    layout_height = int(fig.layout.height) if fig.layout.height else 420
    fig.write_image(
        path,
        width=width or layout_width,
        height=height or layout_height,
        scale=1.6,
    )
    return path


def number_display_equations(markdown: str) -> str:
    """Convert generated display-math blocks to numbered LaTeX equations."""
    output: list[str] = []
    in_equation = False
    in_fence = False
    for line in markdown.splitlines():
        stripped = line.strip()
        if stripped.startswith("```"):
            in_fence = not in_fence
            output.append(line)
            continue
        if not in_fence and stripped == "$$":
            output.append(r"\begin{equation}" if not in_equation else r"\end{equation}")
            in_equation = not in_equation
            continue
        output.append(line)
    if in_equation:
        raise ValueError("unclosed display-math block while numbering equations")
    return "\n".join(output) + ("\n" if markdown.endswith("\n") else "")


def format_error(value: float) -> str:
    value = float(value)
    if not np.isfinite(value):
        return "nan"
    if abs(value) < 1e-3:
        return f"{value:.2e}"
    return f"{value:.4g}"


def format_signed_error(value: float) -> str:
    value = float(value)
    if not np.isfinite(value):
        return "nan"
    if value == 0.0:
        return "0"
    return f"{value:+.3g}"


def format_sigma(value: float) -> str:
    value = float(value)
    if value == 0.0:
        return "0"
    if abs(value) >= 1e-3:
        return f"{value:g}"
    return f"{value:.0e}"


def format_signed_pp(value: float) -> str:
    value = float(value)
    if not np.isfinite(value):
        return "nan pp"
    if value == 0.0:
        return "0 pp"
    return f"{value:+.1f} pp"


def format_ratio(value: float) -> str:
    value = float(value)
    if not np.isfinite(value):
        return "nan"
    if value >= 1.0e5:
        return f"{value:.2g}x"
    if value >= 100.0:
        return f"{value:.0f}x"
    return f"{value:.1f}x"


def markdown_table(frame: pd.DataFrame) -> str:
    headers = [str(column) for column in frame.columns]
    rows = [[format_cell(value) for value in row] for row in frame.to_numpy()]
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]
    header = (
        "| "
        + " | ".join(headers[index].ljust(widths[index]) for index in range(len(headers)))
        + " |"
    )
    separator = (
        "| " + " | ".join("-" * widths[index] for index in range(len(headers))) + " |"
    )
    body = [
        "| "
        + " | ".join(row[index].ljust(widths[index]) for index in range(len(headers)))
        + " |"
        for row in rows
    ]
    return "\n".join([header, separator, *body])


def format_cell(value: object) -> str:
    if pd.isna(value):
        return ""
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


def relative_asset(path: Path) -> str:
    return path.relative_to(REPORT_DIR).as_posix()


if __name__ == "__main__":
    main()
