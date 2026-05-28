"""Reusable scalar quadrature experiments using shared spectral fits."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from matplotlib.colors import TwoSlopeNorm
from plotly.subplots import make_subplots

import imuFactors.spectral as spectral


DEFAULT_INTERVAL = (0.0, 1.0)
METRIC_COLUMNS = ("end_error", "rmse_error", "max_error")
METRIC_LABELS = {
    "end_error": "Final integration error",
    "rmse_error": "RMSE integration error",
    "max_error": "Absolute max integration error",
}
_NOISE_BAND_COLORS = ("#4d4d4d", "#2a6f97", "#b23a48")
_NOISE_BAND_NAMES = ("low noise", "mid noise", "high noise")


ArrayFunction = Callable[[np.ndarray], np.ndarray]


@dataclass(frozen=True)
class ScalarFunction:
    """Scalar function and antiderivative used for quadrature experiments."""

    name: str
    value: ArrayFunction
    antiderivative: ArrayFunction


@dataclass(frozen=True)
class ScalarMonteCarloResult:
    """Monte Carlo metrics and Chebyshev-minus-trapezoid comparisons."""

    method_metrics: pd.DataFrame
    comparisons: pd.DataFrame


@dataclass(frozen=True)
class _ChebyshevExperimentPlan:
    """Precomputed matrices for repeated GTSAM Chebyshev2 scalar fits."""

    sample_times: np.ndarray
    evaluation_times: np.ndarray
    N: int
    interval: tuple[float, float]
    fit_plan: spectral.BasisPlan
    final_weights: np.ndarray
    curve_weights: np.ndarray
    lambda1: float = 0.0

    @classmethod
    def build(
        cls,
        sample_times: np.ndarray,
        evaluation_times: np.ndarray,
        N: int,
        interval: tuple[float, float],
        lambda1: float = 0.0,
    ) -> "_ChebyshevExperimentPlan":
        fit_plan = spectral.basis_plan_from_coordinates(
            N,
            sample_times,
            basis="chebyshev2",
            interval=interval,
            lambda1=lambda1,
        )
        final_weights = spectral.chebyshev2_integration_weights(N, interval)
        curve_weights = spectral.chebyshev2_integral_curve_matrix(
            N, evaluation_times, interval
        )
        return cls(
            sample_times=sample_times,
            evaluation_times=evaluation_times,
            N=N,
            interval=interval,
            fit_plan=fit_plan,
            final_weights=final_weights,
            curve_weights=curve_weights,
            lambda1=float(lambda1),
        )

    def fit_node_values(self, sample_values: np.ndarray) -> np.ndarray:
        values = np.asarray(sample_values, dtype=float)
        if values.ndim == 1:
            return self.fit_plan.fit(values)
        return values @ self.fit_plan.solver.T

    def integral_curves(
        self, sample_values: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        node_values = self.fit_node_values(sample_values)
        if node_values.ndim == 1:
            curve = self.curve_weights @ node_values
            final = float(self.final_weights @ node_values)
            return np.asarray([final]), curve.reshape(1, -1)
        final = node_values @ self.final_weights
        curves = node_values @ self.curve_weights.T
        return final, curves


def chebyshev_trial_curves(
    sample_times: Sequence[float],
    sample_values: Sequence[float] | np.ndarray,
    evaluation_times: Sequence[float],
    N: int,
    interval: tuple[float, float] = DEFAULT_INTERVAL,
    lambda1: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Fit ``m`` scalar samples with ``N`` Chebyshev2 CGL nodes."""

    sample_time_array = np.asarray(sample_times, dtype=float)
    evaluation_time_array = np.asarray(evaluation_times, dtype=float)
    plan = _ChebyshevExperimentPlan.build(
        sample_time_array,
        evaluation_time_array,
        int(N),
        interval,
        lambda1=lambda1,
    )
    final, curves = plan.integral_curves(np.asarray(sample_values, dtype=float))
    if np.asarray(sample_values).ndim == 1:
        return final[0], curves[0]
    return final, curves


def trapezoid_integral_curve(
    sample_times: Sequence[float],
    sample_values: Sequence[float] | np.ndarray,
    evaluation_times: Sequence[float],
) -> np.ndarray:
    """Evaluate the cumulative trapezoidal integral at arbitrary times."""

    sample_time_array = np.asarray(sample_times, dtype=float)
    sample_value_array = np.asarray(sample_values, dtype=float)
    evaluation_time_array = np.asarray(evaluation_times, dtype=float)
    curves = _trapezoid_integral_curves(
        sample_time_array, sample_value_array, evaluation_time_array
    )
    if sample_value_array.ndim == 1:
        return curves[0]
    return curves


def scalar_function_from_chebyshev2_nodes(
    name: str,
    node_values: Sequence[float],
    interval: tuple[float, float] = DEFAULT_INTERVAL,
) -> ScalarFunction:
    """Create a scalar function from ``N`` GTSAM Chebyshev2 CGL values."""

    values = np.asarray(node_values, dtype=float).reshape(-1)
    N = values.size
    integration_matrix = spectral.chebyshev2_integration_matrix(N, interval)
    antiderivative_nodes = integration_matrix @ values

    def evaluate(times: np.ndarray) -> np.ndarray:
        time_array = np.asarray(times, dtype=float)
        weights = spectral.chebyshev2_weight_matrix(N, time_array, interval)
        return weights @ values

    def antiderivative(times: np.ndarray) -> np.ndarray:
        time_array = np.asarray(times, dtype=float)
        weights = spectral.chebyshev2_weight_matrix(N + 1, time_array, interval)
        return weights @ antiderivative_nodes

    return ScalarFunction(name=name, value=evaluate, antiderivative=antiderivative)


def run_scalar_monte_carlo(
    functions: Sequence[ScalarFunction],
    m_values: Iterable[int],
    N_values: Iterable[int],
    noise_fractions: Iterable[float],
    *,
    num_seeds: int = 100,
    seed: int = 0,
    interval: tuple[float, float] = DEFAULT_INTERVAL,
    evaluation_count: int = 201,
    lambda1: float = 0.0,
) -> ScalarMonteCarloResult:
    """Run noisy quadrature comparisons over ``m`` samples and ``N`` CGL nodes."""

    a, b = interval
    lambda1 = float(lambda1)
    evaluation_times = np.linspace(a, b, evaluation_count)
    m_values = [int(m) for m in m_values]
    N_values = [int(N) for N in N_values]
    noise_fractions = [float(noise_fraction) for noise_fraction in noise_fractions]

    method_rows: list[dict[str, float | int | str]] = []
    comparison_rows: list[dict[str, float | int | str]] = []
    plan_cache: dict[tuple[int, int, float], _ChebyshevExperimentPlan] = {}

    for function_index, function in enumerate(functions):
        true_curve = _true_integral_curve(function, evaluation_times, interval)
        true_final = float(true_curve[-1])
        value_range = _function_range(function, interval)

        for m in m_values:
            sample_times = np.linspace(a, b, m)
            clean_samples = np.asarray(function.value(sample_times), dtype=float)
            standard_noises = np.random.default_rng(
                seed + 1000003 * function_index + 1009 * m
            ).normal(size=(num_seeds, m))
            clean_trapezoid_curve = _trapezoid_integral_curves(
                sample_times, clean_samples, evaluation_times
            )[0]
            noise_trapezoid_curves = _trapezoid_integral_curves(
                sample_times, standard_noises, evaluation_times
            )

            valid_plans = {}
            for N in N_values:
                if N > m:
                    continue
                cache_key = (m, N, lambda1)
                if cache_key not in plan_cache:
                    plan_cache[cache_key] = _ChebyshevExperimentPlan.build(
                        sample_times,
                        evaluation_times,
                        N,
                        interval,
                        lambda1=lambda1,
                    )
                valid_plans[N] = plan_cache[cache_key]

            chebyshev_components = {}
            for N, plan in valid_plans.items():
                clean_final, clean_curve = plan.integral_curves(clean_samples)
                noise_final, noise_curves = plan.integral_curves(standard_noises)
                chebyshev_components[N] = (
                    float(clean_final[0]),
                    clean_curve[0],
                    noise_final,
                    noise_curves,
                )

            for noise_fraction in noise_fractions:
                noise_std = noise_fraction * value_range
                trapezoid_curves = (
                    clean_trapezoid_curve[None, :] + noise_std * noise_trapezoid_curves
                )
                trapezoid_final = trapezoid_curves[:, -1]
                trapezoid_metrics = _average_metrics(
                    trapezoid_final, trapezoid_curves, true_final, true_curve
                )
                method_rows.append(
                    _method_row(
                        function.name,
                        "trapezoid",
                        noise_fraction,
                        noise_std,
                        m,
                        np.nan,
                        trapezoid_metrics,
                        lambda1=0.0,
                    )
                )

                for N, components in chebyshev_components.items():
                    (
                        clean_chebyshev_final,
                        clean_chebyshev_curve,
                        noise_chebyshev_final,
                        noise_chebyshev_curves,
                    ) = components
                    chebyshev_final = (
                        clean_chebyshev_final + noise_std * noise_chebyshev_final
                    )
                    chebyshev_curves = (
                        clean_chebyshev_curve[None, :]
                        + noise_std * noise_chebyshev_curves
                    )
                    chebyshev_metrics = _average_metrics(
                        chebyshev_final, chebyshev_curves, true_final, true_curve
                    )
                    method_rows.append(
                        _method_row(
                            function.name,
                            "chebyshev2",
                            noise_fraction,
                            noise_std,
                            m,
                            N,
                            chebyshev_metrics,
                            lambda1=lambda1,
                        )
                    )
                    comparison_rows.append(
                        _comparison_row(
                            function.name,
                            noise_fraction,
                            noise_std,
                            m,
                            N,
                            chebyshev_metrics,
                            trapezoid_metrics,
                            lambda1=lambda1,
                        )
                    )

    return ScalarMonteCarloResult(
        method_metrics=pd.DataFrame(method_rows),
        comparisons=pd.DataFrame(comparison_rows),
    )


def plot_fixed_N_comparison(
    comparisons: pd.DataFrame,
    function_name: str,
    *,
    selected_N_values: Sequence[int] = (2, 9, 17, 24),
    metrics: Sequence[str] = METRIC_COLUMNS,
    x_column: str = "noise_std",
    cmap: str = "coolwarm",
) -> plt.Figure:
    """Plot metric differences over noise and ``m`` samples for fixed ``N``."""

    fig, axes = plt.subplots(
        len(metrics),
        len(selected_N_values),
        figsize=(3.6 * len(selected_N_values), 3.0 * len(metrics)),
        squeeze=False,
        constrained_layout=True,
    )
    subset = comparisons[comparisons["function"] == function_name]
    for row_index, metric in enumerate(metrics):
        for col_index, N in enumerate(selected_N_values):
            axis = axes[row_index, col_index]
            grid_subset = subset[subset["N"] == N]
            _contour_metric_grid(
                axis,
                grid_subset,
                x_column=x_column,
                y_column="m",
                metric=metric,
                cmap=cmap,
            )
            axis.set_title(f"{METRIC_LABELS.get(metric, metric)}, N={N}")
            axis.set_xlabel("Noise std. dev. sigma")
            axis.set_ylabel("m samples")
    fig.suptitle(
        f"{function_name}: Chebyshev2 minus trapezoid (negative is better)",
        fontsize=14,
    )
    return fig


def plot_fixed_m_comparison(
    comparisons: pd.DataFrame,
    function_name: str,
    *,
    selected_m_values: Sequence[int] = (10, 20, 30, 40, 50),
    metrics: Sequence[str] = METRIC_COLUMNS,
    x_column: str = "noise_std",
    cmap: str = "coolwarm",
    show_sqrt_m: bool = False,
) -> plt.Figure:
    """Plot metric differences over noise and ``N`` for fixed ``m``."""

    fig, axes = plt.subplots(
        len(metrics),
        len(selected_m_values),
        figsize=(3.6 * len(selected_m_values), 3.0 * len(metrics)),
        squeeze=False,
        constrained_layout=True,
    )
    subset = comparisons[comparisons["function"] == function_name]
    for row_index, metric in enumerate(metrics):
        for col_index, m in enumerate(selected_m_values):
            axis = axes[row_index, col_index]
            grid_subset = subset[subset["m"] == m]
            _contour_metric_grid(
                axis,
                grid_subset,
                x_column=x_column,
                y_column="N",
                metric=metric,
                cmap=cmap,
            )
            axis.set_title(f"{METRIC_LABELS.get(metric, metric)}, m={m}")
            axis.set_xlabel("Noise std. dev. sigma")
            axis.set_ylabel("N CGL nodes (n=N-1)")
            if show_sqrt_m:
                axis.axhline(
                    np.sqrt(m),
                    color="lightgrey",
                    linestyle="--",
                    linewidth=1.5,
                )
    fig.suptitle(
        f"{function_name}: Chebyshev2 minus trapezoid (negative is better)",
        fontsize=14,
    )
    return fig


def plot_function_noise_comparison(
    comparisons: pd.DataFrame,
    function_names: Sequence[str],
    selected_noise_fractions: Sequence[float],
    *,
    metric: str = "end_error",
    cmap: str = "coolwarm",
) -> plt.Figure:
    """Plot one metric over ``m`` and ``N`` across functions/noise levels."""

    fig, axes = plt.subplots(
        len(function_names),
        len(selected_noise_fractions),
        figsize=(3.6 * len(selected_noise_fractions), 3.0 * len(function_names)),
        squeeze=False,
        constrained_layout=True,
    )
    for row_index, function_name in enumerate(function_names):
        function_subset = comparisons[comparisons["function"] == function_name]
        for col_index, requested_noise_fraction in enumerate(selected_noise_fractions):
            axis = axes[row_index, col_index]
            noise_fraction = _nearest_value(
                function_subset["noise_fraction"], requested_noise_fraction
            )
            grid_subset = function_subset[
                np.isclose(function_subset["noise_fraction"], noise_fraction)
            ]
            _contour_metric_grid(
                axis,
                grid_subset,
                x_column="m",
                y_column="N",
                metric=metric,
                cmap=cmap,
            )
            axis.set_title(f"{function_name}, noise fraction={noise_fraction:.3g}")
            axis.set_xlabel("m samples")
            axis.set_ylabel("N CGL nodes (n=N-1)")
    fig.suptitle(
        f"{METRIC_LABELS.get(metric, metric)}: Chebyshev2 minus trapezoid",
        fontsize=14,
    )
    return fig


def plot_advantage_curves_by_m(
    comparisons: pd.DataFrame,
    function_name: str,
    *,
    selected_m_values: Sequence[int] | None = None,
    metric: str = "rmse_error",
    noise_band_count: int = 3,
    y_percentile_range: tuple[float, float] | None = (5.0, 95.0),
    y_range_min_N: int | None = None,
) -> go.Figure:
    """Plot Chebyshev2 advantage over trapezoid as ``N`` varies for fixed ``m``.

    The y axis is percentile-clipped by default so rare large outliers do not
    dominate the scale. Pass ``y_percentile_range=None`` for autoranging. Pass
    ``y_range_min_N`` to compute the y-axis range only from ``N`` values at or
    above that threshold while still plotting all ``N`` values.
    """

    if metric not in METRIC_COLUMNS:
        raise ValueError(f"Unknown metric {metric!r}; expected one of {METRIC_COLUMNS}")
    _validate_percentile_range(y_percentile_range)

    subset = _function_subset(comparisons, function_name)
    m_values = _selected_m_values(subset, selected_m_values)
    if not m_values:
        return _empty_plotly_figure(f"No comparison rows for {function_name}")

    fig = make_subplots(
        rows=1,
        cols=len(m_values),
        subplot_titles=[f"m={m}" for m in m_values],
        shared_yaxes=True,
        horizontal_spacing=0.035,
    )
    noise_band_labels = _noise_band_labels(subset["noise_fraction"], noise_band_count)
    band_order = list(dict.fromkeys(noise_band_labels.values()))
    colors = _noise_band_colors(len(band_order))
    plotted_advantages: list[np.ndarray] = []

    for col_index, m in enumerate(m_values, start=1):
        panel = subset[subset["m"] == m].copy()
        if panel.empty:
            fig.add_annotation(
                text="no rows",
                x=0.5,
                y=0.5,
                xref=f"x{col_index if col_index > 1 else ''} domain",
                yref=f"y{col_index if col_index > 1 else ''} domain",
                showarrow=False,
            )
            continue

        panel["advantage"] = -panel[metric]
        panel["noise_band"] = panel["noise_fraction"].map(noise_band_labels)
        band_summary = (
            panel.groupby(["N", "noise_band"], as_index=False)["advantage"]
            .median()
            .sort_values(["noise_band", "N"])
        )
        plotted_advantages.append(_axis_range_advantages(band_summary, y_range_min_N))
        for band_index, band_label in enumerate(band_order):
            band_data = band_summary[band_summary["noise_band"] == band_label]
            if band_data.empty:
                continue
            fig.add_trace(
                go.Scatter(
                    x=band_data["N"],
                    y=band_data["advantage"],
                    mode="lines+markers",
                    line=dict(width=1.5, color=colors[band_index % len(colors)]),
                    marker=dict(size=5),
                    name=band_label,
                    legendgroup=band_label,
                    showlegend=col_index == 1,
                    hovertemplate=(
                        "m=%{customdata[0]}<br>"
                        "N=%{x}<br>"
                        "n=%{customdata[2]}<br>"
                        "noise band=%{customdata[1]}<br>"
                        "advantage=%{y:.4g}<extra></extra>"
                    ),
                    customdata=np.column_stack(
                        [
                            np.full(len(band_data), m),
                            np.full(len(band_data), band_label),
                            band_data["N"].to_numpy(dtype=int) - 1,
                        ]
                    ),
                ),
                row=1,
                col=col_index,
            )

        best = _best_N_for_metric(panel, metric)
        if best is not None:
            fig.add_trace(
                go.Scatter(
                    x=[best["N"]],
                    y=[best["advantage"]],
                    mode="markers",
                    marker=dict(size=9, symbol="diamond", color="black"),
                    name="best median N",
                    legendgroup="best median N",
                    showlegend=col_index == 1,
                    hovertemplate=(
                        "best median N=%{x}<br>"
                        "median advantage=%{y:.4g}<extra></extra>"
                    ),
                ),
                row=1,
                col=col_index,
            )

        fig.add_hline(
            y=0.0,
            line_width=1,
            line_color="rgba(0,0,0,0.35)",
            row=1,
            col=col_index,
        )
        fig.add_vline(
            x=np.sqrt(m),
            line_width=1,
            line_dash="dash",
            line_color="lightgrey",
            row=1,
            col=col_index,
        )

    fig.update_layout(
        title=(
            f"{function_name}: Chebyshev2 advantage by N at fixed m "
            f"({METRIC_LABELS[metric]})"
        ),
        template="plotly_white",
        height=360,
        width=max(900, 240 * len(m_values)),
        legend=dict(orientation="h", yanchor="bottom", y=1.08, xanchor="left", x=0.0),
        margin=dict(l=55, r=20, t=85, b=45),
    )
    fig.update_xaxes(title_text="N CGL nodes (n=N-1)", showgrid=False, ticks="outside")
    y_axis_settings = dict(
        title_text="trapezoid error - Chebyshev2 error",
        showgrid=True,
        gridcolor="rgba(0,0,0,0.08)",
        zeroline=False,
        ticks="outside",
    )
    y_axis_range = _robust_axis_range(plotted_advantages, y_percentile_range)
    if y_axis_range is not None:
        y_axis_settings["range"] = y_axis_range
    fig.update_yaxes(**y_axis_settings)
    return fig


def robust_N_summary(
    comparisons: pd.DataFrame,
    function_name: str,
    *,
    selected_m_values: Sequence[int] | None = None,
    metrics: Sequence[str] = METRIC_COLUMNS,
) -> pd.DataFrame:
    """Summarize metric-wise and robust ``N`` choices for each fixed ``m``."""

    for metric in metrics:
        if metric not in METRIC_COLUMNS:
            raise ValueError(
                f"Unknown metric {metric!r}; expected one of {METRIC_COLUMNS}"
            )

    subset = _function_subset(comparisons, function_name)
    m_values = _selected_m_values(subset, selected_m_values)
    rows: list[dict[str, float | int | str]] = []

    for m in m_values:
        panel = subset[subset["m"] == m].copy()
        if panel.empty:
            continue

        N_grid = np.asarray(sorted(panel["N"].dropna().unique()), dtype=int)
        if N_grid.size == 0:
            continue

        metric_summaries = {
            metric: _median_advantage_by_N(panel, metric).reindex(N_grid)
            for metric in metrics
        }
        best_by_metric = {
            metric: _best_N_from_advantage(metric_summary)
            for metric, metric_summary in metric_summaries.items()
        }
        ranks = pd.concat(
            [
                metric_summary.rank(ascending=False, method="average")
                for metric_summary in metric_summaries.values()
            ],
            axis=1,
        )
        rank_score = ranks.mean(axis=1)
        robust_N = _best_N_from_score(rank_score)
        robust_rows = panel[panel["N"] == robust_N]
        robust_advantages = -robust_rows[list(metrics)].to_numpy(dtype=float)
        win_rate = float(np.mean(robust_advantages > 0.0))
        rmse_summary = metric_summaries.get(
            "rmse_error", next(iter(metric_summaries.values()))
        )

        row: dict[str, float | int | str] = {
            "function": function_name,
            "m": int(m),
            "sqrt_m": float(np.sqrt(m)),
            "robust_N": int(robust_N),
            "robust_rank_score": float(rank_score.loc[robust_N]),
            "win_rate": win_rate,
            "rmse_advantage_at_robust_N": float(rmse_summary.loc[robust_N]),
            "rmse_advantage_sparkline": _ascii_sparkline(
                rmse_summary.reindex(N_grid).to_numpy(dtype=float)
            ),
        }
        for metric, best_N in best_by_metric.items():
            row[f"best_{metric}_N"] = int(best_N)
        rows.append(row)

    return pd.DataFrame(rows)


def plot_robust_N_table(
    comparisons: pd.DataFrame,
    function_name: str,
    *,
    selected_m_values: Sequence[int] | None = None,
    metrics: Sequence[str] = METRIC_COLUMNS,
) -> go.Figure:
    """Plot a compact table of ideal and robust ``N`` choices for fixed ``m``."""

    summary = robust_N_summary(
        comparisons,
        function_name,
        selected_m_values=selected_m_values,
        metrics=metrics,
    )
    if summary.empty:
        return _empty_plotly_figure(f"No comparison rows for {function_name}")

    columns = [
        ("m", "m"),
        ("sqrt_m", "sqrt(m)"),
        ("best_end_error_N", "best final N"),
        ("best_rmse_error_N", "best RMSE N"),
        ("best_max_error_N", "best max N"),
        ("robust_N", "robust N"),
        ("win_rate", "win rate"),
        ("rmse_advantage_at_robust_N", "RMSE advantage"),
        ("rmse_advantage_sparkline", "RMSE advantage over N"),
    ]
    available_columns = [(key, label) for key, label in columns if key in summary]
    alignments = [
        "left" if key == "rmse_advantage_sparkline" else "right"
        for key, _ in available_columns
    ]
    column_widths = {
        "m": 0.55,
        "sqrt_m": 0.75,
        "best_end_error_N": 0.85,
        "best_rmse_error_N": 0.9,
        "best_max_error_N": 0.75,
        "robust_N": 0.75,
        "win_rate": 0.75,
        "rmse_advantage_at_robust_N": 1.05,
        "rmse_advantage_sparkline": 1.9,
    }
    values: list[list[str | int]] = []
    for key, _ in available_columns:
        if key == "sqrt_m":
            values.append([f"{value:.2f}" for value in summary[key]])
        elif key == "win_rate":
            values.append([f"{100.0 * value:.0f}%" for value in summary[key]])
        elif key == "rmse_advantage_at_robust_N":
            values.append([f"{value:.4g}" for value in summary[key]])
        elif key == "rmse_advantage_sparkline":
            values.append(summary[key].astype(str).tolist())
        else:
            values.append(summary[key].astype(int).tolist())

    fig = go.Figure(
        data=[
            go.Table(
                columnwidth=[
                    column_widths.get(key, 0.85) for key, _ in available_columns
                ],
                header=dict(
                    values=[label for _, label in available_columns],
                    fill_color="rgba(0,0,0,0.06)",
                    line_color="rgba(0,0,0,0.15)",
                    align=alignments,
                    font=dict(size=12, color="black"),
                    height=28,
                ),
                cells=dict(
                    values=values,
                    fill_color="white",
                    line_color="rgba(0,0,0,0.08)",
                    align=alignments,
                    font=dict(size=12, color="black", family="monospace"),
                    height=28,
                ),
            )
        ]
    )
    fig.update_layout(
        title=(
            f"{function_name}: ideal N by metric and robust N "
            "(RMSE sparkline is N ascending)"
        ),
        template="plotly_white",
        height=max(240, 120 + 32 * len(summary)),
        margin=dict(l=20, r=20, t=70, b=20),
    )
    return fig


def _trapezoid_integral_curves(
    sample_times: np.ndarray,
    sample_values: np.ndarray,
    evaluation_times: np.ndarray,
) -> np.ndarray:
    values = np.asarray(sample_values, dtype=float)
    if values.ndim == 1:
        values = values.reshape(1, -1)
    if values.shape[1] != sample_times.size:
        raise ValueError("sample_values must align with sample_times")
    if sample_times.size < 2:
        raise ValueError("trapezoidal integration needs at least two samples")
    if not np.all(np.diff(sample_times) > 0):
        raise ValueError("sample_times must be strictly increasing")

    segment_widths = np.diff(sample_times)
    segment_areas = 0.5 * segment_widths[None, :] * (values[:, :-1] + values[:, 1:])
    cumulative = np.zeros((values.shape[0], sample_times.size))
    cumulative[:, 1:] = np.cumsum(segment_areas, axis=1)

    indices = np.searchsorted(sample_times, evaluation_times, side="right") - 1
    indices = np.clip(indices, 0, sample_times.size - 2)
    left_times = sample_times[indices]
    right_times = sample_times[indices + 1]
    left_values = values[:, indices]
    right_values = values[:, indices + 1]
    slopes = (right_values - left_values) / (right_times - left_times)
    dt = evaluation_times - left_times
    interpolated_values = left_values + slopes * dt
    curves = cumulative[:, indices] + 0.5 * dt * (left_values + interpolated_values)

    curves[:, evaluation_times <= sample_times[0]] = 0.0
    curves[:, evaluation_times >= sample_times[-1]] = cumulative[:, -1:]
    return curves


def _true_integral_curve(
    function: ScalarFunction,
    evaluation_times: np.ndarray,
    interval: tuple[float, float],
) -> np.ndarray:
    a, _ = interval
    baseline = np.asarray(function.antiderivative(np.asarray([a])), dtype=float)[0]
    return np.asarray(function.antiderivative(evaluation_times), dtype=float) - baseline


def _function_range(
    function: ScalarFunction,
    interval: tuple[float, float],
    grid_size: int = 4097,
) -> float:
    a, b = interval
    values = np.asarray(function.value(np.linspace(a, b, grid_size)), dtype=float)
    return float(np.max(values) - np.min(values))


def _average_metrics(
    final_estimates: np.ndarray,
    curves: np.ndarray,
    true_final: float,
    true_curve: np.ndarray,
) -> dict[str, float]:
    errors = curves - true_curve[None, :]
    return {
        "end_error": float(np.mean(np.abs(final_estimates - true_final))),
        "rmse_error": float(np.mean(np.sqrt(np.mean(errors * errors, axis=1)))),
        "max_error": float(np.mean(np.max(np.abs(errors), axis=1))),
    }


def _method_row(
    function_name: str,
    method: str,
    noise_fraction: float,
    noise_std: float,
    m: int,
    N: float,
    metrics: dict[str, float],
    *,
    lambda1: float,
) -> dict[str, float | int | str]:
    row: dict[str, float | int | str] = {
        "function": function_name,
        "method": method,
        "noise_fraction": noise_fraction,
        "noise_std": noise_std,
        "m": m,
        "N": N,
        "n": _n_from_N(N),
        "lambda1": float(lambda1),
    }
    row.update(metrics)
    return row


def _comparison_row(
    function_name: str,
    noise_fraction: float,
    noise_std: float,
    m: int,
    N: int,
    chebyshev_metrics: dict[str, float],
    trapezoid_metrics: dict[str, float],
    *,
    lambda1: float,
) -> dict[str, float | int | str]:
    row: dict[str, float | int | str] = {
        "function": function_name,
        "noise_fraction": noise_fraction,
        "noise_std": noise_std,
        "m": m,
        "N": N,
        "n": _n_from_N(N),
        "lambda1": float(lambda1),
    }
    for metric in METRIC_COLUMNS:
        row[metric] = chebyshev_metrics[metric] - trapezoid_metrics[metric]
    return row


def _n_from_N(N: float) -> float | int:
    if not np.isfinite(N):
        return np.nan
    return int(N) - 1


def _nearest_value(values: pd.Series, requested_value: float) -> float:
    unique_values = np.asarray(sorted(values.dropna().unique()), dtype=float)
    if unique_values.size == 0:
        return float(requested_value)
    return float(unique_values[np.argmin(np.abs(unique_values - requested_value))])


def _contour_metric_grid(
    axis: plt.Axes,
    data: pd.DataFrame,
    *,
    x_column: str,
    y_column: str,
    metric: str,
    cmap: str,
) -> None:
    if data.empty:
        axis.text(0.5, 0.5, "No valid N <= m", ha="center", va="center")
        return

    grid = data.pivot_table(index=y_column, columns=x_column, values=metric)
    x_values = grid.columns.to_numpy(dtype=float)
    y_values = grid.index.to_numpy(dtype=float)
    z_values = np.ma.masked_invalid(grid.to_numpy(dtype=float))
    if z_values.count() == 0:
        axis.text(0.5, 0.5, "No finite values", ha="center", va="center")
        return

    finite_abs_max = float(np.max(np.abs(z_values.compressed())))
    if finite_abs_max == 0.0:
        finite_abs_max = 1.0
    color_map = plt.get_cmap(cmap).copy()
    color_map.set_bad("white")
    levels = np.linspace(-finite_abs_max, finite_abs_max, 21)
    contour = axis.contourf(
        x_values,
        y_values,
        z_values,
        levels=levels,
        cmap=color_map,
        norm=TwoSlopeNorm(vcenter=0.0, vmin=-finite_abs_max, vmax=finite_abs_max),
        extend="both",
    )
    axis.figure.colorbar(contour, ax=axis, label="Cheb2 - trapezoid")


def _function_subset(comparisons: pd.DataFrame, function_name: str) -> pd.DataFrame:
    return comparisons[comparisons["function"] == function_name].copy()


def _selected_m_values(
    data: pd.DataFrame,
    selected_m_values: Sequence[int] | None,
) -> list[int]:
    if selected_m_values is None:
        return [int(value) for value in sorted(data["m"].dropna().unique())]
    available = set(int(value) for value in data["m"].dropna().unique())
    return [int(value) for value in selected_m_values if int(value) in available]


def _noise_band_labels(values: pd.Series, band_count: int) -> dict[float, str]:
    unique_values = np.asarray(sorted(values.dropna().unique()), dtype=float)
    if unique_values.size == 0:
        return {}
    groups = np.array_split(unique_values, min(max(1, band_count), unique_values.size))
    labels: dict[float, str] = {}
    for group_index, group in enumerate(groups):
        if len(groups) <= len(_NOISE_BAND_NAMES):
            name = _NOISE_BAND_NAMES[group_index]
        else:
            name = f"noise band {group_index + 1}"
        for value in group:
            labels[float(value)] = name
    return labels


def _noise_band_colors(count: int) -> tuple[str, ...]:
    if count <= len(_NOISE_BAND_COLORS):
        return _NOISE_BAND_COLORS[:count]
    return tuple(
        _NOISE_BAND_COLORS[index % len(_NOISE_BAND_COLORS)] for index in range(count)
    )


def _validate_percentile_range(
    percentile_range: tuple[float, float] | None,
) -> None:
    if percentile_range is None:
        return
    low, high = percentile_range
    if not (0.0 <= low < high <= 100.0):
        raise ValueError("percentile range must satisfy 0 <= low < high <= 100")


def _robust_axis_range(
    value_groups: Sequence[np.ndarray],
    percentile_range: tuple[float, float] | None,
) -> list[float] | None:
    if percentile_range is None:
        return None
    finite_values = [
        np.asarray(values, dtype=float)[np.isfinite(values)] for values in value_groups
    ]
    finite_values = [values for values in finite_values if values.size > 0]
    if not finite_values:
        return None

    values = np.concatenate(finite_values)
    low, high = np.percentile(values, percentile_range)
    low = min(float(low), 0.0)
    high = max(float(high), 0.0)
    if np.isclose(low, high):
        margin = max(1.0, 0.1 * abs(low))
    else:
        margin = 0.05 * (high - low)
    return [low - margin, high + margin]


def _axis_range_advantages(
    band_summary: pd.DataFrame,
    y_range_min_N: int | None,
) -> np.ndarray:
    if y_range_min_N is None:
        return band_summary["advantage"].to_numpy(dtype=float)
    focused = band_summary[band_summary["N"] >= y_range_min_N]
    if focused.empty:
        return band_summary["advantage"].to_numpy(dtype=float)
    return focused["advantage"].to_numpy(dtype=float)


def _median_advantage_by_N(data: pd.DataFrame, metric: str) -> pd.Series:
    advantage = data.assign(advantage=-data[metric])
    return advantage.groupby("N")["advantage"].median().sort_index()


def _best_N_for_metric(
    data: pd.DataFrame, metric: str
) -> dict[str, float | int] | None:
    summary = _median_advantage_by_N(data, metric)
    if summary.empty:
        return None
    best_N = _best_N_from_advantage(summary)
    return {"N": int(best_N), "advantage": float(summary.loc[best_N])}


def _best_N_from_advantage(advantage_by_N: pd.Series) -> int:
    candidates = (
        advantage_by_N.dropna()
        .rename("advantage")
        .reset_index()
        .sort_values(["advantage", "N"], ascending=[False, True])
    )
    if candidates.empty:
        raise ValueError("No finite advantage values")
    return int(candidates.iloc[0]["N"])


def _best_N_from_score(score_by_N: pd.Series) -> int:
    candidates = (
        score_by_N.dropna()
        .rename("score")
        .reset_index()
        .sort_values(["score", "N"], ascending=[True, True])
    )
    if candidates.empty:
        raise ValueError("No finite score values")
    return int(candidates.iloc[0]["N"])


def _ascii_sparkline(values: np.ndarray) -> str:
    glyphs = "._:-=+*#%@"
    value_array = np.asarray(values, dtype=float)
    finite = np.isfinite(value_array)
    if not np.any(finite):
        return ""
    finite_values = value_array[finite]
    minimum = float(np.min(finite_values))
    maximum = float(np.max(finite_values))
    if np.isclose(maximum, minimum):
        scaled = np.full(value_array.shape, len(glyphs) // 2, dtype=int)
    else:
        scaled = np.zeros(value_array.shape, dtype=int)
        scaled[finite] = np.round(
            (value_array[finite] - minimum) / (maximum - minimum) * (len(glyphs) - 1)
        ).astype(int)
        scaled = np.clip(scaled, 0, len(glyphs) - 1)
    return "".join(
        glyphs[index] if is_finite else " " for index, is_finite in zip(scaled, finite)
    )


def _empty_plotly_figure(message: str) -> go.Figure:
    fig = go.Figure()
    fig.add_annotation(text=message, x=0.5, y=0.5, showarrow=False)
    fig.update_layout(template="plotly_white", height=260)
    return fig
