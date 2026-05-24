"""Reusable scalar quadrature experiments using GTSAM Chebyshev2."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

import gtsam
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import TwoSlopeNorm


DEFAULT_INTERVAL = (0.0, 1.0)
METRIC_COLUMNS = ("end_error", "rmse_error", "max_error")
METRIC_LABELS = {
    "end_error": "Final integration error",
    "rmse_error": "RMSE integration error",
    "max_error": "Absolute max integration error",
}


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
    node_count: int
    interval: tuple[float, float]
    fit_solver: np.ndarray
    final_weights: np.ndarray
    curve_weights: np.ndarray

    @classmethod
    def build(
        cls,
        sample_times: np.ndarray,
        evaluation_times: np.ndarray,
        node_count: int,
        interval: tuple[float, float],
    ) -> "_ChebyshevExperimentPlan":
        a, b = interval
        weight_matrix = np.asarray(
            gtsam.Chebyshev2.WeightMatrix(node_count, sample_times, a, b),
            dtype=float,
        )
        final_weights = np.asarray(
            gtsam.Chebyshev2.IntegrationWeights(node_count, a, b),
            dtype=float,
        ).reshape(node_count)
        integration_matrix = np.asarray(
            gtsam.Chebyshev2.IntegrationMatrix(node_count, a, b),
            dtype=float,
        )
        integral_evaluation_matrix = np.asarray(
            gtsam.Chebyshev2.WeightMatrix(
                node_count + 1, evaluation_times, a, b
            ),
            dtype=float,
        )
        curve_weights = integral_evaluation_matrix @ integration_matrix
        return cls(
            sample_times=sample_times,
            evaluation_times=evaluation_times,
            node_count=node_count,
            interval=interval,
            fit_solver=np.linalg.pinv(weight_matrix),
            final_weights=final_weights,
            curve_weights=curve_weights,
        )

    def fit_node_values(self, sample_values: np.ndarray) -> np.ndarray:
        values = np.asarray(sample_values, dtype=float)
        if values.ndim == 1:
            return self.fit_solver @ values
        return values @ self.fit_solver.T

    def integral_curves(self, sample_values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
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
    node_count: int,
    interval: tuple[float, float] = DEFAULT_INTERVAL,
) -> tuple[np.ndarray, np.ndarray]:
    """Fit scalar samples with GTSAM Chebyshev2 and evaluate integral curves."""

    sample_time_array = np.asarray(sample_times, dtype=float)
    evaluation_time_array = np.asarray(evaluation_times, dtype=float)
    plan = _ChebyshevExperimentPlan.build(
        sample_time_array, evaluation_time_array, int(node_count), interval
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
    """Create a scalar function from values at GTSAM Chebyshev2 nodes."""

    values = np.asarray(node_values, dtype=float).reshape(-1)
    node_count = values.size
    a, b = interval
    integration_matrix = np.asarray(
        gtsam.Chebyshev2.IntegrationMatrix(node_count, a, b),
        dtype=float,
    )
    antiderivative_nodes = integration_matrix @ values

    def evaluate(times: np.ndarray) -> np.ndarray:
        time_array = np.asarray(times, dtype=float)
        weights = np.asarray(
            gtsam.Chebyshev2.WeightMatrix(node_count, time_array, a, b),
            dtype=float,
        )
        return weights @ values

    def antiderivative(times: np.ndarray) -> np.ndarray:
        time_array = np.asarray(times, dtype=float)
        weights = np.asarray(
            gtsam.Chebyshev2.WeightMatrix(node_count + 1, time_array, a, b),
            dtype=float,
        )
        return weights @ antiderivative_nodes

    return ScalarFunction(name=name, value=evaluate, antiderivative=antiderivative)


def run_scalar_monte_carlo(
    functions: Sequence[ScalarFunction],
    sample_counts: Iterable[int],
    chebyshev_node_counts: Iterable[int],
    noise_fractions: Iterable[float],
    *,
    num_seeds: int = 100,
    seed: int = 0,
    interval: tuple[float, float] = DEFAULT_INTERVAL,
    evaluation_count: int = 201,
) -> ScalarMonteCarloResult:
    """Run scalar noisy integration comparisons over a Monte Carlo grid."""

    a, b = interval
    evaluation_times = np.linspace(a, b, evaluation_count)
    sample_counts = [int(sample_count) for sample_count in sample_counts]
    chebyshev_node_counts = [
        int(node_count) for node_count in chebyshev_node_counts
    ]
    noise_fractions = [float(noise_fraction) for noise_fraction in noise_fractions]

    method_rows: list[dict[str, float | int | str]] = []
    comparison_rows: list[dict[str, float | int | str]] = []
    plan_cache: dict[tuple[int, int], _ChebyshevExperimentPlan] = {}

    for function_index, function in enumerate(functions):
        true_curve = _true_integral_curve(function, evaluation_times, interval)
        true_final = float(true_curve[-1])
        value_range = _function_range(function, interval)

        for sample_count in sample_counts:
            sample_times = np.linspace(a, b, sample_count)
            clean_samples = np.asarray(function.value(sample_times), dtype=float)
            standard_noises = np.random.default_rng(
                seed + 1000003 * function_index + 1009 * sample_count
            ).normal(size=(num_seeds, sample_count))
            clean_trapezoid_curve = _trapezoid_integral_curves(
                sample_times, clean_samples, evaluation_times
            )[0]
            noise_trapezoid_curves = _trapezoid_integral_curves(
                sample_times, standard_noises, evaluation_times
            )

            valid_plans = {}
            for node_count in chebyshev_node_counts:
                if node_count > sample_count:
                    continue
                cache_key = (sample_count, node_count)
                if cache_key not in plan_cache:
                    plan_cache[cache_key] = _ChebyshevExperimentPlan.build(
                        sample_times, evaluation_times, node_count, interval
                    )
                valid_plans[node_count] = plan_cache[cache_key]

            chebyshev_components = {}
            for node_count, plan in valid_plans.items():
                clean_final, clean_curve = plan.integral_curves(clean_samples)
                noise_final, noise_curves = plan.integral_curves(standard_noises)
                chebyshev_components[node_count] = (
                    float(clean_final[0]),
                    clean_curve[0],
                    noise_final,
                    noise_curves,
                )

            for noise_fraction in noise_fractions:
                noise_std = noise_fraction * value_range
                trapezoid_curves = (
                    clean_trapezoid_curve[None, :]
                    + noise_std * noise_trapezoid_curves
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
                        sample_count,
                        np.nan,
                        trapezoid_metrics,
                    )
                )

                for node_count, components in chebyshev_components.items():
                    (
                        clean_chebyshev_final,
                        clean_chebyshev_curve,
                        noise_chebyshev_final,
                        noise_chebyshev_curves,
                    ) = components
                    chebyshev_final = (
                        clean_chebyshev_final
                        + noise_std * noise_chebyshev_final
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
                            sample_count,
                            node_count,
                            chebyshev_metrics,
                        )
                    )
                    comparison_rows.append(
                        _comparison_row(
                            function.name,
                            noise_fraction,
                            noise_std,
                            sample_count,
                            node_count,
                            chebyshev_metrics,
                            trapezoid_metrics,
                        )
                    )

    return ScalarMonteCarloResult(
        method_metrics=pd.DataFrame(method_rows),
        comparisons=pd.DataFrame(comparison_rows),
    )


def plot_fixed_node_comparison(
    comparisons: pd.DataFrame,
    function_name: str,
    *,
    selected_nodes: Sequence[int] = (2, 9, 17, 24),
    metrics: Sequence[str] = METRIC_COLUMNS,
    x_column: str = "noise_std",
    cmap: str = "coolwarm",
) -> plt.Figure:
    """Plot metric differences over noise and sample count for fixed M values."""

    fig, axes = plt.subplots(
        len(metrics),
        len(selected_nodes),
        figsize=(3.6 * len(selected_nodes), 3.0 * len(metrics)),
        squeeze=False,
        constrained_layout=True,
    )
    subset = comparisons[comparisons["function"] == function_name]
    for row_index, metric in enumerate(metrics):
        for col_index, node_count in enumerate(selected_nodes):
            axis = axes[row_index, col_index]
            grid_subset = subset[subset["chebyshev_nodes"] == node_count]
            _contour_metric_grid(
                axis,
                grid_subset,
                x_column=x_column,
                y_column="sample_count",
                metric=metric,
                cmap=cmap,
            )
            axis.set_title(f"{METRIC_LABELS.get(metric, metric)}, M={node_count}")
            axis.set_xlabel("Noise std. dev. sigma")
            axis.set_ylabel("Number of samples n")
    fig.suptitle(
        f"{function_name}: Chebyshev2 minus trapezoid (negative is better)",
        fontsize=14,
    )
    return fig


def plot_fixed_sample_comparison(
    comparisons: pd.DataFrame,
    function_name: str,
    *,
    selected_sample_counts: Sequence[int] = (10, 20, 30, 40, 50),
    metrics: Sequence[str] = METRIC_COLUMNS,
    x_column: str = "noise_std",
    cmap: str = "coolwarm",
    show_sqrt_sample_count: bool = False,
) -> plt.Figure:
    """Plot metric differences over noise and M for fixed sample counts."""

    fig, axes = plt.subplots(
        len(metrics),
        len(selected_sample_counts),
        figsize=(3.6 * len(selected_sample_counts), 3.0 * len(metrics)),
        squeeze=False,
        constrained_layout=True,
    )
    subset = comparisons[comparisons["function"] == function_name]
    for row_index, metric in enumerate(metrics):
        for col_index, sample_count in enumerate(selected_sample_counts):
            axis = axes[row_index, col_index]
            grid_subset = subset[subset["sample_count"] == sample_count]
            _contour_metric_grid(
                axis,
                grid_subset,
                x_column=x_column,
                y_column="chebyshev_nodes",
                metric=metric,
                cmap=cmap,
            )
            axis.set_title(f"{METRIC_LABELS.get(metric, metric)}, N={sample_count}")
            axis.set_xlabel("Noise std. dev. sigma")
            axis.set_ylabel("Chebyshev nodes m")
            if show_sqrt_sample_count:
                axis.axhline(
                    np.sqrt(sample_count),
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
    """Plot one metric over sample count and M across functions/noise levels."""

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
                x_column="sample_count",
                y_column="chebyshev_nodes",
                metric=metric,
                cmap=cmap,
            )
            axis.set_title(
                f"{function_name}, noise fraction={noise_fraction:.3g}"
            )
            axis.set_xlabel("Number of samples n")
            axis.set_ylabel("Chebyshev nodes M")
    fig.suptitle(
        f"{METRIC_LABELS.get(metric, metric)}: Chebyshev2 minus trapezoid",
        fontsize=14,
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
    segment_areas = 0.5 * segment_widths[None, :] * (
        values[:, :-1] + values[:, 1:]
    )
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
    curves = cumulative[:, indices] + 0.5 * dt * (
        left_values + interpolated_values
    )

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
    sample_count: int,
    chebyshev_nodes: float,
    metrics: dict[str, float],
) -> dict[str, float | int | str]:
    row: dict[str, float | int | str] = {
        "function": function_name,
        "method": method,
        "noise_fraction": noise_fraction,
        "noise_std": noise_std,
        "sample_count": sample_count,
        "chebyshev_nodes": chebyshev_nodes,
    }
    row.update(metrics)
    return row


def _comparison_row(
    function_name: str,
    noise_fraction: float,
    noise_std: float,
    sample_count: int,
    node_count: int,
    chebyshev_metrics: dict[str, float],
    trapezoid_metrics: dict[str, float],
) -> dict[str, float | int | str]:
    row: dict[str, float | int | str] = {
        "function": function_name,
        "noise_fraction": noise_fraction,
        "noise_std": noise_std,
        "sample_count": sample_count,
        "chebyshev_nodes": node_count,
    }
    for metric in METRIC_COLUMNS:
        row[metric] = chebyshev_metrics[metric] - trapezoid_metrics[metric]
    return row


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
        axis.text(0.5, 0.5, "No valid n >= M", ha="center", va="center")
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
