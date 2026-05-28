"""Spectral basis fits and spectrogram views for merged EuRoC CSVs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

import imuFactors.euroc as euroc
import imuFactors.spectral as spectral


WINDOW_SECONDS = 1.0
DEFAULT_SIGNAL_GROUP = euroc.DEFAULT_SIGNAL_GROUP
DEFAULT_BASIS = spectral.DEFAULT_BASIS


@dataclass
class SpectrogramFit:
    """Fixed-interval spectral basis fits for a selected signal block."""

    path: Path
    basis: str
    N: int
    columns: list[str]
    dataframe: pd.DataFrame
    time: np.ndarray
    starts: np.ndarray
    steps_per_window: int
    m: int
    window_seconds: float
    dt: float
    basis_coordinates: np.ndarray
    weight_matrix: np.ndarray
    coeffs: np.ndarray
    coeffs_standardized: np.ndarray
    coeff_energy: np.ndarray
    samples: np.ndarray
    reconstructed: np.ndarray
    rmse: np.ndarray
    center: np.ndarray
    scale: np.ndarray
    activity: np.ndarray
    high_order_ratio: np.ndarray
    lambda1: float = 0.0

    @property
    def n(self) -> int:
        """Polynomial degree for Chebyshev bases, with ``N = n + 1``."""
        return self.N - 1

    @property
    def max_harmonic(self) -> int:
        """Maximum Fourier harmonic represented by ``N`` fitted parameters."""
        return self.N // 2

    @property
    def basis_name(self) -> str:
        """Human-readable basis name."""
        return spectral.basis_display_name(self.basis)

    @property
    def window_count(self) -> int:
        """Number of complete fixed-duration intervals."""
        return int(self.starts.size)


def robust_center_scale(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Compute robust per-column center and scale for unit-comparable spectra."""
    center = np.nanmedian(values, axis=0)
    mad = np.nanmedian(np.abs(values - center), axis=0)
    robust_sigma = 1.4826 * mad
    standard_sigma = np.nanstd(values, axis=0)
    scale = np.where(robust_sigma > 1e-12, robust_sigma, standard_sigma)
    return center, np.where(scale > 1e-12, scale, 1.0)


def interval_window_starts(
    time: Sequence[float] | np.ndarray,
    *,
    window_seconds: float = WINDOW_SECONDS,
) -> tuple[np.ndarray, int, float]:
    """Return starts for complete closed fixed-duration windows.

    For a 1.0-second interval at 200 Hz, this yields 201 samples per closed
    window and advances by 200 samples, so adjacent windows share one endpoint.
    """
    time_array = np.asarray(time, dtype=float)
    dt = float(np.median(np.diff(time_array)))
    steps_per_window = int(round(window_seconds / dt))
    if steps_per_window < 1:
        raise ValueError("Window duration is shorter than one sample interval")

    starts = np.arange(0, len(time_array) - steps_per_window, steps_per_window)
    durations = time_array[starts + steps_per_window] - time_array[starts]
    tolerance = max(1e-6, 0.1 * dt)
    complete = starts[np.abs(durations - window_seconds) <= tolerance]
    if complete.size == 0:
        raise ValueError("No complete fixed-duration windows found")
    return complete, steps_per_window, dt


def fit_spectral_windows(
    path: str | Path,
    N: int,
    *,
    basis: str = DEFAULT_BASIS,
    signal_group: str = DEFAULT_SIGNAL_GROUP,
    columns: Sequence[str] | None = None,
    window_seconds: float = WINDOW_SECONDS,
    lambda1: float = 0.0,
) -> SpectrogramFit:
    """Fit ``N`` spectral basis parameters for each complete interval."""
    csv_path = Path(path)
    normalized_basis = spectral.normalize_basis_name(basis)
    dataframe = euroc.load_euroc_csv(csv_path)
    groups = euroc.available_signal_groups(dataframe)
    selected_columns = (
        list(columns) if columns is not None else groups.get(signal_group)
    )
    if selected_columns is None:
        raise KeyError(
            f"Unknown signal group {signal_group!r}; available: {sorted(groups)}"
        )

    selected_columns = [
        column for column in selected_columns if column in dataframe.columns
    ]
    if not selected_columns:
        raise ValueError("No selected columns are present in the dataframe")

    time = dataframe["t"].to_numpy(dtype=float)
    starts, steps_per_window, dt = interval_window_starts(
        time, window_seconds=window_seconds
    )
    m = steps_per_window + 1
    plan = spectral.basis_plan(
        int(N),
        m,
        basis=normalized_basis,
        window_seconds=window_seconds,
        lambda1=lambda1,
    )

    values = dataframe[selected_columns].to_numpy(dtype=float)
    center, scale = robust_center_scale(values)
    samples, reconstructed, coeffs = _fit_windows(values, starts, m, plan)
    rmse = np.sqrt(np.mean((reconstructed - samples) ** 2, axis=1))
    coeffs_standardized, coeff_energy = standardized_coefficient_energy(
        coeffs, center, scale
    )
    activity = standardized_window_activity(samples, center, scale)
    high_order_ratio = high_order_energy_ratio(coeff_energy)

    return SpectrogramFit(
        path=csv_path,
        basis=normalized_basis,
        N=int(N),
        columns=selected_columns,
        dataframe=dataframe,
        time=time,
        starts=starts,
        steps_per_window=steps_per_window,
        m=m,
        window_seconds=window_seconds,
        dt=dt,
        basis_coordinates=plan.coordinates,
        weight_matrix=plan.weight_matrix,
        coeffs=coeffs,
        coeffs_standardized=coeffs_standardized,
        coeff_energy=coeff_energy,
        samples=samples,
        reconstructed=reconstructed,
        rmse=rmse,
        center=center,
        scale=scale,
        activity=activity,
        high_order_ratio=high_order_ratio,
        lambda1=plan.lambda1,
    )


def _fit_windows(
    values: np.ndarray,
    starts: np.ndarray,
    m: int,
    plan: spectral.BasisPlan,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    window_count = int(starts.size)
    component_count = int(values.shape[1])
    N = int(plan.N)
    samples = np.empty((window_count, m, component_count), dtype=float)
    reconstructed = np.empty_like(samples)
    coeffs = np.empty((window_count, N, component_count), dtype=float)

    for window_index, start in enumerate(starts):
        stop = int(start) + m
        y = values[int(start) : stop, :]
        coefficient_matrix = plan.fit(y)
        samples[window_index] = y
        coeffs[window_index] = coefficient_matrix
        reconstructed[window_index] = plan.reconstruct(coefficient_matrix)
    return samples, reconstructed, coeffs


def standardized_coefficient_energy(
    coeffs: np.ndarray,
    center: np.ndarray,
    scale: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return standardized coefficients and RMS energy per basis index."""
    standardized = coeffs / scale.reshape(1, 1, -1)
    standardized[:, 0, :] -= (center / scale).reshape(1, -1)
    energy = np.sqrt(np.mean(standardized * standardized, axis=2))
    return standardized, energy


def standardized_window_activity(
    samples: np.ndarray,
    center: np.ndarray,
    scale: np.ndarray,
) -> np.ndarray:
    """Return RMS standardized within-window variance for each interval."""
    standardized = (samples - center.reshape(1, 1, -1)) / scale.reshape(1, 1, -1)
    return np.sqrt(np.mean(np.var(standardized, axis=1), axis=1))


def high_order_energy_ratio(coeff_energy: np.ndarray) -> np.ndarray:
    """Return fraction of non-DC energy in the upper half of basis orders."""
    N = int(coeff_energy.shape[1])
    high_start = max(2, N // 2)
    high_energy = np.sum(coeff_energy[:, high_start:], axis=1)
    non_dc_energy = np.sum(coeff_energy[:, 1:], axis=1)
    return high_energy / np.maximum(non_dc_energy, 1e-12)


def normalized_rmse(result: SpectrogramFit) -> np.ndarray:
    """Return per-window RMS fit error after component scale normalization."""
    return np.sqrt(np.mean((result.rmse / result.scale.reshape(1, -1)) ** 2, axis=1))


def aggressive_window_scores(result: SpectrogramFit) -> np.ndarray:
    """Return the score used to rank high-activity, high-order windows."""
    return _zscore(result.activity) + 1.5 * _zscore(result.high_order_ratio)


def characteristic_windows(result: SpectrogramFit) -> dict[str, int]:
    """Choose one easy and one aggressive interval from fit diagnostics."""
    easy_score = (
        _zscore(result.activity)
        + _zscore(result.high_order_ratio)
        + _zscore(normalized_rmse(result))
    )
    aggressive_score = aggressive_window_scores(result)
    easy = int(np.nanargmin(easy_score))
    aggressive = int(np.nanargmax(aggressive_score))
    return {"easy": easy, "aggressive": aggressive}


def window_start_seconds(result: SpectrogramFit, window_index: int) -> float:
    """Return window start time relative to the beginning of the file."""
    return float(result.time[result.starts[window_index]] - result.time[0])


def basis_order_summary(result: SpectrogramFit) -> str:
    """Return a compact basis-order description for tables."""
    return spectral.basis_order_summary(result.basis, result.N)


def basis_tick_labels(result: SpectrogramFit) -> list[str]:
    """Return coefficient labels for the fitted basis."""
    return spectral.basis_tick_labels(result.basis, result.N)


def basis_axis_title(result: SpectrogramFit) -> str:
    """Return an x-axis title matching the fitted parameterization."""
    return spectral.basis_axis_title(result.basis)


def summary_table(result: SpectrogramFit) -> pd.DataFrame:
    """Return a compact dataframe describing the current fit."""
    duration = float(result.time[-1] - result.time[0])
    rows = [
        ("file", result.path.name),
        ("basis", result.basis_name),
        ("selected columns", ", ".join(result.columns)),
        ("N", result.N),
        ("n", result.n),
        ("basis order", basis_order_summary(result)),
        ("lambda1 p' penalty", result.lambda1),
        ("interval length [s]", result.window_seconds),
        ("complete intervals", result.window_count),
        ("m samples per closed window", result.m),
        ("median dt", result.dt),
        ("effective rate", 1.0 / result.dt),
        ("file duration", duration),
    ]
    return pd.DataFrame(rows, columns=["quantity", "value"])


def interval_metrics_table(
    result: SpectrogramFit,
    selected: dict[str, int],
) -> pd.DataFrame:
    """Return diagnostics for selected interval labels."""
    nrms = normalized_rmse(result)
    return pd.DataFrame(
        [
            {
                "label": label,
                "window_index": index,
                "start_s": window_start_seconds(result, index),
                "activity": result.activity[index],
                "high_order_ratio": result.high_order_ratio[index],
                "normalized_rmse": nrms[index],
            }
            for label, index in selected.items()
        ]
    )


def plot_window_characteristics(
    result: SpectrogramFit,
    selected: dict[str, int] | None = None,
) -> go.Figure:
    """Plot diagnostics used to identify easy and aggressive windows."""
    selected = characteristic_windows(result) if selected is None else selected
    starts_s = np.array(
        [window_start_seconds(result, i) for i in range(result.window_count)]
    )
    nrms = normalized_rmse(result)
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=result.activity,
            y=result.high_order_ratio,
            mode="markers",
            text=[
                f"window {i}, t={starts_s[i]:.1f}s" for i in range(result.window_count)
            ],
            marker=dict(
                size=8,
                color=nrms,
                colorscale="Turbo",
                showscale=True,
                colorbar_title="norm RMSE",
            ),
            name="all windows",
        )
    )
    marker_symbols = {"easy": "circle-open", "aggressive": "x"}
    for label, index in selected.items():
        fig.add_trace(
            go.Scatter(
                x=[result.activity[index]],
                y=[result.high_order_ratio[index]],
                mode="markers+text",
                text=[label],
                textposition="top center",
                marker=dict(
                    size=14,
                    symbol=marker_symbols.get(label, "diamond"),
                    color="black",
                    line=dict(width=2),
                ),
                name=label,
            )
        )
    fig.update_layout(
        title="Window characteristics used to choose examples",
        xaxis_title="standardized within-window activity",
        yaxis_title="high-order coefficient energy ratio",
        height=450,
        margin=dict(l=70, r=40, t=70, b=60),
    )
    return fig


def plot_interval_fit(
    result: SpectrogramFit,
    window_index: int,
    label: str = "selected",
    *,
    max_components: int = 6,
) -> go.Figure:
    """Plot original samples and spectral basis fit for one interval."""
    component_count = min(max_components, len(result.columns))
    columns = result.columns[:component_count]
    seconds = np.linspace(0.0, result.window_seconds, result.m)
    fig = make_subplots(
        rows=component_count,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.025,
        subplot_titles=columns,
    )
    for row, column in enumerate(columns, start=1):
        component = result.columns.index(column)
        fig.add_trace(
            go.Scatter(
                x=seconds,
                y=result.samples[window_index, :, component],
                mode="markers",
                marker=dict(size=4),
                name=f"{column} samples",
                legendgroup=column,
            ),
            row=row,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=seconds,
                y=result.reconstructed[window_index, :, component],
                mode="lines",
                line=dict(width=2),
                name=f"{column} {result.basis_name} fit",
                legendgroup=column,
            ),
            row=row,
            col=1,
        )
        fig.update_yaxes(title_text=column, row=row, col=1)
    fig.update_xaxes(title_text="seconds inside window", row=component_count, col=1)
    fig.update_layout(
        title=(
            f"{label.title()} interval fit: window {window_index}, "
            f"start {window_start_seconds(result, window_index):.1f}s"
        ),
        height=max(320, 180 * component_count),
        margin=dict(l=70, r=30, t=80, b=60),
    )
    return fig


def plot_interval_coefficients(
    result: SpectrogramFit,
    window_index: int,
    label: str = "selected",
    *,
    standardized: bool = False,
) -> go.Figure:
    """Plot spectral weights for all components in one interval."""
    matrix = (
        result.coeffs_standardized[window_index].T
        if standardized
        else result.coeffs[window_index].T
    )
    zmax = float(np.nanmax(np.abs(matrix))) if matrix.size else 1.0
    if zmax <= 0.0:
        zmax = 1.0
    title_prefix = "standardized" if standardized else "raw"
    fig = go.Figure(
        data=go.Heatmap(
            z=matrix,
            x=basis_tick_labels(result),
            y=result.columns,
            colorscale="RdBu",
            zmid=0.0,
            zmin=-zmax,
            zmax=zmax,
            colorbar_title="weight",
        )
    )
    fig.update_layout(
        title=(
            f"{label.title()} interval {title_prefix} spectral weights: "
            f"window {window_index}, start "
            f"{window_start_seconds(result, window_index):.1f}s"
        ),
        xaxis_title=basis_axis_title(result),
        yaxis_title="signal component",
        height=max(360, 26 * len(result.columns) + 160),
        margin=dict(l=120, r=40, t=80, b=60),
    )
    return fig


def plot_coefficient_spectrogram(
    result: SpectrogramFit,
    *,
    log_scale: bool = True,
) -> go.Figure:
    """Plot the ``window_count x N`` coefficient-energy spectrogram."""
    z = result.coeff_energy
    colorbar_title = "standardized RMS weight"
    if log_scale:
        z = np.log10(z + 1e-12)
        colorbar_title = "log10 standardized RMS weight"
    starts_s = np.array(
        [window_start_seconds(result, i) for i in range(result.window_count)]
    )
    fig = go.Figure(
        data=go.Heatmap(
            z=z,
            x=basis_tick_labels(result),
            y=starts_s,
            colorscale="Viridis",
            colorbar_title=colorbar_title,
        )
    )
    fig.update_layout(
        title="Coefficient spectrogram (window_count x N basis weights)",
        xaxis_title=basis_axis_title(result),
        yaxis_title="window start time [s]",
        yaxis_autorange="reversed",
        height=max(420, min(900, 220 + 3 * result.window_count)),
        margin=dict(l=80, r=40, t=80, b=60),
    )
    return fig


def plot_average_spectra(
    result: SpectrogramFit,
    *,
    part_count: int = 4,
) -> go.Figure:
    """Plot mean coefficient spectra for the whole file and trajectory parts."""
    basis_indices = np.arange(result.N)
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=basis_indices,
            y=np.mean(result.coeff_energy, axis=0),
            mode="lines+markers",
            line=dict(width=4, color="black"),
            name="whole file",
        )
    )
    for label, start, stop in trajectory_parts(result, part_count):
        fig.add_trace(
            go.Scatter(
                x=basis_indices,
                y=np.mean(result.coeff_energy[start:stop], axis=0),
                mode="lines+markers",
                name=label,
            )
        )
    fig.update_layout(
        title="Average standardized coefficient spectra",
        xaxis_title=f"{basis_axis_title(result)} index k",
        yaxis_title="mean RMS coefficient energy across selected components",
        height=450,
        margin=dict(l=80, r=40, t=80, b=60),
    )
    return fig


def plot_average_spectrogram_parts(
    result: SpectrogramFit,
    *,
    part_count: int = 4,
) -> go.Figure:
    """Plot average coefficient spectrogram rows for whole file and parts."""
    rows = [np.mean(result.coeff_energy, axis=0)]
    labels = ["whole file"]
    for label, start, stop in trajectory_parts(result, part_count):
        rows.append(np.mean(result.coeff_energy[start:stop], axis=0))
        labels.append(label)

    fig = go.Figure(
        data=go.Heatmap(
            z=np.log10(np.vstack(rows) + 1e-12),
            x=basis_tick_labels(result),
            y=labels,
            colorscale="Viridis",
            colorbar_title="log10 mean RMS weight",
        )
    )
    fig.update_layout(
        title="Average coefficient spectrograms: whole file and trajectory parts",
        xaxis_title=basis_axis_title(result),
        yaxis_title="trajectory section",
        height=360,
        margin=dict(l=130, r=40, t=80, b=60),
    )
    return fig


def plot_dataset_average_spectrograms(
    results: Mapping[str, SpectrogramFit],
    *,
    log_scale: bool = True,
) -> go.Figure:
    """Compare whole-file average coefficient spectra across datasets."""
    if not results:
        raise ValueError("results must contain at least one dataset")

    labels = list(results.keys())
    first = next(iter(results.values()))
    for label, result in results.items():
        if result.N != first.N:
            raise ValueError(f"{label} has N={result.N}, expected " f"N={first.N}")
        if result.basis != first.basis:
            raise ValueError(
                f"{label} uses basis={result.basis!r}, expected {first.basis!r}"
            )

    rows = np.vstack(
        [np.mean(result.coeff_energy, axis=0) for result in results.values()]
    )
    colorbar_title = "mean RMS weight"
    if log_scale:
        rows = np.log10(rows + 1e-12)
        colorbar_title = "log10 mean RMS weight"

    fig = go.Figure(
        data=go.Heatmap(
            z=rows,
            x=basis_tick_labels(first),
            y=labels,
            colorscale="Viridis",
            colorbar_title=colorbar_title,
        )
    )
    fig.update_layout(
        title="Whole-file average coefficient spectrograms across datasets",
        xaxis_title=basis_axis_title(first),
        yaxis_title="dataset",
        height=max(420, 40 * len(labels) + 180),
        margin=dict(l=100, r=40, t=80, b=60),
    )
    return fig


def trajectory_parts(
    result: SpectrogramFit,
    part_count: int,
) -> list[tuple[str, int, int]]:
    """Return labeled contiguous trajectory sections in window-index space."""
    edges = np.linspace(0, result.window_count, part_count + 1, dtype=int)
    parts = []
    for part in range(part_count):
        start, stop = int(edges[part]), int(edges[part + 1])
        if stop <= start:
            continue
        start_s = window_start_seconds(result, start)
        end_s = window_start_seconds(result, stop - 1) + result.window_seconds
        parts.append((f"part {part + 1}: {start_s:.0f}-{end_s:.0f}s", start, stop))
    return parts


def _zscore(values: np.ndarray) -> np.ndarray:
    sigma = float(np.nanstd(values))
    if sigma <= 1e-12:
        return np.zeros_like(values)
    return (values - float(np.nanmean(values))) / sigma
