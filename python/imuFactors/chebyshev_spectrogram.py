"""Spectral Chebyshev fits and spectrogram views for merged EuRoC CSVs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import gtsam
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots


WINDOW_SECONDS = 1.0
DEFAULT_SIGNAL_GROUP = "imu"

SIGNAL_GROUPS = {
    "imu": ["w_x", "w_y", "w_z", "a_x", "a_y", "a_z"],
    "imu_norms": ["gyro_norm", "accel_norm"],
    "gyro": ["w_x", "w_y", "w_z"],
    "accel": ["a_x", "a_y", "a_z"],
    "position": ["p_x", "p_y", "p_z"],
    "velocity": ["v_x", "v_y", "v_z"],
    "gyro_bias": ["b_w_x", "b_w_y", "b_w_z"],
    "accel_bias": ["b_a_x", "b_a_y", "b_a_z"],
    "quaternion_components": ["q_w", "q_x", "q_y", "q_z"],
    "state_plus_bias": [
        "q_w",
        "q_x",
        "q_y",
        "q_z",
        "v_x",
        "v_y",
        "v_z",
        "p_x",
        "p_y",
        "p_z",
        "b_w_x",
        "b_w_y",
        "b_w_z",
        "b_a_x",
        "b_a_y",
        "b_a_z",
    ],
    "all_numeric": [],
}


@dataclass
class ChebyshevSpectrogramFit:
    """Fixed-interval spectral Chebyshev fits for a selected signal block."""

    path: Path
    coefficient_count: int
    columns: list[str]
    dataframe: pd.DataFrame
    time: np.ndarray
    starts: np.ndarray
    steps_per_window: int
    sample_count: int
    window_seconds: float
    dt: float
    tau: np.ndarray
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

    @property
    def degree(self) -> int:
        """Polynomial degree, where coefficient_count = degree + 1."""
        return self.coefficient_count - 1

    @property
    def window_count(self) -> int:
        """Number of complete fixed-duration intervals."""
        return int(self.starts.size)


def discover_euroc_files(data_dir: str | Path) -> list[Path]:
    """Return sorted merged EuRoC CSV files under ``data_dir``."""
    return sorted(Path(data_dir).glob("euroc_*.csv"))


def load_euroc_csv(
    path: str | Path,
    *,
    continuous_quaternions: bool = True,
    include_imu_norms: bool = True,
) -> pd.DataFrame:
    """Load a merged EuRoC CSV and add useful derived signal columns."""
    dataframe = pd.read_csv(path)
    quat_cols = ["q_w", "q_x", "q_y", "q_z"]
    if continuous_quaternions and all(column in dataframe.columns for column in quat_cols):
        quat = dataframe[quat_cols].to_numpy(dtype=float, copy=True)
        for i in range(1, len(quat)):
            if float(np.dot(quat[i - 1], quat[i])) < 0.0:
                quat[i] *= -1.0
        dataframe.loc[:, quat_cols] = quat
    if include_imu_norms:
        add_imu_norm_columns(dataframe)
    return dataframe


def add_imu_norm_columns(dataframe: pd.DataFrame) -> pd.DataFrame:
    """Add ``gyro_norm`` and ``accel_norm`` columns when IMU axes are present."""
    gyro_columns = ["w_x", "w_y", "w_z"]
    accel_columns = ["a_x", "a_y", "a_z"]
    if all(column in dataframe.columns for column in gyro_columns):
        gyro = dataframe[gyro_columns].to_numpy(dtype=float)
        dataframe["gyro_norm"] = np.linalg.norm(gyro, axis=1)
    if all(column in dataframe.columns for column in accel_columns):
        accel = dataframe[accel_columns].to_numpy(dtype=float)
        dataframe["accel_norm"] = np.linalg.norm(accel, axis=1)
    return dataframe


def available_signal_groups(dataframe: pd.DataFrame) -> dict[str, list[str]]:
    """Return signal groups whose columns are present in ``dataframe``."""
    groups: dict[str, list[str]] = {}
    for name, columns in SIGNAL_GROUPS.items():
        if name == "all_numeric":
            continue
        present = [column for column in columns if column in dataframe.columns]
        if present:
            groups[name] = present
    groups["all_numeric"] = [
        column
        for column in dataframe.columns
        if column != "t" and pd.api.types.is_numeric_dtype(dataframe[column])
    ]
    return groups


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


def one_second_window_starts(
    time: Sequence[float] | np.ndarray,
    *,
    window_seconds: float = WINDOW_SECONDS,
) -> tuple[np.ndarray, int, float]:
    """Compatibility wrapper for ``interval_window_starts``."""
    return interval_window_starts(time, window_seconds=window_seconds)


def chebyshev1_design(
    coefficient_count: int,
    sample_count: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return canonical samples, Chebyshev1 weight matrix, and LS solver."""
    if coefficient_count < 1:
        raise ValueError("coefficient_count must be positive")
    if coefficient_count > sample_count:
        raise ValueError(
            f"coefficient_count={coefficient_count} exceeds the "
            f"{sample_count} samples in a window"
        )
    tau = np.linspace(-1.0, 1.0, sample_count)
    weight_matrix = np.asarray(
        gtsam.Chebyshev1Basis.WeightMatrix(int(coefficient_count), tau),
        dtype=float,
    )
    return tau, weight_matrix, np.linalg.pinv(weight_matrix)


def fit_spectral_chebyshev_windows(
    path: str | Path,
    coefficient_count: int,
    *,
    signal_group: str = DEFAULT_SIGNAL_GROUP,
    columns: Sequence[str] | None = None,
    window_seconds: float = WINDOW_SECONDS,
) -> ChebyshevSpectrogramFit:
    """Fit spectral Chebyshev coefficients for each complete interval."""
    csv_path = Path(path)
    dataframe = load_euroc_csv(csv_path)
    groups = available_signal_groups(dataframe)
    selected_columns = list(columns) if columns is not None else groups.get(signal_group)
    if selected_columns is None:
        raise KeyError(
            f"Unknown signal group {signal_group!r}; available: {sorted(groups)}"
        )

    selected_columns = [column for column in selected_columns if column in dataframe.columns]
    if not selected_columns:
        raise ValueError("No selected columns are present in the dataframe")

    time = dataframe["t"].to_numpy(dtype=float)
    starts, steps_per_window, dt = interval_window_starts(
        time, window_seconds=window_seconds
    )
    sample_count = steps_per_window + 1
    tau, weight_matrix, solver = chebyshev1_design(
        int(coefficient_count), sample_count
    )

    values = dataframe[selected_columns].to_numpy(dtype=float)
    center, scale = robust_center_scale(values)
    samples, reconstructed, coeffs = _fit_windows(
        values, starts, sample_count, solver, weight_matrix
    )
    rmse = np.sqrt(np.mean((reconstructed - samples) ** 2, axis=1))
    coeffs_standardized, coeff_energy = standardized_coefficient_energy(
        coeffs, center, scale
    )
    activity = standardized_window_activity(samples, center, scale)
    high_order_ratio = high_order_energy_ratio(coeff_energy)

    return ChebyshevSpectrogramFit(
        path=csv_path,
        coefficient_count=int(coefficient_count),
        columns=selected_columns,
        dataframe=dataframe,
        time=time,
        starts=starts,
        steps_per_window=steps_per_window,
        sample_count=sample_count,
        window_seconds=window_seconds,
        dt=dt,
        tau=tau,
        weight_matrix=weight_matrix,
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
    )


def _fit_windows(
    values: np.ndarray,
    starts: np.ndarray,
    sample_count: int,
    solver: np.ndarray,
    weight_matrix: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    window_count = int(starts.size)
    component_count = int(values.shape[1])
    coefficient_count = int(solver.shape[0])
    samples = np.empty((window_count, sample_count, component_count), dtype=float)
    reconstructed = np.empty_like(samples)
    coeffs = np.empty((window_count, coefficient_count, component_count), dtype=float)

    for window_index, start in enumerate(starts):
        stop = int(start) + sample_count
        y = values[int(start) : stop, :]
        coefficient_matrix = solver @ y
        samples[window_index] = y
        coeffs[window_index] = coefficient_matrix
        reconstructed[window_index] = weight_matrix @ coefficient_matrix
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
    coefficient_count = int(coeff_energy.shape[1])
    high_start = max(2, coefficient_count // 2)
    high_energy = np.sum(coeff_energy[:, high_start:], axis=1)
    non_dc_energy = np.sum(coeff_energy[:, 1:], axis=1)
    return high_energy / np.maximum(non_dc_energy, 1e-12)


def normalized_rmse(result: ChebyshevSpectrogramFit) -> np.ndarray:
    """Return per-window RMS fit error after component scale normalization."""
    return np.sqrt(np.mean((result.rmse / result.scale.reshape(1, -1)) ** 2, axis=1))


def characteristic_windows(result: ChebyshevSpectrogramFit) -> dict[str, int]:
    """Choose one easy and one aggressive interval from fit diagnostics."""
    easy_score = (
        _zscore(result.activity)
        + _zscore(result.high_order_ratio)
        + _zscore(normalized_rmse(result))
    )
    aggressive_score = _zscore(result.activity) + 1.5 * _zscore(
        result.high_order_ratio
    )
    easy = int(np.nanargmin(easy_score))
    aggressive = int(np.nanargmax(aggressive_score))
    if aggressive == easy and result.window_count > 1:
        aggressive = int(np.nanargmax(result.activity))
    return {"easy": easy, "aggressive": aggressive}


def window_start_seconds(result: ChebyshevSpectrogramFit, window_index: int) -> float:
    """Return window start time relative to the beginning of the file."""
    return float(result.time[result.starts[window_index]] - result.time[0])


def summary_table(result: ChebyshevSpectrogramFit) -> pd.DataFrame:
    """Return a compact dataframe describing the current fit."""
    duration = float(result.time[-1] - result.time[0])
    rows = [
        ("file", result.path.name),
        ("selected columns", ", ".join(result.columns)),
        ("N", result.coefficient_count),
        ("polynomial degree", result.degree),
        ("interval length [s]", result.window_seconds),
        ("complete intervals", result.window_count),
        ("samples per closed window", result.sample_count),
        ("median dt", result.dt),
        ("effective rate", 1.0 / result.dt),
        ("file duration", duration),
    ]
    return pd.DataFrame(rows, columns=["quantity", "value"])


def interval_metrics_table(
    result: ChebyshevSpectrogramFit,
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
    result: ChebyshevSpectrogramFit,
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
                f"window {i}, t={starts_s[i]:.1f}s"
                for i in range(result.window_count)
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
    result: ChebyshevSpectrogramFit,
    window_index: int,
    label: str = "selected",
    *,
    max_components: int = 6,
) -> go.Figure:
    """Plot original samples and spectral Chebyshev fit for one interval."""
    component_count = min(max_components, len(result.columns))
    columns = result.columns[:component_count]
    seconds = np.linspace(0.0, result.window_seconds, result.sample_count)
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
                name=f"{column} Cheb fit",
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
    result: ChebyshevSpectrogramFit,
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
            x=[f"T{k}" for k in range(result.coefficient_count)],
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
        xaxis_title="Chebyshev spectral basis",
        yaxis_title="signal component",
        height=max(360, 26 * len(result.columns) + 160),
        margin=dict(l=120, r=40, t=80, b=60),
    )
    return fig


def plot_coefficient_spectrogram(
    result: ChebyshevSpectrogramFit,
    *,
    log_scale: bool = True,
) -> go.Figure:
    """Plot the ``m x N`` coefficient-energy spectrogram."""
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
            x=[f"T{k}" for k in range(result.coefficient_count)],
            y=starts_s,
            colorscale="Viridis",
            colorbar_title=colorbar_title,
        )
    )
    fig.update_layout(
        title="Coefficient spectrogram (m intervals x N basis weights)",
        xaxis_title="Chebyshev spectral basis",
        yaxis_title="window start time [s]",
        height=max(420, min(900, 220 + 3 * result.window_count)),
        margin=dict(l=80, r=40, t=80, b=60),
    )
    return fig


def plot_average_spectra(
    result: ChebyshevSpectrogramFit,
    *,
    part_count: int = 4,
) -> go.Figure:
    """Plot mean coefficient spectra for the whole file and trajectory parts."""
    basis = np.arange(result.coefficient_count)
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=basis,
            y=np.mean(result.coeff_energy, axis=0),
            mode="lines+markers",
            line=dict(width=4, color="black"),
            name="whole file",
        )
    )
    for label, start, stop in trajectory_parts(result, part_count):
        fig.add_trace(
            go.Scatter(
                x=basis,
                y=np.mean(result.coeff_energy[start:stop], axis=0),
                mode="lines+markers",
                name=label,
            )
        )
    fig.update_layout(
        title="Average standardized coefficient spectra",
        xaxis_title="Chebyshev spectral basis index k",
        yaxis_title="mean RMS coefficient energy across selected components",
        height=450,
        margin=dict(l=80, r=40, t=80, b=60),
    )
    return fig


def plot_average_spectrogram_parts(
    result: ChebyshevSpectrogramFit,
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
            x=[f"T{k}" for k in range(result.coefficient_count)],
            y=labels,
            colorscale="Viridis",
            colorbar_title="log10 mean RMS weight",
        )
    )
    fig.update_layout(
        title="Average coefficient spectrograms: whole file and trajectory parts",
        xaxis_title="Chebyshev spectral basis",
        yaxis_title="trajectory section",
        height=360,
        margin=dict(l=130, r=40, t=80, b=60),
    )
    return fig


def trajectory_parts(
    result: ChebyshevSpectrogramFit,
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
