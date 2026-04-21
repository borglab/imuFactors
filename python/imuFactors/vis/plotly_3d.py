#!/usr/bin/env python3
"""
Plotly-based interactive trajectory visualization.
Creates interactive HTML visualizations of ground truth vs predicted trajectories.
All figures support legend-click toggling to show/hide individual traces.
"""

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
import pandas as pd
from typing import Optional, List, Tuple, Dict
import os
from .trajectory_loader import (
    TrajectoryData,
    load_trajectory,
    discover_trajectories,
    load_trajectory_from_build,
    load_best_worst_trajectories,
    discover_intervals,
    load_nees_summary,
    compute_acceleration,
    DEFAULT_BUILD_DIR,
)

# ---------------------------------------------------------------------------
# Shared style constants
# ---------------------------------------------------------------------------

GT_COLOR = "#2E4057"
GT_DASH  = "dot"

INTERVAL_COLORS = {
    "2s":  "#E63946",
    "5s":  "#F77F00",
    "10s": "#7209B7",
}
INTERVAL_DASHES = {
    "2s":  "solid",
    "5s":  "dash",
    "10s": "dashdot",
}
INTERVAL_LABELS = {
    "2s":  "0.2 s",
    "5s":  "0.5 s",
    "10s": "1.0 s",
}

_PLOTLY_TEMPLATE = "plotly_white"
_LAYOUT_DEFAULTS = dict(
    template=_PLOTLY_TEMPLATE,
    legend=dict(
        yanchor="top", y=0.99,
        xanchor="left", x=0.01,
        bgcolor="rgba(255,255,255,0.85)",
        bordercolor="rgba(0,0,0,0.15)",
        borderwidth=1,
        itemclick="toggle",
        itemdoubleclick="toggleothers",
    ),
    hoverlabel=dict(bgcolor="white", font_size=12),
)


def _valid_mask(pos: np.ndarray) -> np.ndarray:
    """Return boolean mask of rows that are finite and non-zero."""
    return (
        np.isfinite(pos).all(axis=1)
        & (np.abs(pos).max(axis=1) > 1e-9)
    )


def save_html(fig: go.Figure, path: str) -> None:
    """
    Write a Plotly figure to a self-contained HTML file.

    Args:
        fig:  Plotly Figure object
        path: Destination file path (will be created/overwritten)
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.write_html(
        path,
        include_plotlyjs="cdn",   # smaller file; requires internet to view
        full_html=True,
        config={
            "scrollZoom": True,
            "displayModeBar": True,
            "modeBarButtonsToRemove": ["lasso2d", "select2d"],
            "toImageButtonOptions": {"format": "png", "width": 1600, "height": 900},
        },
    )
    print(f"✓ Saved HTML: {path}")


# ---------------------------------------------------------------------------
# 3-D trajectory (single interval)
# ---------------------------------------------------------------------------

def plot_3d_trajectory(
    trajectory: TrajectoryData,
    title: str = "3D Trajectory",
    downsample: int = 1,
    show_legend: bool = True,
    save_path: Optional[str] = None,
) -> go.Figure:
    """
    Interactive 3-D plot: ground truth (dotted) vs prediction (solid).
    Legend entries are click-toggleable.

    Args:
        trajectory:   TrajectoryData with gt_* and pred_* fields
        title:        Plot title
        downsample:   Plot every Nth point for performance
        show_legend:  Show legend
        save_path:    If provided, save HTML to this path

    Returns:
        Plotly Figure
    """
    fig = go.Figure()
    ds = slice(None, None, downsample)

    # Ground truth
    fig.add_trace(go.Scatter3d(
        x=trajectory.gt_position[ds, 0],
        y=trajectory.gt_position[ds, 1],
        z=trajectory.gt_position[ds, 2],
        mode="lines",
        name="Ground Truth",
        line=dict(color=GT_COLOR, width=6, dash=GT_DASH),
        opacity=0.85,
        hovertemplate="<b>GT</b><br>X: %{x:.3f} m<br>Y: %{y:.3f} m<br>Z: %{z:.3f} m<extra></extra>",
    ))

    # Prediction
    if trajectory.pred_position is not None:
        mask = _valid_mask(trajectory.pred_position)
        if mask.any():
            idx = np.where(mask)[0][::downsample]
            fig.add_trace(go.Scatter3d(
                x=trajectory.pred_position[idx, 0],
                y=trajectory.pred_position[idx, 1],
                z=trajectory.pred_position[idx, 2],
                mode="lines+markers",
                name="Predicted",
                line=dict(color=INTERVAL_COLORS["2s"], width=3),
                marker=dict(size=2, color=INTERVAL_COLORS["2s"]),
                opacity=0.9,
                hovertemplate="<b>Pred</b><br>X: %{x:.3f} m<br>Y: %{y:.3f} m<br>Z: %{z:.3f} m<extra></extra>",
            ))

    # Start / end markers
    for label, idx_, color, symbol in [
        ("Start", 0,  "#06A77D", "circle"),
        ("End",   -1, "#1E88E5", "square"),
    ]:
        fig.add_trace(go.Scatter3d(
            x=[trajectory.gt_position[idx_, 0]],
            y=[trajectory.gt_position[idx_, 1]],
            z=[trajectory.gt_position[idx_, 2]],
            mode="markers",
            name=label,
            marker=dict(size=10, color=color, symbol=symbol,
                        line=dict(color="white", width=2)),
            hovertemplate=f"<b>{label}</b><br>X: %{{x:.3f}}<br>Y: %{{y:.3f}}<br>Z: %{{z:.3f}}<extra></extra>",
        ))

    fig.update_layout(
        title=dict(text=title, x=0.5, xanchor="center", font=dict(size=18)),
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            bgcolor="#F5F5F5", aspectmode="data",
        ),
        showlegend=show_legend,
        height=700,
        **_LAYOUT_DEFAULTS,
    )

    if save_path:
        save_html(fig, save_path)

    return fig


# ---------------------------------------------------------------------------
# 3-D trajectory — multi-interval comparison
# ---------------------------------------------------------------------------

def plot_3d_trajectory_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: Optional[List[str]] = None,
    title: Optional[str] = None,
    downsample: int = 1,
    save_path: Optional[str] = None,
) -> go.Figure:
    """
    Interactive 3-D plot comparing ground truth against multiple preintegration
    intervals.  Each trace can be toggled individually via the legend.

    Args:
        dataset_name: Dataset name (e.g. "MH01")
        filter_name:  Filter name (e.g. "gal3")
        build_dir:    Build directory containing CSV files
        intervals:    Intervals to compare; auto-discovered if None
        title:        Optional custom title
        downsample:   Plot every Nth point
        save_path:    If provided, save HTML to this path

    Returns:
        Plotly Figure
    """
    if intervals is None:
        intervals = discover_intervals(dataset_name, filter_name, build_dir) or ["2s", "5s", "10s"]

    if title is None:
        title = f"3D Trajectory — {filter_name.upper()} · {dataset_name}"

    # Load ground truth from first available interval
    gt_traj = None
    for iv in intervals:
        t = load_trajectory_from_build(filter_name, dataset_name, iv, build_dir)
        if t is not None:
            gt_traj = t
            break

    if gt_traj is None:
        print(f"Warning: no trajectory files found for {filter_name}/{dataset_name}")
        return go.Figure()

    fig = go.Figure()
    ds = slice(None, None, downsample)

    # Ground truth
    fig.add_trace(go.Scatter3d(
        x=gt_traj.gt_position[ds, 0],
        y=gt_traj.gt_position[ds, 1],
        z=gt_traj.gt_position[ds, 2],
        mode="lines",
        name="Ground Truth",
        line=dict(color=GT_COLOR, width=6, dash=GT_DASH),
        opacity=0.85,
        hovertemplate="<b>GT</b><br>X: %{x:.3f}<br>Y: %{y:.3f}<br>Z: %{z:.3f}<extra></extra>",
    ))

    # One trace per interval
    for iv in intervals:
        traj = load_trajectory_from_build(filter_name, dataset_name, iv, build_dir)
        if traj is None or traj.pred_position is None:
            continue

        mask = _valid_mask(traj.pred_position)
        if not mask.any():
            continue

        idx = np.where(mask)[0][::downsample]
        color = INTERVAL_COLORS.get(iv, "#888888")
        dash  = INTERVAL_DASHES.get(iv, "solid")
        label = INTERVAL_LABELS.get(iv, iv)

        fig.add_trace(go.Scatter3d(
            x=traj.pred_position[idx, 0],
            y=traj.pred_position[idx, 1],
            z=traj.pred_position[idx, 2],
            mode="lines",
            name=f"{filter_name.upper()} ({label})",
            line=dict(color=color, width=3, dash=dash),
            opacity=0.88,
            hovertemplate=f"<b>{filter_name.upper()} {label}</b><br>X: %{{x:.3f}}<br>Y: %{{y:.3f}}<br>Z: %{{z:.3f}}<extra></extra>",
        ))

    # Start / end
    for label, pos_idx, color, symbol in [
        ("Start",  0,  "#06A77D", "circle"),
        ("End",   -1,  "#1E88E5", "square"),
    ]:
        fig.add_trace(go.Scatter3d(
            x=[gt_traj.gt_position[pos_idx, 0]],
            y=[gt_traj.gt_position[pos_idx, 1]],
            z=[gt_traj.gt_position[pos_idx, 2]],
            mode="markers",
            name=label,
            marker=dict(size=10, color=color, symbol=symbol,
                        line=dict(color="white", width=2)),
            hovertemplate=f"<b>{label}</b><br>X: %{{x:.3f}}<br>Y: %{{y:.3f}}<br>Z: %{{z:.3f}}<extra></extra>",
        ))

    fig.update_layout(
        title=dict(text=title, x=0.5, xanchor="center", font=dict(size=18)),
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            bgcolor="#F5F5F5", aspectmode="data",
        ),
        height=720,
        **_LAYOUT_DEFAULTS,
    )

    if save_path:
        save_html(fig, save_path)

    return fig


# ---------------------------------------------------------------------------
# Generic multi-interval time-series helper
# ---------------------------------------------------------------------------

def _plot_timeseries_multi_interval(
    dataset_name: str,
    filter_name: str,
    build_dir: str,
    intervals: List[str],
    gt_getter,          # callable(traj) -> np.ndarray  shape (N, 3)
    pred_getter,        # callable(traj) -> np.ndarray | None  shape (N, 3)
    row_labels: List[str],       # e.g. ["X (m)", "Y (m)", "Z (m)"]
    subplot_titles: List[str],
    title: str,
    save_path: Optional[str],
    time_getter=None,   # callable(traj) -> np.ndarray  (for derived signals like accel)
) -> go.Figure:
    """
    Build a 3-row shared-x Plotly figure comparing GT vs N intervals.
    Every trace is independently toggleable via the legend.
    """
    gt_traj = None
    for iv in intervals:
        t = load_trajectory_from_build(filter_name, dataset_name, iv, build_dir)
        if t is not None:
            gt_traj = t
            break

    if gt_traj is None:
        print(f"Warning: no data for {filter_name}/{dataset_name}")
        return go.Figure()

    gt_data = gt_getter(gt_traj)
    gt_time = time_getter(gt_traj) if time_getter else gt_traj.timestamp
    n_rows  = gt_data.shape[1] if gt_data.ndim == 2 else 1

    fig = make_subplots(
        rows=n_rows, cols=1,
        subplot_titles=subplot_titles,
        shared_xaxes=True,
        vertical_spacing=0.07,
    )

    for i in range(n_rows):
        row = i + 1
        show_legend_gt = i == 0

        # Ground truth trace
        fig.add_trace(go.Scatter(
            x=gt_time,
            y=gt_data[:, i],
            mode="lines",
            name="Ground Truth",
            legendgroup="gt",
            showlegend=show_legend_gt,
            line=dict(color=GT_COLOR, width=3),
            opacity=0.92,
            hovertemplate=f"<b>GT {subplot_titles[i]}</b><br>t=%{{x:.3f}} s<br>val=%{{y:.4f}}<extra></extra>",
        ), row=row, col=1)

        # One trace per interval
        for iv in intervals:
            traj = load_trajectory_from_build(filter_name, dataset_name, iv, build_dir)
            if traj is None:
                continue

            pred_data = pred_getter(traj)
            if pred_data is None:
                continue

            pred_time = time_getter(traj) if time_getter else traj.timestamp

            # validity filter
            if pred_data.ndim == 2:
                mask = np.isfinite(pred_data).all(axis=1) & (np.abs(pred_data).max(axis=1) > 1e-9)
            else:
                mask = np.isfinite(pred_data)

            if not mask.any():
                continue

            color = INTERVAL_COLORS.get(iv, "#888888")
            dash  = INTERVAL_DASHES.get(iv, "solid")
            label = INTERVAL_LABELS.get(iv, iv)
            show_legend_pred = i == 0

            fig.add_trace(go.Scatter(
                x=pred_time[mask],
                y=pred_data[mask, i] if pred_data.ndim == 2 else pred_data[mask],
                mode="lines",
                name=f"{filter_name.upper()} ({label})",
                legendgroup=f"pred_{iv}",
                showlegend=show_legend_pred,
                line=dict(color=color, width=2, dash=dash),
                opacity=0.85,
                hovertemplate=f"<b>{filter_name.upper()} {label}</b><br>t=%{{x:.3f}} s<br>val=%{{y:.4f}}<extra></extra>",
            ), row=row, col=1)

        fig.update_yaxes(title_text=row_labels[i], row=row, col=1)

    fig.update_xaxes(title_text="Time (s)", row=n_rows, col=1)
    fig.update_layout(
        title=dict(text=title, x=0.5, xanchor="center", font=dict(size=16)),
        height=300 * n_rows + 100,
        hovermode="x unified",
        **_LAYOUT_DEFAULTS,
    )

    if save_path:
        save_html(fig, save_path)

    return fig


# ---------------------------------------------------------------------------
# Position time series — multi-interval
# ---------------------------------------------------------------------------

def plot_position_timeseries_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: Optional[List[str]] = None,
    title: Optional[str] = None,
    save_path: Optional[str] = None,
) -> go.Figure:
    """
    Interactive position time-series (X, Y, Z) comparing GT vs multiple intervals.

    Args:
        dataset_name: Dataset name
        filter_name:  Filter name
        build_dir:    Build directory
        intervals:    Intervals to compare; auto-discovered if None
        title:        Optional custom title
        save_path:    If provided, save HTML to this path

    Returns:
        Plotly Figure
    """
    if intervals is None:
        intervals = discover_intervals(dataset_name, filter_name, build_dir) or ["2s", "5s", "10s"]
    if title is None:
        title = f"Position Time Series — {filter_name.upper()} · {dataset_name}"

    return _plot_timeseries_multi_interval(
        dataset_name, filter_name, build_dir, intervals,
        gt_getter=lambda t: t.gt_position,
        pred_getter=lambda t: t.pred_position,
        row_labels=["X (m)", "Y (m)", "Z (m)"],
        subplot_titles=["X Position", "Y Position", "Z Position"],
        title=title,
        save_path=save_path,
    )


# ---------------------------------------------------------------------------
# Velocity time series — multi-interval
# ---------------------------------------------------------------------------

def plot_velocity_timeseries_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: Optional[List[str]] = None,
    title: Optional[str] = None,
    save_path: Optional[str] = None,
) -> go.Figure:
    """Interactive velocity time-series (Vx, Vy, Vz) comparing GT vs multiple intervals."""
    if intervals is None:
        intervals = discover_intervals(dataset_name, filter_name, build_dir) or ["2s", "5s", "10s"]
    if title is None:
        title = f"Velocity Time Series — {filter_name.upper()} · {dataset_name}"

    return _plot_timeseries_multi_interval(
        dataset_name, filter_name, build_dir, intervals,
        gt_getter=lambda t: t.gt_velocity,
        pred_getter=lambda t: t.pred_velocity,
        row_labels=["Vx (m/s)", "Vy (m/s)", "Vz (m/s)"],
        subplot_titles=["X Velocity", "Y Velocity", "Z Velocity"],
        title=title,
        save_path=save_path,
    )


# ---------------------------------------------------------------------------
# Acceleration time series — multi-interval
# ---------------------------------------------------------------------------

def plot_acceleration_timeseries_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: Optional[List[str]] = None,
    title: Optional[str] = None,
    save_path: Optional[str] = None,
) -> go.Figure:
    """
    Interactive acceleration time-series (Ax, Ay, Az) computed by differentiating
    velocity.  Comparing GT vs multiple intervals.
    """
    if intervals is None:
        intervals = discover_intervals(dataset_name, filter_name, build_dir) or ["2s", "5s", "10s"]
    if title is None:
        title = f"Acceleration Time Series — {filter_name.upper()} · {dataset_name}"

    def _gt_accel(t: TrajectoryData) -> np.ndarray:
        if t.gt_acceleration is not None:
            return t.gt_acceleration
        return compute_acceleration(t.gt_velocity, t.timestamp)

    def _pred_accel(t: TrajectoryData) -> Optional[np.ndarray]:
        if t.pred_velocity is None:
            return None
        if t.pred_acceleration is not None:
            return t.pred_acceleration
        return compute_acceleration(t.pred_velocity, t.timestamp)

    def _accel_time(t: TrajectoryData) -> np.ndarray:
        return t.timestamp[:-1]  # one fewer point after differentiation

    return _plot_timeseries_multi_interval(
        dataset_name, filter_name, build_dir, intervals,
        gt_getter=_gt_accel,
        pred_getter=_pred_accel,
        row_labels=["Ax (m/s²)", "Ay (m/s²)", "Az (m/s²)"],
        subplot_titles=["X Acceleration", "Y Acceleration", "Z Acceleration"],
        title=title,
        save_path=save_path,
        time_getter=_accel_time,
    )


# ---------------------------------------------------------------------------
# Orientation time series — multi-interval
# ---------------------------------------------------------------------------

def plot_orientation_timeseries_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: Optional[List[str]] = None,
    title: Optional[str] = None,
    save_path: Optional[str] = None,
) -> go.Figure:
    """Interactive orientation time-series (Roll, Pitch, Yaw) comparing GT vs multiple intervals."""
    if intervals is None:
        intervals = discover_intervals(dataset_name, filter_name, build_dir) or ["2s", "5s", "10s"]
    if title is None:
        title = f"Orientation Time Series — {filter_name.upper()} · {dataset_name}"

    return _plot_timeseries_multi_interval(
        dataset_name, filter_name, build_dir, intervals,
        gt_getter=lambda t: t.gt_rpy,
        pred_getter=lambda t: t.pred_rpy,
        row_labels=["Roll (°)", "Pitch (°)", "Yaw (°)"],
        subplot_titles=["Roll", "Pitch", "Yaw"],
        title=title,
        save_path=save_path,
    )


# ---------------------------------------------------------------------------
# Displacement time series — multi-interval
# ---------------------------------------------------------------------------

def plot_displacement_timeseries_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: Optional[List[str]] = None,
    title: Optional[str] = None,
    save_path: Optional[str] = None,
) -> go.Figure:
    """Interactive displacement time-series (ΔX, ΔY, ΔZ) comparing GT vs multiple intervals."""
    if intervals is None:
        intervals = discover_intervals(dataset_name, filter_name, build_dir) or ["2s", "5s", "10s"]
    if title is None:
        title = f"Displacement Time Series — {filter_name.upper()} · {dataset_name}"

    def _gt_disp(t: TrajectoryData) -> np.ndarray:
        return np.diff(t.gt_position, axis=0)

    def _pred_disp(t: TrajectoryData) -> Optional[np.ndarray]:
        if t.pred_position is None:
            return None
        return np.diff(t.pred_position, axis=0)

    def _disp_time(t: TrajectoryData) -> np.ndarray:
        return t.timestamp[:-1]

    return _plot_timeseries_multi_interval(
        dataset_name, filter_name, build_dir, intervals,
        gt_getter=_gt_disp,
        pred_getter=_pred_disp,
        row_labels=["ΔX (m)", "ΔY (m)", "ΔZ (m)"],
        subplot_titles=["X Displacement", "Y Displacement", "Z Displacement"],
        title=title,
        save_path=save_path,
        time_getter=_disp_time,
    )


# ---------------------------------------------------------------------------
# Position time-series from build (single interval, legacy API)
# ---------------------------------------------------------------------------

def plot_position_timeseries_from_build(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: Optional[List[str]] = None,
    title: Optional[str] = None,
    save_path: Optional[str] = None,
) -> go.Figure:
    """Alias for plot_position_timeseries_multi_interval (legacy name kept for compatibility)."""
    return plot_position_timeseries_multi_interval(
        dataset_name, filter_name, build_dir, intervals, title, save_path
    )


# ---------------------------------------------------------------------------
# 3-D trajectory from build (single interval)
# ---------------------------------------------------------------------------

def plot_3d_trajectory_from_build(
    dataset_name: str,
    filter_name: str = "gal3",
    interval: str = "2s",
    build_dir: str = DEFAULT_BUILD_DIR,
    title: Optional[str] = None,
    downsample: int = 1,
    save_path: Optional[str] = None,
) -> go.Figure:
    """
    Interactive 3-D trajectory for a single interval loaded from the build folder.

    Args:
        dataset_name: Dataset name
        filter_name:  Filter name
        interval:     Preintegration interval (e.g. "2s")
        build_dir:    Build directory
        title:        Optional custom title
        downsample:   Plot every Nth point
        save_path:    If provided, save HTML to this path

    Returns:
        Plotly Figure
    """
    traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
    if traj is None:
        print(f"Warning: could not load {filter_name}_{dataset_name}_{interval}")
        return go.Figure()

    if title is None:
        title = f"3D Trajectory — {filter_name.upper()} · {dataset_name} ({interval})"

    return plot_3d_trajectory(traj, title=title, downsample=downsample, save_path=save_path)


# ---------------------------------------------------------------------------
# Legacy / comparison helpers
# ---------------------------------------------------------------------------

def plot_3d_trajectory_comparison(
    filter_name: str,
    dataset_name: str,
    data_dir: str = ".",
    preintegration_times: List[str] = ["2s", "5s", "10s"],
    title: Optional[str] = None,
    save_path: Optional[str] = None,
) -> go.Figure:
    """
    3-D trajectory comparing multiple preintegration times (discovers from data_dir).
    Prefer plot_3d_trajectory_multi_interval for build-folder workflows.
    """
    if title is None:
        title = f"3D Trajectory — {filter_name.upper()} · {dataset_name}"

    fig = go.Figure()
    trajectories = discover_trajectories(filter_name, dataset_name, data_dir)

    if not trajectories:
        print(f"Warning: no files found for {filter_name}/{dataset_name} in {data_dir}")
        return fig

    first_traj = load_trajectory(list(trajectories.values())[0])
    if first_traj is None:
        return fig

    fig.add_trace(go.Scatter3d(
        x=first_traj.gt_position[:, 0],
        y=first_traj.gt_position[:, 1],
        z=first_traj.gt_position[:, 2],
        mode="lines", name="Ground Truth",
        line=dict(color=GT_COLOR, width=6, dash=GT_DASH),
        opacity=0.80,
    ))

    for suffix in preintegration_times:
        if suffix not in trajectories:
            continue
        traj = load_trajectory(trajectories[suffix])
        if traj is None or traj.pred_position is None:
            continue

        mask = _valid_mask(traj.pred_position)
        if not mask.any():
            continue

        color = INTERVAL_COLORS.get(suffix, "#888888")
        dash  = INTERVAL_DASHES.get(suffix, "solid")
        label = INTERVAL_LABELS.get(suffix, suffix)

        fig.add_trace(go.Scatter3d(
            x=traj.pred_position[mask, 0],
            y=traj.pred_position[mask, 1],
            z=traj.pred_position[mask, 2],
            mode="lines",
            name=f"{filter_name.upper()} ({label})",
            line=dict(color=color, width=3, dash=dash),
            opacity=0.88,
        ))

    fig.update_layout(
        title=dict(text=title, x=0.5, xanchor="center", font=dict(size=18)),
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            bgcolor="#F5F5F5", aspectmode="data",
        ),
        height=700,
        **_LAYOUT_DEFAULTS,
    )

    if save_path:
        save_html(fig, save_path)

    return fig


def plot_comparison(
    dataset_name: str,
    ground_truth: TrajectoryData,
    best_trajectory: TrajectoryData,
    worst_trajectory: TrajectoryData,
    best_nees: float,
    worst_nees: float,
    best_alpha_gyro: float = 13.0,
    best_alpha_acc: float = 9.4,
    worst_alpha_gyro: float = 0.5,
    worst_alpha_acc: float = 0.5,
    save_path: Optional[str] = None,
) -> go.Figure:
    """3-D + error + XY-projection comparison of best vs worst noise parameters."""
    best_errors = np.linalg.norm(
        best_trajectory.gt_position - ground_truth.gt_position[:len(best_trajectory.gt_position)],
        axis=1,
    )
    worst_errors = np.linalg.norm(
        worst_trajectory.gt_position - ground_truth.gt_position[:len(worst_trajectory.gt_position)],
        axis=1,
    )

    fig = make_subplots(
        rows=2, cols=2,
        specs=[[{"type": "scatter3d", "rowspan": 2}, {"type": "scatter"}],
               [None,                                  {"type": "scatter"}]],
        subplot_titles=(f"{dataset_name}: 3D Trajectory",
                        "Position Error Over Time", "XY Plane Projection"),
        vertical_spacing=0.12, horizontal_spacing=0.1,
    )

    ds = max(1, len(ground_truth.gt_position) // 500)

    for trace, name_, color_, width_ in [
        (ground_truth.gt_position,    "Ground Truth", "black", 6),
        (best_trajectory.gt_position,
         f"Best (αG={best_alpha_gyro}, αA={best_alpha_acc}, NEES={best_nees:.3f})",
         "green", 3),
        (worst_trajectory.gt_position,
         f"Worst (αG={worst_alpha_gyro}, αA={worst_alpha_acc}, NEES={worst_nees:.3f})",
         "red", 3),
    ]:
        fig.add_trace(go.Scatter3d(
            x=trace[::ds, 0], y=trace[::ds, 1], z=trace[::ds, 2],
            mode="lines", name=name_,
            line=dict(color=color_, width=width_), opacity=0.9,
        ), row=1, col=1)

    fig.add_trace(go.Scatter(
        x=best_trajectory.timestamp, y=best_errors, mode="lines",
        name=f"Best Error (mean {np.mean(best_errors):.3f} m)",
        line=dict(color="green", width=2),
    ), row=1, col=2)
    fig.add_trace(go.Scatter(
        x=worst_trajectory.timestamp, y=worst_errors, mode="lines",
        name=f"Worst Error (mean {np.mean(worst_errors):.3f} m)",
        line=dict(color="red", width=2),
    ), row=1, col=2)

    for pos, name_, color_, width_, show_ in [
        (ground_truth.gt_position,    "GT (XY)",    "black", 4, False),
        (best_trajectory.gt_position, "Best (XY)",  "green", 2, False),
        (worst_trajectory.gt_position,"Worst (XY)", "red",   2, False),
    ]:
        fig.add_trace(go.Scatter(
            x=pos[::ds, 0], y=pos[::ds, 1], mode="lines",
            name=name_, line=dict(color=color_, width=width_), showlegend=show_,
        ), row=2, col=2)

    fig.update_layout(
        title=dict(
            text=f"{dataset_name}: Best vs Worst Noise Parameters",
            x=0.5, xanchor="center", font=dict(size=18),
        ),
        height=800,
        hovermode="closest",
        **_LAYOUT_DEFAULTS,
    )
    fig.update_scenes(xaxis_title="X (m)", yaxis_title="Y (m)",
                      zaxis_title="Z (m)", aspectmode="data")
    fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_yaxes(title_text="Position Error (m)", row=1, col=2)
    fig.update_xaxes(title_text="X (m)", row=2, col=2)
    fig.update_yaxes(title_text="Y (m)", row=2, col=2,
                     scaleanchor="x", scaleratio=1)

    if save_path:
        save_html(fig, save_path)

    return fig


def plot_best_worst_comparison_plotly(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    nees_summary_path: Optional[str] = None,
    title: Optional[str] = None,
    save_path: Optional[str] = None,
) -> go.Figure:
    """Interactive 3-D comparison of best vs worst noise calibration from build folder."""
    best_traj, worst_traj = load_best_worst_trajectories(dataset_name, filter_name, build_dir)

    if best_traj is None or worst_traj is None:
        print(f"Warning: could not load best/worst trajectories for {dataset_name}")
        return go.Figure()

    best_nees = worst_nees = 0.0
    if nees_summary_path and os.path.exists(nees_summary_path):
        nees_df = load_nees_summary(nees_summary_path)
        if nees_df is not None:
            row = nees_df[nees_df["dataset"] == dataset_name]
            if not row.empty:
                best_nees  = float(row["best_nees"].values[0])
                worst_nees = float(row["worst_nees"].values[0])

    if title is None:
        title = f"Best vs Worst Noise Calibration — {dataset_name}"

    return plot_comparison(
        dataset_name,
        ground_truth=best_traj,          # GT lives inside each TrajectoryData
        best_trajectory=best_traj,
        worst_trajectory=worst_traj,
        best_nees=best_nees,
        worst_nees=worst_nees,
        save_path=save_path,
    )


def create_nees_summary_figure(
    nees_summary: pd.DataFrame,
    save_path: Optional[str] = None,
) -> go.Figure:
    """Bar chart comparing NEES values across datasets (log-y scale)."""
    fig = go.Figure()

    fig.add_trace(go.Bar(
        x=nees_summary["dataset"], y=nees_summary["best_nees"],
        name="Best", marker_color="green",
        hovertemplate="<b>%{x}</b><br>NEES: %{y:.4f}<extra></extra>",
    ))
    fig.add_trace(go.Bar(
        x=nees_summary["dataset"], y=nees_summary["worst_nees"],
        name="Worst", marker_color="red",
        hovertemplate="<b>%{x}</b><br>NEES: %{y:.4f}<extra></extra>",
    ))

    fig.update_layout(
        title="NEES Comparison Across All Datasets",
        xaxis_title="Dataset",
        yaxis_title="NEES Value",
        yaxis_type="log",
        barmode="group",
        height=500,
        hovermode="x unified",
        **_LAYOUT_DEFAULTS,
    )

    if save_path:
        save_html(fig, save_path)

    return fig


def create_nees_summary_from_build(
    build_dir: str = DEFAULT_BUILD_DIR,
    output_path: Optional[str] = None,
) -> pd.DataFrame:
    """Build a NEES summary DataFrame from per-dataset NEES CSV files in build_dir."""
    nees_files = []
    for f in os.listdir(build_dir):
        if f.startswith("gal3_nees_") and f.endswith(".csv"):
            parts = f.replace("gal3_nees_", "").replace(".csv", "").split("_")
            if len(parts) >= 2:
                nees_files.append((parts[0], parts[1], os.path.join(build_dir, f)))

    datasets: Dict[str, Dict[str, float]] = {}
    for dataset, interval, filepath in nees_files:
        try:
            df = pd.read_csv(filepath)
            datasets.setdefault(dataset, {})[interval] = float(df["nees"].mean())
        except Exception as e:
            print(f"Warning: could not load {filepath}: {e}")

    summary_data = []
    for dataset, ivs in sorted(datasets.items()):
        best  = ivs.get("2s")
        worst = ivs.get("10s") or ivs.get("5s")
        summary_data.append({
            "dataset":    dataset,
            "best_nees":  best,
            "worst_nees": worst,
            "nees_ratio": (worst / best) if (best and worst) else None,
        })

    summary_df = pd.DataFrame(summary_data)

    if output_path and not summary_df.empty:
        summary_df.to_csv(output_path, index=False)
        print(f"Saved: {output_path}")

    return summary_df