#!/usr/bin/env python3
"""
Matplotlib-based time series and frequency visualization.
Creates static plots for position, velocity, orientation, and frequency spectra.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Optional, List, Tuple, Dict
import os
from .trajectory_loader import (
    TrajectoryData, 
    load_trajectory, 
    discover_trajectories,
    load_trajectory_from_build,
    load_best_worst_trajectories,
    load_nees_timeseries,
    discover_intervals,
    DEFAULT_BUILD_DIR,
)


# Color scheme
COLORS = {
    'gt': '#2E4057',      # Dark blue - ground truth
    '2s': '#E63946',      # Red - 0.2s preintegration
    '5s': '#F77F00',      # Orange - 0.5s preintegration
    '10s': '#7209B7',     # Purple - 1.0s preintegration
}

LINESTYLES = {
    'gt': '-',
    '2s': '--',
    '5s': ':',
    '10s': '-.',
}


def plot_position_timeseries(
    trajectory: TrajectoryData,
    title: str = "Position Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot X, Y, Z position time series comparing ground truth vs predictions.
    
    Args:
        trajectory: TrajectoryData with gt_ and pred_ fields
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig, axes = plt.subplots(3, 1, figsize=(18, 12), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    labels = ['X', 'Y', 'Z']
    
    for i, (ax, label) in enumerate(zip(axes, labels)):
        # Ground truth - solid line
        ax.plot(
            trajectory.timestamp, trajectory.gt_position[:, i],
            color=COLORS['gt'], linewidth=3, linestyle='-',
            label='Ground Truth', alpha=0.85
        )
        
        # Predictions - dashed line (if available)
        if trajectory.pred_position is not None:
            # Use valid predictions only (non-zero)
            valid_mask = (
                np.isfinite(trajectory.pred_position[:, i]) &
                (trajectory.pred_position[:, i] != 0)
            )
            if valid_mask.any():
                ax.plot(
                    trajectory.timestamp[valid_mask], 
                    trajectory.pred_position[valid_mask, i],
                    color=COLORS['2s'], linewidth=2.5, linestyle='--',
                    label='Predicted', alpha=0.85
                )
        
        ax.set_ylabel(f'{label} Position (m)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.set_facecolor('#F5F5F5')
    
    axes[-1].set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_velocity_timeseries(
    trajectory: TrajectoryData,
    title: str = "Velocity Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot Vx, Vy, Vz velocity time series comparing ground truth vs predictions.
    
    Args:
        trajectory: TrajectoryData with gt_ and pred_ fields
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig, axes = plt.subplots(3, 1, figsize=(18, 12), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    labels = ['Vx', 'Vy', 'Vz']
    
    for i, (ax, label) in enumerate(zip(axes, labels)):
        # Ground truth - solid line
        ax.plot(
            trajectory.timestamp, trajectory.gt_velocity[:, i],
            color=COLORS['gt'], linewidth=3, linestyle='-',
            label='Ground Truth', alpha=0.85
        )
        
        # Predictions - dashed line (if available)
        if trajectory.pred_velocity is not None:
            valid_mask = (
                np.isfinite(trajectory.pred_velocity[:, i]) &
                (np.abs(trajectory.pred_velocity[:, i]) > 1e-6)
            )
            if valid_mask.any():
                ax.plot(
                    trajectory.timestamp[valid_mask], 
                    trajectory.pred_velocity[valid_mask, i],
                    color=COLORS['2s'], linewidth=2.5, linestyle='--',
                    label='Predicted', alpha=0.85
                )
        
        ax.set_ylabel(f'{label} (m/s)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.set_facecolor('#F5F5F5')
    
    axes[-1].set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_orientation_timeseries(
    trajectory: TrajectoryData,
    title: str = "Orientation Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot Roll, Pitch, Yaw time series comparing ground truth vs predictions.
    
    Args:
        trajectory: TrajectoryData with gt_ and pred_ fields
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig, axes = plt.subplots(3, 1, figsize=(18, 12), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    labels = ['Roll', 'Pitch', 'Yaw']
    
    for i, (ax, label) in enumerate(zip(axes, labels)):
        # Ground truth - solid line
        ax.plot(
            trajectory.timestamp, trajectory.gt_rpy[:, i],
            color=COLORS['gt'], linewidth=3, linestyle='-',
            label='Ground Truth', alpha=0.85
        )
        
        # Predictions - dashed line (if available)
        if trajectory.pred_rpy is not None:
            valid_mask = np.isfinite(trajectory.pred_rpy[:, i])
            if valid_mask.any():
                ax.plot(
                    trajectory.timestamp[valid_mask], 
                    trajectory.pred_rpy[valid_mask, i],
                    color=COLORS['2s'], linewidth=2.5, linestyle='--',
                    label='Predicted', alpha=0.85
                )
        
        ax.set_ylabel(f'{label} (deg)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.set_facecolor('#F5F5F5')
    
    axes[-1].set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_position_timeseries_multi(
    filter_name: str,
    dataset_name: str,
    data_dir: str = ".",
    preintegration_times: List[str] = ['2s', '5s', '10s'],
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot position time series comparing multiple preintegration times.
    
    Args:
        filter_name: Filter name (e.g., "gal3", "navstate")
        dataset_name: Dataset name (e.g., "MH01", "V202")
        data_dir: Directory containing CSV files
        preintegration_times: List of preintegration intervals
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig, axes = plt.subplots(3, 1, figsize=(18, 12), sharex=True)
    fig.suptitle(
        f'Position Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}',
        fontsize=16, fontweight='bold'
    )
    
    labels = ['X', 'Y', 'Z']
    trajectories = discover_trajectories(filter_name, dataset_name, data_dir)
    
    if not trajectories:
        print(f"Warning: No trajectory files found")
        return fig
    
    # Load ground truth from first available
    gt_traj = load_trajectory(list(trajectories.values())[0])
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth
        if gt_traj is not None:
            ax.plot(
                gt_traj.timestamp, gt_traj.position[:, i],
                color=COLORS['gt'], linewidth=3, linestyle='-',
                label='Ground Truth', alpha=0.85
            )
        
        # Predictions for each preintegration time
        for suffix in preintegration_times:
            if suffix not in trajectories:
                continue
            
            traj = load_trajectory(trajectories[suffix])
            if traj is None:
                continue
            
            color = COLORS.get(suffix, '#888888')
            linestyle = LINESTYLES.get(suffix, '--')
            
            ax.plot(
                traj.timestamp, traj.position[:, i],
                color=color, linewidth=2.5, linestyle=linestyle,
                label=f'{filter_name.upper()} ({suffix})', alpha=0.85
            )
        
        ax.set_ylabel(f'{label} Position (m)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.set_facecolor('#F5F5F5')
    
    axes[-1].set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_3d_trajectory_matplotlib(
    trajectory: TrajectoryData,
    title: str = "3D Trajectory",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot 3D trajectory using matplotlib (static, not interactive).
    Shows both ground truth and predictions.
    
    Args:
        trajectory: TrajectoryData with gt_ and pred_ fields
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Ground truth - dotted line
    ax.plot(
        trajectory.gt_position[:, 0],
        trajectory.gt_position[:, 1],
        trajectory.gt_position[:, 2],
        color=COLORS['gt'], linewidth=4, linestyle=':',
        label='Ground Truth', alpha=0.8
    )
    
    # Predictions - solid line (if available)
    if trajectory.pred_position is not None:
        valid_mask = (
            np.isfinite(trajectory.pred_position).all(axis=1) &
            (trajectory.pred_position[:, 0] != 0) &
            (trajectory.pred_position[:, 1] != 0) &
            (trajectory.pred_position[:, 2] != 0)
        )
        if valid_mask.any():
            ax.plot(
                trajectory.pred_position[valid_mask, 0],
                trajectory.pred_position[valid_mask, 1],
                trajectory.pred_position[valid_mask, 2],
                color=COLORS['2s'], linewidth=2.5, linestyle='-',
                label='Predicted', alpha=0.85
            )
    
    # Start/end markers (using ground truth)
    ax.scatter(
        trajectory.gt_position[0, 0],
        trajectory.gt_position[0, 1],
        trajectory.gt_position[0, 2],
        c='#06A77D', s=150, marker='o', edgecolors='white', linewidths=2,
        label='Start'
    )
    ax.scatter(
        trajectory.gt_position[-1, 0],
        trajectory.gt_position[-1, 1],
        trajectory.gt_position[-1, 2],
        c='#1E88E5', s=150, marker='s', edgecolors='white', linewidths=2,
        label='End'
    )
    
    ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(fontsize=10, loc='best')
    ax.grid(True, alpha=0.25, linestyle='--')
    ax.set_facecolor('#F5F5F5')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_frequency_spectrum(
    trajectory: TrajectoryData,
    title: str = "Frequency Spectrum",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot frequency spectrum for position, velocity, and orientation.
    
    Args:
        trajectory: Trajectory data
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    # Compute sampling rate
    dt = trajectory.timestamp[1] - trajectory.timestamp[0]
    sampling_rate = 1.0 / dt
    nyquist = sampling_rate / 2.0
    
    fig, axes = plt.subplots(3, 3, figsize=(20, 14))
    fig.suptitle(f'Frequency Spectrum @ {sampling_rate:.0f}Hz - {title}', 
                 fontsize=16, fontweight='bold')
    
    # Position spectra
    position_cols = ['X', 'Y', 'Z']
    for i, col_label in enumerate(position_cols):
        ax = axes[0, i]
        signal = trajectory.position[:, i]
        
        fft_vals = np.fft.rfft(signal)
        freqs = np.fft.rfftfreq(len(signal), dt)
        magnitudes = np.abs(fft_vals)
        
        ax.semilogy(freqs, magnitudes, color=COLORS['gt'], linewidth=2)
        ax.axvline(x=nyquist, color='red', linestyle='-', linewidth=2,
                   label=f'Nyquist ({nyquist:.0f} Hz)')
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Magnitude (log)')
        ax.set_title(f'Position {col_label}')
        ax.grid(True, alpha=0.25)
        ax.set_xlim([0, sampling_rate])
        ax.legend(fontsize=7)
    
    # Velocity spectra
    for i, col_label in enumerate(position_cols):
        ax = axes[1, i]
        signal = trajectory.velocity[:, i]
        
        fft_vals = np.fft.rfft(signal)
        freqs = np.fft.rfftfreq(len(signal), dt)
        magnitudes = np.abs(fft_vals)
        
        ax.semilogy(freqs, magnitudes, color=COLORS['gt'], linewidth=2)
        ax.axvline(x=nyquist, color='red', linestyle='-', linewidth=2)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Magnitude (log)')
        ax.set_title(f'Velocity {col_label}')
        ax.grid(True, alpha=0.25)
        ax.set_xlim([0, sampling_rate])
    
    # Orientation spectra
    orient_labels = ['Roll', 'Pitch', 'Yaw']
    for i, col_label in enumerate(orient_labels):
        ax = axes[2, i]
        signal = trajectory.rpy[:, i]
        
        fft_vals = np.fft.rfft(signal)
        freqs = np.fft.rfftfreq(len(signal), dt)
        magnitudes = np.abs(fft_vals)
        
        ax.semilogy(freqs, magnitudes, color=COLORS['gt'], linewidth=2)
        ax.axvline(x=nyquist, color='red', linestyle='-', linewidth=2)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Magnitude (log)')
        ax.set_title(f'{col_label}')
        ax.grid(True, alpha=0.25)
        ax.set_xlim([0, sampling_rate])
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_displacement_timeseries(
    ground_truth: TrajectoryData,
    predicted: Optional[TrajectoryData] = None,
    title: str = "Displacement Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot displacement (delta position) time series.
    
    Args:
        ground_truth: Ground truth trajectory
        predicted: Predicted trajectory (optional)
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig, axes = plt.subplots(3, 1, figsize=(18, 12), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    labels = ['ΔX', 'ΔY', 'ΔZ']
    
    # Ground truth displacement
    gt_disp = np.diff(ground_truth.position, axis=0)
    gt_times = ground_truth.timestamp[:-1]
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth displacement
        ax.plot(
            gt_times, gt_disp[:, i],
            color='#070707', linewidth=3, linestyle='-',
            label='Ground Truth Δ', alpha=1.0
        )
        
        # Predicted displacement
        if predicted is not None:
            pred_disp = np.diff(predicted.position, axis=0)
            pred_times = predicted.timestamp[:-1]
            ax.plot(
                pred_times, pred_disp[:, i],
                color=COLORS['2s'], linewidth=2.5, linestyle='--',
                label='Predicted Δ', alpha=0.5
            )
        
        ax.set_ylabel(f'{label} (m)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
        ax.set_facecolor('#F5F5F5')
    
    axes[-1].set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


# =============================================================================
# Build folder visualization functions
# =============================================================================

def plot_position_timeseries_from_build(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: List[str] = None,
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot position time series comparing multiple preintegration intervals.
    Loads directly from build folder.
    
    Args:
        dataset_name: Dataset name (e.g., "MH01", "V202")
        filter_name: Filter name (e.g., "gal3", "navstate")
        build_dir: Path to build directory
        intervals: List of intervals to compare (default: ["2s", "5s", "10s"])
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    if intervals is None:
        intervals = ["2s", "5s", "10s"]
    
    fig, axes = plt.subplots(3, 1, figsize=(18, 12), sharex=True)
    fig.suptitle(
        f'Position Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}',
        fontsize=16, fontweight='bold'
    )
    
    labels = ['X', 'Y', 'Z']
    color_map = {"2s": "#E63946", "5s": "#F77F00", "10s": "#7209B7"}
    ls_map = {"2s": "--", "5s": ":", "10s": "-."}
    
    # Load trajectories for each interval
    trajectories = {}
    for interval in intervals:
        traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
        if traj is not None:
            trajectories[interval] = traj
    
    if not trajectories:
        print(f"Warning: No trajectory files found for {filter_name} - {dataset_name}")
        return fig
    
    # Use first trajectory as ground truth reference
    first_interval = list(trajectories.keys())[0]
    gt_traj = trajectories[first_interval]
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth - solid line
        ax.plot(
            gt_traj.timestamp, gt_traj.gt_position[:, i],
            color=COLORS['gt'], linewidth=3, linestyle='-',
            label='Ground Truth', alpha=0.85
        )
        
        # Predictions for each interval - different line styles
        for interval, traj in trajectories.items():
            color = color_map.get(interval, "#888888")
            linestyle = ls_map.get(interval, "--")
            
            if traj.pred_position is not None:
                valid_mask = (
                    np.isfinite(traj.pred_position[:, i]) &
                    (traj.pred_position[:, i] != 0)
                )
                if valid_mask.any():
                    ax.plot(
                        traj.timestamp[valid_mask], 
                        traj.pred_position[valid_mask, i],
                        color=color, linewidth=2.5, linestyle=linestyle,
                        label=f'{filter_name.upper()} ({interval})', alpha=0.85
                    )
        
        ax.set_ylabel(f'{label} Position (m)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.set_facecolor('#F5F5F5')
    
    axes[-1].set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_3d_trajectory_from_build(
    dataset_name: str,
    filter_name: str = "gal3",
    interval: str = "2s",
    build_dir: str = DEFAULT_BUILD_DIR,
    title: Optional[str] = None,
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot 3D trajectory from build folder.
    
    Args:
        dataset_name: Dataset name
        filter_name: Filter name
        interval: Time interval
        build_dir: Build directory path
        title: Optional custom title
        save_path: Optional save path
        
    Returns:
        Matplotlib Figure
    """
    traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
    
    if traj is None:
        print(f"Warning: Could not load trajectory {filter_name}_{dataset_name}_{interval}")
        return plt.figure()
    
    if title is None:
        title = f'3D Trajectory: {filter_name.upper()} - {dataset_name} ({interval})'
    
    return plot_3d_trajectory_matplotlib(traj, title=title, save_path=save_path)


def plot_best_worst_comparison(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot best vs worst noise calibration comparison.
    
    Args:
        dataset_name: Dataset name
        filter_name: Filter name
        build_dir: Build directory path
        save_path: Optional save path
        
    Returns:
        Matplotlib Figure
    """
    best_traj, worst_traj = load_best_worst_trajectories(dataset_name, filter_name, build_dir)
    
    if best_traj is None or worst_traj is None:
        print(f"Warning: Could not load best/worst trajectories for {dataset_name}")
        return plt.figure()
    
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Ground truth
    ax.plot(
        best_traj.position[:, 0],
        best_traj.position[:, 1],
        best_traj.position[:, 2],
        color=COLORS['gt'], linewidth=4, linestyle=':',
        label='Ground Truth', alpha=0.8
    )
    
    # Best trajectory
    ax.plot(
        best_traj.position[:, 0],
        best_traj.position[:, 1],
        best_traj.position[:, 2],
        color='green', linewidth=2.5, linestyle='-',
        label='Best (αG=13.0, αA=9.4)', alpha=0.85
    )
    
    # Worst trajectory
    ax.plot(
        worst_traj.position[:, 0],
        worst_traj.position[:, 1],
        worst_traj.position[:, 2],
        color='red', linewidth=2.5, linestyle='-',
        label='Worst (αG=0.5, αA=0.5)', alpha=0.85
    )
    
    # Start/end markers
    ax.scatter(
        best_traj.position[0, 0], best_traj.position[0, 1], best_traj.position[0, 2],
        c='#06A77D', s=150, marker='o', edgecolors='white', linewidths=2,
        label='Start'
    )
    ax.scatter(
        best_traj.position[-1, 0], best_traj.position[-1, 1], best_traj.position[-1, 2],
        c='#1E88E5', s=150, marker='s', edgecolors='white', linewidths=2,
        label='End'
    )
    
    ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=12, fontweight='bold')
    ax.set_title(
        f'Best vs Worst Noise Calibration: {dataset_name}',
        fontsize=14, fontweight='bold'
    )
    ax.legend(fontsize=10, loc='best')
    ax.grid(True, alpha=0.25, linestyle='--')
    ax.set_facecolor('#F5F5F5')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_nees_timeseries(
    dataset_name: str,
    filter_name: str = "gal3",
    interval: str = "2s",
    build_dir: str = DEFAULT_BUILD_DIR,
    title: Optional[str] = None,
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot NEES time series.
    
    Args:
        dataset_name: Dataset name
        filter_name: Filter name
        interval: Time interval
        build_dir: Build directory path
        title: Optional custom title
        save_path: Optional save path
        
    Returns:
        Matplotlib Figure
    """
    nees_df = load_nees_timeseries(dataset_name, filter_name, interval, build_dir)
    
    if nees_df is None:
        print(f"Warning: Could not load NEES data for {filter_name}_{dataset_name}_{interval}")
        return plt.figure()
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    if title is None:
        title = f'NEES Time Series - {filter_name.upper()} - {dataset_name} ({interval})'
    
    ax.plot(nees_df['timestamp'], nees_df['nees'], color='#E63946', linewidth=2)
    ax.axhline(y=1.0, color='green', linestyle='--', linewidth=2, label='Optimal (NEES=1)')
    ax.axhline(y=3.0, color='orange', linestyle='--', linewidth=2, label='Threshold (NEES=3)')
    
    ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    ax.set_ylabel('NEES', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.25, linestyle='--')
    ax.legend(fontsize=10)
    ax.set_facecolor('#F5F5F5')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_all_intervals_comparison(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    save_dir: Optional[str] = None
) -> Dict[str, plt.Figure]:
    """
    Generate all visualization plots for a dataset across all intervals.
    
    Args:
        dataset_name: Dataset name
        filter_name: Filter name
        build_dir: Build directory path
        save_dir: Optional directory to save figures
        
    Returns:
        Dict of figure names to Figure objects
    """
    intervals = discover_intervals(dataset_name, filter_name, build_dir)
    
    if not intervals:
        print(f"Warning: No intervals found for {dataset_name}")
        return {}
    
    figures = {}
    
    # 3D trajectory for each interval
    for interval in intervals:
        if save_dir:
            save_path = os.path.join(save_dir, f'trajectory_3d_{dataset_name}_{interval}.png')
        else:
            save_path = None
        
        fig = plot_3d_trajectory_from_build(
            dataset_name, filter_name, interval, build_dir,
            save_path=save_path
        )
        figures[f'3d_{interval}'] = fig
    
    # Combined position time series
    if save_dir:
        save_path = os.path.join(save_dir, f'position_timeseries_{dataset_name}.png')
    else:
        save_path = None
    
    fig = plot_position_timeseries_from_build(
        dataset_name, filter_name, build_dir,
        save_path=save_path
    )
    figures['position_multi'] = fig
    
    # NEES time series for each interval
    for interval in intervals:
        if save_dir:
            save_path = os.path.join(save_dir, f'nees_timeseries_{dataset_name}_{interval}.png')
        else:
            save_path = None
        
        fig = plot_nees_timeseries(
            dataset_name, filter_name, interval, build_dir,
            save_path=save_path
        )
        figures[f'nees_{interval}'] = fig
    
    return figures