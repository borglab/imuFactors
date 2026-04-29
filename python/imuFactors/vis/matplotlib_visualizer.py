#!/usr/bin/env python3
"""
Matplotlib-based time series and 3D trajectory visualization.
Creates static plots for position, velocity, acceleration, orientation, and displacement.

FIXED VERSION:
- Proper ground truth vs prediction handling
- Consistent filtering across all parameters
- Acceleration visualization
- Multi-interval comparison against single GT
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Optional, List, Dict, Tuple
import os
from imuFactors.vis.trajectory_loader import (
    TrajectoryData,
    load_trajectory,
    load_trajectory_from_build,
    discover_intervals,
    compute_acceleration,
)


# Color scheme for preintegration times
INTERVAL_COLORS = {
    '2s': '#E63946',      # Red
    '5s': '#F77F00',      # Orange
    '10s': '#7209B7',     # Purple
}

INTERVAL_LINESTYLES = {
    '2s': '--',
    '5s': ':',
    '10s': '-.',
}

GT_COLOR = '#2E4057'  # Dark blue
GT_LINESTYLE = '-'


def _get_valid_indices(data: np.ndarray, axis: int = 0) -> np.ndarray:
    """Get indices where data is finite and non-zero."""
    if data.shape[1] == 3:
        return np.isfinite(data).all(axis=1) & (np.abs(data).max(axis=1) > 1e-6)
    return np.isfinite(data)


def plot_position_timeseries(
    trajectory: TrajectoryData,
    title: str = "Position Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot X, Y, Z position time series."""
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    labels = ['X', 'Y', 'Z']
    
    for i, (ax, label) in enumerate(zip(axes, labels)):
        # Ground truth
        ax.plot(
            trajectory.timestamp, trajectory.gt_position[:, i],
            color=GT_COLOR, linewidth=3, linestyle=GT_LINESTYLE,
            label='Ground Truth', alpha=0.9, zorder=2
        )
        
        # Predicted
        if trajectory.pred_position is not None:
            valid_idx = _get_valid_indices(trajectory.pred_position)
            if valid_idx.any():
                ax.plot(
                    trajectory.timestamp[valid_idx],
                    trajectory.pred_position[valid_idx, i],
                    color=INTERVAL_COLORS['2s'], linewidth=2, linestyle='--',
                    label='Predicted', alpha=0.8, zorder=1
                )
        
        ax.set_ylabel(f'{label} Position (m)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_velocity_timeseries(
    trajectory: TrajectoryData,
    title: str = "Velocity Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot Vx, Vy, Vz velocity time series."""
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    labels = ['Vx', 'Vy', 'Vz']
    
    for i, (ax, label) in enumerate(zip(axes, labels)):
        # Ground truth
        ax.plot(
            trajectory.timestamp, trajectory.gt_velocity[:, i],
            color=GT_COLOR, linewidth=3, linestyle=GT_LINESTYLE,
            label='Ground Truth', alpha=0.9, zorder=2
        )
        
        # Predicted
        if trajectory.pred_velocity is not None:
            valid_idx = _get_valid_indices(trajectory.pred_velocity)
            if valid_idx.any():
                ax.plot(
                    trajectory.timestamp[valid_idx],
                    trajectory.pred_velocity[valid_idx, i],
                    color=INTERVAL_COLORS['2s'], linewidth=2, linestyle='--',
                    label='Predicted', alpha=0.8, zorder=1
                )
        
        ax.set_ylabel(f'{label} (m/s)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_acceleration_timeseries(
    trajectory: TrajectoryData,
    title: str = "Acceleration Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot Ax, Ay, Az acceleration time series."""
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    # Compute accelerations if not available
    gt_accel = trajectory.gt_acceleration
    if gt_accel is None:
        gt_accel = compute_acceleration(trajectory.gt_velocity, trajectory.timestamp)
    
    pred_accel = trajectory.pred_acceleration
    if pred_accel is None and trajectory.pred_velocity is not None:
        pred_accel = compute_acceleration(trajectory.pred_velocity, trajectory.timestamp)
    
    labels = ['Ax', 'Ay', 'Az']
    accel_time = trajectory.timestamp[:-1]  # One less point due to differentiation
    
    for i, (ax, label) in enumerate(zip(axes, labels)):
        # Ground truth acceleration
        if len(gt_accel) > 0:
            ax.plot(
                accel_time, gt_accel[:, i],
                color=GT_COLOR, linewidth=3, linestyle=GT_LINESTYLE,
                label='Ground Truth', alpha=0.9, zorder=2
            )
        
        # Predicted acceleration
        if pred_accel is not None and len(pred_accel) > 0:
            valid_idx = _get_valid_indices(pred_accel)
            if valid_idx.any():
                ax.plot(
                    accel_time[valid_idx],
                    pred_accel[valid_idx, i],
                    color=INTERVAL_COLORS['2s'], linewidth=2, linestyle='--',
                    label='Predicted', alpha=0.8, zorder=1
                )
        
        ax.set_ylabel(f'{label} (m/s²)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.4)
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_orientation_timeseries(
    trajectory: TrajectoryData,
    title: str = "Orientation Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot Roll, Pitch, Yaw time series."""
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    labels = ['Roll', 'Pitch', 'Yaw']
    
    for i, (ax, label) in enumerate(zip(axes, labels)):
        # Ground truth
        ax.plot(
            trajectory.timestamp, trajectory.gt_rpy[:, i],
            color=GT_COLOR, linewidth=3, linestyle=GT_LINESTYLE,
            label='Ground Truth', alpha=0.9, zorder=2
        )
        
        # Predicted
        if trajectory.pred_rpy is not None:
            valid_idx = np.isfinite(trajectory.pred_rpy[:, i])
            if valid_idx.any():
                ax.plot(
                    trajectory.timestamp[valid_idx],
                    trajectory.pred_rpy[valid_idx, i],
                    color=INTERVAL_COLORS['2s'], linewidth=2, linestyle='--',
                    label='Predicted', alpha=0.8, zorder=1
                )
        
        ax.set_ylabel(f'{label} (deg)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_displacement_timeseries(
    trajectory: TrajectoryData,
    title: str = "Displacement Time Series",
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot displacement (delta position) time series."""
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    # Compute displacements
    gt_disp = np.diff(trajectory.gt_position, axis=0)
    gt_times = trajectory.timestamp[:-1]
    
    labels = ['ΔX', 'ΔY', 'ΔZ']
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth displacement
        ax.plot(
            gt_times, gt_disp[:, i],
            color=GT_COLOR, linewidth=3, linestyle=GT_LINESTYLE,
            label='Ground Truth Δ', alpha=0.9, zorder=2
        )
        
        # Predicted displacement
        if trajectory.pred_position is not None:
            pred_disp = np.diff(trajectory.pred_position, axis=0)
            pred_times = trajectory.timestamp[:-1]
            valid_idx = _get_valid_indices(pred_disp)
            if valid_idx.any():
                ax.plot(
                    pred_times[valid_idx],
                    pred_disp[valid_idx, i],
                    color=INTERVAL_COLORS['2s'], linewidth=2, linestyle='--',
                    label='Predicted Δ', alpha=0.8, zorder=1
                )
        
        ax.set_ylabel(f'{label} (m)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9, loc='best')
        ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.4)
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_3d_trajectory(
    trajectory: TrajectoryData,
    title: str = "3D Trajectory",
    save_path: Optional[str] = None,
    downsample: int = 1
) -> plt.Figure:
    """Plot 3D trajectory using matplotlib (static)."""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    ds = downsample
    
    # Ground truth
    ax.plot(
        trajectory.gt_position[::ds, 0],
        trajectory.gt_position[::ds, 1],
        trajectory.gt_position[::ds, 2],
        color=GT_COLOR, linewidth=4, linestyle=':',
        label='Ground Truth', alpha=0.85, zorder=2
    )
    
    # Predicted
    if trajectory.pred_position is not None:
        valid_idx = _get_valid_indices(trajectory.pred_position)
        if valid_idx.any():
            valid_pos = trajectory.pred_position[valid_idx][::ds]
            ax.plot(
                valid_pos[:, 0],
                valid_pos[:, 1],
                valid_pos[:, 2],
                color=INTERVAL_COLORS['2s'], linewidth=2.5, linestyle='-',
                label='Predicted', alpha=0.85, zorder=1
            )
    
    # Start/end markers
    ax.scatter(
        trajectory.gt_position[0, 0],
        trajectory.gt_position[0, 1],
        trajectory.gt_position[0, 2],
        c='#06A77D', s=150, marker='o', edgecolors='white', linewidths=2,
        label='Start', zorder=3
    )
    ax.scatter(
        trajectory.gt_position[-1, 0],
        trajectory.gt_position[-1, 1],
        trajectory.gt_position[-1, 2],
        c='#1E88E5', s=150, marker='s', edgecolors='white', linewidths=2,
        label='End', zorder=3
    )
    
    ax.set_xlabel('X (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=11, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=11, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(fontsize=10, loc='best')
    ax.grid(True, alpha=0.25, linestyle='--')
    ax.set_facecolor('#FAFAFA')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_position_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = ".",
    intervals: Optional[List[str]] = None,
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Plot position time series comparing multiple preintegration intervals against GT.
    
    THIS IS THE MAIN FUNCTION YOU NEED - compares all intervals against a single GT.
    """
    if intervals is None:
        intervals = ['2s', '5s', '10s']
    
    fig, axes = plt.subplots(3, 1, figsize=(16, 11), sharex=True)
    fig.suptitle(
        f'Position Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}\nGround Truth vs All Preintegration Times',
        fontsize=14, fontweight='bold'
    )
    
    # Load ground truth from first available trajectory (all have same GT)
    gt_traj = load_trajectory_from_build(filter_name, dataset_name, intervals[0], build_dir)
    if gt_traj is None:
        print(f"✗ Could not load trajectory for {filter_name} - {dataset_name}")
        return fig
    
    labels = ['X', 'Y', 'Z']
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth - bold solid line
        ax.plot(
            gt_traj.timestamp, gt_traj.gt_position[:, i],
            color=GT_COLOR, linewidth=3.5, linestyle=GT_LINESTYLE,
            label='Ground Truth', alpha=0.95, zorder=10
        )
        
        # Predictions for each interval
        for interval in intervals:
            traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
            if traj is None or traj.pred_position is None:
                continue
            
            valid_idx = _get_valid_indices(traj.pred_position)
            if not valid_idx.any():
                continue
            
            color = INTERVAL_COLORS.get(interval, '#888888')
            linestyle = INTERVAL_LINESTYLES.get(interval, '--')
            
            ax.plot(
                traj.timestamp[valid_idx],
                traj.pred_position[valid_idx, i],
                color=color, linewidth=2.2, linestyle=linestyle,
                label=f'{filter_name.upper()} ({interval})', alpha=0.85, zorder=5-intervals.index(interval)
            )
        
        ax.set_ylabel(f'{label} Position (m)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9, loc='best', framealpha=0.95)
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_velocity_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = ".",
    intervals: Optional[List[str]] = None,
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot velocity time series comparing multiple preintegration intervals against GT."""
    if intervals is None:
        intervals = ['2s', '5s', '10s']
    
    fig, axes = plt.subplots(3, 1, figsize=(16, 11), sharex=True)
    fig.suptitle(
        f'Velocity Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}\nGround Truth vs All Preintegration Times',
        fontsize=14, fontweight='bold'
    )
    
    gt_traj = load_trajectory_from_build(filter_name, dataset_name, intervals[0], build_dir)
    if gt_traj is None:
        print(f"✗ Could not load trajectory for {filter_name} - {dataset_name}")
        return fig
    
    labels = ['Vx', 'Vy', 'Vz']
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth
        ax.plot(
            gt_traj.timestamp, gt_traj.gt_velocity[:, i],
            color=GT_COLOR, linewidth=3.5, linestyle=GT_LINESTYLE,
            label='Ground Truth', alpha=0.95, zorder=10
        )
        
        # Predictions for each interval
        for interval in intervals:
            traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
            if traj is None or traj.pred_velocity is None:
                continue
            
            valid_idx = _get_valid_indices(traj.pred_velocity)
            if not valid_idx.any():
                continue
            
            color = INTERVAL_COLORS.get(interval, '#888888')
            linestyle = INTERVAL_LINESTYLES.get(interval, '--')
            
            ax.plot(
                traj.timestamp[valid_idx],
                traj.pred_velocity[valid_idx, i],
                color=color, linewidth=2.2, linestyle=linestyle,
                label=f'{filter_name.upper()} ({interval})', alpha=0.85, zorder=5-intervals.index(interval)
            )
        
        ax.set_ylabel(f'{label} (m/s)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9, loc='best', framealpha=0.95)
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_acceleration_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = ".",
    intervals: Optional[List[str]] = None,
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot acceleration time series comparing multiple preintegration intervals against GT."""
    if intervals is None:
        intervals = ['2s', '5s', '10s']
    
    fig, axes = plt.subplots(3, 1, figsize=(16, 11), sharex=True)
    fig.suptitle(
        f'Acceleration Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}\nGround Truth vs All Preintegration Times',
        fontsize=14, fontweight='bold'
    )
    
    gt_traj = load_trajectory_from_build(filter_name, dataset_name, intervals[0], build_dir, compute_accel=True)
    if gt_traj is None:
        print(f"✗ Could not load trajectory for {filter_name} - {dataset_name}")
        return fig
    
    # Compute GT acceleration
    gt_accel = gt_traj.gt_acceleration
    if gt_accel is None:
        gt_accel = compute_acceleration(gt_traj.gt_velocity, gt_traj.timestamp)
    
    accel_time = gt_traj.timestamp[:-1]
    labels = ['Ax', 'Ay', 'Az']
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth
        if len(gt_accel) > 0:
            ax.plot(
                accel_time, gt_accel[:, i],
                color=GT_COLOR, linewidth=3.5, linestyle=GT_LINESTYLE,
                label='Ground Truth', alpha=0.95, zorder=10
            )
        
        # Predictions for each interval
        for interval in intervals:
            traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir, compute_accel=True)
            if traj is None or traj.pred_velocity is None:
                continue
            
            pred_accel = traj.pred_acceleration
            if pred_accel is None:
                pred_accel = compute_acceleration(traj.pred_velocity, traj.timestamp)
            
            if len(pred_accel) == 0:
                continue
            
            valid_idx = _get_valid_indices(pred_accel)
            if not valid_idx.any():
                continue
            
            color = INTERVAL_COLORS.get(interval, '#888888')
            linestyle = INTERVAL_LINESTYLES.get(interval, '--')
            
            ax.plot(
                accel_time[valid_idx],
                pred_accel[valid_idx, i],
                color=color, linewidth=2.2, linestyle=linestyle,
                label=f'{filter_name.upper()} ({interval})', alpha=0.85, zorder=5-intervals.index(interval)
            )
        
        ax.set_ylabel(f'{label} (m/s²)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
        ax.legend(fontsize=9, loc='best', framealpha=0.95)
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_orientation_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = ".",
    intervals: Optional[List[str]] = None,
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot orientation (RPY) time series comparing multiple preintegration intervals against GT."""
    if intervals is None:
        intervals = ['2s', '5s', '10s']
    
    fig, axes = plt.subplots(3, 1, figsize=(16, 11), sharex=True)
    fig.suptitle(
        f'Orientation Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}\nGround Truth vs All Preintegration Times',
        fontsize=14, fontweight='bold'
    )
    
    gt_traj = load_trajectory_from_build(filter_name, dataset_name, intervals[0], build_dir)
    if gt_traj is None:
        print(f"✗ Could not load trajectory for {filter_name} - {dataset_name}")
        return fig
    
    labels = ['Roll', 'Pitch', 'Yaw']
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth
        ax.plot(
            gt_traj.timestamp, gt_traj.gt_rpy[:, i],
            color=GT_COLOR, linewidth=3.5, linestyle=GT_LINESTYLE,
            label='Ground Truth', alpha=0.95, zorder=10
        )
        
        # Predictions for each interval
        for interval in intervals:
            traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
            if traj is None or traj.pred_rpy is None:
                continue
            
            valid_idx = np.isfinite(traj.pred_rpy[:, i])
            if not valid_idx.any():
                continue
            
            color = INTERVAL_COLORS.get(interval, '#888888')
            linestyle = INTERVAL_LINESTYLES.get(interval, '--')
            
            ax.plot(
                traj.timestamp[valid_idx],
                traj.pred_rpy[valid_idx, i],
                color=color, linewidth=2.2, linestyle=linestyle,
                label=f'{filter_name.upper()} ({interval})', alpha=0.85, zorder=5-intervals.index(interval)
            )
        
        ax.set_ylabel(f'{label} (deg)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9, loc='best', framealpha=0.95)
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_displacement_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = ".",
    intervals: Optional[List[str]] = None,
    save_path: Optional[str] = None
) -> plt.Figure:
    """Plot displacement time series comparing multiple preintegration intervals against GT."""
    if intervals is None:
        intervals = ['2s', '5s', '10s']
    
    fig, axes = plt.subplots(3, 1, figsize=(16, 11), sharex=True)
    fig.suptitle(
        f'Displacement Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}\nGround Truth vs All Preintegration Times',
        fontsize=14, fontweight='bold'
    )
    
    gt_traj = load_trajectory_from_build(filter_name, dataset_name, intervals[0], build_dir)
    if gt_traj is None:
        print(f"✗ Could not load trajectory for {filter_name} - {dataset_name}")
        return fig
    
    gt_disp = np.diff(gt_traj.gt_position, axis=0)
    gt_times = gt_traj.timestamp[:-1]
    labels = ['ΔX', 'ΔY', 'ΔZ']
    
    for i, label in enumerate(labels):
        ax = axes[i]
        
        # Ground truth
        ax.plot(
            gt_times, gt_disp[:, i],
            color=GT_COLOR, linewidth=3.5, linestyle=GT_LINESTYLE,
            label='Ground Truth Δ', alpha=0.95, zorder=10
        )
        
        # Predictions for each interval
        for interval in intervals:
            traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
            if traj is None or traj.pred_position is None:
                continue
            
            pred_disp = np.diff(traj.pred_position, axis=0)
            pred_times = traj.timestamp[:-1]
            valid_idx = _get_valid_indices(pred_disp)
            if not valid_idx.any():
                continue
            
            color = INTERVAL_COLORS.get(interval, '#888888')
            linestyle = INTERVAL_LINESTYLES.get(interval, '--')
            
            ax.plot(
                pred_times[valid_idx],
                pred_disp[valid_idx, i],
                color=color, linewidth=2.2, linestyle=linestyle,
                label=f'{filter_name.upper()} ({interval})', alpha=0.85, zorder=5-intervals.index(interval)
            )
        
        ax.set_ylabel(f'{label} (m)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
        ax.legend(fontsize=9, loc='best', framealpha=0.95)
        ax.set_facecolor('#FAFAFA')
    
    axes[-1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig


def plot_3d_trajectory_multi_interval(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = ".",
    intervals: Optional[List[str]] = None,
    save_path: Optional[str] = None,
    downsample: int = 1
) -> plt.Figure:
    """Plot 3D trajectories comparing multiple preintegration intervals against GT."""
    if intervals is None:
        intervals = ['2s', '5s', '10s']
    
    fig = plt.figure(figsize=(14, 11))
    ax = fig.add_subplot(111, projection='3d')
    
    gt_traj = load_trajectory_from_build(filter_name, dataset_name, intervals[0], build_dir)
    if gt_traj is None:
        print(f"✗ Could not load trajectory for {filter_name} - {dataset_name}")
        return fig
    
    ds = downsample
    
    # Ground truth - dotted line
    ax.plot(
        gt_traj.gt_position[::ds, 0],
        gt_traj.gt_position[::ds, 1],
        gt_traj.gt_position[::ds, 2],
        color=GT_COLOR, linewidth=4, linestyle=':',
        label='Ground Truth', alpha=0.85, zorder=10
    )
    
    # Predictions for each interval
    for interval in intervals:
        traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
        if traj is None or traj.pred_position is None:
            continue
        
        valid_idx = _get_valid_indices(traj.pred_position)
        if not valid_idx.any():
            continue
        
        valid_pos = traj.pred_position[valid_idx][::ds]
        color = INTERVAL_COLORS.get(interval, '#888888')
        
        ax.plot(
            valid_pos[:, 0],
            valid_pos[:, 1],
            valid_pos[:, 2],
            color=color, linewidth=2.5, linestyle='-',
            label=f'{filter_name.upper()} ({interval})', alpha=0.85, zorder=5-intervals.index(interval)
        )
    
    # Start/end markers
    ax.scatter(
        gt_traj.gt_position[0, 0],
        gt_traj.gt_position[0, 1],
        gt_traj.gt_position[0, 2],
        c='#06A77D', s=150, marker='o', edgecolors='white', linewidths=2,
        label='Start', zorder=15
    )
    ax.scatter(
        gt_traj.gt_position[-1, 0],
        gt_traj.gt_position[-1, 1],
        gt_traj.gt_position[-1, 2],
        c='#1E88E5', s=150, marker='s', edgecolors='white', linewidths=2,
        label='End', zorder=15
    )
    
    ax.set_xlabel('X (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=11, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=11, fontweight='bold')
    ax.set_title(
        f'3D Trajectory: {filter_name.upper()} - {dataset_name}\nGround Truth vs All Preintegration Times',
        fontsize=13, fontweight='bold'
    )
    ax.legend(fontsize=10, loc='best')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_facecolor('#FAFAFA')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
    
    return fig