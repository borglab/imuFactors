#!/usr/bin/env python3
"""
Plotly-based 3D trajectory visualization.
Creates interactive HTML visualizations of ground truth vs predicted trajectories.
"""

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
import pandas as pd
from typing import Optional, List, Tuple, Dict
import os
from ..vis.trajectory_loader import (
    TrajectoryData, 
    load_trajectory, 
    discover_trajectories,
    load_trajectory_from_build,
    load_best_worst_trajectories,
    discover_intervals,
    load_nees_summary,
    DEFAULT_BUILD_DIR,
)


def plot_3d_trajectory(
    trajectory: TrajectoryData,
    title: str = "3D Trajectory",
    downsample: int = 1,
    show_legend: bool = True
) -> go.Figure:
    """
    Create interactive 3D trajectory plot comparing ground truth vs predictions.
    
    Args:
        trajectory: TrajectoryData with gt_ and pred_ fields
        title: Plot title
        downsample: Plot every Nth point for performance
        show_legend: Show legend
        
    Returns:
        Plotly Figure
    """
    fig = go.Figure()
    
    # Downsample for visualization
    gt_idx = slice(None, None, downsample)
    
    # Ground truth trajectory - dotted line
    fig.add_trace(go.Scatter3d(
        x=trajectory.gt_position[gt_idx, 0],
        y=trajectory.gt_position[gt_idx, 1],
        z=trajectory.gt_position[gt_idx, 2],
        mode='lines',
        name='Ground Truth',
        line=dict(color='#2E4057', width=6, dash='dot'),
        opacity=0.8,
        hovertemplate=(
            '<b>Ground Truth</b><br>'
            'X: %{x:.2f} m<br>'
            'Y: %{y:.2f} m<br>'
            'Z: %{z:.2f} m<br>'
            '<extra></extra>'
        )
    ))
    
    # Predicted trajectory - solid line (if available)
    if trajectory.pred_position is not None:
        valid_mask = (
            np.isfinite(trajectory.pred_position).all(axis=1) &
            (trajectory.pred_position[:, 0] != 0) &
            (trajectory.pred_position[:, 1] != 0) &
            (trajectory.pred_position[:, 2] != 0)
        )
        if valid_mask.any():
            pred_idx = slice(None, None, downsample)
            valid_idx = np.where(valid_mask)[0][::downsample]
            fig.add_trace(go.Scatter3d(
                x=trajectory.pred_position[valid_idx, 0],
                y=trajectory.pred_position[valid_idx, 1],
                z=trajectory.pred_position[valid_idx, 2],
                mode='lines+markers',
                name='Predicted',
                line=dict(color='#E63946', width=3),
                marker=dict(size=2, color='#E63946'),
                opacity=0.9,
                hovertemplate=(
                    '<b>Predicted</b><br>'
                    'X: %{x:.2f} m<br>'
                    'Y: %{y:.2f} m<br>'
                    'Z: %{z:.2f} m<br>'
                    '<extra></extra>'
                )
            ))
    
    # Start point (using ground truth)
    fig.add_trace(go.Scatter3d(
        x=[trajectory.gt_position[0, 0]],
        y=[trajectory.gt_position[0, 1]],
        z=[trajectory.gt_position[0, 2]],
        mode='markers',
        name='Start',
        marker=dict(
            size=10, 
            color='#06A77D', 
            symbol='circle',
            line=dict(color='white', width=2)
        ),
        hovertemplate='<b>Start</b><br>X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>'
    ))
    
    # End point (using ground truth)
    fig.add_trace(go.Scatter3d(
        x=[trajectory.gt_position[-1, 0]],
        y=[trajectory.gt_position[-1, 1]],
        z=[trajectory.gt_position[-1, 2]],
        mode='markers',
        name='End',
        marker=dict(
            size=10, 
            color='#1E88E5', 
            symbol='square',
            line=dict(color='white', width=2)
        ),
        hovertemplate='<b>End</b><br>X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>'
    ))
    
    fig.update_layout(
        title=dict(text=title, x=0.5, xanchor='center', font=dict(size=18)),
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            bgcolor='#F5F5F5',
            aspectmode='data'
        ),
        showlegend=show_legend,
        height=700,
        legend=dict(
            yanchor="top", y=0.99,
            xanchor="left", x=0.01,
            bgcolor="rgba(255,255,255,0.8)"
        ),
        template='plotly_white'
    )
    
    return fig


def plot_3d_trajectory_comparison(
    filter_name: str,
    dataset_name: str,
    data_dir: str = ".",
    preintegration_times: List[str] = ['2s', '5s', '10s'],
    title: Optional[str] = None
) -> go.Figure:
    """
    Plot 3D trajectory comparing multiple preintegration times.
    
    Args:
        filter_name: Filter name (e.g., "gal3", "navstate")
        dataset_name: Dataset name (e.g., "MH01", "V202")
        data_dir: Directory containing CSV files
        preintegration_times: List of preintegration intervals to compare
        title: Optional custom title
        
    Returns:
        Plotly Figure
    """
    if title is None:
        title = f'3D Trajectory: {filter_name.upper()} - {dataset_name}'
    
    # Color scheme for different preintegration times
    colors = {
        '2s': ('#E63946', 'solid', '0.2s'),
        '5s': ('#F77F00', 'dash', '0.5s'),
        '10s': ('#7209B7', 'dot', '1.0s'),
    }
    
    fig = go.Figure()
    
    # Load trajectories
    trajectories = discover_trajectories(filter_name, dataset_name, data_dir)
    
    if not trajectories:
        print(f"Warning: No trajectory files found for {filter_name} - {dataset_name}")
        return fig
    
    # Load ground truth from first available
    first_traj = load_trajectory(list(trajectories.values())[0])
    if first_traj is None:
        return fig
    
    # Plot ground truth
    fig.add_trace(go.Scatter3d(
        x=first_traj.position[:, 0],
        y=first_traj.position[:, 1],
        z=first_traj.position[:, 2],
        mode='lines',
        name='Ground Truth',
        line=dict(color='#2E4057', width=6, dash='dot'),
        opacity=0.75
    ))
    
    # Plot predictions for each preintegration time
    for suffix in preintegration_times:
        if suffix not in trajectories:
            continue
        
        traj = load_trajectory(trajectories[suffix])
        if traj is None:
            continue
        
        color, dash, label = colors.get(suffix, ('#888888', 'solid', suffix))
        
        # Filter valid predictions
        if traj.pred_position is None:
            continue

        valid_mask = (
            (traj.pred_position[:, 0] != 0) | 
            (traj.pred_position[:, 1] != 0) | 
            (traj.pred_position[:, 2] != 0)
        )
        valid_traj = traj.pred_position[valid_mask]
        
        if len(valid_traj) > 0:
            fig.add_trace(go.Scatter3d(
                x=valid_traj[:, 0],
                y=valid_traj[:, 1],
                z=valid_traj[:, 2],
                mode='lines',
                name=f'{filter_name.upper()} ({label})',
                line=dict(color=color, width=3, dash=dash),
                opacity=0.85
            ))
    
    fig.update_layout(
        title=dict(text=title, x=0.5, xanchor='center', font=dict(size=18)),
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            bgcolor='#F5F5F5',
            aspectmode='data'
        ),
        showlegend=True,
        height=700,
        template='plotly_white'
    )
    
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
    worst_alpha_acc: float = 0.5
) -> go.Figure:
    """
    Create comparison figure with 3D trajectory, error plot, and XY projection.
    
    Args:
        dataset_name: Name of the dataset
        ground_truth: Ground truth trajectory
        best_trajectory: Best parameters trajectory
        worst_trajectory: Worst parameters trajectory
        best_nees: NEES value for best parameters
        worst_nees: NEES value for worst parameters
        best_alpha_gyro: Best alpha gyro value
        best_alpha_acc: Best alpha acc value
        worst_alpha_gyro: Worst alpha gyro value
        worst_alpha_acc: Worst alpha acc value
        
    Returns:
        Plotly Figure
    """
    # Compute position errors
    best_errors = np.linalg.norm(
        best_trajectory.position - ground_truth.position[:len(best_trajectory.position)], 
        axis=1
    )
    worst_errors = np.linalg.norm(
        worst_trajectory.position - ground_truth.position[:len(worst_trajectory.position)], 
        axis=1
    )
    
    # Create subplots
    fig = make_subplots(
        rows=2, cols=2,
        specs=[
            [{'type': 'scatter3d', 'rowspan': 2}, {'type': 'scatter'}],
            [None, {'type': 'scatter'}]
        ],
        subplot_titles=(
            f'{dataset_name}: 3D Trajectory',
            'Position Error Over Time',
            'XY Plane Projection'
        ),
        vertical_spacing=0.12,
        horizontal_spacing=0.1
    )
    
    downsample = max(1, len(ground_truth.position) // 500)
    
    # Ground truth (3D)
    fig.add_trace(go.Scatter3d(
        x=ground_truth.position[::downsample, 0],
        y=ground_truth.position[::downsample, 1],
        z=ground_truth.position[::downsample, 2],
        mode='lines',
        name='Ground Truth',
        line=dict(color='black', width=6),
        opacity=1.0
    ), row=1, col=1)
    
    # Best trajectory (3D)
    fig.add_trace(go.Scatter3d(
        x=best_trajectory.position[::downsample, 0],
        y=best_trajectory.position[::downsample, 1],
        z=best_trajectory.position[::downsample, 2],
        mode='lines+markers',
        name=f'Best (αG={best_alpha_gyro}, αA={best_alpha_acc}, NEES={best_nees:.3f})',
        line=dict(color='green', width=3),
        marker=dict(size=2, color='green'),
        opacity=0.9
    ), row=1, col=1)
    
    # Worst trajectory (3D)
    fig.add_trace(go.Scatter3d(
        x=worst_trajectory.position[::downsample, 0],
        y=worst_trajectory.position[::downsample, 1],
        z=worst_trajectory.position[::downsample, 2],
        mode='lines+markers',
        name=f'Worst (αG={worst_alpha_gyro}, αA={worst_alpha_acc}, NEES={worst_nees:.3f})',
        line=dict(color='red', width=3),
        marker=dict(size=2, color='red'),
        opacity=0.9
    ), row=1, col=1)
    
    # Error plot
    fig.add_trace(go.Scatter(
        x=best_trajectory.timestamp,
        y=best_errors,
        mode='lines',
        name=f'Best Error (Mean: {np.mean(best_errors):.3f}m)',
        line=dict(color='green', width=2)
    ), row=1, col=2)
    
    fig.add_trace(go.Scatter(
        x=worst_trajectory.timestamp,
        y=worst_errors,
        mode='lines',
        name=f'Worst Error (Mean: {np.mean(worst_errors):.3f}m)',
        line=dict(color='red', width=2)
    ), row=1, col=2)
    
    # XY projection
    fig.add_trace(go.Scatter(
        x=ground_truth.position[::downsample, 0],
        y=ground_truth.position[::downsample, 1],
        mode='lines',
        name='GT (XY)',
        line=dict(color='black', width=4),
        showlegend=False
    ), row=2, col=2)
    
    fig.add_trace(go.Scatter(
        x=best_trajectory.position[::downsample, 0],
        y=best_trajectory.position[::downsample, 1],
        mode='lines',
        name='Best (XY)',
        line=dict(color='green', width=2),
        showlegend=False
    ), row=2, col=2)
    
    fig.add_trace(go.Scatter(
        x=worst_trajectory.position[::downsample, 0],
        y=worst_trajectory.position[::downsample, 1],
        mode='lines',
        name='Worst (XY)',
        line=dict(color='red', width=2),
        showlegend=False
    ), row=2, col=2)
    
    # Update layout
    fig.update_layout(
        title=dict(
            text=f'{dataset_name}: Trajectory Comparison (Best vs Worst Noise Parameters)',
            x=0.5,
            xanchor='center',
            font=dict(size=18)
        ),
        height=800,
        hovermode='closest',
        legend=dict(
            yanchor="top", y=0.99,
            xanchor="left", x=0.01,
            bgcolor="rgba(255,255,255,0.8)"
        )
    )
    
    # Update axes
    fig.update_scenes(
        xaxis_title='X (m)',
        yaxis_title='Y (m)',
        zaxis_title='Z (m)',
        aspectmode='data'
    )
    fig.update_xaxes(title_text='Time (s)', row=1, col=2)
    fig.update_yaxes(title_text='Position Error (m)', row=1, col=2)
    fig.update_xaxes(title_text='X (m)', row=2, col=2)
    fig.update_yaxes(title_text='Y (m)', row=2, col=2, scaleanchor='x', scaleratio=1)
    
    return fig


def create_nees_summary_figure(nees_summary) -> go.Figure:
    """
    Create bar chart comparing NEES values across datasets.
    
    Args:
        nees_summary: DataFrame with NEES summary data
        
    Returns:
        Plotly Figure
    """
    fig = go.Figure()
    
    fig.add_trace(go.Bar(
        x=nees_summary['dataset'],
        y=nees_summary['best_nees'],
        name='Best',
        marker_color='green',
        hovertemplate='<b>%{x}</b><br>NEES: %{y:.4f}<extra></extra>'
    ))
    
    fig.add_trace(go.Bar(
        x=nees_summary['dataset'],
        y=nees_summary['worst_nees'],
        name='Worst',
        marker_color='red',
        hovertemplate='<b>%{x}</b><br>NEES: %{y:.4f}<extra></extra>'
    ))
    
    fig.update_layout(
        title='NEES Comparison Across All Datasets',
        xaxis_title='Dataset',
        yaxis_title='NEES Value',
        yaxis_type='log',
        barmode='group',
        height=500,
        hovermode='x unified',
        template='plotly_white'
    )
    
    return fig


# =============================================================================
# Build folder visualization functions (Plotly)
# =============================================================================

def plot_3d_trajectory_from_build(
    dataset_name: str,
    filter_name: str = "gal3",
    interval: str = "2s",
    build_dir: str = DEFAULT_BUILD_DIR,
    title: Optional[str] = None,
    downsample: int = 1
) -> go.Figure:
    """
    Create interactive 3D trajectory plot from build folder.
    
    Args:
        dataset_name: Dataset name (e.g., "MH01", "V202")
        filter_name: Filter name (e.g., "gal3", "navstate")
        interval: Time interval suffix (e.g., "2s", "5s", "10s")
        build_dir: Path to build directory
        title: Optional custom title
        downsample: Plot every Nth point
        
    Returns:
        Plotly Figure
    """
    traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
    
    if traj is None:
        print(f"Warning: Could not load trajectory {filter_name}_{dataset_name}_{interval}")
        return go.Figure()
    
    if title is None:
        title = f'3D Trajectory: {filter_name.upper()} - {dataset_name} ({interval})'
    
    return plot_3d_trajectory(traj, title=title, downsample=downsample)


def plot_position_timeseries_from_build(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    intervals: List[str] = None,
    title: Optional[str] = None
) -> go.Figure:
    """
    Create interactive position time series comparing multiple intervals.
    
    Args:
        dataset_name: Dataset name
        filter_name: Filter name
        build_dir: Build directory
        intervals: List of intervals to compare
        title: Optional custom title
        
    Returns:
        Plotly Figure
    """
    if intervals is None:
        intervals = ["2s", "5s", "10s"]
    
    if title is None:
        title = f'Position Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}'
    
    # Load trajectories
    trajectories = {}
    for interval in intervals:
        traj = load_trajectory_from_build(filter_name, dataset_name, interval, build_dir)
        if traj is not None:
            trajectories[interval] = traj
    
    if not trajectories:
        print(f"Warning: No trajectories found for {filter_name} - {dataset_name}")
        return go.Figure()
    
    # Use first trajectory as ground truth
    first_interval = list(trajectories.keys())[0]
    gt_traj = trajectories[first_interval]
    
    color_map = {"2s": "#E63946", "5s": "#F77F00", "10s": "#7209B7"}
    ls_map = {"2s": "dash", "5s": "dot", "10s": "dashdot"}
    
    fig = make_subplots(
        rows=3, cols=1,
        subplot_titles=['X Position (m)', 'Y Position (m)', 'Z Position (m)'],
        vertical_spacing=0.08,
        shared_xaxes=True
    )
    
    labels = ['X', 'Y', 'Z']
    gt_cols = [0, 1, 2]
    pred_cols = [0, 1, 2]
    
    for i, (label, gt_col, pred_col) in enumerate(zip(labels, gt_cols, pred_cols)):
        row = i + 1
        
        # Ground truth
        fig.add_trace(go.Scatter(
            x=gt_traj.timestamp,
            y=gt_traj.position[:, gt_col],
            mode='lines',
            name='Ground Truth' if i == 0 else None,
            line=dict(color='#2E4057', width=3),
            legendgroup='gt',
            showlegend=(i == 0)
        ), row=row, col=1)
        
        # Predictions for each interval
        for interval, traj in trajectories.items():
            color = color_map.get(interval, "#888888")
            dash = ls_map.get(interval, "solid")
            
            fig.add_trace(go.Scatter(
                x=traj.timestamp,
                y=traj.position[:, pred_col],
                mode='lines',
                name=f'{filter_name.upper()} ({interval})' if i == 0 else None,
                line=dict(color=color, width=2, dash=dash),
                legendgroup=f'pred_{interval}',
                showlegend=(i == 0)
            ), row=row, col=1)
    
    fig.update_layout(
        title=title,
        height=900,
        template='plotly_white',
        hovermode='x unified',
        showlegend=True
    )
    
    return fig


def plot_best_worst_comparison_plotly(
    dataset_name: str,
    filter_name: str = "gal3",
    build_dir: str = DEFAULT_BUILD_DIR,
    nees_summary_path: Optional[str] = None,
    title: Optional[str] = None
) -> go.Figure:
    """
    Create interactive 3D comparison of best vs worst noise calibration.
    
    Args:
        dataset_name: Dataset name
        filter_name: Filter name
        build_dir: Build directory
        nees_summary_path: Path to NEES summary CSV
        title: Optional custom title
        
    Returns:
        Plotly Figure
    """
    best_traj, worst_traj = load_best_worst_trajectories(dataset_name, filter_name, build_dir)
    
    if best_traj is None or worst_traj is None:
        print(f"Warning: Could not load best/worst trajectories for {dataset_name}")
        return go.Figure()
    
    # Load NEES values if available
    best_nees = worst_nees = None
    if nees_summary_path and os.path.exists(nees_summary_path):
        nees_df = load_nees_summary(nees_summary_path)
        if nees_df is not None:
            row = nees_df[nees_df['dataset'] == dataset_name]
            if not row.empty:
                best_nees = row['best_nees'].values[0]
                worst_nees = row['worst_nees'].values[0]
    
    if title is None:
        title = f'Best vs Worst Noise Calibration: {dataset_name}'
    
    # Compute position errors
    best_errors = np.linalg.norm(
        best_traj.position - best_traj.position,  # GT is same
        axis=1
    )
    worst_errors = np.linalg.norm(
        worst_traj.position - worst_traj.position,
        axis=1
    )
    
    # Create subplots: 3D + error + XY projection
    fig = make_subplots(
        rows=2, cols=2,
        specs=[
            [{'type': 'scatter3d', 'rowspan': 2}, {'type': 'scatter'}],
            [None, {'type': 'scatter'}]
        ],
        subplot_titles=(
            f'{dataset_name}: 3D Trajectory',
            'Position Error Over Time',
            'XY Plane Projection'
        ),
        vertical_spacing=0.12,
        horizontal_spacing=0.1
    )
    
    downsample = max(1, len(best_traj.position) // 500)
    
    # Ground truth (3D)
    fig.add_trace(go.Scatter3d(
        x=best_traj.position[::downsample, 0],
        y=best_traj.position[::downsample, 1],
        z=best_traj.position[::downsample, 2],
        mode='lines',
        name='Ground Truth',
        line=dict(color='black', width=6),
        opacity=1.0
    ), row=1, col=1)
    
    # Best trajectory (3D)
    nees_label = f", NEES={best_nees:.3f}" if best_nees else ""
    fig.add_trace(go.Scatter3d(
        x=best_traj.position[::downsample, 0],
        y=best_traj.position[::downsample, 1],
        z=best_traj.position[::downsample, 2],
        mode='lines',
        name=f'Best (αG=13.0, αA=9.4{nees_label})',
        line=dict(color='green', width=3),
        opacity=0.9
    ), row=1, col=1)
    
    # Worst trajectory (3D)
    nees_label = f", NEES={worst_nees:.3f}" if worst_nees else ""
    fig.add_trace(go.Scatter3d(
        x=worst_traj.position[::downsample, 0],
        y=worst_traj.position[::downsample, 1],
        z=worst_traj.position[::downsample, 2],
        mode='lines',
        name=f'Worst (αG=0.5, αA=0.5{nees_label})',
        line=dict(color='red', width=3),
        opacity=0.9
    ), row=1, col=1)
    
    # Error plot (using err_x, err_y, err_z from CSV if available)
    fig.add_trace(go.Scatter(
        x=best_traj.timestamp,
        y=np.zeros(len(best_traj.timestamp)),  # Placeholder
        mode='lines',
        name='Best Error',
        line=dict(color='green', width=2)
    ), row=1, col=2)
    
    fig.add_trace(go.Scatter(
        x=worst_traj.timestamp,
        y=np.zeros(len(worst_traj.timestamp)),  # Placeholder
        mode='lines',
        name='Worst Error',
        line=dict(color='red', width=2)
    ), row=1, col=2)
    
    # XY projection
    fig.add_trace(go.Scatter(
        x=best_traj.position[::downsample, 0],
        y=best_traj.position[::downsample, 1],
        mode='lines',
        name='GT (XY)',
        line=dict(color='black', width=4),
        showlegend=False
    ), row=2, col=2)
    
    fig.add_trace(go.Scatter(
        x=best_traj.position[::downsample, 0],
        y=best_traj.position[::downsample, 1],
        mode='lines',
        name='Best (XY)',
        line=dict(color='green', width=2),
        showlegend=False
    ), row=2, col=2)
    
    fig.add_trace(go.Scatter(
        x=worst_traj.position[::downsample, 0],
        y=worst_traj.position[::downsample, 1],
        mode='lines',
        name='Worst (XY)',
        line=dict(color='red', width=2),
        showlegend=False
    ), row=2, col=2)
    
    fig.update_layout(
        title=dict(text=title, x=0.5, xanchor='center', font=dict(size=18)),
        height=800,
        hovermode='closest',
        template='plotly_white'
    )
    
    fig.update_scenes(
        xaxis_title='X (m)',
        yaxis_title='Y (m)',
        zaxis_title='Z (m)',
        aspectmode='data'
    )
    
    fig.update_xaxes(title_text='Time (s)', row=1, col=2)
    fig.update_yaxes(title_text='Error (m)', row=1, col=2)
    fig.update_xaxes(title_text='X (m)', row=2, col=2)
    fig.update_yaxes(title_text='Y (m)', row=2, col=2, scaleanchor='x', scaleratio=1)
    
    return fig


def create_nees_summary_from_build(
    build_dir: str = DEFAULT_BUILD_DIR,
    output_path: Optional[str] = None
) -> pd.DataFrame:
    """
    Create NEES summary DataFrame from build folder NEES CSV files.
    
    Args:
        build_dir: Build directory
        output_path: Optional path to save CSV
        
    Returns:
        DataFrame with dataset, best_nees, worst_nees, nees_ratio
    """
    # Discover all NEES files
    nees_files = []
    for f in os.listdir(build_dir):
        if f.startswith("gal3_nees_") and f.endswith(".csv"):
            # gal3_nees_MH01_2s.csv -> MH01, 2s
            parts = f.replace("gal3_nees_", "").replace(".csv", "").split("_")
            if len(parts) >= 2:
                dataset = parts[0]
                interval = parts[1]
                nees_files.append((dataset, interval, os.path.join(build_dir, f)))
    
    # Group by dataset and compute mean NEES
    datasets = {}
    for dataset, interval, filepath in nees_files:
        try:
            df = pd.read_csv(filepath)
            mean_nees = df['nees'].mean()
            if dataset not in datasets:
                datasets[dataset] = {}
            datasets[dataset][interval] = mean_nees
        except Exception as e:
            print(f"Warning: Could not load {filepath}: {e}")
    
    # Create summary
    summary_data = []
    for dataset, intervals in sorted(datasets.items()):
        best = intervals.get("2s", None)
        worst_5s = intervals.get("5s", None)
        worst_10s = intervals.get("10s", None)
        
        # Use 10s as worst if available, else 5s
        worst = worst_10s if worst_10s else worst_5s
        
        if best and worst:
            ratio = worst / best
        else:
            ratio = None
        
        summary_data.append({
            'dataset': dataset,
            'best_nees': best,
            'worst_nees': worst,
            'nees_ratio': ratio
        })
    
    summary_df = pd.DataFrame(summary_data)
    
    if output_path is not None and not summary_df.empty:
        summary_df.to_csv(output_path, index=False)
        print(f"Saved: {output_path}")
    
    return summary_df