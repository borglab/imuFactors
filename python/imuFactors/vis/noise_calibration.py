#!/usr/bin/env python3
"""
Noise calibration visualization.
Creates bar charts comparing best/worst NEES values and alpha parameters.
"""

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import matplotlib.pyplot as plt
from typing import Optional, List
from .trajectory_loader import load_nees_summary, discover_datasets


def plot_nees_comparison(
    nees_summary: pd.DataFrame,
    title: str = "NEES Comparison Across Datasets",
    save_path: Optional[str] = None
) -> go.Figure:
    """
    Create interactive Plotly bar chart comparing NEES values.
    
    Args:
        nees_summary: DataFrame with NEES summary data
        title: Plot title
        save_path: Optional path to save HTML
        
    Returns:
        Plotly Figure
    """
    fig = go.Figure()
    
    # Best NEES bars
    fig.add_trace(go.Bar(
        x=nees_summary['dataset'],
        y=nees_summary['best_nees'],
        name='Best (αG=13.0, αA=9.4)',
        marker_color='green',
        hovertemplate=(
            '<b>%{x}</b><br>'
            'NEES: %{y:.4f}<br>'
            '<extra></extra>'
        )
    ))
    
    # Worst NEES bars
    fig.add_trace(go.Bar(
        x=nees_summary['dataset'],
        y=nees_summary['worst_nees'],
        name='Worst (αG=0.5, αA=0.5)',
        marker_color='red',
        hovertemplate=(
            '<b>%{x}</b><br>'
            'NEES: %{y:.4f}<br>'
            '<extra></extra>'
        )
    ))
    
    fig.update_layout(
        title=dict(text=title, x=0.5, font=dict(size=18)),
        xaxis_title='Dataset',
        yaxis_title='NEES Value',
        yaxis_type='log',
        barmode='group',
        height=500,
        hovermode='x unified',
        template='plotly_white'
    )
    
    if save_path:
        fig.write_html(save_path)
        print(f"Saved: {save_path}")
    
    return fig


def plot_nees_comparison_matplotlib(
    nees_summary: pd.DataFrame,
    title: str = "NEES Comparison Across Datasets",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Create static matplotlib bar chart comparing NEES values.
    
    Args:
        nees_summary: DataFrame with NEES summary data
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig, ax = plt.subplots(figsize=(12, 6))
    
    x = np.arange(len(nees_summary['dataset']))
    width = 0.35
    
    ax.bar(
        x - width/2, nees_summary['best_nees'],
        width, label='Best (αG=13.0, αA=9.4)',
        color='green', alpha=0.8
    )
    ax.bar(
        x + width/2, nees_summary['worst_nees'],
        width, label='Worst (αG=0.5, αA=0.5)',
        color='red', alpha=0.8
    )
    
    ax.set_xlabel('Dataset', fontsize=12, fontweight='bold')
    ax.set_ylabel('NEES Value', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(nees_summary['dataset'])
    ax.set_yscale('log')
    ax.legend()
    ax.grid(True, alpha=0.25, axis='y')
    ax.set_facecolor('#F5F5F5')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_alpha_parameters(
    nees_summary: pd.DataFrame,
    title: str = "Noise Calibration Parameters",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Create grouped bar chart showing alpha gyro and alpha acc values.
    
    Args:
        nees_summary: DataFrame with NEES summary data
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    datasets = nees_summary['dataset']
    x = np.arange(len(datasets))
    width = 0.35
    
    # Alpha Gyro plot
    ax = axes[0]
    ax.bar(
        x - width/2, nees_summary.get('best_alpha_gyro', [13.0] * len(datasets)),
        width, label='Best αG', color='green', alpha=0.8
    )
    ax.bar(
        x + width/2, nees_summary.get('worst_alpha_gyro', [0.5] * len(datasets)),
        width, label='Worst αG', color='red', alpha=0.8
    )
    ax.set_xlabel('Dataset', fontsize=12)
    ax.set_ylabel('Alpha Gyro', fontsize=12)
    ax.set_title('Alpha Gyro', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(datasets, rotation=45)
    ax.legend()
    ax.grid(True, alpha=0.25, axis='y')
    
    # Alpha Acc plot
    ax = axes[1]
    ax.bar(
        x - width/2, nees_summary.get('best_alpha_acc', [9.4] * len(datasets)),
        width, label='Best αA', color='green', alpha=0.8
    )
    ax.bar(
        x + width/2, nees_summary.get('worst_alpha_acc', [0.5] * len(datasets)),
        width, label='Worst αA', color='red', alpha=0.8
    )
    ax.set_xlabel('Dataset', fontsize=12)
    ax.set_ylabel('Alpha Acc', fontsize=12)
    ax.set_title('Alpha Accelerometer', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(datasets, rotation=45)
    ax.legend()
    ax.grid(True, alpha=0.25, axis='y')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def plot_nees_ratio(
    nees_summary: pd.DataFrame,
    title: str = "NEES Ratio (Worst/Best)",
    save_path: Optional[str] = None
) -> plt.Figure:
    """
    Create bar chart showing NEES ratio (worst/best) for each dataset.
    
    Args:
        nees_summary: DataFrame with NEES summary data
        title: Plot title
        save_path: Optional path to save figure
        
    Returns:
        Matplotlib Figure
    """
    fig, ax = plt.subplots(figsize=(12, 6))
    
    ratios = nees_summary['worst_nees'] / nees_summary['best_nees']
    
    bars = ax.bar(
        nees_summary['dataset'], ratios,
        color=plt.cm.RdYlGn_r(np.linspace(0.2, 0.8, len(ratios))),
        edgecolor='black', linewidth=1
    )
    
    # Add value labels on bars
    for bar, ratio in zip(bars, ratios):
        height = bar.get_height()
        ax.annotate(
            f'{ratio:.1f}x',
            xy=(bar.get_x() + bar.get_width() / 2, height),
            xytext=(0, 3),
            textcoords="offset points",
            ha='center', va='bottom',
            fontsize=10, fontweight='bold'
        )
    
    ax.set_xlabel('Dataset', fontsize=12, fontweight='bold')
    ax.set_ylabel('NEES Ratio (Worst / Best)', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.25, axis='y')
    ax.set_facecolor('#F5F5F5')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {save_path}")
    
    return fig


def generate_noise_calibration_report(
    nees_summary: pd.DataFrame,
    output_dir: str = ".",
    formats: List[str] = ['html', 'png']
) -> None:
    """
    Generate complete noise calibration visualization report.
    
    Args:
        nees_summary: DataFrame with NEES summary data
        output_dir: Directory to save outputs
        formats: List of output formats ('html', 'png')
    """
    print("Generating noise calibration report...")
    
    # NEES comparison
    if 'html' in formats:
        plot_nees_comparison(
            nees_summary,
            save_path=f"{output_dir}/nees_comparison.html"
        )
    
    if 'png' in formats:
        plot_nees_comparison_matplotlib(
            nees_summary,
            save_path=f"{output_dir}/nees_comparison.png"
        )
        plot_alpha_parameters(
            nees_summary,
            save_path=f"{output_dir}/alpha_parameters.png"
        )
        plot_nees_ratio(
            nees_summary,
            save_path=f"{output_dir}/nees_ratio.png"
        )
    
    print("Report generation complete!")


# Convenience function for quick loading and plotting
def quick_nees_plot(
    csv_path: str = "nees_summary.csv",
    output_dir: str = "."
) -> None:
    """
    Quick function to load NEES summary and generate plots.
    
    Args:
        csv_path: Path to NEES summary CSV
        output_dir: Output directory for plots
    """
    nees_summary = load_nees_summary(csv_path)
    
    if nees_summary is None:
        print(f"Error: Could not load {csv_path}")
        return
    
    generate_noise_calibration_report(nees_summary, output_dir)