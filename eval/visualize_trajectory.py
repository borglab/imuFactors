#!/usr/bin/env python3
"""
Trajectory visualization - Multi-filter support with comprehensive plotting
Visualizes ground truth vs predicted trajectories and frequency analysis
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def load_trajectory(filename):
    """Load trajectory data from CSV"""
    if not os.path.exists(filename):
        print(f"⚠️  {filename} not found")
        return None
    df = pd.read_csv(filename)
    return df

def plot_3d_trajectory_single(filterName, datasetName, preintegrationTime, timeSuffix, figureNumber):
    """
    Plot 3D trajectory comparing ground truth against a single preintegration time.
    Each preintegration time gets its own dedicated plot.
    """
    trajectoryFile = f'{filterName}_trajectory_{datasetName}_{timeSuffix}.csv'
    
    if not os.path.exists(trajectoryFile):
        print(f"⚠️  {trajectoryFile} not found")
        return
    
    dataFrame = load_trajectory(trajectoryFile)
    if dataFrame is None:
        return
    
    figure = plt.figure(figureNumber, figsize=(14, 10))
    axes = figure.add_subplot(111, projection='3d')
    
    # Plot complete ground truth trajectory - all points
    axes.plot(dataFrame['gt_x'], dataFrame['gt_y'], dataFrame['gt_z'], 
            color='#2E4057', linewidth=4, linestyle=':', 
            label='Ground Truth', alpha=0.8, zorder=5)
    
    # Filter predictions to only VALID data (remove NaN/zeros at end)
    predictionMask = (dataFrame['pred_x'].notna() & 
                     (dataFrame['pred_x'] != 0) & 
                     (dataFrame['pred_y'] != 0) & 
                     (dataFrame['pred_z'] != 0))
    validPredictions = dataFrame[predictionMask]
        
    # Plot prediction for this preintegration time - solid line, only valid data
    if len(validPredictions) > 0:
        axes.plot(validPredictions['pred_x'], validPredictions['pred_y'], validPredictions['pred_z'], 
                color='#E63946', linewidth=2.5, linestyle='-',
                label=f'{filterName.upper()} Prediction ({preintegrationTime}s)', alpha=0.85, zorder=10)
    
    # Mark start and end points using GROUND TRUTH (not predictions)
    axes.scatter(dataFrame['gt_x'].iloc[0], dataFrame['gt_y'].iloc[0], dataFrame['gt_z'].iloc[0], 
              c='#06A77D', s=150, marker='o', edgecolors='white', linewidths=2,
              label='Start', zorder=15)
    axes.scatter(dataFrame['gt_x'].iloc[-1], dataFrame['gt_y'].iloc[-1], dataFrame['gt_z'].iloc[-1], 
              c='#1E88E5', s=150, marker='s', edgecolors='white', linewidths=2,
              label='End', zorder=15)
    
    axes.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    axes.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    axes.set_zlabel('Z (m)', fontsize=12, fontweight='bold')
    axes.set_title(f'3D Trajectory: {filterName.upper()} - {datasetName}\n'
                 f'Ground Truth vs {preintegrationTime}s Preintegration', fontsize=14, fontweight='bold')
    axes.legend(fontsize=10, loc='best', framealpha=0.95, edgecolor='gray')
    axes.grid(True, alpha=0.25, linestyle='--')
    axes.set_facecolor('#F5F5F5')
    
    filename = f'trajectory_3d_{filterName}_{datasetName}_{timeSuffix}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_3d_trajectory_combined(filterName, datasetName, figureNumber):
    """
    Plot all three preintegration times on one figure for comparison
    """
    figure = plt.figure(figureNumber, figsize=(18, 14))
    axes = figure.add_subplot(111, projection='3d')
    
    preintegrationConfigs = {
        '0.2s': ('2s', '#E63946', '-'),      # red solid
        '0.5s': ('5s', '#F77F00', '--'),     # orange dashed
        '1.0s': ('10s', '#7209B7', ':')      # purple dotted
    }
    
    # Load and plot ground truth from first available file
    firstTrajectory = None
    for suffix in ['2s', '5s', '10s']:
        trajectoryFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
        if os.path.exists(trajectoryFile):
            firstTrajectory = load_trajectory(trajectoryFile)
            break
    
    if firstTrajectory is None:
        print(f"⚠️  No trajectory files found for {filterName} - {datasetName}")
        return
    
    # Plot complete ground truth trajectory
    axes.plot(firstTrajectory['gt_x'], firstTrajectory['gt_y'], firstTrajectory['gt_z'], 
            color='#2E4057', linewidth=4.5, linestyle=':',
            label='Ground Truth', alpha=0.75, zorder=5)
    
    # Plot predictions for each preintegration time (filtered)
    for preintegrationLabel, (suffix, color, lineStyle) in preintegrationConfigs.items():
        trajectoryFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
        
        if not os.path.exists(trajectoryFile):
            print(f"⚠️  {trajectoryFile} not found, skipping {preintegrationLabel}")
            continue
        
        dataFrame = load_trajectory(trajectoryFile)
        if dataFrame is None:
            continue
        
        # Filter to only VALID predictions
        predictionMask = (dataFrame['pred_x'].notna() & 
                         (dataFrame['pred_x'] != 0) & 
                         (dataFrame['pred_y'] != 0) & 
                         (dataFrame['pred_z'] != 0))
        validPredictions = dataFrame[predictionMask]
        
        if len(validPredictions) > 0:
            axes.plot(validPredictions['pred_x'], validPredictions['pred_y'], validPredictions['pred_z'], 
                    color=color, linestyle=lineStyle, linewidth=3, 
                    label=f'{filterName.upper()} ({preintegrationLabel})', alpha=0.85, zorder=10)
    
    axes.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    axes.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    axes.set_zlabel('Z (m)', fontsize=12, fontweight='bold')
    axes.set_title(f'3D Trajectory Comparison: {filterName.upper()} - {datasetName}\n'
                 f'Ground Truth vs All Preintegration Times', fontsize=14, fontweight='bold')
    axes.legend(fontsize=11, loc='best', framealpha=0.95, edgecolor='gray')
    axes.grid(True, alpha=0.25, linestyle='--')
    axes.set_facecolor('#F5F5F5')
    
    filename = f'trajectory_3d_combined_{filterName}_{datasetName}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_time_series_position(filter_name, dataset_name, fignum=2):
    """
    Plot position (x, y, z) time series at 200 Hz for all preintegration times
    """
    fig, axes = plt.subplots(3, 1, figsize=(18, 12))
    fig.suptitle(f'Position Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}', 
                 fontsize=16, fontweight='bold')
    
    preint_configs = {
        '0.2s': ('2s', '#E63946', '--'),
        '0.5s': ('5s', '#F77F00', ':'),
        '1.0s': ('10s', '#7209B7', '-.')
    }
    
    axes_labels = ['X', 'Y', 'Z']
    gt_cols = ['gt_x', 'gt_y', 'gt_z']
    pred_cols = ['pred_x', 'pred_y', 'pred_z']
    
    # Load ground truth
    first_traj = None
    for suffix in ['2s', '5s', '10s']:
        traj_file = f'{filter_name}_trajectory_{dataset_name}_{suffix}.csv'
        if os.path.exists(traj_file):
            first_traj = load_trajectory(traj_file)
            break
    
    if first_traj is None:
        print(f"⚠️  No trajectory files found")
        return
    
    for i, (ax, label, gt_col) in enumerate(zip(axes, axes_labels, gt_cols)):
        # Plot ground truth - SOLID
        ax.plot(first_traj['timestamp'], first_traj[gt_col], 
                color='#2E4057', linewidth=3, linestyle='-',
                label='Ground Truth', alpha=0.85, zorder=5)
        
        # Plot predictions for each preintegration time - DASHED
        for preint_label, (suffix, color, linestyle) in preint_configs.items():
            traj_file = f'{filter_name}_trajectory_{dataset_name}_{suffix}.csv'
            
            if not os.path.exists(traj_file):
                continue
            
            df = load_trajectory(traj_file)
            if df is None:
                continue
            
            # Filter to only VALID predictions (remove NaN/zeros at end)
            prediction_mask = df[pred_cols[i]].notna() & (df[pred_cols[i]] != 0)
            valid_predictions = df[prediction_mask]
            
            # Only plot if we have valid data
            if len(valid_predictions) > 0:
                ax.plot(valid_predictions['timestamp'], valid_predictions[pred_cols[i]], 
                        color=color, linestyle=linestyle, linewidth=2.5,
                        label=f'{filter_name.upper()} ({preint_label})', alpha=0.85, zorder=10)
        
        ax.set_ylabel(f'{label} Position (m)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best', framealpha=0.95, edgecolor='gray')
        ax.set_facecolor('#F5F5F5')
        
        if i == 2:
            ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    filename = f'timeseries_position_{filter_name}_{dataset_name}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_time_series_velocity(filter_name, dataset_name, fignum=3):
    """
    Plot velocity (vx, vy, vz) time series at 200 Hz for all preintegration times
    """
    fig, axes = plt.subplots(3, 1, figsize=(18, 12))
    fig.suptitle(f' Veocity Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}', 
                 fontsize=16, fontweight='bold')
    
    preint_configs = {
        '0.2s': ('2s', '#E63946', '--'),
        '0.5s': ('5s', '#F77F00', ':'),
        '1.0s': ('10s', '#7209B7', '-.')
    }
    
    axes_labels = ['Vx', 'Vy', 'Vz']
    gt_cols = ['gt_vx', 'gt_vy', 'gt_vz']
    pred_cols = ['pred_vx', 'pred_vy', 'pred_vz']
    
    # Get ground truth
    first_traj = None
    for suffix in ['2s', '5s', '10s']:
        traj_file = f'{filter_name}_trajectory_{dataset_name}_{suffix}.csv'
        if os.path.exists(traj_file):
            first_traj = load_trajectory(traj_file)
            break
    
    if first_traj is None:
        return
    
    for i, (ax, label, gt_col) in enumerate(zip(axes, axes_labels, gt_cols)):
        # Plot ground truth - SOLID
        ax.plot(first_traj['timestamp'], first_traj[gt_col], 
                color='#2E4057', linewidth=3, linestyle='-',
                label='Ground Truth', alpha=0.85, zorder=5)
        
        # Plot predictions - DASHED
        for preint_label, (suffix, color, linestyle) in preint_configs.items():
            traj_file = f'{filter_name}_trajectory_{dataset_name}_{suffix}.csv'
            
            if not os.path.exists(traj_file):
                continue
            
            df = load_trajectory(traj_file)
            if df is None:
                continue
            
            # Filter to only VALID predictions (remove NaN/zeros at end)
            prediction_mask = df[pred_cols[i]].notna() & (df[pred_cols[i]] != 0)
            valid_predictions = df[prediction_mask]
            
            # Only plot if we have valid data
            if len(valid_predictions) > 0:
                ax.plot(valid_predictions['timestamp'], valid_predictions[pred_cols[i]], 
                        color=color, linestyle=linestyle, linewidth=2.5,
                        label=f'{filter_name.upper()} ({preint_label})', alpha=0.85, zorder=10)
        
        ax.set_ylabel(f'{label} (m/s)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best', framealpha=0.95, edgecolor='gray')
        ax.set_facecolor('#F5F5F5')
        
        if i == 2:
            ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    filename = f'timeseries_velocity_{filter_name}_{dataset_name}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_time_series_orientation(filter_name, dataset_name, fignum=4):
    """
    Plot orientation (roll, pitch, yaw) time series at 200 Hz for all preintegration times
    """
    fig, axes = plt.subplots(3, 1, figsize=(18, 12))
    fig.suptitle(f'Orientation Time Series @ 200Hz - {filter_name.upper()} - {dataset_name}', 
                 fontsize=16, fontweight='bold')
    
    preint_configs = {
        '0.2s': ('2s', '#E63946', '--'),
        '0.5s': ('5s', '#F77F00', ':'),
        '1.0s': ('10s', '#7209B7', '-.')
    }
    
    axes_labels = ['Roll', 'Pitch', 'Yaw']
    gt_cols = ['gt_roll', 'gt_pitch', 'gt_yaw']
    pred_cols = ['pred_roll', 'pred_pitch', 'pred_yaw']
    
    # Get ground truth
    first_traj = None
    for suffix in ['2s', '5s', '10s']:
        traj_file = f'{filter_name}_trajectory_{dataset_name}_{suffix}.csv'
        if os.path.exists(traj_file):
            first_traj = load_trajectory(traj_file)
            break
    
    if first_traj is None:
        return
    
    for i, (ax, label, gt_col) in enumerate(zip(axes, axes_labels, gt_cols)):
        # Plot ground truth - SOLID
        ax.plot(first_traj['timestamp'], first_traj[gt_col], 
                color='#2E4057', linewidth=3, linestyle='-',
                label='Ground Truth', alpha=0.85, zorder=5)
        
        # Plot predictions - DASHED
        for preint_label, (suffix, color, linestyle) in preint_configs.items():
            traj_file = f'{filter_name}_trajectory_{dataset_name}_{suffix}.csv'
            
            if not os.path.exists(traj_file):
                continue
            
            df = load_trajectory(traj_file)
            if df is None:
                continue
            
            # Filter to only VALID predictions (remove NaN/zeros at end)
            prediction_mask = df[pred_cols[i]].notna() & (df[pred_cols[i]] != 0)
            valid_predictions = df[prediction_mask]
            
            # Only plot if we have valid data
            if len(valid_predictions) > 0:
                ax.plot(valid_predictions['timestamp'], valid_predictions[pred_cols[i]], 
                        color=color, linestyle=linestyle, linewidth=2.5,
                        label=f'{filter_name.upper()} ({preint_label})', alpha=0.85, zorder=10)
        
        ax.set_ylabel(f'{label} (deg)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best', framealpha=0.95, edgecolor='gray')
        ax.set_facecolor('#F5F5F5')
        
        if i == 2:
            ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    filename = f'timeseries_orientation_{filter_name}_{dataset_name}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_frequency_spectrum(filterName, datasetName, figureNumber=5):
    """
    Plot frequency spectrum showing FULL 0-200 Hz range with mirrored spectrum
    Shows both positive frequencies (0-100 Hz) and their mirror (100-200 Hz)
    """
    figure, axes = plt.subplots(3, 3, figsize=(20, 14))
    figure.suptitle(f'Frequency Spectrum @ 200Hz - {filterName.upper()} - {datasetName}', 
                 fontsize=16, fontweight='bold')
    
    # Load trajectory
    trajectoryFile = None
    for suffix in ['2s', '5s', '10s']:
        testFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
        if os.path.exists(testFile):
            trajectoryFile = testFile
            break
    
    if trajectoryFile is None:
        print(f"⚠️  No trajectory files found")
        return
    
    dataFrame = load_trajectory(trajectoryFile)
    if dataFrame is None:
        return
    
    timestampDelta = dataFrame['timestamp'].iloc[1] - dataFrame['timestamp'].iloc[0]
    samplingRate = 1.0 / timestampDelta
    nyquistFrequency = samplingRate / 2.0
    
    print(f"Sampling rate: {samplingRate:.2f} Hz")
    print(f"Nyquist frequency: {nyquistFrequency:.2f} Hz")
    
    colors = ['#E63946', '#F77F00', '#06A77D']
    
    # Position spectrum (0-200 Hz with mirroring)
    positionColumns = ['gt_x', 'gt_y', 'gt_z']
    for i, column in enumerate(positionColumns):
        ax = axes[0, i]
        signal = dataFrame[column].values
        
        # Use rfft for real signals (0 to Nyquist)
        fftValues = np.fft.rfft(signal)
        fftFrequencies = np.fft.rfftfreq(len(signal), timestampDelta)
        magnitudes = np.abs(fftValues)
        
    
        frequencies_full = np.concatenate([
            fftFrequencies,                                    # 0 to 100 Hz
            samplingRate - fftFrequencies[-2:0:-1]            # 100 to 200 Hz (mirrored)
        ])
        magnitudes_full = np.concatenate([
            magnitudes,                                        # 0 to 100 Hz
            magnitudes[-2:0:-1]                                # Mirror (symmetric)
        ])
        
        # Plot full 0-200 Hz spectrum
        ax.semilogy(frequencies_full, magnitudes_full, color=colors[i], linewidth=2, alpha=0.8)
        
        # Reference lines
        ax.axvline(x=1.0, color='gray', linestyle='--', alpha=0.4, linewidth=1, label='1 Hz')
        ax.axvline(x=10.0, color='purple', linestyle='--', alpha=0.4, linewidth=1, label='10 Hz')
        ax.axvline(x=50.0, color='orange', linestyle='--', alpha=0.5, linewidth=1.5, label='50 Hz')
        ax.axvline(x=nyquistFrequency, color='red', linestyle='-', linewidth=2, 
                   label=f'Nyquist ({nyquistFrequency:.0f} Hz)')
        ax.axvline(x=samplingRate, color='darkred', linestyle='-', linewidth=2.5, 
                   label=f'Sampling ({samplingRate:.0f} Hz)')
        
        # Add shaded region for mirrored frequencies (100-200 Hz)
        ax.axvspan(nyquistFrequency, samplingRate, alpha=0.15, color='blue', 
                   label='Mirrored Spectrum')
        
        ax.set_xlabel('Frequency (Hz)', fontsize=10, fontweight='bold')
        ax.set_ylabel('Magnitude (log scale)', fontsize=10, fontweight='bold')
        ax.set_title(f'Position {["X", "Y", "Z"][i]} Spectrum', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--', which='both')
        ax.set_xlim([0, samplingRate])  # Full 0-200 Hz
        ax.legend(fontsize=7, loc='upper right')
        ax.set_facecolor('#F5F5F5')
    
    # Velocity spectrum (same mirroring logic)
    velocityColumns = ['gt_vx', 'gt_vy', 'gt_vz']
    for i, column in enumerate(velocityColumns):
        ax = axes[1, i]
        signal = dataFrame[column].values
        
        fftValues = np.fft.rfft(signal)
        fftFrequencies = np.fft.rfftfreq(len(signal), timestampDelta)
        magnitudes = np.abs(fftValues)
        
        # Mirror spectrum
        frequencies_full = np.concatenate([
            fftFrequencies,
            samplingRate - fftFrequencies[-2:0:-1]
        ])
        magnitudes_full = np.concatenate([
            magnitudes,
            magnitudes[-2:0:-1]
        ])
        
        ax.semilogy(frequencies_full, magnitudes_full, color=colors[i], linewidth=2, alpha=0.8)
        
        ax.axvline(x=1.0, color='gray', linestyle='--', alpha=0.4, linewidth=1)
        ax.axvline(x=10.0, color='purple', linestyle='--', alpha=0.4, linewidth=1)
        ax.axvline(x=50.0, color='orange', linestyle='--', alpha=0.5, linewidth=1.5)
        ax.axvline(x=nyquistFrequency, color='red', linestyle='-', linewidth=2, 
                   label=f'Nyquist ({nyquistFrequency:.0f} Hz)')
        ax.axvline(x=samplingRate, color='darkred', linestyle='-', linewidth=2.5, 
                   label=f'Sampling ({samplingRate:.0f} Hz)')
        
        ax.axvspan(nyquistFrequency, samplingRate, alpha=0.15, color='blue', 
                   label='Mirrored Spectrum')
        
        ax.set_xlabel('Frequency (Hz)', fontsize=10, fontweight='bold')
        ax.set_ylabel('Magnitude (log scale)', fontsize=10, fontweight='bold')
        ax.set_title(f' Veocity {["X", "Y", "Z"][i]} Spectrum', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--', which='both')
        ax.set_xlim([0, samplingRate])
        ax.legend(fontsize=7, loc='upper right')
        ax.set_facecolor('#F5F5F5')
    
    # Orientation spectrum (same mirroring logic)
    orientationColumns = ['gt_roll', 'gt_pitch', 'gt_yaw']
    for i, column in enumerate(orientationColumns):
        ax = axes[2, i]
        signal = dataFrame[column].values
        
        fftValues = np.fft.rfft(signal)
        fftFrequencies = np.fft.rfftfreq(len(signal), timestampDelta)
        magnitudes = np.abs(fftValues)
        
        # Mirror spectrum
        frequencies_full = np.concatenate([
            fftFrequencies,
            samplingRate - fftFrequencies[-2:0:-1]
        ])
        magnitudes_full = np.concatenate([
            magnitudes,
            magnitudes[-2:0:-1]
        ])
        
        ax.semilogy(frequencies_full, magnitudes_full, color=colors[i], linewidth=2, alpha=0.8)
        
        ax.axvline(x=1.0, color='gray', linestyle='--', alpha=0.4, linewidth=1)
        ax.axvline(x=10.0, color='purple', linestyle='--', alpha=0.4, linewidth=1)
        ax.axvline(x=50.0, color='orange', linestyle='--', alpha=0.5, linewidth=1.5)
        ax.axvline(x=nyquistFrequency, color='red', linestyle='-', linewidth=2, 
                   label=f'Nyquist ({nyquistFrequency:.0f} Hz)')
        ax.axvline(x=samplingRate, color='darkred', linestyle='-', linewidth=2.5, 
                   label=f'Sampling ({samplingRate:.0f} Hz)')
        
        ax.axvspan(nyquistFrequency, samplingRate, alpha=0.15, color='blue', 
                   label='Mirrored Spectrum')
        
        ax.set_xlabel('Frequency (Hz)', fontsize=10, fontweight='bold')
        ax.set_ylabel('Magnitude (log scale)', fontsize=10, fontweight='bold')
        ax.set_title(f'{["Roll", "Pitch", "Yaw"][i]} Spectrum', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--', which='both')
        ax.set_xlim([0, samplingRate])
        ax.legend(fontsize=7, loc='upper right')
        ax.set_facecolor('#F5F5F5')
    
    plt.tight_layout()
    filename = f'frequency_spectrum_{filterName}_{datasetName}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_high_frequency_zoom(filterName, datasetName, figureNumber=6):
    """
    Zoomed view of 40-200 Hz (NO SHADING, just data)
    """
    figure, axes = plt.subplots(3, 3, figsize=(20, 14))
    figure.suptitle(f'High-Frequency Content (40-200 Hz) - {filterName.upper()} - {datasetName}', 
                 fontsize=16, fontweight='bold')
    
    trajectoryFile = None
    for suffix in ['2s', '5s', '10s']:
        testFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
        if os.path.exists(testFile):
            trajectoryFile = testFile
            break
    
    if trajectoryFile is None:
        return
    
    dataFrame = load_trajectory(trajectoryFile)
    if dataFrame is None:
        return
    
    timestampDelta = dataFrame['timestamp'].iloc[1] - dataFrame['timestamp'].iloc[0]
    samplingRate = 1.0 / timestampDelta
    nyquistFrequency = samplingRate / 2.0
    
    colors = ['#E63946', '#F77F00', '#06A77D']
    
    # Position high-frequency zoom (40-200 Hz)
    positionColumns = ['gt_x', 'gt_y', 'gt_z']
    for i, column in enumerate(positionColumns):
        ax = axes[0, i]
        signal = dataFrame[column].values
        
        # Use rfft for real signals (only returns 0-Nyquist)
        fftValues = np.fft.rfft(signal)
        fftFrequencies = np.fft.rfftfreq(len(signal), timestampDelta)
        magnitudes = np.abs(fftValues)
        
        # Filter to high-frequency range (40-200 Hz)
        highFrequencyMask = (fftFrequencies >= 40) & (fftFrequencies <= samplingRate)
        highFrequencies = fftFrequencies[highFrequencyMask]
        highMagnitudes = magnitudes[highFrequencyMask]
        
        ax.plot(highFrequencies, highMagnitudes, color=colors[i], linewidth=2, alpha=0.8)
        
        # Mark key frequencies
        ax.axvline(x=50.0, color='orange', linestyle='--', linewidth=2, label='50 Hz')
        ax.axvline(x=80.0, color='purple', linestyle='--', linewidth=2, label='80 Hz')
        ax.axvline(x=nyquistFrequency, color='red', linestyle='-', linewidth=2, 
                   label=f'Nyquist ({nyquistFrequency:.0f} Hz)')
        ax.axvline(x=samplingRate, color='darkred', linestyle='-', linewidth=3, 
                   label=f'Sampling Rate ({samplingRate:.0f} Hz)')
        
        ax.set_xlabel('Frequency (Hz)', fontsize=10, fontweight='bold')
        ax.set_ylabel('Magnitude', fontsize=10, fontweight='bold')
        ax.set_title(f'Position {["X", "Y", "Z"][i]} (40-200 Hz)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.set_xlim([40, samplingRate])  # Show 40-200 Hz
        ax.legend(fontsize=8)
        ax.set_facecolor('#F5F5F5')
    
    # Velocity high-frequency zoom (40-200 Hz)
    velocityColumns = ['gt_vx', 'gt_vy', 'gt_vz']
    for i, column in enumerate(velocityColumns):
        ax = axes[1, i]
        signal = dataFrame[column].values
        
        # Use rfft for real signals (only returns 0-Nyquist)
        fftValues = np.fft.rfft(signal)
        fftFrequencies = np.fft.rfftfreq(len(signal), timestampDelta)
        magnitudes = np.abs(fftValues)
        
        highFrequencyMask = (fftFrequencies >= 40) & (fftFrequencies <= samplingRate)
        highFrequencies = fftFrequencies[highFrequencyMask]
        highMagnitudes = magnitudes[highFrequencyMask]
        
        ax.plot(highFrequencies, highMagnitudes, color=colors[i], linewidth=2, alpha=0.8)
        
        ax.axvline(x=50.0, color='orange', linestyle='--', linewidth=2, label='50 Hz')
        ax.axvline(x=80.0, color='purple', linestyle='--', linewidth=2, label='80 Hz')
        ax.axvline(x=nyquistFrequency, color='red', linestyle='-', linewidth=2, 
                   label=f'Nyquist ({nyquistFrequency:.0f} Hz)')
        ax.axvline(x=samplingRate, color='darkred', linestyle='-', linewidth=3, 
                   label=f'Sampling Rate ({samplingRate:.0f} Hz)')
        
        ax.set_xlabel('Frequency (Hz)', fontsize=10, fontweight='bold')
        ax.set_ylabel('Magnitude', fontsize=10, fontweight='bold')
        ax.set_title(f' Veocity {["X", "Y", "Z"][i]} (40-200 Hz)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=8)
        ax.set_facecolor('#F5F5F5')
    
    # Orientation high-frequency zoom (40-200 Hz)
    orientationColumns = ['gt_roll', 'gt_pitch', 'gt_yaw']
    for i, column in enumerate(orientationColumns):
        ax = axes[2, i]
        signal = dataFrame[column].values
        
        # Use rfft for real signals (only returns 0-Nyquist)
        fftValues = np.fft.rfft(signal)
        fftFrequencies = np.fft.rfftfreq(len(signal), timestampDelta)
        magnitudes = np.abs(fftValues)
        
        highFrequencyMask = (fftFrequencies >= 40) & (fftFrequencies <= samplingRate)
        highFrequencies = fftFrequencies[highFrequencyMask]
        highMagnitudes = magnitudes[highFrequencyMask]
        
        ax.plot(highFrequencies, highMagnitudes, color=colors[i], linewidth=2, alpha=0.8)
        
        ax.axvline(x=50.0, color='orange', linestyle='--', linewidth=2, label='50 Hz')
        ax.axvline(x=80.0, color='purple', linestyle='--', linewidth=2, label='80 Hz')
        ax.axvline(x=nyquistFrequency, color='red', linestyle='-', linewidth=2, 
                   label=f'Nyquist ({nyquistFrequency:.0f} Hz)')
        ax.axvline(x=samplingRate, color='darkred', linestyle='-', linewidth=3, 
                   label=f'Sampling Rate ({samplingRate:.0f} Hz)')
        
        ax.set_xlabel('Frequency (Hz)', fontsize=10, fontweight='bold')
        ax.set_ylabel('Magnitude', fontsize=10, fontweight='bold')
        ax.set_title(f'{["Roll", "Pitch", "Yaw"][i]} (40-200 Hz)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=8)
        ax.set_facecolor('#F5F5F5')
    
    plt.tight_layout()
    filename = f'frequency_spectrum_highfreq_{filterName}_{datasetName}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_time_series_acceleration(filterName, datasetName, figureNumber=7):
    """
    Plot accelerometer measurements (ax, ay, az) time series at 200 Hz
    Shows ground truth acceleration vs time with distinct colors and spacing
    """
    figure, axes = plt.subplots(3, 1, figsize=(18, 12))
    figure.suptitle(f'Acceleration Time Series @ 200Hz - {filterName.upper()} - {datasetName}', 
                 fontsize=16, fontweight='bold')
    
    preintegrationConfigs = {
        '0.2s': ('2s', '#E63946', '--'),     # Red
        '0.5s': ('5s', '#F77F00', '-.'),     # Orange  
        '1.0s': ('10s', '#7209B7', ':')      # Purple
    }
    
    axesLabels = ['Ax', 'Ay', 'Az']
    gtColumns = ['gt_ax', 'gt_ay', 'gt_az']
    predColumns = ['pred_ax', 'pred_ay', 'pred_az']
    
    # Load ground truth from first available file
    firstTrajectory = None
    for suffix in ['2s', '5s', '10s']:
        trajectoryFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
        if os.path.exists(trajectoryFile):
            firstTrajectory = load_trajectory(trajectoryFile)
            break
    
    if firstTrajectory is None:
        print(f"⚠️  No trajectory files found for acceleration plot")
        return
    
    for i, (ax, label, gtColumn) in enumerate(zip(axes, axesLabels, gtColumns)):
        # Check if acceleration columns exist
        if gtColumn not in firstTrajectory.columns:
            ax.text(0.5, 0.5, f'No {label} data available', 
                   ha='center', va='center', fontsize=14, color='red')
            ax.set_ylabel(f'{label} (m/s²)', fontsize=12, fontweight='bold')
            continue
        
        # Plot ground truth - THICK SOLID BLACK
        ax.plot(firstTrajectory['timestamp'], firstTrajectory[gtColumn], 
                color='#000000', linewidth=4, linestyle='-',
                label='Ground Truth', alpha=1, zorder=1)
        
        # Plot predictions with vertical offsets and distinct styling
        offsetValues = [0.0, 0.3, -0.3]  # Small vertical offsets for visual separation
        
        for idx, (preintegrationLabel, (suffix, color, lineStyle)) in enumerate(preintegrationConfigs.items()):
            trajectoryFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
            
            if not os.path.exists(trajectoryFile):
                continue
            
            dataFrame = load_trajectory(trajectoryFile)
            if dataFrame is None or predColumns[i] not in dataFrame.columns:
                continue
            
            # Filter to only VALID predictions
            predictionMask = dataFrame[predColumns[i]].notna() & (dataFrame[predColumns[i]] != 0)
            validPredictions = dataFrame[predictionMask]
            
            # Only plot if we have valid data
            if len(validPredictions) > 0:
                # Apply small vertical offset for visual separation
                accelerationData = validPredictions[predColumns[i]] + offsetValues[idx]
                
                ax.plot(validPredictions['timestamp'], accelerationData, 
                        color=color, linestyle=lineStyle, linewidth=3.5,
                        label=f'{filterName.upper()} ({preintegrationLabel})', 
                        alpha=0.4, zorder=10 + idx)
        
        ax.set_ylabel(f'{label} (m/s²)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=10, loc='best', framealpha=0.95, edgecolor='gray')
        ax.set_facecolor('#F8F8F8')
        
        # Add horizontal reference line at zero
        ax.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.4)
        
        if i == 2:
            ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    filename = f'timeseries_acceleration_{filterName}_{datasetName}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_time_series_displacement(filterName, datasetName, figureNumber=8):
    """
    Plot displacement/delta position (dx, dy, dz) time series at 200 Hz
    Shows incremental position changes between consecutive predictions
    """
    figure, axes = plt.subplots(3, 1, figsize=(18, 12))
    figure.suptitle(f'Displacement (Δ Position) Time Series @ 200Hz - {filterName.upper()} - {datasetName}', 
                 fontsize=16, fontweight='bold')
    
    preintegrationConfigs = {
        '0.2s': ('2s', '#E63946', '--'),
        '0.5s': ('5s', '#F77F00', ':'),
        '1.0s': ('10s', '#7209B7', '-.')
    }
    
    axesLabels = ['ΔX', 'ΔY', 'ΔZ']
    gtPositionColumns = ['gt_x', 'gt_y', 'gt_z']
    predPositionColumns = ['pred_x', 'pred_y', 'pred_z']
    
    # Load ground truth from first available file
    firstTrajectory = None
    for suffix in ['2s', '5s', '10s']:
        trajectoryFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
        if os.path.exists(trajectoryFile):
            firstTrajectory = load_trajectory(trajectoryFile)
            break
    
    if firstTrajectory is None:
        print(f"⚠️  No trajectory files found for displacement plot")
        return
    
    for i, (ax, label, gtColumn) in enumerate(zip(axes, axesLabels, gtPositionColumns)):
        # Compute ground truth displacement (difference between consecutive positions)
        gtDisplacement = firstTrajectory[gtColumn].diff()
        
        # Plot ground truth displacement - SOLID
        ax.plot(firstTrajectory['timestamp'], gtDisplacement, 
                color="#070707", linewidth=3, linestyle='-',
                label='Ground Truth Δ', alpha=1, zorder=1)
        
        # Plot predictions for each preintegration time - DASHED
        for preintegrationLabel, (suffix, color, lineStyle) in preintegrationConfigs.items():
            trajectoryFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
            
            if not os.path.exists(trajectoryFile):
                continue
            
            dataFrame = load_trajectory(trajectoryFile)
            if dataFrame is None:
                continue
            
            # Filter to only VALID predictions
            predictionMask = dataFrame[predPositionColumns[i]].notna() & (dataFrame[predPositionColumns[i]] != 0)
            validPredictions = dataFrame[predictionMask]
            
            # Compute predicted displacement
            if len(validPredictions) > 0:
                predDisplacement = validPredictions[predPositionColumns[i]].diff()
                
                ax.plot(validPredictions['timestamp'], predDisplacement, 
                        color=color, linestyle=lineStyle, linewidth=2.5,
                        label=f'{filterName.upper()} ({preintegrationLabel})', alpha=0.5, zorder=10)
        
        ax.set_ylabel(f'{label} (m)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best', framealpha=0.95, edgecolor='gray')
        ax.set_facecolor('#F5F5F5')
        ax.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)  # Zero reference
        
        if i == 2:
            ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    filename = f'timeseries_displacement_{filterName}_{datasetName}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_time_series_integrated_displacement(filterName, datasetName, figureNumber=9):
    """
    Plot cumulative displacement from t=0 (integrated velocity)
    Shows total distance traveled in each axis
    """
    figure, axes = plt.subplots(3, 1, figsize=(18, 12))
    figure.suptitle(f'Cumulative Displacement from t=0 @ 200Hz - {filterName.upper()} - {datasetName}', 
                 fontsize=16, fontweight='bold')
    
    preintegrationConfigs = {
        '0.2s': ('2s', '#E63946', '--'),
        '0.5s': ('5s', '#F77F00', ':'),
        '1.0s': ('10s', '#7209B7', '-.')
    }
    
    axesLabels = ['X', 'Y', 'Z']
    gtPositionColumns = ['gt_x', 'gt_y', 'gt_z']
    predPositionColumns = ['pred_x', 'pred_y', 'pred_z']
    
    # Load ground truth from first available file
    firstTrajectory = None
    for suffix in ['2s', '5s', '10s']:
        trajectoryFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
        if os.path.exists(trajectoryFile):
            firstTrajectory = load_trajectory(trajectoryFile)
            break
    
    if firstTrajectory is None:
        print(f"⚠️  No trajectory files found for integrated displacement plot")
        return
    
    for i, (ax, label, gtColumn) in enumerate(zip(axes, axesLabels, gtPositionColumns)):
        # Compute cumulative displacement from initial position
        initialPosition = firstTrajectory[gtColumn].iloc[0]
        gtCumulativeDisplacement = firstTrajectory[gtColumn] - initialPosition
        
        # Plot ground truth cumulative displacement - SOLID
        ax.plot(firstTrajectory['timestamp'], gtCumulativeDisplacement, 
                color='#2E4057', linewidth=3, linestyle='-',
                label='Ground Truth', alpha=0.85, zorder=5)
        
        # Plot predictions for each preintegration time - DASHED
        for preintegrationLabel, (suffix, color, lineStyle) in preintegrationConfigs.items():
            trajectoryFile = f'{filterName}_trajectory_{datasetName}_{suffix}.csv'
            
            if not os.path.exists(trajectoryFile):
                continue
            
            dataFrame = load_trajectory(trajectoryFile)
            if dataFrame is None:
                continue
            
            # Filter to only VALID predictions
            predictionMask = dataFrame[predPositionColumns[i]].notna() & (dataFrame[predPositionColumns[i]] != 0)
            validPredictions = dataFrame[predictionMask]
            
            if len(validPredictions) > 0:
                # Compute cumulative displacement from initial prediction
                predInitialPosition = validPredictions[predPositionColumns[i]].iloc[0]
                predCumulativeDisplacement = validPredictions[predPositionColumns[i]] - predInitialPosition
                
                ax.plot(validPredictions['timestamp'], predCumulativeDisplacement, 
                        color=color, linestyle=lineStyle, linewidth=2.5,
                        label=f'{filterName.upper()} ({preintegrationLabel})', alpha=0.85, zorder=10)
        
        ax.set_ylabel(f'{label} Cumulative (m)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.25, linestyle='--')
        ax.legend(fontsize=9, loc='best', framealpha=0.95, edgecolor='gray')
        ax.set_facecolor('#F5F5F5')
        ax.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
        
        if i == 2:
            ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    filename = f'timeseries_cumulative_displacement_{filterName}_{datasetName}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def process_filter_dataset(filterName, datasetName):
    """Process a single filter-dataset combination for all preintegration times"""
    print(f"\n{'='*80}")
    print(f"  Processing {filterName} - {datasetName}")
    print('='*80)
    
    # Generate figure numbers
    figureNumberBase = (100 if filterName == "gal3" else 200) + \
                      (0 if datasetName == "MH01" else 50)
    
    # Plot separate 3D trajectories for each preintegration time
    preintegrationConfigs = [
        ('0.2', '2s'),
        ('0.5', '5s'),
        ('1.0', '10s')
    ]
    
    for idx, (preint_time, time_suffix) in enumerate(preintegrationConfigs):
        plot_3d_trajectory_single(filterName, datasetName, preint_time, 
                                  time_suffix, figureNumberBase + idx + 1)
    
    # Also create combined plot showing all three
    plot_3d_trajectory_combined(filterName, datasetName, figureNumberBase + 10)
    
    # Time series plots
    plot_time_series_position(filterName, datasetName, figureNumberBase + 20)
    plot_time_series_velocity(filterName, datasetName, figureNumberBase + 30)
    plot_time_series_orientation(filterName, datasetName, figureNumberBase + 40)
    
    # Frequency spectrum analysis
    plot_frequency_spectrum(filterName, datasetName, figureNumberBase + 50)
    plot_high_frequency_zoom(filterName, datasetName, figureNumberBase + 60)
    
    # NEW: Acceleration and displacement plots
    plot_time_series_acceleration(filterName, datasetName, figureNumberBase + 70)
    plot_time_series_displacement(filterName, datasetName, figureNumberBase + 80)
    plot_time_series_integrated_displacement(filterName, datasetName, figureNumberBase + 90)

if __name__ == '__main__':
    # Enable interactive mode for displaying plots
    plt.ion()
    
    # Process all filter-dataset combinations
    for filterName in ["gal3", "navstate"]:
        for datasetName in ["MH01", "V202"]:
            process_filter_dataset(filterName, datasetName)
    
    print(f"\n{'='*80}")
    print("✓ All visualizations complete!")
    print('='*80)
    
    # Keep plots open
    plt.show(block=True)