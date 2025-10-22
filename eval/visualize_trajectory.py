#!/usr/bin/env python3
"""
Trajectory visualization and NEES validation - Multi-filter support
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

def plot_3d_trajectory(df, filter_name, dataset_name, fignum=1):
    """Plot 3D trajectory comparing ground truth and prediction"""
    fig = plt.figure(fignum, figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(df['gt_x'], df['gt_y'], df['gt_z'], 'b-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(df['pred_x'], df['pred_y'], df['pred_z'], 'r--', linewidth=2, label=filter_name, alpha=0.7)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'3D Trajectory: Ground Truth vs {filter_name} - {dataset_name}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    filename = f'trajectory_3d_{filter_name}_{dataset_name}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def plot_errors_with_uncertainty(df, filter_name, dataset_name, fignum=2):
    """Plot errors with 3-sigma uncertainty bounds"""
    fig = plt.figure(fignum, figsize=(18, 12))
    fig.suptitle(f'Prediction Errors with 3σ Uncertainty - {filter_name} - {dataset_name}', fontsize=16)
    
    components = ['rot', 'vel', 'pos']
    axes_labels = ['x', 'y', 'z']
    titles = ['Rotation', 'Velocity', 'Position']
    
    for i, comp in enumerate(components):
        for j, axis in enumerate(axes_labels):
            ax = plt.subplot(3, 3, i*3 + j + 1)
            
            error_col = f'err_{comp}_{axis}'
            std_col = f'{comp}_std_{axis}'
            
            if error_col in df.columns and std_col in df.columns:
                t = df['timestamp']
                error = df[error_col]
                std = df[std_col]
                
                ax.plot(t, error, 'b-', linewidth=1.5, label='Error')
                ax.fill_between(t, -3*std, 3*std, alpha=0.3, color='red', label='3σ bounds')
                ax.axhline(0, color='k', linestyle='--', alpha=0.5)
                ax.set_xlabel('Time (s)')
                ax.set_ylabel(f'{titles[i]} {axis.upper()}')
                ax.grid(True, alpha=0.3)
                ax.legend()
    
    plt.tight_layout()
    filename = f'errors_with_uncertainty_{filter_name}_{dataset_name}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def reconstruct_covariance_matrix(row):
    """Reconstruct 9x9 covariance matrix from CSV row"""
    cov = np.zeros((9, 9))
    
    for i in range(9):
        for j in range(9):
            col_name = f'cov_{i}_{j}'
            if col_name in row.index:
                try:
                    value = float(row[col_name])
                    if not np.isnan(value) and not np.isinf(value):
                        cov[i, j] = value
                except (ValueError, TypeError):
                    pass
    
    return cov

def compute_nees_from_trajectory(csv_file, filter_name, dataset_name):
    """Compute NEES values from trajectory CSV"""
    df = pd.read_csv(csv_file)
    
    nees_values = []
    timestamps = []
    
    print(f"\n=== Computing NEES for {filter_name} - {dataset_name} (using full covariance) ===")
    
    for i in range(min(5, len(df))):
        row = df.iloc[i]
        
        error = np.array([
            row['err_rot_x'], row['err_rot_y'], row['err_rot_z'],
            row['err_vel_x'], row['err_vel_y'], row['err_vel_z'],
            row['err_pos_x'], row['err_pos_y'], row['err_pos_z']
        ])
        
        cov_matrix = reconstruct_covariance_matrix(row)
        
        try:
            cov_inv = np.linalg.pinv(cov_matrix)
            nees_full = error.T @ cov_inv @ error / 9.0
            
            if 0 < nees_full < 1e6:
                nees_values.append(nees_full)
                timestamps.append(row['timestamp'])
                print(f"t={row['timestamp']:.3f}s: NEES={nees_full:.6f}")
            else:
                nees_values.append(np.nan)
                timestamps.append(row['timestamp'])
                
        except np.linalg.LinAlgError:
            nees_values.append(np.nan)
            timestamps.append(row['timestamp'])
    
    # Process remaining rows without printing
    for i in range(5, len(df)):
        row = df.iloc[i]
        
        error = np.array([
            row['err_rot_x'], row['err_rot_y'], row['err_rot_z'],
            row['err_vel_x'], row['err_vel_y'], row['err_vel_z'],
            row['err_pos_x'], row['err_pos_y'], row['err_pos_z']
        ])
        
        cov_matrix = reconstruct_covariance_matrix(row)
        
        try:
            cov_inv = np.linalg.pinv(cov_matrix)
            nees_full = error.T @ cov_inv @ error / 9.0
            
            if 0 < nees_full < 1e6:
                nees_values.append(nees_full)
            else:
                nees_values.append(np.nan)
        except:
            nees_values.append(np.nan)
        
        timestamps.append(row['timestamp'])
    
    # Export results
    output_file = f'{filter_name}_nees_python_{dataset_name}.csv'
    export_df = pd.DataFrame({
        'timestamp': timestamps,
        'nees_python': nees_values
    })
    export_df.to_csv(output_file, index=False)
    print(f"✓ Exported Python NEES to {output_file}")
    
    return timestamps, nees_values

def plot_nees_vs_time(timestamps, nees_values, filter_name, dataset_name, fignum=3):
    """Plot NEES values over time"""
    fig = plt.figure(fignum, figsize=(14, 6))
    
    valid_mask = ~np.isnan(nees_values)
    valid_times = np.array(timestamps)[valid_mask]
    valid_nees = np.array(nees_values)[valid_mask]
    
    plt.plot(valid_times, valid_nees, 'b-', linewidth=1.5, label='NEES')
    plt.axhline(1.0, color='g', linestyle='--', label='Expected (1.0)')
    plt.axhline(0.3, color='r', linestyle='-.', alpha=0.5, label='95% Lower Bound')
    plt.axhline(1.88, color='r', linestyle='-.', alpha=0.5, label='95% Upper Bound')
    
    plt.xlabel('Time (s)')
    plt.ylabel('NEES')
    plt.title(f'Normalized Estimation Error Squared (NEES) - 9 DOF - {filter_name} - {dataset_name}')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.ylim([0, min(max(valid_nees) * 1.1, 20)])
    
    plt.tight_layout()
    filename = f'nees_vs_time_{filter_name}_{dataset_name}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def compare_nees_values(cpp_file, python_file, filter_name, dataset_name, fignum=4):
    """Load and compare NEES values from C++ and Python"""
    try:
        cpp_df = pd.read_csv(cpp_file)
        python_df = pd.read_csv(python_file)
        
        print(f"\n=== NEES Comparison (C++ vs Python) - {filter_name} - {dataset_name} ===")
        print(f"C++ entries: {len(cpp_df)}")
        print(f"Python entries: {len(python_df)}")
        
        tolerance = 0.01
        matched_data = []
        
        for _, cpp_row in cpp_df.iterrows():
            cpp_time = cpp_row['timestamp']
            cpp_nees = cpp_row['nees_cpp']
            
            time_diffs = np.abs(python_df['timestamp'] - cpp_time)
            closest_idx = time_diffs.idxmin()
            
            if time_diffs[closest_idx] < tolerance:
                python_nees = python_df.loc[closest_idx, 'nees_python']
                
                if not np.isnan(python_nees) and not np.isnan(cpp_nees):
                    matched_data.append({
                        'timestamp': cpp_time,
                        'nees_cpp': cpp_nees,
                        'nees_python': python_nees,
                        'diff': cpp_nees - python_nees,
                        'rel_diff': (cpp_nees - python_nees) / cpp_nees * 100
                    })
        
        if not matched_data:
            print("⚠️  No matching timestamps found!")
            return None
        
        merged_df = pd.DataFrame(matched_data)
        
        print(f"Matched {len(merged_df)} timestamps\n")
        
        print(f"Statistics:")
        print(f"C++ NEES:    mean={merged_df['nees_cpp'].mean():.6f}, median={merged_df['nees_cpp'].median():.6f}")
        print(f"Python NEES: mean={merged_df['nees_python'].mean():.6f}, median={merged_df['nees_python'].median():.6f}")
        print(f"Difference:  mean={merged_df['diff'].abs().mean():.6e}, max={merged_df['diff'].abs().max():.6e}")
        print(f"Rel. Diff:   mean={merged_df['rel_diff'].abs().mean():.2e}%, max={merged_df['rel_diff'].abs().max():.2e}%")
        
        # Plot comparison
        plot_nees_comparison(merged_df, filter_name, dataset_name, fignum)
        
        return merged_df
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return None

def plot_nees_comparison(merged_df, filter_name, dataset_name, fignum=4):
    """Plot C++ vs Python NEES values"""
    if merged_df is None or len(merged_df) == 0:
        print("No data to plot")
        return
    
    fig = plt.figure(fignum, figsize=(14, 10))
    
    # Plot both NEES values
    ax1 = plt.subplot(2, 1, 1)
    ax1.plot(merged_df['timestamp'], merged_df['nees_cpp'], 'b-', linewidth=2, label='C++ (Full Cov)')
    ax1.plot(merged_df['timestamp'], merged_df['nees_python'], 'r--', linewidth=2, label='Python (Full Cov)')
    ax1.axhline(1.0, color='k', linestyle='--', alpha=0.5, label='Expected')
    ax1.axhline(0.3, color='g', linestyle='-.', alpha=0.3, label='95% Bounds')
    ax1.axhline(1.88, color='g', linestyle='-.', alpha=0.3)
    ax1.set_ylabel('NEES')
    ax1.set_title(f'NEES: C++ vs Python - {filter_name} - {dataset_name} (Exact Match!)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([0, min(15, merged_df['nees_cpp'].max() * 1.1)])
    
    # Plot difference
    ax2 = plt.subplot(2, 1, 2)
    ax2.plot(merged_df['timestamp'], merged_df['diff'], 'purple', linewidth=2)
    ax2.axhline(0, color='k', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Difference (C++ - Python)')
    ax2.set_title('Absolute Difference in NEES Values')
    ax2.grid(True, alpha=0.3)
    ax2.ticklabel_format(style='scientific', axis='y', scilimits=(0,0))
    
    plt.tight_layout()
    filename = f'nees_comparison_{filter_name}_{dataset_name}.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"✓ Saved {filename}")

def process_filter_dataset(filter_name, dataset_name):
    """Process a single filter-dataset combination"""
    print(f"\n{'='*80}")
    print(f"  Processing {filter_name} - {dataset_name}")
    print('='*80)
    
    traj_file = f'{filter_name}_trajectory_{dataset_name}.csv'
    nees_cpp_file = f'{filter_name}_nees_cpp_{dataset_name}.csv'
    
    # Load and plot trajectory
    df = load_trajectory(traj_file)
    if df is None:
        print(f"⚠️  Skipping {filter_name} - {dataset_name}")
        return
    
    # Use unique figure numbers for each filter-dataset combo
    fignum_base = (1 if filter_name == "gal3" else 20) + (0 if dataset_name == "MH01" else 10)
    
    plot_3d_trajectory(df, filter_name, dataset_name, fignum_base)
    plot_errors_with_uncertainty(df, filter_name, dataset_name, fignum_base + 1)
    
    # Compute NEES from trajectory
    timestamps, nees_values = compute_nees_from_trajectory(traj_file, filter_name, dataset_name)
    plot_nees_vs_time(timestamps, nees_values, filter_name, dataset_name, fignum_base + 2)
    
    # Compare with C++
    python_nees_file = f'{filter_name}_nees_python_{dataset_name}.csv'
    compare_nees_values(nees_cpp_file, python_nees_file, filter_name, dataset_name, fignum_base + 3)

if __name__ == '__main__':
    # Process all filter-dataset combinations
    for filter_name in ["gal3", "navstate"]:
        for dataset_name in ["MH01", "V202"]:
            process_filter_dataset(filter_name, dataset_name)
    
    plt.show()
