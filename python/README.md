# imuFactors Python Visualization Package

Visualization utilities for IMU factor evaluation, supporting both Plotly (interactive) and Matplotlib (static) plots.

## Installation

```bash
# From source
cd python
pip install -e .

# Or install dependencies directly
pip install numpy pandas plotly matplotlib
```

## Quick Start

```python
from imuFactors.vis import (
    load_trajectory,
    plot_3d_trajectory,
    plot_position_timeseries,
    plot_nees_comparison,
)

# Load trajectory data
traj = load_trajectory("gal3_trajectory_MH01_2s.csv")

# Create interactive 3D plot (opens in browser)
fig = plot_3d_trajectory(traj)
fig.show()

# Create static time series plot
plot_position_timeseries(traj, save_path="position_timeseries.png")

# Load and plot NEES comparison
nees_summary = load_nees_summary("nees_summary.csv")
plot_nees_comparison(nees_summary, save_path="nees_comparison.html")
```

## Modules

### trajectory_loader
Loads CSV data from C++ evaluation apps.

```python
from imuFactors.vis import load_trajectory, load_nees_summary, discover_trajectories

# Load single trajectory
traj = load_trajectory("gal3_trajectory_MH01_2s.csv")

# Discover all trajectories for a dataset
trajectories = discover_trajectories("gal3", "MH01")
# Returns: {'2s': 'path/to/gal3_trajectory_MH01_2s.csv', ...}

# Load NEES summary
nees = load_nees_summary("nees_summary.csv")
```

### plotly_visualizer
Interactive HTML visualizations using Plotly.

```python
from imuFactors.vis import plot_3d_trajectory, plot_comparison, create_nees_summary_figure

# 3D trajectory
fig = plot_3d_trajectory(gt_traj, pred_traj, title="My Trajectory")
fig.show()
fig.write_html("trajectory.html")

# Compare best/worst noise parameters
fig = plot_comparison("MH01", gt, best, worst, best_nees=0.5, worst_nees=2.3)
fig.write_html("comparison.html")

# NEES summary bar chart
fig = create_nees_summary_figure(nees_summary)
fig.show()
```

### matplotlib_visualizer
Static PNG visualizations using Matplotlib.

```python
from imuFactors.vis import (
    plot_position_timeseries,
    plot_velocity_timeseries,
    plot_orientation_timeseries,
    plot_frequency_spectrum,
    plot_3d_trajectory_matplotlib,
)

# Time series plots
plot_position_timeseries(traj, save_path="position.png")
plot_velocity_timeseries(traj, save_path="velocity.png")
plot_orientation_timeseries(traj, save_path="orientation.png")

# 3D static plot
plot_3d_trajectory_matplotlib(gt, pred, save_path="trajectory_3d.png")

# Frequency analysis
plot_frequency_spectrum(traj, save_path="spectrum.png")
```

### noise_calibration
Visualize noise calibration results (best/worst NEES).

```python
from imuFactors.vis import (
    plot_nees_comparison,
    plot_alpha_parameters,
    plot_nees_ratio,
    generate_noise_calibration_report,
)

# Bar charts
plot_nees_comparison(nees_summary, save_path="nees.png")
plot_alpha_parameters(nees_summary, save_path="alpha.png")
plot_nees_ratio(nees_summary, save_path="ratio.png")

# Generate full report
generate_noise_calibration_report(nees_summary, output_dir="./plots")
```

## Expected CSV Formats

### Trajectory CSV (from evalExportTrajectories.cpp)
```
timestamp,gt_x,gt_y,gt_z,gt_vx,gt_vy,gt_vz,gt_roll,gt_pitch,gt_yaw,
pred_x,pred_y,pred_z,pred_vx,pred_vy,pred_vz,pred_roll,pred_pitch,pred_yaw
```

### NEES Summary CSV (from evalNoiseCalibration.cpp)
```
dataset,best_nees,worst_nees,nees_ratio,best_alpha_gyro,best_alpha_acc,
worst_alpha_gyro,worst_alpha_acc
```

## Requirements

- Python >= 3.8
- numpy >= 1.20
- pandas >= 1.3
- plotly >= 5.0
- matplotlib >= 3.4

## License

See LICENSE in repository root (BSD-3-Clause).