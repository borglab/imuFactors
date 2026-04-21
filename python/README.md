# imuFactors Python Visualization Package

Visualization utilities for IMU factor evaluation. Supports **Plotly** (interactive HTML) and **Matplotlib** (static PNG) outputs, with per-trace legend toggling in all interactive plots.

## Installation

```bash
# From source
cd python
pip install -e .

# Or install dependencies directly
pip install numpy pandas plotly matplotlib
```

---

## Quick Start

```python
from imuFactors.vis import (
    load_trajectory_from_build,
    plot_3d_trajectory_multi_interval,
    plot_position_timeseries_multi_interval,
    plot_nees_comparison,
)

# Interactive 3-D plot comparing all preintegration intervals — opens in browser
fig = plot_3d_trajectory_multi_interval("MH01", filter_name="gal3", build_dir="./build")
fig.show()

# Save as self-contained HTML (legend-toggleable traces)
fig = plot_position_timeseries_multi_interval(
    "MH01", filter_name="gal3", build_dir="./build",
    save_path="./vis/MH01/html/position.html",
)
```

---

## Generating All Visualizations

The `generate_visualizations.py` script auto-discovers every dataset in the build folder and writes both PNG and HTML outputs.

```bash
# PNG + HTML for all datasets
python generate_visualizations.py --build-dir ./build --output-dir ./visualizations

# Specific datasets only
python generate_visualizations.py --datasets MH01 V202 V203 --build-dir ./build

# Interactive HTML only (no matplotlib required at runtime)
python generate_visualizations.py --no-png

# Static PNG only
python generate_visualizations.py --no-html
```

### Output structure

```
visualizations/
└── MH01/
    ├── 3d_trajectory.png
    ├── position.png
    ├── velocity.png
    ├── acceleration.png
    ├── orientation.png
    ├── displacement.png
    └── html/
        ├── 3d_trajectory.html   ← interactive, legend-toggleable
        ├── position.html
        ├── velocity.html
        ├── acceleration.html
        ├── orientation.html
        └── displacement.html
```

Each HTML file is fully self-contained (Plotly loaded from CDN). Open in any browser — no server required.

---

## Interactive HTML Features

All Plotly figures support:

| Interaction | Action |
|---|---|
| **Toggle a trace** | Single-click its legend entry |
| **Isolate a trace** | Double-click its legend entry |
| **Zoom / pan** | Click-drag on the plot |
| **Scroll zoom** | Mouse wheel |
| **Export PNG** | Camera icon in the mode bar |
| **Reset view** | Home icon in the mode bar |

This lets you, for example, hide all predictions and examine ground truth alone, or isolate the 10 s preintegration interval against GT.

---

## Modules

### `trajectory_loader`

Loads CSV output from the C++ evaluation apps and exposes a `TrajectoryData` dataclass with separate `gt_*` and `pred_*` fields.

```python
from imuFactors.vis import (
    load_trajectory,
    load_trajectory_from_build,
    discover_intervals,
    discover_all_datasets,
    TrajectoryData,
    DEFAULT_BUILD_DIR,
)

# Load a single CSV directly
traj = load_trajectory("gal3_trajectory_MH01_2s.csv")
# traj.gt_position  → (N, 3) ground truth XYZ
# traj.pred_position → (N, 3) predicted XYZ
# traj.gt_velocity, traj.pred_velocity, traj.gt_rpy, traj.pred_rpy ...

# Load from build folder by name
traj = load_trajectory_from_build("gal3", "MH01", interval="2s", build_dir="./build")

# Discover what's available
datasets  = discover_all_datasets("./build")   # {"gal3": ["MH01", "V202", ...]}
intervals = discover_intervals("MH01", "gal3", "./build")  # ["2s", "5s", "10s"]
```

---

### `plotly_3d` — Interactive HTML plots

All functions accept an optional `save_path` argument. When provided, the figure is written to a self-contained HTML file automatically.

#### 3-D trajectory

```python
from imuFactors.vis import (
    plot_3d_trajectory,                 # single TrajectoryData object
    plot_3d_trajectory_multi_interval,  # GT + all intervals from build folder
    plot_3d_trajectory_from_build,      # single interval from build folder
)

# Multi-interval comparison (most common)
fig = plot_3d_trajectory_multi_interval(
    "MH01",
    filter_name="gal3",
    build_dir="./build",
    intervals=["2s", "5s", "10s"],  # omit to auto-discover
    save_path="./vis/MH01/html/3d_trajectory.html",
)
fig.show()

# Single interval
fig = plot_3d_trajectory_from_build("MH01", interval="5s", build_dir="./build")
fig.show()
```

#### Time series (position, velocity, acceleration, orientation, displacement)

Each function follows the same signature and produces a 3-row shared-x figure.

```python
from imuFactors.vis import (
    plot_position_timeseries_multi_interval,
    plot_velocity_timeseries_multi_interval,
    plot_acceleration_timeseries_multi_interval,   # derived from velocity via finite diff
    plot_orientation_timeseries_multi_interval,    # roll, pitch, yaw (degrees)
    plot_displacement_timeseries_multi_interval,   # Δposition between timesteps
)

for plot_fn, name in [
    (plot_position_timeseries_multi_interval,    "position"),
    (plot_velocity_timeseries_multi_interval,    "velocity"),
    (plot_acceleration_timeseries_multi_interval,"acceleration"),
    (plot_orientation_timeseries_multi_interval, "orientation"),
    (plot_displacement_timeseries_multi_interval,"displacement"),
]:
    plot_fn(
        "MH01",
        filter_name="gal3",
        build_dir="./build",
        save_path=f"./vis/MH01/html/{name}.html",
    )
```

#### Best vs worst noise calibration

```python
from imuFactors.vis import plot_best_worst_comparison_plotly, plot_comparison

# From build folder (loads BEST/WORST CSVs automatically)
fig = plot_best_worst_comparison_plotly(
    "MH01",
    filter_name="gal3",
    build_dir="./build",
    nees_summary_path="./build/nees_summary.csv",
    save_path="./vis/MH01/html/best_worst.html",
)

# From pre-loaded TrajectoryData objects
fig = plot_comparison(
    "MH01", ground_truth=gt, best_trajectory=best, worst_trajectory=worst,
    best_nees=0.42, worst_nees=3.71,
    save_path="./vis/MH01/html/comparison.html",
)
```

#### NEES summary

```python
from imuFactors.vis import create_nees_summary_figure, create_nees_summary_from_build

# Build summary from per-dataset NEES CSVs in build folder
summary_df = create_nees_summary_from_build("./build", output_path="nees_summary.csv")

# Grouped bar chart (log-y scale)
fig = create_nees_summary_figure(summary_df, save_path="nees_summary.html")
fig.show()
```

#### Saving HTML manually

```python
from imuFactors.vis import save_html

fig = plot_3d_trajectory_multi_interval("MH01", build_dir="./build")
save_html(fig, "./my_output/MH01_3d.html")
```

---

### `matplotlib_visualizer` — Static PNG plots

```python
from imuFactors.vis import (
    # Single TrajectoryData
    plot_position_timeseries,
    plot_velocity_timeseries,
    plot_acceleration_timeseries,
    plot_orientation_timeseries,
    plot_displacement_timeseries,
    plot_3d_trajectory_matplotlib,
    # Multi-interval comparisons (mirrors Plotly API)
    plot_position_multi_interval,
    plot_velocity_multi_interval,
    plot_acceleration_multi_interval,
    plot_orientation_multi_interval,
    plot_displacement_multi_interval,
    plot_3d_trajectory_multi_interval_matplotlib,
)

# Single-trajectory plots (require a loaded TrajectoryData)
traj = load_trajectory_from_build("gal3", "MH01", "2s", "./build")
plot_position_timeseries(traj, save_path="position.png")
plot_3d_trajectory_matplotlib(traj, save_path="trajectory_3d.png")

# Multi-interval comparison (loads from build folder, same API as Plotly versions)
plot_position_multi_interval("MH01", filter_name="gal3", build_dir="./build",
                              save_path="position_multi.png")
plot_3d_trajectory_multi_interval_matplotlib("MH01", build_dir="./build",
                                              save_path="3d_multi.png")
```

---

### `noise_calibration` — Noise parameter analysis

```python
from imuFactors.vis import (
    plot_nees_comparison,
    plot_nees_comparison_matplotlib,
    plot_alpha_parameters,
    plot_nees_ratio,
    generate_noise_calibration_report,
    quick_nees_plot,
)

nees_summary = load_nees_summary("nees_summary.csv")

plot_nees_comparison(nees_summary, save_path="nees.html")         # Plotly
plot_nees_comparison_matplotlib(nees_summary, save_path="nees.png")  # Matplotlib
plot_alpha_parameters(nees_summary, save_path="alpha.png")
plot_nees_ratio(nees_summary, save_path="ratio.png")

# Full report: generates all noise calibration plots in one call
generate_noise_calibration_report(nees_summary, output_dir="./plots")
```

---

## Expected CSV Formats

### Trajectory CSV (`gal3_trajectory_<DATASET>_<INTERVAL>.csv`)

Output of `evalExportTrajectories.cpp`:

```
timestamp,
gt_x,gt_y,gt_z,
gt_vx,gt_vy,gt_vz,
gt_roll,gt_pitch,gt_yaw,
pred_x,pred_y,pred_z,
pred_vx,pred_vy,pred_vz,
pred_roll,pred_pitch,pred_yaw
```

### NEES summary CSV (`nees_summary.csv`)

Output of `evalNoiseCalibration.cpp`:

```
dataset,best_nees,worst_nees,nees_ratio,
best_alpha_gyro,best_alpha_acc,
worst_alpha_gyro,worst_alpha_acc
```

### Per-dataset NEES timeseries (`gal3_nees_<DATASET>_<INTERVAL>.csv`)

```
timestamp,nees
```

---

## Requirements

| Package | Minimum version |
|---|---|
| Python | 3.8 |
| numpy | 1.20 |
| pandas | 1.3 |
| plotly | 5.0 |
| matplotlib | 3.4 |

---

## License

See `LICENSE` in the repository root (BSD-3-Clause).