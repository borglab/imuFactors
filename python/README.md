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

## Delama Gal3 Runner

This repository now includes a Delama Gal3 evaluator in
`imuFactors.delama_gal3.preintegration_delama_gal3`.

Install torch in your active Python environment, then run:

```bash
cd python
pip install -e .[delama_gal3]
```

Then run from the repository root:

```bash
./.venv/bin/python python/run_delama_gal3.py
```

If you are already inside the `python/` directory, this also works:

```bash
python run_delama_gal3.py
```

Common options:

```bash
python run_delama_gal3.py \
    --dataset-glob "../data/euroc/euroc_*.csv" \
    --output-dir "../build/results/delama_gal3" \
    --alpha 8.4 \
    --preint-times 0.2 0.5 1.0
```

Outputs include per-sequence pickle files plus summary CSV files in the output
directory.

---

## Generating All Visualizations

The `imuFactors.visualize_all` script auto-discovers every dataset in the build folder and writes both PNG and HTML outputs.

```bash
# PNG + HTML for all datasets
python -m imuFactors.visualize_all --build-dir ./build --output-dir ./visualizations

# Specific datasets only
python -m imuFactors.visualize_all --datasets MH01 V202 V203 --build-dir ./build

# Specific intervals only
python -m imuFactors.visualize_all --datasets MH01 --intervals 2s 5s --build-dir ./build

# Interactive HTML only (no matplotlib required at runtime)
python -m imuFactors.visualize_all --no-png

# Static PNG only
python -m imuFactors.visualize_all --no-html
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

Each HTML file is a standalone HTML document that does not require a local server; Plotly is loaded from the CDN. Open in any browser.

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
    save_path="./vis/MH01/html/best_worst.html",
)

# From pre-loaded TrajectoryData objects
fig = plot_comparison(
    "MH01", ground_truth=gt, best_trajectory=best, worst_trajectory=worst,
    best_nees=0.42, worst_nees=3.71,
    save_path="./vis/MH01/html/comparison.html",
)
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
## Unified Gal(3) factor comparison

From the repository root, build and run the four-method comparison with the
`py312` conda environment:

```bash
cmake -S . -B build
make -C build -j6
conda run -n py312 python python/run_unified_imu_comparison.py
```

This runs `quadrature`, `manifold`, `galilean` (GTSAM's
`PreintegratedImuMeasurementsG`), and `delama_gal3_python` on all 11 bundled
EuRoC sequences at 0.2, 0.5, and 1.0 seconds with uniform gyro/accelerometer
noise scaling alpha=8.4. Use `--dataset MH01` for the end-to-end smoke run.
PyTorch must be installed in that environment. Optional `--binary`,
`--data-dir`, `--results-root`, and `--threads` arguments control paths and
CPU parallelism; the comparison methods, alpha, and intervals are fixed.
`--integration-covariance <q>` configures independent continuous position-drive
covariance in m²/s (default `1e-8`). It must be finite and nonnegative; zero is
valid. The same value is passed to all methods and is not scaled by alpha.

The orchestrator stages the C++ result outside the viewer discovery tree,
appends Python metrics and summaries using the existing CSV headers and run
identity, checks complete window coverage, and publishes one package under
`build/results/evalQuadratureImuFactorDiagnostics/<run_id>`. C++ metadata and
dataset membership are preserved verbatim, so `output_root` and `cli_args`
record the original staging location. Incomplete packages are never published.
The existing viewer discovers the completed package automatically.

Both implementations use the first CSV timestep and integrate `[start,start+N)`
with `N=round(interval/dt)`, using `start+N` as both endpoint and next start.
Delama retains its Gal(3) x gal(3) propagation, initial ground-truth bias, zero
initial covariance, and native error/covariance pairing for normalized 9-DOF
NEES with `1e-12` diagonal regularization. Its native rotation/velocity/position
blocks remain available for NEES and legacy standalone RMSE fields. Canonical
`endpoint_v2` rows instead report physical endpoint errors in R/P/V order:
`[Log(R_predᵀ R_gt), p_gt − p_pred, v_gt − v_pred]`, in radians, meters, and m/s.
Python transports its native covariance by the Gal(3) adjoint of the inverse
predicted increment, permutes R/V/P to R/P/V, and rotates position and velocity
perturbations into world coordinates. C++ rotates its prediction-tangent
covariance into the same reporting coordinates. Displayed sigmas are component
RMS values `sqrt(trace(block)/3)`, not standard deviations of the error norms.
NEES retains the native residual/covariance pair: adding `1e-12 I` in different
coordinates is not invariant under adjoint transport. Unregularized NEES is
invariant when residual and covariance are transported together.

Both loaders normalize ground-truth quaternions without modifying source CSVs;
nonfinite quaternions or norms at most `1e-12` fail with the source row. Python
adds independent `q * dt * I` to the native position covariance after each
propagation step. Predicted means and the propagation algorithm are unchanged.
Configuration labels include `endpoint_v2` and q; every new package includes
`verification.json` with the conventions, noise parameters, and coverage checks.
Historical packages retain their original reporting conventions.

Summaries use population variance, the ordinary median, and C++'s P95 order
statistic at `floor(0.95*(n-1))`.

The standalone `python/run_delama_gal3.py` still writes pickle and summary
outputs through the same window evaluator and accepts `--integration-covariance`.
Legacy `rmse_rotation_deg`, `rmse_position_m`, and `rmse_velocity_mps` remain
native-log metrics. New result fields `predicted_endpoints`, `physical_error`,
`reporting_covariance`, and `predicted_increment` expose the physical evaluation. The factor harness accepts
`--no-galilean`; the former `--delama-gal3` and `--no-delama-gal3` EKF switches
have been removed. Historical tangent and EKF packages remain viewable.

Relevant tests (run C++ tests with escalated permissions):

```bash
make -C build -j6 testImuNEES.run testAppUtils.run testDatasetSanity.run testResultsWriter.run exportGalileanParity
PYTHONPATH=python conda run -n py312 python -m pytest python/tests -q
```

The `exportGalileanParity` test helper emits full-precision predictions, native
factor residuals, and covariance for a CSV and q. Python parity tests cover all
11 sequences and all three intervals, with position/velocity/rotation tolerances
`1e-8 m`, `1e-9 m/s`, and `1e-12` per matrix entry, and transported covariance
`atol=5e-11, rtol=1e-5`. MH01 additionally checks q=0 and q=2e-6.

Open the published run in the viewer:

```bash
conda run -n py312 python -m viewer.app --results-root build/results
```

Visit <http://127.0.0.1:8050> and select the new run ID.

## Held-out MH/V noise calibration

Use sequence-held-out calibration to assess whether one covariance model per
acquisition group generalizes better than a globally calibrated model:

```bash
make -C build -j6 exportGalileanParity testImuNEES.run testAppUtils.run testDatasetSanity.run
conda run -n py312 python python/run_group_noise_calibration.py
```

The runner compares the fixed alpha=8.4, q=1e-8 baseline with two protocols:
`global_loso` trains on all other sequences; `group_loso` trains only on other
MH or V sequences. Every interval of the held-out sequence remains excluded.
The fit uses Gaussian negative log-likelihood in physical reporting coordinates,
including the covariance log determinant. It weights sequences, intervals, and
the manifold/GTSAM Galilean methods equally, counting the equivalent Python
Galilean implementation only once. One alpha/q pair applies to all three methods
and intervals in each fold. Quadrature is excluded from this manuscript-focused
study. No reference windows are removed.

The covariance model is linear in `(alpha/8.4)^2` and q. The optimizer profiles
out the overall scale, scans and refines the remaining ratio, and explicitly
considers q=0. Every fitted held-out setting is checked against direct C++ and
Python propagation. Predicted motion and physical endpoint errors stay fixed;
only uncertainty, likelihood, and NEES change. NEES retains its native residual
and covariance, with the existing 1e-12 diagonal regularization.

Outputs under `build/noise-calibration/<timestamp>` include source hashes,
training membership and parameters for all 22 folds, the full-precision cache,
held-out likelihood/NEES/95% ellipsoid coverage, and an audit of reference
position increments against integrated reference velocity. Reference
inconsistencies are diagnostic findings, not exclusion criteria. Global and
group cross-validation publish separate viewer-compatible packages under
`build/results/evalGroupNoiseCalibration`, each containing 28,554 window rows
and 99 summaries. The original results and manuscript are preserved.

`report.md` explains the weighting and interpretation; accompanying HTML plots
show held-out metrics and the largest MH reference discontinuities. Lower NLL
indicates a better predictive covariance model, but does not imply a more accurate
mean trajectory. Coverage is descriptive because windows across interval
partitions overlap and real reference errors need not follow the assumed model.

## Fixed MH/V manuscript rerun

After choosing the grouping protocol, rerun all methods with the rounded full-group
settings (these are descriptive refitted results, not held-out scores):

```bash
conda run -n py312 python python/run_fixed_group_imu_comparison.py
```

MH uses alpha=9.85 and q=4e-5 m²/s; V uses alpha=16 and q=0. The runner directly
reruns C++ and Python on every sequence, retains manifold/Galilean/Delama rows,
validates the shared group configurations and window coverage, and publishes
one fresh package under `build/results/evalFixedGroupImuComparison`.

Generate the manuscript's pooled median table and all 99 per-sequence
supplemental rows from that package:

```bash
conda run -n py312 python python/export_euroc_paper_tables.py \
  build/results/evalFixedGroupImuComparison/<run_id> --output build/paper-tables
```

The exporter emits `euroc-section.tex`, `appendix-euroc-comparison.tex`, and
`table-values.json`. All reported performance statistics are medians; the
supplement includes median predicted sigmas and uses sequence mnemonics only.
The supplemental table uses `longtable` after the manuscript switches to one
column. No windows are excluded when calculating the medians.

### Retune the mean of block medians

To target the **arithmetic mean of per-sequence/per-interval NEES medians** at
one separately in MH and V, use the full-precision cache from the earlier
calibration and the fixed-group package as the starting point:

```bash
conda run -n py312 python python/fit_group_median_noise.py \
  --cache build/noise-calibration/<calibration_id>/cache \
  --seed-package build/results/evalFixedGroupImuComparison/<run_id> \
  --output build/median-noise-calibration/calibration.json
conda run -n py312 python python/run_fixed_group_imu_comparison.py \
  --calibration-json build/median-noise-calibration/calibration.json
```

This gives equal weight to each sequence, interval, and manifold/GTSAM Galilean
method; the equivalent Python Galilean method is counted once. It preserves
`q/alpha²`, fitting a single positive covariance multiplier `c` per group:
`alpha_new = sqrt(c) * alpha_old`, `q_new = c * q_old`. The native NEES residuals
are unchanged and the `1e-12` diagonal regularization stays fixed. The fitter
checks source hashes and reproduces the seed package from the cached arrays.
The runner performs fresh C++ and Python evaluations, requires the resulting
group averages to be within `1e-5` of one, and publishes a new
`evalMedianGroupImuComparison` package with exact settings and fit provenance.
The same table exporter accepts this package and explains the revised objective.

This is descriptive calibration on the full groups. It does not target each
individual median, a pooled median, or the mean of all window NEES values.
One is a chosen median normalization; the ideal Gaussian median of normalized
nine-dimensional NEES is approximately 0.927. Retain full parameter precision
when rerunning; the manuscript rounds settings for display.

To include quadrature in a separate viewer package at those same fitted settings:

```bash
conda run -n py312 python python/run_fixed_group_imu_comparison.py \
  --calibration-json reports/euroc_median_calibration/calibration.json \
  --include-quadrature
```

This directly reruns all four methods and publishes 38,072 window rows and 132
summaries under `build/results/evalMedianGroupImuComparisonWithQuadrature`.
Quadrature uses the existing parameters and does not enter the calibration
objective. The three-method manuscript packages remain available.
