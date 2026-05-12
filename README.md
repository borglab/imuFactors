# imuFactors

Evaluation code for preintegrated IMU factors and EKF-based NEES studies on EuRoC-format CSV data.

All analysis apps now write a shared canonical CSV result package so downstream
Python tooling can load every run through the same schema.

## Current Repository Structure

```text
imuFactors/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── src/
│   ├── AppUtils.h
│   ├── Dataset.h / Dataset.cpp
│   ├── NEESEvaluator.h / NEESEvaluator.cpp
│   ├── EKFNEESEvaluator.h / EKFNEESEvaluator.cpp
│   ├── PIMs.h
│   ├── ResultsSchema.h
│   ├── ResultsWriter.h / ResultsWriter.cpp
│   ├── ResultsAdapters.h
│   ├── QuadratureRunner.h
│   ├── TrajectoryValidator.h / TrajectoryValidator.cpp
│   ├── NoiseCalibration.h
│   └── nees.h / nees.cpp
├── apps/
│   ├── evalGal3NavStateImuEKFNEES.cpp
│   ├── evalNoiseCalibration.cpp
│   ├── evalExportTrajectories.cpp
│   ├── evalReducedNeesWithPriorCovariance.cpp
│   └── evalQuadratureImuFactorDiagnostics.cpp
├── tests/
│   ├── CMakeLists.txt
│   ├── testImuNEES.cpp
│   ├── testDatasetSanity.cpp
│   └── testResultsWriter.cpp
├── integration/
│   ├── CMakeLists.txt
│   ├── testImuFactorMC.cpp
│   └── testAHRSFactorMC.cpp
└── data/
    └── euroc/
        ├── euroc_MH01.csv ... euroc_MH05.csv
        └── euroc_V101.csv ... euroc_V203.csv
```

## What Each Module Does

- `src/Dataset.*`: Loads EuRoC CSV files and builds `NavState` + bias + IMU measurement vectors. Also creates preintegration parameter objects from fixed noise baselines and scaling factors.
- `src/NEESEvaluator.*`: Computes 15-DOF NEES statistics for preintegrated IMU windows.
- `src/EKFNEESEvaluator.*`: Compares `Gal3ImuEKF` and `NavStateImuEKF`, computes windowed NEES, and produces canonical trajectory/window artifacts.
- `src/PIMs.h`: Shared preintegration helpers and shared window-evaluation types.
- `src/ResultsSchema.h`: Canonical row definitions and fixed CSV column order for all apps.
- `src/ResultsWriter.*`: Shared run-directory creation and canonical CSV writing.
- `src/ResultsAdapters.h`: Thin adapters that translate evaluator outputs into canonical row writes.
- `src/QuadratureRunner.h`: Shared quadrature runner and dataset-app bootstrap helpers.
- `src/TrajectoryValidator.*`: 9D navigation error and covariance utilities used by EKF export paths.
- `src/NoiseCalibration.h`: Header-only calibration utilities for dataset discovery and alpha-grid search.
- `src/AppUtils.h`: Shared CLI parsing and dataset/interval selection helpers for dataset-driven apps.
- `src/nees.*`: Shared, documented equations for NEES and descriptive statistics.

## Build

### Configure (local tests only)

```bash
cmake -S . -B build -DGTSAM_DIR=~/git/gtsam-private/build
```

### Configure (include long-running integration MC tests)

```bash
cmake -S . -B build \
  -DGTSAM_DIR=~/git/gtsam-private/build \
  -DIMUFACTORS_BUILD_INTEGRATION_TESTS=ON
```

### Build

```bash
cmake --build build -j6
```

## Running Tests

### List registered tests

```bash
cd build
ctest -N
```

### Local tests (`tests/`)

- `tests/testImuNEES.cpp` -> `testImuNEES` -> `testImuNEES.run`
- `tests/testDatasetSanity.cpp` -> `testDatasetSanity` -> `testDatasetSanity.run`
- `tests/testResultsWriter.cpp` -> `testResultsWriter` -> `testResultsWriter.run`

```bash
cd build
make -j6 testImuNEES.run
make -j6 testDatasetSanity.run
make -j6 testResultsWriter.run
```

### Monte Carlo tests (`integration/`, opt-in)

- `integration/testImuFactorMC.cpp` -> `testImuFactorMC` -> `testImuFactorMC.run`
- `integration/testAHRSFactorMC.cpp` -> `testAHRSFactorMC` -> `testAHRSFactorMC.run`

```bash
cd build
make -j6 testImuFactorMC.run
make -j6 testAHRSFactorMC.run
```

## Running Apps

From the build directory, every app writes results to:

```text
./results/<app-name>/<run-id>/
```

Each run directory contains the same file set:

```text
run_metadata.csv
datasets.csv
window_metrics.csv
window_summaries.csv
trajectory_samples.csv
calibration_trials.csv
calibration_summaries.csv
```

Unused files are still created with headers and zero rows so downstream loaders
can assume a fixed package shape.

The shared method labels are:

- `quadrature`
- `manifold`
- `tangent`
- `delama_gal3`
- `gal3_imu_ekf`
- `navstate_imu_ekf`

From the build directory:

```bash
cd build
./evalGal3NavStateImuEKFNEES [--data-dir <path>] [--output-root <path>] [--dataset <name>]
./evalNoiseCalibration [--output-root <path>] [dataset_type] [search_mode]
./evalExportTrajectories [options]
./evalReducedNeesWithPriorCovariance [options]
./evalQuadratureImuFactorDiagnostics [options]
```

Examples:

```bash
cd build
./evalGal3NavStateImuEKFNEES --dataset MH01 --output-root ./results
./evalNoiseCalibration machine_hall coarse
./evalNoiseCalibration --output-root ./results all both
./evalExportTrajectories --dataset-type vicon --output-root ./results
./evalExportTrajectories --best-gyro 13.0 --best-acc 9.4 --worst-gyro 0.5 --worst-acc 0.5
./evalReducedNeesWithPriorCovariance --dataset MH01 --max-intervals 1
./evalQuadratureImuFactorDiagnostics --dataset MH01 --max-intervals 1
```

### Canonical CSV Package

The canonical export contract is intended for one Python loader and one
visualization stack across all apps.

- `run_metadata.csv`: one row per app invocation with run id, app name, CLI
  args, output root, timestamp, and repo version if available.
- `datasets.csv`: dataset membership for the run, including source path and
  dataset-group label.
- `window_metrics.csv`: long-form per-window metrics shared by quadrature and
  EKF windowed analyses.
- `window_summaries.csv`: per-dataset, per-method summary statistics for those
  window metrics.
- `trajectory_samples.csv`: canonical trajectory export with GT state,
  predicted state, 9D error components, sigma summaries, and flattened 9x9
  covariance.
- `calibration_trials.csv`: one row per dataset per calibration trial.
- `calibration_summaries.csv`: best/worst summary rows for each calibration
  study.

### Quadrature Analysis Apps

Both quadrature analysis apps accept the same dataset-selection flags and write
the same canonical package:

```bash
--data-dir <path>       # dataset directory, default ../data/euroc/
--output-root <path>    # results root directory, default ./results
--dataset <name>        # restrict to one dataset, e.g. MH01 or euroc_MH01.csv
--max-intervals <count> # use only the first N default intervals
--no-delama-gal3        # skip delama_gal3 method rows in canonical tables
```

Use `evalReducedNeesWithPriorCovariance` for the normalized-NEES comparison
with initial state/bias covariance folded into the analysis, and
`evalQuadratureImuFactorDiagnostics` for the fuller NEES/error export over the
same intervals and datasets.

## Python Summary Viewer

The repository now includes a small Dash-based summary viewer for the canonical
CSV packages. The MVP is intentionally read-only and summary-focused.

### One-time setup

Use the `py312` conda environment for all Python tooling in this repository.

```bash
conda activate py312
pip install dash
```

### Launch

From the repository root:

```bash
python -m viewer.app
python -m viewer.nees_app
```

By default the viewer scans `build/results`. Override that with:

```bash
python -m viewer.app --results-root build/ui_probe
python -m viewer.nees_app --results-root build/ui_probe
```

Optional flags:

```bash
python -m viewer.app --results-root build/results --host 127.0.0.1 --port 8050
python -m viewer.nees_app --results-root build/results --host 127.0.0.1 --port 8051
```

### What The MVP Shows

- a discovered run catalog from one results root
- a refresh button to rescan that results root without restarting Dash
- run metadata from `run_metadata.csv`
- dataset membership from `datasets.csv`
- `window_summaries.csv` when populated
- a pivoted window-summary comparison table that lines methods up side-by-side
- `calibration_summaries.csv` when populated
- multi-run summary comparison across the active run plus additional selected runs

The app treats zero-byte, missing, or header-only CSVs as intentional empty
states where possible, so incomplete result packages do not crash the UI.

The NEES diagnostics viewer adds:

- normalized-NEES scatter and distribution views from `window_metrics.csv`
- outlier-window selection for interval drill-down within one run
- interval-detail plots from `trajectory_samples.csv` when a method exports rich trajectory diagnostics
- covariance/correlation heatmaps and whitened 2D residual projections for the active interval

### Current MVP Limitations

- summary tables only
- no `window_metrics.csv` detail tables yet
- no trajectory views from `trajectory_samples.csv` yet
- no graphs yet
- no write-back or package repair behavior

## Viewer Roadmap

Planned next steps for the Python viewer:

- detailed `window_metrics.csv` tables with additional filtering
- trajectory tables with careful sampling and downsampling controls
- Plotly charts for summaries and trajectory-derived metrics
- richer validation and diagnostics for incomplete or partially written runs
