# imuFactors

Evaluation code for preintegrated IMU factors and EKF-based NEES studies on EuRoC-format CSV data.

## Current Repository Structure

```text
imuFactors/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── tests/
│   ├── testDatasetSanity.cpp
│   └── testImuNEES.cpp
└── eval/
    ├── Dataset.h / Dataset.cpp
    ├── NEESEvaluator.h / NEESEvaluator.cpp
    ├── EKFNEESEvaluator.h / EKFNEESEvaluator.cpp
    ├── TrajectoryValidator.h / TrajectoryValidator.cpp
    ├── NoiseCalibration.h
    ├── evalImuFactor.cpp (reference, not built as local test)
    ├── evalAHRSFactor.cpp (reference, not built as local test)
    ├── evalGal3NavStateImuEKFNEES.cpp
    ├── evalNoiseCalibration.cpp
    ├── evalExportTrajectories.cpp
    └── data/
        └── euroc/
            ├── euroc_MH01.csv ... euroc_MH05.csv
            └── euroc_V101.csv ... euroc_V203.csv
```

## What Each Module Does (Current State)

- `eval/Dataset.*`: Loads EuRoC CSV files and builds `NavState` + bias + IMU measurement vectors. Also creates preintegration parameter objects from fixed noise baselines and scaling factors.
- `eval/NEESEvaluator.*`: Computes 15-DOF NEES statistics for preintegrated IMU windows.
- `eval/EKFNEESEvaluator.*`: Compares `Gal3ImuEKF` and `NavStateImuEKF`, computes windowed NEES, and exports trajectory/NEES CSV files.
- `eval/TrajectoryValidator.*`: CSV export helpers and RMS error summaries for 9D navigation error vectors.
- `eval/NoiseCalibration.h`: Header-only calibration utilities for dataset discovery and alpha-grid search.

## CLI Entry Points in `eval/`

- `evalGal3NavStateImuEKFNEES.cpp`: Prints Gal3 vs NavState EKF NEES comparison table.
- `evalNoiseCalibration.cpp`: Coarse/fine grid search for `(alphaGyro, alphaAcc)`.
- `evalExportTrajectories.cpp`: Exports trajectories and NEES summaries for “best vs worst” noise settings.

## Local Tests in `tests/`

- `testImuNEES.cpp`: Basic NEES run over one dataset and several intervals.
- `testDatasetSanity.cpp`: Dataset load/config and NEES statistics sanity checks.

## Build and Run (as currently written)

### Dependencies

- CMake
- C++17 compiler
- GTSAM installed with CMake package config (`GTSAMConfig.cmake`)
- CppUnitLite from GTSAM toolchain

### Configure

```bash
cmake -S . -B build -DGTSAM_DIR=~/git/gtsam-private/build
```

### Build

```bash
cmake --build build -j6
```

### Run tests (CTest-style target executables)

```bash
cd build
ctest --output-on-failure
```

### Run individual tests (`make -j6 testXXX.run`)

```bash
cd build
make -j6 testImuNEES.run
make -j6 testDatasetSanity.run
```

Only local tests in `tests/` are wired into this repository build; upstream GTSAM tests are intentionally excluded.

## Important Current Caveats

- Data paths are inconsistent (`./data/euroc/...` vs `../eval/data/euroc/...`) and depend on run directory.
- `evalExportTrajectories.cpp` references `../eval/plot_trajectories_plotly.py`, but that script is not in this repository.

## Suggested Next Structural Step

Split reusable code and runnable programs:

- Move reusable components to `src/` + `include/` (`Dataset`, evaluators, validators).
- Keep CLI/test entry points in `apps/` or `eval/`.
- Build a shared/static library once, and link each executable against it.
- Replace `file(GLOB ...)` with explicit target definitions.
