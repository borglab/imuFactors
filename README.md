# imuFactors

Evaluation code for preintegrated IMU factors and EKF-based NEES studies on EuRoC-format CSV data.

## Current Repository Structure

```text
imuFactors/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── src/
│   ├── Dataset.h / Dataset.cpp
│   ├── NEESEvaluator.h / NEESEvaluator.cpp
│   ├── EKFNEESEvaluator.h / EKFNEESEvaluator.cpp
│   ├── TrajectoryValidator.h / TrajectoryValidator.cpp
│   └── NoiseCalibration.h
├── apps/
│   ├── evalGal3NavStateImuEKFNEES.cpp
│   ├── evalNoiseCalibration.cpp
│   └── evalExportTrajectories.cpp
├── tests/
│   ├── CMakeLists.txt
│   ├── testImuNEES.cpp
│   └── testDatasetSanity.cpp
├── legacy/
│   ├── evalImuFactor.cpp
│   └── evalAHRSFactor.cpp
└── data/
    └── euroc/
        ├── euroc_MH01.csv ... euroc_MH05.csv
        └── euroc_V101.csv ... euroc_V203.csv
```

## What Each Module Does

- `src/Dataset.*`: Loads EuRoC CSV files and builds `NavState` + bias + IMU measurement vectors. Also creates preintegration parameter objects from fixed noise baselines and scaling factors.
- `src/NEESEvaluator.*`: Computes 15-DOF NEES statistics for preintegrated IMU windows.
- `src/EKFNEESEvaluator.*`: Compares `Gal3ImuEKF` and `NavStateImuEKF`, computes windowed NEES, and exports trajectory/NEES CSV files.
- `src/TrajectoryValidator.*`: CSV export helpers and RMS error summaries for 9D navigation error vectors.
- `src/NoiseCalibration.h`: Header-only calibration utilities for dataset discovery and alpha-grid search.

## CLI Entry Points

- `apps/evalGal3NavStateImuEKFNEES.cpp`: Prints Gal3 vs NavState EKF NEES comparison table.
- `apps/evalNoiseCalibration.cpp`: Coarse/fine grid search for `(alphaGyro, alphaAcc)`.
- `apps/evalExportTrajectories.cpp`: Exports trajectories and NEES summaries for best vs worst noise settings.

## Local Tests

- `tests/testImuNEES.cpp` -> target `testImuNEES` -> run target `testImuNEES.run`
- `tests/testDatasetSanity.cpp` -> target `testDatasetSanity` -> run target `testDatasetSanity.run`

Only local tests in `tests/` are wired into this repository build. Files in `legacy/` are reference-only and not built.

## Build and Run

### Configure

```bash
cmake -S . -B build -DGTSAM_DIR=~/git/gtsam-private/build
```

### Build

```bash
cmake --build build -j6
```

### List tests

```bash
cd build
ctest -N
```

### Run all local tests

```bash
cd build
ctest --output-on-failure
```

### Run one local test (`make -j6 testXXX.run`)

```bash
cd build
make -j6 testImuNEES.run
make -j6 testDatasetSanity.run
```
