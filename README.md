# imuFactors

Evaluation code for preintegrated IMU factors and EKF-based NEES studies on EuRoC-format CSV data.

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
│   └── testDatasetSanity.cpp
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
- `src/EKFNEESEvaluator.*`: Compares `Gal3ImuEKF` and `NavStateImuEKF`, computes windowed NEES, and exports trajectory/NEES CSV files.
- `src/TrajectoryValidator.*`: CSV export helpers and RMS error summaries for 9D navigation error vectors.
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

```bash
cd build
make -j6 testImuNEES.run
make -j6 testDatasetSanity.run
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

From the build directory:

```bash
cd build
./evalGal3NavStateImuEKFNEES
./evalNoiseCalibration [dataset_type] [search_mode]
./evalExportTrajectories [options]
./evalReducedNeesWithPriorCovariance [options]
./evalQuadratureImuFactorDiagnostics [options]
```

Examples:

```bash
cd build
./evalNoiseCalibration machine_hall coarse
./evalNoiseCalibration all both
./evalExportTrajectories --dataset-type vicon
./evalExportTrajectories --best-gyro 13.0 --best-acc 9.4 --worst-gyro 0.5 --worst-acc 0.5
./evalReducedNeesWithPriorCovariance --dataset MH01 --max-intervals 1
./evalQuadratureImuFactorDiagnostics --dataset MH01 --max-intervals 1
```

### Quadrature Analysis Apps

Both quadrature analysis apps accept the same dataset-selection flags:

```bash
--data-dir <path>       # dataset directory, default ../data/euroc/
--dataset <name>        # restrict to one dataset, e.g. MH01 or euroc_MH01.csv
--max-intervals <count> # use only the first N default intervals
```

Use `evalReducedNeesWithPriorCovariance` for the reduced-NEES comparison table with
initial state/bias covariance folded into the comparison, and
`evalQuadratureImuFactorDiagnostics` for the fuller NEES/error summary tables.
