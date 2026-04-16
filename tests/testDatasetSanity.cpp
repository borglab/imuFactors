/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testDatasetSanity.cpp
 * @brief  Lightweight sanity tests for dataset loading and NEES statistics
 */

#include <CppUnitLite/TestHarness.h>
#include "Dataset.h"
#include "NEESEvaluator.h"

#include <cmath>

using namespace gtsam;

/* ************************************************************************* */
TEST(Dataset, LoadEurocCsv) {
    Dataset dataset("../data/euroc/euroc_MH01.csv");

    const auto& states = dataset.getStates();
    const auto& imu = dataset.getImuData();

    EXPECT(!states.empty());
    EXPECT(states.size() == imu.size());
    EXPECT(std::abs(states.front().timestamp) < 1e-12);
    EXPECT(dataset.getGravity() > 0.0);
}

/* ************************************************************************* */
TEST(Dataset, ConfigureImuParamsScaling) {
    Dataset dataset("../data/euroc/euroc_MH01.csv");

    auto params = dataset.configureImuParams(2.0, 3.0);

    const double sigmaGyro = std::sqrt(params->gyroscopeCovariance(0, 0));
    const double sigmaAcc = std::sqrt(params->accelerometerCovariance(0, 0));

    EXPECT(std::abs(sigmaGyro - (2.0 * 1.6968e-4)) < 1e-12);
    EXPECT(std::abs(sigmaAcc - (3.0 * 2.0000e-3)) < 1e-12);
}

/* ************************************************************************* */
TEST(NEES, ComputeStatistics) {
    const std::vector<double> values{1.0, 2.0, 3.0, 4.0};
    auto stats = NEESEvaluator::computeStatistics(values, 0.2);

    EXPECT(std::abs(stats.mean - 2.5) < 1e-12);
    EXPECT(std::abs(stats.median - 2.5) < 1e-12);
    EXPECT(std::abs(stats.variance - 1.25) < 1e-12);
    EXPECT(std::abs(stats.preintTime - 0.2) < 1e-12);
    EXPECT(stats.neesValues.size() == values.size());
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
