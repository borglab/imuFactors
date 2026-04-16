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

#include <algorithm>
#include <cmath>

#include "Dataset.h"
#include "NEESEvaluator.h"
#include "Window.h"

using namespace gtsam;

namespace {

struct CountingIntegrator {
  size_t callCount = 0;
  Vector3 lastAcc = Vector3::Zero();
  Vector3 lastOmega = Vector3::Zero();
  double lastDt = 0.0;

  void integrateMeasurement(const Vector3& acc, const Vector3& omega, double dt) {
    ++callCount;
    lastAcc = acc;
    lastOmega = omega;
    lastDt = dt;
  }
};

}  // namespace

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
TEST(Dataset, CompleteWindowsForInterval) {
  Dataset dataset("../data/euroc/euroc_MH01.csv");

  const auto& states = dataset.getStates();
  const auto& imu = dataset.getImuData();
  const size_t sampleCount = std::min(states.size(), imu.size());

  EXPECT(std::abs(dataset.timestep() - 0.004999876022338867) < 1e-12);
  EXPECT(dataset.stepsForInterval(0.1) == 20);
  EXPECT(dataset.stepsForInterval(0.2) == 40);
  EXPECT(dataset.stepsForInterval(0.5) == 100);
  EXPECT(dataset.stepsForInterval(1.0) == 200);

  const auto windows = dataset.completeWindowsForInterval(0.2);
  EXPECT(!windows.empty());
  EXPECT(windows.front().startIndex() == 0);
  EXPECT(windows.front().endIndex() == 40);
  EXPECT(std::abs(windows.front().startTime() - states[0].timestamp) < 1e-12);
  EXPECT(std::abs(windows.front().endTime() - states[40].timestamp) < 1e-12);
  EXPECT(windows.front().stepCount() == 40);
  EXPECT(std::abs(
             windows.front().duration() -
             (states[40].timestamp - states[0].timestamp)) < 1e-12);
  EXPECT(std::abs(windows.front().timestep() - dataset.timestep()) < 1e-12);
  EXPECT(&windows.front().initialStateMeasurement() == &states[0]);
  EXPECT(&windows.front().terminalStateMeasurement() == &states[40]);
  EXPECT(&windows.front().initialState() == &states[0].navState);
  EXPECT(&windows.front().terminalState() == &states[40].navState);
  EXPECT((windows.front().initialBias().vector() - states[0].bias.vector()).norm() < 1e-12);
  EXPECT((windows.front().terminalBias().vector() - states[40].bias.vector()).norm() < 1e-12);
  EXPECT(windows.size() == (sampleCount - 1) / 40);
  EXPECT(windows.back().endIndex() == windows.size() * 40);
  EXPECT(windows.back().endIndex() < sampleCount);
  EXPECT((sampleCount - 1) - windows.back().endIndex() < 40);
}

/* ************************************************************************* */
TEST(Dataset, WindowMeasurementTraversal) {
  Dataset dataset("../data/euroc/euroc_MH01.csv");

  const auto windows = dataset.completeWindowsForInterval(0.2);
  EXPECT(!windows.empty());
  if (windows.empty()) {
    return;
  }

  const Window& window = windows.front();
  const auto& imu = dataset.getImuData();
  size_t visitedSamples = 0;
  const Dataset::ImuMeasurement* firstMeasurement = nullptr;
  const Dataset::ImuMeasurement* lastMeasurement = nullptr;
  window.forEachImuMeasurement([&](const Dataset::ImuMeasurement& measurement) {
    if (visitedSamples == 0) {
      firstMeasurement = &measurement;
    }
    lastMeasurement = &measurement;
    ++visitedSamples;
  });

  EXPECT(visitedSamples == window.stepCount());
  EXPECT(firstMeasurement == &imu[window.startIndex()]);
  EXPECT(lastMeasurement == &imu[window.endIndex() - 1]);

  CountingIntegrator integrator;
  window.integrateMeasurements(integrator);
  EXPECT(integrator.callCount == window.stepCount());
  EXPECT((integrator.lastAcc - imu[window.endIndex() - 1].acc).norm() < 1e-12);
  EXPECT((integrator.lastOmega - imu[window.endIndex() - 1].omega).norm() < 1e-12);
  EXPECT(std::abs(integrator.lastDt - dataset.timestep()) < 1e-12);
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
