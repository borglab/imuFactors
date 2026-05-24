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

  void integrateMeasurement(const Vector3& acc, const Vector3& omega,
                            double dt) {
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

  const auto& truth = dataset.truth;
  const auto& imu = dataset.imu;

  EXPECT(!truth.empty());
  EXPECT(truth.size() == imu.size());
  EXPECT(std::abs(truth.front().timestamp) < 1e-12);
  EXPECT(dataset.g > 0.0);
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

  const auto& truth = dataset.truth;
  const auto& imu = dataset.imu;
  const size_t sampleCount = std::min(truth.size(), imu.size());

  EXPECT(std::abs(dataset.timestep() - 0.004999876022338867) < 1e-12);
  EXPECT(dataset.stepsForInterval(0.1) == 20);
  EXPECT(dataset.stepsForInterval(0.2) == 40);
  EXPECT(dataset.stepsForInterval(0.5) == 100);
  EXPECT(dataset.stepsForInterval(1.0) == 200);

  const auto windows = dataset.completeWindowsForInterval(0.2);
  EXPECT(!windows.empty());
  EXPECT(windows.front().start == 0);
  EXPECT(windows.front().end == 40);
  EXPECT(&windows.front().initialTruth() == &truth[0]);
  EXPECT(&windows.front().terminalTruth() == &truth[40]);
  EXPECT(&windows.front().initialTruth().navState == &truth[0].navState);
  EXPECT(&windows.front().terminalTruth().navState == &truth[40].navState);
  EXPECT((windows.front().initialTruth().bias.vector() - truth[0].bias.vector())
             .norm() < 1e-12);
  EXPECT(
      (windows.front().terminalTruth().bias.vector() - truth[40].bias.vector())
          .norm() < 1e-12);
  EXPECT(windows.size() == (sampleCount - 1) / 40);
  EXPECT(windows.back().end == windows.size() * 40);
  EXPECT(windows.back().end < sampleCount);
  EXPECT((sampleCount - 1) - windows.back().end < 40);
}

/* ************************************************************************* */
TEST(Dataset, WindowInitialTruthVelocityNormStatistics) {
  Dataset dataset("../data/euroc/euroc_MH01.csv");

  const auto windows = dataset.completeWindowsForInterval(0.2);
  EXPECT(!windows.empty());
  if (windows.empty()) {
    return;
  }

  double velocityNormSum = 0.0;
  double velocityNormSquaredSum = 0.0;
  double minInitialVelocityNorm =
      windows.front().initialTruth().navState.velocity().norm();
  for (const Window& window : windows) {
    const double velocityNorm =
        window.initialTruth().navState.velocity().norm();
    EXPECT(std::isfinite(velocityNorm));
    velocityNormSum += velocityNorm;
    velocityNormSquaredSum += velocityNorm * velocityNorm;
    minInitialVelocityNorm = std::min(minInitialVelocityNorm, velocityNorm);
  }

  const double numWindows = static_cast<double>(windows.size());
  const double meanVelocityNorm = velocityNormSum / numWindows;
  const double velocityNormVariance = std::max(
      0.0, velocityNormSquaredSum / numWindows -
               meanVelocityNorm * meanVelocityNorm);
  const double velocityNormStddev = std::sqrt(velocityNormVariance);

  EXPECT(minInitialVelocityNorm > 1e-12);
  EXPECT(std::abs(meanVelocityNorm - 0.50659674603679938) < 1e-12);
  EXPECT(std::abs(velocityNormStddev - 0.265506565520361) < 1e-12);
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
  const auto& imu = dataset.imu;
  size_t visitedSamples = 0;
  const Dataset::ImuMeasurement* firstMeasurement = nullptr;
  const Dataset::ImuMeasurement* lastMeasurement = nullptr;
  window.forEachImuSample([&](const Dataset::ImuMeasurement& measurement) {
    if (visitedSamples == 0) {
      firstMeasurement = &measurement;
    }
    lastMeasurement = &measurement;
    ++visitedSamples;
  });

  EXPECT(visitedSamples == window.end - window.start);
  EXPECT(firstMeasurement == &imu[window.start]);
  EXPECT(lastMeasurement == &imu[window.end - 1]);

  CountingIntegrator integrator;
  window.integrateMeasurements(integrator);
  EXPECT(integrator.callCount == window.end - window.start);
  EXPECT((integrator.lastAcc - imu[window.end - 1].acc).norm() < 1e-12);
  EXPECT((integrator.lastOmega - imu[window.end - 1].omega).norm() < 1e-12);
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
