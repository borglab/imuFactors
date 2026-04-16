/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    nees.cpp
 * @brief   Shared NEES/statistics equations/utilities for IMU factor evaluation
 */

#include "nees.h"

#include <algorithm>
#include <cmath>

namespace gtsam {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
}  // namespace

std::optional<double> normalizedQuadraticForm(
    const Vector& error,
    const Matrix& covariance,
    double degreesOfFreedom) {
  if (degreesOfFreedom <= 0.0) return std::nullopt;
  try {
    return (error.transpose() * covariance.inverse() * error)(0, 0) / degreesOfFreedom;
  } catch (...) {
    return std::nullopt;
  }
}

double computeMean(const std::vector<double>& values) {
  if (values.empty()) return 0.0;

  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  return sum / values.size();
}

double computeMedian(const std::vector<double>& values) {
  if (values.empty()) return 0.0;

  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  const size_t n = sorted.size();
  if (n % 2 == 0) {
    return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
  }
  return sorted[n / 2];
}

double computeVariance(const std::vector<double>& values, double meanValue) {
  if (values.empty()) return 0.0;

  double accum = 0.0;
  for (const double value : values) {
    const double delta = value - meanValue;
    accum += delta * delta;
  }
  return accum / values.size();
}

double computePercentile(const std::vector<double>& values, double percentile) {
  if (values.empty()) return 0.0;

  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  const double boundedPercentile = std::clamp(percentile, 0.0, 100.0) / 100.0;
  const size_t index =
      static_cast<size_t>(std::floor(boundedPercentile * static_cast<double>(sorted.size() - 1)));
  return sorted[index];
}

Vector3 radiansToDegrees(const Vector3& rpyRadians) { return rpyRadians * kRadToDeg; }

}  // namespace gtsam
