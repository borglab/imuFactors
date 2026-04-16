/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    Dataset.cpp
 * @brief   EuRoC dataset loader and container implementation
 * @author  Alec Kain
 */

#include "Dataset.h"
#include "Window.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace gtsam {

/// Single-parameter version: uniform scaling
Dataset::NoiseParams Dataset::computeNoiseParams(double alpha) const {
  return {
      alpha * 1.6968e-4,  // sigmaGyro
      alpha * 2.0000e-3,  // sigmaAcc
  };
}

/// Two-parameter version: separate gyro and accel scaling
Dataset::NoiseParams Dataset::computeNoiseParams(double alphaGyro,
                                                 double alphaAcc) const {
  return {
      alphaGyro * 1.6968e-4,  // sigmaGyro (scaled independently)
      alphaAcc * 2.0000e-3,   // sigmaAcc  (scaled independently)
  };
}

void Dataset::setImuCovariances(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const NoiseParams& noise) const {
  params->setGyroscopeCovariance(Matrix3::Identity() *
                                 std::pow(noise.sigmaGyro, 2));
  params->setAccelerometerCovariance(Matrix3::Identity() *
                                     std::pow(noise.sigmaAcc, 2));
  params->setIntegrationCovariance(Matrix3::Zero());
}

double Dataset::timestep() const {
  if (states_.size() < 2) {
    throw std::runtime_error(
        "Dataset must contain at least two state samples.");
  }

  const double dt = states_[1].timestamp - states_[0].timestamp;
  if (dt <= 0.0) {
    throw std::runtime_error("Dataset timestep must be positive.");
  }
  return dt;
}

size_t Dataset::stepsForInterval(double intervalSeconds) const {
  if (intervalSeconds <= 0.0) {
    throw std::runtime_error("Interval must be positive.");
  }
  return std::max<size_t>(
      1, static_cast<size_t>(std::llround(intervalSeconds / timestep())));
}

std::vector<Window> Dataset::completeWindows(size_t stepsPerWindow) const {
  if (stepsPerWindow == 0) {
    throw std::runtime_error("stepsPerWindow must be positive.");
  }

  std::vector<Window> windows;
  const size_t sampleCount = synchronizedSampleCount();
  for (size_t startIndex = 0; startIndex + stepsPerWindow < sampleCount;
       startIndex += stepsPerWindow) {
    const size_t endIndex = startIndex + stepsPerWindow;
    windows.emplace_back(*this, startIndex, endIndex);
  }
  return windows;
}

std::vector<Window> Dataset::completeWindowsForInterval(
    double intervalSeconds) const {
  return completeWindows(stepsForInterval(intervalSeconds));
}

std::shared_ptr<PreintegrationCombinedParams> Dataset::configureImuParams(
    double alpha) const {
  auto params = PreintegrationCombinedParams::MakeSharedD(getGravity());
  params->n_gravity = Vector3(0, 0, -getGravity());
  setImuCovariances(params, computeNoiseParams(alpha));
  return params;
}

std::shared_ptr<PreintegrationCombinedParams> Dataset::configureImuParams(
    double alphaGyro, double alphaAcc) const {
  auto params = PreintegrationCombinedParams::MakeSharedD(getGravity());
  params->n_gravity = Vector3(0, 0, -getGravity());
  setImuCovariances(params, computeNoiseParams(alphaGyro, alphaAcc));
  return params;
}

// Main constructor implementation

Dataset::Dataset(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + filename);
  }

  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("Empty file: " + filename);
  }

  double timeStart = 0;
  bool isFirst = true;

  while (std::getline(file, line)) {
    std::vector<double> row;
    std::stringstream ss(line);
    std::string value;
    while (std::getline(ss, value, ',')) {
      row.push_back(std::stod(value));
    }

    if (isFirst) {
      timeStart = row[0];
      isFirst = false;
    }

    double timestamp = row[0] - timeStart;

    // Parse state measurement
    Rot3 rotation = Rot3::Quaternion(row[1], row[2], row[3], row[4]);
    Point3 velocity(row[5], row[6], row[7]);
    Point3 position(row[8], row[9], row[10]);
    NavState navState(rotation, position, velocity);

    Vector3 gyroBias(row[11], row[12], row[13]);
    Vector3 accBias(row[14], row[15], row[16]);
    imuBias::ConstantBias bias(accBias, gyroBias);

    states_.push_back({timestamp, navState, bias});

    // Parse IMU measurement
    Vector3 omega(row[17], row[18], row[19]);
    Vector3 acc(row[20], row[21], row[22]);
    imuData_.push_back({timestamp, omega, acc});
  }

  if (states_.empty() || imuData_.empty()) {
    throw std::runtime_error("No valid data found in file: " + filename);
  }
}

size_t Dataset::synchronizedSampleCount() const {
  return std::min(states_.size(), imuData_.size());
}

}  // namespace gtsam
