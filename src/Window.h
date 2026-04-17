/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <cstddef>
#include <utility>

#include "Dataset.h"

namespace gtsam {

/**
 * @brief Read-only handle for one complete integration window in a dataset
 *
 * A window references a source dataset and exposes the initial and terminal
 * states, biases, timestamps, and IMU measurements over the range
 * [start, end).
 */
struct Window {
  const Dataset* dataset;  ///< Source dataset
  size_t start;  ///< Initial state and IMU sample index
  size_t end;    ///< Terminal state index

  /// Number of integration steps in the window
  size_t stepCount() const { return end - start; }

  /// Window start time in seconds
  double startTime() const { return initialTruth().timestamp; }

  /// Window end time in seconds
  double endTime() const { return terminalTruth().timestamp; }

  /// Window duration in seconds
  double duration() const { return endTime() - startTime(); }

  /// Dataset timestep in seconds
  double timestep() const { return dataset->timestep(); }

  /// Initial state measurement
  const Dataset::Truth& initialTruth() const { return dataset->truth[start]; }

  /// Terminal state measurement
  const Dataset::Truth& terminalTruth() const { return dataset->truth[end]; }

  /**
   * @brief Call a function for each IMU measurement in the window
   * @param function Callable receiving `const Dataset::ImuMeasurement&`
   */
  template <typename Function>
  void forEachImuMeasurement(Function&& function) const {
    const auto& imuMeasurements = dataset->imu;
    for (size_t sampleIndex = start; sampleIndex < end; ++sampleIndex) {
      function(imuMeasurements[sampleIndex]);
    }
  }

  /**
   * @brief Integrate every IMU measurement in the window into an integrator
   * @param integrator Object exposing `integrateMeasurement(acc, omega, dt)`
   */
  template <typename Integrator>
  void integrateMeasurements(Integrator& integrator) const {
    const double dt = timestep();
    forEachImuMeasurement([&integrator,
                           dt](const Dataset::ImuMeasurement& measurement) {
      integrator.integrateMeasurement(measurement.acc, measurement.omega, dt);
    });
  }
};

}  // namespace gtsam
