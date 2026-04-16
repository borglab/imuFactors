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
 * [startIndex, endIndex).
 */
class Window {
 public:
  /**
   * @brief Construct one dataset-backed integration window
   * @param dataset Source dataset
   * @param startIndex Initial state and IMU sample index
   * @param endIndex Terminal state index, exclusive for IMU integration
   */
  Window(const Dataset& dataset, size_t startIndex, size_t endIndex);

  /// Initial state and IMU sample index
  size_t startIndex() const { return startIndex_; }

  /// Terminal state index
  size_t endIndex() const { return endIndex_; }

  /// Number of integration steps in the window
  size_t stepCount() const { return endIndex_ - startIndex_; }

  /// Window start time in seconds
  double startTime() const;

  /// Window end time in seconds
  double endTime() const;

  /// Window duration in seconds
  double duration() const;

  /// Dataset timestep in seconds
  double timestep() const;

  /// Initial state measurement
  const Dataset::StateMeasurement& initialStateMeasurement() const;

  /// Terminal state measurement
  const Dataset::StateMeasurement& terminalStateMeasurement() const;

  /// Initial navigation state
  const NavState& initialState() const;

  /// Terminal navigation state
  const NavState& terminalState() const;

  /// Initial bias state
  const imuBias::ConstantBias& initialBias() const;

  /// Terminal bias state
  const imuBias::ConstantBias& terminalBias() const;

  /**
   * @brief Call a function for each IMU measurement in the window
   * @param function Callable receiving `const Dataset::ImuMeasurement&`
   */
  template <typename Function>
  void forEachImuMeasurement(Function&& function) const {
    const auto& imuMeasurements = dataset_->getImuData();
    for (size_t sampleIndex = startIndex_; sampleIndex < endIndex_;
         ++sampleIndex) {
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

 private:
  const Dataset* dataset_;
  size_t startIndex_;
  size_t endIndex_;
};

}  // namespace gtsam
