/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>

#include <memory>
#include <string>
#include <vector>

namespace gtsam {

/**
 * @brief Container and loader for EuRoC MAV dataset
 *
 * Handles loading and storing synchronized IMU and ground truth state
 * measurements from the EuRoC MAV dataset format.
 */
class Dataset {
 public:
  /**
   * @brief Single IMU measurement with timestamp
   */
  struct ImuMeasurement {
    double timestamp;  ///< Time of measurement in seconds
    Vector3 omega;     ///< Angular velocity measurement in rad/s
    Vector3 acc;       ///< Linear acceleration measurement in m/s^2
  };

  /**
   * @brief Ground truth state measurement with timestamp
   */
  struct StateMeasurement {
    double timestamp;   ///< Time of measurement in seconds
    NavState navState;  ///< Navigation state (position, velocity, rotation)
    imuBias::ConstantBias bias;  ///< IMU bias states
  };

  /**
   * @brief Parameters for IMU noise characteristics
   */
  struct NoiseParams {
    double sigmaGyro;  ///< Gyroscope noise standard deviation
    double sigmaAcc;   ///< Accelerometer noise standard deviation
  };

  /**
   * @brief Index range for one complete fixed-step integration window
   *
   * The IMU integration range is [startIndex, endIndex), and the terminal
   * state for the window is states[endIndex].
   */
  struct Window {
    size_t startIndex;  ///< Initial state and bias index
    size_t endIndex;    ///< Terminal state index
    double startTime;   ///< Window start time in seconds
    double endTime;     ///< Window end time in seconds

    /// Number of integration steps in this window
    size_t stepCount() const { return endIndex - startIndex; }
  };

  /**
   * @brief Loads and validates EuRoC dataset from file
   *
   * @param filename Path to the dataset CSV file
   * @throw std::runtime_error if file cannot be opened or data is invalid
   */
  explicit Dataset(const std::string& filename);

  /**
   * @brief Get vector of ground truth state measurements
   * @return Const reference to state measurements
   */
  const std::vector<StateMeasurement>& getStates() const { return states_; }

  /**
   * @brief Get vector of IMU measurements
   * @return Const reference to IMU measurements
   */
  const std::vector<ImuMeasurement>& getImuData() const { return imuData_; }

  /**
   * @brief Get gravity constant used in the dataset
   * @return Gravity value in m/s^2
   */
  double getGravity() const { return gravity_; }

  /**
   * @brief Get the dataset timestep from the synchronized state trajectory
   * @return Positive timestep in seconds
   * @throw std::runtime_error if fewer than two state samples exist or the
   *         timestep is not positive
   */
  double timestep() const;

  /**
   * @brief Convert a requested interval into an integer step count
   * @param intervalSeconds Requested interval duration in seconds
   * @return Number of integration steps, rounded to the nearest timestep
   * @throw std::runtime_error if the interval is not positive
   */
  size_t stepsForInterval(double intervalSeconds) const;

  /**
   * @brief Enumerate all complete fixed-step windows in the dataset
   * @param stepsPerWindow Number of integration steps per window
   * @return Complete non-overlapping windows, dropping any trailing partial
   * window
   * @throw std::runtime_error if stepsPerWindow is zero
   */
  std::vector<Window> completeWindows(size_t stepsPerWindow) const;

  /**
   * @brief Enumerate all complete windows for a requested interval
   * @param intervalSeconds Requested interval duration in seconds
   * @return Complete non-overlapping windows for the nearest timestep count
   */
  std::vector<Window> completeWindowsForInterval(double intervalSeconds) const;

  /**
   * @brief Configure IMU preintegration parameters for this dataset
   * @param alphaGyro Scaling factor for gyro noise parameters
   * @param alphaAcc Scaling factor for accel noise parameters
   * @return Configured preintegration parameters
   */
  std::shared_ptr<PreintegrationCombinedParams> configureImuParams(
      double alphaGyro, double alphaAcc) const;

  /**
   * @brief Configure IMU preintegration parameters for this dataset
   * @param alpha Scaling factor for noise parameters (default: 3.0)
   * @return Configured preintegration parameters
   */
  std::shared_ptr<PreintegrationCombinedParams> configureImuParams(
      double alpha = 3.0) const;

 private:
  std::vector<StateMeasurement> states_;
  std::vector<ImuMeasurement> imuData_;
  const double gravity_ = 9.81;

  /// Compute noise parameters with uniform scaling
  /// @param alpha Scaling factor applied to all noise parameters
  /// @return Noise parameters structure
  NoiseParams computeNoiseParams(double alpha) const;

  /// Compute noise parameters with separate gyro and accel scaling
  /// @param alphaGyro Scaling factor for gyroscope noise
  /// @param alphaAcc Scaling factor for accelerometer noise
  /// @return Noise parameters structure
  NoiseParams computeNoiseParams(double alphaGyro, double alphaAcc) const;

  /// Set IMU covariances in preintegration parameters
  /// @param params Preintegration parameters to modify
  /// @param noise Noise parameters to apply
  void setImuCovariances(
      const std::shared_ptr<PreintegrationCombinedParams>& params,
      const NoiseParams& noise) const;

  /// Number of synchronized samples available in both state and IMU streams
  size_t synchronizedSampleCount() const;
};
}  // namespace gtsam
