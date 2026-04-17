/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   TrajectoryValidator.h
 * @brief  Trajectory validation utilities for EKF evaluation
 * @author Alec Kain
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <string>
#include <vector>

namespace gtsam {

/// Trajectory validation and export utilities
struct TrajectoryPoint {
  double timestamp = 0.0;               ///< Time in seconds
  Point3 position = Point3::Zero();     ///< Position in meters
  Vector3 velocity = Vector3::Zero();   ///< Velocity in m/s
  Vector3 rpy = Vector3::Zero();        ///< Roll-pitch-yaw in degrees
  Matrix9 covariance = Matrix9::Zero();  ///< 9x9 covariance matrix
};

class TrajectoryValidator {
 public:
  /// Data structure for trajectory point with covariance
  /// Extract standard deviations from covariance matrix diagonal
  static Vector9 extractStdDev(const Matrix9& covariance);

  /**
   * Export trajectory data to CSV file
   * @param filename Output CSV filename
   * @param groundTruthTrajectory Ground truth trajectory points
   * @param predictedTrajectory Predicted trajectory points with covariance
   * @param errorTrajectory Error vectors (9D: rotation, position, velocity)
   */
  static void exportToCSV(
      const std::string& filename,
      const std::vector<TrajectoryPoint>& groundTruthTrajectory,
      const std::vector<TrajectoryPoint>& predictedTrajectory,
      const std::vector<Vector9>& errorTrajectory);

  /**
   * Print RMS error statistics to console
   * @param errors Vector of 9D error vectors (rotation, velocity, position)
   */
  static void printErrorStatistics(const std::vector<Vector9>& errors);
};

}  // namespace gtsam
