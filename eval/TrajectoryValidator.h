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
 * @brief  Trajectory validation and CSV export utilities
 * @author Alec Kain
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <vector>
#include <string>

namespace gtsam {

/// Trajectory validation utilities
class TrajectoryValidator {
 public:
  /// Trajectory point with timestamp and covariance
  struct TrajectoryPoint {
    double timestamp;
    Vector3 position;
    Vector3 velocity;
    Vector3 rpy;  // Roll-Pitch-Yaw in degrees
    Matrix9 covariance;  // Full 9x9 covariance matrix
  };

  /// Export trajectory data to CSV with full covariance
  static void exportToCSV(
      const std::string& filename,
      const std::vector<TrajectoryPoint>& groundTruth,
      const std::vector<TrajectoryPoint>& predicted,
      const std::vector<Vector9>& errors);

  /// Export raw trajectories without pre-computed errors
  static void exportRawTrajectories(
      const std::string& filename,
      const std::vector<TrajectoryPoint>& groundTruth,
      const std::vector<TrajectoryPoint>& predicted);

  /// Print error statistics
  static void printErrorStatistics(const std::vector<Vector9>& errors);

 private:
  /// Extract diagonal standard deviations from covariance
  static Vector9 extractStdDev(const Matrix9& covariance);
};

}  // namespace gtsam