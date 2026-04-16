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

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <vector>
#include <string>

namespace gtsam {

/// Trajectory validation and export utilities
class TrajectoryValidator {
public:
    /// Data structure for trajectory point with covariance
    struct TrajectoryPoint {
        double timestamp;           ///< Time in seconds
        Point3 position;            ///< Position in meters
        Vector3 velocity;           ///< Velocity in m/s
        Vector3 rpy;                ///< Roll-pitch-yaw in degrees
        Matrix9 covariance;         ///< 9x9 covariance matrix (rotation, position, velocity)

        /// Default constructor
        TrajectoryPoint()
            : timestamp(0.0),
              position(Point3::Zero()),
              velocity(Vector3::Zero()),
              rpy(Vector3::Zero()),
              covariance(Matrix9::Zero()) {}
    };

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

} // namespace gtsam