/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    nees.h
 * @brief   Shared NEES/statistics equations/utilities for IMU factor evaluation
 *
 * This file centralizes equations that were previously duplicated across
 * evaluators, including:
 * - normalized quadratic forms used for NEES
 * - basic descriptive statistics
 * - radians-to-degrees conversion for roll-pitch-yaw vectors
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <optional>
#include <vector>

namespace gtsam {

/**
 * @brief Compute normalized quadratic form x' * P^{-1} * x / d.
 *
 * This is the core equation used for normalized estimation error metrics
 * (e.g. NEES). Returns nullopt if matrix inversion fails.
 *
 * @param error Error vector x
 * @param covariance Covariance matrix P
 * @param degreesOfFreedom Normalization term d (must be > 0)
 * @return Normalized scalar value, or nullopt on numerical failure
 */
std::optional<double> normalizedQuadraticForm(
    const Vector& error,
    const Matrix& covariance,
    double degreesOfFreedom);

/**
 * Arithmetic mean over values (returns 0.0 for empty input).
 */
double computeMean(const std::vector<double>& values);

/**
 * Median over values (returns 0.0 for empty input).
 */
double computeMedian(const std::vector<double>& values);

/**
 * Population variance over values using a caller-provided mean
 * (returns 0.0 for empty input).
 */
double computeVariance(const std::vector<double>& values, double meanValue);

/**
 * @brief Convert a roll-pitch-yaw vector from radians to degrees.
 * @param rpyRadians RPY angles in radians
 * @return RPY angles in degrees
 */
Vector3 radiansToDegrees(const Vector3& rpyRadians);

}  // namespace gtsam
