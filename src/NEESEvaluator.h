/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NEESEvaluator.h
 * @brief   Normalized Estimation Error Squared (NEES) Metrics Evaluator
 * @author  Alec Kain
 */

#pragma once

#include "Dataset.h"
#include "NEESResults.h"
#include "Window.h"
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <optional>
#include <vector>

namespace gtsam {

/// NEES evaluation for IMU preintegration
class NEESEvaluator {
public:


    /// Evaluates NEES values given a dataset (Dataset)
    explicit NEESEvaluator(const Dataset& dataset) : dataset_(dataset) {}

    /// Method for running the evaluator with parameters for interval (double) and alpha size (double)
    /// Returns NEES results instead of printing
    NEESResults run(double interval, double alpha = 3.0) const;

    /// Compute statistics from NEES values (made public for EKF evaluator)
    static NEESResults computeStatistics(const std::vector<double>& neesResults, double preintTime);

private:
    const Dataset& dataset_;

    /// Build the navigation-plus-bias error vector for one propagated window
    Vector computeError(const NavState& predicted, 
                       const NavState& actual,
                       const imuBias::ConstantBias& biasPred,
                       const imuBias::ConstantBias& biasActual) const;

    /// Compute normalized estimation error squared from an error/covariance
    /// pair
    std::optional<double> computeNEES(const Vector& error,
                                      const Matrix& covMatrix) const;

    /// Evaluate one complete integration window
    std::optional<double> calculateWindowNEES(
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        const Window& window) const;
};

} // namespace gtsam
