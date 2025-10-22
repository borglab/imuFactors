/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ValidationReport.h
 * @brief  Validation report utilities for NEES evaluation
 * @author Alec Kain
 */

#pragma once

#include "NEESEvaluator.h"
#include <string>
#include <iostream>
#include <iomanip>

namespace gtsam {

/// Utilities for formatting and printing validation reports
class ValidationReport {
public:
    /// Structure to hold results for a single dataset
    struct DatasetResults {
        std::string datasetName;
        
        /// Gal3ImuEKF results at different time intervals
        NEESEvaluator::NEESResults gal3_0_2s;
        NEESEvaluator::NEESResults gal3_0_5s;
        NEESEvaluator::NEESResults gal3_1_0s;
        
        /// NavStateImuEKF results at different time intervals
        NEESEvaluator::NEESResults navstate_0_2s;
        NEESEvaluator::NEESResults navstate_0_5s;
        NEESEvaluator::NEESResults navstate_1_0s;
    };

    /// Print results for a single dataset
    static void printDatasetResults(const DatasetResults& results);

    /// Print results for a single method (Gal3 or NavState)
    static void printMethodResults(
        const std::string& datasetName,
        const std::string& methodName,
        const NEESEvaluator::NEESResults& results_0_2s,
        const NEESEvaluator::NEESResults& results_0_5s,
        const NEESEvaluator::NEESResults& results_1_0s);
};

} // namespace gtsam