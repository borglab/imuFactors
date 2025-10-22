/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ValidationReport.cpp
 * @brief  Implementation of validation report utilities
 * @author Alec Kain
 */

#include "ValidationReport.h"

namespace gtsam {

void ValidationReport::printMethodResults(
    const std::string& datasetName,
    const std::string& methodName,
    const NEESEvaluator::NEESResults& results_0_2s,
    const NEESEvaluator::NEESResults& results_0_5s,
    const NEESEvaluator::NEESResults& results_1_0s) {
    
    std::cout << std::left << std::setw(16) << datasetName
              << std::setw(24) << methodName
              << std::fixed << std::setprecision(3)
              << std::setw(8) << results_0_2s.median
              << std::setw(8) << results_0_5s.median
              << std::setw(8) << results_1_0s.median
              << std::endl;
}

void ValidationReport::printDatasetResults(const DatasetResults& results) {
    // Print Gal3ImuEKF results
    printMethodResults(
        results.datasetName,
        "Gal3ImuEKF:",
        results.gal3_0_2s,
        results.gal3_0_5s,
        results.gal3_1_0s
    );
    
    // Print NavStateImuEKF results
    printMethodResults(
        results.datasetName,
        "NavStateImuEKF:",
        results.navstate_0_2s,
        results.navstate_0_5s,
        results.navstate_1_0s
    );
}

} // namespace gtsam