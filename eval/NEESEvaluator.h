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
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <optional>
#include <vector>

namespace gtsam {

class NEESEvaluator {
public:
    /**
     * @brief NEES evaluation results
     */
    struct NEESResults {
        std::vector<double> neesValues;
        double mean;
        double median;
        double variance;
        double preintTime;
    };

    // Evaluates NEES values given a dataset (Dataset)
    explicit NEESEvaluator(const Dataset& dataset) : dataset_(dataset) {}

    // Method for running the evaluator with parameters for interval (double) and alpha size (double)
    // Returns NEES results instead of printing
    NEESResults run(double interval, double alpha = 3.0) const; 

    // Static method for printing NEES results
    static void printNeesStatistics(const NEESResults& results);

private:
    const Dataset& dataset_;

    // Helper functions ordered "up" - used functions defined before calling functions
    
    bool isValidWindow(int startIdx, int endIdx) const;
    
    Vector computeError(const NavState& predicted, 
                       const NavState& actual,
                       const imuBias::ConstantBias& biasPred,
                       const imuBias::ConstantBias& biasActual) const;
                       
    std::optional<double> computeNEES(const Vector& error, const Matrix& covMatrix) const;
    
    std::optional<double> calculateWindowNEES(const std::shared_ptr<PreintegrationCombinedParams>& params,
                                            int startIdx, int endIdx, double dt) const;
    
    std::vector<double> processTimeWindow(const std::shared_ptr<PreintegrationCombinedParams>& params,
                                        int windowCount, int windowSize, double dt) const;
                                        
    NEESResults processTimeWindow(const std::shared_ptr<PreintegrationCombinedParams>& params,
                                double preintTime, double dt) const;
    
    // Statistics computation helper functions
    static double computeMean(const std::vector<double>& values);
    static double computeMedian(const std::vector<double>& values);
    static double computeVariance(const std::vector<double>& values, double mean);
    
    static NEESResults computeStatistics(const std::vector<double>& neesResults, double preintTime);
};

} // namespace gtsam