/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NEESEvaluator.cpp
 * @brief   Normalized Estimation Error Squared (NEES) Metrics Evaluator
 * @author  Alec Kain
 */

#include "NEESEvaluator.h"
#include <algorithm>
#include <iostream>

namespace gtsam {

// Helper functions ordered "up" - used functions defined before calling functions

bool NEESEvaluator::isValidWindow(int startIdx, int endIdx) const {
    return endIdx <= dataset_.getStates().size() && endIdx <= dataset_.getImuData().size();
}

Vector NEESEvaluator::computeError(const NavState& predicted, 
                                 const NavState& actual,
                                 const imuBias::ConstantBias& biasPred,
                                 const imuBias::ConstantBias& biasActual) const {
    Vector15 error;
    error << predicted.localCoordinates(actual),
             biasActual.vector() - biasPred.vector();
    return error;
}

std::optional<double> NEESEvaluator::computeNEES(const Vector& error, const Matrix& covMatrix) const {
    try {
        return (error.transpose() * covMatrix.inverse() * error)(0,0) / 15.0;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<double> NEESEvaluator::calculateWindowNEES(const std::shared_ptr<PreintegrationCombinedParams>& params,
                                                       int startIdx, int endIdx, double dt) const {
    if (!isValidWindow(startIdx, endIdx)) return std::nullopt;
    
    const auto& states = dataset_.getStates();
    const auto& imuData = dataset_.getImuData();
    
    PreintegratedCombinedMeasurements pim(params, states[startIdx].bias);
    
    for (int k = startIdx; k < endIdx - 1; k++) {
        pim.integrateMeasurement(imuData[k].acc, imuData[k].omega, dt);
    }
    
    auto predicted = pim.predict(states[startIdx].navState, states[startIdx].bias);
    auto error = computeError(predicted, states[endIdx - 1].navState,
                            states[startIdx].bias, states[endIdx - 1].bias);
    return computeNEES(error, pim.preintMeasCov());
}

std::vector<double> NEESEvaluator::processTimeWindow(const std::shared_ptr<PreintegrationCombinedParams>& params,
                                                   int windowCount, int windowSize, double dt) const {
    std::vector<double> neesResults;
    for (int m = 0; m < windowCount; m++) {
        auto nees = calculateWindowNEES(params, m*windowSize, 
                                      std::min((m+1)*windowSize, (int)dataset_.getStates().size()), dt);
        if (nees) neesResults.push_back(*nees);
    }
    return neesResults;
}

NEESEvaluator::NEESResults NEESEvaluator::processTimeWindow(const std::shared_ptr<PreintegrationCombinedParams>& params,
                                                          double preintTime, double dt) const {
    const auto& states = dataset_.getStates();
    double totalTime = states.back().timestamp - states.front().timestamp;
    int windowCount = static_cast<int>(totalTime / preintTime);
    int windowSize = static_cast<int>(states.size() / windowCount);
    auto neesValues = processTimeWindow(params, windowCount, windowSize, dt);
    return NEESEvaluator::computeStatistics(neesValues, preintTime);
}

// Statistics computation helper functions

double NEESEvaluator::computeMean(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    
    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / values.size();
}

double NEESEvaluator::computeMedian(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    
    std::vector<double> sortedValues = values;
    std::sort(sortedValues.begin(), sortedValues.end());
    size_t n = sortedValues.size();
    
    if (n % 2 == 0) {
        return (sortedValues[n/2 - 1] + sortedValues[n/2]) / 2.0;
    } else {
        return sortedValues[n/2];
    }
}

double NEESEvaluator::computeVariance(const std::vector<double>& values, double mean) {
    if (values.empty()) return 0.0;
    
    double variance = 0.0;
    for (double value : values) {
        variance += (value - mean) * (value - mean);
    }
    return variance / values.size();
}

NEESEvaluator::NEESResults NEESEvaluator::computeStatistics(const std::vector<double>& neesResults, double preintTime) {
    NEESResults results;
    results.neesValues = neesResults;
    results.preintTime = preintTime;
    
    if (neesResults.empty()) {
        results.mean = 0.0;
        results.median = 0.0;
        results.variance = 0.0;
        return results;
    }

    // Use helper functions to compute statistics
    results.mean = computeMean(neesResults);
    results.median = computeMedian(neesResults);
    results.variance = computeVariance(neesResults, results.mean);
    
    return results;
}

void NEESEvaluator::printNeesStatistics(const NEESResults& results) {
    std::cout << "\nAnalyzing with preintegration time: " << results.preintTime << " s\n";
    std::cout << "ANEES (15-DOF):    mean | median | variance\n"
              << "--------------------------------------------\n"
              << "Results:   " << results.mean << " | " << results.median << " | " << results.variance << "\n";
}

NEESEvaluator::NEESResults NEESEvaluator::run(double interval, double alpha) const {
    // Get configured IMU parameters from the dataset
    auto params = dataset_.configureImuParams(alpha);
    double dt = dataset_.getStates()[1].timestamp - dataset_.getStates()[0].timestamp;
    return processTimeWindow(params, interval, dt);
}

} // namespace gtsam