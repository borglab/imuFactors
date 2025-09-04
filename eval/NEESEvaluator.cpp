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

NEESEvaluator::NoiseParams NEESEvaluator::computeNoiseParams(double alpha) const {
    // Dataset-dependent noise parameters - could be extended to analyze actual dataset statistics
    return {
        alpha * 1.6968e-4,  // sigmaGyro
        alpha * 2.0000e-3,  // sigmaAcc
        alpha * 1.9393e-5,  // sigmaGyroBias
        alpha * 3.0000e-3   // sigmaAccBias
    };
}

void NEESEvaluator::setImuCovariances(const std::shared_ptr<PreintegrationCombinedParams>& params, 
                                     const NoiseParams& noise) const {
    params->setGyroscopeCovariance(Matrix3::Identity() * pow(noise.sigmaGyro, 2));
    params->setAccelerometerCovariance(Matrix3::Identity() * pow(noise.sigmaAcc, 2));
    params->setIntegrationCovariance(Matrix3::Zero());
    params->setBiasAccCovariance(Matrix3::Identity() * pow(noise.sigmaAccBias, 2));
    params->setBiasOmegaCovariance(Matrix3::Identity() * pow(noise.sigmaGyroBias, 2));
}

std::shared_ptr<PreintegrationCombinedParams> NEESEvaluator::configureImuParams(double alpha) const {
    auto params = PreintegrationCombinedParams::MakeSharedD(dataset_.getGravity());
    params->n_gravity = Vector3(0, 0, -dataset_.getGravity());
    setImuCovariances(params, computeNoiseParams(alpha));
    return params;
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

bool NEESEvaluator::isValidWindow(int startIdx, int endIdx) const {
    return endIdx <= dataset_.getStates().size() && endIdx <= dataset_.getImuData().size();
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

    // Calculate mean
    double sum = 0.0;
    for (double value : neesResults) {
        sum += value;
    }
    results.mean = sum / neesResults.size();
    
    // Calculate median
    std::vector<double> sortedValues = neesResults;
    std::sort(sortedValues.begin(), sortedValues.end());
    size_t n = sortedValues.size();
    if (n % 2 == 0) {
        results.median = (sortedValues[n/2 - 1] + sortedValues[n/2]) / 2.0;
    } else {
        results.median = sortedValues[n/2];
    }
    
    // Calculate variance
    double variance = 0.0;
    for (double value : neesResults) {
        variance += (value - results.mean) * (value - results.mean);
    }
    results.variance = variance / neesResults.size();
    
    return results;
}

void NEESEvaluator::printNeesStatistics(const NEESResults& results) {
    std::cout << "\nAnalyzing with preintegration time: " << results.preintTime << " s\n";
    std::cout << "ANEES (15-DOF):    mean | median | variance\n"
              << "--------------------------------------------\n"
              << "Results:   " << results.mean << " | " << results.median << " | " << results.variance << "\n";
}

NEESEvaluator::NEESResults NEESEvaluator::run(double interval, double alpha) const {
    auto params = configureImuParams(alpha);
    double dt = dataset_.getStates()[1].timestamp - dataset_.getStates()[0].timestamp;
    return processTimeWindow(params, interval, dt);
}

} // namespace gtsam