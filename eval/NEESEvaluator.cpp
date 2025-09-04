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

NEESEvaluator::NoiseParams NEESEvaluator::computeNoiseParams(double alpha) {
    return {
        alpha * 1.6968e-4,  // sigmaGyro
        alpha * 2.0000e-3,  // sigmaAcc
        alpha * 1.9393e-5,  // sigmaGyroBias
        alpha * 3.0000e-3   // sigmaAccBias
    };
}

void NEESEvaluator::setImuCovariances(const std::shared_ptr<PreintegrationCombinedParams>& params, 
                                     const NoiseParams& noise) {
    params->setGyroscopeCovariance(Matrix3::Identity() * pow(noise.sigmaGyro, 2));
    params->setAccelerometerCovariance(Matrix3::Identity() * pow(noise.sigmaAcc, 2));
    params->setIntegrationCovariance(Matrix3::Zero());
    params->setBiasAccCovariance(Matrix3::Identity() * pow(noise.sigmaAccBias, 2));
    params->setBiasOmegaCovariance(Matrix3::Identity() * pow(noise.sigmaGyroBias, 2));
}

std::shared_ptr<PreintegrationCombinedParams> NEESEvaluator::configureImuParams(double alpha) {
    auto params = PreintegrationCombinedParams::MakeSharedD(dataset_.getGravity());
    params->n_gravity = Vector3(0, 0, -dataset_.getGravity());
    setImuCovariances(params, computeNoiseParams(alpha));
    return params;
}

Vector NEESEvaluator::computeError(const NavState& predicted, 
                                 const NavState& actual,
                                 const imuBias::ConstantBias& biasPred,
                                 const imuBias::ConstantBias& biasActual) {
    Vector15 error;
    error << predicted.localCoordinates(actual),
             biasActual.vector() - biasPred.vector();
    return error;
}

std::optional<double> NEESEvaluator::computeNEES(const Vector& error, const Matrix& covMatrix) {
    try {
        return (error.transpose() * covMatrix.inverse() * error)(0,0) / 15.0;
    } catch (...) {
        return std::nullopt;
    }
}

bool NEESEvaluator::isValidWindow(const Dataset& data, int startIdx, int endIdx) {
    return endIdx <= data.getStates().size() && endIdx <= data.getImuData().size();
}

std::optional<double> NEESEvaluator::calculateWindowNEES(const Dataset& data, 
                                                       const std::shared_ptr<PreintegrationCombinedParams>& params,
                                                       int startIdx, int endIdx, double dt) {
    if (!isValidWindow(data, startIdx, endIdx)) return std::nullopt;
    
    const auto& states = data.getStates();
    const auto& imuData = data.getImuData();
    
    PreintegratedCombinedMeasurements pim(params, states[startIdx].bias);
    
    for (int k = startIdx; k < endIdx - 1; k++) {
        pim.integrateMeasurement(imuData[k].acc, imuData[k].omega, dt);
    }
    
    auto predicted = pim.predict(states[startIdx].navState, states[startIdx].bias);
    auto error = computeError(predicted, states[endIdx - 1].navState,
                            states[startIdx].bias, states[endIdx - 1].bias);
    return computeNEES(error, pim.preintMeasCov());
}

std::vector<double> NEESEvaluator::processTimeWindow(const Dataset& data,
                                                   const std::shared_ptr<PreintegrationCombinedParams>& params,
                                                   int windowCount, int windowSize, double dt) {
    std::vector<double> neesResults;
    for (int m = 0; m < windowCount; m++) {
        auto nees = calculateWindowNEES(data, params, m*windowSize, 
                                      std::min((m+1)*windowSize, (int)data.getStates().size()), dt);
        if (nees) neesResults.push_back(*nees);
    }
    return neesResults;
}

void NEESEvaluator::processTimeWindow(const Dataset& data,
                                    const std::shared_ptr<PreintegrationCombinedParams>& params,
                                    double preintTime, double dt) {
    std::cout << "\nAnalyzing with preintegration time: " << preintTime << " s\n";
    const auto& states = data.getStates();
    double totalTime = states.back().timestamp - states.front().timestamp;
    int windowCount = static_cast<int>(totalTime / preintTime);
    int windowSize = static_cast<int>(states.size() / windowCount);
    auto results = processTimeWindow(data, params, windowCount, windowSize, dt);
    printNeesStatistics(results, preintTime);
}

void NEESEvaluator::printNeesStatistics(const std::vector<double>& neesResults, double preintTime) {
    if (neesResults.empty()) {
        throw std::runtime_error("No valid NEES results for preintegration time " + 
                               std::to_string(preintTime));
    }

    Eigen::Map<const Eigen::VectorXd> neesVector(neesResults.data(), neesResults.size());
    double mean = neesVector.mean();
    auto sortedResults = neesResults;
    std::sort(sortedResults.begin(), sortedResults.end());
    double median = sortedResults[sortedResults.size()/2];
    double variance = (neesVector.array() - mean).square().mean();

    std::cout << "ANEES (15-DOF):    mean | median | variance\n"
              << "--------------------------------------------\n"
              << "Results:   " << mean << " | " << median << " | " << variance << "\n";
}

void NEESEvaluator::run(double interval, double alpha) {
    auto params = configureImuParams(alpha);
    double dt = dataset_.getStates()[1].timestamp - dataset_.getStates()[0].timestamp;
    processTimeWindow(dataset_, params, interval, dt);
}

} // namespace gtsam