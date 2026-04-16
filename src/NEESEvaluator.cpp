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
#include "NEESResults.h"
#include "nees.h"
#include <iostream>

namespace gtsam {

Vector NEESEvaluator::computeError(const NavState& predicted, 
                                 const NavState& actual,
                                 const imuBias::ConstantBias& biasPred,
                                 const imuBias::ConstantBias& biasActual) const {
    Vector15 error;
    error << predicted.logmap(actual),
             biasActual.vector() - biasPred.vector();
    return error;
}

std::optional<double> NEESEvaluator::computeNEES(const Vector& error, const Matrix& covMatrix) const {
    return normalizedQuadraticForm(error, covMatrix, 15.0);
}

std::optional<double> NEESEvaluator::calculateWindowNEES(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const Dataset::Window& window, double dt) const {
  const auto& states = dataset_.getStates();
  const auto& imuData = dataset_.getImuData();

  PreintegratedCombinedMeasurements pim(params, states[window.startIndex].bias);

  for (size_t sampleIndex = window.startIndex; sampleIndex < window.endIndex;
       ++sampleIndex) {
    const auto& measurement = imuData[sampleIndex];
    pim.integrateMeasurement(measurement.acc, measurement.omega, dt);
  }

  const auto predicted = pim.predict(states[window.startIndex].navState,
                                     states[window.startIndex].bias);
  const auto error = computeError(predicted, states[window.endIndex].navState,
                                  states[window.startIndex].bias,
                                  states[window.endIndex].bias);
  return computeNEES(error, pim.preintMeasCov());
}

NEESResults NEESEvaluator::computeStatistics(const std::vector<double>& neesResults, double preintTime) {
    NEESResults results;
    results.neesValues = neesResults;
    results.preintTime = preintTime;
    
    if (neesResults.empty()) {
        results.mean = 0.0;
        results.median = 0.0;
        results.variance = 0.0;
        return results;
    }

    // Use shared math utilities for statistics.
    results.mean = computeMean(neesResults);
    results.median = computeMedian(neesResults);
    results.variance = computeVariance(neesResults, results.mean);
    
    return results;
}

NEESResults NEESEvaluator::run(double interval, double alpha) const {
  auto params = dataset_.configureImuParams(alpha);
  const double dt = dataset_.timestep();
  const auto windows = dataset_.completeWindowsForInterval(interval);

  std::vector<double> neesResults;
  neesResults.reserve(windows.size());
  for (const auto& window : windows) {
    auto nees = calculateWindowNEES(params, window, dt);
    if (nees) {
      neesResults.push_back(*nees);
    }
  }
  return computeStatistics(neesResults, interval);
}

} // namespace gtsam
