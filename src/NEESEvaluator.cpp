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
    const Window& window) const {
  PreintegratedCombinedMeasurements pim(params, window.initialBias());
  window.integrateMeasurements(pim);

  const auto predicted = pim.predict(window.initialState(), window.initialBias());
  const auto error = computeError(predicted, window.terminalState(),
                                  window.initialBias(), window.terminalBias());
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
  const auto windows = dataset_.completeWindowsForInterval(interval);

  std::vector<double> neesResults;
  neesResults.reserve(windows.size());
  for (const auto& window : windows) {
    auto nees = calculateWindowNEES(params, window);
    if (nees) {
      neesResults.push_back(*nees);
    }
  }
  return computeStatistics(neesResults, interval);
}

} // namespace gtsam
