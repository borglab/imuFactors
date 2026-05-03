#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/QuadratureImuFactor.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "Window.h"
#include "nees.h"

namespace gtsam {

/**
 * Reduced per-window metrics shared across the evaluation apps.
 */
struct WindowResult {
  double normalizedNees = 0.0;
  double rotErrorNorm = 0.0;
  double rotPredSigma = 0.0;
  double posErrorNorm = 0.0;
  double posPredSigma = 0.0;
  double velErrorNorm = 0.0;
  double velPredSigma = 0.0;
};

/**
 * @brief Aggregate summary across a set of per-window metrics.
 */
struct WindowResultSummary {
  size_t numWindows = 0;
  double normalizedNeesMean = 0.0;
  double normalizedNeesMedian = 0.0;
  double normalizedNeesP95 = 0.0;
  double normalizedNeesVariance = 0.0;
  double rotErrorMedian = 0.0;
  double rotPredSigmaMedian = 0.0;
  double posErrorMedian = 0.0;
  double posPredSigmaMedian = 0.0;
  double velErrorMedian = 0.0;
  double velPredSigmaMedian = 0.0;
};

/**
 * @brief One evaluated window with sample/time bounds plus reduced metrics.
 */
struct WindowEvaluation {
  size_t windowIndex = 0;
  size_t startSample = 0;
  size_t endSample = 0;
  double startTime = 0.0;
  double endTime = 0.0;
  WindowResult metrics;
};

/**
 * Optional prior covariance to fold into the preintegrated covariance.
 */
struct InitialCovarianceOptions {
  Matrix9 navCovariance = Matrix9::Zero();
  Matrix6 biasCovariance = Matrix6::Zero();
};

/**
 * Build a preintegrated IMU measurement object over one window at the given
 * bias linearization point.
 */
template <class PIMType>
PIMType buildPreintegrated(const Window& window,
                           const std::shared_ptr<PreintegrationParams>& params,
                           const imuBias::ConstantBias& bias, size_t N = 0) {
  (void)N;
  PIMType preintegrated(params, bias);
  window.integrateMeasurements(preintegrated);
  return preintegrated;
}

/**
 * Build a quadrature preintegrated IMU measurement object and finalize its
 * integration interval before use.
 */
template <>
inline PreintegratedImuMeasurementsQ
buildPreintegrated<PreintegratedImuMeasurementsQ>(
    const Window& window, const std::shared_ptr<PreintegrationParams>& params,
    const imuBias::ConstantBias& bias, size_t N) {
  PreintegratedImuMeasurementsQ preintegrated(params, bias, N);
  window.integrateMeasurements(preintegrated);
  preintegrated.endPreintegration(window.terminalTruth().timestamp -
                                  window.initialTruth().timestamp);
  return preintegrated;
}

/**
 * Fold optional initial uncertainty into PIM types that expose mutable
 * preintegrated measurement covariance.
 */
template <class PIMType>
void applyInitialCovariance(PIMType* preintegrated,
                            const imuBias::ConstantBias& initialBias,
                            const InitialCovarianceOptions& initialCovariance) {
  Matrix96 biasJacobian;
  preintegrated->biasCorrectedDelta(initialBias, biasJacobian);
  const Matrix9 totalCovariance =
      preintegrated->preintMeasCov() + initialCovariance.navCovariance +
      biasJacobian * initialCovariance.biasCovariance *
          biasJacobian.transpose();
  preintegrated->setPreintMeasCov(totalCovariance);
}

/**
 * Fold optional initial uncertainty into quadrature PIMs via their cache-aware
 * covariance API.
 */
inline void applyInitialCovariance(
    PreintegratedImuMeasurementsQ* preintegrated,
    const imuBias::ConstantBias& initialBias,
    const InitialCovarianceOptions& initialCovariance) {
  (void)initialBias;
  preintegrated->setInitialCovariances(initialCovariance.navCovariance,
                                       initialCovariance.biasCovariance);
}

/**
 * Convert a 3x3 covariance block into an RMS sigma proxy.
 */
inline double covarianceBlockSigma(const Matrix9& covariance, int blockStart) {
  return std::sqrt(std::max(
      0.0, covariance.block<3, 3>(blockStart, blockStart).trace() / 3.0));
}

/**
 * Reduce a 9D error/covariance pair into the per-window reporting metrics.
 */
inline WindowResult makeWindowResult(const Vector9& error,
                                     const Matrix9& covariance,
                                     double normalizedNees) {
  WindowResult result;
  result.normalizedNees = normalizedNees;
  result.rotErrorNorm = error.head<3>().norm();
  result.rotPredSigma = covarianceBlockSigma(covariance, 0);
  result.posErrorNorm = error.segment<3>(3).norm();
  result.posPredSigma = covarianceBlockSigma(covariance, 3);
  result.velErrorNorm = error.tail<3>().norm();
  result.velPredSigma = covarianceBlockSigma(covariance, 6);
  return result;
}

/**
 * Reduce predict-vs-ground-truth state differences into reporting metrics while
 * keeping NEES tied to the factor residual.
 */
template <class PIMType>
WindowResult makePredictionWindowResult(
    const PIMType& preintegrated, const Window& window,
    const imuBias::ConstantBias& initialBias, const Matrix9& covariance,
    double normalizedNees) {
  const NavState predicted =
      preintegrated.predict(window.initialTruth().navState, initialBias);
  const NavState& groundTruth = window.terminalTruth().navState;
  const Pose3 poseError = groundTruth.pose().between(predicted.pose());

  WindowResult result;
  result.normalizedNees = normalizedNees;
  result.rotErrorNorm = Rot3::Logmap(poseError.rotation()).norm();
  result.rotPredSigma = covarianceBlockSigma(covariance, 0);
  result.posErrorNorm = poseError.translation().norm();
  result.posPredSigma = covarianceBlockSigma(covariance, 3);
  result.velErrorNorm = (predicted.velocity() - groundTruth.velocity()).norm();
  result.velPredSigma = covarianceBlockSigma(covariance, 6);
  return result;
}

/**
 * @brief Summarize a collection of per-window results.
 */
inline WindowResultSummary summarizeWindowResults(
    const std::vector<WindowResult>& results) {
  WindowResultSummary summary;
  if (results.empty()) {
    return summary;
  }

  const auto project = [&results](auto member) {
    std::vector<double> values;
    values.reserve(results.size());
    for (const auto& result : results) {
      values.push_back(result.*member);
    }
    return values;
  };

  const std::vector<double> normalizedNeesValues =
      project(&WindowResult::normalizedNees);
  summary.numWindows = normalizedNeesValues.size();
  summary.normalizedNeesMean = computeMean(normalizedNeesValues);
  summary.normalizedNeesMedian = computeMedian(normalizedNeesValues);
  summary.normalizedNeesP95 = computePercentile(normalizedNeesValues, 95.0);
  summary.normalizedNeesVariance =
      computeVariance(normalizedNeesValues, summary.normalizedNeesMean);
  summary.rotErrorMedian = computeMedian(project(&WindowResult::rotErrorNorm));
  summary.rotPredSigmaMedian =
      computeMedian(project(&WindowResult::rotPredSigma));
  summary.posErrorMedian = computeMedian(project(&WindowResult::posErrorNorm));
  summary.posPredSigmaMedian =
      computeMedian(project(&WindowResult::posPredSigma));
  summary.velErrorMedian = computeMedian(project(&WindowResult::velErrorNorm));
  summary.velPredSigmaMedian =
      computeMedian(project(&WindowResult::velPredSigma));
  return summary;
}

/**
 * Evaluate one window for a standard preintegration type.
 */
template <class PIMType>
std::optional<WindowResult> evaluateWindow(
    const Window& window, const std::shared_ptr<PreintegrationParams>& params,
    std::optional<InitialCovarianceOptions> initialCovariance = std::nullopt,
    size_t quadratureOrder = 0) {
  const imuBias::ConstantBias& initialBias = window.initialTruth().bias;
  auto preintegrated =
      buildPreintegrated<PIMType>(window, params, initialBias, quadratureOrder);

  if (initialCovariance) {
    applyInitialCovariance(&preintegrated, initialBias, *initialCovariance);
  }

  ImuFactor2T<PIMType> factor(symbol_shorthand::X(1), symbol_shorthand::X(2),
                              symbol_shorthand::B(1), preintegrated);
  const Vector9 error =
      factor.evaluateError(window.initialTruth().navState,
                           window.terminalTruth().navState, initialBias);
  const Matrix9 covariance = preintegrated.preintMeasCov();
  const auto normalizedNees = normalizedNEES(error, covariance, 9.0);
  if (!normalizedNees || !std::isfinite(*normalizedNees)) {
    return std::nullopt;
  }
  return makePredictionWindowResult(preintegrated, window, initialBias,
                                    covariance, *normalizedNees);
}

}  // namespace gtsam
