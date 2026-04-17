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
  preintegrated.endPreintegration(preintegrated.deltaTij());
  return preintegrated;
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
 * Evaluate one window for a standard preintegration type.
 */
template <class PIMType>
std::optional<WindowResult> evaluateWindow(
    const Window& window, const std::shared_ptr<PreintegrationParams>& params,
    std::optional<InitialCovarianceOptions> initialCovariance = std::nullopt,
    size_t quadratureOrder = 0) {
  (void)quadratureOrder;
  const imuBias::ConstantBias& initialBias = window.initialTruth().bias;
  auto preintegrated = buildPreintegrated<PIMType>(window, params, initialBias);

  if (initialCovariance) {
    Matrix96 biasJacobian;
    preintegrated.biasCorrectedDelta(initialBias, biasJacobian);
    const Matrix9 totalCovariance =
        preintegrated.preintMeasCov() + initialCovariance->navCovariance +
        biasJacobian * initialCovariance->biasCovariance *
            biasJacobian.transpose();
    preintegrated.setPreintMeasCov(totalCovariance);
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
  return makeWindowResult(error, covariance, *normalizedNees);
}

/**
 * Convert quadrature preintegration output into the shared 9D residual space.
 */
inline Vector9 quadratureDeltaForBias(
    const Window& window, const std::shared_ptr<PreintegrationParams>& params,
    size_t quadratureOrder, const imuBias::ConstantBias& bias) {
  const auto preintegrated =
      buildPreintegrated<PreintegratedImuMeasurementsQ>(
          window, params, bias, quadratureOrder);

  Vector9 delta;
  delta << preintegrated.deltaRij().logmap(Rot3()), preintegrated.deltaPij(),
      preintegrated.deltaVij();
  return delta;
}

/**
 * Evaluate one window for quadrature preintegration.
 */
template <>
inline std::optional<WindowResult> evaluateWindow<PreintegratedImuMeasurementsQ>(
    const Window& window, const std::shared_ptr<PreintegrationParams>& params,
    std::optional<InitialCovarianceOptions> initialCovariance,
    size_t quadratureOrder) {
  const imuBias::ConstantBias& initialBias = window.initialTruth().bias;
  auto preintegrated = buildPreintegrated<PreintegratedImuMeasurementsQ>(
      window, params, initialBias, quadratureOrder);

  if (initialCovariance) {
    constexpr double kBiasPerturbation = 1e-5;
    Matrix96 biasJacobian;
    const Vector6 biasVector = initialBias.vector();
    for (int column = 0; column < 6; ++column) {
      Vector6 plusBias = biasVector;
      Vector6 minusBias = biasVector;
      plusBias(column) += kBiasPerturbation;
      minusBias(column) -= kBiasPerturbation;

      const Vector9 deltaPlus = quadratureDeltaForBias(
          window, params, quadratureOrder,
          imuBias::ConstantBias(plusBias.head<3>(), plusBias.tail<3>()));
      const Vector9 deltaMinus = quadratureDeltaForBias(
          window, params, quadratureOrder,
          imuBias::ConstantBias(minusBias.head<3>(), minusBias.tail<3>()));
      biasJacobian.col(column) =
          (deltaPlus - deltaMinus) / (2.0 * kBiasPerturbation);
    }

    preintegrated.setInitialCovariance(
        initialCovariance->navCovariance +
        biasJacobian * initialCovariance->biasCovariance *
            biasJacobian.transpose());
  }

  ImuFactorT<PreintegratedImuMeasurementsQ> factor(
      symbol_shorthand::X(1), symbol_shorthand::V(1), symbol_shorthand::X(2),
      symbol_shorthand::V(2), symbol_shorthand::B(1), preintegrated);
  const Vector9 error = factor.evaluateError(
      window.initialTruth().navState.pose(),
      window.initialTruth().navState.velocity(),
      window.terminalTruth().navState.pose(),
      window.terminalTruth().navState.velocity(), initialBias);
  const Matrix9 covariance = preintegrated.preintMeasCov();
  const auto normalizedNees = normalizedNEES(error, covariance, 9.0);
  if (!normalizedNees || !std::isfinite(*normalizedNees)) {
    return std::nullopt;
  }
  return makeWindowResult(error, covariance, *normalizedNees);
}

}  // namespace gtsam
