#pragma once

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/QuadratureImuFactor.h>

#include <memory>

#include "Window.h"

namespace gtsam {

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

}  // namespace gtsam
