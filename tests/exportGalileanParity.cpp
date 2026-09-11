#include <gtsam/navigation/GalileanImuFactor.h>
#include "QuadratureRunner.h"
#include <iomanip>
#include <iostream>
using namespace gtsam;
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: exportGalileanParity <csv> <integration-covariance>\n";
    return 2;
  }
  const Dataset dataset(argv[1]);
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->gyroscopeCovariance = I_3x3 * std::pow(8.4 * 1.6968e-4, 2);
  params->accelerometerCovariance = I_3x3 * std::pow(8.4 * 2e-3, 2);
  params->integrationCovariance = I_3x3 * parseIntegrationCovariance(argv[2]);
  std::cout << std::setprecision(17) << "interval,start,end";
  for (const auto& prefix : {"pred", "pim"})
    for (int i = 0; i < 15; ++i) std::cout << ',' << prefix << '_' << i;
  for (int i = 0; i < 9; ++i) std::cout << ",error_" << i;
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 9; ++j) std::cout << ",cov_" << i << '_' << j;
  std::cout << '\n';
  const auto dumpState = [](const NavState& state) {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) std::cout << ',' << state.R()(i,j);
    for (int i = 0; i < 3; ++i) std::cout << ',' << state.position()(i);
    for (int i = 0; i < 3; ++i) std::cout << ',' << state.velocity()(i);
  };
  for (double interval : {0.2, 0.5, 1.0}) {
    for (const auto& window : dataset.completeWindowsForInterval(interval)) {
      const auto& bias = window.initialTruth().bias;
      const auto pim = buildPreintegrated<PreintegratedImuMeasurementsG>(window, params, bias);
      const auto predicted = pim.predict(window.initialTruth().navState, bias);
      GalileanImuFactor2 factor(0, 1, 2, pim);
      const Vector9 error = factor.evaluateError(window.initialTruth().navState, window.terminalTruth().navState, bias);
      const Matrix9 covariance = pim.preintMeasCov();
      std::cout << interval << ',' << window.start << ',' << window.end;
      dumpState(predicted);
      dumpState(pim.deltaXij());
      for (int i = 0; i < 9; ++i) std::cout << ',' << error(i);
      for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j) std::cout << ',' << covariance(i,j);
      std::cout << '\n';
    }
  }
}
