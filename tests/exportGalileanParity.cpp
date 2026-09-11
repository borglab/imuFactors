#include <gtsam/navigation/GalileanImuFactor.h>

#include <iomanip>
#include <iostream>

#include "QuadratureRunner.h"
using namespace gtsam;
template <class PIM>
void exportWindows(const Dataset &dataset,
                   const std::shared_ptr<PreintegrationParams> &params) {
  std::cout << std::setprecision(17) << "interval,start,end";
  for (const auto &prefix : {"pred", "pim"})
    for (int i = 0; i < 15; ++i) std::cout << ',' << prefix << '_' << i;
  for (int i = 0; i < 9; ++i) std::cout << ",error_" << i;
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 9; ++j) std::cout << ",cov_" << i << '_' << j;
  std::cout << '\n';
  const auto dumpState = [](const NavState &state) {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) std::cout << ',' << state.R()(i, j);
    for (int i = 0; i < 3; ++i) std::cout << ',' << state.position()(i);
    for (int i = 0; i < 3; ++i) std::cout << ',' << state.velocity()(i);
  };
  for (double interval : {0.2, 0.5, 1.0}) {
    for (const auto &window : dataset.completeWindowsForInterval(interval)) {
      const auto &bias = window.initialTruth().bias;
      const auto pim = buildPreintegrated<PIM>(window, params, bias);
      const auto predicted = pim.predict(window.initialTruth().navState, bias);
      ImuFactor2T<PIM> factor(0, 1, 2, pim);
      const Vector9 error =
          factor.evaluateError(window.initialTruth().navState,
                               window.terminalTruth().navState, bias);
      const Matrix9 covariance = pim.preintMeasCov();
      std::cout << interval << ',' << window.start << ',' << window.end;
      dumpState(predicted);
      dumpState(pim.deltaXij());
      for (int i = 0; i < 9; ++i) std::cout << ',' << error(i);
      for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j) std::cout << ',' << covariance(i, j);
      std::cout << '\n';
    }
  }
}

int main(int argc, char **argv) {
  try {
    if (argc < 3 || argc > 5) {
      throw std::runtime_error(
          "Usage: exportGalileanParity <csv> <q> [galilean|manifold] [alpha]");
    }
    const Dataset dataset(argv[1]);
    const double alpha =
        argc == 5 ? parsePositiveDoubleOption("alpha", argv[4]) : 8.4;
    const auto params = makePreintegrationParams(
        AlphaPair{alpha, alpha}, parseIntegrationCovariance(argv[2]));
    const std::string method = argc >= 4 ? argv[3] : "galilean";
    if (method == "galilean") {
      exportWindows<PIMGalilean>(dataset, params);
    } else if (method == "manifold") {
      exportWindows<PIMManifold>(dataset, params);
    } else {
      throw std::runtime_error("Expected galilean or manifold method");
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
