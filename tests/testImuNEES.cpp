/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testImuNEES.cpp
 * @brief  Evaluations for NEES
 * @author Alec Kain
 */

#include <CppUnitLite/TestHarness.h>

#include <fstream>

#include "NEESEvaluator.h"
#include "PIMs.h"

#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

using namespace std;
using namespace gtsam;

namespace {

using PIMQuadrature = PreintegratedImuMeasurementsQ;
using PIMTangent = PreintegratedImuMeasurementsT<TangentPreintegration>;
using PIMManifold = PreintegratedImuMeasurementsT<ManifoldPreintegration>;

shared_ptr<PreintegrationParams> createPreintegrationParams() {
  constexpr double kAlpha = 3.0;
  constexpr double kSigmaGyro = kAlpha * 1.6968e-4;
  constexpr double kSigmaAcc = kAlpha * 2.0000e-3;

  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = I_3x3 * kSigmaAcc * kSigmaAcc;
  params->gyroscopeCovariance = I_3x3 * kSigmaGyro * kSigmaGyro;
  params->integrationCovariance = I_3x3 * 1e-8;
  return params;
}

bool isFinite(const WindowResult& result) {
  return std::isfinite(result.normalizedNees) &&
         std::isfinite(result.rotErrorNorm) &&
         std::isfinite(result.rotPredSigma) &&
         std::isfinite(result.posErrorNorm) &&
         std::isfinite(result.posPredSigma) &&
         std::isfinite(result.velErrorNorm) &&
         std::isfinite(result.velPredSigma);
}

bool hasNonDecreasingPredictedSigma(const WindowResult& baseline,
                                    const WindowResult& withPrior) {
  return withPrior.rotPredSigma >= baseline.rotPredSigma &&
         withPrior.posPredSigma >= baseline.posPredSigma &&
         withPrior.velPredSigma >= baseline.velPredSigma;
}

}  // namespace

/**
 * This test suite uses the EuRoC MAV dataset, published in:
 * M. Burri, J. Nikolic, P. Gohl, T. Schneider, J. Rehder, S. Omari, M. Achtelik
 * and R. Siegwart, The EuRoC micro aerial vehicle datasets, International
 * Journal of Robotic Research, DOI: 10.1177/0278364915620033, early 2016.
 *
 * Data download available at:
 * https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
 */

/* ************************************************************************* */
TEST(ImuFactor, NEES) {
  try {
    const string dataPath = "../data/euroc/euroc_V202.csv";
    auto dataset = Dataset(dataPath);
    auto evaluator = NEESEvaluator(dataset);

    // Test multiple preintegration intervals
    vector<double> intervals = {0.1, 0.2, 0.5, 1.0};
    for (double interval : intervals) {
      auto results = evaluator.run(interval, 3.0);  // alpha = 3
      results.printStatistics();
      EXPECT(!results.neesValues.empty());
    }
  } catch (const exception& e) {
    FAIL(e.what());
  }
}

/* ************************************************************************* */
TEST(ImuFactor, WindowEvaluation) {
  try {
    const string dataPath = "../data/euroc/euroc_V202.csv";
    const Dataset dataset(dataPath);
    const size_t stepsPerWindow = dataset.stepsForInterval(0.2);
    const vector<Window> windows = dataset.completeWindows(stepsPerWindow);
    EXPECT(!windows.empty());
    if (windows.empty()) return;

    const Window& window = windows.front();
    const auto params = createPreintegrationParams();
    const InitialCovarianceOptions initialCovariance{
        Matrix9::Identity() * 5e-6, Matrix6::Identity() * 1e-1};

    const auto manifold = evaluateWindow<PIMManifold>(window, params);
    const auto tangent = evaluateWindow<PIMTangent>(window, params);
    const auto quadrature = evaluateWindow<PIMQuadrature>(window, params,
                                                          std::nullopt, 3);
    const auto manifoldWithPrior =
        evaluateWindow<PIMManifold>(window, params, initialCovariance);
    const auto quadratureWithPrior =
        evaluateWindow<PIMQuadrature>(window, params, initialCovariance, 3);

    EXPECT(manifold.has_value());
    EXPECT(tangent.has_value());
    EXPECT(quadrature.has_value());
    EXPECT(manifoldWithPrior.has_value());
    EXPECT(quadratureWithPrior.has_value());

    if (!manifold || !tangent || !quadrature || !manifoldWithPrior ||
        !quadratureWithPrior) {
      return;
    }

    EXPECT(isFinite(*manifold));
    EXPECT(isFinite(*tangent));
    EXPECT(isFinite(*quadrature));
    EXPECT(isFinite(*manifoldWithPrior));
    EXPECT(isFinite(*quadratureWithPrior));

    EXPECT(hasNonDecreasingPredictedSigma(*manifold, *manifoldWithPrior));
    EXPECT(hasNonDecreasingPredictedSigma(*quadrature, *quadratureWithPrior));
  } catch (const exception& e) {
    FAIL(e.what());
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
