/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalQuadratureImuFactorDiagnostics.cpp
 * @brief  Minimal NEES and error analysis for Quadrature, Manifold, and
 * Tangent IMU preintegration
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

#include "AppUtils.h"
#include "QuadratureRunner.h"
#include "ResultsAdapters.h"

using namespace gtsam;
using namespace std;

namespace {

constexpr double kDefaultAlpha = 8.4;
}  // namespace

int main(int argc, char* argv[]) {
  const auto datasetCli = resolveDatasetAppCli(argc, argv);
  const QuadratureAppOptions appOptions =
      parseQuadratureAppArguments(datasetCli.remainingArgs, argv[0],
                                  kDefaultAlpha);

  return runDatasetApp(
      datasetCli, argc, argv,
      [&](ResultsWriter* writer, const std::string& datasetGroup) {
        return QuadratureRunner(
            appOptions, writer, datasetGroup, std::nullopt,
            [&appOptions](const std::string& datasetName) {
              return makePreintegrationParams(
                  alphaForDataset(appOptions, datasetName));
            },
            [&appOptions](const std::string& datasetName) {
              return alphaConfigLabel(alphaForDataset(appOptions, datasetName));
            });
      });
}
