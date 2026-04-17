/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalReducedNeesWithPriorCovariance.cpp
 * @brief  Compare normalized NEES for Quadrature, Manifold, and Tangent IMU
 * preintegration
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "PIMs.h"
#include "ResultsAdapters.h"
#include "ResultsWriter.h"

using namespace gtsam;
using namespace std;

using PIMQuadrature = PreintegratedImuMeasurementsQ;
using PIMTangent = PreintegratedImuMeasurementsT<TangentPreintegration>;
using PIMManifold = PreintegratedImuMeasurementsT<ManifoldPreintegration>;

namespace {

constexpr double kAlpha = 3.0;
constexpr double kSigmaGyro = kAlpha * 1.6968e-4;
constexpr double kSigmaAcc = kAlpha * 2.0000e-3;
constexpr double kInitialStateCovariance = 5e-6;
constexpr double kInitialBiasCovariance = 1e-1;
constexpr const char* kConfigLabel = "default";

struct AppOptions {
  size_t maxIntervals = 0;
};

Matrix9 initialNavCovariance() {
  return Matrix9::Identity() * kInitialStateCovariance;
}

Matrix6 initialBiasCovariance() {
  return Matrix6::Identity() * kInitialBiasCovariance;
}

shared_ptr<PreintegrationParams> createPreintegrationParams() {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = I_3x3 * kSigmaAcc * kSigmaAcc;
  params->gyroscopeCovariance = I_3x3 * kSigmaGyro * kSigmaGyro;
  params->integrationCovariance = I_3x3 * 1e-8;
  return params;
}

void printUsage(const char* programName) {
  cout << "Usage: " << programName
       << " [--data-dir <path>] [--output-root <path>] [--dataset <name>]"
       << " [--max-intervals <count>]\n";
}

AppOptions parseAppArguments(const vector<string>& arguments,
                             const char* programName) {
  for (const string& argument : arguments) {
    if (isHelpArgument(argument)) {
      printUsage(programName);
      std::exit(0);
    }
  }
  return {parseMaxIntervalsArgument(arguments)};
}

struct RunForDataset {
  RunForDataset(const AppOptions& options, ResultsWriter* writer)
      : intervals(selectIntervals(defaultQuadratureIntervals(),
                                  options.maxIntervals)),
        writer(writer) {}

  vector<double> intervals;
  ResultsWriter* writer = nullptr;
  string datasetGroup = "all";

  void operator()(const string& datasetName, const Dataset& dataset) {
    writer->writeDataset(
        makeDatasetRow(*writer, datasetName, dataset, datasetGroup));
    for (double intervalSeconds : intervals) {
      const size_t samplesPerWindow = dataset.stepsForInterval(intervalSeconds);
      const size_t quadratureNodes = max<size_t>(
          3, static_cast<size_t>(floor(sqrt(static_cast<double>(
                                   samplesPerWindow)))));
      const auto windows = dataset.completeWindows(samplesPerWindow);
      const auto params = createPreintegrationParams();

      const InitialCovarianceOptions initialCovariance{initialNavCovariance(),
                                                       initialBiasCovariance()};
      writeWindowRows(
          writer, datasetName, "quadrature", kConfigLabel, intervalSeconds,
          samplesPerWindow, quadratureNodes,
          collectWindowEvaluations<PIMQuadrature>(windows, params,
                                                  initialCovariance,
                                                  quadratureNodes));
      writeWindowRows(
          writer, datasetName, "manifold", kConfigLabel, intervalSeconds,
          samplesPerWindow, 0,
          collectWindowEvaluations<PIMManifold>(windows, params,
                                                initialCovariance));
      writeWindowRows(
          writer, datasetName, "tangent", kConfigLabel, intervalSeconds,
          samplesPerWindow, 0,
          collectWindowEvaluations<PIMTangent>(windows, params,
                                               initialCovariance));
    }
  }
};

string datasetGroupLabel(const ResolvedDatasetCli& datasetCli) {
  return datasetCli.options.datasetName ? *datasetCli.options.datasetName
                                        : "all";
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto datasetCli = resolveDatasetAppCli(argc, argv);
  const AppOptions appOptions =
      parseAppArguments(datasetCli.remainingArgs, argv[0]);

  ResultsWriter writer(argv[0], datasetCli.options.outputRoot);
  writeCanonicalRunMetadata(&writer, argc, argv);
  const string datasetGroup = datasetGroupLabel(datasetCli);

  RunForDataset runner(appOptions, &writer);
  runner.datasetGroup = datasetGroup;
  const int status = runForDatasets(datasetCli, runner);
  if (status != 0) {
    return status;
  }

  cout << "Results written to " << writer.runDirectory() << "\n";
  return 0;
}
