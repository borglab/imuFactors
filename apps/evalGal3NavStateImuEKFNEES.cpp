/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalGal3NavStateImuEKFNEES.cpp
 * @brief  Canonical EKF export for Gal3 and NavState IMU EKF variants
 */

#include "AppUtils.h"
#include "EKFNEESEvaluator.h"
#include "QuadratureRunner.h"
#include "ResultsAdapters.h"

using namespace gtsam;
using namespace std;

namespace {

constexpr double kAlpha = 3.0;
constexpr const char* kConfigLabel = "default";

struct AppOptions {};

void printUsage(const char* programName) { printDatasetAppUsage(programName); }

AppOptions parseAppArguments(const vector<string>& arguments,
                             const char* programName) {
  for (const string& argument : arguments) {
    if (isHelpArgument(argument)) {
      printUsage(programName);
      std::exit(0);
    }
    throw runtime_error("Unknown argument: " + argument);
  }
  return {};
}

struct RunForDataset {
  explicit RunForDataset(ResultsWriter* writer) : writer(writer) {}

  vector<double> intervals = defaultQuadratureIntervals();
  ResultsWriter* writer = nullptr;
  string datasetGroup = "all";

  void operator()(const string& datasetName, const Dataset& dataset) {
    writer->writeDataset(
        makeDatasetRow(*writer, datasetName, dataset, datasetGroup));
    EKFNEESEvaluator evaluator(dataset);
    for (const double intervalSeconds : intervals) {
      writeEkfArtifacts(
          writer, datasetName, "gal3_imu_ekf", kConfigLabel,
          evaluator.computeGal3ImuEKFArtifacts(intervalSeconds, kAlpha));
      writeEkfArtifacts(
          writer, datasetName, "navstate_imu_ekf", kConfigLabel,
          evaluator.computeNavStateImuEKFArtifacts(intervalSeconds, kAlpha));
    }
  }
};

}  // namespace

int main(int argc, char* argv[]) {
  const auto datasetCli = resolveDatasetAppCli(argc, argv);
  const AppOptions appOptions =
      parseAppArguments(datasetCli.remainingArgs, argv[0]);
  (void)appOptions;

  return runDatasetApp(
      datasetCli, argc, argv,
      [&](ResultsWriter* writer, const std::string& datasetGroup) {
        RunForDataset runner(writer);
        runner.datasetGroup = datasetGroup;
        return runner;
      });
}
