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

constexpr double kDefaultAlpha = 3.0;
constexpr const char* kConfigLabel = "default";

struct AppOptions {
  double alpha = kDefaultAlpha;
};

void printUsage(const char* programName) {
  printDatasetAppUsage(programName);
  cout << "  --alpha <value>         Noise scaling factor (default: "
       << kDefaultAlpha << ")\n";
}

AppOptions parseAppArguments(const vector<string>& arguments,
                             const char* programName) {
  AppOptions options;
  for (size_t index = 0; index < arguments.size(); ++index) {
    const string& argument = arguments[index];
    if (isHelpArgument(argument)) {
      printUsage(programName);
      std::exit(0);
    }
    if (argument == "--alpha") {
      if (index + 1 >= arguments.size()) {
        throw runtime_error("Missing value for --alpha");
      }
      options.alpha = parsePositiveDoubleOption("--alpha", arguments[++index]);
      continue;
    }
    throw runtime_error("Unknown argument: " + argument);
  }
  return options;
}

struct RunForDataset {
  RunForDataset(ResultsWriter* writer, double alpha)
      : writer(writer), alpha(alpha) {}

  vector<double> intervals = defaultQuadratureIntervals();
  ResultsWriter* writer = nullptr;
  double alpha = kDefaultAlpha;
  string datasetGroup = "all";

  void operator()(const string& datasetName, const Dataset& dataset) {
    writer->writeDataset(
        makeDatasetRow(*writer, datasetName, dataset, datasetGroup));
    EKFNEESEvaluator evaluator(dataset);
    for (const double intervalSeconds : intervals) {
      writeEkfArtifacts(
          writer, datasetName, "gal3_imu_ekf", kConfigLabel,
          evaluator.computeGal3ImuEKFArtifacts(intervalSeconds, alpha));
      writeEkfArtifacts(
          writer, datasetName, "navstate_imu_ekf", kConfigLabel,
          evaluator.computeNavStateImuEKFArtifacts(intervalSeconds, alpha));
    }
  }
};

}  // namespace

int main(int argc, char* argv[]) {
  const auto datasetCli = resolveDatasetAppCli(argc, argv);
  const AppOptions appOptions =
      parseAppArguments(datasetCli.remainingArgs, argv[0]);

  return runDatasetApp(
      datasetCli, argc, argv,
      [&](ResultsWriter* writer, const std::string& datasetGroup) {
        RunForDataset runner(writer, appOptions.alpha);
        runner.datasetGroup = datasetGroup;
        return runner;
      });
}
