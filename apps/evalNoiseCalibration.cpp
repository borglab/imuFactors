/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalNoiseCalibration.cpp
 * @brief  Unified canonical export for noise calibration studies
 */

#include "NoiseCalibration.h"
#include "ResultsAdapters.h"
#include "ResultsWriter.h"

using namespace gtsam;
using namespace std;

namespace {

struct AppOptions {
  string datasetType = "all";
  string searchMode = "both";
  string outputRoot = "./results";
};

void printUsage(const char* programName) {
  cout << "Usage: " << programName
       << " [--output-root <path>] [dataset_type] [search_mode]\n";
  cout << "  dataset_type: machine_hall | vicon | all\n";
  cout << "  search_mode: coarse | fine | both\n";
}

AppOptions parseAppArguments(int argc, char* argv[]) {
  vector<string> arguments;
  arguments.reserve(max(0, argc - 1));
  for (int index = 1; index < argc; ++index) {
    arguments.push_back(argv[index]);
  }

  const ParsedOutputRootCli outputRootCli =
      parseOutputRootCliArguments(arguments);
  AppOptions options;
  options.outputRoot = outputRootCli.outputRoot;

  if (!outputRootCli.remainingArgs.empty()) {
    const string& firstArgument = outputRootCli.remainingArgs[0];
    if (firstArgument == "help" || firstArgument == "-h" ||
        firstArgument == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    }
    options.datasetType = firstArgument;
  }
  if (outputRootCli.remainingArgs.size() >= 2) {
    options.searchMode = outputRootCli.remainingArgs[1];
  }
  if (outputRootCli.remainingArgs.size() > 2) {
    throw runtime_error("Too many arguments");
  }
  if (options.searchMode != "coarse" && options.searchMode != "fine" &&
      options.searchMode != "both") {
    throw runtime_error("Invalid search mode: " + options.searchMode);
  }
  return options;
}

NoiseCalibrationStudy runFineSearchAround(
    const vector<pair<string, string>>& datasets,
    const NoiseTrialResult& coarseBest) {
  NoiseCalibrationStudy study;
  for (const auto& [name, path] : datasets) {
    (void)path;
    study.datasetNames.push_back(name);
  }

  const double gyroMin = max(0.1, coarseBest.alphaGyro * 0.7);
  const double gyroMax = coarseBest.alphaGyro * 1.3;
  const double accMin = max(0.1, coarseBest.alphaAcc * 0.7);
  const double accMax = coarseBest.alphaAcc * 1.3;

  double bestSumDev = 1e9;
  double worstSumDev = 0.0;

  for (double alphaGyro = gyroMin; alphaGyro <= gyroMax; alphaGyro += 0.2) {
    for (double alphaAcc = accMin; alphaAcc <= accMax; alphaAcc += 0.2) {
      NoiseTrialResult trial;
      trial.alphaGyro = alphaGyro;
      trial.alphaAcc = alphaAcc;

      for (const auto& [name, path] : datasets) {
        Dataset dataset(path);
        EKFNEESEvaluator evaluator(dataset);

        auto params = dataset.configureImuParams(alphaGyro, alphaAcc);
        auto result = evaluator.runGal3ImuEKF(0.2, params, name);
        trial.datasetNees[name] = result.median;
      }

      computeMetrics(trial);
      study.trials.push_back(trial);

      if (trial.sumDeviations < bestSumDev) {
        bestSumDev = trial.sumDeviations;
        study.bestResult = trial;
      }

      if (trial.sumDeviations > worstSumDev) {
        worstSumDev = trial.sumDeviations;
        study.worstResult = trial;
      }
    }
  }

  return study;
}

void writeStudy(ResultsWriter* writer, const string& datasetGroup,
                const string& studyName, const NoiseCalibrationStudy& study) {
  for (const auto& trial : study.trials) {
    for (const auto& datasetName : study.datasetNames) {
      writer->writeCalibrationTrial({writer->runId(), writer->appName(),
                                     datasetName, studyName, datasetGroup,
                                     trial.alphaGyro, trial.alphaAcc,
                                     trial.datasetNees.at(datasetName),
                                     trial.meanNees, trial.sumDeviations});
    }
  }

  writer->writeCalibrationSummary(
      {writer->runId(), writer->appName(), studyName, "best",
       study.bestResult.alphaGyro, study.bestResult.alphaAcc,
       study.bestResult.meanNees, study.bestResult.sumDeviations});
  writer->writeCalibrationSummary(
      {writer->runId(), writer->appName(), studyName, "worst",
       study.worstResult.alphaGyro, study.worstResult.alphaAcc,
       study.worstResult.meanNees, study.worstResult.sumDeviations});
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const AppOptions options = parseAppArguments(argc, argv);

    DatasetFilter filter = DatasetFilters::all;
    string studyName = "all";
    if (options.datasetType == "machine_hall" || options.datasetType == "mh") {
      filter = DatasetFilters::machineHall;
      studyName = "machine_hall";
    } else if (options.datasetType == "vicon" || options.datasetType == "v") {
      filter = DatasetFilters::viconRoom;
      studyName = "vicon";
    } else if (options.datasetType != "all") {
      throw runtime_error("Invalid dataset type: " + options.datasetType);
    }

    const auto datasets = discoverFilteredDatasets("../data/euroc/", filter);
    if (datasets.empty()) {
      cerr << "No datasets found in ../data/euroc/\n";
      return 1;
    }

    ResultsWriter writer(argv[0], options.outputRoot);
    writeCanonicalRunMetadata(&writer, argc, argv);
    for (const auto& [datasetName, datasetPath] : datasets) {
      writer.writeDataset(makeDatasetRow(writer, datasetName, datasetPath,
                                         options.datasetType));
    }

    NoiseCalibrationStudy coarseStudy;
    NoiseCalibrationStudy fineStudy;
    bool hasCoarse = false;
    bool hasFine = false;

    if (options.searchMode == "coarse" || options.searchMode == "both") {
      coarseStudy = runFilteredCalibration(datasets, studyName);
      writeStudy(&writer, options.datasetType, studyName + "_coarse",
                 coarseStudy);
      hasCoarse = true;
    }

    if (options.searchMode == "fine" || options.searchMode == "both") {
      if (!hasCoarse) {
        throw runtime_error("Fine search requires a preceding coarse search");
      }
      fineStudy = runFineSearchAround(datasets, coarseStudy.bestResult);
      writeStudy(&writer, options.datasetType, studyName + "_fine", fineStudy);
      hasFine = true;
    }

    if (!hasCoarse && !hasFine) {
      throw runtime_error("No calibration phase selected");
    }

    cout << "Results written to " << writer.runDirectory() << "\n";
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
