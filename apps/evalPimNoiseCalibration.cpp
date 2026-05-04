/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalPimNoiseCalibration.cpp
 * @brief  Coarse PIM noise calibration over separate gyro and accel scales.
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "AppUtils.h"
#include "QuadratureRunner.h"
#include "ResultsAdapters.h"

using namespace gtsam;
using namespace std;

namespace {

constexpr double kDefaultIntervalSeconds = 0.2;

struct AppOptions {
  AppCliOptions datasetOptions;
  double intervalSeconds = kDefaultIntervalSeconds;
};

struct MethodDatasetNees {
  string dataset;
  string method;
  double nees = 0.0;
};

struct TrialEvaluation {
  double alphaGyro = 0.0;
  double alphaAcc = 0.0;
  vector<MethodDatasetNees> values;
};

struct ScoreSummary {
  double meanNees = 0.0;
  double sumDeviations = 0.0;
};

struct StudyBestWorst {
  TrialEvaluation best;
  ScoreSummary bestScore;
  TrialEvaluation worst;
  ScoreSummary worstScore;
};

void printUsage(const char* programName) {
  printDatasetAppUsage(programName);
  cout << "  --interval <seconds>    PIM window interval (default: "
       << kDefaultIntervalSeconds << ")\n";
}

AppOptions parseArguments(int argc, char* argv[]) {
  vector<string> arguments;
  arguments.reserve(max(0, argc - 1));
  for (int index = 1; index < argc; ++index) {
    arguments.push_back(argv[index]);
  }

  const ParsedAppCliOptions parsed = parseDatasetAppCliOptions(arguments);
  if (parsed.options.datasetName) {
    throw runtime_error(
        "PIM calibration tunes dataset classes only; --dataset is not "
        "supported.");
  }

  AppOptions options;
  options.datasetOptions = parsed.options;
  for (size_t index = 0; index < parsed.remainingArgs.size(); ++index) {
    const string& argument = parsed.remainingArgs[index];
    if (isHelpArgument(argument)) {
      printUsage(argv[0]);
      exit(0);
    }
    if (argument == "--interval") {
      if (index + 1 >= parsed.remainingArgs.size()) {
        throw runtime_error("Missing value for --interval");
      }
      options.intervalSeconds =
          parsePositiveDoubleOption("--interval", parsed.remainingArgs[++index]);
      continue;
    }
    throw runtime_error("Unknown argument: " + argument);
  }
  return options;
}

vector<double> coarseAlphaRange() {
  return {0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 13.0};
}

bool isMachineHall(const string& datasetName) {
  return DatasetFilters::machineHall(datasetName);
}

bool isViconRoom(const string& datasetName) {
  return DatasetFilters::viconRoom(datasetName);
}

bool datasetInGroup(const string& datasetName, const string& datasetGroup) {
  if (datasetGroup == "all") {
    return true;
  }
  if (datasetGroup == "machine_hall") {
    return isMachineHall(datasetName);
  }
  if (datasetGroup == "vicon") {
    return isViconRoom(datasetName);
  }
  throw runtime_error("Unknown dataset group: " + datasetGroup);
}

template <class PIMType>
double medianNeesForMethod(const Dataset& dataset,
                           const shared_ptr<PreintegrationParams>& params,
                           double intervalSeconds, size_t quadratureOrder = 0) {
  const size_t samplesPerWindow = dataset.stepsForInterval(intervalSeconds);
  const auto windows = dataset.completeWindows(samplesPerWindow);
  const auto evaluations = collectWindowEvaluations<PIMType>(
      windows, params, nullopt, quadratureOrder);

  vector<double> values;
  values.reserve(evaluations.size());
  for (const auto& evaluation : evaluations) {
    values.push_back(evaluation.metrics.normalizedNees);
  }
  if (values.empty()) {
    return numeric_limits<double>::quiet_NaN();
  }
  return computeMedian(values);
}

TrialEvaluation evaluateTrial(
    const vector<pair<string, Dataset>>& datasets, double alphaGyro,
    double alphaAcc, double intervalSeconds) {
  TrialEvaluation trial;
  trial.alphaGyro = alphaGyro;
  trial.alphaAcc = alphaAcc;

  const auto params = makePreintegrationParams(alphaGyro * 1.6968e-4,
                                               alphaAcc * 2.0000e-3);
  for (const auto& [datasetName, dataset] : datasets) {
    const size_t samplesPerWindow = dataset.stepsForInterval(intervalSeconds);
    const size_t quadratureNodes = max<size_t>(
        3, static_cast<size_t>(
               floor(sqrt(static_cast<double>(samplesPerWindow)))));

    trial.values.push_back(
        {datasetName, "quadrature",
         medianNeesForMethod<PIMQuadrature>(dataset, params, intervalSeconds,
                                            quadratureNodes)});
    trial.values.push_back(
        {datasetName, "tangent",
         medianNeesForMethod<PIMTangent>(dataset, params, intervalSeconds)});
  }
  return trial;
}

ScoreSummary scoreTrial(const TrialEvaluation& trial,
                        const string& datasetGroup, const string& method) {
  vector<double> values;
  for (const auto& value : trial.values) {
    if (!datasetInGroup(value.dataset, datasetGroup)) {
      continue;
    }
    if (method != "combined" && value.method != method) {
      continue;
    }
    if (isfinite(value.nees)) {
      values.push_back(value.nees);
    }
  }
  if (values.empty()) {
    throw runtime_error("No NEES values for study " + datasetGroup + "/" +
                        method);
  }

  ScoreSummary score;
  score.meanNees = accumulate(values.begin(), values.end(), 0.0) /
                   static_cast<double>(values.size());
  for (const double value : values) {
    score.sumDeviations += abs(value - 1.0);
  }
  return score;
}

StudyBestWorst findBestWorst(const vector<TrialEvaluation>& trials,
                             const string& datasetGroup,
                             const string& method) {
  StudyBestWorst result;
  double bestDeviation = numeric_limits<double>::infinity();
  double worstDeviation = -numeric_limits<double>::infinity();
  for (const auto& trial : trials) {
    const ScoreSummary score = scoreTrial(trial, datasetGroup, method);
    if (score.sumDeviations < bestDeviation) {
      bestDeviation = score.sumDeviations;
      result.best = trial;
      result.bestScore = score;
    }
    if (score.sumDeviations > worstDeviation) {
      worstDeviation = score.sumDeviations;
      result.worst = trial;
      result.worstScore = score;
    }
  }
  return result;
}

vector<MethodDatasetNees> valuesForStudy(const TrialEvaluation& trial,
                                         const string& datasetGroup,
                                         const string& method) {
  vector<MethodDatasetNees> values;
  for (const auto& value : trial.values) {
    if (!datasetInGroup(value.dataset, datasetGroup)) {
      continue;
    }
    if (method != "combined" && value.method != method) {
      continue;
    }
    values.push_back(value);
  }
  return values;
}

void writeStudyRows(ResultsWriter* writer, const vector<TrialEvaluation>& trials,
                    const string& datasetGroup, const string& method) {
  const string studyName = datasetGroup + "_pim_" + method + "_coarse";
  for (const auto& trial : trials) {
    const ScoreSummary score = scoreTrial(trial, datasetGroup, method);
    const auto values = valuesForStudy(trial, datasetGroup, method);
    for (const auto& value : values) {
      const string datasetLabel =
          method == "combined" ? value.dataset + "/" + value.method
                               : value.dataset;
      writer->writeCalibrationTrial(
          {writer->runId(), writer->appName(), datasetLabel, studyName,
           datasetGroup, trial.alphaGyro, trial.alphaAcc, value.nees,
           score.meanNees, score.sumDeviations});
    }
  }

  const StudyBestWorst bestWorst =
      findBestWorst(trials, datasetGroup, method);
  writer->writeCalibrationSummary(
      {writer->runId(), writer->appName(), studyName, "best",
       bestWorst.best.alphaGyro, bestWorst.best.alphaAcc,
       bestWorst.bestScore.meanNees, bestWorst.bestScore.sumDeviations});
  writer->writeCalibrationSummary(
      {writer->runId(), writer->appName(), studyName, "worst",
       bestWorst.worst.alphaGyro, bestWorst.worst.alphaAcc,
       bestWorst.worstScore.meanNees, bestWorst.worstScore.sumDeviations});
}

vector<pair<string, Dataset>> loadDatasets(
    const vector<pair<string, string>>& datasetPaths) {
  vector<pair<string, Dataset>> datasets;
  datasets.reserve(datasetPaths.size());
  for (const auto& [datasetName, datasetPath] : datasetPaths) {
    datasets.emplace_back(datasetName, Dataset(datasetPath));
  }
  return datasets;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const AppOptions options = parseArguments(argc, argv);
    const auto datasetPaths =
        resolveDatasets(options.datasetOptions, DatasetFilters::all);
    if (datasetPaths.empty()) {
      cerr << "No datasets found in " << options.datasetOptions.dataDirectory
           << "\n";
      return 1;
    }

    ResultsWriter writer(argv[0], options.datasetOptions.outputRoot);
    writeCanonicalRunMetadata(&writer, argc, argv);
    for (const auto& [datasetName, datasetPath] : datasetPaths) {
      writer.writeDataset(makeDatasetRow(writer, datasetName, datasetPath,
                                         "all"));
    }

    const auto datasets = loadDatasets(datasetPaths);
    vector<TrialEvaluation> trials;
    const vector<double> alphaValues = coarseAlphaRange();
    trials.reserve(alphaValues.size() * alphaValues.size());
    for (const double alphaGyro : alphaValues) {
      for (const double alphaAcc : alphaValues) {
        cout << "Evaluating alpha_gyro=" << alphaGyro
             << ", alpha_acc=" << alphaAcc << "...\n";
        trials.push_back(evaluateTrial(datasets, alphaGyro, alphaAcc,
                                       options.intervalSeconds));
      }
    }

    const vector<string> datasetGroups = {"all", "machine_hall", "vicon"};
    const vector<string> methods = {"combined", "quadrature", "tangent"};
    for (const auto& datasetGroup : datasetGroups) {
      for (const auto& method : methods) {
        writeStudyRows(&writer, trials, datasetGroup, method);
      }
    }

    cout << "Results written to " << writer.runDirectory() << "\n";
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
