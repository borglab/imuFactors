/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalExportTrajectories.cpp
 * @brief  Export canonical trajectory packages for best and worst noise
 * parameters
 */

#include <iostream>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "EKFNEESEvaluator.h"
#include "PIMs.h"
#include "ResultsWriter.h"

using namespace gtsam;
using namespace std;

namespace {

constexpr double kIntervalSeconds = 0.2;

struct NoiseParameters {
  double alphaGyro = 0.0;
  double alphaAcc = 0.0;
  string label;
};

struct AppOptions {
  NoiseParameters best{13.0, 9.4, "best"};
  NoiseParameters worst{0.5, 0.5, "worst"};
  string datasetType = "all";
  string outputRoot = "./results";
};

void printUsage(const char* programName) {
  cout << "Usage: " << programName << " [options]\n";
  cout << "  --output-root <path>    Results root directory (default: "
          "./results)\n";
  cout << "  --best-gyro <value>     Gyroscope scaling for best config\n";
  cout << "  --best-acc <value>      Accelerometer scaling for best config\n";
  cout << "  --worst-gyro <value>    Gyroscope scaling for worst config\n";
  cout << "  --worst-acc <value>     Accelerometer scaling for worst config\n";
  cout << "  --dataset-type <type>   all | machine_hall | vicon\n";
  cout << "  " << programName
       << " <bestAlphaGyro> <bestAlphaAcc> <worstAlphaGyro> <worstAlphaAcc>\n";
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

  if (outputRootCli.remainingArgs.size() == 4 &&
      outputRootCli.remainingArgs[0].rfind("--", 0) != 0) {
    options.best.alphaGyro = stod(outputRootCli.remainingArgs[0]);
    options.best.alphaAcc = stod(outputRootCli.remainingArgs[1]);
    options.worst.alphaGyro = stod(outputRootCli.remainingArgs[2]);
    options.worst.alphaAcc = stod(outputRootCli.remainingArgs[3]);
    return options;
  }

  for (size_t index = 0; index < outputRootCli.remainingArgs.size(); ++index) {
    const string& argument = outputRootCli.remainingArgs[index];
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (argument == "--best-gyro") {
      options.best.alphaGyro = stod(outputRootCli.remainingArgs.at(++index));
      continue;
    }
    if (argument == "--best-acc") {
      options.best.alphaAcc = stod(outputRootCli.remainingArgs.at(++index));
      continue;
    }
    if (argument == "--worst-gyro") {
      options.worst.alphaGyro = stod(outputRootCli.remainingArgs.at(++index));
      continue;
    }
    if (argument == "--worst-acc") {
      options.worst.alphaAcc = stod(outputRootCli.remainingArgs.at(++index));
      continue;
    }
    if (argument == "--dataset-type") {
      options.datasetType = outputRootCli.remainingArgs.at(++index);
      continue;
    }
    throw runtime_error("Unknown argument: " + argument);
  }

  return options;
}

DatasetFilter selectFilter(const string& datasetType) {
  if (datasetType == "machine_hall" || datasetType == "mh") {
    return DatasetFilters::machineHall;
  }
  if (datasetType == "vicon" || datasetType == "v") {
    return DatasetFilters::viconRoom;
  }
  return DatasetFilters::all;
}

TrajectorySampleRow makeTrajectoryRow(
    const ResultsWriter& writer, const string& datasetName,
    const string& configLabel, const EKFNEESEvaluator::RunArtifacts& artifacts,
    const EKFNEESEvaluator::TrajectorySample& sample) {
  TrajectorySampleRow row;
  row.runId = writer.runId();
  row.appName = writer.appName();
  row.dataset = datasetName;
  row.method = "gal3_imu_ekf";
  row.configLabel = configLabel;
  row.intervalSeconds = artifacts.preintegrationTime;
  row.samplesPerWindow = artifacts.samplesPerWindow;
  row.timestamp = sample.groundTruth.timestamp;
  row.gtX = sample.groundTruth.position.x();
  row.gtY = sample.groundTruth.position.y();
  row.gtZ = sample.groundTruth.position.z();
  row.gtVx = sample.groundTruth.velocity.x();
  row.gtVy = sample.groundTruth.velocity.y();
  row.gtVz = sample.groundTruth.velocity.z();
  row.gtRoll = sample.groundTruth.rpy.x();
  row.gtPitch = sample.groundTruth.rpy.y();
  row.gtYaw = sample.groundTruth.rpy.z();
  row.predX = sample.predicted.position.x();
  row.predY = sample.predicted.position.y();
  row.predZ = sample.predicted.position.z();
  row.predVx = sample.predicted.velocity.x();
  row.predVy = sample.predicted.velocity.y();
  row.predVz = sample.predicted.velocity.z();
  row.predRoll = sample.predicted.rpy.x();
  row.predPitch = sample.predicted.rpy.y();
  row.predYaw = sample.predicted.rpy.z();
  row.errRotX = sample.error(0);
  row.errRotY = sample.error(1);
  row.errRotZ = sample.error(2);
  row.errPosX = sample.error(3);
  row.errPosY = sample.error(4);
  row.errPosZ = sample.error(5);
  row.errVelX = sample.error(6);
  row.errVelY = sample.error(7);
  row.errVelZ = sample.error(8);
  row.rotPredSigma = covarianceBlockSigma(sample.predicted.covariance, 0);
  row.posPredSigma = covarianceBlockSigma(sample.predicted.covariance, 3);
  row.velPredSigma = covarianceBlockSigma(sample.predicted.covariance, 6);
  row.covariance = sample.predicted.covariance;
  return row;
}

WindowSummaryRow makeSummaryRow(const ResultsWriter& writer,
                                const string& datasetName,
                                const string& configLabel,
                                const EKFNEESEvaluator::RunArtifacts& artifacts,
                                const WindowResultSummary& summary) {
  WindowSummaryRow row;
  row.runId = writer.runId();
  row.appName = writer.appName();
  row.dataset = datasetName;
  row.method = "gal3_imu_ekf";
  row.configLabel = configLabel;
  row.intervalSeconds = artifacts.preintegrationTime;
  row.samplesPerWindow = artifacts.samplesPerWindow;
  row.quadratureNodes = 0;
  row.sampleCount = summary.sampleCount;
  row.normalizedNeesMean = summary.normalizedNeesMean;
  row.normalizedNeesMedian = summary.normalizedNeesMedian;
  row.normalizedNeesP95 = summary.normalizedNeesP95;
  row.normalizedNeesVariance = summary.normalizedNeesVariance;
  row.rotErrorMedian = summary.rotErrorMedian;
  row.rotPredSigmaMedian = summary.rotPredSigmaMedian;
  row.posErrorMedian = summary.posErrorMedian;
  row.posPredSigmaMedian = summary.posPredSigmaMedian;
  row.velErrorMedian = summary.velErrorMedian;
  row.velPredSigmaMedian = summary.velPredSigmaMedian;
  return row;
}

void writeArtifacts(ResultsWriter* writer, const string& datasetName,
                    const string& configLabel,
                    const EKFNEESEvaluator::RunArtifacts& artifacts) {
  vector<WindowResult> results;
  results.reserve(artifacts.windowEvaluations.size());
  for (const auto& evaluation : artifacts.windowEvaluations) {
    WindowMetricRow row;
    row.runId = writer->runId();
    row.appName = writer->appName();
    row.dataset = datasetName;
    row.method = "gal3_imu_ekf";
    row.configLabel = configLabel;
    row.intervalSeconds = artifacts.preintegrationTime;
    row.samplesPerWindow = artifacts.samplesPerWindow;
    row.quadratureNodes = 0;
    row.windowIndex = evaluation.windowIndex;
    row.windowStartSample = evaluation.startSample;
    row.windowEndSample = evaluation.endSample;
    row.windowStartTime = evaluation.startTime;
    row.windowEndTime = evaluation.endTime;
    row.normalizedNees = evaluation.metrics.normalizedNees;
    row.rotErrorNorm = evaluation.metrics.rotErrorNorm;
    row.rotPredSigma = evaluation.metrics.rotPredSigma;
    row.posErrorNorm = evaluation.metrics.posErrorNorm;
    row.posPredSigma = evaluation.metrics.posPredSigma;
    row.velErrorNorm = evaluation.metrics.velErrorNorm;
    row.velPredSigma = evaluation.metrics.velPredSigma;
    writer->writeWindowMetric(row);
    results.push_back(evaluation.metrics);
  }
  writer->writeWindowSummary(makeSummaryRow(*writer, datasetName, configLabel,
                                            artifacts,
                                            summarizeWindowResults(results)));

  for (const auto& sample : artifacts.trajectorySamples) {
    writer->writeTrajectorySample(makeTrajectoryRow(
        *writer, datasetName, configLabel, artifacts, sample));
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const AppOptions options = parseAppArguments(argc, argv);
    const DatasetFilter filter = selectFilter(options.datasetType);
    const auto datasets = discoverFilteredDatasets("../data/euroc/", filter);
    if (datasets.empty()) {
      cerr << "No datasets found in ../data/euroc/\n";
      return 1;
    }

    ResultsWriter writer(argv[0], options.outputRoot);
    writer.writeRunMetadata({writer.runId(), writer.appName(),
                             writer.timestampUtc(),
                             joinCommandLineArguments(argc, argv),
                             writer.outputRoot().string(), ""});
    for (const auto& [datasetName, datasetPath] : datasets) {
      writer.writeDataset({writer.runId(), writer.appName(), datasetName,
                           datasetPath, options.datasetType});
    }

    for (const auto& [datasetName, datasetPath] : datasets) {
      Dataset dataset(datasetPath);
      EKFNEESEvaluator evaluator(dataset);

      writeArtifacts(&writer, datasetName, options.best.label,
                     evaluator.computeGal3ImuEKFArtifacts(
                         kIntervalSeconds,
                         dataset.configureImuParams(options.best.alphaGyro,
                                                    options.best.alphaAcc)));
      writeArtifacts(&writer, datasetName, options.worst.label,
                     evaluator.computeGal3ImuEKFArtifacts(
                         kIntervalSeconds,
                         dataset.configureImuParams(options.worst.alphaGyro,
                                                    options.worst.alphaAcc)));
    }

    cout << "Results written to " << writer.runDirectory() << "\n";
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
