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

#include <iostream>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "EKFNEESEvaluator.h"
#include "PIMs.h"
#include "ResultsWriter.h"
#include "TrajectoryValidator.h"

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

WindowSummaryRow makeSummaryRow(const ResultsWriter& writer,
                                const string& datasetName,
                                const string& method,
                                const EKFNEESEvaluator::RunArtifacts& artifacts,
                                const WindowResultSummary& summary) {
  WindowSummaryRow row;
  row.runId = writer.runId();
  row.appName = writer.appName();
  row.dataset = datasetName;
  row.method = method;
  row.configLabel = kConfigLabel;
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

TrajectorySampleRow makeTrajectoryRow(
    const ResultsWriter& writer, const string& datasetName,
    const string& method, const EKFNEESEvaluator::RunArtifacts& artifacts,
    const EKFNEESEvaluator::TrajectorySample& sample) {
  TrajectorySampleRow row;
  row.runId = writer.runId();
  row.appName = writer.appName();
  row.dataset = datasetName;
  row.method = method;
  row.configLabel = kConfigLabel;
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

void writeArtifacts(ResultsWriter* writer, const string& datasetName,
                    const string& method,
                    const EKFNEESEvaluator::RunArtifacts& artifacts) {
  vector<WindowResult> results;
  results.reserve(artifacts.windowEvaluations.size());
  for (const auto& evaluation : artifacts.windowEvaluations) {
    WindowMetricRow row;
    row.runId = writer->runId();
    row.appName = writer->appName();
    row.dataset = datasetName;
    row.method = method;
    row.configLabel = kConfigLabel;
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

  writer->writeWindowSummary(
      makeSummaryRow(*writer, datasetName, method, artifacts,
                     summarizeWindowResults(results)));

  for (const auto& sample : artifacts.trajectorySamples) {
    writer->writeTrajectorySample(
        makeTrajectoryRow(*writer, datasetName, method, artifacts, sample));
  }
}

struct RunForDataset {
  explicit RunForDataset(ResultsWriter* writer) : writer(writer) {}

  vector<double> intervals = defaultQuadratureIntervals();
  ResultsWriter* writer = nullptr;

  void operator()(const string& datasetName, const Dataset& dataset) {
    EKFNEESEvaluator evaluator(dataset);
    for (const double intervalSeconds : intervals) {
      writeArtifacts(writer, datasetName, "gal3_imu_ekf",
                     evaluator.computeGal3ImuEKFArtifacts(intervalSeconds,
                                                          kAlpha));
      writeArtifacts(writer, datasetName, "navstate_imu_ekf",
                     evaluator.computeNavStateImuEKFArtifacts(intervalSeconds,
                                                              kAlpha));
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
  (void)appOptions;

  ResultsWriter writer(argv[0], datasetCli.options.outputRoot);
  writer.writeRunMetadata(
      {writer.runId(), writer.appName(), writer.timestampUtc(),
       joinCommandLineArguments(argc, argv), writer.outputRoot().string(), ""});
  const string datasetGroup = datasetGroupLabel(datasetCli);
  for (const auto& [datasetName, datasetPath] : datasetCli.datasets) {
    writer.writeDataset({writer.runId(), writer.appName(), datasetName,
                         datasetPath, datasetGroup});
  }

  RunForDataset runner(&writer);
  const int status = runForDatasets(datasetCli, runner);
  if (status != 0) {
    return status;
  }

  cout << "Results written to " << writer.runDirectory() << "\n";
  return 0;
}
