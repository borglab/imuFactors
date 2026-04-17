/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "EKFNEESEvaluator.h"
#include "PIMs.h"
#include "ResultsWriter.h"
#include "TrajectoryValidator.h"
#include "Window.h"

namespace gtsam {

/**
 * @brief Shared window record with timing/sample bounds plus metrics.
 */
struct WindowEvaluationRecord {
  size_t windowIndex = 0;
  size_t startSample = 0;
  size_t endSample = 0;
  double startTime = 0.0;
  double endTime = 0.0;
  WindowResult metrics;
};

/**
 * @brief Build a dataset row for one processed dataset.
 */
inline DatasetRow makeDatasetRow(const ResultsWriter& writer,
                                 const std::string& datasetName,
                                 const std::string& datasetPath,
                                 const std::string& datasetGroup) {
  return {writer.runId(), writer.appName(), datasetName, datasetPath,
          datasetGroup};
}

/**
 * @brief Build a dataset row directly from a loaded dataset.
 */
inline DatasetRow makeDatasetRow(const ResultsWriter& writer,
                                 const std::string& datasetName,
                                 const Dataset& dataset,
                                 const std::string& datasetGroup) {
  return makeDatasetRow(writer, datasetName, dataset.sourcePath(), datasetGroup);
}

/**
 * @brief Write one canonical run metadata row.
 */
inline void writeCanonicalRunMetadata(ResultsWriter* writer, int argc,
                                      char* argv[]) {
  writer->writeRunMetadata(
      {writer->runId(), writer->appName(), writer->timestampUtc(),
       joinCommandLineArguments(argc, argv), writer->outputRoot().string(), ""});
}

/**
 * @brief Evaluate a set of windows for one PIM type.
 */
template <class PIMType>
inline std::vector<WindowEvaluationRecord> collectWindowEvaluations(
    const std::vector<Window>& windows,
    const std::shared_ptr<PreintegrationParams>& params,
    std::optional<InitialCovarianceOptions> initialCovariance = std::nullopt,
    size_t quadratureOrder = 0) {
  std::vector<WindowEvaluationRecord> evaluations;
  evaluations.reserve(windows.size());
  for (size_t windowIndex = 0; windowIndex < windows.size(); ++windowIndex) {
    const auto& window = windows[windowIndex];
    const auto result = evaluateWindow<PIMType>(window, params, initialCovariance,
                                                quadratureOrder);
    if (!result) {
      continue;
    }
    evaluations.push_back({windowIndex, window.start, window.end,
                           window.initialTruth().timestamp,
                           window.terminalTruth().timestamp, *result});
  }
  return evaluations;
}

/**
 * @brief Build a canonical window summary row.
 */
inline WindowSummaryRow makeWindowSummaryRow(
    const ResultsWriter& writer, const std::string& datasetName,
    const std::string& method, const std::string& configLabel,
    double intervalSeconds, size_t samplesPerWindow, size_t quadratureNodes,
    const WindowResultSummary& summary) {
  WindowSummaryRow row;
  row.runId = writer.runId();
  row.appName = writer.appName();
  row.dataset = datasetName;
  row.method = method;
  row.configLabel = configLabel;
  row.intervalSeconds = intervalSeconds;
  row.samplesPerWindow = samplesPerWindow;
  row.quadratureNodes = quadratureNodes;
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

/**
 * @brief Write canonical window metrics and the matching summary row.
 */
inline void writeWindowRows(ResultsWriter* writer,
                            const std::string& datasetName,
                            const std::string& method,
                            const std::string& configLabel,
                            double intervalSeconds, size_t samplesPerWindow,
                            size_t quadratureNodes,
                            const std::vector<WindowEvaluationRecord>& evaluations) {
  std::vector<WindowResult> results;
  results.reserve(evaluations.size());
  for (const auto& evaluation : evaluations) {
    WindowMetricRow row;
    row.runId = writer->runId();
    row.appName = writer->appName();
    row.dataset = datasetName;
    row.method = method;
    row.configLabel = configLabel;
    row.intervalSeconds = intervalSeconds;
    row.samplesPerWindow = samplesPerWindow;
    row.quadratureNodes = quadratureNodes;
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

  writer->writeWindowSummary(makeWindowSummaryRow(
      *writer, datasetName, method, configLabel, intervalSeconds,
      samplesPerWindow, quadratureNodes, summarizeWindowResults(results)));
}

/**
 * @brief Build one canonical trajectory sample row from EKF artifacts.
 */
inline TrajectorySampleRow makeTrajectorySampleRow(
    const ResultsWriter& writer, const std::string& datasetName,
    const std::string& method, const std::string& configLabel,
    const EKFNEESEvaluator::RunArtifacts& artifacts,
    const EKFNEESEvaluator::TrajectorySample& sample) {
  TrajectorySampleRow row;
  row.runId = writer.runId();
  row.appName = writer.appName();
  row.dataset = datasetName;
  row.method = method;
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

/**
 * @brief Write canonical window and trajectory exports for one EKF artifact
 * bundle.
 */
inline void writeEkfArtifacts(ResultsWriter* writer,
                              const std::string& datasetName,
                              const std::string& method,
                              const std::string& configLabel,
                              const EKFNEESEvaluator::RunArtifacts& artifacts) {
  std::vector<WindowEvaluationRecord> evaluations;
  evaluations.reserve(artifacts.windowEvaluations.size());
  for (const auto& evaluation : artifacts.windowEvaluations) {
    evaluations.push_back({evaluation.windowIndex, evaluation.startSample,
                           evaluation.endSample, evaluation.startTime,
                           evaluation.endTime, evaluation.metrics});
  }

  writeWindowRows(writer, datasetName, method, configLabel,
                  artifacts.preintegrationTime, artifacts.samplesPerWindow, 0,
                  evaluations);

  for (const auto& sample : artifacts.trajectorySamples) {
    writer->writeTrajectorySample(makeTrajectorySampleRow(
        *writer, datasetName, method, configLabel, artifacts, sample));
  }
}

}  // namespace gtsam
