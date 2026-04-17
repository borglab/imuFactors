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

#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "PIMs.h"
#include "ResultsWriter.h"
#include "Window.h"

using namespace gtsam;
using namespace std;

using PIMQuadrature = PreintegratedImuMeasurementsQ;
using PIMTangent = PreintegratedImuMeasurementsT<TangentPreintegration>;
using PIMManifold = PreintegratedImuMeasurementsT<ManifoldPreintegration>;

namespace {

constexpr double kAlpha = 8.4;
constexpr double kSigmaGyro = kAlpha * 1.6968e-4;
constexpr double kSigmaAcc = kAlpha * 2.0000e-3;
constexpr const char* kConfigLabel = "default";

struct AppOptions {
  size_t maxIntervals = 0;
};

struct WindowEvaluationRecord {
  size_t windowIndex = 0;
  size_t startSample = 0;
  size_t endSample = 0;
  double startTime = 0.0;
  double endTime = 0.0;
  WindowResult metrics;
};

shared_ptr<PreintegrationParams> createPreintegrationParams() {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = I_3x3 * kSigmaAcc * kSigmaAcc;
  params->gyroscopeCovariance = I_3x3 * kSigmaGyro * kSigmaGyro;
  params->integrationCovariance = I_3x3 * 1e-8;
  return params;
}

template <class PIMType>
vector<WindowEvaluationRecord> collectWindowEvaluations(
    const vector<Window>& windows,
    const shared_ptr<PreintegrationParams>& params,
    size_t quadratureOrder = 0) {
  vector<WindowEvaluationRecord> evaluations;
  evaluations.reserve(windows.size());
  for (size_t windowIndex = 0; windowIndex < windows.size(); ++windowIndex) {
    const auto& window = windows[windowIndex];
    const auto result =
        evaluateWindow<PIMType>(window, params, std::nullopt, quadratureOrder);
    if (!result) {
      continue;
    }
    evaluations.push_back({windowIndex, window.start, window.end,
                           window.initialTruth().timestamp,
                           window.terminalTruth().timestamp, *result});
  }
  return evaluations;
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

WindowSummaryRow makeSummaryRow(const ResultsWriter& writer,
                                const string& datasetName, const string& method,
                                double intervalSeconds, size_t samplesPerWindow,
                                size_t quadratureNodes,
                                const WindowResultSummary& summary) {
  WindowSummaryRow row;
  row.runId = writer.runId();
  row.appName = writer.appName();
  row.dataset = datasetName;
  row.method = method;
  row.configLabel = kConfigLabel;
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

void writeWindowEvaluations(const ResultsWriter& writer, ResultsWriter* sink,
                            const string& datasetName, const string& method,
                            double intervalSeconds, size_t samplesPerWindow,
                            size_t quadratureNodes,
                            const vector<WindowEvaluationRecord>& evaluations) {
  vector<WindowResult> results;
  results.reserve(evaluations.size());
  for (const auto& evaluation : evaluations) {
    WindowMetricRow row;
    row.runId = writer.runId();
    row.appName = writer.appName();
    row.dataset = datasetName;
    row.method = method;
    row.configLabel = kConfigLabel;
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
    sink->writeWindowMetric(row);
    results.push_back(evaluation.metrics);
  }
  sink->writeWindowSummary(makeSummaryRow(
      writer, datasetName, method, intervalSeconds, samplesPerWindow,
      quadratureNodes, summarizeWindowResults(results)));
}

struct RunForDataset {
  RunForDataset(const AppOptions& options, ResultsWriter* writer)
      : intervals(selectIntervals(defaultQuadratureIntervals(),
                                  options.maxIntervals)),
        writer(writer) {}

  vector<double> intervals;
  ResultsWriter* writer = nullptr;

  void operator()(const string& datasetName, const Dataset& dataset) {
    for (double intervalSeconds : intervals) {
      const size_t samplesPerWindow = dataset.stepsForInterval(intervalSeconds);
      const size_t quadratureNodes = max<size_t>(
          3, static_cast<size_t>(
                 floor(sqrt(static_cast<double>(samplesPerWindow)))));
      const auto windows = dataset.completeWindows(samplesPerWindow);
      const auto params = createPreintegrationParams();

      writeWindowEvaluations(*writer, writer, datasetName, "quadrature",
                             intervalSeconds, samplesPerWindow, quadratureNodes,
                             collectWindowEvaluations<PIMQuadrature>(
                                 windows, params, quadratureNodes));
      writeWindowEvaluations(
          *writer, writer, datasetName, "manifold", intervalSeconds,
          samplesPerWindow, 0,
          collectWindowEvaluations<PIMManifold>(windows, params));
      writeWindowEvaluations(
          *writer, writer, datasetName, "tangent", intervalSeconds,
          samplesPerWindow, 0,
          collectWindowEvaluations<PIMTangent>(windows, params));
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
  writer.writeRunMetadata(
      {writer.runId(), writer.appName(), writer.timestampUtc(),
       joinCommandLineArguments(argc, argv), writer.outputRoot().string(), ""});
  const string datasetGroup = datasetGroupLabel(datasetCli);
  for (const auto& [datasetName, datasetPath] : datasetCli.datasets) {
    writer.writeDataset({writer.runId(), writer.appName(), datasetName,
                         datasetPath, datasetGroup});
  }

  RunForDataset runner(appOptions, &writer);
  const int status = runForDatasets(datasetCli, runner);
  if (status != 0) {
    return status;
  }

  cout << "Results written to " << writer.runDirectory() << "\n";
  return 0;
}
