/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "ResultsWriter.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace gtsam {

namespace {

std::string currentTimestampUtc() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
  const auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch()) %
      1000;

  std::tm utcTime{};
#if defined(_WIN32)
  gmtime_s(&utcTime, &nowTime);
#else
  gmtime_r(&nowTime, &utcTime);
#endif

  std::ostringstream stream;
  stream << std::put_time(&utcTime, "%Y-%m-%dT%H:%M:%S") << "."
         << std::setw(3) << std::setfill('0') << milliseconds.count() << "Z";
  return stream.str();
}

std::string createRunIdFromTimestamp(const std::string& timestampUtc) {
  std::string runId;
  runId.reserve(timestampUtc.size());
  for (const char character : timestampUtc) {
    if ((character >= '0' && character <= '9') || character == 'T' ||
        character == 'Z') {
      runId += character;
    }
  }
  return runId;
}

std::string normalizeAppName(const std::string& programName) {
  return std::filesystem::path(programName).filename().string();
}

void writeRow(std::ofstream& stream, const std::string& row) {
  stream << row << "\n";
  if (!stream.good()) {
    throw std::runtime_error("Failed to write canonical results row.");
  }
}

}  // namespace

std::string joinCommandLineArguments(int argc, char* argv[]) {
  std::ostringstream stream;
  for (int index = 0; index < argc; ++index) {
    if (index > 0) {
      stream << " ";
    }
    stream << argv[index];
  }
  return stream.str();
}

ParsedOutputRootCli parseOutputRootCliArguments(
    const std::vector<std::string>& arguments) {
  ParsedOutputRootCli parsed;
  for (size_t index = 0; index < arguments.size(); ++index) {
    const std::string& argument = arguments[index];
    if (argument == "--output-root") {
      if (index + 1 >= arguments.size()) {
        throw std::runtime_error("Missing value for --output-root");
      }
      parsed.outputRoot = arguments[++index];
      continue;
    }
    parsed.remainingArgs.push_back(argument);
  }
  return parsed;
}

ResultsWriter::ResultsWriter(const std::string& programName,
                             const std::string& outputRoot,
                             const std::string& runId)
    : appName_(normalizeAppName(programName)),
      timestampUtc_(currentTimestampUtc()),
      outputRoot_(outputRoot),
      runId_(runId.empty() ? createRunIdFromTimestamp(timestampUtc_) : runId),
      runDirectory_(outputRoot_ / appName_ / runId_) {
  std::filesystem::create_directories(runDirectory_);

  runMetadataFile_ =
      openFileWithHeader("run_metadata.csv", runMetadataHeader());
  datasetsFile_ = openFileWithHeader("datasets.csv", datasetHeader());
  windowMetricsFile_ =
      openFileWithHeader("window_metrics.csv", windowMetricHeader());
  windowSummariesFile_ =
      openFileWithHeader("window_summaries.csv", windowSummaryHeader());
  trajectorySamplesFile_ =
      openFileWithHeader("trajectory_samples.csv", trajectorySampleHeader());
  calibrationTrialsFile_ =
      openFileWithHeader("calibration_trials.csv", calibrationTrialHeader());
  calibrationSummariesFile_ = openFileWithHeader("calibration_summaries.csv",
                                                 calibrationSummaryHeader());
}

std::ofstream ResultsWriter::openFileWithHeader(
    const std::string& filename, const std::string& header) const {
  const std::filesystem::path path = runDirectory_ / filename;
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("Failed to open canonical results file: " +
                             path.string());
  }
  stream << header << "\n";
  if (!stream.good()) {
    throw std::runtime_error("Failed to write canonical results header: " +
                             path.string());
  }
  return stream;
}

void ResultsWriter::writeRunMetadata(const RunMetadataRow& row) {
  writeRow(runMetadataFile_, toCsvRow(row));
}

void ResultsWriter::writeDataset(const DatasetRow& row) {
  writeRow(datasetsFile_, toCsvRow(row));
}

void ResultsWriter::writeWindowMetric(const WindowMetricRow& row) {
  writeRow(windowMetricsFile_, toCsvRow(row));
}

void ResultsWriter::writeWindowSummary(const WindowSummaryRow& row) {
  writeRow(windowSummariesFile_, toCsvRow(row));
}

void ResultsWriter::writeTrajectorySample(const TrajectorySampleRow& row) {
  writeRow(trajectorySamplesFile_, toCsvRow(row));
}

void ResultsWriter::writeCalibrationTrial(const CalibrationTrialRow& row) {
  writeRow(calibrationTrialsFile_, toCsvRow(row));
}

void ResultsWriter::writeCalibrationSummary(const CalibrationSummaryRow& row) {
  writeRow(calibrationSummariesFile_, toCsvRow(row));
}

}  // namespace gtsam
