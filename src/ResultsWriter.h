/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "ResultsSchema.h"

namespace gtsam {

/**
 * @brief Parse an optional `--output-root <path>` argument.
 */
struct ParsedOutputRootCli {
  std::string outputRoot = "./results";
  std::vector<std::string> remainingArgs;
};

/**
 * @brief Join command-line arguments into one display string.
 */
std::string joinCommandLineArguments(int argc, char* argv[]);

/**
 * @brief Parse an optional `--output-root <path>` argument.
 *
 * Unrecognized arguments are preserved in order.
 */
ParsedOutputRootCli parseOutputRootCliArguments(
    const std::vector<std::string>& arguments);

/**
 * @brief Shared canonical results package writer.
 */
class ResultsWriter {
 public:
  /**
   * @brief Construct a writer for one application run.
   * @param programName Executable name, usually `argv[0]`
   * @param outputRoot Root results directory
   * @param runId Optional fixed run id for tests
   */
  ResultsWriter(const std::string& programName, const std::string& outputRoot,
                const std::string& runId = "");

  /**
   * @brief Application name used in output rows and directories.
   */
  const std::string& appName() const { return appName_; }

  /**
   * @brief Generated run identifier.
   */
  const std::string& runId() const { return runId_; }

  /**
   * @brief Run creation timestamp in UTC.
   */
  const std::string& timestampUtc() const { return timestampUtc_; }

  /**
   * @brief Canonical output root.
   */
  const std::filesystem::path& outputRoot() const { return outputRoot_; }

  /**
   * @brief Canonical output directory for this run.
   */
  const std::filesystem::path& runDirectory() const { return runDirectory_; }

  /// Write one metadata row.
  void writeRunMetadata(const RunMetadataRow& row);

  /// Write one dataset row.
  void writeDataset(const DatasetRow& row);

  /// Write one window metric row.
  void writeWindowMetric(const WindowMetricRow& row);

  /// Write one window summary row.
  void writeWindowSummary(const WindowSummaryRow& row);

  /// Write one trajectory sample row.
  void writeTrajectorySample(const TrajectorySampleRow& row);

  /// Write one calibration trial row.
  void writeCalibrationTrial(const CalibrationTrialRow& row);

  /// Write one calibration summary row.
  void writeCalibrationSummary(const CalibrationSummaryRow& row);

 private:
  std::string appName_;
  std::string timestampUtc_;
  std::string runId_;
  std::filesystem::path outputRoot_;
  std::filesystem::path runDirectory_;

  std::ofstream runMetadataFile_;
  std::ofstream datasetsFile_;
  std::ofstream windowMetricsFile_;
  std::ofstream windowSummariesFile_;
  std::ofstream trajectorySamplesFile_;
  std::ofstream calibrationTrialsFile_;
  std::ofstream calibrationSummariesFile_;

  /**
   * @brief Open a run file and write its canonical header.
   */
  std::ofstream openFileWithHeader(const std::string& filename,
                                   const std::string& header) const;
};

}  // namespace gtsam
