/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testResultsWriter.cpp
 * @brief  Unit tests for canonical result package writing
 */

#include <CppUnitLite/TestHarness.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ResultsWriter.h"

using namespace gtsam;

namespace {

struct ScopedTempDirectory {
  explicit ScopedTempDirectory(const std::string& directoryName)
      : path(std::filesystem::current_path() / directoryName) {
    std::filesystem::remove_all(path);
    std::filesystem::create_directories(path);
  }

  ~ScopedTempDirectory() { std::filesystem::remove_all(path); }

  std::filesystem::path path;
};

std::vector<std::string> readLines(const std::filesystem::path& path) {
  std::ifstream stream(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  return lines;
}

}  // namespace

/* ************************************************************************* */
TEST(ResultsWriter, CreatesCanonicalFileSet) {
  const ScopedTempDirectory tempDirectory("results_writer_test_tmp");
  {
    const ResultsWriter writer("/tmp/evalExample", tempDirectory.path.string(),
                               "fixed_run");

    const std::filesystem::path runDirectory =
        tempDirectory.path / "evalExample" / "fixed_run";
    EXPECT(writer.runDirectory() == runDirectory);
    EXPECT(std::filesystem::exists(runDirectory / "run_metadata.csv"));
    EXPECT(std::filesystem::exists(runDirectory / "datasets.csv"));
    EXPECT(std::filesystem::exists(runDirectory / "window_metrics.csv"));
    EXPECT(std::filesystem::exists(runDirectory / "window_summaries.csv"));
    EXPECT(std::filesystem::exists(runDirectory / "trajectory_samples.csv"));
    EXPECT(std::filesystem::exists(runDirectory / "calibration_trials.csv"));
    EXPECT(std::filesystem::exists(runDirectory / "calibration_summaries.csv"));
  }

  const std::filesystem::path runDirectory =
      tempDirectory.path / "evalExample" / "fixed_run";
  EXPECT(readLines(runDirectory / "window_metrics.csv").size() == 1);
  EXPECT(readLines(runDirectory / "calibration_trials.csv").size() == 1);
}

/* ************************************************************************* */
TEST(ResultsWriter, GeneratesNonEmptyRunId) {
  const ScopedTempDirectory tempDirectory("results_writer_run_id_tmp");
  const ResultsWriter writer("evalExample", tempDirectory.path.string());

  EXPECT(!writer.runId().empty());
  EXPECT(writer.runDirectory().filename() == writer.runId());
}

/* ************************************************************************* */
TEST(ResultsWriter, WritesCanonicalHeadersAndRows) {
  const ScopedTempDirectory tempDirectory("results_writer_rows_tmp");
  {
    ResultsWriter writer("evalExample", tempDirectory.path.string(),
                         "fixed_run");

    writer.writeRunMetadata(
        {"fixed_run", "evalExample", "2026-04-17T12:00:00Z",
         "./evalExample --flag value", tempDirectory.path.string(), "abc123"});
    writer.writeDataset({"fixed_run", "evalExample", "MH01",
                         "../data/euroc/euroc_MH01.csv", "all"});

    TrajectorySampleRow trajectoryRow;
    trajectoryRow.runId = "fixed_run";
    trajectoryRow.appName = "evalExample";
    trajectoryRow.dataset = "MH01";
    trajectoryRow.method = "gal3_imu_ekf";
    trajectoryRow.configLabel = "default";
    trajectoryRow.intervalSeconds = 0.2;
    trajectoryRow.samplesPerWindow = 40;
    trajectoryRow.timestamp = 1.0;
    trajectoryRow.covariance = Matrix9::Identity();
    writer.writeTrajectorySample(trajectoryRow);
  }

  const std::filesystem::path runDirectory =
      tempDirectory.path / "evalExample" / "fixed_run";
  const std::vector<std::string> metadataLines =
      readLines(runDirectory / "run_metadata.csv");
  EXPECT(metadataLines.size() == 2);
  EXPECT(metadataLines[0] == runMetadataHeader());
  EXPECT(metadataLines[1].find("fixed_run") != std::string::npos);

  const std::vector<std::string> trajectoryLines =
      readLines(runDirectory / "trajectory_samples.csv");
  EXPECT(trajectoryLines.size() == 2);
  EXPECT(trajectoryLines[0] == trajectorySampleHeader());
  EXPECT(trajectoryLines[0].find("interval_seconds") != std::string::npos);
  EXPECT(trajectoryLines[0].find("cov_8_8") != std::string::npos);
  EXPECT(trajectoryLines[1].find("gal3_imu_ekf") != std::string::npos);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
