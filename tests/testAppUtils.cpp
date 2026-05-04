/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testAppUtils.cpp
 * @brief  Unit tests for dataset CLI and discovery helpers
 */

#include <CppUnitLite/TestHarness.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "QuadratureRunner.h"

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

void createEmptyFile(const std::filesystem::path& path) {
  std::ofstream stream(path);
  stream << "placeholder\n";
}

struct CountingRunner {
  size_t callCount = 0;

  void operator()(const std::string&, const Dataset&) { ++callCount; }
};

struct ThrowingRunner {
  void operator()(const std::string&, const Dataset&) {
    throw std::runtime_error("runner failed");
  }
};

}  // namespace

/* ************************************************************************* */
TEST(AppUtils, NormalizeDatasetName) {
  EXPECT(normalizeDatasetName("MH01") == "MH01");
  EXPECT(normalizeDatasetName("euroc_MH01.csv") == "MH01");
  EXPECT(normalizeDatasetName("/tmp/euroc_V202.csv") == "V202");
}

/* ************************************************************************* */
TEST(AppUtils, ParseDatasetCliOptionsPreservesRemainingArgs) {
  const ParsedAppCliOptions parsedOptions = parseDatasetAppCliOptions(
      {"--dataset", "euroc_MH01.csv", "--data-dir", "/tmp/euroc",
       "--output-root", "/tmp/results",
       "--max-intervals", "2", "--custom-flag"});

  EXPECT(parsedOptions.options.dataDirectory == "/tmp/euroc");
  EXPECT(parsedOptions.options.outputRoot == "/tmp/results");
  EXPECT(parsedOptions.options.datasetName.has_value());
  EXPECT(parsedOptions.options.datasetName.value() == "MH01");
  EXPECT(parsedOptions.remainingArgs.size() == 3);
  EXPECT(parsedOptions.remainingArgs[0] == "--max-intervals");
  EXPECT(parsedOptions.remainingArgs[1] == "2");
  EXPECT(parsedOptions.remainingArgs[2] == "--custom-flag");
}

/* ************************************************************************* */
TEST(AppUtils, ResolveDatasetCliFiltersAndSortsDatasets) {
  const ScopedTempDirectory tempDirectory("app_utils_test_tmp");
  createEmptyFile(tempDirectory.path / "euroc_V202.csv");
  createEmptyFile(tempDirectory.path / "euroc_MH01.csv");
  createEmptyFile(tempDirectory.path / "notes.txt");

  const ResolvedDatasetCli allDatasets =
      resolveDatasetAppCli({"--data-dir", tempDirectory.path.string()});
  EXPECT(allDatasets.datasets.size() == 2);
  EXPECT(allDatasets.datasets[0].first == "MH01");
  EXPECT(allDatasets.datasets[1].first == "V202");

  const ResolvedDatasetCli filteredDataset =
      resolveDatasetAppCli({"--data-dir", tempDirectory.path.string(),
                            "--dataset", "euroc_V202.csv"});
  EXPECT(filteredDataset.datasets.size() == 1);
  EXPECT(filteredDataset.datasets[0].first == "V202");
  EXPECT(filteredDataset.remainingArgs.empty());
}

/* ************************************************************************* */
TEST(AppUtils, ParseMaxIntervalsArgument) {
  EXPECT(parseMaxIntervalsArgument({}) == 0);
  EXPECT(parseMaxIntervalsArgument({"--max-intervals", "2"}) == 2);
}

/* ************************************************************************* */
TEST(AppUtils, ParseQuadratureAlphaArguments) {
  const QuadratureAppOptions defaultOptions =
      parseQuadratureAppArguments({}, "test", 8.4);
  EXPECT_DOUBLES_EQUAL(8.4, defaultOptions.alphaGyro, 1e-9);
  EXPECT_DOUBLES_EQUAL(8.4, defaultOptions.alphaAcc, 1e-9);

  const QuadratureAppOptions sharedAlphaOptions =
      parseQuadratureAppArguments({"--alpha", "4.2"}, "test", 8.4);
  EXPECT_DOUBLES_EQUAL(4.2, sharedAlphaOptions.alphaGyro, 1e-9);
  EXPECT_DOUBLES_EQUAL(4.2, sharedAlphaOptions.alphaAcc, 1e-9);

  const QuadratureAppOptions splitAlphaOptions = parseQuadratureAppArguments(
      {"--alpha", "4.2", "--alpha-gyro", "13.0", "--alpha-acc", "9.4",
       "--max-intervals", "2"},
      "test", 8.4);
  EXPECT_DOUBLES_EQUAL(13.0, splitAlphaOptions.alphaGyro, 1e-9);
  EXPECT_DOUBLES_EQUAL(9.4, splitAlphaOptions.alphaAcc, 1e-9);
  EXPECT(splitAlphaOptions.maxIntervals == 2);
}

/* ************************************************************************* */
TEST(AppUtils, RunForDatasetsHandlesEmptyAndExceptions) {
  std::ostringstream capturedErrors;
  std::streambuf* const originalErrorBuffer = std::cerr.rdbuf();
  std::cerr.rdbuf(capturedErrors.rdbuf());

  const ResolvedDatasetCli emptyCli{{"/tmp/missing", "./results", std::nullopt},
                                    {}, {}};
  CountingRunner countingRunner;
  const int emptyStatus = runForDatasets(emptyCli, countingRunner);
  EXPECT(emptyStatus == 1);
  EXPECT(countingRunner.callCount == 0);
  EXPECT(capturedErrors.str().find("No datasets found in /tmp/missing") !=
         std::string::npos);

  capturedErrors.str("");
  capturedErrors.clear();

  const ResolvedDatasetCli datasetCli = resolveDatasetAppCli(
      {"--data-dir", "../data/euroc", "--dataset", "MH01"});
  ThrowingRunner throwingRunner;
  const int exceptionStatus = runForDatasets(datasetCli, throwingRunner);
  EXPECT(exceptionStatus == 1);
  EXPECT(capturedErrors.str().find("Error: runner failed") !=
         std::string::npos);

  std::cerr.rdbuf(originalErrorBuffer);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
