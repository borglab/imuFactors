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
 * @brief  Clean NEES comparison table for EKF variants
 * @author Alec Kain
 */

#include <iomanip>
#include <iostream>
#include <vector>

#include "AppUtils.h"
#include "EKFNEESEvaluator.h"

using namespace gtsam;
using namespace std;

namespace {

constexpr double kAlpha = 3.0;
const vector<double> kIntervals{0.2, 0.5, 1.0};

/// NEES results for all intervals.
struct DatasetResults {
  NEESResults gal3_0_2s;
  NEESResults gal3_0_5s;
  NEESResults gal3_1_0s;
  NEESResults navstate_0_2s;
  NEESResults navstate_0_5s;
  NEESResults navstate_1_0s;
};

struct ResultRow {
  string datasetName;
  DatasetResults results;
};

/// Print usage for the EKF comparison app.
void printUsage(const char* programName) { printDatasetAppUsage(programName); }

/// Validate any EKF-specific trailing arguments.
void parseAppArguments(const vector<string>& arguments,
                       const char* programName) {
  for (const string& argument : arguments) {
    if (isHelpArgument(argument)) {
      printUsage(programName);
      std::exit(0);
    }
    throw runtime_error("Unknown argument: " + argument);
  }
}

/// Print table header.
static void printTableHeader() {
  cout << "\n" << string(70, '=') << "\n";
  cout << "Dataset\t\tMethod\t\t\t0.2s\t0.5s\t1.0s\n";
  cout << string(70, '-') << "\n";
}

/// Print NEES results row.
static void printTableRow(const string& datasetName, const string& methodName,
                          const NEESResults& result_0_2s,
                          const NEESResults& result_0_5s,
                          const NEESResults& result_1_0s) {
  cout << datasetName << "\t" << methodName << ":\t" << fixed << setprecision(3)
       << result_0_2s.median << "\t" << result_0_5s.median << "\t"
       << result_1_0s.median << "\n";
}

/// Accumulate EKF comparison rows for each requested dataset.
struct RunForDataset {
  vector<ResultRow> rows;

  void operator()(const string& datasetName, const Dataset& dataset) {
    EKFNEESEvaluator evaluator(dataset);

    DatasetResults results;
    results.gal3_0_2s =
        evaluator.runGal3ImuEKF(kIntervals[0], kAlpha, datasetName);
    results.gal3_0_5s =
        evaluator.runGal3ImuEKF(kIntervals[1], kAlpha, datasetName);
    results.gal3_1_0s =
        evaluator.runGal3ImuEKF(kIntervals[2], kAlpha, datasetName);
    results.navstate_0_2s =
        evaluator.runNavStateImuEKF(kIntervals[0], kAlpha, datasetName);
    results.navstate_0_5s =
        evaluator.runNavStateImuEKF(kIntervals[1], kAlpha, datasetName);
    results.navstate_1_0s =
        evaluator.runNavStateImuEKF(kIntervals[2], kAlpha, datasetName);
    rows.push_back({datasetName, results});
  }

  void print() const {
    printTableHeader();
    for (size_t index = 0; index < rows.size(); ++index) {
      const ResultRow& row = rows[index];
      printTableRow(row.datasetName, "Gal3ImuEKF", row.results.gal3_0_2s,
                    row.results.gal3_0_5s, row.results.gal3_1_0s);
      printTableRow(row.datasetName, "NavStateImuEKF",
                    row.results.navstate_0_2s, row.results.navstate_0_5s,
                    row.results.navstate_1_0s);
      if (index + 1 != rows.size()) {
        cout << string(70, '-') << "\n";
      }
    }
    cout << string(70, '=') << "\n";
  }
};

}  // namespace

/// Main evaluation program.
int main(int argc, char* argv[]) {
  const auto datasetCli = resolveDatasetAppCli(argc, argv);
  parseAppArguments(datasetCli.remainingArgs, argv[0]);

  RunForDataset runner;
  const int status = runForDatasets(datasetCli, runner);
  if (status != 0) {
    return status;
  }
  runner.print();
  return 0;
}
