/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalReducedNeesWithPriorCovariance.cpp
 * @brief  Compare normalized NEES for Quadrature, Manifold, and Tangent IMU
 * preintegration
 */

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "Dataset.h"
#include "NoiseCalibration.h"
#include "PIMs.h"
#include "Window.h"
#include "nees.h"

using namespace gtsam;
using namespace std;

using PIMQuadrature = PreintegratedImuMeasurementsQ;
using PIMTangent = PreintegratedImuMeasurementsT<TangentPreintegration>;
using PIMManifold = PreintegratedImuMeasurementsT<ManifoldPreintegration>;

namespace {

// Scale the nominal sensor noise for the prior-aware normalized-NEES
// comparison.
constexpr double kAlpha = 3.0;
constexpr double kSigmaGyro = kAlpha * 1.6968e-4;
constexpr double kSigmaAcc = kAlpha * 2.0000e-3;
constexpr double kInitialStateCovariance = 5e-6;
constexpr double kInitialBiasCovariance = 1e-1;

// Only the normalized-NEES distribution is reported in this script.
struct Summary {
  double normalizedNeesMedian = 0.0;
  double normalizedNeesMean = 0.0;
  double normalizedNeesP95 = 0.0;
  size_t sampleCount = 0;
};

// Model uncertainty in the initial pose, velocity, and rotation state.
Matrix9 initialNavCovariance() {
  return Matrix9::Identity() * kInitialStateCovariance;
}

// Model uncertainty in the initial accelerometer and gyro biases.
Matrix6 initialBiasCovariance() {
  return Matrix6::Identity() * kInitialBiasCovariance;
}

// Build the shared IMU noise model used by all three preintegration methods.
shared_ptr<PreintegrationParams> createPreintegrationParams() {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = I_3x3 * kSigmaAcc * kSigmaAcc;
  params->gyroscopeCovariance = I_3x3 * kSigmaGyro * kSigmaGyro;
  params->integrationCovariance = I_3x3 * 1e-8;
  return params;
}

Summary summarizeNormalizedNees(const vector<double>& normalizedNeesValues) {
  Summary summary;
  if (normalizedNeesValues.empty()) {
    return summary;
  }

  // Report the normalized-NEES distribution using a few standard summary
  // statistics.
  summary.sampleCount = normalizedNeesValues.size();
  summary.normalizedNeesMean = computeMean(normalizedNeesValues);
  summary.normalizedNeesMedian = computeMedian(normalizedNeesValues);
  summary.normalizedNeesP95 = computePercentile(normalizedNeesValues, 95.0);
  return summary;
}

optional<double> computeQuadratureNormalizedNeesForWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params,
    size_t N) {
  const InitialCovarianceOptions initialCovariance{initialNavCovariance(),
                                                   initialBiasCovariance()};
  const auto result =
      evaluateWindow<PIMQuadrature>(window, params, initialCovariance, N);
  return result ? optional<double>(result->normalizedNees) : nullopt;
}

template <class PIMType>
optional<double> computeStandardNormalizedNeesForWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params) {
  const InitialCovarianceOptions initialCovariance{initialNavCovariance(),
                                                   initialBiasCovariance()};
  const auto result =
      evaluateWindow<PIMType>(window, params, initialCovariance);
  return result ? optional<double>(result->normalizedNees) : nullopt;
}

Summary runQuadratureNormalizedNees(const vector<Window>& windows,
                                    size_t quadratureOrder) {
  vector<double> normalizedNeesValues;
  const auto params = createPreintegrationParams();

  // Score each valid window and keep only the normalized-NEES scalar.
  for (const auto& window : windows) {
    const auto normalizedNees = computeQuadratureNormalizedNeesForWindow(
        window, params, quadratureOrder);
    if (normalizedNees) normalizedNeesValues.push_back(*normalizedNees);
  }

  return summarizeNormalizedNees(normalizedNeesValues);
}

template <class PIMType>
Summary runStandardNormalizedNees(const vector<Window>& windows) {
  vector<double> normalizedNeesValues;
  const auto params = createPreintegrationParams();

  // Score each valid window for the chosen standard preintegration method.
  for (const auto& window : windows) {
    const auto normalizedNees =
        computeStandardNormalizedNeesForWindow<PIMType>(window, params);
    if (normalizedNees) {
      normalizedNeesValues.push_back(*normalizedNees);
    }
  }

  return summarizeNormalizedNees(normalizedNeesValues);
}

struct AppOptions {
  size_t maxIntervals = 0;
};

struct Row {
  string dataset;
  double interval = 0.0;
  size_t samplesPerWindow = 0;
  size_t quadratureNodes = 0;
  Summary quadrature;
  Summary manifold;
  Summary tangent;
};

void printUsage(const char* programName) {
  cout << "Usage: " << programName
       << " [--data-dir <path>] [--dataset <name>] [--max-intervals <count>]\n";
  cout << "  --data-dir <path>       Dataset directory (default: "
          "../data/euroc/)\n";
  cout << "  --dataset <name>        Restrict to one dataset (e.g. MH01 or "
          "euroc_MH01.csv)\n";
  cout << "  --max-intervals <count> Restrict to first N default intervals\n";
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

struct RunForDataset {
  explicit RunForDataset(vector<double> intervals)
      : intervals(std::move(intervals)) {}

  vector<double> intervals;
  vector<Row> rows;

  void operator()(const string& datasetName, const Dataset& dataset) {
    for (double intervalSeconds : intervals) {
      const size_t m = dataset.stepsForInterval(intervalSeconds);
      const size_t N = max<size_t>(
          3, static_cast<size_t>(floor(sqrt(static_cast<double>(m)))));

      // Compare the prior-aware normalized-NEES score across all three
      // methods.
      const auto windows = dataset.completeWindows(m);
      Row row;
      row.dataset = datasetName;
      row.interval = intervalSeconds;
      row.samplesPerWindow = m;
      row.quadratureNodes = N;
      row.quadrature = runQuadratureNormalizedNees(windows, N);
      row.manifold = runStandardNormalizedNees<PIMManifold>(windows);
      row.tangent = runStandardNormalizedNees<PIMTangent>(windows);
      rows.push_back(row);
    }
  }
};
}  // namespace

int main(int argc, char* argv[]) {
  // Resolve the datasets and interval subset requested on the command line.
  const auto datasetCli = resolveDatasetAppCli(argc, argv);
  const AppOptions appOptions =
      parseAppArguments(datasetCli.remainingArgs, argv[0]);

  const vector<double> defaultIntervals = defaultQuadratureIntervals();
  const vector<double> intervals =
      selectIntervals(defaultIntervals, appOptions.maxIntervals);

  // Print a compact one-table summary comparing normalized NEES.
  cout << "# Quadrature vs Manifold vs Tangent normalized NEES\n";
  cout << "# alpha=" << kAlpha << " sigma_gyro=" << kSigmaGyro
       << " sigma_acc=" << kSigmaAcc << "\n\n";
  cout << "| dataset | dt(s) | m | N | Quadrature | Manifold | Tangent |\n";
  cout << "|---|---:|---:|---:|---:|---:|---:|\n";

  RunForDataset runner(intervals);
  const int status = runForDatasets(datasetCli, runner);
  if (status != 0) {
    return status;
  }

  for (const auto& row : runner.rows) {
    cout << "| " << row.dataset << " | " << fixed << setprecision(1)
         << row.interval << " | " << row.samplesPerWindow << " | "
         << row.quadratureNodes << " | " << setprecision(3)
         << row.quadrature.normalizedNeesMedian << " | "
         << row.manifold.normalizedNeesMedian << " | "
         << row.tangent.normalizedNeesMedian << " |\n";
  }
  return 0;
}
