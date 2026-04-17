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
  const InitialCovarianceOptions initialCovariance{
      initialNavCovariance(), initialBiasCovariance()};
  const auto result =
      evaluateWindow<PIMQuadrature>(window, params, initialCovariance, N);
  return result ? optional<double>(result->normalizedNees) : nullopt;
}

template <class PIMType>
optional<double> computeStandardNormalizedNeesForWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params) {
  const InitialCovarianceOptions initialCovariance{
      initialNavCovariance(), initialBiasCovariance()};
  const auto result = evaluateWindow<PIMType>(window, params, initialCovariance);
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
}  // namespace

int main(int argc, char* argv[]) {
  try {
    // Resolve the datasets and interval subset requested on the command line.
    const AppCliOptions options = parseDatasetAppCliOptions(argc, argv);
    const vector<pair<string, string>> discoveredDatasets =
        discoverFilteredDatasets(options.dataDirectory, DatasetFilters::all);
    const vector<pair<string, string>> datasets =
        selectDatasets(discoveredDatasets, options.datasetName);

    if (datasets.empty()) {
      cerr << "No datasets found in " << options.dataDirectory << "\n";
      return 1;
    }

    const vector<double> defaultIntervals = defaultQuadratureIntervals();
    const vector<double> intervals =
        selectIntervals(defaultIntervals, options.maxIntervals);

    // Print a compact one-table summary comparing normalized NEES.
    cout << "# Quadrature vs Manifold vs Tangent normalized NEES\n";
    cout << "# alpha=" << kAlpha << " sigma_gyro=" << kSigmaGyro
         << " sigma_acc=" << kSigmaAcc << "\n\n";
    cout << "| dataset | dt(s) | m | N | Quadrature | Manifold | Tangent |\n";
    cout << "|---|---:|---:|---:|---:|---:|---:|\n";

    for (const auto& datasetEntry : datasets) {
      const string& datasetPath = datasetEntry.second;
      Dataset dataset(datasetPath);
      if (dataset.truth.size() < 2) continue;
      const string& datasetName = datasetEntry.first;

      for (double intervalSeconds : intervals) {
        const size_t m = dataset.stepsForInterval(intervalSeconds);
        const size_t N = max<size_t>(
            3, static_cast<size_t>(floor(sqrt(static_cast<double>(m)))));

        // Compare the prior-aware normalized-NEES score across all three
        // methods.
        const auto windows = dataset.completeWindows(m);
        const Summary quadrature = runQuadratureNormalizedNees(windows, N);
        const Summary manifold = runStandardNormalizedNees<PIMManifold>(windows);
        const Summary tangent = runStandardNormalizedNees<PIMTangent>(windows);

        cout << "| " << datasetName << " | " << fixed << setprecision(1)
             << intervalSeconds << " | " << m << " | " << N << " | "
             << setprecision(3) << quadrature.normalizedNeesMedian << " | "
             << manifold.normalizedNeesMedian << " | "
             << tangent.normalizedNeesMedian << " |\n";
      }
    }
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
