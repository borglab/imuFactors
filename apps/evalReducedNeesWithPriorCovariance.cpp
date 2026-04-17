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
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/QuadratureImuFactor.h>
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
using QuadratureImuFactor = ImuFactorT<PreintegratedImuMeasurementsQ>;
using PIMTangent = PreintegratedImuMeasurementsT<TangentPreintegration>;
using PIMManifold = PreintegratedImuMeasurementsT<ManifoldPreintegration>;

using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

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

Vector9 computeQuadratureDeltaForBias(
    const Window& window, const shared_ptr<PreintegrationParams>& params,
    size_t quadratureOrder, const imuBias::ConstantBias& bias) {
  // Convert quadrature's preintegrated state into the 9D residual space used by
  // NEES.
  const auto preintegrated =
      buildPreintegrated<PIMQuadrature>(window, params, bias, quadratureOrder);

  Vector9 delta;
  delta << preintegrated.deltaRij().logmap(Rot3()), preintegrated.deltaPij(),
      preintegrated.deltaVij();
  return delta;
}

optional<double> computeQuadratureNormalizedNeesForWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params,
    size_t N) {
  const imuBias::ConstantBias& initialBias = window.initialTruth().bias;
  // Start from the nominal quadrature preintegration at the ground-truth bias.
  auto preintegrated =
      buildPreintegrated<PIMQuadrature>(window, params, initialBias, N);

  // Approximate the bias Jacobian numerically because the quadrature path does
  // not expose it.
  constexpr double kBiasPerturbation = 1e-5;
  Matrix96 biasJacobian;
  const Vector6 biasVector = initialBias.vector();
  for (int column = 0; column < 6; ++column) {
    Vector6 plusBias = biasVector;
    Vector6 minusBias = biasVector;
    plusBias(column) += kBiasPerturbation;
    minusBias(column) -= kBiasPerturbation;

    const Vector9 deltaPlus = computeQuadratureDeltaForBias(
        window, params, N,
        imuBias::ConstantBias(plusBias.head<3>(), plusBias.tail<3>()));
    const Vector9 deltaMinus = computeQuadratureDeltaForBias(
        window, params, N,
        imuBias::ConstantBias(minusBias.head<3>(), minusBias.tail<3>()));
    biasJacobian.col(column) =
        (deltaPlus - deltaMinus) / (2.0 * kBiasPerturbation);
  }

  // Fold initial state and bias cov into covariance before scoring NEES.
  preintegrated.setInitialCovariance(initialNavCovariance() +
                                     biasJacobian * initialBiasCovariance() *
                                         biasJacobian.transpose());

  QuadratureImuFactor factor(X(1), V(1), X(2), V(2), B(1), preintegrated);
  const Vector9 error = factor.evaluateError(
      window.initialTruth().navState.pose(),
      window.initialTruth().navState.velocity(),
      window.terminalTruth().navState.pose(),
      window.terminalTruth().navState.velocity(), initialBias);
  const auto normalizedNees =
      normalizedNEES(error, preintegrated.preintMeasCov(), 9.0);
  if (!normalizedNees || !std::isfinite(*normalizedNees)) {
    return nullopt;
  }
  return *normalizedNees;
}

template <class PIMType>
optional<double> computeStandardNormalizedNeesForWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params) {
  const imuBias::ConstantBias& initialBias = window.initialTruth().bias;
  // Start from the nominal standard preintegration at the ground-truth bias.
  auto preintegrated = buildPreintegrated<PIMType>(window, params, initialBias);

  // Standard preintegration provides the bias Jacobian directly.
  Matrix96 biasJacobian;
  preintegrated.biasCorrectedDelta(initialBias, biasJacobian);
  const Matrix9 totalCovariance =
      preintegrated.preintMeasCov() + initialNavCovariance() +
      biasJacobian * initialBiasCovariance() * biasJacobian.transpose();
  preintegrated.setPreintMeasCov(totalCovariance);

  ImuFactor2T<PIMType> factor(X(1), X(2), B(1), preintegrated);
  const Vector9 error =
      factor.evaluateError(window.initialTruth().navState,
                           window.terminalTruth().navState, initialBias);
  const auto normalizedNees =
      normalizedNEES(error, preintegrated.preintMeasCov(), 9.0);
  if (!normalizedNees || !std::isfinite(*normalizedNees)) {
    return nullopt;
  }
  return *normalizedNees;
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
