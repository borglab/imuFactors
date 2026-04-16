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
 * @brief  Compare reduced NEES for Quadrature, Manifold, and Tangent IMU preintegration
 */

#include "AppUtils.h"
#include "Dataset.h"
#include "NoiseCalibration.h"
#include "Window.h"
#include "nees.h"

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

constexpr double kAlpha = 3.0;
constexpr double kSigmaGyro = kAlpha * 1.6968e-4;
constexpr double kSigmaAcc = kAlpha * 2.0000e-3;
constexpr double kInitialStateCovariance = 5e-6;
constexpr double kInitialBiasCovariance = 1e-1;

struct MethodSummary {
  double reducedNeesMedian = 0.0;
  double reducedNeesMean = 0.0;
  double reducedNeesP95 = 0.0;
  size_t sampleCount = 0;
};

Matrix9 initialNavCovariance() { return Matrix9::Identity() * kInitialStateCovariance; }

Matrix6 initialBiasCovariance() { return Matrix6::Identity() * kInitialBiasCovariance; }

shared_ptr<PreintegrationParams> createPreintegrationParams() {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = I_3x3 * kSigmaAcc * kSigmaAcc;
  params->gyroscopeCovariance = I_3x3 * kSigmaGyro * kSigmaGyro;
  params->integrationCovariance = I_3x3 * 1e-8;
  return params;
}

optional<double> computeReducedNees(const Vector9& error, const Matrix9& covariance) {
  const Matrix9 regularizedCovariance = covariance + 1e-12 * Matrix9::Identity();
  const auto nees = normalizedQuadraticForm(error, regularizedCovariance, 9.0);
  if (!nees || !std::isfinite(*nees)) {
    return nullopt;
  }
  return *nees;
}

MethodSummary summarizeReducedNees(const vector<double>& reducedNeesValues) {
  MethodSummary summary;
  if (reducedNeesValues.empty()) {
    return summary;
  }

  summary.sampleCount = reducedNeesValues.size();
  summary.reducedNeesMean = computeMean(reducedNeesValues);
  summary.reducedNeesMedian = computeMedian(reducedNeesValues);
  summary.reducedNeesP95 = computePercentile(reducedNeesValues, 95.0);
  return summary;
}

PIMQuadrature buildQuadraturePreintegrated(
    const Window& window,
    const shared_ptr<PreintegrationParams>& params,
    size_t quadratureOrder,
    const imuBias::ConstantBias& bias) {
  PIMQuadrature preintegrated(params, bias, quadratureOrder);
  window.integrateMeasurements(preintegrated);
  preintegrated.endPreintegration(preintegrated.deltaTij());
  return preintegrated;
}

template <class PIMType>
PIMType buildStandardPreintegrated(
    const Window& window,
    const shared_ptr<PreintegrationParams>& params,
    const imuBias::ConstantBias& bias) {
  PIMType preintegrated(params, bias);
  window.integrateMeasurements(preintegrated);
  return preintegrated;
}

Vector9 computeQuadratureDeltaForBias(
    const Window& window,
    const shared_ptr<PreintegrationParams>& params,
    size_t quadratureOrder,
    const imuBias::ConstantBias& bias) {
  const auto preintegrated = buildQuadraturePreintegrated(window, params, quadratureOrder, bias);

  Vector9 delta;
  delta << preintegrated.deltaRij().logmap(Rot3()), preintegrated.deltaPij(),
      preintegrated.deltaVij();
  return delta;
}

optional<double> computeQuadratureReducedNeesForWindow(
    const Window& window,
    const shared_ptr<PreintegrationParams>& params,
    size_t quadratureOrder) {
  const imuBias::ConstantBias& initialBias = window.initialBias();
  auto preintegrated =
      buildQuadraturePreintegrated(window, params, quadratureOrder, initialBias);

  constexpr double kBiasPerturbation = 1e-5;
  Matrix96 biasJacobian;
  const Vector6 biasVector = initialBias.vector();
  for (int column = 0; column < 6; ++column) {
    Vector6 plusBias = biasVector;
    Vector6 minusBias = biasVector;
    plusBias(column) += kBiasPerturbation;
    minusBias(column) -= kBiasPerturbation;

    const Vector9 deltaPlus = computeQuadratureDeltaForBias(
        window, params, quadratureOrder,
        imuBias::ConstantBias(plusBias.head<3>(), plusBias.tail<3>()));
    const Vector9 deltaMinus = computeQuadratureDeltaForBias(
        window, params, quadratureOrder,
        imuBias::ConstantBias(minusBias.head<3>(), minusBias.tail<3>()));
    biasJacobian.col(column) = (deltaPlus - deltaMinus) / (2.0 * kBiasPerturbation);
  }

  preintegrated.setInitialCovariance(
      initialNavCovariance() + biasJacobian * initialBiasCovariance() * biasJacobian.transpose());

  QuadratureImuFactor factor(X(1), V(1), X(2), V(2), B(1), preintegrated);
  const Vector9 error = factor.evaluateError(
      window.initialState().pose(), window.initialState().velocity(),
      window.terminalState().pose(), window.terminalState().velocity(), initialBias);
  return computeReducedNees(error, preintegrated.preintMeasCov());
}

template <class PIMType>
optional<double> computeStandardReducedNeesForWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params) {
  const imuBias::ConstantBias& initialBias = window.initialBias();
  auto preintegrated = buildStandardPreintegrated<PIMType>(window, params, initialBias);

  Matrix96 biasJacobian;
  preintegrated.biasCorrectedDelta(initialBias, biasJacobian);
  const Matrix9 totalCovariance =
      preintegrated.preintMeasCov() + initialNavCovariance() +
      biasJacobian * initialBiasCovariance() * biasJacobian.transpose();
  preintegrated.setPreintMeasCov(totalCovariance);

  ImuFactor2T<PIMType> factor(X(1), X(2), B(1), preintegrated);
  const Vector9 error =
      factor.evaluateError(window.initialState(), window.terminalState(), initialBias);
  return computeReducedNees(error, preintegrated.preintMeasCov());
}

MethodSummary runQuadratureReducedNees(
    const vector<Window>& windows, size_t quadratureOrder) {

  vector<double> reducedNeesValues;
  const auto params = createPreintegrationParams();

  for (const auto& window : windows) {
    const auto reducedNees =
        computeQuadratureReducedNeesForWindow(window, params, quadratureOrder);
    if (reducedNees) {
      reducedNeesValues.push_back(*reducedNees);
    }
  }

  return summarizeReducedNees(reducedNeesValues);
}

template <class PIMType>
MethodSummary runStandardReducedNees(const vector<Window>& windows) {
  vector<double> reducedNeesValues;
  const auto params = createPreintegrationParams();

  for (const auto& window : windows) {
    const auto reducedNees = computeStandardReducedNeesForWindow<PIMType>(window, params);
    if (reducedNees) {
      reducedNeesValues.push_back(*reducedNees);
    }
  }

  return summarizeReducedNees(reducedNeesValues);
}
}  // namespace

int main(int argc, char* argv[]) {
  try {
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
    const vector<double> intervals = selectIntervals(defaultIntervals, options.maxIntervals);

    cout << "# Quadrature vs Manifold vs Tangent reduced NEES\n";
    cout << "# alpha=" << kAlpha << " sigma_gyro=" << kSigmaGyro << " sigma_acc=" << kSigmaAcc << "\n\n";
    cout << "| dataset | dt(s) | m | N | Quadrature | Manifold | Tangent |\n";
    cout << "|---|---:|---:|---:|---:|---:|---:|\n";

    for (const auto& datasetEntry : datasets) {
      const string& datasetName = datasetEntry.first;
      const string& datasetPath = datasetEntry.second;
      Dataset dataset(datasetPath);

      const auto& states = dataset.getStates();
      if (states.size() < 2) {
        continue;
      }
      for (double intervalSeconds : intervals) {
        const size_t windowSize = dataset.stepsForInterval(intervalSeconds);
        const auto windows = dataset.completeWindows(windowSize);
        const size_t quadratureOrder = max<size_t>(
            3, static_cast<size_t>(floor(sqrt(static_cast<double>(windowSize)))));

        const MethodSummary quadrature =
            runQuadratureReducedNees(windows, quadratureOrder);
        const MethodSummary manifold = runStandardReducedNees<PIMManifold>(windows);
        const MethodSummary tangent = runStandardReducedNees<PIMTangent>(windows);

        cout << "| " << datasetName << " | " << fixed << setprecision(1) << intervalSeconds << " | "
             << windowSize << " | " << quadratureOrder << " | " << setprecision(3)
             << quadrature.reducedNeesMedian << " | " << manifold.reducedNeesMedian << " | "
             << tangent.reducedNeesMedian << " |\n";
      }
    }
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
