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
 * @brief  Minimal NEES and error analysis for Quadrature, Manifold, and Tangent IMU preintegration
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

constexpr double kAlpha = 8.4;
constexpr double kSigmaGyro = kAlpha * 1.6968e-4;
constexpr double kSigmaAcc = kAlpha * 2.0000e-3;

struct NeesStats {
  double mean = 0.0;
  double median = 0.0;
  double p95 = 0.0;
  size_t samples = 0;
};

struct MethodSummary {
  NeesStats nees;
  double rotErrorMedian = 0.0;
  double rotPredSigmaMedian = 0.0;
  double posErrorMedian = 0.0;
  double posPredSigmaMedian = 0.0;
  double velErrorMedian = 0.0;
  double velPredSigmaMedian = 0.0;
  size_t samples = 0;
  size_t nodeCount = 0;
};

struct MethodSample {
  double reducedNees = 0.0;
  double rotErrorNorm = 0.0;
  double rotPredSigma = 0.0;
  double posErrorNorm = 0.0;
  double posPredSigma = 0.0;
  double velErrorNorm = 0.0;
  double velPredSigma = 0.0;
};

struct Row {
  string dataset;
  double interval = 0.0;
  size_t samplesPerWindow = 0;
  size_t quadratureNodes = 0;
  MethodSummary quadrature;
  MethodSummary manifold;
  MethodSummary tangent;
};

shared_ptr<PreintegrationParams> createPreintegrationParams() {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = I_3x3 * kSigmaAcc * kSigmaAcc;
  params->gyroscopeCovariance = I_3x3 * kSigmaGyro * kSigmaGyro;
  params->integrationCovariance = I_3x3 * 1e-8;
  return params;
}

optional<double> computeReducedNees(const Vector9& error, const Matrix9& covariance) {
  const Matrix9 regularizedCovariance = covariance + 1e-12 * Matrix9::Identity();
  const auto value = normalizedQuadraticForm(error, regularizedCovariance, 9.0);
  if (!value || !std::isfinite(*value)) {
    return nullopt;
  }
  return *value;
}

double covarianceBlockSigma(const Matrix9& covariance, int blockStart) {
  return std::sqrt(std::max(0.0, covariance.block<3, 3>(blockStart, blockStart).trace() / 3.0));
}

NeesStats summarizeNees(const vector<double>& reducedNeesValues) {
  NeesStats stats;
  if (reducedNeesValues.empty()) {
    return stats;
  }
  stats.samples = reducedNeesValues.size();
  stats.mean = computeMean(reducedNeesValues);
  stats.median = computeMedian(reducedNeesValues);
  stats.p95 = computePercentile(reducedNeesValues, 95.0);
  return stats;
}

double summarizeMedian(const vector<double>& values) { return computeMedian(values); }

MethodSample makeMethodSample(const Vector9& error, const Matrix9& covariance, double reducedNees) {
  MethodSample sample;
  sample.reducedNees = reducedNees;
  sample.rotErrorNorm = error.head<3>().norm();
  sample.rotPredSigma = covarianceBlockSigma(covariance, 0);
  sample.posErrorNorm = error.segment<3>(3).norm();
  sample.posPredSigma = covarianceBlockSigma(covariance, 3);
  sample.velErrorNorm = error.tail<3>().norm();
  sample.velPredSigma = covarianceBlockSigma(covariance, 6);
  return sample;
}

void appendMethodSample(
    const MethodSample& sample,
    vector<double>& reducedNeesValues,
    vector<double>& rotErrorNorms,
    vector<double>& rotPredSigmas,
    vector<double>& posErrorNorms,
    vector<double>& posPredSigmas,
    vector<double>& velErrorNorms,
    vector<double>& velPredSigmas) {
  reducedNeesValues.push_back(sample.reducedNees);
  rotErrorNorms.push_back(sample.rotErrorNorm);
  rotPredSigmas.push_back(sample.rotPredSigma);
  posErrorNorms.push_back(sample.posErrorNorm);
  posPredSigmas.push_back(sample.posPredSigma);
  velErrorNorms.push_back(sample.velErrorNorm);
  velPredSigmas.push_back(sample.velPredSigma);
}

optional<MethodSample> analyzeQuadratureWindow(
    const Window& window,
    const shared_ptr<PreintegrationParams>& params,
    size_t quadratureNodes) {
  PIMQuadrature preintegrated(params, window.initialBias(), quadratureNodes);
  window.integrateMeasurements(preintegrated);
  preintegrated.endPreintegration(preintegrated.deltaTij());

  const Matrix9 covariance = preintegrated.preintMeasCov();
  QuadratureImuFactor factor(X(1), V(1), X(2), V(2), B(1), preintegrated);
  const Vector9 error = factor.evaluateError(
      window.initialState().pose(), window.initialState().velocity(),
      window.terminalState().pose(), window.terminalState().velocity(),
      window.initialBias());

  const auto reducedNees = computeReducedNees(error, covariance);
  if (!reducedNees) {
    return nullopt;
  }
  return makeMethodSample(error, covariance, *reducedNees);
}

template <class PIMType>
optional<MethodSample> analyzeStandardWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params) {
  PIMType preintegrated(params, window.initialBias());
  window.integrateMeasurements(preintegrated);

  ImuFactor2T<PIMType> factor(X(1), X(2), B(1), preintegrated);
  const Vector9 error =
      factor.evaluateError(window.initialState(), window.terminalState(), window.initialBias());
  const Matrix9 covariance = preintegrated.preintMeasCov();

  const auto reducedNees = computeReducedNees(error, covariance);
  if (!reducedNees) {
    return nullopt;
  }
  return makeMethodSample(error, covariance, *reducedNees);
}

MethodSummary runQuadratureNees(const vector<Window>& windows, size_t quadratureNodes) {

  MethodSummary summary;
  summary.nodeCount = quadratureNodes;
  vector<double> reducedNeesValues;
  vector<double> rotErrorNorms;
  vector<double> rotPredSigmas;
  vector<double> posErrorNorms;
  vector<double> posPredSigmas;
  vector<double> velErrorNorms;
  vector<double> velPredSigmas;
  const auto params = createPreintegrationParams();

  for (const auto& window : windows) {
    const auto sample = analyzeQuadratureWindow(window, params, quadratureNodes);
    if (!sample) {
      continue;
    }
    appendMethodSample(*sample, reducedNeesValues, rotErrorNorms, rotPredSigmas,
                       posErrorNorms, posPredSigmas, velErrorNorms, velPredSigmas);
  }

  summary.nees = summarizeNees(reducedNeesValues);
  summary.rotErrorMedian = summarizeMedian(rotErrorNorms);
  summary.rotPredSigmaMedian = summarizeMedian(rotPredSigmas);
  summary.posErrorMedian = summarizeMedian(posErrorNorms);
  summary.posPredSigmaMedian = summarizeMedian(posPredSigmas);
  summary.velErrorMedian = summarizeMedian(velErrorNorms);
  summary.velPredSigmaMedian = summarizeMedian(velPredSigmas);
  summary.samples = summary.nees.samples;
  return summary;
}

template <class PIMType>
MethodSummary runStandardNees(const vector<Window>& windows) {
  MethodSummary summary;
  vector<double> reducedNeesValues;
  vector<double> rotErrorNorms;
  vector<double> rotPredSigmas;
  vector<double> posErrorNorms;
  vector<double> posPredSigmas;
  vector<double> velErrorNorms;
  vector<double> velPredSigmas;
  const auto params = createPreintegrationParams();

  for (const auto& window : windows) {
    const auto sample = analyzeStandardWindow<PIMType>(window, params);
    if (!sample) {
      continue;
    }
    appendMethodSample(*sample, reducedNeesValues, rotErrorNorms, rotPredSigmas,
                       posErrorNorms, posPredSigmas, velErrorNorms, velPredSigmas);
  }

  summary.nees = summarizeNees(reducedNeesValues);
  summary.rotErrorMedian = summarizeMedian(rotErrorNorms);
  summary.rotPredSigmaMedian = summarizeMedian(rotPredSigmas);
  summary.posErrorMedian = summarizeMedian(posErrorNorms);
  summary.posPredSigmaMedian = summarizeMedian(posPredSigmas);
  summary.velErrorMedian = summarizeMedian(velErrorNorms);
  summary.velPredSigmaMedian = summarizeMedian(velPredSigmas);
  summary.samples = summary.nees.samples;
  return summary;
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

    vector<Row> rows;
    for (const auto& datasetEntry : datasets) {
      const string& datasetName = datasetEntry.first;
      const string& datasetPath = datasetEntry.second;
      Dataset dataset(datasetPath);
      const auto& states = dataset.getStates();
      if (states.size() < 2) {
        continue;
      }
      for (double intervalSeconds : intervals) {
        const size_t samplesPerWindow = dataset.stepsForInterval(intervalSeconds);
        const auto windows = dataset.completeWindows(samplesPerWindow);
        const size_t quadratureNodes = max<size_t>(
            3, static_cast<size_t>(floor(sqrt(static_cast<double>(samplesPerWindow)))));

        Row row;
        row.dataset = datasetName;
        row.interval = intervalSeconds;
        row.samplesPerWindow = samplesPerWindow;
        row.quadratureNodes = quadratureNodes;
        row.quadrature = runQuadratureNees(windows, quadratureNodes);
        row.manifold = runStandardNees<PIMManifold>(windows);
        row.tangent = runStandardNees<PIMTangent>(windows);
        rows.push_back(row);

        cout << "[done] " << datasetName << " dt=" << fixed << setprecision(1) << intervalSeconds
             << "s m=" << samplesPerWindow << " N=" << quadratureNodes
             << " NEES_median=" << setprecision(3) << row.quadrature.nees.median << "\n";
      }
    }

    cout << "\n## Table 1: NEES Median (full 9-DOF, reduced)\n\n";
    cout << "| dataset | dt(s) | m | N | quad | manifold | tangent |\n";
    cout << "|---|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& row : rows) {
      cout << "| " << row.dataset << " | " << fixed << setprecision(1) << row.interval << " | "
           << row.samplesPerWindow << " | " << row.quadratureNodes << " | " << setprecision(3)
           << row.quadrature.nees.median << " | " << row.manifold.nees.median << " | "
           << row.tangent.nees.median << " |\n";
    }

    cout << "\n## Table 1b: Quad Error vs Predicted Sigma (median)\n\n";
    cout << "| dataset | dt(s) | m | N | NEES"
         << " | err_rot | sig_rot | err_pos | sig_pos | err_vel | sig_vel |\n";
    cout << "|---|---:|---:|---:|---:"
         << "|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& row : rows) {
      cout << "| " << row.dataset << " | " << fixed << setprecision(1) << row.interval << " | "
           << row.samplesPerWindow << " | " << row.quadratureNodes << " | " << setprecision(3)
           << row.quadrature.nees.median << " | " << setprecision(6)
           << row.quadrature.rotErrorMedian << " | " << row.quadrature.rotPredSigmaMedian << " | "
           << row.quadrature.posErrorMedian << " | " << row.quadrature.posPredSigmaMedian << " | "
           << row.quadrature.velErrorMedian << " | " << row.quadrature.velPredSigmaMedian
           << " |\n";
    }

    cout << "\n## Table 2: Median Errors per Dataset\n\n";
    cout << "| dataset | dt(s) | m | N"
         << " | q_rot | m_rot | t_rot"
         << " | q_pos | m_pos | t_pos"
         << " | q_vel | m_vel | t_vel |\n";
    cout << "|---|---:|---:|---:"
         << "|---:|---:|---:"
         << "|---:|---:|---:"
         << "|---:|---:|---:|\n";
    for (const auto& row : rows) {
      cout << "| " << row.dataset << " | " << fixed << setprecision(1) << row.interval << " | "
           << row.samplesPerWindow << " | " << row.quadratureNodes << " | " << setprecision(4)
           << row.quadrature.rotErrorMedian << " | " << row.manifold.rotErrorMedian << " | "
           << row.tangent.rotErrorMedian << " | " << row.quadrature.posErrorMedian << " | "
           << row.manifold.posErrorMedian << " | " << row.tangent.posErrorMedian << " | "
           << row.quadrature.velErrorMedian << " | " << row.manifold.velErrorMedian << " | "
           << row.tangent.velErrorMedian << " |\n";
    }

    cout << "\n## Table 3: Aggregated Mean-of-Median Errors\n\n";
    cout << "| dt(s) | m | N"
         << " | q_rot | m_rot | t_rot"
         << " | q_pos | m_pos | t_pos"
         << " | q_vel | m_vel | t_vel |\n";
    cout << "|---:|---:|---:"
         << "|---:|---:|---:"
         << "|---:|---:|---:"
         << "|---:|---:|---:|\n";
    for (double intervalSeconds : intervals) {
      double qRot = 0.0;
      double mRot = 0.0;
      double tRot = 0.0;
      double qPos = 0.0;
      double mPos = 0.0;
      double tPos = 0.0;
      double qVel = 0.0;
      double mVel = 0.0;
      double tVel = 0.0;
      size_t count = 0;
      size_t samplesPerWindow = 0;
      size_t quadratureNodes = 0;

      for (const auto& row : rows) {
        if (row.interval != intervalSeconds) {
          continue;
        }
        samplesPerWindow = row.samplesPerWindow;
        quadratureNodes = row.quadratureNodes;
        qRot += row.quadrature.rotErrorMedian;
        mRot += row.manifold.rotErrorMedian;
        tRot += row.tangent.rotErrorMedian;
        qPos += row.quadrature.posErrorMedian;
        mPos += row.manifold.posErrorMedian;
        tPos += row.tangent.posErrorMedian;
        qVel += row.quadrature.velErrorMedian;
        mVel += row.manifold.velErrorMedian;
        tVel += row.tangent.velErrorMedian;
        ++count;
      }

      if (count == 0) {
        continue;
      }
      const double sampleCount = static_cast<double>(count);
      cout << "| " << fixed << setprecision(1) << intervalSeconds << " | " << samplesPerWindow << " | "
           << quadratureNodes << " | " << setprecision(4) << qRot / sampleCount << " | "
           << mRot / sampleCount << " | " << tRot / sampleCount << " | " << qPos / sampleCount
           << " | " << mPos / sampleCount << " | " << tPos / sampleCount << " | "
           << qVel / sampleCount << " | " << mVel / sampleCount << " | " << tVel / sampleCount
           << " |\n";
    }

    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
