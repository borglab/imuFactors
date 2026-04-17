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
 * @brief  Minimal NEES and error analysis for Quadrature, Manifold, and Tangent
 * IMU preintegration
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

// Scale the nominal sensor noise to the level used in this diagnostic sweep.
constexpr double kAlpha = 8.4;
constexpr double kSigmaGyro = kAlpha * 1.6968e-4;
constexpr double kSigmaAcc = kAlpha * 2.0000e-3;

// Summaries of the normalized-NEES distribution across all valid windows.
struct NeesStats {
  double mean = 0.0;
  double median = 0.0;
  double p95 = 0.0;
  size_t samples = 0;
};

// Final per-method aggregates printed in the markdown tables.
struct Summary {
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

// Raw per-window measurements before they are collapsed into medians.
struct MethodSample {
  double normalizedNees = 0.0;
  double rotErrorNorm = 0.0;
  double rotPredSigma = 0.0;
  double posErrorNorm = 0.0;
  double posPredSigma = 0.0;
  double velErrorNorm = 0.0;
  double velPredSigma = 0.0;
};

// Hold all samples for one method and summarize them in one place.
struct MethodSamples {
  vector<MethodSample> values;

  void append(const MethodSample& sample) { values.push_back(sample); }

  Summary summarize(size_t nodeCount = 0) const;
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

// Build the shared IMU noise model used by all three preintegration methods.
shared_ptr<PreintegrationParams> createPreintegrationParams() {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = I_3x3 * kSigmaAcc * kSigmaAcc;
  params->gyroscopeCovariance = I_3x3 * kSigmaGyro * kSigmaGyro;
  params->integrationCovariance = I_3x3 * 1e-8;
  return params;
}

double covarianceBlockSigma(const Matrix9& covariance, int blockStart) {
  // Convert a 3x3 covariance block into a scalar sigma proxy via RMS variance.
  return std::sqrt(std::max(
      0.0, covariance.block<3, 3>(blockStart, blockStart).trace() / 3.0));
}

NeesStats summarizeNormalizedNees(const vector<double>& normalizedNeesValues) {
  NeesStats stats;
  if (normalizedNeesValues.empty()) {
    return stats;
  }
  // Report the same normalized-NEES distribution using several robust summary
  // statistics.
  stats.samples = normalizedNeesValues.size();
  stats.mean = computeMean(normalizedNeesValues);
  stats.median = computeMedian(normalizedNeesValues);
  stats.p95 = computePercentile(normalizedNeesValues, 95.0);
  return stats;
}

double summarizeMedian(const vector<double>& values) {
  return computeMedian(values);
}

Summary MethodSamples::summarize(size_t nodeCount) const {
  Summary summary;
  summary.nodeCount = nodeCount;

  // Project one field across all windows so we can reuse the scalar reducers.
  const auto project = [this](auto member) {
    vector<double> projectedValues;
    projectedValues.reserve(values.size());
    for (const auto& sample : values) {
      projectedValues.push_back(sample.*member);
    }
    return projectedValues;
  };

  const vector<double> normalizedNeesValues =
      project(&MethodSample::normalizedNees);
  summary.nees = summarizeNormalizedNees(normalizedNeesValues);
  summary.rotErrorMedian =
      summarizeMedian(project(&MethodSample::rotErrorNorm));
  summary.rotPredSigmaMedian =
      summarizeMedian(project(&MethodSample::rotPredSigma));
  summary.posErrorMedian =
      summarizeMedian(project(&MethodSample::posErrorNorm));
  summary.posPredSigmaMedian =
      summarizeMedian(project(&MethodSample::posPredSigma));
  summary.velErrorMedian =
      summarizeMedian(project(&MethodSample::velErrorNorm));
  summary.velPredSigmaMedian =
      summarizeMedian(project(&MethodSample::velPredSigma));
  summary.samples = summary.nees.samples;
  return summary;
}

MethodSample makeMethodSample(const Vector9& error, const Matrix9& covariance,
                              double normalizedNees) {
  MethodSample sample;
  sample.normalizedNees = normalizedNees;
  sample.rotErrorNorm = error.head<3>().norm();
  sample.rotPredSigma = covarianceBlockSigma(covariance, 0);
  sample.posErrorNorm = error.segment<3>(3).norm();
  sample.posPredSigma = covarianceBlockSigma(covariance, 3);
  sample.velErrorNorm = error.tail<3>().norm();
  sample.velPredSigma = covarianceBlockSigma(covariance, 6);
  return sample;
}

optional<MethodSample> analyzeQuadratureWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params,
    size_t N) {
  // Run quadrature preintegration on one window and close the integration
  // interval.
  const PIMQuadrature preintegrated = buildPreintegrated<PIMQuadrature>(
      window, params, window.initialTruth().bias, N);

  // Evaluate the factor at ground truth to get the residual and predicted
  // covariance.
  const Matrix9 covariance = preintegrated.preintMeasCov();
  QuadratureImuFactor factor(X(1), V(1), X(2), V(2), B(1), preintegrated);
  const Vector9 error = factor.evaluateError(
      window.initialTruth().navState.pose(),
      window.initialTruth().navState.velocity(),
      window.terminalTruth().navState.pose(),
      window.terminalTruth().navState.velocity(), window.initialTruth().bias);

  const auto normalizedNees = normalizedNEES(error, covariance, 9.0);
  if (!normalizedNees || !std::isfinite(*normalizedNees)) {
    return nullopt;
  }
  return makeMethodSample(error, covariance, *normalizedNees);
}

template <class PIMType>
optional<MethodSample> analyzeStandardWindow(
    const Window& window, const shared_ptr<PreintegrationParams>& params) {
  // Run the standard manifold or tangent preintegration on one window.
  const PIMType preintegrated =
      buildPreintegrated<PIMType>(window, params, window.initialTruth().bias);

  // The standard factor exposes the same 9D residual, so we summarize it
  // identically.
  ImuFactor2T<PIMType> factor(X(1), X(2), B(1), preintegrated);
  const Vector9 error = factor.evaluateError(window.initialTruth().navState,
                                             window.terminalTruth().navState,
                                             window.initialTruth().bias);
  const Matrix9 covariance = preintegrated.preintMeasCov();

  const auto normalizedNees = normalizedNEES(error, covariance, 9.0);
  if (!normalizedNees || !std::isfinite(*normalizedNees)) {
    return nullopt;
  }
  return makeMethodSample(error, covariance, *normalizedNees);
}

Summary runQuadratureNees(const vector<Window>& windows,
                          size_t quadratureNodes) {
  MethodSamples samples;
  const auto params = createPreintegrationParams();

  // Accumulate one sample per valid window for the quadrature method.
  for (const auto& window : windows) {
    const auto sample =
        analyzeQuadratureWindow(window, params, quadratureNodes);
    if (sample) samples.append(*sample);
  }

  return samples.summarize(quadratureNodes);
}

template <class PIMType>
Summary runStandardNees(const vector<Window>& windows) {
  MethodSamples samples;
  const auto params = createPreintegrationParams();

  // Accumulate one sample per valid window for the chosen standard method.
  for (const auto& window : windows) {
    const auto sample = analyzeStandardWindow<PIMType>(window, params);
    if (sample) samples.append(*sample);
  }

  return samples.summarize();
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

    // Evaluate all requested datasets and window lengths.
    vector<Row> rows;
    for (const auto& datasetEntry : datasets) {
      const string& datasetPath = datasetEntry.second;
      Dataset dataset(datasetPath);
      if (dataset.truth.size() < 2) continue;
      const string& datasetName = datasetEntry.first;

      for (double intervalSeconds : intervals) {
        const size_t m = dataset.stepsForInterval(intervalSeconds);
        const size_t N = max<size_t>(
            3, static_cast<size_t>(floor(sqrt(static_cast<double>(m)))));

        // Compare quadrature against the two standard preintegration variants.
        Row row;
        row.dataset = datasetName;
        row.interval = intervalSeconds;
        row.samplesPerWindow = m;
        row.quadratureNodes = N;
        const auto windows = dataset.completeWindows(m);
        row.quadrature = runQuadratureNees(windows, N);
        row.manifold = runStandardNees<PIMManifold>(windows);
        row.tangent = runStandardNees<PIMTangent>(windows);
        rows.push_back(row);

        cout << "[done] " << datasetName << " dt=" << fixed << setprecision(1)
             << intervalSeconds << "s m=" << m << " N=" << N
             << " NEES_median=" << setprecision(3) << row.quadrature.nees.median
             << "\n";
      }
    }

    // Table 1 is the direct normalized-NEES comparison across the three
    // methods.
    cout << "\n## Table 1: Normalized NEES Median (full 9-DOF)\n\n";
    cout << "| dataset | dt(s) | m | N | quad | manifold | tangent |\n";
    cout << "|---|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& row : rows) {
      cout << "| " << row.dataset << " | " << fixed << setprecision(1)
           << row.interval << " | " << row.samplesPerWindow << " | "
           << row.quadratureNodes << " | " << setprecision(3)
           << row.quadrature.nees.median << " | " << row.manifold.nees.median
           << " | " << row.tangent.nees.median << " |\n";
    }

    // Table 1b exposes how quadrature's observed errors compare to its
    // covariance scale.
    cout << "\n## Table 1b: Quad Error vs Predicted Sigma (median)\n\n";
    cout << "| dataset | dt(s) | m | N | NEES"
         << " | err_rot | sig_rot | err_pos | sig_pos | err_vel | sig_vel |\n";
    cout << "|---|---:|---:|---:|---:"
         << "|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& row : rows) {
      cout << "| " << row.dataset << " | " << fixed << setprecision(1)
           << row.interval << " | " << row.samplesPerWindow << " | "
           << row.quadratureNodes << " | " << setprecision(3)
           << row.quadrature.nees.median << " | " << setprecision(6)
           << row.quadrature.rotErrorMedian << " | "
           << row.quadrature.rotPredSigmaMedian << " | "
           << row.quadrature.posErrorMedian << " | "
           << row.quadrature.posPredSigmaMedian << " | "
           << row.quadrature.velErrorMedian << " | "
           << row.quadrature.velPredSigmaMedian << " |\n";
    }

    // Table 2 compares median error magnitudes directly for each method.
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
      cout << "| " << row.dataset << " | " << fixed << setprecision(1)
           << row.interval << " | " << row.samplesPerWindow << " | "
           << row.quadratureNodes << " | " << setprecision(4)
           << row.quadrature.rotErrorMedian << " | "
           << row.manifold.rotErrorMedian << " | " << row.tangent.rotErrorMedian
           << " | " << row.quadrature.posErrorMedian << " | "
           << row.manifold.posErrorMedian << " | " << row.tangent.posErrorMedian
           << " | " << row.quadrature.velErrorMedian << " | "
           << row.manifold.velErrorMedian << " | " << row.tangent.velErrorMedian
           << " |\n";
    }

    // Table 3 averages the per-dataset medians to get one row per interval.
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

      // Gather all rows that share this interval across the selected datasets.
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
      // Normalize the accumulated medians by the number of contributing
      // datasets.
      const double sampleCount = static_cast<double>(count);
      cout << "| " << fixed << setprecision(1) << intervalSeconds << " | "
           << samplesPerWindow << " | " << quadratureNodes << " | "
           << setprecision(4) << qRot / sampleCount << " | "
           << mRot / sampleCount << " | " << tRot / sampleCount << " | "
           << qPos / sampleCount << " | " << mPos / sampleCount << " | "
           << tPos / sampleCount << " | " << qVel / sampleCount << " | "
           << mVel / sampleCount << " | " << tVel / sampleCount << " |\n";
    }

    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
