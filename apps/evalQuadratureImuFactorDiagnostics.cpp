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

// Hold all samples for one method and summarize them in one place.
struct MethodSamples {
  vector<WindowResult> values;

  void append(const WindowResult& sample) { values.push_back(sample); }

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
      project(&WindowResult::normalizedNees);
  summary.nees = summarizeNormalizedNees(normalizedNeesValues);
  summary.rotErrorMedian =
      summarizeMedian(project(&WindowResult::rotErrorNorm));
  summary.rotPredSigmaMedian =
      summarizeMedian(project(&WindowResult::rotPredSigma));
  summary.posErrorMedian =
      summarizeMedian(project(&WindowResult::posErrorNorm));
  summary.posPredSigmaMedian =
      summarizeMedian(project(&WindowResult::posPredSigma));
  summary.velErrorMedian =
      summarizeMedian(project(&WindowResult::velErrorNorm));
  summary.velPredSigmaMedian =
      summarizeMedian(project(&WindowResult::velPredSigma));
  summary.samples = summary.nees.samples;
  return summary;
}

Summary runQuadratureNees(const vector<Window>& windows,
                          size_t quadratureNodes) {
  MethodSamples samples;
  const auto params = createPreintegrationParams();

  // Accumulate one sample per valid window for the quadrature method.
  for (const auto& window : windows) {
    const auto sample = evaluateWindow<PIMQuadrature>(
        window, params, std::nullopt, quadratureNodes);
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
    const auto sample = evaluateWindow<PIMType>(window, params);
    if (sample) samples.append(*sample);
  }

  return samples.summarize();
}

struct AppOptions {
  size_t maxIntervals = 0;
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

  // Evaluate all requested datasets and window lengths.
  RunForDataset runner(intervals);
  const int status = runForDatasets(datasetCli, runner);
  if (status != 0) {
    return status;
  }
  const vector<Row>& rows = runner.rows;

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
  for (double intervalSeconds : runner.intervals) {
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
         << setprecision(4) << qRot / sampleCount << " | " << mRot / sampleCount
         << " | " << tRot / sampleCount << " | " << qPos / sampleCount << " | "
         << mPos / sampleCount << " | " << tPos / sampleCount << " | "
         << qVel / sampleCount << " | " << mVel / sampleCount << " | "
         << tVel / sampleCount << " |\n";
  }

  return 0;
}
