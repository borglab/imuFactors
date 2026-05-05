/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved.
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalPimTiltObserver.cpp
 * @brief  Compare IMU-rate and PIM-window-rate tilt observer updates.
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/PreintegrationParams.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "ResultsAdapters.h"
#include "ResultsSchema.h"
#include "ResultsWriter.h"
#include "TiltObserver.h"

using namespace gtsam;
using namespace std;

namespace {

constexpr double kDefaultTauP = 5.0;
constexpr double kDefaultTauI = 6.0;
constexpr double kDefaultIntervalSeconds = 0.2;
constexpr double kDefaultSpecificForceToleranceG = 0.3;
constexpr double kGravityMagnitude = 9.81;

using ManifoldPim = PreintegratedImuMeasurementsT<ManifoldPreintegration>;

struct AppOptions {
  AppCliOptions datasetOptions;
  double tauP = kDefaultTauP;
  double tauI = kDefaultTauI;
  double intervalSeconds = kDefaultIntervalSeconds;
  double specificForceToleranceG = kDefaultSpecificForceToleranceG;
  bool initializeFromGroundTruth = false;
};

struct WindowAverage {
  Vector3 omega = Vector3::Zero();
  Vector3 acc = Vector3::Zero();
  size_t sampleCount = 0;
};

struct ErrorAccumulator {
  size_t count = 0;
  double rollSquared = 0.0;
  double pitchSquared = 0.0;
  double biasSquared = 0.0;

  void add(double rollError, double pitchError, const Vector3& biasError) {
    ++count;
    rollSquared += rollError * rollError;
    pitchSquared += pitchError * pitchError;
    biasSquared += biasError.squaredNorm();
  }

  double rollRmse() const { return count == 0 ? 0.0 : sqrt(rollSquared / count); }
  double pitchRmse() const {
    return count == 0 ? 0.0 : sqrt(pitchSquared / count);
  }
  double biasVectorRmse() const {
    return count == 0 ? 0.0 : sqrt(biasSquared / count);
  }
};

struct MethodSummary {
  string dataset;
  string method;
  ErrorAccumulator errors;
};

void printUsage(const char* programName) {
  printDatasetAppUsage(programName);
  cout << "  --tau-p <seconds>       Proportional time constant (default: "
       << kDefaultTauP << ")\n";
  cout << "  --tau-i <seconds>       Integral time constant (default: "
       << kDefaultTauI << ")\n";
  cout << "  --interval <seconds>    PIM window interval (default: "
       << kDefaultIntervalSeconds << ")\n";
  cout << "  --specific-force-tolerance-g <fraction> Accept averaged "
          "specific-force updates within this fraction of g (default: "
       << kDefaultSpecificForceToleranceG << ")\n";
  cout << "  --initialize-from-ground-truth Seed tilt and gyro bias from the "
          "first ground-truth state\n";
}

AppOptions parseArguments(int argc, char* argv[]) {
  const ParsedAppCliOptions parsed = parseDatasetAppCliOptions(argc, argv);
  AppOptions options;
  options.datasetOptions = parsed.options;

  for (size_t index = 0; index < parsed.remainingArgs.size(); ++index) {
    const string& argument = parsed.remainingArgs[index];
    if (isHelpArgument(argument)) {
      printUsage(argv[0]);
      exit(0);
    }
    if (argument == "--tau-p") {
      if (index + 1 >= parsed.remainingArgs.size()) {
        throw runtime_error("Missing value for --tau-p");
      }
      options.tauP =
          parsePositiveDoubleOption("--tau-p", parsed.remainingArgs[++index]);
      continue;
    }
    if (argument == "--tau-i") {
      if (index + 1 >= parsed.remainingArgs.size()) {
        throw runtime_error("Missing value for --tau-i");
      }
      options.tauI =
          parsePositiveDoubleOption("--tau-i", parsed.remainingArgs[++index]);
      continue;
    }
    if (argument == "--interval") {
      if (index + 1 >= parsed.remainingArgs.size()) {
        throw runtime_error("Missing value for --interval");
      }
      options.intervalSeconds =
          parsePositiveDoubleOption("--interval", parsed.remainingArgs[++index]);
      continue;
    }
    if (argument == "--specific-force-tolerance-g") {
      if (index + 1 >= parsed.remainingArgs.size()) {
        throw runtime_error("Missing value for --specific-force-tolerance-g");
      }
      options.specificForceToleranceG = parsePositiveDoubleOption(
          "--specific-force-tolerance-g", parsed.remainingArgs[++index]);
      continue;
    }
    if (argument == "--initialize-from-ground-truth") {
      options.initializeFromGroundTruth = true;
      continue;
    }
    throw runtime_error("Unknown argument: " + argument);
  }

  return options;
}

string pimTiltObserverHeader() {
  return "run_id,app_name,dataset,method,interval_seconds,samples_per_window,"
         "window_index,window_start_sample,window_end_sample,"
         "window_start_time,window_end_time,timestamp,tau_p,tau_i,"
         "gt_roll_rad,gt_pitch_rad,est_roll_rad,est_pitch_rad,"
         "roll_error_rad,pitch_error_rad,"
         "gt_gyro_bias_x,gt_gyro_bias_y,gt_gyro_bias_z,"
         "est_gyro_bias_x,est_gyro_bias_y,est_gyro_bias_z,"
         "bias_error_norm,avg_omega_x,avg_omega_y,avg_omega_z,"
         "avg_acc_x,avg_acc_y,avg_acc_z,"
         "delta_rot_x,delta_rot_y,delta_rot_z";
}

string pimTiltObserverSummaryHeader() {
  return "run_id,app_name,dataset,method,interval_seconds,samples_per_window,"
         "num_windows,roll_rmse_rad,pitch_rmse_rad,bias_vector_rmse";
}

double wrapAngle(double angle) { return atan2(sin(angle), cos(angle)); }

Unit3 groundTruthGravityDirection(const Dataset::Truth& truth) {
  const Unit3 upDirection(truth.navState.attitude().matrix().transpose() *
                          Vector3::UnitZ());
  return Unit3(-upDirection.point3());
}

Vector2 groundTruthRollPitch(const Dataset::Truth& truth) {
  return rollPitchFromGravityDirection(groundTruthGravityDirection(truth));
}

TiltObserver makeObserver(const Dataset& dataset, const AppOptions& options,
                          double dt) {
  const double specificForceTolerance =
      options.specificForceToleranceG * kGravityMagnitude;
  if (options.initializeFromGroundTruth) {
    return TiltObserver(options.tauP, options.tauI, dt,
                        groundTruthGravityDirection(dataset.truth.front()),
                        dataset.truth.front().bias.gyroscope(),
                        kGravityMagnitude, specificForceTolerance);
  }
  return TiltObserver(options.tauP, options.tauI, dt, kGravityMagnitude,
                      specificForceTolerance);
}

WindowAverage averageWindowMeasurements(const Window& window) {
  WindowAverage average;
  window.forEachImuSample(
      [&average](const Dataset::ImuMeasurement& measurement) {
        average.omega += measurement.omega;
        average.acc += measurement.acc;
        ++average.sampleCount;
      });
  if (average.sampleCount == 0) {
    throw runtime_error("Cannot average an empty window.");
  }
  const double scale = 1.0 / static_cast<double>(average.sampleCount);
  average.omega *= scale;
  average.acc *= scale;
  return average;
}

Rot3 pimDeltaRotation(const Window& window,
                      const shared_ptr<PreintegrationParams>& params,
                      const Vector3& gyroBias) {
  ManifoldPim pim(params, imuBias::ConstantBias(Vector3::Zero(), gyroBias));
  window.integrateMeasurements(pim);
  return pim.deltaRij();
}

void writeObserverRow(ofstream& stream, const ResultsWriter& writer,
                      const string& datasetName, const string& method,
                      double intervalSeconds, size_t samplesPerWindow,
                      size_t windowIndex, const Window& window, double tauP,
                      double tauI, const Vector2& gtRollPitch,
                      const Vector2& estRollPitch, double rollError,
                      double pitchError, const Vector3& gtGyroBias,
                      const Vector3& estGyroBias,
                      const WindowAverage& windowAverage,
                      const Vector3& deltaRotationLog) {
  const Vector3 biasError = estGyroBias - gtGyroBias;
  stream << csvEscape(writer.runId()) << "," << csvEscape(writer.appName())
         << "," << csvEscape(datasetName) << "," << csvEscape(method) << ","
         << intervalSeconds << "," << samplesPerWindow << "," << windowIndex
         << "," << window.start << "," << window.end << ","
         << window.initialTruth().timestamp << ","
         << window.terminalTruth().timestamp << ","
         << window.terminalTruth().timestamp << "," << tauP << "," << tauI
         << "," << gtRollPitch.x() << "," << gtRollPitch.y() << ","
         << estRollPitch.x() << "," << estRollPitch.y() << "," << rollError
         << "," << pitchError << "," << gtGyroBias.x() << ","
         << gtGyroBias.y() << "," << gtGyroBias.z() << "," << estGyroBias.x()
         << "," << estGyroBias.y() << "," << estGyroBias.z() << ","
         << biasError.norm() << "," << windowAverage.omega.x() << ","
         << windowAverage.omega.y() << "," << windowAverage.omega.z() << ","
         << windowAverage.acc.x() << "," << windowAverage.acc.y() << ","
         << windowAverage.acc.z() << "," << deltaRotationLog.x() << ","
         << deltaRotationLog.y() << "," << deltaRotationLog.z() << "\n";
  if (!stream.good()) {
    throw runtime_error("Failed to write PIM tilt observer CSV row.");
  }
}

void addObserverResult(const ResultsWriter& writer, ofstream& stream,
                       MethodSummary* summary, const string& datasetName,
                       const string& method, double intervalSeconds,
                       size_t samplesPerWindow, size_t windowIndex,
                       const Window& window, double tauP, double tauI,
                       const TiltObserver& observer,
                       const WindowAverage& windowAverage,
                       const Vector3& deltaRotationLog) {
  if (!observer.n_hat || !observer.b_hat) {
    return;
  }

  const auto& truth = window.terminalTruth();
  const Vector2 gtRollPitch = groundTruthRollPitch(truth);
  const Vector2 estRollPitch =
      rollPitchFromGravityDirection(observer.n_hat.value());
  const double rollError = wrapAngle(estRollPitch.x() - gtRollPitch.x());
  const double pitchError = wrapAngle(estRollPitch.y() - gtRollPitch.y());
  const Vector3 gtGyroBias = truth.bias.gyroscope();
  const Vector3 estGyroBias = observer.b_hat.value();
  summary->errors.add(rollError, pitchError, estGyroBias - gtGyroBias);
  writeObserverRow(stream, writer, datasetName, method, intervalSeconds,
                   samplesPerWindow, windowIndex, window, tauP, tauI,
                   gtRollPitch, estRollPitch, rollError, pitchError,
                   gtGyroBias, estGyroBias, windowAverage, deltaRotationLog);
}

void writeSummaryRow(ofstream& stream, const ResultsWriter& writer,
                     const MethodSummary& summary, double intervalSeconds,
                     size_t samplesPerWindow) {
  stream << csvEscape(writer.runId()) << "," << csvEscape(writer.appName())
         << "," << csvEscape(summary.dataset) << ","
         << csvEscape(summary.method) << "," << intervalSeconds << ","
         << samplesPerWindow << "," << summary.errors.count << ","
         << summary.errors.rollRmse() << "," << summary.errors.pitchRmse()
         << "," << summary.errors.biasVectorRmse() << "\n";
  if (!stream.good()) {
    throw runtime_error("Failed to write PIM tilt observer summary row.");
  }
}

void runDataset(const ResultsWriter& writer, ofstream& summaryStream,
                const string& datasetName, const Dataset& dataset,
                const AppOptions& options) {
  const size_t samplesPerWindow =
      dataset.stepsForInterval(options.intervalSeconds);
  const double windowDt = static_cast<double>(samplesPerWindow) *
                          dataset.timestep();
  const auto windows = dataset.completeWindows(samplesPerWindow);
  const auto pimParams = PreintegrationParams::MakeSharedU(kGravityMagnitude);

  const filesystem::path outputPath =
      writer.runDirectory() / ("tilt_observer_pim_" + datasetName + ".csv");
  ofstream stream(outputPath);
  if (!stream.is_open()) {
    throw runtime_error("Failed to open PIM tilt observer CSV: " +
                        outputPath.string());
  }
  stream << setprecision(17);
  stream << pimTiltObserverHeader() << "\n";

  TiltObserver imuRateObserver =
      makeObserver(dataset, options, dataset.timestep());
  TiltObserver pimWindowObserver = makeObserver(dataset, options, windowDt);
  MethodSummary imuRateSummary{datasetName, "imu_rate", {}};
  MethodSummary pimWindowSummary{datasetName, "pim_window", {}};

  for (size_t windowIndex = 0; windowIndex < windows.size(); ++windowIndex) {
    const Window& window = windows[windowIndex];
    const WindowAverage windowAverage = averageWindowMeasurements(window);

    window.forEachImuSample(
        [&imuRateObserver](const Dataset::ImuMeasurement& measurement) {
          imuRateObserver(measurement.omega, measurement.acc);
        });

    if (!pimWindowObserver.b_hat) {
      pimWindowObserver.predict(windowAverage.omega);
    }
    const Vector3 pimGyroBias =
        pimWindowObserver.b_hat.value_or(Vector3::Zero());
    const Rot3 deltaRotation =
        pimDeltaRotation(window, pimParams, pimGyroBias);
    const Vector3 deltaRotationLog = Rot3::Logmap(deltaRotation);
    pimWindowObserver(deltaRotation, windowAverage.acc);

    addObserverResult(writer, stream, &imuRateSummary, datasetName, "imu_rate",
                      options.intervalSeconds, samplesPerWindow, windowIndex,
                      window, options.tauP, options.tauI, imuRateObserver,
                      windowAverage, deltaRotationLog);
    addObserverResult(writer, stream, &pimWindowSummary, datasetName,
                      "pim_window", options.intervalSeconds, samplesPerWindow,
                      windowIndex, window, options.tauP, options.tauI,
                      pimWindowObserver, windowAverage, deltaRotationLog);
  }

  writeSummaryRow(summaryStream, writer, imuRateSummary, options.intervalSeconds,
                  samplesPerWindow);
  writeSummaryRow(summaryStream, writer, pimWindowSummary,
                  options.intervalSeconds, samplesPerWindow);

  cout << "Wrote " << datasetName << " (" << windows.size() << " windows, "
       << samplesPerWindow << " samples/window)\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const AppOptions options = parseArguments(argc, argv);
    const ResolvedDatasetCli datasetCli{
        options.datasetOptions, resolveDatasets(options.datasetOptions), {}};
    if (datasetCli.datasets.empty()) {
      cerr << "No datasets found in " << options.datasetOptions.dataDirectory
           << "\n";
      return 1;
    }

    ResultsWriter writer(argv[0], options.datasetOptions.outputRoot);
    writeCanonicalRunMetadata(&writer, argc, argv);

    const filesystem::path summaryPath =
        writer.runDirectory() / "tilt_observer_pim_summaries.csv";
    ofstream summaryStream(summaryPath);
    if (!summaryStream.is_open()) {
      throw runtime_error("Failed to open PIM tilt observer summary CSV: " +
                          summaryPath.string());
    }
    summaryStream << setprecision(17);
    summaryStream << pimTiltObserverSummaryHeader() << "\n";

    const string datasetGroup = resolvedDatasetGroupLabel(datasetCli);
    for (const auto& [datasetName, datasetPath] : datasetCli.datasets) {
      Dataset dataset(datasetPath);
      if (dataset.truth.size() < 2) {
        continue;
      }
      writer.writeDataset(
          makeDatasetRow(writer, datasetName, dataset, datasetGroup));
      runDataset(writer, summaryStream, datasetName, dataset, options);
    }

    cout << "Results written to " << writer.runDirectory() << "\n";
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
