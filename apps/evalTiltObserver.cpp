/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved.
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalTiltObserver.cpp
 * @brief  Run the tilt/gyro-bias observer on each EuRoC dataset.
 */

#include <gtsam/geometry/Unit3.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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
constexpr double kDefaultSpecificForceToleranceG = 0.3;
constexpr double kGravityMagnitude = 9.81;

struct AppOptions {
  AppCliOptions datasetOptions;
  double tauP = kDefaultTauP;
  double tauI = kDefaultTauI;
  double specificForceToleranceG = kDefaultSpecificForceToleranceG;
  bool initializeFromGroundTruth = false;
};

void printUsage(const char* programName) {
  printDatasetAppUsage(programName);
  cout << "  --tau-p <seconds>       Proportional time constant (default: "
       << kDefaultTauP << ")\n";
  cout << "  --tau-i <seconds>       Integral time constant (default: "
       << kDefaultTauI << ")\n";
  cout << "  --specific-force-tolerance-g <fraction> Accept accelerometer "
          "updates within this fraction of g (default: "
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

string tiltObserverHeader() {
  return "run_id,app_name,dataset,sample_index,timestamp,tau_p,tau_i,"
         "gt_roll_rad,gt_pitch_rad,est_roll_rad,est_pitch_rad,"
         "gt_gyro_bias_x,gt_gyro_bias_y,gt_gyro_bias_z,"
         "est_gyro_bias_x,est_gyro_bias_y,est_gyro_bias_z,"
         "omega_x,omega_y,omega_z,acc_x,acc_y,acc_z";
}

void writeTiltObserverRow(ofstream& stream, const ResultsWriter& writer,
                          const string& datasetName, size_t sampleIndex,
                          double timestamp, double tauP, double tauI,
                          const Vector2& gtRollPitch,
                          const Vector2& estRollPitch,
                          const Vector3& gtGyroBias, const Vector3& estGyroBias,
                          const Dataset::ImuMeasurement& imuMeasurement) {
  stream << csvEscape(writer.runId()) << "," << csvEscape(writer.appName())
         << "," << csvEscape(datasetName) << "," << sampleIndex << ","
         << timestamp << "," << tauP << "," << tauI << "," << gtRollPitch.x()
         << "," << gtRollPitch.y() << "," << estRollPitch.x() << ","
         << estRollPitch.y() << "," << gtGyroBias.x() << "," << gtGyroBias.y()
         << "," << gtGyroBias.z() << "," << estGyroBias.x() << ","
         << estGyroBias.y() << "," << estGyroBias.z() << ","
         << imuMeasurement.omega.x() << "," << imuMeasurement.omega.y() << ","
         << imuMeasurement.omega.z() << "," << imuMeasurement.acc.x() << ","
         << imuMeasurement.acc.y() << "," << imuMeasurement.acc.z() << "\n";
  if (!stream.good()) {
    throw runtime_error("Failed to write tilt observer CSV row.");
  }
}

void runDataset(const ResultsWriter& writer, const string& datasetName,
                const Dataset& dataset, double tauP, double tauI,
                double specificForceToleranceG,
                bool initializeFromGroundTruth) {
  const filesystem::path outputPath =
      writer.runDirectory() / ("tilt_observer_" + datasetName + ".csv");
  ofstream stream(outputPath);
  if (!stream.is_open()) {
    throw runtime_error("Failed to open tilt observer CSV: " +
                        outputPath.string());
  }
  stream << setprecision(17);
  stream << tiltObserverHeader() << "\n";

  const size_t sampleCount = min(dataset.truth.size(), dataset.imu.size());
  const double specificForceTolerance =
      specificForceToleranceG * kGravityMagnitude;
  const Unit3 initialUpDirection(
      dataset.truth.front().navState.attitude().matrix().transpose() *
      Vector3::UnitZ());
  TiltObserver observer =
      initializeFromGroundTruth
          ? TiltObserver(tauP, tauI, dataset.timestep(),
                         Unit3(-initialUpDirection.point3()),
                         dataset.truth.front().bias.gyroscope(),
                         kGravityMagnitude, specificForceTolerance)
          : TiltObserver(tauP, tauI, dataset.timestep(), kGravityMagnitude,
                         specificForceTolerance);
  for (size_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
    const auto& imuMeasurement = dataset.imu[sampleIndex];
    const auto& truth = dataset.truth[sampleIndex];

    observer(imuMeasurement.omega, imuMeasurement.acc);
    if (!observer.n_hat || !observer.b_hat) {
      continue;
    }

    const Unit3 gtUpDirection(truth.navState.attitude().matrix().transpose() *
                              Vector3::UnitZ());
    const Vector2 gtRollPitch = rollPitchFromUpDirection(gtUpDirection);
    const Vector2 estRollPitch =
        rollPitchFromGravityDirection(observer.n_hat.value());
    writeTiltObserverRow(stream, writer, datasetName, sampleIndex,
                         truth.timestamp, tauP, tauI, gtRollPitch, estRollPitch,
                         truth.bias.gyroscope(), observer.b_hat.value(),
                         imuMeasurement);
  }
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

    const string datasetGroup = resolvedDatasetGroupLabel(datasetCli);
    for (const auto& [datasetName, datasetPath] : datasetCli.datasets) {
      Dataset dataset(datasetPath);
      if (dataset.truth.size() < 2) {
        continue;
      }
      writer.writeDataset(
          makeDatasetRow(writer, datasetName, dataset, datasetGroup));
      runDataset(writer, datasetName, dataset, options.tauP, options.tauI,
                 options.specificForceToleranceG,
                 options.initializeFromGroundTruth);
      cout << "Wrote " << datasetName << "\n";
    }

    cout << "Results written to " << writer.runDirectory() << "\n";
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
