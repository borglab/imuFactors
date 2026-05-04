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
#include <cmath>
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

constexpr double kDefaultTauP = 0.25;
constexpr double kDefaultTauI = 3.0;
constexpr double kDefaultInitializationSeconds = 0.5;

struct AppOptions {
  AppCliOptions datasetOptions;
  double tauP = kDefaultTauP;
  double tauI = kDefaultTauI;
  double initializationSeconds = kDefaultInitializationSeconds;
};

void printUsage(const char* programName) {
  printDatasetAppUsage(programName);
  cout << "  --tau-p <seconds>       Proportional time constant (default: "
       << kDefaultTauP << ")\n";
  cout << "  --tau-i <seconds>       Integral time constant (default: "
       << kDefaultTauI << ")\n";
  cout << "  --init-seconds <seconds> Average initial accelerometer samples "
          "over this duration (default: "
       << kDefaultInitializationSeconds << ")\n";
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
    if (argument == "--init-seconds") {
      if (index + 1 >= parsed.remainingArgs.size()) {
        throw runtime_error("Missing value for --init-seconds");
      }
      options.initializationSeconds = parsePositiveDoubleOption(
          "--init-seconds", parsed.remainingArgs[++index]);
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

Unit3 initialGravityDirection(const Dataset& dataset,
                              double initializationSeconds) {
  const size_t sampleCount = min(dataset.truth.size(), dataset.imu.size());
  if (sampleCount == 0) {
    throw runtime_error("Cannot initialize tilt observer from an empty dataset.");
  }

  const double requestedInitializationSamples =
      initializationSeconds / dataset.timestep();
  const size_t initializationSamples =
      min(sampleCount, max<size_t>(
                           1, static_cast<size_t>(
                                  llround(requestedInitializationSamples))));
  Vector3 gravitySum = Vector3::Zero();
  for (size_t sampleIndex = 0; sampleIndex < initializationSamples;
       ++sampleIndex) {
    const Vector3& specificForce = dataset.imu[sampleIndex].acc;
    if (specificForce.norm() > 1e-9) {
      gravitySum += gravityDirectionFromSpecificForce(specificForce).point3();
    }
  }
  if (gravitySum.norm() <= 1e-9) {
    return gravityDirectionFromSpecificForce(dataset.imu.front().acc);
  }
  return Unit3(gravitySum.normalized());
}

void writeTiltObserverRow(ofstream& stream, const ResultsWriter& writer,
                          const string& datasetName, size_t sampleIndex,
                          double timestamp, double tauP, double tauI,
                          const Vector2& gtRollPitch,
                          const Vector2& estRollPitch,
                          const Vector3& gtGyroBias,
                          const Vector3& estGyroBias,
                          const Dataset::ImuMeasurement& imuMeasurement) {
  stream << csvEscape(writer.runId()) << "," << csvEscape(writer.appName())
         << "," << csvEscape(datasetName) << "," << sampleIndex << ","
         << timestamp << "," << tauP << "," << tauI << ","
         << gtRollPitch.x() << "," << gtRollPitch.y() << ","
         << estRollPitch.x() << "," << estRollPitch.y() << ","
         << gtGyroBias.x() << "," << gtGyroBias.y() << ","
         << gtGyroBias.z() << "," << estGyroBias.x() << ","
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
                double initializationSeconds) {
  const filesystem::path outputPath =
      writer.runDirectory() / ("tilt_observer_" + datasetName + ".csv");
  ofstream stream(outputPath);
  if (!stream.is_open()) {
    throw runtime_error("Failed to open tilt observer CSV: " +
                        outputPath.string());
  }
  stream << setprecision(17);
  stream << tiltObserverHeader() << "\n";

  TiltObserver observer(tauP, tauI, dataset.timestep(),
                        initialGravityDirection(dataset,
                                                initializationSeconds));
  const size_t sampleCount = min(dataset.truth.size(), dataset.imu.size());
  for (size_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
    const auto& imuMeasurement = dataset.imu[sampleIndex];
    const auto& truth = dataset.truth[sampleIndex];

    observer(imuMeasurement.omega, imuMeasurement.acc);

    const Unit3 gtUpDirection(
        truth.navState.attitude().matrix().transpose() * Vector3::UnitZ());
    const Vector2 gtRollPitch = rollPitchFromUpDirection(gtUpDirection);
    const Vector2 estRollPitch =
        rollPitchFromGravityDirection(observer.x_hat.n);
    writeTiltObserverRow(stream, writer, datasetName, sampleIndex,
                         truth.timestamp, tauP, tauI, gtRollPitch,
                         estRollPitch, truth.bias.gyroscope(),
                         observer.x_hat.b, imuMeasurement);
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
                 options.initializationSeconds);
      cout << "Wrote " << datasetName << "\n";
    }

    cout << "Results written to " << writer.runDirectory() << "\n";
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
