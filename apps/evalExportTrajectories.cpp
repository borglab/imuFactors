/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalExportTrajectories.cpp
 * @brief  Export canonical trajectory packages for best and worst noise
 * parameters
 */

#include <iostream>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "EKFNEESEvaluator.h"
#include "ResultsAdapters.h"
#include "ResultsWriter.h"

using namespace gtsam;
using namespace std;

namespace {

constexpr double kIntervalSeconds = 0.2;

struct NoiseParameters {
  double alphaGyro = 0.0;
  double alphaAcc = 0.0;
  string label;
};

struct AppOptions {
  NoiseParameters best{13.0, 9.4, "best"};
  NoiseParameters worst{0.5, 0.5, "worst"};
  string datasetType = "all";
  string outputRoot = "./results";
};

void printUsage(const char* programName) {
  cout << "Usage: " << programName << " [options]\n";
  cout << "  --output-root <path>    Results root directory (default: "
          "./results)\n";
  cout << "  --best-gyro <value>     Gyroscope scaling for best config\n";
  cout << "  --best-acc <value>      Accelerometer scaling for best config\n";
  cout << "  --worst-gyro <value>    Gyroscope scaling for worst config\n";
  cout << "  --worst-acc <value>     Accelerometer scaling for worst config\n";
  cout << "  --dataset-type <type>   all | machine_hall | vicon\n";
  cout << "  " << programName
       << " <bestAlphaGyro> <bestAlphaAcc> <worstAlphaGyro> <worstAlphaAcc>\n";
}

AppOptions parseAppArguments(int argc, char* argv[]) {
  vector<string> arguments;
  arguments.reserve(max(0, argc - 1));
  for (int index = 1; index < argc; ++index) {
    arguments.push_back(argv[index]);
  }

  const ParsedOutputRootCli outputRootCli =
      parseOutputRootCliArguments(arguments);
  AppOptions options;
  options.outputRoot = outputRootCli.outputRoot;

  if (outputRootCli.remainingArgs.size() == 4 &&
      outputRootCli.remainingArgs[0].rfind("--", 0) != 0) {
    options.best.alphaGyro = stod(outputRootCli.remainingArgs[0]);
    options.best.alphaAcc = stod(outputRootCli.remainingArgs[1]);
    options.worst.alphaGyro = stod(outputRootCli.remainingArgs[2]);
    options.worst.alphaAcc = stod(outputRootCli.remainingArgs[3]);
    return options;
  }

  for (size_t index = 0; index < outputRootCli.remainingArgs.size(); ++index) {
    const string& argument = outputRootCli.remainingArgs[index];
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (argument == "--best-gyro") {
      options.best.alphaGyro = stod(outputRootCli.remainingArgs.at(++index));
      continue;
    }
    if (argument == "--best-acc") {
      options.best.alphaAcc = stod(outputRootCli.remainingArgs.at(++index));
      continue;
    }
    if (argument == "--worst-gyro") {
      options.worst.alphaGyro = stod(outputRootCli.remainingArgs.at(++index));
      continue;
    }
    if (argument == "--worst-acc") {
      options.worst.alphaAcc = stod(outputRootCli.remainingArgs.at(++index));
      continue;
    }
    if (argument == "--dataset-type") {
      options.datasetType = outputRootCli.remainingArgs.at(++index);
      continue;
    }
    throw runtime_error("Unknown argument: " + argument);
  }

  return options;
}

DatasetFilter selectFilter(const string& datasetType) {
  if (datasetType == "machine_hall" || datasetType == "mh") {
    return DatasetFilters::machineHall;
  }
  if (datasetType == "vicon" || datasetType == "v") {
    return DatasetFilters::viconRoom;
  }
  return DatasetFilters::all;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const AppOptions options = parseAppArguments(argc, argv);
    const DatasetFilter filter = selectFilter(options.datasetType);
    const auto datasets = discoverFilteredDatasets("../data/euroc/", filter);
    if (datasets.empty()) {
      cerr << "No datasets found in ../data/euroc/\n";
      return 1;
    }

    ResultsWriter writer(argv[0], options.outputRoot);
    writeCanonicalRunMetadata(&writer, argc, argv);

    for (const auto& [datasetName, datasetPath] : datasets) {
      writer.writeDataset(makeDatasetRow(writer, datasetName, datasetPath,
                                         options.datasetType));
      Dataset dataset(datasetPath);
      EKFNEESEvaluator evaluator(dataset);

      writeEkfArtifacts(&writer, datasetName, "gal3_imu_ekf",
                        options.best.label,
                        evaluator.computeGal3ImuEKFArtifacts(
                            kIntervalSeconds,
                            dataset.configureImuParams(options.best.alphaGyro,
                                                       options.best.alphaAcc)));
      writeEkfArtifacts(
          &writer, datasetName, "gal3_imu_ekf", options.worst.label,
          evaluator.computeGal3ImuEKFArtifacts(
              kIntervalSeconds,
              dataset.configureImuParams(options.worst.alphaGyro,
                                         options.worst.alphaAcc)));
    }

    cout << "Results written to " << writer.runDirectory() << "\n";
    return 0;
  } catch (const exception& error) {
    cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
