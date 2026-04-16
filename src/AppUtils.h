/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include "Dataset.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Common CLI options for dataset-driven analysis apps.
 */
struct AppCliOptions {
  std::string dataDirectory = "../data/euroc/";
  std::optional<std::string> datasetName;
  size_t maxIntervals = 0;
};

/**
 * Default interval set used by the quadrature analysis apps.
 */
inline std::vector<double> defaultQuadratureIntervals() { return {0.2, 0.5, 1.0}; }

/**
 * Print common usage help for dataset-driven analysis apps.
 * @param programName Executable name from argv[0]
 */
inline void printDatasetAppUsage(const char* programName) {
  std::cout << "Usage: " << programName
            << " [--data-dir <path>] [--dataset <name>] [--max-intervals <count>]\n";
  std::cout << "  --data-dir <path>       Dataset directory (default: ../data/euroc/)\n";
  std::cout << "  --dataset <name>        Restrict to one dataset (e.g. MH01 or euroc_MH01.csv)\n";
  std::cout << "  --max-intervals <count> Restrict to first N default intervals\n";
}

/**
 * Normalize dataset arguments such as euroc_MH01.csv to MH01.
 * @param datasetArgument User-supplied dataset name or file name
 * @return Normalized dataset identifier
 */
inline std::string normalizeDatasetName(const std::string& datasetArgument) {
  std::string normalized = std::filesystem::path(datasetArgument).stem().string();
  if (normalized.rfind("euroc_", 0) == 0) {
    normalized = normalized.substr(6);
  }
  return normalized;
}

/**
 * Parse common dataset-analysis CLI flags.
 * @param argc Command-line argc
 * @param argv Command-line argv
 * @return Parsed options
 */
inline AppCliOptions parseDatasetAppCliOptions(int argc, char* argv[]) {
  AppCliOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      printDatasetAppUsage(argv[0]);
      std::exit(0);
    }
    if (argument == "--data-dir") {
      if (index + 1 >= argc) {
        throw std::runtime_error("Missing value for --data-dir");
      }
      options.dataDirectory = argv[++index];
      continue;
    }
    if (argument == "--dataset") {
      if (index + 1 >= argc) {
        throw std::runtime_error("Missing value for --dataset");
      }
      options.datasetName = normalizeDatasetName(argv[++index]);
      continue;
    }
    if (argument == "--max-intervals") {
      if (index + 1 >= argc) {
        throw std::runtime_error("Missing value for --max-intervals");
      }
      const int requestedCount = std::stoi(argv[++index]);
      if (requestedCount <= 0) {
        throw std::runtime_error("--max-intervals must be positive.");
      }
      options.maxIntervals = static_cast<size_t>(requestedCount);
      continue;
    }
    throw std::runtime_error("Unknown argument: " + argument);
  }
  return options;
}

/**
 * Restrict a discovered dataset list to one named dataset when requested.
 * @param allDatasets Discovered dataset name/path pairs
 * @param datasetName Optional dataset filter
 * @return Either the full input list or a single matching dataset
 */
inline std::vector<std::pair<std::string, std::string>> selectDatasets(
    const std::vector<std::pair<std::string, std::string>>& allDatasets,
    const std::optional<std::string>& datasetName) {
  if (!datasetName) {
    return allDatasets;
  }

  for (const auto& datasetEntry : allDatasets) {
    if (datasetEntry.first == *datasetName) {
      return {datasetEntry};
    }
  }
  throw std::runtime_error("Requested dataset not found: " + *datasetName);
}

/**
 * Restrict interval count while preserving the existing order.
 * @param defaultIntervals Full interval list
 * @param maxIntervals Requested limit, or zero for no limit
 * @return Selected interval prefix
 */
inline std::vector<double> selectIntervals(
    const std::vector<double>& defaultIntervals, size_t maxIntervals) {
  if (defaultIntervals.empty()) {
    return {};
  }
  const size_t intervalCount = (maxIntervals == 0)
      ? defaultIntervals.size()
      : std::min(maxIntervals, defaultIntervals.size());
  return std::vector<double>(defaultIntervals.begin(), defaultIntervals.begin() + intervalCount);
}

/**
 * Compute the number of IMU samples in a window for the requested interval.
 * @param dataset Dataset providing the timestamp cadence
 * @param intervalSeconds Requested integration interval in seconds
 * @return Number of samples per integration window
 */
inline size_t computeWindowSize(const Dataset& dataset, double intervalSeconds) {
  const auto& states = dataset.getStates();
  if (states.size() < 2) {
    throw std::runtime_error("Dataset must contain at least two state samples.");
  }
  const double timestep = states[1].timestamp - states[0].timestamp;
  if (timestep <= 0.0) {
    throw std::runtime_error("Dataset timestep must be positive.");
  }
  const size_t windowSize = static_cast<size_t>(intervalSeconds / timestep);
  return std::max<size_t>(windowSize, 1);
}

}  // namespace gtsam
