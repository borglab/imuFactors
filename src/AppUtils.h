/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <algorithm>
#include <filesystem>
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Dataset.h"

namespace gtsam {

/**
 * Common CLI options for dataset-driven analysis apps.
 */
struct AppCliOptions {
  std::string dataDirectory = "../data/euroc/";
  std::optional<std::string> datasetName;
};

/**
 * Parsed dataset CLI options plus application-specific trailing arguments.
 */
struct ParsedAppCliOptions {
  AppCliOptions options;
  std::vector<std::string> remainingArgs;
};

/**
 * Resolved datasets plus any application-specific trailing arguments.
 */
struct ResolvedDatasetCli {
  AppCliOptions options;
  std::vector<std::pair<std::string, std::string>> datasets;
  std::vector<std::string> remainingArgs;
};

/// Dataset filter predicate type.
using DatasetFilter = std::function<bool(const std::string&)>;

/// Predefined dataset filters shared by dataset-driven apps.
namespace DatasetFilters {

/// Filter for Machine Hall datasets (MH-series).
inline bool machineHall(const std::string& name) {
  return name.find("MH") == 0;
}

/// Filter for Vicon Room datasets (V-series).
inline bool viconRoom(const std::string& name) { return name.find("V") == 0; }

/// Filter that accepts all datasets.
inline bool all(const std::string&) { return true; }

}  // namespace DatasetFilters

/**
 * Default interval set used by the quadrature analysis apps.
 */
inline std::vector<double> defaultQuadratureIntervals() {
  return {0.2, 0.5, 1.0};
}

/**
 * Print common usage help for dataset-driven analysis apps.
 * @param programName Executable name from argv[0]
 */
inline void printDatasetAppUsage(const char* programName) {
  std::cout << "Usage: " << programName
            << " [--data-dir <path>] [--dataset <name>]\n";
  std::cout << "  --data-dir <path>       Dataset directory (default: "
               "../data/euroc/)\n";
  std::cout << "  --dataset <name>        Restrict to one dataset (e.g. MH01 "
               "or euroc_MH01.csv)\n";
}

/**
 * Normalize dataset arguments such as euroc_MH01.csv to MH01.
 * @param datasetArgument User-supplied dataset name or file name
 * @return Normalized dataset identifier
 */
inline std::string normalizeDatasetName(const std::string& datasetArgument) {
  std::string normalized =
      std::filesystem::path(datasetArgument).stem().string();
  if (normalized.rfind("euroc_", 0) == 0) {
    normalized = normalized.substr(6);
  }
  return normalized;
}

/**
 * Check whether an argument requests help.
 * @param argument Command-line token
 * @return True when the token is --help or -h
 */
inline bool isHelpArgument(const std::string& argument) {
  return argument == "--help" || argument == "-h";
}

/**
 * Parse shared dataset-analysis CLI flags and preserve trailing app-specific
 * arguments.
 * @param arguments Command-line arguments excluding argv[0]
 * @return Parsed dataset options plus unconsumed arguments
 */
inline ParsedAppCliOptions parseDatasetAppCliOptions(
    const std::vector<std::string>& arguments) {
  ParsedAppCliOptions parsedOptions;
  for (size_t index = 0; index < arguments.size(); ++index) {
    const std::string& argument = arguments[index];
    if (argument == "--data-dir") {
      if (index + 1 >= arguments.size()) {
        throw std::runtime_error("Missing value for --data-dir");
      }
      parsedOptions.options.dataDirectory = arguments[++index];
      continue;
    }
    if (argument == "--dataset") {
      if (index + 1 >= arguments.size()) {
        throw std::runtime_error("Missing value for --dataset");
      }
      parsedOptions.options.datasetName =
          normalizeDatasetName(arguments[++index]);
      continue;
    }
    parsedOptions.remainingArgs.push_back(argument);
  }
  return parsedOptions;
}

/**
 * Parse shared dataset-analysis CLI flags from argc/argv.
 * @param argc Command-line argc
 * @param argv Command-line argv
 * @return Parsed dataset options plus unconsumed arguments
 */
inline ParsedAppCliOptions parseDatasetAppCliOptions(int argc, char* argv[]) {
  std::vector<std::string> arguments;
  arguments.reserve(std::max(0, argc - 1));
  for (int index = 1; index < argc; ++index) {
    arguments.push_back(argv[index]);
  }
  return parseDatasetAppCliOptions(arguments);
}

/**
 * Discover datasets matching a filter predicate.
 * @param dataDirectory Directory that contains EuRoC CSV files
 * @param filter Predicate that selects dataset names
 * @return Sorted dataset name/path pairs
 */
inline std::vector<std::pair<std::string, std::string>>
discoverFilteredDatasets(const std::string& dataDirectory,
                         DatasetFilter filter) {
  namespace fs = std::filesystem;
  std::vector<std::pair<std::string, std::string>> datasets;

  if (!fs::exists(dataDirectory)) {
    std::cerr << "❌ Data directory not found: " << dataDirectory << std::endl;
    return datasets;
  }

  for (const auto& entry : fs::directory_iterator(dataDirectory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".csv") {
      std::string filename = entry.path().filename().string();
      if (filename.find("euroc_") == 0) {
        std::string name = normalizeDatasetName(filename);
        if (filter(name)) {
          datasets.push_back({name, entry.path().string()});
        }
      }
    }
  }

  std::sort(datasets.begin(), datasets.end());
  return datasets;
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
 * Resolve the datasets requested by the shared dataset CLI options.
 * @param options Parsed dataset CLI options
 * @param filter Dataset discovery predicate
 * @return Selected dataset name/path pairs
 */
inline std::vector<std::pair<std::string, std::string>> resolveDatasets(
    const AppCliOptions& options, DatasetFilter filter = DatasetFilters::all) {
  const auto discoveredDatasets =
      discoverFilteredDatasets(options.dataDirectory, filter);
  return selectDatasets(discoveredDatasets, options.datasetName);
}

/**
 * Parse shared dataset CLI flags and immediately resolve the matching
 * datasets.
 * @param arguments Command-line arguments excluding argv[0]
 * @param filter Dataset discovery predicate
 * @return Resolved datasets plus unconsumed arguments
 */
inline ResolvedDatasetCli resolveDatasetAppCli(
    const std::vector<std::string>& arguments,
    DatasetFilter filter = DatasetFilters::all) {
  const ParsedAppCliOptions parsedOptions =
      parseDatasetAppCliOptions(arguments);
  return {parsedOptions.options, resolveDatasets(parsedOptions.options, filter),
          parsedOptions.remainingArgs};
}

/**
 * Parse shared dataset CLI flags from argc/argv and immediately resolve the
 * matching datasets.
 * @param argc Command-line argc
 * @param argv Command-line argv
 * @param filter Dataset discovery predicate
 * @return Resolved datasets plus unconsumed arguments
 */
inline ResolvedDatasetCli resolveDatasetAppCli(
    int argc, char* argv[], DatasetFilter filter = DatasetFilters::all) {
  const ParsedAppCliOptions parsedOptions =
      parseDatasetAppCliOptions(argc, argv);
  return {parsedOptions.options, resolveDatasets(parsedOptions.options, filter),
          parsedOptions.remainingArgs};
}

/**
 * Parse the shared quadrature interval limit argument.
 * @param arguments Unconsumed application-specific arguments
 * @return Requested interval limit, or zero when unspecified
 */
inline size_t parseMaxIntervalsArgument(
    const std::vector<std::string>& arguments) {
  size_t maxIntervals = 0;
  for (size_t index = 0; index < arguments.size(); ++index) {
    const std::string& argument = arguments[index];
    if (isHelpArgument(argument)) {
      continue;
    }
    if (argument == "--max-intervals") {
      if (index + 1 >= arguments.size()) {
        throw std::runtime_error("Missing value for --max-intervals");
      }
      const int requestedCount = std::stoi(arguments[++index]);
      if (requestedCount <= 0) {
        throw std::runtime_error("--max-intervals must be positive.");
      }
      maxIntervals = static_cast<size_t>(requestedCount);
      continue;
    }
    throw std::runtime_error("Unknown argument: " + argument);
  }
  return maxIntervals;
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
  const size_t intervalCount =
      (maxIntervals == 0) ? defaultIntervals.size()
                          : std::min(maxIntervals, defaultIntervals.size());
  return std::vector<double>(defaultIntervals.begin(),
                             defaultIntervals.begin() + intervalCount);
}

/**
 * Construct each dataset centrally and invoke a mutable runner for it.
 * Datasets with fewer than two truth states are skipped.
 * @tparam Runner Callable type with operator()(const std::string&, const
 * Dataset&)
 * @param datasetCli Resolved dataset CLI options
 * @param runner Mutable per-dataset runner
 * @return Exit code style status: 0 on success, 1 on empty input or dataset
 * processing failure
 */
template <class Runner>
inline int runForDatasets(const ResolvedDatasetCli& datasetCli,
                          Runner& runner) {
  if (datasetCli.datasets.empty()) {
    std::cerr << "No datasets found in " << datasetCli.options.dataDirectory
              << "\n";
    return 1;
  }

  try {
    for (const auto& [datasetName, datasetPath] : datasetCli.datasets) {
      Dataset dataset(datasetPath);
      if (dataset.truth.size() < 2) {
        continue;
      }
      runner(datasetName, dataset);
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}

}  // namespace gtsam
