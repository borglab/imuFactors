/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>

#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "AppUtils.h"
#include "ResultsAdapters.h"

namespace gtsam {

using PIMQuadrature = PreintegratedImuMeasurementsQ;
using PIMTangent = PreintegratedImuMeasurementsT<TangentPreintegration>;
using PIMManifold = PreintegratedImuMeasurementsT<ManifoldPreintegration>;

struct QuadratureAppOptions {
  size_t maxIntervals = 0;
  double alphaGyro = 3.0;
  double alphaAcc = 3.0;
};

inline QuadratureAppOptions parseQuadratureAppArguments(
    const std::vector<std::string>& arguments, const char* programName,
    double defaultAlpha = 3.0) {
  QuadratureAppOptions options;
  options.alphaGyro = defaultAlpha;
  options.alphaAcc = defaultAlpha;
  std::vector<std::string> intervalArguments;
  for (size_t index = 0; index < arguments.size(); ++index) {
    const std::string& argument = arguments[index];
    if (isHelpArgument(argument)) {
      printQuadratureAppUsage(programName, defaultAlpha);
      std::exit(0);
    }
    if (argument == "--alpha") {
      if (index + 1 >= arguments.size()) {
        throw std::runtime_error("Missing value for --alpha");
      }
      const double alpha =
          parsePositiveDoubleOption("--alpha", arguments[++index]);
      options.alphaGyro = alpha;
      options.alphaAcc = alpha;
      continue;
    }
    if (argument == "--alpha-gyro") {
      if (index + 1 >= arguments.size()) {
        throw std::runtime_error("Missing value for --alpha-gyro");
      }
      options.alphaGyro =
          parsePositiveDoubleOption("--alpha-gyro", arguments[++index]);
      continue;
    }
    if (argument == "--alpha-acc") {
      if (index + 1 >= arguments.size()) {
        throw std::runtime_error("Missing value for --alpha-acc");
      }
      options.alphaAcc =
          parsePositiveDoubleOption("--alpha-acc", arguments[++index]);
      continue;
    }
    intervalArguments.push_back(argument);
  }
  options.maxIntervals = parseMaxIntervalsArgument(intervalArguments);
  return options;
}

inline std::string alphaConfigLabel(const QuadratureAppOptions& options) {
  std::ostringstream stream;
  stream << "alpha_g" << options.alphaGyro << "_a" << options.alphaAcc;
  return stream.str();
}

inline std::shared_ptr<PreintegrationParams> makePreintegrationParams(
    double sigmaGyro, double sigmaAcc) {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = I_3x3 * sigmaAcc * sigmaAcc;
  params->gyroscopeCovariance = I_3x3 * sigmaGyro * sigmaGyro;
  params->integrationCovariance = I_3x3 * 1e-8;
  return params;
}

class QuadratureRunner {
 public:
  QuadratureRunner(const QuadratureAppOptions& options, ResultsWriter* writer,
                   const std::string& datasetGroup,
                   const std::shared_ptr<PreintegrationParams>& params,
                   std::optional<InitialCovarianceOptions> initialCovariance,
                   const std::string& configLabel = "default")
      : intervals_(selectIntervals(defaultQuadratureIntervals(),
                                   options.maxIntervals)),
        writer_(writer),
        datasetGroup_(datasetGroup),
        params_(params),
        initialCovariance_(initialCovariance),
        configLabel_(configLabel) {}

  void operator()(const std::string& datasetName, const Dataset& dataset) {
    struct IntervalEvaluationSummary {
      double intervalSeconds = 0.0;
      size_t samplesPerWindow = 0;
      size_t candidateWindows = 0;
      size_t quadratureEvaluated = 0;
      size_t manifoldEvaluated = 0;
      size_t tangentEvaluated = 0;
    };

    std::vector<IntervalEvaluationSummary> intervalSummaries;
    intervalSummaries.reserve(intervals_.size());

    std::cout << "Processing dataset " << datasetName << "...\n";
    writer_->writeDataset(
        makeDatasetRow(*writer_, datasetName, dataset, datasetGroup_));
    for (const double intervalSeconds : intervals_) {
      const size_t samplesPerWindow = dataset.stepsForInterval(intervalSeconds);
      const size_t quadratureNodes = std::max<size_t>(
          3, static_cast<size_t>(
                 std::floor(std::sqrt(static_cast<double>(samplesPerWindow)))));
      const auto windows = dataset.completeWindows(samplesPerWindow);
      const auto quadratureEvaluations =
          collectWindowEvaluations<PIMQuadrature>(
              windows, params_, initialCovariance_, quadratureNodes);
      const auto manifoldEvaluations = collectWindowEvaluations<PIMManifold>(
          windows, params_, initialCovariance_);
      const auto tangentEvaluations = collectWindowEvaluations<PIMTangent>(
          windows, params_, initialCovariance_);

      writeWindowRows(writer_, datasetName, "quadrature", configLabel_,
                      intervalSeconds, samplesPerWindow, quadratureNodes,
                      quadratureEvaluations);
      writeWindowRows(writer_, datasetName, "manifold", configLabel_,
                      intervalSeconds, samplesPerWindow, 0,
                      manifoldEvaluations);
      writeWindowRows(writer_, datasetName, "tangent", configLabel_,
                      intervalSeconds, samplesPerWindow, 0, tangentEvaluations);

      intervalSummaries.push_back({intervalSeconds, samplesPerWindow,
                                   windows.size(), quadratureEvaluations.size(),
                                   manifoldEvaluations.size(),
                                   tangentEvaluations.size()});
    }

    std::cout << "Finished dataset " << datasetName << ":\n";
    for (const auto& summary : intervalSummaries) {
      std::cout << "  interval " << summary.intervalSeconds << "s ("
                << summary.samplesPerWindow
                << " samples/window): " << summary.candidateWindows
                << " candidate windows, evaluated "
                << summary.quadratureEvaluated << " quadrature, "
                << summary.manifoldEvaluated << " manifold, "
                << summary.tangentEvaluated << " tangent\n";
    }
  }

 private:
  std::vector<double> intervals_;
  ResultsWriter* writer_;
  std::string datasetGroup_;
  std::shared_ptr<PreintegrationParams> params_;
  std::optional<InitialCovarianceOptions> initialCovariance_;
  std::string configLabel_;
};

template <class RunnerFactory>
inline int runDatasetApp(const ResolvedDatasetCli& datasetCli, int argc,
                         char* argv[], RunnerFactory&& makeRunner) {
  ResultsWriter writer(argv[0], datasetCli.options.outputRoot);
  writeCanonicalRunMetadata(&writer, argc, argv);
  auto runner = makeRunner(&writer, resolvedDatasetGroupLabel(datasetCli));
  const int status = runForDatasets(datasetCli, runner);
  if (status != 0) {
    return status;
  }
  std::cout << "Results written to " << writer.runDirectory() << "\n";
  return 0;
}

}  // namespace gtsam
