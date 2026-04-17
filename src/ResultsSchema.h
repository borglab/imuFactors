/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <sstream>
#include <string>
#include <vector>

namespace gtsam {

/**
 * @brief One metadata row per application run.
 */
struct RunMetadataRow {
  std::string runId;
  std::string appName;
  std::string timestampUtc;
  std::string cliArgs;
  std::string outputRoot;
  std::string repoVersion;
};

/**
 * @brief Dataset membership for one run.
 */
struct DatasetRow {
  std::string runId;
  std::string appName;
  std::string dataset;
  std::string sourcePath;
  std::string datasetGroup;
};

/**
 * @brief Per-window metric row shared by all window-based apps.
 */
struct WindowMetricRow {
  std::string runId;
  std::string appName;
  std::string dataset;
  std::string method;
  std::string configLabel;
  double intervalSeconds = 0.0;
  size_t samplesPerWindow = 0;
  size_t quadratureNodes = 0;
  size_t windowIndex = 0;
  size_t windowStartSample = 0;
  size_t windowEndSample = 0;
  double windowStartTime = 0.0;
  double windowEndTime = 0.0;
  double normalizedNees = 0.0;
  double rotErrorNorm = 0.0;
  double rotPredSigma = 0.0;
  double posErrorNorm = 0.0;
  double posPredSigma = 0.0;
  double velErrorNorm = 0.0;
  double velPredSigma = 0.0;
};

/**
 * @brief Aggregated window summary row.
 */
struct WindowSummaryRow {
  std::string runId;
  std::string appName;
  std::string dataset;
  std::string method;
  std::string configLabel;
  double intervalSeconds = 0.0;
  size_t samplesPerWindow = 0;
  size_t quadratureNodes = 0;
  size_t sampleCount = 0;
  double normalizedNeesMean = 0.0;
  double normalizedNeesMedian = 0.0;
  double normalizedNeesP95 = 0.0;
  double normalizedNeesVariance = 0.0;
  double rotErrorMedian = 0.0;
  double rotPredSigmaMedian = 0.0;
  double posErrorMedian = 0.0;
  double posPredSigmaMedian = 0.0;
  double velErrorMedian = 0.0;
  double velPredSigmaMedian = 0.0;
};

/**
 * @brief Canonical trajectory sample row with GT, prediction, error, and full
 * covariance.
 */
struct TrajectorySampleRow {
  std::string runId;
  std::string appName;
  std::string dataset;
  std::string method;
  std::string configLabel;
  double intervalSeconds = 0.0;
  size_t samplesPerWindow = 0;
  double timestamp = 0.0;
  double gtX = 0.0;
  double gtY = 0.0;
  double gtZ = 0.0;
  double gtVx = 0.0;
  double gtVy = 0.0;
  double gtVz = 0.0;
  double gtRoll = 0.0;
  double gtPitch = 0.0;
  double gtYaw = 0.0;
  double predX = 0.0;
  double predY = 0.0;
  double predZ = 0.0;
  double predVx = 0.0;
  double predVy = 0.0;
  double predVz = 0.0;
  double predRoll = 0.0;
  double predPitch = 0.0;
  double predYaw = 0.0;
  double errRotX = 0.0;
  double errRotY = 0.0;
  double errRotZ = 0.0;
  double errPosX = 0.0;
  double errPosY = 0.0;
  double errPosZ = 0.0;
  double errVelX = 0.0;
  double errVelY = 0.0;
  double errVelZ = 0.0;
  double rotPredSigma = 0.0;
  double posPredSigma = 0.0;
  double velPredSigma = 0.0;
  Matrix9 covariance = Matrix9::Zero();
};

/**
 * @brief One calibration trial row per dataset/trial pair.
 */
struct CalibrationTrialRow {
  std::string runId;
  std::string appName;
  std::string dataset;
  std::string studyName;
  std::string datasetGroup;
  double alphaGyro = 0.0;
  double alphaAcc = 0.0;
  double datasetNees = 0.0;
  double trialMeanNees = 0.0;
  double trialSumDeviations = 0.0;
};

/**
 * @brief Calibration summary row for best/worst study outcomes.
 */
struct CalibrationSummaryRow {
  std::string runId;
  std::string appName;
  std::string studyName;
  std::string resultLabel;
  double alphaGyro = 0.0;
  double alphaAcc = 0.0;
  double meanNees = 0.0;
  double sumDeviations = 0.0;
};

inline std::string csvEscape(const std::string& value) {
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }

  std::string escaped = "\"";
  for (const char character : value) {
    if (character == '"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += "\"";
  return escaped;
}

template <typename Value>
inline void appendCsvField(std::ostringstream& stream, const Value& value) {
  stream << value;
}

inline void appendCsvField(std::ostringstream& stream, const std::string& value) {
  stream << csvEscape(value);
}

inline void appendCsvField(std::ostringstream& stream, const char* value) {
  stream << csvEscape(value == nullptr ? std::string() : std::string(value));
}

template <typename First, typename... Rest>
inline std::string makeCsvRow(const First& first, const Rest&... rest) {
  std::ostringstream stream;
  appendCsvField(stream, first);
  ((stream << ",", appendCsvField(stream, rest)), ...);
  return stream.str();
}

inline std::string runMetadataHeader() {
  return "run_id,app_name,timestamp_utc,cli_args,output_root,repo_version";
}

inline std::string datasetHeader() {
  return "run_id,app_name,dataset,source_path,dataset_group";
}

inline std::string windowMetricHeader() {
  return "run_id,app_name,dataset,method,config_label,interval_seconds,"
         "samples_per_window,quadrature_nodes,window_index,"
         "window_start_sample,window_end_sample,window_start_time,"
         "window_end_time,normalized_nees,rot_error_norm,rot_pred_sigma,"
         "pos_error_norm,pos_pred_sigma,vel_error_norm,vel_pred_sigma";
}

inline std::string windowSummaryHeader() {
  return "run_id,app_name,dataset,method,config_label,interval_seconds,"
         "samples_per_window,quadrature_nodes,sample_count,"
         "normalized_nees_mean,normalized_nees_median,normalized_nees_p95,"
         "normalized_nees_variance,rot_error_median,rot_pred_sigma_median,"
         "pos_error_median,pos_pred_sigma_median,vel_error_median,"
         "vel_pred_sigma_median";
}

inline std::string trajectorySampleHeader() {
  std::ostringstream stream;
  stream
      << "run_id,app_name,dataset,method,config_label,interval_seconds,"
      << "samples_per_window,timestamp,gt_x,gt_y,gt_z,gt_vx,gt_vy,gt_vz,"
      << "gt_roll,gt_pitch,gt_yaw,pred_x,pred_y,pred_z,pred_vx,pred_vy,"
      << "pred_vz,pred_roll,pred_pitch,pred_yaw,err_rot_x,err_rot_y,"
      << "err_rot_z,err_pos_x,err_pos_y,err_pos_z,err_vel_x,err_vel_y,"
      << "err_vel_z,rot_pred_sigma,pos_pred_sigma,vel_pred_sigma";
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      stream << ",cov_" << row << "_" << column;
    }
  }
  return stream.str();
}

inline std::string calibrationTrialHeader() {
  return "run_id,app_name,dataset,study_name,dataset_group,alpha_gyro,"
         "alpha_acc,dataset_nees,trial_mean_nees,trial_sum_deviations";
}

inline std::string calibrationSummaryHeader() {
  return "run_id,app_name,study_name,result_label,alpha_gyro,alpha_acc,"
         "mean_nees,sum_deviations";
}

inline std::string toCsvRow(const RunMetadataRow& row) {
  return makeCsvRow(row.runId, row.appName, row.timestampUtc, row.cliArgs,
                    row.outputRoot, row.repoVersion);
}

inline std::string toCsvRow(const DatasetRow& row) {
  return makeCsvRow(row.runId, row.appName, row.dataset, row.sourcePath,
                    row.datasetGroup);
}

inline std::string toCsvRow(const WindowMetricRow& row) {
  return makeCsvRow(row.runId, row.appName, row.dataset, row.method,
                    row.configLabel, row.intervalSeconds, row.samplesPerWindow,
                    row.quadratureNodes, row.windowIndex, row.windowStartSample,
                    row.windowEndSample, row.windowStartTime, row.windowEndTime,
                    row.normalizedNees, row.rotErrorNorm, row.rotPredSigma,
                    row.posErrorNorm, row.posPredSigma, row.velErrorNorm,
                    row.velPredSigma);
}

inline std::string toCsvRow(const WindowSummaryRow& row) {
  return makeCsvRow(
      row.runId, row.appName, row.dataset, row.method, row.configLabel,
      row.intervalSeconds, row.samplesPerWindow, row.quadratureNodes,
      row.sampleCount, row.normalizedNeesMean, row.normalizedNeesMedian,
      row.normalizedNeesP95, row.normalizedNeesVariance, row.rotErrorMedian,
      row.rotPredSigmaMedian, row.posErrorMedian, row.posPredSigmaMedian,
      row.velErrorMedian, row.velPredSigmaMedian);
}

inline std::string toCsvRow(const TrajectorySampleRow& row) {
  std::ostringstream stream;
  stream << makeCsvRow(
      row.runId, row.appName, row.dataset, row.method, row.configLabel,
      row.intervalSeconds, row.samplesPerWindow, row.timestamp, row.gtX,
      row.gtY, row.gtZ, row.gtVx, row.gtVy, row.gtVz, row.gtRoll,
      row.gtPitch, row.gtYaw, row.predX, row.predY, row.predZ, row.predVx,
      row.predVy, row.predVz, row.predRoll, row.predPitch, row.predYaw,
      row.errRotX, row.errRotY, row.errRotZ, row.errPosX, row.errPosY,
      row.errPosZ, row.errVelX, row.errVelY, row.errVelZ, row.rotPredSigma,
      row.posPredSigma, row.velPredSigma);
  for (int matrixRow = 0; matrixRow < 9; ++matrixRow) {
    for (int matrixColumn = 0; matrixColumn < 9; ++matrixColumn) {
      stream << "," << row.covariance(matrixRow, matrixColumn);
    }
  }
  return stream.str();
}

inline std::string toCsvRow(const CalibrationTrialRow& row) {
  return makeCsvRow(row.runId, row.appName, row.dataset, row.studyName,
                    row.datasetGroup, row.alphaGyro, row.alphaAcc,
                    row.datasetNees, row.trialMeanNees,
                    row.trialSumDeviations);
}

inline std::string toCsvRow(const CalibrationSummaryRow& row) {
  return makeCsvRow(row.runId, row.appName, row.studyName, row.resultLabel,
                    row.alphaGyro, row.alphaAcc, row.meanNees,
                    row.sumDeviations);
}

}  // namespace gtsam
