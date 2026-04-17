/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   EKFNEESEvaluator.h
 * @brief  NEES evaluator for EKF comparison
 * @author Alec Kain
 */

#pragma once

#include <gtsam/navigation/Gal3ImuEKF.h>
#include <gtsam/navigation/NavStateImuEKF.h>

#include "Dataset.h"
#include "NEESEvaluator.h"
#include "NEESResults.h"
#include "PIMs.h"
#include "TrajectoryValidator.h"

namespace gtsam {

/**
 * @brief NEES evaluator specifically for EKF comparison between Gal3ImuEKF and
 * NavStateImuEKF
 *
 * Provides methods to run both EKF implementations and compare their NEES
 * values across different preintegration time windows.
 */
class EKFNEESEvaluator {
 public:
  /**
   * @brief One trajectory sample with GT, prediction, and 9D error.
   */
  struct TrajectorySample {
    size_t sampleIndex = 0;
    TrajectoryPoint groundTruth;
    TrajectoryPoint predicted;
    Vector9 error = Vector9::Zero();
  };

  /**
   * @brief Shared rich output for one EKF run.
   */
  struct RunArtifacts {
    double preintegrationTime = 0.0;
    size_t samplesPerWindow = 0;
    std::vector<double> neesValues;
    std::vector<WindowEvaluation> windowEvaluations;
    std::vector<TrajectorySample> trajectorySamples;
  };

  /// Constructor
  explicit EKFNEESEvaluator(const Dataset& dataset);

  /// Compute rich Gal3 EKF artifacts with alpha scaling.
  RunArtifacts computeGal3ImuEKFArtifacts(double interval, double alpha) const;

  /// Compute rich Gal3 EKF artifacts with custom parameters.
  RunArtifacts computeGal3ImuEKFArtifacts(
      double interval,
      const std::shared_ptr<PreintegrationCombinedParams>& params) const;

  /// Compute rich NavState EKF artifacts with alpha scaling.
  RunArtifacts computeNavStateImuEKFArtifacts(double interval,
                                              double alpha) const;

  /// Compute rich NavState EKF artifacts with custom parameters.
  RunArtifacts computeNavStateImuEKFArtifacts(
      double interval,
      const std::shared_ptr<PreintegrationCombinedParams>& params) const;

  /// Run Gal3 IMU EKF with alpha scaling
  /// @param interval Preintegration time window in seconds
  /// @param alpha Noise scaling factor
  /// @return NEES results
  NEESResults runGal3ImuEKF(double interval, double alpha) const;

  /// Run Gal3 IMU EKF with alpha scaling and dataset name
  /// @param interval Preintegration time window in seconds
  /// @param alpha Noise scaling factor
  /// @param datasetName Name for output files
  /// @return NEES results
  NEESResults runGal3ImuEKF(double interval, double alpha,
                            const std::string& datasetName) const;

  /// Run Gal3 IMU EKF with custom parameters
  /// @param interval Preintegration time window
  /// @param params Custom preintegration parameters
  /// @param datasetName Name for output files
  /// @return NEES results
  NEESResults runGal3ImuEKF(
      double interval,
      const std::shared_ptr<PreintegrationCombinedParams>& params,
      const std::string& datasetName = "default") const;

  /// Run NavState IMU EKF with alpha scaling
  /// @param interval Preintegration time window in seconds
  /// @param alpha Noise scaling factor
  /// @return NEES results
  NEESResults runNavStateImuEKF(double interval, double alpha) const;

  /// Run NavState IMU EKF with alpha scaling and dataset name
  /// @param interval Preintegration time window in seconds
  /// @param alpha Noise scaling factor
  /// @param datasetName Name for output files
  /// @return NEES results
  NEESResults runNavStateImuEKF(double interval, double alpha,
                                const std::string& datasetName) const;

  /// Run NavState IMU EKF with custom parameters
  /// @param interval Preintegration time window
  /// @param params Custom preintegration parameters
  /// @param datasetName Name for output files
  /// @return NEES results
  NEESResults runNavStateImuEKF(
      double interval,
      const std::shared_ptr<PreintegrationCombinedParams>& params,
      const std::string& datasetName = "default") const;

 private:
  const Dataset& dataset_;

  /// Process time window for Gal3 EKF
  RunArtifacts processTimeWindowWithGal3EKF(
      const std::shared_ptr<PreintegrationCombinedParams>& params,
      double preintegrationTime, double dt) const;

  /// Process time window for NavState EKF
  RunArtifacts processTimeWindowWithNavStateEKF(
      const std::shared_ptr<PreintegrationCombinedParams>& params,
      double preintegrationTime, double dt) const;

  Gal3 convertToGal3(const NavState& navState, double time) const;

  Gal3ImuEKF initializeGal3EKF(
      const NavState& initialState,
      const std::shared_ptr<PreintegrationCombinedParams>& params) const;

  NavStateImuEKF initializeNavStateEKF(
      const NavState& initialState,
      const std::shared_ptr<PreintegrationCombinedParams>& params) const;

  Vector9 computeGal3Error(const Gal3& predicted,
                           const Gal3& groundTruth) const;

  Matrix9 extractNavigationCovariance(const Gal3ImuEKF& ekf) const;

  /// Create ground truth trajectory point from NavState
  TrajectoryPoint createGroundTruthPoint(const NavState& navState,
                                         double timestamp) const;

  /// Create predicted trajectory point from Gal3 state
  TrajectoryPoint createPredictedPointFromGal3(const Gal3& gal3State,
                                               const Matrix9& covariance,
                                               double timestamp) const;

  /// Create predicted trajectory point from NavState
  TrajectoryPoint createPredictedPointFromNavState(const NavState& navState,
                                                   const Matrix9& covariance,
                                                   double timestamp) const;
};

}  // namespace gtsam
