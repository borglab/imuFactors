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

#include "NEESEvaluator.h"
#include "Dataset.h"
#include "TrajectoryValidator.h"
#include <gtsam/navigation/Gal3ImuEKF.h>
#include <gtsam/navigation/NavStateImuEKF.h>

namespace gtsam {

/**
 * @brief NEES evaluator specifically for EKF comparison between Gal3ImuEKF and NavStateImuEKF
 * 
 * Provides methods to run both EKF implementations and compare their NEES values
 * across different preintegration time windows.
 */
class EKFNEESEvaluator {
public:
    /// Constructor
    explicit EKFNEESEvaluator(const Dataset& dataset);
    
    /// Run Gal3 IMU EKF with alpha scaling
    /// @param interval Preintegration time window in seconds
    /// @param alpha Noise scaling factor
    /// @return NEES results
    NEESEvaluator::NEESResults runGal3ImuEKF(double interval, 
                                            double alpha) const;
    
    /// Run Gal3 IMU EKF with alpha scaling and dataset name
    /// @param interval Preintegration time window in seconds
    /// @param alpha Noise scaling factor
    /// @param datasetName Name for output files
    /// @return NEES results
    NEESEvaluator::NEESResults runGal3ImuEKF(double interval, 
                                            double alpha,
                                            const std::string& datasetName) const;
    
    /// Run Gal3 IMU EKF with custom parameters
    /// @param interval Preintegration time window
    /// @param params Custom preintegration parameters
    /// @param datasetName Name for output files
    /// @return NEES results
    NEESEvaluator::NEESResults runGal3ImuEKF(
        double interval, 
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        const std::string& datasetName = "default") const;
    
    /// Run NavState IMU EKF with alpha scaling
    /// @param interval Preintegration time window in seconds
    /// @param alpha Noise scaling factor
    /// @return NEES results
    NEESEvaluator::NEESResults runNavStateImuEKF(double interval, double alpha) const;
    
    /// Run NavState IMU EKF with alpha scaling and dataset name
    /// @param interval Preintegration time window in seconds
    /// @param alpha Noise scaling factor
    /// @param datasetName Name for output files
    /// @return NEES results
    NEESEvaluator::NEESResults runNavStateImuEKF(double interval, 
                                                double alpha,
                                                const std::string& datasetName) const;
    
    /// Run NavState IMU EKF with custom parameters
    /// @param interval Preintegration time window
    /// @param params Custom preintegration parameters
    /// @param datasetName Name for output files
    /// @return NEES results
    NEESEvaluator::NEESResults runNavStateImuEKF(
        double interval,
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        const std::string& datasetName = "default") const;

private:
    const Dataset& dataset_;
    
    /// Compute timestep from dataset
    double computeTimestep() const;
    
    /// Process time window for Gal3 EKF
    NEESEvaluator::NEESResults processTimeWindowWithGal3EKF(
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        double preintegrationTime, double dt,
        const std::string& datasetName) const;
    
    /// Process time window for NavState EKF
    NEESEvaluator::NEESResults processTimeWindowWithNavStateEKF(
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        double preintegrationTime, double dt,
        const std::string& datasetName) const;
    
    /// Helper functions
    std::optional<double> computeNEES(const Vector& error, 
                                     const Matrix& covarianceMatrix) const;
    
    Gal3 convertToGal3(const NavState& navState, double time) const;
    
    Gal3ImuEKF initializeGal3EKF(
        const NavState& initialState,
        const std::shared_ptr<PreintegrationCombinedParams>& params) const;
    
    NavStateImuEKF initializeNavStateEKF(
        const NavState& initialState,
        const std::shared_ptr<PreintegrationCombinedParams>& params) const;
    
    Vector9 computeGal3Error(const Gal3& predicted, const Gal3& groundTruth) const;
    
    Matrix9 extractNavigationCovariance(const Gal3ImuEKF& ekf) const;
    
    
    /// Create ground truth trajectory point from NavState
    TrajectoryValidator::TrajectoryPoint createGroundTruthPoint(
        const NavState& navState, double timestamp) const;
    
    /// Create predicted trajectory point from Gal3 state
    TrajectoryValidator::TrajectoryPoint createPredictedPointFromGal3(
        const Gal3& gal3State, const Matrix9& covariance, double timestamp) const;
    
    /// Create predicted trajectory point from NavState
    TrajectoryValidator::TrajectoryPoint createPredictedPointFromNavState(
        const NavState& navState, const Matrix9& covariance, double timestamp) const;
    
    /// Export trajectory results to CSV
    void exportTrajectoryResults(
        const std::vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
        const std::vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
        const std::vector<Vector9>& errorTrajectory,
        const std::string& filterName,
        const std::string& datasetName,
        const std::string& preintegrationTime) const;
    
    /// Export NEES values to CSV
    void exportNEESValues(
        const std::string& filterName,
        const std::string& datasetName,
        const std::string& preintegrationTime,
        const std::vector<TrajectoryValidator::TrajectoryPoint>& trajectory,
        const std::vector<double>& neesValues) const;
};

} // namespace gtsam