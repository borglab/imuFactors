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
#include <gtsam/navigation/Gal3ImuEKF.h>
#include <gtsam/navigation/NavStateImuEKF.h>
#include "TrajectoryValidator.h"

namespace gtsam {

/// Extended NEES evaluator for EKF comparison
class EKFNEESEvaluator {
public:
    /// Window indices for processing
    struct WindowIndices {
        int startIndex;
        int endIndex;
    };

    /// Constructor
    explicit EKFNEESEvaluator(const Dataset& dataset);

    /// Compute timestep from dataset
    double computeTimestep() const;

    /// Run Gal3ImuEKF evaluation (uses default dataset name)
    NEESEvaluator::NEESResults runGal3ImuEKF(double interval, double alpha) const;

    /// Run Gal3ImuEKF evaluation with custom dataset name for file output
    NEESEvaluator::NEESResults runGal3ImuEKF(double interval, double alpha, 
                                             const std::string& datasetName) const;

    /// Run NavStateImuEKF evaluation
    NEESEvaluator::NEESResults runNavStateImuEKF(double interval, double alpha) const;

    /// Run NavStateImuEKF evaluation with custom dataset name for file output
    NEESEvaluator::NEESResults runNavStateImuEKF(double interval, double alpha,
                                                  const std::string& datasetName) const;

private:
    const Dataset& dataset_;

    /// Compute NEES value
    std::optional<double> computeNEES(const Vector& error, 
                                      const Matrix& covarianceMatrix) const;

    /// Compute window indices
    WindowIndices computeWindowIndices(int windowNumber, int windowSize) const;

    /// Check if window is valid
    bool isWindowValid(const WindowIndices& window) const;

    /// Gal3 helper functions
    Gal3 convertToGal3(const NavState& navState, double time) const;
    Matrix initializeGal3Covariance() const;
    Gal3ImuEKF initializeGal3EKF(const NavState& initialState,
                                  const std::shared_ptr<PreintegrationCombinedParams>& params) const;
    void propagateGal3EKF(Gal3ImuEKF& ekf, const WindowIndices& window,
                         const imuBias::ConstantBias& bias, double dt) const;
    Vector9 computeGal3Error(const Gal3& predicted, const Gal3& groundTruth) const;
    Matrix9 extractNavigationCovariance(const Gal3ImuEKF& ekf) const;

    /// Trajectory data management
    TrajectoryValidator::TrajectoryPoint createGroundTruthPoint(
        const NavState& navState, double timestamp) const;
    TrajectoryValidator::TrajectoryPoint createPredictedPointFromGal3(
        const Gal3& gal3State, const Matrix9& covariance, double timestamp) const;
    TrajectoryValidator::TrajectoryPoint createPredictedPointFromNavState(
        const NavState& navState, const Matrix9& covariance, double timestamp) const;
    void storeTrajectoryData(
        std::vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
        std::vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
        std::vector<Vector9>& errorTrajectory,
        const Gal3& predictedGal3, const NavState& groundTruthNavState,
        const Matrix9& navigationCovariance, const Vector9& navigationError,
        double timestamp) const;

    /// Process single Gal3 window
    std::optional<double> processGal3Window(
        const WindowIndices& window,
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        double dt,
        std::vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
        std::vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
        std::vector<Vector9>& errorTrajectory) const;

    /// Export trajectory results with dataset name
    void exportTrajectoryResults(
        const std::vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
        const std::vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
        const std::vector<Vector9>& errorTrajectory,
        const std::string& filterName,
        const std::string& datasetName) const;

    /// Export NEES values with dataset name
    void exportNEESValues(
        const std::string& filterName,
        const std::string& datasetName,
        const std::vector<TrajectoryValidator::TrajectoryPoint>& trajectory,
        const std::vector<double>& neesValues) const;

    /// Process time window with Gal3ImuEKF (with dataset name)
    NEESEvaluator::NEESResults processTimeWindowWithGal3EKF(
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        double preintegrationTime, double dt,
        const std::string& datasetName) const;

    /// NavState helper functions
    NavStateImuEKF initializeNavStateEKF(const NavState& initialState,
                                          const std::shared_ptr<PreintegrationCombinedParams>& params) const;
    void propagateNavStateEKF(NavStateImuEKF& ekf, const WindowIndices& window,
                              const imuBias::ConstantBias& bias, double dt) const;
    Vector9 computeNavStateError(const NavState& predicted, 
                                 const NavState& groundTruth) const;

    /// Process single NavState window
    std::optional<double> processNavStateWindow(
        const WindowIndices& window,
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        double dt,
        std::vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
        std::vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
        std::vector<Vector9>& errorTrajectory) const;

    /// Process time window with NavStateImuEKF
    NEESEvaluator::NEESResults processTimeWindowWithNavStateEKF(
        const std::shared_ptr<PreintegrationCombinedParams>& params,
        double preintegrationTime, double dt,
        const std::string& datasetName) const;
};

} // namespace gtsam