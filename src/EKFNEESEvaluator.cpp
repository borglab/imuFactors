/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   EKFNEESEvaluator.cpp
 * @brief  Implementation of NEES evaluator for EKF comparison
 * @author Alec Kain
 */

#include "EKFNEESEvaluator.h"
#include "Window.h"

#include <iostream>

using namespace std;

namespace gtsam {

namespace {

template <typename PredictFunction>
void propagateWindowMeasurements(
    const Window& window,
    const imuBias::ConstantBias& windowBias,
    PredictFunction&& predictFunction) {
  window.forEachImuSample([&](const Dataset::ImuMeasurement& measurement) {
    const Vector3 omega = measurement.omega - windowBias.gyroscope();
    const Vector3 acceleration = measurement.acc - windowBias.accelerometer();
    predictFunction(omega, acceleration);
  });
}

void fillWindowPrediction(
    const Window& window,
    const std::vector<Dataset::Truth>& states,
    const TrajectoryPoint& predictedPoint,
    const Vector9& error,
    std::vector<TrajectoryPoint>& predictedTrajectory,
    std::vector<Vector9>& errorTrajectory) {
  for (size_t sampleIndex = window.start; sampleIndex <= window.end;
       ++sampleIndex) {
    predictedTrajectory[sampleIndex] = predictedPoint;
    predictedTrajectory[sampleIndex].timestamp = states[sampleIndex].timestamp;
    errorTrajectory[sampleIndex] = error;
  }
}

}  // namespace

EKFNEESEvaluator::EKFNEESEvaluator(const Dataset& dataset) : dataset_(dataset) {}

EKFNEESEvaluator::RunArtifacts EKFNEESEvaluator::computeGal3ImuEKFArtifacts(
    double interval, double alpha) const {
  return computeGal3ImuEKFArtifacts(interval, dataset_.configureImuParams(alpha));
}

EKFNEESEvaluator::RunArtifacts EKFNEESEvaluator::computeGal3ImuEKFArtifacts(
    double interval,
    const std::shared_ptr<PreintegrationCombinedParams>& params) const {
  return processTimeWindowWithGal3EKF(params, interval, dataset_.timestep());
}

EKFNEESEvaluator::RunArtifacts
EKFNEESEvaluator::computeNavStateImuEKFArtifacts(double interval,
                                                 double alpha) const {
  return computeNavStateImuEKFArtifacts(interval,
                                        dataset_.configureImuParams(alpha));
}

EKFNEESEvaluator::RunArtifacts
EKFNEESEvaluator::computeNavStateImuEKFArtifacts(
    double interval,
    const std::shared_ptr<PreintegrationCombinedParams>& params) const {
  return processTimeWindowWithNavStateEKF(params, interval, dataset_.timestep());
}

NEESResults EKFNEESEvaluator::runGal3ImuEKF(double interval, double alpha) const {
    return NEESEvaluator::computeStatistics(
        computeGal3ImuEKFArtifacts(interval, alpha).neesValues, interval);
}

/// NEW: 3-parameter version with alpha and dataset name
NEESResults EKFNEESEvaluator::runGal3ImuEKF(
    double interval, double alpha, const std::string& datasetName) const {
    (void)datasetName;
    return NEESEvaluator::computeStatistics(
        computeGal3ImuEKFArtifacts(interval, alpha).neesValues, interval);
}

NEESResults EKFNEESEvaluator::runGal3ImuEKF(
    double interval, 
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const std::string& datasetName) const {
    (void)datasetName;
    return NEESEvaluator::computeStatistics(
        computeGal3ImuEKFArtifacts(interval, params).neesValues, interval);
}

NEESResults EKFNEESEvaluator::runNavStateImuEKF(double interval, double alpha) const {
    return NEESEvaluator::computeStatistics(
        computeNavStateImuEKFArtifacts(interval, alpha).neesValues, interval);
}

/// NEW: 3-parameter version with alpha and dataset name
NEESResults EKFNEESEvaluator::runNavStateImuEKF(
    double interval, double alpha, const std::string& datasetName) const {
    (void)datasetName;
    return NEESEvaluator::computeStatistics(
        computeNavStateImuEKFArtifacts(interval, alpha).neesValues, interval);
}

NEESResults EKFNEESEvaluator::runNavStateImuEKF(
    double interval,
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const std::string& datasetName) const {
    (void)datasetName;
    return NEESEvaluator::computeStatistics(
        computeNavStateImuEKFArtifacts(interval, params).neesValues, interval);
}

Gal3 EKFNEESEvaluator::convertToGal3(const NavState& navState, double time) const {
    return Gal3(navState.pose().rotation(), 
               navState.pose().translation(),
               navState.velocity(), time);
}

Gal3ImuEKF EKFNEESEvaluator::initializeGal3EKF(
    const NavState& initialState,
    const std::shared_ptr<PreintegrationCombinedParams>& params) const {
    Gal3 initialGal3State = convertToGal3(initialState, 0.0);
    
    /// Start with zero initial covariance - perfect knowledge
    Matrix initialCovariance = Matrix::Zero(10, 10);
    initialCovariance(9, 9) = 0.0;
    
    return Gal3ImuEKF(initialGal3State, initialCovariance, params, 
                     Gal3ImuEKF::TRACK_TIME_NO_COVARIANCE);
}

NavStateImuEKF EKFNEESEvaluator::initializeNavStateEKF(
    const NavState& initialState,
    const std::shared_ptr<PreintegrationCombinedParams>& params) const {
    Matrix9 initialCovariance = Matrix9::Zero();
    
    auto navStateParams = std::make_shared<PreintegrationParams>(params->n_gravity);
    navStateParams->accelerometerCovariance = params->accelerometerCovariance;
    navStateParams->gyroscopeCovariance = params->gyroscopeCovariance;
    navStateParams->integrationCovariance = params->integrationCovariance;
    navStateParams->use2ndOrderCoriolis = params->use2ndOrderCoriolis;
    
    return NavStateImuEKF(initialState, initialCovariance, navStateParams);
}

Vector9 EKFNEESEvaluator::computeGal3Error(const Gal3& predicted, const Gal3& groundTruth) const {
    Vector10 error10 = predicted.logmap(groundTruth);
    return error10.head<9>();
}

Matrix9 EKFNEESEvaluator::extractNavigationCovariance(const Gal3ImuEKF& ekf) const {
    Matrix ekfCovariance = ekf.covariance();
    return ekfCovariance.block<9, 9>(0, 0);
}

TrajectoryPoint EKFNEESEvaluator::createGroundTruthPoint(
    const NavState& navState, double timestamp) const {
    TrajectoryPoint point;
    point.timestamp = timestamp;
    point.position = navState.position();
    point.velocity = navState.velocity();
    point.rpy = radiansToDegrees(navState.attitude().rpy());
    return point;
}

TrajectoryPoint EKFNEESEvaluator::createPredictedPointFromGal3(
    const Gal3& gal3State, const Matrix9& covariance, double timestamp) const {
    TrajectoryPoint point;
    point.timestamp = timestamp;
    point.position = gal3State.translation();
    point.velocity = gal3State.velocity();
    point.rpy = radiansToDegrees(gal3State.attitude().rpy());
    point.covariance = covariance;
    return point;
}

TrajectoryPoint EKFNEESEvaluator::createPredictedPointFromNavState(
    const NavState& navState, const Matrix9& covariance, double timestamp) const {
    TrajectoryPoint point;
    point.timestamp = timestamp;
    point.position = navState.position();
    point.velocity = navState.velocity();
    point.rpy = radiansToDegrees(navState.attitude().rpy());
    point.covariance = covariance;
    return point;
}

EKFNEESEvaluator::RunArtifacts EKFNEESEvaluator::processTimeWindowWithGal3EKF(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double preintegrationTime, double dt) const {
    RunArtifacts artifacts;
    artifacts.preintegrationTime = preintegrationTime;

    vector<TrajectoryPoint> predictedTrajectory;
    vector<Vector9> errorTrajectory;

    const auto& states = dataset_.truth;
    const size_t stepsPerWindow = dataset_.stepsForInterval(preintegrationTime);
    artifacts.samplesPerWindow = stepsPerWindow;
    const auto windows = dataset_.completeWindows(stepsPerWindow);
    const size_t trajectoryLength = windows.empty() ? 0 : windows.back().end + 1;

    predictedTrajectory.resize(trajectoryLength);
    errorTrajectory.resize(trajectoryLength, Vector9::Zero());

    for (size_t windowIdx = 0; windowIdx < windows.size(); ++windowIdx) {
        const auto& window = windows[windowIdx];
        Gal3ImuEKF ekf = initializeGal3EKF(window.initialTruth().navState, params);
        const imuBias::ConstantBias& windowBias = window.initialTruth().bias;

        propagateWindowMeasurements(window, windowBias, [&](const Vector3& omega,
                                                            const Vector3& acceleration) {
            ekf.predict(omega, acceleration, dt);
        });
        
        /// Get end-of-window prediction
        const Gal3 predictedGal3 = ekf.state();
        const NavState& groundTruthNavState = window.terminalTruth().navState;
        const Gal3 groundTruthGal3 = convertToGal3(groundTruthNavState, predictedGal3.time());
        const Vector9 navigationError = computeGal3Error(predictedGal3, groundTruthGal3);
        const Matrix9 navigationCovariance = extractNavigationCovariance(ekf);

        fillWindowPrediction(
            window, states,
            createPredictedPointFromGal3(
                predictedGal3, navigationCovariance, window.terminalTruth().timestamp),
            navigationError, predictedTrajectory, errorTrajectory);

        auto nees = normalizedNEES(
            navigationError, navigationCovariance,
            static_cast<double>(navigationError.size()));
        if (nees) {
            artifacts.neesValues.push_back(*nees);
            artifacts.windowEvaluations.push_back(
                {windowIdx, window.start, window.end,
                 window.initialTruth().timestamp, window.terminalTruth().timestamp,
                 makeWindowResult(navigationError, navigationCovariance, *nees)});
        }
    }

    for (size_t sampleIndex = 0; sampleIndex < trajectoryLength; ++sampleIndex) {
        artifacts.trajectorySamples.push_back(
            {sampleIndex,
             createGroundTruthPoint(states[sampleIndex].navState,
                                    states[sampleIndex].timestamp),
             predictedTrajectory[sampleIndex], errorTrajectory[sampleIndex]});
    }

    return artifacts;
}

/// Same for NavState
EKFNEESEvaluator::RunArtifacts EKFNEESEvaluator::processTimeWindowWithNavStateEKF(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double preintegrationTime, double dt) const {
    RunArtifacts artifacts;
    artifacts.preintegrationTime = preintegrationTime;

    vector<TrajectoryPoint> predictedTrajectory;
    vector<Vector9> errorTrajectory;

    const auto& states = dataset_.truth;
    const size_t stepsPerWindow = dataset_.stepsForInterval(preintegrationTime);
    artifacts.samplesPerWindow = stepsPerWindow;
    const auto windows = dataset_.completeWindows(stepsPerWindow);
    const size_t trajectoryLength = windows.empty() ? 0 : windows.back().end + 1;

    predictedTrajectory.resize(trajectoryLength);
    errorTrajectory.resize(trajectoryLength, Vector9::Zero());

    for (size_t windowIdx = 0; windowIdx < windows.size(); ++windowIdx) {
        const auto& window = windows[windowIdx];
        NavStateImuEKF ekf = initializeNavStateEKF(window.initialTruth().navState, params);
        const imuBias::ConstantBias& windowBias = window.initialTruth().bias;

        propagateWindowMeasurements(window, windowBias, [&](const Vector3& omega,
                                                            const Vector3& acceleration) {
            ekf.predict(omega, acceleration, dt);
        });
        
        const NavState predicted = ekf.state();
        const NavState& groundTruth = window.terminalTruth().navState;
        const Vector9 error = groundTruth.logmap(predicted);
        const Matrix9 navigationCovariance = ekf.covariance();

        fillWindowPrediction(
            window, states,
            createPredictedPointFromNavState(
                predicted, navigationCovariance, window.terminalTruth().timestamp),
            error, predictedTrajectory, errorTrajectory);

        auto nees = normalizedNEES(
            error, navigationCovariance, static_cast<double>(error.size()));
        if (nees) {
            artifacts.neesValues.push_back(*nees);
            artifacts.windowEvaluations.push_back(
                {windowIdx, window.start, window.end,
                 window.initialTruth().timestamp, window.terminalTruth().timestamp,
                 makeWindowResult(error, navigationCovariance, *nees)});
        }
    }

    for (size_t sampleIndex = 0; sampleIndex < trajectoryLength; ++sampleIndex) {
        artifacts.trajectorySamples.push_back(
            {sampleIndex,
             createGroundTruthPoint(states[sampleIndex].navState,
                                    states[sampleIndex].timestamp),
             predictedTrajectory[sampleIndex], errorTrajectory[sampleIndex]});
    }

    return artifacts;
}
}
