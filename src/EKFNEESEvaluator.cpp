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
#include "TrajectoryValidator.h"
#include "Window.h"
#include "nees.h"

#include <fstream>
#include <iomanip>
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

std::string preintegrationTimeLabel(double preintegrationTime) {
  return std::to_string(static_cast<int>(preintegrationTime * 10)) + "s";
}

}  // namespace

EKFNEESEvaluator::EKFNEESEvaluator(const Dataset& dataset) : dataset_(dataset) {}

NEESResults EKFNEESEvaluator::runGal3ImuEKF(double interval, double alpha) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = dataset_.timestep();
    return processTimeWindowWithGal3EKF(params, interval, dt, "default");
}

/// NEW: 3-parameter version with alpha and dataset name
NEESResults EKFNEESEvaluator::runGal3ImuEKF(
    double interval, double alpha, const std::string& datasetName) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = dataset_.timestep();
    return processTimeWindowWithGal3EKF(params, interval, dt, datasetName);
}

NEESResults EKFNEESEvaluator::runGal3ImuEKF(
    double interval, 
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const std::string& datasetName) const {
    double dt = dataset_.timestep();
    return processTimeWindowWithGal3EKF(params, interval, dt, datasetName);
}

NEESResults EKFNEESEvaluator::runNavStateImuEKF(double interval, double alpha) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = dataset_.timestep();
    return processTimeWindowWithNavStateEKF(params, interval, dt, "default");
}

/// NEW: 3-parameter version with alpha and dataset name
NEESResults EKFNEESEvaluator::runNavStateImuEKF(
    double interval, double alpha, const std::string& datasetName) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = dataset_.timestep();
    return processTimeWindowWithNavStateEKF(params, interval, dt, datasetName);
}

NEESResults EKFNEESEvaluator::runNavStateImuEKF(
    double interval,
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const std::string& datasetName) const {
    double dt = dataset_.timestep();
    return processTimeWindowWithNavStateEKF(params, interval, dt, datasetName);
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

void EKFNEESEvaluator::exportTrajectoryResults(
    const vector<TrajectoryPoint>& groundTruthTrajectory,
    const vector<TrajectoryPoint>& predictedTrajectory,
    const vector<Vector9>& errorTrajectory,
    const std::string& filterName,
    const std::string& datasetName,
    const std::string& preintegrationTime) const {
    
    std::string filename = filterName + "_trajectory_" + datasetName + "_" + preintegrationTime + ".csv";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    /// CSV header WITHOUT acceleration columns
    file << "timestamp,gt_x,gt_y,gt_z,gt_vx,gt_vy,gt_vz,gt_roll,gt_pitch,gt_yaw,"
         << "pred_x,pred_y,pred_z,pred_vx,pred_vy,pred_vz,pred_roll,pred_pitch,pred_yaw,"
         << "err_x,err_y,err_z,err_vx,err_vy,err_vz,err_roll,err_pitch,err_yaw\n";
    
    for (size_t i = 0; i < groundTruthTrajectory.size(); i++) {
        const auto& gt = groundTruthTrajectory[i];
        const auto& pred = predictedTrajectory[i];
        const auto& err = errorTrajectory[i];
        
        /// Ground truth (position, velocity, orientation)
        file << std::fixed << std::setprecision(6)
             << gt.timestamp << ","
             << gt.position.x() << "," << gt.position.y() << "," << gt.position.z() << ","
             << gt.velocity.x() << "," << gt.velocity.y() << "," << gt.velocity.z() << ","
             << gt.rpy.x() << "," << gt.rpy.y() << "," << gt.rpy.z() << ",";
        
        /// Prediction (position, velocity, orientation)
        file << pred.position.x() << "," << pred.position.y() << "," << pred.position.z() << ","
             << pred.velocity.x() << "," << pred.velocity.y() << "," << pred.velocity.z() << ","
             << pred.rpy.x() << "," << pred.rpy.y() << "," << pred.rpy.z() << ",";
        
        /// Error (9D: position, velocity, orientation)
        file << err[0] << "," << err[1] << "," << err[2] << ","  // position error
             << err[3] << "," << err[4] << "," << err[5] << ","  // velocity error
             << err[6] << "," << err[7] << "," << err[8] << "\n";  // orientation error
    }
    
    file.close();
    std::cout << "✓ Exported trajectory: " << filename << std::endl;
}

void EKFNEESEvaluator::exportNEESValues(
    const std::string& filterName,
    const std::string& datasetName,
    const std::string& preintegrationTime,
    const std::vector<TrajectoryPoint>& trajectory,
    const std::vector<double>& neesValues) const {
    
    std::string filename = filterName + "_nees_" + datasetName + "_" + preintegrationTime + ".csv";
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open NEES file: " << filename << std::endl;
        return;
    }
    
    file << "timestamp,nees\n";
    file << std::fixed << std::setprecision(6);
    
    for (size_t i = 0; i < neesValues.size(); ++i) {
        size_t trajIdx = std::min(i * (trajectory.size() / neesValues.size()), 
                                 trajectory.size() - 1);
        file << trajectory[trajIdx].timestamp << "," << neesValues[i] << "\n";
    }
    
    file.close();
    std::cout << "✓ Exported NEES values to " << filename << std::endl;
}

NEESResults EKFNEESEvaluator::processTimeWindowWithGal3EKF(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double preintegrationTime, double dt,
    const std::string& datasetName) const {
    
    vector<double> neesValues;
    vector<TrajectoryPoint> groundTruthTrajectory;
    vector<TrajectoryPoint> predictedTrajectory;
    vector<Vector9> errorTrajectory;
    
    const auto& states = dataset_.truth;
    const int stepsPerWindow = static_cast<int>(dataset_.stepsForInterval(preintegrationTime));
    const auto windows = dataset_.completeWindows(static_cast<size_t>(stepsPerWindow));
    const int numCompleteWindows = static_cast<int>(windows.size());
    const size_t actualEndIndex = windows.empty() ? 0 : windows.back().end;
    
    /// Store COMPLETE ground truth trajectory (all points)
    std::cout << "Storing complete ground truth: " << states.size() << " points" << std::endl;
    for (size_t i = 0; i < states.size(); i++) {
        groundTruthTrajectory.push_back(
            createGroundTruthPoint(states[i].navState, states[i].timestamp));
    }
    
    std::cout << "Gal3 " << datasetName << " @ " << preintegrationTime 
              << "s: " << stepsPerWindow << " steps/window, " 
              << numCompleteWindows << " complete windows, "
              << "ending at index " << actualEndIndex << "/" << (states.size() - 1) << std::endl;
    
    /// Initialize prediction trajectory with placeholders
    predictedTrajectory.resize(states.size());
    errorTrajectory.resize(states.size());
    
    /// Process each COMPLETE window
    for (size_t windowIdx = 0; windowIdx < windows.size(); ++windowIdx) {
        const auto& window = windows[windowIdx];
        
        /// RESET: Initialize new EKF at window start
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
        
        /// FILL THE ENTIRE WINDOW with this prediction (piecewise constant)
        fillWindowPrediction(
            window, states,
            createPredictedPointFromGal3(
                predictedGal3, navigationCovariance, window.terminalTruth().timestamp),
            navigationError, predictedTrajectory, errorTrajectory);
        
        /// Compute NEES (only once per window)
        auto nees = normalizedNEES(
            navigationError, navigationCovariance,
            static_cast<double>(navigationError.size()));
        if (nees) {
            neesValues.push_back(*nees);
            if (windowIdx < 3) {
                std::cout << "  Window " << (windowIdx + 1) 
                          << ": NEES = " << *nees << std::endl;
            }
        }
    }
    
    std::string timeStr = preintegrationTimeLabel(preintegrationTime);
    
    std::cout << "Exporting: GT=" << groundTruthTrajectory.size() << std::endl;
    
    exportTrajectoryResults(groundTruthTrajectory, predictedTrajectory, errorTrajectory, 
                          "gal3", datasetName, timeStr);
    exportNEESValues("gal3", datasetName, timeStr, predictedTrajectory, neesValues);
    
    TrajectoryValidator::printErrorStatistics(errorTrajectory);
    
    return NEESEvaluator::computeStatistics(neesValues, preintegrationTime);
}

/// Same for NavState
NEESResults EKFNEESEvaluator::processTimeWindowWithNavStateEKF(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double preintegrationTime, double dt,
    const std::string& datasetName) const {
    
    vector<double> neesValues;
    vector<TrajectoryPoint> groundTruthTrajectory;
    vector<TrajectoryPoint> predictedTrajectory;
    vector<Vector9> errorTrajectory;
    
    const auto& states = dataset_.truth;
    const int stepsPerWindow = static_cast<int>(dataset_.stepsForInterval(preintegrationTime));
    const auto windows = dataset_.completeWindows(static_cast<size_t>(stepsPerWindow));
    const int numCompleteWindows = static_cast<int>(windows.size());
    const size_t actualEndIndex = windows.empty() ? 0 : windows.back().end;
    
    /// Store complete ground truth
    for (size_t i = 0; i < states.size(); i++) {
        groundTruthTrajectory.push_back(
            createGroundTruthPoint(states[i].navState, states[i].timestamp));
    }
    
    std::cout << "NavState " << datasetName << " @ " << preintegrationTime 
              << "s: " << stepsPerWindow << " steps/window, " 
              << numCompleteWindows << " complete windows, "
              << "ending at index " << actualEndIndex << "/" << (states.size() - 1) << std::endl;
    
    /// Pre-allocate prediction arrays
    predictedTrajectory.resize(states.size());
    errorTrajectory.resize(states.size());
    
    for (const auto& window : windows) {
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
        
        /// Fill entire window with constant prediction
        fillWindowPrediction(
            window, states,
            createPredictedPointFromNavState(
                predicted, navigationCovariance, window.terminalTruth().timestamp),
            error, predictedTrajectory, errorTrajectory);
        
        auto nees = normalizedNEES(
            error, navigationCovariance, static_cast<double>(error.size()));
        if (nees) {
            neesValues.push_back(*nees);
        }
    }
    
    std::string timeStr = preintegrationTimeLabel(preintegrationTime);
    
    exportTrajectoryResults(groundTruthTrajectory, predictedTrajectory, errorTrajectory,
                          "navstate", datasetName, timeStr);
    exportNEESValues("navstate", datasetName, timeStr, predictedTrajectory, neesValues);
    
    TrajectoryValidator::printErrorStatistics(errorTrajectory);
    
    return NEESEvaluator::computeStatistics(neesValues, preintegrationTime);
}
}
