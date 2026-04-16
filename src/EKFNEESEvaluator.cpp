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
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

namespace gtsam {

EKFNEESEvaluator::EKFNEESEvaluator(const Dataset& dataset) : dataset_(dataset) {}

double EKFNEESEvaluator::computeTimestep() const {
    const auto& states = dataset_.getStates();
    return states[1].timestamp - states[0].timestamp;
}

NEESEvaluator::NEESResults EKFNEESEvaluator::runGal3ImuEKF(double interval, double alpha) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = computeTimestep();
    return processTimeWindowWithGal3EKF(params, interval, dt, "default");
}

/// NEW: 3-parameter version with alpha and dataset name
NEESEvaluator::NEESResults EKFNEESEvaluator::runGal3ImuEKF(
    double interval, double alpha, const std::string& datasetName) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = computeTimestep();
    return processTimeWindowWithGal3EKF(params, interval, dt, datasetName);
}

NEESEvaluator::NEESResults EKFNEESEvaluator::runGal3ImuEKF(
    double interval, 
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const std::string& datasetName) const {
    double dt = computeTimestep();
    return processTimeWindowWithGal3EKF(params, interval, dt, datasetName);
}

NEESEvaluator::NEESResults EKFNEESEvaluator::runNavStateImuEKF(double interval, double alpha) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = computeTimestep();
    return processTimeWindowWithNavStateEKF(params, interval, dt, "default");
}

/// NEW: 3-parameter version with alpha and dataset name
NEESEvaluator::NEESResults EKFNEESEvaluator::runNavStateImuEKF(
    double interval, double alpha, const std::string& datasetName) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = computeTimestep();
    return processTimeWindowWithNavStateEKF(params, interval, dt, datasetName);
}

NEESEvaluator::NEESResults EKFNEESEvaluator::runNavStateImuEKF(
    double interval,
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const std::string& datasetName) const {
    double dt = computeTimestep();
    return processTimeWindowWithNavStateEKF(params, interval, dt, datasetName);
}

std::optional<double> EKFNEESEvaluator::computeNEES(const Vector& error, 
                                                     const Matrix& covarianceMatrix) const {
    try {
        const size_t degreesOfFreedom = error.size();
        const Matrix covarianceInverse = covarianceMatrix.inverse();
        const double nees = (error.transpose() * covarianceInverse * error)(0, 0) / degreesOfFreedom;
        return (nees > 0.0) ? std::optional<double>(nees) : std::nullopt;
    } catch (...) {
        return std::nullopt;
    }
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

TrajectoryValidator::TrajectoryPoint EKFNEESEvaluator::createGroundTruthPoint(
    const NavState& navState, double timestamp) const {
    TrajectoryValidator::TrajectoryPoint point;
    point.timestamp = timestamp;
    point.position = navState.position();
    point.velocity = navState.velocity();
    point.rpy = navState.attitude().rpy() * 180.0 / M_PI;
    return point;
}

TrajectoryValidator::TrajectoryPoint EKFNEESEvaluator::createPredictedPointFromGal3(
    const Gal3& gal3State, const Matrix9& covariance, double timestamp) const {
    TrajectoryValidator::TrajectoryPoint point;
    point.timestamp = timestamp;
    point.position = gal3State.translation();
    point.velocity = gal3State.velocity();
    point.rpy = gal3State.attitude().rpy() * 180.0 / M_PI;
    point.covariance = covariance;
    return point;
}

TrajectoryValidator::TrajectoryPoint EKFNEESEvaluator::createPredictedPointFromNavState(
    const NavState& navState, const Matrix9& covariance, double timestamp) const {
    TrajectoryValidator::TrajectoryPoint point;
    point.timestamp = timestamp;
    point.position = navState.position();
    point.velocity = navState.velocity();
    point.rpy = navState.attitude().rpy() * 180.0 / M_PI;
    point.covariance = covariance;
    return point;
}

void EKFNEESEvaluator::exportTrajectoryResults(
    const vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
    const vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
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
    const std::vector<TrajectoryValidator::TrajectoryPoint>& trajectory,
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

NEESEvaluator::NEESResults EKFNEESEvaluator::processTimeWindowWithGal3EKF(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double preintegrationTime, double dt,
    const std::string& datasetName) const {
    
    vector<double> neesValues;
    vector<TrajectoryValidator::TrajectoryPoint> groundTruthTrajectory;
    vector<TrajectoryValidator::TrajectoryPoint> predictedTrajectory;
    vector<Vector9> errorTrajectory;
    
    const auto& states = dataset_.getStates();
    const auto& imuData = dataset_.getImuData();
    
    /// Store COMPLETE ground truth trajectory (all points)
    std::cout << "Storing complete ground truth: " << states.size() << " points" << std::endl;
    for (size_t i = 0; i < states.size(); i++) {
        groundTruthTrajectory.push_back(
            createGroundTruthPoint(states[i].navState, states[i].timestamp));
    }
    
    /// Compute steps per window
    const int stepsPerWindow = static_cast<int>(preintegrationTime / dt + 0.5);
    const int numCompleteWindows = (states.size() - 1) / stepsPerWindow;  // Integer division
    
    /// Calculate actual end index (exclude incomplete final window)
    const size_t actualEndIndex = numCompleteWindows * stepsPerWindow;
    
    std::cout << "Gal3 " << datasetName << " @ " << preintegrationTime 
              << "s: " << stepsPerWindow << " steps/window, " 
              << numCompleteWindows << " complete windows, "
              << "ending at index " << actualEndIndex << "/" << (states.size() - 1) << std::endl;
    
    /// Initialize prediction trajectory with placeholders
    predictedTrajectory.resize(states.size());
    errorTrajectory.resize(states.size());
    
    /// Process each COMPLETE window
    for (int windowIdx = 0; windowIdx < numCompleteWindows; windowIdx++) {
        const size_t windowStart = windowIdx * stepsPerWindow;
        const size_t windowEnd = windowStart + stepsPerWindow;  // No min() needed
        
        /// RESET: Initialize new EKF at window start
        Gal3ImuEKF ekf = initializeGal3EKF(states[windowStart].navState, params);
        const imuBias::ConstantBias windowBias = states[windowStart].bias;
        
        /// Integrate through this window
        for (size_t k = windowStart; k < windowEnd; k++) {
            Vector3 omega = imuData[k].omega - windowBias.gyroscope();
            Vector3 acceleration = imuData[k].acc - windowBias.accelerometer();
            
            ekf.predict(omega, acceleration, dt);
        }
        
        /// Get end-of-window prediction
        const Gal3 predictedGal3 = ekf.state();
        const NavState& groundTruthNavState = states[windowEnd].navState;
        const Gal3 groundTruthGal3 = convertToGal3(groundTruthNavState, predictedGal3.time());
        const Vector9 navigationError = computeGal3Error(predictedGal3, groundTruthGal3);
        const Matrix9 navigationCovariance = extractNavigationCovariance(ekf);
        
        /// FILL THE ENTIRE WINDOW with this prediction (piecewise constant)
        for (size_t k = windowStart; k <= windowEnd; k++) {
            predictedTrajectory[k] = createPredictedPointFromGal3(
                predictedGal3, navigationCovariance, states[k].timestamp);
            errorTrajectory[k] = navigationError;
        }
        
        /// Compute NEES (only once per window)
        auto nees = computeNEES(navigationError, navigationCovariance);
        if (nees) {
            neesValues.push_back(*nees);
            if (windowIdx < 3) {
                std::cout << "  Window " << (windowIdx + 1) 
                          << ": NEES = " << *nees << std::endl;
            }
        }
    }
    
    std::string timeStr = std::to_string(static_cast<int>(preintegrationTime * 10)) + "s";
    
    std::cout << "Exporting: GT=" << groundTruthTrajectory.size() << std::endl;
    
    exportTrajectoryResults(groundTruthTrajectory, predictedTrajectory, errorTrajectory, 
                          "gal3", datasetName, timeStr);
    exportNEESValues("gal3", datasetName, timeStr, predictedTrajectory, neesValues);
    
    TrajectoryValidator::printErrorStatistics(errorTrajectory);
    
    return NEESEvaluator::computeStatistics(neesValues, preintegrationTime);
}

/// Same for NavState
NEESEvaluator::NEESResults EKFNEESEvaluator::processTimeWindowWithNavStateEKF(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double preintegrationTime, double dt,
    const std::string& datasetName) const {
    
    vector<double> neesValues;
    vector<TrajectoryValidator::TrajectoryPoint> groundTruthTrajectory;
    vector<TrajectoryValidator::TrajectoryPoint> predictedTrajectory;
    vector<Vector9> errorTrajectory;
    
    const auto& states = dataset_.getStates();
    const auto& imuData = dataset_.getImuData();
    
    /// Store complete ground truth
    for (size_t i = 0; i < states.size(); i++) {
        groundTruthTrajectory.push_back(
            createGroundTruthPoint(states[i].navState, states[i].timestamp));
    }
    
    const int stepsPerWindow = static_cast<int>(preintegrationTime / dt + 0.5);
    const int numCompleteWindows = (states.size() - 1) / stepsPerWindow;
    const size_t actualEndIndex = numCompleteWindows * stepsPerWindow;
    
    std::cout << "NavState " << datasetName << " @ " << preintegrationTime 
              << "s: " << stepsPerWindow << " steps/window, " 
              << numCompleteWindows << " complete windows, "
              << "ending at index " << actualEndIndex << "/" << (states.size() - 1) << std::endl;
    
    /// Pre-allocate prediction arrays
    predictedTrajectory.resize(states.size());
    errorTrajectory.resize(states.size());
    
    for (int windowIdx = 0; windowIdx < numCompleteWindows; windowIdx++) {
        const size_t windowStart = windowIdx * stepsPerWindow;
        const size_t windowEnd = windowStart + stepsPerWindow;  // No min() needed
        
        NavStateImuEKF ekf = initializeNavStateEKF(states[windowStart].navState, params);
        const imuBias::ConstantBias windowBias = states[windowStart].bias;
        
        for (size_t k = windowStart; k < windowEnd; k++) {
            Vector3 omega = imuData[k].omega - windowBias.gyroscope();
            Vector3 acceleration = imuData[k].acc - windowBias.accelerometer();
            ekf.predict(omega, acceleration, dt);
        }
        
        const NavState predicted = ekf.state();
        const NavState& groundTruth = states[windowEnd].navState;
        const Vector9 error = groundTruth.logmap(predicted);
        const Matrix9 navigationCovariance = ekf.covariance();
        
        /// Fill entire window with constant prediction
        for (size_t k = windowStart; k <= windowEnd; k++) {
            predictedTrajectory[k] = createPredictedPointFromNavState(
                predicted, navigationCovariance, states[k].timestamp);
            errorTrajectory[k] = error;
        }
        
        auto nees = computeNEES(error, navigationCovariance);
        if (nees) {
            neesValues.push_back(*nees);
        }
    }
    
    std::string timeStr = std::to_string(static_cast<int>(preintegrationTime * 10)) + "s";
    
    exportTrajectoryResults(groundTruthTrajectory, predictedTrajectory, errorTrajectory,
                          "navstate", datasetName, timeStr);
    exportNEESValues("navstate", datasetName, timeStr, predictedTrajectory, neesValues);
    
    TrajectoryValidator::printErrorStatistics(errorTrajectory);
    
    return NEESEvaluator::computeStatistics(neesValues, preintegrationTime);
}
}