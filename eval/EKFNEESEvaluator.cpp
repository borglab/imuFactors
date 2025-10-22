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

NEESEvaluator::NEESResults EKFNEESEvaluator::runGal3ImuEKF(double interval, double alpha, 
                                                           const std::string& datasetName) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = computeTimestep();
    return processTimeWindowWithGal3EKF(params, interval, dt, datasetName);
}

NEESEvaluator::NEESResults EKFNEESEvaluator::runNavStateImuEKF(double interval, double alpha) const {
    auto params = dataset_.configureImuParams(alpha);
    double dt = computeTimestep();
    return processTimeWindowWithNavStateEKF(params, interval, dt, "default");
}

NEESEvaluator::NEESResults EKFNEESEvaluator::runNavStateImuEKF(double interval, double alpha,
                                                                const std::string& datasetName) const {
    auto params = dataset_.configureImuParams(alpha);
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

EKFNEESEvaluator::WindowIndices EKFNEESEvaluator::computeWindowIndices(
    int windowNumber, int windowSize) const {
    const int startIndex = windowNumber * windowSize;
    const int endIndex = std::min((windowNumber + 1) * windowSize, 
                                 static_cast<int>(dataset_.getStates().size()));
    return {startIndex, endIndex};
}

bool EKFNEESEvaluator::isWindowValid(const WindowIndices& window) const {
    const int stateSize = static_cast<int>(dataset_.getStates().size());
    const int imuSize = static_cast<int>(dataset_.getImuData().size());
    return window.endIndex < stateSize && window.endIndex < imuSize;
}

Gal3 EKFNEESEvaluator::convertToGal3(const NavState& navState, double time) const {
    return Gal3(navState.pose().rotation(), 
               navState.pose().translation(),
               navState.velocity(), time);
}

Matrix EKFNEESEvaluator::initializeGal3Covariance() const {
    Matrix initialCovariance = Matrix::Zero(10, 10);
    initialCovariance(9, 9) = 0.0;
    return initialCovariance;
}

Gal3ImuEKF EKFNEESEvaluator::initializeGal3EKF(
    const NavState& initialState,
    const std::shared_ptr<PreintegrationCombinedParams>& params) const {
    Gal3 initialGal3State = convertToGal3(initialState, 0.0);
    Matrix initialCovariance = initializeGal3Covariance();
    return Gal3ImuEKF(initialGal3State, initialCovariance, params, 
                     Gal3ImuEKF::TRACK_TIME_NO_COVARIANCE);
}

void EKFNEESEvaluator::propagateGal3EKF(Gal3ImuEKF& ekf, const WindowIndices& window,
                                        const imuBias::ConstantBias& bias, double dt) const {
    const auto& imuData = dataset_.getImuData();
    
    for (int k = window.startIndex; k < window.endIndex - 1; k++) {
        Vector3 omega = imuData[k].omega - bias.gyroscope();
        Vector3 acceleration = imuData[k].acc - bias.accelerometer();
        ekf.predict(omega, acceleration, dt);
    }
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

void EKFNEESEvaluator::storeTrajectoryData(
    vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
    vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
    vector<Vector9>& errorTrajectory,
    const Gal3& predictedGal3, const NavState& groundTruthNavState,
    const Matrix9& navigationCovariance, const Vector9& navigationError,
    double timestamp) const {
    
    groundTruthTrajectory.push_back(createGroundTruthPoint(groundTruthNavState, timestamp));
    predictedTrajectory.push_back(createPredictedPointFromGal3(predictedGal3, navigationCovariance, timestamp));
    errorTrajectory.push_back(navigationError);
}

std::optional<double> EKFNEESEvaluator::processGal3Window(
    const WindowIndices& window,
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double dt,
    vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
    vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
    vector<Vector9>& errorTrajectory) const {
    
    if (!isWindowValid(window)) return std::nullopt;
    
    const auto& states = dataset_.getStates();
    Gal3ImuEKF ekf = initializeGal3EKF(states[window.startIndex].navState, params);
    propagateGal3EKF(ekf, window, states[window.startIndex].bias, dt);
    
    const Gal3 predictedGal3 = ekf.state();
    const NavState& groundTruthNavState = states[window.endIndex - 1].navState;
    const Gal3 groundTruthGal3 = convertToGal3(groundTruthNavState, predictedGal3.time());
    
    const Vector9 navigationError = computeGal3Error(predictedGal3, groundTruthGal3);
    const Matrix9 navigationCovariance = extractNavigationCovariance(ekf);
    
    storeTrajectoryData(groundTruthTrajectory, predictedTrajectory, errorTrajectory,
                      predictedGal3, groundTruthNavState, navigationCovariance,
                      navigationError, states[window.endIndex - 1].timestamp);
    
    return computeNEES(navigationError, navigationCovariance);
}

void EKFNEESEvaluator::exportTrajectoryResults(
    const vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
    const vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
    const vector<Vector9>& errorTrajectory,
    const std::string& filterName,
    const std::string& datasetName) const {
    
    std::string filename = filterName + "_trajectory_" + datasetName + ".csv";
    TrajectoryValidator::exportToCSV(filename, 
                                    groundTruthTrajectory, 
                                    predictedTrajectory, 
                                    errorTrajectory);
    TrajectoryValidator::printErrorStatistics(errorTrajectory);
}

void EKFNEESEvaluator::exportNEESValues(
    const std::string& filterName,
    const std::string& datasetName,
    const std::vector<TrajectoryValidator::TrajectoryPoint>& trajectory,
    const std::vector<double>& neesValues) const {
    
    std::string filename = filterName + "_nees_cpp_" + datasetName + ".csv";
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open NEES file: " << filename << std::endl;
        return;
    }
    
    file << "timestamp,nees_cpp\n";
    file << std::fixed << std::setprecision(6);
    
    for (size_t i = 0; i < std::min(trajectory.size(), neesValues.size()); ++i) {
        file << trajectory[i].timestamp << "," << neesValues[i] << "\n";
    }
    
    file.close();
    std::cout << "✓ Exported NEES values to " << filename << std::endl;
}

NEESEvaluator::NEESResults EKFNEESEvaluator::processTimeWindowWithGal3EKF(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double preintegrationTime, double dt,
    const std::string& datasetName) const {
    
    const double totalTime = dataset_.getStates().back().timestamp - 
                            dataset_.getStates().front().timestamp;
    const int windowCount = static_cast<int>(totalTime / preintegrationTime);
    const int windowSize = static_cast<int>(dataset_.getStates().size() / windowCount);
    
    vector<double> neesValues;
    vector<TrajectoryValidator::TrajectoryPoint> groundTruthTrajectory;
    vector<TrajectoryValidator::TrajectoryPoint> predictedTrajectory;
    vector<Vector9> errorTrajectory;
    
    for (int m = 0; m < windowCount; m++) {
        WindowIndices window = computeWindowIndices(m, windowSize);
        auto nees = processGal3Window(window, params, dt, groundTruthTrajectory, 
                                     predictedTrajectory, errorTrajectory);
        if (nees) {
            neesValues.push_back(*nees);
        }
    }
    
    exportTrajectoryResults(groundTruthTrajectory, predictedTrajectory, errorTrajectory, 
                          "gal3", datasetName);
    exportNEESValues("gal3", datasetName, predictedTrajectory, neesValues);
    
    return NEESEvaluator::computeStatistics(neesValues, preintegrationTime);
}

NavStateImuEKF EKFNEESEvaluator::initializeNavStateEKF(
    const NavState& initialState,
    const std::shared_ptr<PreintegrationCombinedParams>& params) const {
    Matrix9 initialCovariance = Matrix9::Zero();
    return NavStateImuEKF(initialState, initialCovariance, params);
}

void EKFNEESEvaluator::propagateNavStateEKF(NavStateImuEKF& ekf, const WindowIndices& window,
                                            const imuBias::ConstantBias& bias, double dt) const {
    const auto& imuData = dataset_.getImuData();
    
    for (int k = window.startIndex; k < window.endIndex - 1; k++) {
        Vector3 omega = imuData[k].omega - bias.gyroscope();
        Vector3 acceleration = imuData[k].acc - bias.accelerometer();
        ekf.predict(omega, acceleration, dt);
    }
}

Vector9 EKFNEESEvaluator::computeNavStateError(const NavState& predicted, 
                                              const NavState& groundTruth) const {
    return predicted.logmap(groundTruth);
}

std::optional<double> EKFNEESEvaluator::processNavStateWindow(
    const WindowIndices& window,
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double dt,
    vector<TrajectoryValidator::TrajectoryPoint>& groundTruthTrajectory,
    vector<TrajectoryValidator::TrajectoryPoint>& predictedTrajectory,
    vector<Vector9>& errorTrajectory) const {
    
    if (!isWindowValid(window)) return std::nullopt;
    
    const auto& states = dataset_.getStates();
    NavStateImuEKF ekf = initializeNavStateEKF(states[window.startIndex].navState, params);
    propagateNavStateEKF(ekf, window, states[window.startIndex].bias, dt);
    
    const NavState predicted = ekf.state();
    const NavState& groundTruth = states[window.endIndex - 1].navState;
    const Vector9 error = computeNavStateError(predicted, groundTruth);
    const Matrix9 navigationCovariance = ekf.covariance();
    
    // Store trajectory data
    groundTruthTrajectory.push_back(createGroundTruthPoint(groundTruth, states[window.endIndex - 1].timestamp));
    predictedTrajectory.push_back(createPredictedPointFromNavState(predicted, navigationCovariance, 
                                                                   states[window.endIndex - 1].timestamp));
    errorTrajectory.push_back(error);
    
    return computeNEES(error, navigationCovariance);
}

NEESEvaluator::NEESResults EKFNEESEvaluator::processTimeWindowWithNavStateEKF(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    double preintegrationTime, double dt,
    const std::string& datasetName) const {
    
    const double totalTime = dataset_.getStates().back().timestamp - 
                            dataset_.getStates().front().timestamp;
    const int windowCount = static_cast<int>(totalTime / preintegrationTime);
    const int windowSize = static_cast<int>(dataset_.getStates().size() / windowCount);
    
    vector<double> neesValues;
    vector<TrajectoryValidator::TrajectoryPoint> groundTruthTrajectory;
    vector<TrajectoryValidator::TrajectoryPoint> predictedTrajectory;
    vector<Vector9> errorTrajectory;
    
    for (int m = 0; m < windowCount; m++) {
        WindowIndices window = computeWindowIndices(m, windowSize);
        auto nees = processNavStateWindow(window, params, dt, groundTruthTrajectory,
                                         predictedTrajectory, errorTrajectory);
        if (nees) neesValues.push_back(*nees);
    }
    
    exportTrajectoryResults(groundTruthTrajectory, predictedTrajectory, errorTrajectory,
                          "navstate", datasetName);
    exportNEESValues("navstate", datasetName, predictedTrajectory, neesValues);
    
    return NEESEvaluator::computeStatistics(neesValues, preintegrationTime);
}

} // namespace gtsam