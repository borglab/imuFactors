#include "NEESEvaluator.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

namespace gtsam {

NEESEvaluator::NoiseParams NEESEvaluator::computeNoiseParams(double alpha) {
    return {
        alpha * 1.6968e-4,  // sigma_gyro
        alpha * 2.0000e-3,  // sigma_acc
        alpha * 1.9393e-5,  // sigma_gyro_bias
        alpha * 3.0000e-3   // sigma_acc_bias
    };
}

void NEESEvaluator::setImuCovariances(const std::shared_ptr<PreintegrationCombinedParams>& params, 
                                     const NoiseParams& noise) {
    params->setGyroscopeCovariance(Matrix3::Identity() * pow(noise.sigma_gyro, 2));
    params->setAccelerometerCovariance(Matrix3::Identity() * pow(noise.sigma_acc, 2));
    params->setIntegrationCovariance(Matrix3::Zero());
    params->setBiasAccCovariance(Matrix3::Identity() * pow(noise.sigma_acc_bias, 2));
    params->setBiasOmegaCovariance(Matrix3::Identity() * pow(noise.sigma_gyro_bias, 2));
}

std::shared_ptr<PreintegrationCombinedParams> NEESEvaluator::configureImuParams(double alpha) {
    auto params = PreintegrationCombinedParams::MakeSharedD(kGravity);
    params->n_gravity = Vector3(0, 0, -kGravity);
    setImuCovariances(params, computeNoiseParams(alpha));
    return params;
}

void NEESEvaluator::integrateWindow(PreintegratedCombinedMeasurements& pim,
                                   const GTData& data,
                                   int start_idx, int end_idx, double dt) {
    for (int k = start_idx; k < end_idx - 1; k++) {
        pim.integrateMeasurement(data.measured_accs[k], data.measured_omegas[k], dt);
    }
}

Vector NEESEvaluator::computeError(const NavState& predicted, 
                                  const NavState& actual,
                                  const imuBias::ConstantBias& bias_pred,
                                  const imuBias::ConstantBias& bias_actual) {
    Vector15 error;
    error << predicted.localCoordinates(actual),
             bias_actual.vector() - bias_pred.vector();
    return error;
}

std::optional<double> NEESEvaluator::computeNEES(const Vector& error, const Matrix& P) {
    try {
        return (error.transpose() * P.inverse() * error)(0,0) / 15.0;
    } catch (...) {
        return std::nullopt;
    }
}

bool NEESEvaluator::isValidWindow(const GTData& data, int start_idx, int end_idx) {
    return end_idx <= data.nav_states.size() &&
           end_idx <= data.measured_accs.size() &&
           end_idx <= data.measured_omegas.size();
}

std::optional<double> NEESEvaluator::processWindow(PreintegratedCombinedMeasurements& pim,
                                                 const GTData& data,
                                                 int start_idx, int end_idx, double dt) {
    integrateWindow(pim, data, start_idx, end_idx, dt);
    auto predicted = pim.predict(data.nav_states[start_idx], data.biases[start_idx]);
    auto error = computeError(predicted, data.nav_states[end_idx - 1],
                            data.biases[start_idx], data.biases[end_idx - 1]);
    return computeNEES(error, pim.preintMeasCov());
}

std::vector<double> NEESEvaluator::processTimeWindow(const GTData& data,
                                                   const std::shared_ptr<PreintegrationCombinedParams>& params,
                                                   int M, int N, double dt) {
    std::vector<double> nees_results;
    for (int m = 0; m < M; m++) {
        auto nees = calculateWindowNEES(data, params, m*N, 
                                      std::min((m+1)*N, (int)data.nav_states.size()), dt);
        if (nees) nees_results.push_back(*nees);
    }
    return nees_results;
}

std::optional<double> NEESEvaluator::calculateWindowNEES(const GTData& data, 
                                                       const std::shared_ptr<PreintegrationCombinedParams>& params,
                                                       int start_idx, int end_idx, double dt) {
    if (!isValidWindow(data, start_idx, end_idx)) return std::nullopt;
    PreintegratedCombinedMeasurements pim(params, data.biases[start_idx]);
    return processWindow(pim, data, start_idx, end_idx, dt);
}

void NEESEvaluator::printNeesStatistics(const std::vector<double>& nees_results, 
                                       double preint_time) {
    if (nees_results.empty()) {
        std::cerr << "No valid NEES results for preintegration time " << preint_time << std::endl;
        return;
    }

    Eigen::Map<const Eigen::VectorXd> nees_vector(nees_results.data(), nees_results.size());
    double mean = nees_vector.mean();
    auto sorted_results = nees_results;
    std::sort(sorted_results.begin(), sorted_results.end());
    double median = sorted_results[sorted_results.size()/2];
    double variance = (nees_vector.array() - mean).square().mean();

    std::cout << "ANEES (15-DOF):    mean | median | variance\n"
              << "--------------------------------------------\n"
              << "Results:   " << mean << " | " << median << " | " << variance << "\n";
}

NEESEvaluator::GTData NEESEvaluator::loadEurocData(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    GTData data;
    std::string line;
    if (!std::getline(file, line)) {
        throw std::runtime_error("Empty file: " + filename);
    }

    double t0 = 0;
    bool first = true;

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }

        if (first) {
            t0 = row[0];
            first = false;
        }

        data.timestamps.push_back(row[0] - t0);
        
        // Parse quaternion (w,x,y,z)
        Rot3 rot = Rot3::Quaternion(row[1], row[2], row[3], row[4]);
        Point3 vel(row[5], row[6], row[7]);
        Point3 pos(row[8], row[9], row[10]);
        data.nav_states.push_back(NavState(rot, pos, vel));

        Vector3 gyro_bias(row[11], row[12], row[13]);
        Vector3 acc_bias(row[14], row[15], row[16]);
        data.biases.push_back(imuBias::ConstantBias(acc_bias, gyro_bias));

        data.measured_omegas.push_back(Vector3(row[17], row[18], row[19]));
        data.measured_accs.push_back(Vector3(row[20], row[21], row[22]));
    }
    return data;
}

void NEESEvaluator::validateData(const GTData& data) {
    if (data.timestamps.size() < 2) {
        throw std::runtime_error("Insufficient data points");
    }
}

void NEESEvaluator::processAllTimeWindows(const GTData& data, 
                                        const std::vector<double>& preint_times,
                                        double alpha) {
    auto params = configureImuParams(alpha);
    double dt = data.timestamps[1] - data.timestamps[0];
    
    for (double preint_time : preint_times) {
        processTimeWindow(data, params, preint_time, dt);
    }
}

void NEESEvaluator::processTimeWindow(const GTData& data,
                                    const std::shared_ptr<PreintegrationCombinedParams>& params,
                                    double preint_time, double dt) {
    std::cout << "\nAnalyzing with preintegration time: " << preint_time << " s\n";
    int M = static_cast<int>(data.timestamps.back() / preint_time);
    int N = static_cast<int>(data.nav_states.size() / M);
    auto results = processTimeWindow(data, params, M, N, dt);
    printNeesStatistics(results, preint_time);
}

void NEESEvaluator::evaluateNEES(const std::string& filename, 
                                const std::vector<double>& preint_times,
                                double alpha) {
    GTData data = loadEurocData(filename);
    validateData(data);
    processAllTimeWindows(data, preint_times, alpha);
}

} // namespace gtsam