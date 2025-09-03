#pragma once

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <optional>
#include <vector>

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NEESEvaluator.h
 * @brief   Normalized Estimation Error Squared (NEES) Metrics Evaluator
 * @author  Alec Kain
 */

namespace gtsam {

/**
 * @brief Evaluator for Normalized Estimation Error Squared (NEES) metrics
 * 
 * This class evaluates the NEES metric for IMU preintegration over different time windows.
 * It uses the EuRoC MAV dataset for ground truth and IMU measurements.
 * 
 * The NEES metric helps assess the consistency of the estimator by comparing
 * the predicted state with ground truth, normalized by the estimator's covariance.
 */

class NEESEvaluator {
public:
    struct NoiseParams {
        double sigma_gyro;
        double sigma_acc;
        double sigma_gyro_bias;
        double sigma_acc_bias;
    };

    struct GTData {
        std::vector<NavState> nav_states;
        std::vector<imuBias::ConstantBias> biases;
        std::vector<Vector3> measured_omegas;
        std::vector<Vector3> measured_accs;
        std::vector<double> timestamps;
    };

    void evaluateNEES(const std::string& filename, 
                     const std::vector<double>& preint_times,
                     double alpha = 3.0);

private:
    const double kGravity = 9.81;

    NoiseParams computeNoiseParams(double alpha);
    void setImuCovariances(const std::shared_ptr<PreintegrationCombinedParams>& params, 
                          const NoiseParams& noise);
    std::shared_ptr<PreintegrationCombinedParams> configureImuParams(double alpha);
    void integrateWindow(PreintegratedCombinedMeasurements& pim,
                        const GTData& data,
                        int start_idx, int end_idx, double dt);
    Vector computeError(const NavState& predicted, 
                       const NavState& actual,
                       const imuBias::ConstantBias& bias_pred,
                       const imuBias::ConstantBias& bias_actual);
    std::optional<double> computeNEES(const Vector& error, const Matrix& P);
    std::optional<double> calculateWindowNEES(const GTData& data, 
                                            const std::shared_ptr<PreintegrationCombinedParams>& params,
                                            int start_idx, int end_idx, double dt);
    bool isValidWindow(const GTData& data, int start_idx, int end_idx);
    std::optional<double> processWindow(PreintegratedCombinedMeasurements& pim,
                                      const GTData& data,
                                      int start_idx, int end_idx, double dt);
    std::vector<double> processTimeWindow(const GTData& data,
                                        const std::shared_ptr<PreintegrationCombinedParams>& params,
                                        int M, int N, double dt);
    void printNeesStatistics(const std::vector<double>& nees_results, double preint_time);
    GTData loadEurocData(const std::string& filename);
    void validateData(const GTData& data);
    void processAllTimeWindows(const GTData& data, 
                             const std::vector<double>& preint_times,
                             double alpha);
    void processTimeWindow(const GTData& data,
                          const std::shared_ptr<PreintegrationCombinedParams>& params,
                          double preint_time, double dt);
};

} // namespace gtsam