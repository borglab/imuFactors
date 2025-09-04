/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NEESEvaluator.h
 * @brief   Normalized Estimation Error Squared (NEES) Metrics Evaluator
 * @author  Alec Kain
 */

#pragma once

#include "Dataset.h"
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <optional>
#include <vector>

namespace gtsam {

class NEESEvaluator {
public:
    /**
     * @brief Parameters for IMU noise characteristics
     */
    struct NoiseParams {
        double sigmaGyro;      ///< Gyroscope noise standard deviation
        double sigmaAcc;       ///< Accelerometer noise standard deviation
        double sigmaGyroBias;  ///< Gyroscope bias random walk standard deviation
        double sigmaAccBias;   ///< Accelerometer bias random walk standard deviation
    };

    explicit NEESEvaluator(const Dataset& dataset) : dataset_(dataset) {}
	
	/**
	 * Calculate NEES by chopping up dataset in small intervals.
	 * @param intervalLength length of interval (seconds)
	 * @param alpha "fudge" factor for noise parameters
	 */
    void run(double interval, double alpha = 3.0);

private:
    const Dataset& dataset_;

    NoiseParams computeNoiseParams(double alpha);
    
    void setImuCovariances(const std::shared_ptr<PreintegrationCombinedParams>& params, 
                          const NoiseParams& noise);
                          
    std::shared_ptr<PreintegrationCombinedParams> configureImuParams(double alpha);
    
    Vector computeError(const NavState& predicted, 
                       const NavState& actual,
                       const imuBias::ConstantBias& biasPred,
                       const imuBias::ConstantBias& biasActual);
                       
    std::optional<double> computeNEES(const Vector& error, const Matrix& covMatrix);
    
    std::optional<double> calculateWindowNEES(const Dataset& data, 
                                            const std::shared_ptr<PreintegrationCombinedParams>& params,
                                            int startIdx, int endIdx, double dt);
                                            
    bool isValidWindow(const Dataset& data, int startIdx, int endIdx);
    
    std::vector<double> processTimeWindow(const Dataset& data,
                                        const std::shared_ptr<PreintegrationCombinedParams>& params,
                                        int windowCount, int windowSize, double dt);
                                        
    void processTimeWindow(const Dataset& data,
                          const std::shared_ptr<PreintegrationCombinedParams>& params,
                          double preintTime, double dt);
                          
    void printNeesStatistics(const std::vector<double>& neesResults, double preintTime);
};

} // namespace gtsam