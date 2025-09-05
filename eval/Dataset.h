/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <string>
#include <vector>
#include <memory>

namespace gtsam {

/**
 * @brief Container and loader for EuRoC MAV dataset
 * 
 * Handles loading and storing synchronized IMU and ground truth state measurements
 * from the EuRoC MAV dataset format.
 */
class Dataset {
public:
    /**
     * @brief Single IMU measurement with timestamp
     */
    struct ImuMeasurement {
        double timestamp;  ///< Time of measurement in seconds
        Vector3 omega;    ///< Angular velocity measurement in rad/s
        Vector3 acc;      ///< Linear acceleration measurement in m/s^2
    };

    /**
     * @brief Ground truth state measurement with timestamp
     */
    struct StateMeasurement {
        double timestamp;              ///< Time of measurement in seconds
        NavState navState;            ///< Navigation state (position, velocity, rotation)
        imuBias::ConstantBias bias;   ///< IMU bias states
    };

    /**
     * @brief Parameters for IMU noise characteristics
     */
    struct NoiseParams {
        double sigmaGyro;      ///< Gyroscope noise standard deviation
        double sigmaAcc;       ///< Accelerometer noise standard deviation
        double sigmaGyroBias;  ///< Gyroscope bias random walk standard deviation
        double sigmaAccBias;   ///< Accelerometer bias random walk standard deviation
    };

    /**
     * @brief Loads and validates EuRoC dataset from file
     * 
     * @param filename Path to the dataset CSV file
     * @throw std::runtime_error if file cannot be opened or data is invalid
     */
    explicit Dataset(const std::string& filename);

    /**
     * @brief Get vector of ground truth state measurements
     * @return Const reference to state measurements
     */
    const std::vector<StateMeasurement>& getStates() const { return states_; }

    /**
     * @brief Get vector of IMU measurements
     * @return Const reference to IMU measurements
     */
    const std::vector<ImuMeasurement>& getImuData() const { return imuData_; }

    /**
     * @brief Get gravity constant used in the dataset
     * @return Gravity value in m/s^2
     */
    double getGravity() const { return gravity_; }

    /**
     * @brief Configure IMU preintegration parameters for this dataset
     * @param alpha Scaling factor for noise parameters (default: 3.0)
     * @return Configured preintegration parameters
     */
    std::shared_ptr<PreintegrationCombinedParams> configureImuParams(double alpha = 3.0) const;

private:
    std::vector<StateMeasurement> states_;
    std::vector<ImuMeasurement> imuData_;
    const double gravity_ = 9.81;
    
    /**
     * @brief Compute noise parameters based on dataset characteristics
     * @param alpha Scaling factor for noise parameters
     * @return Noise parameters structure
     */
    NoiseParams computeNoiseParams(double alpha) const;

    /**
     * @brief Set IMU covariances in preintegration parameters
     * @param params Preintegration parameters to modify
     * @param noise Noise parameters to apply
     */
    void setImuCovariances(const std::shared_ptr<PreintegrationCombinedParams>& params, 
                          const NoiseParams& noise) const;
};
} // namespace gtsam