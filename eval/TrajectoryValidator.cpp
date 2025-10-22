/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   TrajectoryValidator.cpp
 * @brief  Implementation of trajectory validation utilities
 * @author Alec Kain
 */

#include "TrajectoryValidator.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace gtsam {

Vector9 TrajectoryValidator::extractStdDev(const Matrix9& covariance) {
    Vector9 stdDev;
    for (int i = 0; i < 9; ++i) {
        stdDev(i) = std::sqrt(std::abs(covariance(i, i)));
    }
    return stdDev;
}

void TrajectoryValidator::exportToCSV(
    const std::string& filename,
    const std::vector<TrajectoryPoint>& groundTruth,
    const std::vector<TrajectoryPoint>& predicted,
    const std::vector<Vector9>& errors) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    // Write header
    file << "timestamp,"
         << "gt_x,gt_y,gt_z,gt_vx,gt_vy,gt_vz,gt_roll,gt_pitch,gt_yaw,"
         << "pred_x,pred_y,pred_z,pred_vx,pred_vy,pred_vz,pred_roll,pred_pitch,pred_yaw,"
         << "err_rot_x,err_rot_y,err_rot_z,err_vel_x,err_vel_y,err_vel_z,err_pos_x,err_pos_y,err_pos_z,"
         << "rot_std_x,rot_std_y,rot_std_z,vel_std_x,vel_std_y,vel_std_z,pos_std_x,pos_std_y,pos_std_z";
    
    // Add all 81 covariance elements
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            file << ",cov_" << i << "_" << j;
        }
    }
    file << "\n";
    
    // Write data with high precision
    file << std::scientific << std::setprecision(10);
    
    for (size_t i = 0; i < groundTruth.size(); ++i) {
        const auto& gt = groundTruth[i];
        const auto& pred = predicted[i];
        const auto& error = errors[i];
        
        Vector9 stdDev = extractStdDev(pred.covariance);
        
        // Basic data
        file << gt.timestamp << ",";
        
        // Ground truth
        file << gt.position.x() << "," << gt.position.y() << "," << gt.position.z() << ","
             << gt.velocity.x() << "," << gt.velocity.y() << "," << gt.velocity.z() << ","
             << gt.rpy.x() << "," << gt.rpy.y() << "," << gt.rpy.z() << ",";
        
        // Predicted
        file << pred.position.x() << "," << pred.position.y() << "," << pred.position.z() << ","
             << pred.velocity.x() << "," << pred.velocity.y() << "," << pred.velocity.z() << ","
             << pred.rpy.x() << "," << pred.rpy.y() << "," << pred.rpy.z() << ",";
        
        // Errors
        file << error(0) << "," << error(1) << "," << error(2) << ","
             << error(3) << "," << error(4) << "," << error(5) << ","
             << error(6) << "," << error(7) << "," << error(8) << ",";
        
        // Standard deviations
        file << stdDev(0) << "," << stdDev(1) << "," << stdDev(2) << ","
             << stdDev(3) << "," << stdDev(4) << "," << stdDev(5) << ","
             << stdDev(6) << "," << stdDev(7) << "," << stdDev(8);
        
        // Full 9x9 covariance matrix
        for (int row = 0; row < 9; ++row) {
            for (int col = 0; col < 9; ++col) {
                file << "," << pred.covariance(row, col);
            }
        }
        
        file << "\n";
    }
    
    file.close();
    std::cout << "✓ Exported trajectory to " << filename << std::endl;
}

void TrajectoryValidator::printErrorStatistics(const std::vector<Vector9>& errors) {
    if (errors.empty()) {
        std::cout << "No errors to analyze.\n";
        return;
    }
    
    Vector3 rmsRotation = Vector3::Zero();
    Vector3 rmsVelocity = Vector3::Zero();
    Vector3 rmsPosition = Vector3::Zero();
    
    for (const auto& error : errors) {
        rmsRotation += error.head<3>().array().square().matrix();
        rmsVelocity += error.segment<3>(3).array().square().matrix();
        rmsPosition += error.tail<3>().array().square().matrix();
    }
    
    const size_t n = errors.size();
    rmsRotation = (rmsRotation / n).array().sqrt().matrix();
    rmsVelocity = (rmsVelocity / n).array().sqrt().matrix();
    rmsPosition = (rmsPosition / n).array().sqrt().matrix();
    
    std::cout << "\n=== Error Statistics ===" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "RMS Rotation (rad):  [" << rmsRotation.transpose() << "]" << std::endl;
    std::cout << "RMS Velocity (m/s):  [" << rmsVelocity.transpose() << "]" << std::endl;
    std::cout << "RMS Position (m):    [" << rmsPosition.transpose() << "]" << std::endl;
}

} // namespace gtsam