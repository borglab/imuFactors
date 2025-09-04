/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    Dataset.cpp
 * @brief   EuRoC dataset loader and container implementation
 * @author  Alec Kain
 */

#include "Dataset.h"
#include <fstream>
#include <sstream>

namespace gtsam {

Dataset::Dataset(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::string line;
    if (!std::getline(file, line)) {
        throw std::runtime_error("Empty file: " + filename);
    }

    double timeStart = 0;
    bool isFirst = true;

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }

        if (isFirst) {
            timeStart = row[0];
            isFirst = false;
        }

        double timestamp = row[0] - timeStart;
        
        // Parse state measurement
        Rot3 rotation = Rot3::Quaternion(row[1], row[2], row[3], row[4]);
        Point3 velocity(row[5], row[6], row[7]);
        Point3 position(row[8], row[9], row[10]);
        NavState navState(rotation, position, velocity);

        Vector3 gyroBias(row[11], row[12], row[13]);
        Vector3 accBias(row[14], row[15], row[16]);
        imuBias::ConstantBias bias(accBias, gyroBias);

        states_.push_back({timestamp, navState, bias});

        // Parse IMU measurement
        Vector3 omega(row[17], row[18], row[19]);
        Vector3 acc(row[20], row[21], row[22]);
        imuData_.push_back({timestamp, omega, acc});
    }

    if (states_.empty() || imuData_.empty()) {
        throw std::runtime_error("No valid data found in file: " + filename);
    }
}

} // namespace gtsam