/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   NoiseCalibration.h
 * @brief  Common utilities for noise calibration across dataset subsets
 * @author Alec Kain
 */

#pragma once

#include "EKFNEESEvaluator.h"
#include "Dataset.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <map>
#include <filesystem>
#include <functional>

namespace gtsam {

/// Results for a single (alphaGyro, alphaAcc) combination
struct NoiseTrialResult {
    double alphaGyro;
    double alphaAcc;
    std::map<std::string, double> datasetNees;  ///< NEES per dataset
    double meanNees;           ///< Mean NEES across all datasets
    double sumDeviations;      ///< Sum of |NEES - 1| across datasets
};

/// Results for entire calibration study
struct NoiseCalibrationStudy {
    std::vector<NoiseTrialResult> trials;
    NoiseTrialResult bestResult;
    NoiseTrialResult worstResult;
    std::vector<std::string> datasetNames;
};

/// Compute deviation metrics for a trial
inline void computeMetrics(NoiseTrialResult& result) {
    double sum = 0.0;
    double sumDev = 0.0;
    for (const auto& [name, nees] : result.datasetNees) {
        sum += nees;
        sumDev += std::abs(nees - 1.0);
    }
    result.meanNees = sum / result.datasetNees.size();
    result.sumDeviations = sumDev;
}

/// Dataset filter predicate type
using DatasetFilter = std::function<bool(const std::string&)>;

/// Predefined dataset filters
namespace DatasetFilters {
    /// Filter for Machine Hall datasets (MH-series)
    inline bool machineHall(const std::string& name) {
        return name.find("MH") == 0;
    }
    
    /// Filter for Vicon Room datasets (V-series)
    inline bool viconRoom(const std::string& name) {
        return name.find("V") == 0;
    }
    
    /// Filter that accepts all datasets
    inline bool all(const std::string&) {
        return true;
    }
}

/// Discover datasets matching a filter predicate
inline std::vector<std::pair<std::string, std::string>> discoverFilteredDatasets(
    const std::string& dataDirectory, 
    DatasetFilter filter) {
    
    namespace fs = std::filesystem;
    std::vector<std::pair<std::string, std::string>> datasets;
    
    if (!fs::exists(dataDirectory)) {
        std::cerr << "❌ Data directory not found: " << dataDirectory << std::endl;
        return datasets;
    }
    
    for (const auto& entry : fs::directory_iterator(dataDirectory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            std::string filename = entry.path().filename().string();
            if (filename.find("euroc_") == 0) {
                std::string name = filename.substr(6);
                name = name.substr(0, name.find(".csv"));
                
                // Apply filter
                if (filter(name)) {
                    datasets.push_back({name, entry.path().string()});
                }
            }
        }
    }
    
    std::sort(datasets.begin(), datasets.end());
    return datasets;
}

/// Run calibration with custom dataset filter and study name
inline NoiseCalibrationStudy runFilteredCalibration(
    const std::vector<std::pair<std::string, std::string>>& datasets,
    const std::string& studyName) {
    
    using namespace std;
    
    NoiseCalibrationStudy study;
    for (const auto& [name, path] : datasets) {
        study.datasetNames.push_back(name);
    }
    
    cout << "\n" << string(100, '=') << "\n";
    cout << studyName << " NOISE CALIBRATION (0.2s NEES)\n";
    cout << "Datasets: ";
    for (const auto& name : study.datasetNames) cout << name << " ";
    cout << "\n" << string(100, '=') << "\n";
    
    std::vector<double> alphaGyroRange = {0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 13.0};
    std::vector<double> alphaAccRange  = {0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 13.0};
    
    cout << "\nGyro alphas: ";
    for (double a : alphaGyroRange) cout << a << " ";
    cout << "\nAccel alphas: ";
    for (double a : alphaAccRange) cout << a << " ";
    cout << "\n\n";
    
    double bestSumDev = 1e9;
    double worstSumDev = 0.0;
    
    cout << "αGyro\tαAcc";
    for (const auto& name : study.datasetNames) {
        cout << "\t" << name;
    }
    cout << "\tMean\tSumDev\n";
    cout << string(100, '-') << "\n";
    
    for (double alphaGyro : alphaGyroRange) {
        for (double alphaAcc : alphaAccRange) {
            NoiseTrialResult trial;
            trial.alphaGyro = alphaGyro;
            trial.alphaAcc = alphaAcc;
            
            for (const auto& [name, path] : datasets) {
                Dataset dataset(path);
                EKFNEESEvaluator evaluator(dataset);
                
                auto params = dataset.configureImuParams(alphaGyro, alphaAcc);
                auto result = evaluator.runGal3ImuEKF(0.2, params, name);
                
                trial.datasetNees[name] = result.median;
            }
            
            computeMetrics(trial);
            study.trials.push_back(trial);
            
            cout << fixed << setprecision(2)
                 << alphaGyro << "\t" << alphaAcc;
            for (const auto& name : study.datasetNames) {
                cout << "\t" << trial.datasetNees[name];
            }
            cout << "\t" << trial.meanNees << "\t" << trial.sumDeviations << "\n";
            
            if (trial.sumDeviations < bestSumDev) {
                bestSumDev = trial.sumDeviations;
                study.bestResult = trial;
            }
            
            if (trial.sumDeviations > worstSumDev) {
                worstSumDev = trial.sumDeviations;
                study.worstResult = trial;
            }
        }
    }
    
    return study;
}

/// Export results to CSV
inline void exportCalibrationResults(const NoiseCalibrationStudy& study, const std::string& filename) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }
    
    file << "alpha_gyro,alpha_acc";
    for (const auto& name : study.datasetNames) {
        file << ",nees_" << name;
    }
    file << ",mean_nees,sum_deviations\n";
    
    for (const auto& trial : study.trials) {
        file << trial.alphaGyro << "," << trial.alphaAcc;
        for (const auto& name : study.datasetNames) {
            file << "," << trial.datasetNees.at(name);
        }
        file << "," << trial.meanNees << "," << trial.sumDeviations << "\n";
    }
    
    file.close();
    std::cout << "✓ Exported results to " << filename << "\n";
}

/// Print summary
inline void printCalibrationSummary(const NoiseCalibrationStudy& study, const std::string& datasetType) {
    using namespace std;
    
    cout << "\n" << string(100, '=') << "\n";
    cout << datasetType << " DATASETS: OPTIMAL NOISE PARAMETERS\n";
    cout << string(100, '=') << "\n\n";
    
    cout << "Result\t\tαGyro\tαAcc";
    for (const auto& name : study.datasetNames) {
        cout << "\t" << name;
    }
    cout << "\tMean\tSumDev\n";
    cout << string(100, '-') << "\n";
    
    cout << fixed << setprecision(3);
    cout << "BEST\t\t" 
         << study.bestResult.alphaGyro << "\t"
         << study.bestResult.alphaAcc;
    for (const auto& name : study.datasetNames) {
        cout << "\t" << study.bestResult.datasetNees.at(name);
    }
    cout << "\t" << study.bestResult.meanNees << "\t"
         << study.bestResult.sumDeviations << "\n";
    
    cout << "WORST\t\t"
         << study.worstResult.alphaGyro << "\t"
         << study.worstResult.alphaAcc;
    for (const auto& name : study.datasetNames) {
        cout << "\t" << study.worstResult.datasetNees.at(name);
    }
    cout << "\t" << study.worstResult.meanNees << "\t"
         << study.worstResult.sumDeviations << "\n";
    
    cout << string(100, '=') << "\n";
    
    cout << "\n📊 RECOMMENDED PARAMETERS (" << datasetType << "):\n";
    cout << "   αGyro = " << study.bestResult.alphaGyro << "\n";
    cout << "   αAcc  = " << study.bestResult.alphaAcc << "\n";
    cout << "   Mean NEES across " << study.datasetNames.size() << " datasets: " 
         << study.bestResult.meanNees << "\n";
    
    cout << "\n⚠️  WORST PARAMETERS:\n";
    cout << "   αGyro = " << study.worstResult.alphaGyro << "\n";
    cout << "   αAcc  = " << study.worstResult.alphaAcc << "\n";
    cout << "   Mean NEES: " << study.worstResult.meanNees << "\n";
}

}  // namespace gtsam