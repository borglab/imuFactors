/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalNoiseCalibration_MachineHall.cpp
 * @brief  Coarse Grid Search for Machine Hall Datasets Only
 * @author Alec Kain
 */

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

using namespace gtsam;
using namespace std;
namespace fs = std::filesystem;

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
void computeMetrics(NoiseTrialResult& result) {
    double sum = 0.0;
    double sumDev = 0.0;
    for (const auto& [name, nees] : result.datasetNees) {
        sum += nees;
        sumDev += std::abs(nees - 1.0);
    }
    result.meanNees = sum / result.datasetNees.size();
    result.sumDeviations = sumDev;
}

/// Discover Machine Hall datasets only
std::vector<std::pair<std::string, std::string>> discoverMachineHallDatasets(const std::string& dataDir) {
    std::vector<std::pair<std::string, std::string>> datasets;
    
    if (!fs::exists(dataDir)) {
        std::cerr << "❌ Data directory not found: " << dataDir << std::endl;
        return datasets;
    }
    
    for (const auto& entry : fs::directory_iterator(dataDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            std::string filename = entry.path().filename().string();
            // Only include MH-series datasets
            if (filename.find("euroc_MH") == 0) {
                std::string name = filename.substr(6);
                name = name.substr(0, name.find(".csv"));
                datasets.push_back({name, entry.path().string()});
            }
        }
    }
    
    std::sort(datasets.begin(), datasets.end());
    return datasets;
}

/// Run coarse 2D grid search
NoiseCalibrationStudy runMachineHallCalibration(
    const std::vector<std::pair<std::string, std::string>>& datasets) {
    
    NoiseCalibrationStudy study;
    for (const auto& [name, path] : datasets) {
        study.datasetNames.push_back(name);
    }
    
    cout << "\n" << string(100, '=') << "\n";
    cout << "MACHINE HALL DATASET NOISE CALIBRATION (0.2s NEES)\n";
    cout << "Datasets: ";
    for (const auto& name : study.datasetNames) cout << name << " ";
    cout << "\n" << string(100, '=') << "\n";
    
    std::vector<double> alphaGyroRange = {0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 13.0, 15.0, 20.0};
    std::vector<double> alphaAccRange  = {0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 13.0, 15.0, 20.0};
    
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
void exportResults(const NoiseCalibrationStudy& study, const std::string& filename) {
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
    cout << "✓ Exported results to " << filename << "\n";
}

/// Print summary
void printSummary(const NoiseCalibrationStudy& study) {
    cout << "\n" << string(100, '=') << "\n";
    cout << "MACHINE HALL DATASETS: OPTIMAL NOISE PARAMETERS\n";
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
    
    cout << "\n📊 RECOMMENDED PARAMETERS (MACHINE HALL):\n";
    cout << "   αGyro = " << study.bestResult.alphaGyro << "\n";
    cout << "   αAcc  = " << study.bestResult.alphaAcc << "\n";
    cout << "   Mean NEES across " << study.datasetNames.size() << " datasets: " 
         << study.bestResult.meanNees << "\n";
    
    cout << "\n⚠️  WORST PARAMETERS:\n";
    cout << "   αGyro = " << study.worstResult.alphaGyro << "\n";
    cout << "   αAcc  = " << study.worstResult.alphaAcc << "\n";
    cout << "   Mean NEES: " << study.worstResult.meanNees << "\n";
}

/// Main program
int main(int argc, char* argv[]) {
    try {
        cout << "\n";
        cout << "╔════════════════════════════════════════════════════════════════╗\n";
        cout << "║  MACHINE HALL DATASET NOISE CALIBRATION                        ║\n";
        cout << "║  Goal: Minimize |NEES_0.2s - 1| across MH-series datasets    ║\n";
        cout << "╚════════════════════════════════════════════════════════════════╝\n";
        
        const std::string dataDir = "../eval/data/euroc/";
        
        auto datasets = discoverMachineHallDatasets(dataDir);
        
        if (datasets.empty()) {
            std::cerr << "❌ No Machine Hall datasets found in " << dataDir << std::endl;
            return 1;
        }
        
        cout << "\n✓ Found " << datasets.size() << " Machine Hall datasets:\n";
        for (const auto& [name, path] : datasets) {
            cout << "  • " << name << " (" << path << ")\n";
        }
        
        // Run calibration
        NoiseCalibrationStudy study = runMachineHallCalibration(datasets);
        exportResults(study, "noise_calibration_machine_hall.csv");
        
        // Print summary
        printSummary(study);
        
        cout << "\n✅ Machine Hall calibration complete!\n";
        cout << "📊 Check noise_calibration_machine_hall.csv for detailed results\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
}