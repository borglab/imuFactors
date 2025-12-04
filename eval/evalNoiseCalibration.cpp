/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalNoiseCalibration.cpp
 * @brief  2D Grid Search for Optimal Gyroscope and Accelerometer Noise Scaling
 * @author Alec Kain
 */

#include "EKFNEESEvaluator.h"
#include "Dataset.h"
#include "TrajectoryValidator.h"
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

/// Discover all EuRoC datasets in the data directory
std::vector<std::pair<std::string, std::string>> discoverDatasets(const std::string& dataDir) {
    std::vector<std::pair<std::string, std::string>> datasets;
    
    if (!fs::exists(dataDir)) {
        std::cerr << "❌ Data directory not found: " << dataDir << std::endl;
        return datasets;
    }
    
    for (const auto& entry : fs::directory_iterator(dataDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            std::string filename = entry.path().filename().string();
            if (filename.find("euroc_") == 0) {
                std::string name = filename.substr(6);
                name = name.substr(0, name.find(".csv"));
                datasets.push_back({name, entry.path().string()});
            }
        }
    }
    
    std::sort(datasets.begin(), datasets.end());
    return datasets;
}

/// Export ground truth trajectory to CSV (one file per dataset)
void exportGroundTruthTrajectory(const Dataset& dataset, const std::string& datasetName) {
    std::string filename = "trajectory_" + datasetName + "_groundtruth.csv";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }
    
    file << "timestamp,px,py,pz,vx,vy,vz,roll,pitch,yaw\n";
    file << std::fixed << std::setprecision(6);
    
    const auto& states = dataset.getStates();
    for (const auto& state : states) {
        const NavState& navState = state.navState;
        Vector3 rpy = navState.attitude().rpy() * 180.0 / M_PI;
        
        file << state.timestamp << ","
             << navState.position().x() << "," << navState.position().y() << "," << navState.position().z() << ","
             << navState.velocity().x() << "," << navState.velocity().y() << "," << navState.velocity().z() << ","
             << rpy.x() << "," << rpy.y() << "," << rpy.z() << "\n";
    }
    
    file.close();
    std::cout << "  ✓ Exported ground truth: " << filename << " (" << states.size() << " points)\n";
}

/// Export predicted trajectory with piecewise constant predictions
void exportPredictedTrajectory(
    const Dataset& dataset,
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const std::string& datasetName,
    const std::string& suffix,
    double preintegrationTime) {
    
    std::string filename = "trajectory_" + datasetName + "_" + suffix + "_Gal3ImuEKF_0.2s.csv";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }
    
    file << "timestamp,px,py,pz,vx,vy,vz,roll,pitch,yaw\n";
    file << std::fixed << std::setprecision(6);
    
    const auto& states = dataset.getStates();
    const auto& imuData = dataset.getImuData();
    
    if (states.empty() || imuData.empty()) {
        std::cerr << "Error: Empty dataset for " << datasetName << std::endl;
        file.close();
        return;
    }
    
    /// Compute window parameters
    double dt = states[1].timestamp - states[0].timestamp;
    const int stepsPerWindow = static_cast<int>(std::round(preintegrationTime / dt));
    const int numCompleteWindows = (states.size() - 1) / stepsPerWindow;
    
    std::cout << "  Generating " << datasetName << " " << suffix << ": "
              << stepsPerWindow << " steps/window, "
              << numCompleteWindows << " windows, "
              << states.size() << " total points\n";
    
    /// Process each window
    for (int windowIdx = 0; windowIdx < numCompleteWindows; windowIdx++) {
        const size_t windowStart = windowIdx * stepsPerWindow;
        const size_t windowEnd = windowStart + stepsPerWindow;
        
        /// Initialize Gal3 at window start
        const NavState& initialNavState = states[windowStart].navState;
        const Gal3 initialGal3 = Gal3(
            initialNavState.attitude(),
            initialNavState.position(),
            initialNavState.velocity(),
            states[windowStart].timestamp
        );
        
        /// Initialize with 10x10 covariance
        Matrix initialCovariance = Matrix::Zero(10, 10);
        
        /// Create EKF
        Gal3ImuEKF ekf(initialGal3, initialCovariance, params);
        
        const imuBias::ConstantBias windowBias = states[windowStart].bias;
        
        /// Integrate through window
        for (size_t k = windowStart; k < windowEnd && k < imuData.size(); k++) {
            Vector3 omega = imuData[k].omega - windowBias.gyroscope();
            Vector3 acceleration = imuData[k].acc - windowBias.accelerometer();
            ekf.predict(omega, acceleration, dt);
        }
        
        /// Get prediction at end of window
        const Gal3 predicted = ekf.state();
        Vector3 rpy = predicted.attitude().rpy() * 180.0 / M_PI;
        
        /// Write prediction for ALL timestamps in this window (piecewise constant)
        for (size_t k = windowStart; k <= windowEnd && k < states.size(); k++) {
            file << states[k].timestamp << ","
                 << predicted.translation().x() << "," 
                 << predicted.translation().y() << "," 
                 << predicted.translation().z() << ","
                 << predicted.velocity().x() << "," 
                 << predicted.velocity().y() << "," 
                 << predicted.velocity().z() << ","
                 << rpy.x() << "," << rpy.y() << "," << rpy.z() << "\n";
        }
    }
    
    /// Handle any remaining points beyond the last complete window
    const size_t lastWindowEnd = numCompleteWindows * stepsPerWindow;
    if (lastWindowEnd < states.size()) {
        std::cout << "  Warning: " << (states.size() - lastWindowEnd) 
                  << " points at end of trajectory not covered by complete windows\n";
        
        /// For remaining points, just repeat the last prediction
        if (numCompleteWindows > 0) {
            /// Re-run last window to get final prediction
            const size_t windowStart = (numCompleteWindows - 1) * stepsPerWindow;
            const size_t windowEnd = windowStart + stepsPerWindow;
            
            const NavState& initialNavState = states[windowStart].navState;
            const Gal3 initialGal3 = Gal3(
                initialNavState.attitude(),
                initialNavState.position(),
                initialNavState.velocity(),
                states[windowStart].timestamp
            );
            
            Matrix initialCovariance = Matrix::Zero(10, 10);
            Gal3ImuEKF ekf(initialGal3, initialCovariance, params);
            const imuBias::ConstantBias windowBias = states[windowStart].bias;
            
            for (size_t k = windowStart; k < windowEnd && k < imuData.size(); k++) {
                Vector3 omega = imuData[k].omega - windowBias.gyroscope();
                Vector3 acceleration = imuData[k].acc - windowBias.accelerometer();
                ekf.predict(omega, acceleration, dt);
            }
            
            const Gal3 predicted = ekf.state();
            Vector3 rpy = predicted.attitude().rpy() * 180.0 / M_PI;
            
            /// Write for remaining points
            for (size_t k = lastWindowEnd; k < states.size(); k++) {
                file << states[k].timestamp << ","
                     << predicted.translation().x() << "," 
                     << predicted.translation().y() << "," 
                     << predicted.translation().z() << ","
                     << predicted.velocity().x() << "," 
                     << predicted.velocity().y() << "," 
                     << predicted.velocity().z() << ","
                     << rpy.x() << "," << rpy.y() << "," << rpy.z() << "\n";
            }
        }
    }
    
    file.close();
    std::cout << "  ✓ Exported prediction: " << filename << " (" << states.size() << " points)\n";
}

/// Run 2D grid search over alphaGyro and alphaAcc
NoiseCalibrationStudy runNoiseCalibration(
    const std::vector<std::pair<std::string, std::string>>& datasets) {
    
    NoiseCalibrationStudy study;
    for (const auto& [name, path] : datasets) {
        study.datasetNames.push_back(name);
    }
    
    cout << "\n" << string(100, '=') << "\n";
    cout << "2D NOISE CALIBRATION: OPTIMIZING GYRO & ACCEL SCALING (0.2s NEES)\n";
    cout << "Datasets: ";
    for (const auto& name : study.datasetNames) cout << name << " ";
    cout << "\n" << string(100, '=') << "\n";
    
    std::vector<double> alphaGyroRange = {0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0};
    std::vector<double> alphaAccRange  = {0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0};
    
    cout << "\n=== COARSE 2D GRID SEARCH ===\n";
    cout << "Gyro alphas: ";
    for (double a : alphaGyroRange) cout << a << " ";
    cout << "\nAccel alphas: ";
    for (double a : alphaAccRange) cout << a << " ";
    cout << "\n\n";
    
    double bestSumDev = 1000.0;
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
    
    cout << "\n✓ Best (αGyro, αAcc): (" 
         << study.bestResult.alphaGyro << ", " 
         << study.bestResult.alphaAcc << ")\n";
    cout << "  Mean NEES: " << study.bestResult.meanNees << "\n";
    cout << "  Sum Deviations: " << study.bestResult.sumDeviations << "\n";
    
    cout << "\n✗ Worst (αGyro, αAcc): (" 
         << study.worstResult.alphaGyro << ", " 
         << study.worstResult.alphaAcc << ")\n";
    cout << "  Mean NEES: " << study.worstResult.meanNees << "\n";
    cout << "  Sum Deviations: " << study.worstResult.sumDeviations << "\n";
    
    return study;
}

/// Fine search around best coarse result
NoiseCalibrationStudy refineNoiseCalibration(
    const std::vector<std::pair<std::string, std::string>>& datasets,
    const NoiseTrialResult& coarseBest) {
    
    NoiseCalibrationStudy study;
    for (const auto& [name, path] : datasets) {
        study.datasetNames.push_back(name);
    }
    
    cout << "\n=== FINE 2D GRID SEARCH ===\n";
    
    double gyroMin = std::max(0.1, coarseBest.alphaGyro * 0.7);
    double gyroMax = coarseBest.alphaGyro * 1.3;
    double accMin = std::max(0.1, coarseBest.alphaAcc * 0.7);
    double accMax = coarseBest.alphaAcc * 1.3;
    
    cout << "Refining gyro: [" << gyroMin << ", " << gyroMax << "]\n";
    cout << "Refining accel: [" << accMin << ", " << accMax << "]\n\n";
    
    double bestSumDev = 1000.0;
    
    cout << "αGyro\tαAcc";
    for (const auto& name : study.datasetNames) {
        cout << "\t" << name;
    }
    cout << "\tMean\tSumDev\n";
    cout << string(100, '-') << "\n";
    
    for (double alphaGyro = gyroMin; alphaGyro <= gyroMax; alphaGyro += 0.2) {
        for (double alphaAcc = accMin; alphaAcc <= accMax; alphaAcc += 0.2) {
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
        }
    }
    
    cout << "\n✓ FINAL Best (αGyro, αAcc): (" 
         << study.bestResult.alphaGyro << ", " 
         << study.bestResult.alphaAcc << ")\n";
    cout << "  Mean NEES: " << study.bestResult.meanNees << "\n";
    cout << "  Sum Deviations: " << study.bestResult.sumDeviations << "\n";
    
    return study;
}

/// Generate trajectory comparison files for visualization
void generateTrajectoryComparisons(
    const std::vector<std::pair<std::string, std::string>>& datasets,
    const NoiseTrialResult& bestAlphas,
    const NoiseTrialResult& worstAlphas) {
    
    cout << "\n" << string(100, '=') << "\n";
    cout << "GENERATING TRAJECTORY COMPARISONS\n";
    cout << string(100, '=') << "\n";
    
    for (const auto& [name, path] : datasets) {
        cout << "\nProcessing " << name << "...\n";
        
        Dataset dataset(path);
        
        /// Export ground truth (once per dataset)
        exportGroundTruthTrajectory(dataset, name);
        
        /// Export BEST prediction
        auto bestParams = dataset.configureImuParams(bestAlphas.alphaGyro, bestAlphas.alphaAcc);
        cout << "  Running BEST alphas (" << bestAlphas.alphaGyro 
             << ", " << bestAlphas.alphaAcc << ")...\n";
        exportPredictedTrajectory(dataset, bestParams, name, "BEST", 0.2);
        
        /// Export WORST prediction
        auto worstParams = dataset.configureImuParams(worstAlphas.alphaGyro, worstAlphas.alphaAcc);
        cout << "  Running WORST alphas (" << worstAlphas.alphaGyro 
             << ", " << worstAlphas.alphaAcc << ")...\n";
        exportPredictedTrajectory(dataset, worstParams, name, "WORST", 0.2);
        
        cout << "  ✓ All trajectories exported for " << name << "\n";
    }
    
    cout << "\n✓ All trajectory comparisons generated!\n";
    cout << "  Ground truth files: trajectory_*_groundtruth.csv\n";
    cout << "  Best predictions: trajectory_*_BEST_Gal3ImuEKF_0.2s.csv\n";
    cout << "  Worst predictions: trajectory_*_WORST_Gal3ImuEKF_0.2s.csv\n";
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

/// Print final summary
void printSummary(const NoiseCalibrationStudy& coarse, const NoiseCalibrationStudy& fine) {
    cout << "\n" << string(100, '=') << "\n";
    cout << "FINAL SUMMARY: OPTIMAL GYRO & ACCEL NOISE SCALING (0.2s NEES)\n";
    cout << string(100, '=') << "\n\n";
    
    cout << "Phase\t\tαGyro\tαAcc";
    for (const auto& name : fine.datasetNames) {
        cout << "\t" << name;
    }
    cout << "\tMean\tSumDev\n";
    cout << string(100, '-') << "\n";
    
    cout << fixed << setprecision(3);
    cout << "Coarse\t\t" 
         << coarse.bestResult.alphaGyro << "\t"
         << coarse.bestResult.alphaAcc;
    for (const auto& name : coarse.datasetNames) {
        cout << "\t" << coarse.bestResult.datasetNees.at(name);
    }
    cout << "\t" << coarse.bestResult.meanNees << "\t"
         << coarse.bestResult.sumDeviations << "\n";
    
    cout << "Fine\t\t"
         << fine.bestResult.alphaGyro << "\t"
         << fine.bestResult.alphaAcc;
    for (const auto& name : fine.datasetNames) {
        cout << "\t" << fine.bestResult.datasetNees.at(name);
    }
    cout << "\t" << fine.bestResult.meanNees << "\t"
         << fine.bestResult.sumDeviations << "\n";
    
    cout << string(100, '=') << "\n";
    
    double improvement = (coarse.bestResult.sumDeviations - fine.bestResult.sumDeviations) 
                        / coarse.bestResult.sumDeviations * 100.0;
    cout << "\n📈 Fine search improved NEES deviation by " 
         << fixed << setprecision(1) << improvement << "%\n";
    
    cout << "\n📊 RECOMMENDED PARAMETERS:\n";
    cout << "   αGyro = " << fine.bestResult.alphaGyro << "\n";
    cout << "   αAcc  = " << fine.bestResult.alphaAcc << "\n";
    cout << "   Mean NEES across " << fine.datasetNames.size() << " datasets: " 
         << fine.bestResult.meanNees << "\n";
    
    cout << "\n⚠️  WORST PARAMETERS (for comparison):\n";
    cout << "   αGyro = " << coarse.worstResult.alphaGyro << "\n";
    cout << "   αAcc  = " << coarse.worstResult.alphaAcc << "\n";
    cout << "   Mean NEES: " << coarse.worstResult.meanNees << "\n";
}

/// Main program
int main(int argc, char* argv[]) {
    try {
        cout << "\n";
        cout << "╔════════════════════════════════════════════════════════════════╗\n";
        cout << "║  2D NOISE CALIBRATION: GYRO & ACCEL SCALING                    ║\n";
        cout << "║  Goal: Minimize |NEES_0.2s -1| across ALL EuRoC datasets     ║\n";
        cout << "╚════════════════════════════════════════════════════════════════╝\n";
        
        const std::string dataDir = "../eval/data/euroc/";
        
        auto datasets = discoverDatasets(dataDir);
        
        if (datasets.empty()) {
            std::cerr << "❌ No datasets found in " << dataDir << std::endl;
            return 1;
        }
        
        cout << "\n✓ Found " << datasets.size() << " datasets:\n";
        for (const auto& [name, path] : datasets) {
            cout << "  • " << name << " (" << path << ")\n";
        }
        
        // Phase 1: Coarse search
        NoiseCalibrationStudy coarseStudy = runNoiseCalibration(datasets);
        exportResults(coarseStudy, "noise_calibration_2d_coarse.csv");
        
        // Phase 2: Fine search
        NoiseCalibrationStudy fineStudy = refineNoiseCalibration(datasets, 
                                                                coarseStudy.bestResult);
        exportResults(fineStudy, "noise_calibration_2d_fine.csv");
        
        // Print final summary
        printSummary(coarseStudy, fineStudy);
        
        // Phase 3: Generate trajectory comparisons
        generateTrajectoryComparisons(datasets, fineStudy.bestResult, coarseStudy.worstResult);
        
        cout << "\n✅ 2D noise calibration study complete!\n";
        cout << "📊 Check noise_calibration_2d_*.csv files for detailed results\n";
        cout << "📈 Check trajectory_*.csv files for visualization\n";
        cout << "\nNext: Run 'python3 plot_noise_calibration_trajectories.py' to visualize\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
}