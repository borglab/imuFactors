/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalExportTrajectories.cpp
 * @brief  Export trajectories using optimal and worst noise parameters for visualization
 * @author Alec Kain
 */

#include "EKFNEESEvaluator.h"
#include "Dataset.h"
#include "TrajectoryValidator.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>

using namespace gtsam;
using namespace std;
namespace fs = std::filesystem;

/// Noise parameter configurations
struct NoiseParameters {
    double alphaGyro;
    double alphaAcc;
    std::string label;
};

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

/// Export ground truth trajectory to CSV
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

/// Main program
int main(int argc, char* argv[]) {
    try {
        cout << "\n";
        cout << "╔════════════════════════════════════════════════════════════════╗\n";
        cout << "║  TRAJECTORY EXPORT: BEST vs WORST NOISE PARAMETERS            ║\n";
        cout << "╚════════════════════════════════════════════════════════════════╝\n";
        
        const std::string dataDir = "../eval/data/euroc/";
        
        NoiseParameters bestParams{13.0, 9.4, "BEST"};
        NoiseParameters worstParams{0.5, 0.5, "WORST"};
        
        cout << "\n📊 PARAMETERS:\n";
        cout << "  Best:  αGyro = " << bestParams.alphaGyro << ", αAcc = " << bestParams.alphaAcc << "\n";
        cout << "  Worst: αGyro = " << worstParams.alphaGyro << ", αAcc = " << worstParams.alphaAcc << "\n";
        
        auto datasets = discoverDatasets(dataDir);
        
        if (datasets.empty()) {
            std::cerr << "❌ No datasets found in " << dataDir << std::endl;
            return 1;
        }
        
        cout << "\n✓ Found " << datasets.size() << " datasets\n";
        
        /// Export NEES summary
        std::ofstream neesSummary("nees_summary.csv");
        neesSummary << "dataset,best_nees,worst_nees,nees_ratio\n";
        neesSummary << std::fixed << std::setprecision(4);
        
        cout << "\n" << string(80, '=') << "\n";
        cout << "EXPORTING TRAJECTORIES AND COMPUTING NEES\n";
        cout << string(80, '=') << "\n";
        
        cout << "\nDataset\t\tBest NEES\tWorst NEES\tRatio (Worse/Better)\n";
        cout << string(80, '-') << "\n";
        
        for (const auto& [name, path] : datasets) {
            cout << "\n" << name << ":\n";
            
            Dataset dataset(path);
            
            /// Export ground truth (once per dataset)
            exportGroundTruthTrajectory(dataset, name);
            
            /// Create EKFNEESEvaluator for this dataset
            EKFNEESEvaluator evaluator(dataset);
            
            auto bestImuParams = dataset.configureImuParams(bestParams.alphaGyro, bestParams.alphaAcc);
            
            // This automatically exports trajectory files:
            // - trajectory_<name>_BEST_Gal3ImuEKF_0.2s_predicted.csv
            // - trajectory_<name>_BEST_Gal3ImuEKF_0.2s_errors.csv
            auto bestResult = evaluator.runGal3ImuEKF(
                0.2,                                // preintegrationTime
                bestImuParams,                      // IMU parameters
                name + "_" + bestParams.label       // suffix for filename
            );
            
            double bestNees = bestResult.median;
            std::cout << "  ✓ BEST: Exported trajectories and computed NEES = " << bestNees << "\n";
            
            /// ============================================================
            /// WORST PARAMETERS: Use EKFNEESEvaluator::runGal3ImuEKF
            /// ============================================================
            auto worstImuParams = dataset.configureImuParams(worstParams.alphaGyro, worstParams.alphaAcc);
            
            // This automatically exports trajectory files:
            // - trajectory_<name>_WORST_Gal3ImuEKF_0.2s_predicted.csv
            // - trajectory_<name>_WORST_Gal3ImuEKF_0.2s_errors.csv
            auto worstResult = evaluator.runGal3ImuEKF(
                0.2,                                 // preintegrationTime
                worstImuParams,                      // IMU parameters
                name + "_" + worstParams.label       // suffix for filename
            );
            
            double worstNees = worstResult.median;
            std::cout << "  ✓ WORST: Exported trajectories and computed NEES = " << worstNees << "\n";
            
            /// Print and save NEES comparison
            double neesRatio = (bestNees > 0) ? (worstNees / bestNees) : 0.0;
            cout << "  " << name << "\t\t" 
                 << std::setw(8) << bestNees << "\t" 
                 << std::setw(8) << worstNees << "\t"
                 << std::setw(8) << neesRatio << "x\n";
            
            neesSummary << name << "," << bestNees << "," << worstNees << "," << neesRatio << "\n";
            
            cout << "  ✓ Completed " << name << "\n";
        }
        
        neesSummary.close();
        
        cout << "\n" << string(80, '=') << "\n";
        cout << "✅ All trajectories and NEES values exported!\n";
        cout << "════════════════════════════════════════════════════════════════\n";
        
        cout << "\n📊 Next: Run 'python3 ../eval/plot_trajectories_plotly.py' for interactive visualization\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
}