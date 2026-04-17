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
#include "NoiseCalibration.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <sstream>

using namespace gtsam;
using namespace std;
namespace fs = std::filesystem;

/// Noise parameter configurations
struct NoiseParameters {
    double alphaGyro;
    double alphaAcc;
    std::string label;
};

/// Print usage instructions
void printUsage(const char* programName) {
    cout << "\nUsage: " << programName << " [options]\n";
    cout << "\nOptions:\n";
    cout << "  --best-gyro <value>     Gyroscope noise scaling for optimal parameters (default: 13.0)\n";
    cout << "  --best-acc <value>      Accelerometer noise scaling for optimal parameters (default: 9.4)\n";
    cout << "  --worst-gyro <value>    Gyroscope noise scaling for worst parameters (default: 0.5)\n";
    cout << "  --worst-acc <value>     Accelerometer noise scaling for worst parameters (default: 0.5)\n";
    cout << "  --dataset-type <type>   Dataset type: 'all', 'machine_hall', 'vicon' (default: all)\n";
    cout << "  --help, -h              Show this help message\n";
    cout << "\nShort form (positional arguments):\n";
    cout << "  " << programName << " <bestAlphaGyro> <bestAlphaAcc> <worstAlphaGyro> <worstAlphaAcc>\n";
    cout << "\nExamples:\n";
    cout << "  " << programName << "                           # Use default parameters\n";
    cout << "  " << programName << " 13.0 9.4 0.5 0.5          # Set all parameters\n";
    cout << "  " << programName << " --dataset-type vicon     # Export only Vicon datasets\n";
    cout << "  " << programName << " --best-gyro 15.0 --best-acc 10.0\n";
}

/// Parse named command-line arguments
bool parseNamedArguments(int argc, char* argv[], 
                        double& bestAlphaGyro, double& bestAlphaAcc,
                        double& worstAlphaGyro, double& worstAlphaAcc,
                        string& datasetType) {
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            exit(0);
        } else if (arg == "--best-gyro" && i + 1 < argc) {
            bestAlphaGyro = std::stod(argv[++i]);
        } else if (arg == "--best-acc" && i + 1 < argc) {
            bestAlphaAcc = std::stod(argv[++i]);
        } else if (arg == "--worst-gyro" && i + 1 < argc) {
            worstAlphaGyro = std::stod(argv[++i]);
        } else if (arg == "--worst-acc" && i + 1 < argc) {
            worstAlphaAcc = std::stod(argv[++i]);
        } else if (arg == "--dataset-type" && i + 1 < argc) {
            datasetType = argv[++i];
        } else {
            cerr << "❌ Unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
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

    for (const auto& truth : dataset.truth) {
      const NavState& trueState = truth.navState;
      Vector3 rpy = trueState.attitude().rpy() * 180.0 / M_PI;

      file << truth.timestamp << "," << trueState.position().x() << ","
           << trueState.position().y() << "," << trueState.position().z() << ","
           << trueState.velocity().x() << "," << trueState.velocity().y() << ","
           << trueState.velocity().z() << "," << rpy.x() << "," << rpy.y()
           << "," << rpy.z() << "\n";
    }

    file.close();
    std::cout << "  ✓ Exported ground truth: " << filename << "\n";
}

/// Main program
int main(int argc, char* argv[]) {
    try {
        cout << "\n";
        cout << "╔════════════════════════════════════════════════════════════════╗\n";
        cout << "║  TRAJECTORY EXPORT: BEST vs WORST NOISE PARAMETERS            ║\n";
        cout << "╚════════════════════════════════════════════════════════════════╝\n";
        
        // Default values from optimal calibration results
        double bestAlphaGyro = 13.0;
        double bestAlphaAcc = 9.4;
        double worstAlphaGyro = 0.5;
        double worstAlphaAcc = 0.5;
        string datasetType = "all";
        
        // Parse command-line arguments
        if (argc == 5) {
            // Short form: positional arguments
            try {
                bestAlphaGyro = std::stod(argv[1]);
                bestAlphaAcc = std::stod(argv[2]);
                worstAlphaGyro = std::stod(argv[3]);
                worstAlphaAcc = std::stod(argv[4]);
                cout << "\n✓ Using custom noise parameters from command line\n";
            } catch (const std::exception& e) {
                std::cerr << "❌ Invalid noise parameter(s): " << e.what() << std::endl;
                printUsage(argv[0]);
                return 1;
            }
        } else if (argc > 1) {
            // Named arguments
            if (!parseNamedArguments(argc, argv, bestAlphaGyro, bestAlphaAcc,
                                    worstAlphaGyro, worstAlphaAcc, datasetType)) {
                return 1;
            }
            cout << "\n✓ Using custom parameters\n";
        } else {
            cout << "\n✓ Using default noise parameters (from optimal calibration)\n";
        }
        
        NoiseParameters bestParams{bestAlphaGyro, bestAlphaAcc, "BEST"};
        NoiseParameters worstParams{worstAlphaGyro, worstAlphaAcc, "WORST"};
        
        const std::string dataDirectory = "../data/euroc/";
        
        cout << "\n📊 PARAMETERS:\n";
        cout << "  Best:  αGyro = " << bestParams.alphaGyro << ", αAcc = " << bestParams.alphaAcc << "\n";
        cout << "  Worst: αGyro = " << worstParams.alphaGyro << ", αAcc = " << worstParams.alphaAcc << "\n";
        cout << "  Dataset type: " << datasetType << "\n";
        
        // Select dataset filter
        DatasetFilter filter;
        if (datasetType == "machine_hall" || datasetType == "mh") {
            filter = DatasetFilters::machineHall;
        } else if (datasetType == "vicon" || datasetType == "v") {
            filter = DatasetFilters::viconRoom;
        } else {
            filter = DatasetFilters::all;
        }
        
        auto datasets = discoverFilteredDatasets(dataDirectory, filter);
        
        if (datasets.empty()) {
            std::cerr << "❌ No datasets found in " << dataDirectory << std::endl;
            return 1;
        }
        
        cout << "\n✓ Found " << datasets.size() << " datasets\n";
        
        /// Export NEES summary
        std::ofstream neesSummary("nees_summary_" + datasetType + ".csv");
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
            
            /// BEST PARAMETERS
            auto bestImuParams = dataset.configureImuParams(bestParams.alphaGyro, bestParams.alphaAcc);
            auto bestResult = evaluator.runGal3ImuEKF(0.2, bestImuParams, name + "_" + bestParams.label);
            double bestNees = bestResult.median;
            std::cout << "  ✓ BEST: Exported trajectories and computed NEES = " << bestNees << "\n";
            
            /// WORST PARAMETERS
            auto worstImuParams = dataset.configureImuParams(worstParams.alphaGyro, worstParams.alphaAcc);
            auto worstResult = evaluator.runGal3ImuEKF(0.2, worstImuParams, name + "_" + worstParams.label);
            double worstNees = worstResult.median;
            std::cout << "  ✓ WORST: Exported trajectories and computed NEES = " << worstNees << "\n";
            
            /// Print and save NEES comparison
            double neesRatio = (bestNees > 0) ? (worstNees / bestNees) : 0.0;
            cout << "  " << name << "\t\t" 
                 << std::setw(8) << bestNees << "\t" 
                 << std::setw(8) << worstNees << "\t"
                 << std::setw(8) << neesRatio << "x\n";
            
            neesSummary << name << "," << bestNees << "," << worstNees << "," << neesRatio << "\n";
        }
        
        neesSummary.close();
        
        cout << "\n" << string(80, '=') << "\n";
        cout << "✅ All trajectories and NEES values exported!\n";
        cout << "════════════════════════════════════════════════════════════════\n";
        cout << "\n📊 NEES summary saved to: nees_summary_" << datasetType << ".csv\n";
        cout << "📊 Next: Run 'python3 ../apps/plot_trajectories_plotly.py' for interactive visualization\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
}
