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
 * @brief  Unified noise calibration for dataset subsets (Machine Hall, Vicon Room, or All)
 *         with optional coarse and fine grid search
 * @author Alec Kain
 */

#include "NoiseCalibration.h"

using namespace gtsam;
using namespace std;

/// Print usage instructions
void printUsage(const char* programName) {
    cout << "\nUsage: " << programName << " [dataset_type] [search_mode]\n";
    cout << "\nArguments:\n";
    cout << "  dataset_type - Which datasets to calibrate on (default: all)\n";
    cout << "                 Options: 'machine_hall', 'vicon', 'all'\n";
    cout << "  search_mode  - Search strategy (default: both)\n";
    cout << "                 Options: 'coarse', 'fine', 'both'\n";
    cout << "\nSearch Modes:\n";
    cout << "  coarse       - Run coarse grid search only (faster)\n";
    cout << "  fine         - Run fine grid search only (requires prior coarse results)\n";
    cout << "  both         - Run coarse followed by fine search (most thorough)\n";
    cout << "\nExamples:\n";
    cout << "  " << programName << " machine_hall           # Coarse+fine on MH-series\n";
    cout << "  " << programName << " vicon coarse           # Coarse search on V-series only\n";
    cout << "  " << programName << " all both               # Full search on all datasets\n";
}

/// Run fine search around best coarse result
NoiseCalibrationStudy runFineSearchAround(
    const std::vector<std::pair<std::string, std::string>>& datasets,
    const NoiseTrialResult& coarseBest,
    const std::string& studyName) {
    
    NoiseCalibrationStudy study;
    for (const auto& [name, path] : datasets) {
        study.datasetNames.push_back(name);
    }
    
    cout << "\n" << string(100, '=') << "\n";
    cout << "FINE GRID SEARCH: " << studyName << "\n";
    cout << string(100, '=') << "\n";
    
    double gyroMin = std::max(0.1, coarseBest.alphaGyro * 0.7);
    double gyroMax = coarseBest.alphaGyro * 1.3;
    double accMin = std::max(0.1, coarseBest.alphaAcc * 0.7);
    double accMax = coarseBest.alphaAcc * 1.3;
    
    cout << "\nRefining around coarse best: (αGyro=" << coarseBest.alphaGyro 
         << ", αAcc=" << coarseBest.alphaAcc << ")\n";
    cout << "Gyro range: [" << gyroMin << ", " << gyroMax << "]\n";
    cout << "Accel range: [" << accMin << ", " << accMax << "]\n\n";
    
    double bestSumDev = 1e9;
    double worstSumDev = 0.0;
    
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
            
            if (trial.sumDeviations > worstSumDev) {
                worstSumDev = trial.sumDeviations;
                study.worstResult = trial;
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

/// Print comparison summary when both coarse and fine are run
void printComparisonSummary(const NoiseCalibrationStudy& coarse, 
                            const NoiseCalibrationStudy& fine,
                            const std::string& datasetType) {
    cout << "\n" << string(100, '=') << "\n";
    cout << "COMPARISON SUMMARY: " << datasetType << "\n";
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
    
    cout << "\n📊 FINAL RECOMMENDED PARAMETERS (" << datasetType << "):\n";
    cout << "   αGyro = " << fine.bestResult.alphaGyro << "\n";
    cout << "   αAcc  = " << fine.bestResult.alphaAcc << "\n";
    cout << "   Mean NEES across " << fine.datasetNames.size() << " datasets: " 
         << fine.bestResult.meanNees << "\n";
}

/// Main program
int main(int argc, char* argv[]) {
    try {
        // Default settings
        string datasetType = "all";
        string searchMode = "both";
        DatasetFilter filter = DatasetFilters::all;
        string studyName = "ALL DATASETS";
        string outputFilePrefix = "noise_calibration_all";
        
        // Parse command-line arguments
        if (argc >= 2) {
            datasetType = string(argv[1]);
            
            if (datasetType == "machine_hall" || datasetType == "mh") {
                filter = DatasetFilters::machineHall;
                studyName = "MACHINE HALL";
                outputFilePrefix = "noise_calibration_machine_hall";
            } else if (datasetType == "vicon" || datasetType == "v") {
                filter = DatasetFilters::viconRoom;
                studyName = "VICON ROOM";
                outputFilePrefix = "noise_calibration_vicon";
            } else if (datasetType == "all") {
                // Already set to defaults
            } else if (datasetType == "help" || datasetType == "-h" || datasetType == "--help") {
                printUsage(argv[0]);
                return 0;
            } else {
                cerr << "❌ Invalid dataset type: " << datasetType << "\n";
                printUsage(argv[0]);
                return 1;
            }
        }
        
        if (argc >= 3) {
            searchMode = string(argv[2]);
            if (searchMode != "coarse" && searchMode != "fine" && searchMode != "both") {
                cerr << "❌ Invalid search mode: " << searchMode << "\n";
                printUsage(argv[0]);
                return 1;
            }
        }
        
        if (argc > 3) {
            cerr << "❌ Too many arguments\n";
            printUsage(argv[0]);
            return 1;
        }
        
        // Print header
        cout << "\n";
        cout << "╔════════════════════════════════════════════════════════════════╗\n";
        cout << "║  " << left << setw(60) << (studyName + " NOISE CALIBRATION") << "║\n";
        cout << "║  Search mode: " << left << setw(47) << searchMode << "║\n";
        cout << "║  Goal: Minimize |NEES_0.2s - 1| across selected datasets     ║\n";
        cout << "╚════════════════════════════════════════════════════════════════╝\n";
        
        const std::string dataDirectory = "../eval/data/euroc/";
        
        // Discover datasets using the selected filter
        auto datasets = discoverFilteredDatasets(dataDirectory, filter);
        
        if (datasets.empty()) {
            std::cerr << "❌ No datasets found in " << dataDirectory << std::endl;
            return 1;
        }
        
        cout << "\n✓ Found " << datasets.size() << " datasets:\n";
        for (const auto& [name, path] : datasets) {
            cout << "  • " << name << "\n";
        }
        
        NoiseCalibrationStudy coarseStudy, fineStudy;
        bool hasCoarse = false, hasFine = false;
        
        // Phase 1: Coarse search (if requested)
        if (searchMode == "coarse" || searchMode == "both") {
            coarseStudy = runFilteredCalibration(datasets, studyName);
            exportCalibrationResults(coarseStudy, outputFilePrefix + "_coarse.csv");
            printCalibrationSummary(coarseStudy, studyName);
            hasCoarse = true;
            
            cout << "\n✅ Coarse search complete!\n";
            cout << "📊 Results saved to: " << outputFilePrefix << "_coarse.csv\n";
        }
        
        // Phase 2: Fine search (if requested)
        if (searchMode == "fine" || searchMode == "both") {
            // For fine-only mode, need to get coarse best from somewhere
            // This is a simplified version - in production, you'd load from CSV
            if (!hasCoarse) {
                cerr << "\n⚠️  Fine search requires coarse results.\n";
                cerr << "    Run coarse search first or use 'both' mode.\n";
                return 1;
            }
            
            fineStudy = runFineSearchAround(datasets, coarseStudy.bestResult, studyName);
            exportCalibrationResults(fineStudy, outputFilePrefix + "_fine.csv");
            hasFine = true;
            
            cout << "\n✅ Fine search complete!\n";
            cout << "📊 Results saved to: " << outputFilePrefix << "_fine.csv\n";
        }
        
        // Print comparison summary if both were run
        if (hasCoarse && hasFine) {
            printComparisonSummary(coarseStudy, fineStudy, studyName);
        }
        
        // Final summary
        cout << "\n" << string(100, '=') << "\n";
        cout << "✅ " << studyName << " calibration complete!\n";
        cout << string(100, '=') << "\n";
        
        if (hasCoarse && hasFine) {
            cout << "\n📊 FINAL OPTIMAL PARAMETERS:\n";
            cout << "   αGyro = " << fineStudy.bestResult.alphaGyro << "\n";
            cout << "   αAcc  = " << fineStudy.bestResult.alphaAcc << "\n";
            cout << "   Mean NEES: " << fineStudy.bestResult.meanNees << "\n";
        } else if (hasCoarse) {
            cout << "\n📊 COARSE OPTIMAL PARAMETERS:\n";
            cout << "   αGyro = " << coarseStudy.bestResult.alphaGyro << "\n";
            cout << "   αAcc  = " << coarseStudy.bestResult.alphaAcc << "\n";
            cout << "   Mean NEES: " << coarseStudy.bestResult.meanNees << "\n";
            cout << "\n💡 Tip: Run with 'both' or 'fine' mode for better results\n";
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
}