/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   evalGal3NavStateImuEKFNEES.cpp
 * @brief  Clean NEES comparison table for EKF variants
 * @author Alec Kain
 */

#include "EKFNEESEvaluator.h"
#include <iostream>
#include <iomanip>

using namespace gtsam;
using namespace std;

/// NEES results for all intervals
struct DatasetResults {
    NEESResults gal3_0_2s;
    NEESResults gal3_0_5s;
    NEESResults gal3_1_0s;
    NEESResults navstate_0_2s;
    NEESResults navstate_0_5s;
    NEESResults navstate_1_0s;
};

/// Print table header
static void printTableHeader() {
    cout << "\n" << string(70, '=') << "\n";
    cout << "Dataset\t\tMethod\t\t\t0.2s\t0.5s\t1.0s\n";
    cout << string(70, '-') << "\n";
}

/// Print NEES results row
static void printTableRow(const string& datasetName,
                         const string& methodName,
                         const NEESResults& result_0_2s,
                         const NEESResults& result_0_5s,
                         const NEESResults& result_1_0s) {
    cout << datasetName << "\t" << methodName << ":\t"
         << fixed << setprecision(3)
         << result_0_2s.median << "\t"
         << result_0_5s.median << "\t"
         << result_1_0s.median << "\n";
}

/// Evaluate single dataset
static DatasetResults evaluateDataset(const string& datasetPath, const string& datasetName) {
    Dataset dataset(datasetPath);
    EKFNEESEvaluator evaluator(dataset);
    
    DatasetResults results;
    
    // Evaluate Gal3ImuEKF - use public API with dataset name
    results.gal3_0_2s = evaluator.runGal3ImuEKF(0.2, 3, datasetName);
    results.gal3_0_5s = evaluator.runGal3ImuEKF(0.5, 3, datasetName);
    results.gal3_1_0s = evaluator.runGal3ImuEKF(1.0, 3, datasetName);
    
    // Evaluate NavStateImuEKF - use public API with dataset name
    results.navstate_0_2s = evaluator.runNavStateImuEKF(0.2, 3, datasetName);
    results.navstate_0_5s = evaluator.runNavStateImuEKF(0.5, 3, datasetName);
    results.navstate_1_0s = evaluator.runNavStateImuEKF(1.0, 3.0, datasetName);
    
    return results;
}

/// Main evaluation program
int main(int argc, char* argv[]) {
    try {
        printTableHeader();
        
        // Evaluate MH01 dataset
        const string mh01Path = "../data/euroc/euroc_MH01.csv";
        DatasetResults mh01Results = evaluateDataset(mh01Path, "MH01");
        
        printTableRow("MH01", "Gal3ImuEKF", 
                     mh01Results.gal3_0_2s, mh01Results.gal3_0_5s, mh01Results.gal3_1_0s);
        printTableRow("MH01", "NavStateImuEKF", 
                     mh01Results.navstate_0_2s, mh01Results.navstate_0_5s, mh01Results.navstate_1_0s);
        
        cout << string(70, '-') << "\n";
        
        // Evaluate V202 dataset
        const string v202Path = "../data/euroc/euroc_V202.csv";
        DatasetResults v202Results = evaluateDataset(v202Path, "V202");
        
        printTableRow("V202", "Gal3ImuEKF", 
                     v202Results.gal3_0_2s, v202Results.gal3_0_5s, v202Results.gal3_1_0s);
        printTableRow("V202", "NavStateImuEKF", 
                     v202Results.navstate_0_2s, v202Results.navstate_0_5s, v202Results.navstate_1_0s);
        
        cout << string(70, '=') << "\n\n";
        cout << "Final table results for 0.2, 0.5, 1s preintegration times:" << "\n";
        printTableRow("MH01", "Gal3ImuEKF", 
                     mh01Results.gal3_0_2s, mh01Results.gal3_0_5s, mh01Results.gal3_1_0s);
        printTableRow("MH01", "NavStateImuEKF", 
                     mh01Results.navstate_0_2s, mh01Results.navstate_0_5s, mh01Results.navstate_1_0s);
        printTableRow("V202", "Gal3ImuEKF", 
                     v202Results.gal3_0_2s, v202Results.gal3_0_5s, v202Results.gal3_1_0s);
        printTableRow("V202", "NavStateImuEKF", 
                     v202Results.navstate_0_2s, v202Results.navstate_0_5s, v202Results.navstate_1_0s);
        
        return 0;
        
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
