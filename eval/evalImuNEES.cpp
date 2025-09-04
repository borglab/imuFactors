/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   evalImuNEES.cpp
 * @brief  Evaluations for NEES
 * @author Alec Kain
 */

#include <CppUnitLite/TestHarness.h>
#include "NEESEvaluator.h"
#include <fstream>

using namespace std;
using namespace gtsam;

/**
 * This test suite uses the EuRoC MAV dataset, published in:
 * M. Burri, J. Nikolic, P. Gohl, T. Schneider, J. Rehder, S. Omari, M. Achtelik 
 * and R. Siegwart, The EuRoC micro aerial vehicle datasets, International 
 * Journal of Robotic Research, DOI: 10.1177/0278364915620033, early 2016.
 * 
 * Data download available at:
 * https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
 */

/* ************************************************************************* */
TEST(ImuFactor, NEES) {
    try {
        const string dataPath = "../eval/data/euroc/euroc_V202.csv";
        auto dataset = Dataset(dataPath);
        auto evaluator = NEESEvaluator(dataset);
        
        // Test multiple preintegration intervals
        vector<double> intervals = {0.1, 0.2, 0.5, 1.0};
        for (double interval : intervals) {
            auto results = evaluator.run(interval, 3.0);  // alpha = 3
            NEESEvaluator::printNeesStatistics(results);
            EXPECT(!results.neesValues.empty());
        }
    } catch (const exception& e) {
        FAIL(e.what());
    }
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}