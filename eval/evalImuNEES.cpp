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
        NEESEvaluator evaluator;
        vector<double> preint_times = {0.1, 0.2, 0.5, 1.0};
        const string data_path = "../eval/data/euroc/euroc_V202.csv";
        
        evaluator.evaluateNEES(data_path, preint_times, 3.0);  // alpha = 3
    } catch (const exception& e) {
        FAIL(e.what());
    }
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}