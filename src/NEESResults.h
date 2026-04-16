/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NEESResults.h
 * @brief   Struct for NEES evaluation results
 * @author  Alec Kain
 */

#pragma once

#include <vector>

namespace gtsam {

/// NEES evaluation results
struct NEESResults {
  std::vector<double> neesValues;
  double mean;
  double median;
  double variance;
  double preintTime;

  /// Print NEES statistics to stdout
  void printStatistics() const {
    std::cout << "\nAnalyzing with preintegration time: " << preintTime
              << " s\n";
    std::cout << "ANEES (15-DOF):    mean | median | variance\n"
              << "--------------------------------------------\n"
              << "Results:   " << mean << " | " << median << " | " << variance
              << "\n";
  }
};

}  // namespace gtsam
