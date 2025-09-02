#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/debug.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/navigation/tests/imuFactorTesting.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>

using namespace gtsam;
using namespace std;

// Use GTSAM's existing typedef or define with different name
typedef Eigen::Matrix<double, 15, 15> Matrix15x15;
typedef Eigen::Matrix<double, 15, 1> Vector15;

class NEESEvaluator {
private:
  const double kGravity = 9.81;

public:
  struct GTData {
    vector<NavState> nav_states;
    vector<imuBias::ConstantBias> biases;
    vector<Vector3> measured_omegas;
    vector<Vector3> measured_accs;
    vector<double> timestamps;
  };

  GTData loadEurocData(const string& filename) {
    GTData data;
    ifstream file(filename);
    
    if (!file.is_open()) {
      cerr << "Error: Cannot open file " << filename << endl;
      return data;
    }
    
    string line;
    getline(file, line);  // Skip header

    double t0 = 0;
    bool first = true;

    while (getline(file, line)) {
      if (line.empty()) continue;
      
      vector<double> row;
      stringstream ss(line);
      string value;
      
      while (getline(ss, value, ',')) {
        try {
          row.push_back(stod(value));
        } catch (const exception& e) {
          cerr << "Error parsing value: " << value << endl;
          continue;
        }
      }

      if (row.size() < 23) {
        cerr << "Warning: Skipping row with insufficient columns" << endl;
        continue;
      }

      if (first) {
        t0 = row[0];
        first = false;
      }

      data.timestamps.push_back(row[0] - t0);

      // Parse quaternion (w,x,y,z)
      Rot3 rot = Rot3::Quaternion(row[1], row[2], row[3], row[4]);
      Point3 vel(row[5], row[6], row[7]);
      Point3 pos(row[8], row[9], row[10]);
      data.nav_states.push_back(NavState(rot, pos, vel));

      Vector3 gyro_bias(row[11], row[12], row[13]);
      Vector3 acc_bias(row[14], row[15], row[16]);
      data.biases.push_back(imuBias::ConstantBias(acc_bias, gyro_bias));

      data.measured_omegas.push_back(Vector3(row[17], row[18], row[19]));
      data.measured_accs.push_back(Vector3(row[20], row[21], row[22]));
    }
    
    file.close();
    cout << "Loaded " << data.timestamps.size() << " data points from " << filename << endl;
    
    return data;
  }

  void evaluateNEES(const string& filename, const vector<double>& preint_times,
                   double alpha = 3.0) {
    GTData data = loadEurocData(filename);
    
    if (data.timestamps.empty()) {
      cerr << "Error: No data loaded" << endl;
      return;
    }
    
    if (data.timestamps.size() < 2) {
      cerr << "Error: Insufficient data points" << endl;
      return;
    }
    
    double sigma_gyro = alpha * 1.6968e-4;
    double sigma_acc = alpha * 2.0000e-3;
    double sigma_gyro_bias = alpha * 1.9393e-5;
    double sigma_acc_bias = alpha * 3.0000e-3;

    // Create PreintegrationCombinedParams with gravity magnitude
    // The constructor takes gravity magnitude, then we access n_gravity directly
    auto params = PreintegrationCombinedParams::MakeSharedD(kGravity);
    
    // Set gravity vector directly (it's a public member)
    params->n_gravity = Vector3(0, 0, -kGravity);
    
    // Set measurement covariances
    params->setGyroscopeCovariance(Matrix3::Identity() * pow(sigma_gyro, 2));
    params->setAccelerometerCovariance(Matrix3::Identity() * pow(sigma_acc, 2));
    
    // Set integration covariance to zero (matching Python implementation)
    params->setIntegrationCovariance(Matrix3::Zero());
    
    // Set bias random walk parameters
    params->setBiasAccCovariance(Matrix3::Identity() * pow(sigma_acc_bias, 2));
    params->setBiasOmegaCovariance(Matrix3::Identity() * pow(sigma_gyro_bias, 2));

    double dt = data.timestamps[1] - data.timestamps[0];
    double duration = data.timestamps.back();
    
    cout << "Data duration: " << duration << " seconds" << endl;
    cout << "Sampling rate: " << 1.0/dt << " Hz" << endl;
    cout << "GTSAM 15-DOF parameters configured successfully." << endl;

    for (double preint_time : preint_times) {
      cout << "----------------------------------------------------------------------" << endl;
      cout << "Analyzing with preintegration time: " << preint_time << " s\n";
      
      int N = static_cast<int>(preint_time / dt);  // Number of measurements per window
      if (N <= 0) {
        cerr << "Error: Invalid window size" << endl;
        continue;
      }
      
      int M = static_cast<int>(duration / preint_time);  // Number of windows
      if (M <= 0) {
        cerr << "Error: Not enough data for window size" << endl;
        continue;
      }
      
      N = static_cast<int>(data.nav_states.size() / M);  // Recalculate N based on M
      
      cout << "Test Setup: M=" << M << " lap(s), N=" << N << " states per lap." << endl;
      
      vector<double> nees_results;

      for (int m = 0; m < M; m++) {
        int start_idx = m * N;
        int end_idx = start_idx + N;
        
        if (end_idx > data.nav_states.size()) {
          continue;
        }

        // State and bias at the beginning and end of the subtrajectory
        NavState state_i_gt = data.nav_states[start_idx];
        imuBias::ConstantBias bias_i_gt = data.biases[start_idx];
        NavState state_j_gt = data.nav_states[end_idx - 1];
        imuBias::ConstantBias bias_j_gt = data.biases[end_idx - 1];

        // Create PreintegratedCombinedMeasurements with initial bias
        PreintegratedCombinedMeasurements pim(params, bias_i_gt);

        // Integrate measurements for this window
        for (int k = start_idx; k < end_idx - 1; k++) {
          if (k < data.measured_accs.size() && k < data.measured_omegas.size()) {
            pim.integrateMeasurement(data.measured_accs[k],
                                   data.measured_omegas[k], dt);
          }
        }

        // Predict and compute error
        NavState predicted_state = pim.predict(state_i_gt, bias_i_gt);
        
        // Calculate errors
        Vector9 nav_error = predicted_state.localCoordinates(state_j_gt);
        Vector6 bias_error = bias_j_gt.vector() - bias_i_gt.vector();
        
        // Combine errors into 15-DOF vector
        Vector15 error;
        error << nav_error, bias_error;
        
        // Get the 15x15 covariance matrix
        Matrix15x15 P = pim.preintMeasCov();
        
        // Calculate NEES
        try {
          double nees = error.transpose() * P.inverse() * error;
          nees_results.push_back(nees / 15.0);  // Normalize by DOF
        } catch (const exception& e) {
          // Skip if matrix inversion fails
          continue;
        }
      }

      if (nees_results.empty()) {
        cerr << "No valid NEES results for preintegration time " << preint_time << endl;
        continue;
      }

      // Analyze and Print Results
      Eigen::VectorXd nees_vector(nees_results.size());
      for (size_t i = 0; i < nees_results.size(); i++) {
        nees_vector(i) = nees_results[i];
      }

      double nees_mean = nees_vector.mean();
      sort(nees_results.begin(), nees_results.end());
      double nees_median = nees_results[nees_results.size()/2];
      double nees_var = (nees_vector.array() - nees_mean).square().mean();

      cout << "ANEES (15-DOF):    mean | median | variance\n";
      cout << "--------------------------------------------\n";
      cout << "GTSAM Results:   " << nees_mean << " |  " << nees_median << "  | " << nees_var << "\n";
      cout << "----------------------------------------------------------------------" << endl;
    }
  }
};

/* ************************************************************************* */
TEST(ImuFactor, NEES) {
  NEESEvaluator evaluator;
  vector<double> preint_times = {0.1, 0.2, 0.5, 1.0};
  
  const string data_path = "../eval/data/euroc/euroc_V202.csv";
  
  ifstream test_file(data_path);
  if (!test_file.is_open()) {
    cerr << "Error: Cannot find test data file at: " << data_path << endl;
    return;
  }
  test_file.close();
  
  evaluator.evaluateNEES(data_path, preint_times, 3.0);  // alpha = 3
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}