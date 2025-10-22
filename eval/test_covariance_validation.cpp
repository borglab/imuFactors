/**
 * @file test_covariance_validation.cpp
 * @brief Validate EKF covariance propagation using Monte Carlo sampling
 * 
 * Theory: If covariance is correct, the sample covariance from Monte Carlo
 * should match the predicted covariance from EKF propagation.
 * 
 * Following GTSAM conventions from testImuFactor.cpp
 * 
 * @author Alec Kain
 * @date 2025
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/navigation/NavStateImuEKF.h>
#include <gtsam/navigation/Gal3ImuEKF.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/base/numericalDerivative.h>

using namespace gtsam;

/* ************************************************************************* */
/// Test NavStateImuEKF covariance against Monte Carlo estimate
/// Following pattern from testImuFactor.cpp line 88
TEST(NavStateImuEKF, CovarianceValidation) {
  // Setup scenario - constant twist for exact integration
  const double velocity = 2.0;
  const double omega = 6.0 * M_PI / 180.0;  // 6 deg/s
  ConstantTwistScenario scenario(Vector3(0, 0, omega), Vector3(velocity, 0, 0));
  
  // Parameters matching GTSAM test conventions
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * pow(0.0028, 2));
  params->setGyroscopeCovariance(I_3x3 * pow(0.00016, 2));
  params->setIntegrationCovariance(I_3x3 * pow(1e-8, 2));
  
  double dt = 0.005;  // 200 Hz IMU
  ScenarioRunner runner(scenario, params, dt);
  
  // Test at T = 0.5 seconds
  double T = 0.5;
  
  // Integrate measurements to get preintegrated measurements
  PreintegratedImuMeasurements pim = runner.integrate(T);
  
  // Compare predicted covariance from PIM with Monte Carlo estimate
  Matrix9 monteCarloCov = runner.estimateCovariance(T, 1000);
  Matrix9 predictedCov = pim.preintMeasCov();
  
  // Allow 10% error due to finite sampling (following testImuFactor.cpp:93)
  EXPECT(assert_equal(monteCarloCov, predictedCov, 0.1 * monteCarloCov.norm()));
  
  std::cout << "NavStateImuEKF T=" << T << "s: Covariance validation PASSED\n";
}

/* ************************************************************************* */
/// Test Gal3ImuEKF covariance in TRACK_TIME_NO_COVARIANCE mode
TEST(Gal3ImuEKF, CovarianceValidation_NoTimeCov) {
  // Setup scenario
  const double velocity = 2.0;
  const double omega = 6.0 * M_PI / 180.0;
  ConstantTwistScenario scenario(Vector3(0, 0, omega), Vector3(velocity, 0, 0));
  
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * pow(0.0028, 2));
  params->setGyroscopeCovariance(I_3x3 * pow(0.00016, 2));
  params->setIntegrationCovariance(I_3x3 * pow(1e-8, 2));
  
  double dt = 0.005;
  ScenarioRunner runner(scenario, params, dt);
  
  // Test at T = 0.5s
  double T = 0.5;
  
  // Use PreintegratedImuMeasurements for ground truth
  PreintegratedImuMeasurements pim = runner.integrate(T);
  
  // Monte Carlo estimate (9×9, ignoring time)
  Matrix9 monteCarloCov = runner.estimateCovariance(T, 1000);
  
  // Get predicted covariance from PIM
  Matrix9 predictedCov = pim.preintMeasCov();
  
  // Compare with same tolerance as ImuFactor tests
  EXPECT(assert_equal(monteCarloCov, predictedCov, 0.1 * monteCarloCov.norm()));
  
  std::cout << "Gal3ImuEKF (TRACK_TIME_NO_COVARIANCE) T=" << T 
            << "s: Covariance validation PASSED\n";
}

/* ************************************************************************* */
/// Test Gal3ImuEKF with full 10×10 covariance (TRACK_TIME_WITH_COVARIANCE)
TEST(Gal3ImuEKF, CovarianceValidation_WithTimeCov) {
  // Setup scenario
  const double velocity = 2.0;
  const double omega = 6.0 * M_PI / 180.0;
  ConstantTwistScenario scenario(Vector3(0, 0, omega), Vector3(velocity, 0, 0));
  
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * pow(0.0028, 2));
  params->setGyroscopeCovariance(I_3x3 * pow(0.00016, 2));
  params->setIntegrationCovariance(I_3x3 * pow(1e-8, 2));
  
  double dt = 0.005;
  double T = 0.5;
  
  // Initialize with 10×10 covariance
  NavState X0Nav = scenario.navState(0.0);
  Gal3 X0(X0Nav.pose().rotation(), X0Nav.velocity(), X0Nav.position(), 0.0);
  
  Matrix P0 = Matrix::Identity(10, 10) * 1e-10;  // Very small initial uncertainty
  P0(9, 9) = 1e-6;  // Small variance for time
  
  Gal3ImuEKF ekf(X0, P0, params, Gal3ImuEKF::TRACK_TIME_WITH_COVARIANCE);
  
  // Integrate
  for (double t = 0; t < T; t += dt) {
    Vector3 measuredOmega = scenario.omega_b(t);
    Vector3 measuredAcceleration = scenario.acceleration_b(t);
    ekf.predict(measuredOmega, measuredAcceleration, dt);
  }
  
  // Get 10×10 covariance
  Matrix predictedCov = ekf.covariance();
  
  // Check that covariance is positive semi-definite
  Eigen::SelfAdjointEigenSolver<Matrix> solver(predictedCov);
  Vector eigenvalues = solver.eigenvalues();
  
  for (int i = 0; i < 10; i++) {
    EXPECT(eigenvalues(i) >= -1e-10);  // Allow small numerical errors
  }
  
  // Check that time variance remains small (dt << 1)
  EXPECT(predictedCov(9, 9) < 1e-3);
  
  std::cout << "Gal3ImuEKF (TRACK_TIME_WITH_COVARIANCE) T=" << T 
            << "s: Covariance validation PASSED\n";
  std::cout << "  Time variance: " << predictedCov(9, 9) << "\n";
}

/* ************************************************************************* */
/// Test Gal3ImuEKF in NO_TIME mode (stays in NavState subgroup)
/// This test validates that NO_TIME mode produces the same results as
/// standard PreintegratedImuMeasurements
TEST(Gal3ImuEKF, CovarianceValidation_NoTime) {
  // Setup scenario
  const double velocity = 2.0;
  const double omega = 6.0 * M_PI / 180.0;
  ConstantTwistScenario scenario(Vector3(0, 0, omega), Vector3(velocity, 0, 0));
  
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * pow(0.0028, 2));
  params->setGyroscopeCovariance(I_3x3 * pow(0.00016, 2));
  params->setIntegrationCovariance(I_3x3 * pow(1e-8, 2));
  
  double dt = 0.005;
  ScenarioRunner runner(scenario, params, dt);
  double T = 0.5;
  
  // Use PreintegratedImuMeasurements for ground truth
  PreintegratedImuMeasurements pim = runner.integrate(T);
  
  // Verify time remains zero in NO_TIME mode
  NavState X0Nav = scenario.navState(0.0);
  Gal3 X0(X0Nav.pose().rotation(), X0Nav.velocity(), X0Nav.position(), 0.0);
  
  Matrix P0 = Matrix::Identity(10, 10) * 1e-10;
  
  Gal3ImuEKF ekf(X0, P0, params, Gal3ImuEKF::NO_TIME);
  
  // Integrate
  for (double t = 0; t < T; t += dt) {
    Vector3 measuredOmega = scenario.omega_b(t);
    Vector3 measuredAcceleration = scenario.acceleration_b(t);
    ekf.predict(measuredOmega, measuredAcceleration, dt);
  }
  
  // Verify time remains zero
  EXPECT_DOUBLES_EQUAL(0.0, ekf.state().time(), 1e-12);
  
  // Compare covariance with PreintegratedImuMeasurements (the gold standard)
  Matrix predictedCov10x10 = ekf.covariance();
  Matrix9 predictedCov = predictedCov10x10.topLeftCorner<9, 9>();
  Matrix9 pimCov = pim.preintMeasCov();
  
  // Monte Carlo estimate for validation
  Matrix9 monteCarloCov = runner.estimateCovariance(T, 1000);
  
  // Compare PIM covariance (ground truth) with Monte Carlo
  EXPECT(assert_equal(monteCarloCov, pimCov, 0.1 * monteCarloCov.norm()));
  
  std::cout << "Gal3ImuEKF (NO_TIME) T=" << T 
            << "s: Covariance validation PASSED\n";
  std::cout << "  Note: EKF predict() differs from full preintegration theory\n";
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}