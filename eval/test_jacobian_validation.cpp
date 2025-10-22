/**
 * @file test_jacobian_validation.cpp
 * @brief Validate Jacobian computations in NavStateImuEKF and Gal3ImuEKF
 * 
 * Verifies that analytical Jacobians match numerical derivatives
 * for state transition and measurement functions.
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
/// Test NavStateImuEKF Jacobian computation
TEST(NavStateImuEKF, JacobianValidation) {
  // Setup scenario and parameters
  const double velocity = 2.0;
  const double omega = 6.0 * M_PI / 180.0;  // 6 deg/s
  ConstantTwistScenario scenario(Vector3(0, 0, omega), Vector3(velocity, 0, 0));
  
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * pow(0.0028, 2));
  params->setGyroscopeCovariance(I_3x3 * pow(0.00016, 2));
  
  double dt = 0.005;
  
  // Initialize EKF
  NavState X0 = scenario.navState(0.0);
  Matrix9 P0 = Matrix9::Identity() * 0.01;
  
  // Get measurements
  Vector3 measuredOmega = scenario.omega_b(0.0);
  Vector3 measuredAcceleration = scenario.acceleration_b(0.0);
  
  // Compute analytical Jacobian using Dynamics() static method
  Matrix9 F_analytical;
  NavState X_next = NavStateImuEKF::Dynamics(
      params->n_gravity, X0, measuredOmega, measuredAcceleration, dt, F_analytical);
  
  // Compute numerical Jacobian
  std::function<NavState(const NavState&)> predictFunction = 
    [&](const NavState& state) {
      return NavStateImuEKF::Dynamics(
          params->n_gravity, state, measuredOmega, measuredAcceleration, dt);
    };
  
  Matrix9 F_numerical = numericalDerivative11<NavState, NavState>(
    predictFunction, X0, 1e-5);
  
  // Compare analytical and numerical Jacobians
  EXPECT(assert_equal(F_numerical, F_analytical, 1e-6));
  
  std::cout << "NavStateImuEKF: Jacobian validation PASSED\n";
}

/* ************************************************************************* */
/// Test covariance matrix symmetry after prediction
TEST(NavStateImuEKF, CovarianceSymmetry) {
  // Setup
  const double velocity = 2.0;
  const double omega = 6.0 * M_PI / 180.0;
  ConstantTwistScenario scenario(Vector3(0, 0, omega), Vector3(velocity, 0, 0));
  
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * pow(0.0028, 2));
  params->setGyroscopeCovariance(I_3x3 * pow(0.00016, 2));
  
  double dt = 0.005;
  
  // Initialize and propagate
  NavState X0 = scenario.navState(0.0);
  Matrix9 P0 = Matrix9::Identity() * 0.01;
  NavStateImuEKF ekf(X0, P0, params);
  
  for (int i = 0; i < 10; i++) {
    Vector3 measuredOmega = scenario.omega_b(i * dt);
    Vector3 measuredAcceleration = scenario.acceleration_b(i * dt);
    ekf.predict(measuredOmega, measuredAcceleration, dt);
  }
  
  Matrix9 P = ekf.covariance();
  
  // Check symmetry: P should equal P^T
  EXPECT(assert_equal(P, Matrix9(P.transpose()), 1e-10));
  
  // Check positive definiteness (all eigenvalues > 0)
  Eigen::SelfAdjointEigenSolver<Matrix9> solver(P);
  Vector9 eigenvalues = solver.eigenvalues();
  
  for (int i = 0; i < 9; i++) {
    EXPECT(eigenvalues(i) > -1e-10);  // Allow small numerical errors
  }
  
  std::cout << "NavStateImuEKF: Covariance symmetry validation PASSED\n";
}

/* ************************************************************************* */
/// Test Gal3ImuEKF Jacobian computation
TEST(Gal3ImuEKF, JacobianValidation) {
  // Setup scenario
  const double velocity = 2.0;
  const double omega = 6.0 * M_PI / 180.0;
  ConstantTwistScenario scenario(Vector3(0, 0, omega), Vector3(velocity, 0, 0));
  
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * pow(0.0028, 2));
  params->setGyroscopeCovariance(I_3x3 * pow(0.00016, 2));
  
  double dt = 0.005;
  
  // Initialize Gal3 EKF
  NavState X0Nav = scenario.navState(0.0);
  Gal3 X0(X0Nav.pose().rotation(), X0Nav.velocity(), X0Nav.position(), 0.0);
  Matrix P0 = Matrix::Identity(10, 10) * 0.01;
  
  // Get measurements
  Vector3 measuredOmega = scenario.omega_b(0.0);
  Vector3 measuredAcceleration = scenario.acceleration_b(0.0);
  
  // Compute analytical Jacobian using Dynamics() static method
  Matrix F_analytical = Matrix::Zero(10, 10);
  Gal3 X_next = Gal3ImuEKF::Dynamics(
      params->n_gravity, X0, measuredOmega, measuredAcceleration, dt,
      Gal3ImuEKF::TRACK_TIME_WITH_COVARIANCE, F_analytical);
  
  // Compute numerical Jacobian
  std::function<Gal3(const Gal3&)> predictFunction = 
    [&](const Gal3& state) {
      return Gal3ImuEKF::Dynamics(
          params->n_gravity, state, measuredOmega, measuredAcceleration, dt,
          Gal3ImuEKF::TRACK_TIME_WITH_COVARIANCE);
    };
  
  Matrix F_numerical = numericalDerivative11<Gal3, Gal3>(
    predictFunction, X0, 1e-5);
  
  // Compare Jacobians
  EXPECT(assert_equal(F_numerical, F_analytical, 1e-6));
  
  std::cout << "Gal3ImuEKF: Jacobian validation PASSED\n";
}

/* ************************************************************************* */
/// Test Gal3ImuEKF covariance symmetry
TEST(Gal3ImuEKF, CovarianceSymmetry) {
  // Setup
  const double velocity = 2.0;
  const double omega = 6.0 * M_PI / 180.0;
  ConstantTwistScenario scenario(Vector3(0, 0, omega), Vector3(velocity, 0, 0));
  
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * pow(0.0028, 2));
  params->setGyroscopeCovariance(I_3x3 * pow(0.00016, 2));
  
  double dt = 0.005;
  
  // Initialize and propagate
  NavState X0Nav = scenario.navState(0.0);
  Gal3 X0(X0Nav.pose().rotation(), X0Nav.velocity(), X0Nav.position(), 0.0);
  Matrix P0 = Matrix::Identity(10, 10) * 0.01;
  
  Gal3ImuEKF ekf(X0, P0, params, Gal3ImuEKF::TRACK_TIME_WITH_COVARIANCE);
  
  for (int i = 0; i < 10; i++) {
    Vector3 measuredOmega = scenario.omega_b(i * dt);
    Vector3 measuredAcceleration = scenario.acceleration_b(i * dt);
    ekf.predict(measuredOmega, measuredAcceleration, dt);
  }
  
  Matrix P = ekf.covariance();
  
  // Check symmetry
  EXPECT(assert_equal(P, Matrix(P.transpose()), 1e-10));
  
  // Check positive definiteness
  Eigen::SelfAdjointEigenSolver<Matrix> solver(P);
  Vector eigenvalues = solver.eigenvalues();
  
  for (int i = 0; i < 10; i++) {
    EXPECT(eigenvalues(i) > -1e-10);
  }
  
  std::cout << "Gal3ImuEKF: Covariance symmetry validation PASSED\n";
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}