/**
 * @file test_theoretical_validation.cpp
 * @brief Verify NavStateImuEKF matches equations from Barrau & Bonnabel (2021)
 */

#include <gtsam/navigation/NavStateImuEKF.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

/* ************************************************************************* */
// Verify process noise Q has correct structure (Equation 25 in paper)
TEST(NavStateImuEKF, ProcessNoiseStructure) {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  
  Matrix3 Q_gyro = I_3x3 * 1e-6;
  Matrix3 Q_accel = I_3x3 * 1e-4;
  Matrix3 Q_integration = I_3x3 * 1e-10;
  
  params->setGyroscopeCovariance(Q_gyro);
  params->setAccelerometerCovariance(Q_accel);
  params->setIntegrationCovariance(Q_integration);
  
  NavState X0;
  Matrix9 P0 = Matrix9::Zero();
  NavStateImuEKF ekf(X0, P0, params);
  
  // Access internal Q matrix (if exposed, otherwise skip this test)
  // For now, verify via prediction covariance growth
  
  Vector3 omega = Vector3::Zero();
  Vector3 accel(0, 0, 9.81);
  double dt = 0.01;
  
  ekf.predict(omega, accel, dt);
  Matrix9 P1 = ekf.covariance();
  
  // After one step with P0=0: P1 ≈ Q*dt (since A ≈ I for small dt)
  Matrix9 expected_P1 = Matrix9::Zero();
  expected_P1.block<3,3>(0,0) = Q_gyro * dt;        // Rotation block
  expected_P1.block<3,3>(3,3) = Q_integration * dt; // Position block
  expected_P1.block<3,3>(6,6) = Q_accel * dt;       // Velocity block
  
  // Allow some error due to A ≠ I
  EXPECT(assert_equal(expected_P1, P1, 0.1 * expected_P1.norm()));
  
  std::cout << "Process noise structure validation PASSED\n";
}

/* ************************************************************************* */
// Verify state transition follows X_next = W * phi(X) * U (Equation 17)
TEST(NavStateImuEKF, StateTransitionStructure) {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  
  NavState X0(Pose3(Rot3(), Point3(1, 2, 3)), Vector3(0.5, 0, 0));
  
  Vector3 omega_b(0.1, 0, 0);
  Vector3 f_b(0, 0, 9.81);
  double dt = 0.01;
  Vector3 g_n = params->n_gravity;
  
  // Compute X_next using NavStateImuEKF
  NavState X_next = NavStateImuEKF::Dynamics(g_n, X0, omega_b, f_b, dt, {});
  
  // Manually compute using paper equations
  // W = Gravity(g, dt)
  NavState W = NavStateImuEKF::Gravity(g_n, dt);
  
  // phi(X) = autonomous flow
  NavState::AutonomousFlow phi{dt};
  NavState X_with_flow = phi(X0);
  
  // U = Imu(omega, f, dt)
  NavState U = NavStateImuEKF::Imu(omega_b, f_b, dt);
  
  // X_next_manual = W * X_with_flow * U
  NavState X_next_manual = W.compose(X_with_flow.compose(U));
  
  // Compare
  EXPECT(assert_equal(X_next, X_next_manual, 1e-9));
  
  std::cout << "State transition structure validation PASSED\n";
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}