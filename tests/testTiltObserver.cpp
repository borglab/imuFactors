/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved.
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testTiltObserver.cpp
 * @brief  Unit tests for the tilt and gyroscope-bias observer helpers
 */

#include <CppUnitLite/TestHarness.h>

#include <cmath>

#include "TiltObserver.h"

using namespace gtsam;

/* ************************************************************************* */
TEST(TiltObserver, RollPitchFromUpDirection) {
  const double roll = 0.4;
  const double pitch = -0.2;
  const double yaw = 0.7;
  const Rot3 rotation = Rot3::Ypr(yaw, pitch, roll);
  const Unit3 upDirection(rotation.matrix().transpose() * Vector3::UnitZ());
  const Unit3 gravityDirection(-upDirection.point3());

  const Vector2 rollPitch = rollPitchFromUpDirection(upDirection);
  const Vector2 gravityRollPitch =
      rollPitchFromGravityDirection(gravityDirection);

  EXPECT_DOUBLES_EQUAL(roll, rollPitch.x(), 1e-12);
  EXPECT_DOUBLES_EQUAL(pitch, rollPitch.y(), 1e-12);
  EXPECT_DOUBLES_EQUAL(roll, gravityRollPitch.x(), 1e-12);
  EXPECT_DOUBLES_EQUAL(pitch, gravityRollPitch.y(), 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, StationaryAlignedSampleLeavesStateUnchanged) {
  TiltObserver observer(1.0, 10.0, 0.005);

  observer(Vector3::Zero(), Vector3(0, 0, 9.81));

  EXPECT(observer.b_hat.has_value());
  EXPECT(observer.n_hat.has_value());
  EXPECT(observer.b_hat->norm() < 1e-12);
  EXPECT((observer.n_hat->point3() - Vector3(0, 0, -1)).norm() < 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, DefaultObserverStartsUninitialized) {
  const TiltObserver observer(0.25, 3.0, 0.005);

  EXPECT(!observer.b_hat.has_value());
  EXPECT(!observer.n_hat.has_value());
}

/* ************************************************************************* */
TEST(TiltObserver, PredictInitializesBiasFromGyroSample) {
  TiltObserver observer(0.25, 3.0, 0.005);
  const Vector3 omega_b(0.1, -0.2, 0.3);

  observer.predict(omega_b);

  EXPECT(observer.b_hat.has_value());
  EXPECT(!observer.n_hat.has_value());
  EXPECT((observer.b_hat.value() - omega_b).norm() < 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, UpdateInitializesGravityFromSpecificForce) {
  TiltObserver observer(0.25, 3.0, 0.005);
  const Vector3 f_b(9.81, 0.0, 0.0);

  observer.update(f_b);

  EXPECT(!observer.b_hat.has_value());
  EXPECT(observer.n_hat.has_value());
  EXPECT((observer.n_hat->point3() - Vector3(-1, 0, 0)).norm() < 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, UpdateSkipsSpecificForceMagnitudeFarFromGravity) {
  TiltObserver observer(0.25, 3.0, 0.005);

  observer.update(Vector3(2.0 * 9.81, 0.0, 0.0));

  EXPECT(!observer.b_hat.has_value());
  EXPECT(!observer.n_hat.has_value());
}

/* ************************************************************************* */
TEST(TiltObserver, FirstOperatorCallInitializesBothStates) {
  const Vector3 f_b(9.81, 0.0, 0.0);
  const Vector3 omega_b(0.1, -0.2, 0.3);
  TiltObserver observer(0.25, 3.0, 0.005);

  observer(omega_b, f_b);

  EXPECT(observer.b_hat.has_value());
  EXPECT(observer.n_hat.has_value());
  EXPECT((observer.b_hat.value() - omega_b).norm() < 1e-12);
  EXPECT((observer.n_hat->point3() - Vector3(-1, 0, 0)).norm() < 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, PredictWithDeltaRotationMatchesIntegratedGyroPrediction) {
  const double dt = 0.2;
  const Vector3 angularVelocity(0.3, -0.2, 0.1);
  const Unit3 initialGravity(Vector3(-0.2, 0.4, -0.8944271909999159));
  TiltObserver gyroObserver(0.25, 3.0, dt, initialGravity, Vector3::Zero());
  TiltObserver pimObserver(0.25, 3.0, dt, initialGravity, Vector3::Zero());

  gyroObserver.predict(angularVelocity);
  pimObserver.predict(Rot3::Expmap(angularVelocity * dt));

  EXPECT((gyroObserver.n_hat->point3() - pimObserver.n_hat->point3()).norm() <
         1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, WindowOperatorInitializesGravityFromAverageSpecificForce) {
  TiltObserver observer(0.25, 3.0, 0.2);
  const Vector3 averageSpecificForce(0.0, 9.81, 0.0);

  observer(Rot3(), averageSpecificForce);

  EXPECT(!observer.b_hat.has_value());
  EXPECT(observer.n_hat.has_value());
  EXPECT((observer.n_hat->point3() - Vector3(0, -1, 0)).norm() < 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, GeodesicCorrectionMovesTowardGravityMeasurement) {
  TiltObserver observer(0.25, 30.0, 0.005, Unit3(Vector3(0, 0, -1)));
  const Unit3 measuredGravity(Vector3(0.2, 0.0, -0.98));
  const Vector3 f_b = -9.81 * measuredGravity.point3();

  const double dotBefore =
      observer.n_hat->point3().dot(measuredGravity.point3());

  observer(Vector3::Zero(), f_b);

  const double dotAfter =
      observer.n_hat->point3().dot(measuredGravity.point3());
  EXPECT(dotAfter > dotBefore);
}

/* ************************************************************************* */
TEST(TiltObserver, BiasUpdateMovesTowardStaticGyroBias) {
  const Vector3 trueGyroBias(0.02, 0.0, 0.0);
  const Vector3 f_b(0.0, 0.0, 9.81);
  TiltObserver observer(0.25, 3.0, 0.005,
                        gravityDirectionFromSpecificForce(f_b));

  for (size_t i = 0; i < 1000; ++i) {
    observer(trueGyroBias, f_b);
  }

  EXPECT(observer.b_hat->x() > 0.0);
  EXPECT(std::abs(observer.b_hat->x() - trueGyroBias.x()) < trueGyroBias.x());
}

/* ************************************************************************* */
TEST(TiltObserver, BiasParallelToGravityIsUnobservable) {
  const Vector3 f_b(9.81, 0.0, 0.0);
  TiltObserver observer(0.25, 3.0, 0.005,
                        gravityDirectionFromSpecificForce(f_b));

  for (size_t i = 0; i < 1000; ++i) {
    observer(Vector3(0.02, 0.0, 0.0), f_b);
  }

  EXPECT(observer.b_hat->norm() < 1e-12);
  EXPECT((observer.n_hat->point3() - Vector3(-1, 0, 0)).norm() < 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, ObservableMotionRecoversThreeAxisGyroBias) {
  const double dt = 0.005;
  const Vector3 trueGyroBias(0.02, -0.015, 0.01);
  Vector3 trueGravity = Vector3(-0.3, 0.4, -0.866).normalized();
  TiltObserver observer(0.25, 1.0, dt, Unit3(trueGravity));

  for (size_t i = 0; i < 16000; ++i) {
    const double time = static_cast<double>(i) * dt;
    const Vector3 angularVelocity(
        0.45 * std::sin(0.7 * time) + 0.15 * std::cos(0.17 * time),
        0.35 * std::cos(0.53 * time), 0.28 * std::sin(0.31 * time) + 0.12);
    trueGravity =
        (Rot3::Expmap(-angularVelocity * dt) * trueGravity).normalized();

    observer(angularVelocity + trueGyroBias, -9.81 * trueGravity);
  }

  EXPECT((observer.b_hat.value() - trueGyroBias).norm() < 1e-4);
  EXPECT((observer.n_hat->point3() - trueGravity).norm() < 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
