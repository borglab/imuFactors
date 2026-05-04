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

  EXPECT(observer.b_hat.norm() < 1e-12);
  EXPECT((observer.n_hat.point3() - Vector3(0, 0, -1)).norm() < 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, InitialGravityDirectionStartsAlignedWithSpecificForce) {
  const Vector3 specificForce(9.81, 0.0, 0.0);
  TiltObserver observer(0.25, 3.0, 0.005,
                        gravityDirectionFromSpecificForce(specificForce));

  EXPECT((observer.n_hat.point3() - Vector3(-1, 0, 0)).norm() < 1e-12);

  observer(Vector3::Zero(), specificForce);

  EXPECT((observer.n_hat.point3() - Vector3(-1, 0, 0)).norm() < 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, GeodesicCorrectionMovesTowardGravityMeasurement) {
  TiltObserver observer(0.25, 30.0, 0.005);
  const Unit3 measuredGravity(Vector3(0.2, 0.0, -0.98));
  const Vector3 specificForce = -9.81 * measuredGravity.point3();

  const double dotBefore =
      observer.n_hat.point3().dot(measuredGravity.point3());

  observer(Vector3::Zero(), specificForce);

  const double dotAfter = observer.n_hat.point3().dot(measuredGravity.point3());
  EXPECT(dotAfter > dotBefore);
}

/* ************************************************************************* */
TEST(TiltObserver, BiasUpdateMovesTowardStaticGyroBias) {
  TiltObserver observer(0.25, 3.0, 0.005);
  const Vector3 trueGyroBias(0.02, 0.0, 0.0);
  const Vector3 specificForce(0.0, 0.0, 9.81);

  for (size_t i = 0; i < 1000; ++i) {
    observer(trueGyroBias, specificForce);
  }

  EXPECT(observer.b_hat.x() > 0.0);
  EXPECT(std::abs(observer.b_hat.x() - trueGyroBias.x()) < trueGyroBias.x());
}

/* ************************************************************************* */
TEST(TiltObserver, BiasParallelToGravityIsUnobservable) {
  const Vector3 specificForce(9.81, 0.0, 0.0);
  TiltObserver observer(0.25, 3.0, 0.005,
                        gravityDirectionFromSpecificForce(specificForce));

  for (size_t i = 0; i < 1000; ++i) {
    observer(Vector3(0.02, 0.0, 0.0), specificForce);
  }

  EXPECT(observer.b_hat.norm() < 1e-12);
  EXPECT((observer.n_hat.point3() - Vector3(-1, 0, 0)).norm() < 1e-12);
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

  EXPECT((observer.b_hat - trueGyroBias).norm() < 1e-4);
  EXPECT((observer.n_hat.point3() - trueGravity).norm() < 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
