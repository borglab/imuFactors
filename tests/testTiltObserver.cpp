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

  const Vector2 rollPitch = rollPitchFromUpDirection(upDirection);

  EXPECT_DOUBLES_EQUAL(roll, rollPitch.x(), 1e-12);
  EXPECT_DOUBLES_EQUAL(pitch, rollPitch.y(), 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, BiasRetractionIsAdditive) {
  const TiltBias state{Unit3(Vector3(0, 0, -1)), Vector3(1.0, 2.0, 3.0)};

  const TiltBias retracted =
      state.retract(Vector2::Zero(), Vector3(0.1, -0.2, 0.3));

  EXPECT((retracted.b - Vector3(1.1, 1.8, 3.3)).norm() < 1e-12);
  EXPECT(retracted.n.point3().dot(state.n.point3()) > 1.0 - 1e-12);
}

/* ************************************************************************* */
TEST(TiltObserver, StationaryAlignedSampleLeavesStateUnchanged) {
  TiltObserver observer(1.0, 10.0, 0.005);

  observer(Vector3::Zero(), Vector3(0, 0, -9.81));

  EXPECT(observer.x_hat.b.norm() < 1e-12);
  EXPECT((observer.x_hat.n.point3() - Vector3(0, 0, -1)).norm() < 1e-12);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
