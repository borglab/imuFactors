/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved.
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>

#include <algorithm>
#include <cmath>

namespace gtsam {

/**
 * First-order tilt and gyroscope-bias observer driven by IMU measurements.
 */
struct TiltObserver {
  Unit3 n_hat;
  Vector3 b_hat;
  double kP, kI, dt;

  /**
   * Construct the observer with an initial gravity direction estimate.
   * @param tauP Proportional time constant in seconds
   * @param tauI Integral time constant in seconds
   * @param dt IMU sample period in seconds
   * @param initialGravityDirection Initial unit gravity direction in body frame
   * @param initialGyroBias Initial gyroscope bias in rad/s
   */
  TiltObserver(double tauP, double tauI, double dt,
               const Unit3& initialGravityDirection = Unit3(0, 0, -1),
               const Vector3& initialGyroBias = Vector3::Zero())
      : n_hat{initialGravityDirection},
        b_hat{initialGyroBias},
        kP(dt / tauP),
        kI(dt / (tauI * tauI)),
        dt(dt) {}

  /**
   * Predict the gravity direction using one gyroscope sample.
   * @param omega_b Measured body-frame angular velocity in rad/s
   */
  void predict(const Vector3& omega_b) {
    const Vector3 omega_hat = omega_b - b_hat;
    n_hat = Rot3::Expmap(-omega_hat * dt) * n_hat;
  }

  /**
   * Update the gravity direction and bias using one accelerometer sample.
   * @param f_b Measured body-frame specific force in m/s^2
   */
  void update(const Vector3& f_b) {
    const Vector3 y = -f_b.normalized();
    const auto e = n_hat.cross(y);
    n_hat = Rot3::Expmap(kP * e) * n_hat;
    b_hat += kI * e;
  }

  /**
   * Advance the observer by one IMU sample.
   * @param omega_b Measured body-frame angular velocity in rad/s
   * @param f_b Measured body-frame specific force in m/s^2
   */
  void operator()(const Vector3& omega_b, const Vector3& f_b) {
    predict(omega_b);
    update(f_b);
  }
};

/**
 * Convert a body-frame up direction into roll and pitch angles.
 *
 * For a body-to-navigation rotation R = Rz(yaw) Ry(pitch) Rx(roll), the
 * body-frame up direction is R^T [0, 0, 1]. Yaw is intentionally unobservable.
 *
 * @return Vector2(roll, pitch), both in radians
 */
inline Vector2 rollPitchFromUpDirection(const Unit3& upDirection) {
  const Vector3 n = upDirection.point3();
  return Vector2(std::atan2(n.y(), n.z()),
                 std::asin(std::clamp(-n.x(), -1.0, 1.0)));
}

/**
 * Convert a body-frame gravity direction into roll and pitch angles.
 * @return Vector2(roll, pitch), both in radians
 */
inline Vector2 rollPitchFromGravityDirection(const Unit3& gravityDirection) {
  return rollPitchFromUpDirection(Unit3(-gravityDirection.point3()));
}

/**
 * Convert a measured body-frame specific force into a gravity direction.
 * @return Unit gravity direction in the body frame
 */
inline Unit3 gravityDirectionFromSpecificForce(const Vector3& f_b) {
  return Unit3((-f_b).normalized());
}

}  // namespace gtsam
