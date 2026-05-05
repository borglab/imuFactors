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
#include <optional>

namespace gtsam {

/**
 * First-order tilt and gyroscope-bias observer driven by IMU measurements.
 */
struct TiltObserver {
  std::optional<Unit3> n_hat;
  std::optional<Vector3> b_hat;
  double kP, kI, dt, g, tol;

  /**
   * Construct the observer with lazy state initialization.
   * @param tauP Proportional time constant in seconds
   * @param tauI Integral time constant in seconds
   * @param dt IMU sample period in seconds
   * @param g Nominal gravity magnitude in m/s^2
   * @param tol Accepted deviation from gravity
   * magnitude in m/s^2
   */
  TiltObserver(double tauP, double tauI, double dt, double g = 9.81,
               double tol = 0.3 * 9.81)
      : kP(dt / tauP), kI(dt / (tauI * tauI)), dt(dt), g(g), tol(tol) {}

  /**
   * Construct the observer with an initial state estimate.
   * @param tauP Proportional time constant in seconds
   * @param tauI Integral time constant in seconds
   * @param dt IMU sample period in seconds
   * @param initialGravityDirection Initial unit gravity direction in body frame
   * @param initialGyroBias Initial gyroscope bias in rad/s
   * @param g Nominal gravity magnitude in m/s^2
   * @param tol Accepted deviation from gravity
   * magnitude in m/s^2
   */
  TiltObserver(double tauP, double tauI, double dt,
               const Unit3& initialGravityDirection,
               const Vector3& initialGyroBias = Vector3::Zero(),
               double g = 9.81, double tol = 0.3 * 9.81)
      : n_hat{initialGravityDirection},
        b_hat{initialGyroBias},
        kP(dt / tauP),
        kI(dt / (tauI * tauI)),
        dt(dt),
        g(g),
        tol(tol) {}

  /**
   * Predict the gravity direction using one gyroscope sample.
   * @param omega_b Measured body-frame angular velocity in rad/s
   */
  void predict(const Vector3& omega_b) {
    if (!b_hat) {
      b_hat = omega_b;
      return;
    }
    if (n_hat) {
      const Vector3 omega_hat = omega_b - b_hat.value();
      n_hat = Rot3::Expmap(-omega_hat * dt) * n_hat.value();
    }
  }

  /**
   * Predict the gravity direction using a bias-corrected relative rotation.
   *
   * The rotation should be the PIM-style body rotation from the current frame to
   * the next frame, equivalent to integrating `omega_b - b_hat` over the
   * interval. The body-frame gravity direction transforms by the inverse of
   * this relative rotation.
   *
   * @param deltaRij Bias-corrected body rotation from frame i to frame j
   */
  void predict(const Rot3& deltaRij) {
    if (n_hat) {
      n_hat = deltaRij.inverse() * n_hat.value();
    }
  }

  /**
   * Update the gravity direction and bias using one accelerometer sample.
   * @param f_b Measured body-frame specific force in m/s^2
   */
  void update(const Vector3& f_b) {
    const double magnitude = f_b.norm();
    if (std::abs(magnitude - g) > tol) return;
    const Vector3 y = -f_b / magnitude;
    if (!n_hat) {
      n_hat = Unit3(y);
      return;
    }
    const auto e = n_hat->cross(y);
    n_hat = Rot3::Expmap(kP * e) * n_hat.value();
    if (b_hat) {
      b_hat = b_hat.value() + kI * e;
    }
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

  /**
   * Advance the observer by one preintegrated window.
   *
   * @param deltaRij Bias-corrected body rotation from frame i to frame j
   * @param averageSpecificForce Window-averaged body-frame specific force
   */
  void operator()(const Rot3& deltaRij, const Vector3& averageSpecificForce) {
    predict(deltaRij);
    update(averageSpecificForce);
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
