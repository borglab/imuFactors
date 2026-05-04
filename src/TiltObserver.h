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
 * Tilt direction and gyroscope bias state for the nonlinear observer.
 */
struct TiltBias {
  Unit3 n;
  Vector3 b;

  /// Retract a tilt perturbation and additive gyroscope-bias increment.
  TiltBias retract(const Vector2& xi, const Vector3& db) const {
    return {n.retract(xi), b + db};
  }
};

/**
 * First-order tilt and gyroscope-bias observer driven by IMU measurements.
 */
struct TiltObserver {
  TiltBias x_hat;
  double kP;
  double kI;
  double dt;

  /**
   * Construct the observer from proportional/integral time constants.
   * @param tauP Proportional time constant in seconds
   * @param tauI Integral time constant in seconds
   * @param dt IMU sample period in seconds
   */
  TiltObserver(double tauP, double tauI, double dt)
      : x_hat{Unit3(Vector3(0, 0, -1)), Vector3::Zero()},
        kP(dt / tauP),
        kI(dt / (tauI * tauI)),
        dt(dt) {}

  /**
   * Advance the observer by one IMU sample.
   * @param omega_b Measured body-frame angular velocity in rad/s
   * @param f_b Measured body-frame specific force in m/s^2
   */
  void operator()(const Vector3& omega_b, const Vector3& f_b) {
    const auto n1 = Rot3::Expmap((x_hat.b - omega_b) * dt) *
                    x_hat.n.point3();
    const auto e = n1.cross(f_b.normalized());
    const Unit3 predictedDirection(n1);
    const Unit3 correctedDirection((n1 + kP * e).normalized());
    x_hat = TiltBias{predictedDirection, x_hat.b}.retract(
        predictedDirection.localCoordinates(correctedDirection), -kI * e);
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

}  // namespace gtsam
