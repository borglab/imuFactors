#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Rot3.h>

namespace imuFactors {

/** Coefficient-space Rodrigues-vector Picard iteration (Wu et al., TAES 2019).
 * Uses g = 2 tan(theta/2) axis, not the unscaled Gibbs vector. Coefficients
 * multiply first-kind Chebyshev polynomials T_k on the normalized interval.
 * Nonlinear products are formed without intermediate truncation; the integrated
 * polynomial is truncated once per iteration, as in equations (16)-(17).
 */
class FastRodFIter {
 public:
  struct Result {
    gtsam::Matrix coefficients;
    size_t iterations = 0;
    double residual = 0.0;
    bool converged = false;

    /** Evaluate the Rodrigues polynomial at normalized time tau in [-1,1]. */
    gtsam::Vector3 evaluate(double tau) const;
    /** Convert the final scaled Rodrigues vector to an SO(3) increment. */
    gtsam::Rot3 rotation() const;
  };

  /** Precompute integration for degree-n gyros and degree-nT attitude.
   * Requires duration > 0 and nT >= n+1. The initial polynomial is zero.
   * tolerance=0 requests exactly maxIterations updates (no early stopping).
   * Otherwise the stopping norm is the L1 sum of coefficient-vector norms,
   * which bounds the maximum change in the Rodrigues polynomial on [-1,1].
   */
  FastRodFIter(size_t gyroDegree, size_t attitudeDegree, double duration,
               double tolerance = 1e-12, size_t maxIterations = 50);

  /** Integrate a 3 x (gyroDegree+1) matrix of angular-rate coefficients. */
  Result integrate(const gtsam::Matrix& gyroCoefficients) const;

  /** One full Picard update, exposed for independent algebraic verification. */
  gtsam::Matrix iterate(const gtsam::Matrix& coefficients,
                        const gtsam::Matrix& gyroCoefficients) const;

  /** Matrix B with B(i,k)=T_k(tau_i), for fitting or evaluating coefficients.
   */
  static gtsam::Matrix basis(const gtsam::Vector& normalizedTimes,
                             size_t degree);

  /** Matrix mapping RHS coefficients to integrated, truncated coefficients.
   * Includes the lower-limit constant from every RHS coefficient, even those
   * whose nonconstant antiderivative terms lie above the retained degree.
   */
  static gtsam::Matrix integrationMatrix(size_t rhsDegree, size_t outputDegree,
                                         double duration);

 private:
  size_t gyroDegree_, attitudeDegree_, maxIterations_;
  double tolerance_;
  gtsam::Matrix integration_;
};

}  // namespace imuFactors
