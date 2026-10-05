#include "FastRodFIter.h"

#include <cmath>
#include <stdexcept>

namespace imuFactors {
using gtsam::Matrix;
using gtsam::Vector3;

FastRodFIter::FastRodFIter(size_t gyroDegree, size_t attitudeDegree,
                           double duration, double tolerance,
                           size_t maxIterations)
    : gyroDegree_(gyroDegree),
      attitudeDegree_(attitudeDegree),
      maxIterations_(maxIterations),
      tolerance_(tolerance) {
  if (attitudeDegree < gyroDegree + 1 || !std::isfinite(duration) ||
      duration <= 0 || !std::isfinite(tolerance) || tolerance < 0 ||
      maxIterations == 0) {
    throw std::invalid_argument(
        "Invalid FastRodFIter degrees, duration, or stopping settings");
  }
  integration_ = integrationMatrix(2 * attitudeDegree + gyroDegree,
                                   attitudeDegree, duration);
}

Matrix FastRodFIter::basis(const gtsam::Vector& times, size_t degree) {
  Matrix values(times.size(), degree + 1);
  values.col(0).setOnes();
  if (degree > 0) values.col(1) = times;
  for (size_t k = 2; k <= degree; ++k)
    values.col(k) =
        2 * times.cwiseProduct(values.col(k - 1)) - values.col(k - 2);
  return values;
}

Matrix FastRodFIter::integrationMatrix(size_t rhsDegree, size_t outputDegree,
                                       double duration) {
  if (!std::isfinite(duration) || duration <= 0)
    throw std::invalid_argument(
        "Integration duration must be finite and positive");
  Matrix integral = Matrix::Zero(rhsDegree + 1, outputDegree + 1);
  integral(0, 0) = 1;
  if (outputDegree >= 1) integral(0, 1) = 1;
  if (rhsDegree >= 1) {
    integral(1, 0) = -0.25;
    if (outputDegree >= 2) integral(1, 2) = 0.25;
  }
  for (size_t k = 2; k <= rhsDegree; ++k) {
    const double index = static_cast<double>(k);
    integral(k, 0) = (k % 2 ? 1.0 : -1.0) / (index * index - 1.0);
    if (k + 1 <= outputDegree) integral(k, k + 1) = 1.0 / (2 * (index + 1));
    if (k - 1 <= outputDegree) integral(k, k - 1) = -1.0 / (2 * (index - 1));
  }
  return (duration / 2) * integral;
}

Matrix FastRodFIter::iterate(const Matrix& state, const Matrix& gyro) const {
  if (state.rows() != 3 ||
      state.cols() != static_cast<int>(attitudeDegree_ + 1) ||
      gyro.rows() != 3 || gyro.cols() != static_cast<int>(gyroDegree_ + 1) ||
      !state.allFinite() || !gyro.allFinite())
    throw std::invalid_argument("Invalid FastRodFIter coefficient matrix");

  Matrix rhs = Matrix::Zero(3, 2 * attitudeDegree_ + gyroDegree_ + 1);
  rhs.leftCols(gyro.cols()) = gyro;
  gtsam::Vector dot = gtsam::Vector::Zero(attitudeDegree_ + gyroDegree_ + 1);
  // T_i T_j = (T_{i+j} + T_{|i-j|})/2, including coincident indices.
  for (int i = 0; i < state.cols(); ++i) {
    const Vector3 value = state.col(i);
    for (int j = 0; j < gyro.cols(); ++j) {
      const Vector3 omega = gyro.col(j);
      const Vector3 cross = 0.25 * value.cross(omega);
      rhs.col(i + j) += cross;
      rhs.col(std::abs(i - j)) += cross;
      const double product = 0.5 * value.dot(omega);
      dot(i + j) += product;
      dot(std::abs(i - j)) += product;
    }
  }
  // Associate g(g^T omega) to avoid the literal cubic triple sum in (17).
  // Neither product is truncated, so this is the same coefficient algebra.
  for (int i = 0; i < state.cols(); ++i) {
    const Vector3 value = state.col(i);
    for (int j = 0; j < dot.size(); ++j) {
      const Vector3 term = (0.125 * dot(j)) * value;
      rhs.col(i + j) += term;
      rhs.col(std::abs(i - j)) += term;
    }
  }
  return rhs * integration_;
}

FastRodFIter::Result FastRodFIter::integrate(const Matrix& gyro) const {
  Result result;
  result.coefficients = Matrix::Zero(3, attitudeDegree_ + 1);
  for (size_t iteration = 0; iteration < maxIterations_; ++iteration) {
    Matrix next = iterate(result.coefficients, gyro);
    result.residual = (next - result.coefficients).colwise().norm().sum();
    result.coefficients = std::move(next);
    result.iterations = iteration + 1;
    if (!result.coefficients.allFinite())
      throw std::runtime_error(
          "FastRodFIter diverged to non-finite coefficients");
    if (tolerance_ > 0 && result.residual < tolerance_) {
      result.converged = true;
      break;
    }
  }
  return result;
}

Vector3 FastRodFIter::Result::evaluate(double tau) const {
  if (!std::isfinite(tau) || tau < -1 || tau > 1 || coefficients.cols() == 0)
    throw std::invalid_argument(
        "Evaluation requires tau in [-1,1] and a result");
  // Clenshaw evaluation without an evaluation-matrix allocation.
  Vector3 next = Vector3::Zero(), afterNext = Vector3::Zero();
  for (int k = coefficients.cols() - 1; k >= 1; --k) {
    const Vector3 current = coefficients.col(k) + 2 * tau * next - afterNext;
    afterNext = next;
    next = current;
  }
  return coefficients.col(0) + tau * next - afterNext;
}

gtsam::Rot3 FastRodFIter::Result::rotation() const {
  const Vector3 vector = evaluate(1.0);
  const double denominator = std::sqrt(4 + vector.squaredNorm());
  return gtsam::Rot3::Quaternion(2 / denominator, vector.x() / denominator,
                                 vector.y() / denominator,
                                 vector.z() / denominator);
}
}  // namespace imuFactors
