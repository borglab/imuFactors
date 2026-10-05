#include <CppUnitLite/TestHarness.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/basis/PsyChMagPI.h>

#include <Eigen/QR>
#include <limits>

#include "FastRodFIter.h"

using namespace gtsam;
using imuFactors::FastRodFIter;

TEST(FastRodFIter, IntegralAndHighDegreeConstant) {
  const Matrix integral = FastRodFIter::integrationMatrix(4, 5, 2.0);
  EXPECT_DOUBLES_EQUAL(1, integral(0, 0), 1e-15);
  EXPECT_DOUBLES_EQUAL(1, integral(0, 1), 1e-15);
  EXPECT_DOUBLES_EQUAL(-0.25, integral(1, 0), 1e-15);
  EXPECT_DOUBLES_EQUAL(0.25, integral(1, 2), 1e-15);
  EXPECT_DOUBLES_EQUAL(-1.0 / 3, integral(2, 0), 1e-15);
  EXPECT_DOUBLES_EQUAL(-0.5, integral(2, 1), 1e-15);
  EXPECT_DOUBLES_EQUAL(1.0 / 6, integral(2, 3), 1e-15);
  const Matrix truncated = FastRodFIter::integrationMatrix(4, 2, 2.0);
  EXPECT_DOUBLES_EQUAL(-1.0 / 15, truncated(4, 0), 1e-15);
  const Vector minusOne = Vector::Constant(1, -1.0);
  EXPECT((integral * FastRodFIter::basis(minusOne, 5).transpose()).norm() <
         1e-15);
}

TEST(FastRodFIter, ProductMatchesIndependentPointwisePolynomial) {
  const size_t gyroDegree = 2, attitudeDegree = 4;
  Matrix gyro(3, 3), state(3, 5);
  for (int i = 0; i < gyro.size(); ++i)
    gyro.data()[i] = 0.3 * std::sin(i + 1.0);
  for (int i = 0; i < state.size(); ++i)
    state.data()[i] = 0.04 * std::cos(i + 1.0);
  const size_t rhsDegree = 2 * attitudeDegree + gyroDegree;
  const Vector times = Chebyshev2::Points(rhsDegree + 1);
  const Matrix sampledState =
      state * FastRodFIter::basis(times, attitudeDegree).transpose();
  const Matrix sampledGyro =
      gyro * FastRodFIter::basis(times, gyroDegree).transpose();
  Matrix rhs(3, times.size());
  for (int j = 0; j < times.size(); ++j) {
    const Vector3 g = sampledState.col(j), omega = sampledGyro.col(j);
    rhs.col(j) = omega + 0.5 * g.cross(omega) + 0.25 * g * g.dot(omega);
  }
  const Matrix coefficients = FastRodFIter::basis(times, rhsDegree)
                                  .colPivHouseholderQr()
                                  .solve(rhs.transpose())
                                  .transpose();
  const Matrix expected = coefficients * FastRodFIter::integrationMatrix(
                                             rhsDegree, attitudeDegree, .2);
  const Matrix actual =
      FastRodFIter(gyroDegree, attitudeDegree, .2).iterate(state, gyro);
  EXPECT((actual - expected).norm() < 1e-14);
}

TEST(FastRodFIter, ZeroConstantAndClenshaw) {
  const FastRodFIter integrator(0, 10, .2, 1e-14);
  const auto zero = integrator.integrate(Matrix::Zero(3, 1));
  EXPECT(zero.converged);
  EXPECT(zero.iterations == 1);
  const Vector3 omega(.3, -.2, .5);
  const auto result = integrator.integrate(omega);
  EXPECT(result.converged);
  EXPECT(
      Rot3::Logmap(Rot3::Expmap(.2 * omega).between(result.rotation())).norm() <
      1e-13);
  for (double tau : {-1.0, -.4, .0, .7, 1.0}) {
    const Vector3 expected =
        2 * std::tan(.05 * (tau + 1) * omega.norm()) * omega.normalized();
    EXPECT((result.evaluate(tau) - expected).norm() < 1e-13);
  }
  const auto fixed = FastRodFIter(0, 10, .2, 0.0, 7).integrate(omega);
  EXPECT(fixed.iterations == 7);
  EXPECT(!fixed.converged);
}

TEST(FastRodFIter, WuEightIncrementConing) {
  const double duration = .08, alpha = 10 * std::acos(-1.0) / 180;
  const double frequency = .74 * std::acos(-1.0);
  const Vector edges = Vector::LinSpaced(9, -1, 1);
  const Matrix primitives =
      FastRodFIter::basis(edges, 8) *
      FastRodFIter::integrationMatrix(7, 8, duration).transpose();
  const Matrix design = primitives.bottomRows(8) - primitives.topRows(8);
  Matrix increments(3, 8);
  for (int i = 0; i < 8; ++i) {
    const double a = i * .01, b = a + .01;
    increments.col(i) << -frequency * (1 - std::cos(alpha)) * .01,
        std::sin(alpha) * (std::cos(frequency * b) - std::cos(frequency * a)),
        std::sin(alpha) * (std::sin(frequency * b) - std::sin(frequency * a));
  }
  const Matrix gyro =
      design.colPivHouseholderQr().solve(increments.transpose()).transpose();
  const auto result = FastRodFIter(7, 8, duration, 0.0, 7).integrate(gyro);
  const auto orientation = [&](double t) {
    return Rot3::Quaternion(std::cos(alpha / 2), 0,
                            std::sin(alpha / 2) * std::cos(frequency * t),
                            std::sin(alpha / 2) * std::sin(frequency * t));
  };
  const Rot3 reference = orientation(0).between(orientation(duration));
  EXPECT(Rot3::Logmap(reference.between(result.rotation())).norm() < 1e-13);
  const Matrix nodal =
      gyro * FastRodFIter::basis(Chebyshev2::Points(8), 7).transpose();
  for (bool approximate : {false, true}) {
    const PseudoSpectralPiParams params(PseudoSpectralPiInit::RK2, 1e-13, 50,
                                        approximate);
    const PsyChMagPI<Rot3> psych(8, 0, duration, params);
    EXPECT(Rot3::Logmap(reference.between(psych.integratePose(Rot3(), nodal)))
               .norm() < 1e-10);
  }
}

TEST(FastRodFIter, InvalidInput) {
  CHECK_EXCEPTION(FastRodFIter(2, 2, .2), std::invalid_argument);
  CHECK_EXCEPTION(FastRodFIter(2, 3, 0), std::invalid_argument);
  CHECK_EXCEPTION(FastRodFIter(2, 3, .2, -1), std::invalid_argument);
  CHECK_EXCEPTION(FastRodFIter(2, 3, .2, 1e-12, 0), std::invalid_argument);
  CHECK_EXCEPTION(FastRodFIter(2, 3, .2).integrate(Matrix::Zero(3, 4)),
                  std::invalid_argument);
  Matrix invalid = Matrix::Zero(3, 3);
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  CHECK_EXCEPTION(FastRodFIter(2, 3, .2).integrate(invalid),
                  std::invalid_argument);
}

TEST(FastRodFIter, NoncommutingPolynomialMatchesIndependentRK4) {
  Matrix gyro(3, 4);
  gyro << .7, .3, -.1, .02, -.4, .1, .2, -.03, .5, -.2, .05, .01;
  const double duration = .2;
  const auto result = FastRodFIter(3, 16, duration, 1e-14).integrate(gyro);
  EXPECT(result.converged);
  const auto rhs = [&](double t, const Matrix3& rotation) -> Matrix3 {
    const double tau = 2 * t / duration - 1;
    const Vector3 omega = gyro.col(0) + tau * gyro.col(1) +
                          (2 * tau * tau - 1) * gyro.col(2) +
                          (4 * tau * tau * tau - 3 * tau) * gyro.col(3);
    Matrix3 skew;
    skew << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(), -omega.y(),
        omega.x(), 0;
    return rotation * skew;
  };
  Matrix3 rotation = Matrix3::Identity();
  const double step = duration / 2000;
  for (int i = 0; i < 2000; ++i) {
    const double t = i * step;
    const Matrix3 k1 = rhs(t, rotation);
    const Matrix3 k2 = rhs(t + step / 2, rotation + step / 2 * k1);
    const Matrix3 k3 = rhs(t + step / 2, rotation + step / 2 * k2);
    const Matrix3 k4 = rhs(t + step, rotation + step * k3);
    rotation += step / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
  }
  EXPECT(Rot3::Logmap(Rot3::ClosestTo(rotation).between(result.rotation()))
             .norm() < 1e-12);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
