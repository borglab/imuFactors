// Native benchmark: no Python calls and no Picard history in timed regions.
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/basis/PsyChMagPI.h>

#include <Eigen/QR>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "FastRodFIter.h"

using namespace gtsam;
using imuFactors::FastRodFIter;

namespace {
struct Scenario {
  std::string name;
  size_t degree;
  double duration;
  Vector times;  // Normalized rate times, or interval edges for increments.
  Matrix measurements;
  bool increments = false;
  Rot3 reference;
};

Matrix designMatrix(const Scenario& scenario) {
  if (!scenario.increments)
    return FastRodFIter::basis(scenario.times, scenario.degree);
  const Matrix primitives =
      FastRodFIter::basis(scenario.times, scenario.degree + 1) *
      FastRodFIter::integrationMatrix(scenario.degree, scenario.degree + 1,
                                      scenario.duration)
          .transpose();
  const int count = primitives.rows() - 1;
  return primitives.bottomRows(count) - primitives.topRows(count);
}

Scenario coning(const std::string& name, int count, size_t degree,
                double duration, bool increments,
                double frequency = .74 * std::acos(-1.0)) {
  const double alpha = 10 * std::acos(-1.0) / 180;
  Scenario scenario{name,
                    degree,
                    duration,
                    Vector::LinSpaced(count + (increments ? 1 : 0), -1, 1),
                    Matrix(3, count),
                    increments,
                    Rot3()};
  for (int i = 0; i < count; ++i) {
    if (increments) {
      const double a = i * duration / count, b = (i + 1) * duration / count;
      scenario.measurements.col(i)
          << -frequency * (1 - std::cos(alpha)) * (b - a),
          std::sin(alpha) * (std::cos(frequency * b) - std::cos(frequency * a)),
          std::sin(alpha) * (std::sin(frequency * b) - std::sin(frequency * a));
    } else {
      const double t = i * duration / (count - 1);
      scenario.measurements.col(i) << -frequency * (1 - std::cos(alpha)),
          -frequency * std::sin(alpha) * std::sin(frequency * t),
          frequency * std::sin(alpha) * std::cos(frequency * t);
    }
  }
  const Rot3 initial =
      Rot3::Quaternion(std::cos(alpha / 2), 0, std::sin(alpha / 2), 0);
  const Rot3 final =
      Rot3::Quaternion(std::cos(alpha / 2), 0,
                       std::sin(alpha / 2) * std::cos(frequency * duration),
                       std::sin(alpha / 2) * std::sin(frequency * duration));
  scenario.reference = initial.between(final);
  return scenario;
}

// Rows: name,noise_fraction,sample_index,time_s,wx,wy,wz,reference quaternion
// wxyz.
std::vector<Scenario> loadEuroc(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open EuRoC fixture " + path);
  std::vector<Scenario> scenarios;
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    std::string dataset;
    double noise, time, x, y, z, qw, qx, qy, qz;
    int index;
    if (!(row >> dataset >> noise >> index >> time >> x >> y >> z >> qw >> qx >>
          qy >> qz))
      throw std::runtime_error("Malformed EuRoC fixture row");
    if (index == 0) {
      std::ostringstream name;
      name << dataset << "_noise" << noise;
      scenarios.push_back({name.str(), 12, .2, Vector::LinSpaced(41, -1, 1),
                           Matrix::Constant(3, 41, std::nan("")), false,
                           Rot3::Quaternion(qw, qx, qy, qz)});
    }
    if (scenarios.empty() || index < 0 || index >= 41 ||
        std::abs(time - .005 * index) > 1e-12)
      throw std::runtime_error("Expected 41-sample 200 Hz EuRoC fixture");
    scenarios.back().measurements.col(index) = Vector3(x, y, z);
  }
  for (const auto& scenario : scenarios)
    if (!scenario.measurements.allFinite())
      throw std::runtime_error("Incomplete EuRoC fixture");
  return scenarios;
}

volatile double checksum = 0;

struct Timing {
  double median, p10, p90;
};

Timing timeCalls(const std::function<Rot3()>& call, size_t repeats) {
  for (int i = 0; i < 30; ++i) checksum += call().matrix()(0, 0);
  std::vector<double> times;
  for (int batch = 0; batch < 15; ++batch) {
    const auto start = std::chrono::steady_clock::now();
    double sum = 0;
    for (size_t i = 0; i < repeats; ++i) sum += call().matrix()(0, 0);
    const auto stop = std::chrono::steady_clock::now();
    checksum += sum;
    times.push_back(
        std::chrono::duration<double, std::micro>(stop - start).count() /
        repeats);
  }
  std::sort(times.begin(), times.end());
  return {times[7], times[1], times[13]};
}

void benchmark(const Scenario& scenario, size_t repeats) {
  const Matrix design = designMatrix(scenario);
  const Matrix fit = design.colPivHouseholderQr().solve(
      Matrix::Identity(design.rows(), design.rows()));
  const Matrix coefficients = scenario.measurements * fit.transpose();
  const Matrix evaluation =
      FastRodFIter::basis(Chebyshev2::Points(scenario.degree + 1),
                          scenario.degree)
          .transpose();
  const Matrix nodalFit = fit.transpose() * evaluation;
  const Matrix nodal = coefficients * evaluation;
  for (bool fixed : {true, false}) {
    const double tolerance = fixed ? 0.0 : 1e-12;
    const size_t maxIterations = fixed ? 7 : 50;
    const FastRodFIter rod(scenario.degree, scenario.degree + 1,
                           scenario.duration, tolerance, maxIterations);
    // Same zero initialization for fixed-seven work comparison; production RK2
    // initialization for the converged PS-CPI comparison.
    const auto initialization =
        fixed ? PseudoSpectralPiInit::ZERO : PseudoSpectralPiInit::RK2;
    const PsyChMagPI<Rot3> psychApprox(
        scenario.degree + 1, 0, scenario.duration,
        PseudoSpectralPiParams(initialization, tolerance, maxIterations, true));
    const PsyChMagPI<Rot3> psychExact(
        scenario.degree + 1, 0, scenario.duration,
        PseudoSpectralPiParams(initialization, tolerance, maxIterations,
                               false));
    const auto rodResult = rod.integrate(coefficients);
    const auto approxHistory = psychApprox.integrateWithHistory(nodal);
    const auto exactHistory = psychExact.integrateWithHistory(nodal);
    if (!fixed && (!rodResult.converged || !approxHistory.converged ||
                   !exactHistory.converged))
      throw std::runtime_error("Convergence failure: " + scenario.name);
    for (const std::string mode : {"prepared", "cached_fit", "fresh_setup"}) {
      // Randomize method order per scenario/mode to reduce systematic drift.
      std::vector<int> methods{0, 1, 2};
      std::mt19937 rng(
          static_cast<unsigned>(scenario.name.size() + mode.size() + fixed));
      std::shuffle(methods.begin(), methods.end(), rng);
      for (int method : methods) {
        const bool approximate = method == 1;
        const auto& psych = approximate ? psychApprox : psychExact;
        const auto call = [&]() -> Rot3 {
          if (mode == "prepared")
            return method == 0 ? rod.integrate(coefficients).rotation()
                               : psych.integratePose(Rot3(), nodal);
          if (mode == "cached_fit")
            return method == 0
                       ? rod.integrate(scenario.measurements * fit.transpose())
                             .rotation()
                       : psych.integratePose(Rot3(),
                                             scenario.measurements * nodalFit);
          // Include factorization, transforms, construction and integration.
          const Matrix freshDesign = designMatrix(scenario);
          const Matrix freshCoefficients =
              freshDesign.colPivHouseholderQr()
                  .solve(scenario.measurements.transpose())
                  .transpose();
          if (method == 0)
            return FastRodFIter(scenario.degree, scenario.degree + 1,
                                scenario.duration, tolerance, maxIterations)
                .integrate(freshCoefficients)
                .rotation();
          const Matrix freshNodes =
              freshCoefficients *
              FastRodFIter::basis(Chebyshev2::Points(scenario.degree + 1),
                                  scenario.degree)
                  .transpose();
          return PsyChMagPI<Rot3>(
                     scenario.degree + 1, 0, scenario.duration,
                     PseudoSpectralPiParams(initialization, tolerance,
                                            maxIterations, approximate))
              .integratePose(Rot3(), freshNodes);
        };
        const Timing timing = timeCalls(call, repeats);
        const Rot3 rotation = call();
        const auto& history = approximate ? approxHistory : exactHistory;
        const size_t iterations =
            method == 0 ? rodResult.iterations : history.errorNorm.size();
        const double residual =
            method == 0 ? rodResult.residual : history.errorNorm.back();
        const double error =
            Rot3::Logmap(scenario.reference.between(rotation)).norm();
        const char* name =
            method == 0 ? "FastRodFIter"
                        : (approximate ? "PS-CPI_approx" : "PS-CPI_exact");
        std::cout << scenario.name << ',' << scenario.measurements.cols() << ','
                  << scenario.degree << ',' << scenario.duration << ','
                  << (scenario.increments ? "increments" : "rates") << ','
                  << (fixed ? "fixed7_zero" : "converged") << ',' << mode << ','
                  << name << ',' << timing.median << ',' << timing.p10 << ','
                  << timing.p90 << ',' << iterations << ',' << residual << ','
                  << error << '\n';
      }
    }
  }
}
}  // namespace

int main(int argc, char** argv) {
#ifndef NDEBUG
  std::cerr << "Use a Release build for this benchmark.\n";
  return 1;
#endif
  try {
    const size_t repeats = argc > 1 ? std::stoul(argv[1]) : 100;
    if (repeats == 0) throw std::invalid_argument("repeats must be positive");
    std::vector<Scenario> scenarios{
        coning("Wu_coning", 8, 7, .08, true),
        coning("coning_200Hz", 41, 12, .2, false),
        coning("coning_fast_200Hz", 41, 12, .2, false, 10 * std::acos(-1.0))};
    if (argc > 2) {
      const auto euroc = loadEuroc(argv[2]);
      scenarios.insert(scenarios.end(), euroc.begin(), euroc.end());
    }
    std::cout
        << std::setprecision(12)
        << "scenario,samples,gyro_degree,duration_s,input,stopping,mode,method,"
           "median_us,p10_us,p90_us,iterations,residual,error_rad\n";
    for (const auto& scenario : scenarios) benchmark(scenario, repeats);
    std::cerr << "checksum=" << checksum << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
