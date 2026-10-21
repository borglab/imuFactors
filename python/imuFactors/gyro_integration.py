"""SO(3) gyro integration helpers used by report generators and tests."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Sequence

import gtsam
import numpy as np

from imuFactors import spectral


@dataclass(frozen=True)
class PsyChMagPIResult:
    """Final rotation and convergence metadata from PsyChMagPI."""

    rotation: gtsam.Rot3
    N: int
    lambda1: float
    use_approximate_integration_matrix: bool
    cgl_times: np.ndarray
    fit_relative_error: float
    error_norm: np.ndarray
    iterations: int
    final_residual: float
    converged: bool
    runtime_s: float


def rotation_from_quaternion(quaternion_wxyz: Sequence[float] | np.ndarray) -> gtsam.Rot3:
    """Return a ``Rot3`` from a EuRoC/GTSAM ``[w, x, y, z]`` quaternion."""
    quaternion = np.asarray(quaternion_wxyz, dtype=float).reshape(4)
    return gtsam.Rot3.Quaternion(
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
    )


def rotation_error_degrees(reference: gtsam.Rot3, estimate: gtsam.Rot3) -> float:
    """Return geodesic SO(3) error angle in degrees."""
    tangent_error = np.asarray(gtsam.Rot3.Logmap(reference.between(estimate)), dtype=float)
    return float(np.degrees(np.linalg.norm(tangent_error)))


def fit_vector_samples_to_chebyshev_nodes(
    sample_times: Sequence[float] | np.ndarray,
    values: Sequence[Sequence[float]] | np.ndarray,
    N: int,
    *,
    lambda1: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Fit vector samples to CGL nodal values.

    If ``lambda1`` is positive, use the Chebyshev2 first-derivative Sobolev
    penalty from :mod:`imuFactors.spectral`. Returns the CGL node times, fitted
    values at those nodes, fitted values at the original sample times, and the
    relative Frobenius fit error.
    """
    times, samples = _validated_time_series(sample_times, values)
    node_count = int(N)
    if node_count < 2:
        raise ValueError("N must be at least 2")
    if node_count > times.size:
        raise ValueError("N must not exceed the number of samples")
    interval = (float(times[0]), float(times[-1]))
    plan = spectral.basis_plan_from_coordinates(
        node_count,
        times,
        basis="chebyshev2",
        interval=interval,
        window_seconds=interval[1] - interval[0],
        lambda1=float(lambda1),
    )
    nodal_values = plan.fit(samples)
    fitted_samples = plan.reconstruct(nodal_values)
    denominator = max(float(np.linalg.norm(samples, "fro")), 1e-15)
    fit_relative_error = float(np.linalg.norm(fitted_samples - samples, "fro") / denominator)
    cgl_times = spectral.chebyshev2_points(node_count, interval)
    return cgl_times, nodal_values, fitted_samples, fit_relative_error


def integrate_rk2_rot3(
    initial_rotation: gtsam.Rot3,
    sample_times: Sequence[float] | np.ndarray,
    angular_velocity: Sequence[Sequence[float]] | np.ndarray,
) -> gtsam.Rot3:
    """Integrate body-frame gyro samples with Lie-group midpoint/RK2 updates."""
    times, omega = _validated_time_series(sample_times, angular_velocity)
    rotation = initial_rotation
    for index in range(times.size - 1):
        dt = float(times[index + 1] - times[index])
        omega_midpoint = 0.5 * (omega[index] + omega[index + 1])
        rotation = rotation.compose(gtsam.Rot3.Expmap(dt * omega_midpoint))
    return rotation


def integrate_magnus2_rot3(
    initial_rotation: gtsam.Rot3,
    sample_times: Sequence[float] | np.ndarray,
    angular_velocity: Sequence[Sequence[float]] | np.ndarray,
) -> gtsam.Rot3:
    """Integrate body-frame gyro samples with endpoint SO(3) Magnus-2 updates."""
    times, omega = _validated_time_series(sample_times, angular_velocity)
    rotation = initial_rotation
    for index in range(times.size - 1):
        dt = float(times[index + 1] - times[index])
        delta = (
            0.5 * dt * (omega[index] + omega[index + 1])
            + (dt * dt / 12.0) * np.cross(omega[index + 1], omega[index])
        )
        rotation = rotation.compose(gtsam.Rot3.Expmap(delta))
    return rotation


def trapezoid_delta_theta(
    sample_times: Sequence[float] | np.ndarray,
    angular_velocity: Sequence[Sequence[float]] | np.ndarray,
) -> np.ndarray:
    """Return trapezoidal gyro angle increments between adjacent samples."""
    times, omega = _validated_time_series(sample_times, angular_velocity)
    dt = np.diff(times).reshape(-1, 1)
    return 0.5 * dt * (omega[:-1] + omega[1:])


def integrate_d3_single_speed_rot3(
    initial_rotation: gtsam.Rot3,
    sample_times: Sequence[float] | np.ndarray,
    angular_velocity: Sequence[Sequence[float]] | np.ndarray,
    *,
    previous_delta: Sequence[float] | np.ndarray | None = None,
) -> gtsam.Rot3:
    """Integrate gyro samples with the D3 single-speed coning correction."""
    deltas = trapezoid_delta_theta(sample_times, angular_velocity)
    previous = (
        np.zeros(3, dtype=float)
        if previous_delta is None
        else np.asarray(previous_delta, dtype=float).reshape(3)
    )
    rotation = initial_rotation
    for delta in deltas:
        corrected = delta + (1.0 / 12.0) * np.cross(previous, delta)
        rotation = rotation.compose(gtsam.Rot3.Expmap(corrected))
        previous = delta
    return rotation


def integrate_psychmagpi_rot3(
    initial_rotation: gtsam.Rot3,
    sample_times: Sequence[float] | np.ndarray,
    angular_velocity: Sequence[Sequence[float]] | np.ndarray,
    *,
    N: int | None = None,
    tolerance: float = 1e-12,
    max_iterations: int = 50,
    init_method: gtsam.PseudoSpectralPiInit = gtsam.PseudoSpectralPiInit.RK2,
    lambda1: float = 0.0,
    use_approximate_integration_matrix: bool = False,
) -> PsyChMagPIResult:
    """Integrate gyro samples with GTSAM's ``PsyChMagPIRot3`` wrapper.

    The input preparation fits all samples to Chebyshev-Gauss-Lobatto nodal
    values. ``lambda1`` optionally applies the Chebyshev2 first-derivative
    Sobolev penalty. When ``N`` is omitted, the report rule ``ceil(sqrt(m))``
    is used. The integration-matrix flag selects the exact ``N -> N+1`` matrix
    by default or the historical approximate ``N -> N`` matrix when requested.
    """
    times, omega = _validated_time_series(sample_times, angular_velocity)
    node_count = int(np.ceil(np.sqrt(times.size))) if N is None else int(N)
    if node_count < 2:
        raise ValueError("N must be at least 2")
    if node_count > times.size:
        raise ValueError("N must not exceed the number of samples")

    interval = (float(times[0]), float(times[-1]))
    cgl_times, cgl_omega, _, fit_relative_error = (
        fit_vector_samples_to_chebyshev_nodes(
            times,
            omega,
            node_count,
            lambda1=lambda1,
        )
    )

    params = make_pseudospectral_pi_params(
        init_method,
        tolerance=float(tolerance),
        max_iterations=int(max_iterations),
        use_approximate_integration_matrix=bool(use_approximate_integration_matrix),
    )
    integrator = gtsam.PsyChMagPIRot3(node_count, interval[0], interval[1], params)
    start = time.perf_counter()
    history = integrator.integrateWithHistory(cgl_omega.T)
    final_zeta = np.asarray(history.finalZ, dtype=float)[:, -1]
    rotation = initial_rotation.compose(gtsam.Rot3.Expmap(final_zeta))
    runtime_s = time.perf_counter() - start

    error_norm = np.asarray(history.errorNorm, dtype=float).reshape(-1)
    final_residual = float(error_norm[-1]) if error_norm.size else float("nan")
    return PsyChMagPIResult(
        rotation=rotation,
        N=node_count,
        lambda1=float(lambda1),
        use_approximate_integration_matrix=bool(use_approximate_integration_matrix),
        cgl_times=cgl_times,
        fit_relative_error=fit_relative_error,
        error_norm=error_norm,
        iterations=int(error_norm.size),
        final_residual=final_residual,
        converged=bool(history.converged),
        runtime_s=runtime_s,
    )


def make_pseudospectral_pi_params(
    init_method: gtsam.PseudoSpectralPiInit,
    *,
    tolerance: float,
    max_iterations: int,
    use_approximate_integration_matrix: bool = False,
) -> gtsam.PseudoSpectralPiParams:
    """Construct PsyChMagPI parameters with the integration-matrix flag."""
    try:
        return gtsam.PseudoSpectralPiParams(
            init_method,
            float(tolerance),
            int(max_iterations),
            bool(use_approximate_integration_matrix),
        )
    except TypeError:
        if use_approximate_integration_matrix:
            raise
        return gtsam.PseudoSpectralPiParams(
            init_method,
            float(tolerance),
            int(max_iterations),
        )


def _validated_time_series(
    sample_times: Sequence[float] | np.ndarray,
    angular_velocity: Sequence[Sequence[float]] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    times = np.asarray(sample_times, dtype=float).reshape(-1)
    omega = np.asarray(angular_velocity, dtype=float)
    if times.size < 2:
        raise ValueError("sample_times must contain at least two samples")
    if omega.shape != (times.size, 3):
        raise ValueError("angular_velocity must have shape (len(sample_times), 3)")
    if not np.all(np.isfinite(times)) or not np.all(np.isfinite(omega)):
        raise ValueError("sample_times and angular_velocity must be finite")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("sample_times must be strictly increasing")
    return times, omega
