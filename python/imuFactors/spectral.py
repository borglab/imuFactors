"""Shared spectral basis construction, fitting, and interpolation helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import gtsam
import numpy as np


DEFAULT_BASIS = "chebyshev"
DEFAULT_WINDOW_SECONDS = 1.0
BASIS_OPTIONS = ("chebyshev", "chebyshev2", "fourier")
BASIS_DISPLAY_NAMES = {
    "chebyshev": "Chebyshev",
    "chebyshev2": "Chebyshev2 pseudo-spectral",
    "fourier": "Fourier",
}


@dataclass(frozen=True)
class BasisPlan:
    """Precomputed linear maps for fitting and reconstructing one basis.

    ``m`` is the number of input samples used in the fit. ``N`` is the fitted
    parameter count. For Chebyshev polynomial bases, ``N`` is also the number
    of CGL nodes and ``n = N - 1`` is the polynomial degree. For Fourier,
    ``N`` is only the fitted parameter count; ``n`` is not a polynomial degree.
    """

    basis: str
    N: int
    m: int
    coordinates: np.ndarray
    weight_matrix: np.ndarray
    solver: np.ndarray
    window_seconds: float = DEFAULT_WINDOW_SECONDS
    lambda1: float = 0.0
    n: int = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "N", int(self.N))
        object.__setattr__(self, "m", int(self.m))
        object.__setattr__(self, "n", int(self.N) - 1)
        if self.N < 1:
            raise ValueError("N must be positive")
        if self.m < 1:
            raise ValueError("m must be positive")
        if self.coordinates.shape[0] != self.m:
            raise ValueError("coordinates must have length m")
        if self.weight_matrix.shape != (self.m, self.N):
            raise ValueError("weight_matrix must have shape (m, N)")
        if self.solver.shape != (self.N, self.m):
            raise ValueError("solver must have shape (N, m)")

    @property
    def max_harmonic(self) -> int:
        """Maximum Fourier harmonic represented by this basis."""
        return self.N // 2

    @property
    def basis_name(self) -> str:
        """Human-readable basis name."""
        return basis_display_name(self.basis)

    def fit(self, values: Sequence[float] | np.ndarray) -> np.ndarray:
        """Fit basis parameters to samples.

        Samples may be shaped as ``(m,)``, ``(m, components)``, ``(..., m)``,
        or ``(..., m, components)``.
        """
        array = np.asarray(values, dtype=float)
        if array.ndim == 1:
            if array.shape[0] != self.m:
                raise ValueError("values must align with m")
            return self.solver @ array
        if array.shape[0] == self.m:
            return self.solver @ array
        if array.shape[-1] == self.m:
            return array @ self.solver.T
        if array.ndim >= 2 and array.shape[-2] == self.m:
            return np.einsum("ks,...sc->...kc", self.solver, array)
        raise ValueError("values must contain an m axis")

    def reconstruct(self, coeffs: Sequence[float] | np.ndarray) -> np.ndarray:
        """Reconstruct samples from fitted basis parameters."""
        array = np.asarray(coeffs, dtype=float)
        if array.ndim == 1:
            if array.shape[0] != self.N:
                raise ValueError("coeffs must align with N")
            return self.weight_matrix @ array
        if array.shape[0] == self.N:
            return self.weight_matrix @ array
        if array.shape[-1] == self.N:
            return array @ self.weight_matrix.T
        if array.ndim >= 2 and array.shape[-2] == self.N:
            return np.einsum("sk,...kc->...sc", self.weight_matrix, array)
        raise ValueError("coeffs must contain an N axis")


def normalize_basis_name(basis: str) -> str:
    """Normalize and validate a supported spectral basis name."""
    normalized = basis.strip().lower().replace("_", "-")
    aliases = {
        "cheb": "chebyshev",
        "chebyshev1": "chebyshev",
        "chebyshev-1": "chebyshev",
        "chebyshev1basis": "chebyshev",
        "chebyshev-1-basis": "chebyshev",
        "chebyshev2basis": "chebyshev2",
        "chebyshev-2": "chebyshev2",
        "chebyshev-2-basis": "chebyshev2",
        "chebyshev2-pseudospectral": "chebyshev2",
        "chebyshev2-pseudo-spectral": "chebyshev2",
        "pseudospectral": "chebyshev2",
        "pseudo-spectral": "chebyshev2",
        "fourierbasis": "fourier",
        "fourier-basis": "fourier",
    }
    normalized = aliases.get(normalized, normalized)
    if normalized not in BASIS_OPTIONS:
        raise ValueError(
            f"Unknown basis {basis!r}; expected one of {', '.join(BASIS_OPTIONS)}"
        )
    return normalized


def basis_display_name(basis: str) -> str:
    """Return a human-readable basis name."""
    return BASIS_DISPLAY_NAMES[normalize_basis_name(basis)]


def basis_coordinates(
    m: int,
    *,
    basis: str = DEFAULT_BASIS,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return canonical coordinates for ``m`` uniformly spaced samples."""
    m = int(m)
    if m < 1:
        raise ValueError("m must be positive")
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev":
        return np.linspace(-1.0, 1.0, m)
    if normalized_basis == "chebyshev2":
        a, b = _validated_interval(interval)
        return np.linspace(a, b, m)
    if normalized_basis == "fourier":
        return np.linspace(0.0, 2.0 * np.pi, m)
    raise AssertionError(f"unhandled basis {normalized_basis}")


def basis_plan(
    N: int,
    m: int,
    *,
    basis: str = DEFAULT_BASIS,
    window_seconds: float = DEFAULT_WINDOW_SECONDS,
    lambda1: float = 0.0,
) -> BasisPlan:
    """Build a canonical basis fit plan for uniformly sampled windows.

    ``m`` is the number of input samples. ``N`` is the fitted parameter count.
    For Chebyshev2, ``N`` is the number of CGL nodes and ``n = N - 1``.
    """
    coordinates = basis_coordinates(m, basis=basis)
    return basis_plan_from_coordinates(
        N,
        coordinates,
        basis=basis,
        interval=(0.0, 1.0),
        window_seconds=window_seconds,
        lambda1=lambda1,
    )


def basis_plan_from_coordinates(
    N: int,
    coordinates: Sequence[float] | np.ndarray,
    *,
    basis: str = DEFAULT_BASIS,
    interval: tuple[float, float] = (0.0, 1.0),
    window_seconds: float | None = None,
    lambda1: float = 0.0,
) -> BasisPlan:
    """Build a basis fit plan for explicit sample coordinates."""
    N = int(N)
    coordinate_array = np.asarray(coordinates, dtype=float).reshape(-1)
    m = int(coordinate_array.size)
    if N < 1:
        raise ValueError("N must be positive")
    if m < 1:
        raise ValueError("coordinates must not be empty")
    if N > m:
        raise ValueError(f"N={N} exceeds the {m} samples in the design")
    normalized_basis = normalize_basis_name(basis)
    if window_seconds is None:
        window_seconds = float(interval[1] - interval[0])
    if window_seconds <= 0.0:
        raise ValueError("window_seconds must be positive")

    weight_matrix = basis_weight_matrix(
        N,
        coordinate_array,
        basis=normalized_basis,
        interval=interval,
    )
    solver = basis_solver(
        weight_matrix,
        normalized_basis,
        N,
        window_seconds=window_seconds,
        lambda1=lambda1,
    )
    return BasisPlan(
        basis=normalized_basis,
        N=N,
        m=m,
        coordinates=coordinate_array,
        weight_matrix=weight_matrix,
        solver=solver,
        window_seconds=float(window_seconds),
        lambda1=float(lambda1),
    )


def basis_weight_matrix(
    N: int,
    coordinates: Sequence[float] | np.ndarray,
    *,
    basis: str = DEFAULT_BASIS,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return the GTSAM-backed design matrix for one basis."""
    N = int(N)
    if N < 1:
        raise ValueError("N must be positive")
    coordinate_array = np.asarray(coordinates, dtype=float).reshape(-1)
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev":
        return np.asarray(
            gtsam.Chebyshev1Basis.WeightMatrix(N, coordinate_array),
            dtype=float,
        )
    if normalized_basis == "chebyshev2":
        a, b = _validated_interval(interval)
        return np.asarray(
            gtsam.Chebyshev2.WeightMatrix(N, coordinate_array, a, b),
            dtype=float,
        )
    if normalized_basis == "fourier":
        return np.asarray(
            gtsam.FourierBasis.WeightMatrix(N, coordinate_array),
            dtype=float,
        )
    raise AssertionError(f"unhandled basis {normalized_basis}")


def basis_solver(
    weight_matrix: np.ndarray,
    basis: str,
    N: int,
    *,
    window_seconds: float = DEFAULT_WINDOW_SECONDS,
    lambda1: float = 0.0,
) -> np.ndarray:
    """Return an unregularized or Sobolev-regularized linear solver."""
    N = int(N)
    normalized_basis = normalize_basis_name(basis)
    matrix = np.asarray(weight_matrix, dtype=float)
    if matrix.ndim != 2:
        raise ValueError("weight_matrix must be two-dimensional")
    if matrix.shape[1] != N:
        raise ValueError("N must match weight_matrix columns")
    if window_seconds <= 0.0:
        raise ValueError("window_seconds must be positive")
    lambda1 = float(lambda1)
    if lambda1 < 0.0:
        raise ValueError("lambda1 must be non-negative")
    if lambda1 == 0.0:
        return np.linalg.pinv(matrix)
    if normalized_basis != "chebyshev2":
        raise ValueError(
            "lambda1 Sobolev regularization is only supported for Chebyshev2"
        )

    lhs = matrix.T @ matrix
    lhs += lambda1 * chebyshev2_first_derivative_penalty(
        N, window_seconds=window_seconds
    )
    rhs = matrix.T
    return np.linalg.solve(lhs, rhs)


def chebyshev2_points(
    N: int,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return ``N`` Chebyshev-Gauss-Lobatto nodes for Chebyshev2."""
    a, b = _validated_interval(interval)
    return np.asarray(gtsam.Chebyshev2.Points(int(N), a, b), dtype=float)


def chebyshev2_weight_matrix(
    N: int,
    times: Sequence[float] | np.ndarray,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return a Chebyshev2 interpolation matrix at arbitrary times."""
    return basis_weight_matrix(
        N,
        times,
        basis="chebyshev2",
        interval=interval,
    )


def chebyshev2_integration_weights(
    N: int,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return weights that integrate ``N`` Chebyshev2 CGL node values."""
    a, b = _validated_interval(interval)
    N = int(N)
    return np.asarray(
        gtsam.Chebyshev2.IntegrationWeights(N, a, b),
        dtype=float,
    ).reshape(N)


def chebyshev2_integration_matrix(
    N: int,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return the Chebyshev2 matrix mapping ``N`` values to integral nodes."""
    a, b = _validated_interval(interval)
    return np.asarray(
        gtsam.Chebyshev2.IntegrationMatrix(int(N), a, b),
        dtype=float,
    )


def chebyshev2_integral_curve_matrix(
    N: int,
    evaluation_times: Sequence[float] | np.ndarray,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return weights mapping ``N`` Chebyshev2 values to integral curves."""
    integration_matrix = chebyshev2_integration_matrix(N, interval)
    evaluation_matrix = chebyshev2_weight_matrix(
        int(N) + 1,
        evaluation_times,
        interval=interval,
    )
    return evaluation_matrix @ integration_matrix


def chebyshev2_first_derivative_penalty(
    N: int,
    *,
    window_seconds: float = DEFAULT_WINDOW_SECONDS,
) -> np.ndarray:
    """Return the CGL nodal penalty matrix for ``integral |p'(t)|^2 dt``."""
    N = int(N)
    if N < 1:
        raise ValueError("N must be positive")
    if window_seconds <= 0.0:
        raise ValueError("window_seconds must be positive")
    if N == 1:
        return np.zeros((1, 1), dtype=float)

    derivative = np.asarray(
        gtsam.Chebyshev2.DifferentiationMatrix(N, 0.0, 1.0),
        dtype=float,
    )
    weights = chebyshev2_integration_weights(N, (0.0, 1.0))
    penalty = derivative.T @ (weights.reshape(-1, 1) * derivative)
    return penalty / float(window_seconds)


def basis_order_summary(basis: str, N: int) -> str:
    """Return a compact basis-order description for tables."""
    N = int(N)
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev":
        return f"degree {N - 1}"
    if normalized_basis == "chebyshev2":
        return f"N={N} CGL node values, n={N - 1}"
    if normalized_basis == "fourier":
        if N == 1:
            return "constant only"
        return f"{N} Fourier parameters, max harmonic {N // 2}"
    raise AssertionError(f"unhandled basis {normalized_basis}")


def basis_tick_labels(basis: str, N: int) -> list[str]:
    """Return coefficient labels for a basis parameterization."""
    N = int(N)
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev":
        return [f"T{k}" for k in range(N)]
    if normalized_basis == "chebyshev2":
        return [f"cgl{k}" for k in range(N)]
    if normalized_basis == "fourier":
        labels = ["1"]
        harmonic = 1
        for index in range(1, N):
            if index % 2 == 1:
                labels.append(f"cos{harmonic}")
            else:
                labels.append(f"sin{harmonic}")
                harmonic += 1
        return labels
    raise AssertionError(f"unhandled basis {normalized_basis}")


def basis_axis_title(basis: str) -> str:
    """Return an x-axis title matching a basis parameterization."""
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev2":
        return "Chebyshev2 pseudo-spectral CGL nodes"
    return f"{basis_display_name(normalized_basis)} spectral basis"


def _validated_interval(interval: tuple[float, float]) -> tuple[float, float]:
    a, b = float(interval[0]), float(interval[1])
    if not np.isfinite(a) or not np.isfinite(b) or b <= a:
        raise ValueError("interval must satisfy finite a < b")
    return a, b
