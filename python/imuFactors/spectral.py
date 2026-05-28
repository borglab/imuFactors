"""Shared spectral basis construction, fitting, and interpolation helpers."""

from __future__ import annotations

from dataclasses import dataclass
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
    """Precomputed linear maps for fitting and reconstructing one basis."""

    basis: str
    parameter_count: int
    coordinates: np.ndarray
    weight_matrix: np.ndarray
    solver: np.ndarray
    window_seconds: float = DEFAULT_WINDOW_SECONDS
    lambda1: float = 0.0

    @property
    def degree(self) -> int:
        """Polynomial degree for polynomial bases."""
        return self.parameter_count - 1

    @property
    def max_harmonic(self) -> int:
        """Maximum Fourier harmonic represented by this basis."""
        return self.parameter_count // 2

    @property
    def basis_name(self) -> str:
        """Human-readable basis name."""
        return basis_display_name(self.basis)

    def fit(self, values: Sequence[float] | np.ndarray) -> np.ndarray:
        """Fit basis parameters to samples.

        Samples may be shaped as ``(sample_count,)``,
        ``(sample_count, components)``, ``(..., sample_count)``, or
        ``(..., sample_count, components)``.
        """
        array = np.asarray(values, dtype=float)
        sample_count = int(self.weight_matrix.shape[0])
        if array.ndim == 1:
            if array.shape[0] != sample_count:
                raise ValueError("values must align with plan sample count")
            return self.solver @ array
        if array.shape[0] == sample_count:
            return self.solver @ array
        if array.shape[-1] == sample_count:
            return array @ self.solver.T
        if array.ndim >= 2 and array.shape[-2] == sample_count:
            return np.einsum("ks,...sc->...kc", self.solver, array)
        raise ValueError("values must contain a sample-count axis")

    def reconstruct(self, coeffs: Sequence[float] | np.ndarray) -> np.ndarray:
        """Reconstruct samples from basis parameters."""
        array = np.asarray(coeffs, dtype=float)
        parameter_count = int(self.weight_matrix.shape[1])
        if array.ndim == 1:
            if array.shape[0] != parameter_count:
                raise ValueError("coeffs must align with plan parameter count")
            return self.weight_matrix @ array
        if array.shape[0] == parameter_count:
            return self.weight_matrix @ array
        if array.shape[-1] == parameter_count:
            return array @ self.weight_matrix.T
        if array.ndim >= 2 and array.shape[-2] == parameter_count:
            return np.einsum("sk,...kc->...sc", self.weight_matrix, array)
        raise ValueError("coeffs must contain a parameter-count axis")


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
    sample_count: int,
    *,
    basis: str = DEFAULT_BASIS,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return canonical sample coordinates for a basis design."""
    if sample_count < 1:
        raise ValueError("sample_count must be positive")
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev":
        return np.linspace(-1.0, 1.0, sample_count)
    if normalized_basis == "chebyshev2":
        a, b = _validated_interval(interval)
        return np.linspace(a, b, sample_count)
    if normalized_basis == "fourier":
        return np.linspace(0.0, 2.0 * np.pi, sample_count)
    raise AssertionError(f"unhandled basis {normalized_basis}")


def basis_plan(
    parameter_count: int,
    sample_count: int,
    *,
    basis: str = DEFAULT_BASIS,
    window_seconds: float = DEFAULT_WINDOW_SECONDS,
    lambda1: float = 0.0,
) -> BasisPlan:
    """Build a canonical basis fit plan for uniformly sampled windows."""
    coordinates = basis_coordinates(sample_count, basis=basis)
    return basis_plan_from_coordinates(
        parameter_count,
        coordinates,
        basis=basis,
        interval=(0.0, 1.0),
        window_seconds=window_seconds,
        lambda1=lambda1,
    )


def basis_plan_from_coordinates(
    parameter_count: int,
    coordinates: Sequence[float] | np.ndarray,
    *,
    basis: str = DEFAULT_BASIS,
    interval: tuple[float, float] = (0.0, 1.0),
    window_seconds: float | None = None,
    lambda1: float = 0.0,
) -> BasisPlan:
    """Build a basis fit plan for explicit sample coordinates."""
    coordinate_array = np.asarray(coordinates, dtype=float).reshape(-1)
    if parameter_count < 1:
        raise ValueError("parameter_count must be positive")
    if coordinate_array.size < 1:
        raise ValueError("coordinates must not be empty")
    if parameter_count > coordinate_array.size:
        raise ValueError(
            f"parameter_count={parameter_count} exceeds the "
            f"{coordinate_array.size} samples in the design"
        )
    normalized_basis = normalize_basis_name(basis)
    if window_seconds is None:
        window_seconds = float(interval[1] - interval[0])
    if window_seconds <= 0.0:
        raise ValueError("window_seconds must be positive")

    weight_matrix = basis_weight_matrix(
        parameter_count,
        coordinate_array,
        basis=normalized_basis,
        interval=interval,
    )
    solver = basis_solver(
        weight_matrix,
        normalized_basis,
        parameter_count,
        window_seconds=window_seconds,
        lambda1=lambda1,
    )
    return BasisPlan(
        basis=normalized_basis,
        parameter_count=int(parameter_count),
        coordinates=coordinate_array,
        weight_matrix=weight_matrix,
        solver=solver,
        window_seconds=float(window_seconds),
        lambda1=float(lambda1),
    )


def basis_weight_matrix(
    parameter_count: int,
    coordinates: Sequence[float] | np.ndarray,
    *,
    basis: str = DEFAULT_BASIS,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return the GTSAM-backed design matrix for one basis."""
    if parameter_count < 1:
        raise ValueError("parameter_count must be positive")
    coordinate_array = np.asarray(coordinates, dtype=float).reshape(-1)
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev":
        return np.asarray(
            gtsam.Chebyshev1Basis.WeightMatrix(int(parameter_count), coordinate_array),
            dtype=float,
        )
    if normalized_basis == "chebyshev2":
        a, b = _validated_interval(interval)
        return np.asarray(
            gtsam.Chebyshev2.WeightMatrix(
                int(parameter_count), coordinate_array, a, b
            ),
            dtype=float,
        )
    if normalized_basis == "fourier":
        return np.asarray(
            gtsam.FourierBasis.WeightMatrix(int(parameter_count), coordinate_array),
            dtype=float,
        )
    raise AssertionError(f"unhandled basis {normalized_basis}")


def basis_solver(
    weight_matrix: np.ndarray,
    basis: str,
    parameter_count: int,
    *,
    window_seconds: float = DEFAULT_WINDOW_SECONDS,
    lambda1: float = 0.0,
) -> np.ndarray:
    """Return an unregularized or Sobolev-regularized linear solver."""
    normalized_basis = normalize_basis_name(basis)
    matrix = np.asarray(weight_matrix, dtype=float)
    if matrix.ndim != 2:
        raise ValueError("weight_matrix must be two-dimensional")
    if matrix.shape[1] != int(parameter_count):
        raise ValueError("parameter_count must match weight_matrix columns")
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
        int(parameter_count), window_seconds=window_seconds
    )
    rhs = matrix.T
    return np.linalg.solve(lhs, rhs)


def chebyshev2_points(
    parameter_count: int,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return Chebyshev-Gauss-Lobatto nodes for the Chebyshev2 basis."""
    a, b = _validated_interval(interval)
    return np.asarray(
        gtsam.Chebyshev2.Points(int(parameter_count), a, b), dtype=float
    )


def chebyshev2_weight_matrix(
    parameter_count: int,
    times: Sequence[float] | np.ndarray,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return a Chebyshev2 interpolation matrix at arbitrary times."""
    return basis_weight_matrix(
        parameter_count,
        times,
        basis="chebyshev2",
        interval=interval,
    )


def chebyshev2_integration_weights(
    parameter_count: int,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return weights that integrate Chebyshev2 node values over an interval."""
    a, b = _validated_interval(interval)
    return np.asarray(
        gtsam.Chebyshev2.IntegrationWeights(int(parameter_count), a, b),
        dtype=float,
    ).reshape(int(parameter_count))


def chebyshev2_integration_matrix(
    parameter_count: int,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return the Chebyshev2 matrix mapping node values to integral nodes."""
    a, b = _validated_interval(interval)
    return np.asarray(
        gtsam.Chebyshev2.IntegrationMatrix(int(parameter_count), a, b),
        dtype=float,
    )


def chebyshev2_integral_curve_matrix(
    parameter_count: int,
    evaluation_times: Sequence[float] | np.ndarray,
    interval: tuple[float, float] = (0.0, 1.0),
) -> np.ndarray:
    """Return weights mapping Chebyshev2 node values to integral curves."""
    integration_matrix = chebyshev2_integration_matrix(parameter_count, interval)
    evaluation_matrix = chebyshev2_weight_matrix(
        parameter_count + 1,
        evaluation_times,
        interval,
    )
    return evaluation_matrix @ integration_matrix


def chebyshev2_first_derivative_penalty(
    parameter_count: int,
    *,
    window_seconds: float = DEFAULT_WINDOW_SECONDS,
) -> np.ndarray:
    """Return the CGL nodal penalty matrix for ``integral |p'(t)|^2 dt``."""
    if parameter_count < 1:
        raise ValueError("parameter_count must be positive")
    if window_seconds <= 0.0:
        raise ValueError("window_seconds must be positive")
    if parameter_count == 1:
        return np.zeros((1, 1), dtype=float)

    derivative = np.asarray(
        gtsam.Chebyshev2.DifferentiationMatrix(int(parameter_count), 0.0, 1.0),
        dtype=float,
    )
    weights = chebyshev2_integration_weights(parameter_count, (0.0, 1.0))
    penalty = derivative.T @ (weights.reshape(-1, 1) * derivative)
    return penalty / float(window_seconds)


def basis_order_summary(basis: str, parameter_count: int) -> str:
    """Return a compact basis-order description for tables."""
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev":
        return f"degree {parameter_count - 1}"
    if normalized_basis == "chebyshev2":
        return f"{parameter_count} CGL node values"
    if normalized_basis == "fourier":
        if parameter_count == 1:
            return "constant only"
        return f"max harmonic {parameter_count // 2}"
    raise AssertionError(f"unhandled basis {normalized_basis}")


def basis_tick_labels(basis: str, parameter_count: int) -> list[str]:
    """Return coefficient labels for a basis parameterization."""
    normalized_basis = normalize_basis_name(basis)
    if normalized_basis == "chebyshev":
        return [f"T{k}" for k in range(parameter_count)]
    if normalized_basis == "chebyshev2":
        return [f"cgl{k}" for k in range(parameter_count)]
    if normalized_basis == "fourier":
        labels = ["1"]
        harmonic = 1
        for index in range(1, parameter_count):
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
