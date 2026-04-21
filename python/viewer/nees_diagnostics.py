"""Numerical helpers for NEES trajectory diagnostics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

ERROR_COMPONENT_COLUMNS = (
    "err_rot_x",
    "err_rot_y",
    "err_rot_z",
    "err_pos_x",
    "err_pos_y",
    "err_pos_z",
    "err_vel_x",
    "err_vel_y",
    "err_vel_z",
)

WHITENED_COMPONENT_COLUMNS = tuple(f"w{index}" for index in range(9))
NORMALIZED_COMPONENT_COLUMNS = (
    "norm_rot_x",
    "norm_rot_y",
    "norm_rot_z",
    "norm_pos_x",
    "norm_pos_y",
    "norm_pos_z",
    "norm_vel_x",
    "norm_vel_y",
    "norm_vel_z",
)
PREDICTED_SIGMA_COLUMNS = ("rot_pred_sigma", "pos_pred_sigma", "vel_pred_sigma")
COVARIANCE_COLUMNS = tuple(f"cov_{row}_{column}" for row in range(9) for column in range(9))
TRAJECTORY_REQUIRED_COLUMNS = (
    "dataset",
    "method",
    "config_label",
    "interval_seconds",
    "timestamp",
    *ERROR_COMPONENT_COLUMNS,
    *PREDICTED_SIGMA_COLUMNS,
    *COVARIANCE_COLUMNS,
)
CORRELATION_EPSILON = 1e-12


@dataclass(frozen=True)
class RowDiagnostics:
    """Derived NEES diagnostics for one trajectory sample row."""

    sample_nees: float
    whitened_residual: np.ndarray | None
    normalized_components: np.ndarray
    jitter: float
    whitening_ok: bool


def rebuild_error_vector(row: pd.Series | dict[str, object]) -> np.ndarray:
    """Rebuild the canonical 9D error vector from one row."""

    return np.array([float(row[column]) for column in ERROR_COMPONENT_COLUMNS], dtype=float)


def rebuild_covariance_matrix(row: pd.Series | dict[str, object]) -> np.ndarray:
    """Rebuild the flattened 9x9 covariance matrix from one row."""

    values = [float(row[column]) for column in COVARIANCE_COLUMNS]
    return np.array(values, dtype=float).reshape(9, 9)


def _component_normalized_residuals(error: np.ndarray, covariance: np.ndarray) -> np.ndarray:
    diagonal = np.diag(covariance)
    normalized = np.full(error.shape, np.nan, dtype=float)
    valid = diagonal > 0.0
    normalized[valid] = error[valid] / np.sqrt(diagonal[valid])
    return normalized


def whiten_residual(
    error: np.ndarray,
    covariance: np.ndarray,
    *,
    initial_jitter: float = 1e-9,
    max_attempts: int = 5,
) -> tuple[np.ndarray | None, float]:
    """Whiten one residual vector with a numerically safe Cholesky factorization."""

    identity = np.eye(covariance.shape[0], dtype=float)
    jitter = 0.0
    for attempt in range(max_attempts):
        adjusted = covariance if jitter == 0.0 else covariance + identity * jitter
        try:
            factor = np.linalg.cholesky(adjusted)
        except np.linalg.LinAlgError:
            jitter = initial_jitter if jitter == 0.0 else jitter * 10.0
            continue
        return np.linalg.solve(factor, error), jitter
    return None, jitter


def compute_row_diagnostics(
    row: pd.Series | dict[str, object],
    *,
    initial_jitter: float = 1e-9,
) -> RowDiagnostics:
    """Compute NEES, whitened residuals, and axis-normalized residuals for one row."""

    error = rebuild_error_vector(row)
    covariance = rebuild_covariance_matrix(row)
    normalized = _component_normalized_residuals(error, covariance)
    whitened, jitter = whiten_residual(error, covariance, initial_jitter=initial_jitter)
    if whitened is None:
        return RowDiagnostics(
            sample_nees=np.nan,
            whitened_residual=None,
            normalized_components=normalized,
            jitter=jitter,
            whitening_ok=False,
        )
    return RowDiagnostics(
        sample_nees=float(np.dot(whitened, whitened) / error.size),
        whitened_residual=whitened,
        normalized_components=normalized,
        jitter=jitter,
        whitening_ok=True,
    )


def augment_trajectory_samples(frame: pd.DataFrame) -> tuple[pd.DataFrame, int]:
    """Add sample-level NEES and residual diagnostics for one interval slice."""

    if frame.empty:
        return frame.copy(), 0

    augmented = frame.reset_index(drop=True).copy()
    skipped = 0
    for column in ("sample_nees", "whitening_jitter", "whitening_ok", *WHITENED_COMPONENT_COLUMNS, *NORMALIZED_COMPONENT_COLUMNS):
        augmented[column] = np.nan

    augmented["whitening_ok"] = False
    for index, row in augmented.iterrows():
        diagnostics = compute_row_diagnostics(row)
        augmented.at[index, "sample_nees"] = diagnostics.sample_nees
        augmented.at[index, "whitening_jitter"] = diagnostics.jitter
        augmented.at[index, "whitening_ok"] = diagnostics.whitening_ok
        for component_index, column in enumerate(WHITENED_COMPONENT_COLUMNS):
            augmented.at[index, column] = (
                np.nan if diagnostics.whitened_residual is None else diagnostics.whitened_residual[component_index]
            )
        for component_index, column in enumerate(NORMALIZED_COMPONENT_COLUMNS):
            augmented.at[index, column] = diagnostics.normalized_components[component_index]
        if not diagnostics.whitening_ok:
            skipped += 1
    return augmented, skipped


def covariance_to_correlation(covariance: np.ndarray) -> np.ndarray:
    """Convert a covariance matrix into a correlation matrix."""

    diagonal = np.sqrt(np.clip(np.diag(covariance), a_min=0.0, a_max=None))
    scale = np.outer(diagonal, diagonal)
    correlation = np.zeros_like(covariance)
    valid = scale > CORRELATION_EPSILON
    correlation[valid] = covariance[valid] / scale[valid]
    np.fill_diagonal(correlation, np.where(diagonal > CORRELATION_EPSILON, 1.0, 0.0))
    return correlation
