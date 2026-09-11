"""Two-parameter, sequence-held-out covariance calibration in physical coordinates."""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq, minimize_scalar

BASE_ALPHA = 8.4


@dataclass
class LikelihoodBlock:
    """Whitened Gaussian likelihood for one equally weighted sequence/interval/method."""
    eigenvalues: np.ndarray
    squared_errors: np.ndarray
    logdet_base: np.ndarray

    @classmethod
    def from_arrays(cls, errors, covariance, position_drive):
        cholesky = np.linalg.cholesky(covariance)
        inverse = np.linalg.solve(cholesky, np.broadcast_to(np.eye(9), covariance.shape))
        whitened_drive = inverse @ position_drive @ inverse.transpose(0, 2, 1)
        eigenvalues, vectors = np.linalg.eigh(whitened_drive)
        if eigenvalues.min() < -1e-6:
            raise ValueError('Position-drive covariance must be positive semidefinite')
        whitened_errors = np.linalg.solve(cholesky, errors[..., None])[..., 0]
        projected = np.einsum('nji,nj->ni', vectors, whitened_errors)
        return cls(np.maximum(eigenvalues, 0), projected**2,
                   2 * np.log(cholesky.diagonal(axis1=1, axis2=2)).sum(axis=1))


def position_drive_covariance(count, duration, position_start=3):
    result = np.zeros((count, 9, 9))
    result[:, position_start:position_start+3, position_start:position_start+3] = duration * np.eye(3)
    return result


def covariance_at(base, drive, alpha, q):
    """Noise-only covariance is linear in alpha squared and independent position q."""
    if not np.isfinite(alpha) or alpha <= 0 or not np.isfinite(q) or q < 0:
        raise ValueError('Require finite alpha > 0 and q >= 0')
    return (alpha / BASE_ALPHA)**2 * base + q * drive


def likelihood(blocks, alpha, q):
    """Gaussian NLL, equally weighted across supplied blocks; includes log determinant."""
    scale = (alpha / BASE_ALPHA)**2
    return np.mean([.5 * np.mean(block.logdet_base +
                     np.sum(np.log(scale + q * block.eigenvalues) +
                            block.squared_errors / (scale + q * block.eigenvalues), axis=1) +
                     9 * np.log(2 * np.pi)) for block in blocks])


def fit_noise(blocks):
    """Profile out overall scale; globally scan then refine log(q/scale), including q=0.

    No held-out block enters this function. The fixed log-ratio search range is
    [-35, 5]; a winning endpoint is rejected rather than silently clipped.
    """
    if not blocks:
        raise ValueError('At least one training block is required')

    def profiled(ratio):
        scale = np.mean([np.mean(np.sum(block.squared_errors /
                         (1 + ratio * block.eigenvalues), axis=1)) for block in blocks]) / 9
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError('Cannot fit zero or non-finite empirical error covariance')
        alpha, q = BASE_ALPHA * np.sqrt(scale), ratio * scale
        return likelihood(blocks, alpha, q), alpha, q

    grid = np.linspace(-35., 5., 81)
    scores = np.array([profiled(np.exp(x))[0] for x in grid])
    candidates = [(*profiled(0.), 'q_zero')]
    for index in range(1, len(grid)-1):
        if scores[index] <= min(scores[index-1], scores[index+1]):
            optimum = minimize_scalar(lambda x: profiled(np.exp(x))[0],
                                      bounds=(grid[index-1], grid[index+1]),
                                      method='bounded', options={'xatol': 1e-8})
            if not optimum.success:
                raise RuntimeError('Noise optimization failed')
            candidates.append((*profiled(np.exp(optimum.x)), 'interior'))
    best = min(candidates, key=lambda x: x[0])
    if scores[-1] < best[0] - 1e-8 or scores[0] < best[0] - 1e-8:
        raise RuntimeError('Noise optimum reaches the fixed ratio search boundary')
    return dict(training_nll=float(best[0]), alpha=float(best[1]),
                integration_covariance=float(best[2]), optimum=best[3])


def training_sequences(names, held_out, protocol):
    if protocol not in ('global_loso', 'group_loso') or held_out not in names:
        raise ValueError('Unknown protocol or held-out sequence')
    group = 'MH' if held_out.startswith('MH') else 'V'
    return sorted(name for name in names if name != held_out and
                  (protocol == 'global_loso' or name.startswith(group)))


def normalized_nees(errors, covariance):
    regularized = covariance + 1e-12 * np.eye(9)
    return np.einsum('ni,ni->n', errors,
                     np.linalg.solve(regularized, errors[..., None])[..., 0]) / 9


def fit_mean_median_scale(blocks, target=1., regularization=1e-12):
    """Scale covariances to match an equally weighted mean of block medians.

    Each block is (native_errors, native_covariances). Keep the diagonal
    regularization fixed while scaling covariance; never scale residuals.
    """
    if not blocks or not np.isfinite(target) or target <= 0:
        raise ValueError('Require nonempty blocks and finite positive target')
    if not np.isfinite(regularization) or regularization < 0:
        raise ValueError('Require finite nonnegative regularization')
    spectral = []
    for errors, covariance in blocks:
        if errors.ndim != 2 or errors.shape[1] != 9 or not len(errors):
            raise ValueError('Require nonempty N x 9 residuals')
        if covariance.shape != (len(errors), 9, 9) or not (
                np.isfinite(errors).all() and np.isfinite(covariance).all()):
            raise ValueError('Invalid covariance or residual entries')
        if not np.allclose(covariance, covariance.transpose(0, 2, 1), atol=1e-15, rtol=1e-10):
            raise ValueError('Covariance must be symmetric')
        eigenvalues, vectors = np.linalg.eigh(covariance)
        if np.any(eigenvalues <= 0):
            raise ValueError('Covariance must be positive definite')
        squared = np.einsum('nji,nj->ni', vectors, errors)**2
        spectral.append((eigenvalues, squared))

    def objective(log_scale):
        scale = np.exp(log_scale)
        return float(np.mean([np.median(np.sum(squared / (scale * eigenvalues + regularization), axis=1) / 9)
                              for eigenvalues, squared in spectral]))

    if objective(-30.) <= target or objective(30.) >= target:
        raise ValueError('Target is unreachable within the covariance scale bounds')
    log_scale = brentq(lambda value: objective(value) - target, -30., 30., xtol=1e-13)
    return dict(covariance_scale=float(np.exp(log_scale)), target=target,
                mean_median_before=objective(0.), mean_median_after=objective(log_scale),
                blocks=len(blocks), regularization=regularization)
