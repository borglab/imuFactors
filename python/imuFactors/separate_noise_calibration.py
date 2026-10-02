"""Independent gyro, accelerometer, and position-drive covariance calibration."""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize, minimize_scalar

BASE_ALPHA = 8.4
Q_REFERENCE = 1e-5


def covariance_at(gyro, acc, drive, alpha_gyro, alpha_acc, integration_covariance):
    if (not all(np.isfinite(x) for x in (alpha_gyro, alpha_acc, integration_covariance))
            or alpha_gyro <= 0 or alpha_acc <= 0 or integration_covariance < 0):
        raise ValueError('Require finite positive sensor scales and nonnegative q')
    return (alpha_gyro / BASE_ALPHA)**2 * gyro + (alpha_acc / BASE_ALPHA)**2 * acc + integration_covariance * drive


@dataclass
class ProfileLikelihood:
    """Whitened bases and physical errors, with equal weight per supplied block."""
    gyro: np.ndarray
    acc: np.ndarray
    drive: np.ndarray
    errors: np.ndarray
    logdet_base: np.ndarray
    weights: np.ndarray

    @classmethod
    def from_blocks(cls, blocks):
        if not blocks:
            raise ValueError('Need at least one calibration block')
        arrays = []
        for data in blocks:
            gyro, acc = data['gyro'], data['acc']
            base = gyro + acc
            cholesky = np.linalg.cholesky(base)
            inverse = np.linalg.solve(cholesky, np.broadcast_to(np.eye(9), base.shape))
            def whiten(matrix):
                result = inverse @ matrix @ inverse.transpose(0, 2, 1)
                return .5 * (result + result.transpose(0, 2, 1))
            arrays.append((whiten(gyro), whiten(acc), whiten(data['drive']) * Q_REFERENCE,
                           (inverse @ data['error'][..., None])[..., 0],
                           2 * np.log(cholesky.diagonal(axis1=1, axis2=2)).sum(axis=1),
                           np.full(len(base), 1. / (len(blocks) * len(base)))))
        return cls(*(np.concatenate([row[i] for row in arrays]) for i in range(6)))

    def score(self, ratio_acc, ratio_q):
        """Profile the common scale analytically; q=0 is an explicit boundary."""
        shape = self.gyro + ratio_acc * self.acc + ratio_q * self.drive
        sign, logdet = np.linalg.slogdet(shape)
        if np.any(sign <= 0):
            return dict(nll=float('inf'))
        solved = np.linalg.solve(shape, self.errors[..., None])[..., 0]
        mahalanobis = np.einsum('ni,ni->n', self.errors, solved)
        scale = float(self.weights @ mahalanobis / 9)
        if not np.isfinite(scale) or scale <= 0:
            return dict(nll=float('inf'))
        nll = .5 * (self.weights @ (self.logdet_base + logdet) + 9 * np.log(scale)
                    + 9 + 9 * np.log(2 * np.pi))
        return dict(nll=float(nll), alpha_gyro=BASE_ALPHA * np.sqrt(scale),
                    alpha_acc=BASE_ALPHA * np.sqrt(scale * ratio_acc),
                    integration_covariance=scale * ratio_q * Q_REFERENCE,
                    ratio_acc=float(ratio_acc), ratio_q=float(ratio_q))

    def fit(self, gyro_acc_tied=False):
        """Coarse log-ratio sweep plus multiple local refinements and a q=0 fit.

        Profiling the overall scale solves one dimension of the three-parameter
        search exactly. Boundary winners trigger failure instead of being hidden.
        """
        trace = []
        def evaluate(x, phase, zero_q=False):
            ratio_acc = 1. if gyro_acc_tied else np.exp(x[0])
            ratio_q = 0. if zero_q else np.exp(x[-1])
            result = self.score(ratio_acc, ratio_q)
            trace.append(dict(phase=phase, **result))
            return result['nll']

        # Dimensionless ratios: accel/gyro noise power and q/(gyro power*1e-5).
        acc_grid = [0.] if gyro_acc_tied else np.linspace(-8., 8., 17)
        q_grid = np.linspace(-16., 12., 15)
        seeds = []
        for a in acc_grid:
            evaluate([a], 'sweep_q_zero', True)
            for q in q_grid:
                x = [q] if gyro_acc_tied else [a, q]
                seeds.append((evaluate(x, 'sweep'), x))
        if not gyro_acc_tied:
            zero = minimize_scalar(lambda a: evaluate([a], 'refine_q_zero', True),
                                   bounds=(-12., 12.), method='bounded', options={'xatol': 1e-9})
            if not zero.success:
                raise RuntimeError('q=0 refinement failed')
        bounds = [(-25., 18.)] if gyro_acc_tied else [(-12., 12.), (-25., 18.)]
        for _, seed in sorted(seeds, key=lambda item: item[0])[:3]:
            fit = minimize(lambda x: evaluate(x, 'refine'), seed, method='Nelder-Mead',
                           bounds=bounds, options={'xatol': 1e-6, 'fatol': 1e-10, 'maxiter': 350})
            if not fit.success:
                raise RuntimeError(f'Noise refinement failed: {fit.message}')
        best = min(trace, key=lambda row: row['nll'])
        zero_best = min((row for row in trace if row['ratio_q'] == 0), key=lambda row: row['nll'])
        if zero_best['nll'] <= best['nll'] + 1e-9:
            best = zero_best
        if not np.isfinite(best['nll']):
            raise RuntimeError('No finite likelihood fit')
        if not gyro_acc_tied and abs(np.log(best['ratio_acc'])) > 11.9:
            raise RuntimeError('Sensor noise ratio reached search boundary')
        if best['ratio_q'] > 0 and not -24.9 < np.log(best['ratio_q']) < 17.9:
            raise RuntimeError('Position noise ratio reached search boundary')
        return best, trace
