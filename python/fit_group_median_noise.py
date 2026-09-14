"""Retune fixed MH/V covariance scales to mean per-block median NEES = 1."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from imuFactors.noise_calibration import covariance_at, fit_mean_median_scale, normalized_nees
from run_unified_imu_comparison import EUROC_SEQUENCES, INTERVALS
from run_fixed_group_imu_comparison import validate_settings


def fit_groups(cache, seed_package, data_dir):
    seed = json.loads((seed_package/'verification.json').read_text())
    validate_settings(seed['settings'])
    reference = pd.read_csv(seed_package/'window_metrics.csv')
    settings, fits, cache_hashes = {}, {}, {}
    for name in sorted(EUROC_SEQUENCES):
        source = data_dir/f'euroc_{name}.csv'
        if hashlib.sha256(source.read_bytes()).hexdigest() != seed['source_sha256'][name]:
            raise ValueError(f'Source CSV changed: {name}')
    for group, initial in seed['settings'].items():
        blocks = []
        for name in sorted(n for n in EUROC_SEQUENCES if n.startswith(group)):
            for interval in INTERVALS:
                for method in ('manifold', 'galilean'):
                    path = cache/f'{name}_{method}_{interval}.npz'
                    with np.load(path) as data:
                        errors = data['native_error']
                        covariance = covariance_at(data['native_cov'], data['native_drive'],
                                                   initial['alpha'], initial['integration_covariance'])
                        rows = reference[(reference.dataset == name) & (reference.method == method)
                                         & (reference.interval_seconds == interval)].sort_values('window_index')
                        np.testing.assert_array_equal(rows.window_start_sample, data['starts'])
                        np.testing.assert_array_equal(rows.window_end_sample, data['ends'])
                        np.testing.assert_allclose(normalized_nees(errors, covariance), rows.normalized_nees,
                                                   atol=1e-9, rtol=5e-6)
                    blocks.append((errors, covariance))
                    cache_hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        fit = fit_mean_median_scale(blocks)
        scale = fit['covariance_scale']
        settings[group] = dict(alpha=initial['alpha'] * np.sqrt(scale),
                               integration_covariance=initial['integration_covariance'] * scale)
        fits[group] = fit
    return dict(protocol='group_mean_median', settings=settings, fits=fits,
                seed_package=str(seed_package.resolve()), seed_settings=seed['settings'],
                parameter_selection='Full-group descriptive calibration: equal sequence/interval/manifold-Galilean mean of native normalized NEES medians = 1; preserve q/alpha^2 from the prior fit; not held-out',
                regularization=1e-12, source_sha256=seed['source_sha256'], cache_sha256=cache_hashes)


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--seed-package', type=Path, required=True)
    parser.add_argument('--data-dir', type=Path, default=root/'data/euroc')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = fit_groups(args.cache, args.seed_package, args.data_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(settings=result['settings'], fits=result['fits']), indent=2))


if __name__ == '__main__':
    main()
