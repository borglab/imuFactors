"""Sweep independent gyro/accel noise and q, then retain the mean-median target."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np
import pandas as pd
import torch

from imuFactors.delama_gal3.preintegration_delama_gal3 import load_ground_truth_euroc, physical_reporting
from imuFactors.noise_calibration import fit_mean_median_scale, position_drive_covariance
from imuFactors.separate_noise_calibration import ProfileLikelihood, covariance_at
from run_unified_imu_comparison import EUROC_SEQUENCES, INTERVALS


def exported(binary, source, gyro, acc, q, method):
    result = subprocess.run([str(binary), str(source), repr(q), method, repr(gyro), repr(acc)],
                            check=True, capture_output=True, text=True)
    return pd.read_csv(io.StringIO(result.stdout))


def covariance(rows):
    return rows[[f'cov_{i}_{j}' for i in range(9) for j in range(9)]].to_numpy().reshape(-1, 9, 9)


def prepare_cache(binary, sources, output):
    blocks = {}
    for name, source in sources.items():
        streams = load_ground_truth_euroc(str(source))
        truth = streams[0].cpu().numpy()
        for method in ('manifold', 'galilean'):
            gyro = exported(binary, source, 8.4, 0., 0., method)
            acc = exported(binary, source, 0., 8.4, 0., method)
            mixed = exported(binary, source, 3.7, 12.1, 2e-6, method)
            for interval in INTERVALS:
                g = gyro[np.isclose(gyro.interval, interval)]
                a = acc[np.isclose(acc.interval, interval)]
                m = mixed[np.isclose(mixed.interval, interval)]
                columns = ['start', 'end'] + [f'pred_{i}' for i in range(15)] + [f'error_{i}' for i in range(9)]
                np.testing.assert_array_equal(g[columns], a[columns])
                np.testing.assert_array_equal(g[columns], m[columns])
                native_g, native_a = covariance(g), covariance(a)
                duration = (g.end.iloc[0] - g.start.iloc[0]) * streams[-1]
                drive = position_drive_covariance(len(g), duration)
                np.testing.assert_allclose(covariance(m), covariance_at(native_g, native_a, drive, 3.7, 12.1, 2e-6),
                                           atol=1e-16, rtol=1e-8)
                values = g[[f'pred_{i}' for i in range(15)]].to_numpy()
                predicted = np.broadcast_to(np.eye(5), (len(g), 5, 5)).copy()
                predicted[:, :3, :3] = values[:, :9].reshape(-1, 3, 3)
                predicted[:, :3, 4] = values[:, 9:12]
                predicted[:, :3, 3] = values[:, 12:15]
                world = np.broadcast_to(np.eye(9), native_g.shape).copy()
                world[:, 3:6, 3:6] = predicted[:, :3, :3]
                world[:, 6:9, 6:9] = predicted[:, :3, :3]
                error, _ = physical_reporting(torch.tensor(predicted), torch.tensor(truth[g.end.to_numpy(int)]),
                                               torch.eye(5).repeat(len(g), 1, 1), torch.tensor(native_g))
                data = dict(gyro=world @ native_g @ world.transpose(0, 2, 1),
                            acc=world @ native_a @ world.transpose(0, 2, 1), drive=drive, error=error.numpy(),
                            native_gyro=native_g, native_acc=native_a,
                            native_error=g[[f'error_{i}' for i in range(9)]].to_numpy(),
                            predicted=predicted, starts=g.start.to_numpy(int), ends=g.end.to_numpy(int))
                blocks[name, method, interval] = data
                np.savez_compressed(output/f'{name}_{method}_{interval}.npz', **data)
        print(f'Exported and validated independent covariance bases: {name}', flush=True)
    return blocks


def fit_groups(blocks, output):
    settings, likelihood_fits, diagnostics = {}, {}, {}
    sweep = []
    for group in ('MH', 'V'):
        selected = [data for (name, _, _), data in blocks.items() if name.startswith(group)]
        profile = ProfileLikelihood.from_blocks(selected)
        print(f'Fitting {group}: {len(selected)} equally weighted blocks', flush=True)
        best, rows = profile.fit()
        tied, _ = profile.fit(gyro_acc_tied=True)
        likelihood_fits[group] = {k: best[k] for k in ('alpha_gyro', 'alpha_acc', 'integration_covariance')}
        native = [(data['native_error'], covariance_at(data['native_gyro'], data['native_acc'], data['drive'],
                                                      **likelihood_fits[group])) for data in selected]
        median = fit_mean_median_scale(native)
        scale = median['covariance_scale']
        settings[group] = dict(alpha_gyro=best['alpha_gyro'] * np.sqrt(scale),
                               alpha_acc=best['alpha_acc'] * np.sqrt(scale),
                               integration_covariance=best['integration_covariance'] * scale)
        diagnostics[group] = dict(likelihood_nll=best['nll'], tied_sensor_likelihood_nll=tied['nll'],
                                  median_normalization=median, evaluations=len(rows),
                                  q_zero_boundary=bool(best['integration_covariance'] == 0.))
        sweep.extend(dict(group=group, **row) for row in rows)
        print(json.dumps(dict(group=group, likelihood=likelihood_fits[group], recommended=settings[group],
                              diagnostics=diagnostics[group])), flush=True)
    pd.DataFrame(sweep).to_csv(output/'sweep.csv', index=False)
    return settings, likelihood_fits, diagnostics


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=root/'build/tests/exportGalileanParity')
    parser.add_argument('--data-dir', type=Path, default=root/'data/euroc')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    torch.set_num_threads(1)
    sources = {p.stem.removeprefix('euroc_'): p.resolve() for p in sorted(args.data_dir.glob('euroc_*.csv'))}
    if set(sources) != EUROC_SEQUENCES:
        raise ValueError('Require all 11 EuRoC sequences')
    output = args.output or root/'build/separate-noise-calibration'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output.mkdir(parents=True, exist_ok=False)
    cache = output/'cache'
    cache.mkdir()
    hashes = {name: hashlib.sha256(p.read_bytes()).hexdigest() for name, p in sources.items()}
    blocks = prepare_cache(args.binary, sources, cache)
    settings, likelihood, diagnostics = fit_groups(blocks, output)
    if hashes != {name: hashlib.sha256(p.read_bytes()).hexdigest() for name, p in sources.items()}:
        raise ValueError('Source CSV changed during calibration')
    result = dict(protocol='group_separate_mean_median', settings=settings, likelihood_settings=likelihood,
                  parameter_selection='Independent gyro/accelerometer/q Gaussian-likelihood sweep, followed by common covariance scaling to equal-weight mean block median native NEES=1; full-group descriptive fit, not held-out',
                  source_sha256=hashes, diagnostics=diagnostics, fit_methods=['manifold', 'galilean'],
                  no_window_exclusions=True, covariance_basis_validation='Direct mixed-noise propagation passed on all 11 sequences and intervals',
                  base_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip())
    (output/'calibration.json').write_text(json.dumps(result, indent=2)+'\n')
    print(f'Calibration saved: {output}', flush=True)


if __name__ == '__main__':
    main()
