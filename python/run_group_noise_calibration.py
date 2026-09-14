"""Audit reference jumps, calibrate shared MH/V noise with LOSO, and publish viewer runs."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
import pandas as pd
from scipy.stats import chi2
import torch

from imuFactors.delama_gal3.canonical import WINDOW_FIELDS, SUMMARY_FIELDS, summarize_rows
from imuFactors.delama_gal3.preintegration_delama_gal3 import (
    evaluate_interval, load_ground_truth_euroc, physical_reporting,
)
from imuFactors.noise_calibration import (
    BASE_ALPHA, LikelihoodBlock, covariance_at, fit_noise, likelihood,
    normalized_nees, position_drive_covariance, training_sequences,
)
from run_unified_imu_comparison import (
    CANONICAL_FILES, EUROC_SEQUENCES, INTERVALS, read_csv, validate_package, METHODS as BASELINE_METHODS,
)

METHODS = ('manifold', 'galilean', 'delama_gal3_python')
FIT_METHODS = ('manifold', 'galilean')  # Count equivalent Galilean implementations once.


def cpp_evaluation(binary, source, method, alpha=BASE_ALPHA, q=0.):
    completed = subprocess.run([str(binary), str(source), repr(q), method, repr(alpha)],
                               capture_output=True, text=True, check=True)
    return pd.read_csv(io.StringIO(completed.stdout))


def prepare_cache(binary, sources, output):
    """Cache full precision zero-q evaluations and validate analytic noise scaling."""
    cache = {}
    for name, source in sources.items():
        streams = load_ground_truth_euroc(str(source))
        truth = streams[0].cpu().numpy()
        times = streams[-2].cpu().numpy()
        for method in METHODS:
            cpp = cpp_evaluation(binary, source, method) if method in FIT_METHODS else None
            for interval in INTERVALS:
                result = evaluate_interval(streams, interval, integration_covariance=0.) if cpp is None else None
                if cpp is not None:
                    rows = cpp[np.isclose(cpp.interval, interval)]
                    starts, ends = rows.start.to_numpy(int), rows.end.to_numpy(int)
                    state = np.column_stack([rows[f'pred_{i}'] for i in range(15)])
                    predicted = np.broadcast_to(np.eye(5), (len(rows), 5, 5)).copy()
                    predicted[:, :3, :3] = state[:, :9].reshape(-1, 3, 3)
                    predicted[:, :3, 4] = state[:, 9:12]
                    predicted[:, :3, 3] = state[:, 12:15]
                    native_error = rows[[f'error_{i}' for i in range(9)]].to_numpy()
                    native_cov = rows[[f'cov_{i}_{j}' for i in range(9) for j in range(9)]].to_numpy().reshape(-1, 9, 9)
                    world = np.broadcast_to(np.eye(9), native_cov.shape).copy()
                    world[:, 3:6, 3:6] = predicted[:, :3, :3]
                    world[:, 6:9, 6:9] = predicted[:, :3, :3]
                    reporting_cov = world @ native_cov @ world.transpose(0, 2, 1)
                    # Only the physical error is needed from this helper.
                    errors, _ = physical_reporting(torch.tensor(predicted), torch.tensor(truth[ends]),
                                                   torch.eye(5).repeat(len(rows), 1, 1), torch.tensor(native_cov))
                    errors = errors.numpy()
                else:
                    starts, ends = result['starts'].cpu().numpy(), result['ends'].cpu().numpy()
                    predicted = result['predicted_endpoints'].cpu().numpy()
                    native_error = result['native_error'].cpu().numpy()
                    native_cov = result['native_covariance'].cpu().numpy()
                    reporting_cov = result['reporting_covariance'].cpu().numpy()
                    errors = result['physical_error'].cpu().numpy()
                duration = int(ends[0]-starts[0]) * streams[-1]
                drive = position_drive_covariance(len(starts), duration)
                native_drive = position_drive_covariance(len(starts), duration, 6 if cpp is None else 3)
                key = name, method, interval
                cache[key] = dict(starts=starts, ends=ends, start_times=times[starts], end_times=times[ends],
                                  error=errors, native_error=native_error, native_cov=native_cov,
                                  reporting_cov=reporting_cov, drive=drive, native_drive=native_drive,
                                  predicted=predicted)
                np.savez_compressed(output / f'{name}_{method}_{interval}.npz', **cache[key])
        print(f'Cached {name}', flush=True)
    return cache


def audit_reference(sources, baseline, output):
    """Diagnose GT position/velocity disagreement without deleting or masking windows."""
    summaries, windows = [], []
    metric = pd.read_csv(baseline / 'window_metrics.csv')
    for name, source in sources.items():
        raw = pd.read_csv(source)
        times = raw.t.to_numpy() - raw.t.iloc[0]
        dt = np.diff(times)
        if np.any(dt <= 0):
            raise ValueError(f'Non-increasing timestamp in {source}')
        position = raw[['p_x', 'p_y', 'p_z']].to_numpy()
        velocity = raw[['v_x', 'v_y', 'v_z']].to_numpy()
        discrepancy = np.linalg.norm(np.diff(position, axis=0) -
                                    .5*(velocity[:-1]+velocity[1:])*dt[:, None], axis=1)
        selected = metric[(metric.dataset == name) & (metric.method == 'galilean') &
                          (metric.interval_seconds == .2)].sort_values('normalized_nees', ascending=False)
        top = max(1, int(np.ceil(.01 * len(selected))))
        summaries.append(dict(dataset=name, median_step_discrepancy_m=np.median(discrepancy),
                              p99_step_discrepancy_m=np.quantile(discrepancy, .99),
                              max_step_discrepancy_m=discrepancy.max(),
                              steps_above_1cm=int((discrepancy > .01).sum()),
                              dt_min=dt.min(), dt_max=dt.max(),
                              top_window_count=top,
                              top_1pct_nees_fraction=selected.normalized_nees.iloc[:top].sum()/selected.normalized_nees.sum()))
        for _, row in selected.head(top).iterrows():
            start, end = int(row.window_start_sample), int(row.window_end_sample)
            index = start + np.argmax(discrepancy[start:end])
            windows.append(dict(dataset=name, window_start_time=row.window_start_time,
                                normalized_nees=row.normalized_nees, position_error_m=row.pos_error_norm,
                                reference_step_time=times[index], source_row=index+2,
                                next_source_row=index+3, step_dt=dt[index],
                                reference_step_discrepancy_m=discrepancy[index]))
    pd.DataFrame(summaries).to_csv(output/'reference_audit.csv', index=False)
    pd.DataFrame(windows).to_csv(output/'extreme_windows.csv', index=False)


def score_block(data, alpha, q):
    covariance = covariance_at(data['reporting_cov'], data['drive'], alpha, q)
    native = covariance_at(data['native_cov'], data['native_drive'], alpha, q)
    nees = normalized_nees(data['native_error'], native)
    errors = np.linalg.norm(data['error'].reshape(-1, 3, 3), axis=2)
    sigmas = np.sqrt(np.diagonal(covariance, axis1=1, axis2=2).reshape(-1, 3, 3).mean(axis=2))
    sign, logdet = np.linalg.slogdet(covariance)
    assert np.all(sign == 1)
    physical_mahal = np.einsum('ni,ni->n', data['error'], np.linalg.solve(covariance, data['error'][..., None])[..., 0])
    nll = .5 * (logdet + physical_mahal + 9*np.log(2*np.pi))
    return nees, errors, sigmas, nll


def validate_baseline(cache, baseline):
    """Require full-precision recomputation to reproduce the published baseline CSVs."""
    reference = pd.read_csv(baseline / 'window_metrics.csv')
    for (name, method, interval), data in cache.items():
        rows = reference[(reference.dataset == name) & (reference.method == method) &
                         (reference.interval_seconds == interval)].sort_values('window_index')
        np.testing.assert_array_equal(rows.window_start_sample, data['starts'])
        np.testing.assert_array_equal(rows.window_end_sample, data['ends'])
        nees, errors, sigmas, _ = score_block(data, 8.4, 1e-8)
        np.testing.assert_allclose(rows.normalized_nees, nees, atol=1e-9, rtol=5e-6)
        for index, block in enumerate(('rot', 'pos', 'vel')):
            np.testing.assert_allclose(rows[f'{block}_error_norm'], errors[:, index], atol=1e-9, rtol=5e-6)
            np.testing.assert_allclose(rows[f'{block}_pred_sigma'], sigmas[:, index], atol=1e-9, rtol=5e-6)


def validate_direct(binary, source, cache, alpha, q):
    """Check every held-out window against actual propagation at the fitted parameters."""
    name = source.stem.removeprefix('euroc_')
    for method in FIT_METHODS:
        rows = cpp_evaluation(binary, source, method, alpha, q)
        for interval in INTERVALS:
            selected = rows[np.isclose(rows.interval, interval)]
            actual = selected[[f'cov_{i}_{j}' for i in range(9) for j in range(9)]].to_numpy().reshape(-1, 9, 9)
            data = cache[name, method, interval]
            expected = covariance_at(data['native_cov'], data['native_drive'], alpha, q)
            np.testing.assert_allclose(actual, expected, atol=5e-11, rtol=1e-5)
            np.testing.assert_allclose(selected[[f'error_{i}' for i in range(9)]], data['native_error'], atol=1e-12, rtol=0)
    streams = load_ground_truth_euroc(str(source))
    for interval in INTERVALS:
        actual = evaluate_interval(streams, interval, alpha, q)
        data = cache[name, 'delama_gal3_python', interval]
        np.testing.assert_allclose(actual['native_covariance'].cpu(),
                                   covariance_at(data['native_cov'], data['native_drive'], alpha, q), atol=5e-11, rtol=1e-5)
        np.testing.assert_allclose(actual['predicted_endpoints'].cpu(), data['predicted'], atol=1e-12, rtol=0)


def write_csv(path, fields, rows):
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def publish(baseline, output_root, protocol, cache, fits, sources, report):
    app_name = 'evalGroupNoiseCalibration'
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    identity = dict(run_id=run_id, app_name=app_name)
    rows, summaries = [], []
    for (name, method, interval), data in cache.items():
        fit = fits[protocol, name]
        alpha, q = fit['alpha'], fit['integration_covariance']
        label = f'endpoint_v2_{protocol}_heldout_{name}_alpha{alpha:.17g}_q{q:.17g}'
        base = dict(**identity, dataset=name, method=method, config_label=label,
                    interval_seconds=interval, samples_per_window=int(data['ends'][0]-data['starts'][0]), quadrature_nodes=0)
        nees, errors, sigmas, _ = score_block(data, alpha, q)
        group = []
        for index in range(len(nees)):
            row = dict(**base, window_index=index, window_start_sample=int(data['starts'][index]),
                       window_end_sample=int(data['ends'][index]), window_start_time=data['start_times'][index],
                       window_end_time=data['end_times'][index], normalized_nees=nees[index],
                       rot_error_norm=errors[index, 0], rot_pred_sigma=sigmas[index, 0],
                       pos_error_norm=errors[index, 1], pos_pred_sigma=sigmas[index, 1],
                       vel_error_norm=errors[index, 2], vel_pred_sigma=sigmas[index, 2])
            group.append(row)
        rows.extend(group)
        summaries.append(summarize_rows(group))
    assert len(rows) == 28554 and len(summaries) == 99
    with tempfile.TemporaryDirectory(prefix='noise-calibration-staging-', dir=output_root.parent) as stage:
        package = Path(stage)/run_id
        package.mkdir()
        for filename in CANONICAL_FILES:
            fields, old_rows = read_csv(baseline/filename)
            if filename == 'window_metrics.csv':
                values = rows
            elif filename == 'window_summaries.csv':
                values = summaries
            elif filename == 'run_metadata.csv':
                values = [{**old_rows[0], **identity, 'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                           'cli_args': f'run_group_noise_calibration.py protocol={protocol}',
                           'output_root': str(output_root),
                           'repo_version': subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip()}]
            elif filename == 'datasets.csv':
                values = [{**row, **identity} for row in old_rows]
            else:
                values = []
            write_csv(package/filename, fields, values)
        verification = dict(protocol=protocol, convention='endpoint_v2', report=str(report),
                            methods=list(METHODS), fit_methods=list(FIT_METHODS),
                            folds=[fits[protocol, name] for name in sources],
                            validation=dict(metric_rows=len(rows), summary_rows=len(summaries),
                                            direct_propagation_at_fitted_parameters='passed',
                                            source_windows_preserved=True, no_exclusions=True,
                                            baseline_reproduction='passed at CSV precision'))
        (package/'verification.json').write_text(json.dumps(verification, indent=2)+'\n')
        from viewer.discovery import discover_runs
        from viewer.loading import load_run_data
        entry = discover_runs(package)[0]
        loaded = load_run_data(entry)
        assert len(loaded.window_metrics) == len(rows) and len(loaded.window_summaries) == len(summaries)
        destination=output_root/app_name/run_id
        destination.parent.mkdir(parents=True, exist_ok=True)
        package.rename(destination)
    return destination


def main():
    root = Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=root/'build/results/evalQuadratureImuFactorDiagnostics/20260911T211341359Z')
    parser.add_argument('--data-dir', type=Path, default=root/'data/euroc')
    parser.add_argument('--binary', type=Path, default=root/'build/tests/exportGalileanParity')
    parser.add_argument('--results-root', type=Path, default=root/'build/results')
    parser.add_argument('--output', type=Path)
    args=parser.parse_args()
    torch.set_num_threads(1)
    sources={p.stem.removeprefix('euroc_'):p.resolve() for p in sorted(args.data_dir.glob('euroc_*.csv'))}
    if set(sources) != EUROC_SEQUENCES or not args.binary.is_file():
        parser.error('Require all 11 bundled EuRoC sequences and built exportGalileanParity helper')
    validate_package(args.baseline, sources, BASELINE_METHODS)
    output=args.output or root/'build/noise-calibration'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output.mkdir(parents=True, exist_ok=False)
    cache_dir=output/'cache';cache_dir.mkdir()
    protocol=dict(training='LOSO: exclude entire held-out sequence at every interval',
                  groups={'MH':[n for n in sources if n.startswith('MH')], 'V':[n for n in sources if n.startswith('V')]},
                  weighting='equal sequence, equal interval, equal manifold/Galilean; Delama counted once via Galilean',
                  loss='Gaussian NLL on physical endpoint errors and reporting covariance, including log determinant',
                  baseline=dict(alpha=8.4, integration_covariance=1e-8), no_window_exclusions=True,
                  source_sha256={n:hashlib.sha256(p.read_bytes()).hexdigest() for n,p in sources.items()},
                  implementation_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in [Path(__file__), root/'python/imuFactors/noise_calibration.py',
                                root/'tests/exportGalileanParity.cpp']})
    (output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    audit_reference(sources, args.baseline, output)
    cache=prepare_cache(args.binary, sources, cache_dir)
    validate_baseline(cache, args.baseline)
    blocks={key:LikelihoodBlock.from_arrays(d['error'],d['reporting_cov'],d['drive'])
            for key,d in cache.items() if key[1] in FIT_METHODS}
    fits={}; scores=[]
    for protocol_name in ('global_loso','group_loso'):
        for held_out in sources:
            training=training_sequences(sources, held_out, protocol_name)
            selected=[block for key,block in blocks.items() if key[0] in training]
            fit=fit_noise(selected)
            fit.update(protocol=protocol_name, held_out=held_out, group='MH' if held_out.startswith('MH') else 'V',
                       training_sequences=training, training_blocks=len(selected),
                       baseline_training_nll=float(likelihood(selected,8.4,1e-8)))
            fits[protocol_name,held_out]=fit
            print(json.dumps(fit),flush=True)
            validate_direct(args.binary,sources[held_out],cache,fit['alpha'],fit['integration_covariance'])
    for setting in ('baseline','global_loso','group_loso'):
        for (name,method,interval),data in cache.items():
            fit=dict(alpha=8.4,integration_covariance=1e-8) if setting=='baseline' else fits[setting,name]
            nees,_,_,nll=score_block(data,fit['alpha'],fit['integration_covariance'])
            scores.append(dict(protocol=setting,dataset=name,group='MH' if name.startswith('MH') else 'V',
                               method=method,interval_seconds=interval,num_windows=len(nees),
                               alpha=fit['alpha'],integration_covariance=fit['integration_covariance'],
                               normalized_nees_mean=nees.mean(),normalized_nees_median=np.median(nees),
                               normalized_nees_p95=np.sort(nees)[int(.95*(len(nees)-1))],
                               upper_95_ellipsoid_coverage=np.mean(nees<=chi2.ppf(.95,9)/9),
                               gaussian_nll=nll.mean()))
    pd.DataFrame(scores).to_csv(output/'heldout_scores.csv',index=False)
    (output/'folds.json').write_text(json.dumps(list(fits.values()),indent=2)+'\n')
    pd.DataFrame([{**f,'training_sequences':','.join(f['training_sequences'])} for f in fits.values()]).to_csv(output/'folds.csv',index=False)
    packages={p:str(publish(args.baseline,args.results_root,p,cache,fits,sources,output)) for p in ('global_loso','group_loso')}
    (output/'packages.json').write_text(json.dumps(packages,indent=2)+'\n')
    from imuFactors.calibration_report import write_calibration_report
    write_calibration_report(output, sources)
    print(f'Calibration completed: {output}',flush=True)
    print(json.dumps(packages),flush=True)


if __name__=='__main__':
    main()
