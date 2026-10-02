"""Rerun the fixed MH/V comparison, optionally including quadrature for the viewer."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

from run_unified_imu_comparison import (
    CANONICAL_FILES, CPP_METHODS, EUROC_SEQUENCES, INTERVALS, validate_package,
)

METHODS = ('manifold', 'galilean', 'delama_gal3_python')
GROUP_SETTINGS = {
    'MH': {'alpha': 9.85, 'integration_covariance': 4e-5},
    'V': {'alpha': 16.0, 'integration_covariance': 0.0},
}


def settings_for_dataset(name, group_settings=None):
    if name not in EUROC_SEQUENCES:
        raise ValueError(f'Unknown EuRoC sequence: {name}')
    settings = GROUP_SETTINGS if group_settings is None else group_settings
    return dict(settings['MH' if name.startswith('MH') else 'V'])


def validate_settings(settings):
    if set(settings) != {'MH', 'V'}:
        raise ValueError('Require exactly MH and V settings')
    for values in settings.values():
        if set(values) not in ({'alpha', 'integration_covariance'},
                              {'alpha_gyro', 'alpha_acc', 'integration_covariance'}) or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in values.values()):
            raise ValueError('Require finite alpha and integration_covariance')
        if (any(value <= 0 for name, value in values.items() if name != 'integration_covariance')
                or values['integration_covariance'] < 0):
            raise ValueError('Require alpha > 0 and integration_covariance >= 0')


def write_csv(path, fields, rows):
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run_comparison(binary, data_dir, results_root, threads=1, calibration=None,
                   include_quadrature=False):
    methods = ('quadrature', *METHODS) if include_quadrature else METHODS
    expected_metrics, expected_summaries = 9518 * len(methods), 33 * len(methods)
    group_settings = GROUP_SETTINGS if calibration is None else calibration['settings']
    validate_settings(group_settings)
    protocol = 'fixed_group_refit' if calibration is None else calibration['protocol']
    if protocol not in ('fixed_group_refit', 'group_mean_median', 'group_separate_mean_median'):
        raise ValueError(f'Unknown calibration protocol: {protocol}')
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise FileNotFoundError(f'Missing executable C++ binary: {binary}')
    import torch
    from imuFactors.delama_gal3.preintegration_delama_gal3 import (
        configuration_label, evaluate_interval, load_ground_truth_euroc,
    )
    from imuFactors.delama_gal3.canonical import canonical_rows

    torch.set_num_threads(threads)
    sources = {p.stem.removeprefix('euroc_'): p.resolve()
               for p in sorted(data_dir.glob('euroc_*.csv'))}
    if set(sources) != EUROC_SEQUENCES:
        raise ValueError('Require exactly the 11 bundled EuRoC sequences')
    configs = {name: configuration_label(**settings_for_dataset(name, group_settings)) for name in sources}
    if calibration is not None and calibration['source_sha256'] != {
            name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in sources.items()}:
        raise ValueError('Calibration source CSV hashes do not match')
    results_root = results_root.resolve()
    results_root.parent.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    identity = dict(run_id=run_id, app_name=('evalMedianGroupImuComparison'
                    if protocol == 'group_mean_median' else 'evalFixedGroupImuComparison'))
    if protocol == 'group_separate_mean_median':
        identity['app_name'] = 'evalSeparateNoiseGroupImuComparison'
    if include_quadrature:
        identity['app_name'] += 'WithQuadrature'
    source_hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in sources.items()}
    with tempfile.TemporaryDirectory(prefix='fixed-group-staging-', dir=results_root.parent) as staging:
        staging = Path(staging)
        package = staging/'combined'
        package.mkdir()
        combined = {name: [] for name in CANONICAL_FILES}
        headers = {}
        cpp_metadata = []
        for name, source in sources.items():
            settings = settings_for_dataset(name, group_settings)
            cpp_root = staging/name
            command = [str(binary.resolve()), '--dataset', name, '--data-dir', str(data_dir.resolve()),
                       '--integration-covariance',
                       repr(settings['integration_covariance']), '--output-root', str(cpp_root)]
            if 'alpha' in settings:
                command += ['--alpha', repr(settings['alpha'])]
            else:
                command += ['--alpha-gyro', repr(settings['alpha_gyro']),
                            '--alpha-acc', repr(settings['alpha_acc'])]
            subprocess.run(command, check=True)
            metadata_files = list(cpp_root.rglob('run_metadata.csv'))
            if len(metadata_files) != 1:
                raise ValueError(f'Expected one C++ package for {name}')
            metadata, tables = validate_package(metadata_files[0].parent, {name: source}, CPP_METHODS,
                                                 expected_configs={name: configs[name]})
            cpp_metadata.append(metadata)
            for filename, (fields, values) in tables.items():
                if filename in headers and headers[filename] != fields:
                    raise ValueError(f'Inconsistent canonical header: {filename}')
                headers[filename] = fields
                if filename == 'run_metadata.csv':
                    continue
                for row in values:
                    if filename in ('window_metrics.csv', 'window_summaries.csv') and row['method'] not in methods:
                        continue
                    combined[filename].append({**row, **identity})
            streams = load_ground_truth_euroc(str(source))
            for interval in INTERVALS:
                result = evaluate_interval(streams, interval, **settings)
                rows, summary = canonical_rows(result, run_id, identity['app_name'], name)
                combined['window_metrics.csv'].extend(rows)
                combined['window_summaries.csv'].append(summary)
            print(f'Completed fixed settings: {name}, {settings}', flush=True)
        combined['run_metadata.csv'] = [{
            **cpp_metadata[0], **identity, 'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'cli_args': 'run_fixed_group_imu_comparison.py '
                        + ('--include-quadrature ' if include_quadrature else '')
                        + json.dumps(group_settings, sort_keys=True),
            'output_root': str(results_root),
            'repo_version': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        }]
        for filename in CANONICAL_FILES:
            write_csv(package/filename, headers[filename], combined[filename])
        validate_package(package, sources, methods, expected_configs=configs)
        if (len(combined['window_metrics.csv']) != expected_metrics or
                len(combined['window_summaries.csv']) != expected_summaries):
            raise ValueError('Unexpected full-comparison row counts')
        if any(hashlib.sha256(path.read_bytes()).hexdigest() != source_hashes[name] for name,path in sources.items()):
            raise ValueError('Source CSV changed during the run')
        verification = dict(
            convention='endpoint_v2', protocol=protocol, settings=group_settings,
            parameter_selection=('Rounded full-group Gaussian-NLL refits after LOSO protocol comparison; these rows are descriptive, not held-out'
                                 if calibration is None else calibration['parameter_selection']),
            methods=list(methods), intervals=list(INTERVALS), source_sha256=source_hashes,
            validation=dict(metric_rows=expected_metrics, summary_rows=expected_summaries,
                            method_interval_window_coverage='passed',
                            source_csvs_unchanged=True, all_methods_directly_rerun=True, no_window_exclusions=True),
        )
        if protocol in ('group_mean_median', 'group_separate_mean_median'):
            verification['fit_methods'] = ['manifold', 'galilean']
            verification['quadrature_in_calibration'] = False
            from statistics import mean, median
            medians = {}
            for name in sources:
                for interval in INTERVALS:
                    for method in ('manifold', 'galilean'):
                        values = [float(row['normalized_nees']) for row in combined['window_metrics.csv']
                                  if row['dataset'] == name and row['method'] == method
                                  and float(row['interval_seconds']) == interval]
                        medians[name, interval, method] = median(values)
            achieved = {group: mean(value for (name, _, _), value in medians.items()
                                    if name.startswith(group)) for group in group_settings}
            if any(abs(value - 1.) > 1e-5 for value in achieved.values()):
                raise ValueError(f'Mean-median target failed after direct rerun: {achieved}')
            verification['validation']['mean_of_block_medians'] = achieved
        if calibration is not None:
            (package/'calibration.json').write_text(json.dumps(calibration, indent=2)+'\n')
        (package/'verification.json').write_text(json.dumps(verification, indent=2)+'\n')
        (package/'cpp_source_runs.json').write_text(json.dumps(cpp_metadata, indent=2)+'\n')
        destination = results_root/identity['app_name']/run_id
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            raise FileExistsError(destination)
        package.rename(destination)
    print(f'Fixed-group comparison published: {destination}', flush=True)
    return destination


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=root/'build/evalQuadratureImuFactorDiagnostics')
    parser.add_argument('--data-dir', type=Path, default=root/'data/euroc')
    parser.add_argument('--results-root', type=Path, default=root/'build/results')
    parser.add_argument('--threads', type=int, default=1)
    parser.add_argument('--calibration-json', type=Path,
                        help='Use a calibrated MH/V settings document instead of the original defaults')
    parser.add_argument('--include-quadrature', action='store_true',
                        help='Publish all four methods using the same settings without refitting noise')
    args = parser.parse_args()
    try:
        calibration = json.loads(args.calibration_json.read_text()) if args.calibration_json else None
        run_comparison(args.binary, args.data_dir, args.results_root, args.threads,
                       calibration, args.include_quadrature)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Fixed-group comparison failed: {error}\n')


if __name__ == '__main__':
    main()
