"""Run and atomically publish one four-method EuRoC factor comparison."""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import os
import subprocess
import tempfile

INTERVALS = (0.2, 0.5, 1.0)
CPP_METHODS = ("quadrature", "manifold", "galilean")
METHODS = (*CPP_METHODS, "delama_gal3_python")
EUROC_SEQUENCES = {"MH01", "MH02", "MH03", "MH04", "MH05", "V101", "V102", "V103", "V201", "V202", "V203"}
CANONICAL_FILES = ("run_metadata.csv", "datasets.csv", "window_metrics.csv", "window_summaries.csv",
                   "trajectory_samples.csv", "calibration_trials.csv", "calibration_summaries.csv")


def read_csv(path):
    if not path.is_file():
        raise ValueError(f"Missing canonical file: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields, rows = reader.fieldnames, list(reader)
    if not fields or any(None in row or None in row.values() for row in rows):
        raise ValueError(f"Invalid canonical CSV: {path}")
    return fields, rows


def validate_package(package, datasets, methods, integration_covariance=1e-8):
    """Require exact method/interval coverage and matching complete windows."""
    import math
    from imuFactors.delama_gal3.canonical import WINDOW_FIELDS, SUMMARY_FIELDS

    from imuFactors.delama_gal3.preintegration_delama_gal3 import configuration_label, integration_covariance_value
    integration_covariance = integration_covariance_value(integration_covariance)
    tables = {name: read_csv(package / name) for name in CANONICAL_FILES}
    metadata = tables["run_metadata.csv"][1]
    if len(metadata) != 1 or not metadata[0].get("run_id") or not metadata[0].get("app_name"):
        raise ValueError("Expected exactly one canonical run metadata row")
    identity = metadata[0]
    membership = tables["datasets.csv"][1]
    if len(membership) != len(datasets) or {row["dataset"] for row in membership} != set(datasets):
        raise ValueError("C++ dataset membership does not match requested EuRoC datasets")
    for row in membership:
        if Path(row["source_path"]).resolve() != datasets[row["dataset"]].resolve():
            raise ValueError(f"C++ source path mismatch: {row['dataset']}")
    for name, (_, rows) in tables.items():
        for row in rows:
            if any(row.get(key) != identity[key] for key in ("run_id", "app_name")):
                raise ValueError(f"Run identity mismatch in {name}")
    for name, fields in (("window_metrics.csv", WINDOW_FIELDS), ("window_summaries.csv", SUMMARY_FIELDS)):
        if tables[name][0] != fields:
            raise ValueError(f"Unexpected canonical header in {name}: {tables[name][0]}")
    expected = {(dataset, method, interval) for dataset in datasets for method in methods for interval in INTERVALS}
    grouped = defaultdict(list)
    for row in tables["window_metrics.csv"][1]:
        grouped[row["dataset"], row["method"], float(row["interval_seconds"])].append(row)
    summaries = tables["window_summaries.csv"][1]
    summary_keys = [(row["dataset"], row["method"], float(row["interval_seconds"])) for row in summaries]
    if set(grouped) != expected or set(summary_keys) != expected or len(summary_keys) != len(expected):
        raise ValueError(f"Missing or unexpected method/interval coverage; expected {len(expected)} groups, got {len(grouped)} metrics / {len(summaries)} summaries")
    for row in tables["window_metrics.csv"][1] + summaries:
        if row["config_label"] != configuration_label(8.4, integration_covariance):
            raise ValueError("Expected endpoint_v2, uniform alpha=8.4 and requested integration covariance")
        for field, value in row.items():
            if field not in ("run_id", "app_name", "dataset", "method", "config_label") and not math.isfinite(float(value)):
                raise ValueError(f"Non-finite canonical value: {field}")
    for summary, key in zip(summaries, summary_keys):
        if int(summary["num_windows"]) != len(grouped[key]):
            raise ValueError(f"Summary window count mismatch: {key}")
    # Validate against the source stream, not merely agreement among methods.
    for dataset, source in datasets.items():
        with source.open(newline="") as handle:
            reader = csv.reader(handle)
            next(reader)
            times = [float(row[0]) for row in reader]
        if len(times) < 2 or not all(math.isfinite(time) for time in times):
            raise ValueError(f"Invalid EuRoC timestamps: {source}")
        dt = times[1] - times[0]
        if dt <= 0:
            raise ValueError(f"Invalid EuRoC timestep: {source}")
        for interval in INTERVALS:
            steps = max(1, math.floor(interval / dt + .5))
            starts = list(range(0, len(times) - steps, steps))
            for method in methods:
                rows = grouped[dataset, method, interval]
                if len(rows) != len(starts):
                    raise ValueError(f"Incomplete window coverage: {dataset}/{method}/{interval}")
                for index, (row, start) in enumerate(zip(rows, starts)):
                    expected_ints = {"window_index": index, "samples_per_window": steps,
                                     "window_start_sample": start, "window_end_sample": start + steps}
                    if any(int(row[field]) != value for field, value in expected_ints.items()):
                        raise ValueError(f"Window boundary mismatch: {dataset}/{method}/{interval}/{index}")
                    for field, sample in (("window_start_time", start), ("window_end_time", start + steps)):
                        if not math.isclose(float(row[field]), times[sample] - times[0], rel_tol=5e-6, abs_tol=1e-6):
                            raise ValueError(f"Window timestamp mismatch: {dataset}/{method}/{interval}/{index}")
    return identity, tables


def run_comparison(binary, data_dir, results_root, dataset=None, threads=1, integration_covariance=1e-8):
    """Stage C++, append Delama, validate, then rename into viewer discovery."""
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise FileNotFoundError(f"Missing executable C++ binary: {binary}; build evalQuadratureImuFactorDiagnostics first")
    try:
        import torch
    except ImportError as error:
        raise RuntimeError("PyTorch is required; install torch in the py312 conda environment") from error
    from imuFactors.delama_gal3.preintegration_delama_gal3 import load_ground_truth_euroc, evaluate_interval, integration_covariance_value
    from imuFactors.delama_gal3.canonical import canonical_rows

    integration_covariance = integration_covariance_value(integration_covariance)
    torch.set_num_threads(threads)
    datasets = {path.stem.removeprefix("euroc_"): path.resolve()
                for path in sorted(data_dir.glob("euroc_*.csv")) if path.is_file()}
    requested = {Path(dataset).stem.removeprefix("euroc_")} if dataset else EUROC_SEQUENCES
    if not requested <= datasets.keys():
        raise FileNotFoundError(f"Missing EuRoC datasets in {data_dir}: {sorted(requested - datasets.keys())}")
    if dataset:
        datasets = {name: datasets[name] for name in requested}
    results_root = results_root.resolve()
    results_root.parent.mkdir(parents=True, exist_ok=True)
    # Keep incomplete packages outside results_root, whose viewer scans recursively.
    with tempfile.TemporaryDirectory(prefix="imu-comparison-staging-", dir=results_root.parent) as staging:
        command = [str(binary.resolve()), "--alpha", "8.4", "--integration-covariance", repr(integration_covariance), "--data-dir", str(data_dir.resolve()), "--output-root", staging]
        if dataset:
            command += ["--dataset", next(iter(requested))]
        subprocess.run(command, check=True)
        packages = list(Path(staging).rglob("run_metadata.csv"))
        if len(packages) != 1:
            raise ValueError(f"Expected exactly one C++ package, found {len(packages)}")
        package = packages[0].parent
        identity, tables = validate_package(package, datasets, CPP_METHODS, integration_covariance)
        with (package / "window_metrics.csv").open("a", newline="") as metrics_handle, (package / "window_summaries.csv").open("a", newline="") as summaries_handle:
            metrics_writer = csv.DictWriter(metrics_handle, fieldnames=tables["window_metrics.csv"][0])
            summaries_writer = csv.DictWriter(summaries_handle, fieldnames=tables["window_summaries.csv"][0])
            for name, source in datasets.items():
                streams = load_ground_truth_euroc(str(source))
                for interval in INTERVALS:
                    result = evaluate_interval(streams, interval, 8.4, integration_covariance)
                    rows, summary = canonical_rows(result, identity["run_id"], identity["app_name"], name)
                    metrics_writer.writerows(rows)
                    summaries_writer.writerow(summary)
                    print(f"Delama {name} {interval:.1f}s: {len(rows)} windows", flush=True)
        _, completed = validate_package(package, datasets, METHODS, integration_covariance)
        verification = {
            "convention": "endpoint_v2", "alpha": 8.4,
            "integration_covariance": integration_covariance,
            "integration_covariance_units": "m^2/s (independent position drive)",
            "error": "[Log(R_pred^T R_gt), p_gt-p_pred, v_gt-v_pred]",
            "sigma": "sqrt(trace(reporting covariance block)/3)",
            "nees": "native residual/covariance, diagonal regularization 1e-12, divided by 9",
            "quaternions": "normalized; reject nonfinite or norm <= 1e-12; source CSVs unchanged",
            "validation": {"method_interval_window_coverage": "passed",
                           "metric_rows": len(completed["window_metrics.csv"][1]),
                           "summary_rows": len(completed["window_summaries.csv"][1]),
                           "datasets": sorted(datasets), "methods": list(METHODS)},
        }
        (package / "verification.json").write_text(json.dumps(verification, indent=2) + "\n")
        destination = results_root / identity["app_name"] / identity["run_id"]
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            raise FileExistsError(f"Result package already exists: {destination}")
        package.rename(destination)
    print(f"Unified comparison published: {destination}", flush=True)
    return destination


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=root / "build/evalQuadratureImuFactorDiagnostics")
    parser.add_argument("--data-dir", type=Path, default=root / "data/euroc")
    parser.add_argument("--results-root", type=Path, default=root / "build/results")
    parser.add_argument("--dataset", help="One sequence for smoke testing, e.g. MH01; default: all 11")
    parser.add_argument("--threads", type=int, default=1, help="PyTorch CPU threads (default: 1)")
    parser.add_argument("--integration-covariance", type=float, default=1e-8,
                        help="Continuous position-drive covariance in m²/s (default: 1e-8).")
    args = parser.parse_args()
    try:
        run_comparison(args.binary, args.data_dir, args.results_root, args.dataset, args.threads, args.integration_covariance)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Unified comparison failed: {error}\n")


if __name__ == "__main__":
    main()
