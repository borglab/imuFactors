"""Export matched EuRoC inputs and run the native C++ integration benchmark.

Python only prepares inputs and summarizes CSV output; every timed operation is
C++. Run in py312 with the private wrapper on PYTHONPATH, after building the
benchmarkFastRodFIter Release target. No manuscript files are changed.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess

import numpy as np
import pandas as pd

import generate_taes_euroc_snippet_experiment_report as report


ASSETS = Path(__file__).resolve().parent / "fast_rodfiter_benchmark_assets"
QMD = Path(__file__).resolve().with_name("fast_rodfiter_benchmark.qmd")


def fingerprint(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def export_euroc(path: Path) -> None:
    cache_path = report.chebyshev_report.EUROC_CACHE_PATH
    cache = report.chebyshev_report.load_aggressive_window_cache(cache_path)
    if not report.chebyshev_report.cache_is_current(cache):
        raise ValueError("Existing snippet cache is missing or out of date")
    snippets = report.load_proxy_snippets(
        cache, data_dir=report.DATA_DIR,
        reference_N=report.psych_report.REFERENCE_N,
        middle_interval=report.psych_report.MIDDLE_INTERVAL,
    )
    pd.testing.assert_frame_equal(
        report.proxy_metadata(snippets),
        pd.read_csv(report.ASSET_DIR / "proxy_snippet_metadata.csv"),
        check_dtype=False, check_exact=False, rtol=1e-9, atol=1e-12,
    )
    rows = []
    for index, snippet in enumerate(snippets):
        times = np.linspace(*snippet.middle_interval, 41)
        clean = report.evaluate_proxy_gyro(snippet, times)
        noise = report.standard_noise(report.psych_report.RANDOM_SEED, index, 41, 1)[0]
        quaternion = snippet.reference_rotation.toQuaternion()
        for fraction in (0.0, 0.1, 0.225):
            measured = clean + fraction * snippet.noise_scale * noise
            for sample, (time, omega) in enumerate(zip(times, measured)):
                rows.append([snippet.dataset, fraction, sample, time - times[0], *omega,
                             quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()])
    pd.DataFrame(rows, columns=["dataset", "noise_fraction", "sample_index", "time_s",
                               "wx", "wy", "wz", "qw", "qx", "qy", "qz"]).to_csv(path, index=False)


def summary_table(metrics: pd.DataFrame, stopping: str, mode: str) -> str:
    selected = metrics[metrics["stopping"].eq(stopping) & metrics["mode"].eq(mode)]
    selected = selected.assign(group=np.where(
        selected["scenario"].str.contains("_noise"), "EuRoC (33 windows)", selected["scenario"]
    ))
    medians = selected.groupby(["group", "method"])["median_us"].median().unstack()
    # Compute ratios per matched window before aggregating, not ratios of medians.
    paired = selected.pivot(index=["group", "scenario"], columns="method", values="median_us")
    medians["Rod/approx"] = (paired["FastRodFIter"] / paired["PS-CPI_approx"]).groupby("group").median()
    medians["Rod/exact"] = (paired["FastRodFIter"] / paired["PS-CPI_exact"]).groupby("group").median()
    return medians.reset_index().to_markdown(index=False, floatfmt=".3f")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=report.REPO_ROOT / "build/benchmarkFastRodFIter")
    parser.add_argument("--repeats", type=int, default=500)
    args = parser.parse_args()
    ASSETS.mkdir(exist_ok=True)
    fixture = ASSETS / "euroc_inputs.csv"
    export_euroc(fixture)
    command = [str(args.binary.resolve()), str(args.repeats), str(fixture)]
    print("Running native benchmark:", " ".join(command), flush=True)
    result = subprocess.run(command, text=True, capture_output=True, check=True)
    results_path = ASSETS / "timings.csv"
    results_path.write_text(result.stdout)
    metrics = pd.read_csv(results_path)
    if len(metrics) != 36 * 2 * 3 * 3 or not np.isfinite(
        metrics[["median_us", "residual", "error_rad"]].to_numpy()
    ).all():
        raise ValueError("Incomplete or invalid benchmark output")
    provenance = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": command, "platform": platform.platform(),
        "hardware": subprocess.check_output(["sysctl", "-n", "machdep.cpu.brand_string"], text=True).strip(),
        "compiler": subprocess.check_output(["/usr/bin/c++", "--version"], text=True),
        "binary_sha256": fingerprint(args.binary),
        "linkage": subprocess.check_output(["otool", "-L", str(args.binary)], text=True),
        "gtsam_source_commit": subprocess.check_output(
            ["git", "-C", "/Users/dellaert/git/gtsam-private", "rev-parse", "HEAD"], text=True).strip(),
        "gtsam_source_status": subprocess.check_output(
            ["git", "-C", "/Users/dellaert/git/gtsam-private", "status", "--short"], text=True),
        "runtime_search_paths": subprocess.check_output(["otool", "-l", str(args.binary)], text=True),
        "gtsam_library_sha256": fingerprint(Path("/Users/dellaert/git/gtsam-private/build/gtsam/libgtsam.4.3a2.dylib")),
        "source_sha256": {name: fingerprint(report.REPO_ROOT / name) for name in (
            "src/FastRodFIter.h", "src/FastRodFIter.cpp", "apps/benchmarkFastRodFIter.cpp")},
        "inputs_sha256": fingerprint(fixture), "timings_sha256": fingerprint(results_path),
        "batch_count": 15, "repeats_per_batch": args.repeats, "warmup_calls": 30,
        "checksum": result.stderr.strip(),
    }
    (ASSETS / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    tables = []
    for stopping in ("converged", "fixed7_zero"):
        for mode in ("prepared", "cached_fit", "fresh_setup"):
            tables.append(f"## {stopping}: {mode}\n\n" + summary_table(metrics, stopping, mode))
    accuracy = metrics[metrics["stopping"].eq("converged") & metrics["mode"].eq("prepared")]
    accuracy = accuracy.assign(group=np.where(accuracy["scenario"].str.contains("_noise"),
                                             "EuRoC (33 windows)", accuracy["scenario"]))
    accuracy_summary = accuracy.groupby(["group", "method"]).agg(
        min_iterations=("iterations", "min"), max_iterations=("iterations", "max"),
        max_error_rad=("error_rad", "max"), median_error_rad=("error_rad", "median"),
    ).reset_index().to_markdown(index=False, floatfmt=".6g")
    euroc = metrics[metrics["scenario"].str.contains("_noise") & metrics["stopping"].eq("converged")]
    medians = euroc.groupby(["mode", "method"])["median_us"].median()
    prepared_rod = medians.loc[("prepared", "FastRodFIter")]
    prepared_ps = medians.loc[("prepared", "PS-CPI_approx")]
    fresh_rod = medians.loc[("fresh_setup", "FastRodFIter")]
    fresh_ps = medians.loc[("fresh_setup", "PS-CPI_approx")]
    QMD.write_text(f'''---
title: "Native Fast RodFIter versus private GTSAM PS-CPI"
format: html
---

# Main result

With reusable setup, our current approximate-matrix PS-CPI kernel is faster:
median {prepared_ps:.2f} microseconds versus {prepared_rod:.2f} for Fast RodFIter
over the 33 EuRoC inputs. Rebuilding everything per call reverses that ordering:
{fresh_ps:.2f} microseconds for PS-CPI versus {fresh_rod:.2f} for Fast RodFIter.
The distinction is important: kernel throughput and one-shot end-to-end latency
are different comparisons. The older paper table's Python-wrapper timings include
different overhead and are not directly comparable to these native C++ measurements.

# Scope and implementation

This local C++ implementation follows Wu, Cai, and Truong, *Fast RodFIter for
Attitude Reconstruction From Inertial Measurements*, TAES 55(1), 2019,
doi:10.1109/TAES.2018.2866034, equations (1), (11), (12), and (17).
It is an independent implementation, not the authors' MATLAB code.
It represents `g = 2 tan(theta/2) axis` in first-kind Chebyshev coefficients,
forms exact polynomial products, integrates from the interval start, then truncates
to attitude degree `nT=n+1`. Both intermediate products in `g(g^T omega)` retain
their full degree. Associating these products avoids the literal triple sum and
reduces its arithmetic cost to quadratic in comparable polynomial degrees.
Constants from all integrated high-degree terms are retained before truncation.

The baseline is `gtsam::PsyChMagPI<gtsam::Rot3>` directly from gtsam-private,
not a Python reimplementation. Approximate mode is the historical `N -> N`
integration matrix used in the EuRoC manuscript tables; exact mode uses `N -> N+1`.
Neither method collects iteration history inside timed regions. Both return a final
rotation. Neither computes covariance or additional dense output.

# Matched inputs and timing boundaries

Both methods use the **same least-squares gyro polynomial**. The coefficient fit is
transformed to CGL nodal values for PS-CPI. The 33 EuRoC windows are the eleven
existing smooth proxies, each at noise fractions 0, 0.1 and 0.225, using seed index 0
from the 41-sample experiment's deterministic noise stream. They use 200 ms,
41 samples at 200 Hz, gyro degree 12 (13 coefficients/nodes), and Fast RodFIter
attitude degree 13. This is a speed benchmark, not a new Monte Carlo accuracy study.
Fast RodFIter retains 14 state coefficients; PS-CPI retains 13 state nodes in
approximate mode and 14 in exact mode. Matching the gyro fit does not make their
finite-dimensional state truncation or nonlinear aliasing identical.
The clean-proxy reference rotations use the existing independent DOP853 integration.

Additional analytic cases are Wu's 8 angular increments at 100 Hz over 80 ms
(gyro degree 7, attitude degree 8), 41 instantaneous coning-rate samples at 200 Hz,
and a faster coning case with angular frequency 10 pi rad/s. All coning angles are
10 degrees; analytic relative rotations supply their reference.

- **prepared**: integration only; fitted gyro inputs and integrator objects reused.
- **cached_fit**: apply precomputed sample-to-coefficient/node maps, then integrate;
  integrator objects reused.
- **fresh_setup**: construct and factor the common fitting matrix, fit the data,
  construct each integrator (and the PS-CPI nodal transform), then integrate.
- **converged**: tolerance 1e-12, maximum 50 updates; RodFIter starts at zero,
  while PS-CPI uses the production RK2 initializer. RodFIter checks the sum of
  coefficient-vector change norms; PS-CPI checks the nodal Frobenius change norm.
  These are different stopping norms and parameterizations, not equal error bounds.
- **fixed7_zero**: exactly seven updates from zero for both methods, with no early
  stopping; use the accompanying errors to judge adequacy.

Native Release builds use `-O3 -DNDEBUG` on {provenance['hardware']}.
Each row records the median and empirical 10th/90th-percentile order statistics
of 15 batches of {args.repeats} calls, after 30 warmups. A consumed checksum prevents
dead-code removal. Method order is shuffled within each scenario/mode. These are
single-machine microbenchmarks, not confidence intervals or hardware-independent
speed guarantees. Tables below aggregate per-window medians; matched ratios are
aggregated separately. Runtime columns are in microseconds per call.
**Rod/PS greater than one means PS-CPI is faster.**

{(chr(10) * 2).join(tables)}

# Accuracy and convergence checks (untimed)

All converged runs must satisfy their own stopping criterion; otherwise the benchmark
aborts. Angle errors below are SO(3) geodesic errors in radians. Different truncation
and aliasing conventions can lead to different converged solutions.

{accuracy_summary}

# Reproduction and artifacts

Build with `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`, then
`make -C build -j6 testFastRodFIter.run benchmarkFastRodFIter`.
Run `reports/generate_fast_rodfiter_benchmark.py --repeats {args.repeats}` with the
py312 conda environment and a working private GTSAM wrapper on `PYTHONPATH`.
Python prepares inputs and summarizes output only; all timing is in C++.
Raw inputs, all timings, and binary/source/library hashes are in
`fast_rodfiter_benchmark_assets/`; these generated assets are ignored by Git.
The C++ executable also runs the three analytic cases standalone, with an optional
second argument supplying the EuRoC input CSV.
''')
    print(summary_table(metrics, "converged", "prepared"))
    print("Wrote", QMD)


if __name__ == "__main__":
    main()
