"""Compare private GTSAM's pseudoinverse and exact-nodal PS-CPI on EuRoC.

Run with: conda run -n lidar python reports/run_taes_nodal_monte_carlo.py
Each invocation creates a new timestamped directory; existing results and the
manuscript are never written. --render-only regenerates HTML from a completed run.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

import gtsam
import numpy as np
import pandas as pd

REPORT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(REPORT_DIR.parent / "python"))

from imuFactors import spectral
from imuFactors.gyro_integration import rotation_error_degrees
from taes_euroc_endpoint_table import endpoint_rms_by_exact_noise


OUTPUT_ROOT = REPORT_DIR / "taes_nodal_monte_carlo_assets"
KEYS = ["dataset", "m", "noise_fraction", "seed_index"]
MODES = ("PSEUDOINVERSE", "EXACT_NODAL")
SAMPLE_COUNTS = (10, 20, 30, 40, 41, 50)
DEG_TO_MRAD = np.pi * 1000 / 180


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def make_integrator(count: int, interval: tuple[float, float], mode: str):
    """Select an explicit square mode, never the legacy boolean overload."""
    selected = getattr(gtsam.ChebyshevIntegrationMode, mode)
    params = gtsam.PseudoSpectralPiParams(
        gtsam.PseudoSpectralPiInit.RK2, 1e-12, 50, selected
    )
    if params.useApproximateIntegrationMatrix or params.getIntegrationMode() != selected:
        raise RuntimeError("Explicit integration mode was not preserved by wrapper")
    return gtsam.PsyChMagPIRot3(count, *interval, params)


def rms_mrad(errors: pd.Series) -> float:
    return float(np.sqrt(np.mean(np.square(errors))) * DEG_TO_MRAD)


def paired_trials(metrics: pd.DataFrame) -> pd.DataFrame:
    """Require a complete one-to-one pairing before computing differences."""
    if metrics.duplicated(KEYS + ["mode"]).any():
        raise ValueError("Duplicate paired trial identity")
    if set(metrics["mode"]) != set(MODES):
        raise ValueError("Both integration modes are required")
    paired = metrics.pivot(index=KEYS, columns="mode", values="error_deg")
    if paired.isna().any().any() or not np.isfinite(paired.to_numpy()).all():
        raise ValueError("Missing or non-finite paired errors")
    return paired.reset_index()


def exact_noise_summary(metrics: pd.DataFrame, baseline: pd.DataFrame) -> pd.DataFrame:
    """RMS errors and paired win fractions, separately for every M and noise."""
    paired = paired_trials(metrics)
    records = []
    for (count, noise), rows in paired.groupby(["m", "noise_fraction"], sort=True):
        pseudo, nodal = rows["PSEUDOINVERSE"], rows["EXACT_NODAL"]
        pseudo_rms, nodal_rms = rms_mrad(pseudo), rms_mrad(nodal)
        d3 = baseline[
            baseline.m.eq(count) & baseline.noise_fraction.eq(noise)
            & baseline.method.eq("D3 single-speed")
        ]
        delta = nodal - pseudo
        records.append({
            "M": count, "N": int(metrics.loc[metrics.m.eq(count), "N"].iloc[0]),
            "noise_fraction": noise, "pairs": len(rows),
            "one_speed_rms_mrad": rms_mrad(d3.error_deg),
            "pseudoinverse_rms_mrad": pseudo_rms, "nodal_rms_mrad": nodal_rms,
            "nodal_rms_change_pct": 100 * (nodal_rms / pseudo_rms - 1),
            "nodal_wins_pct": 100 * float((delta < -1e-12).mean()),
            "ties_pct": 100 * float((delta.abs() <= 1e-12).mean()),
        })
    return pd.DataFrame(records)


def band_summary(metrics: pd.DataFrame, baseline: pd.DataFrame) -> pd.DataFrame:
    """Match the five sample counts behind paper Table V, excluding new M=41."""
    selected = metrics[metrics.m.ne(41)].copy()
    selected["group"] = np.where(selected.noise_fraction.eq(0), "none", selected.noise_band)
    records = []
    for group in ("none", "low", "mid", "high"):
        for mode in MODES:
            rows = selected[selected.group.eq(group) & selected["mode"].eq(mode)]
            d3 = baseline[baseline.method.eq("D3 single-speed")][KEYS + ["error_deg"]]
            matched = rows.merge(d3, on=KEYS, suffixes=("", "_one_speed"), validate="one_to_one")
            records.append({
                "noise_group": group, "mode": mode, "trials": len(rows),
                "median_error_deg": rows.error_deg.median(),
                "one_speed_median_deg": matched.error_deg_one_speed.median(),
                "win_vs_one_speed_pct": 100 * (
                    matched.error_deg <= matched.error_deg_one_speed + 1e-15
                ).mean(),
            })
    return pd.DataFrame(records)


def write_report(output: Path) -> None:
    """Render a self-contained HTML report using saved results only."""
    provenance = json.loads((output / "provenance.json").read_text())
    if provenance["status"] != "complete":
        raise ValueError("Refusing to report an incomplete experiment")
    metrics = pd.read_csv(output / "trial_metrics.csv.gz")
    baseline = pd.read_csv(output / "cached_baselines.csv.gz")
    summary = exact_noise_summary(metrics, baseline)
    bands = band_summary(metrics, baseline)
    convergence = metrics.groupby(["m", "N", "mode"]).agg(
        trials=("converged", "size"), converged=("converged", "sum"),
        min_iterations=("iterations", "min"), median_iterations=("iterations", "median"),
        max_iterations=("iterations", "max"), max_residual=("residual", "max"),
    ).reset_index()
    summary.to_csv(output / "exact_noise_summary.csv", index=False)
    bands.to_csv(output / "band_summary.csv", index=False)
    convergence.to_csv(output / "convergence_summary.csv", index=False)

    def table(frame):
        return frame.to_html(index=False, border=0, float_format=lambda x: f"{x:.6g}")

    zero = summary[summary.M.eq(41) & summary.noise_fraction.eq(0)].iloc[0]
    positive = summary[summary.M.eq(41) & summary.noise_fraction.gt(0)]
    sections = [
        "<h1>PS-CPI Monte Carlo: pseudo-inverse versus exact nodal integration</h1>",
        f"<p>Run {html.escape(output.name)} · private GTSAM through its lidar Python wrapper.</p>",
        "<h2>Result at 200 Hz</h2>",
        f"<p>For the noise-free 41-sample case, endpoint RMS changes from "
        f"<strong>{zero.pseudoinverse_rms_mrad:.6g}</strong> to "
        f"<strong>{zero.nodal_rms_mrad:.6g} mrad</strong> "
        f"({zero.nodal_rms_change_pct:+.3f}%). At positive noise, the RMS changes "
        f"range from {positive.nodal_rms_change_pct.min():+.3f}% to "
        f"{positive.nodal_rms_change_pct.max():+.3f}%. Negative change means nodal is better.</p>",
        "<h2>Controlled comparison</h2>",
        "<p>Only the PS-CPI integration matrix changes. Both modes use the same "
        "N input and state nodes, identical least-squares gyro fits, RK2 initialization, "
        "a 10<sup>−12</sup> nodal Frobenius stopping tolerance, and at most 50 updates. "
        "PSEUDOINVERSE is the historical derivative-pseudoinverse matrix; EXACT_NODAL "
        "evaluates the integrated degree-(N−1) RHS polynomial at the original N nodes. "
        "It is not the N→N+1 EXACT_EXTENDED mode.</p>",
        "<p>The grid is 11 EuRoC snippets × 6 sample counts "
        "(10, 20, 30, 40, 41, 50) × 11 exact noise fractions × 100 deterministic "
        "seeds × 2 matrices: 145,200 new PS-CPI integrations. Each interval is 200 ms; "
        "M counts samples, giving sample rate (M−1)/0.2 Hz, and "
        "N = floor(2√(M−1)) + 1. Noise is independent Gaussian component noise "
        "with σᵢ = fraction × rangeₜ‖ωᵢ*(t)‖. Seed streams, smooth proxy construction, "
        "DOP853 clean-reference integration, and zero regularization match the old study. "
        "The two modes receive exactly the same fitted input array in every paired trial.</p>",
        "<p>These are endpoint accuracy and convergence experiments, not a timing "
        "benchmark. One-Speed values are read-only cached controls. Angular-rate "
        "interpolation uses only the unchanged gyro fit, so its Table V results do "
        "not depend on this integration matrix and are not rerun. No attitude-trajectory "
        "or subinterval experiments are included.</p>",
        "<h2>41 samples / 200 Hz: exact-noise RMS</h2>",
        "<p>RMS = √mean(error²), converted from degrees to mrad; 1,100 matched "
        "trials per row and method. Win percentages compare paired endpoint errors; "
        "differences within 10<sup>−12</sup> degrees count as ties. Repeated zero-noise "
        "seeds are identical, not independent observations. No significance claims or "
        "winner bolding are made.</p>",
        table(summary[summary.M.eq(41)]),
        "<h2>Five-rate banded medians (Table V endpoint counterpart)</h2>",
        "<p>Only M = 10, 20, 30, 40, 50 enter these pooled medians; M = 41 is excluded. "
        "The no-noise row is separate from positive low noise. Endpoint errors are "
        "in degrees. Win versus One-Speed retains the old ≤ comparison with a "
        "10<sup>−15</sup>-degree numerical allowance.</p>",
        table(bands),
        "<h2>Convergence</h2>", table(convergence),
        "<h2>Full exact-noise results by sample count</h2>",
    ]
    for count in SAMPLE_COUNTS:
        sections.extend([f"<h3>M = {count}, rate = {(count - 1) / .2:g} Hz</h3>",
                         table(summary[summary.M.eq(count)])])
    sections.extend([
        "<h2>Reproduction of old results and preservation checks</h2>",
        f"<p>Maximum absolute difference between all 72,600 rerun pseudo-inverse "
        f"errors and the old cache: {provenance['max_old_error_delta_deg']:.6g} degrees. "
        f"Validation requires rtol=10<sup>−8</sup>, atol=10<sup>−10</sup> degrees. "
        f"Every old result/cache file hash was unchanged after the run. "
        f"No manuscript files are read or written by this runner.</p>",
        "<p><a href='trial_metrics.csv.gz'>New per-trial results</a> · "
        "<a href='exact_noise_summary.csv'>Exact-noise summary</a> · "
        "<a href='band_summary.csv'>Banded summary</a> · "
        "<a href='provenance.json'>Provenance and hashes</a></p>",
        "<p>Reproduce with <code>conda run -n lidar python "
        "reports/run_taes_nodal_monte_carlo.py</code>. Each run creates a new directory. "
        "To rebuild only this HTML, pass <code>--render-only RUN_DIRECTORY</code>.</p>",
        "<details><summary>Run provenance</summary><pre>"
        + html.escape(json.dumps(provenance, indent=2)) + "</pre></details>",
    ])
    document = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PS-CPI: pseudo-inverse vs nodal Monte Carlo</title>
<style>body{font:16px/1.5 system-ui,sans-serif;max-width:1250px;margin:40px auto;padding:0 24px;color:#182733}
h1{font-size:1.8rem}h2{margin-top:2em}table{font-size:13px;border-collapse:collapse;display:block;overflow-x:auto;margin:20px 0}
td,th{padding:7px 9px;text-align:right;border-bottom:1px solid #d9e0e5;white-space:nowrap}
th{background:#edf3f7}td:first-child,th:first-child{text-align:left}p{max-width:100ch}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}
code{background:#edf3f7;padding:2px 4px}a{color:#09568f}</style><body>"""
    (output / "report.html").write_text(document + "\n".join(sections) + "</body></html>")


def run(output: Path) -> None:
    import generate_taes_euroc_snippet_experiment_report as old

    wrapper = Path(gtsam.gtsam.__file__).resolve()
    library = Path("/Users/dellaert/gtsam-private/lib/libgtsam.4.3a2.dylib")
    linkage = subprocess.check_output(["otool", "-l", str(wrapper)], text=True)
    if str(library.parent) not in linkage:
        raise RuntimeError("Wrapper must link to installed private GTSAM")
    # Snapshot all old assets, not only the two endpoint CSVs.
    protected = {str(p): sha256(p) for p in old.ASSET_DIR.rglob("*") if p.is_file()}
    old_frames = [pd.read_csv(old.ASSET_DIR / name) for name in
                  ("endpoint_trial_metrics.csv.gz", "endpoint_200hz_trial_metrics.csv.gz")]
    cached = pd.concat(old_frames, ignore_index=True)
    for count in SAMPLE_COUNTS:
        endpoint_rms_by_exact_noise(cached, sample_count=count, node_count=old.paper_node_count(count))
    if cached.duplicated(KEYS + ["method"]).any():
        raise ValueError("Duplicate old cache identity")
    cache_path = old.chebyshev_report.EUROC_CACHE_PATH
    protected[str(cache_path)] = sha256(cache_path)
    cache = old.chebyshev_report.load_aggressive_window_cache(cache_path)
    if not old.chebyshev_report.cache_is_current(cache):
        raise ValueError("Snippet selection cache is missing or stale")
    snippets = old.load_proxy_snippets(
        cache, data_dir=old.DATA_DIR, reference_N=old.psych_report.REFERENCE_N,
        middle_interval=old.psych_report.MIDDLE_INTERVAL,
    )
    pd.testing.assert_frame_equal(
        old.proxy_metadata(snippets), pd.read_csv(old.ASSET_DIR / "proxy_snippet_metadata.csv"),
        check_dtype=False, check_exact=False, rtol=1e-9, atol=1e-12,
    )
    expected = pd.MultiIndex.from_product(
        [[s.dataset for s in snippets], SAMPLE_COUNTS, old.psych_report.NOISE_FRACTIONS,
         range(100)], names=KEYS
    )
    cached_ps = cached[cached.method.eq("PsyChMagPI")].set_index(KEYS).sort_index()
    if not cached_ps.index.equals(expected.sort_values()):
        raise ValueError("Old cache does not cover the complete matched grid")
    if (len(snippets) != 11 or old.psych_report.NUM_SEEDS != 100
            or not cached_ps.psychmagpi_integration_matrix.eq("approximate_N_to_N").all()):
        raise ValueError("Unexpected old experiment settings")
    provenance = {
        "status": "running", "created_utc": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(), "wrapper": str(wrapper),
        "wrapper_sha256": sha256(wrapper), "private_library": str(library),
        "private_library_sha256": sha256(library),
        "gtsam_source_commit": subprocess.check_output(
            ["git", "-C", "/Users/dellaert/git/gtsam-private", "rev-parse", "HEAD"], text=True).strip(),
        "gtsam_source_status": subprocess.check_output(
            ["git", "-C", "/Users/dellaert/git/gtsam-private", "status", "--short"], text=True),
        "wrapper_linkage": linkage, "runner_sha256": sha256(Path(__file__)),
        "source_hashes": {str(p): sha256(p) for p in [
            Path(old.__file__), Path(spectral.__file__),
            REPORT_DIR.parent / "python/imuFactors/gyro_proxy_monte_carlo.py"]},
        "source_csv_sha256": {s.dataset: sha256(s.csv_path) for s in snippets},
        "protected_old_sha256": protected, "sample_counts": SAMPLE_COUNTS,
        "noise_fractions": list(old.psych_report.NOISE_FRACTIONS),
        "seeds_per_snippet": 100, "seed": old.psych_report.RANDOM_SEED,
        "modes": MODES, "rule": "N=floor(2*sqrt(M-1))+1", "lambda1": 0,
        "initializer": "RK2", "tolerance": 1e-12, "max_iterations": 50,
    }
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2))
    input_hash = hashlib.sha256()
    frames = []
    started = time.perf_counter()
    for index, snippet in enumerate(snippets):
        rows = []
        for count in SAMPLE_COUNTS:
            times = np.linspace(*snippet.middle_interval, count)
            plan = spectral.basis_plan_from_coordinates(
                old.paper_node_count(count), times, basis="chebyshev2",
                interval=snippet.middle_interval, window_seconds=.2, lambda1=0,
            )
            integrators = {mode: make_integrator(plan.N, snippet.middle_interval, mode) for mode in MODES}
            clean = old.evaluate_proxy_gyro(snippet, times)
            noise = old.standard_noise(old.psych_report.RANDOM_SEED, index, count, 100)
            labels = old.noise_band_labels(old.psych_report.NOISE_FRACTIONS)
            for fraction in old.psych_report.NOISE_FRACTIONS:
                for seed, perturbation in enumerate(noise):
                    fitted = plan.fit(clean + fraction * snippet.noise_scale * perturbation).T
                    input_hash.update(np.ascontiguousarray(fitted).tobytes())
                    for mode, integrator in integrators.items():
                        history = integrator.integrateWithHistory(fitted)
                        state = np.asarray(history.finalZ)
                        residuals = np.asarray(history.errorNorm)
                        if state.shape != (3, plan.N) or not np.isfinite(state).all():
                            raise ValueError("Invalid square-mode trajectory")
                        if not history.converged or residuals[-1] >= 1e-12:
                            raise ValueError(f"Non-convergence: {snippet.dataset}/{count}/{fraction}/{seed}/{mode}")
                        error = rotation_error_degrees(snippet.reference_rotation, gtsam.Rot3.Expmap(state[:, -1]))
                        rows.append({
                            "dataset": snippet.dataset, "m": count, "N": plan.N,
                            "noise_fraction": fraction, "noise_band": labels[fraction],
                            "seed_index": seed, "mode": mode, "error_deg": error,
                            "converged": bool(history.converged), "iterations": len(residuals),
                            "residual": float(residuals[-1]),
                        })
        frame = pd.DataFrame(rows)
        frame.to_csv(output / f"trials_{snippet.dataset}.csv.gz", index=False)
        frames.append(frame)
        print(f"{snippet.dataset}: {len(frame):,} integrations; elapsed {time.perf_counter()-started:.1f}s", flush=True)
    metrics = pd.concat(frames, ignore_index=True)
    paired = paired_trials(metrics).set_index(KEYS).sort_index()
    if not paired.index.equals(expected.sort_values()):
        raise ValueError("New results do not cover the complete matched grid")
    delta = paired.PSEUDOINVERSE - cached_ps.error_deg
    metrics.to_csv(output / "trial_metrics.csv.gz", index=False)
    np.testing.assert_allclose(paired.PSEUDOINVERSE, cached_ps.error_deg, rtol=1e-8, atol=1e-10)
    for filename, fingerprint in protected.items():
        if sha256(Path(filename)) != fingerprint:
            raise RuntimeError(f"Protected old file changed: {filename}")
    cached[cached.method.ne("PsyChMagPI")].to_csv(output / "cached_baselines.csv.gz", index=False)
    provenance.update(
        status="complete", trial_count=len(metrics), all_converged=bool(metrics.converged.all()),
        max_old_error_delta_deg=float(delta.abs().max()), fitted_input_sha256=input_hash.hexdigest(),
        result_sha256=sha256(output / "trial_metrics.csv.gz"), elapsed_s=time.perf_counter()-started,
        old_files_unchanged=True,
    )
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    write_report(output)
    print(f"HTML report: {output / 'report.html'}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render-only", type=Path)
    args = parser.parse_args()
    if args.render_only:
        write_report(args.render_only)
        return
    output = OUTPUT_ROOT / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output.mkdir(parents=True, exist_ok=False)
    print(f"New result directory: {output}", flush=True)
    try:
        run(output)
    except Exception as error:
        (output / "failure.txt").write_text(f"{type(error).__name__}: {error}\n")
        raise


if __name__ == "__main__":
    main()
