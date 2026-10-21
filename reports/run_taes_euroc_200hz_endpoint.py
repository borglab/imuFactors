"""Run only the 41-sample, 200 Hz endpoint experiment for paper Table IV.

Run in py312 with PYTHONPATH pointing to a working gtsam-private wrapper.
Existing caches are never overwritten; paper/report generation only reads them.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import platform
import subprocess
from datetime import datetime, timezone

import gtsam
import numpy as np
import pandas as pd

import generate_taes_euroc_snippet_experiment_report as report
from taes_euroc_endpoint_table import (
    EUROC_RMS_CACHE_NAME,
    EUROC_RMS_SAMPLE_COUNT,
    endpoint_rms_by_exact_noise,
)


def sha256(path: Path) -> str:
    """Fingerprint an experiment input or binary."""
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main() -> None:
    output = report.ASSET_DIR / EUROC_RMS_CACHE_NAME
    provenance_path = output.with_name("endpoint_200hz_provenance.json")
    if output.exists() or provenance_path.exists():
        raise FileExistsError("200 Hz results already exist; refusing to overwrite")
    for name in (
        "PsyChMagPIRot3", "PseudoSpectralPiParams",
        "integrateSequentialRotations", "integrateSingleSpeedConing",
    ):
        if not hasattr(gtsam, name):
            raise RuntimeError(f"Private GTSAM wrapper required: missing {name}")

    wrapper = Path(gtsam.gtsam.__file__).resolve()
    linkage = subprocess.check_output(["otool", "-l", str(wrapper)], text=True)
    private_library = Path("/Users/dellaert/gtsam-private/lib/libgtsam.4.3a2.dylib")
    if str(private_library.parent) not in linkage:
        raise RuntimeError("Wrapper does not have the expected private-library rpath")

    cache_path = report.chebyshev_report.EUROC_CACHE_PATH
    cache = report.chebyshev_report.load_aggressive_window_cache(cache_path)
    if not report.chebyshev_report.cache_is_current(cache):
        raise ValueError("Existing snippet-selection cache is not current")
    snippets = report.load_proxy_snippets(
        cache, data_dir=report.DATA_DIR,
        reference_N=report.psych_report.REFERENCE_N,
        middle_interval=report.psych_report.MIDDLE_INTERVAL,
    )
    old_metadata = pd.read_csv(report.ASSET_DIR / "proxy_snippet_metadata.csv")
    metadata = report.proxy_metadata(snippets)
    pd.testing.assert_frame_equal(metadata, old_metadata, check_dtype=False,
                                  check_exact=False, rtol=1e-9, atol=1e-12)
    if len(snippets) != 11 or report.paper_node_count(EUROC_RMS_SAMPLE_COUNT) != 13:
        raise ValueError("Expected eleven snippets and N=13")
    for snippet in snippets:
        times = np.linspace(*snippet.middle_interval, EUROC_RMS_SAMPLE_COUNT)
        np.testing.assert_allclose(np.diff(times), 0.005, rtol=0, atol=1e-15)

    provenance = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "wrapper_path": str(wrapper), "wrapper_sha256": sha256(wrapper),
        "private_library_path": str(private_library),
        "private_library_sha256": sha256(private_library),
        "wrapper_linkage": linkage,
        "snippet_cache_sha256": sha256(cache_path),
        "source_csv_sha256": {s.dataset: sha256(s.csv_path) for s in snippets},
        "experiment_source_sha256": sha256(Path(report.__file__)),
        "sample_count": 41, "interval_count": 40, "rate_hz": 200,
        "duration_s": 0.2, "node_count": 13,
        "polynomial_rule": "n=floor(2*sqrt(M-1)); N=n+1",
        "approximate_integration_matrix": True, "lambda1": 0.0,
        "benchmark_version": report.ENDPOINT_BENCHMARK_VERSION,
        "noise_fractions": list(report.psych_report.NOISE_FRACTIONS),
        "seeds_per_snippet": report.psych_report.NUM_SEEDS,
        "random_seed": report.psych_report.RANDOM_SEED,
        "tolerance": report.psych_report.PSYCHMAGPI_TOLERANCE,
        "max_iterations": report.psych_report.PSYCHMAGPI_MAX_ITERATIONS,
    }
    print(f"Running 41-sample endpoint trials using {wrapper}", flush=True)
    metrics = report.run_endpoint_results(snippets, sample_counts=(41,))
    summary = endpoint_rms_by_exact_noise(metrics)
    keys = ["dataset", "m", "noise_fraction", "seed_index", "method"]
    if metrics.duplicated(keys).any():
        raise ValueError("Duplicate endpoint trial identities")
    cells = metrics.groupby(["dataset", "noise_fraction", "method"], observed=True)
    if not cells["seed_index"].apply(lambda x: set(x) == set(range(100))).all():
        raise ValueError("Each snippet/noise/method must contain all 100 seeds")
    if not np.isfinite(metrics["error_deg"]).all():
        raise ValueError("Non-finite endpoint errors")
    provenance["trial_count"] = len(metrics)
    provenance["all_ps_cpi_converged"] = bool(
        metrics.loc[metrics["method"].eq("PsyChMagPI"), "converged"].all()
    )
    metrics.to_csv(output, index=False, compression="gzip")
    provenance["result_sha256"] = sha256(output)
    provenance_path.write_text(json.dumps(provenance, indent=2) + "\n")
    summary.to_csv(output.with_name("endpoint_200hz_rms_summary.csv"), index=False)
    print(summary.to_string(index=False))
    print(f"Validated and saved {len(metrics):,} trials to {output}")


if __name__ == "__main__":
    main()
