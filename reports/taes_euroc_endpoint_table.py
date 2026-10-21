"""Shared cache aggregation for the TAES EuRoC endpoint RMS table."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import pandas as pd


EUROC_RMS_SAMPLE_COUNT = 41
EUROC_RMS_CACHE_NAME = "endpoint_200hz_trial_metrics.csv.gz"
EUROC_RMS_NODE_COUNT = 13
EUROC_RMS_BENCHMARK_VERSION = 5
EUROC_RMS_TRIALS_PER_CELL = 1100
EUROC_RMS_NOISE_FRACTIONS = (
    0.0,
    0.025,
    0.05,
    0.06,
    0.075,
    0.1,
    0.12,
    0.15,
    0.17,
    0.2,
    0.225,
)
EUROC_RMS_METHOD_COLUMNS: tuple[tuple[str, str | None], ...] = (
    ("No Coning", None),
    ("Two-Speed", None),
    ("Seq. Rot.", "D2/RK2"),
    ("One-Speed", "D3 single-speed"),
    ("PS-CPI (1 iter.)", None),
    ("PS-CPI (convg.)", "PsyChMagPI"),
)


def endpoint_rms_by_exact_noise(
    metrics: pd.DataFrame,
    *,
    sample_count: int = EUROC_RMS_SAMPLE_COUNT,
    node_count: int = EUROC_RMS_NODE_COUNT,
    benchmark_version: int | None = EUROC_RMS_BENCHMARK_VERSION,
    noise_fractions: Sequence[float] = EUROC_RMS_NOISE_FRACTIONS,
    trials_per_cell: int = EUROC_RMS_TRIALS_PER_CELL,
) -> pd.DataFrame:
    """Return cached endpoint RMS errors in mrad for each exact noise value."""
    required_columns = {
        "m",
        "N_paper",
        "noise_fraction",
        "method",
        "error_deg",
        "converged",
    }
    if benchmark_version is not None:
        required_columns.add("benchmark_version")
    missing_columns = sorted(required_columns - set(metrics.columns))
    if missing_columns:
        raise ValueError(f"endpoint cache is missing columns: {missing_columns}")

    selected = metrics[metrics["m"].eq(int(sample_count))].copy()
    if selected.empty:
        raise ValueError(f"endpoint cache has no M={sample_count} trials")
    _require_single_value(selected, "N_paper", int(node_count))
    if benchmark_version is not None:
        _require_single_value(selected, "benchmark_version", int(benchmark_version))

    expected_noises = np.asarray(tuple(noise_fractions), dtype=float)
    actual_noises = np.sort(selected["noise_fraction"].unique().astype(float))
    if actual_noises.shape != expected_noises.shape or not np.allclose(
        actual_noises,
        expected_noises,
        rtol=0.0,
        atol=1e-12,
    ):
        raise ValueError(
            "endpoint cache noise values do not match the EuRoC RMS table: "
            f"expected {expected_noises.tolist()}, got {actual_noises.tolist()}"
        )

    cached_methods = tuple(
        method for _, method in EUROC_RMS_METHOD_COLUMNS if method is not None
    )
    actual_methods = set(selected["method"].astype(str).unique())
    if actual_methods != set(cached_methods):
        raise ValueError(
            "endpoint cache methods do not match the EuRoC RMS table: "
            f"expected {sorted(cached_methods)}, got {sorted(actual_methods)}"
        )

    group_sizes = selected.groupby(["noise_fraction", "method"], observed=True).size()
    if len(group_sizes) != len(expected_noises) * len(cached_methods):
        raise ValueError(
            "endpoint cache does not contain every EuRoC RMS noise/method cell"
        )
    if not group_sizes.eq(int(trials_per_cell)).all():
        bad_sizes = group_sizes[~group_sizes.eq(int(trials_per_cell))].to_dict()
        raise ValueError(
            f"endpoint cache cells must contain {trials_per_cell} trials: {bad_sizes}"
        )

    psych_rows = selected[selected["method"].astype(str).eq("PsyChMagPI")]
    if not psych_rows["converged"].astype(bool).all():
        raise ValueError("endpoint cache contains non-converged EuRoC RMS trials")

    degrees_to_mrad = np.pi * 1000.0 / 180.0
    summary = (
        selected.assign(error_squared=selected["error_deg"].astype(float) ** 2)
        .groupby(["noise_fraction", "method"], observed=True)["error_squared"]
        .mean()
        .pow(0.5)
        .mul(degrees_to_mrad)
        .unstack("method")
        .reindex(index=expected_noises, columns=list(cached_methods))
        .reset_index()
        .rename(columns={"noise_fraction": "Noise fraction alpha"})
    )
    if summary.isna().any().any():
        raise ValueError("endpoint cache produced missing EuRoC RMS values")
    return summary


def endpoint_metrics_for_sqrt_rule(
    baseline_metrics: pd.DataFrame,
    psych_metrics: pd.DataFrame,
) -> pd.DataFrame:
    """Combine invariant baselines with cached PS-CPI ``N=ceil(sqrt(M))`` trials."""
    keys = ["dataset", "m", "noise_fraction", "seed_index"]
    baseline_methods = {"D2/RK2", "D3 single-speed"}
    baselines = baseline_metrics[
        baseline_metrics["method"].astype(str).isin(baseline_methods)
    ].copy()
    if set(baselines["method"].astype(str).unique()) != baseline_methods:
        raise ValueError("endpoint baseline cache is missing a classical method")

    psych = psych_metrics[
        psych_metrics["method"].astype(str).eq("PsyChMagPI")
        & np.isclose(
            psych_metrics["N"].to_numpy(dtype=float),
            psych_metrics["N_sqrt"].to_numpy(dtype=float),
        )
        & np.isclose(psych_metrics["lambda1"].to_numpy(dtype=float), 0.0)
    ].copy()
    expected_psych_rows = baselines[
        baselines["method"].astype(str).eq("D3 single-speed")
    ]
    if len(psych) != len(expected_psych_rows):
        raise ValueError(
            "old-rule PS-CPI cache does not match the classical baseline coverage"
        )

    d3_errors = expected_psych_rows[keys + ["error_deg"]].rename(
        columns={"error_deg": "d3_error_deg"}
    )
    psych = psych.merge(d3_errors, on=keys, how="inner", validate="one_to_one")
    if len(psych) != len(expected_psych_rows):
        raise ValueError("old-rule PS-CPI trials do not align with the baseline trials")
    psych["method"] = "PsyChMagPI"
    psych["win_rate"] = (
        psych["error_deg"].to_numpy(dtype=float)
        < psych["d3_error_deg"].to_numpy(dtype=float)
    ).astype(float)

    baselines["N_paper"] = np.ceil(
        np.sqrt(baselines["m"].to_numpy(dtype=float))
    ).astype(int)
    psych["N_paper"] = psych["N_sqrt"].astype(int)
    combined = pd.concat([baselines, psych], ignore_index=True, sort=False)
    return combined.drop(columns=["benchmark_version", "d3_error_deg"], errors="ignore")


def euroc_rms_method_values(row: pd.Series) -> tuple[float | None, ...]:
    """Map one RMS summary row to the six Table II-style method columns."""
    return tuple(
        None if method is None else float(row[method])
        for _, method in EUROC_RMS_METHOD_COLUMNS
    )


def _require_single_value(frame: pd.DataFrame, column: str, expected: int) -> None:
    values = sorted(frame[column].dropna().astype(int).unique().tolist())
    if values != [expected]:
        raise ValueError(
            f"endpoint cache M={EUROC_RMS_SAMPLE_COUNT} requires {column}={expected}, "
            f"got {values}"
        )
