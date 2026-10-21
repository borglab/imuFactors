"""Snippet-clustered, paired significance tests for the EuRoC median table.

Under the null, method labels are exchangeable within each snippet. All noise
draws, sample counts, and noise levels in a snippet are swapped together. This
preserves their dependence, including duplicated noise-free seeds. The statistic
is the difference of pooled medians, exactly as displayed in the paper.
"""

from itertools import product

import numpy as np
import pandas as pd


NOISE_GROUPS = ("none", "low", "mid", "high")


def select_noise_group(frame: pd.DataFrame, noise_group: str) -> pd.DataFrame:
    """Separate the noise-free group from the three positive-noise bands."""
    if noise_group == "none":
        return frame[np.isclose(frame["noise_fraction"], 0.0)]
    return frame[
        frame["noise_band"].eq(noise_group) & frame["noise_fraction"].gt(0.0)
    ]


def paired_cluster_median_test(
    frame: pd.DataFrame, baseline: str, column: str,
) -> float:
    """Return an exact two-sided p-value from all snippet-level label swaps."""
    keys = ["dataset", "m", "noise_fraction", "seed_index"]
    selected = frame[frame["method"].isin([baseline, "PsyChMagPI"])]
    paired = selected.pivot(index=keys, columns="method", values=column)
    if paired.empty or set(paired.columns) != {baseline, "PsyChMagPI"}:
        raise ValueError("Missing paired methods")
    if not np.isfinite(paired.to_numpy()).all():
        raise ValueError("Incomplete or non-finite paired trials")
    clusters = paired.index.get_level_values("dataset")
    cluster_index, names = pd.factorize(clusters, sort=True)
    if len(names) > 16:
        raise ValueError("Exact cluster test is intended for at most 16 snippets")
    baseline_values = paired[baseline].to_numpy()
    psych_values = paired["PsyChMagPI"].to_numpy()
    observed = abs(np.median(baseline_values) - np.median(psych_values))
    if observed == 0:
        return 1.0
    extreme = 0
    for swaps in product((False, True), repeat=len(names)):
        mask = np.asarray(swaps)[cluster_index]
        first = np.where(mask, psych_values, baseline_values)
        second = np.where(mask, baseline_values, psych_values)
        difference = abs(np.median(first) - np.median(second))
        extreme += difference >= observed * (1.0 - 1e-12)
    return float(extreme / (2 ** len(names)))


def holm_adjust(p_values: np.ndarray) -> np.ndarray:
    """Holm step-down family-wise adjustment, restored to input order."""
    values = np.asarray(p_values, dtype=float)
    if not np.isfinite(values).all() or ((values < 0) | (values > 1)).any():
        raise ValueError("p-values must be finite and in [0, 1]")
    order = np.argsort(values)
    adjusted = np.empty_like(values)
    adjusted[order] = np.minimum(
        1.0, np.maximum.accumulate(values[order] * np.arange(len(values), 0, -1))
    )
    return adjusted


def median_table_significance(
    endpoint: pd.DataFrame, signal: pd.DataFrame,
) -> pd.DataFrame:
    """Test all twelve comparisons, including both interpolation baselines.

    Testing both baselines before selecting the displayed one avoids treating
    the data-selected best interpolator as a prespecified comparison.
    """
    rows = []
    for group in NOISE_GROUPS:
        for metric, frame, column, baselines in (
            ("endpoint", endpoint, "error_deg", ("D3 single-speed",)),
            ("signal", signal, "gyro_nrmse", ("Linear", "Cubic spline")),
        ):
            band = select_noise_group(frame, group)
            for baseline in baselines:
                rows.append({
                    "noise_group": group, "metric": metric, "baseline": baseline,
                    "p_value": paired_cluster_median_test(band, baseline, column),
                })
    result = pd.DataFrame(rows)
    result["p_holm"] = holm_adjust(result["p_value"].to_numpy())
    return result
