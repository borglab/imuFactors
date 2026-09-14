"""Canonical window rows for the Delama Gal(3) propagation evaluator."""
from __future__ import annotations

import numpy as np
import torch

from .preintegration_delama_gal3 import compute_ext_pose_nees

METHOD = "delama_gal3_python"
IDENTITY_FIELDS = "run_id app_name dataset method config_label interval_seconds samples_per_window quadrature_nodes".split()
METRIC_FIELDS = "normalized_nees rot_error_norm rot_pred_sigma pos_error_norm pos_pred_sigma vel_error_norm vel_pred_sigma".split()
WINDOW_FIELDS = IDENTITY_FIELDS + "window_index window_start_sample window_end_sample window_start_time window_end_time".split() + METRIC_FIELDS
SUMMARY_FIELDS = IDENTITY_FIELDS + "num_windows normalized_nees_mean normalized_nees_median normalized_nees_p95 normalized_nees_variance rot_error_median rot_pred_sigma_median pos_error_median pos_pred_sigma_median vel_error_median vel_pred_sigma_median".split()


def canonical_blocks(error, covariance):
    """Permute native (R,V,P) to (R,P,V), including both covariance axes."""
    order = torch.tensor([0, 1, 2, 6, 7, 8, 3, 4, 5], device=error.device)
    return (error.index_select(-1, order),
            covariance.index_select(-2, order).index_select(-1, order))


def canonical_rows(result, run_id: str, app_name: str, dataset: str):
    """Export radians/meters/m/s and block RMS sigmas with aligned NEES."""
    error, covariance = result["physical_error"], result["reporting_covariance"]
    nees = compute_ext_pose_nees(result["native_covariance"], result["native_error"])
    norms = error.reshape(-1, 3, 3).norm(dim=2)
    sigmas = covariance.diagonal(dim1=-2, dim2=-1).reshape(-1, 3, 3).mean(dim=2).clamp(min=0).sqrt()
    metrics = torch.stack((nees, norms[:, 0], sigmas[:, 0], norms[:, 1],
                           sigmas[:, 1], norms[:, 2], sigmas[:, 2]), dim=1).cpu().numpy()
    if not np.isfinite(metrics).all():
        raise ValueError(f"Non-finite Delama metrics for {dataset}, {result['preint_time']} s")
    identity = dict(zip(IDENTITY_FIELDS, (run_id, app_name, dataset, METHOD,
                        result["config_label"],
                        result["preint_time"], result["steps_per_window"], 0)))
    rows = []
    for index, (start, end) in enumerate(zip(result["starts"].tolist(), result["ends"].tolist())):
        rows.append({**identity, "window_index": index, "window_start_sample": start,
                     "window_end_sample": end, "window_start_time": result["times"][start].item(),
                     "window_end_time": result["times"][end].item(),
                     **dict(zip(METRIC_FIELDS, metrics[index]))})
    return rows, summarize_rows(rows)


def summarize_rows(rows):
    """C++ median, population variance and floor(0.95*(n-1)) percentile."""
    if not rows:
        raise ValueError("Cannot summarize an empty set of windows")
    nees = np.array([row["normalized_nees"] for row in rows])
    summary = {key: rows[0][key] for key in IDENTITY_FIELDS}
    summary.update(num_windows=len(rows), normalized_nees_mean=nees.mean(),
                   normalized_nees_median=np.median(nees),
                   normalized_nees_p95=np.sort(nees)[int(np.floor(.95 * (len(nees) - 1)))],
                   normalized_nees_variance=nees.var(ddof=0))
    for field in METRIC_FIELDS[1:]:
        summary[field.replace("_norm", "") + "_median"] = np.median([row[field] for row in rows])
    return summary
