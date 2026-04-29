"""Data models for Dash viewers over canonical result packages."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


def _empty_frame() -> "pd.DataFrame":
    import pandas as pd

    return pd.DataFrame()


@dataclass(frozen=True)
class RunCatalogEntry:
    """Catalog entry for one discovered result package."""

    app_name: str
    run_id: str
    path: Path
    status: str
    timestamp_utc: str = ""
    cli_args: str = ""
    output_root: str = ""
    repo_version: str = ""
    dataset_count: int = 0
    dataset_group_label: str = ""
    has_window_metrics: bool = False
    has_trajectory_samples: bool = False
    has_window_summaries: bool = False
    has_calibration_summaries: bool = False


@dataclass(frozen=True)
class RunData:
    """Loaded viewer data for one run."""

    entry: RunCatalogEntry
    metadata: "pd.DataFrame"
    datasets: "pd.DataFrame"
    window_metrics: "pd.DataFrame" = field(default_factory=_empty_frame)
    trajectory_index: "pd.DataFrame" = field(default_factory=_empty_frame)
    window_summaries: "pd.DataFrame" = field(default_factory=_empty_frame)
    calibration_summaries: "pd.DataFrame" = field(default_factory=_empty_frame)
