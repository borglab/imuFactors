"""Lazy CSV loading helpers for summary-table viewer state."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import pandas as pd

from .models import RunCatalogEntry, RunData

TRAJECTORY_INDEX_COLUMNS = (
    "dataset",
    "method",
    "config_label",
    "interval_seconds",
    "timestamp",
)


def load_csv_frame(path: Path, usecols: list[str] | tuple[str, ...] | None = None) -> pd.DataFrame:
    """Read one CSV into a DataFrame, normalizing empty states to an empty frame."""

    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()

    try:
        return pd.read_csv(path, usecols=list(usecols) if usecols else None)
    except (OSError, pd.errors.EmptyDataError, pd.errors.ParserError):
        return pd.DataFrame()


@lru_cache(maxsize=12)
def _load_cached_trajectory_frame(path_str: str, columns: tuple[str, ...]) -> pd.DataFrame:
    return load_csv_frame(Path(path_str), usecols=columns)


def load_trajectory_index(run_dir: Path) -> pd.DataFrame:
    """Load compact availability metadata for trajectory samples."""

    frame = _load_cached_trajectory_frame(str(run_dir / "trajectory_samples.csv"), TRAJECTORY_INDEX_COLUMNS)
    if frame.empty:
        return pd.DataFrame(
            columns=[
                "dataset",
                "method",
                "config_label",
                "interval_seconds",
                "sample_count",
                "timestamp_start",
                "timestamp_end",
            ]
        )

    grouped = (
        frame.groupby(["dataset", "method", "config_label", "interval_seconds"], dropna=False)
        .agg(sample_count=("timestamp", "size"), timestamp_start=("timestamp", "min"), timestamp_end=("timestamp", "max"))
        .reset_index()
    )
    return grouped.sort_values(["dataset", "method", "config_label", "interval_seconds"]).reset_index(drop=True)


def load_trajectory_samples(
    run_dir: Path,
    columns: list[str] | tuple[str, ...],
    *,
    dataset: str | None = None,
    method: str | None = None,
    config_label: str | None = None,
    interval_seconds: float | int | None = None,
    start_time: float | int | None = None,
    end_time: float | int | None = None,
) -> pd.DataFrame:
    """Load a filtered trajectory-sample slice using cached column subsets."""

    frame = _load_cached_trajectory_frame(str(run_dir / "trajectory_samples.csv"), tuple(columns))
    if frame.empty:
        return pd.DataFrame(columns=list(columns))

    filtered = frame
    if dataset is not None and "dataset" in filtered.columns:
        filtered = filtered[filtered["dataset"] == dataset]
    if method is not None and "method" in filtered.columns:
        filtered = filtered[filtered["method"] == method]
    if config_label is not None and "config_label" in filtered.columns:
        filtered = filtered[filtered["config_label"] == config_label]
    if interval_seconds is not None and "interval_seconds" in filtered.columns:
        filtered = filtered[filtered["interval_seconds"] == interval_seconds]
    if start_time is not None and "timestamp" in filtered.columns:
        filtered = filtered[filtered["timestamp"] >= start_time]
    if end_time is not None and "timestamp" in filtered.columns:
        filtered = filtered[filtered["timestamp"] <= end_time]
    return filtered.reset_index(drop=True).copy()


def load_run_data(entry: RunCatalogEntry) -> RunData:
    """Load viewer-facing CSVs for one selected run."""

    return RunData(
        entry=entry,
        metadata=load_csv_frame(entry.path / "run_metadata.csv"),
        datasets=load_csv_frame(entry.path / "datasets.csv"),
        window_metrics=load_csv_frame(entry.path / "window_metrics.csv"),
        trajectory_index=load_trajectory_index(entry.path),
        window_summaries=load_csv_frame(entry.path / "window_summaries.csv"),
        calibration_summaries=load_csv_frame(entry.path / "calibration_summaries.csv"),
    )
