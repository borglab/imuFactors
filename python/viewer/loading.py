"""Lazy CSV loading helpers for summary-table viewer state."""

from __future__ import annotations

from pathlib import Path

import pandas as pd

from .models import RunCatalogEntry, RunData


def load_csv_frame(path: Path) -> pd.DataFrame:
    """Read one CSV into a DataFrame, normalizing empty states to an empty frame."""

    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()

    try:
        return pd.read_csv(path)
    except (OSError, pd.errors.EmptyDataError, pd.errors.ParserError):
        return pd.DataFrame()


def load_run_data(entry: RunCatalogEntry) -> RunData:
    """Load only the summary-facing CSVs for one selected run."""

    return RunData(
        entry=entry,
        metadata=load_csv_frame(entry.path / "run_metadata.csv"),
        datasets=load_csv_frame(entry.path / "datasets.csv"),
        window_summaries=load_csv_frame(entry.path / "window_summaries.csv"),
        calibration_summaries=load_csv_frame(entry.path / "calibration_summaries.csv"),
    )
