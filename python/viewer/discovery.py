"""Run discovery and package status inspection for canonical result packages."""

from __future__ import annotations

import csv
from pathlib import Path

from .models import RunCatalogEntry

CANONICAL_RESULT_FILES = (
    "run_metadata.csv",
    "datasets.csv",
    "window_metrics.csv",
    "window_summaries.csv",
    "trajectory_samples.csv",
    "calibration_trials.csv",
    "calibration_summaries.csv",
)

SUMMARY_RESULT_FILES = ("window_summaries.csv", "calibration_summaries.csv")


def _read_csv_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    if not path.exists() or path.stat().st_size == 0:
        return [], []

    try:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fieldnames = reader.fieldnames or []
            rows = list(reader)
    except (OSError, csv.Error):
        return [], []

    return fieldnames, rows


def _csv_is_readable(path: Path) -> bool:
    fieldnames, _ = _read_csv_rows(path)
    return bool(fieldnames)


def _csv_has_rows(path: Path) -> bool:
    _, rows = _read_csv_rows(path)
    return bool(rows)


def _discover_dataset_info(run_dir: Path) -> tuple[int, str]:
    fieldnames, rows = _read_csv_rows(run_dir / "datasets.csv")
    if not fieldnames:
        return 0, ""

    dataset_groups = sorted(
        {
            row.get("dataset_group", "").strip()
            for row in rows
            if row.get("dataset_group", "").strip()
        }
    )
    dataset_count = len(rows)
    if len(dataset_groups) == 1:
        return dataset_count, dataset_groups[0]
    if dataset_count:
        return dataset_count, f"{dataset_count} datasets"
    return 0, ""


def _metadata_fields(run_dir: Path) -> dict[str, str]:
    fieldnames, rows = _read_csv_rows(run_dir / "run_metadata.csv")
    if not fieldnames:
        return {}
    return rows[0] if rows else {}


def _derive_status(run_dir: Path) -> str:
    metadata_ok = _csv_is_readable(run_dir / "run_metadata.csv")
    any_summary_ok = any(_csv_is_readable(run_dir / name) for name in SUMMARY_RESULT_FILES)
    return "ready" if metadata_ok and any_summary_ok else "partial"


def _make_catalog_entry(run_dir: Path) -> RunCatalogEntry:
    metadata = _metadata_fields(run_dir)
    dataset_count, dataset_group_label = _discover_dataset_info(run_dir)

    return RunCatalogEntry(
        app_name=metadata.get("app_name", run_dir.parent.name),
        run_id=metadata.get("run_id", run_dir.name),
        path=run_dir,
        status=_derive_status(run_dir),
        timestamp_utc=metadata.get("timestamp_utc", ""),
        cli_args=metadata.get("cli_args", ""),
        output_root=metadata.get("output_root", ""),
        repo_version=metadata.get("repo_version", ""),
        dataset_count=dataset_count,
        dataset_group_label=dataset_group_label,
        has_window_metrics=_csv_has_rows(run_dir / "window_metrics.csv"),
        has_trajectory_samples=_csv_has_rows(run_dir / "trajectory_samples.csv"),
        has_window_summaries=_csv_has_rows(run_dir / "window_summaries.csv"),
        has_calibration_summaries=_csv_has_rows(run_dir / "calibration_summaries.csv"),
    )


def discover_runs(results_root: str | Path) -> list[RunCatalogEntry]:
    """Discover canonical result-package directories under one results root."""

    root = Path(results_root).expanduser()
    if not root.exists():
        return []

    run_dirs: set[Path] = set()
    for filename in CANONICAL_RESULT_FILES:
        for file_path in root.rglob(filename):
            run_dirs.add(file_path.parent)

    catalog = [_make_catalog_entry(run_dir) for run_dir in run_dirs]
    catalog.sort(
        key=lambda entry: (
            entry.timestamp_utc or entry.run_id,
            entry.app_name,
            entry.run_id,
        ),
        reverse=True,
    )
    return catalog
