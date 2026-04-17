"""Dash application entrypoint for summary-table viewing."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from dash import Dash, Input, Output, callback, html

from .discovery import discover_runs
from .layout import build_shell, empty_state, metadata_card, status_badge
from .loading import load_run_data
from .models import RunCatalogEntry, RunData


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dash summary viewer for imuFactors result packages.")
    parser.add_argument(
        "--results-root",
        default="build/results",
        help="Root directory containing canonical result packages (default: build/results).",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Host interface for the Dash server.")
    parser.add_argument("--port", type=int, default=8050, help="Port for the Dash server.")
    parser.add_argument("--debug", action="store_true", help="Run Dash in debug mode.")
    return parser.parse_args()


def _format_cli_args(cli_args: str) -> str:
    if not cli_args:
        return "—"
    if len(cli_args) <= 120:
        return cli_args
    return cli_args[:117] + "..."


def _serialize_run_data(run_data: RunData) -> dict[str, Any]:
    entry = run_data.entry
    return {
        "entry": {
            "app_name": entry.app_name,
            "run_id": entry.run_id,
            "path": str(entry.path),
            "status": entry.status,
            "timestamp_utc": entry.timestamp_utc,
            "cli_args": entry.cli_args,
            "output_root": entry.output_root,
            "repo_version": entry.repo_version,
            "dataset_count": entry.dataset_count,
            "dataset_group_label": entry.dataset_group_label,
            "has_window_summaries": entry.has_window_summaries,
            "has_calibration_summaries": entry.has_calibration_summaries,
        },
        "metadata_columns": list(run_data.metadata.columns),
        "metadata_rows": run_data.metadata.to_dict("records"),
        "datasets_columns": list(run_data.datasets.columns),
        "datasets_rows": run_data.datasets.to_dict("records"),
        "window_columns": list(run_data.window_summaries.columns),
        "window_rows": run_data.window_summaries.to_dict("records"),
        "calibration_columns": list(run_data.calibration_summaries.columns),
        "calibration_rows": run_data.calibration_summaries.to_dict("records"),
    }


def _deserialize_entry(payload: dict[str, Any]) -> RunCatalogEntry:
    return RunCatalogEntry(
        app_name=payload["app_name"],
        run_id=payload["run_id"],
        path=Path(payload["path"]),
        status=payload["status"],
        timestamp_utc=payload.get("timestamp_utc", ""),
        cli_args=payload.get("cli_args", ""),
        output_root=payload.get("output_root", ""),
        repo_version=payload.get("repo_version", ""),
        dataset_count=payload.get("dataset_count", 0),
        dataset_group_label=payload.get("dataset_group_label", ""),
        has_window_summaries=payload.get("has_window_summaries", False),
        has_calibration_summaries=payload.get("has_calibration_summaries", False),
    )


def _table_columns(columns: list[str]) -> list[dict[str, str]]:
    return [{"name": column, "id": column} for column in columns]


def _filter_rows(rows: list[dict[str, Any]], filters: dict[str, list[Any]]) -> list[dict[str, Any]]:
    filtered = rows
    for field, allowed_values in filters.items():
        if not allowed_values:
            continue
        filtered = [row for row in filtered if row.get(field) in allowed_values]
    return filtered


def _dropdown_options(rows: list[dict[str, Any]], field: str) -> list[dict[str, Any]]:
    values = sorted({row.get(field) for row in rows if row.get(field) not in ("", None)})
    return [{"label": str(value), "value": value} for value in values]


def create_dash_app(results_root: str | Path = "build/results") -> Dash:
    results_root_path = Path(results_root)
    catalog = discover_runs(results_root_path)
    catalog_by_path = {str(entry.path): entry for entry in catalog}

    app = Dash(__name__, title="imuFactors Summary Viewer")
    app.layout = build_shell(catalog, results_root_path)

    @callback(
        Output("selected-run-store", "data"),
        Input("run-select", "value"),
    )
    def load_selected_run(run_path: str | None) -> dict[str, Any]:
        if not run_path:
            return {}
        entry = catalog_by_path.get(run_path)
        if entry is None:
            return {}
        return _serialize_run_data(load_run_data(entry))

    @callback(
        Output("run-header", "children"),
        Output("metadata-grid", "children"),
        Output("datasets-table", "columns"),
        Output("datasets-table", "data"),
        Input("selected-run-store", "data"),
    )
    def render_overview(run_payload: dict[str, Any]) -> tuple[Any, Any, Any, Any]:
        if not run_payload:
            message = empty_state("No run selected. Point the app at a results root with canonical CSV packages.")
            return message, [], [], []

        entry = _deserialize_entry(run_payload["entry"])
        dataset_label = entry.dataset_group_label or (
            f"{entry.dataset_count} datasets" if entry.dataset_count else "No dataset info"
        )
        header = html.Div(
            [
                html.Div(
                    [
                        html.H2(entry.app_name, style={"margin": 0, "fontSize": "30px", "color": "#2d241b"}),
                        html.Div(
                            [
                                status_badge(entry.status),
                                html.Span(
                                    dataset_label,
                                    style={"marginLeft": "12px", "color": "#7a6956", "fontSize": "14px"},
                                ),
                            ],
                            style={"marginTop": "10px"},
                        ),
                    ]
                ),
                html.P(
                    _format_cli_args(entry.cli_args),
                    style={
                        "marginTop": "18px",
                        "marginBottom": "0",
                        "padding": "12px 14px",
                        "backgroundColor": "#f8f4ed",
                        "border": "1px solid #d8cfbf",
                        "borderRadius": "14px",
                        "color": "#5f4f3e",
                        "fontFamily": "Menlo, Monaco, Consolas, monospace",
                        "fontSize": "12px",
                    },
                ),
            ],
            style={"marginBottom": "24px"},
        )

        metadata_grid = [
            metadata_card("Run ID", entry.run_id),
            metadata_card("Timestamp", entry.timestamp_utc or "Unknown"),
            metadata_card("Output Root", entry.output_root or str(entry.path.parent.parent)),
            metadata_card("Repo Version", entry.repo_version or "Unknown"),
        ]

        return (
            header,
            metadata_grid,
            _table_columns(run_payload["datasets_columns"]),
            run_payload["datasets_rows"],
        )

    @callback(
        Output("window-dataset-filter", "options"),
        Output("window-method-filter", "options"),
        Output("window-config-filter", "options"),
        Output("window-interval-filter", "options"),
        Output("window-summary-message", "children"),
        Input("selected-run-store", "data"),
    )
    def populate_window_filters(run_payload: dict[str, Any]) -> tuple[Any, Any, Any, Any, Any]:
        if not run_payload:
            message = empty_state("Select a run to inspect window summaries.")
            return [], [], [], [], message

        rows = run_payload["window_rows"]
        if not rows:
            return [], [], [], [], empty_state(
                "This run has no populated window summary rows. Detailed views and graphs are planned next."
            )

        return (
            _dropdown_options(rows, "dataset"),
            _dropdown_options(rows, "method"),
            _dropdown_options(rows, "config_label"),
            _dropdown_options(rows, "interval_seconds"),
            html.Div(
                "Summary tables are live. TODO breadcrumbs for window detail tables and Plotly trend panels remain in the code.",
                style={"color": "#7a6956", "fontSize": "13px"},
            ),
        )

    @callback(
        Output("window-summary-table", "columns"),
        Output("window-summary-table", "data"),
        Input("selected-run-store", "data"),
        Input("window-dataset-filter", "value"),
        Input("window-method-filter", "value"),
        Input("window-config-filter", "value"),
        Input("window-interval-filter", "value"),
    )
    def render_window_summaries(
        run_payload: dict[str, Any],
        datasets: list[Any] | None,
        methods: list[Any] | None,
        config_labels: list[Any] | None,
        intervals: list[Any] | None,
    ) -> tuple[Any, Any]:
        if not run_payload:
            return [], []

        rows = _filter_rows(
            run_payload["window_rows"],
            {
                "dataset": datasets or [],
                "method": methods or [],
                "config_label": config_labels or [],
                "interval_seconds": intervals or [],
            },
        )
        return _table_columns(run_payload["window_columns"]), rows

    @callback(
        Output("calibration-study-filter", "options"),
        Output("calibration-result-filter", "options"),
        Output("calibration-summary-message", "children"),
        Input("selected-run-store", "data"),
    )
    def populate_calibration_filters(run_payload: dict[str, Any]) -> tuple[Any, Any, Any]:
        if not run_payload:
            return [], [], empty_state("Select a run to inspect calibration summaries.")

        rows = run_payload["calibration_rows"]
        if not rows:
            return [], [], empty_state(
                "This run has no populated calibration summary rows. Multi-run calibration compare is deferred to a later version."
            )

        return (
            _dropdown_options(rows, "study_name"),
            _dropdown_options(rows, "result_label"),
            html.Div(
                "Calibration summaries are shown read-only in v1.",
                style={"color": "#7a6956", "fontSize": "13px"},
            ),
        )

    @callback(
        Output("calibration-summary-table", "columns"),
        Output("calibration-summary-table", "data"),
        Input("selected-run-store", "data"),
        Input("calibration-study-filter", "value"),
        Input("calibration-result-filter", "value"),
    )
    def render_calibration_summaries(
        run_payload: dict[str, Any],
        study_names: list[Any] | None,
        result_labels: list[Any] | None,
    ) -> tuple[Any, Any]:
        if not run_payload:
            return [], []

        rows = _filter_rows(
            run_payload["calibration_rows"],
            {"study_name": study_names or [], "result_label": result_labels or []},
        )
        return _table_columns(run_payload["calibration_columns"]), rows

    # TODO: Add multi-run comparison state once the MVP grows beyond a single selected run.
    # TODO: Add Plotly chart callbacks after detail tables and trajectory summaries are exposed.
    # TODO: Add window_metrics.csv and trajectory_samples.csv panels with explicit sampling controls.
    return app


def main() -> None:
    args = parse_args()
    app = create_dash_app(args.results_root)
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
