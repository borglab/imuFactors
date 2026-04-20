"""Dash application entrypoint for summary-table viewing."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import pandas as pd
from dash import Dash, Input, Output, State, callback, html

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


def _serialize_entry(entry: RunCatalogEntry) -> dict[str, Any]:
    return {
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
    }


def _serialize_run_data(run_data: RunData) -> dict[str, Any]:
    entry = run_data.entry
    return {
        "entry": _serialize_entry(entry),
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


def _serialize_catalog(entries: list[RunCatalogEntry]) -> list[dict[str, Any]]:
    return [_serialize_entry(entry) for entry in entries]


def _deserialize_catalog(payload: list[dict[str, Any]] | None) -> list[RunCatalogEntry]:
    if not payload:
        return []
    return [_deserialize_entry(entry_payload) for entry_payload in payload]


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


WINDOW_COMPARISON_METRIC_GROUPS = {
    "core": [
        "normalized_nees_mean",
        "normalized_nees_median",
        "rot_error_median",
        "pos_error_median",
        "vel_error_median",
    ],
    "nees": [
        "normalized_nees_mean",
        "normalized_nees_median",
        "normalized_nees_p95",
        "normalized_nees_variance",
    ],
    "errors": [
        "rot_error_median",
        "pos_error_median",
        "vel_error_median",
    ],
    "sigmas": [
        "rot_pred_sigma_median",
        "pos_pred_sigma_median",
        "vel_pred_sigma_median",
    ],
    "all": [
        "normalized_nees_mean",
        "normalized_nees_median",
        "normalized_nees_p95",
        "normalized_nees_variance",
        "rot_error_median",
        "rot_pred_sigma_median",
        "pos_error_median",
        "pos_pred_sigma_median",
        "vel_error_median",
        "vel_pred_sigma_median",
    ],
}

WINDOW_METRIC_LABELS = {
    "normalized_nees_mean": "Normalized NEES Mean",
    "normalized_nees_median": "Normalized NEES Median",
    "normalized_nees_p95": "Normalized NEES P95",
    "normalized_nees_variance": "Normalized NEES Variance",
    "rot_error_median": "Rotation Error Median",
    "rot_pred_sigma_median": "Rotation Sigma Median",
    "pos_error_median": "Position Error Median",
    "pos_pred_sigma_median": "Position Sigma Median",
    "vel_error_median": "Velocity Error Median",
    "vel_pred_sigma_median": "Velocity Sigma Median",
}

WINDOW_COMPARISON_INDEX_COLUMNS = [
    ("dataset", "Dataset"),
    ("interval_seconds", "Interval Seconds"),
    ("config_label", "Config Label"),
    ("samples_per_window", "Samples / Window"),
    ("sample_count", "Sample Count"),
]

WINDOW_METRICS_CLOSE_TO_ONE = {
    "normalized_nees_mean",
    "normalized_nees_median",
    "normalized_nees_p95",
}


def _method_sort_key(method: str) -> tuple[int, str]:
    preferred_order = {
        "quadrature": 0,
        "manifold": 1,
        "tangent": 2,
        "gal3_imu_ekf": 3,
        "navstate_imu_ekf": 4,
    }
    return preferred_order.get(method, 99), method


def _pretty_method_name(method: str) -> str:
    return method.replace("_", " ").title()


def _format_significant_digits(value: Any, digits: int = 4) -> Any:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if pd.isna(value):
        return ""
    if isinstance(value, (int, float)):
        return format(float(value), f".{digits}g")
    return value


def _metric_score(metric: str, value: Any) -> float | None:
    if value is None or pd.isna(value):
        return None
    numeric_value = float(value)
    if metric in WINDOW_METRICS_CLOSE_TO_ONE:
        return abs(numeric_value - 1.0)
    return numeric_value


def _is_same_score(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-9, abs_tol=1e-12)


def _build_window_method_comparison(
    rows: list[dict[str, Any]], metric_group: str
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    if not rows:
        return [], [], []

    frame = pd.DataFrame(rows)
    if frame.empty or "method" not in frame.columns:
        return [], [], []

    metrics = [
        metric
        for metric in WINDOW_COMPARISON_METRIC_GROUPS.get(metric_group, WINDOW_COMPARISON_METRIC_GROUPS["core"])
        if metric in frame.columns
    ]
    if not metrics:
        return [], [], []

    index_columns = [column for column, _label in WINDOW_COMPARISON_INDEX_COLUMNS if column in frame.columns]
    if not index_columns:
        return [], [], []

    method_names = sorted(frame["method"].dropna().unique().tolist(), key=_method_sort_key)
    if not method_names:
        return [], [], []

    grouped = frame[index_columns + ["method", *metrics]].copy()
    pivot = grouped.pivot_table(index=index_columns, columns="method", values=metrics, aggfunc="first")
    if pivot.empty:
        return [], [], []

    columns: list[dict[str, Any]] = []
    for column, label in WINDOW_COMPARISON_INDEX_COLUMNS:
        if column in index_columns:
            columns.append({"name": [label, ""], "id": column})

    for metric in metrics:
        metric_label = WINDOW_METRIC_LABELS.get(metric, metric)
        for method_name in method_names:
            column_id = f"{metric}__{method_name}"
            columns.append(
                {
                    "name": [metric_label, _pretty_method_name(method_name)],
                    "id": column_id,
                }
            )

    comparison_rows: list[dict[str, Any]] = []
    style_rules: list[dict[str, Any]] = []
    for index_values, series in pivot.iterrows():
        if not isinstance(index_values, tuple):
            index_values = (index_values,)
        row: dict[str, Any] = {
            index_columns[index]: index_values[index] for index in range(len(index_columns))
        }
        row_index = len(comparison_rows)
        for metric in metrics:
            metric_scores: list[tuple[str, float]] = []
            for method_name in method_names:
                column_id = f"{metric}__{method_name}"
                value = series.get((metric, method_name))
                row[column_id] = _format_significant_digits(value)
                score = _metric_score(metric, value)
                if score is not None:
                    metric_scores.append((column_id, score))

            if metric_scores:
                best_score = min(score for _column_id, score in metric_scores)
                second_best_score = next(
                    (
                        score
                        for score in sorted(score for _column_id, score in metric_scores)
                        if not _is_same_score(score, best_score)
                    ),
                    None,
                )
                for column_id, score in metric_scores:
                    if _is_same_score(score, best_score):
                        style_rules.append(
                            {"if": {"row_index": row_index, "column_id": column_id}, "fontWeight": "700"}
                        )
                    elif second_best_score is not None and _is_same_score(score, second_best_score):
                        style_rules.append(
                            {
                                "if": {"row_index": row_index, "column_id": column_id},
                                "textDecoration": "underline",
                                "textDecorationThickness": "2px",
                            }
                        )
        comparison_rows.append(row)

    return columns, comparison_rows, style_rules


def _resolve_compare_paths(selected_path: str | None, compare_paths: list[str] | None) -> list[str]:
    resolved: list[str] = []
    for path in [selected_path, *(compare_paths or [])]:
        if path and path not in resolved:
            resolved.append(path)
    return resolved


def _load_payload_for_entry(entry: RunCatalogEntry) -> dict[str, Any]:
    return _serialize_run_data(load_run_data(entry))


def _load_compare_payloads(
    catalog_payload: list[dict[str, Any]] | None,
    selected_path: str | None,
    compare_paths: list[str] | None,
) -> list[dict[str, Any]]:
    catalog = _deserialize_catalog(catalog_payload)
    catalog_by_path = {str(entry.path): entry for entry in catalog}
    payloads: list[dict[str, Any]] = []
    for path in _resolve_compare_paths(selected_path, compare_paths):
        entry = catalog_by_path.get(path)
        if entry is None:
            continue
        payloads.append(_load_payload_for_entry(entry))
    return payloads


def _comparison_rows(run_payloads: list[dict[str, Any]], row_key: str) -> tuple[list[str], list[dict[str, Any]]]:
    if not run_payloads:
        return [], []

    base_columns: list[str] = []
    combined_rows: list[dict[str, Any]] = []
    comparison_fields = {"run_label", "app_name", "run_id", "timestamp_utc", "status"}
    for payload in run_payloads:
        entry = _deserialize_entry(payload["entry"])
        rows = payload[row_key]
        if rows and not base_columns:
            base_columns = [column for column in rows[0].keys() if column not in comparison_fields]
        for row in rows:
            payload_row = {key: value for key, value in row.items() if key not in comparison_fields}
            combined_rows.append(
                {
                    "run_label": f"{entry.app_name} | {entry.run_id}",
                    "app_name": entry.app_name,
                    "run_id": entry.run_id,
                    "timestamp_utc": entry.timestamp_utc,
                    "status": entry.status,
                    **payload_row,
                }
            )

    columns = ["run_label", "app_name", "run_id", "timestamp_utc", "status", *base_columns]
    return columns, combined_rows


def create_dash_app(results_root: str | Path = "build/results") -> Dash:
    results_root_path = Path(results_root)
    catalog = discover_runs(results_root_path)

    app = Dash(__name__, title="imuFactors Summary Viewer")
    app.layout = build_shell(catalog, results_root_path)

    @callback(
        Output("catalog-store", "data"),
        Output("run-select", "options"),
        Output("run-select", "value"),
        Output("compare-select", "options"),
        Output("compare-select", "value"),
        Output("run-list-note", "children"),
        Input("refresh-runs-button", "n_clicks"),
        State("run-select", "value"),
        State("compare-select", "value"),
    )
    def refresh_catalog(
        _refresh_clicks: int,
        current_run: str | None,
        current_compare: list[str] | None,
    ) -> tuple[Any, Any, Any, Any, Any, Any]:
        refreshed_catalog = discover_runs(results_root_path)
        options = [
            {
                "label": f"{entry.app_name} | {entry.run_id} | "
                f"{entry.dataset_group_label or (f'{entry.dataset_count} datasets' if entry.dataset_count else 'No dataset info')} | "
                f"{entry.timestamp_utc or 'Unknown timestamp'}",
                "value": str(entry.path),
            }
            for entry in refreshed_catalog
        ]
        values = {option["value"] for option in options}
        selected_value = current_run if current_run in values else (options[0]["value"] if options else None)
        compare_values = [path for path in (current_compare or []) if path in values]
        if selected_value and selected_value not in compare_values:
            compare_values = [selected_value, *compare_values]
        note = f"{len(refreshed_catalog)} runs discovered"
        return (
            _serialize_catalog(refreshed_catalog),
            options,
            selected_value,
            options,
            compare_values,
            note,
        )

    @callback(
        Output("selected-run-store", "data"),
        Input("catalog-store", "data"),
        Input("run-select", "value"),
    )
    def load_selected_run(catalog_payload: list[dict[str, Any]] | None, run_path: str | None) -> dict[str, Any]:
        if not catalog_payload or not run_path:
            return {}
        catalog = _deserialize_catalog(catalog_payload)
        catalog_by_path = {str(entry.path): entry for entry in catalog}
        entry = catalog_by_path.get(run_path)
        if entry is None:
            return {}
        return _load_payload_for_entry(entry)

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
        Output("window-comparison-message", "children"),
        Output("window-method-compare-table", "columns"),
        Output("window-method-compare-table", "data"),
        Output("window-method-compare-table", "style_data_conditional"),
        Input("selected-run-store", "data"),
        Input("window-comparison-metric-set", "value"),
        Input("window-dataset-filter", "value"),
        Input("window-method-filter", "value"),
        Input("window-config-filter", "value"),
        Input("window-interval-filter", "value"),
    )
    def populate_window_filters(
        run_payload: dict[str, Any],
        comparison_metric_set: str,
        selected_datasets: list[Any] | None,
        selected_methods: list[Any] | None,
        selected_configs: list[Any] | None,
        selected_intervals: list[Any] | None,
    ) -> tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any]:
        if not run_payload:
            message = empty_state("Select a run to inspect window summaries.")
            return [], [], [], [], message, message, [], [], []

        rows = run_payload["window_rows"]
        if not rows:
            message = empty_state(
                "This run has no populated window summary rows. Detailed views and graphs are planned next."
            )
            return [], [], [], [], message, message, [], [], []

        filtered_rows = _filter_rows(
            rows,
            {
                "dataset": selected_datasets or [],
                "method": selected_methods or [],
                "config_label": selected_configs or [],
                "interval_seconds": selected_intervals or [],
            },
        )
        comparison_columns, comparison_rows, comparison_styles = _build_window_method_comparison(
            filtered_rows, comparison_metric_set
        )

        if comparison_rows:
            comparison_message: Any = html.Div(
                "Methods are pivoted side-by-side for the current dataset and interval filters.",
                style={"color": "#7a6956", "fontSize": "13px"},
            )
        else:
            comparison_message = empty_state(
                "No comparable method rows remain for the current filters and metric set."
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
            comparison_message,
            comparison_columns,
            comparison_rows,
            comparison_styles,
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
                "This run has no populated calibration summary rows. Comparison is still available when other selected runs contain calibration summaries."
            )

        return (
            _dropdown_options(rows, "study_name"),
            _dropdown_options(rows, "result_label"),
            html.Div(
                "Calibration summaries are shown read-only.",
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

    @callback(
        Output("compare-summary-message", "children"),
        Output("compare-window-dataset-filter", "options"),
        Output("compare-window-method-filter", "options"),
        Output("compare-window-config-filter", "options"),
        Output("compare-window-interval-filter", "options"),
        Output("compare-calibration-study-filter", "options"),
        Output("compare-calibration-result-filter", "options"),
        Output("compare-window-table", "columns"),
        Output("compare-window-table", "data"),
        Output("compare-calibration-table", "columns"),
        Output("compare-calibration-table", "data"),
        Input("catalog-store", "data"),
        Input("run-select", "value"),
        Input("compare-select", "value"),
        Input("compare-window-dataset-filter", "value"),
        Input("compare-window-method-filter", "value"),
        Input("compare-window-config-filter", "value"),
        Input("compare-window-interval-filter", "value"),
        Input("compare-calibration-study-filter", "value"),
        Input("compare-calibration-result-filter", "value"),
    )
    def render_comparisons(
        catalog_payload: list[dict[str, Any]] | None,
        selected_path: str | None,
        compare_paths: list[str] | None,
        compare_window_datasets: list[Any] | None,
        compare_window_methods: list[Any] | None,
        compare_window_configs: list[Any] | None,
        compare_window_intervals: list[Any] | None,
        compare_study_names: list[Any] | None,
        compare_result_labels: list[Any] | None,
    ) -> tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any, Any, Any]:
        payloads = _load_compare_payloads(catalog_payload, selected_path, compare_paths)
        if not payloads:
            message = empty_state("Select an active run to compare summaries.")
            return message, [], [], [], [], [], [], [], [], [], []

        window_columns, window_rows = _comparison_rows(payloads, "window_rows")
        calibration_columns, calibration_rows = _comparison_rows(payloads, "calibration_rows")

        filtered_window_rows = _filter_rows(
            window_rows,
            {
                "dataset": compare_window_datasets or [],
                "method": compare_window_methods or [],
                "config_label": compare_window_configs or [],
                "interval_seconds": compare_window_intervals or [],
            },
        )
        filtered_calibration_rows = _filter_rows(
            calibration_rows,
            {"study_name": compare_study_names or [], "result_label": compare_result_labels or []},
        )

        compared_run_count = len(payloads)
        compared_label = "run" if compared_run_count == 1 else "runs"
        if not window_rows and not calibration_rows:
            message = empty_state("The selected runs do not contain populated summary rows to compare.")
        else:
            message = html.Div(
                f"Comparing {compared_run_count} {compared_label}. "
                "Window and calibration summaries are merged into shared comparison tables.",
                style={"color": "#7a6956", "fontSize": "13px"},
            )

        return (
            message,
            _dropdown_options(window_rows, "dataset"),
            _dropdown_options(window_rows, "method"),
            _dropdown_options(window_rows, "config_label"),
            _dropdown_options(window_rows, "interval_seconds"),
            _dropdown_options(calibration_rows, "study_name"),
            _dropdown_options(calibration_rows, "result_label"),
            _table_columns(window_columns),
            filtered_window_rows,
            _table_columns(calibration_columns),
            filtered_calibration_rows,
        )

    # TODO: Add Plotly chart callbacks after detail tables and trajectory summaries are exposed.
    # TODO: Add window_metrics.csv and trajectory_samples.csv panels with explicit sampling controls.
    return app


def main() -> None:
    args = parse_args()
    app = create_dash_app(args.results_root)
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
