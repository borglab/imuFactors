"""Dash application entrypoint for NEES diagnostics viewing."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import pandas as pd
import plotly.graph_objects as go
from dash import Dash, Input, Output, State, callback, ctx, dash_table, dcc, html
from plotly.subplots import make_subplots

from .discovery import discover_runs
from .layout import (
    ACCENT,
    APP_BACKGROUND,
    MUTED,
    PANEL_BACKGROUND,
    PANEL_BORDER,
    TEXT,
    build_filter_dropdown,
    empty_state,
    format_run_option,
    metadata_card,
    status_badge,
)
from .loading import load_run_data, load_trajectory_samples
from .models import RunCatalogEntry, RunData
from .nees_diagnostics import (
    ERROR_COMPONENT_COLUMNS,
    NORMALIZED_COMPONENT_COLUMNS,
    TRAJECTORY_REQUIRED_COLUMNS,
    WHITENED_COMPONENT_COLUMNS,
    augment_trajectory_samples,
    covariance_to_correlation,
    rebuild_covariance_matrix,
)

WINDOW_PAYLOAD_FIELDS = (
    "dataset",
    "method",
    "config_label",
    "interval_seconds",
    "window_index",
    "window_start_time",
    "window_end_time",
)
OUTLIER_TABLE_COLUMNS = (
    "window_index",
    "dataset",
    "method",
    "config_label",
    "interval_seconds",
    "window_start_time",
    "window_end_time",
    "normalized_nees",
    "rot_error_norm",
    "pos_error_norm",
    "vel_error_norm",
)
OUTLIER_TABLE_LABELS = {
    "window_index": "Window",
    "dataset": "Dataset",
    "method": "Method",
    "config_label": "Config",
    "interval_seconds": "Interval (s)",
    "window_start_time": "Start Time",
    "window_end_time": "End Time",
    "normalized_nees": "Normalized NEES",
    "rot_error_norm": "Rot Error (deg)",
    "pos_error_norm": "Pos Error",
    "vel_error_norm": "Vel Error",
}
OUTLIER_ROTATION_ERROR_COLUMNS = {"rot_error_norm"}
COMPONENT_GROUPS = {
    "Rotation": NORMALIZED_COMPONENT_COLUMNS[:3],
    "Position": NORMALIZED_COMPONENT_COLUMNS[3:6],
    "Velocity": NORMALIZED_COMPONENT_COLUMNS[6:9],
}
COMPONENT_LABELS = {
    "norm_rot_x": "Rot X",
    "norm_rot_y": "Rot Y",
    "norm_rot_z": "Rot Z",
    "norm_pos_x": "Pos X",
    "norm_pos_y": "Pos Y",
    "norm_pos_z": "Pos Z",
    "norm_vel_x": "Vel X",
    "norm_vel_y": "Vel Y",
    "norm_vel_z": "Vel Z",
}
HEATMAP_COMPONENT_LABELS = ("rot_x", "rot_y", "rot_z", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z")
WHITENED_AXIS_OPTIONS = [
    {"label": column, "value": column} for column in WHITENED_COMPONENT_COLUMNS
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dash NEES diagnostics viewer for imuFactors result packages.")
    parser.add_argument(
        "--results-root",
        default="build/results",
        help="Root directory containing canonical result packages (default: build/results).",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Host interface for the Dash server.")
    parser.add_argument("--port", type=int, default=8051, help="Port for the Dash server.")
    parser.add_argument("--debug", action="store_true", help="Run Dash in debug mode.")
    return parser.parse_args()


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
        "has_window_metrics": entry.has_window_metrics,
        "has_trajectory_samples": entry.has_trajectory_samples,
        "has_window_summaries": entry.has_window_summaries,
        "has_calibration_summaries": entry.has_calibration_summaries,
    }


def _serialize_run_data(run_data: RunData) -> dict[str, Any]:
    return {
        "entry": _serialize_entry(run_data.entry),
        "metadata_rows": run_data.metadata.to_dict("records"),
        "datasets_rows": run_data.datasets.to_dict("records"),
        "window_metric_rows": run_data.window_metrics.to_dict("records"),
        "trajectory_index_rows": run_data.trajectory_index.to_dict("records"),
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
        has_window_metrics=payload.get("has_window_metrics", False),
        has_trajectory_samples=payload.get("has_trajectory_samples", False),
        has_window_summaries=payload.get("has_window_summaries", False),
        has_calibration_summaries=payload.get("has_calibration_summaries", False),
    )


def _serialize_catalog(entries: list[RunCatalogEntry]) -> list[dict[str, Any]]:
    return [_serialize_entry(entry) for entry in entries]


def _deserialize_catalog(payload: list[dict[str, Any]] | None) -> list[RunCatalogEntry]:
    if not payload:
        return []
    return [_deserialize_entry(entry_payload) for entry_payload in payload]


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


def _pretty_method_name(method: str) -> str:
    return method.replace("_", " ").title()


def _method_sort_key(method: str) -> tuple[int, str]:
    preferred_order = {
        "quadrature": 0,
        "manifold": 1,
        "tangent": 2,
        "gal3_imu_ekf": 3,
        "navstate_imu_ekf": 4,
    }
    return preferred_order.get(method, 99), method


def _build_data_table(table_id: str, *, row_selectable: str | bool = False, page_size: int = 12) -> dash_table.DataTable:
    return dash_table.DataTable(
        id=table_id,
        page_size=page_size,
        sort_action="native",
        filter_action="native",
        row_selectable=row_selectable,
        selected_rows=[],
        style_as_list_view=False,
        style_table={"overflowX": "auto"},
        style_header={
            "backgroundColor": "#efe7db",
            "border": f"1px solid {PANEL_BORDER}",
            "fontWeight": 700,
            "color": TEXT,
        },
        style_cell={
            "textAlign": "left",
            "padding": "10px 12px",
            "border": f"1px solid {PANEL_BORDER}",
            "backgroundColor": PANEL_BACKGROUND,
            "color": TEXT,
            "fontFamily": "Menlo, Monaco, Consolas, monospace",
            "fontSize": "12px",
            "maxWidth": 260,
            "whiteSpace": "normal",
        },
    )


def _placeholder_figure(title: str, message: str) -> go.Figure:
    figure = go.Figure()
    figure.update_layout(
        title=title,
        paper_bgcolor=PANEL_BACKGROUND,
        plot_bgcolor=PANEL_BACKGROUND,
        font={"color": TEXT},
        margin={"l": 40, "r": 20, "t": 60, "b": 40},
        xaxis={"visible": False},
        yaxis={"visible": False},
        annotations=[
            {
                "text": message,
                "xref": "paper",
                "yref": "paper",
                "x": 0.5,
                "y": 0.5,
                "showarrow": False,
                "font": {"size": 14, "color": MUTED},
            }
        ],
    )
    return figure


def _build_shell(catalog: list[RunCatalogEntry], results_root: Path) -> html.Div:
    options = [format_run_option(entry) for entry in catalog]
    default_value = options[0]["value"] if options else None

    return html.Div(
        [
            dcc.Store(id="catalog-store", data=_serialize_catalog(catalog)),
            dcc.Store(id="selected-run-store"),
            dcc.Store(id="active-window-store"),
            html.Div(
                [
                    html.Div(
                        [
                            html.H1("imuFactors NEES Diagnostics", style={"margin": "0 0 6px 0", "fontSize": "28px", "color": TEXT}),
                            html.P(
                                "Window-level and interval-level NEES diagnostics for canonical result packages.",
                                style={"margin": 0, "color": MUTED},
                            ),
                        ],
                        style={"marginBottom": "24px"},
                    ),
                    html.Div(
                        [
                            html.Div("Results Root", style={"fontSize": "12px", "color": MUTED}),
                            html.Code(str(results_root), style={"color": TEXT, "fontSize": "13px"}),
                        ],
                        style={
                            "padding": "14px 16px",
                            "borderRadius": "14px",
                            "backgroundColor": PANEL_BACKGROUND,
                            "border": f"1px solid {PANEL_BORDER}",
                            "marginBottom": "18px",
                        },
                    ),
                    html.Label("Run", htmlFor="run-select", style={"fontSize": "12px", "color": MUTED}),
                    dcc.Dropdown(
                        id="run-select",
                        options=options,
                        value=default_value,
                        placeholder="Select a discovered run",
                        clearable=False,
                    ),
                    html.Button(
                        "Refresh Runs",
                        id="refresh-runs-button",
                        n_clicks=0,
                        style={
                            "marginTop": "18px",
                            "width": "100%",
                            "padding": "12px 14px",
                            "borderRadius": "12px",
                            "border": f"1px solid {ACCENT}",
                            "backgroundColor": ACCENT,
                            "color": "#fff8ef",
                            "fontWeight": 700,
                            "cursor": "pointer",
                        },
                    ),
                    html.Div(id="run-list-note", children=f"{len(catalog)} runs discovered", style={"marginTop": "14px", "fontSize": "13px", "color": MUTED}),
                    html.Div(
                        [
                            html.Div("V1 focus", style={"fontSize": "12px", "color": MUTED}),
                            html.Ul(
                                [
                                    html.Li("Scalar NEES views for all methods"),
                                    html.Li("Rich interval detail for runs with trajectory samples"),
                                    html.Li("Single-run drill-down, no cross-run overlays"),
                                ],
                                style={"paddingLeft": "20px", "margin": "10px 0 0 0", "color": TEXT},
                            ),
                        ],
                        style={"marginTop": "24px"},
                    ),
                ],
                style={
                    "width": "320px",
                    "padding": "28px",
                    "borderRight": f"1px solid {PANEL_BORDER}",
                    "background": "linear-gradient(180deg, #f7f1e7 0%, #f2ebdf 100%)",
                },
            ),
            html.Div(
                [
                    html.Div(id="run-header"),
                    html.Div(id="run-metadata-grid", style={"display": "grid", "gap": "12px", "gridTemplateColumns": "repeat(auto-fit, minmax(220px, 1fr))"}),
                    html.Div(
                        [
                            build_filter_dropdown("dataset-filter", "Dataset"),
                            build_filter_dropdown("method-filter", "Method"),
                            build_filter_dropdown("config-filter", "Config Label"),
                            build_filter_dropdown("interval-filter", "Interval Seconds"),
                        ],
                        style={"display": "flex", "flexWrap": "wrap", "gap": "12px", "marginTop": "24px"},
                    ),
                    html.Div(id="overview-message", style={"marginTop": "14px"}),
                    html.Div(
                        [
                            dcc.Graph(id="overview-nees-graph", figure=_placeholder_figure("Window NEES", "Select a run to inspect NEES windows."), style={"flex": "2 1 480px"}),
                            dcc.Graph(id="overview-distribution-graph", figure=_placeholder_figure("NEES Distribution", "Select a run to inspect NEES windows."), style={"flex": "1 1 320px"}),
                        ],
                        style={"display": "flex", "flexWrap": "wrap", "gap": "16px", "marginTop": "18px"},
                    ),
                    html.Div(
                        [
                            html.H3("Outlier Windows", style={"color": TEXT, "marginBottom": "12px"}),
                            _build_data_table("outlier-table", row_selectable="single", page_size=10),
                        ],
                        style={"marginTop": "24px"},
                    ),
                    html.Div(
                        [
                            html.H3("Interval Detail", style={"color": TEXT, "marginBottom": "12px"}),
                            html.Div(id="interval-detail-status"),
                            html.Div(id="active-window-metadata", style={"display": "grid", "gap": "12px", "gridTemplateColumns": "repeat(auto-fit, minmax(180px, 1fr))", "marginTop": "16px"}),
                            html.Div(
                                [
                                    dcc.Graph(id="sample-nees-graph", figure=_placeholder_figure("Sample NEES", "Select an active window to inspect interval detail."), style={"flex": "1 1 320px"}),
                                    dcc.Graph(id="normalized-residual-graph", figure=_placeholder_figure("Normalized Residuals", "Select an active window to inspect interval detail."), style={"flex": "1 1 420px"}),
                                    dcc.Graph(id="predicted-sigma-graph", figure=_placeholder_figure("Predicted Sigmas", "Select an active window to inspect interval detail."), style={"flex": "1 1 320px"}),
                                ],
                                style={"display": "flex", "flexWrap": "wrap", "gap": "16px", "marginTop": "18px"},
                            ),
                        ],
                        style={"marginTop": "32px"},
                    ),
                    html.Div(
                        [
                            html.H3("Covariance Detail", style={"color": TEXT, "marginBottom": "12px"}),
                            html.Div(id="covariance-detail-status"),
                            html.Div(
                                [
                                    html.Div(
                                        [
                                            html.Label("Sample Timestamp", htmlFor="sample-timestamp-select", style={"fontSize": "12px", "color": MUTED}),
                                            dcc.Dropdown(id="sample-timestamp-select", placeholder="Select a sample in the active interval", clearable=False),
                                        ],
                                        style={"minWidth": "220px", "flex": "2 1 220px"},
                                    ),
                                    html.Div(
                                        [
                                            html.Label("Heatmap Mode", htmlFor="heatmap-mode", style={"fontSize": "12px", "color": MUTED}),
                                            dcc.Dropdown(
                                                id="heatmap-mode",
                                                options=[{"label": "Correlation", "value": "correlation"}, {"label": "Covariance", "value": "covariance"}],
                                                value="correlation",
                                                clearable=False,
                                            ),
                                        ],
                                        style={"minWidth": "180px", "flex": "1 1 180px"},
                                    ),
                                    html.Div(
                                        [
                                            html.Label("Whitened X", htmlFor="whitened-x-axis", style={"fontSize": "12px", "color": MUTED}),
                                            dcc.Dropdown(id="whitened-x-axis", options=WHITENED_AXIS_OPTIONS, value="w0", clearable=False),
                                        ],
                                        style={"minWidth": "160px", "flex": "1 1 160px"},
                                    ),
                                    html.Div(
                                        [
                                            html.Label("Whitened Y", htmlFor="whitened-y-axis", style={"fontSize": "12px", "color": MUTED}),
                                            dcc.Dropdown(id="whitened-y-axis", options=WHITENED_AXIS_OPTIONS, value="w1", clearable=False),
                                        ],
                                        style={"minWidth": "160px", "flex": "1 1 160px"},
                                    ),
                                ],
                                style={"display": "flex", "flexWrap": "wrap", "gap": "12px", "marginTop": "18px"},
                            ),
                            html.Div(
                                [
                                    dcc.Graph(id="covariance-heatmap", figure=_placeholder_figure("Covariance / Correlation", "Select an active interval sample."), style={"flex": "1 1 420px"}),
                                    dcc.Graph(id="whitened-scatter", figure=_placeholder_figure("Whitened Projection", "Select an active interval sample."), style={"flex": "1 1 420px"}),
                                ],
                                style={"display": "flex", "flexWrap": "wrap", "gap": "16px", "marginTop": "18px"},
                            ),
                        ],
                        style={"marginTop": "32px"},
                    ),
                ],
                style={"flex": "1", "padding": "28px", "backgroundColor": APP_BACKGROUND, "overflowY": "auto"},
            ),
        ],
        style={"display": "flex", "minHeight": "100vh", "backgroundColor": APP_BACKGROUND, "fontFamily": "Avenir Next, Helvetica Neue, sans-serif"},
    )


def _window_row_to_active_window(row: dict[str, Any]) -> dict[str, Any]:
    return {field: row.get(field) for field in WINDOW_PAYLOAD_FIELDS}


def _active_window_matches(row: dict[str, Any], active_window: dict[str, Any] | None) -> bool:
    if not active_window:
        return False
    return all(row.get(field) == active_window.get(field) for field in WINDOW_PAYLOAD_FIELDS)


def _build_outlier_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    sorted_rows = sorted(
        rows, key=lambda row: float(row.get("normalized_nees", -math.inf)),
        reverse=True)
    display_rows: list[dict[str, Any]] = []
    for row in sorted_rows:
        display_row = dict(row)
        for column in OUTLIER_ROTATION_ERROR_COLUMNS:
            if column in display_row and display_row[column] is not None:
                try:
                    display_row[column] = math.degrees(float(display_row[column]))
                except (TypeError, ValueError):
                    pass
        display_rows.append(display_row)
    return display_rows


def _outlier_table_columns(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    available_columns = [column for column in OUTLIER_TABLE_COLUMNS if any(column in row for row in rows)]
    return [{"name": OUTLIER_TABLE_LABELS.get(column, column), "id": column} for column in available_columns]


def _selection_has_trajectory_data(index_rows: list[dict[str, Any]], active_window: dict[str, Any] | None) -> bool:
    if not active_window:
        return False
    for row in index_rows:
        if (
            row.get("dataset") == active_window.get("dataset")
            and row.get("method") == active_window.get("method")
            and row.get("config_label") == active_window.get("config_label")
            and row.get("interval_seconds") == active_window.get("interval_seconds")
        ):
            return True
    return False


def _resolve_clicked_window(click_payload: dict[str, Any] | None) -> dict[str, Any] | None:
    if not click_payload or not click_payload.get("points"):
        return None
    point = click_payload["points"][0]
    customdata = point.get("customdata")
    if not isinstance(customdata, list) or len(customdata) != len(WINDOW_PAYLOAD_FIELDS):
        return None
    return {WINDOW_PAYLOAD_FIELDS[index]: customdata[index] for index in range(len(WINDOW_PAYLOAD_FIELDS))}


def _resolve_active_window(
    table_rows: list[dict[str, Any]],
    current_active: dict[str, Any] | None,
    selected_rows: list[int] | None,
    click_payload: dict[str, Any] | None,
    trigger_id: str | None,
) -> dict[str, Any]:
    if not table_rows:
        return {}
    if trigger_id == "overview-nees-graph":
        clicked_window = _resolve_clicked_window(click_payload)
        if clicked_window:
            return clicked_window
    if trigger_id == "outlier-table" and selected_rows:
        selected_index = selected_rows[0]
        if 0 <= selected_index < len(table_rows):
            return _window_row_to_active_window(table_rows[selected_index])
    for row in table_rows:
        if _active_window_matches(row, current_active):
            return _window_row_to_active_window(row)
    return _window_row_to_active_window(table_rows[0])


def _window_cards(active_window: dict[str, Any]) -> list[html.Div]:
    if not active_window:
        return []
    return [
        metadata_card("Dataset", str(active_window.get("dataset", "—"))),
        metadata_card("Method", _pretty_method_name(str(active_window.get("method", "—")))),
        metadata_card("Config", str(active_window.get("config_label", "—"))),
        metadata_card("Interval", str(active_window.get("interval_seconds", "—"))),
        metadata_card("Window", str(active_window.get("window_index", "—"))),
        metadata_card(
            "Time Range",
            f"{active_window.get('window_start_time', '—')} → {active_window.get('window_end_time', '—')}",
        ),
    ]


def _figure_layout(title: str) -> dict[str, Any]:
    return {
        "title": title,
        "paper_bgcolor": PANEL_BACKGROUND,
        "plot_bgcolor": PANEL_BACKGROUND,
        "font": {"color": TEXT},
        "margin": {"l": 48, "r": 20, "t": 60, "b": 48},
        "legend": {"orientation": "h", "yanchor": "bottom", "y": 1.02, "xanchor": "left", "x": 0.0},
    }


def _build_overview_figure(rows: list[dict[str, Any]]) -> go.Figure:
    if not rows:
        return _placeholder_figure("Window NEES", "No NEES windows match the current filters.")

    frame = pd.DataFrame(rows).sort_values(["method", "window_index"])
    figure = go.Figure()
    for method_name in sorted(frame["method"].dropna().unique().tolist(), key=_method_sort_key):
        method_frame = frame[frame["method"] == method_name]
        customdata = method_frame[list(WINDOW_PAYLOAD_FIELDS)].values.tolist()
        figure.add_trace(
            go.Scatter(
                x=method_frame["window_index"],
                y=method_frame["normalized_nees"],
                mode="lines+markers",
                name=_pretty_method_name(method_name),
                customdata=customdata,
                hovertemplate=(
                    "method=%{fullData.name}<br>"
                    "window=%{x}<br>"
                    "normalized_nees=%{y:.4f}<br>"
                    "interval=%{customdata[3]}<extra></extra>"
                ),
            )
        )
    figure.add_hline(y=1.0, line_dash="dash", line_color=ACCENT)
    figure.update_layout(**_figure_layout("Normalized NEES vs. Window Index"))
    figure.update_xaxes(title="Window Index", gridcolor="#e9decc")
    figure.update_yaxes(title="Normalized NEES", gridcolor="#e9decc")
    return figure


def _build_distribution_figure(rows: list[dict[str, Any]]) -> go.Figure:
    if not rows:
        return _placeholder_figure("NEES Distribution", "No NEES windows match the current filters.")

    frame = pd.DataFrame(rows)
    figure = go.Figure()
    for method_name in sorted(frame["method"].dropna().unique().tolist(), key=_method_sort_key):
        method_frame = frame[frame["method"] == method_name]
        figure.add_trace(
            go.Histogram(
                x=method_frame["normalized_nees"],
                name=_pretty_method_name(method_name),
                opacity=0.75,
            )
        )
    figure.add_vline(x=1.0, line_dash="dash", line_color=ACCENT)
    figure.update_layout(barmode="overlay", **_figure_layout("Normalized NEES Distribution"))
    figure.update_xaxes(title="Normalized NEES", gridcolor="#e9decc")
    figure.update_yaxes(title="Window Count", gridcolor="#e9decc")
    return figure


def _load_active_window_frame(entry: RunCatalogEntry, active_window: dict[str, Any]) -> tuple[pd.DataFrame, int]:
    frame = load_trajectory_samples(
        entry.path,
        TRAJECTORY_REQUIRED_COLUMNS,
        dataset=active_window.get("dataset"),
        method=active_window.get("method"),
        config_label=active_window.get("config_label"),
        interval_seconds=active_window.get("interval_seconds"),
        start_time=active_window.get("window_start_time"),
        end_time=active_window.get("window_end_time"),
    )
    return augment_trajectory_samples(frame)


def _detail_status_message(sample_count: int, skipped_count: int) -> html.Div:
    details = [f"{sample_count} trajectory samples in the active interval."]
    if skipped_count:
        details.append(f"{skipped_count} samples were skipped because covariance whitening failed.")
    else:
        details.append("All samples whitened successfully.")
    return html.Div(" ".join(details), style={"color": MUTED, "fontSize": "13px"})


def _build_sample_nees_figure(frame: pd.DataFrame) -> go.Figure:
    if frame.empty:
        return _placeholder_figure("Sample NEES", "No trajectory samples are available in the active interval.")
    figure = go.Figure()
    figure.add_trace(
        go.Scatter(
            x=frame["timestamp"],
            y=frame["sample_nees"],
            mode="lines+markers",
            name="Sample NEES",
            marker={"size": 6},
        )
    )
    figure.add_hline(y=1.0, line_dash="dash", line_color=ACCENT)
    figure.update_layout(**_figure_layout("Sample-Level Normalized NEES"))
    figure.update_xaxes(title="Timestamp", gridcolor="#e9decc")
    figure.update_yaxes(title="Sample NEES", gridcolor="#e9decc")
    return figure


def _build_normalized_residual_figure(frame: pd.DataFrame) -> go.Figure:
    if frame.empty:
        return _placeholder_figure("Normalized Residuals", "No trajectory samples are available in the active interval.")

    figure = make_subplots(rows=3, cols=1, shared_xaxes=True, subplot_titles=list(COMPONENT_GROUPS.keys()), vertical_spacing=0.08)
    for row_index, (group_label, component_columns) in enumerate(COMPONENT_GROUPS.items(), start=1):
        for column in component_columns:
            figure.add_trace(
                go.Scatter(
                    x=frame["timestamp"],
                    y=frame[column],
                    mode="lines",
                    name=COMPONENT_LABELS[column],
                    legendgroup=group_label,
                ),
                row=row_index,
                col=1,
            )
        figure.add_hline(y=0.0, line_dash="dot", line_color="#b39a79", row=row_index, col=1)
    figure.update_layout(**_figure_layout("Component-Normalized Residuals"))
    figure.update_xaxes(title="Timestamp", row=3, col=1, gridcolor="#e9decc")
    for row_index in range(1, 4):
        figure.update_yaxes(title="e / sigma", row=row_index, col=1, gridcolor="#e9decc")
    return figure


def _build_sigma_figure(frame: pd.DataFrame) -> go.Figure:
    if frame.empty:
        return _placeholder_figure("Predicted Sigmas", "No trajectory samples are available in the active interval.")

    figure = go.Figure()
    for column, label in (
        ("rot_pred_sigma", "Rotation"),
        ("pos_pred_sigma", "Position"),
        ("vel_pred_sigma", "Velocity"),
    ):
        figure.add_trace(go.Scatter(x=frame["timestamp"], y=frame[column], mode="lines", name=label))
    figure.update_layout(**_figure_layout("Predicted Sigma Traces"))
    figure.update_xaxes(title="Timestamp", gridcolor="#e9decc")
    figure.update_yaxes(title="Predicted Sigma", gridcolor="#e9decc")
    return figure


def _timestamp_options(frame: pd.DataFrame) -> list[dict[str, Any]]:
    return [{"label": f"{timestamp:.6g}", "value": timestamp} for timestamp in frame["timestamp"].tolist()]


def _select_sample_row(frame: pd.DataFrame, selected_timestamp: float | None) -> pd.Series | None:
    if frame.empty:
        return None
    if selected_timestamp is None:
        return frame.iloc[0]
    exact = frame[frame["timestamp"] == selected_timestamp]
    if not exact.empty:
        return exact.iloc[0]
    nearest_index = (frame["timestamp"] - selected_timestamp).abs().idxmin()
    return frame.loc[nearest_index]


def _build_heatmap_figure(sample_row: pd.Series | None, mode: str) -> go.Figure:
    if sample_row is None:
        return _placeholder_figure("Covariance / Correlation", "Select a sample in the active interval.")

    covariance = rebuild_covariance_matrix(sample_row)
    matrix = covariance_to_correlation(covariance) if mode == "correlation" else covariance
    colorbar_title = "Corr" if mode == "correlation" else "Cov"
    figure = go.Figure(
        data=go.Heatmap(
            z=matrix,
            x=HEATMAP_COMPONENT_LABELS,
            y=HEATMAP_COMPONENT_LABELS,
            colorscale="RdBu",
            zmid=0.0 if mode == "correlation" else None,
            colorbar={"title": colorbar_title},
        )
    )
    figure.update_layout(**_figure_layout("Correlation Heatmap" if mode == "correlation" else "Covariance Heatmap"))
    figure.update_xaxes(side="top")
    return figure


def _build_whitened_scatter(frame: pd.DataFrame, x_axis: str, y_axis: str) -> go.Figure:
    if frame.empty or x_axis not in frame.columns or y_axis not in frame.columns:
        return _placeholder_figure("Whitened Projection", "No whitened residuals are available in the active interval.")

    valid = frame[frame["whitening_ok"]].copy()
    if valid.empty:
        return _placeholder_figure("Whitened Projection", "All samples were skipped during covariance whitening.")

    figure = go.Figure()
    figure.add_trace(
        go.Scatter(
            x=valid[x_axis],
            y=valid[y_axis],
            mode="markers",
            marker={"size": 8, "color": valid["sample_nees"], "colorscale": "YlOrBr", "showscale": True, "colorbar": {"title": "NEES"}},
            text=[f"t={timestamp:.6g}" for timestamp in valid["timestamp"]],
            hovertemplate="%{text}<br>x=%{x:.4f}<br>y=%{y:.4f}<extra></extra>",
            name="Whitened samples",
        )
    )
    figure.add_hline(y=0.0, line_dash="dot", line_color="#b39a79")
    figure.add_vline(x=0.0, line_dash="dot", line_color="#b39a79")
    figure.update_layout(**_figure_layout(f"Whitened Projection: {x_axis} vs {y_axis}"))
    figure.update_xaxes(title=x_axis, gridcolor="#e9decc")
    figure.update_yaxes(title=y_axis, gridcolor="#e9decc")
    return figure


def create_dash_app(results_root: str | Path = "build/results") -> Dash:
    results_root_path = Path(results_root)
    catalog = discover_runs(results_root_path)

    app = Dash(__name__, title="imuFactors NEES Diagnostics")
    app.layout = _build_shell(catalog, results_root_path)

    @callback(
        Output("catalog-store", "data"),
        Output("run-select", "options"),
        Output("run-select", "value"),
        Output("run-list-note", "children"),
        Input("refresh-runs-button", "n_clicks"),
        State("run-select", "value"),
    )
    def refresh_catalog(
        _refresh_clicks: int,
        current_run: str | None,
    ) -> tuple[Any, Any, Any, Any]:
        refreshed_catalog = discover_runs(results_root_path)
        options = [format_run_option(entry) for entry in refreshed_catalog]
        values = {option["value"] for option in options}
        selected_value = current_run if current_run in values else (options[0]["value"] if options else None)
        note = f"{len(refreshed_catalog)} runs discovered"
        return _serialize_catalog(refreshed_catalog), options, selected_value, note

    @callback(
        Output("selected-run-store", "data"),
        Input("catalog-store", "data"),
        Input("run-select", "value"),
    )
    def load_selected_run(catalog_payload: list[dict[str, Any]] | None, run_path: str | None) -> dict[str, Any]:
        if not catalog_payload or not run_path:
            return {}
        catalog_by_path = {str(entry.path): entry for entry in _deserialize_catalog(catalog_payload)}
        entry = catalog_by_path.get(run_path)
        if entry is None:
            return {}
        return _serialize_run_data(load_run_data(entry))

    @callback(
        Output("run-header", "children"),
        Output("run-metadata-grid", "children"),
        Input("selected-run-store", "data"),
    )
    def render_run_header(run_payload: dict[str, Any]) -> tuple[Any, Any]:
        if not run_payload:
            return empty_state("No run selected. Point the app at a results root with canonical CSV packages."), []

        entry = _deserialize_entry(run_payload["entry"])
        dataset_label = entry.dataset_group_label or (
            f"{entry.dataset_count} datasets" if entry.dataset_count else "No dataset info"
        )
        header = html.Div(
            [
                html.Div(
                    [
                        html.H2(entry.app_name, style={"margin": 0, "fontSize": "30px", "color": TEXT}),
                        html.Div(
                            [
                                status_badge(entry.status),
                                html.Span(dataset_label, style={"marginLeft": "12px", "color": MUTED, "fontSize": "14px"}),
                            ],
                            style={"marginTop": "10px"},
                        ),
                    ]
                ),
                html.P(
                    entry.cli_args or "No CLI arguments recorded for this run.",
                    style={
                        "marginTop": "18px",
                        "marginBottom": "0",
                        "padding": "12px 14px",
                        "backgroundColor": "#f8f4ed",
                        "border": f"1px solid {PANEL_BORDER}",
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
            metadata_card("Window Rows", str(len(run_payload["window_metric_rows"]))),
            metadata_card("Trajectory Combos", str(len(run_payload["trajectory_index_rows"]))),
            metadata_card("Rich Diagnostics", "Available" if entry.has_trajectory_samples else "Not exported"),
        ]
        return header, metadata_grid

    @callback(
        Output("dataset-filter", "options"),
        Output("method-filter", "options"),
        Output("config-filter", "options"),
        Output("interval-filter", "options"),
        Output("overview-message", "children"),
        Output("overview-nees-graph", "figure"),
        Output("overview-distribution-graph", "figure"),
        Output("outlier-table", "columns"),
        Output("outlier-table", "data"),
        Input("selected-run-store", "data"),
        Input("dataset-filter", "value"),
        Input("method-filter", "value"),
        Input("config-filter", "value"),
        Input("interval-filter", "value"),
    )
    def render_overview(
        run_payload: dict[str, Any],
        datasets: list[Any] | None,
        methods: list[Any] | None,
        config_labels: list[Any] | None,
        intervals: list[Any] | None,
    ) -> tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any]:
        if not run_payload:
            message = empty_state("Select a run to inspect NEES diagnostics.")
            return [], [], [], [], message, _placeholder_figure("Window NEES", "Select a run to inspect NEES windows."), _placeholder_figure("NEES Distribution", "Select a run to inspect NEES windows."), [], []

        rows = run_payload["window_metric_rows"]
        if not rows:
            message = empty_state("This run has no populated window metrics. NEES diagnostics require window_metrics.csv rows.")
            return (
                [],
                [],
                [],
                [],
                message,
                _placeholder_figure("Window NEES", "This run has no NEES windows."),
                _placeholder_figure("NEES Distribution", "This run has no NEES windows."),
                [],
                [],
            )

        filtered_rows = _filter_rows(
            rows,
            {
                "dataset": datasets or [],
                "method": methods or [],
                "config_label": config_labels or [],
                "interval_seconds": intervals or [],
            },
        )
        overview_message: Any
        if filtered_rows:
            overview_message = html.Div(
                f"{len(filtered_rows)} windows match the current filters. Click the scatter or select an outlier row to inspect one interval.",
                style={"color": MUTED, "fontSize": "13px"},
            )
        else:
            overview_message = empty_state("No NEES windows remain for the current filter selection.")
        outlier_rows = _build_outlier_rows(filtered_rows)
        return (
            _dropdown_options(rows, "dataset"),
            _dropdown_options(rows, "method"),
            _dropdown_options(rows, "config_label"),
            _dropdown_options(rows, "interval_seconds"),
            overview_message,
            _build_overview_figure(filtered_rows),
            _build_distribution_figure(filtered_rows),
            _outlier_table_columns(outlier_rows),
            outlier_rows,
        )

    @callback(
        Output("active-window-store", "data"),
        Input("outlier-table", "data"),
        Input("outlier-table", "selected_rows"),
        Input("overview-nees-graph", "clickData"),
        State("active-window-store", "data"),
    )
    def update_active_window(
        table_rows: list[dict[str, Any]] | None,
        selected_rows: list[int] | None,
        click_payload: dict[str, Any] | None,
        current_active: dict[str, Any] | None,
    ) -> dict[str, Any]:
        return _resolve_active_window(table_rows or [], current_active, selected_rows, click_payload, ctx.triggered_id)

    @callback(
        Output("outlier-table", "selected_rows"),
        Input("outlier-table", "data"),
        Input("active-window-store", "data"),
    )
    def sync_selected_row(table_rows: list[dict[str, Any]] | None, active_window: dict[str, Any] | None) -> list[int]:
        if not table_rows or not active_window:
            return []
        for index, row in enumerate(table_rows):
            if _active_window_matches(row, active_window):
                return [index]
        return []

    @callback(
        Output("interval-detail-status", "children"),
        Output("active-window-metadata", "children"),
        Output("sample-nees-graph", "figure"),
        Output("normalized-residual-graph", "figure"),
        Output("predicted-sigma-graph", "figure"),
        Output("sample-timestamp-select", "options"),
        Output("sample-timestamp-select", "value"),
        Input("selected-run-store", "data"),
        Input("active-window-store", "data"),
    )
    def render_interval_detail(run_payload: dict[str, Any], active_window: dict[str, Any]) -> tuple[Any, Any, Any, Any, Any, Any, Any]:
        if not run_payload or not active_window:
            message = empty_state("Select an active window to inspect interval detail.")
            placeholder = _placeholder_figure("Sample NEES", "Select an active window to inspect interval detail.")
            return message, [], placeholder, _placeholder_figure("Normalized Residuals", "Select an active window to inspect interval detail."), _placeholder_figure("Predicted Sigmas", "Select an active window to inspect interval detail."), [], None

        metadata_children = _window_cards(active_window)
        if not _selection_has_trajectory_data(run_payload["trajectory_index_rows"], active_window):
            message = empty_state("Rich interval diagnostics are unavailable for this method or run because no trajectory samples were exported.")
            return message, metadata_children, _placeholder_figure("Sample NEES", "Rich interval diagnostics are unavailable for this method or run."), _placeholder_figure("Normalized Residuals", "Rich interval diagnostics are unavailable for this method or run."), _placeholder_figure("Predicted Sigmas", "Rich interval diagnostics are unavailable for this method or run."), [], None

        entry = _deserialize_entry(run_payload["entry"])
        detail_frame, skipped_count = _load_active_window_frame(entry, active_window)
        if detail_frame.empty:
            message = empty_state("No trajectory rows were found for the active interval after timestamp filtering.")
            return message, metadata_children, _placeholder_figure("Sample NEES", "No trajectory rows were found for the active interval."), _placeholder_figure("Normalized Residuals", "No trajectory rows were found for the active interval."), _placeholder_figure("Predicted Sigmas", "No trajectory rows were found for the active interval."), [], None

        timestamp_options = _timestamp_options(detail_frame)
        default_timestamp = timestamp_options[0]["value"] if timestamp_options else None
        return (
            _detail_status_message(len(detail_frame), skipped_count),
            metadata_children,
            _build_sample_nees_figure(detail_frame),
            _build_normalized_residual_figure(detail_frame),
            _build_sigma_figure(detail_frame),
            timestamp_options,
            default_timestamp,
        )

    @callback(
        Output("covariance-detail-status", "children"),
        Output("covariance-heatmap", "figure"),
        Output("whitened-scatter", "figure"),
        Input("selected-run-store", "data"),
        Input("active-window-store", "data"),
        Input("sample-timestamp-select", "value"),
        Input("heatmap-mode", "value"),
        Input("whitened-x-axis", "value"),
        Input("whitened-y-axis", "value"),
    )
    def render_covariance_detail(
        run_payload: dict[str, Any],
        active_window: dict[str, Any],
        selected_timestamp: float | None,
        heatmap_mode: str,
        whitened_x_axis: str,
        whitened_y_axis: str,
    ) -> tuple[Any, Any, Any]:
        if not run_payload or not active_window:
            message = empty_state("Select an active interval sample to inspect covariance detail.")
            return message, _placeholder_figure("Covariance / Correlation", "Select an active interval sample."), _placeholder_figure("Whitened Projection", "Select an active interval sample.")

        if not _selection_has_trajectory_data(run_payload["trajectory_index_rows"], active_window):
            message = empty_state("Rich interval diagnostics are unavailable for this method or run because no trajectory samples were exported.")
            return message, _placeholder_figure("Covariance / Correlation", "Rich interval diagnostics are unavailable for this method or run."), _placeholder_figure("Whitened Projection", "Rich interval diagnostics are unavailable for this method or run.")

        entry = _deserialize_entry(run_payload["entry"])
        detail_frame, skipped_count = _load_active_window_frame(entry, active_window)
        if detail_frame.empty:
            message = empty_state("No trajectory rows were found for the active interval after timestamp filtering.")
            return message, _placeholder_figure("Covariance / Correlation", "No trajectory rows were found for the active interval."), _placeholder_figure("Whitened Projection", "No trajectory rows were found for the active interval.")

        sample_row = _select_sample_row(detail_frame, selected_timestamp)
        status = html.Div(
            f"Selected sample timestamp: {sample_row['timestamp']:.6g}. {len(detail_frame)} samples in the interval, {skipped_count} skipped during whitening.",
            style={"color": MUTED, "fontSize": "13px"},
        )
        return (
            status,
            _build_heatmap_figure(sample_row, heatmap_mode),
            _build_whitened_scatter(detail_frame, whitened_x_axis, whitened_y_axis),
        )

    return app


def main() -> None:
    args = parse_args()
    app = create_dash_app(args.results_root)
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
