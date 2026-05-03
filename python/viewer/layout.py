"""Dash layout helpers and styling for the summary viewer."""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from dash import dash_table, dcc, html

from .models import RunCatalogEntry

APP_BACKGROUND = "#f4f1ea"
PANEL_BACKGROUND = "#fcfaf6"
PANEL_BORDER = "#d8cfbf"
ACCENT = "#8f5d2b"
TEXT = "#2d241b"
MUTED = "#7a6956"


def format_local_timestamp(timestamp_utc: str) -> str:
    """Format an ISO UTC timestamp in the server's local timezone."""

    if not timestamp_utc:
        return "Unknown timestamp"
    try:
        normalized = timestamp_utc.replace("Z", "+00:00")
        parsed = datetime.fromisoformat(normalized)
    except ValueError:
        return timestamp_utc
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=UTC)
    local_time = parsed.astimezone()
    return local_time.strftime("%Y-%m-%d %H:%M:%S %Z")


def _status_colors(status: str) -> tuple[str, str]:
    if status == "ready":
        return "#e5f4e8", "#1f6b36"
    return "#fff2d9", "#8a5a10"


def format_run_option(entry: RunCatalogEntry) -> dict[str, str]:
    dataset_label = entry.dataset_group_label or (
        f"{entry.dataset_count} datasets" if entry.dataset_count else "No dataset info"
    )
    timestamp = format_local_timestamp(entry.timestamp_utc)
    label = f"{entry.app_name} | {entry.run_id} | {dataset_label} | {timestamp}"
    return {"label": label, "value": str(entry.path)}


def status_badge(status: str) -> html.Span:
    background, foreground = _status_colors(status)
    return html.Span(
        status.upper(),
        style={
            "display": "inline-block",
            "padding": "4px 10px",
            "borderRadius": "999px",
            "backgroundColor": background,
            "color": foreground,
            "fontSize": "12px",
            "fontWeight": 700,
            "letterSpacing": "0.08em",
        },
    )


def empty_state(message: str) -> html.Div:
    return html.Div(
        message,
        style={
            "padding": "20px",
            "border": f"1px dashed {PANEL_BORDER}",
            "borderRadius": "14px",
            "color": MUTED,
            "backgroundColor": "#f8f4ed",
        },
    )


def metadata_card(label: str, value: str) -> html.Div:
    return html.Div(
        [
            html.Div(label, style={"fontSize": "12px", "color": MUTED, "textTransform": "uppercase"}),
            html.Div(value or "—", style={"fontSize": "15px", "fontWeight": 600, "color": TEXT}),
        ],
        style={
            "padding": "14px 16px",
            "backgroundColor": PANEL_BACKGROUND,
            "border": f"1px solid {PANEL_BORDER}",
            "borderRadius": "14px",
        },
    )


def build_filter_dropdown(component_id: str, label: str) -> html.Div:
    return html.Div(
        [
            html.Label(label, htmlFor=component_id, style={"fontSize": "12px", "color": MUTED}),
            dcc.Dropdown(id=component_id, multi=True, placeholder=f"Filter {label.lower()}"),
        ],
        style={"minWidth": "180px", "flex": "1 1 180px"},
    )


def build_data_table(table_id: str, merge_duplicate_headers: bool = False) -> dash_table.DataTable:
    return dash_table.DataTable(
        id=table_id,
        page_action="none",
        sort_action="native",
        filter_action="none",
        fixed_rows={"headers": True},
        merge_duplicate_headers=merge_duplicate_headers,
        style_as_list_view=False,
        virtualization=True,
        style_table={
            "height": "560px",
            "overflowX": "auto",
            "overflowY": "auto",
        },
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
            "maxWidth": 320,
            "overflow": "hidden",
            "textOverflow": "ellipsis",
            "whiteSpace": "nowrap",
        },
    )


def build_shell(
    catalog: list[RunCatalogEntry],
    results_root: Path,
    catalog_data: list[dict[str, Any]] | None = None,
) -> html.Div:
    options = [format_run_option(entry) for entry in catalog]
    default_value = options[0]["value"] if options else None

    return html.Div(
        [
            dcc.Store(id="catalog-store", data=catalog_data or []),
            dcc.Store(id="selected-run-store"),
            html.Div(
                [
                    html.Div(
                        [
                            html.H1(
                                "imuFactors Summary Viewer",
                                style={"margin": "0 0 6px 0", "fontSize": "28px", "color": TEXT},
                            ),
                            html.P(
                                "CSV-backed Dash viewer for canonical result packages.",
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
                    html.Div(style={"height": "16px"}),
                    html.Label("Compare Against", htmlFor="compare-select", style={"fontSize": "12px", "color": MUTED}),
                    dcc.Dropdown(
                        id="compare-select",
                        options=options,
                        value=[default_value] if default_value else [],
                        placeholder="Select additional runs for comparison",
                        multi=True,
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
                    html.Div(
                        id="run-list-note",
                        children=f"{len(catalog)} runs discovered",
                        style={"marginTop": "14px", "fontSize": "13px", "color": MUTED},
                    ),
                            html.Div(
                                [
                                    html.Div("Planned next:", style={"fontSize": "12px", "color": MUTED}),
                                    html.Ul(
                                        [
                                            html.Li("Window detail tables"),
                                            html.Li("Trajectory views and graphs"),
                                            html.Li("Plotly trend panels"),
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
                    dcc.Tabs(
                        id="summary-tabs",
                        value="overview",
                        children=[
                            dcc.Tab(
                                label="Overview",
                                value="overview",
                                children=[
                                    html.Div(
                                        id="metadata-grid",
                                        style={
                                            "display": "grid",
                                            "gap": "12px",
                                            "gridTemplateColumns": "repeat(auto-fit, minmax(220px, 1fr))",
                                        },
                                    ),
                                    html.Div(
                                        [
                                            html.H3("Datasets", style={"color": TEXT}),
                                            build_data_table("datasets-table"),
                                        ],
                                        style={"marginTop": "24px"},
                                    ),
                                ],
                            ),
                            dcc.Tab(
                                label="Window Summaries",
                                value="window-summaries",
                                children=[
                                    html.Div(
                                        [
                                            build_filter_dropdown("window-dataset-filter", "Dataset"),
                                            build_filter_dropdown("window-method-filter", "Method"),
                                            build_filter_dropdown("window-config-filter", "Config Label"),
                                            build_filter_dropdown("window-interval-filter", "Interval Seconds"),
                                            html.Div(
                                                [
                                                    html.Label(
                                                        "Comparison Metric Set",
                                                        htmlFor="window-comparison-metric-set",
                                                        style={"fontSize": "12px", "color": MUTED},
                                                    ),
                                                    dcc.Dropdown(
                                                        id="window-comparison-metric-set",
                                                        options=[
                                                            {"label": "Core Metrics", "value": "core"},
                                                            {"label": "NEES Statistics", "value": "nees"},
                                                            {"label": "Error Medians", "value": "errors"},
                                                            {"label": "Sigma Medians", "value": "sigmas"},
                                                            {"label": "All Summary Metrics", "value": "all"},
                                                        ],
                                                        value="core",
                                                        clearable=False,
                                                    ),
                                                ],
                                                style={"minWidth": "220px", "flex": "1 1 220px"},
                                            ),
                                        ],
                                        style={
                                            "display": "flex",
                                            "gap": "12px",
                                            "flexWrap": "wrap",
                                            "margin": "18px 0",
                                        },
                                    ),
                                    html.Div(id="window-summary-message", style={"marginBottom": "12px"}),
                                    html.H3("Method Comparison", style={"color": TEXT}),
                                    html.Div(
                                        id="window-comparison-message",
                                        style={"marginBottom": "12px", "color": MUTED, "fontSize": "13px"},
                                    ),
                                    build_data_table("window-method-compare-table", merge_duplicate_headers=True),
                                    html.H3("Raw Summary Rows", style={"color": TEXT, "marginTop": "28px"}),
                                    build_data_table("window-summary-table"),
                                ],
                            ),
                            dcc.Tab(
                                label="Calibration Summaries",
                                value="calibration-summaries",
                                children=[
                                    html.Div(
                                        [
                                            build_filter_dropdown("calibration-study-filter", "Study Name"),
                                            build_filter_dropdown("calibration-result-filter", "Result Label"),
                                        ],
                                        style={
                                            "display": "flex",
                                            "gap": "12px",
                                            "flexWrap": "wrap",
                                            "margin": "18px 0",
                                        },
                                    ),
                                    html.Div(id="calibration-summary-message", style={"marginBottom": "12px"}),
                                    build_data_table("calibration-summary-table"),
                                ],
                            ),
                            dcc.Tab(
                                label="Compare",
                                value="compare",
                                children=[
                                    html.Div(
                                        "Comparison includes the active run plus any additional selected runs from the sidebar.",
                                        style={"margin": "18px 0", "color": MUTED, "fontSize": "13px"},
                                    ),
                                    html.Div(id="compare-summary-message", style={"marginBottom": "12px"}),
                                    html.Div(
                                        [
                                            build_filter_dropdown("compare-window-dataset-filter", "Dataset"),
                                            build_filter_dropdown("compare-window-method-filter", "Method"),
                                            build_filter_dropdown("compare-window-config-filter", "Config Label"),
                                            build_filter_dropdown("compare-window-interval-filter", "Interval Seconds"),
                                        ],
                                        style={
                                            "display": "flex",
                                            "gap": "12px",
                                            "flexWrap": "wrap",
                                            "margin": "18px 0",
                                        },
                                    ),
                                    html.H3("Window Summary Comparison", style={"color": TEXT}),
                                    build_data_table("compare-window-table"),
                                    html.Div(
                                        [
                                            build_filter_dropdown("compare-calibration-study-filter", "Study Name"),
                                            build_filter_dropdown("compare-calibration-result-filter", "Result Label"),
                                        ],
                                        style={
                                            "display": "flex",
                                            "gap": "12px",
                                            "flexWrap": "wrap",
                                            "margin": "28px 0 18px 0",
                                        },
                                    ),
                                    html.H3("Calibration Summary Comparison", style={"color": TEXT}),
                                    build_data_table("compare-calibration-table"),
                                ],
                            ),
                        ],
                    ),
                ],
                style={"flex": "1 1 auto", "padding": "28px", "overflow": "auto"},
            ),
        ],
        style={
            "display": "flex",
            "minHeight": "100vh",
            "background": f"radial-gradient(circle at top left, #fff7eb 0%, {APP_BACKGROUND} 55%)",
            "fontFamily": "'Avenir Next', 'Segoe UI', sans-serif",
        },
    )
