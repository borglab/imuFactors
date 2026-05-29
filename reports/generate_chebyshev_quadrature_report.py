"""Generate the Quarto report and static assets for the quadrature study."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import sys
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots


REPORT_DIR = Path(__file__).resolve().parent
REPO_ROOT = REPORT_DIR.parent
PYTHON_DIR = REPO_ROOT / "python"
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

import imuFactors.euroc as euroc
import imuFactors.scalar_quadrature as scalar_quadrature
import imuFactors.spectral as spectral
import imuFactors.spectrogram as spectrogram


ASSET_DIR = REPORT_DIR / "chebyshev_quadrature_report_assets"
QMD_PATH = REPORT_DIR / "chebyshev_quadrature_report.qmd"

INTERVAL = (0.0, 1.0)
M_VALUES = [10, 20, 30, 40, 50]
N_VALUES = np.arange(2, 11)
NOISE_FRACTIONS = np.array(
    [0.0, 0.025, 0.05, 0.06, 0.075, 0.10, 0.12, 0.15, 0.17, 0.20, 0.225]
)
NUM_SEEDS = 100
RANDOM_SEED = 20260523
EVALUATION_COUNT = 151
REGULARIZED_LAMBDA1 = 0.005

OLD_M_VALUES = np.arange(5, 49)
OLD_N_VALUES = np.arange(2, 25)
OLD_FIXED_N_VALUES = [2, 9, 17, 24]
OLD_SELECTED_NOISE_FRACTIONS = [0.06, 0.12, 0.17, 0.225]

POLYNOMIAL_N = 5
POLYNOMIAL_SEED = 8675309

DATA_DIR = REPO_ROOT / "data" / "euroc"
EUROC_CACHE_PATH = DATA_DIR / "euroc_aggressive_gyro_norm_cheb2_n50_windows.npz"
SIGNAL_COLUMN = "gyro_norm"
REFERENCE_BASIS = "chebyshev2"
REFERENCE_N = 50
REFERENCE_WINDOW_SECONDS = 1.0
REFERENCE_LAMBDA1 = 0.0
FULL_INTERVAL = (0.0, 1.0)
MIDDLE_INTERVAL = (0.4, 0.6)
SELECTION_INTERVAL = MIDDLE_INTERVAL
SELECTION_N = 10
CACHE_VERSION = 2


@dataclass(frozen=True)
class FigurePair:
    label: str
    advantage_path: Path
    table_path: Path


@dataclass(frozen=True)
class FamilyArtifacts:
    family: str
    lambda1: float
    pairs: list[FigurePair]
    comparisons: pd.DataFrame
    robust_summary: pd.DataFrame


def main() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)

    analytic_functions = build_analytic_functions()
    random_functions, random_nodes = build_random_polynomial_functions()
    euroc_cache = load_or_build_euroc_cache()
    euroc_functions = build_euroc_functions(euroc_cache)

    old_image_paths = generate_old_scalar_images(analytic_functions)
    euroc_preview_path = write_plotly_image(
        plot_cached_window_previews(euroc_cache),
        ASSET_DIR / "euroc_aggressive_snippet_previews.png",
        width=1400,
    )

    families: list[FamilyArtifacts] = []
    families.extend(
        generate_decision_assets(
            family="analytic scalar functions",
            prefix="analytic",
            functions=analytic_functions,
            interval=INTERVAL,
            y_range_min_N=3,
        )
    )
    families.extend(
        generate_decision_assets(
            family="random degree-4 polynomials",
            prefix="random_degree4",
            functions=random_functions,
            interval=INTERVAL,
            y_range_min_N=POLYNOMIAL_N,
        )
    )
    families.extend(
        generate_decision_assets(
            family="EuRoC aggressive 200 ms snippets",
            prefix="euroc_aggressive",
            functions=euroc_functions,
            interval=MIDDLE_INTERVAL,
            y_range_min_N=5,
        )
    )

    summary_table = summarize_families(families)
    summary_table.to_csv(ASSET_DIR / "overall_summary.csv", index=False)
    random_node_table = random_node_dataframe(random_nodes)
    random_node_table.to_csv(ASSET_DIR / "random_degree4_node_values.csv")
    euroc_metadata = euroc_metadata_dataframe(euroc_cache)
    euroc_metadata.to_csv(ASSET_DIR / "euroc_aggressive_snippet_metadata.csv", index=False)

    write_report_qmd(
        old_image_paths=old_image_paths,
        family_artifacts=families,
        summary_table=summary_table,
        random_node_table=random_node_table,
        euroc_metadata=euroc_metadata,
        euroc_preview_path=euroc_preview_path,
    )


def build_analytic_functions() -> list[scalar_quadrature.ScalarFunction]:
    return [
        scalar_quadrature.ScalarFunction(
            name="tanh(t)",
            value=lambda t: np.tanh(t),
            antiderivative=lambda t: np.log(np.cosh(t)),
        ),
        scalar_quadrature.ScalarFunction(
            name="sin(2*pi*t)",
            value=lambda t: np.sin(2.0 * np.pi * t),
            antiderivative=lambda t: (1.0 - np.cos(2.0 * np.pi * t)) / (2.0 * np.pi),
        ),
        scalar_quadrature.ScalarFunction(
            name="pi*t^(31/4)",
            value=lambda t: np.pi * t ** (31.0 / 4.0),
            antiderivative=lambda t: np.pi * t ** (35.0 / 4.0) / (35.0 / 4.0),
        ),
    ]


def build_random_polynomial_functions() -> tuple[list[scalar_quadrature.ScalarFunction], np.ndarray]:
    rng = np.random.default_rng(POLYNOMIAL_SEED)
    node_values = rng.uniform(-0.5, 0.5, size=(3, POLYNOMIAL_N))
    functions = [
        scalar_quadrature.scalar_function_from_chebyshev2_nodes(
            f"random degree-4 polynomial {index + 1}", values, INTERVAL
        )
        for index, values in enumerate(node_values)
    ]
    return functions, node_values


def generate_old_scalar_images(
    functions: Sequence[scalar_quadrature.ScalarFunction],
) -> dict[str, Path]:
    result = scalar_quadrature.run_scalar_monte_carlo(
        functions,
        m_values=OLD_M_VALUES,
        N_values=OLD_N_VALUES,
        noise_fractions=NOISE_FRACTIONS,
        num_seeds=NUM_SEEDS,
        seed=RANDOM_SEED,
        interval=INTERVAL,
        evaluation_count=EVALUATION_COUNT,
        lambda1=0.0,
    )
    comparisons = result.comparisons
    fixed_n = scalar_quadrature.plot_fixed_N_comparison(
        comparisons,
        function_name="tanh(t)",
        selected_N_values=OLD_FIXED_N_VALUES,
    )
    fixed_n_path = write_matplotlib_image(
        fixed_n, ASSET_DIR / "old_scalar_fixed_N_tanh.png"
    )
    plt.close(fixed_n)

    function_noise = scalar_quadrature.plot_function_noise_comparison(
        comparisons,
        function_names=[function.name for function in functions],
        selected_noise_fractions=OLD_SELECTED_NOISE_FRACTIONS,
        metric="end_error",
    )
    function_noise_path = write_matplotlib_image(
        function_noise, ASSET_DIR / "old_scalar_function_noise_end_error.png"
    )
    plt.close(function_noise)

    return {
        "fixed_n": fixed_n_path,
        "function_noise": function_noise_path,
    }


def generate_decision_assets(
    *,
    family: str,
    prefix: str,
    functions: Sequence[scalar_quadrature.ScalarFunction],
    interval: tuple[float, float],
    y_range_min_N: int,
) -> list[FamilyArtifacts]:
    artifacts: list[FamilyArtifacts] = []
    for lambda1 in (0.0, REGULARIZED_LAMBDA1):
        comparisons = run_fixed_m_experiment(
            functions,
            interval=interval,
            lambda1=lambda1,
        )
        pairs: list[FigurePair] = []
        robust_rows: list[pd.DataFrame] = []
        lambda_slug = lambda_slug_for(lambda1)
        for function in functions:
            function_slug = slugify(function.name)
            advantage = scalar_quadrature.plot_advantage_curves_by_m(
                comparisons,
                function.name,
                selected_m_values=M_VALUES,
                metric="rmse_error",
                y_range_min_N=y_range_min_N,
            )
            advantage_path = write_plotly_image(
                advantage,
                ASSET_DIR / f"{prefix}_{lambda_slug}_{function_slug}_advantage.png",
            )
            robust_table = scalar_quadrature.plot_robust_N_table(
                comparisons,
                function.name,
                selected_m_values=M_VALUES,
            )
            table_path = write_plotly_image(
                robust_table,
                ASSET_DIR / f"{prefix}_{lambda_slug}_{function_slug}_robust_N.png",
                width=1500,
            )
            pairs.append(
                FigurePair(
                    label=function.name,
                    advantage_path=advantage_path,
                    table_path=table_path,
                )
            )
            robust_summary = scalar_quadrature.robust_N_summary(
                comparisons,
                function.name,
                selected_m_values=M_VALUES,
            )
            robust_summary["lambda1"] = lambda1
            robust_rows.append(robust_summary)

        artifacts.append(
            FamilyArtifacts(
                family=family,
                lambda1=lambda1,
                pairs=pairs,
                comparisons=comparisons,
                robust_summary=pd.concat(robust_rows, ignore_index=True),
            )
        )
    return artifacts


def run_fixed_m_experiment(
    functions: Sequence[scalar_quadrature.ScalarFunction],
    *,
    interval: tuple[float, float],
    lambda1: float,
) -> pd.DataFrame:
    runs = []
    for m in M_VALUES:
        runs.append(
            scalar_quadrature.run_scalar_monte_carlo(
                functions,
                m_values=[m],
                N_values=N_VALUES,
                noise_fractions=NOISE_FRACTIONS,
                num_seeds=NUM_SEEDS,
                seed=RANDOM_SEED,
                interval=interval,
                evaluation_count=EVALUATION_COUNT,
                lambda1=lambda1,
            )
        )
    return pd.concat([run.comparisons for run in runs], ignore_index=True)


def load_or_build_euroc_cache() -> dict[str, np.ndarray]:
    data_files = euroc.discover_euroc_files(DATA_DIR)
    if not data_files:
        raise FileNotFoundError(f"No EuRoC CSV files found in {DATA_DIR}")
    if EUROC_CACHE_PATH.exists():
        cache = load_aggressive_window_cache(EUROC_CACHE_PATH)
        if cache_is_current(cache):
            return cache
    return build_aggressive_window_cache(data_files, EUROC_CACHE_PATH)


def cache_is_current(cache: dict[str, np.ndarray]) -> bool:
    required = {
        "cache_version",
        "selection_interval",
        "selection_N",
        "selection_activity",
        "selection_high_order_ratio",
        "selection_normalized_rmse",
        "snippet_start_seconds",
        "snippet_end_seconds",
        "window_activity",
        "window_high_order_ratio",
        "window_normalized_rmse",
    }
    if not required.issubset(cache):
        return False
    return (
        int(cache["cache_version"]) == CACHE_VERSION
        and int(cache["selection_N"]) == SELECTION_N
        and np.allclose(cache["selection_interval"], SELECTION_INTERVAL)
        and int(cache["reference_N"]) == REFERENCE_N
        and np.isclose(float(cache["reference_window_seconds"]), REFERENCE_WINDOW_SECONDS)
    )


def dataset_name(path: Path) -> str:
    return path.stem.removeprefix("euroc_")


def selected_window_row(path: Path, result: spectrogram.SpectrogramFit) -> dict[str, object]:
    diagnostics = spectrogram.interval_window_diagnostics(
        result,
        SELECTION_INTERVAL,
        N=SELECTION_N,
    )
    scores = np.asarray(diagnostics["aggressive_score"], dtype=float)
    window_index = int(np.nanargmax(scores))
    window_start_s = spectrogram.window_start_seconds(result, window_index)
    full_nrms = spectrogram.normalized_rmse(result)
    return {
        "dataset": dataset_name(path),
        "csv_path": str(path),
        "window_index": window_index,
        "start_seconds": window_start_s,
        "snippet_start_seconds": window_start_s + SELECTION_INTERVAL[0],
        "snippet_end_seconds": window_start_s + SELECTION_INTERVAL[1],
        "aggressive_score": float(scores[window_index]),
        "selection_activity": float(diagnostics["activity"][window_index]),
        "selection_high_order_ratio": float(diagnostics["high_order_ratio"][window_index]),
        "selection_normalized_rmse": float(diagnostics["normalized_rmse"][window_index]),
        "window_activity": float(result.activity[window_index]),
        "window_high_order_ratio": float(result.high_order_ratio[window_index]),
        "window_normalized_rmse": float(full_nrms[window_index]),
        "raw_samples": result.samples[window_index, :, 0].astype(float),
        "cgl_node_values": result.coeffs[window_index, :, 0].astype(float),
        "sample_seconds": np.linspace(0.0, result.window_seconds, result.m),
    }


def build_aggressive_window_cache(data_files: list[Path], cache_path: Path) -> dict[str, np.ndarray]:
    rows = []
    for path in data_files:
        print(f"fitting {dataset_name(path)}...")
        result = spectrogram.fit_spectral_windows(
            path,
            N=REFERENCE_N,
            basis=REFERENCE_BASIS,
            columns=[SIGNAL_COLUMN],
            window_seconds=REFERENCE_WINDOW_SECONDS,
            lambda1=REFERENCE_LAMBDA1,
        )
        rows.append(selected_window_row(path, result))

    sample_seconds = rows[0]["sample_seconds"]
    if any(row["sample_seconds"].shape != sample_seconds.shape for row in rows):
        raise ValueError("Selected windows do not all have the same m")

    payload = {
        "cache_version": np.array(CACHE_VERSION, dtype=int),
        "datasets": np.array([row["dataset"] for row in rows]),
        "csv_paths": np.array([row["csv_path"] for row in rows]),
        "window_indices": np.array([row["window_index"] for row in rows], dtype=int),
        "start_seconds": np.array([row["start_seconds"] for row in rows], dtype=float),
        "snippet_start_seconds": np.array([row["snippet_start_seconds"] for row in rows], dtype=float),
        "snippet_end_seconds": np.array([row["snippet_end_seconds"] for row in rows], dtype=float),
        "aggressive_scores": np.array([row["aggressive_score"] for row in rows], dtype=float),
        "selection_activity": np.array([row["selection_activity"] for row in rows], dtype=float),
        "selection_high_order_ratio": np.array([row["selection_high_order_ratio"] for row in rows], dtype=float),
        "selection_normalized_rmse": np.array([row["selection_normalized_rmse"] for row in rows], dtype=float),
        "window_activity": np.array([row["window_activity"] for row in rows], dtype=float),
        "window_high_order_ratio": np.array([row["window_high_order_ratio"] for row in rows], dtype=float),
        "window_normalized_rmse": np.array([row["window_normalized_rmse"] for row in rows], dtype=float),
        "raw_samples": np.stack([row["raw_samples"] for row in rows]),
        "cgl_node_values": np.stack([row["cgl_node_values"] for row in rows]),
        "sample_seconds": sample_seconds,
        "cgl_node_seconds": spectral.chebyshev2_points(REFERENCE_N, FULL_INTERVAL),
        "reference_N": np.array(REFERENCE_N, dtype=int),
        "reference_window_seconds": np.array(REFERENCE_WINDOW_SECONDS, dtype=float),
        "middle_interval": np.array(MIDDLE_INTERVAL, dtype=float),
        "selection_interval": np.array(SELECTION_INTERVAL, dtype=float),
        "selection_N": np.array(SELECTION_N, dtype=int),
    }
    payload["activity"] = payload["selection_activity"]
    payload["high_order_ratio"] = payload["selection_high_order_ratio"]
    payload["normalized_rmse"] = payload["selection_normalized_rmse"]
    np.savez_compressed(cache_path, **payload)
    return payload


def load_aggressive_window_cache(cache_path: Path) -> dict[str, np.ndarray]:
    with np.load(cache_path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def build_euroc_functions(
    cache: dict[str, np.ndarray],
) -> list[scalar_quadrature.ScalarFunction]:
    return [
        scalar_quadrature.scalar_function_from_chebyshev2_nodes(
            f"{dataset} center-200ms aggressive gyro_norm", cgl_node_values, FULL_INTERVAL
        )
        for dataset, cgl_node_values in zip(cache["datasets"], cache["cgl_node_values"])
    ]


def plot_cached_window_previews(cache: dict[str, np.ndarray]) -> go.Figure:
    datasets = list(cache["datasets"])
    rows = len(datasets)
    fig = make_subplots(
        rows=rows,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.012,
        subplot_titles=datasets,
    )
    dense_seconds = np.linspace(0.0, 1.0, 401)
    for row, (dataset, cgl_node_values, raw_samples) in enumerate(
        zip(cache["datasets"], cache["cgl_node_values"], cache["raw_samples"]), start=1
    ):
        function = scalar_quadrature.scalar_function_from_chebyshev2_nodes(
            str(dataset), cgl_node_values, FULL_INTERVAL
        )
        fig.add_trace(
            go.Scatter(
                x=cache["sample_seconds"],
                y=raw_samples,
                mode="markers",
                marker=dict(size=3),
                name=f"{dataset} samples",
                showlegend=False,
            ),
            row=row,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=dense_seconds,
                y=function.value(dense_seconds),
                mode="lines",
                line=dict(width=2),
                name=f"{dataset} Chebyshev2 fit",
                showlegend=False,
            ),
            row=row,
            col=1,
        )
        fig.add_vrect(
            x0=MIDDLE_INTERVAL[0],
            x1=MIDDLE_INTERVAL[1],
            fillcolor="rgba(80, 140, 255, 0.16)",
            line_width=0,
            row=row,
            col=1,
        )
        fig.update_yaxes(title_text="gyro", row=row, col=1)
    fig.update_xaxes(
        title_text="seconds inside selected 1 s reference window", row=rows, col=1
    )
    fig.update_layout(
        title="Selected 1-second reference windows ranked by their center 200 ms snippets",
        template="plotly_white",
        height=max(700, 145 * rows),
        margin=dict(l=70, r=30, t=80, b=55),
    )
    return fig


def summarize_families(families: Sequence[FamilyArtifacts]) -> pd.DataFrame:
    rows = []
    for artifacts in families:
        overall = scalar_quadrature.win_rate_summary(
            artifacts.comparisons,
            group_by=("lambda1",),
            metric="rmse_error",
        )
        if overall.empty:
            continue
        record = overall.iloc[0].to_dict()
        robust = artifacts.robust_summary
        rows.append(
            {
                "family": artifacts.family,
                "lambda1": artifacts.lambda1,
                "overall_rmse_win_rate": float(record["win_rate"]),
                "overall_median_rmse_advantage": float(record["median_advantage"]),
                "overall_mean_rmse_advantage": float(record["mean_advantage"]),
                "overall_rows": int(record["rows"]),
                "robust_N_mean_win_rate": float(robust["win_rate"].mean()),
                "robust_N_median_win_rate": float(robust["win_rate"].median()),
                "robust_N_positive_advantage_rows": float(
                    np.mean(robust["rmse_advantage_at_robust_N"] > 0.0)
                ),
                "robust_rows": int(len(robust)),
            }
        )
    return pd.DataFrame(rows)


def random_node_dataframe(node_values: np.ndarray) -> pd.DataFrame:
    node_times = spectral.chebyshev2_points(POLYNOMIAL_N, INTERVAL)
    return pd.DataFrame(
        node_values,
        columns=[f"f({time:.3f})" for time in node_times],
        index=[f"random degree-4 polynomial {index + 1}" for index in range(3)],
    )


def euroc_metadata_dataframe(cache: dict[str, np.ndarray]) -> pd.DataFrame:
    return (
        pd.DataFrame(
            {
                "dataset": cache["datasets"],
                "window_index": cache["window_indices"],
                "window_start_s": cache["start_seconds"],
                "snippet_start_s": cache["snippet_start_seconds"],
                "snippet_end_s": cache["snippet_end_seconds"],
                "center_200ms_score": cache["aggressive_scores"],
                "center_activity": cache["selection_activity"],
                "center_high_order_ratio": cache["selection_high_order_ratio"],
                "center_normalized_rmse": cache["selection_normalized_rmse"],
            }
        )
        .sort_values("center_200ms_score", ascending=False)
        .reset_index(drop=True)
    )


def write_report_qmd(
    *,
    old_image_paths: dict[str, Path],
    family_artifacts: Sequence[FamilyArtifacts],
    summary_table: pd.DataFrame,
    random_node_table: pd.DataFrame,
    euroc_metadata: pd.DataFrame,
    euroc_preview_path: Path,
) -> None:
    sections = []
    sections.append(report_header())
    sections.append(executive_summary(summary_table))
    sections.append(old_reproduction_section(old_image_paths))
    sections.append(
        decision_family_section(
            "Decision-oriented analytic scalar views",
            "The same three analytic functions are now shown through the decision-oriented Plotly exports. The unregularized block is the baseline; the regularized block uses lambda1 = 0.005. Positive RMSE advantage means trapezoid error minus Chebyshev2 error is positive, so Chebyshev2 is better.",
            [artifact for artifact in family_artifacts if artifact.family == "analytic scalar functions"],
        )
    )
    sections.append(random_polynomial_section(random_node_table, family_artifacts))
    sections.append(euroc_section(euroc_metadata, euroc_preview_path, family_artifacts))
    QMD_PATH.write_text("\n\n".join(sections) + "\n", encoding="utf-8")
    print(f"wrote {QMD_PATH}")


def report_header() -> str:
    return """---
title: "Chebyshev Quadrature Versus Trapezoid Integration"
subtitle: "Scalar, random-polynomial, and EuRoC aggressive-snippet evidence"
author: "Frank Dellaert"
date: "2026-05-29"
format:
  pdf:
    toc: true
    number-sections: true
    colorlinks: true
    fig-pos: H
    geometry:
      - top=0.65in
      - bottom=0.65in
      - left=0.65in
      - right=0.65in
fontsize: 10pt
---
"""


def executive_summary(summary_table: pd.DataFrame) -> str:
    display = summary_table.copy()
    display["lambda1"] = display["lambda1"].map(lambda value: f"{value:g}")
    for column in [
        "overall_rmse_win_rate",
        "robust_N_mean_win_rate",
        "robust_N_median_win_rate",
        "robust_N_positive_advantage_rows",
    ]:
        display[column] = display[column].map(lambda value: f"{100.0 * value:.0f}%")
    for column in ["overall_median_rmse_advantage", "overall_mean_rmse_advantage"]:
        display[column] = display[column].map(lambda value: f"{value:.4g}")
    display = display[
        [
            "family",
            "lambda1",
            "overall_rmse_win_rate",
            "overall_median_rmse_advantage",
            "robust_N_mean_win_rate",
            "robust_N_positive_advantage_rows",
        ]
    ]
    table = markdown_table(
        display.rename(
            columns={
                "family": "Family",
                "lambda1": "lambda1",
                "overall_rmse_win_rate": "Overall RMSE win rate",
                "overall_median_rmse_advantage": "Median RMSE advantage",
                "robust_N_mean_win_rate": "Mean robust-N win rate",
                "robust_N_positive_advantage_rows": "Robust rows with positive advantage",
            }
        )
    )
    return f"""# Executive Summary

The evidence supports a conditional, not universal, case for Chebyshev2 quadrature. The corrected Chebyshev2 integration path now evaluates the antiderivative in an `N+1` representation, and the regularized runs use the derivative-matrix Tikhonov term with `lambda1 = 0.005`. Those fixes make the comparison fairer, but they do not make Chebyshev2 dominate trapezoid integration in every regime.

The consistent pattern is:

- Chebyshev2 is strongest when the signal is smooth and the chosen `N` is modest, usually near or below `sqrt(m)`.
- Trapezoid remains a strong baseline because the samples are uniformly spaced and the data are noisy. It is local and less exposed to global fit artifacts.
- Tikhonov regularization helps most when the Chebyshev2 fit would otherwise use too much high-order freedom. It can improve robustness, but it is still a modeling choice rather than a guaranteed win.
- The EuRoC snippets are the most relevant evidence. They show that Chebyshev2 can win on aggressive 200 ms windows, but the advantage depends on `N`, noise level, and dataset.

Summary over the decision-view comparisons:

{table}

How to read the figures: positive advantage means `trapezoid error - Chebyshev2 error > 0`, so Chebyshev2 has lower error. The black diamond marks the best median RMSE `N`, and the dashed vertical line marks `sqrt(m)`.
"""


def old_reproduction_section(old_image_paths: dict[str, Path]) -> str:
    return f"""# Reproducing the May 24 Scalar Comparison

The May 24 scalar notebook can be reproduced with the current corrected integration code. The two figures below are deliberately representative old-style image plots: they establish continuity with Benjamin's original scalar comparison, but they are not the best way to choose `N` because the sign, noise dependence, and fixed-`N` slices are hard to compare directly.

![Representative old fixed-N scalar heatmaps for `tanh(t)`.]({relative_asset(old_image_paths["fixed_n"])}){{width=100%}}

![Representative old scalar heatmaps across all three functions at selected noise levels.]({relative_asset(old_image_paths["function_noise"])}){{width=100%}}
"""


def decision_family_section(
    title: str,
    intro: str,
    artifacts: Sequence[FamilyArtifacts],
) -> str:
    blocks = [f"# {title}", intro]
    for artifact in sorted(artifacts, key=lambda item: item.lambda1):
        blocks.append(f"## {lambda_label(artifact.lambda1)}")
        for pair in artifact.pairs:
            blocks.append(
                f"### `{pair.label}`\n\n"
                f"![RMSE advantage curves for `{pair.label}` with {lambda_label(artifact.lambda1)}.]({relative_asset(pair.advantage_path)}){{width=100%}}\n\n"
                f"![Robust `N` table for `{pair.label}` with {lambda_label(artifact.lambda1)}.]({relative_asset(pair.table_path)}){{width=100%}}"
            )
    return "\n\n".join(blocks)


def random_polynomial_section(
    random_node_table: pd.DataFrame,
    family_artifacts: Sequence[FamilyArtifacts],
) -> str:
    table = markdown_table(
        random_node_table.reset_index(names="Function").round(4)
    )
    intro = f"""The random-polynomial notebook broadens the scalar comparison beyond the three hand-picked analytic functions. Each degree-4 polynomial is defined by fixed random values at five CGL nodes on `[0, 1]`; the same Monte Carlo grid is then used for no Tikhonov and for `lambda1 = 0.005`.

{table}
"""
    return decision_family_section(
        "Random Degree-4 Polynomials",
        intro,
        [
            artifact
            for artifact in family_artifacts
            if artifact.family == "random degree-4 polynomials"
        ],
    )


def euroc_section(
    euroc_metadata: pd.DataFrame,
    euroc_preview_path: Path,
    family_artifacts: Sequence[FamilyArtifacts],
) -> str:
    compact = pd.DataFrame(
        {
            "dataset": euroc_metadata["dataset"],
            "window": euroc_metadata["window_index"],
            "snippet_s": [
                f"{start:.1f}-{end:.1f}"
                for start, end in zip(
                    euroc_metadata["snippet_start_s"],
                    euroc_metadata["snippet_end_s"],
                )
            ],
            "score": euroc_metadata["center_200ms_score"].map(
                lambda value: f"{float(value):.4g}"
            ),
            "activity": euroc_metadata["center_activity"].map(
                lambda value: f"{float(value):.4g}"
            ),
            "high_ratio": euroc_metadata["center_high_order_ratio"].map(
                lambda value: f"{float(value):.4g}"
            ),
            "nRMSE": euroc_metadata["center_normalized_rmse"].map(
                lambda value: f"{float(value):.4g}"
            ),
        }
    )
    table = markdown_table(compact)
    decision = decision_family_section(
        "EuRoC Decision-oriented Views",
        "The following pages show the same decision views for every selected EuRoC snippet, first without Tikhonov and then with `lambda1 = 0.005`.",
        [
            artifact
            for artifact in family_artifacts
            if artifact.family == "EuRoC aggressive 200 ms snippets"
        ],
    )
    return f"""# EuRoC Aggressive 200 ms Snippets

The EuRoC extension changes the question from synthetic scalar functions to IMU-like signals. For each merged EuRoC CSV, the workflow fits one-second `gyro_norm` windows with Chebyshev2 using `N = 50`. It then scores only the center interval `[0.4, 0.6]` with the local aggressive-window diagnostic, so the selected window is aggressive in the 200 ms interval actually used by the quadrature comparison. The resulting CGL node values define the reference scalar function; the Monte Carlo comparison samples only that middle 200 ms interval.

The figure below shows all selected snippets. Points are the raw uniformly sampled IMU values in the selected one-second window, the line is the `N = 50` Chebyshev2 reference fit, and the shaded region is the 200 ms interval used for quadrature.

![All selected EuRoC aggressive snippets.]({relative_asset(euroc_preview_path)}){{width=100%}}

Selection metadata:

{table}

{decision}
"""


def markdown_table(frame: pd.DataFrame) -> str:
    headers = [str(column) for column in frame.columns]
    rows = [[format_cell(value) for value in row] for row in frame.to_numpy()]
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]
    header = "| " + " | ".join(
        headers[index].ljust(widths[index]) for index in range(len(headers))
    ) + " |"
    separator = "| " + " | ".join("-" * widths[index] for index in range(len(headers))) + " |"
    body = [
        "| " + " | ".join(
            row[index].ljust(widths[index]) for index in range(len(headers))
        ) + " |"
        for row in rows
    ]
    return "\n".join([header, separator, *body])


def format_cell(value: object) -> str:
    if pd.isna(value):
        return ""
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


def lambda_label(lambda1: float) -> str:
    if np.isclose(lambda1, 0.0):
        return "No Tikhonov (`lambda1 = 0`)"
    return f"Tikhonov (`lambda1 = {lambda1:g}`)"


def lambda_slug_for(lambda1: float) -> str:
    if np.isclose(lambda1, 0.0):
        return "lambda1_0"
    return f"lambda1_{lambda1:g}".replace(".", "p")


def slugify(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")


def write_matplotlib_image(fig: plt.Figure, path: Path) -> Path:
    fig.savefig(path, dpi=180, bbox_inches="tight")
    return path


def write_plotly_image(
    fig: go.Figure,
    path: Path,
    *,
    width: int | None = None,
    height: int | None = None,
) -> Path:
    layout_width = int(fig.layout.width) if fig.layout.width else 1200
    layout_height = int(fig.layout.height) if fig.layout.height else 420
    fig.write_image(
        path,
        width=width or layout_width,
        height=height or layout_height,
        scale=1.6,
    )
    return path


def relative_asset(path: Path) -> str:
    return path.relative_to(REPORT_DIR).as_posix()


if __name__ == "__main__":
    main()
