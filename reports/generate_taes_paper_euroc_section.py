"""Generate compact EuRoC proxy assets for the TAES paper."""

from __future__ import annotations

from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


REPORT_DIR = Path(__file__).resolve().parent
REPO_ROOT = REPORT_DIR.parent
PYTHON_DIR = REPO_ROOT / "python"
PAPER_DIR = Path("/Users/dellaert/papers/25-09-TAES")
ASSET_DIR = REPORT_DIR / "taes_euroc_snippet_experiment_assets"

if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))
if str(REPORT_DIR) not in sys.path:
    sys.path.insert(0, str(REPORT_DIR))

import generate_chebyshev_quadrature_report as chebyshev_report
import generate_psychmagpi_gyro_report as psych_report
from imuFactors.gyro_proxy_monte_carlo import evaluate_proxy_gyro, load_proxy_snippets


def main() -> None:
    PAPER_DIR.mkdir(parents=True, exist_ok=True)
    metadata = pd.read_csv(ASSET_DIR / "proxy_snippet_metadata.csv")
    snippets = load_proxy_snippets(
        chebyshev_report.load_or_build_euroc_cache(),
        data_dir=REPO_ROOT / "data" / "euroc",
        reference_N=psych_report.REFERENCE_N,
        middle_interval=psych_report.MIDDLE_INTERVAL,
    )
    representative_index = choose_representative_snippet(metadata)
    write_representative_snippet(snippets[representative_index])
    write_compact_tables(metadata)


def choose_representative_snippet(metadata: pd.DataFrame) -> int:
    columns = [
        "fit_relative_error",
        "noise_scale_rad_s",
        "aggressive_score",
        "center_activity",
        "center_high_order_ratio",
        "center_normalized_rmse",
    ]
    values = metadata[columns].to_numpy(dtype=float)
    center = np.median(values, axis=0)
    scale = np.median(np.abs(values - center), axis=0)
    scale[scale == 0.0] = 1.0
    distances = np.sqrt(np.sum(((values - center) / scale) ** 2, axis=1))
    return int(np.argmin(distances))


def write_representative_snippet(snippet) -> None:
    interval_start, interval_stop = snippet.middle_interval
    dense_times = np.linspace(interval_start, interval_stop, 401)
    dense_ms = 1000.0 * (dense_times - interval_start)
    dense_values = evaluate_proxy_gyro(snippet, dense_times)

    raw_times = np.asarray(snippet.sample_seconds, dtype=float)
    mask = (raw_times >= interval_start) & (raw_times <= interval_stop)
    raw_ms = 1000.0 * (raw_times[mask] - interval_start)
    raw_values = np.asarray(snippet.raw_bias_corrected_gyro, dtype=float)[mask]

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 7,
            "axes.labelsize": 7,
            "xtick.labelsize": 6,
            "ytick.labelsize": 6,
            "legend.fontsize": 6,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    fig, ax = plt.subplots(figsize=(3.45, 1.45))
    colors = ["#111111", "#7a1f1f", "#1f4e79"]
    linestyles = ["-", "--", ":"]
    labels = [r"$\omega_x$", r"$\omega_y$", r"$\omega_z$"]
    for axis, (color, linestyle, label) in enumerate(zip(colors, linestyles, labels)):
        ax.plot(
            raw_ms,
            raw_values[:, axis],
            ".",
            color=color,
            alpha=0.28,
            markersize=1.8,
            markeredgewidth=0,
        )
        ax.plot(
            dense_ms,
            dense_values[:, axis],
            linestyle=linestyle,
            color=color,
            linewidth=1.15,
            label=label,
        )

    ax.set_xlabel("time (ms)")
    ax.set_ylabel(r"$\omega$ (rad/s)")
    ax.set_xlim(float(dense_ms[0]), float(dense_ms[-1]))
    ax.grid(True, axis="y", linewidth=0.25, color="0.82")
    ax.grid(False, axis="x")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.legend(loc="upper right", frameon=False, ncol=3, handlelength=2.0, columnspacing=0.8)
    fig.tight_layout(pad=0.35)
    fig.savefig(PAPER_DIR / "EurocRepresentativeSnippet.pdf", bbox_inches="tight")
    plt.close(fig)


def write_compact_tables(metadata: pd.DataFrame) -> None:
    endpoint = pd.read_csv(ASSET_DIR / "endpoint_trial_metrics.csv.gz")
    signal = pd.read_csv(ASSET_DIR / "interpolation_signal_metrics.csv.gz")
    trajectory = pd.read_csv(ASSET_DIR / "interpolation_trajectory_metrics.csv.gz")
    subinterval = pd.read_csv(ASSET_DIR / "interpolation_subinterval_metrics.csv.gz")
    runtime = pd.read_csv(ASSET_DIR / "runtime_overall_summary.csv")
    snippet_count = int(len(metadata))
    snippet_names = ", ".join(metadata["dataset"].astype(str).tolist())

    lines = [
        r"\subsection{Drone Experiments}",
        "",
        r"\begin{figure}[ht]",
        r"  \centering",
        r"  \includegraphics[width=0.48\textwidth]{EurocRepresentativeSnippet.pdf}",
        r"  \caption{Representative 200~ms drone-rate proxy; points are bias-corrected samples and curves are the smooth signal used for Monte Carlo sampling.}",
        r"  \label{fig:EurocRepresentativeSnippet}",
        r"\end{figure}",
        "",
        f"A final experimental study was done using realistic angular rates derived from the EuRoC MAV data set~\\cite{{Burri:2016}}. To this end we identified the most aggressive 1-second flight sections for each of the eleven EuRoC sequences ({snippet_names}), and fit each of the three angular rates in them with a high-degree polynomial. We then used the middle 200~ms interval as the basis for an additional Monte Carlo study. In this way the comparison uses realistic motion bandwidth without treating noise-contaminated raw gyro samples as ground truth. Figure~\\ref{{fig:EurocRepresentativeSnippet}} illustrates one representative interval.",
        "",
        r"\begin{table}[ht]",
        r"  \centering",
        r"  \caption{Drone proxy summary. \SecondRevision{All error columns are smaller-is-better; bold marks the lower value in each comparison. Non-PS denotes the best non-PS-CPI method: linear or cubic interpolation for $\bomega$, and linear, cubic, RK2, or Magnus-2 for attitude trajectories and subintervals.}}",
        r"  \label{tab:EurocProxySummary}",
        r"  \scriptsize",
        r"  \setlength{\tabcolsep}{1.0pt}",
        r"  \begin{tabular}{l | c c c | c c | c c | c c}",
        r"    \hline",
        r"    & \multicolumn{3}{c|}{Endpoint} & \multicolumn{2}{c|}{$\bomega$ interp.} & \multicolumn{2}{c|}{Att. traj.} & \multicolumn{2}{c}{Subint.} \\",
        r"    Noise & One-Speed & PS-CPI & Win & \SecondRevision{Non-PS} & PS-CPI & \SecondRevision{Non-PS} & PS-CPI & \SecondRevision{Non-PS} & PS-CPI \\",
        r"    \hline",
    ]
    for band in ["low", "mid", "high"]:
        lines.append("    " + " & ".join(compact_summary_row(endpoint, signal, trajectory, subinterval, band)) + r" \\")
    lines.extend(
        [
            r"    \hline",
            r"  \end{tabular}",
            r"  \vspace{0.2em}",
            r"  \setlength{\tabcolsep}{4pt}",
            r"  \begin{tabular}{l | c c c}",
            r"    \hline",
            r"      Runtime & Seq. Rot. & One-Speed & PS-CPI \\",
            r"    \hline",
            "    " + runtime_row(runtime, "median_runtime_us", r"$\mu$s"),
            "    " + runtime_relative_row(runtime),
            r"    \hline",
            r"  \end{tabular}",
            r"\end{table}",
            "",
            r"The drone proxy results show that PS-CPI is nearly tied with one-speed endpoint integration and most useful as a continuous interpolation model. Table~\ref{tab:EurocProxySummary} reports median endpoint error in degrees, PS-CPI win rate against the one-speed update, normalized angular-rate interpolation error, attitude-trajectory RMS error, random-subinterval RMS error, and median runtime. \SecondRevision{Against the strongest non-PS-CPI angular-rate interpolator, PS-CPI reduces the median interpolation error by 33\% in the middle-noise band and 43\% in the high-noise band. The attitude-trajectory and subinterval comparisons are closer.}",
            "",
            r"\begin{itemize}",
            r"  \item \textbf{Noise} groups the Monte Carlo trials into low, middle, and high injected gyro-noise bands.",
            r"  \item \textbf{Endpoint} reports median final SO(3) attitude error in degrees after the 200~ms interval, where smaller is better. \textbf{One-Speed} is the classical single-speed coning update, \textbf{PS-CPI} is the pseudo-spectral Chebyshev-Picard update, and \textbf{Win} is the fraction of matched trials in which PS-CPI has lower final error than One-Speed.",
            r"  \item \textbf{$\bomega$ interp.} reports unitless normalized angular-rate interpolation error on a dense grid over the interval, where smaller is better. \textbf{Baseline} is the better of componentwise linear and natural cubic-spline interpolation in that noise band, while \textbf{PS-CPI} is the corresponding polynomial-fit result.",
            r"  \item \textbf{Att. traj.} reports RMS SO(3) attitude-trajectory error in degrees over the dense 200~ms trajectory, rather than only the final rotation. \textbf{Baseline} is the best non-PS-CPI result among linear, cubic, RK2, and Magnus-2.",
            r"  \item \textbf{Subint.} reports RMS random-subinterval rotation error in degrees for relative rotations between pairs of times inside the 200~ms interval. This smaller-is-better metric tests whether a method preserves useful local attitude increments, not just the endpoint.",
            r"  \item \textbf{Runtime} gives median Python-level wall-clock time in microseconds per wrapped GTSAM integration call, where smaller is faster. \textbf{Seq. Rot.} is the sequential-rotation/RK2 baseline, \textbf{One-Speed} is the single-speed coning baseline, \textbf{PS-CPI} includes the sample-to-CGL fit used by the report, and \textbf{Rel.} normalizes those timings by the One-Speed runtime.",
            r"\end{itemize}",
            "",
        ]
    )
    (PAPER_DIR / "EurocExperiment.tex").write_text("\n".join(lines), encoding="utf-8")


def compact_summary_row(
    endpoint: pd.DataFrame,
    signal: pd.DataFrame,
    trajectory: pd.DataFrame,
    subinterval: pd.DataFrame,
    band: str,
) -> list[str]:
    endpoint_band = endpoint[endpoint["noise_band"].astype(str) == band]
    signal_band = signal[signal["noise_band"].astype(str) == band]
    trajectory_band = trajectory[trajectory["noise_band"].astype(str) == band]
    subinterval_band = subinterval[subinterval["noise_band"].astype(str) == band]

    d3_endpoint = median_metric(endpoint_band, "D3 single-speed", "error_deg")
    psych_endpoint = median_metric(endpoint_band, "PsyChMagPI", "error_deg")
    psych_win = mean_metric(endpoint_band, "PsyChMagPI", "win_rate")
    signal_best = best_method_metric(
        signal_band,
        "gyro_nrmse",
        smaller_is_better=True,
        excluded_methods=frozenset({"PsyChMagPI"}),
    )
    signal_psych = median_metric(signal_band, "PsyChMagPI", "gyro_nrmse")
    trajectory_best = best_method_metric(
        trajectory_band,
        "trajectory_rms_error_deg",
        smaller_is_better=True,
        excluded_methods=frozenset({"PsyChMagPI"}),
    )
    trajectory_psych = median_metric(
        trajectory_band,
        "PsyChMagPI",
        "trajectory_rms_error_deg",
    )
    subinterval_best = best_method_metric(
        subinterval_band,
        "rms_error_deg",
        smaller_is_better=True,
        weight_column="subinterval_count",
        excluded_methods=frozenset({"PsyChMagPI"}),
    )
    subinterval_psych = weighted_method_metric(
        subinterval_band,
        "PsyChMagPI",
        "rms_error_deg",
        "subinterval_count",
    )
    endpoint_pair = highlight_lower(d3_endpoint, psych_endpoint)
    signal_pair = highlight_lower(signal_best, signal_psych)
    trajectory_pair = highlight_lower(trajectory_best, trajectory_psych)
    subinterval_pair = highlight_lower(subinterval_best, subinterval_psych)
    return [
        band.title(),
        *endpoint_pair,
        f"{100.0 * psych_win:.0f}\\%",
        *signal_pair,
        *trajectory_pair,
        *subinterval_pair,
    ]


def median_metric(frame: pd.DataFrame, method: str, column: str) -> float:
    values = frame[frame["method"].astype(str) == method][column].to_numpy(dtype=float)
    return float(np.median(values))


def mean_metric(frame: pd.DataFrame, method: str, column: str) -> float:
    values = frame[frame["method"].astype(str) == method][column].to_numpy(dtype=float)
    return float(np.mean(values))


def best_method_metric(
    frame: pd.DataFrame,
    column: str,
    *,
    smaller_is_better: bool,
    weight_column: str | None = None,
    excluded_methods: frozenset[str] = frozenset(),
) -> float:
    rows = []
    for method, group in frame.groupby("method", observed=True):
        if str(method) in excluded_methods:
            continue
        if weight_column is None:
            value = float(np.median(group[column].to_numpy(dtype=float)))
        else:
            value = weighted_median_like(group[column], group[weight_column])
        rows.append((str(method), value))
    rows.sort(key=lambda item: item[1], reverse=not smaller_is_better)
    return rows[0][1]


def highlight_lower(first: float, second: float) -> list[str]:
    """Format a pair and bold its lower-error entry."""
    formatted = [fmt(first), fmt(second)]
    lower_index = int(second < first)
    formatted[lower_index] = rf"\textbf{{{formatted[lower_index]}}}"
    return [rf"\SecondRevision{{{value}}}" for value in formatted]


def weighted_method_metric(
    frame: pd.DataFrame,
    method: str,
    column: str,
    weight_column: str,
) -> float:
    group = frame[frame["method"].astype(str) == method]
    return weighted_median_like(group[column], group[weight_column])


def weighted_median_like(values: pd.Series, weights: pd.Series) -> float:
    values_array = values.to_numpy(dtype=float)
    weights_array = weights.to_numpy(dtype=float)
    return float(np.average(values_array, weights=weights_array))


def runtime_row(runtime: pd.DataFrame, column: str, units: str) -> str:
    values = runtime_values(runtime, column)
    return (
        rf"{units} & {fmt(values['D2/RK2'])} & {fmt(values['D3 single-speed'])} "
        rf"& {fmt(values['PsyChMagPI'])} \\"
    )


def runtime_relative_row(runtime: pd.DataFrame) -> str:
    values = runtime_values(runtime, "median_runtime_us")
    d3 = values["D3 single-speed"]
    return (
        r"Rel. & "
        + f"{values['D2/RK2'] / d3:.1f}"
        + " & 1.0 & "
        + f"{values['PsyChMagPI'] / d3:.1f}"
        + r" \\"
    )


def runtime_values(runtime: pd.DataFrame, column: str) -> dict[str, float]:
    return {
        str(row["method"]): float(row[column])
        for _, row in runtime.iterrows()
    }


def fmt(value: float) -> str:
    value = float(value)
    if abs(value) >= 100.0:
        return f"{value:.0f}"
    if abs(value) >= 10.0:
        return f"{value:.1f}"
    if abs(value) >= 1.0:
        return f"{value:.2f}"
    if abs(value) >= 0.1:
        return f"{value:.3f}"
    return f"{value:.4f}"


if __name__ == "__main__":
    main()
