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
PAPER_NOISE_GROUPS = ("none", "low", "mid", "high")

if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))
if str(REPORT_DIR) not in sys.path:
    sys.path.insert(0, str(REPORT_DIR))

from taes_euroc_endpoint_table import (
    EUROC_RMS_CACHE_NAME,
    endpoint_rms_by_exact_noise,
    euroc_rms_method_values,
)
from taes_euroc_significance import median_table_significance, select_noise_group


def main() -> None:
    import generate_chebyshev_quadrature_report as chebyshev_report
    import generate_psychmagpi_gyro_report as psych_report
    from imuFactors.gyro_proxy_monte_carlo import load_proxy_snippets

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
    from imuFactors.gyro_proxy_monte_carlo import evaluate_proxy_gyro

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
    endpoint_rms = endpoint_rms_by_exact_noise(
        pd.read_csv(ASSET_DIR / EUROC_RMS_CACHE_NAME)
    )
    signal = pd.read_csv(ASSET_DIR / "interpolation_signal_metrics.csv.gz")
    significance = median_table_significance(endpoint, signal)
    significance.to_csv(ASSET_DIR / "median_table_significance.csv", index=False)
    runtime = pd.read_csv(ASSET_DIR / "runtime_overall_summary.csv")
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
        *euroc_rms_table_lines(endpoint_rms),
        r"\begin{table}[ht]",
        r"  \centering",
        r"  \caption{Drone proxy summary. \SecondRevision{All error columns are smaller-is-better. Bold indicates a statistically significant improvement after correcting for multiple comparisons (Holm method, 5\% family-wise significance level). Non-PS denotes the better of linear and cubic interpolation for $\bomega$. The ``None'' row has no added noise; ``Low'' contains only positive injected noise. For $M$ samples, PS-CPI uses $n=\lfloor2\sqrt{M-1}\rfloor$ and $N=n+1$ CGL nodes.}}",
        r"  \label{tab:EurocProxySummary}",
        r"  \scriptsize",
        r"  \setlength{\tabcolsep}{4pt}",
        r"  \begin{tabular}{l | c c c | c c}",
        r"    \hline",
        r"    & \multicolumn{3}{c|}{Endpoint} & \multicolumn{2}{c}{$\bomega$ interp.} \\",
        r"    Noise & One-Speed & PS-CPI & Win & \SecondRevision{Non-PS} & PS-CPI \\",
        r"    \hline",
    ]
    for noise_group in PAPER_NOISE_GROUPS:
        lines.append(
            "    "
            + " & ".join(
                compact_summary_row(
                    endpoint,
                    signal,
                    noise_group,
                    significance,
                )
            )
            + r" \\"
        )
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
            r"\SecondRevision{For snippet $i$, the value in the first column is the ratio $\sigma_i/\operatorname{range}_t\|\bomega_i^\star(t)\|$, where $\sigma_i$ is the injected component-noise standard deviation. Because the angular-rate range varies by snippet, Table~\ref{tab:EurocProxyRms} reports this dimensionless fraction rather than a single noise standard deviation in rad/s. For PS-CPI, the rule $n=\lfloor2\sqrt{M-1}\rfloor$ gives $N=n+1=13$ CGL nodes. Em dashes mark methods that were not evaluated in this experiment.}",
            "",
            r"\SecondRevision{Boldface in Table~\ref{tab:EurocProxySummary} uses exact two-sided paired permutation tests of the difference in pooled medians, swapping method labels jointly for all trials within each of the 11 snippets. Holm correction covers the 12 comparisons: One-Speed versus PS-CPI and each of the two angular-rate interpolators versus PS-CPI, in each of four noise groups. PS-CPI interpolation is bold only if it is significantly better than both interpolators. Repeated noise-free trials do not count as independent snippets.}",
            "",
            r"\SecondRevision{Table~\ref{tab:EurocProxySummary} summarizes medians across all five sample counts and groups positive noise into bands, whereas Table~\ref{tab:EurocProxyRms} fixes $M=41$ samples ($m=40$ intervals, 200~Hz) and reports the same RMS final attitude-propagation metric used in Tables~\ref{tab:ConingAngVel} and~\ref{tab:ConingDelTheta} at every sampled noise fraction. In the no-noise row, converged PS-CPI has the lowest RMS error. At positive noise, Sequential Rotation and One-Speed are nearly identical and slightly lower than converged PS-CPI.}",
            "",
            r"The drone proxy results show that PS-CPI is most effective for noise-free endpoint integration and for angular-rate interpolation with positive noise. \SecondRevision{Table~\ref{tab:EurocProxySummary} reports median endpoint error in degrees, PS-CPI win rate against the one-speed update, normalized angular-rate interpolation error, and median runtime. The table explicitly separates trials with no added noise from trials with positive low noise. With no added noise, PS-CPI reduces the median endpoint error from 0.0013~deg to 0.0003~deg and wins 80\% of matched trials. With positive noise, the endpoint errors remain close and One-Speed has the lower median. Against the strongest non-PS-CPI angular-rate interpolator, PS-CPI reduces the median interpolation error by 26\% in the positive-low-noise band, 25\% in the middle-noise band, and 27\% in the high-noise band.}",
            "",
            r"\begin{itemize}",
            r"  \item \textbf{Noise} separates trials with no added noise from positive low, middle, and high injected gyro-noise bands.",
            r"  \item \textbf{Endpoint} reports median final SO(3) attitude error in degrees after the 200~ms interval, where smaller is better. \textbf{One-Speed} is the classical single-speed coning update, \textbf{PS-CPI} is the pseudo-spectral Chebyshev-Picard update, and \textbf{Win} is the fraction of matched trials in which PS-CPI has lower final error than One-Speed.",
            r"  \item \textbf{$\bomega$ interp.} reports unitless normalized angular-rate interpolation error on a dense grid over the interval, where smaller is better. \textbf{Baseline} is the better of componentwise linear and natural cubic-spline interpolation in that noise band, while \textbf{PS-CPI} is the corresponding polynomial-fit result.",
            r"  \item \textbf{Runtime} gives median Python-level wall-clock time in microseconds per wrapped GTSAM integration call, where smaller is faster. \textbf{Seq. Rot.} is the sequential-rotation/RK2 baseline, \textbf{One-Speed} is the single-speed coning baseline, \textbf{PS-CPI} includes the sample-to-CGL fit used by the report, and \textbf{Rel.} normalizes those timings by the One-Speed runtime.",
            r"\end{itemize}",
            "",
        ]
    )
    (PAPER_DIR / "EurocExperiment.tex").write_text("\n".join(lines), encoding="utf-8")


def euroc_rms_table_lines(
    summary: pd.DataFrame,
    *,
    label: str = "tab:EurocProxyRms",
    caption: str | None = None,
) -> list[str]:
    """Return the complete EuRoC endpoint RMS table."""
    if caption is None:
        caption = (
            r"RMS of attitude propagation error (in mrad) for 200~ms "
            r"EuRoC-derived rate-gyro proxies with $M=41$ samples (200~Hz). "
            r"Each entry aggregates 1,100 Monte Carlo trials: 100 noise draws "
            r"for each of 11 snippets."
        )
    return [
        r"\begin{table}[th]",
        r"  \centering",
        "  " + rf"\caption{{{paper_revision(caption)}}}",
        "  " + rf"\label{{{label}}}",
        r"  \scriptsize",
        r"  \setlength{\tabcolsep}{2.5pt}",
        r"  \begin{tabular}{l | c c c c | c c}",
        r"    \hline",
        r"    & \multicolumn{4}{c|}{\SecondRevision{Classical Methods}} & \multicolumn{2}{c}{\SecondRevision{This Work}} \\",
        r"    \hline",
        r"    \SecondRevision{Sec.} & \SecondRevision{II.D.1} & \SecondRevision{II.D.4} & \SecondRevision{II.D.2} & \SecondRevision{II.D.3} & \SecondRevision{IV.A} & \SecondRevision{IV.A} \\",
        r"    \hline",
        r"    \SecondRevision{Noise} & \SecondRevision{No} & \SecondRevision{Two-} & \SecondRevision{Seq.} & \SecondRevision{One-} & \SecondRevision{PS-CPI} & \SecondRevision{PS-CPI} \\",
        r"    \SecondRevision{fraction} & \SecondRevision{Coning} & \SecondRevision{Speed} & \SecondRevision{Rot.} & \SecondRevision{Speed} & \SecondRevision{(1 iter.)} & \SecondRevision{(convg.)} \\",
        r"    \hline",
        *euroc_rms_latex_rows(summary),
        r"    \hline",
        r"  \end{tabular}",
        r"\end{table}",
        "",
    ]


def euroc_rms_latex_rows(summary: pd.DataFrame) -> list[str]:
    """Format EuRoC RMS rows with every visible entry marked as a revision."""
    rows = []
    for _, summary_row in summary.iterrows():
        values = [
            paper_revision(
                format_noise_fraction(float(summary_row["Noise fraction alpha"]))
            )
        ]
        values.extend(
            paper_revision(r"\textemdash" if value is None else f"{value:.3f}")
            for value in euroc_rms_method_values(summary_row)
        )
        rows.append("    " + " & ".join(values) + r" \\")
    return rows


def paper_revision(value: str) -> str:
    """Wrap visible EuRoC RMS content in the paper's blue revision command."""
    return rf"\SecondRevision{{{value}}}"


def format_noise_fraction(value: float) -> str:
    """Format exact EuRoC RMS noise fractions without dropping used precision."""
    return "0.000" if np.isclose(value, 0.0) else f"{value:.3f}"


def compact_summary_row(
    endpoint: pd.DataFrame,
    signal: pd.DataFrame,
    noise_group: str,
    significance: pd.DataFrame,
) -> list[str]:
    endpoint_band = select_noise_group(endpoint, noise_group)
    signal_band = select_noise_group(signal, noise_group)

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
    tests = significance[significance["noise_group"].eq(noise_group)]
    endpoint_significant = bool(
        tests.loc[tests["metric"].eq("endpoint"), "p_holm"].iloc[0] < 0.05
    )
    signal_medians = signal_band.groupby("method", observed=True)["gyro_nrmse"].median()
    best_baseline = signal_medians.drop("PsyChMagPI").idxmin()
    signal_tests = tests[tests["metric"].eq("signal")].set_index("baseline")
    signal_significant = bool(
        signal_tests["p_holm"].lt(0.05).all()
        if signal_psych < signal_best
        else signal_tests.loc[best_baseline, "p_holm"] < 0.05
    )
    endpoint_pair = highlight_lower(d3_endpoint, psych_endpoint, endpoint_significant)
    signal_pair = highlight_lower(signal_best, signal_psych, signal_significant)
    label = noise_group_label(noise_group)
    win = f"{100.0 * psych_win:.0f}\\%"
    if noise_group in {"none", "low"}:
        label = rf"\SecondRevision{{{label}}}"
        win = rf"\SecondRevision{{{win}}}"
    return [
        label,
        *endpoint_pair,
        win,
        *signal_pair,
    ]


def noise_group_label(noise_group: str) -> str:
    """Return the paper label for an explicit noise group."""
    return "None" if noise_group == "none" else noise_group.title()


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
    excluded_methods: frozenset[str] = frozenset(),
) -> float:
    rows = []
    for method, group in frame.groupby("method", observed=True):
        if str(method) in excluded_methods:
            continue
        value = float(np.median(group[column].to_numpy(dtype=float)))
        rows.append((str(method), value))
    rows.sort(key=lambda item: item[1], reverse=not smaller_is_better)
    return rows[0][1]


def highlight_lower(first: float, second: float, significant: bool) -> list[str]:
    """Bold a strictly lower error only when the adjusted test is significant."""
    formatted = [fmt_error(first), fmt_error(second)]
    if significant and first != second:
        lower_index = int(second < first)
        formatted[lower_index] = rf"\textbf{{{formatted[lower_index]}}}"
    return [rf"\SecondRevision{{{value}}}" for value in formatted]


def fmt_error(value: float) -> str:
    """Format compact-table errors consistently to four decimal places."""
    return f"{float(value):.4f}"


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
