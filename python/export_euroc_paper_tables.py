"""Generate median-only manuscript and complete per-sequence supplemental tables."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import pandas as pd

from run_fixed_group_imu_comparison import GROUP_SETTINGS, METHODS, validate_settings

LABELS = {'manifold':'Manifold', 'galilean':'Galilean (left)', 'delama_gal3_python':'Delama (right)'}


def values(rows, sigmas=False):
    cells=[f'{rows.normalized_nees.median():.4f}']
    for block,scale in [('rot',1000),('pos',1000),('vel',100)]:
        value=f'{rows[f"{block}_error_norm"].median()*scale:.3f}'
        if sigmas:
            value+=f' ({rows[f"{block}_pred_sigma"].median()*scale:.3f})'
        cells.append(value)
    return cells


def export_tables(package, output):
    verification=json.loads((package/'verification.json').read_text())
    if verification.get('protocol') not in ('fixed_group_refit', 'group_mean_median'):
        raise ValueError('Require a fixed-group or mean-median calibrated rerun')
    validate_settings(verification['settings'])
    median_fit = verification['protocol'] == 'group_mean_median'
    if not median_fit and verification['settings'] != GROUP_SETTINGS:
        raise ValueError('Unknown original fixed-group settings')
    frame=pd.read_csv(package/'window_metrics.csv')
    if set(frame.method)!=set(METHODS) or len(frame)!=28554:
        raise ValueError('Expected all three methods and 28,554 windows')
    main_rows=[];supplement_rows=[];numeric=[]
    for interval in [.2,.5,1.]:
        for index,method in enumerate(METHODS):
            rows=frame[(frame.interval_seconds==interval)&(frame.method==method)]
            main_rows.append('    '+(f'{interval:.1f}' if index==0 else '')+' & '+LABELS[method]+' & '+' & '.join(values(rows))+r'\\')
            numeric.append(dict(table='main',interval=interval,method=method,values=values(rows),windows=len(rows)))
        if interval!=1.:
            main_rows.append(r'    \midrule')
    for name in sorted(frame.dataset.unique()):
        for interval in [.2,.5,1.]:
            for index,method in enumerate(METHODS):
                rows=frame[(frame.dataset==name)&(frame.interval_seconds==interval)&(frame.method==method)]
                if rows.empty:
                    raise ValueError(f'Missing {name}/{interval}/{method}')
                prefix=[name,f'{interval:.1f}',str(len(rows))] if index==0 else ['','','']
                end=r'\\*' if index<2 else r'\\'
                supplement_rows.append('    '+' & '.join(prefix+[LABELS[method]]+values(rows,True))+end)
                numeric.append(dict(table='supplement',dataset=name,interval=interval,method=method,values=values(rows,True),windows=len(rows)))
            supplement_rows.append(r'    \addlinespace[3pt]')
    main=r'''\section{EuRoC Preintegration Comparison}

% Source run RUN_ID; endpoint_v2; MH alpha=9.85, q=4e-5; V alpha=16, q=0.
% Pool individual windows, not per-sequence medians. No windows are removed.
\begin{table}[!ht]
  \caption{Median preintegration metrics on all 11 EuRoC sequences,
    with fixed MH/V noise settings. Statistics pool 5,954 windows at
    0.2~s, 2,378 at 0.5~s, and 1,186 at 1.0~s per method.
    NEES is normalized by nine; errors are physical endpoint norms.}
  \label{tab:euroc-preintegration}
  \centering
  \small
  \setlength{\tabcolsep}{3pt}
  \begin{tabular}{@{}rlrrrr@{}}
    \toprule
    $T$ & Method & NEES & $R$ & $p$ & $v$\\
    (s) & & Median & (mrad) & (mm) & (cm/s)\\
    \midrule
MAIN_ROWS
    \bottomrule
  \end{tabular}
\end{table}

Table~\ref{tab:euroc-preintegration} compares the manifold preintegrator
of Forster et al.~\cite{Forster17tro_preintegration}, our left-invariant
Galilean implementation in GTSAM, and a Python implementation of the
right-invariant Galilean formulation of Delama et
al.~\cite{Delama25ral_galileanPreintegration} on all 11 EuRoC sequences.
For each sequence, we use the first timestamp difference as the fixed
IMU timestep $\Delta t$ (nominal sampling rate 200~Hz) and form windows of
$N=\operatorname{round}(T/\Delta t)$ samples for
$T\in\{0.2,0.5,1.0\}$~s.
Each window integrates samples $[k,k+N)$, compares the prediction with
ground truth at $k+N$, and advances the next start by $N$.
The methods receive identical samples and endpoints; pooled medians
weight sequences in proportion to their complete-window counts.

We report medians to describe typical-window behavior in the presence
of isolated reference-trajectory inconsistencies.
For example, the merged MH04 reference contains a 5-ms position step
that differs by approximately 13~cm from the displacement implied by
its recorded velocities; similar isolated discrepancies occur in MH03
and MH05. Such events can dominate arithmetic averages of NEES.
All windows remain included, so using medians does not remove these
outliers or establish distributional consistency.
Table~\ref{tab:euroc-sequence-medians} in the supplemental material
provides the complete per-sequence breakdown, including median
predicted sigmas.

Every window starts from its ground-truth navigation state and IMU bias,
with zero initial covariance and constant bias during preintegration.
We normalize ground-truth quaternions before constructing rotations and
fix gravity at $9.81\,\mathrm{m/s^2}$.
The continuous sensor-noise densities are
\begin{align*}
  \sigma_\omega &= \alpha(1.6968\times10^{-4})\,
                   \mathrm{rad/s}/\sqrt{\mathrm{Hz}},\\
  \sigma_a &= \alpha(2.0\times10^{-3})\,
                   \mathrm{m/s^2}/\sqrt{\mathrm{Hz}}.
\end{align*}
We use $(\alpha,q)=(9.85,4\times10^{-5}\,\mathrm{m^2/s})$ for MH
and $(16,0)$ for V, shared by every method and interval within each group.
The independent position-drive covariance contributes $q\Delta t I_3$
per step and is not scaled by $\alpha$.
A leave-one-sequence-out comparison favored separate MH/V calibration
over global calibration in held-out Gaussian likelihood on all 11
sequences. We then refit within each full group and rounded the parameters
to obtain the fixed settings used here; these tables are descriptive
full-group-calibrated results, not held-out evaluations.
The fitting objective weights sequences, intervals, and the manifold and
GTSAM Galilean methods equally, counting the equivalent Python Galilean
implementation only once.

The reported errors measure physical endpoint disagreement in common
coordinates:
\[
  e=\begin{bmatrix}
    \Log(R_{\rm pred}\T R_{\rm gt})\\
    p_{\rm gt}-p_{\rm pred}\\
    v_{\rm gt}-v_{\rm pred}
  \end{bmatrix}.
\]
We transport Delama's native covariance by the adjoint of the inverse
predicted Galilean increment, reorder its rotation--velocity--position
blocks to rotation--position--velocity, and rotate position and velocity
perturbations into world coordinates.
The GTSAM prediction-tangent covariance is rotated into the same reporting
coordinates. NEES is computed separately from each method's native
residual $r$ and covariance $P$ as $r\T(P+10^{-12}I_9)^{-1}r/9$.

The two Galilean implementations agree to every digit displayed in
Table~\ref{tab:euroc-preintegration}, despite using opposite invariant
error conventions.
Full-precision comparisons on every window satisfy tolerances of
$10^{-8}$~m in predicted position, $10^{-9}$~m/s in velocity, and
$10^{-12}$ per rotation-matrix entry; the largest absolute difference
in transported covariance entries is below $10^{-10}$.
This verifies the common held-input motion and conditional nine-dimensional
uncertainty under the fixed-bias setup, rather than comparing their joint
bias-uncertainty models. Unregularized NEES is coordinate invariant,
while the fixed diagonal regularization can introduce small differences.

'''.replace('RUN_ID',package.name).replace('MAIN_ROWS','\n'.join(main_rows))
    supplement=r'''% Generated from canonical run RUN_ID; all 99 sequence/interval/method groups.
\section{Complete EuRoC Results}
\label{app:euroc-results}

The following table lists every sequence by its mnemonic and reports only
median performance metrics. The fixed settings are $\alpha=9.85$,
$q=4\times10^{-5}\,\mathrm{m^2/s}$ for MH and $\alpha=16$, $q=0$ for V.
These are the same full-group-calibrated runs summarized in
Table~\ref{tab:euroc-preintegration}; no windows are excluded.
$N_w$ is the number of complete windows for each method at that duration.
Each error cell gives the median physical endpoint error norm, followed
in parentheses by the median predicted component-RMS sigma,
$\sqrt{\operatorname{tr}(P_{bb})/3}$.
These sigmas are component uncertainties, not standard deviations of
error norms. NEES retains each method's native residual/covariance pair
and the $10^{-12}I_9$ regularization, and is normalized by nine.
Medians limit the influence of the isolated reference discontinuities
on the summary; they do not assert Gaussian tails or nominal coverage.

\begingroup
\small
\setlength{\tabcolsep}{4pt}
\begin{longtable}{@{}lrrlrrrr@{}}
  \caption{Complete per-sequence EuRoC medians. Parentheses contain median predicted sigmas.}
  \label{tab:euroc-sequence-medians}\\
  \toprule
  Sequence & $T$ (s) & $N_w$ & Method & NEES & $R$ (mrad) & $p$ (mm) & $v$ (cm/s)\\
  \midrule
  \endfirsthead
  \multicolumn{8}{c}{\tablename~\thetable\ (continued)}\\
  \toprule
  Sequence & $T$ (s) & $N_w$ & Method & NEES & $R$ (mrad) & $p$ (mm) & $v$ (cm/s)\\
  \midrule
  \endhead
  \midrule
  \multicolumn{8}{r}{Continued on the next page}\\
  \endfoot
  \bottomrule
  \endlastfoot
SUPPLEMENT_ROWS
\end{longtable}
\endgroup

'''.replace('RUN_ID',package.name).replace('SUPPLEMENT_ROWS','\n'.join(supplement_rows))
    if median_fit:
        old = r'''A leave-one-sequence-out comparison favored separate MH/V calibration
over global calibration in held-out Gaussian likelihood on all 11
sequences. We then refit within each full group and rounded the parameters
to obtain the fixed settings used here; these tables are descriptive
full-group-calibrated results, not held-out evaluations.
The fitting objective weights sequences, intervals, and the manifold and
GTSAM Galilean methods equally, counting the equivalent Python Galilean
implementation only once.'''
        new = r'''We fit a covariance multiplier $c$ per group to make the arithmetic
mean of sequence/interval/method NEES medians equal one.
Each sequence, interval, and manifold/GTSAM Galilean method has equal
weight; the equivalent Python implementation is counted only once.
We preserve $q/\alpha^2$ from the prior Gaussian-likelihood fit, setting
$\alpha\leftarrow\sqrt{c}\alpha$ and $q\leftarrow cq$, with the
$10^{-12}I_9$ regularization fixed.
This descriptive full-group calibration achieves one in both groups,
but individual and pooled medians need not equal one.
It is not a held-out evaluation; one is a chosen normalization,
whereas the ideal Gaussian median of $\chi^2_9/9$ is approximately 0.927.'''
        if old not in main:
            raise ValueError('Missing calibration paragraph')
        main = main.replace(old, new)
        mh, v = verification['settings']['MH'], verification['settings']['V']
        mantissa, exponent = f"{mh['integration_covariance']:.5e}".split('e')
        q_tex = mantissa + r'\times10^{' + str(int(exponent)) + '}'
        for before, after in [('9.85', f"{mh['alpha']:.6g}"),
                              (r'4\times10^{-5}', q_tex),
                              ('$(16,0)$', f"$({v['alpha']:.6g},0)$"),
                              (r'$\alpha=16$', r'$\alpha=' + f"{v['alpha']:.6g}" + '$')]:
            main = main.replace(before, after)
            supplement = supplement.replace(before, after)
        main = '\n'.join(line for line in main.split('\n') if not line.startswith('% Source run'))
        main = main.replace(r'\section{EuRoC Preintegration Comparison}',
                            r'\section{EuRoC Preintegration Comparison}' + '\n% Source run ' + package.name
                            + '; group_mean_median; exact settings in verification.json.')
    output.mkdir(parents=True,exist_ok=True)
    (output/'euroc-section.tex').write_text(main)
    (output/'appendix-euroc-comparison.tex').write_text(supplement)
    (output/'table-values.json').write_text(json.dumps(dict(package=str(package),rows=numeric),indent=2)+'\n')
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('package',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(export_tables(args.package.resolve(),args.output.resolve()))


if __name__=='__main__':
    main()
