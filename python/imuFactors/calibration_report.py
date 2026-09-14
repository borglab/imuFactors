"""Human-readable analysis of the held-out calibration and reference audit."""
from pathlib import Path
import json

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def markdown_table(frame, decimals=4):
    columns=list(frame.columns)
    lines=['| '+' | '.join(columns)+' |','| '+' | '.join(['---']*len(columns))+' |']
    for row in frame.itertuples(index=False, name=None):
        lines.append('| '+' | '.join(f'{x:.{decimals}g}' if isinstance(x,(float,np.floating)) else str(x) for x in row)+' |')
    return '\n'.join(lines)


def write_calibration_report(output, sources):
    output=Path(output)
    scores=pd.read_csv(output/'heldout_scores.csv')
    folds=pd.read_csv(output/'folds.csv')
    audit=pd.read_csv(output/'reference_audit.csv')
    extremes=pd.read_csv(output/'extreme_windows.csv')
    independent=scores[scores.method.isin(['manifold','galilean'])]
    nll=independent.groupby(['group','protocol']).gaussian_nll.mean().unstack()
    nll=nll[['baseline','global_loso','group_loso']]
    paired=independent.groupby(['dataset','protocol']).gaussian_nll.mean().unstack()
    paired['group_minus_global']=paired.group_loso-paired.global_loso
    paired.to_csv(output/'paired_sequence_nll.csv')
    summary=[]
    gal=scores[scores.method=='galilean']
    for (protocol,group,interval), rows in gal.groupby(['protocol','group','interval_seconds']):
        summary.append(dict(protocol=protocol,group=group,interval=interval,
                            mean_nees=np.average(rows.normalized_nees_mean,weights=rows.num_windows),
                            coverage_95=np.average(rows.upper_95_ellipsoid_coverage,weights=rows.num_windows)))
    summary=pd.DataFrame(summary)
    summary.to_csv(output/'heldout_group_summary.csv',index=False)
    figure=make_subplots(rows=2,cols=2,subplot_titles=['MH: mean normalized NEES','V: mean normalized NEES',
                                                     'MH: 95% ellipsoid coverage','V: 95% ellipsoid coverage'])
    colors={'baseline':'#555555','global_loso':'#2563a6','group_loso':'#d36b22'}
    for col,group in enumerate(['MH','V'],1):
        for protocol in colors:
            block=summary[(summary.group==group)&(summary.protocol==protocol)]
            for row,metric in [(1,'mean_nees'),(2,'coverage_95')]:
                figure.add_trace(go.Scatter(x=block.interval,y=block[metric],mode='lines+markers',
                                           name=protocol,legendgroup=protocol,showlegend=col==1 and row==1,
                                           line_color=colors[protocol]),row=row,col=col)
        figure.add_hline(y=1,line_dash='dot',line_color='#aaaaaa',row=1,col=col)
        figure.add_hline(y=.95,line_dash='dot',line_color='#aaaaaa',row=2,col=col)
        figure.update_yaxes(range=[0,1.02],row=2,col=col)
    figure.update_xaxes(title_text='Window duration (s)',tickvals=[.2,.5,1.])
    figure.update_layout(title='Held-out Galilean calibration: every reference window retained',
                         template='plotly_white',height=750,width=1100)
    figure.write_html(output/'heldout_comparison.html',include_plotlyjs=True)
    jumps=make_subplots(rows=3,cols=1,subplot_titles=['MH03','MH04','MH05'],vertical_spacing=.09)
    for row,name in enumerate(['MH03','MH04','MH05'],1):
        raw=pd.read_csv(sources[name])
        times=raw.t.to_numpy()-raw.t.iloc[0];dt=np.diff(times)
        position=raw[['p_x','p_y','p_z']].to_numpy();velocity=raw[['v_x','v_y','v_z']].to_numpy()
        discrepancy=np.linalg.norm(np.diff(position,axis=0)-.5*(velocity[:-1]+velocity[1:])*dt[:,None],axis=1)
        peak=times[np.argmax(discrepancy)]
        select=(times[:-1]>=peak-.2)&(times[:-1]<=peak+.2)
        jumps.add_trace(go.Scatter(x=times[:-1][select],y=1000*discrepancy[select],mode='lines+markers',
                                   name=name,showlegend=False),row=row,col=1)
        jumps.update_yaxes(title_text='Discrepancy (mm)',row=row,col=1)
    jumps.update_xaxes(title_text='Time from sequence start (s)')
    jumps.update_layout(title='Reference position step minus trapezoidal velocity displacement',
                        template='plotly_white',height=850,width=1000)
    jumps.write_html(output/'reference_discontinuities.html',include_plotlyjs=True)
    ranges=folds.groupby(['protocol','group']).agg(alpha_min=('alpha','min'),alpha_max=('alpha','max'),
                                                 q_min=('integration_covariance','min'),q_max=('integration_covariance','max')).reset_index()
    audit_table=audit[audit.dataset.str.startswith('MH')][['dataset','max_step_discrepancy_m','top_1pct_nees_fraction']]
    packages=json.loads((output/'packages.json').read_text())
    wins=int((paired.group_minus_global<0).sum())
    baseline_better = paired.index[paired.baseline < paired.group_loso].tolist()
    lines=[
        '# Held-out MH/V noise calibration',
        'The group-specific model improves sequence-balanced held-out Gaussian NLL over the global model '
        f'on {wins} of {len(paired)} sequences. Lower NLL is better; it includes the covariance log-determinant '
        'and therefore penalizes excessive uncertainty. The fixed baseline remains unchanged.',
        '## Protocol',
        'Leave out one complete sequence at a time. Global LOSO trains on the other ten sequences; group LOSO '
        'trains on the remaining four MH or five V sequences. Fit one shared alpha and integration covariance q '
        'across all three intervals and apply it to manifold, GTSAM Galilean, and Python Delama. The fitting '
        'loss gives equal weight to sequences, intervals, and the manifold/GTSAM Galilean methods; the '
        'numerically equivalent Python Galilean implementation is not counted a second time. No windows '
        'are excluded, no interval-specific or method-specific parameters are fitted, and held-out errors '
        'never enter the corresponding fit. Calibration uses physical errors/covariance; reported NEES '
        'uses each native residual/covariance pair with 1e-12 diagonal regularization.',
        'The unregularized Gaussian likelihood profiles out the overall covariance scale and searches the '
        'remaining ratio q/(alpha/8.4)^2, including q=0. Covariance is reconstructed linearly from the '
        'zero-q basis and checked against direct C++ and Python propagation at every fitted held-out setting. '
        'The reference state and bias initialize each window, and initial covariance is zero.',
        '## Reference audit',
        'The diagnostic is the norm of (p[i+1]-p[i]) - (v[i+1]+v[i])*dt/2. Large values reveal '
        'inconsistency between the position and velocity streams in the merged reference. This audit '
        'does not establish whether the original reference estimation or preprocessing introduced the '
        'discontinuity. A 1 cm threshold is recorded for descriptive counts only and is not used for fitting.',
        markdown_table(audit_table),
        'The worst MH04 event has about 13 cm of unexplained reference displacement within one 5 ms step. '
        'Corresponding events are about 8.1 cm in MH03 and 5.3 cm in MH05. The median step discrepancies '
        'are below a micrometer. The top-window fractions use the highest ceil(1% of windows) baseline '
        'Galilean NEES values at 0.2 s. These localized events explain why the MH mean can be huge '
        'while its median is modest. See [reference discontinuities](reference_discontinuities.html) and '
        '[source-row diagnostics](extreme_windows.csv).',
        '## Held-out likelihood',
        'Each entry below averages the two independent methods, three intervals, and sequences equally. '
        'NLL uses the same physical coordinates and units in every comparison; its absolute value is not '
        'a dimensionless quality score.',
        markdown_table(nll.reset_index()),
        markdown_table(paired.reset_index()),
        'The original baseline still has better held-out NLL than the group model on ' +
        ', '.join(baseline_better) + '. Thus the grouping improves over global recalibration, '
        'but does not dominate the fixed baseline on every sequence. Near-unit mean NEES also '
        'does not ensure nominal 95% coverage or Gaussian tails.',
        '## Fitted parameter ranges',
        'Ranges span the independently trained held-out folds; they are not confidence intervals. q has '
        'units m^2/s and alpha scales the gyro and accelerometer noise standard deviations.',
        markdown_table(ranges),
        '## NEES and coverage',
        'The following Galilean results pool held-out windows within each group and interval. Coverage is '
        'the fraction with native normalized NEES <= chi2_9(0.95)/9 (the upper 95% ellipsoid). It is '
        'descriptive: adjacent windows and the three interval partitions are not independent Monte Carlo '
        'trials. Mean normalized NEES has ideal expectation one under the assumed model, while '
        'the median does not have target one.',
        markdown_table(summary),
        'See [interactive comparison](heldout_comparison.html). Calibration changes covariance, sigmas, '
        'and NEES, but leaves predicted motion and physical errors unchanged. A better likelihood does '
        'not imply a better mean trajectory, nor does inflated q repair a discontinuous reference. '
        'Use the cross-validation result to assess the grouping rule, rather than selecting the best '
        'parameters after observing the held-out test sequence.',
        '## Reproducibility and packages',
        'The protocol, source SHA-256 hashes, per-fold training membership, fitted parameters, full '
        'precision cache, and per-sequence scores accompany this report. Each viewer package contains '
        '28,554 metric rows and 99 summaries (three methods, all 11 sequences and intervals).',
        '\n'.join(f'- {name}: `{path}`' for name,path in packages.items()),
    ]
    (output/'report.md').write_text('\n\n'.join(lines)+'\n')
    return output/'report.md'
