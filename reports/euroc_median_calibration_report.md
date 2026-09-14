# EuRoC mean-of-medians noise calibration

Date: 2026-09-11. Implementation: `ae0fbe1`. Manuscript: `dd784e1` in the FiltersTutorial repository.

The new MH/V settings make the equally weighted arithmetic mean of per-sequence, per-interval, per-method normalized-NEES medians equal to one. All 11 sequences and all windows are retained. This is descriptive calibration on the full groups, not a held-out evaluation.

| Group | alpha | q (m²/s) | Achieved mean of medians | Mean NEES with the same block weights |
|---|---:|---:|---:|---:|
| MH | 6.6864088476456782 | 1.8432039280733609e-05 | 1.0000002833 | 2.168744 |
| V | 14.265918553947698 | 0 | 1.0000002500 | 1.260278 |

## Objective and scope

Each sequence, interval (0.2, 0.5, 1.0 s), and manifold/GTSAM Galilean method has equal weight. The equivalent Python Delama implementation is included in the comparison but is not counted again in the fit. There are 30 calibration blocks for MH and 36 for V. The objective is the mean of block medians, not a pooled median or a mean across individual windows.

The preceding Gaussian-likelihood study compared global and MH/V leave-one-sequence-out calibration, favoring MH/V in held-out likelihood on all 11 sequences. Its rounded full-group settings were MH alpha=9.85, q=4e-5 and V alpha=16, q=0. This retuning preserves q/alpha² from those settings and fits a single covariance multiplier c per group, with alpha_new=sqrt(c)*alpha_old and q_new=c*q_old. It does not claim a new held-out result for the median-based objective.

The fit uses native residual/covariance pairs and includes the fixed 1e-12 diagonal regularization without scaling it. Residuals and predicted means are unchanged. The target of one is a chosen median normalization; the ideal Gaussian median of chi-square(9)/9 is approximately 0.927. Large outliers remain in the data, and the mean NEES remains above one after this median-based retuning.

## Pooled Galilean results

The manuscript pools windows within each interval. Those pooled medians need not equal the calibration target. Both Galilean implementations agree at the displayed precision.

| Interval (s) | Pooled mean NEES | Pooled median NEES |
|---|---:|---:|
| 0.2 | 1.170088 | 0.561206 |
| 0.5 | 1.649267 | 0.900025 |
| 1.0 | 2.110767 | 1.388255 |

## Validation and preserved evidence

- Fresh C++ and Python runs produced 28,554 window rows and 99 summaries across three methods, 11 sequences, and three intervals. Quadrature and EKFs are excluded.
- Source CSV hashes and complete window boundaries are preserved. Physical endpoint errors are unchanged from the seed run.
- All 86 Python tests passed. Escalated `make -j6 testImuNEES.run testAppUtils.run testDatasetSanity.run` passed.
- Full-precision Galilean comparisons passed on every window. Maximum absolute component differences were 4.92e-10 m in position, 8.72e-13 m/s in velocity, and 4.00e-15 per rotation-matrix entry.
- Maximum raw transported covariance-entry difference was 5.04e-11. The comparison uses atol=5e-11, rtol=1e-5 after dividing both covariances by (alpha/8.4)²; the fixed NEES diagonal regularization is coordinate dependent.
- Viewer loading, preferred method order, HTTP response, and 33 comparison-table rows passed.
- All 108 exported paper rows (9 pooled and 99 per-sequence) were checked against the canonical window data. The 14-page PDF compiled without final warnings; main table page 8 and supplemental pages 10–11 were visually verified.

Durable evidence: [exact calibration and cache hashes](euroc_median_calibration/calibration.json), [validation and source hashes](euroc_median_calibration/verification.json), [full-precision parity measurements](euroc_median_calibration/full_precision_parity.json), and [all 99 canonical summaries](euroc_median_calibration/window_summaries.csv). The summary CSV retains means, medians, P95, population variance, and physical errors/sigmas; the paper displays medians only.

The complete viewer package is local at `build/results/evalMedianGroupImuComparison/20260911T225945937249Z`. Historical packages were preserved. The package metadata records the pre-commit HEAD; the implementation used in the run is now committed as `ae0fbe1`.

## Reproduction

Use the commands in [the Python README](../python/README.md#retune-the-mean-of-block-medians) to recreate the cache, fit the covariance scales, rerun the comparison, and export the tables. The committed calibration JSON can also drive a fresh rerun directly:

```bash
conda run -n py312 python python/run_fixed_group_imu_comparison.py \
  --calibration-json reports/euroc_median_calibration/calibration.json
```

Run from the repository root. Build `evalQuadratureImuFactorDiagnostics` first and retain the bundled source CSVs matching the recorded hashes. Use the full-precision parameters in the JSON; displayed paper settings are rounded.
