# Independent gyro/accelerometer EuRoC calibration

Base: `6c156c8` (merged PR #19). Branch: `codex/separate-gyro-accel-calibration`. Fit: `20260914T171127Z`.

Allowing independent sensor scales improves the full-group Gaussian likelihood substantially in Machine Hall and slightly in Vicon Room. The recommended triplets below preserve the earlier mean-of-block-medians NEES target of one. All 11 sequences and all windows remain included; these are descriptive full-group results, not held-out generalization scores.

## Recommended settings: mean of block medians = 1

| Group | Gyro alpha | Accelerometer alpha | q (m²/s) | Achieved mean of medians | Mean NEES |
|---|---:|---:|---:|---:|---:|
| MH | 3.10910372571003 | 9.75984113537511 | 1.69745240716305e-05 | 0.999999667 | 1.868071 |
| V | 13.537475897322 | 14.5208515964082 | 0 | 0.999999847 | 1.273252 |

## Likelihood-optimal settings before median normalization

| Group | Gyro alpha | Accelerometer alpha | q (m²/s) | Tied-alpha NLL | Separate-alpha NLL | Improvement (nats/window) |
|---|---:|---:|---:|---:|---:|---:|
| MH | 4.24945069 | 13.33952396 | 3.170974666e-05 | -36.409632 | -38.039922 | 1.630290 |
| V | 15.27552273 | 16.38515188 | 0 | -35.054668 | -35.063354 | 0.008686 |

The likelihood optimum and the recommended median-normalized triplet answer different questions. The recommended values preserve the fitted sensor/position noise ratios and multiply covariance by one common factor per group. The arithmetic mean of sequence/interval/method NEES medians is one; individual or pooled medians need not be. One is a chosen normalization; the ideal Gaussian median of chi-square(9)/9 is about 0.927.

## Search and weighting

The sensor densities are alpha_gyro × 1.6968e-4 rad/s/√Hz and alpha_acc × 2e-3 m/s²/√Hz. The independent continuous position-drive covariance q has units m²/s and contributes q × dt × I per step. Each sequence, interval (0.2/0.5/1.0 s), and manifold/GTSAM Galilean method has equal weight. The equivalent Python Galilean implementation and quadrature are evaluated but are not counted in fitting.

Independent covariance bases give P = (alpha_gyro/8.4)² Pg + (alpha_acc/8.4)² Pa + q Q. The overall covariance scale is profiled analytically at each pair of ratios. The sweep uses 17 log accelerometer/gyro power ratios from -8 to 8, 15 log position-drive ratios from -16 to 12, and an explicit q=0 boundary. Three promising candidates are refined with bounded Nelder–Mead; q=0 also receives a separate refinement. Ratios use q_reference=1e-5. Boundary winners are rejected. This identifies the best solution found by the documented sweep/refinements, not a proof of global optimality.

Gaussian NLL uses physical endpoint errors and reporting covariance, including log determinants. After fitting noise ratios, a scalar root solve adjusts alpha_gyro and alpha_acc by sqrt(c) and q by c so the native normalized-NEES mean of medians equals one, with the 1e-12 diagonal regularization held fixed.

## Verification

- Independent covariance bases reproduce fresh propagation at mixed sensor scales and q on every sequence and interval.
- Fresh four-method rerun: 38,072 window rows, 132 summaries; same configurations, window boundaries and source hashes across methods.
- Physical endpoint errors are unchanged from the previous tied-sensor run.
- Fitted-setting C++/Python Galilean prediction and transported-covariance parity passed on all 11 sequences and three intervals. Covariance tolerance is 5e-11 absolute and 1e-5 relative after normalizing both matrices by max(alpha_gyro,alpha_acc)²/8.4².
- All 99 Python tests and the relevant escalated C++ tests passed. Viewer loading, four-method order, 33 comparison rows, and HTTP response passed.

Viewer package: `build/results/evalSeparateNoiseGroupImuComparisonWithQuadrature/20260914T171354722977Z`.

Exact fitted parameters, sweep/refinement evaluations, parity results, source hashes and canonical summaries are saved alongside this report under `euroc_separate_noise_calibration/`. Regenerate with the independent-sensor commands in `python/README.md`. Historical packages and manuscript files are preserved.
