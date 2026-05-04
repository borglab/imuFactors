# Alpha Calibration and PIM NEES Report

Generated: 2026-05-04 00:56 UTC / 2026-05-03 20:56 America/New_York.

## Best alpha values

Best values from the all-dataset fine grid search:

- alpha_gyro: 16.3
- alpha_acc: 8.8

The coarse best was alpha_gyro=13.0, alpha_acc=10.0. Alec's later default trajectory-export values in the repository were alpha_gyro=13.0 and alpha_acc=9.4; the rerun here found a slightly different all-dataset fine-grid optimum because the current fine search expanded around the coarse best alpha_gyro=13.0, alpha_acc=10.0 and evaluated all current EuRoC CSVs.

## How the values were obtained

I traced the existing Alec calibration implementation through `src/NoiseCalibration.h` and `apps/evalNoiseCalibration.cpp`. The method is a two-stage deterministic 2D grid search over separate gyroscope and accelerometer noise scaling factors:

1. Coarse search over alpha values {0.5, 1, 2, 3, 5, 7, 10, 13} for both gyro and accelerometer.
2. Fine search around the coarse best, from 0.7x to 1.3x for each alpha, in increments of 0.2.
3. For each pair, run `EKFNEESEvaluator::runGal3ImuEKF(0.2, params, dataset)` on every dataset.
4. Score each pair using each dataset's median normalized NEES and minimize `sum(abs(dataset_median_nees - 1))` across datasets.

Calibration command:

```bash
cd /Users/dellaert/git/imuFactors/build
./evalNoiseCalibration --output-root ./alpha_study_results all both
```

Calibration output directory:

`/Users/dellaert/git/imuFactors/build/alpha_study_results/evalNoiseCalibration/20260504T004715238Z`

Calibration summary:

| study_name | result_label | alpha_gyro | alpha_acc | mean_nees | sum_deviations |
| --- | --- | --- | --- | --- | --- |
| all_coarse | best | 13.0 | 10.0 | 0.748146 | 4.574670 |
| all_coarse | worst | 0.5 | 0.5 | 339.860000 | 3727.460000 |
| all_fine | best | 16.3 | 8.8 | 0.851440 | 4.117190 |
| all_fine | worst | 9.1 | 7.0 | 1.526830 | 7.318740 |

Fine-search dataset medians at alpha_gyro=16.3, alpha_acc=8.8:

| dataset | dataset_nees |
| --- | --- |
| MH01 | 0.471717 |
| MH02 | 0.372901 |
| MH03 | 0.551759 |
| MH04 | 0.515455 |
| MH05 | 0.593163 |
| V101 | 0.971808 |
| V102 | 0.647523 |
| V103 | 1.093620 |
| V201 | 1.014840 |
| V202 | 1.383970 |
| V203 | 1.749090 |

Top 10 fine-search candidates:

| alpha_gyro | alpha_acc | trial_mean_nees | trial_sum_deviations |
| --- | --- | --- | --- |
| 16.3 | 8.8 | 0.851440 | 4.117190 |
| 16.5 | 8.8 | 0.848830 | 4.120100 |
| 16.7 | 9.0 | 0.813527 | 4.124850 |
| 16.1 | 8.8 | 0.853573 | 4.125940 |
| 16.7 | 8.8 | 0.846510 | 4.126180 |
| 16.9 | 8.8 | 0.844878 | 4.128050 |
| 16.9 | 9.0 | 0.811198 | 4.132200 |
| 16.5 | 9.0 | 0.815704 | 4.133830 |
| 16.3 | 9.0 | 0.818780 | 4.136660 |
| 16.1 | 9.0 | 0.820936 | 4.139760 |

## PIM NEES reruns

I added separate PIM app CLI flags `--alpha-gyro` and `--alpha-acc` while preserving the old coupled `--alpha` flag. I then ran both PIM NEES apps on all datasets and all default intervals (0.2, 0.5, 1.0 seconds):

```bash
cd /Users/dellaert/git/imuFactors/build
./evalReducedNeesWithPriorCovariance --output-root ./alpha_study_results
./evalReducedNeesWithPriorCovariance --output-root ./alpha_study_results --alpha-gyro 16.3 --alpha-acc 8.8
./evalQuadratureImuFactorDiagnostics --output-root ./alpha_study_results
./evalQuadratureImuFactorDiagnostics --output-root ./alpha_study_results --alpha-gyro 16.3 --alpha-acc 8.8
```

Run directories:

- reduced default: `/Users/dellaert/git/imuFactors/build/alpha_study_results/evalReducedNeesWithPriorCovariance/20260504T005543025Z`
- reduced tuned: `/Users/dellaert/git/imuFactors/build/alpha_study_results/evalReducedNeesWithPriorCovariance/20260504T005552165Z`
- diagnostics default: `/Users/dellaert/git/imuFactors/build/alpha_study_results/evalQuadratureImuFactorDiagnostics/20260504T005600142Z`
- diagnostics tuned: `/Users/dellaert/git/imuFactors/build/alpha_study_results/evalQuadratureImuFactorDiagnostics/20260504T005606866Z`

### Overall PIM NEES effect

These aggregate all windows across every dataset and interval for each method.

| app | setting | method | num_windows | nees_mean | nees_median | nees_p95 | sum_abs_deviation_from_1 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| reduced_with_prior | default_alpha_3.0 | manifold | 9518 | 0.534655 | 0.031473 | 1.567302 | 11595.849 |
| reduced_with_prior | default_alpha_3.0 | quadrature | 9518 | 0.539812 | 0.033317 | 1.567246 | 11550.441 |
| reduced_with_prior | default_alpha_3.0 | tangent | 9518 | 0.534774 | 0.031473 | 1.568264 | 11596.808 |
| reduced_with_prior | split_alpha_g16.3_a8.8 | manifold | 9518 | 0.303220 | 0.028265 | 0.624088 | 10343.180 |
| reduced_with_prior | split_alpha_g16.3_a8.8 | quadrature | 9518 | 0.305225 | 0.030496 | 0.616483 | 10311.162 |
| reduced_with_prior | split_alpha_g16.3_a8.8 | tangent | 9518 | 0.303553 | 0.028265 | 0.625215 | 10344.174 |
| diagnostics_no_prior | default_alpha_8.4 | manifold | 9518 | 4.996746 | 1.270685 | 7.883719 | 41734.885 |
| diagnostics_no_prior | default_alpha_8.4 | quadrature | 9518 | 4.852913 | 1.165295 | 7.952106 | 40715.010 |
| diagnostics_no_prior | default_alpha_8.4 | tangent | 9518 | 4.999915 | 1.270720 | 7.938147 | 41765.174 |
| diagnostics_no_prior | split_alpha_g16.3_a8.8 | manifold | 9518 | 4.117099 | 0.946837 | 5.699537 | 34676.767 |
| diagnostics_no_prior | split_alpha_g16.3_a8.8 | quadrature | 9518 | 4.057696 | 0.955091 | 5.623317 | 34051.175 |
| diagnostics_no_prior | split_alpha_g16.3_a8.8 | tangent | 9518 | 4.121295 | 0.946794 | 5.730556 | 34716.423 |

### PIM NEES effect by interval

Delta is tuned minus default. Negative mean/median delta means the calibrated alphas reduced the normalized NEES statistic.

| app | method | interval_seconds | default_mean | tuned_mean | mean_delta | default_median | tuned_median | median_delta |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| reduced_with_prior | manifold | 0.2 | 0.177653 | 0.171727 | -0.005926 | 0.016713 | 0.016275 | -0.000438 |
| reduced_with_prior | manifold | 0.5 | 0.659775 | 0.444897 | -0.214879 | 0.111917 | 0.076780 | -0.035136 |
| reduced_with_prior | manifold | 1.0 | 2.076020 | 0.679274 | -1.396746 | 0.862639 | 0.283907 | -0.578731 |
| reduced_with_prior | quadrature | 0.2 | 0.178603 | 0.172505 | -0.006098 | 0.018223 | 0.017745 | -0.000478 |
| reduced_with_prior | quadrature | 0.5 | 0.669410 | 0.449417 | -0.219993 | 0.127070 | 0.087354 | -0.039716 |
| reduced_with_prior | quadrature | 1.0 | 2.093316 | 0.682396 | -1.410920 | 0.952636 | 0.310107 | -0.642529 |
| reduced_with_prior | tangent | 0.2 | 0.177653 | 0.171727 | -0.005926 | 0.016713 | 0.016275 | -0.000438 |
| reduced_with_prior | tangent | 0.5 | 0.659780 | 0.444949 | -0.214831 | 0.111917 | 0.076807 | -0.035110 |
| reduced_with_prior | tangent | 1.0 | 2.076963 | 0.681843 | -1.395120 | 0.862641 | 0.284050 | -0.578591 |
| diagnostics_no_prior | manifold | 0.2 | 5.796291 | 4.923482 | -0.872809 | 1.064755 | 0.785836 | -0.278919 |
| diagnostics_no_prior | manifold | 0.5 | 3.505612 | 2.695356 | -0.810256 | 1.572765 | 1.206025 | -0.366740 |
| diagnostics_no_prior | manifold | 1.0 | 3.972651 | 2.919544 | -1.053107 | 2.354510 | 1.797055 | -0.557455 |
| diagnostics_no_prior | quadrature | 0.2 | 5.639252 | 4.844844 | -0.794408 | 0.956083 | 0.793825 | -0.162258 |
| diagnostics_no_prior | quadrature | 0.5 | 3.358447 | 2.658125 | -0.700322 | 1.530625 | 1.203750 | -0.326875 |
| diagnostics_no_prior | quadrature | 1.0 | 3.901796 | 2.912253 | -0.989544 | 2.243060 | 1.770555 | -0.472505 |
| diagnostics_no_prior | tangent | 0.2 | 5.796152 | 4.923589 | -0.872563 | 1.065325 | 0.785331 | -0.279994 |
| diagnostics_no_prior | tangent | 0.5 | 3.507460 | 2.698743 | -0.808716 | 1.571715 | 1.204280 | -0.367435 |
| diagnostics_no_prior | tangent | 1.0 | 3.995077 | 2.945891 | -1.049186 | 2.351155 | 1.789980 | -0.561175 |

## Interpretation

For `evalReducedNeesWithPriorCovariance`, the split-alpha setting lowers normalized NEES relative to the app's default alpha=3.0. Across all windows, quadrature mean NEES drops from 0.539812 to 0.305225, manifold from 0.534655 to 0.303220, and tangent from 0.534774 to 0.303553. The p95 also drops from about 1.57 to about 0.62, so this setting makes the reduced/prior-covariance PIM NEES more conservative overall.

For `evalQuadratureImuFactorDiagnostics`, the split-alpha setting also lowers normalized NEES relative to that app's default alpha=8.4. Across all windows, quadrature mean NEES drops from 4.852913 to 4.057696, manifold from 4.996746 to 4.117099, and tangent from 4.999915 to 4.121295. The medians move closer to 1 for all three methods, but the means remain well above 1 because the high-NEES tail is still large.

The calibrated values are therefore best for Alec's Gal3 EKF calibration objective and consistently reduce PIM NEES, but they do not make the two PIM app families agree around NEES=1. A PIM-specific alpha search would be the right next step if the objective is to tune PIM NEES itself rather than reuse the EKF calibration objective.
