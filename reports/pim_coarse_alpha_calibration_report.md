# PIM Coarse Alpha Calibration Report

Generated: 2026-05-04 02:04 UTC / 2026-05-03 22:04 America/New_York.

## Main Results

Recommended combined-objective values for the dataset classes:

| dataset_class | alpha_gyro | alpha_acc | mean_nees | sum_deviations |
| --- | --- | --- | --- | --- |
| all | 13.0 | 10.0 | 0.730000 | 9.366500 |
| machine_hall | 5.0 | 7.0 | 0.967204 | 1.342120 |
| vicon | 13.0 | 10.0 | 0.998951 | 3.439090 |

These are unchanged from the earlier three-method search, but the objective is now correctly balanced: quadrature and tangent each get one vote. Manifold is excluded from the calibration objective because it tracks tangent very closely in this code path; including both would effectively give the tangent/manifold implementation family twice the weight of quadrature. Tangent is the representative non-quadrature PIM in the combined objective.

## Search Method

I reran `evalPimNoiseCalibration` after changing it to omit manifold from both the combined and method-specific calibration studies. It uses:

- PIM methods in the objective: quadrature and tangent.
- Dataset classes: all, machine_hall, vicon.
- Window interval: 0.2 s.
- Prior covariance: none, matching `evalQuadratureImuFactorDiagnostics`.
- Coarse grid only: alpha values {0.5, 1, 2, 3, 5, 7, 10, 13} for gyro and accelerometer.
- Criterion: minimize `sum(abs(median_normalized_nees - 1))` over the selected class. For the combined objective, the sum includes every dataset-method pair in the class for quadrature and tangent only.

Command run:

```bash
cd /Users/dellaert/git/imuFactors/build
./evalPimNoiseCalibration --output-root ./pim_alpha_study_results_no_manifold
```

Canonical output directory:

`/Users/dellaert/git/imuFactors/build/pim_alpha_study_results_no_manifold/evalPimNoiseCalibration/20260504T020331353Z`

Copied summary CSVs:

- `/Users/dellaert/git/imuFactors/reports/pim_coarse_alpha_summaries.csv`
- `/Users/dellaert/git/imuFactors/reports/pim_coarse_alpha_trials.csv`

## Method-Specific Optima

| dataset_class | method | alpha_gyro | alpha_acc | mean_nees | sum_deviations |
| --- | --- | --- | --- | --- | --- |
| all | quadrature | 13.0 | 10.0 | 0.715207 | 4.759940 |
| all | tangent | 13.0 | 10.0 | 0.744794 | 4.606560 |
| machine_hall | quadrature | 2.0 | 10.0 | 0.879305 | 0.610693 |
| machine_hall | tangent | 7.0 | 7.0 | 0.894602 | 0.633030 |
| vicon | quadrature | 13.0 | 10.0 | 0.969477 | 1.810350 |
| vicon | tangent | 13.0 | 10.0 | 1.028430 | 1.628740 |

## Class Details

### all

Combined optimum: alpha_gyro=13.0, alpha_acc=10.0; mean NEES=0.730000; sum abs deviation=9.366500.

Best combined-candidate median NEES by dataset and method:

| dataset | quadrature | tangent |
| --- | --- | --- |
| MH01 | 0.381334 | 0.379129 |
| MH02 | 0.306681 | 0.298980 |
| MH03 | 0.430605 | 0.467229 |
| MH04 | 0.456465 | 0.412540 |
| MH05 | 0.475329 | 0.464299 |
| V101 | 0.906778 | 0.917853 |
| V102 | 0.501168 | 0.564421 |
| V103 | 0.826750 | 0.945111 |
| V201 | 0.777774 | 0.843524 |
| V202 | 0.990784 | 1.218470 |
| V203 | 1.813610 | 1.681180 |

Top combined coarse candidates:

| alpha_gyro | alpha_acc | mean_nees | sum_deviations |
| --- | --- | --- | --- |
| 13.0 | 10.0 | 0.730000 | 9.366500 |
| 10.0 | 10.0 | 0.810620 | 9.825060 |
| 13.0 | 7.0 | 1.344070 | 11.662000 |
| 7.0 | 10.0 | 0.987129 | 11.679700 |
| 7.0 | 13.0 | 0.718707 | 11.799000 |
| 10.0 | 13.0 | 0.549771 | 11.816000 |
| 13.0 | 13.0 | 0.480040 | 12.327000 |
| 10.0 | 7.0 | 1.439150 | 13.300900 |

### machine_hall

Combined optimum: alpha_gyro=5.0, alpha_acc=7.0; mean NEES=0.967204; sum abs deviation=1.342120.

Best combined-candidate median NEES by dataset and method:

| dataset | quadrature | tangent |
| --- | --- | --- |
| MH01 | 0.827736 | 1.012580 |
| MH02 | 0.658384 | 0.777227 |
| MH03 | 0.927702 | 1.268320 |
| MH04 | 0.973914 | 1.066580 |
| MH05 | 1.035180 | 1.124420 |

Top combined coarse candidates:

| alpha_gyro | alpha_acc | mean_nees | sum_deviations |
| --- | --- | --- | --- |
| 5.0 | 7.0 | 0.967204 | 1.342120 |
| 7.0 | 7.0 | 0.870103 | 1.405010 |
| 10.0 | 7.0 | 0.818018 | 1.819820 |
| 13.0 | 7.0 | 0.798227 | 2.017730 |
| 3.0 | 10.0 | 0.868402 | 2.825010 |
| 3.0 | 7.0 | 1.294540 | 3.422410 |
| 3.0 | 13.0 | 0.680629 | 3.674230 |
| 5.0 | 10.0 | 0.572176 | 4.278240 |

### vicon

Combined optimum: alpha_gyro=13.0, alpha_acc=10.0; mean NEES=0.998951; sum abs deviation=3.439090.

Best combined-candidate median NEES by dataset and method:

| dataset | quadrature | tangent |
| --- | --- | --- |
| V101 | 0.906778 | 0.917853 |
| V102 | 0.501168 | 0.564421 |
| V103 | 0.826750 | 0.945111 |
| V201 | 0.777774 | 0.843524 |
| V202 | 0.990784 | 1.218470 |
| V203 | 1.813610 | 1.681180 |

Top combined coarse candidates:

| alpha_gyro | alpha_acc | mean_nees | sum_deviations |
| --- | --- | --- | --- |
| 13.0 | 10.0 | 0.998951 | 3.439090 |
| 10.0 | 10.0 | 1.129670 | 4.102620 |
| 10.0 | 13.0 | 0.779799 | 4.553340 |
| 13.0 | 13.0 | 0.668880 | 4.861330 |
| 7.0 | 13.0 | 1.047300 | 5.043010 |
| 7.0 | 10.0 | 1.409910 | 6.477690 |
| 5.0 | 13.0 | 1.528250 | 8.742000 |
| 13.0 | 7.0 | 1.798940 | 9.644220 |


## Notes

The Machine Hall class remains internally mixed: quadrature alone picks `(2.0, 10.0)`, while tangent picks `(7.0, 7.0)`. The combined Machine Hall recommendation `(5.0, 7.0)` is the compromise across quadrature and tangent. Vicon remains consistent: quadrature, tangent, and the combined objective all pick `(13.0, 10.0)` on the coarse grid.

The app defaults in `QuadratureRunner` already match these rerun combined-objective values: Machine Hall uses `(5.0, 7.0)` and Vicon uses `(13.0, 10.0)` when no alpha override is provided.
