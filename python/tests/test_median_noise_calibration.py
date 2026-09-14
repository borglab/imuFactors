import numpy as np
import pytest

from imuFactors.noise_calibration import fit_mean_median_scale, normalized_nees
from run_fixed_group_imu_comparison import validate_settings


def block(values, variance=1.):
    errors = np.zeros((len(values), 9))
    errors[:, 0] = np.sqrt(9 * np.asarray(values) * variance)
    covariance = np.broadcast_to(variance * np.eye(9), (len(values), 9, 9)).copy()
    return errors, covariance


def test_equal_block_weight_uses_medians_not_window_means_or_pooled_median():
    blocks = [block([1., 2., 1000.]), block([4.] * 101)]
    fit = fit_mean_median_scale(blocks, regularization=0.)
    assert fit['mean_median_before'] == 3.
    assert fit['covariance_scale'] == pytest.approx(3.)
    assert fit['mean_median_after'] == pytest.approx(1.)


def test_regularization_is_not_scaled_and_agrees_with_direct_solve():
    errors, covariance = block([1., 2., 3.], variance=1e-12)
    fit = fit_mean_median_scale([(errors, covariance)], target=1.2)
    expected_scale = 2. / 1.2 - 1.
    assert fit['covariance_scale'] == pytest.approx(expected_scale)
    assert np.median(normalized_nees(errors, covariance * expected_scale)) == pytest.approx(1.2)


def test_scale_preserves_noise_ratio_and_zero_position_drive():
    fit = fit_mean_median_scale([block([.2, .5, 1.])])
    scale = fit['covariance_scale']
    for alpha, q in [(9.85, 4e-5), (16., 0.)]:
        new_alpha, new_q = alpha * np.sqrt(scale), q * scale
        assert new_q / new_alpha**2 == pytest.approx(q / alpha**2)


def test_unreachable_target_and_invalid_inputs_fail():
    with pytest.raises(ValueError, match='unreachable'):
        fit_mean_median_scale([block([0., 0.])])
    for target in [0., -1., np.nan, np.inf]:
        with pytest.raises(ValueError):
            fit_mean_median_scale([block([1.])], target=target)
    with pytest.raises(ValueError):
        fit_mean_median_scale([])
    errors, covariance = block([1.])
    covariance[0, 0, 0] = -1.
    with pytest.raises(ValueError, match='positive definite'):
        fit_mean_median_scale([(errors, covariance)])


@pytest.mark.parametrize('alpha,q', [(0., 0.), (np.inf, 0.), (1., -1.), (1., np.nan)])
def test_settings_reject_invalid_noise(alpha, q):
    with pytest.raises(ValueError):
        validate_settings({group: dict(alpha=alpha, integration_covariance=q) for group in ('MH', 'V')})
