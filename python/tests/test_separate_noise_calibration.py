import numpy as np
import pytest

from imuFactors.separate_noise_calibration import ProfileLikelihood, covariance_at
from imuFactors.delama_gal3.preintegration_delama_gal3 import configuration_label, evaluate_interval, noise_scales
from run_fixed_group_imu_comparison import validate_settings
from test_endpoint_reporting import write_stream
from test_noise_calibration import exact_moment_errors


def synthetic_block(gyro=4., acc=12., q=3e-6):
    # Independent rotation, velocity and position information identifies all 3 scales.
    g = np.diag([2., 3., 4., .1, .2, .3, .2, .1, .4]) * 1e-7
    a = np.diag([0., 0., 0., 2., 3., 4., 5., 6., 7.]) * 1e-7
    d = np.diag([0., 0., 0., .5, .5, .5, 0., 0., 0.])
    truth = covariance_at(g, a, d, gyro, acc, q)
    return {k: np.repeat(v[None], 18, axis=0) for k, v in [('gyro', g), ('acc', a), ('drive', d)]} | {
        'error': exact_moment_errors(truth)}


@pytest.mark.parametrize('q', [0., 3e-6])
def test_three_parameter_fit_recovers_known_covariance(q):
    profile = ProfileLikelihood.from_blocks([synthetic_block(q=q)])
    result, trace = profile.fit()
    assert result['alpha_gyro'] == pytest.approx(4., rel=2e-5)
    assert result['alpha_acc'] == pytest.approx(12., rel=2e-5)
    assert result['integration_covariance'] == pytest.approx(q, rel=2e-5, abs=1e-12)
    assert any(row['ratio_q'] == 0 for row in trace)
    tied, _ = profile.fit(gyro_acc_tied=True)
    assert result['nll'] < tied['nll']


def test_profile_matches_direct_nll_and_equal_block_weighting():
    block = synthetic_block()
    doubled = {k: np.repeat(v, 4, axis=0) for k, v in block.items()}
    profile = ProfileLikelihood.from_blocks([block, doubled])
    result = profile.score((12./4.)**2, 3e-6 / ((4./8.4)**2 * 1e-5))
    covariance = covariance_at(block['gyro'], block['acc'], block['drive'], 4., 12., 3e-6)
    errors = block['error']
    direct = .5*np.mean(np.linalg.slogdet(covariance)[1] +
                       np.einsum('ni,ni->n', errors, np.linalg.solve(covariance, errors[..., None])[..., 0])
                       + 9*np.log(2*np.pi))
    assert result['nll'] == pytest.approx(direct)
    assert result == pytest.approx(ProfileLikelihood.from_blocks([block]).score(9., 3e-6 / ((4./8.4)**2 * 1e-5)))


def test_python_uniform_compatibility_and_independent_scales(tmp_path):
    import torch
    streams = write_stream(tmp_path/'input.csv', [.8, .2, -.3, .4])
    old = evaluate_interval(streams, .2, alpha=5.)
    explicit = evaluate_interval(streams, .2, alpha_gyro=5., alpha_acc=5.)
    separate = evaluate_interval(streams, .2, alpha_gyro=3., alpha_acc=11.)
    torch.testing.assert_close(old['native_covariance'], explicit['native_covariance'], atol=0, rtol=0)
    torch.testing.assert_close(old['predicted_endpoints'], separate['predicted_endpoints'], atol=0, rtol=0)
    assert configuration_label(alpha_gyro=3., alpha_acc=11.).startswith('alpha_g3_a11_')
    assert separate['alpha_gyro'] == 3. and separate['alpha_acc'] == 11.
    validate_settings({g: dict(alpha_gyro=3., alpha_acc=11., integration_covariance=0.) for g in ('MH', 'V')})


@pytest.mark.parametrize('gyro,acc,q', [(0., 1., 0.), (1., -1., 0.), (np.nan, 1., 0.), (1., np.inf, 0.), (1., 1., -1.)])
def test_invalid_separate_parameters(gyro, acc, q):
    with pytest.raises(ValueError):
        covariance_at(np.eye(9), np.eye(9), np.eye(9), gyro, acc, q)
    if q >= 0:
        with pytest.raises(ValueError):
            noise_scales(alpha_gyro=gyro, alpha_acc=acc)
    with pytest.raises(ValueError):
        validate_settings({g: dict(alpha_gyro=gyro, alpha_acc=acc, integration_covariance=q) for g in ('MH', 'V')})
