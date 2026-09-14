"""Physical reporting, quaternion input, and independent position-drive contracts."""
import numpy as np
import pytest
import torch

from imuFactors.delama_gal3.lie_group_utils import G3, SO3
from imuFactors.delama_gal3.preintegration_delama_gal3 import (
    evaluate_interval, load_ground_truth_euroc, physical_reporting,
    integration_covariance_value, compute_ext_pose_nees,
)
from imuFactors.delama_gal3.canonical import canonical_rows
from imuFactors.delama_gal3.utils import DEVICE
from run_unified_imu_comparison import validate_package, METHODS
from test_unified_comparison import package


def write_stream(path, quaternion):
    data = np.zeros((201, 23))
    data[:, 0] = np.arange(201) * .005
    data[:, 1:5] = quaternion
    data[:, 17:20] = [.3, -.2, .1]
    data[:, 20:23] = [1., 2., 9.81]
    np.savetxt(path, data, delimiter=',', header=','.join(map(str, range(23))), comments='')
    return load_ground_truth_euroc(str(path))


def test_quaternion_normalization(tmp_path):
    q = np.array([.8, .2, -.3, .4])
    a = write_stream(tmp_path / 'unit.csv', q / np.linalg.norm(q))
    for scale in (2., -3., 1e-8):
        b = write_stream(tmp_path / 'scaled.csv', scale * q)
        torch.testing.assert_close(a[0], b[0], atol=1e-15, rtol=1e-15)


@pytest.mark.parametrize('q', [[0., 0, 0, 0], [1e-14, 0, 0, 0], [np.nan, 0, 0, 0], [np.inf, 0, 0, 0]])
def test_invalid_quaternion(tmp_path, q):
    with pytest.raises(ValueError, match=r'quaternion.*bad.csv, row 2'):
        write_stream(tmp_path / 'bad.csv', q)


def test_physical_distance_and_covariance_transport():
    increment = G3.exp(torch.tensor([[.2, -.1, .3, 2., 1., -.4, .5, .7, -.1, 1.]], device=DEVICE))
    truth = increment.clone()
    truth[:, :3, 3] += torch.tensor([.2, -.1, .3], device=DEVICE)
    truth[:, :3, 4] += torch.tensor([1., 2., 2.], device=DEVICE)
    native_error = G3.log(truth.bmm(G3.inv(increment)))[:, :9]
    generator = torch.arange(81., device=DEVICE).reshape(9, 9) / 100
    covariance = (generator @ generator.T + torch.eye(9, device=DEVICE)).unsqueeze(0)
    physical, report = physical_reporting(increment, truth, increment, covariance)
    assert physical[:, 3:6].norm() == pytest.approx(3.)
    assert abs(native_error[:, 6:9].norm().item() - 3.) > .01
    # Build the full coordinate map independently, including permutation on both axes.
    adjoint = G3.Ad(G3.inv(increment))[:, :9, :9]
    permutation = torch.eye(9, device=DEVICE)[[0, 1, 2, 6, 7, 8, 3, 4, 5]]
    world = torch.block_diag(torch.eye(3, device=DEVICE), increment[0, :3, :3], increment[0, :3, :3])
    transport = world @ permutation @ adjoint[0]
    torch.testing.assert_close(report[0], transport @ covariance[0] @ transport.T)
    transformed_error = transport @ native_error[0]
    before = native_error[0] @ torch.linalg.solve(covariance[0], native_error[0])
    after = transformed_error @ torch.linalg.solve(report[0], transformed_error)
    torch.testing.assert_close(before, after)
    # Fixed diagonal jitter is coordinate dependent. Transport the jitter for invariance.
    jitter = 1e-12 * torch.eye(9, device=DEVICE)
    aligned_regularized = report[0] + transport @ jitter @ transport.T
    torch.testing.assert_close(compute_ext_pose_nees(covariance, native_error)[0],
                               transformed_error @ torch.linalg.solve(aligned_regularized, transformed_error) / 9)
    result = dict(physical_error=physical, reporting_covariance=report,
                  native_error=native_error, native_covariance=covariance,
                  config_label='synthetic_endpoint_v2', preint_time=1., steps_per_window=200,
                  starts=torch.tensor([0]), ends=torch.tensor([1]), times=torch.tensor([0., 1.]))
    rows, _ = canonical_rows(result, 'run', 'app', 'synthetic')
    assert rows[0]['pos_error_norm'] == pytest.approx(3.)
    assert rows[0]['pos_pred_sigma'] == pytest.approx(torch.trace(report[0, 3:6, 3:6]).sqrt().item() / np.sqrt(3))


@pytest.mark.parametrize('q', [0., 1e-8, 2e-6])
def test_independent_position_drive(tmp_path, q):
    streams = write_stream(tmp_path / 'moving.csv', [.8, .2, -.3, .4])
    baseline = evaluate_interval(streams, .2, integration_covariance=0.)
    result = evaluate_interval(streams, .2, integration_covariance=q)
    torch.testing.assert_close(result['predicted_endpoints'], baseline['predicted_endpoints'], atol=0, rtol=0)
    expected = torch.zeros_like(result['native_covariance'])
    expected[:, 6:9, 6:9] = q * result['window_duration_actual'] * torch.eye(3, device=DEVICE)
    torch.testing.assert_close(result['native_covariance'] - baseline['native_covariance'], expected, atol=1e-18, rtol=1e-9)


@pytest.mark.parametrize('q', [-1., float('nan'), float('inf'), -float('inf')])
def test_invalid_integration_noise(q):
    with pytest.raises(ValueError, match='finite and nonnegative'):
        integration_covariance_value(q)
    with pytest.raises(ValueError, match='finite and nonnegative'):
        evaluate_interval(None, .2, integration_covariance=q)


def test_package_rejects_wrong_requested_noise(package):
    with pytest.raises(ValueError, match='integration covariance'):
        validate_package(*package, METHODS, integration_covariance=0.)
