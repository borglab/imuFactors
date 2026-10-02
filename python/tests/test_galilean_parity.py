"""Reproducible full-precision parity; build exportGalileanParity before running."""
import io
from pathlib import Path
import subprocess

import numpy as np
import pytest
import torch

from imuFactors.delama_gal3.preintegration_delama_gal3 import evaluate_interval, load_ground_truth_euroc

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('source', sorted((ROOT / 'data/euroc').glob('euroc_*.csv')), ids=lambda p: p.stem)
def test_full_precision_parity(source):
    check_parity(source, 1e-8)


@pytest.mark.parametrize('q', [0., 2e-6])
def test_configurable_noise_parity(q):
    check_parity(ROOT / 'data/euroc/euroc_MH01.csv', q)


@pytest.mark.parametrize('name', ['MH01', 'V101'])
@pytest.mark.parametrize('q', [0., 2e-6])
def test_separate_sensor_scales_parity(name, q):
    check_parity(ROOT / f'data/euroc/euroc_{name}.csv', q, 3.7, 12.1)


def check_parity(source, q, alpha_gyro=8.4, alpha_acc=8.4):
    helper = ROOT / 'build/tests/exportGalileanParity'
    assert helper.is_file(), 'Build exportGalileanParity before running parity tests'
    output = subprocess.run([str(helper), str(source), repr(q), 'galilean', repr(alpha_gyro), repr(alpha_acc)],
                            check=True, capture_output=True, text=True).stdout
    data = np.genfromtxt(io.StringIO(output), delimiter=',', names=True)
    streams = load_ground_truth_euroc(str(source))
    torch.set_num_threads(1)
    for interval in (.2, .5, 1.):
        rows = data[np.isclose(data['interval'], interval)]
        result = evaluate_interval(streams, interval, integration_covariance=q,
                                   alpha_gyro=alpha_gyro, alpha_acc=alpha_acc)
        predicted = result['predicted_endpoints'].cpu().numpy()
        cpp_predicted = np.column_stack([rows[f'pred_{i}'] for i in range(15)])
        np.testing.assert_array_equal(rows['start'], result['starts'].cpu())
        np.testing.assert_array_equal(rows['end'], result['ends'].cpu())
        np.testing.assert_allclose(predicted[:, :3, :3].reshape(-1, 9), cpp_predicted[:, :9], atol=1e-12, rtol=0)
        np.testing.assert_allclose(predicted[:, :3, 4], cpp_predicted[:, 9:12], atol=1e-8, rtol=0)
        np.testing.assert_allclose(predicted[:, :3, 3], cpp_predicted[:, 12:15], atol=1e-9, rtol=0)
        covariance = np.stack([rows[f'cov_{i}_{j}'] for i in range(9) for j in range(9)], axis=1).reshape(-1, 9, 9)
        world = np.broadcast_to(np.eye(9), covariance.shape).copy()
        rotation = cpp_predicted[:, :9].reshape(-1, 3, 3)
        world[:, 3:6, 3:6] = rotation
        world[:, 6:9, 6:9] = rotation
        reporting = world @ covariance @ world.transpose(0, 2, 1)
        np.testing.assert_allclose(result['reporting_covariance'].cpu(), reporting, atol=5e-11, rtol=1e-5)
