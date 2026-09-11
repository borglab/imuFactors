"""Numerical contracts shared by C++ and the unified Delama export."""
import csv
import re
from pathlib import Path

import numpy as np
import pytest
import torch

from imuFactors.delama_gal3.canonical import (
    WINDOW_FIELDS, SUMMARY_FIELDS, canonical_blocks, canonical_rows, summarize_rows,
)
from imuFactors.delama_gal3.preintegration_delama_gal3 import (
    configuration_label, compute_ext_pose_nees, evaluate_interval, load_ground_truth_euroc, window_bounds,
)
from imuFactors.delama_gal3.utils import DEVICE
from run_unified_imu_comparison import (
    CANONICAL_FILES, CPP_METHODS, INTERVALS, METHODS, read_csv, run_comparison, validate_package,
)


def test_window_boundaries():
    steps, starts, ends = window_bounds(402, 1., .005)
    assert steps == 200
    assert starts.tolist() == [0, 200]
    assert ends.tolist() == [200, 400]
    assert window_bounds(400, 1., .005)[1].tolist() == [0]
    assert window_bounds(401, 1., .005)[1].tolist() == [0, 200]
    assert window_bounds(200, 1., .005)[1].tolist() == []
    assert window_bounds(20, .025, .01)[0] == 3  # C++ rounds ties away from zero.
    with pytest.raises(ValueError, match="positive"):
        window_bounds(10, 1., 0.)


def test_block_conversion_and_nees():
    error = torch.arange(1., 10., device=DEVICE).unsqueeze(0)
    matrix = torch.arange(81., device=DEVICE).reshape(9, 9) / 30
    covariance = (matrix @ matrix.T + torch.diag(torch.arange(1., 10., device=DEVICE))).unsqueeze(0)
    converted, aligned = canonical_blocks(error, covariance)
    assert converted.tolist() == [[1, 2, 3, 7, 8, 9, 4, 5, 6]]
    assert aligned[0, 3, 6] == covariance[0, 6, 3]
    torch.testing.assert_close(compute_ext_pose_nees(aligned, converted), compute_ext_pose_nees(covariance, error))
    zeros = torch.zeros(1, 9, 9, device=DEVICE)
    torch.testing.assert_close(compute_ext_pose_nees(zeros, torch.ones(1, 9, device=DEVICE)),
                               torch.tensor([1e12], device=DEVICE))


def test_evaluator_initial_bias_and_canonical_shape(tmp_path):
    # Constant state: specific force cancels gravity after initial bias removal.
    count = 9
    data = np.zeros((count, 23))
    data[:, 0] = 100. + np.arange(count) * .005
    data[-1, 0] += .001  # Loader must use the first dt, not average dt.
    data[:, 1] = 1.
    data[:, 11:14] = [.1, .2, .3]
    data[:, 14:17] = [.4, .5, .6]
    data[:, 17:20] = data[:, 11:14]
    data[:, 20:23] = data[:, 14:17] + [0, 0, 9.81]
    source = tmp_path / "euroc_fake.csv"
    np.savetxt(source, data, delimiter=",", header=",".join(str(i) for i in range(23)), comments="")
    streams = load_ground_truth_euroc(str(source))
    assert streams[-1] == data[1, 0] - data[0, 0]
    result = evaluate_interval(streams, .02, integration_covariance=0)
    assert result["starts"].tolist() == [0, 4]
    assert result["ends"].tolist() == [4, 8]
    assert result["native_error"].abs().max() < 1e-10
    assert torch.isfinite(result["native_covariance"]).all()
    smaller_noise = evaluate_interval(streams, .02, alpha=4.2, integration_covariance=0)
    torch.testing.assert_close(result["native_covariance"], 4 * smaller_noise["native_covariance"])
    smaller_rows, _ = canonical_rows(smaller_noise, "run", "app", "fake")
    assert smaller_rows[0]["config_label"] == configuration_label(4.2, 0)
    # Distinct blocks make accidental position/velocity exchange visible in rows.
    result["native_error"][:] = torch.tensor([1, 0, 0, 2, 0, 0, 3, 0, 0], device=DEVICE)
    result["native_covariance"][:] = torch.diag(torch.tensor([1., 1., 1., 4., 4., 4., 9., 9., 9.], device=DEVICE))
    result["physical_error"][:] = torch.tensor([1, 0, 0, 3, 0, 0, 2, 0, 0], device=DEVICE)
    result["reporting_covariance"][:] = torch.diag(torch.tensor([1., 1., 1., 9., 9., 9., 4., 4., 4.], device=DEVICE))
    rows, summary = canonical_rows(result, "run", "app", "fake")
    assert list(rows[0]) == WINDOW_FIELDS
    assert list(summary) == SUMMARY_FIELDS
    assert rows[0]["method"] == "delama_gal3_python"
    assert rows[0]["pos_error_norm"] == rows[0]["pos_pred_sigma"] == 3
    assert rows[0]["vel_error_norm"] == rows[0]["vel_pred_sigma"] == 2
    assert rows[0]["normalized_nees"] == pytest.approx(1 / 3)
    values = [1., 2., 3., 4.]
    summary = summarize_rows([{**rows[0], "normalized_nees": value} for value in values])
    assert summary["normalized_nees_mean"] == 2.5
    assert summary["normalized_nees_median"] == 2.5
    assert summary["normalized_nees_variance"] == 1.25
    assert summary["normalized_nees_p95"] == 3.
    assert summarize_rows([rows[0]])["normalized_nees_variance"] == 0


def test_headers_match_cpp():
    source = (Path(__file__).resolve().parents[2] / "src/ResultsSchema.h").read_text()
    for function, fields in (("windowMetricHeader", WINDOW_FIELDS), ("windowSummaryHeader", SUMMARY_FIELDS)):
        body = source.split(f"inline std::string {function}() {{", 1)[1].split("}", 1)[0]
        assert "".join(re.findall(r'"([^"]*)"', body)).split(",") == fields


def write_csv(path, fields, rows=()):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


@pytest.fixture
def package(tmp_path):
    source = tmp_path / "euroc_MH01.csv"
    write_csv(source, ["time"], [{"time": index * .005} for index in range(201)])
    folder = tmp_path / "package"
    folder.mkdir()
    identity = {"run_id": "run", "app_name": "app"}
    for name in CANONICAL_FILES:
        write_csv(folder / name, identity)
    write_csv(folder / "run_metadata.csv", identity, [identity])
    member = {**identity, "dataset": "MH01", "source_path": str(source), "dataset_group": "MH01"}
    write_csv(folder / "datasets.csv", member, [member])
    rows, summaries = [], []
    for method in METHODS:
        for interval in INTERVALS:
            steps = round(interval / .005)
            group = []
            for index, start in enumerate(range(0, 201 - steps, steps)):
                row = dict.fromkeys(WINDOW_FIELDS, 0)
                row.update(**identity, dataset="MH01", method=method, config_label=configuration_label(8.4, 1e-8),
                           interval_seconds=interval, samples_per_window=steps, window_index=index,
                           window_start_sample=start, window_end_sample=start+steps,
                           window_start_time=start*.005, window_end_time=(start+steps)*.005)
                group.append(row)
            rows.extend(group)
            summaries.append(summarize_rows(group))
    write_csv(folder / "window_metrics.csv", WINDOW_FIELDS, rows)
    write_csv(folder / "window_summaries.csv", SUMMARY_FIELDS, summaries)
    return folder, {"MH01": source}


def test_package_coverage_and_boundaries(package):
    folder, datasets = package
    validate_package(folder, datasets, METHODS)
    fields, rows = read_csv(folder / "window_metrics.csv")
    rows[0]["window_end_sample"] = 41
    write_csv(folder / "window_metrics.csv", fields, rows)
    with pytest.raises(ValueError, match="boundary mismatch"):
        validate_package(folder, datasets, METHODS)
    write_csv(folder / "window_metrics.csv", fields, rows[1:])
    with pytest.raises(ValueError, match="count mismatch"):
        validate_package(folder, datasets, METHODS)


def test_package_rejects_missing_methods_and_files(package):
    folder, datasets = package
    with pytest.raises(ValueError, match="coverage"):
        validate_package(folder, datasets, CPP_METHODS)
    (folder / "window_summaries.csv").unlink()
    with pytest.raises(ValueError, match="Missing canonical file"):
        validate_package(folder, datasets, METHODS)


def test_missing_binary_does_not_publish(tmp_path):
    with pytest.raises(FileNotFoundError, match=r"C\+\+ binary"):
        run_comparison(tmp_path / "missing", tmp_path, tmp_path / "results")
    assert not (tmp_path / "results").exists()


def test_viewer_order():
    from python.viewer.app import _method_sort_key
    from python.viewer.nees_app import _method_sort_key as nees_sort_key
    for key in (_method_sort_key, nees_sort_key):
        assert sorted(reversed(METHODS), key=key) == list(METHODS)
        assert key("tangent") > key("delama_gal3_python")


def test_visualization_exports_remain_available_lazily(monkeypatch):
    import imuFactors
    from types import SimpleNamespace

    assert set(imuFactors.__all__) == set(imuFactors._EXPORT_MODULES)
    calls = []
    sentinel = object()

    def import_plotting_module(name, package):
        calls.append((name, package))
        return SimpleNamespace(plot_comparison=sentinel)

    monkeypatch.setattr(imuFactors, "import_module", import_plotting_module)
    try:
        assert imuFactors.__getattr__("plot_comparison") is sentinel
        assert calls == [(".vis.plotly_3d", "imuFactors")]
    finally:
        imuFactors.__dict__.pop("plot_comparison", None)
    with pytest.raises(AttributeError):
        imuFactors.__getattr__("unknown_export")
