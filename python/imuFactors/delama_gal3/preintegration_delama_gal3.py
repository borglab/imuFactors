from __future__ import annotations

import argparse
import csv
from glob import glob
import os
from pathlib import Path

import numpy as np
import torch

from .utils import DEVICE, baxat, bdot, bmv, pdump
from .lie_group_utils import SO3, SE3_2, G3
from .preintegration_utils import inv_Gamma_G3, Gamma_G3


torch.set_default_dtype(torch.float64)


def load_ground_truth_euroc(filename: str):
    """Load EuRoC ground truth and IMU streams from the merged csv."""
    data = torch.from_numpy(np.genfromtxt(filename, delimiter=",", skip_header=1)).to(
        DEVICE
    )

    if data.ndim != 2 or data.shape[0] < 2 or data.shape[1] != 23:
        raise ValueError(f"Expected at least two finite 23-column EuRoC rows in {filename}")

    t = (data[:, 0] - data[0, 0]).to(float).to(DEVICE)
    dt = (t[1] - t[0]).item()
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError(f"Invalid first-sample timestep in {filename}")

    quaternions = data[:, 1:5]
    norms = torch.hypot(torch.hypot(quaternions[:, 0], quaternions[:, 1]),
                        torch.hypot(quaternions[:, 2], quaternions[:, 3]))
    invalid = (~torch.isfinite(quaternions).all(dim=1) |
               ~torch.isfinite(norms) | (norms <= 1e-12))
    if invalid.any():
        row = invalid.nonzero()[0, 0].item() + 2
        raise ValueError(f"Invalid ground-truth quaternion in {filename}, row {row}")
    invalid_rows = ~torch.isfinite(data).all(dim=1)
    if invalid_rows.any():
        row = invalid_rows.nonzero()[0, 0].item() + 2
        raise ValueError(f"Non-finite EuRoC value in {filename}, row {row}")
    R_gt = SO3.from_quaternion(quaternions / norms[:, None]).to(float).to(DEVICE)
    V_gt = data[:, 5:8].to(float).to(DEVICE)
    P_gt = data[:, 8:11].to(float).to(DEVICE)

    bias_gyro = data[:, 11:14].to(float).to(DEVICE)
    bias_acc = data[:, 14:17].to(float).to(DEVICE)

    gyro_imu_gt = data[:, 17:20].to(float).to(DEVICE)
    acc_imu_gt = data[:, 20:23].to(float).to(DEVICE)

    poses_gt = torch.eye(5, device=DEVICE).repeat(len(R_gt), 1, 1)
    poses_gt[:, :3, :3] = R_gt
    poses_gt[:, :3, 3] = V_gt
    poses_gt[:, :3, 4] = P_gt

    return poses_gt, bias_gyro, bias_acc, gyro_imu_gt, acc_imu_gt, t, dt


def compound_delama_gal3(T0, gamma0, Sigma0, w_k, dt, Q_g3):
    """One IMU propagation step for Gal(3) x gal(3) (method 6)."""
    AdT0 = G3.Ad(T0)
    T0_inv = G3.inv(T0)

    w_hat = w_k + bmv(G3.Ad(T0_inv), gamma0)
    w0 = bmv(AdT0, w_k) + gamma0

    T = T0.bmm(G3.exp(w_hat * dt))
    gamma = bmv(G3.Ad(T.bmm(T0_inv)), gamma0)

    Qd = Q_g3.repeat(T0.shape[0], 1, 1)

    A = torch.eye(20, device=DEVICE).repeat(T0.shape[0], 1, 1)
    A[:, :10, 10:20] = G3.left_jacobian(w0 * dt) * dt
    A[:, 10:20, 10:20] = G3.Ad(G3.exp(w0 * dt))

    B = torch.zeros(20, 20, device=DEVICE).repeat(T0.shape[0], 1, 1)
    B[:, :10, :10] = -AdT0.bmm(G3.left_jacobian(w_hat * dt)) * dt
    B[:, 10:20, 10:20] = -G3.Ad(T) * dt

    Sigma = baxat(A, Sigma0) + baxat(B, Qd)
    Sigma = 0.5 * (Sigma + torch.transpose(Sigma, 1, 2))

    return T, gamma, Sigma


def compute_ext_pose_nees(P_pose, xi_pose):
    """Extended-pose NEES with dof=9."""
    dof = 9
    jitter = 1e-12 * torch.eye(dof, device=DEVICE).repeat(P_pose.shape[0], 1, 1)
    solved = torch.linalg.solve(P_pose + jitter, xi_pose.unsqueeze(2)).squeeze(2)
    return bdot(xi_pose, solved) / dof


def rmse_from_error_block(err_block):
    """RMSE over batched 3D error vectors."""
    return torch.sqrt(torch.mean(torch.sum(err_block**2, dim=1)))


def list_euroc_csv_files(dataset_glob: str):
    files = sorted(glob(dataset_glob))
    if len(files) == 0:
        raise FileNotFoundError(f"No files found matching {dataset_glob}")
    return files


def write_summary_csv(output_path: Path, headers: list[str], rows: list[tuple]):
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(headers)
        writer.writerows(rows)


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parents[3]
    default_dataset_glob = str(repo_root / "data/euroc/euroc_*.csv")
    default_output_dir = str(repo_root / "build/results/delama_gal3")

    parser = argparse.ArgumentParser(
        description=(
            "Delama Gal3 preintegration evaluation over EuRoC CSV files."
        )
    )
    parser.add_argument(
        "--dataset-glob",
        type=str,
        default=default_dataset_glob,
        help="Glob pattern for merged EuRoC CSV files.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=default_output_dir,
        help="Directory where per-sequence pickles and summaries are written.",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=8.4,
        help="Noise scale factor applied to baseline coefficients.",
    )
    parser.add_argument(
        "--preint-times",
        type=float,
        nargs="+",
        default=[0.2, 0.5, 1.0],
        help="Preintegration window sizes in seconds.",
    )
    parser.add_argument("--integration-covariance", type=integration_covariance_value, default=1e-8,
                        help="Continuous position-drive covariance in m²/s, independent of alpha.")
    return parser


def window_bounds(sample_count: int, interval: float, dt: float):
    """C++ llround for positive intervals, with shared endpoint and next start."""
    if not np.isfinite(interval) or not np.isfinite(dt) or interval <= 0 or dt <= 0:
        raise ValueError("Interval and timestep must be finite and positive")
    steps = max(1, int(np.floor(interval / dt + 0.5)))
    starts = torch.arange(0, max(0, sample_count - steps), steps, device=DEVICE)
    return steps, starts, starts + steps


def integration_covariance_value(value):
    """Validate the independent continuous position-drive covariance (m²/s)."""
    value = float(value)
    if not np.isfinite(value) or value < 0:
        raise ValueError("Integration covariance must be finite and nonnegative")
    return value


def configuration_label(alpha, integration_covariance):
    return f"alpha_g{alpha:g}_a{alpha:g}_endpoint_v2_intcov{integration_covariance:.17g}"


def physical_reporting(predicted, truth, increment, native_covariance):
    """Physical R/P/V errors and covariance transported from native right errors."""
    rotation = predicted[:, :3, :3]
    error = torch.cat((SO3.log(rotation.transpose(1, 2).bmm(truth[:, :3, :3])),
                       truth[:, :3, 4] - predicted[:, :3, 4],
                       truth[:, :3, 3] - predicted[:, :3, 3]), dim=1)
    adjoint = G3.Ad(G3.inv(increment))[:, :9, :9]
    order = [0, 1, 2, 6, 7, 8, 3, 4, 5]
    transport = adjoint[:, order, :]
    world = torch.eye(9, device=rotation.device).repeat(len(rotation), 1, 1)
    world[:, 3:6, 3:6] = rotation
    world[:, 6:9, 6:9] = rotation
    transport = world.bmm(transport)
    return error, baxat(transport, native_covariance)


@torch.no_grad()
def evaluate_interval(streams, interval: float, alpha: float = 8.4,
                      integration_covariance: float = 1e-8):
    """Evaluate all complete windows with initial GT bias and zero covariance."""
    if not np.isfinite(alpha) or alpha <= 0:
        raise ValueError("Noise scale alpha must be finite and positive")
    integration_covariance = integration_covariance_value(integration_covariance)
    T, b_omega, b_acc, omegas, accs, time_vec, dt = streams
    steps_per_window, starts, ends = window_bounds(len(T), interval, dt)
    M = len(starts)
    if M == 0:
        raise ValueError(f"No complete windows for interval {interval}")
    indices = starts[:, None] + torch.arange(steps_per_window, device=DEVICE)
    w = torch.cat((omegas[indices] - b_omega[starts, None],
                   accs[indices] - b_acc[starts, None],
                   torch.zeros(M, steps_per_window, 3, device=DEVICE),
                   torch.ones(M, steps_per_window, 1, device=DEVICE)), dim=2)
    g = torch.tensor([0, 0, -9.81], device=DEVICE)
    Q_g3 = torch.zeros(20, 20, device=DEVICE)
    Q_g3[:3, :3] = (alpha * 1.6968e-4)**2 / dt * torch.eye(3, device=DEVICE)
    Q_g3[3:6, 3:6] = (alpha * 2.0000e-3)**2 / dt * torch.eye(3, device=DEVICE)
    # Initialize with zero covariance while using each window's
    # ground-truth initial bias for measurement debiasing above.
    Y_delama_gal3 = torch.eye(5, device=DEVICE).repeat(M, 1, 1)
    gamma_delama_gal3 = torch.zeros(M, 10, device=DEVICE)
    covariance_delama_gal3 = torch.zeros(M, 20, 20, device=DEVICE)

    for k in range(steps_per_window):
        Y_delama_gal3, gamma_delama_gal3, covariance_delama_gal3 = compound_delama_gal3(
            Y_delama_gal3, gamma_delama_gal3, covariance_delama_gal3, w[:, k], dt, Q_g3
        )
        # Independent position drive, not held gyro/accelerometer input noise.
        covariance_delama_gal3[:, 6:9, 6:9] += (
            integration_covariance * dt * torch.eye(3, device=DEVICE))

    t_end = steps_per_window * dt
    Y_true_end = (
        SE3_2.inv(T[starts])
        .bmm(inv_Gamma_G3(g, t_end).repeat(M, 1, 1))
        .bmm(T[ends])
    )

    xi_Y_delama_gal3 = G3.log(Y_true_end.bmm(G3.inv(Y_delama_gal3)))

    # The native error and covariance share the (rotation, velocity, position) order.
    nees_endpoint = compute_ext_pose_nees(
        covariance_delama_gal3[:, :9, :9], xi_Y_delama_gal3[:, :9]
    )

    rot_rmse_deg = torch.rad2deg(rmse_from_error_block(xi_Y_delama_gal3[:, :3]))
    pos_rmse = rmse_from_error_block(xi_Y_delama_gal3[:, 6:9])
    vel_rmse = rmse_from_error_block(xi_Y_delama_gal3[:, 3:6])

    predicted = Gamma_G3(g, t_end).repeat(M, 1, 1).bmm(T[starts]).bmm(Y_delama_gal3)
    physical_error, reporting_covariance = physical_reporting(
        predicted, T[ends], Y_delama_gal3, covariance_delama_gal3[:, :9, :9])
    result = {
        "convention": "endpoint_v2",
        "integration_covariance": integration_covariance,
        "config_label": configuration_label(alpha, integration_covariance),
        "predicted_endpoints": predicted,
        "predicted_increment": Y_delama_gal3,
        "physical_error": physical_error,
        "reporting_covariance": reporting_covariance,
        "alpha": alpha,
        "preint_time": interval,
        "M": M,
        "starts": starts,
        "ends": ends,
        "times": time_vec,
        "native_error": xi_Y_delama_gal3[:, :9],
        "native_covariance": covariance_delama_gal3[:, :9, :9],
        "steps_per_window": steps_per_window,
        "window_duration_actual": t_end,
        "endpoint_nees": nees_endpoint,
        "anees_mean": nees_endpoint.mean().item(),
        "anees_median": float(np.median(nees_endpoint.cpu().numpy())),
        "anees_var": nees_endpoint.var(correction=0).item(),
        "rmse_rotation_deg": rot_rmse_deg.item(),
        "rmse_position_m": pos_rmse.item(),
        "rmse_velocity_mps": vel_rmse.item(),
    }
    return result


def main() -> None:
    args = build_parser().parse_args()

    alpha = float(args.alpha)
    sigma_gyro_base = alpha * 1.6968e-4
    sigma_acc_base = alpha * 2.0000e-3
    g = torch.tensor([0, 0, -9.81], device=DEVICE)
    preint_times = torch.tensor(args.preint_times, device=DEVICE)
    output_dir = Path(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    dataset_files = list_euroc_csv_files(args.dataset_glob)
    print("Running", len(dataset_files), "files:")
    for dataset_file in dataset_files:
        print(" -", dataset_file)

    all_results = {}
    nees_summary_rows = []
    rmse_summary_rows = []

    for filename in dataset_files:
        sequence_name = os.path.splitext(os.path.basename(filename))[0]
        sequence_short = sequence_name.replace("euroc_", "")
        print(
            "=============================================================================="
        )
        print("Sequence:", sequence_name)

        T, b_omega, b_acc, omegas, accs, time_vec, dt = load_ground_truth_euroc(filename)

        sigma_gyro = sigma_gyro_base / np.sqrt(dt)
        sigma_acc = sigma_acc_base / np.sqrt(dt)

        imu_freq = 1 / dt
        duration = time_vec[-1].item()

        params = {
            "filename": filename,
            "integration_covariance": args.integration_covariance,
            "convention": "endpoint_v2",
            "config_label": configuration_label(alpha, args.integration_covariance),
            "duration": duration,
            "imu_freq": imu_freq,
            "dt": dt,
            "sigma_gyro": sigma_gyro,
            "sigma_acc": sigma_acc,
            "g": g,
            "preint_times": preint_times,
            "notes": (
                "Delama Gal3 ext-pose NEES with ground-truth initial bias "
                "and zero initial covariance."
            ),
        }
        pdump(params, output_dir, sequence_name + "_params_delama_gal3.p")

        per_sequence_results = {}

        for preint_time in preint_times:
            if len(window_bounds(len(T), preint_time.item(), dt)[1]) == 0:
                print("Skipping", preint_time.item(), "s (window too large for sequence)")
                continue
            result = evaluate_interval(
                (T, b_omega, b_acc, omegas, accs, time_vec, dt),
                preint_time.item(), alpha, args.integration_covariance)
            per_sequence_results[preint_time.item()] = result

            window_label = f"{preint_time.item():.1f}s"
            nees_summary_rows.append(
                (
                    sequence_short,
                    window_label,
                    result["anees_median"],
                    result["anees_mean"],
                    result["anees_var"],
                )
            )
            rmse_summary_rows.append(
                (
                    sequence_short,
                    window_label,
                    result["rmse_rotation_deg"],
                    result["rmse_position_m"],
                    result["rmse_velocity_mps"],
                )
            )

            out_name = sequence_name + "_delama_gal3_dt_" + str(preint_time.item()) + ".p"
            pdump(result, output_dir, out_name)

        all_results[sequence_name] = per_sequence_results

    print("==============================================================================")
    print("NEES Summary")
    print("Sequence | Window | Median NEES")
    for seq, window, median_nees, _, _ in nees_summary_rows:
        print(f"{seq} | {window} | Median NEES: {median_nees:.3f}")

    print("==============================================================================")
    print("RMSE Summary")
    print("Sequence | Window | RMSE rot [deg] | RMSE pos [m] | RMSE vel [m/s]")
    for seq, window, rmse_rot, rmse_pos, rmse_vel in rmse_summary_rows:
        print(
            f"{seq} | {window} | RMSE rot [deg]: {rmse_rot:.6f} | "
            f"RMSE pos [m]: {rmse_pos:.6f} | RMSE vel [m/s]: {rmse_vel:.6f}"
        )

    pdump(all_results, output_dir, "delama_gal3_all_files.p")
    write_summary_csv(
        output_dir / "nees_summary.csv",
        ["sequence", "window", "median_nees", "mean_nees", "variance_nees"],
        nees_summary_rows,
    )
    write_summary_csv(
        output_dir / "rmse_summary.csv",
        [
            "sequence",
            "window",
            "rmse_rotation_deg",
            "rmse_position_m",
            "rmse_velocity_mps",
        ],
        rmse_summary_rows,
    )


if __name__ == "__main__":
    main()
