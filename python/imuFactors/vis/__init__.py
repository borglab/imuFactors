# imuFactors visualization subpackage

from .trajectory_loader import (
    TrajectoryData,
    load_trajectory,
    load_nees_summary,
    discover_trajectories,
    discover_datasets,
    compute_acceleration,
    load_trajectory_from_build,
    load_best_worst_trajectories,
    discover_all_datasets,
    discover_intervals,
    DEFAULT_BUILD_DIR,
)

from .plotly_3d import (
    # Core 3-D
    plot_3d_trajectory,
    plot_3d_trajectory_from_build,
    plot_3d_trajectory_comparison,
    plot_3d_trajectory_multi_interval,
    # Time series — multi-interval (interactive HTML)
    plot_position_timeseries_multi_interval,
    plot_velocity_timeseries_multi_interval,
    plot_acceleration_timeseries_multi_interval,
    plot_orientation_timeseries_multi_interval,
    plot_displacement_timeseries_multi_interval,
    # Legacy alias
    plot_position_timeseries_from_build as plot_position_timeseries_from_build_plotly,
    # Comparison / summary
    plot_comparison,
    plot_best_worst_comparison_plotly,
    create_nees_summary_figure,
    create_nees_summary_from_build,
    # HTML export helper
    save_html,
)

from .matplotlib_visualizer import (
    # Single-trajectory plots
    plot_position_timeseries,
    plot_velocity_timeseries,
    plot_acceleration_timeseries,
    plot_orientation_timeseries,
    plot_displacement_timeseries,
    plot_3d_trajectory as plot_3d_trajectory_matplotlib,
    # Multi-interval comparison plots
    plot_position_multi_interval,
    plot_velocity_multi_interval,
    plot_acceleration_multi_interval,
    plot_orientation_multi_interval,
    plot_displacement_multi_interval,
    plot_3d_trajectory_multi_interval as plot_3d_trajectory_multi_interval_matplotlib,
)

from .noise_calibration import (
    plot_nees_comparison,
    plot_nees_comparison_matplotlib,
    plot_alpha_parameters,
    plot_nees_ratio,
    generate_noise_calibration_report,
    quick_nees_plot,
)

__all__ = [
    # ── Loader ────────────────────────────────────────────────────────
    "TrajectoryData",
    "load_trajectory",
    "load_nees_summary",
    "discover_trajectories",
    "discover_datasets",
    "compute_acceleration",
    "load_trajectory_from_build",
    "load_best_worst_trajectories",
    "discover_all_datasets",
    "discover_intervals",
    "DEFAULT_BUILD_DIR",
    # ── Plotly — 3-D ──────────────────────────────────────────────────
    "plot_3d_trajectory",
    "plot_3d_trajectory_from_build",
    "plot_3d_trajectory_comparison",
    "plot_3d_trajectory_multi_interval",
    # ── Plotly — time series (multi-interval, interactive) ────────────
    "plot_position_timeseries_multi_interval",
    "plot_velocity_timeseries_multi_interval",
    "plot_acceleration_timeseries_multi_interval",
    "plot_orientation_timeseries_multi_interval",
    "plot_displacement_timeseries_multi_interval",
    "plot_position_timeseries_from_build_plotly",
    # ── Plotly — comparison / summary ─────────────────────────────────
    "plot_comparison",
    "plot_best_worst_comparison_plotly",
    "create_nees_summary_figure",
    "create_nees_summary_from_build",
    "save_html",
    # ── Matplotlib — single trajectory ────────────────────────────────
    "plot_position_timeseries",
    "plot_velocity_timeseries",
    "plot_acceleration_timeseries",
    "plot_orientation_timeseries",
    "plot_displacement_timeseries",
    "plot_3d_trajectory_matplotlib",
    # ── Matplotlib — multi-interval ───────────────────────────────────
    "plot_position_multi_interval",
    "plot_velocity_multi_interval",
    "plot_acceleration_multi_interval",
    "plot_orientation_multi_interval",
    "plot_displacement_multi_interval",
    "plot_3d_trajectory_multi_interval_matplotlib",
    # ── Noise calibration ─────────────────────────────────────────────
    "plot_nees_comparison",
    "plot_nees_comparison_matplotlib",
    "plot_alpha_parameters",
    "plot_nees_ratio",
    "generate_noise_calibration_report",
    "quick_nees_plot",
]