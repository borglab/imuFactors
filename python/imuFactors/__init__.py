# imuFactors Python Visualization Package
# Author: Alec Kain
# License: See LICENSE in repository root

"""imuFactors visualization utilities for EKF trajectory analysis."""

from importlib import import_module

# Keep numerical evaluators independent of plotting imports and font discovery.
_EXPORT_MODULES = {
    **dict.fromkeys(("load_trajectory", "load_nees_summary", "TrajectoryData", "DEFAULT_BUILD_DIR"), "trajectory_loader"),
    **dict.fromkeys(("plot_3d_trajectory", "plot_comparison"), "plotly_3d"),
    **dict.fromkeys((
        "plot_position_timeseries", "plot_velocity_timeseries", "plot_acceleration_timeseries",
        "plot_orientation_timeseries", "plot_displacement_timeseries", "plot_position_multi_interval",
        "plot_velocity_multi_interval", "plot_acceleration_multi_interval", "plot_orientation_multi_interval",
        "plot_displacement_multi_interval", "plot_3d_trajectory_multi_interval",
    ), "matplotlib_visualizer"),
    **dict.fromkeys(("plot_nees_comparison", "discover_datasets"), "noise_calibration"),
}


def __getattr__(name):
    """Load existing public visualization exports only when requested."""
    if name not in _EXPORT_MODULES:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(f".vis.{_EXPORT_MODULES[name]}", __name__), name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))


__version__ = "1.0.0"

__all__ = [
    # Loader
    "load_trajectory",
    "load_nees_summary",
    "TrajectoryData",
    "DEFAULT_BUILD_DIR",
    # Plotly
    "plot_3d_trajectory",
    "plot_comparison",
    # Matplotlib — single trajectory
    "plot_position_timeseries",
    "plot_velocity_timeseries",
    "plot_acceleration_timeseries",
    "plot_orientation_timeseries",
    "plot_displacement_timeseries",
    # Matplotlib — multi-interval comparisons
    "plot_position_multi_interval",
    "plot_velocity_multi_interval",
    "plot_acceleration_multi_interval",
    "plot_orientation_multi_interval",
    "plot_displacement_multi_interval",
    "plot_3d_trajectory_multi_interval",
    # Noise calibration
    "plot_nees_comparison",
    "discover_datasets",
]