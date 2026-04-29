# imuFactors Python Visualization Package
# Author: Alec Kain
# License: See LICENSE in repository root

"""imuFactors visualization utilities for EKF trajectory analysis."""

from imuFactors.vis.trajectory_loader import (
    load_trajectory,
    load_nees_summary,
    TrajectoryData,
    DEFAULT_BUILD_DIR,
)
from imuFactors.vis.plotly_3d import plot_3d_trajectory, plot_comparison
from imuFactors.vis.matplotlib_visualizer import (
    plot_position_timeseries,
    plot_velocity_timeseries,
    plot_acceleration_timeseries,
    plot_orientation_timeseries,
    plot_displacement_timeseries,
    plot_position_multi_interval,
    plot_velocity_multi_interval,
    plot_acceleration_multi_interval,
    plot_orientation_multi_interval,
    plot_displacement_multi_interval,
    plot_3d_trajectory_multi_interval,
)
from imuFactors.vis.noise_calibration import plot_nees_comparison, discover_datasets

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