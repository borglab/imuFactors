"""Scalar quadrature experiment utilities."""

from imuFactors.quadrature.scalar import (
    DEFAULT_INTERVAL,
    METRIC_COLUMNS,
    ScalarFunction,
    ScalarMonteCarloResult,
    chebyshev_trial_curves,
    plot_fixed_node_comparison,
    plot_fixed_sample_comparison,
    plot_function_noise_comparison,
    run_scalar_monte_carlo,
    scalar_function_from_chebyshev2_nodes,
    trapezoid_integral_curve,
)

__all__ = [
    "DEFAULT_INTERVAL",
    "METRIC_COLUMNS",
    "ScalarFunction",
    "ScalarMonteCarloResult",
    "chebyshev_trial_curves",
    "plot_fixed_node_comparison",
    "plot_fixed_sample_comparison",
    "plot_function_noise_comparison",
    "run_scalar_monte_carlo",
    "scalar_function_from_chebyshev2_nodes",
    "trapezoid_integral_curve",
]
