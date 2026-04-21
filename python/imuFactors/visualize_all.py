#!/usr/bin/env python3
"""
Comprehensive visualization script for imuFactors trajectory and NEES analysis.
Generates interactive 3D trajectory plots, time series comparisons across preintegration times,
and noise calibration analysis with best/worst alpha parameter comparisons.

Usage:
    python generate_all_visualizations.py [--build-dir PATH] [--output-dir PATH] [--datasets DATASET1 DATASET2 ...]
"""

import os
import sys
import argparse
from pathlib import Path
from typing import Optional, List, Dict

# Add parent directories to path for imports
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))
# Import visualization functions
try:
    from imuFactors.vis import (
        plot_3d_trajectory,
        plot_position_timeseries,
        plot_velocity_timeseries,
        plot_orientation_timeseries,
        plot_frequency_spectrum,
        plot_displacement_timeseries,
        plot_position_timeseries_from_build,
        plot_3d_trajectory_from_build,
        plot_best_worst_comparison,
        plot_nees_timeseries,
        plot_all_intervals_comparison,
        plot_nees_comparison_matplotlib,
        plot_alpha_parameters,
        plot_nees_ratio,
        generate_noise_calibration_report,
    )
    from imuFactors.vis.plotly_3d import (
        plot_3d_trajectory as plot_3d_trajectory_plotly,
        plot_position_timeseries_from_build as plot_position_timeseries_plotly,
        plot_best_worst_comparison_plotly,
        plot_nees_timeseries_plotly,
        create_nees_summary_figure,
    )
    from imuFactors.vis.trajectory_loader import (
        load_trajectory,
        load_nees_summary,
        discover_all_datasets,
        discover_intervals,
        TrajectoryData,
        DEFAULT_BUILD_DIR,
    )
except ImportError as e:
    print(f"Error: Could not import imuFactors visualization modules: {e}")
    print("Make sure imuFactors is installed and PYTHONPATH is configured correctly.")
    sys.exit(1)


class TrajectoryVisualizer:
    """Main class for generating comprehensive trajectory and NEES visualizations."""
    
    def __init__(self, build_dir: str = DEFAULT_BUILD_DIR, output_dir: str = "."):
        """
        Initialize visualizer.
        
        Args:
            build_dir: Path to build directory containing CSV files
            output_dir: Directory to save output files
        """
        self.build_dir = build_dir
        self.output_dir = output_dir
        self.filter_name = "gal3"  # Default filter
        
        # Create output directory if it doesn't exist
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        
        print(f"✓ Output directory: {self.output_dir}")
    
    def discover_datasets(self) -> Dict[str, List[str]]:
        """Discover all available datasets."""
        datasets = discover_all_datasets(self.build_dir)
        print(f"\n📊 Discovered datasets:")
        for filter_name, dataset_list in datasets.items():
            print(f"  {filter_name}: {', '.join(dataset_list)}")
        return datasets
    
    def visualize_single_trajectory(
        self,
        dataset_name: str,
        interval: str = "2s",
        include_frequency: bool = True
    ) -> Dict[str, str]:
        """
        Generate all static matplotlib plots for a single trajectory.
        
        Args:
            dataset_name: Name of dataset (e.g., "MH01")
            interval: Preintegration interval (e.g., "2s", "5s", "10s")
            include_frequency: Include frequency spectrum analysis
            
        Returns:
            Dict mapping plot type to file path
        """
        print(f"\n🔄 Processing {dataset_name} ({interval})...")
        
        # Load trajectory
        traj = load_trajectory(
            os.path.join(self.build_dir, f"{self.filter_name}_trajectory_{dataset_name}_{interval}.csv")
        )
        
        if traj is None:
            print(f"  ⚠ Could not load trajectory for {dataset_name}")
            return {}
        
        results = {}
        
        # Position time series
        print(f"  ├─ Position time series...", end=" ", flush=True)
        try:
            save_path = os.path.join(
                self.output_dir, 
                f"position_timeseries_{dataset_name}_{interval}.png"
            )
            plot_position_timeseries(traj, save_path=save_path)
            results['position'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # Velocity time series
        print(f"  ├─ Velocity time series...", end=" ", flush=True)
        try:
            save_path = os.path.join(
                self.output_dir,
                f"velocity_timeseries_{dataset_name}_{interval}.png"
            )
            plot_velocity_timeseries(traj, save_path=save_path)
            results['velocity'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # Orientation (RPY) time series
        print(f"  ├─ Orientation (RPY) time series...", end=" ", flush=True)
        try:
            save_path = os.path.join(
                self.output_dir,
                f"orientation_timeseries_{dataset_name}_{interval}.png"
            )
            plot_orientation_timeseries(traj, save_path=save_path)
            results['orientation'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # Displacement (delta position)
        print(f"  ├─ Displacement time series...", end=" ", flush=True)
        try:
            save_path = os.path.join(
                self.output_dir,
                f"displacement_timeseries_{dataset_name}_{interval}.png"
            )
            plot_displacement_timeseries(traj, save_path=save_path)
            results['displacement'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # Frequency spectrum
        if include_frequency:
            print(f"  ├─ Frequency spectrum...", end=" ", flush=True)
            try:
                save_path = os.path.join(
                    self.output_dir,
                    f"frequency_spectrum_{dataset_name}_{interval}.png"
                )
                plot_frequency_spectrum(traj, save_path=save_path)
                results['frequency'] = save_path
                print("✓")
            except Exception as e:
                print(f"✗ ({e})")
        
        return results
    
    def visualize_multi_interval_comparison(
        self,
        dataset_name: str,
        intervals: Optional[List[str]] = None
    ) -> Dict[str, str]:
        """
        Generate comparison plots across multiple preintegration intervals.
        
        Args:
            dataset_name: Dataset name
            intervals: List of intervals to compare (default: ["2s", "5s", "10s"])
            
        Returns:
            Dict mapping plot type to file path
        """
        if intervals is None:
            intervals = ["2s", "5s", "10s"]
        
        print(f"\n📈 Multi-interval comparison for {dataset_name}...")
        
        results = {}
        
        # Position comparison across intervals
        print(f"  ├─ Position comparison ({', '.join(intervals)})...", end=" ", flush=True)
        try:
            save_path = os.path.join(
                self.output_dir,
                f"position_comparison_{dataset_name}.png"
            )
            plot_position_timeseries_from_build(
                dataset_name,
                self.filter_name,
                self.build_dir,
                intervals=intervals,
                save_path=save_path
            )
            results['position_multi'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # All intervals comparison
        print(f"  └─ All interval comparisons...", end=" ", flush=True)
        try:
            figs = plot_all_intervals_comparison(
                dataset_name,
                self.filter_name,
                self.build_dir,
                save_dir=self.output_dir
            )
            results['all_intervals'] = len(figs)
            print(f"✓ ({len(figs)} figures)")
        except Exception as e:
            print(f"✗ ({e})")
        
        return results
    
    def visualize_best_worst_comparison(
        self,
        dataset_name: str
    ) -> Dict[str, str]:
        """
        Generate best vs worst noise calibration comparison.
        
        Args:
            dataset_name: Dataset name
            
        Returns:
            Dict mapping plot type to file path
        """
        print(f"\n🎯 Best vs Worst comparison for {dataset_name}...", end=" ", flush=True)
        
        try:
            save_path = os.path.join(
                self.output_dir,
                f"best_worst_comparison_{dataset_name}.png"
            )
            plot_best_worst_comparison(
                dataset_name,
                self.filter_name,
                self.build_dir,
                save_path=save_path
            )
            print("✓")
            return {'best_worst': save_path}
        except Exception as e:
            print(f"✗ ({e})")
            return {}
    
    def visualize_nees_timeseries(
        self,
        dataset_name: str,
        intervals: Optional[List[str]] = None
    ) -> Dict[str, str]:
        """
        Generate NEES time series plots.
        
        Args:
            dataset_name: Dataset name
            intervals: List of intervals (default: ["2s", "5s", "10s"])
            
        Returns:
            Dict mapping plot type to file path
        """
        if intervals is None:
            intervals = ["2s", "5s", "10s"]
        
        print(f"\n📉 NEES time series for {dataset_name}...")
        
        results = {}
        
        for interval in intervals:
            print(f"  ├─ NEES timeseries ({interval})...", end=" ", flush=True)
            try:
                save_path = os.path.join(
                    self.output_dir,
                    f"nees_timeseries_{dataset_name}_{interval}.png"
                )
                plot_nees_timeseries(
                    dataset_name,
                    self.filter_name,
                    interval,
                    self.build_dir,
                    save_path=save_path
                )
                results[f'nees_{interval}'] = save_path
                print("✓")
            except Exception as e:
                print(f"✗ ({e})")
        
        return results
    
    def generate_noise_calibration_report(
        self,
        nees_csv: str = "nees_summary.csv"
    ) -> Dict[str, str]:
        """
        Generate noise calibration analysis report across all datasets.
        
        Args:
            nees_csv: Path to NEES summary CSV
            
        Returns:
            Dict mapping plot type to file path
        """
        print(f"\n🔬 Noise calibration analysis...")
        
        nees_summary = load_nees_summary(nees_csv)
        
        if nees_summary is None:
            print(f"  ⚠ Could not load NEES summary from {nees_csv}")
            return {}
        
        results = {}
        
        # NEES comparison
        print(f"  ├─ NEES comparison...", end=" ", flush=True)
        try:
            save_path = os.path.join(self.output_dir, "nees_comparison.png")
            plot_nees_comparison_matplotlib(nees_summary, save_path=save_path)
            results['nees_comparison'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # Alpha parameters
        print(f"  ├─ Alpha parameters...", end=" ", flush=True)
        try:
            save_path = os.path.join(self.output_dir, "alpha_parameters.png")
            plot_alpha_parameters(nees_summary, save_path=save_path)
            results['alpha_parameters'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # NEES ratio
        print(f"  └─ NEES ratio (worst/best)...", end=" ", flush=True)
        try:
            save_path = os.path.join(self.output_dir, "nees_ratio.png")
            plot_nees_ratio(nees_summary, save_path=save_path)
            results['nees_ratio'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        return results
    
    def generate_plotly_visualizations(
        self,
        dataset_name: str,
        intervals: Optional[List[str]] = None
    ) -> Dict[str, str]:
        """
        Generate interactive Plotly HTML visualizations.
        
        Args:
            dataset_name: Dataset name
            intervals: List of intervals to compare
            
        Returns:
            Dict mapping plot type to file path
        """
        if intervals is None:
            intervals = ["2s", "5s", "10s"]
        
        print(f"\n🌐 Interactive Plotly visualizations for {dataset_name}...")
        
        results = {}
        
        # 3D trajectory comparison
        print(f"  ├─ 3D trajectory comparison (interactive)...", end=" ", flush=True)
        try:
            fig = plot_position_timeseries_plotly(
                dataset_name,
                self.filter_name,
                self.build_dir,
                intervals=intervals,
                title=f'Position Time Series @ 200Hz - {self.filter_name.upper()} - {dataset_name}'
            )
            save_path = os.path.join(
                self.output_dir,
                f"3d_trajectory_comparison_{dataset_name}_interactive.html"
            )
            fig.write_html(save_path)
            results['3d_interactive'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # Best/worst comparison (Plotly)
        print(f"  ├─ Best vs worst comparison (interactive)...", end=" ", flush=True)
        try:
            fig = plot_best_worst_comparison_plotly(
                dataset_name,
                self.filter_name,
                self.build_dir
            )
            save_path = os.path.join(
                self.output_dir,
                f"best_worst_comparison_{dataset_name}_interactive.html"
            )
            fig.write_html(save_path)
            results['best_worst_interactive'] = save_path
            print("✓")
        except Exception as e:
            print(f"✗ ({e})")
        
        # NEES time series (Plotly) for each interval
        for interval in intervals:
            print(f"  ├─ NEES time series - {interval} (interactive)...", end=" ", flush=True)
            try:
                fig = plot_nees_timeseries_plotly(
                    dataset_name,
                    self.filter_name,
                    interval,
                    self.build_dir
                )
                save_path = os.path.join(
                    self.output_dir,
                    f"nees_timeseries_{dataset_name}_{interval}_interactive.html"
                )
                fig.write_html(save_path)
                results[f'nees_interactive_{interval}'] = save_path
                print("✓")
            except Exception as e:
                print(f"✗ ({e})")
        
        return results
    
    def generate_comprehensive_report(
        self,
        datasets: Optional[List[str]] = None,
        nees_csv: str = "nees_summary.csv"
    ) -> None:
        """
        Generate complete visualization report for all datasets.
        
        Args:
            datasets: List of datasets to visualize (auto-discover if None)
            nees_csv: Path to NEES summary CSV
        """
        print("\n" + "="*80)
        print("🚀 IMU FACTORS COMPREHENSIVE VISUALIZATION SUITE")
        print("="*80)
        
        # Discover datasets if not provided
        if datasets is None:
            discovered = self.discover_datasets()
            datasets = discovered.get(self.filter_name, [])
        
        if not datasets:
            print("⚠ No datasets found. Exiting.")
            return
        
        # Generate noise calibration report first
        print("\n" + "-"*80)
        calib_results = self.generate_noise_calibration_report(nees_csv)
        
        # Process each dataset
        for i, dataset in enumerate(datasets, 1):
            print("\n" + "-"*80)
            print(f"DATASET {i}/{len(datasets)}: {dataset}")
            print("-"*80)
            
            # Static matplotlib visualizations
            self.visualize_single_trajectory(dataset, interval="2s")
            self.visualize_multi_interval_comparison(dataset)
            self.visualize_best_worst_comparison(dataset)
            self.visualize_nees_timeseries(dataset)
            
            # Interactive Plotly visualizations
            self.generate_plotly_visualizations(dataset)
        
        # Summary
        print("\n" + "="*80)
        print("✅ REPORT GENERATION COMPLETE!")
        print("="*80)
        print(f"\n📁 Output files saved to: {self.output_dir}")
        print(f"\n📊 Generated visualizations:")
        print(f"  • Static matplotlib plots (PNG)")
        print(f"  • Interactive 3D trajectories (Plotly HTML)")
        print(f"  • NEES time series analysis")
        print(f"  • Noise calibration reports (best/worst alpha parameters)")
        print(f"  • Frequency spectrum analysis")
        print(f"  • Multi-interval comparisons across {len(datasets)} datasets")
        print("\n")


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Generate comprehensive IMU trajectory and NEES visualizations",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate visualizations for all datasets
  python generate_all_visualizations.py --build-dir ./build --output-dir ./plots
  
  # Visualize specific datasets
  python generate_all_visualizations.py --datasets MH01 V202 --output-dir ./plots
  
  # Custom NEES summary file
  python generate_all_visualizations.py --nees-csv results/nees_summary.csv
        """
    )
    
    parser.add_argument(
        "--build-dir",
        default=DEFAULT_BUILD_DIR,
        help=f"Path to build directory (default: {DEFAULT_BUILD_DIR})"
    )
    parser.add_argument(
        "--output-dir",
        default="./trajectory_visualizations",
        help="Directory to save visualizations (default: ./trajectory_visualizations)"
    )
    parser.add_argument(
        "--datasets",
        nargs="+",
        help="Specific datasets to visualize (auto-discover if not provided)"
    )
    parser.add_argument(
        "--nees-csv",
        default="nees_summary.csv",
        help="Path to NEES summary CSV (default: nees_summary.csv)"
    )
    
    args = parser.parse_args()
    
    # Create visualizer
    visualizer = TrajectoryVisualizer(args.build_dir, args.output_dir)
    
    # Generate report
    visualizer.generate_comprehensive_report(args.datasets, args.nees_csv)


if __name__ == "__main__":
    main()