#!/usr/bin/env python3
"""
Complete Visualization Suite for imuFactors Trajectory Analysis

Generates comprehensive visualizations for every discovered dataset:
  - Static PNGs  (matplotlib)  → <output_dir>/<dataset>/
  - Interactive HTML (Plotly)  → <output_dir>/<dataset>/html/

The HTML figures support per-trace toggling via the legend so you can
selectively show/hide individual preintegration intervals or ground truth.

Usage:
    python generate_visualizations.py --build-dir ./build --output-dir ./visualizations
    python generate_visualizations.py --datasets MH01 V202 --build-dir ./build
    python generate_visualizations.py --no-png   # HTML only
    python generate_visualizations.py --no-html  # PNG only
"""

import os
import sys
import argparse
from pathlib import Path
from typing import Optional, List, Dict
import json
import traceback

sys.path.insert(0, os.path.dirname(__file__))

from imuFactors.vis.trajectory_loader import (
    load_trajectory_from_build,
    discover_all_datasets,
    discover_intervals,
    DEFAULT_BUILD_DIR,
)

from imuFactors.vis.matplotlib_visualizer import (
    plot_position_multi_interval,
    plot_velocity_multi_interval,
    plot_acceleration_multi_interval,
    plot_orientation_multi_interval,
    plot_displacement_multi_interval,
    plot_3d_trajectory_multi_interval,
)

from imuFactors.vis.plotly_3d import (
    plot_3d_trajectory_multi_interval       as plotly_3d,
    plot_position_timeseries_multi_interval  as plotly_position,
    plot_velocity_timeseries_multi_interval  as plotly_velocity,
    plot_acceleration_timeseries_multi_interval as plotly_acceleration,
    plot_orientation_timeseries_multi_interval as plotly_orientation,
    plot_displacement_timeseries_multi_interval as plotly_displacement,
    save_html,
)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Visualisation catalogue
# ---------------------------------------------------------------------------

# Each entry: (short_name, matplotlib_func, plotly_func)
_VIS_CATALOGUE = [
    ("3d_trajectory",       plot_3d_trajectory_multi_interval,    plotly_3d),
    ("position",            plot_position_multi_interval,          plotly_position),
    ("velocity",            plot_velocity_multi_interval,          plotly_velocity),
    ("acceleration",        plot_acceleration_multi_interval,      plotly_acceleration),
    ("orientation",         plot_orientation_multi_interval,       plotly_orientation),
    ("displacement",        plot_displacement_multi_interval,      plotly_displacement),
]


# ---------------------------------------------------------------------------
# Suite
# ---------------------------------------------------------------------------

class VisualizationSuite:
    """Generate comprehensive trajectory visualizations (PNG + interactive HTML)."""

    def __init__(
        self,
        build_dir: str = DEFAULT_BUILD_DIR,
        output_dir: str = "visualizations",
        generate_png: bool = True,
        generate_html: bool = True,
    ):
        self.build_dir     = build_dir
        self.output_dir    = Path(output_dir)
        self.generate_png  = generate_png
        self.generate_html = generate_html
        self.filter_name   = "gal3"

        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.results: Dict = {
            "datasets": {},
            "errors":   [],
            "summary":  {"total": 0, "successful": 0, "failed": 0},
        }

        print("\n" + "=" * 80)
        print("🚀  IMU FACTORS TRAJECTORY VISUALIZATION SUITE")
        print("=" * 80)
        print(f"  Build dir : {self.build_dir}")
        print(f"  Output dir: {self.output_dir}")
        print(f"  PNG        : {'yes' if generate_png  else 'no'}")
        print(f"  HTML       : {'yes' if generate_html else 'no'}")
        print()

    # ------------------------------------------------------------------
    def discover_datasets(self) -> Dict[str, List[str]]:
        print("📊 Discovering datasets …")
        datasets = discover_all_datasets(self.build_dir)
        for fn, ds_list in datasets.items():
            if ds_list:
                print(f"   ✓ {fn}: {', '.join(ds_list)}")
        return datasets

    # ------------------------------------------------------------------
    def visualize_dataset(
        self,
        dataset_name: str,
        intervals: Optional[List[str]] = None,
    ) -> Dict[str, Dict[str, str]]:
        """
        Generate all PNG and/or HTML visualizations for one dataset.

        Returns:
            Dict  { vis_name: { "png": path, "html": path } }
        """
        available = discover_intervals(dataset_name, self.filter_name, self.build_dir)
        if not available:
            print(f"  ⚠  No interval files found for {dataset_name}")
            return {}

        ivs = intervals if intervals is not None else available

        # Output directories
        ds_dir      = self.output_dir / dataset_name
        html_dir    = ds_dir / "html"
        ds_dir.mkdir(exist_ok=True)
        if self.generate_html:
            html_dir.mkdir(exist_ok=True)

        print(f"\n{'─' * 80}")
        print(f"📈 Visualizing: {dataset_name}  (intervals: {', '.join(ivs)})")
        print(f"{'─' * 80}")

        results: Dict[str, Dict[str, str]] = {}

        for short_name, mpl_func, plotly_func in _VIS_CATALOGUE:
            entry: Dict[str, str] = {}

            # ── PNG (matplotlib) ─────────────────────────────────────
            if self.generate_png:
                png_path = str(ds_dir / f"{short_name}.png")
                label    = short_name.replace("_", " ").title()
                print(f"  ├─ [PNG ] {label} …", end=" ", flush=True)
                try:
                    fig = mpl_func(
                        dataset_name,
                        filter_name=self.filter_name,
                        build_dir=self.build_dir,
                        intervals=ivs,
                        save_path=png_path,
                    )
                    plt.close(fig)
                    entry["png"] = png_path
                    print("✓")
                except Exception as exc:
                    print("✗")
                    self._record_error(dataset_name, f"PNG/{short_name}", exc)

            # ── HTML (Plotly) ─────────────────────────────────────────
            if self.generate_html:
                html_path = str(html_dir / f"{short_name}.html")
                label     = short_name.replace("_", " ").title()
                print(f"  ├─ [HTML] {label} …", end=" ", flush=True)
                try:
                    fig = plotly_func(
                        dataset_name,
                        filter_name=self.filter_name,
                        build_dir=self.build_dir,
                        intervals=ivs,
                        save_path=html_path,
                    )
                    entry["html"] = html_path
                    print("✓")
                except Exception as exc:
                    print("✗")
                    self._record_error(dataset_name, f"HTML/{short_name}", exc)

            if entry:
                results[short_name] = entry

        return results

    # ------------------------------------------------------------------
    def generate_all(self, datasets: Optional[List[str]] = None) -> None:
        if datasets is None:
            discovered = self.discover_datasets()
            datasets   = discovered.get(self.filter_name, [])

        if not datasets:
            print("\n⚠  No datasets found.")
            return

        self.results["summary"]["total"] = len(datasets)

        for dataset_name in datasets:
            try:
                ds_results = self.visualize_dataset(dataset_name)
                if ds_results:
                    self.results["datasets"][dataset_name] = ds_results
                    self.results["summary"]["successful"] += 1
                else:
                    self.results["summary"]["failed"] += 1
            except Exception as exc:
                self.results["summary"]["failed"] += 1
                self._record_error(dataset_name, "dataset", exc)

        self._print_summary()
        self._save_manifest()

    # ------------------------------------------------------------------
    def _record_error(self, dataset: str, context: str, exc: Exception) -> None:
        msg = f"{dataset}/{context}: {exc}"
        self.results["errors"].append(msg)
        print(f"     ↳ Error: {exc}")
        traceback.print_exc()

    # ------------------------------------------------------------------
    def _print_summary(self) -> None:
        s = self.results["summary"]
        print("\n" + "=" * 80)
        print("✅  VISUALIZATION COMPLETE")
        print("=" * 80)
        print(f"  Total:      {s['total']}")
        print(f"  Successful: {s['successful']}")
        print(f"  Failed:     {s['failed']}")

        if self.results["datasets"]:
            print("\n📁 Generated files:")
            for ds, vis_dict in self.results["datasets"].items():
                print(f"\n  {ds}:")
                for vis_name, paths in vis_dict.items():
                    for fmt, path in paths.items():
                        print(f"    ✓ {vis_name} [{fmt.upper()}]")

        if self.results["errors"]:
            print(f"\n⚠  Errors ({len(self.results['errors'])}):")
            for e in self.results["errors"]:
                print(f"  • {e}")

        print(f"\n📂 Output directory: {self.output_dir}\n")

    # ------------------------------------------------------------------
    def _save_manifest(self) -> None:
        manifest_path = self.output_dir / "MANIFEST.json"
        payload = {
            "summary":    self.results["summary"],
            "datasets":   self.results["datasets"],
            "errors":     self.results["errors"],
            "output_dir": str(self.output_dir),
            "build_dir":  str(self.build_dir),
        }
        try:
            with open(manifest_path, "w") as fh:
                json.dump(payload, fh, indent=2)
            print(f"✓ Manifest saved: {manifest_path}")
        except Exception as exc:
            print(f"✗ Could not save manifest: {exc}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate trajectory visualizations (PNG + interactive HTML)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # All datasets, both PNG and HTML
  python generate_visualizations.py --build-dir ./build --output-dir ./vis

  # Specific datasets only
  python generate_visualizations.py --datasets MH01 V202 --build-dir ./build

  # Interactive HTML only (faster, no matplotlib dependency at runtime)
  python generate_visualizations.py --no-png

  # Static PNG only
  python generate_visualizations.py --no-html
        """,
    )

    parser.add_argument("--build-dir",  default=DEFAULT_BUILD_DIR,
                        help=f"Build directory (default: {DEFAULT_BUILD_DIR})")
    parser.add_argument("--output-dir", default="visualizations",
                        help="Output directory (default: visualizations)")
    parser.add_argument("--datasets",   nargs="+",
                        help="Specific datasets to visualize (auto-discover if omitted)")
    parser.add_argument("--intervals",  nargs="+", default=None,
                        help="Override intervals (default: auto-discover per dataset)")
    parser.add_argument("--no-png",     action="store_true",
                        help="Skip static PNG generation")
    parser.add_argument("--no-html",    action="store_true",
                        help="Skip interactive HTML generation")

    args = parser.parse_args()

    if args.no_png and args.no_html:
        parser.error("--no-png and --no-html cannot both be set")

    suite = VisualizationSuite(
        build_dir=args.build_dir,
        output_dir=args.output_dir,
        generate_png=not args.no_png,
        generate_html=not args.no_html,
    )
    suite.generate_all(datasets=args.datasets)


if __name__ == "__main__":
    main()