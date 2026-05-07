"""Convenience entrypoint for Delama Gal3 evaluation."""

from pathlib import Path
import sys


_LOCAL_PACKAGE_ROOT = Path(__file__).resolve().parent / "imuFactors"
if str(_LOCAL_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_LOCAL_PACKAGE_ROOT))

from delama_gal3.preintegration_delama_gal3 import main


if __name__ == "__main__":
    main()
