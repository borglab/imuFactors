"""Repo-root shim for `python -m viewer.app`."""

from python.viewer import create_dash_app, main

__all__ = ["create_dash_app", "main"]
