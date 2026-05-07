"""Tensor utilities shared by Delama preintegration scripts."""

from __future__ import annotations

from pathlib import Path
import pickle

import torch


DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def bmv(matrix: torch.Tensor, vector: torch.Tensor) -> torch.Tensor:
    """Batch matrix-vector multiplication."""
    return matrix.bmm(vector.unsqueeze(2)).squeeze(2)


def bmtv(matrix: torch.Tensor, vector: torch.Tensor) -> torch.Tensor:
    """Batch matrix-transpose-vector multiplication."""
    return matrix.transpose(1, 2).bmm(vector.unsqueeze(2)).squeeze(2)


def bmtm(matrix_a: torch.Tensor, matrix_b: torch.Tensor) -> torch.Tensor:
    """Batch matrix-transpose-matrix multiplication."""
    return matrix_a.transpose(1, 2).bmm(matrix_b)


def bouter(lhs: torch.Tensor, rhs: torch.Tensor) -> torch.Tensor:
    """Batch outer product for vectors."""
    return lhs.unsqueeze(2).bmm(rhs.unsqueeze(1))


def btrace(matrix: torch.Tensor) -> torch.Tensor:
    """Batch trace for square matrices."""
    return matrix.diagonal(dim1=1, dim2=2).sum(dim=1)


def baxat(matrix_a: torch.Tensor, matrix_x: torch.Tensor) -> torch.Tensor:
    """Compute A X A^T for a batch of matrices."""
    return matrix_a.bmm(matrix_x).bmm(matrix_a.transpose(1, 2))


def bdot(lhs: torch.Tensor, rhs: torch.Tensor) -> torch.Tensor:
    """Batch dot product for row vectors."""
    return (lhs * rhs).sum(dim=1)


def pdump(value: object, output_dir: str | Path, file_name: str) -> None:
    """Pickle-dump a value to output_dir/file_name."""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    with (output_path / file_name).open("wb") as handle:
        pickle.dump(value, handle)
