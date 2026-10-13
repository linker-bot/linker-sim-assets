"""O6-only compatibility entry point for the shared hand-coupling generator."""

from __future__ import annotations

import sys

from .hand_coupling import coupled_mjcf
from .hand_coupling import main as _main

__all__ = ["coupled_mjcf", "main"]


def main(argv: list[str] | None = None) -> int:
    return _main(["--hand", "o6", *(sys.argv[1:] if argv is None else argv)])


if __name__ == "__main__":
    raise SystemExit(main())
