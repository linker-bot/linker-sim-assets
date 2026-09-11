"""linker-robot-assets — bundled robot asset tree + composer + validators.

This package ships:

- the asset tree at ``linker_robot_assets/assets/`` (URDF / MJCF +
  meshes, organised by ``components/`` and ``workstations/``);
- the composer that builds workstation URDFs from components
  (``linker_robot_assets.composer``);
- two validators (``linker_robot_assets.validate_workstation`` and
  ``linker_robot_assets.validate_component_mjcf``);
- a loader API exposed at the package top level for downstream consumers
  (``linker-sim``'s registry, real-robot teleop tools, etc.).

The composer + validators are gated behind the ``[authoring]`` extra so
that runtime consumers (which only need ``asset_root`` /
``load_manifest`` / ``workstations``) get a minimal install.
"""

from __future__ import annotations

from pathlib import Path

import yaml

# Assets ship inside the package (sibling of __init__.py) so editable
# installs and built wheels resolve identically. Layered alongside the
# Python package, not under a parallel `assets/` at the package root —
# this avoids the hatch force-include + editable-install path-drift
# problem.
_ASSET_ROOT = Path(__file__).resolve().parent / "assets"

__all__ = ["asset_root", "workstations", "units", "load_manifest"]


def asset_root() -> Path:
    """Return the on-disk root of the bundled asset tree.

    Layout under the returned path::

        components/{arms,bases,hands}/<name>/{meta.yaml,variants/...}
        workstations/<name>/{recipe.yaml, manifest.yaml,
                             workstation.urdf, workstation.xml}
    """
    return _ASSET_ROOT


def _scan(subdir: str) -> list[str]:
    """List names under `_ASSET_ROOT/<subdir>` that have a committed manifest."""
    root = _ASSET_ROOT / subdir
    if not root.is_dir():
        return []
    return sorted(
        p.name
        for p in root.iterdir()
        if p.is_dir() and (p / "manifest.yaml").is_file()
    )


def workstations() -> list[str]:
    """List full multi-robot workstation scenes that are composed.

    Returns the directory basenames sorted alphabetically. Scenes
    without a committed ``manifest.yaml`` are excluded — those need
    ``python -m linker_robot_assets.composer.compose`` to be runnable.
    """
    return _scan("workstations")


def units() -> list[str]:
    """List atomic single-system units (single-arm / arm+hand / single-hand).

    These are one-articulation composed assets under ``units/`` (solo units
    are auto-derived by ``composer.derive_solo``). Same shape as
    :func:`workstations` — sorted basenames with a committed manifest.
    """
    return _scan("units")


def load_manifest(name: str, *, subdir: str = "workstations") -> dict:
    """Load a ``manifest.yaml`` and return it as a dict.

    Loads from ``workstations/`` by default; pass ``subdir="units"`` for an
    atomic unit. Raises ``FileNotFoundError`` if the directory or manifest is
    missing — call :func:`workstations` / :func:`units` first to enumerate.
    """
    manifest_path = _ASSET_ROOT / subdir / name / "manifest.yaml"
    with manifest_path.open() as f:
        return yaml.safe_load(f)
