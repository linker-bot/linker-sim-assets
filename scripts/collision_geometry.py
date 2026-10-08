"""Conservative fitting of convex collision parts, in metres.

A fit must contain its source hulls AND stay within the configured distance of
its original semantic assembly. Failed or inconclusive checks reject the fit.
"""

from dataclasses import dataclass
from itertools import product
import warnings

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh

EPS = 2e-9


@dataclass
class Polytope:
    points: np.ndarray
    planes: np.ndarray
    lower: np.ndarray
    upper: np.ndarray

    @classmethod
    def make(cls, points: np.ndarray) -> "Polytope":
        hull = ConvexHull(points)
        vertices = np.asarray(points)[hull.vertices]
        return cls(
            vertices,
            np.unique(np.round(hull.equations, 12), axis=0),
            vertices.min(0),
            vertices.max(0),
        )


def within_envelope(
    candidate: Polytope,
    originals: list[Polytope],
    tolerance: float,
    budget: int = 100_000,
) -> bool:
    """Certify the entire candidate, including its interior, against the old union.

    For one convex original, distance is convex, so its maximum on the candidate
    occurs at a vertex. For a union, subdivide candidate bounds and require each
    cell to be covered by one original's inscribed tolerance dilation. Checking
    vertices alone would miss filled holes. The dilation is a Minkowski sum with
    an octahedron of radius tolerance, entirely inside the Euclidean offset.
    """
    if not np.isfinite(tolerance) or tolerance < 0 or not originals:
        raise ValueError("a finite nonnegative tolerance and originals are required")
    if len(originals) == 1:
        old = originals[0]
        inside = np.all(
            candidate.points @ old.planes[:, :3].T + old.planes[:, 3] <= EPS, axis=1
        )
        outside = candidate.points[~inside]
        if not len(outside):
            return True
        hull = trimesh.convex.convex_hull(old.points)
        _, distance, _ = trimesh.proximity.closest_point_naive(hull, outside)
        return bool(distance.max() <= tolerance + EPS)
    offsets = np.vstack((np.eye(3), -np.eye(3))) * tolerance
    cover = [
        Polytope.make((p.points[:, None, :] + offsets[None, :, :]).reshape(-1, 3))
        for p in originals
        if np.all(p.upper + tolerance >= candidate.lower)
        and np.all(p.lower - tolerance <= candidate.upper)
    ]
    center = ((candidate.lower + candidate.upper) / 2)[None, :]
    half = ((candidate.upper - candidate.lower) / 2)[None, :]
    visited = 0
    while len(center):
        visited += len(center)
        if visited > budget:
            return False
        planes = candidate.planes
        value = center @ planes[:, :3].T + planes[:, 3]
        radius = half @ abs(planes[:, :3]).T
        outside = np.any(value - radius > EPS, axis=1)
        center_inside = np.all(value <= EPS, axis=1)
        covered = np.zeros(len(center), dtype=bool)
        point_covered = np.zeros(len(center), dtype=bool)
        for p in cover:
            value = center @ p.planes[:, :3].T + p.planes[:, 3]
            radius = half @ abs(p.planes[:, :3]).T
            covered |= np.all(value + radius <= EPS, axis=1)
            point_covered |= np.all(value <= EPS, axis=1)
        if np.any(center_inside & ~point_covered):
            return False
        remaining = ~(outside | covered)
        center, half = center[remaining], half[remaining]
        axes = half.argmax(1)
        rows = np.arange(len(center))
        half[rows, axes] *= 0.5
        shift = np.zeros_like(half)
        shift[rows, axes] = half[rows, axes]
        center = np.concatenate((center - shift, center + shift))
        half = np.concatenate((half, half))
    return True


@dataclass
class FittedCollider:
    kind: str
    envelope: Polytope
    position: np.ndarray
    rotation: np.ndarray
    size: np.ndarray
    mesh: trimesh.Trimesh | None = None

    @property
    def rpy(self) -> np.ndarray:
        # Euler yaw is nonunique at pitch +/-pi/2; SciPy's yaw=0 is equivalent.
        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore", message="Gimbal lock detected", category=UserWarning
            )
            return Rotation.from_matrix(self.rotation).as_euler("xyz")

    @property
    def quaternion_wxyz(self) -> np.ndarray:
        return np.roll(Rotation.from_matrix(self.rotation).as_quat(), 1)


def fit_collider(
    meshes: list[trimesh.Trimesh], kind: str, oriented: bool = False, axis: int = 2
) -> FittedCollider:
    points = np.vstack([mesh.vertices for mesh in meshes])
    if kind == "mesh":
        mesh = trimesh.convex.convex_hull(points)
        return FittedCollider(
            kind, Polytope.make(points), np.zeros(3), np.eye(3), np.zeros(3), mesh
        )
    rotation = np.eye(3)
    if oriented:
        transform, _ = trimesh.bounds.oriented_bounds(
            trimesh.convex.convex_hull(points)
        )
        rotation = transform[:3, :3].T
    if kind == "cylinder":
        # Native URDF/MJCF cylinders extend along local Z.
        rotation = (
            rotation
            @ Rotation.from_euler(
                "y" if axis == 0 else "x",
                np.pi / 2 if axis == 0 else -np.pi / 2 if axis == 1 else 0,
            ).as_matrix()
        )
    local = points @ rotation
    lower, upper = local.min(0), local.max(0)
    center = (lower + upper) / 2
    size = upper - lower
    if kind == "box":
        envelope = np.array(list(product(*zip(lower, upper, strict=True))))
    elif kind == "cylinder":
        radius = np.linalg.norm((local - center)[:, :2], axis=1).max()
        # Certify a circumscribed polygon, which contains the native cylinder.
        angles = np.arange(64) * 2 * np.pi / 64
        outer = radius / np.cos(np.pi / 64)
        envelope = np.tile(center, (128, 1))
        envelope[:, 0] += np.tile(np.cos(angles) * outer, 2)
        envelope[:, 1] += np.tile(np.sin(angles) * outer, 2)
        envelope[:64, 2], envelope[64:, 2] = lower[2], upper[2]
        size = np.array([radius, size[2]])
    else:
        raise ValueError(f"unsupported collider kind: {kind}")
    return FittedCollider(
        kind, Polytope.make(envelope @ rotation.T), center @ rotation.T, rotation, size
    )
