"""Model-generated field table, indexed by magnetic vector, with local refinement.

Table points are numerical model samples, not additional physical measurements.
The nominal cube is a seed region, never a tracking range cutoff. Outside it,
field-strength-based seeds allow refinement to positions beyond that region.
Matching is separate from selecting a position using movement history.
"""

import heapq
import math

from magtrack_src.geometry.vectors import add, norm, sub
from magtrack_src.position.coil_model import scale_vector
from magtrack_src.position.position_solver import _seeds, _settle


class FieldLookup:
    def __init__(self, coil, range_cm=15.0, spacing_cm=1.0):
        self.coil, self.spacing_cm = coil, spacing_cm
        n = int(math.ceil(range_cm / spacing_cm))
        entries = []
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                for k in range(-n, n + 1):
                    p = add(coil.center, (i * spacing_cm, j * spacing_cm, k * spacing_cm))
                    field = coil.field(p)
                    if all(math.isfinite(v) for v in field):
                        entries.append((field, p))
        self.size = len(entries)
        if not entries:
            raise ValueError("The model cannot generate finite lookup fields.")
        def build(items, depth=0):
            if not items:
                return None
            axis = depth % 3
            items.sort(key=lambda item: item[0][axis])
            mid = len(items) // 2
            return (items[mid], axis, build(items[:mid], depth + 1), build(items[mid + 1:], depth + 1))
        self.tree = build(entries)

    def nearest(self, target, count=12):
        heap, serial = [], 0
        def visit(node):
            nonlocal serial
            if node is None:
                return
            (field, position), axis, left, right = node
            distance = sum((a - b) ** 2 for a, b in zip(field, target))
            serial += 1
            item = (-distance, serial, position)
            if len(heap) < count:
                heapq.heappush(heap, item)
            elif distance < -heap[0][0]:
                heapq.heapreplace(heap, item)
            delta = target[axis] - field[axis]
            first, other = (left, right) if delta < 0 else (right, left)
            visit(first)
            if len(heap) < count or delta * delta <= -heap[0][0]:
                visit(other)
        visit(self.tree)
        return [item[2] for item in sorted(heap, reverse=True)]

    def matches(self, target, tolerance_ut, previous=None):
        """Return finite, field-checked roots; never publish a poor closest fit.

        A continuum in the coil's middle plane is represented by several points
        around its circle. It is not claimed to be an exhaustive finite list.
        """
        if norm(target) < 1e-9:
            return []
        seeds = self.nearest(target)
        if previous is not None:
            seeds.insert(0, previous)
        # These seeds let a finite table serve readings outside its nominal cube.
        seeds.extend(_seeds(self.coil, target))
        found = []
        def keep(p, miss):
            if not all(math.isfinite(v) for v in (*p, miss)) or miss > tolerance_ut:
                return
            if all(norm(sub(p, other[0])) > 0.5 for other in found):
                found.append((tuple(p), miss))
        for seed in seeds:
            try:
                p, miss = _settle(self.coil, target, seed)
                keep(p, miss)
                keep(self.coil.mirror(p), miss)
            except (OverflowError, ValueError, ZeroDivisionError):
                continue
        # A finite two-pole model has a circle of identical vectors in this plane.
        axis = self.coil.axis
        for p, _ in list(found[:2]):
            d = sub(p, self.coil.center)
            along = sum(a * b for a, b in zip(d, axis))
            if abs(along) < 0.05:
                radial = sub(d, scale_vector(axis, along))
                cross = (axis[1]*radial[2]-axis[2]*radial[1], axis[2]*radial[0]-axis[0]*radial[2],
                         axis[0]*radial[1]-axis[1]*radial[0])
                for i in range(8):
                    angle = i * math.pi / 4
                    q = add(self.coil.center, add(scale_vector(radial, math.cos(angle)),
                                                  scale_vector(cross, math.sin(angle))))
                    keep(q, norm(sub(self.coil.field(q), target)))
        return sorted(found, key=lambda item: item[1])
