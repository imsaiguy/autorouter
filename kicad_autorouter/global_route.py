"""Coarse (global) routing: decide which REGIONS each net passes through before the
detailed grid router runs, so the detailed A* searches only a narrow corridor.

The idea is the classic global/detailed split (and the heart of tscircuit's
capacity-mesh router): most routing decisions are made on a small graph of
regions with capacities, and only the final copper is found at full resolution.

Regions  : squares of `region_cells` x `region_cells` grid cells (all layers).
Edges    : between 4-neighbouring regions; capacity = how many tracks can cross
           the shared boundary = usable boundary cells summed over the layers.
Routing  : PathFinder again, but on edges - an edge whose use exceeds its
           capacity gets a higher present price and a growing history cost, and
           only the nets crossing over-full edges are re-routed.
Corridor : the regions a net's global route uses, plus a margin of regions
           round them; the detailed router may only use cells inside it.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

from .problem import BLOCKED, Problem


class GlobalRouter:
    def __init__(self, pb: Problem, region_cells: int = 12, max_passes: int = 25):
        self.pb = pb
        g = pb.grid
        self.R = region_cells
        self.rx = math.ceil(g.nx / self.R)
        self.ry = math.ceil(g.ny / self.R)
        self.nreg = self.rx * self.ry
        self.max_passes = max_passes
        self.cap: Dict[Tuple[int, int], float] = {}
        self._capacities()
        self.use: Dict[Tuple[int, int], int] = {}
        self.hist: Dict[Tuple[int, int], float] = {}
        self.routes: Dict[int, Set[Tuple[int, int]]] = {}   # net id -> edges used
        self.regions: Dict[int, Set[int]] = {}               # net id -> regions used
        self.passes = 0
        self.overflow = 0

    # ------------------------------------------------------------------ geometry
    def region_of_xy(self, x: int, y: int) -> int:
        return (y // self.R) * self.rx + (x // self.R)

    def region_of_cell(self, c: int) -> int:
        g = self.pb.grid
        r = c % g.plane
        return self.region_of_xy(r % g.nx, r // g.nx)

    def _capacities(self) -> None:
        """Usable cells along each region boundary, summed over layers."""
        g, owner, R = self.pb.grid, self.pb.owner, self.R
        nx, ny, plane = g.nx, g.ny, g.plane

        def usable(x, y):
            n = 0
            for l in range(g.nl):
                if owner[l * plane + y * nx + x] != BLOCKED:
                    n += 1
            return n
        for j in range(self.ry):
            for i in range(self.rx):
                a = j * self.rx + i
                if i + 1 < self.rx:                      # vertical boundary at x = (i+1)R
                    x = (i + 1) * R
                    if x < nx:
                        cnt = sum(min(usable(x - 1, y), usable(x, y)) for y in range(j * R, min(ny, (j + 1) * R)))
                        self.cap[(a, a + 1)] = 0.8 * cnt
                if j + 1 < self.ry:                      # horizontal boundary at y = (j+1)R
                    y = (j + 1) * R
                    if y < ny:
                        cnt = sum(min(usable(x, y - 1), usable(x, y)) for x in range(i * R, min(nx, (i + 1) * R)))
                        self.cap[(a, a + self.rx)] = 0.8 * cnt

    def _neighbours(self, a: int):
        i, j = a % self.rx, a // self.rx
        if i > 0:
            yield a - 1, (a - 1, a)
        if i + 1 < self.rx:
            yield a + 1, (a, a + 1)
        if j > 0:
            yield a - self.rx, (a - self.rx, a)
        if j + 1 < self.ry:
            yield a + self.rx, (a, a + self.rx)

    # ------------------------------------------------------------------ search
    def _path(self, sources: Set[int], targets: Set[int], pf: float) -> Optional[List[int]]:
        """Cheapest region path (Dijkstra; the graph is tiny)."""
        dist = {s: 0.0 for s in sources}
        came = {s: -1 for s in sources}
        heap = [(0.0, s) for s in sources]
        heapq.heapify(heap)
        while heap:
            d, a = heapq.heappop(heap)
            if d > dist.get(a, 1e30):
                continue
            if a in targets:
                path = []
                while a != -1:
                    path.append(a)
                    a = came[a]
                return path[::-1]
            for b, e in self._neighbours(a):
                cap = self.cap.get(e, 0.0)
                if cap <= 0:
                    continue
                over = max(0.0, self.use.get(e, 0) + 1 - cap)
                cost = (1.0 + self.hist.get(e, 0.0)) * (1.0 + pf * over)
                nd = d + cost
                if nd < dist.get(b, 1e30):
                    dist[b] = nd
                    came[b] = a
                    heapq.heappush(heap, (nd, b))
        return None

    def _route_net(self, job, pf: float) -> None:
        terms = [{self.region_of_cell(c) for c in t.cells} for t in job.terminals]
        tree: Set[int] = set(terms[0])
        edges: Set[Tuple[int, int]] = set()
        left = list(range(1, len(terms)))
        while left:
            # nearest remaining terminal (by region-centre distance to the tree)
            def gap(k):
                return min(abs(a % self.rx - b % self.rx) + abs(a // self.rx - b // self.rx)
                           for a in terms[k] for b in tree)
            k = min(left, key=gap)
            left.remove(k)
            if terms[k] & tree:
                tree |= terms[k]
                continue
            path = self._path(tree, terms[k], pf)
            if path is None:
                tree |= terms[k]
                continue
            for a, b in zip(path, path[1:]):
                edges.add((min(a, b), max(a, b)))
            tree |= set(path) | terms[k]
        for e in edges:
            self.use[e] = self.use.get(e, 0) + 1
        self.routes[job.id] = edges
        self.regions[job.id] = tree

    def _rip(self, job) -> None:
        for e in self.routes.get(job.id, ()):
            self.use[e] -= 1
        self.routes[job.id] = set()

    def run(self) -> None:
        jobs = [j for j in self.pb.nets if j.fanout_layers is None and len(j.terminals) >= 2]
        jobs.sort(key=lambda j: (j.hpwl(), j.name))
        pf = 0.5
        todo = jobs
        for p in range(1, self.max_passes + 1):
            self.passes = p
            for j in todo:
                self._rip(j)
                self._route_net(j, pf)
            over = {e for e, u in self.use.items() if u > self.cap.get(e, 0.0)}
            self.overflow = len(over)
            if not over:
                break
            for e in over:
                self.hist[e] = self.hist.get(e, 0.0) + 0.5 * (self.use[e] - self.cap.get(e, 0.0))
            pf *= 1.6
            todo = [j for j in jobs if self.routes.get(j.id, set()) & over]

    # ------------------------------------------------------------------ corridors
    def corridor_mask(self, job, margin: int) -> Optional[bytearray]:
        """x/y cells the detailed router may use for this net, or None (no limit)."""
        regs = self.regions.get(job.id)
        if not regs:
            return None
        grown = set()
        for a in regs:
            i, j = a % self.rx, a // self.rx
            for dj in range(-margin, margin + 1):
                for di in range(-margin, margin + 1):
                    ii, jj = i + di, j + dj
                    if 0 <= ii < self.rx and 0 <= jj < self.ry:
                        grown.add(jj * self.rx + ii)
        g = self.pb.grid
        mask = bytearray(g.plane)
        R, nx, ny = self.R, g.nx, g.ny
        for a in grown:
            i, j = a % self.rx, a // self.rx
            for y in range(j * R, min(ny, (j + 1) * R)):
                row = y * nx
                for x in range(i * R, min(nx, (i + 1) * R)):
                    mask[row + x] = 1
        return mask
