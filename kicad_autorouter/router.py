"""Multi-pass negotiated-congestion router (PathFinder, McMurchie & Ebeling 1995).

How it thinks
-------------
Pass 1: every net is routed with A* *as if it were alone* - other nets'
tracks only cost a little extra, so nets are allowed to overlap.

After each pass every grid cell that is used by more than one net is
*overused*.  Two prices are raised:

* present cost  p = 1 + pres_fac * (other nets in the cell)   -> pres_fac grows every pass
* history cost  h += hist_fac * overuse                        -> remembers chronic hot-spots

and the cost of a cell is ``(1 + h) * p``.  Only nets that touch an overused
cell are ripped up and re-routed.  Cheap, uncontested space wins; nets that
have no alternative keep the contested cell because everyone else learns to
avoid it.  When no cell is overused the routing is legal.

Physically it is a market (or a slow anneal): congestion is priced, and the
price only ever goes up, so the system freezes into a legal solution.

Events
------
Pass ``callback`` to watch it.  It is called as ``callback(event, data)``:

    "pass_start"  {"pass", "pres_fac", "nets"}
    "connection"  {"pass", "net", "path", "explored"}   (explored only if record_exploration)
    "net_done"    {"pass", "net"}
    "pass_end"    {"pass", "overused", "conflict_nets", "length_mm", "vias", "routed"}
    "done"        {"result"}

Return ``False`` from the callback to stop early.
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Set, Tuple

from .problem import BLOCKED, FREE, NetJob, Problem

SQ2 = math.sqrt(2.0)
# (dx, dy, length); index = direction id
DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (-1, -1, SQ2)]
VIA_DIR = 8          # arrived through a via: any direction may follow, no second via
START_DIR = 9        # a source cell: any direction may follow
NSTATE = 10          # A* state = cell * NSTATE + arrival direction


def _turn_ok(a: int, b: int) -> bool:
    """Only 0 or 45 degree changes of direction (no 90 deg corners, no hairpins)."""
    if a >= 8:
        return True
    ax, ay, al = DIRS[a]
    bx, by, bl = DIRS[b]
    return (ax * bx + ay * by) / (al * bl) > 0.70


TURN_OK = [[_turn_ok(a, b) for b in range(8)] for a in range(NSTATE)]
DIR_INDEX = {(dx, dy): i for i, (dx, dy, _) in enumerate(DIRS)}


@dataclass
class PassStats:
    number: int
    overused: int
    conflict_nets: int
    length_mm: float
    vias: int
    routed: int
    seconds: float


@dataclass
class NetRoute:
    name: str
    segments: List[Tuple[float, float, float, float, str]] = field(default_factory=list)
    vias: List[Tuple[float, float]] = field(default_factory=list)
    complete: bool = False
    failed_connections: int = 0


@dataclass
class RouteResult:
    nets: Dict[str, NetRoute]
    passes: List[PassStats]
    converged: bool
    removed: List[str]                    # nets dropped to make the result legal
    skipped: Dict[str, str]
    cancelled: bool = False
    pour_warning: bool = False            # some pads may sit on pour islands
    notes: Dict[str, str] = field(default_factory=dict)

    @property
    def complete_nets(self) -> int:
        return sum(1 for n in self.nets.values() if n.complete)

    def summary(self) -> str:
        total = len(self.nets)
        lines = [f"Routed {self.complete_nets}/{total} nets completely in "
                 f"{len(self.passes)} pass(es){' (converged)' if self.converged else ''}."]
        segs = sum(len(n.segments) for n in self.nets.values())
        vias = sum(len(n.vias) for n in self.nets.values())
        lines.append(f"{segs} track segments, {vias} vias.")
        if self.removed:
            lines.append("Removed to keep the board legal: " + ", ".join(self.removed))
        bad = [n.name for n in self.nets.values() if not n.complete and n.name not in self.removed]
        if bad:
            lines.append("Incomplete: " + ", ".join(bad))
        if self.skipped:
            lines.append("Skipped: " + ", ".join(f"{k} ({v})" for k, v in sorted(self.skipped.items())))
        for k, v in sorted(self.notes.items()):
            lines.append(f"Note - {k}: {v}")
        if self.pour_warning:
            lines.append("Warning: some pour pads/vias may end up on islands - check the zone fill.")
        if self.cancelled:
            lines.append("Cancelled by user.")
        return "\n".join(lines)


class Router:
    def __init__(self, problem: Problem,
                 callback: Optional[Callable[[str, dict], Optional[bool]]] = None,
                 record_exploration: bool = False):
        self.pb = problem
        self.cb = callback
        self.record = record_exploration
        g = problem.grid
        self.occ = [0] * g.size
        self.hist = [0.0] * g.size
        self.pres_fac = problem.settings.present_factor
        self.passes: List[PassStats] = []
        self._cancel = False
        # direction cost multipliers per layer and direction
        s = problem.settings
        self.dir_mult: List[List[float]] = []
        for d in problem.layer_dirs:
            row = []
            for (dx, dy, _) in DIRS:
                if d == "H":
                    row.append(1.0 if dy == 0 else (s.wrong_way_cost if dx == 0 else 1.15))
                elif d == "V":
                    row.append(1.0 if dx == 0 else (s.wrong_way_cost if dy == 0 else 1.15))
                else:
                    row.append(1.0)
            self.dir_mult.append(row)
        nx = g.nx
        self.fp_offsets = [dy * nx + dx for (dx, dy) in problem.via_footprint]
        self.fp_xy = problem.via_footprint

    # ------------------------------------------------------------------ events
    def _emit(self, ev: str, **data) -> None:
        if self.cb is not None:
            if self.cb(ev, data) is False:
                self._cancel = True

    # ------------------------------------------------------------------ costs
    def _cell_cost(self, i: int) -> float:
        return (1.0 + self.hist[i]) * (1.0 + self.pres_fac * self.occ[i])

    def _via_ok(self, nid: int, xy: int, x: int, y: int) -> bool:
        o = self.pb.via_owner[xy]
        if o != FREE and o != nid:
            return False
        g = self.pb.grid
        for (dx, dy) in self.fp_xy:
            if not (0 <= x + dx < g.nx and 0 <= y + dy < g.ny):
                return False
        return True

    def _via_extra(self, xy: int, own: Set[int]) -> float:
        """Congestion price of the cells a via at column xy would claim."""
        g = self.pb.grid
        occ, hist, pf = self.occ, self.hist, self.pres_fac
        tot = 0.0
        for l in range(g.nl):
            base = l * g.plane + xy
            for off in self.fp_offsets:
                c = base + off
                if c in own:
                    continue
                o = occ[c]
                h = hist[c]
                if o or h:
                    tot += (1.0 + h) * (1.0 + pf * o) - 1.0
        return tot

    # ------------------------------------------------------------------ A*
    def _astar(self, job: NetJob, sources: Set[int], targets: Optional[Set[int]],
               bbox: Optional[Tuple[int, int, int, int]], own: Set[int],
               target_layers: Optional[Set[int]] = None, goal: Optional[bytearray] = None,
               source_dirs: Optional[Dict[int, int]] = None):
        """Cheapest path from any source cell to any target cell.

        With ``target_layers`` instead of ``targets`` any usable cell on those
        layers is a goal (fan-out of a pad to a plane layer)."""
        pb, g, s = self.pb, self.pb.grid, self.pb.settings
        nx, ny, plane, nl = g.nx, g.ny, g.plane, g.nl
        owner, occ, hist = pb.owner, self.occ, self.hist
        pf = self.pres_fac
        nid = job.id
        bend = s.bend_cost
        via_cost = s.via_cost
        dir_mult = self.dir_mult

        if bbox is None:
            bx0, by0, bx1, by1 = 0, 0, nx - 1, ny - 1
        else:
            bx0, by0, bx1, by1 = bbox

        if target_layers is not None:
            tlayers = set(target_layers)
            layer_only = True
            tx0 = tx1 = ty0 = ty1 = 0
        else:
            layer_only = False
            # heuristic: octile distance to the targets' bounding box
            txs, tys = [], []
            tlayers = set()
            for t in targets:
                r = t % plane
                txs.append(r % nx)
                tys.append(r // nx)
                tlayers.add(t // plane)
            tx0, tx1, ty0, ty1 = min(txs), max(txs), min(tys), max(tys)
        layer_h = via_cost if len(tlayers) < nl else 0.0

        def h(x, y, l):
            if layer_only:
                return 0.0 if l in tlayers else layer_h
            dx = tx0 - x if x < tx0 else (x - tx1 if x > tx1 else 0)
            dy = ty0 - y if y < ty0 else (y - ty1 if y > ty1 else 0)
            if dx < dy:
                dx, dy = dy, dx
            v = dx + (SQ2 - 1.0) * dy
            if layer_h and l not in tlayers:
                v += layer_h
            return v

        # State = cell * NSTATE + arrival direction, so the 45-degree turn rule and the
        # bend cost are exact (the same cell reached heading another way is a new state).
        S = NSTATE
        gbest: Dict[int, float] = {}
        came: Dict[int, int] = {}
        heap = []
        for c in sources:
            r = c % plane
            x, y, l = r % nx, r // nx, c // plane
            # leaving the free end of an earlier route: carry on within 45 degrees
            st = c * S + (source_dirs.get(c, START_DIR) if source_dirs else START_DIR)
            gbest[st] = 0.0
            came[st] = -1
            heap.append((h(x, y, l), 0.0, st))
        heapq.heapify(heap)
        explored = [] if self.record else None
        seen_cells = set()      # cells already expanded once (vias only need that first visit)
        push, pop = heapq.heappush, heapq.heappop
        via_retry: Set[int] = set()   # first visit came through a via: allow one later try
        hit = -1
        while heap:
            f, gc, st = pop(heap)
            if gc > gbest.get(st, 1e30):
                continue
            c, ad = divmod(st, S)
            if (goal[c] and c not in sources) if layer_only else (c in targets):
                hit = st
                break
            first_visit = c not in seen_cells
            if first_visit:
                seen_cells.add(c)
                if explored is not None:
                    explored.append(c)
            l, r = divmod(c, plane)
            y, x = divmod(r, nx)
            dm = dir_mult[l]
            ok_turn = TURN_OK[ad]
            for d in range(8):
                if not ok_turn[d]:
                    continue
                dx, dy, ln = DIRS[d]
                x2, y2 = x + dx, y + dy
                if x2 < bx0 or x2 > bx1 or y2 < by0 or y2 > by1:
                    continue
                n2 = c + dy * nx + dx
                o = owner[n2]
                if o != FREE and o != nid:
                    continue
                if n2 in own:
                    cost = ln
                else:
                    cost = ln * (1.0 + hist[n2]) * (1.0 + pf * occ[n2])
                cost *= dm[d]
                if dx and dy:   # diagonal: also squeezes past two side cells
                    s1 = c + dx
                    s2 = c + dy * nx
                    o1, o2 = owner[s1], owner[s2]
                    if (o1 != FREE and o1 != nid) or (o2 != FREE and o2 != nid):
                        continue
                    if s1 not in own:
                        cost += (1.0 + hist[s1]) * (1.0 + pf * occ[s1]) - 1.0
                    if s2 not in own:
                        cost += (1.0 + hist[s2]) * (1.0 + pf * occ[s2]) - 1.0
                if ad < 8 and ad != d:
                    cost += bend
                ng = gc + cost
                n2s = n2 * S + d
                if ng < gbest.get(n2s, 1e30):
                    gbest[n2s] = ng
                    came[n2s] = st
                    push(heap, (ng + h(x2, y2, l), ng, n2s))
            # vias: jump to any other layer at the same x/y.  The via's cost does not depend
            # on the arrival direction and the first visit of a cell is its cheapest one
            # (A* pops in f order and h depends only on the cell), so expand vias once.
            try_via = False
            if nl > 1:
                if ad == VIA_DIR:
                    if first_visit:
                        via_retry.add(c)          # expand vias on the next visit instead
                elif first_visit or c in via_retry:
                    via_retry.discard(c)
                    try_via = True
            if try_via and self._via_ok(nid, r, x, y):
                vc = via_cost + self._via_extra(r, own)
                for l2 in range(nl):
                    if l2 == l:
                        continue
                    n2 = l2 * plane + r
                    o = owner[n2]
                    if o != FREE and o != nid:
                        continue
                    ng = gc + vc
                    n2s = n2 * S + VIA_DIR
                    if ng < gbest.get(n2s, 1e30):
                        gbest[n2s] = ng
                        came[n2s] = st
                        push(heap, (ng + h(x, y, l2), ng, n2s))
        if hit < 0:
            return None, explored
        path = []
        st = hit
        while st != -1:
            path.append(st // S)
            st = came[st]
        path.reverse()
        return path, explored

    # ------------------------------------------------------------------ nets
    def _path_resources(self, path: List[int]) -> Set[int]:
        g = self.pb.grid
        plane, nx = g.plane, g.nx
        res = set(path)
        for a, b in zip(path, path[1:]):
            la, ra = divmod(a, plane)
            lb, rb = divmod(b, plane)
            if la != lb:   # via
                for l in range(g.nl):
                    base = l * plane + ra
                    for off in self.fp_offsets:
                        res.add(base + off)
                continue
            ya, xa = divmod(ra, nx)
            yb, xb = divmod(rb, nx)
            if xa != xb and ya != yb:
                res.add(la * plane + ya * nx + xb)
                res.add(la * plane + yb * nx + xa)
        return res

    def _rip_up(self, job: NetJob) -> None:
        occ = self.occ
        for c in job.resources:
            occ[c] -= 1
        job.resources = set()
        job.paths = []
        job.failed = 0

    def _route_net(self, job: NetJob, pass_no: int) -> None:
        g = self.pb.grid
        plane, nx = g.plane, g.nx
        terms = job.terminals
        if job.fanout_layers is not None:
            self._route_fanout(job, pass_no)
            return
        # Prim-style order: start at the terminal nearest the centroid, then
        # always connect the closest unconnected terminal to the growing tree.
        cx = sum(t.cx for t in terms) / len(terms)
        cy = sum(t.cy for t in terms) / len(terms)
        start = min(range(len(terms)), key=lambda k: (terms[k].cx - cx) ** 2 + (terms[k].cy - cy) ** 2)
        tree: Set[int] = set(terms[start].cells)
        tree_pts = [(terms[start].cx, terms[start].cy)]
        left = [k for k in range(len(terms)) if k != start]
        own: Set[int] = set()
        pad_cells: Set[int] = set()
        for t in terms:
            pad_cells |= t.cells
        ends: Dict[int, Optional[int]] = {}   # free route end -> direction to continue in
        margin = 6 + 2 * pass_no
        while left:
            k = min(left, key=lambda k: min((terms[k].cx - px) ** 2 + (terms[k].cy - py) ** 2
                                            for (px, py) in tree_pts))
            left.remove(k)
            tgt = terms[k].cells
            xs = [(c % plane) % nx for c in tree] + [(c % plane) % nx for c in tgt]
            ys = [(c % plane) // nx for c in tree] + [(c % plane) // nx for c in tgt]
            bbox = (max(0, min(xs) - margin), max(0, min(ys) - margin),
                    min(g.nx - 1, max(xs) + margin), min(g.ny - 1, max(ys) + margin))
            sdirs = {c: d for c, d in ends.items() if d is not None}
            path, explored = self._astar(job, tree, tgt, bbox, own, source_dirs=sdirs)
            if path is None:
                path, explored = self._astar(job, tree, tgt, None, own, source_dirs=sdirs)
            if path is None and sdirs:   # the 45-degree rule at a route end can be too strict
                path, explored = self._astar(job, tree, tgt, None, own)
            if path is None:
                job.failed += 1
                self._emit("connection", **{"pass": pass_no, "net": job.name, "path": None,
                                            "explored": explored})
                continue
            job.paths.append(path)
            if len(path) >= 2:
                for cell, prev in ((path[-1], path[-2]), (path[0], path[1])):
                    if cell in pad_cells or cell // plane != prev // plane:
                        continue
                    if cell in ends:            # two routes meet here: it is a junction now
                        ends[cell] = None
                    else:
                        r1, r0 = cell % plane, prev % plane
                        ends[cell] = DIR_INDEX.get(((r1 % nx) - (r0 % nx), (r1 // nx) - (r0 // nx)))
            res = self._path_resources(path)
            own |= res
            tree |= set(path)
            tree |= tgt
            tree_pts.append((terms[k].cx, terms[k].cy))
            self._emit("connection", **{"pass": pass_no, "net": job.name, "path": path,
                                        "explored": explored})
            if self._cancel:
                break
        # cells inside the net's own pads/halo cannot be used by anyone else
        owner = self.pb.owner
        nid = job.id
        job.resources = {c for c in own if owner[c] != nid}
        for c in job.resources:
            self.occ[c] += 1

    def _route_fanout(self, job: NetJob, pass_no: int) -> None:
        """Zone net: give every pad that is off the pour layer a stub + via to it."""
        g = self.pb.grid
        plane, nx = g.plane, g.nx
        own: Set[int] = set()
        pour = self._pour_mask(job)
        for t in job.terminals:
            xs = [(c % plane) % nx for c in t.cells]
            ys = [(c % plane) // nx for c in t.cells]
            m = 6 + 2 * pass_no
            bbox = (max(0, min(xs) - m), max(0, min(ys) - m),
                    min(g.nx - 1, max(xs) + m), min(g.ny - 1, max(ys) + m))
            path, explored = self._astar(job, t.cells, None, bbox, own, job.fanout_layers, pour)
            if path is None:
                path, explored = self._astar(job, t.cells, None, None, own, job.fanout_layers, pour)
            if path is None:
                job.failed += 1
            else:
                job.paths.append(path)
                own |= self._path_resources(path)
            self._emit("connection", **{"pass": pass_no, "net": job.name, "path": path,
                                        "explored": explored})
            if self._cancel:
                break
        nid = job.id
        job.resources = {c for c in own if self.pb.owner[c] != nid}
        for c in job.resources:
            self.occ[c] += 1

    # ------------------------------------------------------------------ pours
    def _pour_labels(self, zid: int, layer: int, mine: Set[int]):
        """Estimate a pour on one layer: cells no other net uses, eroded by one
        cell (zone clearance), split into 4-connected regions.

        Returns (label per x/y, size per label, free mask)."""
        g = self.pb.grid
        nx, ny, plane = g.nx, g.ny, g.plane
        owner, occ = self.pb.owner, self.occ
        base = layer * plane
        free = bytearray(plane)
        for xy in range(plane):
            c = base + xy
            o = owner[c]
            if (o == FREE or o == zid) and occ[c] - (1 if c in mine else 0) == 0:
                free[xy] = 1
        ero = bytearray(plane)
        for y in range(1, ny - 1):
            row = y * nx
            for x in range(1, nx - 1):
                xy = row + x
                if free[xy] and free[xy - 1] and free[xy + 1] and free[xy - nx] and free[xy + nx]:
                    ero[xy] = 1
        label = [-1] * plane
        sizes: List[int] = []
        for s0 in range(plane):
            if not ero[s0] or label[s0] >= 0:
                continue
            lab = len(sizes)
            comp = [s0]
            label[s0] = lab
            k = 0
            while k < len(comp):
                a = comp[k]
                k += 1
                ax = a % nx
                for b, ok in ((a - 1, ax > 0), (a + 1, ax < nx - 1), (a - nx, a >= nx), (a + nx, a < plane - nx)):
                    if ok and ero[b] and label[b] < 0:
                        label[b] = lab
                        comp.append(b)
            sizes.append(len(comp))
        return label, sizes, free

    def _pour_mask(self, job: NetJob) -> bytearray:
        """Goal cells for a fan-out via: the main (largest) region of each pour layer."""
        g = self.pb.grid
        mask = bytearray(g.size)
        for l in job.fanout_layers:
            label, sizes, _ = self._pour_labels(job.id, l, job.resources)
            if not sizes:
                continue
            main = max(range(len(sizes)), key=sizes.__getitem__)
            base = l * g.plane
            for xy, lab in enumerate(label):
                if lab == main:
                    mask[base + xy] = 1
        return mask

    def _pour_check(self, jobs: List[NetJob]) -> Tuple[List[NetJob], int]:
        """Find pads/vias of pour nets that routing has cut off onto islands.

        The routed cells walling in each island get more expensive (history)
        and the nets owning them are sent back for re-routing - the pour takes
        part in the negotiation like any other net."""
        pb, g = self.pb, self.pb.grid
        if not pb.zones:
            return [], 0
        plane, nx = g.plane, g.nx
        by_id = {j.id: j for j in jobs}
        users: Dict[int, List[NetJob]] = {}
        zone_layers = set()
        for zl, _ in pb.zones.values():
            zone_layers |= zl
        for j in jobs:
            for c in j.resources:
                if c // plane in zone_layers:
                    users.setdefault(c, []).append(j)
        blamed: Set[int] = set()
        out: List[NetJob] = []
        cut = 0
        hf = pb.settings.history_factor
        for zid, (zl, anchors) in pb.zones.items():
            zjob = by_id.get(zid)
            mine = zjob.resources if zjob else set()
            anc = set(anchors)
            if zjob is not None and zjob.fanout_layers is not None:
                anc |= {p[-1] for p in zjob.paths}
            for l in zl:
                la = [c - l * plane for c in anc if c // plane == l]
                if not la:
                    continue
                label, sizes, free = self._pour_labels(zid, l, mine)
                if not sizes:
                    continue
                main = max(range(len(sizes)), key=sizes.__getitem__)
                done_labels = set()
                for xy in la:
                    lab = label[xy]
                    if lab < 0 or lab == main or lab in done_labels:
                        continue          # not judgeable, fine, or already handled
                    done_labels.add(lab)
                    island = [k for k, v in enumerate(label) if v == lab]
                    walls = set()
                    for a in island:
                        ax = a % nx
                        for b, ok in ((a - 1, ax > 0), (a + 1, ax < nx - 1), (a - nx, a >= nx),
                                      (a + nx, a < plane - nx), (a - 2, ax > 1), (a + 2, ax < nx - 2),
                                      (a - 2 * nx, a >= 2 * nx), (a + 2 * nx, a < plane - 2 * nx)):
                            if ok and label[b] != lab:
                                walls.add(l * plane + b)
                    culprits = set()
                    for c in walls:
                        for j in users.get(c, ()):
                            if j.id != zid:
                                culprits.add(j.id)
                                self.hist[c] += hf
                    if culprits:
                        cut += 1
                        for cid in culprits:
                            if cid not in blamed:
                                blamed.add(cid)
                                out.append(by_id[cid])
                        if zjob is not None and zid not in blamed:
                            blamed.add(zid)
                            out.append(zjob)
        return out, cut

    # ------------------------------------------------------------------ main
    def _overused(self) -> Set[int]:
        occ = self.occ
        out = set()
        for j in self.pb.nets:
            for c in j.resources:
                if occ[c] > 1:
                    out.add(c)
        return out

    def run(self) -> RouteResult:
        pb, s = self.pb, self.pb.settings
        jobs = sorted(pb.nets, key=lambda j: (j.hpwl(), j.name))
        converged = False
        pour_stuck = 0
        pour_warning = False
        to_route = list(jobs)
        for pass_no in range(1, s.max_passes + 1):
            t0 = time.time()
            self._emit("pass_start", **{"pass": pass_no, "pres_fac": self.pres_fac,
                                        "nets": [j.name for j in to_route]})
            for job in to_route:
                self._rip_up(job)
                self._route_net(job, pass_no)
                self._emit("net_done", **{"pass": pass_no, "net": job.name})
                if self._cancel:
                    break
            over = self._overused()
            for c in over:
                self.hist[c] += s.history_factor * (self.occ[c] - 1)
            conflict = [j for j in jobs if any(c in over for c in j.resources)]
            pour_nets, cut_off = self._pour_check(jobs)
            if cut_off:
                pour_stuck += 1
                if pour_stuck > s.max_pour_passes:   # give up on islands, keep the routing
                    pour_nets, cut_off = [], 0
                    pour_warning = True
            conflict += [j for j in pour_nets if j not in conflict]
            st = self._stats(pass_no, over, conflict, time.time() - t0)
            self.passes.append(st)
            self._emit("pass_end", **{"pass": pass_no, "overused": st.overused, "pour_cuts": cut_off,
                                      "conflict_nets": [j.name for j in conflict],
                                      "length_mm": st.length_mm, "vias": st.vias,
                                      "routed": st.routed, "seconds": st.seconds})
            if self._cancel:
                break
            if not over and not cut_off:
                converged = True
                break
            self.pres_fac *= s.present_growth
            to_route = conflict
        removed = [] if converged else self._legalize()
        if not self._cancel:
            n = self._straighten()
            self._emit("cleanup", **{"straightened": n})
        result = self._result(converged, removed)
        result.pour_warning = pour_warning
        self._emit("done", result=result)
        return result

    # ------------------------------------------------------------------ clean-up
    def _straighten(self) -> int:
        """Post-route clean-up: replace zig-zags with straight or straight+45 runs.

        Works on each same-layer run of every path.  A stretch p[i]..p[k] is replaced
        by an octilinear path (diagonal then straight, or straight then diagonal)
        when that path is shorter-or-straighter, keeps every turn at 45 degrees max,
        and uses only cells no other net is using (so the result stays legal).
        Returns the number of replacements made."""
        g = self.pb.grid
        plane, nx = g.plane, g.nx
        owner, occ = self.pb.owner, self.occ
        bend = max(self.pb.settings.bend_cost, 0.5)
        made = 0
        for job in self.pb.nets:
            if not job.paths:
                continue
            nid = job.id
            mine = job.resources

            def free(c):
                o = owner[c]
                if o != FREE and o != nid:
                    return False
                return occ[c] - (1 if c in mine else 0) <= 0

            def legal(cells, l):
                base = l * plane
                for (xa, ya), (xb, yb) in zip(cells, cells[1:]):
                    if not free(base + yb * nx + xb):
                        return False
                    if xa != xb and ya != yb:
                        if not free(base + ya * nx + xb) or not free(base + yb * nx + xa):
                            return False
                return True

            def octi(a, b, diag_first):
                (xa, ya), (xb, yb) = a, b
                dx, dy = xb - xa, yb - ya
                sx = (dx > 0) - (dx < 0)
                sy = (dy > 0) - (dy < 0)
                nd = min(abs(dx), abs(dy))
                ns = max(abs(dx), abs(dy)) - nd
                st = (sx, 0) if abs(dx) > abs(dy) else (0, sy)
                steps = ([(sx, sy)] * nd + [st] * ns) if diag_first else ([st] * ns + [(sx, sy)] * nd)
                out = [a]
                x, y = a
                for (ux, uy) in steps:
                    x, y = x + ux, y + uy
                    out.append((x, y))
                return out

            def cost(prev_d, cells, next_d):
                total = 0.0
                d0 = prev_d
                for (xa, ya), (xb, yb) in zip(cells, cells[1:]):
                    d = DIR_INDEX[(xb - xa, yb - ya)]
                    if d0 is not None:
                        if not TURN_OK[d0][d]:
                            return 1e30
                        if d0 != d:
                            total += bend
                    total += DIRS[d][2]
                    d0 = d
                if next_d is not None and d0 is not None:
                    if not TURN_OK[d0][next_d]:
                        return 1e30
                    if d0 != next_d:
                        total += bend
                return total

            # cells where routes of this net join each other must not move
            fixed = set()
            for p_ in job.paths:
                fixed.add(p_[0])
                fixed.add(p_[-1])
            new_paths = []
            for path in job.paths:
                # split into same-layer runs (vias in between stay where they are)
                runs, cur = [], [path[0]]
                for a, b in zip(path, path[1:]):
                    if a // plane != b // plane:
                        runs.append(cur)
                        cur = [b]
                    else:
                        cur.append(b)
                runs.append(cur)
                rebuilt = []
                pieces = []
                for run in runs:            # split each run at junction cells
                    cur = [run[0]]
                    for c in run[1:]:
                        cur.append(c)
                        if c in fixed and c != run[-1]:
                            pieces.append(cur)
                            cur = [c]
                    pieces.append(cur)
                for run in pieces:
                    l = run[0] // plane
                    pts = [((c % plane) % nx, (c % plane) // nx) for c in run]
                    for _ in range(2):          # two sweeps catch nested zig-zags
                        if len(pts) < 3:
                            break
                        out = [pts[0]]
                        i = 0
                        n = len(pts)
                        while i < n - 1:
                            prev_d = (DIR_INDEX[(out[-1][0] - out[-2][0], out[-1][1] - out[-2][1])]
                                      if len(out) >= 2 else None)
                            done = False
                            for k in range(min(n - 1, i + 80), i + 1, -1):
                                next_d = (DIR_INDEX[(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])]
                                          if k + 1 < n else None)
                                old = pts[i:k + 1]
                                old_c = cost(prev_d, old, next_d)
                                for diag_first in (True, False):
                                    cand = octi(pts[i], pts[k], diag_first)
                                    if cand == old:
                                        continue
                                    if cost(prev_d, cand, next_d) < old_c - 1e-9 and legal(cand, l):
                                        out.extend(cand[1:])
                                        i = k
                                        made += 1
                                        done = True
                                        break
                                if done:
                                    break
                            if not done:
                                out.append(pts[i + 1])
                                i += 1
                        pts = out
                    rebuilt.append([l * plane + y * nx + x for (x, y) in pts])
                merged = []
                for r in rebuilt:
                    merged.extend(r[1:] if merged and r and merged[-1] == r[0] else r)
                new_paths.append(merged)
            # swap in the cleaned paths and re-book their cells
            self._rip_up_keep_failed(job)
            job.paths = new_paths
            res = set()
            for p in new_paths:
                res |= self._path_resources(p)
            job.resources = {c for c in res if owner[c] != nid}
            for c in job.resources:
                occ[c] += 1
        return made

    def _rip_up_keep_failed(self, job: NetJob) -> None:
        for c in job.resources:
            self.occ[c] -= 1
        job.resources = set()

    def _legalize(self) -> List[str]:
        """Not converged: drop the most-conflicted nets until nothing overlaps."""
        removed = []
        while True:
            over = self._overused()
            if not over:
                return removed
            worst = max(self.pb.nets, key=lambda j: (sum(1 for c in j.resources if c in over), -j.hpwl()))
            self._rip_up(worst)
            worst.failed = len(worst.terminals) - 1
            removed.append(worst.name)

    def _stats(self, pass_no, over, conflict, secs) -> PassStats:
        g = self.pb.grid
        length = 0.0
        vias = 0
        routed = 0
        for j in self.pb.nets:
            if j.failed == 0 and j.paths:
                routed += 1
            for p in j.paths:
                for a, b in zip(p, p[1:]):
                    xa, ya, la = g.xyl(a)
                    xb, yb, lb = g.xyl(b)
                    if la != lb:
                        vias += 1
                    else:
                        length += math.hypot(xb - xa, yb - ya) * g.pitch
        return PassStats(pass_no, len(over), len(conflict), round(length, 2), vias, routed, round(secs, 3))

    # ------------------------------------------------------------------ geometry
    def path_geometry(self, path: List[int]):
        """Cells -> (segments, vias) in mm; collinear cells are merged."""
        g = self.pb.grid
        segs, vias = [], []
        pts = [g.xyl(c) for c in path]
        anchors = self.pb.anchors

        def mm(x, y):
            return g.center(x, y)

        run_start = 0
        for k in range(1, len(pts) + 1):
            end_run = k == len(pts) or pts[k][2] != pts[k - 1][2]
            if not end_run:
                continue
            run = pts[run_start:k]
            layer = g.layers[run[0][2]]
            if len(run) > 1:
                cur = run[0]
                d0 = None
                for a, b in zip(run, run[1:]):
                    d = (b[0] - a[0], b[1] - a[1])
                    if d0 is not None and d != d0:
                        segs.append((*mm(cur[0], cur[1]), *mm(a[0], a[1]), layer))
                        cur = a
                    d0 = d
                segs.append((*mm(cur[0], cur[1]), *mm(run[-1][0], run[-1][1]), layer))
            if k < len(pts):
                vias.append(mm(pts[k][0], pts[k][1]))
            run_start = k
        # stubs from the grid cell to the true pad centre
        for end in (path[0], path[-1]):
            if end in anchors:
                x, y, l = g.xyl(end)
                cx, cy = mm(x, y)
                ax, ay = anchors[end]
                if math.hypot(ax - cx, ay - cy) > 1e-4:
                    segs.append((cx, cy, ax, ay, g.layers[l]))
        return segs, vias

    def _result(self, converged: bool, removed: List[str]) -> RouteResult:
        nets: Dict[str, NetRoute] = {}
        s = self.pb.settings
        for j in self.pb.nets:
            nr = NetRoute(j.name)
            if j.name not in removed and (j.failed == 0 or s.keep_failed_partial):
                seen_s, seen_v = set(), set()
                for p in j.paths:
                    sg, vi = self.path_geometry(p)
                    for seg in sg:
                        key = tuple(round(v, 4) if isinstance(v, float) else v for v in seg)
                        rkey = (key[2], key[3], key[0], key[1], key[4])
                        if key not in seen_s and rkey not in seen_s:
                            seen_s.add(key)
                            nr.segments.append(seg)
                    for v in vi:
                        key = (round(v[0], 4), round(v[1], 4))
                        if key not in seen_v:
                            seen_v.add(key)
                            nr.vias.append(v)
            nr.complete = j.failed == 0 and j.name not in removed
            nr.failed_connections = j.failed
            nets[j.name] = nr
        r = RouteResult(nets, self.passes, converged, removed, dict(self.pb.skipped), self._cancel)
        r.notes = dict(self.pb.notes)
        return r


def route(board, settings=None, nets=None, callback=None, record_exploration=False) -> RouteResult:
    """Convenience: build the problem and run the router."""
    from .model import RouteSettings
    from .problem import build_problem
    settings = settings or RouteSettings()
    pb = build_problem(board, settings, nets)
    return Router(pb, callback, record_exploration).run()
