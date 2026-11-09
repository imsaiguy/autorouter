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
MAX_RETRIES = 4       # extra passes for a net whose connections failed (without conflicts)
SHARP_STUB_COST = 25.0   # joining an escape stub at an acute angle (allowed only if nothing else fits)
STALL_PASSES = 8      # stop when the overuse has not reached a new low for this many passes
NECK_COST = 8.0      # extra cost per grid step where a net-class track has to narrow


def _turn_ok(a: int, b: int) -> bool:
    """Only 0 or 45 degree changes of direction (no 90 deg corners, no hairpins)."""
    if a >= 8:
        return True
    ax, ay, al = DIRS[a]
    bx, by, bl = DIRS[b]
    return (ax * bx + ay * by) / (al * bl) > 0.70


TURN_OK = [[_turn_ok(a, b) for b in range(8)] for a in range(NSTATE)]


def _turn_ok90(a: int, b: int) -> bool:
    """Last resort: also 90 degree corners (never sharper)."""
    if a >= 8:
        return True
    ax, ay, al = DIRS[a]
    bx, by, bl = DIRS[b]
    return (ax * bx + ay * by) / (al * bl) > -0.01


TURN_OK90 = [[_turn_ok90(a, b) for b in range(8)] for a in range(NSTATE)]
RIGHT_ANGLE_COST = 6.0   # extra cost of a 90 degree corner (only in the last-resort search)
DIR_INDEX = {(dx, dy): i for i, (dx, dy, _) in enumerate(DIRS)}
REV = [DIR_INDEX[(-dx, -dy)] for (dx, dy, _) in DIRS]


def branch_masks(paths: List[List[int]], skip: Set[int], plane: int, nx: int) -> Dict[int, int]:
    """For every cell on the given routes (except `skip`, e.g. pad cells): a bitmask of
    the directions a NEW branch may leave/enter it by without an acute (< 90 deg)
    wedge against the copper already there - a T, never a sharp Y."""
    legs: Dict[int, Set[Tuple[int, int]]] = {}
    for p in paths:
        for i, c in enumerate(p):
            if c in skip:
                continue
            l, r = divmod(c, plane)
            y, x = divmod(r, nx)
            for j in (i - 1, i + 1):
                if 0 <= j < len(p) and p[j] // plane == l:
                    rj = p[j] % plane
                    legs.setdefault(c, set()).add((rj % nx - x, rj // nx - y))
    out = {}
    for c, ls in legs.items():
        m = 0
        for d, (dx, dy, _) in enumerate(DIRS):
            if all(dx * lx + dy * ly <= 0 for (lx, ly) in ls):
                m |= 1 << d
        out[c] = m
    return out


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
    widths: List[float] = field(default_factory=list)   # per segment (net-class width), mm
    why: List[str] = field(default_factory=list)        # why connections failed (if any)

    def width(self, k: int, default: float) -> float:
        return self.widths[k] if k < len(self.widths) else default


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
            lines.append("Removed to keep the board legal: " + ", ".join(self.removed)
                         + " (still fighting other nets for the same space; try routing it on its own, "
                         "or move parts / free up room)")
        bad = [n.name for n in self.nets.values() if not n.complete and n.name not in self.removed]
        if bad:
            lines.append("Incomplete: " + ", ".join(bad))
            for n in bad:
                for w in self.nets[n].why[:3]:
                    lines.append(f"  {n} - {w}")
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
        # part of occ that is only keep-out round fine-pitch escape stubs: two keep-outs may
        # overlap, a keep-out and another net's track may not
        self.kocc = [0] * g.size
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
        self.global_router = None
        self._corridors: Dict[Tuple[int, int], Optional[bytearray]] = {}
        self.corridor_misses = 0
        self.fp_xy = problem.via_footprint
        self._squares: Dict[int, List[int]] = {}
        self.pool = None
        self.worker_seconds = 0.0
        self._routed_ids: Set[int] = set()
        self.stalled = False
        self._retries: Dict[int, int] = {}
        self._diag_cache: Dict[tuple, str] = {}
        self.pool_dirty: Set[int] = set()
        self.cores_used = 1

    # ------------------------------------------------------------------ net classes
    def _square(self, r: int) -> List[int]:
        """Cell offsets of a (2r+1)^2 square, without the centre."""
        sq = self._squares.get(r)
        if sq is None:
            nx = self.pb.grid.nx
            sq = [dy * nx + dx for dy in range(-r, r + 1) for dx in range(-r, r + 1) if dx or dy]
            self._squares[r] = sq
        return sq

    def _is_wide(self, job: NetJob, c: int) -> bool:
        """Does the track run at its full net-class width through cell c?"""
        if not job.claim_r:
            return False
        nx = self.pb.grid.nx
        return not (job.neck[c // nx] >> (c % nx)) & 1

    def _claims(self, job: NetJob, c: int):
        if self._is_wide(job, c):
            return [c] + [c + o for o in self._square(job.claim_r)]
        return [c]

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
               source_dirs: Optional[Dict[int, int]] = None, allowed: Optional[bytearray] = None,
               branch: Optional[Dict[int, int]] = None,
               goal_box: Optional[Tuple[int, int, int, int]] = None, right_angles: bool = False):
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
        wide = job.claim_r > 0
        if wide:
            wok, neckr = job.wide_ok, job.neck
            hb, HX, hpl = job.half_bad, job.half_x, job.half_plane
            sq = self._square(job.claim_r)
            sq_w = 1.0 / (2 * job.claim_r + 1)

        if bbox is None:
            bx0, by0, bx1, by1 = 0, 0, nx - 1, ny - 1
        else:
            bx0, by0, bx1, by1 = bbox

        if target_layers is not None:
            tlayers = set(target_layers)
            layer_only = True
            tx0 = tx1 = ty0 = ty1 = 0
            if goal_box is not None:          # aim at where the goal cells are
                tx0, ty0, tx1, ty1 = goal_box
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
        escapes = job.escapes

        def esc_cost(c):
            """Congestion price of the keep-out round an escape stub - a route ending or
            starting on that stub claims it, so it must pay for it like for its own cells
            (else two nets kept swapping a stub and a track without ever learning)."""
            tot = 0.0
            for q in escapes[c][2]:
                if q in own:
                    continue
                o_ = occ[q]
                h_ = hist[q]
                if o_ or h_:
                    tot += (1.0 + h_) * (1.0 + pf * o_) - 1.0
            return tot

        def h(x, y, l):
            if layer_only:
                if goal_box is None:
                    return 0.0 if l in tlayers else layer_h
                dx = tx0 - x if x < tx0 else (x - tx1 if x > tx1 else 0)
                dy = ty0 - y if y < ty0 else (y - ty1 if y > ty1 else 0)
                if dx < dy:
                    dx, dy = dy, dx
                return dx + (SQ2 - 1.0) * dy
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
            g0 = esc_cost(c) if (escapes and c in escapes) else 0.0
            gbest[st] = g0
            came[st] = -1
            heap.append((g0 + h(x, y, l), g0, st))
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
                # joining existing copper of this net: only at a right angle or wider
                if not (branch and ad < 8 and c in branch and not (branch[c] >> REV[ad]) & 1):
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
            ok_turn = (TURN_OK90 if right_angles else TURN_OK)[ad]
            bmask = branch.get(c, 255) if (branch and ad == START_DIR) else 255
            for d in range(8):
                if not ok_turn[d] or not (bmask >> d) & 1:
                    continue
                dx, dy, ln = DIRS[d]
                x2, y2 = x + dx, y + dy
                if x2 < bx0 or x2 > bx1 or y2 < by0 or y2 > by1:
                    continue
                if allowed is not None and not allowed[y2 * nx + x2]:
                    continue        # outside this net's global-routing corridor
                n2 = c + dy * nx + dx
                o = owner[n2]
                if o != FREE and o != nid:
                    continue
                if n2 in own:
                    cost = ln
                else:
                    cost = ln * (1.0 + hist[n2]) * (1.0 + pf * occ[n2])
                if wide:        # net-class width: the cells around the centre-line too
                    row = n2 // nx
                    bit = 1 << x2
                    if not wok[row] & bit:
                        continue
                    c_neck = (neckr[c // nx] >> x) & 1
                    if neckr[row] & bit or c_neck:
                        # a narrowed step: only worth it when there is no full-width way in
                        cost += NECK_COST * ln
                    if not neckr[row] & bit:
                        # full width on both ends: the step's middle must clear fixed copper too
                        if not c_neck and hb[l * hpl + (2 * y + dy) * HX + 2 * x + dx]:
                            continue
                        extra = 0.0
                        for off in sq:
                            s_ = n2 + off
                            if s_ in own:
                                continue
                            o_ = occ[s_]
                            h_ = hist[s_]
                            if o_ or h_:
                                extra += (1.0 + h_) * (1.0 + pf * o_) - 1.0
                        cost += extra * ln * sq_w
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
                    if right_angles and not TURN_OK[ad][d]:
                        cost += RIGHT_ANGLE_COST
                if escapes and n2 in escapes and (n2 in targets if not layer_only else goal[n2]):
                    cost += esc_cost(n2)          # arriving on a stub: its keep-out too
                    # and turning onto the stub sharper than 90 degrees makes a sharp corner
                    e_ = escapes[n2]
                    sx, sy = g.center(x2, y2)
                    if dx * (e_[0] - sx) + dy * (e_[1] - sy) < -1e-9:
                        cost += SHARP_STUB_COST
                if ad == START_DIR and escapes and c in escapes:
                    e_ = escapes[c]               # leaving a stub: same rule the other way
                    sx, sy = g.center(x, y)
                    if dx * (sx - e_[0]) + dy * (sy - e_[1]) < -1e-9:
                        cost += SHARP_STUB_COST
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
                    if wide and not (wok[n2 // nx] >> x) & 1:
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
    def _path_resources(self, path: List[int], job: Optional[NetJob] = None) -> Set[int]:
        g = self.pb.grid
        plane, nx = g.plane, g.nx
        res = set(path)
        if job is not None and job.claim_r:
            for c in path:
                if self._is_wide(job, c):
                    res.update(self._claims(job, c))
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

    def _flood(self, job: NetJob, start: Set[int], is_goal, limit: int):
        """Cells a track/via of this net may legally use, reachable from start (other
        routes ignored).  Returns (reached the goal, cells seen, {owner: wall cells})."""
        pb, g = self.pb, self.pb.grid
        nx, ny, plane, nl = g.nx, g.ny, g.plane, g.nl
        owner, nid = pb.owner, job.id
        seen = set(start)
        todo = list(start)
        walls: Dict[int, int] = {}
        k = 0
        while k < len(todo) and len(seen) < limit:
            c = todo[k]
            k += 1
            if is_goal(c) and c not in start:
                return True, len(seen), walls
            l, r = divmod(c, plane)
            y, x = divmod(r, nx)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                x2, y2 = x + dx, y + dy
                if not (0 <= x2 < nx and 0 <= y2 < ny):
                    continue
                n2 = c + dy * nx + dx
                if n2 in seen:
                    continue
                o = owner[n2]
                if o != FREE and o != nid:
                    walls[o] = walls.get(o, 0) + 1
                    continue
                seen.add(n2)
                todo.append(n2)
            if nl > 1 and self._via_ok(nid, r, x, y):
                for l2 in range(nl):
                    n2 = l2 * plane + r
                    if n2 not in seen and owner[n2] in (FREE, nid):
                        seen.add(n2)
                        todo.append(n2)
        return False, len(seen), walls

    def _hopeless(self, job: NetJob, a: Set[int], b: Set[int], limit: int = 6000) -> bool:
        """Quick test before the costly whole-board searches: is one end walled in by
        fixed copper, so no route can exist?  Floods a little way from each end; an end
        whose whole reachable area is small and does not contain the other end is stuck."""
        for s_, o_ in ((a, b), (b, a)):
            reached, n, _ = self._flood(job, s_, lambda c, o_=o_: c in o_, limit)
            if reached:
                return False
            if n < limit:
                return True
        return False

    def _diagnose(self, job: NetJob, start: Set[int], is_goal, label: str,
                  other=None, other_label: str = "", limit: int = 0) -> str:
        """Why could this connection not be made?  Flood out from each end over every
        cell a track or via of this net may legally use, ignoring the other routes.
        The end that cannot get out is walled in by fixed copper: name that copper."""
        key = (job.id, label, len(start), len(other or ()))
        if key in self._diag_cache:          # fixed copper only: the answer never changes
            return self._diag_cache[key]
        msg = self._diagnose_now(job, start, is_goal, label, other, other_label, limit)
        self._diag_cache[key] = msg
        return msg

    def _diagnose_now(self, job, start, is_goal, label, other, other_label, limit) -> str:
        limit = limit or self.pb.grid.size + 1
        reached, n_a, walls = self._flood(job, start, is_goal, limit)
        if reached:
            return (f"{label or 'a pad'}: there is room to reach the rest of the net, but other "
                    "nets' new routes took it - try more passes, or route this net on its own first (Selected)")
        if other is not None:                # the trapped end is the one with less room
            st = set(start)
            _, n_b, walls_b = self._flood(job, other, lambda c: c in st, limit)
            if n_b < n_a:
                walls, label = walls_b, other_label
        names = {i: n for n, i in self.pb.net_ids.items()}
        top = sorted(((v, o) for o, v in walls.items()), reverse=True)
        parts = [("board edge / keep-out / overlapping clearances" if o == BLOCKED else names.get(o, "?"))
                 for _, o in top[:4]]
        tracks = [n for n in parts if n in self.pb.track_nets]
        return (f"{label or 'a pad'}: walled in by other nets' copper ({', '.join(parts)}) - on the routing "
                f"grid ({self.pb.grid.pitch:.2f} mm) no track or via can get out. "
                + (f"Existing tracks of {', '.join(tracks)} are part of the wall: move or remove them "
                   "(or tick 'Remove existing tracks'). " if tracks else
                   "It is pads: the gaps are narrower than the grid can use - route this one by hand."))

    def _book_keepout(self, job: NetJob) -> None:
        """Book the keep-out round the escape stubs this net's routes end on."""
        if not job.escapes:
            return
        keep = set()
        for p in job.paths:
            for end in (p[0], p[-1]):
                e = job.escapes.get(end)
                if e is not None:
                    keep.update(e[2])
        keep -= job.resources
        job.keepout = keep
        occ, kocc = self.occ, self.kocc
        for c in keep:
            occ[c] += 1
            kocc[c] += 1

    def _bookable(self, job: NetJob, cells: Set[int]) -> Set[int]:
        """The cells a route occupies for the congestion count: all of them.  Even cells
        in the net's own pads and clearance halo count - no other net's track may enter
        them, but another net's via or wide track may CLAIM them, and must then see the
        conflict (a via next to a pad could otherwise end up too close to its track)."""
        return set(cells)

    def _rip_up(self, job: NetJob) -> None:
        occ, kocc = self.occ, self.kocc
        for c in job.resources:
            occ[c] -= 1
        for c in job.keepout:
            occ[c] -= 1
            kocc[c] -= 1
        job.keepout = set()
        job.resources = set()
        job.fail_notes = []
        job.paths = []
        job.failed = 0

    def _route_net(self, job: NetJob, pass_no: int) -> None:
        self._routed_ids.add(job.id)
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
            # branching off the middle of an earlier route: at a right angle or wider
            br = branch_masks(job.paths, pad_cells | set(sdirs), plane, nx) if job.paths else None
            path = None
            corridor = self._corridor(job, pass_no)
            if corridor is not None:          # first try inside the global route's corridor
                path, explored = self._astar(job, tree, tgt, None, own, source_dirs=sdirs, allowed=corridor,
                                             branch=br)
                if path is None:
                    self.corridor_misses += 1
            hopeless = path is None and self._hopeless(job, tgt, tree)
            if path is None and not hopeless:
                path, explored = self._astar(job, tree, tgt, bbox, own, source_dirs=sdirs, branch=br)
            # a tight escape (e.g. between fine-pitch pins) may need one 90 degree corner, which
            # the 45-degree rule forbids: try that nearby first - it is cheap, a failed
            # whole-board search is not (two of them took 60 s for one pin on a 4-layer board)
            if path is None and not hopeless:
                path, explored = self._astar(job, tree, tgt, bbox, own, branch=br, right_angles=True)
            if path is None and not hopeless:
                path, explored = self._astar(job, tree, tgt, None, own, source_dirs=sdirs, branch=br)
            if path is None and not hopeless:
                path, explored = self._astar(job, tree, tgt, None, own, right_angles=True)
            if path is None:
                job.failed += 1
                job.fail_notes.append(self._diagnose(
                    job, tgt, lambda c, tr=tree: c in tr, terms[k].label, other=set(tree),
                    other_label=" / ".join(sorted({t_.label for t_ in terms if t_.cells & tree and t_.label}))))
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
            res = self._path_resources(path, job)
            own |= res
            tree |= set(path)
            tree |= tgt
            tree_pts.append((terms[k].cx, terms[k].cy))
            self._emit("connection", **{"pass": pass_no, "net": job.name, "path": path,
                                        "explored": explored})
            if self._cancel:
                job.failed += len(left)          # stopped part-way: not complete
                break
        job.resources = self._bookable(job, own)
        for c in job.resources:
            self.occ[c] += 1
        self._book_keepout(job)

    def _route_fanout(self, job: NetJob, pass_no: int) -> None:
        """Pour net: connect every pad the pour does not reach to the pour.

        Tree-style: the goal starts as the pour's main region and grows with every
        route, so a later pad may join an earlier pad's track instead of running its
        own all the way to the pour.  A pad on another layer drops a via; a pad on
        the pour layer but outside the zone's outline gets a track into the zone.
        If the pour has no usable area at all, the pads are joined to each other."""
        g = self.pb.grid
        plane, nx = g.plane, g.nx
        own: Set[int] = set()
        ny = g.ny
        goal = bytearray(self._pour_mask(job))
        terms = list(job.terminals)
        pad_cells: Set[int] = set()
        for t_ in terms:
            pad_cells |= t_.cells
        if not any(goal):                       # no pour to reach: join the pads instead
            first = terms.pop(0)
            for c in first.cells:
                goal[c] = 1
        all_layers = set(range(g.nl))
        # nearest pads first, so later ones can share their tracks
        zm = self.pb.zone_masks.get(job.id)
        # bounding box (x/y) of the goal cells: the search heads for it instead of
        # spreading in every direction (a small pour far away made each pad's search
        # cover most of the board)
        gx0 = gy0 = 10 ** 9
        gx1 = gy1 = -1
        i = goal.find(1)
        while i >= 0:
            j = goal.find(0, i)                # [i, j) is a run of goal cells
            last = (j if j >= 0 else g.size) - 1
            ra, rb = i % plane, last % plane
            ya, yb = ra // nx, rb // nx
            if i // plane != last // plane or ya != yb:     # wraps a row: whole width
                gx0, gx1 = 0, nx - 1
                gy0, gy1 = (0, ny - 1) if i // plane != last // plane else (min(gy0, ya), max(gy1, yb))
            else:
                gx0, gx1 = min(gx0, ra % nx), max(gx1, rb % nx)
                gy0, gy1 = min(gy0, ya), max(gy1, ya)
            if j < 0:
                break
            i = goal.find(1, j)
        goal_box = (gx0, gy0, gx1, gy1) if gx1 >= 0 else None

        def dist_to_goal(t):
            r0 = next(iter(t.cells)) % plane
            x0, y0 = r0 % nx, r0 // nx
            if goal_box is None:
                return 0
            bx0, by0, bx1, by1 = goal_box
            return max(bx0 - x0, 0, x0 - bx1) + max(by0 - y0, 0, y0 - by1)
        terms.sort(key=dist_to_goal)
        for ti, t in enumerate(terms):
            if any(goal[c] for c in t.cells):    # already touches the tree / pour
                continue
            xs = [(c % plane) % nx for c in t.cells]
            ys = [(c % plane) // nx for c in t.cells]
            m = 6 + 2 * pass_no
            bbox = (max(0, min(xs) - m), max(0, min(ys) - m),
                    min(g.nx - 1, max(xs) + m), min(g.ny - 1, max(ys) + m))
            br = branch_masks(job.paths, pad_cells, plane, nx) if job.paths else None
            path, explored = self._astar(job, t.cells, None, bbox, own, all_layers, goal, branch=br,
                                         goal_box=goal_box)
            hopeless = False
            if path is None:                  # walled in? (a small flood says so cheaply)
                r_, n_, _ = self._flood(job, set(t.cells), lambda c, gl=goal: gl[c], 6000)
                hopeless = not r_ and n_ < 6000
            if path is None and not hopeless:
                path, explored = self._astar(job, t.cells, None, bbox, own, all_layers, goal, branch=br,
                                             goal_box=goal_box, right_angles=True)
            if path is None and not hopeless:
                path, explored = self._astar(job, t.cells, None, None, own, all_layers, goal, branch=br,
                                             goal_box=goal_box)
            if path is None and not hopeless:
                path, explored = self._astar(job, t.cells, None, None, own, all_layers, goal, goal_box=goal_box,
                                             right_angles=True)
            if path is None:
                job.failed += 1
                job.fail_notes.append(self._diagnose(job, t.cells, lambda c, gl=goal: gl[c], t.label))
            else:
                job.paths.append(path)
                own |= self._path_resources(path, job)
                for c in path:                   # the tree grows: later pads may join here
                    goal[c] = 1
                    if goal_box is not None:
                        r_ = c % plane
                        goal_box = (min(goal_box[0], r_ % nx), min(goal_box[1], r_ // nx),
                                    max(goal_box[2], r_ % nx), max(goal_box[3], r_ // nx))
                for c in t.cells:
                    goal[c] = 1
                    if goal_box is not None:
                        r_ = c % plane
                        goal_box = (min(goal_box[0], r_ % nx), min(goal_box[1], r_ // nx),
                                    max(goal_box[2], r_ % nx), max(goal_box[3], r_ // nx))
            self._emit("connection", **{"pass": pass_no, "net": job.name, "path": path,
                                        "explored": explored})
            if self._cancel:
                job.failed += len(terms) - ti - 1   # stopped part-way: not complete
                break
        nid = job.id
        job.resources = self._bookable(job, own)
        for c in job.resources:
            self.occ[c] += 1
        self._book_keepout(job)

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
        zmask = self.pb.zone_masks.get(zid)
        for xy in range(plane):
            c = base + xy
            if zmask is not None and not zmask[c]:
                continue                      # outside the zone outline: no pour there
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
            # x/y columns where this net's vias (existing, and its own new ones) join layers
            vcols = set(pb.zone_vias.get(zid, ()))
            if zjob is not None:
                for p_ in zjob.paths:
                    for a_, b_ in zip(p_, p_[1:]):
                        if a_ // plane != b_ // plane:
                            vcols.add(a_ % plane)
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
                    if len(zl) > 1 and any(label[v] == lab for v in vcols):
                        continue          # a via / through-hole pad joins it to the other pour layers
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

    def _corridor(self, job: NetJob, pass_no: int) -> Optional[bytearray]:
        """The detailed search area the global route allows this net (None = no limit).
        It widens by one region every 3 passes, so stubborn conflicts get room."""
        if self.global_router is None:
            return None
        margin = 1 + (pass_no - 1) // 3
        key = (job.id, margin)
        if key not in self._corridors:
            self._corridors[key] = self.global_router.corridor_mask(job, margin)
        return self._corridors[key]

    def run(self) -> RouteResult:
        pb, s = self.pb, self.pb.settings
        jobs = sorted(pb.nets, key=lambda j: (j.hpwl(), j.name))
        if s.global_routing and pb.nets:
            t0 = time.time()
            from .global_route import GlobalRouter
            self.global_router = GlobalRouter(pb, s.region_cells)
            self.global_router.run()
            gr = self.global_router
            self._emit("global_done", **{"regions": gr.nreg, "passes": gr.passes, "overflow": gr.overflow,
                                         "seconds": round(time.time() - t0, 3)})
        converged = False
        pour_stuck = 0
        pour_warning = False
        to_route = list(jobs)
        self._start_pool(len(jobs))
        try:
            return self._passes(jobs, to_route, converged, pour_stuck, pour_warning)
        finally:
            if self.pool is not None:
                self.pool.close()
                self.pool = None

    # ------------------------------------------------------------------ multi-core
    def _start_pool(self, n_nets: int) -> None:
        from . import parallel
        s = self.pb.settings
        n = s.workers if s.workers > 0 else parallel.default_workers(n_nets)
        self.pool = None
        self.cores_used = 1
        if n <= 1:
            return
        t0 = time.time()
        try:
            self.pool = parallel.WorkerPool(self, n)
            self.cores_used = n
            self._emit("workers", **{"workers": n, "seconds": round(time.time() - t0, 2)})
        except Exception as exc:             # fall back to one core, but say why
            self.pool = None
            self._emit("workers", **{"workers": 1, "error": f"{type(exc).__name__}: {exc}"})

    def _route_pass(self, to_route: List[NetJob], pass_no: int) -> None:
        if self.pool is None or pass_no > 1:
            if self.pool is not None:        # the parallel part is over: free the cores
                self.pool.close()
                self.pool = None
            for k, job in enumerate(to_route):
                self._rip_up(job)
                self._route_net(job, pass_no)
                self._emit("net_done", **{"pass": pass_no, "net": job.name,
                                          "index": k + 1, "total": len(to_route)})
                if self._cancel:
                    break
            return
        # Pass 1 on all cores.  In the first pass every net is routed "as if it were
        # alone" (other nets' copper is cheap to cross), so routing them all at the same
        # time changes little - and it is the costliest pass.  The negotiation that makes
        # the result legal (pass 2 on) stays strictly one net at a time.
        by_id = {j.id: j for j in to_route}
        count = [0]

        def on_done(msg):
            if msg is None:
                self._emit("tick")
                return
            _, nid, paths, res, failed, events, busy, notes = msg
            job = by_id[nid]
            self._routed_ids.add(nid)
            job.paths, job.resources, job.failed = paths, res, failed
            job.fail_notes = list(notes)
            for c in res:
                self.occ[c] += 1
            self._book_keepout(job)
            self.worker_seconds += busy
            net_secs[job.name] = busy
            for ev, d in events:
                self._emit(ev, **d)
            count[0] += 1
            self._emit("net_done", **{"pass": pass_no, "net": job.name,
                                      "index": count[0], "total": len(to_route)})
        for job in to_route:
            self._rip_up(job)
        from .parallel import _Stopped
        net_secs: Dict[str, float] = {}
        t_start = time.time()
        try:
            self.pool.route_wave(to_route, pass_no, on_done, set(), True)
            slow = sorted(net_secs.items(), key=lambda kv: -kv[1])[:5]
            self._emit("pass1_stats", **{"cores": self.pool.n, "nets": len(net_secs),
                                         "seconds": round(time.time() - t_start, 1),
                                         "busy": round(sum(net_secs.values()), 1),
                                         "slowest": [(n, round(v, 1)) for n, v in slow]})
        except _Stopped:
            self.pool = None
            return
        except Exception as exc:              # a worker died: finish on one core
            self._emit("workers", **{"workers": 1, "error": f"{type(exc).__name__}: {exc}"})
            self.pool.close()
            self.pool = None
            for job in to_route:
                if job.id in by_id and not job.paths:
                    self._rip_up(job)
                    self._route_net(job, pass_no)

    def _passes(self, jobs, to_route, converged, pour_stuck, pour_warning) -> RouteResult:
        s = self.pb.settings
        best_over, best_pass = 10 ** 9, 0
        for pass_no in range(1, s.max_passes + 1):
            t0 = time.time()
            self._emit("pass_start", **{"pass": pass_no, "pres_fac": self.pres_fac,
                                        "nets": [j.name for j in to_route]})
            self._route_pass(to_route, pass_no)
            over = self._overused()
            for c in over:
                self.hist[c] += s.history_factor * (self.occ[c] - 1)
            conflict = [j for j in jobs if any(c in over for c in j.resources)
                        or any(c in over for c in j.keepout)]
            pour_nets, cut_off = self._pour_check(jobs)
            if cut_off:
                pour_stuck += 1
                if pour_stuck > s.max_pour_passes:   # give up on islands, keep the routing
                    pour_nets, cut_off = [], 0
                    pour_warning = True
            conflict += [j for j in pour_nets if j not in conflict]
            # nets with connections that failed but overlap nothing would never be tried
            # again; the board has changed since, so give them a few more tries
            for j in jobs:
                if j.failed and j not in conflict and self._retries.get(j.id, 0) < MAX_RETRIES:
                    self._retries[j.id] = self._retries.get(j.id, 0) + 1
                    conflict.append(j)
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
            # stagnation: no new low in overused cells for a while - the remaining few nets
            # are fighting over a spot that cannot hold them all (each pass then only gets
            # slower as the prices climb).  Stop and drop the fewest nets instead.
            n_over = len(over)
            if n_over < best_over:
                best_over, best_pass = n_over, pass_no
            elif over and pass_no - best_pass >= STALL_PASSES and len(conflict) <= 10:
                self.stalled = True
                break
            self.pres_fac *= s.present_growth
            to_route = conflict
        if self._cancel:                      # stopped before some nets were ever routed
            for j in jobs:
                if j.id not in self._routed_ids:
                    j.failed = max(j.failed, len(j.terminals) - (0 if j.fanout_layers is not None else 1))
        removed = [] if converged else self._legalize()
        if not self._cancel:
            self._emit("cleanup_start")
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
            mine = job.resources | job.keepout

            def free(c):
                o = owner[c]
                if o != FREE and o != nid:
                    return False
                return occ[c] - (1 if c in mine else 0) <= 0

            wide = job.claim_r > 0

            def legal(cells, l):
                base = l * plane
                for (xa, ya), (xb, yb) in zip(cells, cells[1:]):
                    if not free(base + yb * nx + xb):
                        return False
                    if wide:
                        c = base + yb * nx + xb
                        if not (job.wide_ok[c // nx] >> xb) & 1:
                            return False
                        if not all(free(q) for q in self._claims(job, c)):
                            return False
                        ca = base + ya * nx + xa
                        if self._is_wide(job, c) and self._is_wide(job, ca) and \
                                job.half_bad[l * job.half_plane + (ya + yb) * job.half_x + xa + xb]:
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
            pads_ = set()
            for t_ in job.terminals:
                pads_ |= t_.cells
            # at a junction (not in a pad) the first/last step keeps its direction,
            # so the clean-up cannot turn a T into a sharp Y
            junction = {c for c in fixed if c not in pads_}
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
                    lock_first = run[0] in junction
                    lock_last = run[-1] in junction
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
                                    if lock_first and i == 0 and cand[1] != old[1]:
                                        continue
                                    if lock_last and k == n - 1 and cand[-2] != old[-2]:
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
                res |= self._path_resources(p, job)
            job.resources = self._bookable(job, res)
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
    def path_geometry(self, path: List[int], job: Optional[NetJob] = None):
        """Cells -> (segments, vias, widths) in mm; collinear cells of the same width are
        merged.  A net-class track is drawn at its class width except where it necks
        down next to its own pads."""
        g = self.pb.grid
        segs, vias, widths = [], [], []
        pts = [g.xyl(c) for c in path]
        anchors = self.pb.anchors
        default_w = self.pb.settings.track_width
        if job is not None and job.width:
            full_w, neck_w = job.width, job.neck_width
        else:
            full_w = neck_w = default_w
        wide = [self._is_wide(job, c) for c in path] if job is not None and job.claim_r else None

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
                cur = 0
                key0 = None
                for i in range(len(run) - 1):
                    a, b = run[i], run[i + 1]
                    wi = full_w if (wide is None or (wide[run_start + i] and wide[run_start + i + 1])) else neck_w
                    key = (b[0] - a[0], b[1] - a[1], wi)
                    if key0 is not None and key != key0:
                        segs.append((*mm(run[cur][0], run[cur][1]), *mm(a[0], a[1]), layer))
                        widths.append(key0[2])
                        cur = i
                    key0 = key
                segs.append((*mm(run[cur][0], run[cur][1]), *mm(run[-1][0], run[-1][1]), layer))
                widths.append(key0[2])
            if k < len(pts):
                vias.append(mm(pts[k][0], pts[k][1]))
            run_start = k
        # stubs from the grid cell to the true pad centre
        esc = job.escapes if job is not None else {}
        for end in ({path[0], path[-1]} if len(path) > 1 else {path[0]}):
            a_ = anchors.get(end) or (esc[end][:2] if end in esc else None)
            if a_ is not None:
                x, y, l = g.xyl(end)
                cx, cy = mm(x, y)
                ax, ay = a_
                if math.hypot(ax - cx, ay - cy) > 1e-4:
                    segs.append((cx, cy, ax, ay, g.layers[l]))
                    # an escape stub keeps the width it was checked at (narrower into fine pitch)
                    widths.append(min(neck_w, esc[end][3]) if end in esc and len(esc[end]) > 3 else neck_w)
        return segs, vias, widths

    def _result(self, converged: bool, removed: List[str]) -> RouteResult:
        nets: Dict[str, NetRoute] = {}
        s = self.pb.settings
        for j in self.pb.nets:
            nr = NetRoute(j.name)
            if j.name not in removed and (j.failed == 0 or s.keep_failed_partial):
                seen_s, seen_v = set(), set()
                for p in j.paths:
                    sg, vi, wd = self.path_geometry(p, j)
                    for seg, wi in zip(sg, wd):
                        key = tuple(round(v, 4) if isinstance(v, float) else v for v in seg)
                        rkey = (key[2], key[3], key[0], key[1], key[4])
                        if key not in seen_s and rkey not in seen_s:
                            seen_s.add(key)
                            nr.segments.append(seg)
                            nr.widths.append(wi)
                    for v in vi:
                        key = (round(v[0], 4), round(v[1], 4))
                        if key not in seen_v:
                            seen_v.add(key)
                            nr.vias.append(v)
            nr.complete = j.failed == 0 and j.unreachable == 0 and j.name not in removed
            nr.failed_connections = j.failed + j.unreachable
            if j.fail_notes and not nr.complete:
                nr.why = sorted(set(j.fail_notes))
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
