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
from array import array
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
RESCUE_SECONDS = 3.0      # per dropped net: time to try routing orders in the rescue pass
H_WEIGHT = 1.2           # weighted A* after pass 1 (routes at most 1.2x the cheapest)
PRES_FAC_MAX = 40.0      # cap on the present-congestion factor
STALL_PASSES = 15     # stop when the overuse has not reached a new low for this many passes
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
    vias: Set[int] = set()
    for p in paths:
        for i, c in enumerate(p):
            if c in skip:
                continue
            l, r = divmod(c, plane)
            y, x = divmod(r, nx)
            for j in (i - 1, i + 1):
                if 0 <= j < len(p):
                    if p[j] // plane == l:
                        rj = p[j] % plane
                        legs.setdefault(c, set()).add((rj % nx - x, rj // nx - y))
                    else:
                        vias.add(c)
    out = {}
    for c, ls in legs.items():
        m = 0
        lone = len(ls) == 1 and c not in vias
        for d, (dx, dy, _) in enumerate(DIRS):
            if lone:
                # the plain END of a track: a new track joining here continues it, so the
                # bend must stay at 45 degrees or less (joining at 90 made an L-corner)
                (lx, ly), = ls
                if (dx * lx + dy * ly) / (math.hypot(dx, dy) * math.hypot(lx, ly)) <= -0.7071:
                    m |= 1 << d
            elif all(dx * lx + dy * ly <= 0 for (lx, ly) in ls):
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
            lines.append("Removed to keep the board legal (tried again on the finished board - no room): "
                         + ", ".join(self.removed))
            for n in self.removed:
                for w in self.nets[n].why[:1]:
                    lines.append(f"  {n} - {w}")
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
        self.occ = array('i', [0]) * g.size       # typed arrays: the compiled search reads them in place
        # part of occ that is only keep-out round fine-pitch escape stubs: two keep-outs may
        # overlap, a keep-out and another net's track may not
        self.kocc = array('i', [0]) * g.size
        # part of occ that is only the side cells of 45-degree steps: two of those may share a
        # cell (the only way is two parallel diagonals, 0.42 pitch apart - legal), a side
        # cell and another net's track or stub keep-out may not
        self.cocc = array('i', [0]) * g.size
        self.hist = array('d', [0.0]) * g.size
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
        self.native = None
        if getattr(problem.settings, "native", True):
            try:
                from . import native as _nat
                lib = _nat.load()
                if lib is not None:
                    if not isinstance(problem.owner, array):
                        problem.owner = array('i', problem.owner)
                    if not isinstance(problem.via_owner, array):
                        problem.via_owner = array('i', problem.via_owner)
                    self.native = _nat.NativeSearch(lib, g, problem.owner, problem.via_owner, self.occ, self.hist,
                                                    self.dir_mult, problem.via_footprint)
                    self.native.set_corners(self.cocc, getattr(problem, "diag_owner", None))
            except Exception:
                self.native = None
        self._squares: Dict[int, List[int]] = {}
        self.pool = None
        self.worker_seconds = 0.0
        self._routed_ids: Set[int] = set()
        self.stalled = False
        self.h_weight = 1.0
        self._retries: Dict[int, int] = {}
        self.fought: Dict[str, List[str]] = {}    # dropped net -> nets it was fighting with
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
    def _wide_args(self, job):
        """Net-class width data for the compiled search (cached on the job)."""
        import ctypes
        from . import native as nat
        if not job.claim_r:
            return (0, None, None, None, 0, 0, None, 0, 0.0, 0.0)
        cache = job.__dict__.get("_wide_c")
        if cache is None:
            sq = self._square(job.claim_r)
            a_sq, n_sq = nat.ints(sq)
            wok = (ctypes.c_ubyte * len(job.wide_ok_b)).from_buffer_copy(job.wide_ok_b)
            neck = (ctypes.c_ubyte * len(job.neck_b)).from_buffer_copy(job.neck_b)
            hb = (ctypes.c_ubyte * len(job.half_bad)).from_buffer_copy(bytes(job.half_bad))
            cache = (1, wok, neck, hb, job.half_x, job.half_plane, a_sq, n_sq,
                     1.0 / (2 * job.claim_r + 1), NECK_COST)
            job._wide_c = cache
        return cache

    def _astar_native(self, job, sources, targets, bbox, own, target_layers, goal, source_dirs,
                      allowed, branch, goal_box, right_angles):
        """The same search in compiled code (native/astar.c).  None = fall back to Python."""
        import ctypes
        from . import native as nat
        g = self.pb.grid
        nx, plane, nl = g.nx, g.plane, g.nl
        occ, hist, pf = self.occ, self.hist, self.pres_fac
        escapes = job.escapes
        layer_only = target_layers is not None

        def esc_cost(c):
            tot = 0.0
            for q in escapes[c][2]:
                if q in own:
                    continue
                o_, h_ = occ[q], hist[q]
                if o_ or h_:
                    tot += (1.0 + h_) * (1.0 + pf * o_) - 1.0
            return tot

        def first_pt(c):
            e_ = escapes[c]
            return e_[4][-1] if len(e_) > 4 and e_[4] else (e_[0], e_[1])
        src = list(sources)
        sd = [source_dirs.get(c, START_DIR) for c in src] if source_dirs else None
        g0 = [esc_cost(c) if c in escapes else 0.0 for c in src] if escapes else None
        if layer_only:
            tl = list(target_layers)
            tg = []
        else:
            tg = list(targets)
            tl = {t // plane for t in tg}
        mask = 0
        for l in tl:
            mask |= 1 << l
        e_in, e_x, e_bad, e_out, e_obad = [], [], [], [], []
        if escapes:
            for c in escapes:
                if (goal[c] if layer_only else c in targets):
                    sx, sy = g.center(*g.xyl(c)[:2])
                    fx, fy = first_pt(c)
                    m = 0
                    for d, (dx, dy, _) in enumerate(DIRS):
                        if dx * (fx - sx) + dy * (fy - sy) < -1e-9:
                            m |= 1 << d
                    e_in.append(c)
                    e_x.append(esc_cost(c))
                    e_bad.append(m)
                if c in sources:
                    sx, sy = g.center(*g.xyl(c)[:2])
                    fx, fy = first_pt(c)
                    m = 0
                    for d, (dx, dy, _) in enumerate(DIRS):
                        if dx * (sx - fx) + dy * (sy - fy) < -1e-9:
                            m |= 1 << d
                    e_out.append(c)
                    e_obad.append(m)
        bx0, by0, bx1, by1 = bbox if bbox is not None else (0, 0, nx - 1, g.ny - 1)
        ns = self.native
        if self.record and ns.expl_cap == 0:
            ns.expl_cap = 200000
            ns.expl = (ctypes.c_int * ns.expl_cap)()
        a_src, n_src = nat.ints(src)
        a_tg, n_tg = nat.ints(tg)
        a_own, n_own = nat.ints(own)
        bm_c = list(branch) if branch else []
        a_bm, n_bm = nat.ints(bm_c)
        a_ein, n_ein = nat.ints(e_in)
        a_eout, n_eout = nat.ints(e_out)
        gb = goal_box if (layer_only and goal_box is not None) else (0, 0, 0, 0)
        n = ns.lib.astar_run(
            ns.ctx, job.id, a_src, nat.ints(sd)[0] if sd else None, nat.dbls(g0) if g0 else None, n_src,
            a_tg, n_tg, nat.view(goal, ctypes.c_ubyte) if layer_only else None, 1 if layer_only else 0, mask,
            bx0, by0, bx1, by1, nat.view(allowed, ctypes.c_ubyte) if allowed is not None else None,
            a_own, n_own, a_bm, nat.u8s([branch[c] for c in bm_c]) if bm_c else None, n_bm,
            a_ein, nat.dbls(e_x), nat.u8s(e_bad), n_ein, a_eout, nat.u8s(e_obad), n_eout,
            pf, self.pb.settings.bend_cost, self.pb.settings.via_cost, self.h_weight,
            1 if right_angles else 0, RIGHT_ANGLE_COST, SHARP_STUB_COST,
            1 if (layer_only and goal_box is not None) else 0, gb[0], gb[1], gb[2], gb[3],
            ns.out, ns.path_cap, ns.expl if self.record else None, ns.expl_cap, ctypes.byref(ns.n_expl),
            *self._wide_args(job))
        if n < 0:
            return None                       # out of memory / too long: let Python do it
        explored = list(ns.expl[:ns.n_expl.value]) if self.record else None
        if n == 0:
            return None, explored
        return list(ns.out[:n]), explored

    def _astar(self, job: NetJob, sources: Set[int], targets: Optional[Set[int]],
               bbox: Optional[Tuple[int, int, int, int]], own: Set[int],
               target_layers: Optional[Set[int]] = None, goal: Optional[bytearray] = None,
               source_dirs: Optional[Dict[int, int]] = None, allowed: Optional[bytearray] = None,
               branch: Optional[Dict[int, int]] = None,
               goal_box: Optional[Tuple[int, int, int, int]] = None, right_angles: bool = False):
        """Cheapest path from any source cell to any target cell.

        With ``target_layers`` instead of ``targets`` any usable cell on those
        layers is a goal (fan-out of a pad to a plane layer)."""
        if self.native is not None and (job.claim_r == 0 or job.wide_ok_b is not None):
            res = self._astar_native(job, sources, targets, bbox, own, target_layers, goal, source_dirs,
                                     allowed, branch, goal_box, right_angles)
            if res is not None:
                return res
        pb, g, s = self.pb, self.pb.grid, self.pb.settings
        nx, ny, plane, nl = g.nx, g.ny, g.plane, g.nl
        owner, occ, hist, cocc = pb.owner, self.occ, self.hist, self.cocc
        dg = getattr(pb, "diag_owner", None)
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
        hw_ = self.h_weight
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
                return (dx + (SQ2 - 1.0) * dy) * hw_
            dx = tx0 - x if x < tx0 else (x - tx1 if x > tx1 else 0)
            dy = ty0 - y if y < ty0 else (y - ty1 if y > ty1 else 0)
            if dx < dy:
                dx, dy = dy, dx
            v = dx + (SQ2 - 1.0) * dy
            if layer_h and l not in tlayers:
                v += layer_h
            return v * hw_

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
                    if dg is not None:       # the step's middle must clear fixed copper
                        od = dg[l * plane + (y if dy > 0 else y2) * nx + (x if dx > 0 else x2)]
                        if od != FREE and od != nid:
                            continue
                    else:
                        o1, o2 = owner[s1], owner[s2]
                        if (o1 != FREE and o1 != nid) or (o2 != FREE and o2 != nid):
                            continue
                    if s1 not in own:
                        cost += (1.0 + hist[s1]) * (1.0 + pf * (occ[s1] - cocc[s1])) - 1.0
                    if s2 not in own:
                        cost += (1.0 + hist[s2]) * (1.0 + pf * (occ[s2] - cocc[s2])) - 1.0
                if ad < 8 and ad != d:
                    cost += bend
                    if right_angles and not TURN_OK[ad][d]:
                        cost += RIGHT_ANGLE_COST
                if escapes and n2 in escapes and (n2 in targets if not layer_only else goal[n2]):
                    cost += esc_cost(n2)          # arriving on a stub: its keep-out too
                    # and turning onto the stub sharper than 90 degrees makes a sharp corner
                    e_ = escapes[n2]
                    sx, sy = g.center(x2, y2)
                    fx, fy = e_[4][-1] if len(e_) > 4 and e_[4] else (e_[0], e_[1])   # first point of the stub
                    if dx * (fx - sx) + dy * (fy - sy) < -1e-9:
                        cost += SHARP_STUB_COST
                if ad == START_DIR and escapes and c in escapes:
                    e_ = escapes[c]               # leaving a stub: same rule the other way
                    sx, sy = g.center(x, y)
                    fx, fy = e_[4][-1] if len(e_) > 4 and e_[4] else (e_[0], e_[1])
                    if dx * (sx - fx) + dy * (sy - fy) < -1e-9:
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
        return res

    def _corner_cells(self, job: NetJob) -> Set[int]:
        """Side cells of the 45-degree steps of the net's paths."""
        g = self.pb.grid
        plane, nx = g.plane, g.nx
        out = set()
        for path in job.paths:
            for a, b in zip(path, path[1:]):
                la, ra = divmod(a, plane)
                lb, rb = divmod(b, plane)
                if la != lb:
                    continue
                ya, xa = divmod(ra, nx)
                yb, xb = divmod(rb, nx)
                if xa != xb and ya != yb:
                    out.add(la * plane + ya * nx + xb)
                    out.add(la * plane + yb * nx + xa)
        return out

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
        """Book the keep-out round the escape stubs this net's routes end on, and the
        side cells of its 45-degree steps."""
        occ, cocc = self.occ, self.cocc
        for c in job.corners:
            occ[c] -= 1
            cocc[c] -= 1
        keep = self._stub_keepout(job)
        corners = self._corner_cells(job) - job.resources - keep
        job.corners = corners
        for c in corners:
            occ[c] += 1
            cocc[c] += 1
        if not keep:
            return
        job.keepout = keep
        kocc = self.kocc
        for c in keep:
            occ[c] += 1
            kocc[c] += 1

    def _stub_keepout(self, job: NetJob) -> Set[int]:
        if not job.escapes:
            return set()
        keep = set()
        for p in job.paths:
            for end in (p[0], p[-1]):
                e = job.escapes.get(end)
                if e is not None:
                    keep.update(e[2])
        keep -= job.resources
        return keep

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
        for c in job.corners:
            occ[c] -= 1
            self.cocc[c] -= 1
        job.corners = set()
        job.keepout = set()
        job.resources = set()
        job.fail_notes = []
        job.paths = []
        job.failed = 0

    def _copper_on_pad(self, job: NetJob, t, c: int) -> bool:
        """Does copper at grid cell c (a track through it, or the escape stub drawn from it)
        actually touch one of terminal t's pads?  On fine pitch two pads of a net can share
        a cell, and copper there may belong to the neighbour only."""
        if not t.pads:
            return True
        g = self.pb.grid
        e = job.escapes.get(c)
        if e is not None:
            # an escape cell's stub (to e's pad centre) is drawn only where a route ENDS on
            # it; a route merely passing through the cell does not reach that pad
            if not any(p_[0] == c or p_[-1] == c for p_ in job.paths):
                return False
            px, py = e[0], e[1]
            if any(p_.distance(px, py) < 1e-6 for p_ in t.pads):
                return True
        x, y, _ = g.xyl(c)
        cx, cy = g.center(x, y)
        half = (job.neck_width or self.pb.settings.track_width) / 2
        return any(p_.distance(cx, cy) < half - 1e-3 for p_ in t.pads)

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
            shared = tgt & tree
            if shared:
                # a grid cell this pad shares with a pad already in the tree (0.5 mm pitch
                # on a coarser grid) is not a connection: real copper must reach the pad
                on_paths = {c for p_ in job.paths for c in p_}
                tgt_ = tgt - {c for c in shared
                              if c not in on_paths or not self._copper_on_pad(job, terms[k], c)}
                if tgt_:
                    tgt = tgt_
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
        pour0 = bytes(goal)                     # the pour estimate alone (no tracks)
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
        def touches(t):
            # on the tree / pour, or right beside the POUR (its estimate is shrunk by the
            # zone clearance, so a pad the real pour reaches can sit just outside it).
            # Not near a track: a track passing 2 cells away is not a connection.
            on_paths = {c for p_ in job.paths for c in p_}
            for c in t.cells:
                # a goal cell counts only if copper really is there (a track or via of this
                # net, or the pour): a cell this pad merely SHARES with a neighbouring pad of
                # the net (0.5 mm pitch on a 0.45 mm grid) is no connection
                if goal[c] and (pour0[c] or (c in on_paths and self._copper_on_pad(job, t, c))):
                    return True
                l_, r_ = divmod(c, plane)
                x_, y_ = r_ % nx, r_ // nx
                for yy in range(max(0, y_ - 1), min(ny, y_ + 2)):
                    row = l_ * plane + yy * nx
                    for xx in range(max(0, x_ - 1), min(nx, x_ + 2)):
                        if pour0[row + xx]:
                            return True
            return False
        # a plane on another layer (e.g. GND on B.Cu): each single-layer pad gets its own
        # via right beside it - short return path, and no tracks wandering between pads
        plane_goal = {}
        if self.pb.settings.via_per_pour_pad and len(job.fanout_layers) > 1:
            per_layer = {l: self._pour_mask_layers(job, {l}) for l in job.fanout_layers}
            for l in job.fanout_layers:
                m_ = bytearray(g.size)
                for l2, mk in per_layer.items():
                    if l2 != l:
                        b_ = l2 * plane
                        m_[b_:b_ + plane] = mk[b_:b_ + plane]
                if any(m_):
                    plane_goal[l] = m_
        via_paths: List[List[int]] = []

        def ti_key(t):
            return min(t.cells)

        # this net's own vias may not sit side by side either (hole-to-hole clearance):
        # while a pad is routed, the via spots round the net's vias so far are closed
        s_ = self.pb.settings
        min_vv = s_.via_drill + max(s_.hole_clearance, 0.25) + 0.01   # hole centre to centre
        kv = int(min_vv / g.pitch)
        near_v = [(dx, dy) for dy in range(-kv, kv + 1) for dx in range(-kv, kv + 1)
                  if (dx or dy) and math.hypot(dx, dy) * g.pitch < min_vv]
        via_owner = self.pb.via_owner
        closed: List[Tuple[int, int]] = []

        def close_round_own_vias():
            cols = {a % plane for p_ in job.paths for a, b in zip(p_, p_[1:]) if a // plane != b // plane}
            for col in cols:
                x0_, y0_ = col % nx, col // nx
                for dx, dy in near_v:
                    x_, y_ = x0_ + dx, y0_ + dy
                    if 0 <= x_ < nx and 0 <= y_ < ny:
                        q = y_ * nx + x_
                        if q not in cols and via_owner[q] != BLOCKED:
                            closed.append((q, via_owner[q]))
                            via_owner[q] = BLOCKED

        def reopen():
            while closed:
                q, v = closed.pop()
                via_owner[q] = v

        try:
            self._fanout_terms(job, pass_no, terms, goal, goal_box, own, pad_cells, all_layers,
                               plane_goal, via_paths, ti_key, touches, close_round_own_vias, reopen)
        finally:
            reopen()
        nid = job.id
        job.resources = self._bookable(job, own)
        for c in job.resources:
            self.occ[c] += 1
        self._book_keepout(job)

    def _fanout_terms(self, job, pass_no, terms, goal, goal_box, own, pad_cells, all_layers,
                      plane_goal, via_paths, ti_key, touches, close_round_own_vias, reopen):
        g = self.pb.grid
        plane, nx, ny = g.plane, g.nx, g.ny
        for ti, t in enumerate(terms):
            reopen()
            close_round_own_vias()
            t_layers = {c // plane for c in t.cells}
            if len(t_layers) == 1 and next(iter(t_layers)) in plane_goal and not self._cancel:
                pg = plane_goal[next(iter(t_layers))]
                xs = [(c % plane) % nx for c in t.cells]
                ys = [(c % plane) // nx for c in t.cells]
                m = 4 + pass_no
                bbox = (max(0, min(xs) - m), max(0, min(ys) - m),
                        min(g.nx - 1, max(xs) + m), min(g.ny - 1, max(ys) + m))
                prev = job.__dict__.setdefault("_prev_via", {}).get(ti_key(t))
                if prev is not None and pass_no > 1 and all(
                        (c in own) or not self.occ[c] for c in self._path_resources(prev, job)) and all(
                        self.pb.via_owner[a % plane] != BLOCKED
                        for a, b in zip(prev, prev[1:]) if a // plane != b // plane):
                    # its via from the last pass is in nobody's way: keep it (re-searching
                    # every pad on every pass doubled the time on a board with 100 GND pads)
                    via_paths.append(prev)
                    job.paths.append(prev)
                    own |= self._path_resources(prev, job)
                    continue
                if via_paths:                    # or join a neighbouring pad's via stub
                    pg = bytearray(pg)
                    for vp in via_paths:
                        for c in vp:
                            pg[c] = 1
                path, explored = self._astar(job, t.cells, None, bbox, own, all_layers, pg)
                if path is not None and pass_no > 1 and g.nl <= 2:
                    # 2 layers: a via costs both layers, so signals come first - where the via
                    # would sit on another net's copper, connect this pad the ordinary way
                    # (pour, or a track to it).  With inner planes, ground vias have priority.
                    occ = self.occ
                    if any(occ[c] for c in self._path_resources(path, job) if c not in own):
                        path = None
                if path is not None and (any(a // plane != b // plane for a, b in zip(path, path[1:]))
                                         or any(path[-1] in vp for vp in via_paths)):
                    via_paths.append(path)
                    job._prev_via[ti_key(t)] = path
                    job.paths.append(path)
                    own |= self._path_resources(path, job)
                    self._emit("connection", **{"pass": pass_no, "net": job.name, "path": path,
                                                "explored": explored})
                    continue
            if touches(t):                       # already joined to the tree / pour
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

    # ------------------------------------------------------------------ pours
    def _pour_labels(self, zid: int, layer: int, mine: Set[int]):
        """Estimate a pour on one layer: cells no other net uses, eroded by one
        cell (zone clearance), split into 4-connected regions.

        Returns (label per x/y, size per label, free mask)."""
        # how far the pour keeps from other copper, in hundredths of a cell
        erode = self.pb.zone_erode.get(zid, 101) if getattr(self.pb, "zone_erode", None) else 101
        if self.native is not None and hasattr(self.native.lib, "pour_labels_k"):
            import ctypes
            from . import native as nat
            g = self.pb.grid
            plane = g.plane
            label = array('i', [-1]) * plane
            sizes = array('i', [0]) * plane
            free = bytearray(plane)
            zmask = self.pb.zone_masks.get(zid)
            a_mine, n_mine = nat.ints(mine)
            n = self.native.lib.pour_labels_k(self.native.ctx, layer * plane, zid,
                                              nat.view(zmask, ctypes.c_ubyte) if zmask is not None else None,
                                              a_mine, n_mine, nat.view(label, ctypes.c_int),
                                              nat.view(sizes, ctypes.c_int), nat.view(free, ctypes.c_ubyte),
                                              erode)
            if n >= 0:
                return label, list(sizes[:n]), free
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
        rr = erode / 100.0
        R_ = int(rr) + 1
        offs = [(dx, dy) for dy in range(-R_, R_ + 1) for dx in range(-R_, R_ + 1)
                if (dx or dy) and dx * dx + dy * dy < rr * rr]
        ero = bytearray(plane)
        for y in range(ny):
            row = y * nx
            for x in range(nx):
                xy = row + x
                if not free[xy]:
                    continue
                ok = True
                for dx, dy in offs:
                    x2, y2 = x + dx, y + dy
                    if not (0 <= x2 < nx and 0 <= y2 < ny) or not free[y2 * nx + x2]:
                        ok = False
                        break
                if ok:
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
        return self._pour_mask_layers(job, job.fanout_layers)

    def _pour_mask_layers(self, job: NetJob, layers) -> bytearray:
        g = self.pb.grid
        mask = bytearray(g.size)
        for l in layers:
            label, sizes, _ = self._pour_labels(job.id, l, job.resources | job.corners)
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
            mine = (zjob.resources | zjob.corners) if zjob else set()
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
                cells_of = None
                for xy in la:
                    lab = label[xy]
                    if lab < 0 or lab == main or lab in done_labels:
                        continue          # not judgeable, fine, or already handled
                    done_labels.add(lab)
                    if len(zl) > 1 and any(label[v] == lab for v in vcols):
                        continue          # a via / through-hole pad joins it to the other pour layers
                    if cells_of is None:          # all labels' cells, in one pass
                        cells_of = {}
                        for k_, v_ in enumerate(label):
                            if v_ >= 0 and v_ != main:
                                cells_of.setdefault(v_, []).append(k_)
                    island = cells_of.get(lab, [])
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
        cocc = self.cocc
        for j in self.pb.nets:
            for c in j.resources:
                if occ[c] > 1:
                    out.add(c)
            for c in j.corners:              # a side cell with a track or stub keep-out on it
                if occ[c] > cocc[c]:
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
        if s.global_routing and pb.nets and not self._cancel:
            self._emit("prep", **{"step": "Planning a coarse route for every net...",
                                  "detail": f"global routing: {len(pb.nets)} nets through regions of the board"})
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
        self._emit("prep", **{"step": "Starting the CPU cores...",
                              "detail": "pass 1 routes every net at once, one net per core"})
        self._emit("engine", **{"native": self.native is not None,
                                "planes": [self.pb.grid.layers[l] for l in sorted(getattr(pb, "plane_layers", ()))]})
        if not self._cancel:
            self._start_pool(len(jobs))
        if self._cancel:                      # Stop pressed while getting ready: nothing routed
            for j in jobs:
                j.failed = max(j.failed, len(j.terminals) - (0 if j.fanout_layers is not None else 1))
            result = self._result(False, [])
            self._emit("done", result=result)
            return result
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
        except parallel._Stopped:             # Stop pressed while the cores were starting
            self.pool = None
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
            # pass 1 finds every net's best path alone; afterwards the board is congested and
            # an exact search pays dearly for a tiny gain - lean towards the target
            self.h_weight = 1.0 if pass_no == 1 else H_WEIGHT
            self._route_pass(to_route, pass_no)
            over = self._overused()
            for c in over:
                self.hist[c] += s.history_factor * (self.occ[c] - 1)
            conflict = [j for j in jobs if any(c in over for c in j.resources)
                        or any(c in over for c in j.keepout) or any(c in over for c in j.corners)]
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
            elif over and pass_no - best_pass >= STALL_PASSES:
                self.stalled = True
                break
            # capped: beyond this the price only blinds the search (it stops heading for its
            # target and explores nearly everything - passes got 7x slower on a dense board)
            self.pres_fac = min(self.pres_fac * s.present_growth, PRES_FAC_MAX)
            to_route = conflict
        if self._cancel:                      # stopped before some nets were ever routed
            for j in jobs:
                if j.id not in self._routed_ids:
                    j.failed = max(j.failed, len(j.terminals) - (0 if j.fanout_layers is not None else 1))
        removed = [] if converged else self._legalize()
        if removed and not self._cancel:
            removed = self._rescue(removed, len(self.passes) + 1)
        if not converged and not self._cancel:
            self._rescue_pours(len(self.passes) + 1)
        if not self._cancel and getattr(self, "cleanup_in_run", True):
            # (front ends that straighten the result with restraighten.straighten_result
            # switch this off, so the work is not done twice)
            self._emit("cleanup_start")
            n = self._straighten()
            self._emit("cleanup", **{"straightened": n})
        result = self._result(converged, removed)
        result.pour_warning = pour_warning
        self._emit("done", result=result)
        return result

    # ------------------------------------------------------------------ clean-up
    def _clean_tick(self, stage: str, job: NetJob, i: int) -> None:
        """Clean-up progress for the view: which stage, which net (it is drawn highlighted,
        and the picture shows the routes as they are straightened)."""
        self._emit("smooth", **{"stage": stage, "net": job.name, "index": i + 1,
                                "total": len(self.pb.nets)})

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
        for ji_, job in enumerate(self.pb.nets):
            self._clean_tick('shortcuts', job, ji_)
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

            def octi_legal(a, b, diag_first, l):
                """The octilinear path a -> b (diagonal then straight, or the other way round),
                checked step by step and abandoned at the first blocked cell (most candidates
                fail within a few steps).  None if blocked."""
                (xa, ya), (xb, yb) = a, b
                dx, dy = xb - xa, yb - ya
                sx = (dx > 0) - (dx < 0)
                sy = (dy > 0) - (dy < 0)
                nd = min(abs(dx), abs(dy))
                ns = max(abs(dx), abs(dy)) - nd
                st = (sx, 0) if abs(dx) > abs(dy) else (0, sy)
                runs_ = ((sx, sy, nd), (st[0], st[1], ns)) if diag_first else ((st[0], st[1], ns), (sx, sy, nd))
                out = [a]
                x, y = a
                for (ux, uy, m_) in runs_:
                    for _ in range(m_):
                        x2, y2 = x + ux, y + uy
                        if not legal(((x, y), (x2, y2)), l):
                            return None
                        out.append((x2, y2))
                        x, y = x2, y2
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
            # (only a real T, three or more same-layer branches - not a via)
            nb_: Dict[int, Set[int]] = {}
            for p_ in job.paths:
                for u, v in zip(p_, p_[1:]):
                    if u // plane == v // plane:
                        nb_.setdefault(u, set()).add(v)
                        nb_.setdefault(v, set()).add(u)
            junction = {c for c in fixed if c not in pads_ and len(nb_.get(c, ())) >= 3}
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
                stub_ends = set(job.escapes) | set(self.pb.anchors)
                for run in pieces:
                    l = run[0] // plane
                    pts = [((c % plane) % nx, (c % plane) // nx) for c in run]
                    # junctions and stub ends (escape / nearest-point stubs to a pad) keep the
                    # direction of their first/last step: the stub is part of the track, and
                    # turning there made a corner the shortcut did not see
                    lock_first = run[0] in junction or (run[0] in stub_ends and run[0] == path[0]) \
                        or (run[0] in stub_ends and run[0] == path[-1])
                    lock_last = run[-1] in junction or (run[-1] in stub_ends and run[-1] in (path[0], path[-1]))
                    for _ in range(2):          # two sweeps catch nested zig-zags
                        if len(pts) < 3:
                            break
                        n = len(pts)
                        # running totals along the old run, so the cost of any stretch i..k
                        # (length, bends, forbidden turns) is O(1) instead of a walk
                        dirs = [DIR_INDEX[(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1])]
                                for j in range(n - 1)]
                        Lp = [0.0] * n
                        Bp = [0] * n
                        Tp = [0] * n
                        for j in range(n - 1):
                            Lp[j + 1] = Lp[j] + DIRS[dirs[j]][2]
                        for j in range(1, n - 1):
                            Bp[j] = Bp[j - 1] + (dirs[j] != dirs[j - 1])
                            Tp[j] = Tp[j - 1] + (not TURN_OK[dirs[j - 1]][dirs[j]])
                        out = [pts[0]]
                        i = 0
                        while i < n - 1:
                            prev_d = (DIR_INDEX[(out[-1][0] - out[-2][0], out[-1][1] - out[-2][1])]
                                      if len(out) >= 2 else None)
                            done = False
                            for k in range(min(n - 1, i + 40), i + 1, -1):   # 40: same result as 80, faster
                                next_d = dirs[k] if k + 1 < n else None
                                # cost of the old stretch i..k (as cost() would compute it)
                                if Tp[k - 1] - Tp[i] or (prev_d is not None and not TURN_OK[prev_d][dirs[i]]) \
                                        or (next_d is not None and not TURN_OK[dirs[k - 1]][next_d]):
                                    old_c = 1e30
                                else:
                                    old_c = (Lp[k] - Lp[i]) + bend * (Bp[k - 1] - Bp[i]
                                                                      + (prev_d is not None and prev_d != dirs[i])
                                                                      + (next_d is not None and dirs[k - 1] != next_d))
                                (xa, ya), (xb, yb) = pts[i], pts[k]
                                ddx, ddy = xb - xa, yb - ya
                                sx = (ddx > 0) - (ddx < 0)
                                sy = (ddy > 0) - (ddy < 0)
                                nd = min(abs(ddx), abs(ddy))
                                ns = max(abs(ddx), abs(ddy)) - nd
                                dd = DIR_INDEX[(sx, sy)] if nd else None
                                ds = (DIR_INDEX[(sx, 0)] if abs(ddx) > abs(ddy) else DIR_INDEX[(0, sy)]) if ns else None
                                for diag_first in (True, False):
                                    runs_ = [(dd, nd), (ds, ns)] if diag_first else [(ds, ns), (dd, nd)]
                                    runs_ = [(d_, m_) for (d_, m_) in runs_ if m_]
                                    if not runs_:
                                        continue
                                    if lock_first and i == 0 and runs_[0][0] != dirs[0]:
                                        continue
                                    if lock_last and k == n - 1 and runs_[-1][0] != dirs[n - 2]:
                                        continue
                                    # the candidate's cost, from its two straight runs
                                    cc, d0 = 0.0, prev_d
                                    for (d_, m_) in runs_:
                                        if d0 is not None:
                                            if not TURN_OK[d0][d_]:
                                                cc = 1e30
                                                break
                                            if d0 != d_:
                                                cc += bend
                                        cc += m_ * DIRS[d_][2]
                                        d0 = d_
                                    if cc < 1e30 and next_d is not None:
                                        if not TURN_OK[d0][next_d]:
                                            cc = 1e30
                                        elif d0 != next_d:
                                            cc += bend
                                    if not cc < old_c - 1e-9:
                                        continue
                                    cand = octi_legal(pts[i], pts[k], diag_first, l)
                                    if cand is not None:
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
            self._book_corners_only(job)
        # the clean-up judges the FINISHED board: no congestion history (cells that were
        # fought over earlier are free now) and no preferred direction per layer - with
        # those, a straight drop on the top layer looked dearer than a bulge round nothing
        hist_mv = memoryview(self.hist).cast("B")
        hist_saved = bytes(hist_mv)
        hist_mv[:] = bytes(len(hist_saved))
        dm_saved = self.dir_mult
        flat = [[1.0] * 8 for _ in dm_saved]
        self.dir_mult = flat
        if self.native is not None:
            self.native.set_dir_mult(flat)
        try:
            # each stage is undone if it cuts a pour into more islands (a track moved across
            # a GND pour left a pad's via on an island)
            self._merge_through()
            made += self._guard_pours(self._reattach_leaves)
            for _ in range(2):             # a second round: neighbours may have moved out of the way
                self._merge_through()      # joins freed by the last stage are not joins any more
                k_ = self._guard_pours(self._smooth)
                made += k_
                if not k_:
                    break
        finally:
            hist_mv[:] = hist_saved
            self.dir_mult = dm_saved
            if self.native is not None:
                self.native.set_dir_mult(dm_saved)
        return made

    def _merge_through(self) -> int:
        """Join two routes of a net that meet end to end where nothing else connects (no
        pad, no third branch, no other route passing through): one route, so the meeting
        point is free to move.  Such a point is left behind when a branch that joined
        there was moved away - kept, it pinned the track and left a peak or a kink."""
        merged = 0
        for job in self.pb.nets:
            if len(job.paths) < 2:
                continue
            pads_ = set()
            for t_ in job.terminals:
                pads_ |= t_.cells
            while True:
                at: Dict[int, List[int]] = {}
                inner: Set[int] = set()
                for k, p_ in enumerate(job.paths):
                    at.setdefault(p_[0], []).append(k)
                    if p_[-1] != p_[0]:
                        at.setdefault(p_[-1], []).append(k)
                    inner.update(p_[1:-1])
                hit = None
                for c, ks in at.items():
                    if len(ks) == 2 and ks[0] != ks[1] and c not in pads_ and c not in inner:
                        hit = (c, ks[0], ks[1])
                        break
                if hit is None:
                    break
                c, i, j = hit
                a_, b_ = job.paths[i], job.paths[j]
                if a_[-1] != c:
                    a_ = a_[::-1]
                if b_[0] != c:
                    b_ = b_[::-1]
                job.paths = [p_ for k, p_ in enumerate(job.paths) if k not in (i, j)] + [a_ + b_[1:]]
                merged += 1
        return merged

    def _stub_turns(self, job: NetJob, p_: List[int], bend: float) -> float:
        """Penalty for how a route meets the escape stubs at its ends (the stub is part of
        the track): a turn of more than 45 degrees at the joint counts as two bends, more
        than 90 degrees (a hairpin at the pin) as a sharp stub."""
        g = self.pb.grid
        plane = g.plane
        pen = 0.0
        if len(p_) < 2:
            return 0.0
        anchors = self.pb.anchors
        for end, nb in ((p_[0], p_[1]), (p_[-1], p_[-2])):
            if end // plane != nb // plane:
                continue
            e = job.escapes.get(end)
            if e is not None:
                px_, py_ = (e[4][-1] if len(e) > 4 and e[4] else (e[0], e[1]))   # stub point next to the cell
            elif end in anchors:
                px_, py_ = anchors[end]          # a "nearest grid point" stub to the pad centre
            else:
                continue
            x_, y_, _ = g.xyl(end)
            cx_, cy_ = g.center(x_, y_)
            xn, yn, _ = g.xyl(nb)
            nxp, nyp = g.center(xn, yn)
            v1 = (cx_ - px_, cy_ - py_)
            v2 = (nxp - cx_, nyp - cy_)
            n1, n2 = math.hypot(*v1), math.hypot(*v2)
            if n1 < 1e-6 or n2 < 1e-6:
                continue
            cs = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
            if cs < -1e-6:
                pen += SHARP_STUB_COST
            elif cs < 0.7071 - 1e-6:
                pen += 2 * bend
        return pen

    def _pour_cells_except(self, nid_: int):
        """Membership test (`c in result`) for cells inside the zone outline of any pour net
        but nid_.  A per-cell count of pours, built once, instead of a set per net."""
        cnt = self.__dict__.get("_pour_count")
        if cnt is None:
            cnt = bytearray(self.pb.grid.size)
            for zm in self.pb.zone_masks.values():
                for i_ in range(len(zm)):
                    if zm[i_] and cnt[i_] < 255:
                        cnt[i_] += 1
            self._pour_count = cnt
        own = self.pb.zone_masks.get(nid_)

        class _Other:
            def __contains__(self_, c):
                return cnt[c] - (own[c] if own is not None else 0) > 0
        return _Other()

    def _guard_pours(self, stage) -> int:
        """Run a clean-up stage; undo all of it if it cut pours into more islands."""
        if not self.pb.zones:
            return stage()
        hist_saved = bytes(memoryview(self.hist).cast("B"))   # the check marks walls in hist
        _, cut0 = self._pour_check(self.pb.nets)
        snap = {id(j): (list(j.paths), j.failed, list(j.fail_notes)) for j in self.pb.nets}
        n = stage()

        def revert(j):
            paths, failed, notes = snap[id(j)]
            self._rip_up(j)
            j.paths = paths
            res = set()
            for q in paths:
                res |= self._path_resources(q, j)
            j.resources = self._bookable(j, res)
            for c in j.resources:
                self.occ[c] += 1
            self._book_keepout(j)
            j.failed, j.fail_notes = failed, notes

        if n:
            for _ in range(4):
                blamed, cut1 = self._pour_check(self.pb.nets)
                if cut1 <= cut0:
                    break
                # undo just the nets that moved in this stage and wall in the new island
                moved = [j for j in blamed if j.paths != snap[id(j)][0]]
                if not moved:
                    break
                for j in moved:
                    revert(j)
            else:
                blamed, cut1 = self._pour_check(self.pb.nets)
            if cut1 > cut0:                     # could not pin it down: undo the whole stage
                for j in self.pb.nets:
                    if j.paths != snap[id(j)][0]:
                        revert(j)
                n = 0
        memoryview(self.hist).cast("B")[:] = hist_saved
        return n

    def _true_pad_cells(self, job: NetJob, t) -> Set[int]:
        """The cells of terminal t whose copper really lands on its pads (an escape cell
        whose stub ends on them, or a cell within half a track of them).  Cached."""
        cache = self.__dict__.setdefault("_true_cells", {})
        key = id(t)
        if key in cache:
            return cache[key]
        cache[key] = out = self._true_pad_cells_calc(job, t)
        return out

    def _true_pad_cells_calc(self, job: NetJob, t) -> Set[int]:
        g = self.pb.grid
        if not t.pads:
            return set(t.cells)
        half = (job.neck_width or self.pb.settings.track_width) / 2
        out = set()
        for c in t.cells:
            e = job.escapes.get(c)
            if e is not None:
                if any(pp.distance(e[0], e[1]) < 1e-6 for pp in t.pads):
                    out.add(c)
            else:
                x_, y_, _ = g.xyl(c)
                cx_, cy_ = g.center(x_, y_)
                if any(pp.distance(cx_, cy_) < half - 1e-3 for pp in t.pads):
                    out.add(c)
        return out

    def _reattach_leaves(self) -> int:
        """Dead-end branches (a pad at one end, nothing else joining them): route each
        again from its pad to ANY point of the net's other copper (or, for a pour net, the
        pour), with long straight runs preferred.  Fixes loops and hooks where a branch
        joined the rest of the net from an awkward side - straightening alone cannot, as
        it keeps both ends where they are.  Kept only if legal and clearly better."""
        g = self.pb.grid
        plane, nx, ny = g.plane, g.nx, g.ny
        s = self.pb.settings
        keep = (self.pres_fac, s.bend_cost, self.h_weight)
        BEND = float(getattr(self, "_smooth_bend", 5.0))
        VIA = s.via_cost
        self.pres_fac, s.bend_cost, self.h_weight = 1e6, BEND, 1.0
        made = 0
        all_layers = set(range(g.nl))

        def cost(p_):
            L, B, V, d0 = 0.0, 0, 0, None
            for a, b in zip(p_, p_[1:]):
                if a // plane != b // plane:
                    V += 1
                    d0 = None
                    continue
                ra, rb = a % plane, b % plane
                d = DIR_INDEX[((rb % nx) - (ra % nx), (rb // nx) - (ra // nx))]
                L += DIRS[d][2]
                if d0 is not None and d != d0:
                    B += 1
                d0 = d
            return L + BEND * B + VIA * V + self._stub_turns(job, p_, BEND)

        try:
            for ji_, job in enumerate(self.pb.nets):
                self._clean_tick('re-attaching branches', job, ji_)
                if len(job.paths) < 2 or job.claim_r or self._cancel:
                    continue
                term_of = {}
                for t in job.terminals:
                    for c in t.cells:
                        term_of[c] = t
                pours_other = self._pour_cells_except(job.id)
                pour_cache = None
                changed = False
                for k in range(len(job.paths)):
                    p_ = job.paths[k]
                    others = [q for j, q in enumerate(job.paths) if j != k]
                    other_cells = {c for q in others for c in q}
                    ends_other = {q[0] for q in others} | {q[-1] for q in others}
                    # which end is the pad?  the other end attaches to the rest of the net
                    if p_[-1] in term_of and p_[-1] not in other_cells:
                        pad_end, attach = p_[-1], p_[0]
                    elif p_[0] in term_of and p_[0] not in other_cells:
                        pad_end, attach = p_[0], p_[-1]
                    else:
                        continue
                    # a leaf: nothing else starts or ends on it (apart from where it attaches)
                    if any(c in ends_other for c in p_ if c != attach):
                        continue
                    t = term_of[pad_end]
                    # without this branch the net falls into pieces; the pad's own piece
                    # (other routes starting from this pad) is NOT a place to re-attach to
                    nodes = [("t", t2) for t2 in job.terminals] + [("p", q) for q in others]
                    parent = list(range(len(nodes)))

                    def find(i_):
                        while parent[i_] != i_:
                            parent[i_] = parent[parent[i_]]
                            i_ = parent[i_]
                        return i_
                    at_cell = {}
                    for i_, (kind_, obj) in enumerate(nodes):
                        for c in (obj.cells if kind_ == "t" else obj):
                            j_ = at_cell.setdefault(c, i_)
                            if j_ != i_:
                                parent[find(i_)] = find(j_)
                    mine_root = find(next(i_ for i_, (kd, o) in enumerate(nodes) if kd == "t" and o is t))
                    roots = {find(i_) for i_ in range(len(nodes))}
                    if job.fanout_layers is None and len(roots) != 2:
                        continue                  # it bridges more than two pieces: leave it
                    if job.fanout_layers is not None and pour_cache is None:
                        pour_cache = bytes(self._pour_mask(job))     # once per net, not per branch
                    pour_m = pour_cache if job.fanout_layers is not None else None
                    goal = bytearray(pour_m) if pour_m is not None else bytearray(g.size)
                    # a pour net may have pieces that are not joined to anything yet (stopped
                    # part-way, or failed pads): only pieces that touch the pour count
                    grounded = None
                    if pour_m is not None:
                        grounded = set()
                        for i_, (kind_, obj) in enumerate(nodes):
                            cells_ = obj.cells if kind_ == "t" else obj
                            if any(pour_m[c] for c in cells_):
                                grounded.add(find(i_))
                    # whatever this route's other end joins must stay connected without it
                    # (re-attaching a pin-to-pin route cut the far pin off): for a pour net
                    # that piece must touch the pour, for others only two pieces may remain
                    ai = at_cell.get(attach)
                    if pour_m is not None:
                        if not ((ai is not None and find(ai) in grounded and find(ai) != mine_root)
                                or (ai is None and pour_m[attach])):
                            continue
                    elif ai is None or find(ai) == mine_root:
                        continue
                    own_piece = set()
                    for i_, (kind_, obj) in enumerate(nodes):
                        cells_ = obj.cells if kind_ == "t" else obj
                        if find(i_) == mine_root:
                            own_piece |= set(cells_)
                        elif grounded is None or find(i_) in grounded:
                            # a pad is a target only where copper really reaches it
                            for c in (self._true_pad_cells(job, obj) if kind_ == "t" else cells_):
                                goal[c] = 1
                    for c in own_piece:
                        goal[c] = 0
                    if not any(goal):
                        continue
                    xs = [(c % plane) % nx for c in p_]
                    ys = [(c % plane) // nx for c in p_]
                    m = 12
                    bbox = (max(0, min(xs) - m), max(0, min(ys) - m),
                            min(nx - 1, max(xs) + m), min(ny - 1, max(ys) + m))
                    pad_cells = set()
                    for t2 in job.terminals:
                        pad_cells |= t2.cells
                    br = branch_masks(others, pad_cells, plane, nx) if others else None
                    mine = set(p_) | self._path_resources(p_, job) | job.corners   # own side cells too
                    # start only where copper really lands on THIS pad (fine pitch: a cell
                    # can be shared with the neighbouring pin of the same net)
                    src = self._true_pad_cells(job, t)
                    if pad_end not in src or not src:
                        continue
                    # a signal's branch stays on the layers it used, with no more vias than it had
                    # (re-attaching with new vias and bottom-layer runs cut a GND pour); only a
                    # pour net's pad may drop a via to its plane
                    old_layers = {c // plane for c in p_}
                    old_vias = sum(1 for a_, b_ in zip(p_, p_[1:]) if a_ // plane != b_ // plane)
                    if job.fanout_layers is None:
                        s.via_cost = 1e6 if old_vias == 0 else VIA
                        lay_goal = bytearray(g.size)          # the goal, on the old layers only
                        for l_ in old_layers:
                            lay_goal[l_ * plane:(l_ + 1) * plane] = goal[l_ * plane:(l_ + 1) * plane]
                        goal_ = lay_goal
                    else:
                        goal_ = goal
                    try:
                        cand, _ = self._astar(job, src, None, bbox, mine, all_layers, goal_, branch=br)
                    except Exception:
                        cand = None
                    finally:
                        s.via_cost = VIA
                    if not cand or len(cand) < 2:
                        continue
                    if job.fanout_layers is None:
                        new_vias = sum(1 for a_, b_ in zip(cand, cand[1:]) if a_ // plane != b_ // plane)
                        if new_vias > old_vias or any(c // plane not in old_layers for c in cand):
                            continue
                    # no more track inside other nets' pours than before: the router's pour
                    # estimate is only an estimate, and a moved track can cut a real pour off
                    # a pad (a re-attached signal ran along a GND pour past a GND pin)
                    if sum(1 for c in cand if c in pours_other) > sum(1 for c in p_ if c in pours_other):
                        continue
                    # the old branch ran pad -> attach; the new one runs pad -> goal
                    old_c = cost(p_)
                    if cost(cand) >= old_c - 1.0:
                        continue
                    own_count = {}
                    for c in job.resources:
                        own_count[c] = own_count.get(c, 0) + 1
                    for c in job.keepout:
                        own_count[c] = own_count.get(c, 0) + 1
                    for c in job.corners:
                        own_count[c] = own_count.get(c, 0) + 1
                    res = self._path_resources(cand, job)
                    if any(self.occ[c] - own_count.get(c, 0) > 0 for c in res):
                        continue                          # another net is in the way
                    job.paths[k] = cand if pad_end == p_[0] else cand[::-1]
                    made += 1
                    changed = True
                if changed:
                    failed, notes = job.failed, job.fail_notes
                    paths = job.paths
                    self._rip_up(job)
                    job.paths = paths
                    res = set()
                    for q in paths:
                        res |= self._path_resources(q, job)
                    job.resources = self._bookable(job, res)
                    for c in job.resources:
                        self.occ[c] += 1
                    self._book_keepout(job)
                    job.failed, job.fail_notes = failed, notes
        finally:
            self.pres_fac, s.bend_cost, self.h_weight = keep
        return made

    def _smooth(self) -> int:
        """Second clean-up stage: re-route every same-layer stretch of track (between vias
        and junctions) with the router's own search on the finished board - other nets'
        copper is a hard obstacle, every bend is expensive - and keep the new stretch if it
        is legal and shorter or has fewer bends.  Catches what the two-segment shortcut
        cannot: dips and bumps round a pad, steps, jogs."""
        g = self.pb.grid
        plane, nx, ny = g.plane, g.nx, g.ny
        s = self.pb.settings
        keep = (self.pres_fac, s.bend_cost, s.via_cost, self.h_weight)
        BEND = float(getattr(self, "_smooth_bend", 5.0))   # a bend costs as much as 5 cells of track
        self.pres_fac, s.bend_cost, s.via_cost, self.h_weight = 1e6, BEND, 1e6, 1.0
        made = 0

        def metrics(cells):
            L, B, d0 = 0.0, 0, None
            for a, b in zip(cells, cells[1:]):
                ra, rb = a % plane, b % plane
                d = DIR_INDEX[((rb % nx) - (ra % nx), (rb // nx) - (ra // nx))]
                L += DIRS[d][2]
                if d0 is not None and d != d0:
                    B += 1
                d0 = d
            return L, B

        def dir_of(a, b):
            ra, rb = a % plane, b % plane
            return DIR_INDEX[((rb % nx) - (ra % nx), (rb // nx) - (ra // nx))]

        try:
            for ji_, job in enumerate(self.pb.nets):
                self._clean_tick('re-routing stretches', job, ji_)
                if not job.paths or job.claim_r or self._cancel:
                    continue                          # wide nets: keep their checked geometry
                ends = set()
                for p_ in job.paths:
                    ends.add(p_[0])
                    ends.add(p_[-1])
                pads_ = set()
                for t_ in job.terminals:
                    pads_ |= t_.cells
                # a real T: three or more same-layer branches meet there (a via, or the
                # end of a stretch running into one, is NOT a junction - its approach
                # direction is free to change, or a dip between two vias could never go)
                nb_: Dict[int, Set[int]] = {}
                for p_ in job.paths:
                    for u, v in zip(p_, p_[1:]):
                        if u // plane == v // plane:
                            nb_.setdefault(u, set()).add(v)
                            nb_.setdefault(v, set()).add(u)
                junction = {c for c in ends if c not in pads_ and len(nb_.get(c, ())) >= 3}
                changed = False
                new_paths = []
                for path in job.paths:
                    # pieces: same layer, split at vias and at cells where other routes join
                    pieces, cur = [], [path[0]]
                    for a, b in zip(path, path[1:]):
                        if a // plane != b // plane:
                            pieces.append((cur, True))
                            cur = [b]
                            continue
                        cur.append(b)
                        if b in ends and b != path[-1]:
                            pieces.append((cur, False))
                            cur = [b]
                    pieces.append((cur, False))
                    out: List[int] = []
                    for pi, (piece, via_after) in enumerate(pieces):
                        # the next stretch (same layer, starting where this one ends) keeps its
                        # first step: this one must meet it at 45 degrees or less
                        nxt = pieces[pi + 1][0] if pi + 1 < len(pieces) else None
                        next_d = dir_of(nxt[0], nxt[1]) if nxt is not None and len(nxt) >= 2 and \
                            nxt[0] == piece[-1] and nxt[1] // plane == nxt[0] // plane else None
                        if out and out[-1] // plane == piece[0] // plane and out[-1] == piece[0]:
                            prev_d = dir_of(out[-2], out[-1]) if len(out) >= 2 else None
                        else:
                            prev_d = None
                        if len(piece) >= 4:
                            a, b = piece[0], piece[-1]
                            L0, B0 = metrics(piece)
                            S0 = self._stub_turns(job, piece, BEND)
                            xa, ya = (a % plane) % nx, (a % plane) // nx
                            xb, yb = (b % plane) % nx, (b % plane) // nx
                            xs = [(c % plane) % nx for c in piece]
                            ys = [(c % plane) // nx for c in piece]
                            m = int(getattr(self, "_smooth_margin", 10))
                            bbox = (max(0, min(xs) - m), max(0, min(ys) - m),
                                    min(nx - 1, max(xs) + m), min(ny - 1, max(ys) + m))
                            mine = set(piece) | self._path_resources(piece, job) | job.corners   # own side cells too
                            sd = {a: prev_d} if prev_d is not None else None
                            try:
                                cand, _ = self._astar(job, {a}, {b}, bbox, mine, source_dirs=sd)
                            except Exception:
                                cand = None
                            ok = cand is not None and len(cand) >= 2 and cand[0] == a and cand[-1] == b \
                                and all(c // plane == a // plane for c in cand)
                            if ok:
                                L1, B1 = metrics(cand)
                                ok = L1 + BEND * B1 + self._stub_turns(job, cand, BEND) < \
                                    L0 + BEND * B0 + S0 - 1e-6
                            if ok and next_d is not None and not TURN_OK[dir_of(cand[-2], cand[-1])][next_d]:
                                ok = False
                            if ok:
                                # no more track inside other nets' pours than before
                                po = self._pour_cells_except(job.id)
                                if sum(1 for c in cand if c in po) > sum(1 for c in piece if c in po):
                                    ok = False
                            if ok:
                                # junction ends keep their direction (a T must not become a Y)
                                if a in junction and dir_of(cand[0], cand[1]) != dir_of(piece[0], piece[1]):
                                    ok = False
                                if b in junction and dir_of(cand[-2], cand[-1]) != dir_of(piece[-2], piece[-1]):
                                    ok = False
                            if ok:
                                # nothing of another net in the way (A*'s price was only a price)
                                res = self._path_resources(cand, job)
                                own_n = {}
                                for c in job.resources:
                                    own_n[c] = own_n.get(c, 0) + 1
                                for c in job.keepout:
                                    own_n[c] = own_n.get(c, 0) + 1
                                for c in job.corners:
                                    own_n[c] = own_n.get(c, 0) + 1
                                ok = all(self.occ[c] - own_n.get(c, 0) <= 0 for c in res)
                            if ok:
                                piece = cand
                                made += 1
                                changed = True
                        out.extend(piece[1:] if out and out[-1] == piece[0] else piece)
                    new_paths.append(out)
                if changed:
                    self._rip_up_keep_failed(job)
                    job.paths = new_paths
                    res = set()
                    for p_ in new_paths:
                        res |= self._path_resources(p_, job)
                    job.resources = self._bookable(job, res)
                    for c in job.resources:
                        self.occ[c] += 1
                    self._book_corners_only(job)
        finally:
            self.pres_fac, s.bend_cost, s.via_cost, self.h_weight = keep
        return made

    def _book_corners_only(self, job: NetJob) -> None:
        occ, cocc = self.occ, self.cocc
        for c in job.corners:
            occ[c] -= 1
            cocc[c] -= 1
        job.corners = self._corner_cells(job) - job.resources - job.keepout
        for c in job.corners:
            occ[c] += 1
            cocc[c] += 1

    def _rip_up_keep_failed(self, job: NetJob) -> None:
        for c in job.resources:
            self.occ[c] -= 1
        job.resources = set()

    def _rescue(self, removed: List[str], pass_no: int) -> List[str]:
        """Second chance for the nets the negotiation had to drop.

        First alone, against the finished board (they were dropped while everyone was
        still moving; the settled board often has room).  If that fails: take up the
        nets it was fighting with, route the dropped net first and then those nets
        around it - kept only if every one of them completes with no overlap, else
        everything is put back exactly as it was."""
        by_name = {j.name: j for j in self.pb.nets}
        keep_pf = self.pres_fac
        self.pres_fac = 1e4                      # any overlap is now prohibitively expensive
        still = []

        def clean(j):
            return not j.failed and not any(self.occ[c] > 1 for c in j.resources) and \
                not any(self.occ[c] > 1 and self.kocc[c] < self.occ[c] for c in j.keepout) and \
                not any(self.occ[c] > self.cocc[c] for c in j.corners)

        def snapshot(j):
            return (list(j.paths), set(j.resources), set(j.keepout), j.failed, set(j.corners))

        def restore(j, snap):
            self._rip_up(j)
            j.paths, j.resources, j.keepout, j.failed = list(snap[0]), set(snap[1]), set(snap[2]), snap[3]
            j.corners = set(snap[4])
            for c in j.resources:
                self.occ[c] += 1
            for c in j.keepout:
                self.occ[c] += 1
                self.kocc[c] += 1
            for c in j.corners:
                self.occ[c] += 1
                self.cocc[c] += 1
        for name in sorted(removed, key=lambda n: by_name[n].hpwl()):
            job = by_name[name]
            self._rip_up(job)
            self._route_net(job, pass_no)
            ok = clean(job)
            if not ok:
                self._rip_up(job)
                # who is really in the way: route it with overlaps allowed (cheap) and see
                # whose copper its preferred path runs through
                self.pres_fac = 1.0
                self._route_net(job, pass_no)
                mine_ = job.resources | job.keepout | job.corners
                hits = {}
                for o in self.pb.nets:
                    if o is not job and o.name not in removed:
                        k_ = len(mine_ & (o.resources | o.keepout | o.corners))
                        if k_:
                            hits[o.name] = k_
                self._rip_up(job)
                self.pres_fac = 1e4
                names = sorted(hits, key=lambda n: -hits[n])
                for n in self.fought.get(name, []):
                    if n not in names:
                        names.append(n)
                # at most 3 others: every net taken up must be routed again (taking up
                # more than were re-routed once left nets with no tracks at all)
                others = [by_name[n] for n in names if n in by_name and n not in removed][:3]
                if others:
                    import itertools
                    snaps = {o.name: snapshot(o) for o in others}
                    group = [job] + others
                    # whoever is routed first takes the lane that suits it: try every order
                    # (at most 24 for 4 nets) - e.g. three nets leaving a QFN row side by side
                    # fit only when each takes the row matching its pin order
                    t_net = time.time()
                    for order in itertools.permutations(group):
                        if time.time() - t_net > RESCUE_SECONDS:
                            break                    # rarely pays off this late: give up
                        for o in others:
                            self._rip_up(o)
                        ok = True
                        for o in order:
                            self._route_net(o, pass_no)
                            if not clean(o):
                                ok = False
                                break
                        if ok:
                            break
                        ok = False
                        for o in group:              # undo this order's routes
                            self._rip_up(o)
                        for o in others:
                            restore(o, snaps[o.name])
                    if not ok:
                        self._rip_up(job)
            if ok:
                self.fought.pop(name, None)
            else:
                job.failed = len(job.terminals) - 1
                still.append(name)
            self._emit("rescue", **{"net": name, "ok": ok})
        self.pres_fac = keep_pf
        return still

    def _legalize(self) -> List[str]:
        """Not converged: drop the most-conflicted nets until nothing overlaps.

        Ground and power come first: a pour net (GND, +3.3V ...) is never dropped as a
        whole - its vias are what the planes hang on (dropping GND left a 4-layer board
        with no ground vias at all).  Signal nets give way; if pour nets still overlap
        each other after that, only their clashing pad connections are taken out."""
        removed = []

        def n_over(j, over):
            return sum(1 for c in j.resources if c in over) + sum(1 for c in j.corners if c in over)

        while True:
            over = self._overused()
            if not over:
                return removed
            signals = [j for j in self.pb.nets if j.fanout_layers is None and n_over(j, over)]
            if not signals:                          # only pour nets clash: trim, don't drop
                for j in self.pb.nets:
                    if j.fanout_layers is not None and n_over(j, over):
                        self._drop_clashing_paths(j, over)
                        break
                continue
            worst = max(signals, key=lambda j: (n_over(j, over), -j.hpwl()))
            mine = (worst.resources | worst.keepout | worst.corners) & over
            self.fought[worst.name] = sorted({j.name for j in self.pb.nets if j is not worst
                                              and not mine.isdisjoint(j.resources | j.keepout | j.corners)})
            self._rip_up(worst)
            worst.failed = len(worst.terminals) - 1
            removed.append(worst.name)

    def _rescue_pours(self, pass_no: int) -> None:
        """Pour-net pads left unconnected (trimmed by legalize, or failed): on the finished
        board, route each one ON ITS OWN to the pour or to the net's existing copper.  A
        new connection is kept only if it overlaps nothing (overlaps are now prohibitively
        expensive) and its via keeps hole-to-hole distance from the net's other vias."""
        g = self.pb.grid
        plane, nx, ny = g.plane, g.nx, g.ny
        s_ = self.pb.settings
        min_vv = s_.via_drill + max(s_.hole_clearance, 0.25) + 0.01
        kv = int(min_vv / g.pitch)
        near_v = [(dx, dy) for dy in range(-kv, kv + 1) for dx in range(-kv, kv + 1)
                  if (dx or dy) and math.hypot(dx, dy) * g.pitch < min_vv]
        keep_pf = self.pres_fac
        self.pres_fac = 1e4
        all_layers = set(range(g.nl))
        try:
            for job in self.pb.nets:
                if job.fanout_layers is None or not job.failed or self._cancel:
                    continue
                pour = bytes(self._pour_mask(job))
                goal = bytearray(pour)
                for p_ in job.paths:
                    for c in p_:
                        goal[c] = 1
                own = set(job.resources)
                fixed = 0
                for t in job.terminals:
                    on_paths = {c for p_ in job.paths for c in p_}
                    if any(c in on_paths and self._copper_on_pad(job, t, c) for c in t.cells):
                        continue                          # already joined by copper
                    if any(pour[c] for c in t.cells):
                        continue                          # on the pour itself
                    xs = [(c % plane) % nx for c in t.cells]
                    ys = [(c % plane) // nx for c in t.cells]
                    m = 10
                    bbox = (max(0, min(xs) - m), max(0, min(ys) - m),
                            min(nx - 1, max(xs) + m), min(ny - 1, max(ys) + m))
                    # the net's own vias keep their distance: close the via spots round them
                    cols = {a_ % plane for p_ in job.paths for a_, b_ in zip(p_, p_[1:])
                            if a_ // plane != b_ // plane}
                    closed = []
                    vo = self.pb.via_owner
                    for col in cols:
                        x0, y0 = col % nx, col // nx
                        for dx, dy in near_v:
                            x_, y_ = x0 + dx, y0 + dy
                            if 0 <= x_ < nx and 0 <= y_ < ny:
                                q = y_ * nx + x_
                                if q not in cols and vo[q] != BLOCKED:
                                    closed.append((q, vo[q]))
                                    vo[q] = BLOCKED
                    try:
                        path, _ = self._astar(job, t.cells, None, bbox, own, all_layers, goal)
                    finally:
                        for q, v in reversed(closed):
                            vo[q] = v
                    if path is None:
                        continue
                    res = self._path_resources(path, job) - own
                    if any(self.occ[c] for c in res):
                        continue                          # would overlap another net
                    job.paths.append(path)
                    for c in res:
                        self.occ[c] += 1
                    job.resources |= res
                    own |= res
                    for c in path:
                        goal[c] = 1
                    fixed += 1
                if fixed:
                    for c in job.keepout:             # re-book it from scratch below
                        self.occ[c] -= 1
                        self.kocc[c] -= 1
                    job.keepout = set()
                    self._book_keepout(job)
                    job.failed = max(0, job.failed - fixed)
                    self._emit("rescue", **{"net": job.name, "ok": job.failed == 0, "pads": fixed})
        finally:
            self.pres_fac = keep_pf

    def _drop_clashing_paths(self, job: NetJob, over: Set[int]) -> None:
        """Take out a pour net's pad connections that overlap other copper, keep the rest."""
        keep, dropped = [], 0
        for p_ in job.paths:
            res = self._path_resources(p_, job)
            if res & over:
                dropped += 1
            else:
                keep.append(p_)
        if not dropped:                              # the clash is in its corners/keep-out
            dropped = len(job.paths)
            keep = []
        failed = job.failed
        notes = job.fail_notes
        self._rip_up(job)
        job.paths = keep
        res = set()
        for p_ in keep:
            res |= self._path_resources(p_, job)
        job.resources = self._bookable(job, res)
        for c in job.resources:
            self.occ[c] += 1
        self._book_keepout(job)
        job.failed = failed + dropped
        job.fail_notes = notes + [f"{dropped} pad connection(s) of {job.name} overlapped another "
                                  "net and were taken out"]

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
                    # an escape stub keeps the width it was checked at (narrower into fine pitch),
                    # and may bend once (a point in a channel the grid cannot hit)
                    wst = min(neck_w, esc[end][3]) if end in esc and len(esc[end]) > 3 else neck_w
                    bend = list(esc[end][4]) if end in esc and len(esc[end]) > 4 else []
                    poly = [(cx, cy)] + bend[::-1] + [(ax, ay)]
                    for (p1, p2) in zip(poly, poly[1:]):
                        segs.append((p1[0], p1[1], p2[0], p2[1], g.layers[l]))
                        widths.append(wst)
        return segs, vias, widths

    def _result(self, converged: bool, removed: List[str]) -> RouteResult:
        nets: Dict[str, NetRoute] = {}
        s = self.pb.settings
        for j in self.pb.nets:
            nr = NetRoute(j.name)
            if j.fanout_layers is None and len(j.terminals) >= 2 and not j.paths and not j.failed:
                j.failed = len(j.terminals) - 1     # never report a net with no tracks as done
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
            if j.name in removed and self.fought.get(j.name):
                nr.why = [f"no room left after the other nets were placed - it was fighting "
                          f"{', '.join(self.fought[j.name][:5])} for the same space"] + nr.why
            nets[j.name] = nr
        r = RouteResult(nets, self.passes, converged, removed, dict(self.pb.skipped), self._cancel)
        r.notes = dict(self.pb.notes)
        return r


def route(board, settings=None, nets=None, callback=None, record_exploration=False) -> RouteResult:
    """Convenience: build the problem and run the router."""
    from .model import RouteSettings
    from .problem import build_problem
    from .problem import add_bars_to_result, same_net_bars
    import dataclasses
    settings = settings or RouteSettings()
    bars = same_net_bars(board, settings, nets)       # fine-pitch same-net neighbours
    b = dataclasses.replace(board, tracks=list(board.tracks) + bars) if bars else board
    pb = build_problem(b, settings, nets)
    res = Router(pb, callback, record_exploration).run()
    add_bars_to_result(res, bars)
    return res
