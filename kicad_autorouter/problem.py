"""Turn a :class:`Board` into a discrete routing problem.

The grid
--------
Track centre-lines live on the centres of a square grid of pitch
``p = track_width + clearance``.  Two tracks of different nets on
orthogonally-adjacent cells are then exactly at minimum clearance, so on the
grid "one cell = one net" *is* the clearance rule.  Two extra rules keep that
true for the other geometry:

* a diagonal step also claims the two side cells it squeezes past
  (the diagonal passes only 0.707 p from their centres);
* a via claims every cell whose centre is closer than
  ``via_r + clearance + track_width/2`` on all layers.

Ownership maps (``owner`` per layer cell, ``via_owner`` per x/y column) mark
where copper that is *not* being routed (pads, existing tracks, board edge)
forbids a net from putting a track centre-line / via centre:

    FREE (-1)      anyone may use it
    n >= 0         only net n may use it (it is inside net n's clearance halo)
    BLOCKED (-2)   nobody (board edge, NPTH hole, unconnected copper,
                   or two different nets' halos overlap)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .model import Board, Pad, RouteSettings, Track, Via

FREE = -1
ESCAPE_FREE_MIN = 3     # a small pad with fewer free grid neighbours than this gets escape stubs
BLOCKED = -2


def seg_point_dist(px, py, x1, y1, x2, y2) -> float:
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 <= 1e-18:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


@dataclass
class Terminal:
    """A set of grid cells that are already electrically one piece
    (a pad, or a cluster of pads + existing copper)."""
    cells: Set[int]
    cx: float           # centroid in mm, used to order connections
    cy: float
    label: str = ""
    pads: list = field(default_factory=list)     # the pads in it


@dataclass
class NetJob:
    name: str
    id: int
    terminals: List[Terminal]
    # filled in by the router
    fanout_layers: Optional[Set[int]] = None   # zone net: connect each terminal to these layers
    paths: List[List[int]] = field(default_factory=list)
    resources: Set[int] = field(default_factory=set)
    keepout: Set[int] = field(default_factory=set)   # cells near its escape stubs (not its own track)
    unreachable: int = 0      # pads that no route can reach (reported as incomplete)
    fail_notes: List[str] = field(default_factory=list)   # why connections failed (last attempt)
    failed: int = 0           # connections that could not be routed at all
    # net-class width: a track wider than the grid's own (or with a bigger clearance)
    # claims every cell within Chebyshev radius claim_r of its centre-line, so other nets
    # keep their distance.  It keeps that width everywhere; only right at one of its own
    # pads, where neighbouring pads leave no room, may it neck down to the grid width.
    width: float = 0.0        # track width to draw (mm)
    neck_width: float = 0.0   # width where it necks down (mm)
    claim_r: int = 0
    # fine-pitch pads: grid cells reached by a short off-grid stub from the pad centre
    # cell -> (pad x, pad y, cells the stub keeps other nets away from)
    escapes: Dict[int, Tuple[float, float, List[int]]] = field(default_factory=dict)
    wide_ok: Optional[List[int]] = None   # per row (layer*ny + y): bit x = full width allowed
    neck: Optional[List[int]] = None      # per row: bit x = neck-down zone (claim only the cell)
    half_bad: Optional[bytearray] = None  # half-pitch map: 1 = too close to fixed copper
    half_x: int = 0
    half_plane: int = 0

    def hpwl(self) -> float:
        xs = [t.cx for t in self.terminals]
        ys = [t.cy for t in self.terminals]
        return (max(xs) - min(xs)) + (max(ys) - min(ys)) if xs else 0.0


class Grid:
    def __init__(self, x0: float, y0: float, pitch: float, nx: int, ny: int, layers: List[str]):
        self.x0, self.y0, self.pitch = x0, y0, pitch
        self.nx, self.ny = nx, ny
        self.layers = layers
        self.nl = len(layers)
        self.plane = nx * ny
        self.size = self.plane * self.nl

    def idx(self, x: int, y: int, l: int) -> int:
        return (l * self.ny + y) * self.nx + x

    def xyl(self, i: int) -> Tuple[int, int, int]:
        l, r = divmod(i, self.plane)
        y, x = divmod(r, self.nx)
        return x, y, l

    def center(self, x: int, y: int) -> Tuple[float, float]:
        return self.x0 + x * self.pitch, self.y0 + y * self.pitch

    def cell_of(self, xmm: float, ymm: float) -> Tuple[int, int]:
        x = int(round((xmm - self.x0) / self.pitch))
        y = int(round((ymm - self.y0) / self.pitch))
        return min(max(x, 0), self.nx - 1), min(max(y, 0), self.ny - 1)

    def cells_near(self, xmin, ymin, xmax, ymax):
        """Grid x/y index ranges whose centres fall in a mm box."""
        p = self.pitch
        x_lo = max(0, int(math.ceil((xmin - self.x0) / p - 1e-9)))
        x_hi = min(self.nx - 1, int(math.floor((xmax - self.x0) / p + 1e-9)))
        y_lo = max(0, int(math.ceil((ymin - self.y0) / p - 1e-9)))
        y_hi = min(self.ny - 1, int(math.floor((ymax - self.y0) / p + 1e-9)))
        return range(x_lo, x_hi + 1), range(y_lo, y_hi + 1)


@dataclass
class Problem:
    grid: Grid
    settings: RouteSettings
    owner: List[int]
    via_owner: List[int]
    nets: List[NetJob]
    net_ids: Dict[str, int]
    anchors: Dict[int, Tuple[float, float]]      # pad cell -> pad centre (for end stubs)
    via_footprint: List[Tuple[int, int]]         # (dx, dy) offsets a via claims
    layer_dirs: List[str]
    skipped: Dict[str, str]                      # net -> reason it was not routed
    zones: Dict[int, Tuple[Set[int], Set[int]]] = field(default_factory=dict)  # net id -> (layers, anchor cells)
    notes: Dict[str, str] = field(default_factory=dict)          # net -> remark for the user
    zone_masks: Dict[int, bytearray] = field(default_factory=dict)  # net id -> cells inside its zones
    zone_vias: Dict[int, Set[int]] = field(default_factory=dict)    # net id -> x/y of its vias / PTH pads


# --------------------------------------------------------------------------
def _mark(arr: List[int], i: int, net_id: int) -> None:
    cur = arr[i]
    if cur == FREE:
        arr[i] = net_id
    elif cur != net_id:
        arr[i] = BLOCKED


def outline_loops(outline: Sequence[Tuple[float, float, float, float]], tol: float = 0.01):
    """Chain Edge.Cuts line pieces into closed loops (lists of points).  A chain that does
    not close is closed straight back to its start."""
    def key(x, y):
        return (round(x / tol), round(y / tol))
    at: Dict[tuple, List[int]] = {}
    for i, (x1, y1, x2, y2) in enumerate(outline):
        at.setdefault(key(x1, y1), []).append(i)
        at.setdefault(key(x2, y2), []).append(i)
    used = [False] * len(outline)
    loops = []
    for i0 in range(len(outline)):
        if used[i0]:
            continue
        used[i0] = True
        x1, y1, x2, y2 = outline[i0]
        pts = [(x1, y1), (x2, y2)]
        start = key(x1, y1)
        cur = key(x2, y2)
        while cur != start:
            nxt = next((j for j in at.get(cur, ()) if not used[j]), None)
            if nxt is None:
                break
            used[nxt] = True
            a1, b1, a2, b2 = outline[nxt]
            if key(a1, b1) == cur:
                pts.append((a2, b2))
                cur = key(a2, b2)
            else:
                pts.append((a1, b1))
                cur = key(a1, b1)
        if len(pts) >= 2 and key(*pts[-1]) == key(*pts[0]):
            pts.pop()                           # closed: do not repeat the first point
        if len(pts) >= 3:
            loops.append(pts)
    return loops


def _inside_poly(x, y, pts) -> bool:
    inside = False
    n = len(pts)
    for i in range(n):
        (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            if x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


def _loop_depths(loops) -> List[int]:
    """How many other loops each loop lies inside: 0 = board outline (outlines that
    touch or overlap all count as board), 1 = cut-out, 2 = island in a cut-out ..."""
    def area(p):
        return abs(sum(p[i][0] * p[(i + 1) % len(p)][1] - p[(i + 1) % len(p)][0] * p[i][1]
                       for i in range(len(p)))) / 2
    areas = [area(p) for p in loops]
    depth = []
    for i, p in enumerate(loops):
        d = 0
        for j, q in enumerate(loops):
            if j != i and areas[j] > areas[i]:
                inside = sum(1 for (x, y) in p if _inside_poly(x, y, q))
                if inside * 2 > len(p):          # most of its corners inside: contained
                    d += 1
        depth.append(d)
    return depth


def _outline_mask(grid: Grid, outline: Sequence[Tuple[float, float, float, float]],
                  margin: float) -> List[bool]:
    """True for x/y columns that are inside the board and >= margin from its edge.

    Inside = inside a board-level loop of Edge.Cuts and not in a cut-out.  Loops that
    touch or overlap each other (e.g. a module footprint's own outline drawn against
    the board's) count as one board, which KiCad itself refuses to build."""
    nx, ny = grid.nx, grid.ny
    ok = [False] * grid.plane
    if not outline:
        return [True] * grid.plane
    loops = outline_loops(outline)
    depth = _loop_depths(loops)
    order = sorted(range(len(loops)), key=depth.__getitem__)    # deeper loops paint last
    yr_of = [(min(y for _, y in p), max(y for _, y in p)) for p in loops]
    for y in range(ny):
        _, ymm = grid.center(0, y)
        row = [-1] * nx
        for li in order:
            lo, hi = yr_of[li]
            if not lo <= ymm <= hi:
                continue
            p = loops[li]
            xs = []
            for i in range(len(p)):
                (x1, y1), (x2, y2) = p[i], p[(i + 1) % len(p)]
                if (y1 <= ymm < y2) or (y2 <= ymm < y1):
                    xs.append(x1 + (ymm - y1) * (x2 - x1) / (y2 - y1))
            xs.sort()
            for a, b in zip(xs[0::2], xs[1::2]):
                xr, _ = grid.cells_near(a, ymm, b, ymm)
                for x in xr:
                    row[x] = depth[li]
        for x in range(nx):
            if row[x] >= 0 and row[x] % 2 == 0:
                ok[y * nx + x] = True
    def point_inside(px, py):
        d = -1
        for li, p in enumerate(loops):
            if depth[li] > d and _inside_poly(px, py, p):
                d = depth[li]
        return d >= 0 and d % 2 == 0

    # remove the edge band - only along REAL edges: a line with board on both sides of
    # it (two outlines that touch or overlap) is no edge at all
    simple = len(loops) <= 1 or all(d % 2 == 1 for d in depth[1:]) and depth.count(0) == 1
    for (x1, y1, x2, y2) in outline:
        L = math.hypot(x2 - x1, y2 - y1)
        xr, yr = grid.cells_near(min(x1, x2) - margin, min(y1, y2) - margin,
                                 max(x1, x2) + margin, max(y1, y2) + margin)
        for y in yr:
            for x in xr:
                cx, cy = grid.center(x, y)
                if seg_point_dist(cx, cy, x1, y1, x2, y2) >= margin:
                    continue
                if not simple and L > 1e-9:
                    tt = max(0.0, min(1.0, ((cx - x1) * (x2 - x1) + (cy - y1) * (y2 - y1)) / (L * L)))
                    px, py = x1 + tt * (x2 - x1), y1 + tt * (y2 - y1)
                    nx_, ny_ = -(y2 - y1) / L * 0.05, (x2 - x1) / L * 0.05
                    if point_inside(px + nx_, py + ny_) == point_inside(px - nx_, py - ny_):
                        continue                 # same on both sides: not a board edge here
                ok[y * nx + x] = False
    return ok


def _rings_inside(grid: "Grid", rings) -> List[int]:
    """x/y cell indices whose centres lie inside the rings (even-odd, so holes work)."""
    segs = []
    for ring in rings:
        for a, b in zip(ring, ring[1:] + ring[:1]):
            segs.append((a[0], a[1], b[0], b[1]))
    if not segs:
        return []
    ys = [p for sg in segs for p in (sg[1], sg[3])]
    out = []
    _, yr = grid.cells_near(grid.x0, min(ys), grid.x0, max(ys))
    for y in yr:
        _, ymm = grid.center(0, y)
        xs = sorted(x1 + (ymm - y1) * (x2 - x1) / (y2 - y1)
                    for (x1, y1, x2, y2) in segs if (y1 <= ymm < y2) or (y2 <= ymm < y1))
        for a, b in zip(xs[0::2], xs[1::2]):
            xr, _ = grid.cells_near(a, ymm, b, ymm)
            out.extend(y * grid.nx + x for x in xr)
    return out


def _cluster(net: str, pads: List[Pad], tracks: List[Track], vias: List[Via]):
    """Group a net's pads/tracks/vias into already-connected clusters (union-find)."""
    items: List[Tuple[str, object]] = [("p", p) for p in pads] + \
        [("t", t) for t in tracks] + [("v", v) for v in vias]
    parent = list(range(len(items)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    order = {"t": 0, "v": 1, "p": 2}

    def touches(a, b) -> bool:
        if order[a[0]] > order[b[0]]:
            a, b = b, a
        (ka, A), (kb, B) = a, b
        if ka == "t" and kb == "t":
            if A.layer != B.layer:
                return False
            tol = max(A.width, B.width) / 2 + 1e-6
            return any(seg_point_dist(px, py, B.x1, B.y1, B.x2, B.y2) <= tol
                       for (px, py) in ((A.x1, A.y1), (A.x2, A.y2))) or \
                any(seg_point_dist(px, py, A.x1, A.y1, A.x2, A.y2) <= tol
                    for (px, py) in ((B.x1, B.y1), (B.x2, B.y2)))
        if ka == "t" and kb == "v":
            return seg_point_dist(B.x, B.y, A.x1, A.y1, A.x2, A.y2) <= B.diameter / 2
        if ka == "t" and kb == "p":
            return A.layer in B.layers and any(
                B.distance(px, py) <= A.width / 2 + 1e-6
                for (px, py) in ((A.x1, A.y1), (A.x2, A.y2)))
        if ka == "v" and kb == "v":
            return math.hypot(A.x - B.x, A.y - B.y) <= (A.diameter + B.diameter) / 2
        if ka == "v" and kb == "p":
            return B.distance(A.x, A.y) <= A.diameter / 2
        # pad-pad: same position or overlapping copper (multi-pad footprints)
        return bool(A.layers & B.layers) and (A.distance(B.x, B.y) <= 0.0 or
                                              B.distance(A.x, A.y) <= 0.0)

    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            if touches(items[i], items[j]):
                union(i, j)
    groups: Dict[int, list] = {}
    for i, it in enumerate(items):
        groups.setdefault(find(i), []).append(it)
    return list(groups.values())


ROUTED, PARTIAL, UNROUTED, POUR, SINGLE = "routed", "partial", "unrouted", "pour", "single"


def net_status(board: Board) -> Dict[str, str]:
    """Classify every net from its existing copper (no grid needed, fast):

    routed    all pads already joined by tracks/vias
    partial   some tracks exist, but pads are still unconnected
    unrouted  no tracks at all, 2+ pads
    pour      has a copper zone (the pour connects it)
    single    fewer than two pads - nothing to route
    """
    pads: Dict[str, List[Pad]] = {}
    for p in board.pads:
        if p.net:
            pads.setdefault(p.net, []).append(p)
    trk: Dict[str, List[Track]] = {}
    for t in board.tracks:
        trk.setdefault(t.net, []).append(t)
    vias: Dict[str, List[Via]] = {}
    for v in board.vias:
        vias.setdefault(v.net, []).append(v)
    out: Dict[str, str] = {}
    for name, ps in pads.items():
        if name in board.zone_nets:
            out[name] = POUR
        elif len(ps) < 2:
            out[name] = SINGLE
        elif not trk.get(name) and not vias.get(name):
            out[name] = UNROUTED
        else:
            clusters = _cluster(name, ps, trk.get(name, []), vias.get(name, []))
            n_with_pads = sum(1 for cl in clusters if any(k == "p" for k, _ in cl))
            out[name] = ROUTED if n_with_pads <= 1 else PARTIAL
    return out


def loose_copper(board: Board, nets: Optional[Iterable[str]] = None) -> Tuple[List[Track], List[Via]]:
    """Track pieces / vias that touch no pad of their net (dead-end copper, e.g. an
    unfinished trace).  Pour nets are skipped - their vias may be stitched to the zone."""
    wanted = set(nets) if nets is not None else None
    pads: Dict[str, List[Pad]] = {}
    for p in board.pads:
        if p.net:
            pads.setdefault(p.net, []).append(p)
    trk: Dict[str, List[Track]] = {}
    for t in board.tracks:
        trk.setdefault(t.net, []).append(t)
    vias: Dict[str, List[Via]] = {}
    for v in board.vias:
        vias.setdefault(v.net, []).append(v)
    lt: List[Track] = []
    lv: List[Via] = []
    for name in set(trk) | set(vias):
        if not name or name in board.zone_nets or (wanted is not None and name not in wanted):
            continue
        for cl in _cluster(name, pads.get(name, []), trk.get(name, []), vias.get(name, [])):
            if not any(k == "p" for k, _ in cl):
                lt += [it for k, it in cl if k == "t"]
                lv += [it for k, it in cl if k == "v"]
    return lt, lv


def nets_to_finish(board: Board) -> List[str]:
    """Nets with missing connections (unrouted or partly routed), plus pour nets
    (their SMD pads may still need a fan-out via).  Existing copper is kept."""
    return sorted(n for n, st in net_status(board).items() if st in (UNROUTED, PARTIAL, POUR))


def build_problem(board: Board, settings: RouteSettings,
                  nets: Optional[Iterable[str]] = None) -> Problem:
    s = settings
    layers = list(s.layers) if s.layers else list(board.copper_layers)
    layers = [l for l in layers if l in board.copper_layers] or list(board.copper_layers)
    lidx = {name: i for i, name in enumerate(layers)}

    p = s.pitch()
    bx0, by0, bx1, by1 = board.bbox()
    nx = int(math.floor((bx1 - bx0) / p)) + 1
    ny = int(math.floor((by1 - by0) / p)) + 1
    grid = Grid(bx0, by0, p, nx, ny, layers)

    w2 = s.track_width / 2.0
    vr = s.via_diameter / 2.0
    owner = [FREE] * grid.size
    via_owner = [FREE] * grid.plane

    # ---- board edge
    inside_trk = _outline_mask(grid, board.outline, s.edge_clearance + w2)
    inside_via = _outline_mask(grid, board.outline, s.edge_clearance + vr)
    for xy in range(grid.plane):
        if not inside_trk[xy]:
            for l in range(grid.nl):
                owner[l * grid.plane + xy] = BLOCKED
        if not inside_via[xy]:
            via_owner[xy] = BLOCKED

    # ---- net ids for every net on the board
    all_nets = sorted({p_.net for p_ in board.pads if p_.net} |
                      {t.net for t in board.tracks if t.net} |
                      {v.net for v in board.vias if v.net})
    net_ids = {n: i for i, n in enumerate(all_nets)}

    def nid(name: str) -> int:
        return net_ids[name] if name else BLOCKED

    # ---- pads: halo for other nets' track centre-lines and via centres
    for pad in board.pads:
        n = nid(pad.net)
        cl = s.clearance_for(pad.net)
        r_trk = cl + w2
        r_via = cl + vr
        R = pad.radius() + max(r_trk, r_via)
        xr, yr = grid.cells_near(pad.x - R, pad.y - R, pad.x + R, pad.y + R)
        pad_l = [lidx[l] for l in pad.layers if l in lidx]
        for y in yr:
            for x in xr:
                cx, cy = grid.center(x, y)
                d = pad.distance(cx, cy)
                if d < r_trk:
                    for l in pad_l:
                        _mark(owner, grid.idx(x, y, l), n)
                if d < r_via:
                    _mark(via_owner, y * nx + x, n)
                if d < vr and not s.allow_via_in_pad:
                    via_owner[y * nx + x] = BLOCKED
        if pad.drill > 0:   # the hole goes through every layer
            hole_r = pad.drill / 2 + cl + w2
            xr, yr = grid.cells_near(pad.x - hole_r, pad.y - hole_r, pad.x + hole_r, pad.y + hole_r)
            for y in yr:
                for x in xr:
                    cx, cy = grid.center(x, y)
                    if math.hypot(cx - pad.x, cy - pad.y) < hole_r:
                        for l in range(grid.nl):
                            if l not in pad_l:
                                _mark(owner, grid.idx(x, y, l), n)

    for (hx, hy, hd) in board.holes:
        hc = max(s.clearance, s.hole_clearance)
        rt, rv = hd / 2 + hc + w2, hd / 2 + hc + vr
        xr, yr = grid.cells_near(hx - rv, hy - rv, hx + rv, hy + rv)
        for y in yr:
            for x in xr:
                cx, cy = grid.center(x, y)
                d = math.hypot(cx - hx, cy - hy)
                if d < rt:
                    for l in range(grid.nl):
                        owner[grid.idx(x, y, l)] = BLOCKED
                if d < rv:
                    via_owner[y * nx + x] = BLOCKED

    # ---- existing tracks / vias
    for t in board.tracks:
        if t.layer not in lidx:
            continue
        n = nid(t.net)
        l = lidx[t.layer]
        cl = s.clearance_for(t.net)
        r_trk = t.width / 2 + cl + w2
        r_via = t.width / 2 + cl + vr
        R = max(r_trk, r_via)
        xr, yr = grid.cells_near(min(t.x1, t.x2) - R, min(t.y1, t.y2) - R,
                                 max(t.x1, t.x2) + R, max(t.y1, t.y2) + R)
        for y in yr:
            for x in xr:
                cx, cy = grid.center(x, y)
                d = seg_point_dist(cx, cy, t.x1, t.y1, t.x2, t.y2)
                if d < r_trk:
                    _mark(owner, grid.idx(x, y, l), n)
                if d < r_via:
                    _mark(via_owner, y * nx + x, n)
    for v in board.vias:
        n = nid(v.net)
        cl = s.clearance_for(v.net)
        r_trk = v.diameter / 2 + cl + w2
        r_via = v.diameter / 2 + cl + vr
        vl = [lidx[l] for l in (v.layers or layers) if l in lidx]
        xr, yr = grid.cells_near(v.x - r_via, v.y - r_via, v.x + r_via, v.y + r_via)
        for y in yr:
            for x in xr:
                cx, cy = grid.center(x, y)
                d = math.hypot(cx - v.x, cy - v.y)
                if d < r_trk:
                    for l in vl:
                        _mark(owner, grid.idx(x, y, l), n)
                if d < r_via:
                    _mark(via_owner, y * nx + x, n)

    # ---- which nets to route
    wanted = set(nets) if nets is not None else set(board.nets())
    skipped: Dict[str, str] = {}
    # ---- where each pour net's zones may put copper (cells inside the zone OUTLINE)
    zone_masks: Dict[int, bytearray] = {}
    for name, zlayers in board.zone_layers.items():
        if name not in net_ids:
            continue
        mask = bytearray(grid.size)
        outlines = board.zone_outlines.get(name)
        if outlines:
            for (layers_, rings) in outlines:
                ls = [lidx[l] for l in layers_ if l in lidx]
                for xy in _rings_inside(grid, rings):
                    for l in ls:
                        mask[l * grid.plane + xy] = 1
        else:   # outline unknown: assume the whole layer (older behaviour)
            for l in (lidx[x] for x in zlayers if x in lidx):
                for xy in range(grid.plane):
                    mask[l * grid.plane + xy] = 1
        zone_masks[net_ids[name]] = mask

    fanout: Dict[str, Set[int]] = {}
    if s.skip_zone_nets:
        for n in sorted(wanted & board.zone_nets):
            zl = {lidx[l] for l in board.zone_layers.get(n, set()) if l in lidx}
            if s.fanout_zone_nets and zl:
                fanout[n] = zl
            else:
                skipped[n] = "has a copper zone (left to the pour)"
        wanted -= set(skipped)

    pads_by_net: Dict[str, List[Pad]] = {}
    for pad in board.pads:
        if pad.net:
            pads_by_net.setdefault(pad.net, []).append(pad)
    trk_by_net: Dict[str, List[Track]] = {}
    for t in board.tracks:
        trk_by_net.setdefault(t.net, []).append(t)
    via_by_net: Dict[str, List[Via]] = {}
    for v in board.vias:
        via_by_net.setdefault(v.net, []).append(v)

    anchors: Dict[int, Tuple[float, float]] = {}
    pad_cells_all: Set[int] = set()           # grid cells inside pads (any net)

    def item_cells(kind, it) -> Tuple[Set[int], Tuple[float, float]]:
        """Grid cells covered by a pad / track / via (pads also record end-stub anchors)."""
        cells: Set[int] = set()
        if kind == "p":
            pl = [lidx[l] for l in it.layers if l in lidx]
            R = it.radius()
            xr, yr = grid.cells_near(it.x - R, it.y - R, it.x + R, it.y + R)
            inside = [(x, y) for y in yr for x in xr if it.distance(*grid.center(x, y)) <= 1e-9]
            # A track that ends anywhere inside the pad is connected (KiCad's rule), so no
            # extra piece is needed - only a pad too small to contain a grid point gets a
            # short stub from the nearest grid point to its centre.
            stub = not inside
            if stub:
                inside = [grid.cell_of(it.x, it.y)]
            for (x, y) in inside:
                for l in pl:
                    c = grid.idx(x, y, l)
                    cells.add(c)
                    pad_cells_all.add(c)
                    if stub:
                        anchors[c] = (it.x, it.y)
            return cells, (it.x, it.y)
        if kind == "t":
            mid = ((it.x1 + it.x2) / 2, (it.y1 + it.y2) / 2)
            if it.layer not in lidx:
                return cells, mid
            l = lidx[it.layer]
            L = math.hypot(it.x2 - it.x1, it.y2 - it.y1)
            steps = max(1, int(L / (grid.pitch / 3)))
            for k in range(steps + 1):
                x, y = grid.cell_of(it.x1 + (it.x2 - it.x1) * k / steps, it.y1 + (it.y2 - it.y1) * k / steps)
                cx, cy = grid.center(x, y)
                if seg_point_dist(cx, cy, it.x1, it.y1, it.x2, it.y2) <= it.width / 2:
                    cells.add(grid.idx(x, y, l))
            return cells, mid
        x, y = grid.cell_of(it.x, it.y)
        for name in (it.layers or layers):
            if name in lidx:
                cells.add(grid.idx(x, y, lidx[name]))
        return cells, (it.x, it.y)

    jobs: List[NetJob] = []
    notes: Dict[str, str] = {}
    for name in sorted(wanted):
        if name not in net_ids:
            skipped[name] = "not on the board"
            continue
        n = net_ids[name]
        clusters = _cluster(name, pads_by_net.get(name, []), trk_by_net.get(name, []),
                            via_by_net.get(name, []))
        terms: List[Terminal] = []
        if name not in fanout:
            # Loose copper (a track piece that touches no pad - e.g. an unfinished trace)
            # is NOT something to connect to: routing to it only grows a dead-end stub.
            # It stays on the board as an obstacle for other nets.
            loose = [cl for cl in clusters if not any(k == "p" for k, _ in cl)]
            if loose:
                notes[name] = (f"{len(loose)} loose track piece(s) not connected to any pad "
                               "were left alone (not routed to)")
                clusters = [cl for cl in clusters if any(k == "p" for k, _ in cl)]
        for cl in clusters:
            cells: Set[int] = set()
            pts = []
            label = ""
            cl_pads = []
            for kind, it in cl:
                c_, pt = item_cells(kind, it)
                if kind == "p":
                    # a grid point inside a fine-pitch pin, at its edge, can be too close to
                    # the neighbouring pin for a track centred there: not a connection point
                    c_ = {c for c in c_ if owner[c] in (FREE, n)}
                cells |= c_
                pts.append(pt)
                if kind == "p":
                    label = label or it.ref
                    cl_pads.append(it)
            if not cells and not cl_pads:
                continue
            for c in cells:   # terminal cells always belong to their net
                owner[c] = n
            cx = sum(p_[0] for p_ in pts) / len(pts)
            cy = sum(p_[1] for p_ in pts) / len(pts)
            terms.append(Terminal(cells, cx, cy, label, cl_pads))
        if name in fanout:
            zl = fanout[name]
            # only pads whose copper lies INSIDE the zone outline are connected by the pour;
            # every other pad (even one on the pour's layer) still has to be routed to it
            zmask = zone_masks.get(n)
            need = [t for t in terms if not (zmask and any(zmask[c] for c in t.cells))]
            if need:
                jobs.append(NetJob(name, n, need, fanout_layers=zl))
            else:
                skipped[name] = "connected by its copper zone"
            continue
        if len(terms) < 2:
            if not terms:
                skipped[name] = "no routable pads"
            elif len(pads_by_net.get(name, [])) < 2:
                skipped[name] = "single pad"
            else:
                skipped[name] = "already complete"
            continue
        jobs.append(NetJob(name, n, terms))

    # ---- copper pours: the pads/vias that rely on each pour (checked while routing,
    #      so that new tracks do not cut them off onto islands)
    zones: Dict[int, Tuple[Set[int], Set[int]]] = {}
    zone_vias: Dict[int, Set[int]] = {}
    for name, zlayers in board.zone_layers.items():
        if name not in net_ids:
            continue
        zl = {lidx[l] for l in zlayers if l in lidx}
        if not zl:
            continue
        cells: Set[int] = set()
        for kind, items in (("p", pads_by_net.get(name, [])), ("t", trk_by_net.get(name, [])),
                            ("v", via_by_net.get(name, []))):
            zmask = zone_masks.get(net_ids[name])
            for it in items:
                c_, _ = item_cells(kind, it)
                cells |= {c for c in c_ if c // grid.plane in zl and (zmask is None or zmask[c])}
        zones[net_ids[name]] = (zl, cells)
        # columns where the net's vias / through-hole pads join its pour layers together
        cols = set()
        for v in via_by_net.get(name, []):
            x_, y_ = grid.cell_of(v.x, v.y)
            cols.add(y_ * grid.nx + x_)
        for p_ in pads_by_net.get(name, []):
            if p_.drill > 0:
                x_, y_ = grid.cell_of(p_.x, p_.y)
                cols.add(y_ * grid.nx + x_)
        zone_vias[net_ids[name]] = cols

    # ---- via footprint (cells a via claims on every layer)
    R = vr + s.clearance + w2
    k = int(math.ceil(R / p))
    fp = [(dx, dy) for dy in range(-k, k + 1) for dx in range(-k, k + 1)
          if math.hypot(dx, dy) * p < R - 1e-9]

    _add_escapes(jobs, grid, owner, s, board, lidx, via_owner, anchors)
    for job in jobs:          # a pin nothing can reach: the net cannot be completed
        dead = [t_ for t_ in job.terminals if not t_.cells]
        if dead:
            job.terminals = [t_ for t_ in job.terminals if t_.cells]
            job.unreachable = len(dead)
            notes[job.name] = ((notes[job.name] + "; ") if job.name in notes else "") + \
                f"no clear way out of pad(s) {', '.join(p_.ref for t_ in dead for p_ in t_.pads)} on the grid"
    jobs = [j for j in jobs if j.terminals]
    for job in jobs:
        _net_class_claims(job, grid, owner, s, board, lidx, pad_cells_all)
        if job.name in s.net_rules:
            W, C = s.net_rules[job.name]
            msg = f"net class: {W:.3f} mm ({W / 0.0254:.0f} mil) track, {max(C, s.clearance):.3f} mm clearance"
            if job.neck_width < W:
                msg += f" (narrows to {job.neck_width:.3f} mm only at a pad with no room)"
            notes[job.name] = (notes[job.name] + "; " + msg) if job.name in notes else msg

    dirs = list(s.layer_directions) if s.layer_directions else \
        [("H" if i % 2 == 0 else "V") for i in range(grid.nl)]
    if grid.nl == 1:
        dirs = ["-"]

    return Problem(grid, s, owner, via_owner, jobs, net_ids, anchors, fp, dirs, skipped, zones, notes,
                   zone_masks, zone_vias)


def _net_class_claims(job: NetJob, grid: Grid, owner: List[int], s: RouteSettings,
                      board: Optional[Board] = None, lidx: Optional[Dict[str, int]] = None,
                      pad_cells: Optional[Set[int]] = None) -> None:
    """Rules for a net-class track that is wider than the grid's own (or needs more clearance).

    * Against other ROUTED nets it claims every cell within Chebyshev radius r of its
      centre-line: a neighbour track (grid width w) on a cell (r+1) pitches away must
      clear it, (r+1) p >= W/2 + w/2 + C.
    * Against FIXED copper (pads, existing tracks, vias, holes, board edge) it is checked
      exactly: every grid point and every half-step point of its centre-line must be at
      least W/2 + clearance from that copper (``job.half_bad``, a half-pitch map).
    * It keeps its width everywhere; only in the last cells before one of its own pads,
      where the neighbouring pads leave no room, may it narrow to the grid width."""
    W = s.width_for(job.name)
    C = s.clearance_for(job.name)
    w = s.track_width
    p = grid.pitch
    job.width = W
    job.neck_width = min(W, w)
    r = max(0, int(math.ceil(((W + w) / 2 + C) / p - 1.0 - 1e-6)))
    # the grid itself already guarantees clearance for a grid-width track
    if W <= w + 1e-9 and C <= s.clearance + 1e-9:
        r = 0
    job.claim_r = r
    if r == 0:
        return
    nx, ny, nl = grid.nx, grid.ny, grid.nl
    HX, HY = 2 * nx - 1, 2 * ny - 1
    hp = HX * HY
    hb = bytearray(nl * hp)                      # 1 = centre-line may not pass this point
    hgrid = Grid(grid.x0, grid.y0, p / 2, HX, HY, grid.layers)
    W2 = W / 2
    # sub-steps between half points are <= p*sqrt2/2 long: a small margin keeps their
    # middle clear too (worst case: a point-like obstacle)
    def grow(R):
        return math.sqrt(R * R + (p * math.sqrt(2) / 4) ** 2)

    def mark(x0, y0, x1, y1, too_close, layers_):
        xr, yr = hgrid.cells_near(x0, y0, x1, y1)
        for hy in yr:
            row = hy * HX
            for hx in xr:
                cx, cy = hgrid.center(hx, hy)
                if too_close(cx, cy):
                    for l in layers_:
                        hb[l * hp + row + hx] = 1

    all_l = range(nl)
    if board is not None:
        lidx = lidx or {name: i for i, name in enumerate(grid.layers)}
        ok = _outline_mask(hgrid, board.outline, s.edge_clearance + W2)
        for k in range(hp):
            if not ok[k]:
                for l in all_l:
                    hb[l * hp + k] = 1
        for pad in board.pads:
            if pad.net == job.name and pad.net:
                continue
            R = grow(W2 + max(C, s.clearance_for(pad.net)))
            pl = [lidx[l] for l in pad.layers if l in lidx]
            Rb = pad.radius() + R
            mark(pad.x - Rb, pad.y - Rb, pad.x + Rb, pad.y + Rb,
                 lambda x, y, pad=pad, R=R: pad.distance(x, y) < R, pl)
            if pad.drill > 0:
                Rh = pad.drill / 2 + grow(W2 + max(C, s.hole_clearance))
                mark(pad.x - Rh, pad.y - Rh, pad.x + Rh, pad.y + Rh,
                     lambda x, y, pad=pad, Rh=Rh: math.hypot(x - pad.x, y - pad.y) < Rh, all_l)
        for tr in board.tracks:
            if tr.net == job.name or tr.layer not in lidx:
                continue
            R = tr.width / 2 + grow(W2 + max(C, s.clearance_for(tr.net)))
            mark(min(tr.x1, tr.x2) - R, min(tr.y1, tr.y2) - R, max(tr.x1, tr.x2) + R, max(tr.y1, tr.y2) + R,
                 lambda x, y, tr=tr, R=R: seg_point_dist(x, y, tr.x1, tr.y1, tr.x2, tr.y2) < R,
                 [lidx[tr.layer]])
        for v in board.vias:
            if v.net == job.name:
                continue
            R = v.diameter / 2 + grow(W2 + max(C, s.clearance_for(v.net)))
            vl = [lidx[l] for l in (v.layers or grid.layers) if l in lidx]
            mark(v.x - R, v.y - R, v.x + R, v.y + R,
                 lambda x, y, v=v, R=R: math.hypot(x - v.x, y - v.y) < R, vl)
        for (hx_, hy_, hd) in board.holes:
            R = hd / 2 + grow(W2 + max(C, s.hole_clearance))
            mark(hx_ - R, hy_ - R, hx_ + R, hy_ + R,
                 lambda x, y, hx_=hx_, hy_=hy_, R=R: math.hypot(x - hx_, y - hy_) < R, all_l)
    else:      # no geometry (tests that build a Problem by hand): fall back to the owner map
        nid = job.id
        for l in all_l:
            for y in range(ny):
                for x in range(nx):
                    o = owner[(l * ny + y) * nx + x]
                    if o != FREE and o != nid:
                        for dy in range(-2 * r - 1, 2 * r + 2):
                            for dx in range(-2 * r - 1, 2 * r + 2):
                                hx, hy = 2 * x + dx, 2 * y + dy
                                if 0 <= hx < HX and 0 <= hy < HY:
                                    hb[l * hp + hy * HX + hx] = 1
    job.half_bad = hb
    job.half_x = HX
    job.half_plane = hp
    full = (1 << nx) - 1
    cellok = []
    for l in all_l:
        for y in range(ny):
            m = 0
            if r <= y < ny - r:                      # the claimed square stays on the grid
                base = l * hp + 2 * y * HX
                for x in range(r, nx - r):
                    if not hb[base + 2 * x]:
                        m |= 1 << x
            cellok.append(m)
    pads = [0] * (nl * ny)
    for t in job.terminals:          # only real pads - never along existing tracks
        for c in t.cells:
            if pad_cells is None or c in pad_cells:
                pads[c // nx] |= 1 << (c % nx)
    k = r + 2
    near = []
    for l in all_l:
        hz = []
        for y in range(ny):
            m = pads[l * ny + y]
            d = m
            for i in range(1, k + 1):
                d |= (m << i) | (m >> i)
            hz.append(d & full)
        for y in range(ny):
            d = 0
            for yy in range(max(0, y - k), min(ny, y + k + 1)):
                d |= hz[yy]
            near.append(d)
    job.neck = [nr & ~ok_ & full for nr, ok_ in zip(near, cellok)]   # narrow only where it must
    job.wide_ok = [(nr | ok_) & full for nr, ok_ in zip(near, cellok)]


def _seg_seg_dist(ax, ay, bx, by, cx, cy, dx, dy) -> float:
    def ccw(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    d1, d2 = ccw(cx, cy, dx, dy, ax, ay), ccw(cx, cy, dx, dy, bx, by)
    d3, d4 = ccw(ax, ay, bx, by, cx, cy), ccw(ax, ay, bx, by, dx, dy)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 and d2 and d3 and d4:
        return 0.0
    return min(seg_point_dist(ax, ay, cx, cy, dx, dy), seg_point_dist(bx, by, cx, cy, dx, dy),
               seg_point_dist(cx, cy, ax, ay, bx, by), seg_point_dist(dx, dy, ax, ay, bx, by))


def _add_escapes(jobs: List[NetJob], grid: Grid, owner: List[int], s: RouteSettings,
                 board: Board, lidx: Dict[str, int], via_owner: List[int],
                 anchors: Dict[int, Tuple[float, float]]) -> None:
    """Fine-pitch pads (QFN, TSSOP ...): the grid may have no legal cell next to the pad,
    because the gap to the neighbouring pins lies inside their clearance halo.  A person
    would run a short track straight off the pin.  Do the same: every grid cell near the
    pad that a straight stub from the pad centre reaches with full clearance (checked
    exactly against other nets' pads, tracks, vias and holes) becomes an extra target;
    the cells near the stub are booked when a route uses it, so other nets keep away.

    Also checks the old "nearest grid point" stub of a pad too narrow to hold a grid
    point: if that stub would pass too close to a neighbouring pin it is dropped and
    the pad gets proper escapes instead."""
    p = grid.pitch
    w2 = s.track_width / 2
    nx = grid.nx
    pads_all = board.pads
    all_stubs = []
    for job in jobs:
        n = job.id
        clj = s.clearance_for(job.name)
        for t in job.terminals:
            for pad in t.pads:
                if min(pad.w, pad.h) >= 1.5 * p:
                    continue                      # big enough: the grid reaches it anyway
                reach = pad.radius() + 3.0 * p
                near = [q for q in pads_all if (q.net != job.name or not q.net) and
                        abs(q.x - pad.x) < reach + q.radius() + 1.0 and abs(q.y - pad.y) < reach + q.radius() + 1.0]
                trk = [tr for tr in board.tracks if tr.net != job.name and
                       min(tr.x1, tr.x2) < pad.x + reach + 1 and max(tr.x1, tr.x2) > pad.x - reach - 1 and
                       min(tr.y1, tr.y2) < pad.y + reach + 1 and max(tr.y1, tr.y2) > pad.y - reach - 1]
                vias = [v for v in board.vias if v.net != job.name and
                        abs(v.x - pad.x) < reach + 1 and abs(v.y - pad.y) < reach + 1]
                holes = [h for h in board.holes if abs(h[0] - pad.x) < reach + 2 and abs(h[1] - pad.y) < reach + 2]

                def stub_ok(bx, by, lname, hw=w2, pad=pad, near=near, trk=trk, vias=vias, holes=holes):
                    L = math.hypot(bx - pad.x, by - pad.y)
                    steps = max(2, int(L / 0.03))
                    sx0, sx1 = min(pad.x, bx), max(pad.x, bx)
                    sy0, sy1 = min(pad.y, by), max(pad.y, by)
                    for q in near:
                        if lname not in q.layers:
                            continue
                        rq = q.radius() + hw + 1.0
                        if q.x < sx0 - rq or q.x > sx1 + rq or q.y < sy0 - rq or q.y > sy1 + rq:
                            continue
                        need = hw + max(clj, s.clearance_for(q.net)) - 1e-6
                        for k in range(steps + 1):
                            if q.distance(pad.x + (bx - pad.x) * k / steps, pad.y + (by - pad.y) * k / steps) < need:
                                return False
                    for tr in trk:
                        if tr.layer == lname and _seg_seg_dist(pad.x, pad.y, bx, by, tr.x1, tr.y1, tr.x2, tr.y2) \
                                < tr.width / 2 + hw + max(clj, s.clearance_for(tr.net)) - 1e-6:
                            return False
                    for v in vias:
                        if seg_point_dist(v.x, v.y, pad.x, pad.y, bx, by) \
                                < v.diameter / 2 + hw + max(clj, s.clearance_for(v.net)) - 1e-6:
                            return False
                    for (hx, hy, hd) in holes:
                        if seg_point_dist(hx, hy, pad.x, pad.y, bx, by) < hd / 2 + hw + max(clj, s.hole_clearance):
                            return False
                    return True

                # the pad's own grid cells
                mine = [c for c in t.cells
                        if math.hypot(*(a - b for a, b in zip(grid.center(*grid.xyl(c)[:2]), (pad.x, pad.y))))
                        <= pad.radius() + 1e-6]
                # a "nearest grid point" stub that would pass too close to a neighbour: drop it
                trapped = False
                for c in list(mine):
                    if anchors.get(c) == (pad.x, pad.y):
                        x0, y0, l0 = grid.xyl(c)
                        if not stub_ok(*grid.center(x0, y0), grid.layers[l0]):
                            t.cells.discard(c)
                            anchors.pop(c, None)
                            mine.remove(c)
                            trapped = True
                if not trapped:
                    # trapped if fewer than three free grid cells touch the pad's own cells
                    free_nb = set()
                    for c in mine:
                        x0, y0, l0 = grid.xyl(c)
                        for dy in (-1, 0, 1):
                            for dx in (-1, 0, 1):
                                if (dx or dy) and 0 <= x0 + dx < grid.nx and 0 <= y0 + dy < grid.ny:
                                    q = c + dy * nx + dx
                                    if q not in t.cells and owner[q] in (FREE, n):
                                        free_nb.add(q)
                    if len(free_nb) >= ESCAPE_FREE_MIN:
                        continue
                short = min(pad.w, pad.h)
                stub_widths = [s.track_width] + [wd for wd in (0.2, 0.18, 0.15, 0.127, 0.1)
                                                 if wd < s.track_width and wd <= short + 1e-9]
                xr, yr = grid.cells_near(pad.x - reach, pad.y - reach, pad.x + reach, pad.y + reach)
                cands = []
                for lname in pad.layers:
                    if lname not in lidx:
                        continue
                    l = lidx[lname]
                    for y in yr:
                        for x in xr:
                            c = grid.idx(x, y, l)
                            if c in t.cells or c in job.escapes:
                                continue
                            o = owner[c]
                            if o != FREE and o != n:
                                continue
                            bx, by = grid.center(x, y)
                            if math.hypot(bx - pad.x, by - pad.y) <= reach:
                                cands.append((c, l, lname, bx, by))
                # the normal width; only if no stub of that width fits anywhere, a narrower one
                # (like a designer necking down into a fine-pitch pin; never wider than the pad)
                chosen, ws = [], s.track_width
                for wd in stub_widths:
                    chosen = [cd for cd in cands if stub_ok(cd[3], cd[4], cd[2], wd / 2)]
                    if chosen:
                        ws = wd
                        break
                for (c, l, lname, bx, by) in chosen:
                    # cells another net's track centre must keep away from: closer than
                    # w + c to the stub (the grid's own rule for two tracks)
                    stub = []
                    R = ws / 2 + w2 + max(clj, s.clearance) - 1e-6
                    sxr, syr = grid.cells_near(min(pad.x, bx) - R, min(pad.y, by) - R,
                                               max(pad.x, bx) + R, max(pad.y, by) + R)
                    for yy in syr:
                        for xx in sxr:
                            ccx, ccy = grid.center(xx, yy)
                            if seg_point_dist(ccx, ccy, pad.x, pad.y, bx, by) < R:
                                stub.append(grid.idx(xx, yy, l))
                    job.escapes[c] = (pad.x, pad.y, stub, ws)
                    t.cells.add(c)
                    all_stubs.append((job, c, l, pad.x, pad.y, bx, by, clj))
                    # other nets' vias: keep them clear of the stub (fixed in advance)
                    Rv = s.via_diameter / 2 + max(clj, s.clearance) + ws / 2
                    vxr, vyr = grid.cells_near(min(pad.x, bx) - Rv, min(pad.y, by) - Rv,
                                               max(pad.x, bx) + Rv, max(pad.y, by) + Rv)
                    for yy in vyr:
                        for xx in vxr:
                            ccx, ccy = grid.center(xx, yy)
                            if seg_point_dist(ccx, ccy, pad.x, pad.y, bx, by) < Rv:
                                _mark(via_owner, yy * nx + xx, n)
    # two stubs of different nets that come closer than w + c cannot both be used: put
    # each one's end cell into the other's keep-out, so the router sees the conflict
    B = 2.0
    buckets: Dict[tuple, list] = {}
    for k, (job, c, l, ax, ay, bx, by, clj) in enumerate(all_stubs):
        for gx in range(int(min(ax, bx) // B) - 1, int(max(ax, bx) // B) + 2):
            for gy in range(int(min(ay, by) // B) - 1, int(max(ay, by) // B) + 2):
                buckets.setdefault((gx, gy), []).append(k)
    seen = set()
    for ks in buckets.values():
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                a, b = ks[i], ks[j]
                if (a, b) in seen:
                    continue
                seen.add((a, b))
                A, Bs = all_stubs[a], all_stubs[b]
                if A[0] is Bs[0] or A[2] != Bs[2]:
                    continue
                need = s.track_width + max(A[7], Bs[7]) - 1e-6
                if _seg_seg_dist(A[3], A[4], A[5], A[6], Bs[3], Bs[4], Bs[5], Bs[6]) < need:
                    A[0].escapes[A[1]][2].append(Bs[1])
                    Bs[0].escapes[Bs[1]][2].append(A[1])
