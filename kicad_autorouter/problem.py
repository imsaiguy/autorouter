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


@dataclass
class NetJob:
    name: str
    id: int
    terminals: List[Terminal]
    # filled in by the router
    fanout_layers: Optional[Set[int]] = None   # zone net: connect each terminal to these layers
    paths: List[List[int]] = field(default_factory=list)
    resources: Set[int] = field(default_factory=set)
    failed: int = 0           # connections that could not be routed at all

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


# --------------------------------------------------------------------------
def _mark(arr: List[int], i: int, net_id: int) -> None:
    cur = arr[i]
    if cur == FREE:
        arr[i] = net_id
    elif cur != net_id:
        arr[i] = BLOCKED


def _outline_mask(grid: Grid, outline: Sequence[Tuple[float, float, float, float]],
                  margin: float) -> List[bool]:
    """True for x/y columns that are inside the board and >= margin from its edge."""
    nx, ny = grid.nx, grid.ny
    ok = [False] * grid.plane
    if not outline:
        return [True] * grid.plane
    for y in range(ny):
        _, ymm = grid.center(0, y)
        xs = []
        for (x1, y1, x2, y2) in outline:
            if (y1 <= ymm < y2) or (y2 <= ymm < y1):
                xs.append(x1 + (ymm - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        for a, b in zip(xs[0::2], xs[1::2]):
            xr, _ = grid.cells_near(a, ymm, b, ymm)
            for x in xr:
                ok[y * nx + x] = True
    # remove the edge band
    for (x1, y1, x2, y2) in outline:
        xr, yr = grid.cells_near(min(x1, x2) - margin, min(y1, y2) - margin,
                                 max(x1, x2) + margin, max(y1, y2) + margin)
        for y in yr:
            for x in xr:
                cx, cy = grid.center(x, y)
                if seg_point_dist(cx, cy, x1, y1, x2, y2) < margin:
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
        r_trk = s.clearance + w2
        r_via = s.clearance + vr
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
            hole_r = pad.drill / 2 + s.clearance + w2
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
        r_trk = t.width / 2 + s.clearance + w2
        r_via = t.width / 2 + s.clearance + vr
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
        r_trk = v.diameter / 2 + s.clearance + w2
        r_via = v.diameter / 2 + s.clearance + vr
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
            for kind, it in cl:
                c_, pt = item_cells(kind, it)
                cells |= c_
                pts.append(pt)
                if kind == "p":
                    label = label or it.ref
            if not cells:
                continue
            for c in cells:   # terminal cells always belong to their net
                owner[c] = n
            cx = sum(p_[0] for p_ in pts) / len(pts)
            cy = sum(p_[1] for p_ in pts) / len(pts)
            terms.append(Terminal(cells, cx, cy, label))
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

    # ---- via footprint (cells a via claims on every layer)
    R = vr + s.clearance + w2
    k = int(math.ceil(R / p))
    fp = [(dx, dy) for dy in range(-k, k + 1) for dx in range(-k, k + 1)
          if math.hypot(dx, dy) * p < R - 1e-9]

    dirs = list(s.layer_directions) if s.layer_directions else \
        [("H" if i % 2 == 0 else "V") for i in range(grid.nl)]
    if grid.nl == 1:
        dirs = ["-"]

    return Problem(grid, s, owner, via_owner, jobs, net_ids, anchors, fp, dirs, skipped, zones, notes,
                   zone_masks)
