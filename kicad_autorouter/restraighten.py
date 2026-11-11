"""Straighten tracks that are already on the board (e.g. a saved result of a stopped run).

The router's clean-up (shortcuts, re-routing stretches, re-attaching dead-end branches)
normally runs on its own routes in memory.  This reads a net's existing tracks and vias
back onto the routing grid, rebuilds the routes from them, runs the same clean-up on the
board as it is, and returns new copper for those nets.

Only copper that lies on the routing grid can be re-shaped (everything the router
itself drew does).  Off-grid pieces - the short stubs into fine-pitch pins, tracks drawn
by hand - stay exactly as they are and are kept as fixed copper of their net.
"""
from __future__ import annotations

import copy
from typing import Dict, List, Optional, Set, Tuple

from .model import Board, RouteSettings, Track, Via
from .problem import build_problem
from .router import RouteResult, Router

TOL = 0.01      # mm: how close a track end must be to a grid point to count as on-grid


def _on_grid(g, x: float, y: float) -> Optional[Tuple[int, int]]:
    cx, cy = g.cell_of(x, y)
    if not (0 <= cx < g.nx and 0 <= cy < g.ny):
        return None
    gx, gy = g.center(cx, cy)
    return (cx, cy) if abs(gx - x) <= TOL and abs(gy - y) <= TOL else None


def _chains(edges: Dict[int, Set[int]], keep_nodes: Set[int]) -> List[List[int]]:
    """Split a graph of grid cells into simple chains between branch points / ends."""
    node = {c for c, nb in edges.items() if len(nb) != 2} | (keep_nodes & set(edges))
    seen: Set[Tuple[int, int]] = set()
    out: List[List[int]] = []
    for a in sorted(node):
        for b in sorted(edges[a]):
            if (a, b) in seen:
                continue
            chain = [a, b]
            seen.add((a, b))
            seen.add((b, a))
            prev, cur = a, b
            while cur not in node:
                nxt = [c for c in edges[cur] if c != prev]
                if not nxt:
                    break
                prev, cur = cur, nxt[0]
                seen.add((prev, cur))
                seen.add((cur, prev))
                chain.append(cur)
            out.append(chain)
    return out


def detect_grid(board: Board, settings: RouteSettings) -> RouteSettings:
    """The routing grid the existing tracks were drawn on (they must land on it to be
    re-shaped).  Tries the settings' own pitch first, then common pitches; also takes
    the most common track width.  Returns a copy of the settings."""
    s = copy.copy(settings)
    pts = [(t.x1, t.y1) for t in board.tracks] + [(t.x2, t.y2) for t in board.tracks]
    if not pts:
        return s
    bx0, by0, _, _ = board.bbox()

    def fit(p_):
        ok = 0
        for (x, y) in pts:
            kx, ky = round((x - bx0) / p_), round((y - by0) / p_)
            if abs(bx0 + kx * p_ - x) <= TOL and abs(by0 + ky * p_ - y) <= TOL:
                ok += 1
        return ok / len(pts)
    best = (fit(s.pitch()), s.pitch())
    if best[0] >= 0.8:
        return s                                          # the board's own rules fit: keep them
    if best[0] < 0.8:
        for k in range(20, 81):
            p_ = round(k * 0.0125, 4)                     # 0.25 .. 1.0 mm
            f = fit(p_)
            if f > best[0] + 1e-9:
                best = (f, p_)
    widths: Dict[float, int] = {}
    for t in board.tracks:
        widths[round(t.width, 4)] = widths.get(round(t.width, 4), 0) + 1
    if widths:
        w = max(widths, key=widths.get)
        if abs(w - s.track_width) > 1e-6 and best[1] > w:
            s.track_width = w
            s.clearance = round(best[1] - w, 4)
    if abs(best[1] - s.pitch()) > 1e-9:
        s.grid = best[1]
    return s


def straighten_existing(board: Board, settings: RouteSettings, nets: Optional[List[str]] = None,
                        callback=None):
    """Clean up the existing on-grid copper of `nets` (default: every net with tracks).

    Returns (result, replaced, fixed): `result` holds the new on-grid copper per net,
    `replaced` the existing Track / Via objects it replaces (delete those), `fixed` the
    off-grid tracks that stay exactly as they are."""
    s = settings
    with_tracks = sorted({t.net for t in board.tracks if t.net})
    names = [n for n in (nets or with_tracks) if n in set(with_tracks)]
    # the board without these nets' copper: obstacles are everything else
    b2 = copy.copy(board)
    sel = set(names)
    b2.tracks = [t for t in board.tracks if t.net not in sel]
    b2.vias = [v for v in board.vias if v.net not in sel]
    pb = build_problem(b2, s, names)
    g = pb.grid
    plane = g.plane
    lidx = {name: i for i, name in enumerate(g.layers)}
    r = Router(pb, callback or (lambda ev, d: None))
    fixed: Dict[str, List[Track]] = {}
    replaced: List = []
    by_name = {j.name: j for j in pb.nets}
    for name in names:
        job = by_name.get(name)
        if job is None:
            continue
        edges: Dict[int, Set[int]] = {}
        keep_nodes: Set[int] = set()

        def link(a: int, b: int) -> None:
            edges.setdefault(a, set()).add(b)
            edges.setdefault(b, set()).add(a)
        for t in board.tracks:
            if t.net != name or t.layer not in lidx:
                continue
            l = lidx[t.layer]
            a, b = _on_grid(g, t.x1, t.y1), _on_grid(g, t.x2, t.y2)
            dx, dy = (b[0] - a[0], b[1] - a[1]) if a and b else (0, 0)
            if not (a and b) or not (dx == 0 or dy == 0 or abs(dx) == abs(dy)) or (dx == 0 and dy == 0):
                fixed.setdefault(name, []).append(t)          # off-grid: keep it as it is
                for e in (a, b):
                    if e:
                        keep_nodes.add(l * plane + e[1] * g.nx + e[0])
                continue
            n = max(abs(dx), abs(dy))
            sx, sy = (dx > 0) - (dx < 0), (dy > 0) - (dy < 0)
            cells = [l * plane + (a[1] + k * sy) * g.nx + (a[0] + k * sx) for k in range(n + 1)]
            for c1, c2 in zip(cells, cells[1:]):
                link(c1, c2)
            replaced.append(t)
        for v in board.vias:
            if v.net != name:
                continue
            e = _on_grid(g, v.x, v.y)
            if not e:
                continue
            xy = e[1] * g.nx + e[0]
            replaced.append(v)
            on = [l for l in range(g.nl) if l * plane + xy in edges]
            if len(on) < 2:
                on = [0, g.nl - 1]
            for la, lb in zip(on, on[1:]):
                link(la * plane + xy, lb * plane + xy)
                keep_nodes.add(la * plane + xy)
                keep_nodes.add(lb * plane + xy)
        # pad cells and the ends of kept stubs must stay chain ends
        for t_ in job.terminals:
            keep_nodes |= t_.cells
        chains = [c for c in _chains(edges, keep_nodes) if len(c) >= 2]
        job.paths = chains
        res = set()
        for p in chains:
            res |= r._path_resources(p, job)
        job.resources = r._bookable(job, res)
        for c in job.resources:
            r.occ[c] += 1
        r._book_keepout(job)
        job.failed = 0
    r._straighten()
    result = r._result(True, [])
    # nets that had nothing on the grid keep their copper untouched
    done = {j.name for j in pb.nets if j.paths}
    replaced = [it for it in replaced if it.net in done]
    result.nets = {n: nr for n, nr in result.nets.items() if n in done}
    return result, replaced, fixed
