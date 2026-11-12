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
import math
from typing import Dict, List, Optional, Set, Tuple

from .model import Board, RouteSettings, Track, Via
from .problem import build_problem, seg_point_dist
from .router import RouteResult, Router

TOL = 0.002     # mm: how close a track end must be to a grid point to count as on-grid
                # (0.01 snapped a pin-to-pin link 0.01 mm off its own other pieces)


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


def _drop_self_loops(r) -> None:
    """Remove route pieces that start and end on the same pad group with nothing else
    joining them: copper that connects nothing (a little triangle of stubs back to one
    pin, left behind when pins were joined by a shorting bar)."""
    for job in r.pb.nets:
        if len(job.paths) < 1:
            continue
        term_of = {}
        for i, t in enumerate(job.terminals):
            for c in t.cells:
                term_of[c] = i
        keep = []
        for k, p in enumerate(job.paths):
            a, b = term_of.get(p[0]), term_of.get(p[-1])
            if a is not None and a == b and len(p) > 1:
                touched = {q[0] for j, q in enumerate(job.paths) if j != k} | \
                          {q[-1] for j, q in enumerate(job.paths) if j != k}
                if not any(c in touched for c in p[1:-1]):
                    continue                     # a loop back to its own pad: drop it
            keep.append(p)
        job.paths = keep


def _square_entries(pb, board: Board, sel: Set[str], s: RouteSettings) -> None:
    """For every fine-pitch pin of these nets: the grid points a stub can reach that
    starts at the pad centre, leaves the pin square and bends only clear of it, checked
    exactly against the copper that stays (pads, other tracks, holes, edge, keep-outs).
    pb.square_entries[job id] = [(pad, terminal, {cell: (bend points, keep-out cells)})].
    The clean-up moves each pin's trace onto one of them (Router._square_pins)."""
    from .pinentry import _Copper, axis_pin, square_stubs
    from .problem import FREE
    g = pb.grid
    lidx = {n: i for i, n in enumerate(g.layers)}
    fixed_t = [(t.x1, t.y1, t.x2, t.y2, t.layer, t.width, t.net) for t in board.tracks if t.net not in sel]
    fixed_v = [(v.x, v.y, v.diameter, v.net) for v in board.vias if v.net not in sel]
    cu = _Copper(board, s, fixed_t, fixed_v)
    w = s.track_width
    out = {}
    for job in pb.nets:
        if s.width_for(job.name) > w + 1e-9 or job.claim_r:
            continue
        R = w / 2 + w / 2 + max(s.clearance_for(job.name), s.clearance) - 1e-6
        lst = []
        for t in job.terminals:
            for pad in t.pads:
                if not axis_pin(pad):
                    continue
                reach = max(pad.w, pad.h) / 2 + 2.0
                entries = {}
                for lname in pad.layers:
                    if lname not in lidx:
                        continue
                    l = lidx[lname]
                    xr, yr = g.cells_near(pad.x - reach, pad.y - reach, pad.x + reach, pad.y + reach)
                    for y in yr:
                        for x in xr:
                            c = g.idx(x, y, l)
                            o = pb.owner[c]
                            if o != FREE and o != job.id:
                                continue
                            bx, by = g.center(x, y)
                            for poly in square_stubs(pad, bx, by, w):
                                if not all(cu.ok(job.name, lname, a[0], a[1], b[0], b[1], w)
                                           for a, b in zip(poly, poly[1:])):
                                    continue
                                xs, ys = [q[0] for q in poly], [q[1] for q in poly]
                                kx, ky = g.cells_near(min(xs) - R, min(ys) - R, max(xs) + R, max(ys) + R)
                                keep = [g.idx(xx, yy, l) for yy in ky for xx in kx
                                        if any(seg_point_dist(*g.center(xx, yy), a[0], a[1], b[0], b[1]) < R
                                               for a, b in zip(poly, poly[1:]))]
                                entries[c] = (poly[1:-1], keep)
                                break
                if entries:
                    lst.append((pad, t, entries))
        if lst:
            out[job.id] = lst
    pb.square_entries = out


def _drop_blocked_stubs(pb, board: Board, sel: Set[str], s: RouteSettings) -> None:
    from .problem import _seg_seg_dist
    g = pb.grid
    trk = [t for t in board.tracks if t.net in sel]
    vias = [v for v in board.vias if v.net in sel]
    for job in pb.nets:
        cl = s.clearance_for(job.name)
        for c, e in list(job.escapes.items()):
            x_, y_, l_ = g.xyl(c)
            lname = g.layers[l_]
            poly = [(e[0], e[1])] + (list(e[4]) if len(e) > 4 and e[4] else []) + [g.center(x_, y_)]
            hw = e[3] / 2
            xs, ys = [q[0] for q in poly], [q[1] for q in poly]
            x0, x1, y0, y1 = min(xs) - 2, max(xs) + 2, min(ys) - 2, max(ys) + 2
            bad = False
            for t in trk:
                if t.net == job.name or t.layer != lname or max(t.x1, t.x2) < x0 or min(t.x1, t.x2) > x1 \
                        or max(t.y1, t.y2) < y0 or min(t.y1, t.y2) > y1:
                    continue
                need = hw + t.width / 2 + max(cl, s.clearance_for(t.net)) - 1e-6
                if any(_seg_seg_dist(a[0], a[1], b[0], b[1], t.x1, t.y1, t.x2, t.y2) < need
                       for a, b in zip(poly, poly[1:])):
                    bad = True
                    break
            if not bad:
                for v in vias:
                    if v.net == job.name or not (x0 < v.x < x1 and y0 < v.y < y1):
                        continue
                    need = hw + v.diameter / 2 + max(cl, s.clearance_for(v.net)) - 1e-6
                    if any(seg_point_dist(v.x, v.y, a[0], a[1], b[0], b[1]) < need for a, b in zip(poly, poly[1:])):
                        bad = True
                        break
            if bad:
                del job.escapes[c]
                for t_ in job.terminals:
                    t_.cells.discard(c)


def _trim_dead_ends(r, held_cells: Optional[Dict[str, Set[int]]] = None) -> None:
    """Trim track that leads nowhere: a route end that touches no pad of its net, no other
    route of it, and (for a pour net) no pour - a stub left hanging in space.  Repeated,
    since trimming one piece can leave another one dangling."""
    plane = r.pb.grid.plane
    for job in r.pb.nets:
        if not job.paths:
            continue
        pads = set()
        for t in job.terminals:
            pads |= r._true_pad_cells(job, t)
        pads |= (held_cells or {}).get(job.name, set())     # ends of kept off-grid copper
        pour = bytes(r._pour_mask(job)) if job.fanout_layers is not None else None
        changed = True
        while changed:
            changed = False
            count: Dict[int, int] = {}
            for p in job.paths:
                for c in set(p):
                    count[c] = count.get(c, 0) + 1

            def held(c, p):
                if c in pads or count.get(c, 0) > 1 or (pour is not None and pour[c]):
                    return True
                # the other end of a via (a layer change right next to it)
                i = p.index(c)
                return any(0 <= j < len(p) and p[j] // plane != c // plane for j in (i - 1, i + 1))
            keep = []
            for p in job.paths:
                q = list(p)
                while len(q) >= 2 and not held(q[0], q):
                    q.pop(0)
                    changed = True
                while len(q) >= 2 and not held(q[-1], q):
                    q.pop()
                    changed = True
                if len(q) >= 2:
                    keep.append(q)
                elif q != p:
                    changed = True
            job.paths = keep


def straighten_existing(board: Board, settings: RouteSettings, nets: Optional[List[str]] = None,
                        callback=None, on_router=None):
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
    # (the twin-pin links stay on the board as they are: copper the other nets must keep
    # clear of - left out, the clean-up ran XTAL1 straight through three of them)
    from .pinentry import FIXED_LINKS, seg_key
    b2.tracks = [t for t in board.tracks if t.net not in sel or
                 seg_key(t.x1, t.y1, t.x2, t.y2, t.layer) in FIXED_LINKS]
    b2.vias = [v for v in board.vias if v.net not in sel]
    pb = build_problem(b2, s, names)
    g = pb.grid
    plane = g.plane
    lidx = {name: i for i, name in enumerate(g.layers)}
    # (the clean-up's events never cancel it: a front end that answers 'stop' while the
    # user's Stop is still set would otherwise end the clean-up after a stopped run)
    cb = (lambda ev, d: (callback(ev, d), None)[1]) if callback else (lambda ev, d: None)
    r = Router(pb, cb)
    fixed: Dict[str, List[Track]] = {}
    replaced: List = []
    held: Dict[str, Set[int]] = {}

    def job_cells(job):
        out = set()
        for t_ in job.terminals:
            out |= t_.cells
        return out
    by_name = {j.name: j for j in pb.nets}
    # the grid was built without these nets' copper, so the pins' stubs were checked
    # without it: a stub that runs too close to another of these nets' tracks or vias is
    # no way in (drawn, it passed 0.05 mm from a GND via)
    _drop_blocked_stubs(pb, board, sel, s)
    # (_square_entries is not computed any more: the stage that used it is not run)
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
            from .pinentry import FIXED_LINKS, seg_key
            if seg_key(t.x1, t.y1, t.x2, t.y2, t.layer) in FIXED_LINKS:
                fixed.setdefault(name, []).append(t)          # a twin link: exactly as drawn
                for e in (a, b):
                    if e:
                        keep_nodes.add(l * plane + e[1] * g.nx + e[0])
                continue
            if bool(a) != bool(b):
                # a pin stub (pad centre line -> grid point): the clean-up draws it again
                # wherever the route ends up meeting the pin - kept, the old one would be
                # left hanging in space once the route moved
                ga, (ox, oy) = (a, (t.x2, t.y2)) if a else (b, (t.x1, t.y1))
                c0 = l * plane + ga[1] * g.nx + ga[0]
                # ... but only if the clean-up would draw it just the same from that point
                # (the rebuilt board has other nets' copper in place, and may offer a
                # different stub for the cell, e.g. to the pad centre past another via)
                e_ = job.escapes.get(c0)
                same = (e_ is not None and math.hypot(e_[0] - ox, e_[1] - oy) <= 1e-3
                        and not (len(e_) > 4 and e_[4])) or \
                    (e_ is None and c0 in pb.anchors and
                     math.hypot(pb.anchors[c0][0] - ox, pb.anchors[c0][1] - oy) <= 1e-3)
                if same and any(c0 in t_.cells and any(pp.distance(ox, oy) <= 1e-3 for pp in t_.pads)
                                for t_ in job.terminals):
                    edges.setdefault(c0, set())
                    keep_nodes.add(c0)
                    replaced.append(t)
                    continue
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
        # pads joined by kept off-grid copper (shorting bars between neighbouring pins,
        # hand-drawn links) are ONE piece: merge their terminals, or the clean-up would
        # treat them as separate pads and could re-attach one to the other
        # (also through a chain of kept pieces: a pin-to-pin link has several)
        fx = fixed.get(name, [])
        if fx:
            par = list(range(len(fx)))

            def fnd(i_):
                while par[i_] != i_:
                    par[i_] = par[par[i_]]
                    i_ = par[i_]
                return i_
            for i_, t1 in enumerate(fx):
                for j_ in range(i_ + 1, len(fx)):
                    t2 = fx[j_]
                    if t1.layer == t2.layer and any(math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < 1e-3
                                                    for p1 in ((t1.x1, t1.y1), (t1.x2, t1.y2))
                                                    for p2 in ((t2.x1, t2.y1), (t2.x2, t2.y2))):
                        par[fnd(i_)] = fnd(j_)
            groups_: Dict[int, List[Track]] = {}
            for i_, t1 in enumerate(fx):
                groups_.setdefault(fnd(i_), []).append(t1)
            for grp_ in groups_.values():
                pts_ = [pt for t1 in grp_ for pt in ((t1.x1, t1.y1), (t1.x2, t1.y2))]
                hit = [t_ for t_ in job.terminals
                       if any(pp.distance(x_, y_) <= 1e-3 for pp in t_.pads for (x_, y_) in pts_)]
                if len(hit) >= 2:
                    keep_t = hit[0]
                    for other in hit[1:]:
                        keep_t.cells |= other.cells
                        keep_t.pads = list(keep_t.pads) + list(other.pads)
                        job.terminals.remove(other)
                    # a twin pair: its inner pin is no place to connect to (see build_problem)
                    from .pinentry import LINK_INNER, inner_key
                    inner_pads = [pp for pp in keep_t.pads if inner_key(name, pp) in LINK_INNER]
                    if inner_pads and len(inner_pads) < len(keep_t.pads):
                        drop_ = set()
                        for c_ in keep_t.cells:
                            e_ = job.escapes.get(c_)
                            pt_ = (e_[0], e_[1]) if e_ is not None else g.center(*g.xyl(c_)[:2])
                            if any(pp.distance(*pt_) <= 1e-6 for pp in inner_pads):
                                drop_.add(c_)
                        keep_t.cells -= drop_
                        keep_t.pads = [pp for pp in keep_t.pads if pp not in inner_pads]
        # pad cells and the ends of kept stubs must stay chain ends
        for t_ in job.terminals:
            keep_nodes |= t_.cells
        held[name] = {c for c in keep_nodes if c not in job_cells(job)}
        # copper that stays where it is (off-grid tracks and vias): a route end that
        # overlaps it is connected, not dangling
        reach = s.track_width / 2
        for t in fixed.get(name, []):
            R_ = t.width / 2 + reach
            xr, yr = g.cells_near(min(t.x1, t.x2) - R_, min(t.y1, t.y2) - R_, max(t.x1, t.x2) + R_, max(t.y1, t.y2) + R_)
            for y in yr:
                for x in xr:
                    cx, cy = g.center(x, y)
                    if seg_point_dist(cx, cy, t.x1, t.y1, t.x2, t.y2) < R_:
                        held[name].add(lidx[t.layer] * plane + y * g.nx + x)
        for v in board.vias:
            if v.net != name or _on_grid(g, v.x, v.y):
                continue
            R_ = v.diameter / 2 + reach
            xr, yr = g.cells_near(v.x - R_, v.y - R_, v.x + R_, v.y + R_)
            for y in yr:
                for x in xr:
                    cx, cy = g.center(x, y)
                    if (cx - v.x) ** 2 + (cy - v.y) ** 2 < R_ * R_:
                        for l in range(g.nl):
                            held[name].add(l * plane + y * g.nx + x)
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
    if on_router is not None:
        on_router(r)                 # e.g. the view draws this router's routes as they change
    r._straighten()
    _drop_self_loops(r)
    _trim_dead_ends(r, held)
    result = r._result(True, [])
    # an off-grid piece (a stub into a pin) that the new copper draws again in the same
    # place is replaced too - otherwise it would end up on the board twice
    for name, ts in list(fixed.items()):
        nr = result.nets.get(name)
        if nr is None:
            continue
        ends = set()
        for (x1, y1, x2, y2, layer) in nr.segments:
            ends.add((round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2), layer))
            ends.add((round(x2, 2), round(y2, 2), round(x1, 2), round(y1, 2), layer))
        keep_ = []
        for t in ts:
            if (round(t.x1, 2), round(t.y1, 2), round(t.x2, 2), round(t.y2, 2), t.layer) in ends:
                replaced.append(t)
            else:
                keep_.append(t)
        fixed[name] = keep_
    # nets that had nothing on the grid keep their copper untouched
    done = {j.name for j in pb.nets if j.paths}
    replaced = [it for it in replaced if it.net in done]
    result.nets = {n: nr for n, nr in result.nets.items() if n in done}
    return result, replaced, fixed


def straighten_result(board: Board, result: RouteResult, settings: RouteSettings, callback=None,
                      on_router=None):
    """Straighten a routing result together with the board it was routed on - exactly
    what 'Straighten existing tracks' does once the result is on the board (used for
    'Straighten, then keep' after a stop, so both give the same copper).

    Returns (final_result, replaced_board_items): write final_result, and delete the
    board's own existing tracks / vias listed in replaced_board_items (copper of the
    same nets that was there before and got re-shaped too)."""
    from .router import NetRoute
    s = settings
    new_t, new_v = [], []
    for name, nr in result.nets.items():
        for k, (x1, y1, x2, y2, layer) in enumerate(nr.segments):
            new_t.append(Track(name, x1, y1, x2, y2, nr.width(k, s.track_width), layer))
        for (x, y) in nr.vias:
            new_v.append(Via(name, x, y, s.via_diameter, s.via_drill))
    nets = sorted(n for n, nr in result.nets.items() if nr.segments or nr.vias)
    if not nets:
        return result, []
    b2 = copy.copy(board)
    b2.tracks = list(board.tracks) + new_t
    b2.vias = list(board.vias) + new_v
    res2, replaced, _fixed = straighten_existing(b2, s, nets, callback, on_router)
    gone = {id(x) for x in replaced}
    out = RouteResult({}, result.passes, result.converged, list(result.removed), dict(result.skipped),
                      result.cancelled)
    out.pour_warning, out.notes = result.pour_warning, dict(result.notes)
    for name, nr in result.nets.items():
        if name not in res2.nets:
            out.nets[name] = nr
            continue
        n2 = res2.nets[name]
        m = NetRoute(name, list(n2.segments), list(n2.vias), nr.complete, nr.failed_connections,
                     list(n2.widths), list(nr.why))
        for t in new_t:                     # pieces of the new copper that stayed as they were
            if t.net == name and id(t) not in gone:
                m.segments.append((t.x1, t.y1, t.x2, t.y2, t.layer))
                m.widths.append(t.width)
        for v in new_v:
            if v.net == name and id(v) not in gone:
                m.vias.append((v.x, v.y))
        out.nets[name] = m
    old_ids = {id(x) for x in board.tracks} | {id(x) for x in board.vias}
    # last touch: traces into fine-pitch pins leave the pin straight along its centre line
    # and turn once at 45 degrees (in real millimetres, checked against all other copper)
    from .pinentry import drop_redundant_exits, tidy_pin_entries
    kept_t = [(t.x1, t.y1, t.x2, t.y2, t.layer, t.width, t.net) for t in board.tracks if id(t) not in gone]
    kept_v = [(v.x, v.y, v.diameter, v.net) for v in board.vias if id(v) not in gone]
    try:
        # (loose copper - touching no pad - joins nothing for real, and may be deleted)
        from .problem import loose_copper
        lt_, lv_ = loose_copper(board)
        loose_ = {id(x) for x in lt_} | {id(x) for x in lv_}
        conn_t = [(t.x1, t.y1, t.x2, t.y2, t.layer, t.width, t.net) for t in board.tracks
                  if id(t) not in gone and id(t) not in loose_]
        conn_v = [(v.x, v.y, v.diameter, v.net) for v in board.vias if id(v) not in gone and id(v) not in loose_]
        drop_redundant_exits(board, out, s, conn_t, conn_v)     # needless second ways out of a pad
    except Exception:
        pass
    try:
        n_pin = tidy_pin_entries(board, out, s, kept_t, kept_v)
        from .pinentry import link_pads
        n_pin += link_pads(board, out, s, kept_t, kept_v)       # pin-to-pin links: one uniform shape
        if callback and n_pin:
            callback("pin_entries", {"redrawn": n_pin})
    except Exception:                      # a nicety: never lose a finished routing over it
        pass
    try:
        from .pinentry import reach_centres
        reach_centres(board, out, s, kept_t, kept_v)     # every trace into a pad runs to its centre
    except Exception:
        pass
    return out, [x for x in replaced if id(x) in old_ids]
