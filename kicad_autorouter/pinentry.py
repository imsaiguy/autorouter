"""Hand-style entries into fine-pitch pins - the last touch on a finished routing.

The router works on a grid, and a 0.5 mm pitch QFP or TSSOP does not sit on it: a trace
reached each pin wherever the grid allowed, often off the pin's centre line or at an odd
angle.  A person draws it differently: the trace leaves the pin straight, from its centre
along its axis, and then turns once at 45 degrees onto its way.

This pass does exactly that on the finished copper, in real millimetres (not on the grid).
For every trace that runs into a long, narrow fine-pitch pin, it looks along the trace for
the farthest point it can reach with "straight out of the pin, then one 45-degree jog"
such that the trace continues from there with a turn of 45 degrees or less - and replaces
the piece up to that point.  Every new piece is checked exactly against all other copper,
holes, keep-outs and the board edge; if nothing fits, the trace stays as it was.
"""
import math
from typing import Dict, List, Optional, Sequence, Tuple

from .model import Board, Pad, RouteSettings
from .problem import _inside_poly, _seg_seg_dist, seg_point_dist

AXIS_ASPECT = 1.5      # a pin this much longer than wide ...
FINE_SHORT = 0.6       # ... and at most this wide (mm) is entered along its axis
MAX_REACH = 3.0        # mm along the trace (from the pin) that may be redrawn
TURN_LIMIT = 45.5      # degrees: the trace must continue from the joint at most this sharply
EPS = 1e-6


def axis_pin(pad: Pad) -> bool:
    """A long, narrow fine-pitch pin (QFP, SOIC, TSSOP, QFN ...)."""
    if pad.poly or pad.shape == "circle":
        return False
    short, long_ = min(pad.w, pad.h), max(pad.w, pad.h)
    return short <= FINE_SHORT and long_ >= AXIS_ASPECT * short


def _frame(pad: Pad):
    """(to_local, to_board, half_length): the pin's axis is local 'along'."""
    a = math.radians(pad.angle)
    ca, sa = math.cos(a), math.sin(a)
    long_x = pad.w >= pad.h
    half = (pad.w if long_x else pad.h) / 2

    def to_local(x, y):
        lx, ly = pad.local(x, y)
        return (lx, ly) if long_x else (ly, lx)

    def to_board(al, sd):
        kx, ky = (al, sd) if long_x else (sd, al)
        return pad.x + kx * ca + ky * sa, pad.y - kx * sa + ky * ca
    return to_local, to_board, half


def _angle(v1, v2) -> float:
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    if l1 < EPS or l2 < EPS:
        return 0.0
    c = (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class _Copper:
    """Everything a new piece of track on one net must keep clear of."""

    def __init__(self, board: Board, s: RouteSettings, tracks, vias):
        self.board, self.s = board, s
        self.tracks = tracks            # [(x1, y1, x2, y2, layer, width, net)]
        self.vias = vias                # [(x, y, diameter, net)]

    def ok(self, net: str, layer: str, x1, y1, x2, y2, w: float, skip_tracks=()) -> bool:
        s, b = self.s, self.board
        hw = w / 2
        cl = s.clearance_for(net)
        x0, x9, y0, y9 = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
        L = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(L / 0.02))
        pts = [(x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n) for k in range(n + 1)]
        for q in b.pads:
            if q.net == net and q.net or layer not in q.layers:
                continue
            r = q.radius() + hw + 1.0
            if q.x < x0 - r or q.x > x9 + r or q.y < y0 - r or q.y > y9 + r:
                continue
            need = hw + max(cl, s.clearance_for(q.net), q.clearance) - EPS
            if any(q.distance(px, py) < need for px, py in pts):
                return False
        for k, (a1, b1, a2, b2, ly, tw, tn) in enumerate(self.tracks):
            if tn == net or ly != layer or k in skip_tracks:
                continue
            if max(a1, a2) < x0 - 2 or min(a1, a2) > x9 + 2 or max(b1, b2) < y0 - 2 or min(b1, b2) > y9 + 2:
                continue
            if _seg_seg_dist(x1, y1, x2, y2, a1, b1, a2, b2) < hw + tw / 2 + max(cl, s.clearance_for(tn)) - EPS:
                return False
        for (vx, vy, vd, vn) in self.vias:
            if vn == net:
                continue
            if seg_point_dist(vx, vy, x1, y1, x2, y2) < hw + vd / 2 + max(cl, s.clearance_for(vn)) - EPS:
                return False
        for (hx, hy, hd) in b.holes:
            if seg_point_dist(hx, hy, x1, y1, x2, y2) < hw + hd / 2 + max(cl, s.hole_clearance) - EPS:
                return False
        for (a1, b1, a2, b2) in b.outline:
            if _seg_seg_dist(x1, y1, x2, y2, a1, b1, a2, b2) < hw + s.edge_clearance - EPS:
                return False
        for (klayers, rings, no_t, _nv) in b.keepouts:
            if no_t and layer in klayers:
                for r_ in rings:
                    if any(_inside_poly(px, py, r_) for px, py in pts):
                        return False
                    for i in range(len(r_)):
                        (a1, b1), (a2, b2) = r_[i], r_[(i + 1) % len(r_)]
                        if _seg_seg_dist(x1, y1, x2, y2, a1, b1, a2, b2) < hw:
                            return False
        return True


def _key(x, y, layer):
    return (round(x, 4), round(y, 4), layer)


def tidy_pin_entries(board: Board, result, settings: RouteSettings, other_tracks: Sequence = (),
                     other_vias: Sequence = ()) -> int:
    """Redraw the entries into fine-pitch pins in `result` (changed in place).

    other_tracks / other_vias: copper that stays on the board besides `result` (the
    board's own tracks and vias that are kept), as (x1, y1, x2, y2, layer, width, net)
    and (x, y, diameter, net).  Returns the number of entries redrawn."""
    s = settings
    pins: Dict[str, List[Pad]] = {}
    for p in board.pads:
        if p.net and axis_pin(p):
            pins.setdefault(p.net, []).append(p)
    if not pins:
        return 0
    # all copper, as one list (indices into it for this result's own segments)
    tracks = list(other_tracks)
    vias = list(other_vias)
    own_idx: Dict[str, List[int]] = {}
    for name, nr in result.nets.items():
        for k, sg in enumerate(nr.segments):
            own_idx.setdefault(name, []).append(len(tracks))
            tracks.append((sg[0], sg[1], sg[2], sg[3], sg[4], nr.width(k, s.track_width), name))
        for (x, y) in nr.vias:
            vias.append((x, y, s.via_diameter, name))
    cu = _Copper(board, s, tracks, vias)
    made = 0
    for name, nr in result.nets.items():
        if name not in pins or not nr.segments:
            continue
        net_pads = [p for p in board.pads if p.net == name]
        made += _tidy_net(name, nr, pins[name], net_pads, cu, own_idx[name], s)
    return made


def _tidy_net(name, nr, pins, net_pads, cu: _Copper, idx: List[int], s: RouteSettings) -> int:
    made = 0
    for pad in pins:
        for layer in pad.layers:
            while True:                     # one entry per pass (the segment list changes)
                r = _tidy_pin(name, nr, pad, layer, net_pads, cu, idx, s)
                if r is None:
                    break
                made += r
                if not r:
                    break
    return made


def _segments_at(nr, layer):
    ends: Dict[tuple, List[Tuple[int, int]]] = {}
    for k, sg in enumerate(nr.segments):
        if sg[4] != layer:
            continue
        ends.setdefault(_key(sg[0], sg[1], layer), []).append((k, 0))
        ends.setdefault(_key(sg[2], sg[3], layer), []).append((k, 1))
    return ends


def _tidy_pin(name, nr, pad: Pad, layer: str, net_pads, cu: _Copper, idx, s) -> Optional[int]:
    """Redraw one entry into `pad` on `layer`.  1 = redrawn, 0 / None = nothing to do."""
    to_local, to_board, half = _frame(pad)
    ends = _segments_at(nr, layer)
    vias = {(round(x, 4), round(y, 4)) for x, y in nr.vias}
    # pieces of track that start inside this pin and leave it
    starts = []
    for k, sg in enumerate(nr.segments):
        if sg[4] != layer:
            continue
        for e in (0, 1):
            x, y = (sg[0], sg[1]) if e == 0 else (sg[2], sg[3])
            ox, oy = (sg[2], sg[3]) if e == 0 else (sg[0], sg[1])
            if pad.distance(x, y) <= EPS and pad.distance(ox, oy) > EPS:
                starts.append((k, e))
    if len(starts) != 1:
        return None                         # none, or several branches out of one pin: leave it
    k0, e0 = starts[0]
    # follow the trace out of the pin: a chain of points through simple joints
    sg = nr.segments[k0]
    p_in = (sg[0], sg[1]) if e0 == 0 else (sg[2], sg[3])
    chain = [p_in]
    segs = []
    k, e = k0, e0
    length = 0.0
    while True:
        sg = nr.segments[k]
        nxt = (sg[2], sg[3]) if e == 0 else (sg[0], sg[1])
        length += math.hypot(nxt[0] - chain[-1][0], nxt[1] - chain[-1][1])
        chain.append(nxt)
        segs.append(k)
        here = ends.get(_key(nxt[0], nxt[1], layer), [])
        if length > MAX_REACH or len(here) != 2 or (round(nxt[0], 4), round(nxt[1], 4)) in vias:
            break
        if any(q is not pad and q.distance(*nxt) <= EPS for q in net_pads):
            return None                     # it runs into another pin (a bar): not an entry
        (ka, ea), (kb, eb) = here
        if ka == kb:
            break
        # the other segment at this joint; continue from the end of it that is here
        k, e = (kb, eb) if ka == k else (ka, ea)
    last_k = segs[-1]
    end_pt = chain[-1]
    end_is_via = (round(end_pt[0], 4), round(end_pt[1], 4)) in vias
    end_free = len(ends.get(_key(end_pt[0], end_pt[1], layer), [])) == 1   # a dead end
    # already clean?  straight out of the centre along the axis, then at most one turn
    cx, cy = pad.x, pad.y
    w_route = nr.width(segs[-1], s.track_width)
    w_min = min(nr.width(k_, s.track_width) for k_ in segs)
    best = None
    for i in range(len(chain) - 1, 0, -1):
        c = chain[i]
        al, sd = to_local(*c)
        if abs(al) <= half + EPS:
            continue
        a_al = al - math.copysign(abs(sd), al)
        if a_al * al <= 0 or abs(a_al) < half - EPS:
            continue                         # the jog would start inside the pin
        A = to_board(a_al, 0.0)
        # how the trace goes on from c
        if i < len(chain) - 1:
            nxt = chain[i + 1]
            v_in = (c[0] - A[0], c[1] - A[1]) if math.hypot(c[0] - A[0], c[1] - A[1]) > EPS else \
                (A[0] - cx, A[1] - cy)
            if _angle(v_in, (nxt[0] - c[0], nxt[1] - c[1])) > TURN_LIMIT:
                continue
        elif not (end_is_via or end_free):
            continue                         # a branch point: its angles are not ours to change
        pieces = [(cx, cy, A[0], A[1])]
        if math.hypot(c[0] - A[0], c[1] - A[1]) > EPS:
            pieces.append((A[0], A[1], c[0], c[1]))
        replaced = {idx[k_] for k_ in segs[:i]}
        for w in sorted({w_route, w_min}, reverse=True):
            if all(cu.ok(name, layer, *pc, w, skip_tracks=replaced) for pc in pieces[1:]) and \
                    _first_ok(cu, name, layer, pieces[0], w, pad, replaced):
                best = (i, pieces, w)
                break
        if best:
            break
    if best is None:
        return 0
    i, pieces, w = best
    old = [nr.segments[k_] for k_ in segs[:i]]
    new = [(a, b_, c_, d, layer) for (a, b_, c_, d) in pieces]
    if _same(old, new):
        return 0
    drop = set(segs[:i])
    keep = [(sg, nr.width(k_, s.track_width)) for k_, sg in enumerate(nr.segments) if k_ not in drop]
    nr.segments = [sg for sg, _ in keep] + new
    nr.widths = [wd for _, wd in keep] + [w] * len(new)
    # the copper list follows (dropped pieces become empty, new ones are appended)
    for k_ in drop:
        t = cu.tracks[idx[k_]]
        cu.tracks[idx[k_]] = (t[0], t[1], t[0], t[1], "", 0.0, name)
    new_idx = [idx[k_] for k_ in range(len(idx)) if k_ not in drop]
    for (a, b_, c_, d, ly) in new:
        new_idx.append(len(cu.tracks))
        cu.tracks.append((a, b_, c_, d, ly, w, name))
    idx[:] = new_idx
    return 1


def _first_ok(cu: _Copper, name, layer, piece, w, pad: Pad, replaced) -> bool:
    """The piece out of the pin starts at the pad centre: check only the part outside
    its own pad (inside, it is the pad's own copper)."""
    return cu.ok(name, layer, *piece, w, skip_tracks=replaced)


def _same(old, new) -> bool:
    def norm(sg):
        a, b = (round(sg[0], 3), round(sg[1], 3)), (round(sg[2], 3), round(sg[3], 3))
        return (min(a, b), max(a, b))
    return sorted(map(norm, old)) == sorted(map(norm, new))
