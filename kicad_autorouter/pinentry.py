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


def exit_side(pad: Pad, x_in: float, y_in: float, x_out: float, y_out: float) -> int:
    """Which side of the pad a trace from (x_in, y_in) inside it to (x_out, y_out) outside
    leaves through: 0 / 1 = the +x / -x side of the pad's own frame, 2 / 3 = +y / -y.
    (For a round pad: the quarter it leaves through.)"""
    L = math.hypot(x_out - x_in, y_out - y_in)
    n = max(4, int(L / 0.01))
    px, py = x_out, y_out
    for k in range(1, n + 1):                    # the first point outside the pad
        qx, qy = x_in + (x_out - x_in) * k / n, y_in + (y_out - y_in) * k / n
        if pad.distance(qx, qy) > 1e-9:
            px, py = qx, qy
            break
    lx, ly = pad.local(px, py)
    ax, ay = lx / max(pad.w / 2, 1e-6), ly / max(pad.h / 2, 1e-6)
    if abs(ax) >= abs(ay):
        return 0 if ax > 0 else 1
    return 2 if ay > 0 else 3


def short_end_sides(pad: Pad):
    """The two sides at the ends of a long pad (where a fine-pitch pin is entered)."""
    return (0, 1) if pad.w >= pad.h else (2, 3)


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
            if q.seg_distance(x1, y1, x2, y2) < need:
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
            for _ in range(3):              # one entry per round (at most one per end, and a
                r = _tidy_pin(name, nr, pad, layer, net_pads, cu, idx, s)   # never-ending
                if not r:                   # back-and-forth between two shapes is cut off)
                    break
                made += r
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
    # pieces of track that start inside this pin and leave it (a shorting bar to the
    # neighbouring pin is not one: it leaves by the long side, the trace by the end)
    starts = []
    for k, sg in enumerate(nr.segments):
        if sg[4] != layer:
            continue
        for e in (0, 1):
            x, y = (sg[0], sg[1]) if e == 0 else (sg[2], sg[3])
            ox, oy = (sg[2], sg[3]) if e == 0 else (sg[0], sg[1])
            if pad.distance(x, y) <= EPS and pad.distance(ox, oy) > EPS and \
                    not any(_bar_mate(q, pad) and q.distance(ox, oy) <= EPS for q in net_pads):
                starts.append((k, e))
    if not starts:
        return None
    if len(starts) > 2:
        return None                         # a pin as a junction of many traces: leave it
    if len(starts) == 2:
        # two traces: fine on two different ends of the pin; then each end on its own
        sides = []
        for k_, e_ in starts:
            sg = nr.segments[k_]
            a_ = (sg[0], sg[1]) if e_ == 0 else (sg[2], sg[3])
            b_ = (sg[2], sg[3]) if e_ == 0 else (sg[0], sg[1])
            sides.append(exit_side(pad, a_[0], a_[1], b_[0], b_[1]))
        if sides[0] == sides[1]:
            return None                     # two out of one side: the clean-up's business
    for k0, e0 in starts:
        r = _tidy_chain(name, nr, pad, layer, net_pads, cu, idx, s, k0, e0, ends, vias,
                        to_local, to_board, half)
        if r:
            return r
    return 0


def _tidy_chain(name, nr, pad, layer, net_pads, cu, idx, s, k0, e0, ends, vias, to_local, to_board, half):
    """Redraw the entry that leaves `pad` with segment k0 (end e0 inside the pad)."""
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
        if any(_bar_mate(q, pad) and q.distance(*nxt) <= EPS for q in net_pads):
            return None                     # it runs into the next pin (a bar): not an entry
        if any(q is not pad and q.distance(*nxt) <= EPS for q in net_pads):
            break                           # it runs into another part's pad: the trace ends there
        (ka, ea), (kb, eb) = here
        if ka == kb:
            break
        # the other segment at this joint; continue from the end of it that is here
        k, e = (kb, eb) if ka == k else (ka, ea)
    last_k = segs[-1]
    end_pt = chain[-1]
    end_is_via = (round(end_pt[0], 4), round(end_pt[1], 4)) in vias or \
        any(q is not pad and q.distance(*end_pt) <= EPS for q in net_pads)
    end_free = len(ends.get(_key(end_pt[0], end_pt[1], layer), [])) == 1   # a dead end
    # the rule for a fine-pitch rectangular pin: the trace starts at the pad centre and
    # leaves square with the pad (straight out, at right angles to the edge it crosses),
    # off-grid if need be; only once it is clear of the pad may it bend to its grid track
    cx, cy = pad.x, pad.y
    w_route = nr.width(segs[-1], s.track_width)
    w_min = min(nr.width(k_, s.track_width) for k_ in segs)
    sg0 = nr.segments[k0]
    q_in = (sg0[0], sg0[1]) if e0 == 0 else (sg0[2], sg0[3])
    q_out = (sg0[2], sg0[3]) if e0 == 0 else (sg0[0], sg0[1])
    side0 = exit_side(pad, q_in[0], q_in[1], q_out[0], q_out[1])
    # which edges to leave by: the one the trace leaves by now - but a trace slipping out
    # of a long side near a corner is first tried out of the nearer end
    sides = [side0]
    ends_ = short_end_sides(pad)
    if side0 not in ends_:
        lx_, ly_ = pad.local(*chain[-1])
        far = lx_ if ends_ == (0, 1) else ly_
        sides = [ends_[0] if far >= 0 else ends_[1], side0]
    # the other tracks at the last point, if it is a junction (the new piece must meet
    # them at 90 degrees or wider: a T, never a sharp Y)
    legs = []
    for (k_, e_) in ends.get(_key(end_pt[0], end_pt[1], layer), []):
        if k_ == segs[-1]:
            continue
        o = nr.segments[k_]
        far = (o[2], o[3]) if e_ == 0 else (o[0], o[1])
        legs.append((far[0] - end_pt[0], far[1] - end_pt[1]))
    best = None
    for fallback in (False, True):
        for side in sides:
            frame = _side_frame(pad, side)
            for i in (range(len(chain) - 1, 0, -1) if not fallback else range(1, len(chain))):
                r = _entry_shapes(pad, frame, chain, i, cx, cy, w_route, fallback,
                                  end_is_via or end_free, legs)
                if not r:
                    continue
                replaced = {idx[k_] for k_ in segs[:i]}
                for poly in r:
                    pieces = [(a_[0], a_[1], b_[0], b_[1]) for a_, b_ in zip(poly, poly[1:])]
                    for w in sorted({w_route, w_min}, reverse=True):
                        if all(cu.ok(name, layer, *pc, w, skip_tracks=replaced) for pc in pieces[1:]) and \
                                _first_ok(cu, name, layer, pieces[0], w, pad, replaced):
                            best = (i, pieces, w)
                            break
                    if best:
                        break
                if best:
                    break
            if best:
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


def _bar_mate(q: Pad, pad: Pad) -> bool:
    """Another pin of the SAME part (a shorting bar joins neighbours of one footprint);
    a track into another part's pad is an ordinary trace, entered properly at both ends."""
    return q is not pad and q.ref.split(".")[0] == pad.ref.split(".")[0] and bool(q.ref)


def _side_frame(pad: Pad, side: int):
    """(to_local, to_board, half): 'along' runs out of the pad through `side` (positive
    outwards), 'sd' across it; half = the pad's half size in that direction."""
    a = math.radians(pad.angle)
    ca, sa = math.cos(a), math.sin(a)
    axis_x = side in (0, 1)
    sgn = 1.0 if side in (0, 2) else -1.0
    half = (pad.w if axis_x else pad.h) / 2

    def to_local(x, y):
        lx, ly = pad.local(x, y)
        return (sgn * lx, ly) if axis_x else (sgn * ly, lx)

    def to_board(al, sd):
        kx, ky = (sgn * al, sd) if axis_x else (sd, sgn * al)
        return pad.x + kx * ca + ky * sa, pad.y - kx * sa + ky * ca
    return to_local, to_board, half


def _entry_shapes(pad, frame, chain, i, cx, cy, w, fallback, end_ok, legs=()):
    """Candidate new entries from the pad centre onto chain[i], leaving the pad square
    through the frame's side.  Clean shapes: straight out + one 45-degree jog; or
    straight out, 45 degrees, short straight piece.  Fallback: straight out until clear
    of the pad, then straight onto chain[i] (a bend of up to 90 degrees where it goes on)."""
    to_local, to_board, half = frame
    c = chain[i]
    al, sd = to_local(*c)
    clear = half + w / 2                       # a bend only where the track is clear of the pad
    if al <= clear + EPS:
        return None
    junction = i == len(chain) - 1 and not end_ok
    limit = 90.5 if fallback else TURN_LIMIT
    shapes = []
    if not fallback:
        a_al = al - abs(sd)
        if a_al >= clear - EPS:
            shapes.append([(cx, cy), to_board(a_al, 0.0), c])
        else:
            for gap in (0.0, 0.1, 0.2):
                run = al - (clear + gap)            # how far the 45-degree piece goes
                if run <= EPS or run > abs(sd) + EPS:
                    continue
                shapes.append([(cx, cy), to_board(clear + gap, 0.0), to_board(al, math.copysign(run, sd)), c])
    else:
        E = to_board(clear, 0.0)
        if pad.seg_distance(E[0], E[1], c[0], c[1]) < w / 2 - 1e-6:
            return None                        # it would run back along the pad
        shapes.append([(cx, cy), E, c])
    out = []
    for poly in shapes:
        poly = [q for j_, q in enumerate(poly)
                if j_ == 0 or math.hypot(q[0] - poly[j_ - 1][0], q[1] - poly[j_ - 1][1]) > EPS]
        if len(poly) < 2:
            continue
        if i < len(chain) - 1:                 # how the trace goes on from c
            nxt = chain[i + 1]
            v_in = (poly[-1][0] - poly[-2][0], poly[-1][1] - poly[-2][1])
            if _angle(v_in, (nxt[0] - c[0], nxt[1] - c[1])) > limit:
                continue
        if junction:                           # meeting other tracks: a T, never a sharp Y
            back = (poly[-2][0] - c[0], poly[-2][1] - c[1])
            if not legs or any(_angle(back, lg) < 89.5 for lg in legs):
                continue
        out.append(poly)
    return out


def _first_ok(cu: _Copper, name, layer, piece, w, pad: Pad, replaced) -> bool:
    """The piece out of the pin starts at the pad centre: check only the part outside
    its own pad (inside, it is the pad's own copper)."""
    return cu.ok(name, layer, *piece, w, skip_tracks=replaced)


def _same(old, new) -> bool:
    def norm(sg):
        a, b = (round(sg[0], 3), round(sg[1], 3)), (round(sg[2], 3), round(sg[3], 3))
        return (min(a, b), max(a, b))
    return sorted(map(norm, old)) == sorted(map(norm, new))


# ---------------------------------------------------------------------------------------
# needless second ways out of a pad: drop exits that only lead back to copper the pad
# already reaches.  Connectivity is real copper overlap (tracks, vias, pads, pours), so
# off-grid copper that overlaps a grid route counts as joined.
class _NetCopper:
    """One net's copper with a spatial index: segments (x1, y1, x2, y2, layer, width,
    removable), vias (x, y, diameter), pads, pour outlines per layer."""
    B = 1.0                                         # mm per index bucket

    def __init__(self, name, segs, vias, pads, zones):
        self.name, self.segs, self.vias, self.pads, self.zones = name, segs, vias, pads, zones
        self.idx: Dict[tuple, List[int]] = {}
        for k, sg in enumerate(segs):
            self._add(k, sg)

    def _cells(self, sg):
        r = sg[5] / 2 + 0.5
        for gx in range(int(math.floor((min(sg[0], sg[2]) - r) / self.B)), int(math.floor((max(sg[0], sg[2]) + r) / self.B)) + 1):
            for gy in range(int(math.floor((min(sg[1], sg[3]) - r) / self.B)), int(math.floor((max(sg[1], sg[3]) + r) / self.B)) + 1):
                yield (gx, gy, sg[4])

    def _add(self, k, sg):
        for c in self._cells(sg):
            self.idx.setdefault(c, []).append(k)

    def near(self, k, alive):
        sg = self.segs[k]
        out = set()
        for c in self._cells(sg):
            for j in self.idx.get(c, ()):
                if j != k and alive[j]:
                    out.add(j)
        return out

    def touch_ss(self, a, b):
        p, q = self.segs[a], self.segs[b]
        return p[4] == q[4] and _seg_seg_dist(p[0], p[1], p[2], p[3], q[0], q[1], q[2], q[3]) < (p[5] + q[5]) / 2 - 1e-6

    def touch_pt(self, k, x, y, alive) -> bool:
        """Does the end (x, y) of segment k overlap any other copper of the net?"""
        sg = self.segs[k]
        hw = sg[5] / 2
        for j in self.near(k, alive):
            o = self.segs[j]
            if o[4] != sg[4]:
                continue
            d = seg_point_dist(x, y, o[0], o[1], o[2], o[3])
            # the routing's own pieces meet at exact points; kept (off-grid) copper is
            # joined wherever the copper overlaps
            if d < 1e-3 or (not (sg[6] and o[6]) and d < hw + o[5] / 2 - 1e-6):
                return True
        for (vx, vy, vd) in self.vias:
            if math.hypot(x - vx, y - vy) < hw + vd / 2 - 1e-6:
                return True
        for p in self.pads:
            if sg[4] in p.layers and p.distance(x, y) < hw - 1e-6:
                return True
        if (self.name, sg[4]) in self.zones and any(_inside_poly(x, y, r) for r in self.zones[(self.name, sg[4])]):
            return True
        return False

    def groups(self, alive, pads=None) -> int:
        """Number of separate pieces the net's pads fall into (pads=None: all)."""
        pads = self.pads if pads is None else pads
        parent = {}

        def find(a):
            parent.setdefault(a, a)
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb
        live = [k for k in range(len(self.segs)) if alive[k]]
        for k in live:
            sg = self.segs[k]
            find(("s", k))
            for j in self.near(k, alive):
                if j > k and self.touch_ss(k, j):
                    union(("s", k), ("s", j))
            for vi, (vx, vy, vd) in enumerate(self.vias):
                if seg_point_dist(vx, vy, sg[0], sg[1], sg[2], sg[3]) < sg[5] / 2 + vd / 2 - 1e-6:
                    union(("s", k), ("v", vi))
        # (pours are NOT counted as joining anything: the fill is cleared round fine-pitch
        # pins and may not reach - trusting the zone outline, a GND pin's only real
        # connection was removed as 'needless')
        for pi, p in enumerate(pads):
            find(("p", pi))
            for k in live:
                sg = self.segs[k]
                if sg[4] in p.layers and (p.distance(sg[0], sg[1]) < sg[5] / 2 or p.distance(sg[2], sg[3]) < sg[5] / 2):
                    union(("p", pi), ("s", k))
            for vi, (vx, vy, vd) in enumerate(self.vias):
                if p.distance(vx, vy) < vd / 2:
                    union(("p", pi), ("v", vi))
        self._find = find
        return len({find(("p", i)) for i in range(len(pads))})

    def trim(self, alive) -> None:
        """Drop removable segments that dangle (an end overlapping nothing), repeatedly."""
        changed = True
        while changed:
            changed = False
            for k, sg in enumerate(self.segs):
                if not alive[k] or not sg[6]:
                    continue
                if not (self.touch_pt(k, sg[0], sg[1], alive) and self.touch_pt(k, sg[2], sg[3], alive)):
                    alive[k] = False
                    changed = True


def drop_redundant_exits(board: Board, result, settings: RouteSettings, other_tracks: Sequence = (),
                         other_vias: Sequence = ()) -> int:
    """Where two traces leave a pad and also meet away from it (a loop over the pad, a
    second way into the same track), the needless one goes (with whatever then dangles).
    A pad that really joins two separate pieces keeps both.  other_tracks / other_vias:
    the net's copper that stays besides `result` (it joins, it is never removed)."""
    s = settings
    zones = {}
    for net, lst in (board.zone_outlines or {}).items():
        for (layers, rings) in lst:
            for ly in layers:
                zones.setdefault((net, ly), []).extend(rings)
    made = 0
    for name, nr in result.nets.items():
        if len(nr.segments) < 2:
            continue
        pads = [p for p in board.pads if p.net == name]
        if not pads:
            continue
        n_own = len(nr.segments)
        segs = [(sg[0], sg[1], sg[2], sg[3], sg[4], nr.width(k, s.track_width), True)
                for k, sg in enumerate(nr.segments)]
        segs += [(t[0], t[1], t[2], t[3], t[4], t[5], False) for t in other_tracks if t[6] == name]
        vias = [(x, y, s.via_diameter) for x, y in nr.vias] + [(v[0], v[1], v[2]) for v in other_vias if v[3] == name]
        cu = _NetCopper(name, segs, vias, pads, zones)
        alive = [True] * len(segs)
        base = cu.groups(alive)
        net_made = 0
        for _ in range(50):                        # each round removes one needless exit
            best = None
            for p in pads:
                exits = []
                for k in range(n_own):
                    sg = segs[k]
                    if not alive[k] or sg[4] not in p.layers:
                        continue
                    for (x, y), (ox, oy) in (((sg[0], sg[1]), (sg[2], sg[3])), ((sg[2], sg[3]), (sg[0], sg[1]))):
                        if p.distance(x, y) <= 1e-6 and p.distance(ox, oy) > 1e-6 and \
                                not any(q is not p and q.distance(ox, oy) <= 1e-6 for q in pads):
                            exits.append(k)
                if len(exits) < 2:
                    continue
                # which exits meet each other without the pad?
                others = [q for q in pads if q is not p]
                cu.groups(alive, others)
                find = cu._find
                roots: Dict[object, List[int]] = {}
                for k in exits:
                    roots.setdefault(find(("s", k)), []).append(k)
                for grp in roots.values():
                    if len(grp) < 2:
                        continue
                    for k in grp:                  # try dropping each; keep the biggest clean-up
                        a2 = list(alive)
                        a2[k] = False
                        cu.trim(a2)
                        if cu.groups(a2) > base:
                            continue
                        gone = sum(math.hypot(segs[j][2] - segs[j][0], segs[j][3] - segs[j][1])
                                   for j in range(len(segs)) if alive[j] and not a2[j])
                        if best is None or gone > best[0]:
                            best = (gone, a2)
            if best is None:
                break
            alive = best[1]
            net_made += 1
        if net_made:
            keep = [k for k in range(n_own) if alive[k]]
            nr.segments = [nr.segments[k] for k in keep]
            nr.widths = [segs[k][5] for k in keep]
            made += net_made
    return made
