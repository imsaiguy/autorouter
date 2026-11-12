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
MIN_JOG = 0.05         # mm: a sideways offset this small is no jog (a 0.01 mm step is a kink)
MIN_PIECE = 0.15       # mm: no straight piece between two bends shorter than this
EXIT_RUN = 0.15        # mm: a trace runs straight this far past the pad's edge before bending
                       # (bending right at the edge, it looked as if it stopped at the pad)
EPS = 1e-6


def entry_pad(pad: Pad) -> bool:
    """A pad whose traces leave it from the centre, square (all rectangular SMD pads up to
    a few mm: chip resistors and capacitors, crystals, SOT and connector pins - a trace
    ending on a pad's corner looked like a loose piece)."""
    if pad.poly or pad.shape == "circle" or pad.drill > 0:
        return False
    return max(pad.w, pad.h) <= 3.0


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

    B = 2.0                             # mm per bucket of the pad index

    def __init__(self, board: Board, s: RouteSettings, tracks, vias):
        self.board, self.s = board, s
        self.tracks = tracks            # [(x1, y1, x2, y2, layer, width, net)]
        self.vias = vias                # [(x, y, diameter, net)]
        self.pad_idx: Dict[tuple, list] = {}
        for q in board.pads:
            r = q.radius()
            for gx in range(int(math.floor((q.x - r) / self.B)), int(math.floor((q.x + r) / self.B)) + 1):
                for gy in range(int(math.floor((q.y - r) / self.B)), int(math.floor((q.y + r) / self.B)) + 1):
                    self.pad_idx.setdefault((gx, gy), []).append(q)

    def _pads_near(self, x0, y0, x9, y9, m):
        seen = set()
        for gx in range(int(math.floor((x0 - m) / self.B)), int(math.floor((x9 + m) / self.B)) + 1):
            for gy in range(int(math.floor((y0 - m) / self.B)), int(math.floor((y9 + m) / self.B)) + 1):
                for q in self.pad_idx.get((gx, gy), ()):
                    if id(q) not in seen:
                        seen.add(id(q))
                        yield q

    def ok(self, net: str, layer: str, x1, y1, x2, y2, w: float, skip_tracks=()) -> bool:
        s, b = self.s, self.board
        hw = w / 2
        cl = s.clearance_for(net)
        x0, x9, y0, y9 = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
        L = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(L / 0.02))
        pts = [(x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n) for k in range(n + 1)]
        for q in self._pads_near(x0, y0, x9, y9, hw + 1.0):
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
        if p.net and entry_pad(p):
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


def _guarded(name, nr, net_pads, cu: "_Copper", idx: List[int], s: RouteSettings, fn) -> int:
    """Run one redraw (fn() -> 1 if it changed something); undo it if any pad or via of
    the net that was joined to another is no longer (a redrawn entry cut +3.3V pins of
    a QFP off their vias - a nicety must never cost a connection)."""
    own = set(idx)
    fixed = [t for k, t in enumerate(cu.tracks) if t[6] == name and k not in own and t[4]]
    vias = [(v[0], v[1], v[2]) for v in cu.vias if v[3] == name]

    def part():
        segs = [(sg[0], sg[1], sg[2], sg[3], sg[4], nr.width(k, s.track_width), True)
                for k, sg in enumerate(nr.segments)]
        segs += [(t[0], t[1], t[2], t[3], t[4], t[5], False) for t in fixed]
        return _NetCopper(name, segs, vias, net_pads, {})
    nc0 = part()
    base = nc0.partition([True] * len(nc0.segs))
    snap = (list(nr.segments), list(nr.widths), list(cu.tracks), list(idx))
    r = fn()
    if not r:
        return 0
    nc1 = part()
    if nc1.keeps_joined(base, [True] * len(nc1.segs)):
        return r
    nr.segments, nr.widths = snap[0], snap[1]
    cu.tracks[:] = snap[2]
    idx[:] = snap[3]
    return 0


def _tidy_net(name, nr, pins, net_pads, cu: _Copper, idx: List[int], s: RouteSettings) -> int:
    made = 0
    for pad in pins:
        for layer in pad.layers:
            for _ in range(3):              # one entry per round (at most one per end, and a
                r = _guarded(name, nr, net_pads, cu, idx, s,     # never-ending back-and-forth
                             lambda: _tidy_pin(name, nr, pad, layer, net_pads, cu, idx, s))
                if not r:                   # between two shapes is cut off)
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
                if seg_key(*sg[:5]) not in FIXED_LINKS:     # (a twin link is drawn already)
                    starts.append((k, e))
    if not starts:
        return None
    # every trace that is alone on its side of the pad is redrawn (two out of one side are
    # the clean-up's business; a pad may be a junction of several, one per side)
    side_of = []
    for k_, e_ in starts:
        sg = nr.segments[k_]
        a_ = (sg[0], sg[1]) if e_ == 0 else (sg[2], sg[3])
        b_ = (sg[2], sg[3]) if e_ == 0 else (sg[0], sg[1])
        side_of.append(exit_side(pad, a_[0], a_[1], b_[0], b_[1]))
    starts = [st for st, sd_ in zip(starts, side_of) if side_of.count(sd_) == 1]
    if not starts:
        return None
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
    if side0 not in ends_ and axis_pin(pad):
        lx_, ly_ = pad.local(*chain[-1])
        far = lx_ if ends_ == (0, 1) else ly_
        sides = [ends_[0] if far >= 0 else ends_[1], side0]
    # an end the pin's twin link leaves by is taken: one trace per side
    link_sides = set()
    for sgl in nr.segments:
        if sgl[4] == layer and seg_key(*sgl[:5]) in FIXED_LINKS:
            for (xa, ya), (xb, yb) in (((sgl[0], sgl[1]), (sgl[2], sgl[3])), ((sgl[2], sgl[3]), (sgl[0], sgl[1]))):
                if pad.distance(xa, ya) <= EPS and pad.distance(xb, yb) > EPS:
                    link_sides.add(exit_side(pad, xa, ya, xb, yb))
    if link_sides:
        sides = [sd_ for sd_ in list(dict.fromkeys(sides + list(ends_))) if sd_ not in link_sides]
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
    for fallback in (False,):           # (an any-angle fallback left 50-60 degree bends)
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


def square_stubs(pad: Pad, bx: float, by: float, w: float):
    """Stub shapes from the centre of a fine-pitch pin to the grid point (bx, by) that
    obey the rule: start at the pad centre, leave square out of one of its ends, bend
    only once the track (width w) is clear of the pad.  Cleanest first: straight + one
    45-degree jog; straight, 45, short straight; straight out and then straight on.
    Each is a list of points (centre ... grid point)."""
    out = []
    for side in short_end_sides(pad):
        to_local, to_board, half = _side_frame(pad, side)
        al, sd = to_local(bx, by)
        for extra in (EXIT_RUN, 0.0):         # a visible straight run first, if it fits
            clear = half + w / 2 + extra
            if al <= clear + EPS:
                continue
            a_al = al - abs(sd)
            if abs(sd) <= MIN_JOG:
                out.append([(pad.x, pad.y), (bx, by)])
            elif a_al >= clear - EPS:
                out.append([(pad.x, pad.y), to_board(a_al, 0.0), (bx, by)])
            else:
                for gap in (0.0, 0.1, 0.2):
                    run = al - (clear + gap)
                    if EPS < run <= abs(sd) + EPS and run * math.sqrt(2) >= MIN_PIECE:
                        out.append([(pad.x, pad.y), to_board(clear + gap, 0.0), to_board(al, math.copysign(run, sd)), (bx, by)])
            E = to_board(clear, 0.0)
            if pad.seg_distance(E[0], E[1], bx, by) >= w / 2 - 1e-6:
                out.append([(pad.x, pad.y), E, (bx, by)])
    clean, seen = [], set()
    for poly in out:
        poly = [q for j, q in enumerate(poly) if j == 0 or math.hypot(q[0] - poly[j - 1][0], q[1] - poly[j - 1][1]) > EPS]
        key = tuple((round(q[0], 4), round(q[1], 4)) for q in poly)
        if len(poly) >= 2 and key not in seen:
            seen.add(key)
            clean.append(poly)
    return clean


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
    if al <= half + w / 2 + EPS:
        return None
    junction = i == len(chain) - 1 and not end_ok
    limit = 90.5 if fallback else TURN_LIMIT
    shapes = []
    # a bend only where the track is clear of the pad - and, where it fits, a visible
    # straight run past the pad's edge first (bending right at the edge, the trace
    # looked as if it stopped short of the pad)
    for extra in (EXIT_RUN, 0.0):
        clear = half + w / 2 + extra
        if al <= clear + EPS:
            continue
        if not fallback:
            a_al = al - abs(sd)
            if abs(sd) <= MIN_JOG:
                shapes.append([(cx, cy), c])          # straight out (a hair off the centre line)
            elif a_al >= clear - EPS:
                shapes.append([(cx, cy), to_board(a_al, 0.0), c])
            else:
                for gap in (0.0, 0.1, 0.2):
                    run = al - (clear + gap)            # how far the 45-degree piece goes
                    if run <= EPS or run > abs(sd) + EPS:
                        continue
                    if run * math.sqrt(2) < MIN_PIECE:
                        continue                        # the 45-degree piece would be a speck
                    if EPS < abs(sd) - run < MIN_PIECE:
                        continue                        # the sideways piece would be a kink
                    shapes.append([(cx, cy), to_board(clear + gap, 0.0), to_board(al, math.copysign(run, sd)), c])
                if extra == 0.0 and end_ok and i == len(chain) - 1 and abs(sd) >= MIN_PIECE:
                    # a via (or open end) right beside the pad, too close for a 45: straight
                    # out, then one clean corner into it - never a speck-sized 45 at the corner
                    shapes.append([(cx, cy), to_board(al, 0.0), c])
        else:
            E = to_board(clear, 0.0)
            if pad.seg_distance(E[0], E[1], c[0], c[1]) >= w / 2 - 1e-6:
                shapes.append([(cx, cy), E, c])    # (else it would run back along the pad)
    out, seen = [], set()
    for poly in shapes:
        poly = [q for j_, q in enumerate(poly)
                if j_ == 0 or math.hypot(q[0] - poly[j_ - 1][0], q[1] - poly[j_ - 1][1]) > EPS]
        key = tuple((round(q[0], 4), round(q[1], 4)) for q in poly)
        if len(poly) < 2 or key in seen:
            continue
        seen.add(key)
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

    def partition(self, alive):
        """Which pads and vias are joined: one root per pad, then per via (a via is what
        a group of pads hangs on a plane by - cutting it off left a GND pin row joined to
        itself by a shorting bar but no longer to the GND plane)."""
        self.groups(alive)
        return [self._find(("p", i)) for i in range(len(self.pads))] + \
            [self._find(("v", i)) for i in range(len(self.vias))]

    def keeps_joined(self, base_part, alive) -> bool:
        """Every pair of pads joined in base_part is still joined (counting pieces is not
        enough: a net already in two pieces stayed 'two' with a connector pin cut off)."""
        new = self.partition(alive)
        root_of = {}
        for b_root, n_root in zip(base_part, new):
            if root_of.setdefault(b_root, n_root) != n_root:
                return False
        return True

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
        segs = [(sg[0], sg[1], sg[2], sg[3], sg[4], nr.width(k, s.track_width),
                 seg_key(*sg[:5]) not in FIXED_LINKS) for k, sg in enumerate(nr.segments)]
        segs += [(t[0], t[1], t[2], t[3], t[4], t[5], False) for t in other_tracks if t[6] == name]
        vias = [(x, y, s.via_diameter) for x, y in nr.vias] + [(v[0], v[1], v[2]) for v in other_vias if v[3] == name]
        cu = _NetCopper(name, segs, vias, pads, zones)
        alive = [True] * len(segs)
        base = cu.partition(alive)
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
                            if sg[6]:                    # (a twin link is never 'needless')
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
                        if not cu.keeps_joined(base, a2):
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


# ---------------------------------------------------------------------------------------
# pad-to-pad links between fine-pitch pins of two parts (a QFN inside a TQFP footprint,
# a connector row next to a chip ...): drawn the way it is done by hand - straight out of
# the centre of one pin along its axis, one 45-degree jog in the middle of the gap,
# straight into the centre of the other pin.  The same shape for every link, so a row
# of them looks uniform.
def _long_axis(pad: Pad):
    a = math.radians(pad.angle)
    ca, sa = math.cos(a), math.sin(a)
    # local x -> board (ca, -sa); local y -> board (sa, ca)
    return ((ca, -sa), pad.w / 2) if pad.w >= pad.h else ((sa, ca), pad.h / 2)


def link_poly(A: Pad, B: Pad, w: float):
    """The hand-drawn link between two parallel fine-pitch pins A and B (B off the end of
    A): from the centre of A straight along its axis, one 45-degree jog in the middle of
    the gap (or, when they are further apart sideways than the gap is long, a symmetric
    dogleg: 45, straight across, 45 back), straight into the centre of B; bends only
    clear of both pins.  The points, or None if it does not fit."""
    if max(B.w, B.h) > max(A.w, A.h) + 1e-6:
        A, B = B, A                          # measured from the bigger pin: its jogs line up
        rev = True
    else:
        rev = False
    (ux, uy), hA = _long_axis(A)
    (vx, vy), hB = _long_axis(B)
    if abs(ux * vy - uy * vx) > 1e-3:
        return None                          # not parallel
    dx, dy = B.x - A.x, B.y - A.y
    along = dx * ux + dy * uy
    if along < 0:
        ux, uy, along = -ux, -uy, -along
    nx_, ny_ = -uy, ux
    sd = dx * nx_ + dy * ny_
    lo, hi = hA + w / 2, along - hB - w / 2          # where bends may go (clear of both pins)

    def at(al, side):
        return (A.x + ux * al + nx_ * side, A.y + uy * al + ny_ * side)
    sg_ = 1.0 if sd >= 0 else -1.0
    if abs(sd) <= MIN_JOG:
        poly = [(A.x, A.y), (B.x, B.y)]
    elif hi - lo <= 1e-6:
        return None                          # no gap between the pins at all
    elif hi - lo >= abs(sd) - 1e-6:
        # the jog right where the bigger pin ends (as drawn by hand): the links of a row
        # of pins then run side by side, parallel, instead of crossing each other's way
        poly = [(A.x, A.y), at(lo, 0.0), at(lo + abs(sd), sd), (B.x, B.y)]
    else:
        # (the piece across at least MIN_PIECE long: a 0.025 mm one is a kink)
        t = min((hi - lo) / 2, (abs(sd) - MIN_PIECE) / 2)
        if t <= MIN_JOG:
            return None
        poly = [(A.x, A.y), at(lo, 0.0), at(lo + t, sg_ * t), at(lo + t, sd - sg_ * t),
                at(lo + 2 * t, sd), (B.x, B.y)]
    poly = [q for j_, q in enumerate(poly) if j_ == 0 or math.hypot(q[0] - poly[j_ - 1][0], q[1] - poly[j_ - 1][1]) > EPS]
    return poly[::-1] if rev else poly


FIXED_LINKS: set = set()      # the twin links of the current routing: never reshaped later
LINK_BIG: dict = {}           # (net, x, y) of the bigger pin -> unit vector of its link's way out
LINK_INNER: set = set()       # (net, x, y) of the smaller pin of each twin pair: no route starts
                              # there - the pair is reached through the bigger pin only


def inner_key(net, pad):
    return (net, round(pad.x, 3), round(pad.y, 3))


def seg_key(x1, y1, x2, y2, layer):
    a, b = (round(x1, 3), round(y1, 3)), (round(x2, 3), round(y2, 3))
    return (min(a, b), max(a, b), layer)


def bracket_poly(A: Pad, B: Pad, w: float, toward: float):
    """Two parallel pins of one part side by side (USB-C D+ on A6 and B6, another pin
    between them), joined as by hand: out of both centres square past the pin ends
    (toward = +1 / -1: which end), a 45-degree corner, straight across, a 45-degree
    corner back.  The points (A centre ... B centre), or None."""
    (ux, uy), hA = _long_axis(A)
    (vx, vy), hB = _long_axis(B)
    if abs(ux * vy - uy * vx) > 1e-3 or abs(hA - hB) > 1e-3:
        return None
    dx, dy = B.x - A.x, B.y - A.y
    if abs(dx * ux + dy * uy) > 0.05:
        return None                          # not side by side (one further out than the other)
    D = math.hypot(dx, dy)
    if not 0.4 < D <= 1.6:
        return None
    nx_, ny_ = dx / D, dy / D
    ux, uy = ux * toward, uy * toward
    k = hA + w / 2 + 0.05
    t = min(0.3, (D - MIN_PIECE) / 2)
    if t <= MIN_JOG:
        return None

    def pt(base, al, side):
        return (base.x + ux * al + nx_ * side, base.y + uy * al + ny_ * side)
    return [(A.x, A.y), pt(A, k, 0.0), pt(A, k + t, t), pt(B, k + t, -t), pt(B, k, 0.0), (B.x, B.y)]


def twin_links(board: Board, settings: RouteSettings, nets=None) -> list:
    """Before routing: a fine-pitch pin with a parallel twin of the same net on ANOTHER
    part right off its end (a QFN inside a TQFP footprint, two packages for one chip) is
    joined to it by the hand-drawn link (link_poly), as a person would - the router then
    takes the pair as one piece and leaves from wherever there is room, instead of
    threading the inner pin out through the gap.  Each pin at most once, every link
    checked against all other copper.  Returns Track objects (like shorting bars)."""
    from .model import Track
    s = settings
    want = set(nets) if nets is not None else None
    pins: Dict[str, List[Pad]] = {}
    for p in board.pads:
        if p.net and axis_pin(p) and len(p.layers) == 1 and (want is None or p.net in want) and \
                s.width_for(p.net) <= s.track_width + 1e-9:
            pins.setdefault(p.net, []).append(p)
    pairs = []
    for net, lst in pins.items():
        for i, A in enumerate(lst):
            for B in lst[i + 1:]:
                if A.layers != B.layers or A.ref.split(".")[0] == B.ref.split(".")[0]:
                    continue
                d = math.hypot(A.x - B.x, A.y - B.y)
                if d <= 4.0:
                    pairs.append((d, net, A, B))
    pairs.sort(key=lambda t: t[0])
    cu = _Copper(board, s, [(t.x1, t.y1, t.x2, t.y2, t.layer, t.width, t.net) for t in board.tracks],
                 [(v.x, v.y, v.diameter, v.net) for v in board.vias])
    used = set()
    out = []
    FIXED_LINKS.clear()
    LINK_INNER.clear()
    LINK_BIG.clear()
    for d, net, A, B in pairs:
        if id(A) in used or id(B) in used:
            continue
        w = min(s.track_width, A.w, A.h, B.w, B.h)
        poly = link_poly(A, B, w)
        if poly is None:
            continue
        layer = next(iter(A.layers))
        # it must not run over another pin of the net's own parts (a link through a third pad)
        if any(q is not A and q is not B and q.net == net and layer in q.layers and
               any(q.seg_distance(a[0], a[1], b[0], b[1]) < w / 2 for a, b in zip(poly, poly[1:]))
               for q in board.pads):
            continue
        if not all(cu.ok(net, layer, a[0], a[1], b[0], b[1], w) for a, b in zip(poly, poly[1:])):
            continue
        used.add(id(A))
        used.add(id(B))
        small = A if max(A.w, A.h) < max(B.w, B.h) else B
        big = B if small is A else A
        LINK_INNER.add(inner_key(net, small))
        q0 = poly[1] if (abs(poly[0][0] - big.x) < 1e-6 and abs(poly[0][1] - big.y) < 1e-6) else poly[-2]
        L_ = math.hypot(q0[0] - big.x, q0[1] - big.y) or 1.0
        LINK_BIG[inner_key(net, big)] = ((q0[0] - big.x) / L_, (q0[1] - big.y) / L_)
        for a, b in zip(poly, poly[1:]):
            out.append(Track(net, a[0], a[1], b[0], b[1], w, layer))
            cu.tracks.append((a[0], a[1], b[0], b[1], layer, w, net))
            FIXED_LINKS.add(seg_key(a[0], a[1], b[0], b[1], layer))
    # pins of ONE part, same net, side by side with another pin between them (USB-C
    # D+ / D- on A6/B6, A7/B7): joined by a bracket past their ends, on the end that faces
    # the rest of the net; the routes then leave from the other ends
    side_pairs = []
    for net, lst in pins.items():
        for i, A in enumerate(lst):
            for B in lst[i + 1:]:
                if id(A) in used or id(B) in used or A.layers != B.layers or \
                        A.ref.split(".")[0] != B.ref.split(".")[0]:
                    continue
                d = math.hypot(A.x - B.x, A.y - B.y)
                if not 0.4 < d <= 1.6:
                    continue
                # only where another net's pin sits between them (the USB-C case) - next-door
                # pins get a shorting bar, and supply pins (GND, +3.3V ...) their own vias
                from .problem import supply_name
                if supply_name(net):
                    continue
                if not any(q.net != net and q.ref.split(".")[0] == A.ref.split(".")[0] and
                           seg_point_dist(q.x, q.y, A.x, A.y, B.x, B.y) < 0.1 for q in board.pads):
                    continue
                side_pairs.append((d, net, A, B))
    side_pairs.sort(key=lambda t: t[0])
    for d, net, A, B in side_pairs:
        if id(A) in used or id(B) in used:
            continue
        w = min(s.track_width, A.w, A.h, B.w, B.h)
        layer = next(iter(A.layers))
        others = [q for q in board.pads if q.net == net and q.ref.split(".")[0] != A.ref.split(".")[0]]
        (ux, uy), _h = _long_axis(A)
        if others:
            cx_ = sum(q.x for q in others) / len(others) - A.x
            cy_ = sum(q.y for q in others) / len(others) - A.y
            order = (1.0, -1.0) if cx_ * ux + cy_ * uy >= 0 else (-1.0, 1.0)
        else:
            order = (1.0, -1.0)
        for toward in order:
            poly = bracket_poly(A, B, w, toward)
            if poly is None:
                break
            if any(q is not A and q is not B and q.net == net and layer in q.layers and
                   any(q.seg_distance(a[0], a[1], b[0], b[1]) < w / 2 for a, b in zip(poly, poly[1:]))
                   for q in board.pads):
                continue
            if not all(cu.ok(net, layer, a[0], a[1], b[0], b[1], w) for a, b in zip(poly, poly[1:])):
                continue
            used.add(id(A))
            used.add(id(B))
            for P_ in (A, B):                 # the bracket's end of each pin is taken
                LINK_BIG[inner_key(net, P_)] = (ux * toward, uy * toward)
            for a, b in zip(poly, poly[1:]):
                out.append(Track(net, a[0], a[1], b[0], b[1], w, layer))
                cu.tracks.append((a[0], a[1], b[0], b[1], layer, w, net))
                FIXED_LINKS.add(seg_key(a[0], a[1], b[0], b[1], layer))
            break
    return out


def link_pads(board: Board, result, settings: RouteSettings, other_tracks: Sequence = (),
              other_vias: Sequence = ()) -> int:
    """Redraw plain traces between two parallel fine-pitch pins of different parts (same
    net, no branch or via on the way) as: centre -> straight -> one 45 in the middle of
    the gap -> straight -> centre.  Checked against all other copper; changed in place."""
    s = settings
    pins: Dict[str, List[Pad]] = {}
    for p in board.pads:
        if p.net and axis_pin(p):
            pins.setdefault(p.net, []).append(p)
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
        plist = pins.get(name)
        if not plist or len(plist) < 2 or not nr.segments:
            continue
        idx = own_idx[name]
        done = set()
        for A in plist:
            for layer in A.layers:
                r = _guarded(name, nr, [q for q in board.pads if q.net == name], cu, idx, s,
                             lambda: _link_from(name, nr, A, layer, plist, cu, idx, s, done))
                made += r
    return made


def _link_from(name, nr, A: Pad, layer, plist, cu, idx, s, done) -> int:
    made = 0
    for _ in range(4):
        ends = _segments_at(nr, layer)
        vias = {(round(x, 4), round(y, 4)) for x, y in nr.vias}
        found = None
        for k0, sg in enumerate(nr.segments):
            if sg[4] != layer:
                continue
            for e0 in (0, 1):
                p_in = (sg[0], sg[1]) if e0 == 0 else (sg[2], sg[3])
                p_out = (sg[2], sg[3]) if e0 == 0 else (sg[0], sg[1])
                if A.distance(*p_in) > EPS or A.distance(*p_out) <= EPS:
                    continue
                # walk the trace: does it run, unbranched, into another part's pin?
                segs, cur, k, e = [k0], p_out, k0, e0
                B = None
                for _step in range(40):
                    hitp = [q for q in plist if q is not A and q.distance(*cur) <= EPS]
                    if hitp:
                        B = hitp[0]
                        break
                    here = ends.get(_key(cur[0], cur[1], layer), [])
                    if len(here) != 2 or (round(cur[0], 4), round(cur[1], 4)) in vias:
                        break
                    (ka, ea), (kb, eb) = here
                    k, e = (kb, eb) if ka == k else (ka, ea)
                    o = nr.segments[k]
                    cur = (o[2], o[3]) if e == 0 else (o[0], o[1])
                    segs.append(k)
                if B is None or B.ref.split(".")[0] == A.ref.split(".")[0] or layer not in B.layers:
                    continue
                key = tuple(sorted((A.ref, B.ref)))
                if key in done:
                    continue
                found = (B, segs, key)
                break
            if found:
                break
        if not found:
            return made
        B, segs, key = found
        done.add(key)
        w = min(nr.width(k_, s.track_width) for k_ in segs)
        poly = link_poly(A, B, w)
        if poly is None:
            continue
        old = [nr.segments[k_] for k_ in segs]
        new = [(a[0], a[1], b[0], b[1], layer) for a, b in zip(poly, poly[1:])]
        if _same(old, new):
            continue
        replaced = {idx[k_] for k_ in segs}
        if not all(cu.ok(name, layer, a[0], a[1], b[0], b[1], w, skip_tracks=replaced) for a, b in zip(poly, poly[1:])):
            continue
        drop = set(segs)
        keep = [(sg, nr.width(k_, s.track_width)) for k_, sg in enumerate(nr.segments) if k_ not in drop]
        nr.segments = [sg for sg, _ in keep] + new
        nr.widths = [wd for _, wd in keep] + [w] * len(new)
        for k_ in drop:
            t = cu.tracks[idx[k_]]
            cu.tracks[idx[k_]] = (t[0], t[1], t[0], t[1], "", 0.0, name)
        new_idx = [idx[k_] for k_ in range(len(idx)) if k_ not in drop]
        for (a, b_, c_, d, ly) in new:
            new_idx.append(len(cu.tracks))
            cu.tracks.append((a, b_, c_, d, ly, w, name))
        idx[:] = new_idx
        made += 1
    return made
