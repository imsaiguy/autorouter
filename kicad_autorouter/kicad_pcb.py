"""Read a ``.kicad_pcb`` file into a :class:`Board`; write routed tracks back.

Works directly on the S-expression file (KiCad 6, 7, 8, 9), so the router can
run outside KiCad - and the result can be checked with KiCad's own DRC.
New items are appended as text, so everything else in the file is untouched.
"""
from __future__ import annotations

import fnmatch
import json
import math
import os
import re
import uuid
from typing import Dict, List, Optional, Tuple

from .model import Board, Pad, RouteSettings, Track, Via

# ----------------------------------------------------------------- s-expressions
_TOKEN = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))', re.S)


class Sym(str):
    """An unquoted atom (keyword or number)."""


def parse_sexpr(text: str):
    stack: List[list] = [[]]
    pos = 0
    n = len(text)
    while pos < n:
        m = _TOKEN.match(text, pos)
        if not m:
            if text[pos:].strip() == "":
                break
            raise ValueError(f"cannot parse near offset {pos}: {text[pos:pos + 40]!r}")
        pos = m.end()
        if m.group(1):
            stack.append([])
        elif m.group(2):
            lst = stack.pop()
            stack[-1].append(lst)
        elif m.group(3) is not None:
            stack[-1].append(m.group(3).replace('\\"', '"').replace("\\\\", "\\"))
        elif m.group(4) is not None:
            stack[-1].append(Sym(m.group(4)))
    return stack[0][0] if stack[0] else []


def _kids(node, name):
    return [c for c in node if isinstance(c, list) and c and c[0] == name]


def _kid(node, name):
    for c in node:
        if isinstance(c, list) and c and c[0] == name:
            return c
    return None


def _f(x) -> float:
    return float(x)


# ----------------------------------------------------------------- helpers
def _rot(x, y, deg):
    """KiCad rotation (CCW on screen, y down)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return x * c + y * s, -x * s + y * c


def _arc_points(sx, sy, mx, my, ex, ey, seg_len=0.5):
    """Approximate a 3-point arc with points."""
    ax, ay, bx, by, cx, cy = sx, sy, mx, my, ex, ey
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return [(sx, sy), (ex, ey)]
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    a0 = math.atan2(ay - uy, ax - ux)
    a1 = math.atan2(by - uy, bx - ux)
    a2 = math.atan2(cy - uy, cx - ux)

    def norm(a):
        while a < 0:
            a += 2 * math.pi
        while a >= 2 * math.pi:
            a -= 2 * math.pi
        return a
    s1 = norm(a1 - a0)
    s2 = norm(a2 - a0)
    sweep = s2 if s1 < s2 else s2 - 2 * math.pi
    n = max(4, int(abs(sweep) * r / seg_len))
    return [(ux + r * math.cos(a0 + sweep * k / n), uy + r * math.sin(a0 + sweep * k / n)) for k in range(n + 1)]



def _pts_xy(pts):
    """The points of a (pts ...) list in order: (xy) points and (arc) pieces (rounded
    corners of zone and polygon outlines - skipping them cut a board-wide GND plane
    down to a strip)."""
    out = []
    if not pts:
        return out
    for q in pts[1:]:
        if not isinstance(q, list) or not q:
            continue
        if q[0] == "xy":
            out.append((_f(q[1]), _f(q[2])))
        elif q[0] == "arc":
            st, md, en = _kid(q, "start"), _kid(q, "mid"), _kid(q, "end")
            if st and md and en:
                ap = _arc_points(_f(st[1]), _f(st[2]), _f(md[1]), _f(md[2]), _f(en[1]), _f(en[2]))
                if out and ap and abs(out[-1][0] - ap[0][0]) < 1e-9 and abs(out[-1][1] - ap[0][1]) < 1e-9:
                    ap = ap[1:]
                out += ap
    return out

def _pad_rings(pad, px, py, angle, w, h, kind):
    """Copper outline(s) of a custom or trapezoid pad, in board mm.

    Exact where the file allows; anything approximated (a stroke width, an arc, a
    bezier, a trapezoid's taper) is approximated OUTWARD, so the router can only
    over-estimate the copper, never place a track too close to it."""
    a = math.radians(angle)
    c, s_ = math.cos(a), math.sin(a)

    def world(ring):
        return [(px + lx * c + ly * s_, py - lx * s_ + ly * c) for (lx, ly) in ring]

    def rect(x0, y0, x1, y1, grow=0.0):
        x0, x1 = min(x0, x1) - grow, max(x0, x1) + grow
        y0, y1 = min(y0, y1) - grow, max(y0, y1) + grow
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    def disc(cx, cy, r, n=24):
        r = r / math.cos(math.pi / n)          # circumscribed: covers the true circle
        return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]

    if kind == "trapezoid":
        d = _kid(pad, "rect_delta")
        grow = max(abs(_f(d[1])), abs(_f(d[2]))) / 2 if d else 0.0
        return [world(rect(-w / 2, -h / 2, w / 2, h / 2, grow))]

    rings = []
    opts = _kid(pad, "options")
    anchor = _kid(opts, "anchor") if opts else None
    if anchor is not None and str(anchor[1]) == "circle":
        rings.append(world(disc(0.0, 0.0, min(w, h) / 2)))
    else:
        rings.append(world(rect(-w / 2, -h / 2, w / 2, h / 2)))
    prims = _kid(pad, "primitives")
    for g in (prims[1:] if prims else []):
        if not isinstance(g, list) or not g:
            continue
        wd = _kid(g, "width")
        half = _f(wd[1]) / 2 if wd else 0.0
        kind_ = str(g[0])
        try:
            if kind_ == "gr_poly":
                pts = _kid(g, "pts")
                xy = _pts_xy(pts)
                if len(xy) >= 3:
                    if half > 0:   # a stroked outline grows by half its width: use its box, grown
                        rings.append(world(rect(min(q[0] for q in xy), min(q[1] for q in xy),
                                                max(q[0] for q in xy), max(q[1] for q in xy), half)))
                    else:
                        rings.append(world(xy))
            elif kind_ == "gr_rect":
                st, en = _kid(g, "start"), _kid(g, "end")
                rings.append(world(rect(_f(st[1]), _f(st[2]), _f(en[1]), _f(en[2]), half)))
            elif kind_ == "gr_circle":
                ce, en = _kid(g, "center"), _kid(g, "end")
                r = math.hypot(_f(en[1]) - _f(ce[1]), _f(en[2]) - _f(ce[2]))
                rings.append(world(disc(_f(ce[1]), _f(ce[2]), r + half)))
            elif kind_ == "gr_line":
                st, en = _kid(g, "start"), _kid(g, "end")
                x1, y1, x2, y2 = _f(st[1]), _f(st[2]), _f(en[1]), _f(en[2])
                L = math.hypot(x2 - x1, y2 - y1) or 1e-9
                ux, uy = (x2 - x1) / L * half, (y2 - y1) / L * half     # along the line
                nx_, ny_ = -uy, ux                                     # across it
                rings.append(world([(x1 - ux + nx_, y1 - uy + ny_), (x2 + ux + nx_, y2 + uy + ny_),
                                    (x2 + ux - nx_, y2 + uy - ny_), (x1 - ux - nx_, y1 - uy - ny_)]))
            else:   # gr_arc, gr_curve, anything else: the box round all its points, grown
                pts = [(_f(q[1]), _f(q[2])) for q in g[1:]
                       if isinstance(q, list) and q and q[0] in ("start", "mid", "end", "center")]
                pts += _pts_xy(_kid(g, "pts"))
                if kind_ == "gr_arc" and len(pts) >= 3:
                    pts = _arc_points(pts[0][0], pts[0][1], pts[1][0], pts[1][1], pts[2][0], pts[2][1], 0.05)
                if pts:
                    rings.append(world(rect(min(q[0] for q in pts), min(q[1] for q in pts),
                                            max(q[0] for q in pts), max(q[1] for q in pts), half + 0.01)))
        except (TypeError, IndexError, ValueError):
            continue
    return rings



def _mask_ring(g_, kind):
    """Outline of a filled gr_poly / gr_rect / gr_circle (for solder-mask openings)."""
    fill = _kid(g_, "fill")
    if not (fill and len(fill) > 1 and str(fill[1]) in ("yes", "solid")):
        return []
    if kind == "gr_poly":
        return _pts_xy(_kid(g_, "pts"))
    if kind == "gr_rect":
        s_, e_ = _kid(g_, "start"), _kid(g_, "end")
        x0, y0, x1, y1 = _f(s_[1]), _f(s_[2]), _f(e_[1]), _f(e_[2])
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    c_, e_ = _kid(g_, "center"), _kid(g_, "end")
    cx, cy = _f(c_[1]), _f(c_[2])
    r_ = math.hypot(_f(e_[1]) - cx, _f(e_[2]) - cy)
    return [(cx + r_ * math.cos(2 * math.pi * k / 32), cy + r_ * math.sin(2 * math.pi * k / 32))
            for k in range(32)]

def _expand_layers(names, copper: List[str]) -> set:
    out = set()
    for n in names:
        n = str(n)
        if n in ("*.Cu", "F&B.Cu"):
            out |= set(copper) if n == "*.Cu" else {"F.Cu", "B.Cu"} & set(copper)
        elif n in copper:
            out.add(n)
    return out


# ----------------------------------------------------------------- reading
class PcbFile:
    """A parsed board file: keeps the original text so new items can be appended."""

    def __init__(self, path: str):
        self.path = path
        with open(path, encoding="utf-8") as fh:
            self.text = fh.read()
        self.tree = parse_sexpr(self.text)
        if not self.tree or self.tree[0] != "kicad_pcb":
            raise ValueError(f"{path} is not a kicad_pcb file")
        v = _kid(self.tree, "version")
        self.version = int(v[1]) if v else 0
        self.copper: List[str] = []
        lay = _kid(self.tree, "layers")
        if lay:
            for L in lay[1:]:
                if isinstance(L, list) and len(L) >= 3 and str(L[1]).endswith(".Cu"):
                    self.copper.append(str(L[1]))
        # KiCad lists F.Cu, In*.Cu..., B.Cu in stack order already
        self.net_codes: Dict[str, int] = {}
        self.names_only = False
        for n in _kids(self.tree, "net"):
            if len(n) >= 3:
                self.net_codes[str(n[2])] = int(n[1])

    # -------------------------------------------------------------- nets
    def _net_of(self, node) -> str:
        n = _kid(node, "net")
        if not n:
            return ""
        if len(n) >= 3:
            return str(n[2])
        if len(n) == 2:
            if isinstance(n[1], Sym) and re.fullmatch(r"-?\d+", n[1]):
                code = int(n[1])
                for name, c in self.net_codes.items():
                    if c == code:
                        return name
                return ""
            self.names_only = True
            return str(n[1])
        return ""

    # -------------------------------------------------------------- board
    def board(self) -> Board:
        cu = self.copper
        b = Board(list(cu), [])
        # outline
        for node in self.tree:
            if not (isinstance(node, list) and node and str(node[0]).startswith("gr_")):
                continue
            layer = _kid(node, "layer")
            if not layer or str(layer[1]) != "Edge.Cuts":
                continue
            b.outline += _edge_segments(node)
        for fp in _kids(self.tree, "footprint") + _kids(self.tree, "module"):
            at = _kid(fp, "at")
            fx, fy = _f(at[1]), _f(at[2])
            frot = _f(at[3]) if len(at) > 3 and not isinstance(at[3], list) else 0.0
            refnode = [c for c in _kids(fp, "fp_text") if len(c) > 2 and c[1] == "reference"]
            ref = str(refnode[0][2]) if refnode else ""
            prop = [c for c in _kids(fp, "property") if len(c) > 2 and c[1] == "Reference"]
            if prop:
                ref = str(prop[0][2])
            for pad in _kids(fp, "pad"):
                kind = str(pad[2])
                shape = str(pad[3])
                pat = _kid(pad, "at")
                px, py = _f(pat[1]), _f(pat[2])
                prot = _f(pat[3]) if len(pat) > 3 else 0.0
                dx, dy = _rot(px, py, frot)
                size = _kid(pad, "size")
                w, h = _f(size[1]), _f(size[2])
                rings = None
                if shape in ("custom", "trapezoid"):
                    rings = _pad_rings(pad, fx + dx, fy + dy, prot, w, h, shape)
                if shape in ("custom", "trapezoid", "chamfered_rect"):
                    shape = "rect"   # chamfered: the full rectangle covers the chamfered copper
                lyr = _kid(pad, "layers")
                layers = _expand_layers(lyr[1:] if lyr else [], cu)
                drill = 0.0
                dr = _kid(pad, "drill")
                if dr:
                    nums = [x for x in dr[1:] if not isinstance(x, list) and re.fullmatch(r"[-\d.]+", str(x))]
                    if nums:
                        drill = _f(nums[0])
                net = self._net_of(pad)
                name = f"{ref}.{pad[1]}"
                if kind == "np_thru_hole":   # mounting hole: a hole, no copper
                    if drill > 0:
                        b.holes.append((fx + dx, fy + dy, drill))
                    continue
                # KiCad stores the pad angle including the footprint rotation
                b.pads.append(Pad(net, fx + dx, fy + dy, w, h, shape, prot, layers,
                                  drill if kind == "thru_hole" else 0.0, name, rings or None))
                # local clearance override: the pad's own, else its footprint's
                lc = _kid(pad, "clearance") or _kid(fp, "clearance")
                if lc and len(lc) > 1:
                    b.pads[-1].clearance = _f(lc[1])
        # copper drawn as graphics (gr_poly / gr_rect / gr_circle / gr_line / gr_arc on a copper
        # layer): real copper - of its net if it has one, else an obstacle for every net
        for kind in ("gr_poly", "gr_rect", "gr_circle", "gr_line", "gr_arc"):
            for g_ in _kids(self.tree, kind):
                lay = _kid(g_, "layer")
                ln = str(lay[1]) if lay else ""
                if ln in ("F.Mask", "B.Mask") and kind in ("gr_poly", "gr_rect", "gr_circle"):
                    # a drawn solder-mask opening exposes the copper under it: another net's
                    # track there would be bared next to it (KiCad: solder mask bridge)
                    ring = _mask_ring(g_, kind)
                    side = cu[0] if ln == "F.Mask" else cu[-1]
                    if len(ring) >= 3:
                        b.keepouts.append(({side}, [ring], True, True))
                    continue
                if ln not in cu:
                    continue
                net = self._net_of(g_) if _kid(g_, "net") else ""
                stroke = _kid(g_, "stroke")
                sw = _f(_kid(stroke, "width")[1]) if stroke and _kid(stroke, "width") else \
                    (_f(_kid(g_, "width")[1]) if _kid(g_, "width") else 0.0)
                fill = _kid(g_, "fill")
                filled = bool(fill) and len(fill) > 1 and str(fill[1]) in ("yes", "solid")
                ring = []
                if kind == "gr_poly":
                    pts = _kid(g_, "pts")
                    ring = _pts_xy(pts)
                elif kind == "gr_rect":
                    s_, e_ = _kid(g_, "start"), _kid(g_, "end")
                    x0, y0, x1, y1 = _f(s_[1]), _f(s_[2]), _f(e_[1]), _f(e_[2])
                    ring = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                elif kind == "gr_circle":
                    c_, e_ = _kid(g_, "center"), _kid(g_, "end")
                    cx, cy = _f(c_[1]), _f(c_[2])
                    r_ = math.hypot(_f(e_[1]) - cx, _f(e_[2]) - cy)
                    ring = [(cx + r_ * math.cos(2 * math.pi * k / 32), cy + r_ * math.sin(2 * math.pi * k / 32))
                            for k in range(32)]
                if ring and filled:
                    xs, ys = [q[0] for q in ring], [q[1] for q in ring]
                    # (grown by half the stroke, as KiCad draws it)
                    b.pads.append(Pad(net, (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                                      max(xs) - min(xs) + sw, max(ys) - min(ys) + sw, "rect", 0.0, {ln},
                                      0.0, f"copper shape ({net or 'no net'})", [ring]))
                    if sw > 0:
                        for a_, c_ in zip(ring, ring[1:] + ring[:1]):
                            b.tracks.append(Track(net, a_[0], a_[1], c_[0], c_[1], sw, ln))
                elif ring:                      # outline only: its stroke is copper
                    for a_, c_ in zip(ring, ring[1:] + ring[:1]):
                        b.tracks.append(Track(net, a_[0], a_[1], c_[0], c_[1], max(sw, 0.01), ln))
                elif kind == "gr_line":
                    s_, e_ = _kid(g_, "start"), _kid(g_, "end")
                    b.tracks.append(Track(net, _f(s_[1]), _f(s_[2]), _f(e_[1]), _f(e_[2]), max(sw, 0.01), ln))
                elif kind == "gr_arc":
                    s_, m_, e_ = _kid(g_, "start"), _kid(g_, "mid"), _kid(g_, "end")
                    if s_ and m_ and e_:
                        pts = _arc_points(_f(s_[1]), _f(s_[2]), _f(m_[1]), _f(m_[2]), _f(e_[1]), _f(e_[2]))
                        for a_, c_ in zip(pts, pts[1:]):
                            b.tracks.append(Track(net, a_[0], a_[1], c_[0], c_[1], max(sw, 0.01), ln))
        for seg in _kids(self.tree, "segment"):
            st, en = _kid(seg, "start"), _kid(seg, "end")
            b.tracks.append(Track(self._net_of(seg), _f(st[1]), _f(st[2]), _f(en[1]), _f(en[2]),
                                  _f(_kid(seg, "width")[1]), str(_kid(seg, "layer")[1])))
        for arc in _kids(self.tree, "arc"):
            st, md, en = _kid(arc, "start"), _kid(arc, "mid"), _kid(arc, "end")
            pts = _arc_points(_f(st[1]), _f(st[2]), _f(md[1]), _f(md[2]), _f(en[1]), _f(en[2]))
            for (a, c) in zip(pts, pts[1:]):
                b.tracks.append(Track(self._net_of(arc), a[0], a[1], c[0], c[1],
                                      _f(_kid(arc, "width")[1]), str(_kid(arc, "layer")[1])))
        for v in _kids(self.tree, "via"):
            at = _kid(v, "at")
            size = _kid(v, "size")
            lyr = _kid(v, "layers")
            vl = None
            if lyr and len(lyr) >= 3:
                names = [str(x) for x in lyr[1:]]
                if names[0] in cu and names[1] in cu:
                    i0, i1 = sorted((cu.index(names[0]), cu.index(names[1])))
                    vl = set(cu[i0:i1 + 1])
            b.vias.append(Via(self._net_of(v), _f(at[1]), _f(at[2]), _f(size[1]) if size else 0.6, 0.0, vl))
        fp_zones = [z for fp in _kids(self.tree, "footprint") for z in _kids(fp, "zone") if _kid(z, "keepout")]
        for z in list(_kids(self.tree, "zone")) + fp_zones:   # footprint rule areas: e.g. an antenna keep-out
            nn = _kid(z, "net_name")
            name = str(nn[1]) if nn else self._net_of(z)
            ko = _kid(z, "keepout")
            if ko:
                def banned(kind):
                    k_ = _kid(ko, kind)
                    return bool(k_) and len(k_) > 1 and str(k_[1]) == "not_allowed"
                if banned("tracks") or banned("vias"):
                    zl = _kid(z, "layers") or _kid(z, "layer")
                    klayers = _expand_layers(zl[1:] if zl else [], cu)
                    rings = []
                    for poly in _kids(z, "polygon"):
                        pts = _kid(poly, "pts")
                        ring = _pts_xy(pts)
                        if len(ring) >= 3:
                            rings.append(ring)
                    if rings and klayers:
                        b.keepouts.append((klayers, rings, banned("tracks"), banned("vias")))
                continue
            if name:
                b.zone_nets.add(name)
                zl = _kid(z, "layers") or _kid(z, "layer")
                zlayers = _expand_layers(zl[1:] if zl else [], cu)
                b.zone_layers.setdefault(name, set()).update(zlayers)
                rings = []
                for poly in _kids(z, "polygon"):
                    pts = _kid(poly, "pts")
                    ring = _pts_xy(pts)
                    if len(ring) >= 3:
                        rings.append(ring)
                if rings:
                    b.zone_outlines.setdefault(name, []).append((zlayers, rings))
                    pr = _kid(z, "priority")
                    b.zone_priority.setdefault(name, []).append(int(_f(pr[1])) if pr else 0)
                    cp = _kid(z, "connect_pads")
                    zc = _kid(cp, "clearance") if cp else None
                    if zc:
                        b.zone_clearance[name] = max(b.zone_clearance.get(name, 0.0), _f(zc[1]))
        return b

    # -------------------------------------------------------------- settings
    def settings_from_project(self, base: Optional[RouteSettings] = None) -> RouteSettings:
        """Default net-class sizes from the .kicad_pro next to the board, if any."""
        s = base or RouteSettings()
        pro = os.path.splitext(self.path)[0] + ".kicad_pro"
        if os.path.exists(pro):
            try:
                with open(pro, encoding="utf-8") as fh:
                    data = json.load(fh)
                ns = data.get("net_settings", {}) or {}
                for cls in ns.get("classes", []):
                    if cls.get("name") == "Default":
                        s.track_width = float(cls.get("track_width", s.track_width))
                        s.clearance = float(cls.get("clearance", s.clearance))
                        s.via_diameter = float(cls.get("via_diameter", s.via_diameter))
                        s.via_drill = float(cls.get("via_drill", s.via_drill))
                s.net_rules = self._net_rules(ns, s)
                rules = data.get("board", {}).get("design_settings", {}).get("rules", {})
                if "min_track_width" in rules:
                    s.min_track_width = float(rules["min_track_width"])
                if "min_copper_edge_clearance" in rules:
                    s.edge_clearance = max(s.edge_clearance, float(rules["min_copper_edge_clearance"]))
                if "min_hole_clearance" in rules:
                    s.hole_clearance = max(s.hole_clearance, float(rules["min_hole_clearance"]))
            except (OSError, ValueError, TypeError):
                pass
        return s

    def _net_rules(self, ns: dict, s: RouteSettings) -> Dict[str, Tuple[float, float]]:
        """net -> (track width, clearance) from the project's net classes.

        KiCad 7+: ``netclass_assignments`` {net: class} and ``netclass_patterns``
        [{netclass, pattern}] (wildcards); KiCad 6: a ``nets`` list in each class."""
        classes = {}
        for cls in ns.get("classes", []):
            name = cls.get("name")
            if not name or name == "Default":
                continue
            try:
                w = float(cls.get("track_width") or s.track_width)
                c = float(cls.get("clearance") or s.clearance)
            except (TypeError, ValueError):
                continue
            classes[name] = (w, c)
        if not classes:
            return {}
        which: Dict[str, str] = {}
        for cls in ns.get("classes", []):
            for n in cls.get("nets", None) or []:
                which[n] = cls.get("name")
        pats = ns.get("netclass_patterns") or []
        if pats:
            try:
                names = self.board().nets()
            except Exception:
                names = []
            for name in names:
                for pt in pats:
                    pat = pt.get("pattern", "")
                    if pat and (fnmatch.fnmatchcase(name, pat) or _regex_match(pat, name)):
                        which.setdefault(name, pt.get("netclass"))
                        break
        assign = ns.get("netclass_assignments") or {}
        for n, cname in assign.items():
            if isinstance(cname, list):          # KiCad 9: a net may carry several classes
                cname = next((c for c in cname if c in classes), cname[0] if cname else None)
            which[n] = cname
        return {n: classes[c] for n, c in which.items() if c in classes}

    # -------------------------------------------------------------- writing
    def _net_token(self, net: str) -> str:
        if self.names_only or not self.net_codes:
            return f'(net "{net}")'
        return f"(net {self.net_codes.get(net, 0)})"

    def add_routes(self, result, settings: RouteSettings, layer_names=None) -> int:
        """Append the routed tracks/vias to the file text. Returns items added."""
        w = settings.track_width
        out = []
        # KiCad 7 and older call the id "tstamp"; KiCad 8 (file version 2024xxxx) "uuid"
        idk = "uuid" if self.version >= 20231231 else "tstamp"
        fmt_id = (lambda: f'({idk} "{uuid.uuid4()}")') if idk == "uuid" else (lambda: f"({idk} {uuid.uuid4()})")
        for name, nr in result.nets.items():
            nt = self._net_token(name)
            for k, (x1, y1, x2, y2, layer) in enumerate(nr.segments):
                if math.hypot(x2 - x1, y2 - y1) < 1e-6:
                    continue
                out.append(f'  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) '
                           f'(width {nr.width(k, w):.4f}) (layer "{layer}") {nt} {fmt_id()})')
            for (x, y) in nr.vias:
                out.append(f'  (via (at {x:.4f} {y:.4f}) (size {settings.via_diameter:.4f}) '
                           f'(drill {settings.via_drill:.4f}) (layers "{self.copper[0]}" "{self.copper[-1]}") '
                           f'{nt} {fmt_id()})')
        if not out:
            return 0
        end = self.text.rstrip().rfind(")")
        self.text = self.text[:end].rstrip() + "\n" + "\n".join(out) + "\n)\n"
        return len(out)

    def save(self, path: Optional[str] = None) -> None:
        with open(path or self.path, "w", encoding="utf-8") as fh:
            fh.write(self.text)


def _regex_match(pat: str, name: str) -> bool:
    """KiCad also accepts regular expressions in net-class patterns."""
    if not any(ch in pat for ch in "^$()[]|\\+{"):
        return False
    try:
        return re.fullmatch(pat, name) is not None
    except re.error:
        return False


def _edge_segments(node) -> List[Tuple[float, float, float, float]]:
    kind = node[0]
    segs = []
    if kind == "gr_line":
        s, e = _kid(node, "start"), _kid(node, "end")
        segs.append((_f(s[1]), _f(s[2]), _f(e[1]), _f(e[2])))
    elif kind == "gr_rect":
        s, e = _kid(node, "start"), _kid(node, "end")
        x0, y0, x1, y1 = _f(s[1]), _f(s[2]), _f(e[1]), _f(e[2])
        segs += [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
    elif kind == "gr_circle":
        c, e = _kid(node, "center"), _kid(node, "end")
        cx, cy = _f(c[1]), _f(c[2])
        r = math.hypot(_f(e[1]) - cx, _f(e[2]) - cy)
        n = 48
        pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]
        segs += [(a[0], a[1], b[0], b[1]) for a, b in zip(pts, pts[1:])]
    elif kind == "gr_arc":
        s, m, e = _kid(node, "start"), _kid(node, "mid"), _kid(node, "end")
        if m is not None:
            pts = _arc_points(_f(s[1]), _f(s[2]), _f(m[1]), _f(m[2]), _f(e[1]), _f(e[2]))
            segs += [(a[0], a[1], b[0], b[1]) for a, b in zip(pts, pts[1:])]
    elif kind == "gr_poly":
        pts = _kid(node, "pts")
        xy = _pts_xy(pts)
        if len(xy) > 1:
            xy.append(xy[0])
            segs += [(a[0], a[1], b[0], b[1]) for a, b in zip(xy, xy[1:])]
    return segs
