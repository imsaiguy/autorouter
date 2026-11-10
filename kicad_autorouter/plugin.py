"""KiCad action plugin: route all nets or only the selection, and watch it think.

Install: copy (or symlink) the ``kicad_autorouter`` folder into KiCad's plugin
directory (Tools > External Plugins > Open Plugin Directory), then
Tools > External Plugins > Refresh Plugins.  A toolbar button appears.

Uses the classic SWIG ``pcbnew`` Python API (KiCad 7, 8 and 9).  The router
itself is API-independent, so an IPC-API front end can reuse everything.
"""
from __future__ import annotations

import math
import os
import time
from typing import Dict, List, Optional, Set, Tuple

import pcbnew
import wx

try:
    from wx.lib.buttons import GenButton as _BigButtonBase   # honours colours on Windows too
except Exception:   # pragma: no cover
    _BigButtonBase = wx.Button

from .model import Board, Pad, RouteSettings, Track, Via
from .problem import BLOCKED, FREE, build_problem, loose_copper, net_status, nets_to_finish
from .router import Router
from .progress import ProgressEstimator, fmt

NM = 1e6

# Items removed from the board are kept referenced here.  KiCad's action-plugin
# undo keeps pointers to removed items (so they must not be deleted), and in
# KiCad 7 letting the Python wrapper of a removed item be garbage-collected
# corrupts SWIG's type table ("'SwigPyObject' object is not iterable").
_GRAVEYARD: list = []


# ======================================================================
# pcbnew compatibility helpers (KiCad 7 / 8 / 9)
# ======================================================================
def _mm(v) -> float:
    return v / NM


def _pt(x_mm: float, y_mm: float):
    x, y = int(round(x_mm * NM)), int(round(y_mm * NM))
    if hasattr(pcbnew, "VECTOR2I"):
        return pcbnew.VECTOR2I(x, y)
    return pcbnew.wxPoint(x, y)


def _try(obj, name, *alts):
    """Call obj.name(), falling back to obj.name(*alt) for APIs that grew a layer argument."""
    fn = getattr(obj, name)
    try:
        return fn()
    except TypeError:
        for a in alts:
            try:
                return fn(*a)
            except TypeError:
                continue
        raise


def _deg(angle) -> float:
    if hasattr(angle, "AsDegrees"):
        return angle.AsDegrees()
    return float(angle) / 10.0   # KiCad 6: tenths of a degree


_SHAPES = {}
for _n, _s in (("PAD_SHAPE_CIRCLE", "circle"), ("PAD_SHAPE_OVAL", "oval"),
               ("PAD_SHAPE_RECT", "rect"), ("PAD_SHAPE_ROUNDRECT", "roundrect")):
    if hasattr(pcbnew, _n):
        _SHAPES[getattr(pcbnew, _n)] = _s


def _pad_outline(pad):
    """Exact copper outline(s) of a custom / trapezoid / chamfered pad, in mm.

    Falls back to the pad's bounding box - always a superset of its copper - if
    KiCad's polygon call is unavailable, so clearance is never under-estimated."""
    poly = None
    for args in ((), (pcbnew.F_Cu,)):
        try:
            poly = pad.GetEffectivePolygon(*args)
            break
        except Exception:
            continue
    rings = []
    try:
        if poly is not None:
            for o in range(poly.OutlineCount()):
                ch = poly.Outline(o)
                rings.append([(_mm(ch.CPoint(i).x), _mm(ch.CPoint(i).y)) for i in range(ch.PointCount())])
    except Exception:
        rings = []
    rings = [r for r in rings if len(r) >= 3]
    if rings:
        return rings
    bb = pad.GetBoundingBox()
    x0, y0 = _mm(bb.GetX()), _mm(bb.GetY())
    x1, y1 = x0 + _mm(bb.GetWidth()), y0 + _mm(bb.GetHeight())
    return [[(x0, y0), (x1, y0), (x1, y1), (x0, y1)]]


def _layer_names(board, lset, allowed: Set[str]) -> Set[str]:
    return {board.GetLayerName(l) for l in lset.CuStack()} & allowed


# ======================================================================
# pcbnew BOARD  ->  model.Board
# ======================================================================
def board_from_pcbnew(kb) -> Tuple[Board, Set[str]]:
    """Returns the model board and the set of nets touched by the selection."""
    copper = [kb.GetLayerName(l) for l in kb.GetEnabledLayers().CuStack()]
    cu = set(copper)
    b = Board(copper, _outline(kb))
    b.outline_warning = OUTLINE_WARNING
    b.refs = {}          # id(model track/via) -> the pcbnew item it came from
    selected: Set[str] = set()
    for fp in kb.GetFootprints():
        fp_sel = fp.IsSelected()
        for pad in fp.Pads():
            net = pad.GetNetname()
            attr = pad.GetAttribute()
            pos = pad.GetPosition()
            size = _try(pad, "GetSize", (pcbnew.F_Cu,))
            if attr == pcbnew.PAD_ATTRIB_NPTH:
                d = pad.GetDrillSize()
                b.holes.append((_mm(pos.x), _mm(pos.y), _mm(max(d.x, d.y))))
                continue
            raw_shape = _try(pad, "GetShape", (pcbnew.F_Cu,))
            shape = _SHAPES.get(raw_shape, "rect")
            poly = None if raw_shape in _SHAPES else _pad_outline(pad)
            drill = _mm(pad.GetDrillSize().x) if attr == pcbnew.PAD_ATTRIB_PTH else 0.0
            b.pads.append(Pad(net, _mm(pos.x), _mm(pos.y), _mm(size.x), _mm(size.y), shape,
                              _deg(pad.GetOrientation()), _layer_names(kb, pad.GetLayerSet(), cu),
                              drill, f"{fp.GetReference()}.{pad.GetNumber()}", poly))
            for getter in (lambda: pad.GetLocalClearance(), lambda: fp.GetLocalClearance()):
                try:
                    lc = getter()          # KiCad 7: int (0 = none); 8+: optional int
                    if lc:
                        b.pads[-1].clearance = _mm(lc)
                        break
                except Exception:
                    continue
            if net and (fp_sel or pad.IsSelected()):
                selected.add(net)
    for t in kb.GetTracks():
        cls = t.GetClass()
        net = t.GetNetname()
        if t.IsSelected() and net:
            selected.add(net)
        if cls in ("PCB_VIA", "VIA"):
            p = t.GetPosition()
            w = _try(t, "GetWidth", (pcbnew.F_Cu,))
            top, bot = t.TopLayer(), t.BottomLayer()
            names = [kb.GetLayerName(top), kb.GetLayerName(bot)]
            vl = None
            if all(n in copper for n in names):
                i0, i1 = sorted(copper.index(n) for n in names)
                vl = set(copper[i0:i1 + 1])
            mv = Via(net, _mm(p.x), _mm(p.y), _mm(w), 0.0, vl)
            b.vias.append(mv)
            b.refs[id(mv)] = t
        else:
            s, e = t.GetStart(), t.GetEnd()
            ln = kb.GetLayerName(t.GetLayer())
            pts = [(_mm(s.x), _mm(s.y)), (_mm(e.x), _mm(e.y))]
            if cls == "PCB_ARC" and hasattr(t, "GetMid"):
                from .kicad_pcb import _arc_points
                m = t.GetMid()
                pts = _arc_points(pts[0][0], pts[0][1], _mm(m.x), _mm(m.y), pts[1][0], pts[1][1])
            for a, c in zip(pts, pts[1:]):
                mt = Track(net, a[0], a[1], c[0], c[1], _mm(t.GetWidth()), ln)
                b.tracks.append(mt)
                b.refs[id(mt)] = t
    fp_zones = []
    for fp in kb.GetFootprints():          # footprint rule areas, e.g. an antenna keep-out
        try:
            fp_zones += [z for z in fp.Zones()
                         if (z.GetIsRuleArea() if hasattr(z, "GetIsRuleArea") else z.GetIsKeepout())]
        except Exception:
            pass
    # copper drawn as graphics on a copper layer: real copper (of its net, else no net)
    for d in kb.GetDrawings():
        try:
            if not hasattr(d, "GetShape") or not d.IsOnCopperLayer():
                continue
            ln = kb.GetLayerName(d.GetLayer())
            if ln not in cu:
                continue
            net = d.GetNetname() if hasattr(d, "GetNetname") else ""
            sw = _mm(d.GetWidth())
            sh = d.GetShape()
            ring = []
            if sh == pcbnew.SHAPE_T_POLY:
                ps = d.GetPolyShape()
                if ps.OutlineCount():
                    ch = ps.Outline(0)
                    ring = [(_mm(ch.CPoint(i).x), _mm(ch.CPoint(i).y)) for i in range(ch.PointCount())]
            elif sh == pcbnew.SHAPE_T_RECT:
                try:
                    ring = [(_mm(c.x), _mm(c.y)) for c in d.GetRectCorners()]
                except Exception:
                    s_, e_ = d.GetStart(), d.GetEnd()
                    ring = [(_mm(s_.x), _mm(s_.y)), (_mm(e_.x), _mm(s_.y)), (_mm(e_.x), _mm(e_.y)), (_mm(s_.x), _mm(e_.y))]
            elif sh == pcbnew.SHAPE_T_CIRCLE:
                c_, r_ = d.GetCenter(), _mm(d.GetRadius())
                ring = [(_mm(c_.x) + r_ * math.cos(2 * math.pi * k / 32), _mm(c_.y) + r_ * math.sin(2 * math.pi * k / 32))
                        for k in range(32)]
            if ring and d.IsFilled():
                xs, ys = [q[0] for q in ring], [q[1] for q in ring]
                b.pads.append(Pad(net, (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(xs) - min(xs) + sw,
                                  max(ys) - min(ys) + sw, "rect", 0.0, {ln}, 0.0, f"copper shape ({net or 'no net'})",
                                  [ring]))
            pts = ring + ring[:1] if ring else [(_mm(d.GetStart().x), _mm(d.GetStart().y)),
                                                 (_mm(d.GetEnd().x), _mm(d.GetEnd().y))]
            if sw > 0 or not (ring and d.IsFilled()):
                for a_, c_ in zip(pts, pts[1:]):
                    b.tracks.append(Track(net, a_[0], a_[1], c_[0], c_[1], max(sw, 0.01), ln))
        except Exception:
            continue
    for z in list(kb.Zones()) + fp_zones:
        rule = z.GetIsRuleArea() if hasattr(z, "GetIsRuleArea") else z.GetIsKeepout()
        if rule:
            try:
                no_t, no_v = bool(z.GetDoNotAllowTracks()), bool(z.GetDoNotAllowVias())
                if no_t or no_v:
                    ol = z.Outline()
                    rings = []
                    for o in range(ol.OutlineCount()):
                        ch = ol.Outline(o)
                        rings.append([(_mm(ch.CPoint(i).x), _mm(ch.CPoint(i).y)) for i in range(ch.PointCount())])
                    rings = [r for r in rings if len(r) >= 3]
                    klayers = _layer_names(kb, z.GetLayerSet(), cu)
                    if rings and klayers:
                        b.keepouts.append((klayers, rings, no_t, no_v))
            except Exception:
                pass
            continue
        net = z.GetNetname()
        if net:
            b.zone_nets.add(net)
            zlayers = _layer_names(kb, z.GetLayerSet(), cu)
            b.zone_layers.setdefault(net, set()).update(zlayers)
            try:
                ol = z.Outline()
                rings = []
                for o in range(ol.OutlineCount()):
                    ch = ol.Outline(o)
                    rings.append([(_mm(ch.CPoint(i).x), _mm(ch.CPoint(i).y)) for i in range(ch.PointCount())])
                rings = [r for r in rings if len(r) >= 3]
                if rings:
                    b.zone_outlines.setdefault(net, []).append((zlayers, rings))
            except Exception:
                pass
        if z.IsSelected() and net:
            selected.add(net)
    return b, selected


def _outline(kb) -> List[Tuple[float, float, float, float]]:
    """Board outline as line segments.  KiCad's own outline when it can build one;
    otherwise (overlapping/touching outlines, a gap, an Edge.Cuts shape inside a
    footprint ...) the Edge.Cuts lines as drawn - never just their bounding box, which
    let the router run tracks outside the board.  Sets OUTLINE_WARNING in that case."""
    global OUTLINE_WARNING
    OUTLINE_WARNING = ""
    segs = []
    ok = False
    refused = False                  # KiCad answered "cannot build it" (not an API mismatch)
    try:
        ps = pcbnew.SHAPE_POLY_SET()
        res = None
        for args in ((ps,), (ps, True), (ps, None, True)):   # signatures differ between versions
            try:
                res = kb.GetBoardPolygonOutlines(*args)
                break
            except TypeError:
                continue
        ok = bool(res)
        refused = res is False
        if ok:
            for o in range(ps.OutlineCount()):
                chains = [ps.Outline(o)] + [ps.Hole(o, h) for h in range(ps.HoleCount(o))]
                for ch in chains:
                    n = ch.PointCount()
                    pts = [ch.CPoint(i) for i in range(n)]
                    for i in range(n):
                        a, c = pts[i], pts[(i + 1) % n]
                        segs.append((_mm(a.x), _mm(a.y), _mm(c.x), _mm(c.y)))
    except Exception:
        segs = []
    if not segs:
        segs = _edge_cuts_as_drawn(kb)
        if segs and refused:
            OUTLINE_WARNING = ("KiCad cannot build a clean board outline from Edge.Cuts (outlines that "
                               "touch or overlap, a gap, or an outline inside a footprint). The router uses "
                               "the Edge.Cuts lines as drawn; outlines that touch count as one board. "
                               "Check Inspect > Design Rules Checker.")
    if not segs:
        bb = kb.GetBoardEdgesBoundingBox()
        x0, y0 = _mm(bb.GetX()), _mm(bb.GetY())
        x1, y1 = x0 + _mm(bb.GetWidth()), y0 + _mm(bb.GetHeight())
        if x1 > x0 and y1 > y0:
            segs = [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
    return segs


OUTLINE_WARNING = ""


def _edge_cuts_as_drawn(kb) -> List[Tuple[float, float, float, float]]:
    """Every Edge.Cuts shape (board drawings and footprint graphics) as line segments."""
    from .kicad_pcb import _arc_points
    items = [d for d in kb.GetDrawings()]
    for fp in kb.GetFootprints():
        try:
            items += list(fp.GraphicalItems())
        except Exception:
            pass
    segs = []

    def P(v):
        return _mm(v.x), _mm(v.y)
    for d in items:
        try:
            if d.GetLayer() != pcbnew.Edge_Cuts or not hasattr(d, "GetShape"):
                continue
            sh = d.GetShape()
            if sh == pcbnew.SHAPE_T_SEGMENT:
                segs.append((*P(d.GetStart()), *P(d.GetEnd())))
            elif sh == pcbnew.SHAPE_T_RECT:
                try:
                    pts = [P(c) for c in d.GetRectCorners()]
                except Exception:
                    (x0, y0), (x1, y1) = P(d.GetStart()), P(d.GetEnd())
                    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                segs += [(*pts[i], *pts[(i + 1) % len(pts)]) for i in range(len(pts))]
            elif sh == pcbnew.SHAPE_T_CIRCLE:
                (cx, cy), r = P(d.GetCenter()), _mm(d.GetRadius())
                n = 48
                pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
                       for k in range(n)]
                segs += [(*pts[i], *pts[(i + 1) % n]) for i in range(n)]
            elif sh == pcbnew.SHAPE_T_ARC:
                pts = _arc_points(*P(d.GetStart()), *P(d.GetArcMid()), *P(d.GetEnd()))
                segs += [(*a, *c) for a, c in zip(pts, pts[1:])]
            elif sh == pcbnew.SHAPE_T_POLY:
                ps = d.GetPolyShape()
                for o in range(ps.OutlineCount()):
                    ch = ps.Outline(o)
                    pts = [P(ch.CPoint(i)) for i in range(ch.PointCount())]
                    segs += [(*pts[i], *pts[(i + 1) % len(pts)]) for i in range(len(pts))]
            else:                                   # bezier etc.: its end points
                segs.append((*P(d.GetStart()), *P(d.GetEnd())))
        except Exception:
            continue
    return segs


def settings_from_pcbnew(kb) -> RouteSettings:
    s = RouteSettings()
    ds = kb.GetDesignSettings()
    nc = None
    for getter in (lambda: ds.m_NetSettings.m_DefaultNetClass,
                   lambda: ds.m_NetSettings.GetDefaultNetclass(),
                   lambda: ds.GetDefault()):
        try:
            nc = getter()
            if nc is not None:
                break
        except Exception:
            continue
    if nc is not None:
        for attr, getter in (("track_width", "GetTrackWidth"), ("clearance", "GetClearance"),
                             ("via_diameter", "GetViaDiameter"), ("via_drill", "GetViaDrill")):
            try:
                v = _mm(getattr(nc, getter)())
                if v > 0:
                    setattr(s, attr, v)
            except Exception:
                pass
    for attr, field in (("edge_clearance", "m_CopperEdgeClearance"), ("hole_clearance", "m_HoleClearance"),
                        ("min_track_width", "m_TrackMinWidth")):
        try:
            v = _mm(getattr(ds, field))
            if v > 0:
                setattr(s, attr, v)
        except Exception:
            pass
    s.net_rules = net_rules_from_pcbnew(kb, s)
    return s


def net_rules_from_pcbnew(kb, s: RouteSettings) -> dict:
    """net -> (track width, clearance) for nets whose net class differs from Default."""
    names = set()
    try:
        for fp in kb.GetFootprints():
            for pad in fp.Pads():
                if pad.GetNetname():
                    names.add(pad.GetNetname())
    except Exception:
        return {}
    ds = kb.GetDesignSettings()
    rules = {}
    for name in names:
        nc = None
        for getter in (lambda: ds.m_NetSettings.GetEffectiveNetClass(name),
                       lambda: kb.FindNet(name).GetNetClass()):
            try:
                nc = getter()
                if nc is not None and hasattr(nc, "GetTrackWidth"):
                    break
                nc = None
            except Exception:
                continue
        if nc is None:
            continue
        try:
            w, c = _mm(nc.GetTrackWidth()), _mm(nc.GetClearance())
        except Exception:
            continue
        if w <= 0:
            w = s.track_width
        if c <= 0:
            c = s.clearance
        if abs(w - s.track_width) > 1e-6 or abs(c - s.clearance) > 1e-6:
            rules[name] = (w, c)
    return rules


# ======================================================================
# writing tracks back
# ======================================================================
class BoardWriter:
    def __init__(self, kb, settings: RouteSettings):
        self.kb = kb
        self.s = settings
        self.items = []
        cu = list(kb.GetEnabledLayers().CuStack())
        self.top, self.bot = cu[0], cu[-1]
        self.TrackCls = getattr(pcbnew, "PCB_TRACK", None) or pcbnew.TRACK
        self.ViaCls = getattr(pcbnew, "PCB_VIA", None) or pcbnew.VIA

    def add_net(self, name, segments, vias, widths=()):
        kb, s = self.kb, self.s
        net = kb.FindNet(name)
        for k, (x1, y1, x2, y2, layer) in enumerate(segments):
            if math.hypot(x2 - x1, y2 - y1) < 1e-6:
                continue
            t = self.TrackCls(kb)
            t.SetStart(_pt(x1, y1))
            t.SetEnd(_pt(x2, y2))
            t.SetWidth(int(round((widths[k] if k < len(widths) else s.track_width) * NM)))
            t.SetLayer(kb.GetLayerID(layer))
            if net is not None:
                t.SetNet(net)
            kb.Add(t)
            self.items.append(t)
        for (x, y) in vias:
            v = self.ViaCls(kb)
            v.SetPosition(_pt(x, y))
            w = int(round(s.via_diameter * NM))
            for args in ((w,), (w, pcbnew.F_Cu), (pcbnew.F_Cu, w)):   # KiCad 7 / 9+ signatures
                try:
                    v.SetWidth(*args)
                    break
                except TypeError:
                    continue
            v.SetDrill(int(round(s.via_drill * NM)))
            v.SetLayerPair(self.top, self.bot)
            if hasattr(pcbnew, "VIATYPE_THROUGH"):
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
            if net is not None:
                v.SetNet(net)
            kb.Add(v)
            self.items.append(v)

    def clear(self):
        for it in self.items:
            self.kb.Remove(it)
        _GRAVEYARD.extend(self.items)
        self.items = []


def remove_existing_tracks(kb, nets: Set[str]) -> int:
    doomed = [t for t in kb.GetTracks() if t.GetNetname() in nets and not t.IsLocked()]
    for t in doomed:
        kb.Remove(t)
    _GRAVEYARD.extend(doomed)
    return len(doomed)


# ======================================================================
# the "thinking" panel
# ======================================================================
# fixed colour per layer NAME: front always red, back always blue, inner layers in turn
INNER_RGB = [((63, 174, 90), "green"), ((160, 90, 209), "purple"), ((43, 181, 176), "teal"),
             ((214, 112, 170), "pink"), ((150, 150, 70), "olive"), ((120, 140, 230), "lavender")]


def layer_colour(name: str):
    """(rgb, colour name) for a copper layer, the same on every board."""
    if name == "F.Cu":
        return (232, 67, 58), "red"
    if name == "B.Cu":
        return (59, 125, 221), "blue"
    try:
        k = int(name[2:name.index(".")]) - 1          # In1.Cu -> 0
    except ValueError:
        k = 0
    return INNER_RGB[k % len(INNER_RGB)]


def layer_legend(layers) -> str:
    return " · ".join(f"{layer_colour(n)[1]} = {n}" for n in layers)


def board_summary(board, cnt) -> str:
    per = {}
    for tr in board.tracks:
        per[tr.layer] = per.get(tr.layer, 0) + 1
    by_layer = ", ".join(f"{n} {per[n]}" for n in board.copper_layers if per.get(n))
    return (f"Board: {cnt['routed']} routed, {cnt['partial']} partly routed, {cnt['unrouted']} unrouted\n"
            f"{len(board.tracks)} existing track segments" + (f" ({by_layer})" if by_layer else "")
            + f", {len(board.vias)} vias")


class ThinkingPanel(wx.Panel):
    """Draws the board grid, learned congestion history, overused cells,
    the last A* wavefront and the current tentative routes."""

    def __init__(self, parent):
        super().__init__(parent, size=(560, 360))
        self.SetBackgroundStyle(wx.BG_STYLE_PAINT)
        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SIZE, lambda e: (self.Refresh(), e.Skip()))
        self.router = None
        self.board = None
        self.explored: List[int] = []
        self.highlight = None
        self.base_img = None
        self._base = None
        self.show_existing = True

    def attach(self, router, board):
        self.router, self.board = router, board
        self.explored = []
        self._base = None
        self.Refresh()

    def rebuild_base(self):
        """Heat (learned congestion) + conflict picture.  It only changes between
        passes, so it is built once per pass, not on every redraw (scanning every
        grid cell in Python on each redraw made routing 2.5x slower)."""
        r = self.router
        g = r.pb.grid
        plane, nl = g.plane, g.nl
        buf = bytearray(bytes((13, 40, 24)) * plane)
        hist, occ = r.hist, r.occ
        for l in range(nl):
            base = l * plane
            for xy in range(plane):
                h = hist[base + xy]
                o = occ[base + xy]
                if h <= 0 and o <= 1:
                    continue
                k3 = xy * 3
                if o > 1:
                    buf[k3], buf[k3 + 1], buf[k3 + 2] = 255, 225, 77
                elif buf[k3] != 255:
                    k = min(1.0, math.log1p(h) / 2.5)
                    R, G, B = int(13 + 217 * k), int(40 + 50 * k), int(24 - 4 * k)
                    if R > buf[k3]:
                        buf[k3], buf[k3 + 1], buf[k3 + 2] = R, G, B
        self._base = buf

    def _image(self):
        r = self.router
        g = r.pb.grid
        plane = g.plane
        if self._base is None:
            self.rebuild_base()
        buf = bytearray(self._base)          # copy of the per-pass picture...
        for c in self.explored:              # ...plus the current wavefront only
            k3 = (c % plane) * 3
            if buf[k3] != 255:
                buf[k3], buf[k3 + 1], buf[k3 + 2] = (buf[k3] + 40) // 2, (buf[k3 + 1] + 200) // 2, (buf[k3 + 2] + 220) // 2
        return wx.Image(g.nx, g.ny, bytes(buf))

    def set_board(self, board):
        self.board = board
        self.Refresh()

    def on_paint(self, _evt):
        dc = wx.AutoBufferedPaintDC(self)
        dc.SetBackground(wx.Brush(wx.Colour(20, 22, 26)))
        dc.Clear()
        b = self.board
        if b is None:
            return
        # board millimetres -> panel pixels
        x0, y0, x1, y1 = b.bbox()
        W, H = self.GetClientSize()
        m = 1.0
        sc = min(W / (x1 - x0 + 2 * m), H / (y1 - y0 + 2 * m))
        ox = (W - (x1 - x0) * sc) / 2 - x0 * sc
        oy = (H - (y1 - y0) * sc) / 2 - y0 * sc

        def M(x, y):
            return int(ox + x * sc), int(oy + y * sc)

        dc.SetPen(wx.TRANSPARENT_PEN)
        dc.SetBrush(wx.Brush(wx.Colour(13, 40, 24)))
        dc.DrawRectangle(*M(x0, y0), int((x1 - x0) * sc), int((y1 - y0) * sc))
        r = self.router
        if r is not None:   # thinking overlay (heat, wavefront, conflicts) placed in mm
            g = r.pb.grid
            px = max(1, int(round(g.pitch * sc)))
            img = self._image().Scale(g.nx * px, g.ny * px, wx.IMAGE_QUALITY_NORMAL)
            img.SetMaskColour(13, 40, 24)
            gx, gy = M(g.x0 - g.pitch / 2, g.y0 - g.pitch / 2)
            dc.DrawBitmap(wx.Bitmap(img), gx, gy, True)
        dc.SetPen(wx.Pen(wx.Colour(229, 211, 138), 1))
        for (a, c, d, e) in b.outline:
            dc.DrawLine(*M(a, c), *M(d, e))
        dc.SetPen(wx.TRANSPARENT_PEN)
        for p in b.pads:
            dc.SetBrush(wx.Brush(wx.Colour(170, 176, 182) if p.net else wx.Colour(100, 104, 108)))
            if p.poly:
                for ring in p.rings():
                    dc.DrawPolygon([wx.Point(*M(x, y)) for x, y in ring])
            elif p.shape == "circle":
                dc.DrawCircle(*M(p.x, p.y), max(1, int(p.w / 2 * sc)))
            else:
                a = math.radians(p.angle)
                ca, sa = math.cos(a), math.sin(a)
                pts = [(p.x + lx * ca + ly * sa, p.y - lx * sa + ly * ca)
                       for (lx, ly) in ((-p.w / 2, -p.h / 2), (p.w / 2, -p.h / 2), (p.w / 2, p.h / 2), (-p.w / 2, p.h / 2))]
                dc.DrawPolygon([wx.Point(*M(x, y)) for x, y in pts])
        # existing copper, dimmed (back layers first)
        if self.show_existing:
            lidx = {n: i for i, n in enumerate(b.copper_layers)}
            for t in sorted(b.tracks, key=lambda t: -lidx.get(t.layer, 0)):
                rr, gg, bb = layer_colour(t.layer)[0]
                col = wx.Colour(int(rr * .62 + 13 * .38), int(gg * .62 + 40 * .38), int(bb * .62 + 24 * .38))
                dc.SetPen(wx.Pen(col, max(2, int(t.width * sc))))
                dc.DrawLine(*M(t.x1, t.y1), *M(t.x2, t.y2))
            dc.SetPen(wx.TRANSPARENT_PEN)
            dc.SetBrush(wx.Brush(wx.Colour(140, 140, 140)))
            for v in b.vias:
                dc.DrawCircle(*M(v.x, v.y), max(2, int(v.diameter / 2 * sc)))
        if r is None:
            return
        g = r.pb.grid

        def P(x, y):
            return M(g.x0 + x * g.pitch, g.y0 + y * g.pitch)
        pens = {}
        vias = []
        for j in r.pb.nets:
            hl = j.name == self.highlight
            for path in j.paths:
                # one polyline per same-layer run (drawing cell by cell was very slow)
                run, layer = [], None
                for c in path:
                    x, y, l = g.xyl(c)
                    if layer is not None and l != layer:
                        if len(run) > 1:
                            key = (layer, hl)
                            if key not in pens:
                                pens[key] = wx.Pen(wx.Colour(*layer_colour(g.layers[layer])[0]), 3 if hl else 2)
                            dc.SetPen(pens[key])
                            dc.DrawLines(run)
                        vias.append(P(x, y))
                        run = []
                    run.append(wx.Point(*P(x, y)))
                    layer = l
                if len(run) > 1:
                    key = (layer, hl)
                    if key not in pens:
                        pens[key] = wx.Pen(wx.Colour(*layer_colour(g.layers[layer])[0]), 3 if hl else 2)
                    dc.SetPen(pens[key])
                    dc.DrawLines(run)
        dc.SetPen(wx.Pen(wx.Colour(220, 220, 220), 1))
        dc.SetBrush(wx.Brush(wx.Colour(200, 200, 200)))
        for (vx, vy) in vias:
            dc.DrawCircle(vx, vy, max(2, int(sc * 0.3)))

    def update(self, explored=None, highlight=None):
        if explored is not None:
            self.explored = explored
        self.highlight = highlight
        self.Refresh()
        self.Update()


# ======================================================================
# dialog
# ======================================================================
class AutorouterDialog(wx.Dialog):
    def __init__(self, parent, kb):
        super().__init__(parent, title="Visible Autorouter", size=(980, 640),
                         style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER | wx.MAXIMIZE_BOX | wx.MINIMIZE_BOX)
        self.kb = kb
        self.board, self.selected = board_from_pcbnew(kb)
        self.s = settings_from_pcbnew(kb)
        self.running = False
        self.stop = False
        self.preview: Optional[BoardWriter] = None
        self._last_draw = 0.0
        self._build()

    # -------------------------------------------------------------- layout
    def _build(self):
        main = wx.BoxSizer(wx.HORIZONTAL)
        left = wx.BoxSizer(wx.VERTICAL)

        n_sel = len(self.selected)
        status = net_status(self.board)
        cnt = {k: sum(1 for v in status.values() if v == k) for k in ("routed", "partial", "unrouted")}
        todo = cnt["partial"] + cnt["unrouted"]
        self.summary = wx.StaticText(self, label=board_summary(self.board, cnt))
        left.Add(self.summary, 0, wx.ALL, 6)
        self.scope = wx.RadioBox(self, label="What to route", choices=[
            f"Unrouted only ({todo} net{'s' if todo != 1 else ''} to finish - existing tracks kept)",
            f"Selected ({n_sel} net{'s' if n_sel != 1 else ''})", "All nets"],
            majorDimension=1, style=wx.RA_SPECIFY_COLS)
        self.scope.SetSelection(1 if n_sel else 0)
        if not n_sel:
            self.scope.EnableItem(1, False)
        left.Add(self.scope, 0, wx.EXPAND | wx.ALL, 6)

        grid = wx.FlexGridSizer(0, 2, 4, 6)
        self.fields = {}
        for key, label in (("track_width", "Track width (mm)"), ("clearance", "Clearance (mm)"),
                           ("via_diameter", "Via diameter (mm)"), ("via_drill", "Via drill (mm)"),
                           ("max_passes", "Max passes"), ("via_cost", "Via cost (grid steps)")):
            grid.Add(wx.StaticText(self, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            tc = wx.TextCtrl(self, value=f"{getattr(self.s, key):g}", size=(80, -1))
            self.fields[key] = tc
            grid.Add(tc)
        box = wx.StaticBoxSizer(wx.VERTICAL, self, "Rules (from the board's Default net class)")
        box.Add(grid, 0, wx.ALL, 4)
        left.Add(box, 0, wx.EXPAND | wx.ALL, 6)

        self.cb_rip = wx.CheckBox(self, label="Remove existing (unlocked) tracks of these nets first")
        n_loose = sum(map(len, loose_copper(self.board)))
        self.cb_loose = wx.CheckBox(self, label="Delete loose track pieces (copper touching no pad)"
                                    + (f" - {n_loose} found" if n_loose else ""))
        self.cb_loose.SetValue(True)          # dead-end copper only gets in the way
        self.cb_show = wx.CheckBox(self, label="Show existing tracks")
        self.cb_show.SetValue(True)
        self.cb_zone = wx.CheckBox(self, label="Connect pour nets (GND, +5...) to their pour")
        self.cb_zone.SetValue(True)
        self.cb_pour_tracks = wx.CheckBox(self, label="Route pour nets with tracks too (don't rely on the pour)")
        self.cb_fill = wx.CheckBox(self, label="Refill zones when done")
        self.cb_fill.SetValue(True)
        self.cb_live = wx.CheckBox(self, label="Live preview on the board after each pass (experimental)")
        for c in (self.cb_rip, self.cb_loose, self.cb_show, self.cb_zone, self.cb_pour_tracks, self.cb_fill,
                  self.cb_live):
            left.Add(c, 0, wx.ALL, 4)

        self.log = wx.TextCtrl(self, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.HSCROLL, size=(380, 200))
        self.log.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        left.Add(self.log, 1, wx.EXPAND | wx.ALL, 6)
        main.Add(left, 0, wx.EXPAND)

        right = wx.BoxSizer(wx.VERTICAL)
        # ---- big buttons + progress, right above the board view
        top = wx.BoxSizer(wx.HORIZONTAL)
        self.b_route = self._big_button("\u25B6  Route", wx.Colour(46, 125, 50))
        self.b_stop = self._big_button("\u25A0  Stop", wx.Colour(198, 40, 40))
        self.b_close = self._big_button("Close", wx.Colour(84, 90, 100))
        self.b_browser = self._big_button("\U0001F310 Browser", wx.Colour(37, 99, 160), width=150)
        self.b_browser.SetToolTip("Close this window and open the browser version of the router "
                                  "(the KiCad 9/10 API front end)")
        self.b_stop.Disable()
        for b in (self.b_route, self.b_stop, self.b_close, self.b_browser):
            top.Add(b, 0, wx.RIGHT, 8)
        prog = wx.BoxSizer(wx.VERTICAL)
        self.progress_text = wx.StaticText(self, label="Ready - press Route to start.")
        f = self.progress_text.GetFont()
        f.SetPointSize(f.GetPointSize() + 2)
        f.SetWeight(wx.FONTWEIGHT_BOLD)
        self.progress_text.SetFont(f)
        self.gauge = wx.Gauge(self, range=1000, size=(-1, 22), style=wx.GA_HORIZONTAL | wx.GA_SMOOTH)
        self.gauge_pass = wx.Gauge(self, range=1000, size=(-1, 8), style=wx.GA_HORIZONTAL | wx.GA_SMOOTH)
        self.progress_detail = wx.StaticText(self, label="")
        self.progress_detail.SetForegroundColour(wx.Colour(90, 90, 90))
        prog.Add(self.progress_text, 0, wx.BOTTOM, 2)
        prog.Add(self.progress_detail, 0, wx.BOTTOM, 4)
        prog.Add(self.gauge, 0, wx.EXPAND | wx.BOTTOM, 3)
        prog.Add(self.gauge_pass, 0, wx.EXPAND)
        top.Add(prog, 1, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 6)
        right.Add(top, 0, wx.EXPAND | wx.ALL, 8)
        self.status = wx.StaticText(self, label="Ready.")
        right.Add(self.status, 0, wx.LEFT | wx.RIGHT, 8)
        self.panel = ThinkingPanel(self)
        right.Add(self.panel, 1, wx.EXPAND | wx.ALL, 6)
        legend = wx.StaticText(self, label=layer_legend(self.board.copper_layers) +
                               " · orange heat = learned congestion · yellow = two nets fighting · cyan = A* wavefront")
        legend.SetForegroundColour(wx.Colour(120, 120, 120))
        right.Add(legend, 0, wx.ALL, 6)
        main.Add(right, 1, wx.EXPAND)
        self.SetSizer(main)
        self._place_window(self.GetParent())

        self.panel.set_board(self.board)
        self.cb_show.Bind(wx.EVT_CHECKBOX, self._on_show)
        self.scope.Bind(wx.EVT_RADIOBOX, lambda e: self._sync_rip())
        self._sync_rip()
        self.b_route.Bind(wx.EVT_BUTTON, self.on_route)
        self.b_stop.Bind(wx.EVT_BUTTON, self._on_stop)
        self.b_close.Bind(wx.EVT_BUTTON, self._on_close_button)
        self.b_browser.Bind(wx.EVT_BUTTON, self._on_browser)
        self.Bind(wx.EVT_CLOSE, self.on_close)

    def _big_button(self, label, colour, width=130):
        sz = wx.Size(width, 46)
        if hasattr(self, "FromDIP"):          # scale with the monitor's DPI setting
            sz = self.FromDIP(sz)
        b = _BigButtonBase(self, wx.ID_ANY, label, size=sz)
        f = b.GetFont()
        f.SetPointSize(f.GetPointSize() + 3)
        f.SetWeight(wx.FONTWEIGHT_BOLD)
        b.SetFont(f)
        try:
            b.SetBackgroundColour(colour)
            b.SetForegroundColour(wx.WHITE)
            if hasattr(b, "SetBezelWidth"):
                b.SetBezelWidth(1)
                b.SetUseFocusIndicator(False)
        except Exception:
            pass
        return b

    # -------------------------------------------------------------- window size
    _CFG_KEYS = ("x", "y", "w", "h", "max")

    def _place_window(self, parent):
        """Last size/position if it still fits a monitor, else 85% of KiCad's monitor."""
        cfg = wx.Config("VisibleAutorouter")
        vals = [cfg.ReadInt("win_" + k, -1) for k in self._CFG_KEYS]
        x, y, w, h, mx = vals
        if w > 300 and h > 200 and wx.Display.GetFromPoint(wx.Point(x + w // 2, y + 20)) != wx.NOT_FOUND:
            self.SetSize(x, y, w, h)
            if mx == 1:
                self.Maximize()
            return
        idx = wx.Display.GetFromWindow(parent) if parent is not None else wx.NOT_FOUND
        if idx == wx.NOT_FOUND:
            idx = 0
        area = wx.Display(idx).GetClientArea()
        w, h = int(area.width * 0.85), int(area.height * 0.85)
        self.SetSize(area.x + (area.width - w) // 2, area.y + (area.height - h) // 2, w, h)

    def _save_window(self):
        try:
            cfg = wx.Config("VisibleAutorouter")
            mx = self.IsMaximized()
            r = self.GetRect()
            if not mx:
                for k, v in zip(("x", "y", "w", "h"), (r.x, r.y, r.width, r.height)):
                    cfg.WriteInt("win_" + k, v)
            cfg.WriteInt("win_max", 1 if mx else 0)
            cfg.Flush()
        except Exception:
            pass

    def _on_close_button(self, _evt):
        if self.running:
            return
        self._save_window()
        if self.IsModal():
            self.EndModal(wx.ID_CANCEL)
        else:
            self.Close()

    def _on_browser(self, _evt):
        if self.running:
            return
        from . import browser_launch
        wx.BeginBusyCursor()
        try:
            ok, msg = browser_launch.launch()
        except Exception as exc:
            ok, msg = False, f"Could not start the browser version: {exc}"
        finally:
            wx.EndBusyCursor()
        if ok:      # close now: KiCad answers the browser version only once this window is gone
            self._on_close_button(None)
        else:
            wx.MessageBox(msg, "Visible Autorouter", wx.OK | wx.ICON_WARNING)

    def _on_stop(self, _evt):
        self.stop = True
        self.b_stop.SetLabel("Stopping...")
        self.b_stop.Disable()
        self.progress_text.SetLabel("Stopping after the current net...")

    def _show_progress(self, force=False):
        now = time.time()
        if not force and now - getattr(self, "_last_prog", 0.0) < 0.1:
            return
        self._last_prog = now
        sn = self.est.snapshot()
        self.gauge.SetValue(int(sn["fraction"] * 1000))
        self.gauge_pass.SetValue(int(1000 * sn["pass_done"] / max(1, sn["pass_total"])))
        if not self.stop:
            self.progress_text.SetLabel(sn["headline"])
            self.progress_detail.SetLabel(sn["detail"])

    def _on_show(self, _evt):
        self.panel.show_existing = self.cb_show.GetValue()
        self.panel.Refresh()

    def _sync_rip(self):
        unrouted = self.scope.GetSelection() == 0
        if unrouted:
            self.cb_rip.SetValue(False)
        self.cb_rip.Enable(not unrouted)

    def say(self, text):
        self.log.AppendText(text + "\n")

    def on_close(self, evt):
        if self.running:
            self.stop = True
            return
        self._save_window()
        evt.Skip()

    # -------------------------------------------------------------- routing
    def _read_settings(self) -> bool:
        try:
            for k, tc in self.fields.items():
                v = float(tc.GetValue())
                setattr(self.s, k, int(v) if k == "max_passes" else v)
        except ValueError:
            wx.MessageBox("Please enter numbers for the rules.", "Visible Autorouter")
            return False
        self.s.fanout_zone_nets = self.cb_zone.GetValue()
        self.s.skip_zone_nets = not self.cb_pour_tracks.GetValue()
        return True

    def on_route(self, _evt):
        if not self._read_settings():
            return
        sel = self.scope.GetSelection()
        if sel == 0:
            nets = nets_to_finish(self.board)
            self.say(f"Unrouted only: finishing {len(nets)} net(s); existing tracks and vias are kept.")
        elif sel == 1:
            nets = sorted(self.selected)
        else:
            nets = None
        removed_now = []          # copper this run deletes first (restored if it is discarded)
        if self.cb_loose.GetValue():
            lt, lv = loose_copper(self.board, None if sel == 0 else nets)
            doomed, seen = [], set()
            for m in lt + lv:
                it = self.board.refs.get(id(m))
                if it is not None and id(it) not in seen and not it.IsLocked():
                    seen.add(id(it))
                    doomed.append(it)
            for it in doomed:
                self.kb.Remove(it)
            _GRAVEYARD.extend(doomed)
            removed_now += doomed
            self.say(f"Deleted {len(doomed)} loose track piece(s)/via(s) that touch no pad.")
            self.board, _ = board_from_pcbnew(self.kb)
            if sel == 0:
                nets = nets_to_finish(self.board)
        if self.cb_rip.GetValue() and sel != 0:
            target = set(nets) if nets else set(self.board.nets())
            n = remove_existing_tracks(self.kb, target)
            removed_now += _GRAVEYARD[len(_GRAVEYARD) - n:] if n else []
            self.say(f"Removed {n} existing track/via item(s).")
            self.board, _ = board_from_pcbnew(self.kb)

        t0 = time.time()
        pb = build_problem(self.board, self.s, nets)
        g = pb.grid
        if not pb.nets:
            self.say("Nothing to route - every net is already connected.")
            self.status.SetLabel("Nothing to route - every net is already connected.")
            return
        self.say(f"Grid {g.nx} x {g.ny} x {g.nl} layers @ {g.pitch:.3f} mm, {len(pb.nets)} nets.")
        if self.board.outline_warning:
            self.say("WARNING: " + self.board.outline_warning)
        router = Router(pb, self.on_event, record_exploration=True)
        self.panel.attach(router, self.board)
        self.router = router
        self.running, self.stop = True, False
        self.est = ProgressEstimator(self.s.max_passes)
        self.gauge.SetValue(0)
        self.gauge_pass.SetValue(0)
        self.progress_text.SetLabel("Starting...")
        self.progress_detail.SetLabel("")
        self.b_route.Disable()
        self.b_close.Disable()
        self.b_stop.SetLabel("\u25A0  Stop")
        self.b_stop.Enable()
        self.preview = BoardWriter(self.kb, self.s) if self.cb_live.GetValue() else None
        try:
            res = router.run()
        finally:
            self.running = False
            self.b_stop.Disable()
            self.b_close.Enable()
            self.b_route.Enable()
            if self.preview is not None:
                self.preview.clear()
        if res.cancelled and not self._keep_after_stop(res, len(removed_now)):
            for it in removed_now:            # put back what this run deleted
                self.kb.Add(it)
            pcbnew.Refresh()
            self.say("Stopped - routing discarded; the board is as it was before Route."
                     + (f" ({len(removed_now)} deleted item(s) restored.)" if removed_now else ""))
            self.status.SetLabel("Stopped - routing discarded.")
            self.progress_text.SetLabel(f"Stopped after {fmt(time.time() - t0)} - discarded")
            self.progress_detail.SetLabel("")
            self.panel.update([], None)
            return
        writer = BoardWriter(self.kb, self.s)
        for name, nr in res.nets.items():
            writer.add_net(name, nr.segments, nr.vias, nr.widths)
        if self.cb_fill.GetValue() and len(list(self.kb.Zones())):
            try:
                pcbnew.ZONE_FILLER(self.kb).Fill(self.kb.Zones())
            except Exception as exc:   # pragma: no cover
                self.say(f"Zone fill failed: {exc}")
        pcbnew.Refresh()
        self.say(res.summary())
        stuck = sorted(n for n, nr in res.nets.items() if not nr.complete)
        if sel == 0 and stuck:
            self.say(f"{len(stuck)} net(s) could not be finished around the existing tracks "
                     f"({', '.join(stuck)}). To let the router move copper: select those parts, "
                     "choose Selected, tick 'Remove existing tracks' and Route again.")
        self.say(f"Added {len(writer.items)} items in {time.time() - t0:.1f}s. "
                 "Close this window, then Edit > Undo reverts everything. Run DRC to double-check.")
        self.status.SetLabel(res.summary().splitlines()[0])
        self.gauge.SetValue(1000)
        self.gauge_pass.SetValue(1000)
        self.progress_text.SetLabel(("Stopped" if res.cancelled else "Done") + f" in {fmt(time.time() - t0)}")
        self.progress_detail.SetLabel(res.summary().splitlines()[0])
        # re-read: the new copper is now "existing" for the view and for a second run
        self.board, self.selected = board_from_pcbnew(self.kb)
        st = net_status(self.board)
        cnt = {k: sum(1 for v in st.values() if v == k) for k in ("routed", "partial", "unrouted")}
        self.summary.SetLabel(board_summary(self.board, cnt))
        self.panel.board = self.board
        self.panel.update([], None)

    def _keep_after_stop(self, res, n_removed: int = 0) -> bool:
        """Stopped part-way: ask whether to keep what has been routed so far."""
        done = res.complete_nets
        total = len(res.nets)
        segs = sum(len(n.segments) for n in res.nets.values())
        msg = (f"Routing was stopped after {len(res.passes)} pass(es).\n\n"
               f"So far: {done} of {total} nets completely routed, {segs} track segments.\n"
               + (f"Nets still in conflict are left out: {', '.join(res.removed)}.\n" if res.removed else "")
               + (f"\n{n_removed} existing track/via item(s) were deleted before routing: Keep leaves "
                  "unfinished nets without them, Discard puts them all back.\n" if n_removed else "")
               + "\nKeep this routing on the board, or throw it away?")
        dlg = wx.MessageDialog(self, msg, "Visible Autorouter - stopped",
                               wx.YES_NO | wx.YES_DEFAULT | wx.ICON_QUESTION)
        try:
            dlg.SetYesNoLabels("Keep what was routed", "Discard it")
        except Exception:
            pass
        keep = dlg.ShowModal() == wx.ID_YES
        dlg.Destroy()
        return keep

    def on_event(self, ev, d):
        self.est.feed(ev, d)
        self._show_progress(force=ev in ("pass_start", "pass_end", "cleanup_start", "done"))
        if ev == "pass1_stats":
            self.say(f"Pass 1: {d['nets']} nets on {d['cores']} cores in {d['seconds']:.1f}s "
                     f"({d['busy']:.1f}s of routing); slowest nets: "
                     + ", ".join(f"{n} {s:.1f}s" for n, s in d["slowest"]))
        if ev == "workers":
            if d.get("error"):
                self.say(f"Using 1 CPU core (could not start more: {d['error']})")
            else:
                self.say(f"Pass 1 on {d['workers']} CPU cores (started in {d['seconds']:.1f}s).")
        if ev == "global_done":
            self.say(f"Global plan: {d['regions']} regions, corridors set in {d['seconds']:.2f}s.")
        if ev == "pass_start":
            self.status.SetLabel(f"Pass {d['pass']}: routing {len(d['nets'])} net(s), "
                                 f"congestion price x{d['pres_fac']:.2f}")
        elif ev == "connection":
            now = time.time()
            if now - self._last_draw > 0.25:     # ~4 redraws a second keeps it live and cheap
                self._last_draw = now
                self.panel.update(d["explored"] or [], d["net"])
        elif ev == "pass_end":
            self.say(f"pass {d['pass']:2d}: {d['overused']:4d} overused, "
                     f"{len(d['conflict_nets']):3d} nets in conflict, {d['vias']:3d} vias")
            self.panel.rebuild_base()
            self.panel.update([], None)
            if self.preview is not None:
                self.preview.clear()
                for j in self.router.pb.nets:
                    segs, vias, wds = [], [], []
                    for p in j.paths:
                        sg, vi, wd = self.router.path_geometry(p, j)
                        segs += sg
                        vias += vi
                        wds += wd
                    self.preview.add_net(j.name, segs, vias, wds)
                pcbnew.Refresh()
        now = time.time()
        if ev != "connection" or now - getattr(self, "_last_yield", 0.0) > 0.05:
            self._last_yield = now
            wx.Yield()          # keep the window responsive (~20x a second is plenty)
        return not self.stop


class AutorouterPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "Visible Autorouter"
        self.category = "Routing"
        self.description = ("Multi-pass negotiated-congestion autorouter. Routes the selected "
                            "nets/footprints, or everything, and shows its thinking.")
        self.show_toolbar_button = True
        icon = os.path.join(os.path.dirname(__file__), "icon.png")
        if os.path.exists(icon):
            self.icon_file_name = icon

    def Run(self):
        kb = pcbnew.GetBoard()
        parent = next((w for w in wx.GetTopLevelWindows() if "pcbnew" in w.GetName().lower()
                       or "PcbFrame" in w.GetName()), None)
        dlg = AutorouterDialog(parent, kb)
        dlg.ShowModal()
        dlg.Destroy()
