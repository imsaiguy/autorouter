"""API-agnostic board model.

Everything the router needs is described with these small dataclasses, in
millimetres, using KiCad's coordinate convention (x right, y *down*).  The
same model is filled either from a ``.kicad_pcb`` file (``kicad_pcb.py``) or
from a live board inside KiCad (``plugin.py``), so the router itself never
touches a KiCad API.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

Point = Tuple[float, float]
Segment = Tuple[float, float, float, float]  # x1, y1, x2, y2


@dataclass
class Pad:
    net: str                    # "" means unconnected copper (an obstacle for everyone)
    x: float
    y: float
    w: float
    h: float
    shape: str = "rect"         # rect | roundrect | circle | oval
    angle: float = 0.0          # degrees, KiCad convention (CCW on screen)
    layers: Set[str] = field(default_factory=set)   # copper layer names
    drill: float = 0.0          # >0 for through-hole pads
    ref: str = ""               # "U1.3" — only used for messages
    # exact copper outline(s), overriding `shape`: one ring [(x, y), ...] or a list of rings
    # (a custom pad can be several separate pieces)
    poly: Optional[list] = None
    clearance: float = 0.0      # the pad's own clearance override (fiducials ...), 0 = none

    def rings(self) -> List[List[Point]]:
        if not self.poly:
            return []
        first = self.poly[0]
        if isinstance(first[0], (int, float)):
            return [self.poly]
        return [r for r in self.poly if r]

    def local(self, px: float, py: float) -> Point:
        """Point -> pad-local frame (undo position and rotation)."""
        dx, dy = px - self.x, py - self.y
        a = math.radians(self.angle)
        c, s = math.cos(a), math.sin(a)
        # KiCad rotates CCW with y pointing down: inverse rotation
        return dx * c - dy * s, dx * s + dy * c

    def distance(self, px: float, py: float) -> float:
        """Distance from a point to the pad copper (0 inside)."""
        if self.poly:
            return min(_poly_distance(r, px, py) for r in self.rings())
        lx, ly = self.local(px, py)
        hw, hh = self.w / 2.0, self.h / 2.0
        if self.shape == "circle":
            return max(0.0, math.hypot(lx, ly) - hw)
        if self.shape == "oval":
            # stadium: a segment along the long axis, inflated by the short half-size
            if hw >= hh:
                r, half = hh, hw - hh
                d = math.hypot(max(abs(lx) - half, 0.0), ly)
            else:
                r, half = hw, hh - hw
                d = math.hypot(lx, max(abs(ly) - half, 0.0))
            return max(0.0, d - r)
        # rect / roundrect / anything else: rectangle (conservative for roundrect)
        dx = max(abs(lx) - hw, 0.0)
        dy = max(abs(ly) - hh, 0.0)
        return math.hypot(dx, dy)

    def radius(self) -> float:
        """Radius of a circle that encloses the pad."""
        if self.poly:
            return max(math.hypot(x - self.x, y - self.y) for r in self.rings() for (x, y) in r)
        return math.hypot(self.w, self.h) / 2.0


def _poly_distance(pts: List[Point], px: float, py: float) -> float:
    """0 inside a closed polygon, otherwise the distance to its boundary."""
    inside = False
    best = float("inf")
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if (y1 > py) != (y2 > py) and px < x1 + (py - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 1e-18 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
        d = math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))
        if d < best:
            best = d
    return 0.0 if inside else best


@dataclass
class Track:
    net: str
    x1: float
    y1: float
    x2: float
    y2: float
    width: float
    layer: str


@dataclass
class Via:
    net: str
    x: float
    y: float
    diameter: float
    drill: float = 0.0
    layers: Optional[Set[str]] = None   # None = through all copper


@dataclass
class Board:
    copper_layers: List[str]
    outline: List[Segment]                     # closed outline(s) as line segments
    pads: List[Pad] = field(default_factory=list)
    tracks: List[Track] = field(default_factory=list)
    vias: List[Via] = field(default_factory=list)
    zone_nets: Set[str] = field(default_factory=set)
    zone_layers: dict = field(default_factory=dict)   # net -> set of copper layers with a pour
    # net -> [(layers, rings)]: each zone's OUTLINE (where it may pour copper), so a pad
    # only counts as connected by a pour if it actually lies inside that pour's area
    zone_outlines: dict = field(default_factory=dict)
    # net -> [priority] in the same order as zone_outlines: where zones of two nets
    # overlap on a layer, KiCad fills the overlap with the higher priority one
    zone_priority: dict = field(default_factory=dict)
    zone_clearance: dict = field(default_factory=dict)   # net -> largest clearance of its zones (mm)
    holes: List[Tuple[float, float, float]] = field(default_factory=list)  # NPTH x, y, dia
    outline_warning: str = ""                  # the outline had to be pieced together (see readers)
    # rule areas (keep-outs): [(layers, rings, no tracks, no vias)]
    keepouts: list = field(default_factory=list)

    def bbox(self) -> Tuple[float, float, float, float]:
        if self.outline:
            xs = [p for s in self.outline for p in (s[0], s[2])]
            ys = [p for s in self.outline for p in (s[1], s[3])]
            return min(xs), min(ys), max(xs), max(ys)
        pts = [(p.x, p.y) for p in self.pads]
        if not pts:
            raise ValueError("board has no outline and no pads")
        m = 2.0
        return (min(p[0] for p in pts) - m, min(p[1] for p in pts) - m,
                max(p[0] for p in pts) + m, max(p[1] for p in pts) + m)

    def nets(self) -> List[str]:
        return sorted({p.net for p in self.pads if p.net})


@dataclass
class RouteSettings:
    track_width: float = 0.25
    clearance: float = 0.2
    via_diameter: float = 0.6
    via_drill: float = 0.3
    edge_clearance: float = 0.5
    hole_clearance: float = 0.25          # copper to drilled hole (KiCad default)
    grid: Optional[float] = None          # None -> track_width + clearance
    layers: Optional[List[str]] = None    # None -> all copper layers
    layer_directions: Optional[Sequence[str]] = None  # 'H'/'V'/'-' per layer; None -> alternate
    via_cost: float = 12.0                # in grid steps
    wrong_way_cost: float = 1.6           # multiplier for moving against the layer's direction
    bend_cost: float = 0.4                # per direction change
    max_passes: int = 40
    global_routing: bool = True           # plan coarse regions first, then route inside corridors
    region_cells: int = 6                 # global-routing region size, in grid cells (6 = 2.7 mm)
    present_factor: float = 0.6           # PathFinder: initial present-congestion factor
    present_growth: float = 1.6           # ...multiplied by this each pass
    history_factor: float = 0.8           # PathFinder: history cost increment per overuse
    skip_zone_nets: bool = True           # nets that own a copper pour (GND...) are left to the pour...
    allow_via_in_pad: bool = False
    max_pour_passes: int = 12             # passes spent trying to keep pours in one piece
    fanout_zone_nets: bool = True         # ...but SMD pads off the pour layer get a stub + via down to it
    # inner layers covered by pours (GND / power planes) stay solid: no other net's track
    # on them, only vias passing through.  True: route signals on them too.
    tracks_on_planes: bool = False
    via_per_pour_pad: bool = True         # a pour on another layer (GND plane): every SMD pad of the
                                          # net gets its own short via down to it
    keep_failed_partial: bool = True      # keep the connections that did route for a failed net
    workers: int = 0                      # CPU cores for routing: 0 = automatic, 1 = single core
    native: bool = True                   # use the compiled search core when there is one
    min_track_width: float = 0.2          # board setup minimum (KiCad's default); stubs never narrower
    # per-net rules from the net classes: net -> (track_width, clearance) in mm.  Nets not
    # listed use track_width / clearance above (the Default class).
    net_rules: Dict[str, Tuple[float, float]] = field(default_factory=dict)

    def width_for(self, net: str) -> float:
        r = self.net_rules.get(net)
        return r[0] if r else self.track_width

    def clearance_for(self, net: str) -> float:
        r = self.net_rules.get(net)
        return max(self.clearance, r[1]) if r else self.clearance

    def pitch(self) -> float:
        return self.grid if self.grid else round(self.track_width + self.clearance, 4)
