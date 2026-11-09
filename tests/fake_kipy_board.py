"""A stand-in for kipy's Board built from REAL kipy objects (no KiCad needed).

It mirrors the calls IpcBoard makes against KiCad 9/10's IPC API, so the
adapter's reading and writing code paths run for real in the tests.
"""
import math
import uuid

from kipy.board_types import BoardSegment, FootprintInstance, Net, Pad as KPad, Track as KTrack, Via as KVia, Zone
from kipy.geometry import PolygonWithHoles, PolyLine, PolyLineNode, Vector2
from kipy.proto.board.board_types_pb2 import BoardLayer, PadType, ZoneType

NM = 1_000_000
LAYERS = {"F.Cu": BoardLayer.BL_F_Cu, "B.Cu": BoardLayer.BL_B_Cu}


def _v(x, y):
    return Vector2.from_xy(int(round(x * NM)), int(round(y * NM)))


def _outline(model_pad):
    """Polygon for a model pad (what KiCad's GetPadShapeAsPolygon returns)."""
    p = model_pad
    if p.shape == "circle":
        r = p.w / 2
        return [(p.x + r * math.cos(2 * math.pi * k / 32), p.y + r * math.sin(2 * math.pi * k / 32)) for k in range(32)]
    a = math.radians(p.angle)
    c, s = math.cos(a), math.sin(a)
    pts = []
    for (lx, ly) in ((-p.w / 2, -p.h / 2), (p.w / 2, -p.h / 2), (p.w / 2, p.h / 2), (-p.w / 2, p.h / 2)):
        pts.append((p.x + lx * c + ly * s, p.y - lx * s + ly * c))
    return pts


class FakeCommit:
    pass


class FakeKiCadBoard:
    def __init__(self, model_board, selected_refs=()):
        self.model = model_board
        self.nets = {}
        self.pads = []
        self.poly = {}
        self.created = []
        self.removed = []
        self.commits = []
        self.fps = []
        refs = {}
        for mp in model_board.pads:
            ref, num = mp.ref.split(".")
            if ref not in refs:
                fp = FootprintInstance()
                fp._proto.id.value = str(uuid.uuid4())
                fp.reference_field.text.value = ref
                refs[ref] = fp
                self.fps.append(fp)
            kp = KPad()
            kp._proto.id.value = str(uuid.uuid4())
            kp._proto.parent.value = refs[ref].id.value
            kp.number = num
            kp.position = _v(mp.x, mp.y)
            if mp.net:
                kp.net = self._net(mp.net)
            kp.pad_type = PadType.PT_PTH if mp.drill > 0 else PadType.PT_SMD
            kp.padstack.layers = [LAYERS[l] for l in sorted(mp.layers)]
            if mp.drill > 0:
                kp.padstack.drill.diameter = _v(mp.drill, mp.drill)
            self.pads.append(kp)
            self.poly[kp.id.value] = _outline(mp)
            if ref in selected_refs:
                defn_pad = KPad()
                defn_pad.CopyFrom = None
        for (hx, hy, hd) in model_board.holes:
            kp = KPad()
            kp._proto.id.value = str(uuid.uuid4())
            kp.position = _v(hx, hy)
            kp.pad_type = PadType.PT_NPTH
            kp.padstack.drill.diameter = _v(hd, hd)
            self.pads.append(kp)
        # existing copper already on the board
        self.existing = []
        for t in model_board.tracks:
            kt = KTrack()
            kt.start, kt.end = _v(t.x1, t.y1), _v(t.x2, t.y2)
            kt.width = int(round(t.width * NM))
            kt.layer = LAYERS[t.layer]
            kt.net = self._net(t.net)
            self.existing.append(kt)
        for v in model_board.vias:
            kv = KVia()
            kv.position = _v(v.x, v.y)
            kv.diameter = int(round(v.diameter * NM))
            kv.drill_diameter = int(round((v.drill or 0.3) * NM))
            kv.net = self._net(v.net)
            self.existing.append(kv)
        self.selection = [p for p in self.pads
                          if any(p.parent and p.parent.value == f.id.value and
                                 f.reference_field.text.value in selected_refs for f in self.fps)]

    def _net(self, name):
        if name not in self.nets:
            self.nets[name] = Net(name=name)
        return self.nets[name]

    # ---- reading
    def get_enabled_layers(self):
        return [BoardLayer.BL_F_Cu, BoardLayer.BL_B_Cu, BoardLayer.BL_Edge_Cuts]

    def get_footprints(self):
        return self.fps

    def get_pads(self):
        return self.pads

    def get_pad_shapes_as_polygons(self, pads, layer=BoardLayer.BL_F_Cu):
        def mk(p):
            pts = self.poly.get(p.id.value)
            if not pts:
                return None
            pl = PolyLine()
            for (x, y) in pts:
                pl.append(PolyLineNode.from_point(_v(x, y)))
            pl.closed = True
            pw = PolygonWithHoles()
            pw.outline = pl
            return pw
        if isinstance(pads, KPad):
            return mk(pads)
        return [q for q in (mk(p) for p in pads) if q is not None]

    def get_tracks(self):
        return [i for i in self.existing + self.created if type(i).__name__ in ("Track", "ArcTrack")]

    def get_vias(self):
        return [i for i in self.existing + self.created if type(i).__name__ == "Via"]

    def get_zones(self):
        out = []
        for net, layers in self.model.zone_layers.items():
            z = Zone()
            z.type = ZoneType.ZT_COPPER
            z.layers = [LAYERS[l] for l in layers]
            z.net = self._net(net)
            out.append(z)
        return out

    def get_shapes(self):
        out = []
        for (x1, y1, x2, y2) in self.model.outline:
            s = BoardSegment()
            s.start, s.end = _v(x1, y1), _v(x2, y2)
            s.layer = BoardLayer.BL_Edge_Cuts
            out.append(s)
        return out

    def get_selection(self, types=None):
        return self.selection

    def get_nets(self, netclass_filter=None):
        for p in self.model.pads:
            if p.net:
                self._net(p.net)
        return list(self.nets.values())

    netclasses = {}          # net name -> kipy NetClass (set by a test)

    def get_netclass_for_nets(self, nets):
        return dict(self.netclasses)

    # ---- writing
    def begin_commit(self):
        c = FakeCommit()
        self.commits.append(["open", c])
        return c

    def create_items(self, items):
        self.created.extend(items)
        return items

    def remove_items(self, items):
        self.removed.extend(items)
        ids = {id(i) for i in items}
        self.created = [c for c in self.created if id(c) not in ids]
        self.existing = [c for c in self.existing if id(c) not in ids]

    def push_commit(self, commit, message=""):
        self.commits[-1][0] = "pushed: " + message

    def drop_commit(self, commit):
        self.commits[-1][0] = "dropped"

    def refill_zones(self, *a, **k):
        self.refilled = True
