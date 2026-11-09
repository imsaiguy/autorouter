"""Fine-pitch pads (a 0.5 mm QFN with 0.25 mm pins, 0.25/0.15 mm rules): the grid
(0.4 mm) often has no legal point next to a pin.  Escape stubs must get the pins out,
and nothing may come closer to a neighbouring pin than the clearance - for any
alignment of the IC against the grid (a user's board left two QFN nets unrouted)."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, os.path.join(ROOT, "examples"), HERE]

from kicad_autorouter import RouteSettings, route  # noqa: E402
from kicad_autorouter.model import Board, Pad  # noqa: E402
from geom_check import clearance_violations, connected  # noqa: E402
from synthetic import rect_outline  # noqa: E402

RULES = dict(track_width=0.25, clearance=0.15)


def qfn_board(cx, cy, pitch=0.5, n=4):
    """QFN-16 (0.5 mm pitch, 0.25 x 0.8 mm pins) with every pin wired to a pad outside."""
    pads, k = [], 0
    for side in range(4):
        for i in range(n):
            o = (i - (n - 1) / 2) * pitch
            x, y, w, h = {0: (cx - 1.5, cy + o, 0.8, 0.25), 1: (cx + o, cy + 1.5, 0.25, 0.8),
                          2: (cx + 1.5, cy - o, 0.8, 0.25), 3: (cx - o, cy - 1.5, 0.25, 0.8)}[side]
            net = f"N{k}"
            k += 1
            pads.append(Pad(net, x, y, w, h, "rect", 0, {"F.Cu"}, ref=f"U1.{k}"))
            ex, ey = {0: (-1, 0), 1: (0, 1), 2: (1, 0), 3: (0, -1)}[side]
            rx, ry = cx + ex * 6 + (o * 3 if ex == 0 else 0), cy + ey * 6 + (o * 3 if ey == 0 else 0)
            pads.append(Pad(net, rx, ry, 0.9, 0.95, "rect", 0, {"F.Cu"}, ref=f"R{k}.1"))
    pads.append(Pad("GND", cx, cy, 1.7, 1.7, "rect", 0, {"F.Cu"}, ref="U1.17"))
    return Board(["F.Cu", "B.Cu"], rect_outline(5, 0, 35, 30), pads)


@pytest.mark.parametrize("off", [0.0, 0.05, 0.1, 0.15, 0.2, 0.25])
def test_qfn_pins_escape_at_any_grid_alignment(off):
    b = qfn_board(20 + off, 15 + off * 0.7)
    s = RouteSettings(**RULES)
    res = route(b, s)
    assert clearance_violations(b, res, s) == []
    done = [n for n, nr in res.nets.items() if nr.complete]
    assert len(done) == len(res.nets), res.summary()
    for n in done:
        assert connected(b, res, n, s.track_width), n


def test_qfn_routing_passes_kicad_drc(tmp_path):
    pcbnew = pytest.importorskip("pcbnew")
    from drc_check import drc
    from kicad_autorouter.kicad_pcb import PcbFile
    MM = pcbnew.FromMM

    def V(x, y):
        return pcbnew.VECTOR2I(MM(x), MM(y))
    model = qfn_board(20.1, 15.07)
    kb = pcbnew.BOARD()
    r = pcbnew.PCB_SHAPE(kb)
    r.SetShape(pcbnew.SHAPE_T_RECT)
    r.SetStart(V(5, 0))
    r.SetEnd(V(35, 30))
    r.SetLayer(pcbnew.Edge_Cuts)
    kb.Add(r)
    nets = {}
    for p in model.pads:
        if p.net not in nets:
            ni = pcbnew.NETINFO_ITEM(kb, p.net)
            kb.Add(ni)
            nets[p.net] = ni
    fps = {}
    for p in model.pads:
        ref, num = p.ref.split(".")
        if ref not in fps:
            fp = pcbnew.FOOTPRINT(kb)
            fp.SetReference(ref)
            kb.Add(fp)
            fps[ref] = fp
        fp = fps[ref]
        pad = pcbnew.PAD(fp)
        pad.SetNumber(num)
        pad.SetShape(pcbnew.PAD_SHAPE_RECT)
        pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pad.SetLayerSet(pad.SMDMask())
        pad.SetSize(V(p.w, p.h))
        if hasattr(pad, "SetFPRelativePosition"):
            pad.SetFPRelativePosition(V(p.x, p.y))
        else:
            pad.SetPos0(V(p.x, p.y))
        pad.SetPosition(V(p.x, p.y))
        pad.SetNet(nets[p.net])
        fp.Add(pad)
    path = str(tmp_path / "qfn.kicad_pcb")
    pcbnew.SaveBoard(path, kb)
    pro = os.path.splitext(path)[0] + ".kicad_pro"
    import json
    data = json.load(open(pro))
    for c in data["net_settings"]["classes"]:
        if c["name"] == "Default":
            c.update(track_width=0.25, clearance=0.15)
    json.dump(data, open(pro, "w"))
    pcb = PcbFile(path)
    s = pcb.settings_from_project()
    assert (s.track_width, s.clearance) == (0.25, 0.15)
    res = route(pcb.board(), s)
    pcb.add_routes(res, s)
    out = str(tmp_path / "routed.kicad_pcb")
    pcb.save(out)
    import shutil
    shutil.copy(pro, str(tmp_path / "routed.kicad_pro"))
    kinds, unconnected, text = drc(out)
    copper = {k: v for k, v in kinds.items() if k in ("clearance", "shorting_items", "tracks_crossing",
                                                      "copper_edge_clearance", "hole_clearance")}
    i = text.find("[clearance]")
    assert copper == {}, text[i:i + 2500]
    assert unconnected == 0, text[text.find("unconnected"):][:1500]
