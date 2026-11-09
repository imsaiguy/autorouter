"""Custom, trapezoid and chamfered pads: the router's idea of the copper must
never be smaller than KiCad's (else a track could be placed too close).

Builds a board with KiCad's own pcbnew API, then probes points around every pad:
any point KiCad's PAD.HitTest calls copper must be inside the router's model.
"""
import math
import os
import sys

import pytest

pcbnew = pytest.importorskip("pcbnew")

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, os.path.join(ROOT, "examples")]

from kicad_autorouter import RouteSettings, route  # noqa: E402
from kicad_autorouter.kicad_pcb import PcbFile  # noqa: E402

MM = pcbnew.FromMM


def V(x, y):
    return pcbnew.VECTOR2I(MM(x), MM(y))


def _pts(pairs):
    v = pcbnew.VECTOR_VECTOR2I()
    for x, y in pairs:
        v.append(V(x, y))
    return v


def _fp(board, ref, x, y, rot=0.0, back=False):
    fp = pcbnew.FOOTPRINT(board)
    fp.SetReference(ref)
    board.Add(fp)
    return fp


def _place(fp, x, y, rot, back):
    fp.SetPosition(V(x, y))
    if rot:
        fp.SetOrientationDegrees(rot)
    if back:
        fp.Flip(V(x, y), False)


def _pad(fp, num, lx, ly, net):
    p = pcbnew.PAD(fp)
    p.SetNumber(str(num))
    p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
    p.SetLayerSet(p.SMDMask())
    if hasattr(p, "SetFPRelativePosition"):
        p.SetFPRelativePosition(V(lx, ly))
    else:
        p.SetPos0(V(lx, ly))
    p.SetPosition(V(lx, ly))
    p.SetNet(net)
    return p


def build(path):
    b = pcbnew.BOARD()
    r = pcbnew.PCB_SHAPE(b)
    r.SetShape(pcbnew.SHAPE_T_RECT)
    r.SetStart(V(0, 0))
    r.SetEnd(V(60, 40))
    r.SetLayer(pcbnew.Edge_Cuts)
    b.Add(r)
    nets = []
    for n in ("A", "B", "C", "D"):
        ni = pcbnew.NETINFO_ITEM(b, n)
        b.Add(ni)
        nets.append(ni)
    for k, (x, y, rot, back) in enumerate([(12, 10, 0, False), (40, 12, 30, False),
                                           (14, 30, 90, False), (44, 30, 0, True)]):
        fp = _fp(b, f"U{k + 1}", x, y)
        # 1: custom pad = round anchor + polygon tab + stroked segment + circle + rect
        p = _pad(fp, 1, -4, 0, nets[k % 4])
        p.SetShape(pcbnew.PAD_SHAPE_CUSTOM)
        p.SetAnchorPadShape(pcbnew.PAD_SHAPE_CIRCLE)
        p.SetSize(V(0.8, 0.8))
        p.AddPrimitivePoly(_pts([(0, -0.3), (2.5, -0.3), (2.5, 0.3), (0, 0.3)]), 0, True)
        p.AddPrimitiveSegment(V(0, 0), V(0, 1.8), MM(0.4))
        p.AddPrimitiveCircle(V(0, -1.6), MM(0.5), 0, True)
        p.AddPrimitiveRect(V(-1.5, -0.4), V(-0.8, 0.4), 0, True)
        fp.Add(p)
        # 2: trapezoid
        p = _pad(fp, 2, 1, 0, nets[(k + 1) % 4])
        p.SetShape(pcbnew.PAD_SHAPE_TRAPEZOID)
        p.SetSize(V(1.6, 1.0))
        p.SetDelta(V(0, 0.5))
        fp.Add(p)
        # 3: chamfered rectangle
        p = _pad(fp, 3, 3.5, 0, nets[(k + 2) % 4])
        p.SetShape(pcbnew.PAD_SHAPE_CHAMFERED_RECT)
        p.SetSize(V(1.2, 1.2))
        p.SetChamferRectRatio(0.3)
        p.SetChamferPositions(0xF)
        fp.Add(p)
        _place(fp, x, y, rot, back)
    b.BuildConnectivity()
    pcbnew.SaveBoard(path, b)
    return path


def _misses(kpad, model_pad, step=0.05, reach=3.5):
    """Points KiCad calls copper that the model calls outside (> 1 micron)."""
    pos = kpad.GetPosition()
    cx, cy = pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y)
    n = int(reach / step)
    bad = 0
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = cx + i * step, cy + j * step
            if kpad.HitTest(V(x, y)) and model_pad.distance(x, y) > 0.005:
                bad += 1
    return bad


@pytest.fixture(scope="module")
def board_path(tmp_path_factory):
    return build(str(tmp_path_factory.mktemp("pads") / "pads.kicad_pcb"))


def test_file_reader_covers_every_copper_point(board_path):
    kb = pcbnew.LoadBoard(board_path)
    model = {p.ref: p for p in PcbFile(board_path).board().pads}
    checked = 0
    for fp in kb.GetFootprints():
        for kp in fp.Pads():
            mp = model[f"{fp.GetReference()}.{kp.GetNumber()}"]
            assert _misses(kp, mp) == 0, f"{fp.GetReference()}.{kp.GetNumber()} under-estimated"
            checked += 1
    assert checked == 12


def test_plugin_reader_covers_every_copper_point(board_path):
    try:
        from kicad_autorouter.plugin import board_from_pcbnew
    except Exception:
        pytest.skip("wxPython not available in this Python")
    kb = pcbnew.LoadBoard(board_path)
    b, _ = board_from_pcbnew(kb)
    model = {p.ref: p for p in b.pads}
    for fp in kb.GetFootprints():
        for kp in fp.Pads():
            mp = model[f"{fp.GetReference()}.{kp.GetNumber()}"]
            assert _misses(kp, mp) == 0, f"{fp.GetReference()}.{kp.GetNumber()} under-estimated"


def test_routing_around_odd_pads_passes_kicad_drc(board_path, tmp_path):
    from drc_check import drc
    pcb = PcbFile(board_path)
    s = RouteSettings()
    res = route(pcb.board(), s)
    pcb.add_routes(res, s)
    out = str(tmp_path / "routed.kicad_pcb")
    pcb.save(out)
    kinds, unconnected, _ = drc(out)
    copper = {k: v for k, v in kinds.items() if k in ("clearance", "shorting_items", "tracks_crossing",
                                                      "copper_edge_clearance", "hole_clearance")}
    assert copper == {}, kinds
