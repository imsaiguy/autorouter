"""Pour nets whose zone does not reach every pad (a small +5 zone in a corner,
pads all over the board) must still be fully connected after routing.

Reproduces a real board where GND and +5 were left in the ratsnest: the router
had treated 'pad on the zone's layer' as 'connected by the zone'.
"""
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


def _zone(board, net, layer, pts):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    ol = z.Outline()
    ol.NewOutline()
    for (x, y) in pts:
        ol.Append(MM(x), MM(y))
    board.Add(z)


def _part(board, ref, x, y, nets, smd=True):
    fp = pcbnew.FOOTPRINT(board)
    fp.SetReference(ref)
    board.Add(fp)
    for k, net in enumerate(nets):
        p = pcbnew.PAD(fp)
        p.SetNumber(str(k + 1))
        lx = (k - (len(nets) - 1) / 2) * 1.9
        if smd:
            p.SetShape(pcbnew.PAD_SHAPE_RECT)
            p.SetSize(V(1.0, 1.3))
            p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
            p.SetLayerSet(p.SMDMask())
        else:
            p.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
            p.SetSize(V(1.7, 1.7))
            p.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
            p.SetLayerSet(p.PTHMask())
            p.SetDrillSize(V(1.0, 1.0))
        if hasattr(p, "SetFPRelativePosition"):
            p.SetFPRelativePosition(V(lx, 0))
        else:
            p.SetPos0(V(lx, 0))
        p.SetPosition(V(lx, 0))
        p.SetNet(net)
        fp.Add(p)
    fp.SetPosition(V(x, y))


def build(path):
    b = pcbnew.BOARD()
    r = pcbnew.PCB_SHAPE(b)
    r.SetShape(pcbnew.SHAPE_T_RECT)
    r.SetStart(V(0, 0))
    r.SetEnd(V(70, 50))
    r.SetLayer(pcbnew.Edge_Cuts)
    b.Add(r)
    nets = {}
    for n in ("GND", "+5", "S1", "S2", "S3"):
        ni = pcbnew.NETINFO_ITEM(b, n)
        b.Add(ni)
        nets[n] = ni
    g, p5 = nets["GND"], nets["+5"]
    # +5 power entry in the bottom-left corner, where the small +5 zone is
    _part(b, "J1", 8, 44, [p5, g], smd=False)
    # SMD parts spread over the board: each needs +5 and GND
    for k, (x, y) in enumerate([(25, 10), (55, 10), (40, 25), (60, 40), (20, 30)]):
        sig = nets[("S1", "S2", "S3")[k % 3]]
        _part(b, f"U{k + 1}", x, y, [p5, sig, g])
    _part(b, "J2", 62, 25, [nets["S1"], nets["S2"], nets["S3"]], smd=False)
    _zone(b, g, pcbnew.B_Cu, [(0.5, 0.5), (69.5, 0.5), (69.5, 49.5), (0.5, 49.5)])   # whole board
    _zone(b, p5, pcbnew.F_Cu, [(1, 36), (16, 36), (16, 49), (1, 49)])                 # small corner
    b.BuildConnectivity()
    pcbnew.SaveBoard(path, b)
    return path


@pytest.fixture(scope="module")
def board_path(tmp_path_factory):
    return build(str(tmp_path_factory.mktemp("pour") / "pour.kicad_pcb"))


def test_zone_outline_is_read(board_path):
    b = PcbFile(board_path).board()
    assert b.zone_layers == {"GND": {"B.Cu"}, "+5": {"F.Cu"}}
    (layers, rings), = b.zone_outlines["+5"]
    xs = [x for x, _ in rings[0]]
    assert layers == {"F.Cu"} and max(xs) == 16      # the small corner zone, not the whole layer


def test_pour_nets_are_fully_connected_after_routing(board_path, tmp_path):
    from drc_check import drc
    pcb = PcbFile(board_path)
    s = RouteSettings()
    b = pcb.board()
    res = route(b, s)
    assert res.nets["+5"].complete and res.nets["GND"].complete
    sys.path.insert(0, HERE)
    from geom_check import acute_junctions
    assert acute_junctions(b, res) == []
    assert res.nets["+5"].segments                      # far +5 pads really got tracks
    pcb.add_routes(res, s)
    out = str(tmp_path / "routed.kicad_pcb")
    pcb.save(out)
    kinds, unconnected, text = drc(out)                  # fills both zones first
    copper = {k: v for k, v in kinds.items() if k in ("clearance", "shorting_items", "tracks_crossing",
                                                      "copper_edge_clearance", "hole_clearance")}
    assert copper == {}, kinds
    assert unconnected == 0, text[text.find("unconnected"):][:1500]


def test_route_pour_nets_with_tracks_option(board_path):
    """'Route pour nets with tracks too': every pad joined by copper, pour or not."""
    sys.path.insert(0, HERE)
    from geom_check import connected
    b = PcbFile(board_path).board()
    res = route(b, RouteSettings(skip_zone_nets=False))
    for net in ("+5", "GND"):
        assert res.nets[net].complete and connected(b, res, net, 0.25), net


def test_pad_next_to_its_own_track_is_not_taken_as_connected():
    """A pour net's pad far from its (small) pour, right beside another pad of the net
    that is routed first: it still needs copper of its own (on the attenuator board,
    three +5 pins were left unconnected because +5 copper ran past them)."""
    import os, sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), "examples"))
    from synthetic import rect_outline
    from kicad_autorouter import RouteSettings, route
    from kicad_autorouter.model import Board, Pad
    pads = [Pad("P5", 5, 5, 1.0, 1.0, "rect", 0, {"F.Cu"}, ref="C1.1"),
            Pad("P5", 35, 5, 1.0, 1.0, "rect", 0, {"F.Cu"}, ref="C2.1"),
            Pad("P5", 36.0, 5, 0.4, 0.4, "rect", 0, {"F.Cu"}, ref="U1.16"),   # 0.3 mm beside C2
            Pad("SIG", 5, 20, 1.0, 1.0, "rect", 0, {"F.Cu"}, ref="R1.1"),
            Pad("SIG", 35, 20, 1.0, 1.0, "rect", 0, {"F.Cu"}, ref="R2.1")]
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 40, 25), pads)
    b.zone_nets = {"P5"}
    b.zone_layers = {"P5": {"F.Cu"}}
    b.zone_outlines = {"P5": [({"F.Cu"}, [[(1, 1), (9, 1), (9, 9), (1, 9)]])]}   # round C1 only
    res = route(b, RouteSettings())
    nr = res.nets["P5"]
    assert nr.complete, res.summary()
    # U1.16 must have copper ending on it
    ends = [(x, y) for (x1, y1, x2, y2, _) in nr.segments for (x, y) in ((x1, y1), (x2, y2))]
    assert any(abs(x - 36.0) <= 0.25 and abs(y - 5) <= 0.35 for (x, y) in ends), nr.segments
