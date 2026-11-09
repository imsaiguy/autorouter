"""Net classes: a net in a "power" class (24 mil = 0.6096 mm track, 0.3 mm clearance)
must be routed at that width, necking down only at its own pads, and the result must
pass KiCad's DRC with the project's net-class rules."""
import json
import os
import shutil
import sys

import pytest

pcbnew = pytest.importorskip("pcbnew")

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, os.path.join(ROOT, "examples"), HERE]

from kicad_autorouter import RouteSettings, route  # noqa: E402
from kicad_autorouter.kicad_pcb import PcbFile  # noqa: E402
from test_pours import _part  # noqa: E402

MM = pcbnew.FromMM
POWER_W, POWER_C = 0.6096, 0.3


def V(x, y):
    return pcbnew.VECTOR2I(MM(x), MM(y))


def build(path):
    b = pcbnew.BOARD()
    r = pcbnew.PCB_SHAPE(b)
    r.SetShape(pcbnew.SHAPE_T_RECT)
    r.SetStart(V(0, 0))
    r.SetEnd(V(60, 40))
    r.SetLayer(pcbnew.Edge_Cuts)
    b.Add(r)
    nets = {}
    for n in ("GND", "+5", "VBAT", "S1", "S2", "S3"):
        ni = pcbnew.NETINFO_ITEM(b, n)
        b.Add(ni)
        nets[n] = ni
    g, p5 = nets["GND"], nets["+5"]
    _part(b, "J1", 6, 34, [p5, g, nets["VBAT"]], smd=False)
    for k, (x, y) in enumerate([(20, 8), (45, 8), (32, 20), (50, 32), (18, 26)]):
        sig = nets[("S1", "S2", "S3")[k % 3]]
        _part(b, f"U{k + 1}", x, y, [p5, sig, g])
    _part(b, "J2", 52, 20, [nets["S1"], nets["S2"], nets["S3"], nets["VBAT"]], smd=False)
    b.BuildConnectivity()
    pcbnew.SaveBoard(path, b)
    pro = os.path.splitext(path)[0] + ".kicad_pro"
    data = json.load(open(pro))
    ns = data["net_settings"]
    default = next(c for c in ns["classes"] if c["name"] == "Default")
    default.update(track_width=0.25, clearance=0.2)
    power = dict(default, name="power", track_width=POWER_W, clearance=POWER_C)
    ns["classes"].append(power)
    ns["netclass_patterns"] = [{"netclass": "power", "pattern": "+5"},
                               {"netclass": "power", "pattern": "VB*"}]
    json.dump(data, open(pro, "w"), indent=2)
    return path


@pytest.fixture(scope="module")
def board_path(tmp_path_factory):
    return build(str(tmp_path_factory.mktemp("nc") / "nc.kicad_pcb"))


def test_project_net_classes_are_read(board_path):
    s = PcbFile(board_path).settings_from_project()
    assert s.track_width == 0.25 and s.clearance == 0.2
    assert s.net_rules == {"+5": (POWER_W, POWER_C), "VBAT": (POWER_W, POWER_C)}


def test_plugin_reads_net_classes(board_path, tmp_path):
    # a fresh path: pcbnew keeps the project it saved with the board (before the classes)
    for ext in (".kicad_pcb", ".kicad_pro"):
        shutil.copy(os.path.splitext(board_path)[0] + ext, str(tmp_path / ("copy" + ext)))
    board_path = str(tmp_path / "copy.kicad_pcb")
    try:
        from kicad_autorouter.plugin import settings_from_pcbnew
    except Exception:
        pytest.skip("wxPython not available in this Python")
    s = settings_from_pcbnew(pcbnew.LoadBoard(board_path))
    assert s.net_rules.get("+5") == pytest.approx((POWER_W, POWER_C))
    assert "S1" not in s.net_rules


def test_power_net_is_routed_wide_and_passes_drc(board_path, tmp_path):
    from drc_check import drc
    pcb = PcbFile(board_path)
    s = pcb.settings_from_project()
    s.skip_zone_nets = False
    res = route(pcb.board(), s)
    for net in ("+5", "VBAT", "GND", "S1", "S2", "S3"):
        assert res.nets[net].complete, (net, res.summary())
    p5 = res.nets["+5"]
    import math
    wide_len = sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1, x2, y2, _), w in zip(p5.segments, p5.widths)
                   if abs(w - POWER_W) < 1e-9)
    all_len = sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1, x2, y2, _) in p5.segments)
    assert wide_len > 0.95 * all_len, (wide_len, all_len)          # constant width (pads have room)
    assert set(res.nets["S1"].widths) == {0.25}
    pcb.add_routes(res, s)
    out = str(tmp_path / "routed.kicad_pcb")
    pcb.save(out)
    shutil.copy(os.path.splitext(board_path)[0] + ".kicad_pro", str(tmp_path / "routed.kicad_pro"))
    kb = pcbnew.LoadBoard(out)
    widths = {round(pcbnew.ToMM(t.GetWidth()), 4) for t in kb.GetTracks()
              if t.GetNetname() == "+5" and t.GetClass() == "PCB_TRACK"}
    assert round(POWER_W, 4) in widths
    kinds, unconnected, text = drc(out)
    # the test footprints' own pads sit 0.2 mm apart, closer than the power class allows:
    # only violations that involve routed copper count
    routed = [blk for blk in text.split("\n[")[1:]
              if blk.split("]")[0] in ("clearance", "shorting_items", "tracks_crossing",
                                       "copper_edge_clearance", "hole_clearance")
              and ("Track" in blk or "Via" in blk)]
    assert routed == [], "\n[".join(routed)
    assert set(kinds) - {"clearance", "silk_over_copper", "lib_footprint_issues"} == set(), kinds
    assert unconnected == 0, text[text.find("unconnected"):][:1500]


def test_dense_board_with_power_class_is_legal():
    """Wide nets threading between SOIC pins and other tracks: exact-geometry clearance
    check with per-segment widths and per-class clearances."""
    from synthetic import dense_board
    from geom_check import acute_junctions, clearance_violations, connected
    b = dense_board()
    power = ["S0", "S1", "S2", "S3", "S4"]
    s = RouteSettings(net_rules={n: (POWER_W, POWER_C) for n in power})
    res = route(b, s)
    bad = clearance_violations(b, res, s)
    assert bad == [], bad[:10]
    assert acute_junctions(b, res) == []
    for n in power:
        nr = res.nets[n]
        if nr.complete:
            assert POWER_W in nr.widths, n
            assert connected(b, res, n, 0.25), n
    assert sum(res.nets[n].complete for n in power) >= 4, res.summary()
