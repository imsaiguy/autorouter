"""The KiCad 9/10 IPC-API front end, driven with real kipy objects."""
import os
import sys
import time

import pytest

kipy = pytest.importorskip("kipy")

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, HERE, os.path.join(ROOT, "examples")]

from fake_kipy_board import FakeKiCadBoard  # noqa: E402
from geom_check import clearance_violations  # noqa: E402

from kicad_autorouter.ipc_plugin import IpcBoard, Session  # noqa: E402
from kicad_autorouter.kicad_pcb import PcbFile  # noqa: E402
from kicad_autorouter.router import NetRoute, RouteResult  # noqa: E402

DEMO = os.path.join(ROOT, "examples", "demo_unrouted.kicad_pcb")


def _to_result(created, s, widths=None):
    """kipy Track/Via objects -> RouteResult (to check them with exact geometry)."""
    from kipy.util.board_layer import canonical_name
    nets = {}
    for it in created:
        nr = nets.setdefault(it.net.name, NetRoute(it.net.name, complete=True))
        if type(it).__name__ == "Via":
            nr.vias.append((it.position.x / 1e6, it.position.y / 1e6))
            assert it.diameter == round(s.via_diameter * 1e6)
        else:
            nr.segments.append((it.start.x / 1e6, it.start.y / 1e6, it.end.x / 1e6, it.end.y / 1e6,
                                canonical_name(it.layer)))
            nr.widths.append(it.width / 1e6)
            if widths is None:
                assert it.width == round(s.track_width * 1e6)
    return RouteResult(nets, [], True, [], {})


def _run(session, opts):
    session.start(opts)
    t0 = time.time()
    while session.running and time.time() - t0 < 120:
        time.sleep(0.05)
    assert not session.running


def test_ipc_reads_board_like_the_file_parser():
    model = PcbFile(DEMO).board()
    ipc = IpcBoard(FakeKiCadBoard(model))
    b = ipc.board
    assert b.copper_layers == ["F.Cu", "B.Cu"]
    assert len(b.pads) == len(model.pads) and len(b.holes) == 2
    assert b.zone_layers == {"GND": {"B.Cu"}}
    ref = {p.ref: p for p in model.pads}
    for p in b.pads:
        m = ref[p.ref]
        assert (p.net, round(p.x, 4), round(p.y, 4), p.layers) == (m.net, round(m.x, 4), round(m.y, 4), m.layers)
        # polygon pad must agree with the analytic shape at its centre and just outside
        assert p.distance(m.x, m.y) == 0
        assert abs(p.distance(m.x + m.radius() + 0.5, m.y) - m.distance(m.x + m.radius() + 0.5, m.y)) < 0.05


def test_ipc_routes_all_and_commits_once():
    model = PcbFile(DEMO).board()
    fake = FakeKiCadBoard(model)
    ipc = IpcBoard(fake)
    session = Session(ipc)
    s = ipc.settings
    _run(session, {"scope": "all", "fill": True})
    log = "\n".join(session.log)
    assert "ERROR" not in log, log
    assert "Routed 12/12 nets completely" in log
    assert [c[0] for c in fake.commits] == ["pushed: Visible Autorouter"]     # exactly one undo step
    assert getattr(fake, "refilled", False)
    res = _to_result(fake.created, s)
    assert clearance_violations(model, res, s) == []
    st = session.state()
    assert st["curve"][-1] == 0 and st["frame"]["paths"]


def test_ipc_routes_only_selection():
    model = PcbFile(DEMO).board()
    fake = FakeKiCadBoard(model, selected_refs=("U1",))
    ipc = IpcBoard(fake)
    assert ipc.selected == {"+3V3", "D1", "D13", "D2", "D7", "GND"}
    _run(Session(ipc), {"scope": "selected"})
    assert {i.net.name for i in fake.created} <= ipc.selected


@pytest.mark.skipif(not os.path.exists(DEMO), reason="demo board missing")
def test_ipc_output_passes_kicad_drc(tmp_path):
    pytest.importorskip("pcbnew")
    from drc_check import drc
    pcb = PcbFile(DEMO)
    fake = FakeKiCadBoard(pcb.board())
    ipc = IpcBoard(fake)
    _run(Session(ipc), {"scope": "all"})
    res = _to_result(fake.created, ipc.settings)
    pcb.add_routes(res, ipc.settings)
    out = str(tmp_path / "ipc_routed.kicad_pcb")
    pcb.save(out)
    kinds, unconnected, _ = drc(out)
    copper = {k: v for k, v in kinds.items() if k in ("clearance", "shorting_items", "tracks_crossing",
                                                      "copper_edge_clearance", "hole_clearance")}
    assert copper == {} and unconnected == 0


# ---------------------------------------------------------------- existing copper
ROUTED = os.path.join(ROOT, "examples", "demo_routed.kicad_pcb")


def _partly_routed_model():
    """The routed demo board with D1 and D2 ripped out completely and half of +3V3 removed."""
    m = PcbFile(ROUTED).board()
    v33 = [t for t in m.tracks if t.net == "+3V3"]
    drop = set(map(id, v33[: len(v33) // 2]))
    m.tracks = [t for t in m.tracks if t.net not in ("D1", "D2") and id(t) not in drop]
    m.vias = [v for v in m.vias if v.net not in ("D1", "D2")]
    return m


def test_ipc_shows_existing_copper_and_net_status():
    model = _partly_routed_model()
    session = Session(IpcBoard(FakeKiCadBoard(model)))
    st = session.static()
    assert len(st["tracks"]) == len(model.tracks) and len(st["vias"]) == len(model.vias)
    assert st["status"]["D1"] == "unrouted" and st["status"]["D2"] == "unrouted"
    assert st["status"]["+3V3"] == "partial" and st["status"]["D7"] == "routed"
    assert set(st["to_finish"]) >= {"D1", "D2", "+3V3"} and "D7" not in st["to_finish"]
    assert "partly routed" in session.status


def _drc_with(model_after, res, settings, tmp_path, name):
    """Write a file holding exactly model_after's copper plus the new items; run KiCad DRC."""
    pytest.importorskip("pcbnew")
    import re
    from drc_check import drc
    pcb = PcbFile(ROUTED)
    keep = {(round(t.x1, 4), round(t.y1, 4), round(t.x2, 4), round(t.y2, 4)) for t in model_after.tracks}
    keepv = {(round(v.x, 4), round(v.y, 4)) for v in model_after.vias}

    def seg_ok(m):
        k = tuple(round(float(m.group(i)), 4) for i in range(1, 5))
        return m.group(0) if k in keep else ""

    def via_ok(m):
        return m.group(0) if (round(float(m.group(1)), 4), round(float(m.group(2)), 4)) in keepv else ""
    text = re.sub(r"\s*\(segment \(start ([-\d.]+) ([-\d.]+)\) \(end ([-\d.]+) ([-\d.]+)\).*?\)\)\n?", seg_ok, pcb.text)
    text = re.sub(r"\s*\(via \(at ([-\d.]+) ([-\d.]+)\).*?\)\)\n?", via_ok, text)
    pcb.text = text
    pcb.add_routes(res, settings)
    out = str(tmp_path / name)
    pcb.save(out)
    kinds, unconnected, _ = drc(out)
    copper = {k: v for k, v in kinds.items() if k in ("clearance", "shorting_items", "tracks_crossing",
                                                      "copper_edge_clearance", "hole_clearance")}
    return copper, unconnected


def test_unrouted_only_keeps_existing_copper(tmp_path):
    model = _partly_routed_model()
    fake = FakeKiCadBoard(model)
    ipc = IpcBoard(fake)
    before = list(fake.existing)
    session = Session(ipc)
    _run(session, {"scope": "unrouted", "rip": True})          # rip must be ignored in this scope
    log = "\n".join(session.log)
    assert "ERROR" not in log, log
    assert fake.removed == [] and fake.existing == before       # nothing existing touched
    touched = {i.net.name for i in fake.created}
    assert touched <= {"D1", "D2", "+3V3", "GND"} and {"D1", "D2", "+3V3"} <= touched
    res = _to_result(fake.created, ipc.settings)
    copper, unconnected = _drc_with(model, res, ipc.settings, tmp_path, "finished.kicad_pcb")
    assert copper == {}
    # the only unconnected items left are the loose pieces the test board was built with
    from kicad_autorouter.problem import loose_copper
    assert unconnected <= sum(map(len, loose_copper(model)))


def test_delete_loose_pieces(tmp_path):
    from kicad_autorouter.problem import loose_copper
    model = _partly_routed_model()
    lt, lv = loose_copper(model)
    n_loose = len(lt) + len(lv)
    assert n_loose > 0
    fake = FakeKiCadBoard(model)
    ipc = IpcBoard(fake)
    assert Session(ipc).static()["loose"] == n_loose
    session = Session(ipc)
    _run(session, {"scope": "unrouted", "remove_loose": True})
    log = "\n".join(session.log)
    assert f"Deleting {n_loose} loose" in log, log
    assert len(fake.removed) == n_loose
    assert len(fake.commits) == 1                                   # still one undo step
    after = _partly_routed_model()
    gone = {(round(v.x, 4), round(v.y, 4)) for v in lv}
    after.vias = [v for v in after.vias if (round(v.x, 4), round(v.y, 4)) not in gone]
    copper, unconnected = _drc_with(after, _to_result(fake.created, ipc.settings), ipc.settings,
                                    tmp_path, "clean.kicad_pcb")
    assert copper == {} and unconnected == 0


def test_unrouted_only_with_nothing_to_do():
    model = PcbFile(ROUTED).board()
    fake = FakeKiCadBoard(model)
    session = Session(IpcBoard(fake))
    _run(session, {"scope": "unrouted"})
    assert fake.created == [] and fake.commits == []
    assert "Nothing to route" in session.status


def test_ipc_uses_net_class_widths():
    from kipy.project_types import NetClass
    model = PcbFile(DEMO).board()
    fake = FakeKiCadBoard(model)
    from kipy.proto.common.types import project_settings_pb2
    default, power = NetClass(project_settings_pb2.NetClass()), NetClass(project_settings_pb2.NetClass())
    default._proto.name, power._proto.name = "Default", "power"
    default.track_width, default.clearance = 250000, 200000
    power.track_width, power.clearance = 609600, 200000
    nets = sorted({p.net for p in model.pads if p.net and p.net != "GND"})
    wide = nets[0]
    fake.netclasses = {n: (power if n == wide else default) for n in nets}
    ipc = IpcBoard(fake)
    s = ipc.settings
    assert s.track_width == 0.25 and s.net_rules == {wide: (0.6096, 0.2)}
    _run(Session(ipc), {"scope": "all"})
    res = _to_result(fake.created, s, widths=True)
    assert 0.6096 in res.nets[wide].widths
    assert all(w == 0.25 for n, nr in res.nets.items() if n != wide for w in nr.widths)
    assert clearance_violations(model, res, s) == []
