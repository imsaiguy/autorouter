import math
import os
import sys

import pytest

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "examples"))
sys.path.insert(0, HERE)

from geom_check import acute_junctions, clearance_violations, connected, dangling_ends, sharp_corners  # noqa: E402
from synthetic import crossing_board, mixed_board, rect_outline  # noqa: E402

from kicad_autorouter import Board, Pad, RouteSettings, route  # noqa: E402
from kicad_autorouter.kicad_pcb import PcbFile, parse_sexpr  # noqa: E402
from kicad_autorouter.problem import build_problem  # noqa: E402
from kicad_autorouter.router import Router  # noqa: E402

DEMO = os.path.join(ROOT, "examples", "demo_unrouted.kicad_pcb")


# ------------------------------------------------------------------ geometry
def test_pad_distance_rotated_rect():
    p = Pad("A", 10, 10, 2.0, 1.0, "rect", 90, {"F.Cu"})
    # rotated 90 deg: now 1 wide (x) and 2 tall (y)
    assert p.distance(10, 10.9) == 0
    assert p.distance(10.9, 10) == pytest.approx(0.4)


def test_pad_distance_circle_and_oval():
    c = Pad("A", 0, 0, 2, 2, "circle")
    assert c.distance(3, 0) == pytest.approx(2)
    o = Pad("A", 0, 0, 4, 2, "oval")
    assert o.distance(2, 0) == 0
    assert o.distance(0, 2) == pytest.approx(1)
    assert o.distance(4, 0) == pytest.approx(2)


def test_parse_sexpr():
    t = parse_sexpr('(a (b 1 2) "x y" (c "q\\"z"))')
    assert t[0] == "a" and t[1] == ["b", "1", "2"] and t[2] == "x y" and t[3][1] == 'q"z'


# ------------------------------------------------------------------ routing
@pytest.mark.parametrize("make", [lambda: crossing_board(8, 1), lambda: crossing_board(10, 2), mixed_board])
def test_routes_are_legal_and_connected(make):
    b = make()
    s = RouteSettings()
    res = route(b, s)
    assert res.converged
    assert clearance_violations(b, res, s) == []
    for name, nr in res.nets.items():
        assert nr.complete
        assert connected(b, res, name, s.track_width), name
    assert dangling_ends(b, res) == []          # no dead-end stubs
    assert sharp_corners(b, res) == []          # every bend is 45 degrees
    assert acute_junctions(b, res) == []        # branches join at 90 degrees or wider


def test_loose_copper_is_not_routed_to():
    """An unfinished trace touching no pad must not become a routing target (it only
    grows a dead-end stub); the net is routed pad-to-pad and the piece is reported."""
    from kicad_autorouter.model import Track
    b = crossing_board(8, 1)
    b.tracks.append(Track("N0", 15, 2.0, 25, 2.0, 0.25, "F.Cu"))      # loose piece of N0
    res = route(b, RouteSettings())
    assert res.nets["N0"].complete
    assert "loose" in res.notes.get("N0", "")
    assert dangling_ends(b, res) == []


def test_straightening_reduces_jogs():
    from kicad_autorouter.problem import build_problem
    from kicad_autorouter.router import Router
    b = mixed_board()
    pb = build_problem(b, RouteSettings())
    r = Router(pb)
    r.run()          # includes the straightening pass
    raw_bends = 0
    for j in pb.nets:
        for p in j.paths:
            pts = [pb.grid.xyl(c) for c in p]
            dirs = [(b2[0] - a[0], b2[1] - a[1]) for a, b2 in zip(pts, pts[1:]) if a[2] == b2[2]]
            raw_bends += sum(1 for d0, d1 in zip(dirs, dirs[1:]) if d0 != d1)
    segs = sum(len(n.segments) for n in r._result(True, []).nets.values())
    assert raw_bends < 60 and segs < 90


def test_route_only_selected_nets():
    b = crossing_board(8, 1)
    res = route(b, RouteSettings(), nets=["N0", "N3"])
    assert set(res.nets) == {"N0", "N3"}


def test_existing_tracks_are_respected():
    """Route two nets, then route the rest around them: still legal."""
    b = crossing_board(8, 1)
    s = RouteSettings()
    first = route(b, s, nets=["N2", "N5"])
    from kicad_autorouter.model import Track, Via
    for n, nr in first.nets.items():
        for (x1, y1, x2, y2, l) in nr.segments:
            b.tracks.append(Track(n, x1, y1, x2, y2, s.track_width, l))
        for (x, y) in nr.vias:
            b.vias.append(Via(n, x, y, s.via_diameter))
    second = route(b, s)
    assert second.skipped.get("N2") == "already complete"
    merged = type(second)({**first.nets, **{k: v for k, v in second.nets.items()}},
                          second.passes, True, [], {})
    assert clearance_violations(b, merged, s) == []


def test_blocked_net_fails_cleanly():
    """A pad boxed in by another net's pads cannot be reached: reported, not crashed."""
    pads = [Pad("A", 5, 5, 1, 1, "rect", 0, {"F.Cu"}), Pad("A", 15, 5, 1, 1, "rect", 0, {"F.Cu"})]
    for k in range(8):
        a = 2 * math.pi * k / 8
        pads.append(Pad("W", 15 + 1.2 * math.cos(a), 5 + 1.2 * math.sin(a), 1.0, 1.0, "circle", 0, {"F.Cu"}))
    b = Board(["F.Cu"], rect_outline(0, 0, 20, 10), pads)
    res = route(b, RouteSettings(max_passes=5), nets=["A"])
    assert not res.nets["A"].complete


def test_events_and_cancel():
    b = crossing_board(8, 1)
    seen = []

    def cb(ev, d):
        seen.append(ev)
        return not (ev == "pass_end")      # stop after the first pass
    pb = build_problem(b, RouteSettings())
    res = Router(pb, cb).run()
    assert res.cancelled and len(res.passes) == 1
    assert seen[0] in ("global_done", "pass_start") and "pass_start" in seen
    assert "connection" in seen and seen[-1] == "done"


def test_multipass_improves():
    """Pass 1 overlaps; later passes negotiate the overuse down to zero."""
    res = route(crossing_board(10, 2), RouteSettings())
    over = [p.overused for p in res.passes]
    assert over[0] > 0 and over[-1] == 0 and len(over) > 1


# ------------------------------------------------------------------ kicad files
@pytest.mark.skipif(not os.path.exists(DEMO), reason="demo board missing")
def test_kicad_file_roundtrip(tmp_path):
    pcb = PcbFile(DEMO)
    board = pcb.board()
    assert len(board.pads) == 60 and board.zone_layers == {"GND": {"B.Cu"}} and len(board.holes) == 2
    s = pcb.settings_from_project()
    res = route(board, s)
    assert all(nr.complete for nr in res.nets.values())
    n = pcb.add_routes(res, s)
    out = tmp_path / "routed.kicad_pcb"
    pcb.save(str(out))
    again = PcbFile(str(out)).board()
    assert len(again.tracks) + len(again.vias) == n
    assert clearance_violations(board, res, s) == []


@pytest.mark.skipif(not os.path.exists(DEMO), reason="demo board missing")
def test_kicad_drc_clean(tmp_path):
    pcbnew = pytest.importorskip("pcbnew")
    from drc_check import drc
    pcb = PcbFile(DEMO)
    s = pcb.settings_from_project()
    res = route(pcb.board(), s)
    pcb.add_routes(res, s)
    out = str(tmp_path / "routed.kicad_pcb")
    pcb.save(out)
    kinds, unconnected, _ = drc(out)
    copper = {k: v for k, v in kinds.items()
              if k in ("clearance", "shorting_items", "tracks_crossing", "copper_edge_clearance",
                       "hole_clearance", "hole_to_hole", "track_dangling", "via_dangling")}
    assert copper == {}
    assert unconnected == 0


def test_progress_estimate_is_monotonic_and_finishes():
    from kicad_autorouter.progress import ProgressEstimator
    pe = ProgressEstimator(RouteSettings().max_passes)
    seen = []

    def cb(ev, d):
        pe.feed(ev, d)
        sn = pe.snapshot()
        seen.append(sn["fraction"])
        assert sn["headline"] and 0.0 <= sn["fraction"] <= 1.0
    route(mixed_board(), RouteSettings(), callback=cb)
    assert all(b >= a for a, b in zip(seen, seen[1:]))     # the bar never goes backwards
    assert seen[-1] == 1.0 and pe.snapshot()["headline"].startswith("Done")


def test_global_routing_on_and_off_both_legal():
    """The coarse global stage only narrows where the detailed search looks:
    with it on or off the result must be complete, legal and connected."""
    from kicad_autorouter.global_route import GlobalRouter
    from kicad_autorouter.problem import build_problem
    from kicad_autorouter.router import Router
    for on in (True, False):
        b = mixed_board()
        s = RouteSettings(global_routing=on)
        events = []
        res = Router(build_problem(b, s), lambda ev, d: events.append(ev)).run()
        assert all(nr.complete for nr in res.nets.values())
        assert clearance_violations(b, res, s) == [] and dangling_ends(b, res) == []
        for name in res.nets:
            assert connected(b, res, name, s.track_width), name
        assert ("global_done" in events) == on
    # a corridor always contains the net's own pins
    pb = build_problem(mixed_board(), RouteSettings())
    gr = GlobalRouter(pb, 6)
    gr.run()
    for j in pb.nets:
        mask = gr.corridor_mask(j, 1)
        for t in j.terminals:
            assert all(mask[c % pb.grid.plane] for c in t.cells)


def test_parallel_first_pass_gives_a_legal_complete_result():
    """Pass 1 on worker processes (forced on, even for a small board)."""
    import os
    b = mixed_board()
    s = RouteSettings(workers=2)
    seen = []
    had_flag = "KICAD_AUTOROUTER_NO_PLUGIN" in os.environ
    res = route(b, s, callback=lambda ev, d: seen.append(d) if ev == "workers" else None)
    assert seen and seen[0]["workers"] == 2 and not seen[0].get("error"), seen
    # the workers' "do not register the toolbar button" flag must not stay in this process
    # (inside KiCad, the button then vanished when the PCB editor was reopened)
    assert ("KICAD_AUTOROUTER_NO_PLUGIN" in os.environ) == had_flag
    assert res.complete_nets == len(res.nets), res.summary()
    assert clearance_violations(b, res, s) == []
    assert acute_junctions(b, res) == []


def test_failed_net_says_what_walls_it_in():
    """A pad boxed in by another net's existing track: the summary names the pad and
    the copper, instead of just 'Incomplete'."""
    from kicad_autorouter.model import Pad, Track
    pads = [Pad("SIG", 10, 10, 1.0, 1.0, "rect", 0, {"F.Cu"}, ref="U1.3"),
            Pad("SIG", 25, 10, 1.0, 1.0, "rect", 0, {"F.Cu"}, ref="C54.1"),
            Pad("PWR", 5, 5, 1.0, 1.0, "rect", 0, {"F.Cu"}, ref="J1.1"),
            Pad("PWR", 5, 15, 1.0, 1.0, "rect", 0, {"F.Cu"}, ref="J1.2")]
    ring = [(8, 8, 12, 8), (12, 8, 12, 12), (12, 12, 8, 12), (8, 12, 8, 8), (5, 5, 8, 8), (8, 12, 5, 15)]
    b = Board(["F.Cu"], rect_outline(0, 0, 30, 20), pads, [Track("PWR", *r, 0.25, "F.Cu") for r in ring])
    res = route(b, RouteSettings(), nets=["SIG"])
    assert not res.nets["SIG"].complete
    why = "\n".join(res.nets["SIG"].why)
    assert "U1.3" in why and "PWR" in why and "walled in" in why, why
    assert "walled in" in res.summary()


def test_compiled_search_gives_the_same_routes_as_python():
    """The compiled core (native/astar.c) must be a drop-in: identical routes."""
    import os
    import kicad_autorouter.native as nat
    nat._TRIED, nat._LIB = False, None
    if nat.load() is None:
        pytest.skip("no compiled core for this platform")
    out = {}
    for mode in ("python", "native"):
        nat._TRIED, nat._LIB = False, None
        if mode == "python":
            os.environ["AUTOROUTER_NO_NATIVE"] = "1"
        try:
            res = route(mixed_board(), RouteSettings(workers=1))
        finally:
            os.environ.pop("AUTOROUTER_NO_NATIVE", None)
        out[mode] = sorted((n, tuple(round(v, 4) for v in sg[:4]), sg[4])
                           for n, nr in res.nets.items() for sg in nr.segments)
    nat._TRIED, nat._LIB = False, None
    assert out["python"] == out["native"]
