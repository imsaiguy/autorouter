"""Routing and straightening on a board that already has copper: locked and kept copper,
grids finer than width + clearance, hole-to-hole distance, keep-out corners, where a
route may join existing copper, and what the file writer leaves alone."""
import json
import math
import os
import shutil
import sys

import pytest

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "examples"))
sys.path.insert(0, HERE)

from geom_check import clearance_violations, connected  # noqa: E402
from synthetic import crossing_board, rect_outline  # noqa: E402

from kicad_autorouter import Board, Pad, RouteSettings, route  # noqa: E402
from kicad_autorouter.kicad_pcb import PcbFile  # noqa: E402
from kicad_autorouter.model import Track, Via  # noqa: E402
from kicad_autorouter.problem import BLOCKED, build_problem  # noqa: E402
from kicad_autorouter.restraighten import straighten_existing  # noqa: E402
from kicad_autorouter.router import NetRoute, RouteResult  # noqa: E402

DEMO = os.path.join(ROOT, "examples", "demo_unrouted.kicad_pcb")


def grid_of(s, w=40.0, h=30.0):
    return build_problem(Board(["F.Cu", "B.Cu"], rect_outline(0, 0, w, h), [tht("X", 2, 2), tht("X", 4, 4)]), s).grid


def tht(net, x, y, ref=""):
    return Pad(net, x, y, 1.2, 1.2, "circle", 0, {"F.Cu", "B.Cu"}, drill=0.6, ref=ref or net)


def detour_board(s, width=None, locked=False, other=()):
    """Net A: two pads on one grid row, joined by an on-grid track that goes round the
    middle of the row. Returns (board, the cells' centres along the row)."""
    g = grid_of(s)
    j = 22
    P = lambda i, dj=0: g.center(i, j + dj)
    pts = [P(12), P(30), P(30, 8), P(60, 8), P(60), P(78)]
    w = width or s.track_width
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 40, 30), [tht("A", *P(12), "J1"), tht("A", *P(78), "J2")])
    b.tracks = [Track("A", *p, *q, w, "F.Cu", locked) for p, q in zip(pts, pts[1:])]
    b.tracks += list(other)
    return b, P


def as_result(board, res, replaced):
    """What is on the board after a straightening: the new copper, and the old minus what it replaced."""
    gone = {id(x) for x in replaced}
    out = RouteResult(dict(res.nets), [], True, [], {})
    for t in board.tracks:
        if id(t) not in gone:
            nr = out.nets.setdefault(t.net, NetRoute(t.net, complete=True))
            nr.segments.append((t.x1, t.y1, t.x2, t.y2, t.layer))
            nr.widths.append(t.width)
    return out


# ------------------------------------------------------------------ straightening
def test_unlocked_detour_is_straightened():
    s = RouteSettings()
    b, _ = detour_board(s)
    res, replaced, _ = straighten_existing(b, s)
    assert len(replaced) == len(b.tracks)
    assert len(res.nets["A"].segments) < len(b.tracks)
    assert connected(b, res, "A", s.track_width)


def test_locked_copper_is_not_reshaped():
    s = RouteSettings()
    b, _ = detour_board(s, locked=True)
    res, replaced, _ = straighten_existing(b, s)
    assert replaced == []
    assert not res.nets.get("A") or not res.nets["A"].segments


def test_copper_of_another_width_is_not_reshaped():
    """Redrawn at the router's own width it would lose the width it was given."""
    s = RouteSettings()
    b, _ = detour_board(s, width=0.4)
    _, replaced, _ = straighten_existing(b, s)
    assert replaced == []


def test_a_via_of_another_size_is_not_reshaped():
    s = RouteSettings()
    b, P = detour_board(s)
    b.vias.append(Via("A", *P(30), 0.9, 0.4))
    _, replaced, _ = straighten_existing(b, s)
    assert not any(isinstance(x, Via) for x in replaced)


def test_kept_copper_is_an_obstacle_for_the_other_nets():
    """Net B's track is off the grid, so it stays as it is. It crosses the row that net
    A's pads stand on: A's detour may be tidied, but not straight through B."""
    s = RouteSettings()
    g = grid_of(s)
    x = g.center(45, 0)[0] + 0.07
    y = g.center(0, 22)[1]
    cross = Track("B", x, y - 1.4, x, y + 1.4, s.track_width, "F.Cu")
    b, _ = detour_board(s, other=[cross])
    b.pads += [Pad("B", x, y - 1.4, 0.3, 0.3, "rect", 0, {"F.Cu"}, ref="R1.1"),
               Pad("B", x, y + 1.4, 0.3, 0.3, "rect", 0, {"F.Cu"}, ref="R1.2")]
    res, replaced, _ = straighten_existing(b, s)
    assert not any(t is cross for t in replaced)
    after = as_result(b, res, replaced)
    kept = Board(b.copper_layers, b.outline, b.pads)
    assert clearance_violations(kept, after, s) == []
    assert connected(kept, after, "A", s.track_width)


# ------------------------------------------------------------------ fine grid
@pytest.mark.parametrize("pitch", [0.15, 0.225])
def test_a_grid_finer_than_width_plus_clearance_is_legal(pitch):
    b = crossing_board(6, 1)
    s = RouteSettings(grid=pitch, max_passes=80, stall_passes=40)
    res = route(b, s)
    assert clearance_violations(b, res, s) == []
    for name, nr in res.nets.items():
        if nr.complete:
            assert connected(b, res, name, s.track_width), name
    assert sum(nr.complete for nr in res.nets.values()) >= 5


# ------------------------------------------------------------------ holes
def test_no_via_within_the_hole_to_hole_distance_of_an_existing_via():
    s = RouteSettings(hole_to_hole=0.5)
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 30, 20),
              [tht("A", 3, 3), tht("A", 27, 17), tht("B", 3, 17), tht("B", 27, 3)])
    b.vias.append(Via("A", 15.07, 10.03, 0.6, 0.0))
    pb = build_problem(b, s)
    g = pb.grid
    reach = s.via_drill + s.hole_to_hole
    near = [(x, y) for y in range(g.ny) for x in range(g.nx)
            if 1e-3 < math.hypot(g.center(x, y)[0] - 15.07, g.center(x, y)[1] - 10.03) < reach]
    assert near
    assert all(pb.via_owner[y * g.nx + x] == BLOCKED for x, y in near)


def test_no_via_within_the_hole_to_hole_distance_of_an_unplated_hole():
    s = RouteSettings(hole_to_hole=0.5)
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 30, 20), [tht("A", 3, 3), tht("A", 27, 17)])
    b.holes.append((15.0, 10.0, 0.65))
    pb = build_problem(b, s)
    g = pb.grid
    reach = 0.65 / 2 + s.via_drill / 2 + s.hole_to_hole
    near = [(x, y) for y in range(g.ny) for x in range(g.nx)
            if math.hypot(g.center(x, y)[0] - 15.0, g.center(x, y)[1] - 10.0) < reach]
    assert all(pb.via_owner[y * g.nx + x] == BLOCKED for x, y in near)


def test_hole_to_hole_rule_is_read_from_the_project(tmp_path):
    pcb = tmp_path / "b.kicad_pcb"
    shutil.copy(DEMO, pcb)
    (tmp_path / "b.kicad_pro").write_text(json.dumps({
        "board": {"design_settings": {"rules": {"min_hole_to_hole": 0.5}}},
        "net_settings": {"classes": [{"name": "Default", "track_width": 0.25, "clearance": 0.2,
                                      "via_diameter": 0.6, "via_drill": 0.3}]}}))
    assert PcbFile(str(pcb)).settings_from_project().hole_to_hole == 0.5


# ------------------------------------------------------------------ keep-outs
def test_no_track_centre_line_near_a_keep_out_corner():
    """A 45-degree step between two cell centres passes closer to a corner than either
    centre does, so the band is grown like every other clearance halo."""
    s = RouteSettings()
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 30, 20), [tht("A", 3, 3), tht("A", 27, 17)])
    ring = [(10.1, 6.1), (20.1, 6.1), (20.1, 14.1), (10.1, 14.1)]
    b.keepouts.append(({"F.Cu"}, [ring], True, False))
    pb = build_problem(b, s)
    g = pb.grid
    band = math.hypot(s.track_width / 2, g.pitch / 2)
    for y in range(g.ny):
        for x in range(g.nx):
            cx, cy = g.center(x, y)
            d = math.hypot(max(10.1 - cx, 0, cx - 20.1), max(6.1 - cy, 0, cy - 14.1))
            if d < band - 1e-6:
                assert pb.owner[g.idx(x, y, 0)] == BLOCKED, (cx, cy, d)


# ------------------------------------------------------------------ joining existing copper
def test_a_route_does_not_join_existing_copper_beside_another_net():
    """A narrow track of net A leaves its pad close past a pad of net B: a full-width
    track centred there would be too close to B, so those points are no place to join."""
    s = RouteSettings(grid=0.15)
    g = grid_of(s)
    y = g.center(0, 60)[1]
    x0, x1 = g.center(50, 0)[0], g.center(110, 0)[0]
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 40, 30), [
        Pad("A", x0, y, 0.3, 0.2, "rect", 0, {"F.Cu"}, ref="U1.1"), tht("A", 35, 25, "J1"),
        Pad("B", (x0 + x1) / 2 - 2, y + 0.4, 6.0, 0.2, "rect", 0, {"F.Cu"}, ref="U1.2"), tht("B", 5, 25, "J2")])
    b.tracks.append(Track("A", x0, y, x1, y, 0.15, "F.Cu"))
    pb = build_problem(b, s)
    job = next(j for j in pb.nets if j.name == "A")
    beside = b.pads[2]
    need = s.clearance + s.track_width / 2
    cells = [c for t in job.terminals for c in t.cells if g.xyl(c)[2] == 0]
    assert cells
    for c in cells:
        cx, cy = g.center(*g.xyl(c)[:2])
        assert beside.distance(cx, cy) >= need - 1e-6, (cx, cy)


# ------------------------------------------------------------------ the file
EXTRA = """  (segment (start 20 20) (end 22 20) (width 0.25) (layer "F.Cu") (net 1) (tstamp 00000000-0000-0000-0000-000000000001))
  (segment (start 22 20) (end 24 20) (width 0.4) (locked) (layer "F.Cu") (net 1) (tstamp 00000000-0000-0000-0000-000000000002))
  (segment (start 24 20) (end 26 20) (width 0.25) (locked yes) (layer "F.Cu") (net 1) (tstamp 00000000-0000-0000-0000-000000000003))
  (segment (start 26 20) (end 28 20) (width 0.25) (locked no) (layer "F.Cu") (net 1) (tstamp 00000000-0000-0000-0000-000000000004))
  (via locked (at 28 20) (size 0.9) (drill 0.4) (layers "F.Cu" "B.Cu") (net 1) (tstamp 00000000-0000-0000-0000-000000000005))
"""


def with_copper(tmp_path):
    text = open(DEMO, encoding="utf-8").read().rstrip()
    assert text.endswith(")")
    p = tmp_path / "c.kicad_pcb"
    p.write_text(text[:-1].rstrip() + "\n" + EXTRA + ")\n", encoding="utf-8")
    return PcbFile(str(p))


def test_locked_is_read_from_the_file(tmp_path):
    board = with_copper(tmp_path).board()
    by_x = {round(t.x1): t.locked for t in board.tracks}
    assert by_x == {20: False, 22: True, 24: True, 26: False}
    assert [v.locked for v in board.vias] == [True]


def test_only_the_named_items_leave_the_file(tmp_path):
    pcb = with_copper(tmp_path)
    board = pcb.board()
    first = next(t for t in board.tracks if round(t.x1) == 20)
    assert pcb.remove_items([first], []) == 1
    assert "(start 20 20)" not in pcb.text
    for kept in ("(start 22 20) (end 24 20) (width 0.4) (locked)", "(locked yes)", "(via locked (at 28 20) (size 0.9)"):
        assert kept in pcb.text
    assert pcb.remove_items([], list(board.vias)) == 1 and "(at 28 20)" not in pcb.text


def test_no_via_is_written_on_an_existing_via_of_its_net(tmp_path):
    pcb = with_copper(tmp_path)
    board = pcb.board()
    s = RouteSettings()
    net = board.vias[0].net
    res = RouteResult({net: NetRoute(net, [], [(28.0, 20.0), (30.0, 20.0)], True)}, [], True, [], {})
    assert pcb.add_routes(res, s) == 1
    assert pcb.text.count("(at 28") == 1 and "(at 30.0000 20.0000)" in pcb.text


def test_the_same_routing_gives_the_same_file(tmp_path):
    texts = []
    for k in range(2):
        d = tmp_path / str(k)
        d.mkdir()
        pcb = PcbFile(shutil.copy(DEMO, d / "b.kicad_pcb"))
        s = pcb.settings_from_project()
        pcb.add_routes(route(pcb.board(), s), s)
        texts.append(pcb.text)
    assert texts[0] == texts[1]
