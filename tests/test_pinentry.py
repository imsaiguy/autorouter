"""Hand-style entries into fine-pitch pins (kicad_autorouter.pinentry)."""
import math
import os
import sys

HERE = os.path.dirname(__file__)
sys.path[:0] = [os.path.dirname(HERE), os.path.join(os.path.dirname(HERE), "examples"), HERE]

from kicad_autorouter.model import Board, Pad, RouteSettings  # noqa: E402
from kicad_autorouter.pinentry import axis_pin, tidy_pin_entries  # noqa: E402
from kicad_autorouter.router import NetRoute, RouteResult  # noqa: E402
from geom_check import clearance_violations  # noqa: E402
from synthetic import rect_outline  # noqa: E402


def _row(n=6, pitch=0.5, y=10.0):
    """A row of 0.3 x 1.5 mm pins, 0.5 mm pitch, long side vertical, nets N0..N5."""
    return [Pad(f"N{i}", 10 + i * pitch, y, 0.3, 1.5, "rect", 0, {"F.Cu"}, ref=f"U1.{i + 1}") for i in range(n)]


def test_axis_pin_shapes():
    assert axis_pin(Pad("A", 0, 0, 0.3, 1.5, "rect", 0, {"F.Cu"}))
    assert not axis_pin(Pad("A", 0, 0, 1.0, 1.0, "rect", 0, {"F.Cu"}))      # square
    assert not axis_pin(Pad("A", 0, 0, 0.8, 2.0, "rect", 0, {"F.Cu"}))      # too wide: not fine pitch


def test_off_centre_entry_is_redrawn_straight_out_of_the_pin():
    pads = _row()
    pads.append(Pad("N2", 12.0, 14.0, 0.8, 0.8, "rect", 0, {"F.Cu"}, ref="R1.1"))
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 30, 20), pads)
    s = RouteSettings(track_width=0.2, clearance=0.15)
    pin = pads[2]                                    # N2 at x = 11.0
    # a grid-made entry: from a point on the pin's end, off its centre line, at an angle
    segs = [(11.08, 10.6, 11.25, 11.15, "F.Cu"), (11.25, 11.15, 11.25, 12.5, "F.Cu"),
            (11.25, 12.5, 12.0, 13.25, "F.Cu"), (12.0, 13.25, 12.0, 14.0, "F.Cu")]
    res = RouteResult({"N2": NetRoute("N2", segs, [], True, 0, [0.2] * 4, [])}, [], True, [], {})
    assert tidy_pin_entries(b, res, s) >= 1            # (the far pad's entry may be redrawn too)
    nr = res.nets["N2"]
    first = [sg for sg in nr.segments if math.hypot(sg[0] - pin.x, sg[1] - pin.y) < 1e-6 or
             math.hypot(sg[2] - pin.x, sg[3] - pin.y) < 1e-6]
    assert len(first) == 1
    sg = first[0]
    assert abs(sg[0] - sg[2]) < 1e-6                 # leaves the pin along its axis (vertical)
    assert clearance_violations(b, res, s) == []


def test_entry_already_clean_is_left_alone():
    pads = _row()
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 30, 20), pads)
    s = RouteSettings(track_width=0.2, clearance=0.15)
    segs = [(11.0, 10.0, 11.0, 12.0, "F.Cu"), (11.0, 12.0, 12.0, 13.0, "F.Cu")]
    res = RouteResult({"N2": NetRoute("N2", list(segs), [], True, 0, [0.2] * 2, [])}, [], True, [], {})
    tidy_pin_entries(b, res, s)
    assert sorted(res.nets["N2"].segments) == sorted(segs)


def test_no_room_keeps_the_original():
    pads = _row()
    # another net's track right where the straight exit would run
    pads.append(Pad("X", 11.0, 11.6, 0.4, 0.4, "rect", 0, {"F.Cu"}, ref="Q1.1"))
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 30, 20), pads)
    s = RouteSettings(track_width=0.2, clearance=0.15)
    segs = [(11.08, 10.6, 11.45, 10.97, "F.Cu"), (11.45, 10.97, 11.45, 13.0, "F.Cu")]
    res = RouteResult({"N2": NetRoute("N2", list(segs), [], True, 0, [0.2] * 2, [])}, [], True, [], {})
    before = clearance_violations(b, res, s)
    tidy_pin_entries(b, res, s)
    assert clearance_violations(b, res, s) == before


def test_kicad_api_pad_polygons_become_rectangles():
    """KiCad's API hands every pad over as a polygon; a (rounded) rectangle must become
    a rectangular pad again, or the fine-pitch pin rules never apply inside KiCad."""
    from kicad_autorouter.ipc_plugin import _rect_like
    assert _rect_like([(0, 0), (1.5, 0), (1.5, 0.55), (0, 0.55)]) == (0.75, 0.275, 1.5, 0.55, "rect")
    r, pts = 0.1, []
    for cx, cy, a0 in ((1.4, 0.1, 270), (1.4, 0.45, 0), (0.1, 0.45, 90), (0.1, 0.1, 180)):
        for k in range(5):
            a = math.radians(a0 + k * 22.5)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    x, y, w, h, shape = _rect_like(pts)
    assert shape == "roundrect" and abs(w - 1.5) < 1e-6 and abs(h - 0.55) < 1e-6
    circle = [(0.5 * math.cos(i * math.pi / 16), 0.5 * math.sin(i * math.pi / 16)) for i in range(32)]
    assert _rect_like(circle) is None                       # round pads keep their outline
    pad = Pad("A", x, y, w, h, shape, 0, {"F.Cu"})
    assert axis_pin(pad)


def test_trace_touching_a_pad_edge_is_run_on_to_the_centre():
    """Pin 44 AVCC: a diagonal ended 0.08 mm outside the pad's end - its round end only
    overlapped the pad.  The last pass runs it on to the pad centre."""
    from kicad_autorouter.pinentry import reach_centres
    pads = _row()
    b = Board(["F.Cu", "B.Cu"], rect_outline(0, 0, 30, 20), pads)
    s = RouteSettings(track_width=0.2, clearance=0.15)
    pin = pads[2]                                    # N2 at (11.0, 10.0), ends at y 9.25 / 10.75
    end = (11.01, 9.25 - 0.08)
    segs = [(9.81, 7.97, end[0], end[1], "F.Cu")]
    res = RouteResult({"N2": NetRoute("N2", segs, [], True, 0, [0.2], [])}, [], True, [], {})
    assert reach_centres(b, res, s) == 1
    nr = res.nets["N2"]
    assert any(math.hypot(sg[2] - pin.x, sg[3] - pin.y) < 1e-9 and
               math.hypot(sg[0] - end[0], sg[1] - end[1]) < 1e-9 for sg in nr.segments)
    assert reach_centres(b, res, s) == 0              # once is enough
