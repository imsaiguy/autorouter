"""Board outlines KiCad cannot turn into one clean shape must still keep every track
inside the board (a user's board had a module footprint's outline touching the
board edge; the router fell back to the bounding box and routed outside)."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, os.path.join(ROOT, "examples"), HERE]

from kicad_autorouter import RouteSettings, route  # noqa: E402
from kicad_autorouter.model import Board, Pad  # noqa: E402
from kicad_autorouter.problem import outline_loops, _loop_depths  # noqa: E402
from synthetic import rect_outline  # noqa: E402


def _rects_board(rects, pads):
    outline = []
    for r in rects:
        outline += rect_outline(*r)
    return Board(["F.Cu", "B.Cu"], outline, pads)


def _pads():
    return [Pad("A", 5, 5, 1.7, 1.7, "circle", 0, {"F.Cu", "B.Cu"}, drill=1.0, ref="J1.1"),
            Pad("A", 20, 45, 1.7, 1.7, "circle", 0, {"F.Cu", "B.Cu"}, drill=1.0, ref="J2.1"),
            Pad("B", 35, 5, 1.7, 1.7, "circle", 0, {"F.Cu", "B.Cu"}, drill=1.0, ref="J1.2"),
            Pad("B", 22.5, 41, 1.7, 1.7, "circle", 0, {"F.Cu", "B.Cu"}, drill=1.0, ref="J2.2")]


def _inside_any(x, y, rects, m):
    """(x, y) and the whole circle of radius m round it lie on the board (union of rects)."""
    def on_board(px, py):
        return any(x0 - 1e-6 <= px <= x1 + 1e-6 and y0 - 1e-6 <= py <= y1 + 1e-6 for (x0, y0, x1, y1) in rects)
    return on_board(x, y) and all(on_board(x + m * math.cos(a * math.pi / 8), y + m * math.sin(a * math.pi / 8))
                                  for a in range(16))


@pytest.mark.parametrize("rects", [
    [(0, 0, 40, 30), (15, 30, 25, 50)],          # module outline touching the board edge
    [(0, 0, 40, 30), (15, 25, 25, 50)],          # ... overlapping it
])
def test_tracks_stay_inside_touching_or_overlapping_outlines(rects):
    b = _rects_board(rects, _pads())
    s = RouteSettings()
    res = route(b, s)
    assert all(nr.complete for nr in res.nets.values()), res.summary()
    margin = s.edge_clearance + s.track_width / 2 - 0.05     # grid rounding
    for nr in res.nets.values():
        for (x1, y1, x2, y2, _) in nr.segments:
            for k in range(11):
                x, y = x1 + (x2 - x1) * k / 10, y1 + (y2 - y1) * k / 10
                assert _inside_any(x, y, rects, margin), (nr.name, x, y)


def test_cutout_is_still_a_hole():
    loops = outline_loops(rect_outline(0, 0, 40, 30) + rect_outline(10, 10, 20, 20))
    assert sorted(_loop_depths(loops)) == [0, 1]
    loops = outline_loops(rect_outline(0, 0, 40, 30) + rect_outline(15, 25, 25, 50))
    assert _loop_depths(loops) == [0, 0]                    # overlap: both are board


def test_plugin_reader_uses_drawn_edges_not_bounding_box():
    pcbnew = pytest.importorskip("pcbnew")
    try:
        from kicad_autorouter.plugin import _outline
        import kicad_autorouter.plugin as plugin
    except Exception:
        pytest.skip("wxPython not available in this Python")
    MM = pcbnew.FromMM
    b = pcbnew.BOARD()
    for (x0, y0, x1, y1) in [(0, 0, 40, 30), (15, 30, 25, 50)]:
        r = pcbnew.PCB_SHAPE(b)
        r.SetShape(pcbnew.SHAPE_T_RECT)
        r.SetStart(pcbnew.VECTOR2I(MM(x0), MM(y0)))
        r.SetEnd(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        r.SetLayer(pcbnew.Edge_Cuts)
        b.Add(r)
    segs = _outline(b)
    assert len(segs) == 8 and plugin.OUTLINE_WARNING
    # the old fallback (bounding box) would put (35, 45) on the board
    from kicad_autorouter.problem import outline_loops, _loop_depths, _inside_poly
    loops = outline_loops(segs)
    assert not any(_inside_poly(35, 45, p) for p in loops)
