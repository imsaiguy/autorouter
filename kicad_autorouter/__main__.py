"""Command line: route a .kicad_pcb file.

    python -m kicad_autorouter board.kicad_pcb -o routed.kicad_pcb
    python -m kicad_autorouter board.kicad_pcb --nets SDA,SCL --gif thinking.gif
    python -m kicad_autorouter board.kicad_pcb --live          # watch it in a window
"""
from __future__ import annotations

import argparse
import sys
import time

from .kicad_pcb import PcbFile
from .model import RouteSettings
from .problem import build_problem
from .router import Router


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="kicad_autorouter", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("board", help="input .kicad_pcb")
    ap.add_argument("-o", "--output", help="output .kicad_pcb (default: <board>-routed.kicad_pcb)")
    ap.add_argument("--nets", help="comma-separated nets to route (default: all)")
    ap.add_argument("--include-zone-nets", action="store_true",
                    help="also route nets that have a copper pour (GND etc.)")
    ap.add_argument("--tracks-on-planes", action="store_true",
                    help="also route signals on inner layers covered by pours (default: keep planes solid)")
    ap.add_argument("--track-width", type=float)
    ap.add_argument("--clearance", type=float)
    ap.add_argument("--via-diameter", type=float)
    ap.add_argument("--via-drill", type=float)
    ap.add_argument("--grid", type=float, help="grid pitch in mm (default: width + clearance)")
    ap.add_argument("--layers", help="comma-separated copper layers to use (default: all)")
    ap.add_argument("--passes", type=int, help="maximum passes (default 40)")
    ap.add_argument("--via-cost", type=float)
    ap.add_argument("--no-global", action="store_true",
                    help="skip the coarse global-routing stage (slower on big boards)")
    ap.add_argument("--workers", type=int,
                    help="CPU cores for the first pass (default: automatic; 1 = single core)")
    ap.add_argument("--region-cells", type=int, help="global-routing region size in grid cells (default 6)")
    ap.add_argument("--gif", help="write an animation of the router thinking")
    ap.add_argument("--png", help="write a picture of the final state")
    ap.add_argument("--live", action="store_true", help="show a live matplotlib window")
    ap.add_argument("-q", "--quiet", action="store_true")
    a = ap.parse_args(argv)

    pcb = PcbFile(a.board)
    board = pcb.board()
    s = pcb.settings_from_project(RouteSettings())
    for k in ("track_width", "clearance", "via_diameter", "via_drill", "grid", "via_cost"):
        v = getattr(a, k)
        if v is not None:
            setattr(s, k, v)
    if a.passes:
        s.max_passes = a.passes
    if a.no_global:
        s.global_routing = False
    if a.region_cells:
        s.region_cells = a.region_cells
    if a.workers:
        s.workers = a.workers
    if a.layers:
        s.layers = [x.strip() for x in a.layers.split(",")]
    s.skip_zone_nets = not a.include_zone_nets
    s.tracks_on_planes = a.tracks_on_planes
    nets = [x.strip() for x in a.nets.split(",")] if a.nets else None

    t0 = time.time()
    pb = build_problem(board, s, nets)
    g = pb.grid
    if not a.quiet:
        print(f"Grid {g.nx} x {g.ny} x {g.nl} layers at {g.pitch:.3f} mm; "
              f"{len(pb.nets)} nets to route ({time.time() - t0:.2f}s to build)")

    viz = None
    if a.gif or a.png or a.live:
        if not a.live:
            import matplotlib
            matplotlib.use("Agg")
        from .viz import Visualizer
        viz = Visualizer(board, pb, live=a.live)

    def cb(ev, d):
        if viz is not None:
            viz.callback(ev, d)
        if not a.quiet and ev == "global_done":
            print(f"  global plan: {d['regions']} regions, {d['passes']} pass(es), "
                  f"{d['overflow']} over-full boundaries ({d['seconds']:.2f}s)")
        if not a.quiet and ev == "pass_end":
            print(f"  pass {d['pass']:2d}: {d['overused']:4d} overused cells, "
                  f"{len(d['conflict_nets']):3d} nets in conflict, {d.get('pour_cuts', 0)} pour cuts, "
                  f"{d['length_mm']:8.1f} mm, {d['vias']:3d} vias  ({d['seconds']:.2f}s)")
        return True

    router = Router(pb, cb, record_exploration=viz is not None)
    if viz is not None:
        viz.attach(router)
    res = router.run()
    print(res.summary())

    out = a.output or a.board.replace(".kicad_pcb", "-routed.kicad_pcb")
    n = pcb.add_routes(res, s)
    pcb.save(out)
    print(f"Wrote {n} items to {out}")
    if viz is not None:
        if a.png:
            viz.save_png(a.png)
            print("Wrote", a.png)
        if a.gif:
            viz.save_gif(a.gif)
            print("Wrote", a.gif)
        if a.live:
            import matplotlib.pyplot as plt
            plt.show()
    return 0 if all(nr.complete for nr in res.nets.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
