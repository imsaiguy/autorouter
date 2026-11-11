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
    ap.add_argument("--straighten-only", action="store_true",
                    help="do not route: clean up the tracks already on the board (all nets, or --nets)")
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

    if a.straighten_only:
        from .restraighten import detect_grid, straighten_existing
        from .router import NetRoute, RouteResult
        s = detect_grid(board, s)
        t0 = time.time()
        res, replaced, fixed = straighten_existing(board, s, nets)
        gone = {id(x) for x in replaced}
        touched = set(res.nets)
        # rewrite these nets: the straightened copper + every piece of theirs that was kept
        keep_t = [t for t in board.tracks if t.net in touched and id(t) not in gone]
        keep_v = [v for v in board.vias if v.net in touched and id(v) not in gone]
        pcb.remove_copper(sorted(touched))
        n = pcb.add_routes(res, s)
        kept = RouteResult({}, [], True, [], {})
        for t in keep_t:
            nr = kept.nets.setdefault(t.net, NetRoute(t.net, complete=True))
            nr.segments.append((t.x1, t.y1, t.x2, t.y2, t.layer))
            nr.widths.append(t.width)
        for v in keep_v:
            kept.nets.setdefault(v.net, NetRoute(v.net, complete=True)).vias.append((v.x, v.y))
        pcb.add_routes(kept, s)
        out = a.output or a.board.replace(".kicad_pcb", "-straightened.kicad_pcb")
        pcb.save(out)
        old_n = sum(1 for x in replaced if hasattr(x, "x1"))
        print(f"Straightened {len(touched)} nets on a {s.pitch():.3f} mm grid: {old_n} track segments -> "
              f"{sum(len(nr.segments) for nr in res.nets.values())} ({len(keep_t)} off-grid pieces kept) "
              f"in {time.time() - t0:.1f}s; wrote {out}")
        return 0

    from .problem import add_bars_to_result, same_net_bars
    import dataclasses
    bars = same_net_bars(board, s, nets)          # fine-pitch same-net neighbours
    board_plain = board
    if bars:
        board = dataclasses.replace(board, tracks=list(board.tracks) + bars)
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
    router.cleanup_in_run = False             # straightened below (the last step, always)
    if viz is not None:
        viz.attach(router)
    res = router.run()
    add_bars_to_result(res, bars)
    board = board_plain
    from .restraighten import straighten_result
    from .router import NetRoute, RouteResult
    res, old_items = straighten_result(board, res, s)
    print(res.summary())

    out = a.output or a.board.replace(".kicad_pcb", "-routed.kicad_pcb")
    if old_items:
        # existing copper of these nets was re-shaped too: rewrite those nets' old copper
        gone = {id(x) for x in old_items}
        touched = sorted({x.net for x in old_items})
        kept = RouteResult({}, [], True, [], {})
        for t in board.tracks:
            if t.net in touched and id(t) not in gone:
                nr = kept.nets.setdefault(t.net, NetRoute(t.net, complete=True))
                nr.segments.append((t.x1, t.y1, t.x2, t.y2, t.layer))
                nr.widths.append(t.width)
        for v in board.vias:
            if v.net in touched and id(v) not in gone:
                kept.nets.setdefault(v.net, NetRoute(v.net, complete=True)).vias.append((v.x, v.y))
        pcb.remove_copper(touched)
        pcb.add_routes(kept, s)
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
