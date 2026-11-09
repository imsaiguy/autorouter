"""KiCad 9/10+ front end using the IPC API (``kicad-python`` / ``kipy``).

KiCad launches ``ipc_action.py`` (see ``plugin.json``) in its own Python
virtual environment.  This module then:

1. reads the open board over the API (exact pad polygons from KiCad itself),
2. serves a small local web page that shows the router thinking - KiCad's
   plugin Python has no GUI toolkit we can rely on, but every OS has a browser,
3. runs the router in a background thread when you press Route,
4. writes the tracks and vias back in ONE commit (one Undo step).

Run it by hand against a running KiCad (API enabled) with
``python -m kicad_autorouter.ipc_plugin``.
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional, Set, Tuple

from .kicad_pcb import _arc_points
from .model import Board, Pad, RouteSettings, Track, Via
from .problem import BLOCKED, build_problem, loose_copper, net_status, nets_to_finish
from .router import Router
from .progress import ProgressEstimator

NM = 1e6


def _mm(v) -> float:
    return v / NM


def _xy(v) -> Tuple[float, float]:
    return v.x / NM, v.y / NM


# ======================================================================
# kipy board  ->  model.Board
# ======================================================================
def _poly_points(pwh) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    for node in pwh.outline.nodes:
        if node.has_point:
            pts.append(_xy(node.point))
        elif node.has_arc:
            a = node.arc
            s, m, e = _xy(a.start), _xy(a.mid), _xy(a.end)
            arc = _arc_points(s[0], s[1], m[0], m[1], e[0], e[1], seg_len=0.1)
            pts.extend(arc[:-1] if pts else arc)
    return pts


class IpcBoard:
    """Everything read from KiCad, plus what is needed to write back."""

    def __init__(self, kboard):
        from kipy.proto.board.board_types_pb2 import BoardLayer, PadType
        from kipy.util.board_layer import canonical_name, is_copper_layer

        self.kb = kboard
        self.refs: Dict[int, object] = {}
        enabled = list(kboard.get_enabled_layers())
        order = {l: i for i, l in enumerate(_copper_order())}
        cu = sorted((l for l in enabled if is_copper_layer(l)), key=lambda l: order.get(l, 99))
        self.layer_enum: Dict[str, int] = {canonical_name(l): l for l in cu}
        copper = list(self.layer_enum)
        b = Board(copper, self._outline(BoardLayer.BL_Edge_Cuts))
        self.selected: Set[str] = set()

        refs = {}
        for fp in kboard.get_footprints():
            try:
                refs[fp.id.value] = fp.reference_field.text.value
            except Exception:
                pass

        pads = list(kboard.get_pads())
        polys = self._pad_polygons(pads, cu, is_copper_layer)
        for pad in pads:
            net = pad.net.name if pad.net else ""
            px, py = _xy(pad.position)
            ps = pad.padstack
            drill = ps.drill.diameter.x / NM if ps.drill else 0.0
            if pad.pad_type == PadType.PT_NPTH:
                if drill > 0:
                    b.holes.append((px, py, drill))
                continue
            layers = {canonical_name(l) for l in ps.layers if l in self.layer_enum.values()}
            poly = polys.get(pad.id.value)
            if poly:
                xs = [p[0] for p in poly]
                ys = [p[1] for p in poly]
                w, h = max(xs) - min(xs), max(ys) - min(ys)
            else:   # no polygon from KiCad: fall back to the padstack size
                cl = ps.copper_layers
                w = h = 1.0
                if cl:
                    w, h = cl[0].size.x / NM, cl[0].size.y / NM
            parent = pad.parent.value if pad.parent else ""
            ref = f"{refs.get(parent, '?')}.{pad.number}"
            b.pads.append(Pad(net, px, py, w, h, "rect", 0.0, layers,
                              drill if pad.pad_type == PadType.PT_PTH else 0.0, ref, poly or None))

        for t in kboard.get_tracks():
            net = t.net.name if t.net else ""
            ln = canonical_name(t.layer)
            if hasattr(t, "mid"):
                s, m, e = _xy(t.start), _xy(t.mid), _xy(t.end)
                pts = _arc_points(s[0], s[1], m[0], m[1], e[0], e[1])
            else:
                pts = [_xy(t.start), _xy(t.end)]
            for a, c in zip(pts, pts[1:]):
                mt = Track(net, a[0], a[1], c[0], c[1], _mm(t.width), ln)
                b.tracks.append(mt)
                self.refs[id(mt)] = t          # model piece -> the KiCad item it came from
        for v in kboard.get_vias():
            x, y = _xy(v.position)
            try:
                d = _mm(v.diameter)
            except Exception:
                d = 0.6
            mv = Via(v.net.name if v.net else "", x, y, d, _mm(v.drill_diameter))
            b.vias.append(mv)
            self.refs[id(mv)] = v
        for z in kboard.get_zones():
            rule = z.is_rule_area() if callable(z.is_rule_area) else z.is_rule_area
            if rule:
                continue
            net = z.net.name if z.net else ""
            if net:
                b.zone_nets.add(net)
                b.zone_layers.setdefault(net, set()).update(
                    canonical_name(l) for l in z.layers if l in self.layer_enum.values())

        # selection: pads, footprints, tracks, vias, zones -> their nets
        try:
            for it in kboard.get_selection():
                net = getattr(it, "net", None)
                if net is not None and getattr(net, "name", ""):
                    self.selected.add(net.name)
                defn = getattr(it, "definition", None)
                if defn is not None:
                    for p in defn.pads:
                        if p.net and p.net.name:
                            self.selected.add(p.net.name)
        except Exception:
            pass
        self.board = b
        self.nets = {n.name: n for n in kboard.get_nets()}
        self.settings = self._settings()

    # ------------------------------------------------------------------
    def _pad_polygons(self, pads, cu, is_copper_layer):
        """Exact pad outlines from KiCad, one request per copper layer."""
        by_layer: Dict[int, list] = {}
        for pad in pads:
            first = next((l for l in cu if l in set(pad.padstack.layers)), None)
            if first is not None:
                by_layer.setdefault(first, []).append(pad)
        out: Dict[str, List[Tuple[float, float]]] = {}
        for layer, group in by_layer.items():
            try:
                res = self.kb.get_pad_shapes_as_polygons(group, layer)
            except Exception:
                res = None
            if res is not None and len(res) == len(group):
                for pad, poly in zip(group, res):
                    out[pad.id.value] = _poly_points(poly)
            else:   # some pads had no polygon: ask one by one
                for pad in group:
                    try:
                        poly = self.kb.get_pad_shapes_as_polygons(pad, layer)
                    except Exception:
                        poly = None
                    if poly is not None:
                        out[pad.id.value] = _poly_points(poly)
        return out

    def _outline(self, edge_layer) -> List[Tuple[float, float, float, float]]:
        segs = []
        for s in self.kb.get_shapes():
            if s.layer != edge_layer:
                continue
            cls = type(s).__name__
            try:
                if cls == "BoardSegment":
                    segs.append((*_xy(s.start), *_xy(s.end)))
                elif cls == "BoardRectangle":
                    (x0, y0), (x1, y1) = _xy(s.top_left), _xy(s.bottom_right)
                    segs += [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
                elif cls == "BoardCircle":
                    cx, cy = _xy(s.center)
                    r = s.radius() / NM if callable(s.radius) else s.radius / NM
                    n = 48
                    pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
                           for k in range(n + 1)]
                    segs += [(a[0], a[1], c[0], c[1]) for a, c in zip(pts, pts[1:])]
                elif cls == "BoardArc":
                    (sx, sy), (mx, my), (ex, ey) = _xy(s.start), _xy(s.mid), _xy(s.end)
                    pts = _arc_points(sx, sy, mx, my, ex, ey)
                    segs += [(a[0], a[1], c[0], c[1]) for a, c in zip(pts, pts[1:])]
                elif cls == "BoardPolygon":
                    for pwh in s.polygons:
                        pts = _poly_points(pwh)
                        pts.append(pts[0])
                        segs += [(a[0], a[1], c[0], c[1]) for a, c in zip(pts, pts[1:])]
                elif cls == "BoardBezier":
                    segs.append((*_xy(s.start), *_xy(s.end)))
            except Exception:
                continue
        return segs

    def _settings(self) -> RouteSettings:
        s = RouteSettings()
        try:
            classes = self.kb.get_netclass_for_nets(list(self.nets.values()))
            ncs = list(classes.values())
            nc = next((c for c in ncs if c.name == "Default"), ncs[0] if ncs else None)
            if nc is not None:
                for attr, val in (("track_width", nc.track_width), ("clearance", nc.clearance),
                                  ("via_diameter", nc.via_diameter), ("via_drill", nc.via_drill)):
                    if val:
                        setattr(s, attr, val / NM)
        except Exception:
            pass
        return s

    # ------------------------------------------------------------------
    def make_items(self, result, s: RouteSettings) -> list:
        from kipy.board_types import Track as KTrack, Via as KVia
        from kipy.geometry import Vector2
        items = []
        for name, nr in result.nets.items():
            net = self.nets.get(name)
            for (x1, y1, x2, y2, layer) in nr.segments:
                if math.hypot(x2 - x1, y2 - y1) < 1e-6:
                    continue
                t = KTrack()
                t.start = Vector2.from_xy(int(round(x1 * NM)), int(round(y1 * NM)))
                t.end = Vector2.from_xy(int(round(x2 * NM)), int(round(y2 * NM)))
                t.width = int(round(s.track_width * NM))
                t.layer = self.layer_enum[layer]
                if net is not None:
                    t.net = net
                items.append(t)
            for (x, y) in nr.vias:
                v = KVia()
                v.position = Vector2.from_xy(int(round(x * NM)), int(round(y * NM)))
                v.diameter = int(round(s.via_diameter * NM))
                v.drill_diameter = int(round(s.via_drill * NM))
                if net is not None:
                    v.net = net
                items.append(v)
        return items

    def kicad_items(self, model_items) -> list:
        """The KiCad items behind model tracks/vias (arc pieces map to one arc), no duplicates."""
        out, seen = [], set()
        for m in model_items:
            it = self.refs.get(id(m))
            if it is not None and id(it) not in seen:
                seen.add(id(it))
                out.append(it)
        return out

    def apply(self, result, s: RouteSettings, remove_nets: Optional[Set[str]] = None,
              remove_items: Optional[list] = None) -> int:
        items = self.make_items(result, s)
        old = []
        if remove_nets:
            old = [t for t in list(self.kb.get_tracks()) + list(self.kb.get_vias())
                   if t.net and t.net.name in remove_nets and not t.locked]
        if remove_items:
            ids = {id(o) for o in old}
            old += [it for it in remove_items if id(it) not in ids and not getattr(it, "locked", False)]
        if not items and not old:
            return 0          # nothing to change: no empty undo step
        commit = self.kb.begin_commit()
        try:
            if old:
                self.kb.remove_items(old)
            if items:
                self.kb.create_items(items)
            self.kb.push_commit(commit, "Visible Autorouter")
        except Exception:
            self.kb.drop_commit(commit)
            raise
        return len(items)


def _empty_result():
    from .router import RouteResult
    return RouteResult({}, [], True, [], {})


def _copper_order():
    from kipy.util.board_layer import iter_copper_layers
    from kipy.proto.board.board_types_pb2 import BoardLayer
    lay = list(iter_copper_layers())
    # F.Cu first, inner layers in order, B.Cu last
    inner = [l for l in lay if l not in (BoardLayer.BL_F_Cu, BoardLayer.BL_B_Cu)]
    return [BoardLayer.BL_F_Cu] + inner + [BoardLayer.BL_B_Cu]


# ======================================================================
# the session: router thread + state for the web page
# ======================================================================
class Session:
    def __init__(self, ipc: IpcBoard):
        self.ipc = ipc
        self.lock = threading.Lock()
        self.version = 0
        self.running = False
        self.stop = False
        self.static_version = 0
        self.status = self._ready_text()
        self.log: List[str] = []
        self.curve: List[int] = []
        self.frame: dict = {}
        self.grid_info: Optional[dict] = None
        self.last_poll = time.time()
        self.closed = False
        self._last_push = 0.0
        self._heat_cache = None
        self.router: Optional[Router] = None

    def _counts(self) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for st in net_status(self.ipc.board).values():
            counts[st] = counts.get(st, 0) + 1
        return counts

    def _ready_text(self) -> str:
        c = self._counts()
        b = self.ipc.board
        todo = c.get("unrouted", 0) + c.get("partial", 0)
        return (f"Ready - {c.get('routed', 0)} nets routed, {c.get('partial', 0)} partly routed, "
                f"{c.get('unrouted', 0)} unrouted ({todo} to finish); {len(b.tracks)} existing track "
                f"segments, {len(b.vias)} vias; layers {', '.join(b.copper_layers)}.")

    # -------------------------------------------------------------- static
    def static(self) -> dict:
        b = self.ipc.board
        lidx = {n: i for i, n in enumerate(b.copper_layers)}
        tracks = [[lidx.get(t.layer, 0), round(t.x1, 4), round(t.y1, 4), round(t.x2, 4), round(t.y2, 4),
                   round(t.width, 4), t.net] for t in b.tracks if t.layer in lidx]
        vias = [[round(v.x, 4), round(v.y, 4), round(v.diameter, 4), v.net] for v in b.vias]
        status = net_status(b)
        pads = []
        for p in b.pads:
            if p.poly:
                for ring in p.rings():
                    pads.append({"poly": [[round(x, 4), round(y, 4)] for x, y in ring], "net": p.net})
            else:
                pads.append({"c": [p.x, p.y], "r": max(p.w, p.h) / 2, "net": p.net})
        return {"outline": b.outline, "pads": pads, "bbox": b.bbox(), "layers": b.copper_layers,
                "selected": sorted(self.ipc.selected), "nets": b.nets(),
                "settings": self.ipc.settings.__dict__,
                "holes": b.holes, "tracks": tracks, "vias": vias,
                "status": status, "counts": self._counts(),
                "to_finish": nets_to_finish(b), "static_version": self.static_version,
                "loose": sum(map(len, loose_copper(b)))}

    def say(self, line: str):
        with self.lock:
            self.log.append(line)
            self.version += 1

    # -------------------------------------------------------------- routing
    def start(self, opts: dict):
        if self.running:
            return
        s = self.ipc.settings
        for k in ("track_width", "clearance", "via_diameter", "via_drill", "via_cost", "edge_clearance"):
            if k in opts:
                setattr(s, k, float(opts[k]))
        if "max_passes" in opts:
            s.max_passes = int(opts["max_passes"])
        s.fanout_zone_nets = bool(opts.get("fanout", True))
        scope = opts.get("scope", "unrouted")
        rip = bool(opts.get("rip"))
        if scope == "selected" and self.ipc.selected:
            nets = sorted(self.ipc.selected)
        elif scope == "unrouted":
            # finish the ratsnest: only missing connections, existing copper is never touched
            nets = nets_to_finish(self.ipc.board)
            rip = False
        else:
            nets = None
        self.running, self.stop = True, False
        self.curve = []
        self.est = ProgressEstimator(s.max_passes)
        with self.lock:
            self.frame = {}
            self.version += 1
        threading.Thread(target=self._run, args=(s, nets, rip, bool(opts.get("fill", True)), scope,
                                                 bool(opts.get("remove_loose"))),
                         daemon=True).start()

    def _run(self, s, nets, rip, fill, scope="all", remove_loose=False):
        try:
            board = self.ipc.board
            remove = None
            loose_items = []
            if remove_loose:
                # dead-end copper (touches no pad): whole board for "unrouted", else the scope
                lt, lv = loose_copper(board, None if scope == "unrouted" else nets)
                loose_items = self.ipc.kicad_items(lt + lv)
                if loose_items:
                    gone = set(map(id, lt)) | set(map(id, lv))
                    board = dataclasses.replace(board, tracks=[t for t in board.tracks if id(t) not in gone],
                                                vias=[v for v in board.vias if id(v) not in gone])
                    self.say(f"Deleting {len(loose_items)} loose track piece(s)/via(s) that touch no pad "
                             f"(nets: {', '.join(sorted({m.net for m in lt + lv}))}).")
            if scope == "unrouted":
                self.say(f"Unrouted only: finishing {len(nets)} net(s); existing tracks and vias are kept.")
                if not nets and not loose_items:
                    self.say("Nothing to route - every net is already connected.")
                    self.status = "Nothing to route - every net is already connected."
                    return
            if rip:
                remove = set(nets) if nets else set(board.nets())
                # work on a copy: the KiCad board only changes when the result is committed
                board = dataclasses.replace(board, tracks=[t for t in board.tracks if t.net not in remove],
                                            vias=[v for v in board.vias if v.net not in remove])
                n_old = len(self.ipc.board.tracks) + len(self.ipc.board.vias) - len(board.tracks) - len(board.vias)
                self.say(f"Re-routing from scratch: {n_old} existing track/via item(s) of these nets will be "
                         "replaced (only if the run completes).")
            t0 = time.time()
            pb = build_problem(board, s, nets)
            g = pb.grid
            with self.lock:
                self.grid_info = {"x0": g.x0, "y0": g.y0, "pitch": g.pitch, "nx": g.nx, "ny": g.ny,
                                  "nl": g.nl, "layers": g.layers}
            if not pb.nets and loose_items:
                n = self.ipc.apply(_empty_result(), s, None, loose_items)
                self.say(f"Nothing to route; removed {n and len(loose_items)} loose piece(s) as ONE undo step.")
                self.status = "Loose copper removed - nothing else to route."
                self._reread()
                return
            if not pb.nets:
                self.say("Nothing to route - every net is already connected"
                         + (" (pour nets are connected by their zones)." if board.zone_nets else "."))
                self.status = "Nothing to route - every net is already connected."
                return
            self.say(f"Grid {g.nx} x {g.ny} x {g.nl} layers @ {g.pitch:.3f} mm, {len(pb.nets)} nets to route.")
            self._heat_cache = None
            self.router = Router(pb, self._event, record_exploration=True)
            res = self.router.run()
            self._push_frame(None, None, force=True)
            self.say(res.summary())
            stuck = sorted(n for n, nr in res.nets.items() if not nr.complete)
            if scope == "unrouted" and stuck:
                self.say(f"{len(stuck)} net(s) could not be finished around the existing tracks "
                         f"({', '.join(stuck)}). To let the router move copper: select those parts, "
                         "choose Selected, tick 'Remove existing tracks' and Route again.")
            if res.cancelled:
                self.say("Stopped - nothing was written to the board.")
                self.status = "Stopped."
                return
            n = self.ipc.apply(res, s, remove, loose_items)
            if fill and board.zone_nets:
                try:
                    self.ipc.kb.refill_zones()
                    self.say("Zones refilled.")
                except Exception as exc:
                    self.say(f"Zone refill failed: {exc}")
            self.say(f"Added {n} tracks/vias in {time.time() - t0:.1f}s as ONE undo step "
                     "(Edit > Undo in KiCad reverts it). Run DRC to double-check.")
            self.status = res.summary().splitlines()[0]
            self._reread()
        except Exception as exc:
            self.say("ERROR: " + "".join(traceback.format_exception_only(type(exc), exc)).strip())
            self.say(traceback.format_exc())
            self.status = "Error - see log."
        finally:
            self.running = False
            with self.lock:
                self.version += 1

    def _reread(self):
        """Re-read the board so the view and a second run see the new copper."""
        try:
            self.ipc = IpcBoard(self.ipc.kb)
            with self.lock:
                self.static_version += 1
            self.say(self._ready_text())
        except Exception:
            pass

    def _event(self, ev, d):
        if getattr(self, "est", None) is not None:
            self.est.feed(ev, d)
            if ev in ("pass_start", "pass_end", "net_done", "cleanup_start", "done"):
                with self.lock:
                    self.version += 1
        if ev == "global_done":
            self.say(f"Global plan: {d['regions']} regions, corridors set in {d['seconds']:.2f}s.")
        if ev == "pass_start":
            self.status = (f"Pass {d['pass']}: routing {len(d['nets'])} net(s), "
                           f"congestion price x{d['pres_fac']:.2f}")
        elif ev == "connection":
            self._push_frame(d.get("explored"), d.get("net"))
        elif ev == "pass_end":
            self.curve.append(d["overused"])
            self.say(f"pass {d['pass']:2d}: {d['overused']:4d} overused cells, "
                     f"{len(d['conflict_nets']):3d} nets in conflict, {d['vias']:3d} vias")
            self._push_frame(None, None, force=True)
        return not self.stop

    def _push_frame(self, explored, net, force=False):
        now = time.time()
        if not force and now - self._last_push < 0.12:
            return
        self._last_push = now
        r = self.router
        if r is None:
            return
        g = r.pb.grid
        plane = g.plane
        paths = []
        vias = []
        for j in r.pb.nets:
            for p in j.paths:
                run, prev = [], None
                for c in p:
                    l, xy = divmod(c, plane)
                    if prev is not None and l != prev:
                        if len(run) > 1:
                            paths.append([prev, run, j.name == net])
                        vias.append(xy)
                        run = []
                    run.append(xy)
                    prev = l
                if len(run) > 1:
                    paths.append([prev, run, j.name == net])
        # heat + conflicts change only between passes: rebuild them then, not per frame
        if force or self._heat_cache is None:
            heat = {}
            over = set()
            hist, occ = r.hist, r.occ
            for l in range(g.nl):
                base = l * plane
                for xy in range(plane):
                    h = hist[base + xy]
                    if h > 0 and h > heat.get(xy, 0):
                        heat[xy] = round(h, 2)
                    if occ[base + xy] > 1:
                        over.add(xy)
            self._heat_cache = (list(heat.items()), sorted(over))
        heat_items, over_list = self._heat_cache
        exp = sorted({c % plane for c in explored}) if explored else []
        with self.lock:
            self.frame = {"paths": paths, "vias": vias, "heat": heat_items, "over": over_list,
                          "explored": exp, "net": net}
            self.version += 1

    def state(self) -> dict:
        self.last_poll = time.time()
        with self.lock:
            prog = self.est.snapshot() if getattr(self, "est", None) is not None else None
            return {"version": self.version, "running": self.running, "status": self.status,
                    "progress": prog, "stopping": self.stop and self.running,
                    "static_version": self.static_version,
                    "log": self.log[-200:], "curve": self.curve, "frame": self.frame, "grid": self.grid_info}


# ======================================================================
# web server
# ======================================================================
def _handler(session: Session, html: bytes):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/index"):
                self._send(200, html, "text/html; charset=utf-8")
            elif self.path == "/static":
                self._send(200, json.dumps(session.static()).encode())
            elif self.path.startswith("/state"):
                self._send(200, json.dumps(session.state()).encode())
            else:
                self._send(404, b"{}")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
            if self.path == "/route":
                session.start(body)
            elif self.path == "/stop":
                session.stop = True
            elif self.path == "/close":
                session.closed = True
            self._send(200, b"{}")
    return H


def serve(ipc: IpcBoard, open_browser: bool = True, port: int = 0) -> None:
    session = Session(ipc)
    with open(os.path.join(os.path.dirname(__file__), "ipc_ui.html"), "rb") as fh:
        html = fh.read()
    httpd = ThreadingHTTPServer(("127.0.0.1", port), _handler(session, html))
    url = f"http://127.0.0.1:{httpd.server_address[1]}/"
    print("Visible Autorouter UI:", url, flush=True)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    if open_browser:
        webbrowser.open(url)
    # live until the page is closed (Close button, or it stops polling)
    first = time.time()
    while True:
        time.sleep(0.5)
        idle = time.time() - session.last_poll
        if session.closed and not session.running:
            break
        if not session.running and idle > 45 and time.time() - first > 120:
            break
    httpd.shutdown()


def main() -> None:
    from kipy import KiCad
    kicad = KiCad(client_name="visible-autorouter", timeout_ms=30000)
    ipc = IpcBoard(kicad.get_board())
    serve(ipc)


if __name__ == "__main__":
    main()
