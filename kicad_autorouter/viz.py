"""Watch the router think (matplotlib).

    pb = build_problem(board, settings)
    viz = Visualizer(board, pb, live=True)          # live window, or live=False to record
    router = Router(pb, viz.callback, record_exploration=True)
    viz.attach(router)
    result = router.run()
    viz.save_gif("routing.gif")                     # frames recorded along the way

What is drawn
-------------
* pads (grey), board outline
* tracks: one colour per copper layer (front red, back blue, inner green/...)
* the A* wavefront of the connection just routed (cells it explored)
* congestion history - the "price map" PathFinder has learned - as a heatmap
* overused cells (two nets in one place) as yellow squares
* a convergence plot: overused cells vs. pass
"""
from __future__ import annotations

import math
from typing import List, Optional

from .model import Board
from .problem import Problem

LAYER_COLORS = ["#e8433a", "#3b7ddd", "#3fae5a", "#c9a227", "#a05ad1", "#2bb5b0", "#d16fa0", "#888888"]


class Visualizer:
    def __init__(self, board: Board, problem: Problem, live: bool = False,
                 wavefront_passes: int = 1, frame_every_connection: bool = True,
                 max_frames: int = 160):
        self.board = board
        self.pb = problem
        self.live = live
        self.wave_passes = wavefront_passes
        self.every_conn = frame_every_connection
        self.max_frames = max_frames
        self.router = None
        self.frames: List[dict] = []
        self.history: List[int] = []     # overused per pass
        self._pass = 0
        self._pres = 0.0
        self._last_over: List[int] = []
        self._fig = None
        self._title_extra = ""

    def attach(self, router) -> None:
        self.router = router

    # ------------------------------------------------------------- snapshot
    def _snapshot(self, explored=None, highlight=None, note=""):
        g = self.pb.grid
        polys = []   # (layer, xs, ys, net)
        vias = []
        for j in self.pb.nets:
            for p in j.paths:
                run = []
                prev_l = None
                for c in p:
                    x, y, l = g.xyl(c)
                    if prev_l is not None and l != prev_l:
                        if len(run) > 1:
                            polys.append((prev_l, [q[0] for q in run], [q[1] for q in run], j.name))
                        vias.append(g.center(x, y))
                        run = []
                    run.append(g.center(x, y))
                    prev_l = l
                if len(run) > 1:
                    polys.append((prev_l, [q[0] for q in run], [q[1] for q in run], j.name))
        hist = None
        r = self.router
        if r is not None:
            hist = [0.0] * g.plane
            for l in range(g.nl):
                base = l * g.plane
                hl = r.hist
                for xy in range(g.plane):
                    v = hl[base + xy]
                    if v > hist[xy]:
                        hist[xy] = v
        exp_xy = None
        if explored:
            exp_xy = sorted({c % g.plane for c in explored})
        return {"pass": self._pass, "pres": self._pres, "polys": polys, "vias": vias,
                "hist": hist, "over": list(self._last_over), "explored": exp_xy,
                "highlight": highlight, "note": note, "curve": list(self.history)}

    def _push(self, frame):
        self.frames.append(frame)
        if self.live:
            self._draw_live(frame)

    # ------------------------------------------------------------- callback
    def callback(self, ev: str, d: dict):
        if ev == "pass_start":
            self._pass, self._pres = d["pass"], d["pres_fac"]
        elif ev == "connection":
            wave = d["explored"] if d["pass"] <= self.wave_passes else None
            if self.every_conn or wave:
                note = f"routing {d['net']}" + ("" if d["path"] else "  - NO PATH")
                self._push(self._snapshot(wave, d["net"], note))
        elif ev == "net_done":
            if not self.every_conn and d["pass"] > self.wave_passes:
                self._push(self._snapshot(None, d["net"], f"re-routed {d['net']}"))
        elif ev == "pass_end":
            g = self.pb.grid
            over = set()
            r = self.router
            if r is not None:
                for j in self.pb.nets:
                    for c in j.resources:
                        if r.occ[c] > 1:
                            over.add(c % g.plane)
            self._last_over = sorted(over)
            self.history.append(d["overused"])
            n = len(d["conflict_nets"])
            note = (f"pass {d['pass']} done: {d['overused']} overused cells, {n} nets in conflict"
                    if d["overused"] else f"pass {d['pass']} done: no conflicts - legal!")
            self._push(self._snapshot(None, None, note))
        elif ev == "done":
            self._last_over = []
            self._push(self._snapshot(None, None, "done: " + d["result"].summary().splitlines()[0]))
            if self.live:
                import matplotlib.pyplot as plt
                plt.ioff()
        return True

    # ------------------------------------------------------------- drawing
    def _setup(self, fig=None):
        import matplotlib
        import matplotlib.pyplot as plt
        if fig is None:
            fig = plt.figure(figsize=(11, 6.2), facecolor="#14161a")
        gs = fig.add_gridspec(1, 2, width_ratios=[3.2, 1], wspace=0.18,
                              left=0.03, right=0.97, top=0.9, bottom=0.08)
        ax = fig.add_subplot(gs[0])
        axc = fig.add_subplot(gs[1])
        return fig, ax, axc

    def _draw_static(self, ax):
        from matplotlib.patches import Circle, FancyBboxPatch, Polygon
        b = self.board
        ax.set_facecolor("#0d2416")
        for (x1, y1, x2, y2) in b.outline:
            ax.plot([x1, x2], [y1, y2], color="#e5d38a", lw=1.2, zorder=1)
        for p in b.pads:
            col = "#9aa0a6" if p.net else "#5f6368"
            if p.poly:
                for ring in p.rings():
                    ax.add_patch(Polygon(ring, closed=True, color=col, zorder=2, alpha=0.9))
            elif p.shape == "circle":
                ax.add_patch(Circle((p.x, p.y), p.w / 2, color=col, zorder=2, alpha=0.9))
            else:
                a = math.radians(p.angle)
                c, s = math.cos(a), math.sin(a)
                pts = []
                for (lx, ly) in ((-p.w / 2, -p.h / 2), (p.w / 2, -p.h / 2), (p.w / 2, p.h / 2), (-p.w / 2, p.h / 2)):
                    pts.append((p.x + lx * c + ly * s, p.y - lx * s + ly * c))
                ax.add_patch(Polygon(pts, closed=True, color=col, zorder=2, alpha=0.9))
            if p.drill > 0:
                ax.add_patch(Circle((p.x, p.y), p.drill / 2, color="#0d2416", zorder=3))
        # existing copper (already on the board), muted so new routes stand out
        lidx = {n: i for i, n in enumerate(b.copper_layers)}
        for t in sorted(b.tracks, key=lambda t: -lidx.get(t.layer, 0)):
            ax.plot([t.x1, t.x2], [t.y1, t.y2], color=LAYER_COLORS[lidx.get(t.layer, 0) % len(LAYER_COLORS)],
                    lw=1.2, alpha=0.45, solid_capstyle="round", zorder=2.5)
        for v in b.vias:
            ax.add_patch(Circle((v.x, v.y), v.diameter / 2, fc="#9a9a9a", ec="none", alpha=0.6, zorder=2.6))
        x0, y0, x1, y1 = b.bbox()
        m = 1.0
        ax.set_xlim(x0 - m, x1 + m)
        ax.set_ylim(y1 + m, y0 - m)   # KiCad y is down
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_visible(False)

    def _draw_frame(self, fig, ax, axc, fr):
        import numpy as np
        from matplotlib.patches import Circle
        g = self.pb.grid
        ax.clear()
        axc.clear()
        self._draw_static(ax)
        ext = (g.x0 - g.pitch / 2, g.x0 + (g.nx - 0.5) * g.pitch,
               g.y0 + (g.ny - 0.5) * g.pitch, g.y0 - g.pitch / 2)
        if fr["hist"] is not None and max(fr["hist"]) > 0:
            h = np.array(fr["hist"], dtype=float).reshape(g.ny, g.nx)
            h = np.ma.masked_where(h <= 0, h)
            ax.imshow(np.log1p(h), extent=ext, cmap="inferno", alpha=0.75, zorder=1.5,
                      interpolation="nearest", vmin=0, vmax=max(1.0, float(np.log1p(h.max()))))
        if fr["explored"]:
            m = np.zeros(g.plane)
            m[fr["explored"]] = 1
            m = np.ma.masked_where(m == 0, m).reshape(g.ny, g.nx)
            ax.imshow(m, extent=ext, cmap="cool", alpha=0.35, zorder=1.6, interpolation="nearest")
        for (l, xs, ys, net) in fr["polys"]:
            hl = fr["highlight"] == net
            ax.plot(xs, ys, color=LAYER_COLORS[l % len(LAYER_COLORS)], lw=2.6 if hl else 1.6,
                    alpha=1.0 if hl else 0.85, solid_capstyle="round", zorder=4 + (l == 0))
        for (vx, vy) in fr["vias"]:
            ax.add_patch(Circle((vx, vy), self.pb.settings.via_diameter / 2, fc="#d8d8d8",
                                ec="#555", lw=0.5, zorder=6))
        if fr["over"]:
            xs = [g.x0 + (c % g.nx) * g.pitch for c in fr["over"]]
            ys = [g.y0 + (c // g.nx) * g.pitch for c in fr["over"]]
            ax.scatter(xs, ys, marker="s", s=22, facecolors="none", edgecolors="#ffe14d",
                       linewidths=1.2, zorder=7)
        fig.suptitle(f"Pass {fr['pass']}   ·   congestion price ×{fr['pres']:.2f}   ·   {fr['note']}",
                     color="#e8eaed", fontsize=11, x=0.03, ha="left")
        # legend-ish text
        names = " ".join(f"■ {n}" for n in g.layers)
        ax.text(0.0, -0.035, "layers: " + ", ".join(g.layers) +
                "   ·   heat = learned congestion history   ·   □ = overused cell   ·   cyan = A* wavefront",
                transform=ax.transAxes, color="#9aa0a6", fontsize=7.5, va="top")
        # convergence curve
        axc.set_facecolor("#1c1f24")
        cur = fr["curve"]
        if cur:
            axc.plot(range(1, len(cur) + 1), cur, "-o", color="#ffe14d", ms=3, lw=1.5)
        axc.set_title("overused cells per pass", color="#e8eaed", fontsize=9)
        axc.set_xlabel("pass", color="#9aa0a6", fontsize=8)
        axc.tick_params(colors="#9aa0a6", labelsize=7)
        for sp in axc.spines.values():
            sp.set_color("#444")
        axc.set_xlim(0.5, max(5, len(cur) + 0.5))
        axc.set_ylim(bottom=0, top=max([1] + cur) * 1.1)

    def _draw_live(self, fr):
        import matplotlib.pyplot as plt
        if self._fig is None:
            plt.ion()
            self._fig, self._ax, self._axc = self._setup()
        self._draw_frame(self._fig, self._ax, self._axc, fr)
        plt.pause(0.001)

    def save_gif(self, path: str, fps: int = 6, hold_last: int = 12, dpi: int = 90) -> None:
        import matplotlib
        matplotlib.use("Agg") if not self.live else None
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, PillowWriter
        frames = self.frames
        if len(frames) > self.max_frames:   # keep every pass_end frame, thin the rest
            keep = [i for i, f in enumerate(frames) if f["note"].startswith(("pass", "done"))]
            others = [i for i in range(len(frames)) if i not in keep]
            budget = max(0, self.max_frames - len(keep))
            step = max(1, math.ceil(len(others) / max(1, budget)))
            idx = sorted(set(keep) | set(others[::step]))
            frames = [frames[i] for i in idx]
        frames = frames + [frames[-1]] * hold_last
        fig, ax, axc = self._setup()

        def upd(k):
            self._draw_frame(fig, ax, axc, frames[k])
            return []

        anim = FuncAnimation(fig, upd, frames=len(frames), blit=False)
        anim.save(path, writer=PillowWriter(fps=fps), dpi=dpi,
                  savefig_kwargs={"facecolor": fig.get_facecolor()})
        plt.close(fig)

    def save_png(self, path: str, frame: int = -1, dpi: int = 130) -> None:
        import matplotlib
        import matplotlib.pyplot as plt
        fig, ax, axc = self._setup()
        self._draw_frame(fig, ax, axc, self.frames[frame])
        fig.savefig(path, dpi=dpi, facecolor=fig.get_facecolor())
        plt.close(fig)
