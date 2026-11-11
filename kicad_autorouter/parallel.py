"""Multi-core routing of the first pass.

In pass 1 every net is routed "as if it were alone", so the nets can be routed
at the same time on separate CPU cores, and pass 1 is the costliest pass.  The
later passes (the negotiation that makes the board legal) run one net at a
time, exactly as before: routing those in parallel was tried and made the
negotiation oscillate (nets in the same batch cannot see each other).

Each worker is a separate Python process (the GIL rules out threads) holding its
own copy of the routing problem, sent once.  The heaviest nets are handed out
first so the cores finish together.
"""
from __future__ import annotations

import multiprocessing
import multiprocessing.connection
import os
import pickle
import sys
import time
import types
from typing import Dict, List, Optional, Tuple


# ---------------------------------------------------------------------- worker side
def _worker_main(conn, blob: bytes, record: bool) -> None:
    from .router import Router
    pb, gr = pickle.loads(blob)
    events: List[tuple] = []
    r = Router(pb, lambda ev, d: events.append((ev, d)) if ev == "connection" else None, record)
    r.global_router = gr
    jobs = {j.id: j for j in pb.nets}
    occ, hist = r.occ, r.hist
    conn.send(("ready", os.getpid()))
    while True:
        msg = conn.recv()
        kind = msg[0]
        if kind == "stop":
            return
        if kind == "sync":                     # ("sync", pres_fac, occ_delta, hist_set)
            _, r.pres_fac, d_occ, d_hist = msg
            for c, v in d_occ:
                occ[c] += v
            for c, v in d_hist:
                hist[c] = v
            continue
        if kind == "route":                    # ("route", pass_no, [net ids])
            _, pass_no, ids = msg
            for nid in ids:
                job = jobs[nid]
                job.paths, job.resources, job.failed = [], set(), 0
                events.clear()
                t0 = time.perf_counter()
                r._route_net(job, pass_no)
                busy = time.perf_counter() - t0
                for c in job.resources:        # the main process books them for everyone
                    occ[c] -= 1
                for c in job.keepout:          # (the main process books the keep-out too)
                    occ[c] -= 1
                    r.kocc[c] -= 1
                job.keepout = set()
                for c in job.corners:          # (and the side cells of its 45-degree steps)
                    occ[c] -= 1
                    r.cocc[c] -= 1
                job.corners = set()
                # the view only needs a glimpse of the search: send the last connection's
                # wavefront, thinned (full lists from every core swamped the main window)
                slim = []
                for k_, (ev, d) in enumerate(events):
                    d = dict(d)
                    ex = d.get("explored")
                    if ex:
                        d["explored"] = ex[::max(1, len(ex) // 3000)] if k_ == len(events) - 1 else None
                    slim.append((ev, d))
                conn.send(("done", nid, job.paths, job.resources, job.failed, slim, busy, job.fail_notes))
            conn.send(("wave_end",))


# ---------------------------------------------------------------------- main side
def _python_for_workers() -> Optional[str]:
    """A real Python executable (inside KiCad, sys.executable may be kicad.exe/pcbnew)."""
    exe = sys.executable or ""
    win = os.name == "nt"
    cands = []
    if os.path.basename(exe).lower().startswith("python"):
        cands.append(exe)
    for d in (os.path.dirname(exe), sys.prefix, sys.base_prefix,
              os.path.join(sys.prefix, "bin"), os.path.join(sys.base_prefix, "bin")):
        if d:
            cands += [os.path.join(d, n) for n in (("pythonw.exe", "python.exe") if win else ("python3", "python"))]
    found = [c for c in cands if c and os.path.isfile(c)]
    if win:   # no console window per worker when started from a GUI
        for c in found:
            w = os.path.join(os.path.dirname(c), "pythonw.exe")
            if os.path.isfile(w):
                return w
    return found[0] if found else None


def default_workers(n_nets: int) -> int:
    cpu = os.cpu_count() or 1
    if cpu < 3 or n_nets < 20:      # starting the workers costs ~1 s
        return 1
    return max(1, min(cpu - 1, 15, n_nets // 4))


class _Stopped(Exception):
    pass


class WorkerPool:
    def __init__(self, router, n: int):
        self.router = router
        self.n = n
        self.conns = []
        self.procs = []
        self.sent_hist: Dict[int, float] = {}
        self.pending_occ: List[Tuple[int, int]] = []
        pb = router.pb
        ctx = multiprocessing.get_context("spawn")
        exe = _python_for_workers()
        if exe is None:
            raise RuntimeError("no Python executable found for worker processes")
        ctx.set_executable(exe)
        blob = pickle.dumps((pb, router.global_router), protocol=pickle.HIGHEST_PROTOCOL)
        # The child would normally re-run the parent's main script (inside KiCad: KiCad's
        # own; in a user script: its unguarded top-level code).  Hide it while starting.
        real_main = sys.modules.get("__main__")
        sys.modules["__main__"] = types.ModuleType("__main__")
        try:
            for _ in range(n):
                router._emit("tick")           # (Stop can be pressed while they start)
                if router._cancel:
                    self.close(fast=True)
                    raise _Stopped()
                a, b = ctx.Pipe()
                p = ctx.Process(target=_worker_main, args=(b, blob, router.record), daemon=True)
                p.start()
                self.conns.append(a)
                self.procs.append(p)
        finally:
            if real_main is not None:
                sys.modules["__main__"] = real_main
        t0 = time.time()
        for c in self.conns:
            while not c.poll(0.1):           # (checking for Stop while the cores start)
                router._emit("tick")
                if router._cancel:
                    self.close(fast=True)
                    raise _Stopped()
                if time.time() - t0 > 120:
                    self.close(fast=True)
                    raise RuntimeError("worker process did not start")
            c.recv()
        self.start_seconds = time.time() - t0
        self.exe = exe
        self.last_occ = list(router.occ)       # what the workers believe

    # keep the workers' congestion state in step with the main router
    def sync(self, dirty, full_hist: bool) -> None:
        r = self.router
        occ, last = r.occ, self.last_occ
        d_occ = []
        for c in dirty:
            v = occ[c] - last[c]
            if v:
                d_occ.append((c, v))
                last[c] = occ[c]
        d_hist = []
        if full_hist:                          # history only changes between passes
            for c, h in enumerate(r.hist):
                if h != self.sent_hist.get(c, 0.0):
                    d_hist.append((c, h))
                    self.sent_hist[c] = h
        msg = ("sync", r.pres_fac, d_occ, d_hist)
        for c in self.conns:
            c.send(msg)

    def route_wave(self, jobs, pass_no, on_done, dirty, full_hist) -> None:
        """Route nets across the workers from one shared queue: biggest nets first, and
        each worker gets its next net the moment it finishes one, so every core stays
        busy until the queue runs dry (handing out fixed shares up front left most cores
        idle while a few worked through lists that turned out longer than estimated)."""
        self.sync(dirty, full_hist)
        queue = [j.id for j in sorted(jobs, key=lambda j: -(j.hpwl() + 1.0) * len(j.terminals))]
        queue.reverse()                        # pop() from the end = biggest first
        busy = set()
        for k in range(self.n):
            if queue:
                self.conns[k].send(("route", pass_no, [queue.pop()]))
                busy.add(k)
        while busy:
            if self.router._cancel:            # Stop pressed: abandon the rest
                self.close(fast=True)
                raise _Stopped()
            ready = multiprocessing.connection.wait([self.conns[k] for k in busy], timeout=0.1)
            if not ready:
                on_done(None)                  # keep the UI alive
                continue
            for c in ready:
                msg = c.recv()
                k = self.conns.index(c)
                if msg[0] == "wave_end":
                    if queue:                  # this worker is free: give it the next net
                        c.send(("route", pass_no, [queue.pop()]))
                    else:
                        busy.discard(k)
                else:
                    on_done(msg)

    def close(self, fast: bool = False) -> None:
        for c in self.conns:
            try:
                c.send(("stop",))
            except Exception:
                pass
        deadline = time.time() + (0.0 if fast else 2.0)
        for p in self.procs:
            p.join(timeout=max(0.0, deadline - time.time()))
            if p.is_alive():
                p.terminate()

