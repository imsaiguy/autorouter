"""Turn router events into progress numbers a person can read: an overall
fraction, the current pass's fraction, and an estimated time remaining.

PathFinder does not know in advance how many passes it needs, so the overall
figure is an *estimate*: it assumes conflicts keep shrinking at the rate seen
so far, and that each net takes as long as the nets routed so far.
"""
import math
import time


class ProgressEstimator:
    def __init__(self, max_passes: int):
        self.max_passes = max_passes
        self.t0 = time.time()
        self.pass_no = 0
        self.pass_total = 1
        self.pass_done = 0
        self.overused = []          # per finished pass
        self.conflict_nets = []     # per finished pass
        self.nets_routed = 0        # net routings completed (all passes)
        self.phase = "starting"
        self.done = False
        self._shown = 0.0           # the bar never moves backwards

    def feed(self, ev: str, d: dict) -> None:
        if ev == "pass_start":
            self.pass_no = d["pass"]
            self.pass_total = max(1, len(d.get("nets", [])))
            self.pass_done = 0
            self.phase = "routing"
        elif ev == "net_done":
            self.pass_done = d.get("index", self.pass_done + 1)
            self.pass_total = d.get("total", self.pass_total)
            self.nets_routed += 1
        elif ev == "pass_end":
            self.overused.append(d["overused"])
            self.conflict_nets.append(len(d.get("conflict_nets", [])))
            self.pass_done = self.pass_total
        elif ev == "cleanup_start":
            self.phase = "straightening"
        elif ev == "done":
            self.phase = "done"
            self.done = True

    # ------------------------------------------------------------------
    def _remaining_routings(self) -> float:
        """Estimated net routings still to do (rest of this pass + future passes)."""
        rest = max(0, self.pass_total - self.pass_done)
        if not self.overused:
            # first pass: guess later passes re-route about as many nets again in total
            return rest + self.pass_total * 1.0
        last_over = self.overused[-1]
        if last_over == 0:
            return rest
        if len(self.overused) >= 2 and self.overused[-2] > 0:
            r = min(0.9, max(0.2, last_over / self.overused[-2]))
        else:
            r = 0.5
        passes_left = min(self.max_passes - self.pass_no,
                          max(1, math.ceil(math.log(max(last_over, 1) + 1) / -math.log(r))))
        per_pass = max(1, self.conflict_nets[-1])
        future = 0.0
        for _ in range(passes_left):
            future += per_pass
            per_pass = max(1.0, per_pass * r)
        return rest + future

    def snapshot(self) -> dict:
        elapsed = time.time() - self.t0
        if self.done:
            return {"phase": "done", "fraction": 1.0, "pass": self.pass_no, "max_passes": self.max_passes,
                    "pass_done": self.pass_total, "pass_total": self.pass_total, "elapsed": elapsed,
                    "eta": 0.0, "overused": self.overused[-1] if self.overused else None,
                    "text": f"Done in {fmt(elapsed)}", "headline": f"Done in {fmt(elapsed)}",
                    "detail": f"{self.pass_no} pass(es)"}
        remaining = self._remaining_routings()
        frac = self.nets_routed / max(1.0, self.nets_routed + remaining)
        if self.phase == "straightening":
            frac = max(frac, 0.97)
        frac = min(0.99, frac)
        # never move backwards: if the estimate grows, the bar just slows down
        frac = max(self._shown, frac)
        self._shown = frac
        eta = None
        if self.nets_routed >= 3 and elapsed > 1.0:
            eta = elapsed / self.nets_routed * remaining
        over = self.overused[-1] if self.overused else None
        if self.phase == "straightening":
            text = "Straightening routes..."
        else:
            text = (f"Pass {self.pass_no}  ·  net {self.pass_done} of {self.pass_total}"
                    + (f"  ·  {over} conflict{'' if over == 1 else 's'} left" if over is not None
                       else "  ·  first pass")
                    + f"  ·  {fmt(elapsed)} elapsed"
                    + (f"  ·  about {fmt(eta)} left" if eta is not None else "  ·  estimating..."))
        pct = int(frac * 100)
        if self.phase == "straightening":
            headline = f"{pct}%  ·  straightening routes..."
            detail = f"{fmt(elapsed)} elapsed"
        else:
            headline = f"{pct}%  ·  " + (f"about {fmt(eta)} left" if eta is not None else "estimating time...")
            detail = (f"Pass {self.pass_no}  ·  net {self.pass_done} of {self.pass_total}  ·  "
                      + (f"{over} conflict{'' if over == 1 else 's'} left" if over is not None else "first pass")
                      + f"  ·  {fmt(elapsed)} elapsed")
        return {"phase": self.phase, "fraction": frac, "pass": self.pass_no, "max_passes": self.max_passes,
                "pass_done": self.pass_done, "pass_total": self.pass_total, "elapsed": elapsed,
                "eta": eta, "overused": over, "text": text, "headline": headline, "detail": detail}


def fmt(seconds) -> str:
    if seconds is None:
        return "?"
    seconds = int(round(seconds))
    return f"{seconds // 60}:{seconds % 60:02d}"
