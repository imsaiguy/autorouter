"""A small, visible, multi-pass autorouter for KiCad 10.

Core (pure Python, plus an optional compiled search in native/):
    model.py    board description (pads, tracks, outline) in mm
    problem.py  board -> routing grid with clearance-aware ownership maps
    router.py   A* + PathFinder negotiated congestion, multi-pass, with events

Front ends:
    ipc_plugin.py  KiCad 10 plugin (IPC API, browser view) - started by ipc_action.py
    kicad_pcb.py   read / write .kicad_pcb files directly   (python -m kicad_autorouter)
    viz.py         matplotlib live view / GIF of the router thinking
"""
__version__ = "0.3.0"
__build__ = "2026-11-12 twin pairs via bigger pin"   # shown in the log: which version routed the board

from .model import Board, Pad, Track, Via, RouteSettings  # noqa: F401
from .router import Router, RouteResult, route              # noqa: F401
