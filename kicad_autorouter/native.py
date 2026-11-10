"""Loader for the compiled A* core (native/astar.c), via ctypes.

The library is plain C (no Python headers), so one file per operating system works
with every Python version.  Shipped: Linux x86-64 and Windows x64.  Anywhere else
(or with AUTOROUTER_NO_NATIVE=1) the router uses its Python search - same results,
just slower.
"""
from __future__ import annotations

import ctypes
import os
import platform
import sys
from typing import Optional

HERE = os.path.dirname(os.path.abspath(__file__))
_LIB = None
_TRIED = False


def _lib_name() -> Optional[str]:
    mach = platform.machine().lower()
    if sys.platform.startswith("win") and mach in ("amd64", "x86_64") and sys.maxsize > 2 ** 32:
        return "astar_win64.dll"
    if sys.platform.startswith("linux") and mach in ("x86_64", "amd64"):
        return "libastar_linux_x86_64.so"
    if sys.platform == "darwin":
        return "libastar_macos.dylib"        # not shipped yet: Python fallback
    return None


def load():
    """The ctypes library, or None (then the Python search is used)."""
    global _LIB, _TRIED
    if _TRIED:
        return _LIB
    _TRIED = True
    if os.environ.get("AUTOROUTER_NO_NATIVE"):
        return None
    name = _lib_name()
    path = os.path.join(HERE, "native", name) if name else None
    if not path or not os.path.isfile(path):
        return None
    try:
        lib = ctypes.CDLL(path)
    except OSError:
        return None
    P_INT, P_DBL, P_U8 = ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_ubyte)
    lib.astar_new.restype = ctypes.c_void_p
    lib.astar_new.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, P_INT, P_INT, P_INT, P_DBL, P_DBL,
                              ctypes.c_int, P_INT, P_INT]
    lib.astar_free.restype = None
    lib.astar_free.argtypes = [ctypes.c_void_p]
    lib.astar_run.restype = ctypes.c_int
    lib.astar_run.argtypes = [ctypes.c_void_p, ctypes.c_int,
                              P_INT, P_INT, P_DBL, ctypes.c_int,          # sources, dirs, g0, n
                              P_INT, ctypes.c_int,                        # targets, n
                              P_U8, ctypes.c_int, ctypes.c_int,           # goal, layer_only, layer mask
                              ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,   # bbox
                              P_U8,                                       # allowed (corridor)
                              P_INT, ctypes.c_int,                        # own, n
                              P_INT, P_U8, ctypes.c_int,                  # branch masks
                              P_INT, P_DBL, P_U8, ctypes.c_int,           # escapes arrived on
                              P_INT, P_U8, ctypes.c_int,                  # escapes left from
                              ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                              ctypes.c_int, ctypes.c_double, ctypes.c_double,
                              ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                              P_INT, ctypes.c_int, P_INT, ctypes.c_int, P_INT]
    _LIB = lib
    return lib


def ints(seq):
    seq = list(seq)
    return (ctypes.c_int * max(1, len(seq)))(*seq), len(seq)


def dbls(seq):
    seq = list(seq)
    return (ctypes.c_double * max(1, len(seq)))(*seq)


def u8s(seq):
    seq = list(seq)
    return (ctypes.c_ubyte * max(1, len(seq)))(*seq)


def view(buf, ctype):
    """A ctypes view (no copy) of an array.array / bytearray."""
    return (ctype * len(buf)).from_buffer(buf)


class NativeSearch:
    """One C search context per router (scratch memory kept between calls)."""

    def __init__(self, lib, grid, owner, via_owner, occ, hist, dir_mult, footprint):
        self.lib = lib
        self._keep = [view(owner, ctypes.c_int), view(via_owner, ctypes.c_int),
                      view(occ, ctypes.c_int), view(hist, ctypes.c_double)]
        flat = [v for row in dir_mult for v in row]
        self._dm = dbls(flat)
        fdx, n = ints([d[0] for d in footprint])
        fdy, _ = ints([d[1] for d in footprint])
        self.ctx = lib.astar_new(grid.nx, grid.ny, grid.nl, self._keep[0], self._keep[1], self._keep[2],
                                 self._keep[3], self._dm, n, fdx, fdy)
        if not self.ctx:
            raise MemoryError("native A* context")
        self.path_cap = grid.size + 16
        self.out = (ctypes.c_int * self.path_cap)()
        self.expl_cap = 0
        self.expl = None
        self.n_expl = ctypes.c_int(0)

    def __del__(self):
        try:
            if self.ctx:
                self.lib.astar_free(self.ctx)
                self.ctx = None
        except Exception:
            pass
