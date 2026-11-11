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
                              P_INT, ctypes.c_int, P_INT, ctypes.c_int, P_INT,
                              ctypes.c_int, P_U8, P_U8, P_U8, ctypes.c_int, ctypes.c_int, P_INT, ctypes.c_int,
                              ctypes.c_double, ctypes.c_double]
    try:
        lib.astar_set_dir_mult.restype = None
        lib.astar_set_dir_mult.argtypes = [ctypes.c_void_p, P_DBL]
    except AttributeError:
        pass
    try:
        lib.astar_set_corners.restype = None
        lib.astar_set_corners.argtypes = [ctypes.c_void_p, P_INT, P_INT]
    except AttributeError:
        pass
    try:
        lib.pour_labels_k.restype = ctypes.c_int
        lib.pour_labels_k.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, P_U8, P_INT, ctypes.c_int,
                                      P_INT, P_INT, P_U8, ctypes.c_int]
    except AttributeError:
        pass
    try:
        lib.pour_labels.restype = ctypes.c_int
        lib.pour_labels.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, P_U8, P_INT, ctypes.c_int,
                                    P_INT, P_INT, P_U8]
    except AttributeError:                    # an older library without it
        pass
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

    def set_dir_mult(self, dir_mult) -> bool:
        """Replace the per-layer direction cost factors (False: this library can't)."""
        if not hasattr(self.lib, "astar_set_dir_mult"):
            return False
        self._dm = dbls([v for row in dir_mult for v in row])
        self.lib.astar_set_dir_mult(self.ctx, self._dm)
        return True

    def set_corners(self, cocc, diag_owner=None) -> None:
        """The side-cell share of occ (two 45-degree steps may share a side cell), and
        who may run a 45-degree step through each 2x2 middle (None: judge by side cells)."""
        if hasattr(self.lib, "astar_set_corners"):
            self._cocc = view(cocc, ctypes.c_int)
            self._dg = view(diag_owner, ctypes.c_int) if diag_owner is not None else None
            self.lib.astar_set_corners(self.ctx, self._cocc, self._dg)

    def __del__(self):
        try:
            if self.ctx:
                self.lib.astar_free(self.ctx)
                self.ctx = None
        except Exception:
            pass
