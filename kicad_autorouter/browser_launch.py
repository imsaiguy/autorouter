"""Start the browser version (the IPC-API front end) from the classic window.

KiCad only shows the browser version's own toolbar button once it has built that
plugin's Python environment (a private venv with kicad-python).  This finds that
environment - or any Python that can import kipy - and starts ipc_action.py with
it, so the browser version can be opened even when its button is missing.
"""
from __future__ import annotations

import os
import platform
import subprocess
import sys
import tempfile
from typing import List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ID = "com.github.imsaiguy.visible-autorouter"
LOG = os.path.join(tempfile.gettempdir(), "visible-autorouter-browser.log")


def _cache_dirs() -> List[str]:
    try:
        sys.path.insert(0, HERE)
        import check_install
        out = []
        for v in check_install.VERSIONS:
            out += check_install.candidates(v)[2]
        return out
    except Exception:
        return []
    finally:
        if sys.path and sys.path[0] == HERE:
            sys.path.pop(0)


def candidates() -> List[str]:
    """Python executables that may have kipy, best first."""
    win = platform.system() == "Windows"
    out = []
    for c in _cache_dirs():
        env = os.path.join(c, "python-environments", PLUGIN_ID)
        out.append(os.path.join(env, "Scripts", "python.exe") if win else os.path.join(env, "bin", "python"))
    return [p for p in dict.fromkeys(out) if os.path.exists(p)]


def has_kipy(py: str) -> bool:
    try:
        r = subprocess.run([py, "-c", "import kipy"], capture_output=True, timeout=60,
                           **_no_window())
        return r.returncode == 0
    except Exception:
        return False


def _no_window() -> dict:
    if platform.system() == "Windows":
        return {"creationflags": 0x08000000}   # CREATE_NO_WINDOW
    return {}


def find_python() -> Optional[str]:
    for py in candidates():
        if has_kipy(py):
            return py
    return None


def launch() -> Tuple[bool, str]:
    """Start the browser version.  Returns (started, python used / message for the user)."""
    py = find_python()
    if py is None:
        looked = ", ".join(os.path.join(c, "python-environments", PLUGIN_ID) for c in _cache_dirs())
        return False, ("The browser version needs the Python environment KiCad builds for it "
                       "(with kicad-python), and it was not found or is not finished.\n\n"
                       "1. Preferences > Preferences > Plugins: tick 'Enable KiCad API'.\n"
                       "2. Restart KiCad and wait about a minute (it needs internet once).\n"
                       "3. Still nothing? Run check_install.py and send me its output.\n\n"
                       f"Looked in: {looked or '(no KiCad cache folder found)'}")
    env = dict(os.environ)
    env["KICAD_AUTOROUTER_NO_PLUGIN"] = "1"
    for k in ("PYTHONHOME", "PYTHONPATH"):    # KiCad's own Python settings would confuse the venv
        env.pop(k, None)
    log = open(LOG, "w", encoding="utf-8")
    subprocess.Popen([py, "-u", os.path.join(HERE, "ipc_action.py")], cwd=HERE, env=env,
                     stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                     close_fds=True, **_no_window())
    log.close()
    # Do not wait here: KiCad answers the browser version only once this window is closed.
    # If it cannot connect, it shows the reason in the web browser itself.
    return True, py


def _read(path: str) -> str:
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            return fh.read()
    except OSError:
        return ""
