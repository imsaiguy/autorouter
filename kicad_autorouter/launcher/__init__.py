"""Toolbar button that starts the Visible Autorouter (KiCad 10).

KiCad 10 shows the plugin's own (IPC API) button only when its per-start checks
pass, and on some PCs it never does.  This tiny classic-style action only
STARTS the real plugin - ipc_action.py, with the Python environment KiCad built
for it - exactly like run_autorouter.bat.  There is no router code in here.

install_plugin.py copies this folder to <KiCad>/10.0/scripting/plugins/ and
writes launcher.json next to it (where the real plugin is).
"""
import json
import os
import subprocess
import sys
import tempfile

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ID = "com.github.imsaiguy.visible-autorouter"
LOG = os.path.join(tempfile.gettempdir(), "visible-autorouter-browser.log")


def _config():
    try:
        with open(os.path.join(HERE, "launcher.json"), encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:
        return {}


def _find_python(cfg):
    win = sys.platform.startswith("win")
    cands = list(cfg.get("env_pythons", []))
    for base in (os.environ.get("LOCALAPPDATA", ""), os.path.expanduser("~/Library/Caches"),
                 os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache")):
        if base:
            for sub in ("KiCad", "kicad"):
                env = os.path.join(base, sub, "10.0", "python-environments", PLUGIN_ID)
                cands.append(os.path.join(env, "Scripts", "python.exe") if win else os.path.join(env, "bin", "python"))
    return next((p for p in cands if p and os.path.isfile(p)), None)


def _message(text):
    try:
        import wx
        wx.MessageBox(text, "Visible Autorouter")
    except Exception:
        print(text)


class VisibleAutorouterLauncher(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "Visible Autorouter"
        self.category = "Routing"
        self.description = "Start the Visible Autorouter (opens in your web browser)"
        self.show_toolbar_button = True
        icon = os.path.join(HERE, "icon.png")
        if os.path.exists(icon):
            self.icon_file_name = icon

    def Run(self):
        cfg = _config()
        entry = cfg.get("ipc_action", "")
        py = _find_python(cfg)
        if not entry or not os.path.isfile(entry):
            _message("The Visible Autorouter plugin was not found.\n\nRun install_plugin.py again "
                     f"(looked for: {entry or 'no launcher.json'}).")
            return
        if py is None:
            _message("The Visible Autorouter needs the Python environment KiCad builds for it, and it "
                     "is not there yet.\n\n1. Preferences > Preferences > Plugins: tick 'Enable KiCad API'.\n"
                     "2. Restart KiCad and wait about a minute (needs internet once).\n"
                     "3. Still nothing? Run check_install.py and send its output.")
            return
        env = dict(os.environ)
        for k in ("PYTHONHOME", "PYTHONPATH"):     # KiCad's own Python settings would confuse the venv
            env.pop(k, None)
        flags = {"creationflags": 0x08000000} if sys.platform.startswith("win") else {}   # no console
        with open(LOG, "w", encoding="utf-8") as log:
            # working folder: not the plugin folder (Windows would keep it busy for the installer)
            subprocess.Popen([py, "-u", entry], cwd=tempfile.gettempdir(), env=env, stdout=log,
                             stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, close_fds=True, **flags)
        # return at once: KiCad answers the API only after this action has finished


VisibleAutorouterLauncher().register()
