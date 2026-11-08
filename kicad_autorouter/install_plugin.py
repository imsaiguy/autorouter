"""One-step installer for KiCad 10 (and 9).  CLOSE KiCad first, then run:

Windows (uses the Python that ships with KiCad - nothing else to install):
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" install_plugin.py

macOS / Linux:
    python3 install_plugin.py

It
  1. finds the folder KiCad 10 loads plugins from (asks Windows for the real
     Documents folder, so OneDrive-redirected Documents work),
  2. copies this kicad_autorouter folder there,
  3. turns on "Enable KiCad API" (off by default; without it no IPC plugin
     button ever appears),
  4. runs check_install.py to confirm.
Then start KiCad, open a board and wait ~1 minute the first time (KiCad sets up
the plugin's Python environment and downloads kicad-python).
"""
print("Visible Autorouter installer starting...", flush=True)

import json
import os
import platform
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_install  # noqa: E402


def kicad_running() -> bool:
    try:
        if platform.system() == "Windows":
            out = subprocess.run(["tasklist"], capture_output=True, text=True, timeout=20).stdout.lower()
            return any(n in out for n in ("kicad.exe", "pcbnew.exe", "eeschema.exe"))
        out = subprocess.run(["ps", "-A", "-o", "comm="], capture_output=True, text=True, timeout=20).stdout.lower()
        return any(os.path.basename(l.strip()) in ("kicad", "pcbnew", "eeschema") for l in out.splitlines())
    except Exception:
        return False


def pick_version() -> str:
    for v in check_install.VERSIONS:
        _, settings, _ = check_install.candidates(v)
        if any(os.path.isdir(d) for d in settings):
            return v
    return check_install.VERSIONS[0]


def copy_plugin(dest_root: str) -> str:
    dest = os.path.join(dest_root, "kicad_autorouter")
    if os.path.abspath(dest) == HERE:
        print(f"  plugin already in place: {dest}")
        return dest
    os.makedirs(dest_root, exist_ok=True)
    if os.path.isdir(dest):
        shutil.rmtree(dest)
    shutil.copytree(HERE, dest, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    print(f"  copied plugin to: {dest}")
    return dest


def remove_strays(version: str, keep: str) -> None:
    """Other copies of this plugin (from earlier attempts) confuse KiCad: it searches the
    whole plugins tree, and two plugins with the same identifier collide."""
    plugin_dirs, _, _ = check_install.candidates(version)
    strays = []
    # NB: <ver>/scripting/plugins/kicad_autorouter is NOT a stray - it is the classic
    # (SWIG) action plugin, which KiCad 10 still loads and which gives a toolbar button.
    keep_classic = {os.path.abspath(os.path.join(os.path.dirname(d), "scripting", "plugins", "kicad_autorouter"))
                    for d in plugin_dirs}
    for root in plugin_dirs:
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            if os.path.abspath(dirpath) == os.path.abspath(keep) or os.path.abspath(dirpath) in keep_classic:
                dirnames[:] = []
                continue
            if os.path.basename(dirpath) == "kicad_autorouter" and (
                    "plugin.json" in filenames or "router.py" in filenames):
                strays.append(dirpath)
                dirnames[:] = []
    if not strays:
        return
    print("\n  Found other copies of the autorouter from earlier attempts:")
    for d in strays:
        print("    " + d)
    try:
        ans = input("  Remove them? (recommended) [Y/n] ").strip().lower()
    except EOFError:
        ans = "n"
    if ans in ("", "y", "yes"):
        for d in strays:
            shutil.rmtree(d, ignore_errors=True)
            print("    removed " + d)
    else:
        print("  Left in place - if the button still doesn't appear, delete them by hand.")
    print()


def fix_toolbar(settings_dirs) -> None:
    """A customised PCB-editor top toolbar saved without the plugin region hides all plugin
    buttons.  Append the region to the end of that toolbar (everything else unchanged)."""
    sdir = next((d for d in settings_dirs if os.path.isdir(d)), None)
    if not sdir:
        return
    path, state = check_install.toolbar_status(sdir)
    if state != "missing":
        return
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    shutil.copy2(path, path + ".autorouter-backup")
    for tb in data.get("toolbars", []):
        if str(tb.get("name", "")).upper() == "TOP_MAIN":
            tb.setdefault("contents", []).append({"type": "CONTROL", "name": check_install.IPC_CONTROL})
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print(f"  added the plugin-button area to your PCB editor top toolbar ({path};"
          " backup: pcbnew-toolbars.json.autorouter-backup)")


def show_button(settings_dirs) -> None:
    """If 'Show button' was saved as off for our action, switch it on."""
    sdir = next((d for d in settings_dirs if os.path.isdir(d)), None)
    if not sdir:
        return
    path, val = check_install.button_setting(sdir)
    if val is not False:
        return
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    shutil.copy2(path, path + ".autorouter-backup")
    for entry in data["plugins"]["actions"]:
        if isinstance(entry, dict) and check_install.ACTION_ID in entry:
            entry[check_install.ACTION_ID] = True
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print(f"  switched ON the toolbar button for Visible Autorouter ({path}; backup made)")


def enable_api(settings_dirs) -> bool:
    target = next((d for d in settings_dirs if os.path.isdir(d)), None)
    if target is None:
        print("  KiCad settings folder not found - start KiCad once, close it, and run this again.")
        return False
    path = os.path.join(target, "kicad_common.json")
    data = {}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        shutil.copy2(path, path + ".autorouter-backup")
    api = data.setdefault("api", {})
    if api.get("enable_server") is True:
        print(f"  KiCad API already enabled ({path})")
        return True
    api["enable_server"] = True
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print(f"  enabled the KiCad API in {path} (backup: kicad_common.json.autorouter-backup)")
    return True


def step(msg):
    print("  - " + msg, flush=True)


def main() -> int:
    step(f"Python {sys.version.split()[0]} at {sys.executable}")
    step("looking for KiCad's folders...")
    version = pick_version()
    plugin_dirs, settings_dirs, _ = check_install.candidates(version)
    step(f"KiCad {version}: plugins folder {plugin_dirs[0]}")
    step(f"settings folder candidates: {', '.join(settings_dirs)}")
    step("checking whether KiCad is running...")
    running = kicad_running()
    if running:
        print("  KiCad is running. Close ALL KiCad windows first (KiCad rewrites its settings when it\n"
              "  exits, which would undo the API setting), then run this again.")
        return 1
    dest = copy_plugin(plugin_dirs[0])
    # Also install the classic action plugin: KiCad 10 still runs those (if it has its
    # Scripting Console), and it shows a toolbar button even where the new-style
    # (IPC) plugin's button does not appear.
    classic_root = os.path.join(os.path.dirname(plugin_dirs[0]), "scripting", "plugins")
    copy_plugin(classic_root)
    remove_strays(version, dest)
    enable_api(settings_dirs)
    fix_toolbar(settings_dirs)
    show_button(settings_dirs)
    print("\nChecking the installation:\n")
    check_install.main()
    print("\nNow start KiCad and open a board in the PCB editor. The first time, wait about a minute\n"
          "(KiCad builds the plugin's Python environment and downloads kicad-python - needs internet).\n"
          "The 'Visible Autorouter' button then appears on the PCB editor's top toolbar.")
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception:
        import traceback
        print("\nINSTALLER ERROR - please copy everything below:", flush=True)
        traceback.print_exc()
        code = 2
    print(f"\nInstaller finished (exit code {code}).", flush=True)
    sys.exit(code)
