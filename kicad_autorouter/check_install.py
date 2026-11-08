"""Why is there no Visible Autorouter button?  Run:   python check_install.py

Checks every condition KiCad 10 (and 9) needs before it shows an IPC plugin's
toolbar button.  Standard library only - run it with any Python 3.
The locations below come from KiCad 10.0's source (common/paths.cpp,
common/api/api_plugin_manager.cpp, scripting/python_manager.cpp).
"""
import json
import os
import platform
import subprocess
import sys

PLUGIN_ID = "com.github.imsaiguy.visible-autorouter"
VERSIONS = ("10.0", "9.0")
OK, BAD, INFO = "  [ OK ]", "  [FAIL]", "  [info]"


def home(*p):
    return os.path.join(os.path.expanduser("~"), *p)


def windows_documents():
    """The real Documents folder (what KiCad uses) - follows OneDrive/redirection."""
    try:
        import ctypes
        from ctypes import wintypes

        class GUID(ctypes.Structure):
            _fields_ = [("Data1", wintypes.DWORD), ("Data2", wintypes.WORD),
                        ("Data3", wintypes.WORD), ("Data4", wintypes.BYTE * 8)]
        # FOLDERID_Documents {FDD39AD0-238F-46AF-ADB4-6C85480369C7}
        fid = GUID(0xFDD39AD0, 0x238F, 0x46AF, (wintypes.BYTE * 8)(0xAD, 0xB4, 0x6C, 0x85, 0x48, 0x03, 0x69, 0xC7))
        out = ctypes.c_wchar_p()
        if ctypes.windll.shell32.SHGetKnownFolderPath(ctypes.byref(fid), 0, None, ctypes.byref(out)) == 0:
            path = out.value
            ctypes.windll.ole32.CoTaskMemFree(out)
            return path
    except Exception:
        pass
    return None


def candidates(version):
    sysname = platform.system()
    docs = os.environ.get("KICAD_DOCUMENTS_HOME")
    if sysname == "Windows":
        appdata = os.environ.get("APPDATA", home("AppData", "Roaming"))
        local = os.environ.get("LOCALAPPDATA", home("AppData", "Local"))
        documents = [docs] if docs else [d for d in (windows_documents(), home("OneDrive", "Documents"),
                                                     home("Documents")) if d]
        documents = list(dict.fromkeys(documents))
        plugin_dirs = [os.path.join(d, "KiCad", version, "plugins") for d in documents]
        settings = [os.path.join(appdata, "kicad", version), os.path.join(appdata, "KiCad", version)]
        cache = [os.path.join(local, "KiCad", version), os.path.join(local, "kicad", version)]
    elif sysname == "Darwin":
        documents = [docs] if docs else [home("Documents")]
        plugin_dirs = [os.path.join(d, "KiCad", version, "plugins") for d in documents]
        settings = [home("Library", "Preferences", "kicad", version)]
        cache = [home("Library", "Caches", "KiCad", version), home("Library", "Caches", "kicad", version)]
    else:
        data = docs or os.environ.get("XDG_DATA_HOME") or home(".local", "share")
        plugin_dirs = [os.path.join(data, "kicad", version, "plugins")]
        cfg = os.environ.get("XDG_CONFIG_HOME") or home(".config")
        settings = [os.path.join(cfg, "kicad", version)]
        cch = os.environ.get("KICAD_CACHE_HOME") or os.environ.get("XDG_CACHE_HOME") or home(".cache")
        cache = [os.path.join(cch, "kicad", version)]
        # Flatpak KiCad keeps everything inside its sandbox
        fp = home(".var", "app", "org.kicad.KiCad")
        plugin_dirs.append(os.path.join(fp, "data", "kicad", version, "plugins"))
        settings.append(os.path.join(fp, "config", "kicad", version))
        cache.append(os.path.join(fp, "cache", "kicad", version))
    return plugin_dirs, settings, cache


def find_plugin_json(root):
    hits = []
    for dirpath, dirnames, filenames in os.walk(root):
        if "plugin.json" in filenames:
            p = os.path.join(dirpath, "plugin.json")
            try:
                with open(p, encoding="utf-8") as fh:
                    ident = json.load(fh).get("identifier")
            except Exception as exc:
                ident = f"<unreadable: {exc}>"
            hits.append((p, ident))
    return hits


def main():
    print(f"Visible Autorouter install check  (OS: {platform.system()} {platform.release()}, "
          f"this Python: {sys.version.split()[0]})\n")
    problems = []
    found_version = None
    for version in VERSIONS:
        plugin_dirs, settings_dirs, cache_dirs = candidates(version)
        existing = [d for d in settings_dirs if os.path.isdir(d)]
        if not existing and not any(os.path.isdir(d) for d in plugin_dirs):
            continue
        found_version = version
        print(f"KiCad {version}")

        # 1. plugin folder
        print(" 1. Plugin folder")
        ours = []
        for d in plugin_dirs:
            if os.path.isdir(d):
                print(f"{INFO} plugins folder: {d}")
                for p, ident in find_plugin_json(d):
                    rel = os.path.relpath(p, d)
                    print(f"{INFO}   found {rel}  (identifier: {ident})")
                    if ident == PLUGIN_ID:
                        ours.append((d, p))
        if ours:
            d, p = ours[0]
            depth = len(os.path.relpath(p, d).split(os.sep))
            print(f"{OK} plugin.json found: {p}")
            if depth > 2:
                print(f"{INFO}   (nested {depth - 1} folders deep - fine, KiCad searches subfolders)")
            for f in ("ipc_action.py", "requirements.txt", "icon.png", "ipc_plugin.py", "ipc_ui.html"):
                if not os.path.exists(os.path.join(os.path.dirname(p), f)):
                    print(f"{BAD} missing {f} next to plugin.json - copy the WHOLE kicad_autorouter folder")
                    problems.append(f"missing {f}")
        else:
            print(f"{BAD} plugin.json for the autorouter not found in: {', '.join(plugin_dirs)}")
            scripting = os.path.join(os.path.dirname(plugin_dirs[0]), "scripting", "plugins")
            if os.path.isdir(os.path.join(scripting, "kicad_autorouter")):
                print(f"{INFO}   It IS in {scripting} - that is the OLD (KiCad 7/8) location.")
            problems.append("copy the kicad_autorouter folder into " + plugin_dirs[0])

        # 2. settings
        print(" 2. KiCad API settings (Preferences > Plugins)")
        interp = ""
        cfg_file = next((os.path.join(d, "kicad_common.json") for d in settings_dirs
                         if os.path.exists(os.path.join(d, "kicad_common.json"))), None)
        if cfg_file:
            try:
                with open(cfg_file, encoding="utf-8") as fh:
                    api = json.load(fh).get("api", {})
            except Exception as exc:
                api = {}
                print(f"{BAD} could not read {cfg_file}: {exc}")
            if api.get("enable_server"):
                print(f"{OK} KiCad API is enabled")
            else:
                print(f"{BAD} KiCad API is OFF (default). Preferences > Preferences > Plugins > "
                      "enable the KiCad API, then restart KiCad.")
                problems.append("enable the KiCad API")
            interp = api.get("interpreter_path", "") or ""
            print(f"{INFO} Python interpreter set in KiCad: {interp or '(empty - KiCad auto-detects)'}")
        else:
            print(f"{INFO} kicad_common.json not found in {settings_dirs} (has this KiCad been run?)")

        # 3. interpreter
        print(" 3. Python interpreter KiCad uses for plugins")
        if interp and os.path.exists(interp):
            try:
                out = subprocess.run([interp, "-c", "import sys,venv;print(sys.version.split()[0])"],
                                     capture_output=True, text=True, timeout=30)
                if out.returncode == 0:
                    ver = out.stdout.strip()
                    print(f"{OK} {interp} runs (Python {ver}, venv module present)")
                    if platform.system() == "Linux" and ver.startswith("3.14"):
                        print(f"{BAD} Python 3.14 on Linux: kicad-python's dependencies have no prebuilt "
                              "wheels yet, and KiCad only installs wheels. Point KiCad at a Python "
                              "3.11-3.13 interpreter in Preferences > Plugins.")
                        problems.append("use Python 3.11-3.13 for KiCad plugins")
                else:
                    print(f"{BAD} {interp} failed: {out.stderr.strip()[:300]}")
                    if "venv" in out.stderr:
                        print(f"{INFO}   On Debian/Ubuntu install it with: sudo apt install python3-venv")
                    problems.append("fix the Python interpreter (needs the venv module)")
            except Exception as exc:
                print(f"{BAD} could not run {interp}: {exc}")
        elif interp:
            print(f"{BAD} configured interpreter does not exist: {interp}")
            problems.append("set a valid Python interpreter in Preferences > Plugins")
        else:
            print(f"{INFO} auto-detect (Windows/macOS: KiCad's bundled Python; Linux: python3 on PATH)")

        # 4. environment
        print(" 4. Plugin Python environment (KiCad builds this, then shows the button)")
        envs = [os.path.join(c, "python-environments", PLUGIN_ID) for c in cache_dirs]
        env = next((e for e in envs if os.path.isdir(e)), None)
        if not env:
            print(f"{BAD} not created yet (looked in {', '.join(envs)}).")
            print(f"{INFO}   KiCad creates it at startup when the API is enabled and it finds plugin.json.")
            problems.append("restart KiCad after fixing the above, then wait ~1 minute")
        else:
            has_cfg = os.path.exists(os.path.join(env, "pyvenv.cfg"))
            kipy = [r for r, dnames, _ in os.walk(env) if os.path.basename(r) == "kipy"]
            print(f"{OK if has_cfg else BAD} environment: {env}" + ("" if has_cfg else " (pyvenv.cfg missing)"))
            if kipy:
                print(f"{OK} kicad-python is installed - the plugin should be READY")
            else:
                print(f"{BAD} kicad-python NOT installed in it: the pip step failed or is still running.")
                print(f"{INFO}   Needs internet access once. Check Preferences > Plugins for an error, "
                      "and the Python version note above.")
                problems.append("kicad-python could not be installed into the plugin environment")
        print()

    if not found_version:
        print("No KiCad 9/10 user folders found. Has KiCad been started at least once on this account?")
        return 1
    if problems:
        print("TO FIX:")
        for i, p in enumerate(dict.fromkeys(problems), 1):
            print(f"  {i}. {p}")
        print("\nThen RESTART KiCad (Refresh Plugins does not rebuild IPC plugin environments).")
    else:
        print("Everything looks right. If there is still no button, check Preferences > PCB Editor > "
              "Plugins (button visibility), and that your toolbar has not been customized to drop "
              "the plugin area.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
