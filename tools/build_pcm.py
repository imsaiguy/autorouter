"""Build the KiCad Plugin and Content Manager (PCM) package.

    python tools/build_pcm.py

Writes into dist/:
  visible-autorouter-<version>.zip   the package: upload it as a GitHub Release asset
  metadata.json                      for the submission: packages/<identifier>/metadata.json
  icon.png                           for the submission: packages/<identifier>/icon.png

The zip holds metadata.json (this version only, no download fields), plugins/ (the
plugin: plugin.json, ipc_action.py, the package) and resources/icon.png (64x64).
The submission metadata.json is the same plus download_url / download_sha256 /
download_size / install_size for this zip.
"""
import hashlib
import io
import json
import os
import re
import sys
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "kicad_autorouter")
DIST = os.path.join(ROOT, "dist")
REPO = "https://github.com/imsaiguy/autorouter"
IDENT = "com.github.imsaiguy.visible-autorouter"
KICAD = "10.0"

# not part of the PCM package: the PowerShell installer and the classic (SWIG) button
# it adds, scripts for running outside KiCad, and caches
SKIP_FILES = {"install_plugin.py", "check_install.py", "run_autorouter.bat", "run_autorouter.sh"}
SKIP_DIRS = {"__pycache__", "launcher"}
SKIP_EXT = {".pyc", ".pyo"}

DESCRIPTION_FULL = """\
A multi-pass, negotiated-congestion autorouter for KiCad 10 that shows its thinking.

Select some footprints or nets (or nothing, for the whole board) and press the toolbar \
button: a window opens in your browser and you watch the router search, rip up and \
re-route pass by pass until every net fits. Then it straightens and tidies the result \
- traces leave pads square from the centre, 45-degree bends, no right angles - and \
writes everything into KiCad as ONE undo step.

Features:
- Routes the selection or all nets; keeps or rips up existing tracks
- Ground and supply nets (GND, VCC, +5V, -12V ...) get priority
- Fine-pitch QFN/TQFP pins: square, centred pad exits and twin-pin links
- Respects the board's net classes, clearances, keep-outs and pours
- Stop at any time and keep or discard what was routed

Before first use: KiCad needs its plugin API switched on - Preferences > Plugins > \
'Enable KiCad API'. Then restart KiCad; the button appears on the PCB editor toolbar.

The fast search core is compiled for Windows and Linux; on macOS the router runs \
in pure Python (slower).
"""


def version() -> str:
    with open(os.path.join(PKG, "__init__.py"), encoding="utf-8") as fh:
        return re.search(r'__version__\s*=\s*"([^"]+)"', fh.read()).group(1)


def icon_64() -> bytes:
    from PIL import Image
    im = Image.open(os.path.join(PKG, "icon@2x.png")).convert("RGBA")
    im = im.resize((64, 64), Image.LANCZOS)
    buf = io.BytesIO()
    im.save(buf, "PNG")
    return buf.getvalue()


def metadata(ver: str) -> dict:
    return {
        "$schema": "https://go.kicad.org/pcm/schemas/v1",
        "name": "Visible Autorouter",
        "description": "Autorouter that shows its thinking: watch it route, then tidy, your board",
        "description_full": DESCRIPTION_FULL,
        "identifier": IDENT,
        "type": "plugin",
        "author": {"name": "imsaiguy", "contact": {"web": REPO}},
        "maintainer": {"name": "imsaiguy", "contact": {"web": REPO}},
        "license": "MIT",
        "resources": {"homepage": REPO, "Issues": REPO + "/issues"},
        "tags": ["pcbnew", "routing", "autorouter", "router"],
        "versions": [{
            "version": ver,
            "status": "stable",
            "kicad_version": KICAD,
            "runtime": "ipc",
        }],
    }


def files():
    for dp, dns, fns in os.walk(PKG):
        dns[:] = sorted(d for d in dns if d not in SKIP_DIRS)
        for f in sorted(fns):
            if f in SKIP_FILES or os.path.splitext(f)[1] in SKIP_EXT:
                continue
            full = os.path.join(dp, f)
            yield full, os.path.relpath(full, PKG).replace(os.sep, "/")


def main() -> int:
    ver = version()
    os.makedirs(DIST, exist_ok=True)
    meta = metadata(ver)
    icon = icon_64()
    name = f"visible-autorouter-{ver}.zip"
    zpath = os.path.join(DIST, name)
    install = 0
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("metadata.json", json.dumps(meta, indent=2) + "\n")
        install += len(json.dumps(meta, indent=2)) + 1
        z.writestr("resources/icon.png", icon)
        install += len(icon)
        for full, rel in files():
            z.write(full, "plugins/" + rel)
            install += os.path.getsize(full)
    data = open(zpath, "rb").read()
    v = meta["versions"][0]
    v["download_url"] = f"{REPO}/releases/download/v{ver}/{name}"
    v["download_sha256"] = hashlib.sha256(data).hexdigest()
    v["download_size"] = len(data)
    v["install_size"] = install
    with open(os.path.join(DIST, "metadata.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=2)
        fh.write("\n")
    with open(os.path.join(DIST, "icon.png"), "wb") as fh:
        fh.write(icon)
    print(f"package   {zpath}  ({len(data)} bytes, installs {install} bytes)")
    print(f"sha256    {v['download_sha256']}")
    print(f"upload as {v['download_url']}")
    print(f"submit    dist/metadata.json + dist/icon.png -> packages/{IDENT}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
