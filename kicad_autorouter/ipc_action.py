"""Entry point KiCad 10 runs for the toolbar button (see plugin.json).

KiCad starts this script with the plugin folder as the working directory, in
a private virtual environment with requirements.txt installed.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _load_package():
    """Make "import kicad_autorouter" work whatever this folder is called: the
    installer copies it as .../plugins/kicad_autorouter, but KiCad's Plugin and
    Content Manager installs it under a folder named after the package identifier."""
    if os.path.basename(HERE) == "kicad_autorouter":
        sys.path.insert(0, os.path.dirname(HERE))
        return
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "kicad_autorouter", os.path.join(HERE, "__init__.py"), submodule_search_locations=[HERE])
    mod = importlib.util.module_from_spec(spec)
    sys.modules["kicad_autorouter"] = mod
    spec.loader.exec_module(mod)


_load_package()
from kicad_autorouter.ipc_plugin import main  # noqa: E402

if __name__ == "__main__":
    main()
