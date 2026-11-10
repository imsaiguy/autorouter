"""Entry point KiCad 10 runs for the toolbar button (see plugin.json).

KiCad starts this script with the plugin folder as the working directory, in
a private virtual environment with requirements.txt installed.
"""
import os
import sys

# the folder that CONTAINS kicad_autorouter/, so "import kicad_autorouter" works
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_autorouter.ipc_plugin import main  # noqa: E402

if __name__ == "__main__":
    main()
