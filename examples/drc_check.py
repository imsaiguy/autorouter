"""Run KiCad's DRC on a board (fills zones first) and summarise the result.

    python3 examples/drc_check.py routed.kicad_pcb
"""
import re
import sys
import tempfile
from collections import Counter

import pcbnew


def drc(path, fill=True, verbose=False):
    b = pcbnew.LoadBoard(path)
    if fill:
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    with tempfile.NamedTemporaryFile(suffix=".rpt", delete=False) as fh:
        rpt = fh.name
    # (KiCad 9 renamed the unit)
    mm = getattr(pcbnew, "EDA_UNITS_MILLIMETRES", None)
    pcbnew.WriteDRCReport(b, rpt, pcbnew.EDA_UNITS_MM if mm is None else mm, True)
    text = open(rpt).read()
    kinds = Counter(re.findall(r"^\[(\w+)\]", text, re.M))
    unconnected = re.search(r"\*\* Found (\d+) unconnected pads", text)
    if verbose:
        print(text)
    return kinds, int(unconnected.group(1)) if unconnected else None, text


if __name__ == "__main__":
    kinds, unc, text = drc(sys.argv[1], verbose="-v" in sys.argv)
    print("DRC violations by type:", dict(kinds) or "none")
    print("Unconnected items:", unc)
