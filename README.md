# Visible Autorouter for KiCad

A multi-pass PCB autorouter that runs **inside KiCad** (toolbar button) or from the
command line, routes **all nets or just the ones you select**, and **shows its
thinking** while it works.

![The router negotiating a scrambled 10-pin crossing](docs/crossing.gif)

*Pass 1: every net takes its own shortest path and nets pile on top of each other
(yellow squares). Each later pass raises the price of contested copper. The
orange heat is the congestion "memory" the router has learned. Nets that have
alternatives move away; the curve on the right shows the overuse falling to
zero.*

## Get it

```bash
git clone https://github.com/imsaiguy/autorouter.git
```

or use **Code ▸ Download ZIP** on https://github.com/imsaiguy/autorouter. The
plugin is the `kicad_autorouter` folder; installation is described below.

## What it does

* **Three scopes:**
  * **Unrouted only** (the default): finishes just the missing connections,
    i.e. the ratsnest. **Existing tracks and vias are never moved or
    removed.** Partly routed nets get only their missing links, and new
    copper joins onto what's there.
  * **Selected:** the nets of the selected footprints, pads or tracks.
  * **All nets.**

  For *Selected* and *All* you can tick **Remove existing tracks** to re-route
  those nets from scratch. Otherwise existing copper is kept: connected pieces
  count as one terminal, and other nets' tracks are obstacles.
* **Sees your existing work.** When it loads the design it shows existing
  tracks and vias (muted, so new routes stand out) and a summary such as
  *"8 routed · 1 partly routed · 2 unrouted"*. If a net can't be finished
  around fixed copper, it says which net and how to free it.
* **Multiple passes.** It uses the negotiated-congestion "PathFinder" algorithm
  with A* search per connection, which is the standard method for FPGA routing.
* **Visible thinking.** You can watch the A* wavefront, tentative tracks,
  overused cells, the learned congestion heatmap, and a convergence curve,
  either in the plugin's live panel, in a matplotlib window, or as a GIF.
* **Pours are part of the negotiation.** GND/power nets with a copper zone are
  left to the pour. SMD pads that are not on the pour layer get a short stub
  plus a via down to it. After each pass the router estimates the pour,
  detects pads or vias stranded on islands, and pushes the tracks that cut
  them off elsewhere.
* **Checked with KiCad's own DRC.** On the demo board the result has
  **0 clearance, short, edge or hole violations and 0 unconnected items**.
  The test suite runs that check.

![Demo board routed (KiCad 7 test board, 12 nets, 2 layers)](docs/demo_board.png)

## How it thinks

The board becomes a grid whose pitch is `track width + clearance`. Tracks run
along cell centres, so *one cell = one net* **is** the clearance rule. Two extra
rules keep that true everywhere else:

* a 45° step also claims the two cells it squeezes past (it passes only 0.707
  pitch from their centres);
* a via claims every cell within `via radius + clearance + track width/2`, on
  every layer.

Pads, existing tracks and the board edge give each cell an owner: free, "only
net *n*" (inside *n*'s clearance halo), or blocked.

Each pass then does this:

```
for each net that is in conflict:              (all nets in pass 1)
    rip up its old route
    grow a tree from one pad; repeatedly A* to the nearest unconnected pad
        cost of a cell = (1 + history) × (1 + present_factor × other_nets_in_cell)
        × direction factor (F.Cu prefers horizontal, B.Cu vertical) + bend + via costs

after the pass:
    overused cell  = used by 2+ nets
    history       += history_factor × overuse        (remembers chronic hot spots)
    present_factor ×= 1.6                             (contention gets pricier every pass)
    pour check: a pour pad/via cut off onto an island → its walls get pricier too
stop when nothing is overused
```

This works like a market. Copper is a shared resource and its price rises with
demand. Nets with cheap alternatives leave the contested cells. Nets with no
alternative stay, because everyone else has learned to avoid that spot. The
present-cost factor only ever rises, so it also resembles annealing: the
system gradually "freezes" into a legal solution. If it hasn't converged after
`max_passes`, the most-conflicted nets are dropped and reported, so the output
is always DRC-legal.

## Install in KiCad

The same `kicad_autorouter` folder contains two plugins. KiCad picks the right
one depending on which folder you copy it into.

### KiCad 10 (and 9): IPC API plugin, recommended

1. **Turn the KiCad API on.** It is **off by default**. Go to
   **Preferences ▸ Preferences ▸ Plugins** and tick the option that enables
   the KiCad API.
2. **Copy the `kicad_autorouter` folder** (the whole folder, not its contents)
   into KiCad's **`plugins`** folder. Create the folder if it doesn't exist.

   | OS | folder |
   |---|---|
   | Windows | `Documents\KiCad\10.0\plugins\` |
   | macOS | `~/Documents/KiCad/10.0/plugins/` |
   | Linux | `~/.local/share/kicad/10.0/plugins/` |

   You should end up with `.../10.0/plugins/kicad_autorouter/plugin.json`.
   **Note: this is `plugins`, not `scripting/plugins`.** Subfolders are fine.
3. **Restart KiCad.** The first time, KiCad builds a small Python environment
   for the plugin and installs `kicad-python` from `requirements.txt`. That
   needs internet access once and can take a minute before the button becomes
   active.
4. Open a board. The **Visible Autorouter** button (two crossing traces) is on
   the PCB editor's toolbar.

Click it and your web browser opens the router's window. It shows your board
with its existing tracks and a routed / partly routed / unrouted summary.
Choose **Unrouted only** (the default), **Selected**, or **All nets**, check
the rules (they come from the Default net class), and press **Route**. The
board view shows the router thinking, and the chart shows conflicts dropping
pass by pass.

When it finishes, the tracks and vias go onto your board as **one undo step**
(Edit ▸ Undo removes them all), and zones are refilled. Press **Close** when
you're done.

![KiCad 10 plugin window: "Unrouted only" just finished 3 nets (bright) around the existing copper (muted)](docs/kicad10_ui.png)

### KiCad 7 / 8: classic action plugin

1. In the PCB editor: **Tools ▸ External Plugins ▸ Open Plugin Directory**.
   This is the `scripting/plugins` folder.
2. Copy the `kicad_autorouter` folder there, then **restart KiCad** (or use
   **Tools ▸ External Plugins ▸ Refresh Plugins**).
3. Use the toolbar button, or **Tools ▸ External Plugins ▸ Visible Autorouter**.
   The dialog opens showing existing tracks and the routed / partly / unrouted
   summary, with the same three scopes (Unrouted only is the default).

![KiCad 7 dialog mid-route (pass 4)](docs/plugin_dialog.png)

### Button not showing?

Run the checker. It needs only the standard library and works with any
Python 3:

```bash
python kicad_autorouter/check_install.py
```

It checks the four conditions KiCad 10 needs before it shows an IPC plugin
button (taken from KiCad 10.0's source) and prints what to fix:

1. **The plugin folder.** `plugin.json` must be under `.../10.0/plugins/`;
   KiCad searches subfolders, so ZIP nesting is fine. `scripting/plugins` is
   the old KiCad 7/8 location.
2. **The KiCad API is enabled.** It is off by default: Preferences ▸
   Preferences ▸ Plugins.
3. **A usable Python interpreter.** Windows and macOS use KiCad's bundled
   Python. Linux uses `python3` and needs the `venv` module
   (`sudo apt install python3-venv` on Debian/Ubuntu). On **Linux with
   Python 3.14**, `kicad-python`'s dependencies have no prebuilt wheels yet,
   so point KiCad at Python 3.11–3.13.
4. **The plugin environment is built.** At startup KiCad creates a virtual
   environment and pip-installs `kicad-python` (needs internet once). **The
   button only appears after this succeeds**, so after changing anything,
   **restart KiCad and wait a minute**. *Refresh Plugins* does not rebuild
   it.

KiCad 7/8: see **Tools ▸ External Plugins ▸ Show Plugin Errors** (or the
Scripting Console).

## Command line (no KiCad GUI needed)

```bash
python -m kicad_autorouter board.kicad_pcb -o routed.kicad_pcb
python -m kicad_autorouter board.kicad_pcb --nets SDA,SCL           # only these nets
python -m kicad_autorouter board.kicad_pcb --gif thinking.gif       # record the thinking
python -m kicad_autorouter board.kicad_pcb --live                   # watch in a window
```

The CLI reads the `.kicad_pro` next to the board for the Default net class.
Options include `--track-width --clearance --via-diameter --via-drill --grid
--layers F.Cu,B.Cu --passes --via-cost --include-zone-nets`. The core is pure
Python with no dependencies. Only `--gif/--png/--live` need matplotlib (and
Pillow for GIFs).

As a library:

```python
from kicad_autorouter import route, RouteSettings
from kicad_autorouter.kicad_pcb import PcbFile

pcb = PcbFile("board.kicad_pcb")
result = route(pcb.board(), pcb.settings_from_project(), nets=["SDA", "SCL"],
               callback=lambda ev, data: print(ev, data.get("pass")))
pcb.add_routes(result, pcb.settings_from_project()); pcb.save("routed.kicad_pcb")
```

## Layout

| file | what |
|---|---|
| `kicad_autorouter/model.py` | board model (pads, tracks, vias, outline) in mm, plus `RouteSettings` |
| `kicad_autorouter/problem.py` | board → grid, clearance halos, terminal clustering, pour anchors |
| `kicad_autorouter/router.py` | A*, PathFinder passes, pour check, events, geometry output |
| `kicad_autorouter/viz.py` | matplotlib live view / GIF / PNG |
| `kicad_autorouter/kicad_pcb.py` | `.kicad_pcb` reader and writer (KiCad 6–9 S-expression format) |
| `kicad_autorouter/plugin.json`, `ipc_action.py`, `requirements.txt` | KiCad 9/10 IPC plugin manifest and entry point |
| `kicad_autorouter/check_install.py` | "why no button?" checker for KiCad 9/10 installs |
| `kicad_autorouter/ipc_plugin.py`, `ipc_ui.html` | KiCad 9/10 adapter (kicad-python), browser thinking view, single-commit writeback |
| `kicad_autorouter/plugin.py` | KiCad 7/8 SWIG action plugin: wx dialog, thinking panel, pcbnew adapter |
| `examples/make_test_board.py` | builds the demo board with KiCad's own `pcbnew` API |
| `examples/drc_check.py` | fills zones and runs KiCad's DRC on a board |
| `tests/` | exact-geometry clearance and connectivity checks, KiCad DRC test, and IPC-adapter tests on real `kipy` objects |

```bash
python -m pytest tests        # the DRC test runs when pcbnew is importable
```

## Status and honest limitations

What's been tested:

* **KiCad 7.0.11 on Linux:** the file reader/writer, the `pcbnew` adapter, and
  the wx dialog itself (driven automatically under a virtual display).
* **KiCad 10 path:** the manifest validates against KiCad 10.0's own plugin
  schema, and the adapter is built against `kicad-python` 0.8 (the KiCad
  10.0.6 API). Tests drive it with real `kipy` objects: reading pads, the
  selection and zones, building tracks and vias, one commit. The browser view
  was tested in Chromium, and the output passes KiCad's DRC. **It has not yet
  been run inside a live KiCad 10.** If anything misbehaves, the plugin's
  error text appears in KiCad's plugin messages.

* One track width and clearance for all routed nets (the Default net class).
  Per-net-class widths are not supported yet.
* Grid router. Very fine-pitch parts (≤ 0.5 mm) may be hard to escape; try a
  smaller `--grid`. Off-grid pads are reached with a short stub to the pad
  centre.
* Through vias only. No differential pairs, no length tuning.
* **Rule areas / keepouts are not read yet.** Check DRC if your board uses them.
* KiCad 7/8 plugin and CLI: custom and trapezoid pads are treated as their
  bounding rectangle (conservative). The KiCad 10 plugin uses KiCad's exact
  pad polygons.
* The pour estimate is approximate. Refill the zones and run DRC afterwards
  (the plugin refills for you).
* Pure Python. About 3 s for the 12-net demo, about 20 s for 34 random nets on
  a 100 × 70 mm board.

## Ideas for next steps

* Live preview on the KiCad 10 canvas itself, in addition to the browser view.
* Per-net-class widths and rule areas.
* A C/Rust inner loop for the A* search.
* A post-route "push-and-shove" cleanup pass to straighten tracks and remove
  extra vias.
