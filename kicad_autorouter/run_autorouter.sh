#!/bin/sh
# Start the Visible Autorouter by hand (KiCad must be open with a board in the PCB editor).
HERE="$(cd "$(dirname "$0")" && pwd)"
ID=com.github.imsaiguy.visible-autorouter
for base in "$HOME/Library/Caches/KiCad" "${XDG_CACHE_HOME:-$HOME/.cache}/kicad" "$HOME/.var/app/org.kicad.KiCad/cache/kicad"; do
  for v in 10.0; do
    py="$base/$v/python-environments/$ID/bin/python"
    [ -x "$py" ] && exec "$py" "$HERE/ipc_action.py"
  done
done
echo "Plugin environment not found - start KiCad once with the KiCad API enabled." >&2
exit 1
