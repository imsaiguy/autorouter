@echo off
rem Double-click to start the Visible Autorouter (KiCad must be open with a board in the PCB editor).
rem Uses the Python environment KiCad built for the plugin; falls back to KiCad's own Python.
setlocal
set "HERE=%~dp0"
for %%V in (10.0) do (
  if exist "%LOCALAPPDATA%\KiCad\%%V\python-environments\com.github.imsaiguy.visible-autorouter\Scripts\python.exe" (
    "%LOCALAPPDATA%\KiCad\%%V\python-environments\com.github.imsaiguy.visible-autorouter\Scripts\python.exe" "%HERE%ipc_action.py"
    goto :done
  )
)
echo The plugin's Python environment was not found.
echo Start KiCad once (with the KiCad API enabled) so it can build it, then try again.
pause
:done
endlocal
