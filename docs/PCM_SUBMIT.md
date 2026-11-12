# Publishing to KiCad's Plugin and Content Manager (PCM)

KiCad's PCM (Tools > Plugin and Content Manager) lists the add-ons in the official
repository, built from https://gitlab.com/kicad/addons/metadata. To appear there, a
package is submitted as a merge request to that GitLab project and reviewed by the
KiCad team. Read https://dev-docs.kicad.org/en/addons/ before the first submission.

## 1. Build the package

    python tools/build_pcm.py

This writes into `dist/` (not committed):

| file | what it is for |
|---|---|
| `visible-autorouter-<version>.zip` | the package KiCad downloads |
| `metadata.json` | the submission file (includes download URL, SHA-256, sizes) |
| `icon.png` | 64x64 icon for the submission |

The version comes from `__version__` in `kicad_autorouter/__init__.py`. Every new
release needs a higher version number - bump it before building.

## 2. Publish the zip as a GitHub Release

On https://github.com/imsaiguy/autorouter: Releases > Draft a new release.
- Tag: `v<version>` (e.g. `v0.3.0`) - it must match the `download_url` the build
  script printed: `.../releases/download/v<version>/visible-autorouter-<version>.zip`
- Attach `dist/visible-autorouter-<version>.zip` as a release asset. Do not rebuild
  the zip after this: its SHA-256 is in `dist/metadata.json`.

## 3. Submit to the KiCad metadata repository

1. Make a GitLab account and fork https://gitlab.com/kicad/addons/metadata
2. In your fork, make a branch (not `main`).
3. Add the folder `packages/com.github.imsaiguy.visible-autorouter/` containing
   `dist/metadata.json` and `dist/icon.png`. (The folder name must equal the identifier.)
4. Push. The fork's CI `validate` stage checks the package; the `build` stage gives a
   temporary repository URL. In KiCad: PCM > Manage... > add that URL, then install
   from it and test - exactly what users will get.
5. Open a merge request to `kicad/addons/metadata`. After review and merge, the package
   shows up in everyone's PCM within about a day.

## Updates

Bump `__version__`, rebuild, make a new GitHub Release with the new zip, and add the new
version object (from the new `dist/metadata.json`) to the `versions` list of
`packages/com.github.imsaiguy.visible-autorouter/metadata.json` in a new merge request.
Keep the old version entries.

## Users installing from PCM

KiCad's plugin API must be on: Preferences > Plugins > "Enable KiCad API", then restart
KiCad. (The PowerShell installer does this automatically; PCM does not.) The package
description says so.
