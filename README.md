# DistingNT-CustomAlgos

Custom C++ algorithm plugins for the [Expert Sleepers disting NT](https://www.expert-sleepers.co.uk/distingNT.html), built against the [distingNT_API](https://github.com/expertsleepersltd/distingNT_API) submodule.

## Quick start (new computer)

1. Clone this repo.
2. Double-click **`install.bat`** (or run it from a terminal). It installs everything needed and is safe to re-run.
3. **Open a new terminal window** (so the PATH changes it made are picked up), then:
   ```
   make build
   ```

That's it — see below for writing your own algorithm and pushing it to hardware.

## What `install.bat` sets up

Runs `tools/install.ps1`, which:

- Initializes the `distingNT_API` git submodule (non-recursive — its own nested `airwindows` submodule is unrelated to building custom algorithms and its long file paths fail to clone on Windows)
- Downloads and installs the ARM GNU Toolchain (10.3-2021.10) to `%LOCALAPPDATA%\Programs\gcc-arm-none-eabi\` (no admin rights needed)
- Downloads and installs GNU Make 4.4.1 to `%LOCALAPPDATA%\Programs\make\`
- Adds Git for Windows' `usr\bin` to PATH — GNU Make on Windows needs a real `sh.exe` there to run the Makefile's recipes (`mkdir -p`, `rm -f`, etc.); without it, Make silently falls back to `cmd.exe` and those recipes fail to parse
- Adds all of the above to your **user** PATH permanently (not just this session)
- Installs the Python packages (`mido`, `python-rtmidi`) that `tools/nt_push.py` needs to talk to the module over MIDI

Everything it installs is detected and skipped on a re-run, so it's safe to run again any time (e.g. after a fresh clone on another machine).

Prerequisites `install.bat` does **not** install for you: [Git for Windows](https://git-scm.com/download/win) and [Python 3](https://www.python.org/downloads/) must already be present.

## Repo layout

| Path | What it is |
|---|---|
| `distingNT_API/` | The API submodule — `include/distingnt/api.h` is what you build against; `examples/` has more reference algorithms than `template.cpp` covers |
| `template.cpp` | Minimal starter algorithm (audio passthrough) — copy this to start a new one |
| `Makefile` | `build` / `hardware` / `push` / `clean` targets |
| `tools/nt_push.py` | Sends a built `.o` to a connected disting NT over MIDI SysEx (no SD card removal needed) |
| `tools/install.ps1`, `tools/build.ps1`, `tools/push.ps1` | Logic behind the `.bat` files |
| `install.bat`, `build.bat`, `push.bat`, `clean.bat` | Double-clickable / drag-and-drop entry points |
| `plugins/` | Build output (`.o` files) — gitignored |

## Writing a new algorithm by hand

1. Copy `template.cpp` to a new name, e.g. `mydelay.cpp`.
2. Open it and work through the `TODO`s:
   - Rename `_templateAlgorithm` to something specific to your algorithm.
   - Pick a **GUID unique among your own plugins** — the 4-char code in `NT_MULTICHAR( 'T', 'p', 'l', '1' )`. It doesn't need to be globally unique across every plugin ever written, just distinct from your other installed ones.
   - Set `.name` and `.description` in the `factory` struct.
   - Add your own entries to the `parameters[]` table and the `kParam...` enum (see `distingNT_API/examples/gain.cpp` for a parameter table with linear/dB/custom-scaled values, and pages).
   - Replace the passthrough loop in `step()` with your actual DSP.
3. For anything beyond passthrough — a custom UI (`draw`), reacting to parameter changes (`parameterChanged`), MIDI input (`midiMessage`), or multiple algorithms in one plugin file — look at the other files in `distingNT_API/examples/` (`gainCustomUI.cpp`, `midiLFO.cpp`, `multiple.cpp`, etc.) and the declarations in `distingNT_API/include/distingnt/api.h`.
4. Build it (see below). Fix any compiler errors/warnings — the build uses `-Wall`.

You don't need to touch the `Makefile` — it picks up every `.cpp` in the repo root automatically.

## Building

```
make build
```
Compiles every `.cpp` in the repo root to `plugins/<name>.o` using the same ARM Cortex-M7 cross-compile flags the API's own examples use. `make hardware` is an alias (there's no separate "hardware" build — every build already targets the module's ARM core).

**Or** drag one or more `.cpp` files onto **`build.bat`** to build just those. Double-click `build.bat` with nothing dropped to build everything.

## Pushing to hardware

Connect the disting NT over USB, then:
```
make push
```
This builds first, then transfers every built `.o` to the module over MIDI SysEx (wake → ensure `/programs/plug-ins` exists → upload in 512-byte chunks → rescan plugins) — the same mechanism [nt_helper](https://github.com/thorinside/nt_helper) uses, so no SD card removal is needed. The module's MIDI port is auto-detected (matched by name containing "disting"); override with `make push MIDI_PORT="some other name"` if you have more than one candidate, or `make push SYSEX_ID=n` if your module isn't using SysEx ID 0.

**Or** drag a `.cpp` or `.o` file onto **`push.bat`** — it builds first if you dropped a `.cpp`, then pushes. Double-click with nothing dropped to be prompted for a filename. The MIDI port is auto-detected the same way.

After pushing, the new/updated algorithm shows up at the **very end** of the "Add algorithm" list on the module — it's easy to miss if you're only looking near the top.

## Cleaning

```
make clean
```
or double-click **`clean.bat`**. Removes everything under `plugins/`.

## Troubleshooting

- **`make` or `arm-none-eabi-c++` not found right after running `install.bat`**: open a *new* terminal window (or re-launch from Explorer) — Windows doesn't refresh PATH in already-open shells.
- **Plugin doesn't seem to have installed**: it's likely just at the bottom of the algorithm list, not missing — scroll all the way down. You can also drop a directory-listing check on `/programs/plug-ins` via `tools/nt_push.py` internals if you need to confirm a file landed.
- **`push` can't find a MIDI port**: make sure the module is connected over USB and shows up as a MIDI device (Windows should list something like "disting NT MIDI OUT"/"disting NT MIDI IN"); pass `MIDI_PORT=` explicitly if you have multiple similarly-named MIDI devices.
