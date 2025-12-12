# SDR++ Copilot Instructions

Purpose: Enable AI coding agents to be immediately productive in this C++/CMake, cross‑platform SDR application by capturing the project's architecture, workflows, and conventions.

## Big Picture
- **Core app:** GUI + DSP runtime under `src/` and `core/`; the executable is `sdrpp` built from `src/main.cpp`. The core loads modules dynamically based on `root/config.json`.
- **Modular design:** Plugins are shared libraries built into subfolders:
  - `source_modules/` (radio front‑ends, e.g., `rtl_sdr_source`, `spyserver_source`)
  - `sink_modules/` (audio/network sinks)
  - `decoder_modules/` (signal decoders, e.g., `meteor_demodulator`, `m17_decoder`)
  - `misc_modules/` (recorder, frequency manager, scanner, etc.)
- **Resources + runtime config:** Development runs use a dedicated root folder (`root_dev`) containing `res/` and `modules/` with `config.json` pointing to built `.dll`/`.so` paths.
- **External DSP libs:** Uses FFTW, VOLK (SIMD), RtAudio/PortAudio; FEC via `core/libcorrect/`.
- **Why modular:** Keeps core lightweight, lets large/optional HW dependencies live in per‑module builds toggled via CMake options described in README.

## Build & Run (Windows)
- **Prereqs:** `cmake`, `vcpkg`, and **PothosSDR** installed at `C:\Program Files\PothosSDR` for broad SDR driver coverage.
- **Install packages with vcpkg:** `fftw3`, `glfw3`, `rtaudio`, `zstd` (use `:x64-windows`).
- **Configure + build:**
  - `mkdir build; cd build`
  - `cmake .. "-DCMAKE_TOOLCHAIN_FILE=<vcpkg_dir>/scripts/buildsystems/vcpkg.cmake" -G "Visual Studio 16 2019"`
  - `cmake --build . --config Release`
- **Dev run (top directory):** `./build/Release/sdrpp.exe -r root_dev -c`
- **Root setup:** Run `create_root.bat`, then edit `root_dev/config.json`:
  - Add module DLL paths, e.g. `./build/radio/Release/radio.dll`, `./build/audio_sink/Release/audio_sink.dll`.
  - Set `modulesDirectory` to `root_dev/modules` and `resourcesDirectory` to `root_dev/res`.

## Build & Run (Linux/BSD)
- **Deps:** `cmake`, `fftw3`, `glfw`, `libvolk`, `zstd` (+ per‑module deps).
- **Configure + build:** `mkdir build; cd build; cmake ..; make -j<N>`
- **Dev run:** `./build/sdrpp -r root_dev`
- **Root setup:** `sh ./create_root.sh`, then update `root_dev/config.json` with `.so` module paths.

## MacOS Notes
- Prefer nightly bundle; building requires enabling `-DUSE_BUNDLE_DEFAULTS=ON` and using PortAudio sinks (`OPT_BUILD_PORTAUDIO_SINK`/`OPT_BUILD_NEW_PORTAUDIO_SINK`). See `readme.md` Mac section for full example.

## CMake Conventions
- **Options control modules:** Each module has `OPT_BUILD_<MODULE>=ON|OFF` (see "Module List" in `readme.md`). Default set avoids heavy or niche deps.
- **Module outputs:** Built under `build/<module>/<config>/` on Windows and `build/<module>/` on Linux/Mac; names match folder (e.g., `recorder.dll` / `recorder.so`).
- **Top‑level:** `CMakeLists.txt` drives core + modules; per‑module CMake files live in their respective directories. Shared helper: `sdrpp_module.cmake`.

## Testing & CI
- **Unit tests:** In `tests/` with `CMakeLists.txt` and `dsp_test.cpp`. Build tests via CMake; run from build tree. If build fails, verify required test deps are present.
- **Nightly builds:** GitHub Actions pipeline creates artifacts; use as reference for supported modules and platform flags.

## Patterns & Code Style
- **SIMD paths:** FFT/DSP implementations provide architecture‑specific files (e.g., `build/fft_mt_r2iq_avx*.cpp`, `..._neon.cpp`). Guard additions with feature detection and keep fallback `*_def.cpp`.
- **Dynamic modules:** Core discovers modules through `config.json`; do not hardcode module lists in core. Add new modules under the appropriate `*_modules/` folder with a CMake option and ensure their shared library name matches the folder.
- **Resources:** UI/resources resolved via `resourcesDirectory`; new assets go under `root/res` and should be referenced relative to that value.
- **Cross‑platform:** Avoid platform‑specific APIs in core; isolate in per‑module implementations with conditional compilation.

## Integration Points
- **Hardware backends:** Airspy, HackRF, BladeRF, LimeSDR, RTL‑SDR, SDRPlay, UHD, etc. Each has a dedicated source module with its own dependency set.
- **Network I/O:** `sdrpp_server_source` and `network_sink` handle streaming; `spyserver_source` integrates with external servers.
- **Audio:** Default on Windows uses RtAudio sinks; Mac uses PortAudio sinks.

## Common Workflows
- **Add a new module:**
  - Create folder under `source_modules/` or `sink_modules/` etc.
  - Add CMake option `OPT_BUILD_<NAME>` and module `CMakeLists.txt` using `sdrpp_module.cmake`.
  - Output a shared library named `<name>.(dll|so)` and document required deps in README’s module table.
  - Update `root_dev/config.json` with the built path and test via dev run.

## Reference Files
- `readme.md`: Build/run instructions, module list, troubleshooting.
- `src/main.cpp`: App entry and core wiring.
- `sdrpp_module.cmake`: Shared module build helper.
- `root/`: Example `modules/` and `res/` layout.
- `core/libcorrect/`: FEC library used by decoders.
- `tests/`: Unit tests scaffold (CMake + examples).

If anything is unclear or missing (e.g., test invocation details for your platform, module scaffolding examples, or specific CI flags you need), tell me what you're trying to do and I will refine these instructions.
