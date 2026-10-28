# build123d.NET

A .NET 10 port of [build123d](https://github.com/gumyr/build123d), the Python CAD library built on
OpenCascade (OCCT). Work in progress. See [docs/design.md](docs/design.md) for the design and the
phase plan.

## Building the native dependencies

OCCT 8.0 and FreeType are built from source by a CMake superbuild in [native/occt](native/occt).
The result lands in `.deps/install/<rid>`.

Requirements: CMake ≥ 3.24 and Ninja. Windows also needs Visual Studio 2022 with the C++ tools.
Linux also needs `libfontconfig1-dev`.

If CMake or Ninja aren't on your PATH, install them into a project-local venv. The Windows script
picks them up from there automatically:

```sh
python -m venv .deps/tools
.deps/tools/Scripts/python -m pip install cmake ninja   # bin/ instead of Scripts/ on Linux/macOS
```

Then build:

```powershell
./native/build-deps.ps1                  # Windows, win-x64 by default
```
```sh
./native/build-deps.sh                   # Linux/macOS, host runtime by default
```
