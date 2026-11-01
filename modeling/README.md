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

## Building the bindings

The OCCT bindings are generated from the whitelist [bindings/occt.txt](bindings/occt.txt). The
generated C++ and C# are committed, so you only need to regenerate after editing the whitelist:

```powershell
./tools/generate-bindings.ps1            # parses the OCCT headers from .deps/install
```

Build the native shim (`b3docct`), then the .NET solution:

```powershell
./native/build-shim.ps1                  # Windows; ./native/build-shim.sh <rid> on Linux/macOS
dotnet test
```

`Build123d.Occt` copies the OCCT, FreeType and shim libraries for the host runtime into the output
directory of anything that references it.

## Packages

```powershell
./tools/pack.ps1                         # Windows x64; -VersionSuffix preview.1 for a pre-release
```

This writes `Build123d` and `Build123d.Occt` to the local folder feed `artifacts/packages`.
`Build123d.Occt` carries the shim, plus only the OCCT and FreeType libraries the shim depends on,
flat under `runtimes/win-x64/native`. The script then runs
[tests/Build123d.Packaging.Tests](tests/Build123d.Packaging.Tests). That test unpacks the packages into
a folder of their own and loads them through a collectible `AssemblyLoadContext`, as Bento's extensions
are loaded. Inside that context it makes and tessellates a box on a background thread, checks the native
libraries came from that folder, and checks the context unloads. The test needs the packages, so it
isn't part of the solution's `dotnet test`. Changes are listed in [CHANGELOG.md](CHANGELOG.md), and the
shipped native code's licences in [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).

## The OCCT.NET backend (spike)

`Build123d` can also run on OCCT.NET, the managed C# port of OCCT, with no native libraries. The native backend stays
the default. To build and test on OCCT.NET, point `OcctNetRoot` at an OCCT.NET checkout (by default `../OCCT.NET`
beside this repository):

```powershell
dotnet test tests/Build123d.Tests -p:Build123dBackend=OcctNet -p:OcctNetRoot=C:/src/OCCT.NET/
```

Each backend builds into folders of its own (`bin-occtnet`, `obj-occtnet`), so the two don't mix. OCCT.NET hasn't
ported 3D fillets, chamfers, offsets, drafts and `BRepFeat` yet: under this backend they throw
`NotSupportedException` ("not yet ported to OCCT.NET: BRepFilletAPI_MakeFillet"). The wrappers in
`src/Build123d.OcctNet/Generated` are regenerated with `./tools/generate-occtnet.ps1` when the native bindings or
OCCT.NET change. See [docs/design.md](docs/design.md), "The OCCT.NET backend".

## Licence

Apache License 2.0, as build123d's (see [LICENSE](LICENSE)). The native libraries the packages ship
are under their own licences: see [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
