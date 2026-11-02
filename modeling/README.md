# build123d.NET

A .NET 10 port of [build123d](https://github.com/gumyr/build123d), the Python CAD library built on
OpenCascade (OCCT). Work in progress. See [docs/design.md](docs/design.md) for the design and the
phase plan.

## Building and testing

`Build123d` runs on OCCT.NET, the managed C# port of OCCT, with no native libraries. OCCT.NET comes from its
packages on nuget.org (`OcctNet.Foundation`, `OcctNet.ModelingData`, `OcctNet.ModelingAlgorithms`,
`OcctNet.DataExchange`, at the version `OcctNetVersion` in `Directory.Build.props`), so a fresh clone needs only the
.NET 10 SDK:

```powershell
dotnet test
```

To work on build123d.NET and OCCT.NET together, point `OcctNetRoot` at an OCCT.NET checkout, and `Build123d.OcctNet`
references its projects in place of the packages:

```powershell
dotnet test -p:OcctNetRoot=C:/src/OCCT.NET/
```

The native backend, `Build123d` over native OCCT through the `b3docct` shim, is opt-in with
`-p:Build123dBackend=Native`, and needs the native build below. It is the reference OCCT.NET's oracle tests compare
against, and is no longer published. Each backend builds into folders of its own, so the two sit side by side: the
default into `bin` and `obj`, native into `bin-native` and `obj-native`. `Build123dNativeRoot` takes the native build
from another checkout (a worktree, say) in place of this one's.

## Building the native dependencies (native backend)

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

Build the native shim (`b3docct`), then the .NET solution on the native backend:

```powershell
./native/build-shim.ps1                  # Windows; ./native/build-shim.sh <rid> on Linux/macOS
dotnet test -p:Build123dBackend=Native
```

`tests/Build123d.Occt.Tests`, the binding's own tests, runs on the native backend only; `dotnet test` skips it on
OCCT.NET.

`Build123d.Occt` copies the OCCT, FreeType and shim libraries for the host runtime into the output
directory of anything that references it.

## Packages

`Build123d` and `Build123d.OcctNet` are published on OCCT.NET, packed with OCCT.NET's packages by OCCT.NET's
`tools/pack-nuget.ps1`. The native backend is no longer published, and its projects aren't packable on their own.
`tools/pack.ps1` still packs it, for its load-context test:

```powershell
./tools/pack.ps1                         # Windows x64; versioned 0.1.0-native by default
```

This writes the native `Build123d` and `Build123d.Occt` to the local folder feed `artifacts/packages`.
`Build123d.Occt` carries the shim, plus only the OCCT and FreeType libraries the shim depends on,
flat under `runtimes/win-x64/native`. The script then runs
[tests/Build123d.Packaging.Tests](tests/Build123d.Packaging.Tests). That test unpacks the packages into
a folder of their own and loads them through a collectible `AssemblyLoadContext`, as Bento's extensions
are loaded. Inside that context it makes and tessellates a box on a background thread, checks the native
libraries came from that folder, and checks the context unloads. The test needs the packages (`B3D_PACKAGES`, which the
script sets), so it isn't part of the solution's `dotnet test`, and skips without them. Changes are listed in [CHANGELOG.md](CHANGELOG.md), and the
shipped native code's licences in [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).

## The OCCT.NET backend

The default backend (see "Building and testing"). OCCT.NET's phase 7
brings 3D fillets, chamfers, offsets, drafts and `BRepFeat`, and its phase 8 STEP, so every class is forwarded; STL,
glTF and 3MF (`Importers`, `Exporters3D`) are OCCT.NET's only. The wrappers in
`src/Build123d.OcctNet/Generated` are regenerated with `./tools/generate-occtnet.ps1` when the native bindings or
OCCT.NET change. See [docs/design.md](docs/design.md), "The OCCT.NET backend".

## Licence

Apache License 2.0, as build123d's (see [LICENSE](LICENSE)). The native libraries the packages ship
are under their own licences: see [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
