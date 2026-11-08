# Kerf.Modeling

A .NET 10 modelling library derived from [build123d](https://github.com/gumyr/build123d), the Python CAD library, over
Kerf's kernel. **To use it, start with [docs/guide.md](docs/guide.md)**, the user guide; coming from build123d,
[docs/name-map.md](docs/name-map.md) lists the names that changed. See [docs/design.md](docs/design.md) for the design and
[docs/one-set-of-shapes.md](docs/one-set-of-shapes.md) for how it sits on the kernel's shapes.

```csharp
using Kerf.Modeling;

Part plate = new Box(80, 60, 10);
plate -= new GridLocations(60, 40, 2, 2) * new Cylinder(5, 10);
Exporters3D.ExportStep(plate, "plate.step");
```

## Building and testing

`Kerf.Modeling` (namespace `Kerf.Modeling`) runs on Kerf.Kernel, managed C# with no native libraries, through the
kernel's public API alone (unit 10M1). It builds against the kernel's projects in the repository's `kernel` folder (`KerfKernelRoot`), so a
fresh clone needs only the .NET 10 SDK:

```powershell
dotnet test
```

Its shapes are the kernel's (`Kerf.Topology.Shape`, `Edge`, `Face`…), with build123d's conveniences as extension members; it
adds modelling objects, `Part`, `Sketch`, `Curve` and `Assembly`, which hold a shape and carry a label, colour and history
([docs/one-set-of-shapes.md](docs/one-set-of-shapes.md)).

## Packages

`Kerf.Modeling` is packed with the kernel's packages by the kernel's `tools/pack-nuget.ps1`. Changes are listed in
[CHANGELOG.md](CHANGELOG.md).

[tests/Kerf.Modeling.Packaging.Tests](tests/Kerf.Modeling.Packaging.Tests) loads `Kerf.Modeling` and the kernel's
assemblies from a folder of their own through a collectible `AssemblyLoadContext`, as Bento loads its extensions, makes
and tessellates a box, checks no native library loads and the context unloads. It needs a build output
(`KERF_MODELING_LIB`, Kerf.Modeling's `bin` folder), so it isn't part of the solution's `dotnet test`, and skips without
it.

## Licence

Apache License 2.0, as build123d's (see [LICENSE](LICENSE)). Kerf.Modeling is a modified version of build123d and isn't
endorsed by its authors. Third-party notices: see [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
