# build123d.NET

A .NET 10 port of [build123d](https://github.com/gumyr/build123d), the Python CAD library built on
OpenCascade (OCCT). Work in progress. See [docs/design.md](docs/design.md) for the design and the
phase plan.

## Building and testing

`Build123d` runs on OCCT.NET, the managed C# port of OCCT, with no native libraries, and calls its types directly. It
builds against the kernel's projects in the repository's `kernel` folder (`OcctNetRoot`), so a fresh clone needs only
the .NET 10 SDK:

```powershell
dotnet test
```

It uses kernel members the published `OcctNet.*` packages (0.1.0-preview.3) don't make public, so it doesn't build
against them. The native backend (`Build123d` over native OCCT through the `b3docct` shim, with its bindings, generator
and native build) and the forwarding layer that gave OCCT.NET the native binding's API (`Build123d.OcctNet`) went in
unit 10B.

## Packages

`Build123d` is published on OCCT.NET, packed with OCCT.NET's packages by the kernel's `tools/pack-nuget.ps1`. Changes are
listed in [CHANGELOG.md](CHANGELOG.md).

[tests/Build123d.Packaging.Tests](tests/Build123d.Packaging.Tests) loads `Build123d` and OCCT.NET's assemblies from a
folder of their own through a collectible `AssemblyLoadContext`, as Bento loads its extensions, makes and tessellates a
box, checks no native library loads and the context unloads. It needs a build output (`B3D_OCCTNET_LIB`, Build123d's
`bin` folder), so it isn't part of the solution's `dotnet test`, and skips without it.

## Licence

Apache License 2.0, as build123d's (see [LICENSE](LICENSE)). Third-party notices: see
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
