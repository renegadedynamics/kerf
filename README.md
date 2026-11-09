# Kerf

A B-rep CAD kernel and modelling library for .NET, in pure managed C#.

Kerf has two layers, each under its own licence, in its own folder:

| Folder | Layer | Derived from | Licence |
|---|---|---|---|
| [kernel/](kernel/) | **Kerf.Kernel**: geometry, topology, Booleans, fillets, offsets, healing, meshing, STEP, STL, glTF and 3MF | [Open CASCADE Technology](https://dev.opencascade.org) 8.0.1, by Open Cascade SAS, ported to C# | LGPL 2.1 with the Open CASCADE exception ([kernel/LICENSE](kernel/LICENSE), [kernel/OCCT_LGPL_EXCEPTION.txt](kernel/OCCT_LGPL_EXCEPTION.txt)) |
| [modeling/](modeling/) | **Kerf.Modeling**: parts, sketches and curves, locations, selectors and the algebra of combining them, over the kernel | [build123d](https://github.com/gumyr/build123d), by the build123d contributors, ported to C# | Apache 2.0 ([modeling/LICENSE](modeling/LICENSE), [modeling/NOTICE](modeling/NOTICE)) |

The modelling layer references the kernel's assemblies; no source file is shared between the two.

## Using it

```powershell
dotnet add package Kerf.Modeling
```

```csharp
using Kerf.Modeling;

Part rounded = Operations.Fillet(new Box(10, 10, 10).Edges(), 1).Value;
Part part = rounded - new Cylinder(2, 20);
Exporters3D.ExportStep(part, "part.step");
```

Operations that can fail for an expected reason return a `Result`, whose diagnostics say why; operators throw. Shapes
are the kernel's structs (`Kerf.Topology`), every value is immutable, and nothing native is loaded.

## Status

0.4.0 is the third release under the name Kerf, and a pre-1.0 one: minor versions may break the API. Kerf replaces the
`OcctNet.*` and `Build123d` packages, which end at 0.1.0. Kerf diverges from OCCT and build123d, and isn't endorsed by
either project. What's next is in [kernel/docs/phase12-plan.md](kernel/docs/phase12-plan.md), and the changes are in
each layer's CHANGELOG ([kernel](kernel/CHANGELOG.md), [modeling](modeling/CHANGELOG.md)).

## Building

Requirements: the .NET 10 SDK and git.

```powershell
./kernel/tools/get-occt.ps1           # OCCT's source, for the corpus and port status
./kernel/tools/get-occt-testdata.ps1  # OCCT's test data (99 MB), for the Draw corpus
dotnet test Kerf.slnx
```

The modelling layer builds against the kernel's projects in this repository and calls the kernel's types directly.
Nothing native is built or loaded: native OCCT, and the modelling layer's native backend, went in phase 10 (unit 10B).

The kernel's oracle tests compare the port with native OCCT's results as recorded (unit 10A), so they run in
`Kerf.slnx` like any other tests. The recordings can't be made again: they are frozen
([kernel/docs/oracle-testing.md](kernel/docs/oracle-testing.md)).

## Credits

- **Open CASCADE Technology,** © Open Cascade SAS, under the LGPL 2.1 with the Open CASCADE exception. Kerf.Kernel is a
  modified version of OCCT 8.0.1; each derived file names its OCCT source in a `// Port of:` header.
- **build123d,** © the build123d contributors, under Apache 2.0. Kerf.Modeling is a modified version of it.
- **Third-party notices:** [kernel/THIRD-PARTY-NOTICES.md](kernel/THIRD-PARTY-NOTICES.md) and
  [modeling/THIRD-PARTY-NOTICES.md](modeling/THIRD-PARTY-NOTICES.md).
