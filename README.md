# Kerf

A B-rep CAD kernel and modelling library for .NET, in pure managed C#.

Kerf has two layers, each under its own licence, in its own folder:

| Folder | Layer | Derived from | Licence |
|---|---|---|---|
| [kernel/](kernel/) | **Kerf.Kernel**: geometry, topology, Booleans, fillets, offsets, healing, meshing, STEP, STL, glTF and 3MF | [Open CASCADE Technology](https://dev.opencascade.org) 8.0.1, by Open Cascade SAS, ported to C# | LGPL 2.1 with the Open CASCADE exception ([kernel/LICENSE](kernel/LICENSE), [kernel/OCCT_LGPL_EXCEPTION.txt](kernel/OCCT_LGPL_EXCEPTION.txt)) |
| [modeling/](modeling/) | **Kerf.Modeling**: parts, sketches and curves, locations, selectors and the algebra of combining them, over the kernel | [build123d](https://github.com/gumyr/build123d), by the build123d contributors, ported to C# | Apache 2.0 ([modeling/LICENSE](modeling/LICENSE), [modeling/NOTICE](modeling/NOTICE)) |

The modelling layer references the kernel's assemblies; no source file is shared between the two.

## Status

The two folders hold the projects as they were published until now:
- `kernel/` is **OCCT.NET**, a class-for-class port of OCCT. It is on nuget.org as `OcctNet.*`.
- `modeling/` is **build123d.NET**, on nuget.org as `Build123d`.

Phase 10 ([kernel/docs/phase10-plan.md](kernel/docs/phase10-plan.md)) turns them into Kerf:
- .NET names and conventions;
- full immutability;
- results in place of expected exceptions;
- the kernel's own tests in place of the comparison with native OCCT.

The packages move to `Kerf.Kernel` and `Kerf.Modeling` at 0.2.0. Kerf diverges from OCCT and build123d from here on, and
isn't endorsed by either project.

## Building

Requirements: the .NET 10 SDK and git.

```powershell
./kernel/tools/get-occt.ps1           # OCCT's source, for the corpus and port status
./kernel/tools/get-occt-testdata.ps1  # OCCT's test data (99 MB), for the Draw corpus
dotnet test Kerf.slnx
```

The modelling layer builds against the kernel's projects in this repository. `-p:UseOcctNetPackages=true` builds it
against the published packages instead.

The kernel's oracle tests compare with native OCCT, through the modelling layer's native backend. They are outside
`Kerf.slnx`, because one restore can't serve the modelling projects on both backends at once. Phase 10 retires them.
- **Running them:** `dotnet test kernel/tests/OcctNet.Oracle.Tests -p:Build123dNativeRoot=<folder>`, where the folder
  holds a native build (`native/out` and `.deps/install`, from `modeling/native`'s scripts).
- **Without a native build,** they are compiled out.

## Credits

- **Open CASCADE Technology,** © Open Cascade SAS, under the LGPL 2.1 with the Open CASCADE exception. Kerf.Kernel is a
  modified version of OCCT 8.0.1; each derived file names its OCCT source in a `// Port of:` header.
- **build123d,** © the build123d contributors, under Apache 2.0. Kerf.Modeling is a modified version of it.
- **Third-party notices:** [kernel/THIRD-PARTY-NOTICES.md](kernel/THIRD-PARTY-NOTICES.md) and
  [modeling/THIRD-PARTY-NOTICES.md](modeling/THIRD-PARTY-NOTICES.md).
