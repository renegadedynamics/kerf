# Kerf.Kernel

A managed .NET CAD kernel derived from [Open CASCADE Technology](https://github.com/Open-Cascade-SAS/OCCT) (OCCT) 8.0.1,
ported to C# and redesigned for .NET. See [docs/api](docs/api/README.md) for the public API,
[docs/design.md](docs/design.md) for the design and [docs/port-status.md](docs/port-status.md) for what has been ported.

- **Pure managed code**: no native libraries, no P/Invoke.
- **.NET names:** the public API is in namespaces by area (`Kerf.Geometry`, `Kerf.Topology`, `Kerf.Booleans`…), without
  OCCT's package prefixes, and members follow .NET's naming. The algorithms are internal, in `Kerf.Engine.<Area>`.
  [docs/api/kerf.map](docs/api/kerf.map) says what each OCCT name became, so OCCT's documentation can still be found;
  each derived file names its OCCT source in a `// Port of:` header.
- **Immutable**: every value you get back is immutable.
- **Results**: an operation that can fail for an expected reason returns a `Result`, with diagnostics.

```csharp
using Kerf.Geometry;

Transform3D rotation = Transform3D.Rotation(Axis3D.ZAxis, Math.PI / 2);
Point3D point = new Point3D(1, 0, 0).Transformed(rotation);   // (0, 1, 0); the original is unchanged
```

## Assemblies

| Assembly | What |
|---|---|
| `Kerf.Kernel.Foundation` | Maths, geometric values (`Point3D`, `Transform3D`…), bounding boxes, the engine's foundations |
| `Kerf.Kernel.ModelingData` | Curves and surfaces, topology (`Shape`, `Edge`, `Face`…), adaptors, extrema |
| `Kerf.Kernel.ModelingAlgorithms` | Construction, Booleans, fillets, offsets, features, healing, meshing |
| `Kerf.Kernel.DataExchange` | STEP, STL, glTF, 3MF, OBJ and BREP |

## Runtime settings for a host

These belong to the application, not the library. For work on many threads, such as a spreadsheet recalculating workbooks at once:

```xml
<PropertyGroup>
  <ServerGarbageCollection>true</ServerGarbageCollection>
  <ConcurrentGarbageCollection>true</ConcurrentGarbageCollection>
  <TieredPGO>false</TieredPGO>
</PropertyGroup>
<ItemGroup>
  <RuntimeHostConfigurationOption Include="System.Runtime.TieredCompilation.CallCountingDelayMs" Value="0" />
</ItemGroup>
```

Measured on 24 logical processors, a round of 24 copies of Bento's examples takes 620 ms against 2,700 ms with the runtime's defaults, and a single recalculation 180 ms against 360, at a peak of about 400 MB against 200. Leave the server collector's dynamic adaptation (DATAS) on: turning it off gains little and takes about 700 MB. Without PGO, code that has settled for minutes is about 6% slower, so a long batch job may prefer to leave it on. See [docs/performance.md](docs/performance.md), "Unit 11D".

## Building

Requirements: the .NET 10 SDK and git.

```powershell
./tools/get-occt.ps1          # OCCT V8_0_1 source into .deps/occt-src (for the corpus test and port status)
./tools/get-occt-testdata.ps1 # OCCT's test data (99 MB) into .deps/occt-testdata, for the Draw corpus
dotnet test
```

`tests/Kerf.Kernel.Oracle.Tests` compares the port with native OCCT's results, recorded when native OCCT was still built
(unit 10A) and frozen since it went (unit 10B): see [docs/oracle-testing.md](docs/oracle-testing.md).

## Contributing

[docs/design-rules.md](docs/design-rules.md) gives the rules for new and redesigned code (phase 10);
[docs/translation-rules.md](docs/translation-rules.md) says how C++ became C# until phase 10, and
[docs/porting-playbook.md](docs/porting-playbook.md) the steps for porting a package.

## Licence

GNU Lesser General Public License 2.1 with the Open CASCADE exception, as OCCT: see [LICENSE](LICENSE),
[OCCT_LGPL_EXCEPTION.txt](OCCT_LGPL_EXCEPTION.txt) and [NOTICE](NOTICE). Kerf.Kernel is a modified version of OCCT 8.0.1
and isn't endorsed by Open Cascade SAS. Delabella's port (`BRepMesh/delabella.cs`) keeps its MIT licence: see
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
