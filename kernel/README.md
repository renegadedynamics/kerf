# Kerf.Kernel

A managed .NET CAD kernel derived from [Open CASCADE Technology](https://github.com/Open-Cascade-SAS/OCCT) (OCCT) 8.0.1,
ported to C# class for class and now being redesigned for .NET (phase 10: [docs/phase10-plan.md](docs/phase10-plan.md)).
See [docs/design.md](docs/design.md) for the design and [docs/port-status.md](docs/port-status.md) for what has been
ported.

- **Pure managed code**: no native libraries, no P/Invoke.
- **.NET names** (unit 10E): types are in namespaces by area (`Kerf.Geometry`, `Kerf.Topology` for the public API,
  `Kerf.Engine.<Area>` for the algorithms), without OCCT's package prefixes, and members follow .NET's naming.
  [docs/api/kerf.map](docs/api/kerf.map) says what each OCCT name became, so OCCT's documentation can still be found;
  each derived file names its OCCT source in a `// Port of:` header.
- **Immutable to callers**: every value you get back is immutable.

```csharp
using Kerf.Engine.Geometry;
using Kerf.Geometry;

Transform3D rotation = Transform3D.Rotation(Constants.OZ(), Math.PI / 2);
Point3D point = new Point3D(1, 0, 0).Transformed(rotation);   // (0, 1, 0); the original is unchanged
```

The public API is still OCCT's classes renamed; units 10F to 10L reshape it as [docs/api](docs/api/README.md) designs.

## Assemblies

| Assembly | What |
|---|---|
| `Kerf.Kernel.Foundation` | Maths, geometric values (`Point3D`, `Transform3D`…), bounding boxes, the engine's foundations |
| `Kerf.Kernel.ModelingData` | Curves and surfaces, topology (`Shape`, `Edge`, `Face`…), adaptors, extrema |
| `Kerf.Kernel.ModelingAlgorithms` | Construction, Booleans, fillets, offsets, features, healing, meshing |
| `Kerf.Kernel.DataExchange` | STEP, STL, glTF, 3MF, OBJ and BREP |

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
