# OCCT.NET

A managed .NET port of [Open CASCADE Technology](https://github.com/Open-Cascade-SAS/OCCT) (OCCT), the open-source
CAD kernel. Work in progress: see [docs/design.md](docs/design.md) for the design and the phase plan, and
[docs/port-status.md](docs/port-status.md) for what has been ported.

- **Pure managed code**: no native libraries, no P/Invoke.
- **Faithful**: ported class for class from OCCT V8_0_1, keeping OCCT's names, so OCCT's documentation applies.
- **Immutable to callers**: every value you get back is immutable; collections are Bento.Collections lists.

```csharp
using Occt;

gp_Trsf aRotation = gp_Trsf.Rotation(gp.OZ(), Math.PI / 2);
gp_Pnt aPoint = new gp_Pnt(1, 0, 0).Transformed(aRotation);   // (0, 1, 0); the original is unchanged
```

## Building

Requirements: the .NET 10 SDK and git.

```powershell
./tools/get-occt.ps1          # OCCT V8_0_1 source into .deps/occt-src (for the corpus test and port status)
./tools/pack-bento.ps1        # Bento.Collections from ../bento into the local feed .deps/nuget
dotnet test
```

`tests/OcctNet.Oracle.Tests` compares the port with native OCCT and needs a sibling build123d.NET checkout with
its native build: see [docs/oracle-testing.md](docs/oracle-testing.md).

## Contributing a port

[docs/translation-rules.md](docs/translation-rules.md) says how C++ becomes C#, and
[docs/porting-playbook.md](docs/porting-playbook.md) the steps for a package.

## Licence

GNU Lesser General Public License 2.1 with the Open CASCADE exception, as OCCT: see [LICENSE](LICENSE),
[OCCT_LGPL_EXCEPTION.txt](OCCT_LGPL_EXCEPTION.txt) and [NOTICE](NOTICE).
