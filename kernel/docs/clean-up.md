# The code clean-up: the survey, and what was done

The owner's direction (decision 1681): "I want to continue to clean up the code (both public facing and private)." This is the list that direction
had no room for, ranked by what a reader or user meets first, with what was done and what's left. All counts were measured on 6 November 2026 at the
branch point, by building with extra analysers on, by reflection over the built assemblies, or by searching the source, as each says. "Engine" means
`Kerf.Engine.*`, which is internal; the public surface is 564 types and 5,855 members (235 and 1,985 of them in `Kerf.Modeling`).

## Public

| What | Found | Done | Left |
|---|---|---|---|
| OCCT-style names in `Kerf.Modeling` (`DPrism`, `GeomAdaptor`, `Polygon.Pts`, `OffsetTopodsFace`) | 4 names | All | |
| One idea, several names in `Kerf.Modeling`: an object's height (`BoxHeight`, `SlotHeight`, against `Superellipse.Height`) | 6 names, on 10 types | All are `Height` | |
| One idea, several names: STL's options (`Tolerance`) against glTF's and 3MF's (`LinearDeflection`) | 2 | All | |
| Stale `Obsolete` in `Kerf.Modeling` (`SlotArc`) | 1 | Deleted | |
| Python helpers public for no reason (`TopologyUtils`) | 1 class | Internal | Question 1: `OneD`, `TwoD` |
| Awkward parameter names in `Kerf.Modeling` (`ax1`, `trsf`, `pln`, `pts`, `xsize`, `lefthand`, `Triangle`'s `A`) | 24 | All | |
| `object` parameters in `Kerf.Modeling` | 7 | | Question 2 |
| OCCT's abbreviations in the kernel's `With…` methods (`WithCirc2d`, `WithHypr`, `WithPln`, `WithTrsf`) | 13 | All | |
| Enum members with OCCT's names (`Extrinsic_XYZ`, `PntMirror`, `CompoundTrsf`) | 28 | All | |
| The nested enum `Direction3D.D` in the API, beside `UnitX` | 2 enums, 12 constructors | Internal | |
| Parameters with OCCT's short names in the kernel (`a1`, `ang`, `p`, `raiseExc`, `isTheNotCheckC0`) | 758 | All | `u1`, `v1`, `a11` and the like stay: they are what the mathematics says |
| The process in XML docs a user reads ("(unit 10M3)", "(decision 127)") | 65 lines in the kernel's public namespaces, 15 in `Kerf.Modeling` | 57 removed | The rest are on internal members of public types, or read as part of a sentence |
| Mentions of OCCT.NET or build123d.NET in source | 0 | | |
| Public members without XML docs (CS1591 with the suppression lifted) | 1,022: Foundation 912, ModelingData 103, ModelingAlgorithms 7 | | Mostly `Kerf.Geometry`'s value types: see "What's left" |
| Summaries of two words or fewer | 44 in `Kerf.Modeling`, 139 in the kernel | | Most are fair (`Millimetre (mm).`) |
| `Get…` methods that could read better (`GetAngle`, `GetTopLevelShapes`, `FaceMesh.GetTriangles`) | 11 | | `GetAngle` is degrees where the kernel's `AngleTo` is radians, so the difference is useful: left alone |
| Overloads only because C++ had them (`Translated(p1, p2)`, `LinearForm`, `CrossCrossed`, `DotCross`) | Not counted | | Question 3 |

## Private

| What | Found | Done | Left |
|---|---|---|---|
| Private members nothing uses (IDE0051, CA1823) | 22 members | All deleted (578 lines); the analysers now fail the build | |
| Fields written and never read (IDE0052) | 78, and 3 more once dead members went | All deleted. The likely ones were compared with OCCT, where they are also dead | |
| `Obsolete` engine members | 16 | All deleted | |
| Unused `using`s (IDE0005) | 4,964 | All, by `dotnet format`; the build now fails on one in `src` | Tests not looked at |
| Suppressions those made needless (`CS0414`) | 54 lines | Those of the deleted fields | |
| Stale TODOs, and comments naming what no longer exists (`TODO(4A)`, "finding #25", `b3d_mesh.cpp`, the native backend) | 16 | All | 13 plain TODOs are OCCT's own remarks, ported |
| IDE1006 exemptions | 3,061 violations in 156 folders | `Kerf.Modeling` needs none (59 fixed); 3 clean kernel folders removed from the list | 3,002: decision 15304's parameters and locals, and uppercase locals from Fortran |
| Unused parameters (IDE0060) | 401: public 305, private 49, internal 28, protected 19 | | Many mirror an OCCT signature or an interface; `dotnet format` has no fix. List in "What's left" |
| Assignments overwritten before they're read (IDE0059) | 5,978 | | Almost all `double x = 0;` before `out x`, in ported code. Harmless, and they run through the hot paths two other units are changing |
| Commented-out code in ported files | About 1,590 lines | | OCCT's own, kept so a file reads against its source. Question 4 |
| `Standard_*` in source | 843, in 333 files | | 332 comment lines and 468 calls of helpers such as `Standard_OutOfRange_Raise_if`. Question 5 |
| Engine types nothing uses | 3 (`DictionaryExtensions` and `ListExtensions` hold extension methods, so aren't dead; `Geom2dGcc` tests only) | | |
| Engine methods no source or test names | 1,288 names, 1,555 declarations, by word search (an undercount); 1,052 more only tests call | | Question 6 |
| The `Transient` base (11M) | 392 direct subclasses; public because public geometry derives from it, and C# won't allow an internal base | | Nothing cheap reduces it |

## Questions for the owner

1. **`OneD` and `TwoD` in `Kerf.Modeling`** are named after build123d's modules (`OneD.EdgesToWires`, `OneD.ConnectedFaces`, `TwoD.SortWiresByBuildOrder`). I recommend moving them to `Wire` and `Edge` as extensions, and internal for `SortWiresByBuildOrder`. It's a matter of where a user looks, so it's yours.
2. **`object` parameters:** `Solid.Sweep(… object? mode …)`, `SweepMulti(… object? binormal)`, `Edge.TrimToOther(object)`, `Edge.FindIntersections(object?)`, `ShapeOf(object)`. They came from Python's unions and fail at run time on the wrong type. I recommend typed overloads for `Edge` and `Wire`, or a small `SweepMode` union. That's about twice the overloads, so it is a design choice.
3. **Overloads from C++ on the value types** (`Translated(p1, p2)`, `LinearForm`, `CrossCrossed`, `DotCross`, the `Multiplied`/`Divided` beside operators): I recommend keeping them until something shows they're in the way. Removing them is cheap but loses the line-for-line reading of ported callers.
4. **OCCT's commented-out code in ported files:** I recommend keeping it. It costs little and it's what a diff against OCCT shows; deleting it gains only tidiness.
5. **`Standard_OutOfRange_Raise_if` and the rest:** they are internal, and they keep OCCT's name so that a port reads against its source. I recommend `Throw.OutOfRangeIf` only if `BSplCLib`'s and `math`'s hot paths are being touched anyway.
6. **Engine methods nothing calls** are mostly the STEP entities' accessors (`StepBasic` 177, `StepKinematics` 140, `StepVisual` 124) and `NCollection`'s: the port kept OCCT's API whole. I recommend leaving the STEP entities, generated and uniform, and pruning the hand-ported rest when each is next touched. About 1,300 methods is a bigger cut than a clean-up should make unasked.

## What's left

- **XML docs on `Kerf.Geometry`'s value types** (912 members). `ModelingAlgorithms`' and `ModelingData`'s 110 could be done and CS1591 turned on for them.
- **IDE1006:** about 60 kernel folders have five violations or fewer, and each one fixed takes a folder off the list.
- **IDE0060's private 49:** they need a semantic tool that changes the callers; `isUnit` and `owned` are deliberate overload markers.
