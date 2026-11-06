# The owner's answers: what Bento sees change

The answers recorded in kernel decision 1790, as built in kernel decisions 20501 to 20506. This note was checked against Bento's `kerf/0.2.0` branch. **A member not listed keeps its name and meaning.**

## What Bento has to change

| Where | Before | After |
|---|---|---|
| `Extensions/Bento.Cad.Tests/IntroductoryExamples.cs`, the volumes of examples 12, 28, 30 and 36 (`IntroductoryExampleTests` and `SampleWorkbookTests` check them to 1e-9) | 14627.106295339287, 251188.19571967638, 64463.381199800904, 30298.935241110394 | 14628.060398053225, 251187.91917067088, 64463.371359647979, 30298.935391416093. Native OCCT's default was off on these, and a fine mesh sides with the new figures |
| `Extensions/Bento.Cad/ModelFunctions.cs`, `Value<T>(Kerf.Result<T>)` | `result.Match(…)` drops a successful result's warnings | Show them, as a sweep's fallback now arrives as one (`ModelingDiagnostics.SweepWarning`, below): how a cell shows a warning is Bento's choice, and until it does, the user can't see which frame was used |

Nothing in `ModelArguments.cs` or `ModelFunctions.cs` calls the members that moved or were retyped: Bento's `CAD_PT_SWEEP` goes through `Operations.Sweep` and `SweepOptions`, whose properties haven't changed, and there's no `CAD_PT_TORUS` yet. `Locations(params IEnumerable<object>)`, which `ModelFunctions.cs` calls, is as it was.

## What changes without a change in Bento

- **`A1.Volume`, `A1.Area` and ordering or grouping by them** are integrated adaptively, to a millionth of each face's contribution. 33 of the 37 introductory examples move by less than 1e-9; swept and B-spline faces move most (example 12, 6.5e-5). A workbook's cells may show different digits.
- **Time:** those calls don't get dearer. See decision 20501 for the figures; over the eleven examples, a volume went from 20.5 to 9.9 ms in all and an area from 13.4 to 10.3 ms, on a loaded machine.
- **A default sweep that used to fail** can now succeed in another frame (`CAD_PT_SWEEP` without `isFrenet`). A formula showing `#VALUE!` may start giving a shape.
- **The centre of a shape** still uses the default integration (decision 20501).

## The rest of the public changes

These don't touch Bento's code as it stands, but are here if a formula or a test starts to use them.

| Before | After |
|---|---|
| `OneD.ConnectedEdges(edge, parent, continuity)`, `OneD.ConnectedFaces(edge, parent)` | `edge.ConnectedEdges(parent, continuity)`, `edge.ConnectedFaces(parent)` |
| `OneD.EdgesToWires(edges, tolerance)` | `Wire.Combine(edges, tolerance)`. Its default tolerance is 1e-9, not 1e-6 |
| `TwoD.SortWiresByBuildOrder` | Internal, with `OneD` and `TwoD` themselves |
| `Solid.Sweep(… object? mode …)`, `Solid.SweepMulti(… object? binormal)` | `SweepMode? `: a `Vector3D`, `Direction3D`, `Wire` or `Edge` converts to one. Anything else no longer compiles, rather than failing when run |
| `edge.TrimToOther(object)` | `edge.TrimToOther(CurveLimit)`: the shapes, `ModelShape`, `Axis`, `Location`, `Plane` and `Point3D` convert to one, as for an arc's limit. An `IShape` or `object` variable needs its own type |
| `edge.FindIntersections(object?, tolerance)`, `FindIntersectionPoints` | Overloads: `()` or `(tolerance)` for this edge crossing itself, `(Edge, tolerance)`, `(Axis, tolerance)`. A curve object is passed as its edge. A plane no longer compiles, rather than throwing |
| `ShapeExtensions.ShapeOf(object)` | Overloads for `IShape`, `Point3D`, `Location`, `Axis` and `Plane` |
| `Solid.MakeTorus(…, startAngle, endAngle)`, `new Torus(…, minorStartAngle, minorEndAngle)`, the kernel's `Solid.Torus` with `MinorStart` and `MinorEnd` | The same calls, but part of the minor circle is now a slice of the tube, closed by a straight line across the arc's ends: 53.5 and not 210.6 for the outer half with R = 5, r = 1 |
| `Sweeps.Sweep` with the default frame | New: `SweepOptions.TryOtherFrames`, true by default; warnings `Sweeps.SweptWithFrenet` and `Sweeps.SweptWithDiscrete` |
