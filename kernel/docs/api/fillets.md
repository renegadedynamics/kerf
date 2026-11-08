# API review: fillets and chamfers

*Unit 10D, draft; unit 10L2 implemented it (README.md, section 11, has what differs and what is left). Namespace `Kerf.Fillets`, in ModelingAlgorithms. Engine: `Kerf.Engine.Fillets` (ChFi3d, ChFiDS,
ChFiKPart, BRepBlend, Blend, BlendFunc, FilletSurf) and `Kerf.Engine.Fillets.Planar` (ChFi2d).*

## Main types

New: `FilletOperations`, `FilletOptions`, `ChamferOptions`, `FilletShape` (Rational, QuasiAngular, Polynomial),
`ChamferSpec` and `FilletSpec` (per-edge values).

```csharp
public static class FilletOperations   // extension members on Shape and Face
{
    // 3D: BRepFilletAPI_MakeFillet
    public static Result<Built<Shape>> Fillet(Shape shape, IEnumerable<Edge> edges, double radius, FilletOptions? options = null,
                                              IProgress<double>? progress = null, CancellationToken cancellationToken = default);
    public static Result<Built<Shape>> Fillet(Shape shape, IEnumerable<FilletSpec> edges, FilletOptions? options = null, …);
    // 3D: BRepFilletAPI_MakeChamfer
    public static Result<Built<Shape>> Chamfer(Shape shape, IEnumerable<Edge> edges, double distance, ChamferOptions? options = null, …);
    public static Result<Built<Shape>> Chamfer(Shape shape, IEnumerable<ChamferSpec> edges, ChamferOptions? options = null, …);
    // 2D, on a planar face's vertices: BRepFilletAPI_MakeFillet2d
    public static Result<Built<Face>> Fillet2D(Face face, IEnumerable<Vertex> vertices, double radius);
    public static Result<Built<Face>> Chamfer2D(Face face, IEnumerable<Vertex> vertices, double distance, double? distance2 = null);
    public static Result<Built<Face>> Chamfer2D(Face face, Vertex vertex, double distance, double angle);
}

public readonly record struct FilletSpec(Edge Edge, double Radius, double? EndRadius = null);  // EndRadius: linear variation
public readonly record struct ChamferSpec(Edge Edge, double Distance, double? Distance2 = null, double? Angle = null, Face? ReferenceFace = null);

public sealed record FilletOptions
{
    public static FilletOptions Default { get; } = new();
    public FilletShape Shape { get; init; } = FilletShape.Rational;
    public Continuity InternalContinuity { get; init; } = Continuity.C1;
    public double AngularTolerance { get; init; } = 0.01;
    public FilletTolerances Tolerances { get; init; } = FilletTolerances.Default;   // SetParams' six values, named
}
```

- **Diagnostics** from `ChFiDS_ErrorStatus` and the stripe errors: `fillet.walking-failure`, `fillet.start-failure`,
  `fillet.twisted-surface`, `fillet.radius-too-large`, `fillet.no-result`, each with the edges (`Shapes`) whose
  contour failed. Today the modelling layer reads `NbFaultyContours`, `FaultyContour`, `StripeStatus` to say which
  edges failed; that becomes the diagnostics' shapes.
- **At a limit** (decision 1847): straight edges between planes, where a face just disappears, are built after ChFi3d fails; `FilletLimit(edges)` gives the largest radius they can take, and a size that doesn't fit fails with `fillet.too-large`, saying what does. A radius within the shape's tolerance of that is that, and one above it fails at once, without trying (decision 1898), unless the next face is within what ChFi3d calls tangent, where there's no limit (1899).
- **A chamfer past a bend** (decision 23801): a length on a plane past the tightest convex circle or ellipse of an edge in it, on the contour ChFi3d carries the edges round, fails with `fillet.too-large` before anything is built, saying what fits. At it, ChFi3d is asked, and a failure says the edge closes to a point. `ChamferLimit(edges)` gives the least of that and a face's width (decision 23803).
- 2D fillets and chamfers return the face; their history maps each vertex to the new edge (`Generated(vertex)`).

## Examples

```csharp
var rounded = FilletOperations.Fillet(box, box.AllEdges().Where(IsVertical), radius: 2);
if (!rounded.IsSuccess) return $"#VALUE! {rounded.Error!.Message}";   // "The fillet of radius 2 failed on 4 edges: …"
```

```csharp
var chamfered = part.Chamfer(topEdges, 1.5, new ChamferOptions());
```

```csharp
var varying = part.Fillet([new FilletSpec(edge, Radius: 1, EndRadius: 3)]);
```

```csharp
Face rounded2D = FilletOperations.Fillet2D(sketchFace, sketchFace.Vertices(), 5).Value;
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `new BRepFilletAPI_MakeFillet(shape)`, `Add(radius, edge)`, `Build()`, `IsDone()` | `FilletOperations.Fillet(shape, edges, radius)` |
| `Add(r1, r2, edge)`, `SetRadius(…)` | `FilletSpec(edge, radius, endRadius)` |
| `SetParams(Tang, Tesp, T2d, TApp3d, TolApp2d, Fleche)`, `SetContinuity` | `FilletOptions.Tolerances`, `InternalContinuity` |
| `ChFi3d_FilletShape` | `FilletShape` |
| `NbFaultyContours`, `FaultyContour`, `StripeStatus`, `ChFiDS_ErrorStatus` | `Diagnostics` with codes and edges |
| `BRepFilletAPI_MakeChamfer` (`Add(d, edge)`, `Add(d1, d2, edge, face)`, `AddDA`) | `Chamfer` with `ChamferSpec` |
| `BRepFilletAPI_MakeFillet2d` (`AddFillet`, `AddChamfer`, `Status`) | `Fillet2D`, `Chamfer2D` |
| `Modified`, `Generated`, `IsDeleted`, `NewFaces(i)` | `Built.History` |

**Disappears:** `BRepFilletAPI_LocalOperation`, the contour/stripe accessors (`NbContours`, `Contour`, `Value`,
`Length`, `FirstVertex`…), `ChFi2d_ConstructionError`.

## Open points

- The modelling layer's chamfer by distance and angle needs a reference face; `ChamferSpec.ReferenceFace` defaults to
  the first face of the edge, as build123d does.
- A partial result (some contours built) isn't returned: OCCT builds all or none for `Build()`. Keep it so.
