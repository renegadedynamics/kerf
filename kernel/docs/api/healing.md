# API review: healing

*Unit 10D, draft. Namespace `Kerf.Healing`, in ModelingAlgorithms. Engine: `Kerf.Engine.Healing` (ShapeFix,
ShapeAnalysis, ShapeUpgrade, ShapeCustom, ShapeBuild, ShapeConstruct, ShapeExtend, ShapeProcess, BRepCheck, Sewing).*

## Main types

New: `HealingOperations`, `FixOptions`, `SewOptions`, `UnifyOptions`, `BSplineConversionOptions`, `ValidationReport`,
`ShapeProblem`.

```csharp
public static class HealingOperations   // extension members on Shape
{
    public static ValidationReport Validate(Shape shape, ValidationOptions? options = null);      // BRepCheck_Analyzer
    public static Result<Built<Shape>> Fix(Shape shape, FixOptions? options = null,                 // ShapeFix_Shape
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default);
    public static Result<Built<Shape>> Sew(IEnumerable<Shape> shapes, SewOptions? options = null, …);    // BRepBuilderAPI_Sewing
    public static Result<Built<Shape>> Unify(Shape shape, UnifyOptions? options = null);           // ShapeUpgrade_UnifySameDomain
    public static Result<Built<Solid>> MakeSolid(Shell shell);                                     // ShapeFix_Solid.SolidFromShell
    public static Result<Built<Shape>> FixWireframe(Shape shape, double tolerance);               // ShapeFix_Wireframe: gaps, small edges
    public static Result<ImmutableArray<Wire>> ConnectEdges(IEnumerable<Edge> edges, double tolerance, bool shared = false);  // ShapeAnalysis_FreeBounds
    public static ImmutableArray<Wire> FreeBoundaries(Shape shellOrFaces, double tolerance);        // open edges of a shell
    public static Result<Built<Shape>> ConvertToBSpline(Shape shape, BSplineConversionOptions? options = null);  // ShapeCustom
}

public sealed record ValidationReport
{
    public bool IsValid { get; init; }
    public ImmutableArray<ShapeProblem> Problems { get; init; }       // each: the sub-shape, a code, a message
}

public sealed record FixOptions
{
    public static FixOptions Default { get; } = new();
    public double Precision { get; init; } = 1e-7;
    public double MaxTolerance { get; init; } = 1.0;
    public bool FixSolids { get; init; } = true;  public bool FixShells { get; init; } = true;
    public bool FixFaces { get; init; } = true;   public bool FixWires { get; init; } = true;
    public bool FixSmallEdges { get; init; } = true;
}

public sealed record UnifyOptions
{
    public bool Edges { get; init; } = true;  public bool Faces { get; init; } = true;
    public bool ConcatBSplines { get; init; } = true;
    public double LinearTolerance { get; init; } = 1e-7;  public double AngularTolerance { get; init; } = 1e-12;
}
```

- **No copy before fixing.** The modelling layer copies a shape (`PrivateCopy(obj, copyGeometry: true)`) before
  `ShapeFix_Shape` today, because ShapeFix writes into its input; the public `Fix` never does (plan 2.4, unit 10H4).
- `Validate` returns a report, not a `Result`: an invalid shape is a finding, not a failed operation.
- **Diagnostics:** `healing.not-fixed` (a warning, with the shapes left invalid), `healing.sewing-failed`,
  `healing.not-a-closed-shell`; `ShapeProblem` codes from `BRepCheck_Status` by name (`check.not-closed`,
  `check.self-intersecting-wire`, `check.bad-orientation`…).

## Examples

```csharp
var report = shape.Validate();
if (!report.IsValid) shape = shape.Fix().Value;
```

```csharp
Shape shell = HealingOperations.Sew(faces, new SewOptions { Tolerance = 1e-4 }).Value;
Solid closed = HealingOperations.MakeSolid((Shell)shell).Value;
```

```csharp
Shape tidy = part.Unify(new UnifyOptions { ConcatBSplines = false }).Value;
```

```csharp
var wires = HealingOperations.ConnectEdges(looseEdges, tolerance: 1e-3);
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `BRepCheck_Analyzer(shape).IsValid()`, `Result(sub)`, `BRepCheck_Status` | `Validate` → `ValidationReport` |
| `new ShapeFix_Shape(copy)`, `SetPrecision`, `SetMaxTolerance`, `FixSolidMode()`… `Perform()`, `Shape()` | `Fix` with `FixOptions` |
| `ShapeFix_Face`, `ShapeFix_Solid`, `ShapeFix_Wireframe` | `FixOptions`, `MakeSolid`, `FixWireframe` |
| `BRepBuilderAPI_Sewing` (`Add`, `Perform`, `SewedShape`) | `Sew` |
| `ShapeUpgrade_UnifySameDomain(shape, edges, faces, concat)`, `Build()`, `History()` | `Unify`; history in `Built` |
| `ShapeAnalysis_FreeBounds.ConnectEdgesToWires`, `GetOpenWires` | `ConnectEdges`, `FreeBoundaries` |
| `ShapeAnalysis_Shell`, `ShapeAnalysis_Edge` | `Validate`'s report |
| `ShapeAnalysis_Surface.ValueOfUV` | `FaceSurface.Parameters` (topology.md) |
| `ShapeCustom.BSplineRestriction`, `ShapeCustom_RestrictionParameters` | `ConvertToBSpline`, `BSplineConversionOptions` |
| `ShapeBuild_ReShape`, `BRepTools_ReShape` | Engine (the working overlay of plan 2.4); `ShapeHistory` outside |

**Disappears:** `ShapeExtend_Status` and the `Status(…)` queries, `ShapeFix_Root`, `ShapeProcess` and its operator
registry (engine: data exchange's healing sequence), `Message_Msg` texts.

## Open points

- `ShapeProcess` (the configurable sequence STEP's reader runs) has no public face; `StepReadOptions.Heal` turns it on
  or off. A public sequence API can wait for a user.
