# API review: Booleans

*Unit 10D, draft; unit 10L2 implemented it (README.md, section 11, has what differs and what is left). Namespace `Kerf.Booleans`, in ModelingAlgorithms. Engine: `Kerf.Engine.Booleans` (BOPAlgo, BOPDS,
BOPTools, BRepAlgoAPI) and `Kerf.Engine.Booleans.Legacy` (TopOpeBRep*, the old Booleans that fillets still use).*

## Main types

New: `BooleanOperations`, `BooleanOptions`, `GlueMode` (Off, Shift, Full), `SplitOptions`.

```csharp
public static class BooleanOperations   // each also an extension member: a.Fuse(b), a.Cut(b), a.Common(b)
{
    public static Result<Built<Shape>> Fuse(Shape a, Shape b, BooleanOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default);
    public static Result<Built<Shape>> Fuse(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, BooleanOptions? options = null, …);
    public static Result<Built<Shape>> FuseAll(IReadOnlyList<Shape> shapes, BooleanOptions? options = null, …);  // n-ary, one operation
    public static Result<Built<Shape>> Cut(Shape a, Shape b, …);       public static Result<Built<Shape>> Cut(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, …);
    public static Result<Built<Shape>> Common(Shape a, Shape b, …);    public static Result<Built<Shape>> Common(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, …);
    public static Result<Built<Shape>> Section(Shape a, Shape b, BooleanOptions? options = null, …);   // edges and vertices
    public static Result<Built<Shape>> Split(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, SplitOptions? options = null, …);
    public static Result<Built<Shape>> Partition(IReadOnlyList<Shape> shapes, BooleanOptions? options = null, …);  // General Fuse
}

public sealed record BooleanOptions
{
    public static BooleanOptions Default { get; } = new();
    public double FuzzyValue { get; init; }                  // 0: exact; OCCT's SetFuzzyValue
    public GlueMode Glue { get; init; } = GlueMode.Off;      // shapes known to touch only by coinciding faces
    public bool Parallel { get; init; } = true;
    public bool CheckInverted { get; init; } = true;
    public bool UseObb { get; init; }
    public bool Simplify { get; init; } = true;              // unify same-domain faces and edges after (SimplifyResult);
                                                             // its history is composed into the result's
    public bool ApproximateSection { get; init; }            // Section only
}
```

- **Non-destructive always:** OCCT's `SetNonDestructive(false)` let the algorithm change its inputs' tolerances; the
  public API never does (plan 2.4). Until 10H2 the engine keeps its copy at entry for this.
- **Diagnostics** from BOPAlgo's alerts: `boolean.self-interference` (with the shapes), `boolean.too-small-edge`,
  `boolean.not-valid-input`, `boolean.unsupported-type`, `boolean.empty-result` (a warning), `boolean.failed`.
  Warnings from a successful run (OCCT's warning alerts) stay on the result.

## Examples

```csharp
Shape bracket = BooleanOperations.Cut(plate, holes).Value;          // holes: a compound or a list of tools
```

```csharp
var fused = a.Fuse(b, new BooleanOptions { FuzzyValue = 1e-5, Glue = GlueMode.Shift });
if (!fused.IsSuccess)
    foreach (var d in fused.Diagnostics) log.Add($"{d.Code}: {d.Message} ({d.Shapes.Length} shapes)");
```

```csharp
var halves = BooleanOperations.Split([part], [splittingFace]).Value;
foreach (Solid half in halves.Shape.Explore<Solid>()) …
```

```csharp
Shape crossing = BooleanOperations.Section(part, planeFace).Value;  // edges where they meet
```

```csharp
var cut = BooleanOperations.Cut(block, cutter);
IReadOnlyList<Shape> newFaces = cut.Value.History.Generated(cutterFace);  // the modelling layer's Select.LAST
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `new BRepAlgoAPI_Fuse(a, b)`, `SetArguments`, `SetTools`, `Build()`, `IsDone()`, `Shape()` | `BooleanOperations.Fuse(a, b)` / `Fuse(arguments, tools)` |
| `BRepAlgoAPI_Cut`, `Common`, `Section` | `Cut`, `Common`, `Section` |
| `BRepAlgoAPI_Splitter`, `BOPAlgo_Splitter` | `Split` |
| `BRepAlgoAPI_BuilderAlgo` (General Fuse) | `Partition` |
| `SetFuzzyValue`, `SetGlue(BOPAlgo_GlueEnum)`, `SetRunParallel`, `SetUseOBB`, `SetCheckInverted` | `BooleanOptions` |
| `SetNonDestructive(true)` | Always |
| `SimplifyResult(…)`, then `ShapeUpgrade_UnifySameDomain` and merged histories | `BooleanOptions.Simplify` |
| `HasErrors()`, `HasWarnings()`, `GetReport()`, `DumpErrors`, `BOPAlgo_Alert…` | `Result.Diagnostics` |
| `Modified(s)`, `Generated(s)`, `IsDeleted(s)`, `History()` | `Built.History` |
| `BRepAlgoAPI_Section(a, b, false)` then `Approximation`, `ComputePCurveOn1` | `BooleanOptions.ApproximateSection`; pcurves always |

**Disappears:** `BRepAlgoAPI_Algo`/`BuilderAlgo`/`BooleanOperation` as types callers see, `BOPAlgo_Options`,
`BOPAlgo_ToolsProvider`, the `Message_Report`.

## Open points

- `Fuse(a, b)` and `FuseAll(shapes)` are separate names so overload resolution never confuses one shape list with
  arguments and tools.
- The modelling layer's "clean" step (unify after every Boolean) becomes `Simplify = true` by default here; build123d
  cleans by default too. Agree that the kernel's default matches.
