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
    public bool ApproximateSection { get; init; } = true;    // Section only (decision 1836)
    public bool? ParallelVolumes { get; init; }              // the volume check's integration on several threads; null: the scope's
}
```

- **Parallel volumes** (decision 2092): `ParallelVolumes = true` integrates the volumes the check measures, where one has
  16 faces or more to integrate, on the thread pool, summed in the faces' order, so the verdict and every figure are the
  same to the bit as off; `false` keeps them on the calling thread; null, the default, follows the scope
  `ParallelVolumes.Use()` (namespace `Kerf`), which is off unless opened. For one large part on a processor that's otherwise
  idle; parts already built side by side pay processor time and waits for it. `MassOptions.ParallelVolumes` is the same
  for the mass properties.

- **Non-destructive always:** OCCT's `SetNonDestructive(false)` let the algorithm change its inputs' tolerances; the
  public API never does (plan 2.4). Until 10H2 the engine keeps its copy at entry for this.
- **Diagnostics** from BOPAlgo's alerts: `boolean.self-interference` (with the shapes), `boolean.too-small-edge`,
  `boolean.not-valid-input`, `boolean.unsupported-type`, `boolean.empty-result` (a warning), `boolean.failed`.
  Warnings from a successful run (OCCT's warning alerts) stay on the result.

- **A result proved wrong fails** with `result.invalid`, carrying the shape, under the same `CheckResult` as the validity check (decisions 1860 to 1868), for Fuse, Cut and Common of solids:

| Operation | Must hold, with A, B and R the volumes |
|---|---|
| Common | R ≤ min(A, B). If it's empty, no two points may lie well inside both |
| Cut | A − B ≤ R ≤ A. If it removed nothing, no two points may lie well inside both and still in R |
| Fuse | max(A, B) ≤ R ≤ A + B |
| Any | No solid of the result inside out |

  A result proved wrong is first made again: with its inputs the other way round (warning `boolean.other-way-round`; decision 1937), then with a fuzzy value of ten times the inputs' largest tolerance, and then of a thousand times it while that's no more than a hundred-thousandth of the largest input's size (warning `boolean.fuzzy-value`, naming it; decisions 2103 and 2117), and then with each crossing near a vertex given a vertex of its own, where OCCT grows the one vertex over them, kept where two ways of doing that agree in volume and area (warning `boolean.crossings-apart`; decision 33704). An invalid result is made again the same ways (decision 33706). A retry is kept only if it's valid and passes every check the first had; otherwise the first failure stands. An empty common, or a cut that removed nothing, is looked for at a grid of points in the boxes' overlap and at points just inside the inputs' faces (decision 2104).

  Each with a margin of a ten-thousandth of the larger input, and more for large tolerances or a fuzzy value. Nothing is said where anything isn't a solid (faces, shells, half-spaces), for an input that isn't valid, or about Split, Partition and Section. `BooleanVolumes.Check(rule, arguments, tools, result, fuzzyValue)` is the check itself, for a caller that runs one with `CheckResult = false` and tidies after, as `Kerf.Modeling` does. `BooleanOperations.Retry(rule, arguments, tools, options)` makes again, in the same ways, a result such a caller proved wrong: the first that passes, with its warning, or `boolean.failed`.

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
