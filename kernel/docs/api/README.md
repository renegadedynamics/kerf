# Kerf.Kernel's public API: the design

*Unit 10D, 3 November 2026. The open questions in section 9 (53 to 65) are answered: decision 1349 settles them, and where it differs from this draft (60, colour; 62, placement; 64, images) the decision wins.*

This is the design that units 10E (the rename), 10F (errors), 10G (immutability) and 10L (the public API's
implementation) build. It follows phase10-plan.md 2.1, 2.2, 2.5 and 2.6 and [design-rules.md](../design-rules.md).

| Document | What |
|---|---|
| This file | The split, and what every area shares: results and diagnostics, options, cancellation, values, shapes, collections |
| [geometry.md](geometry.md) | `Kerf.Geometry`: points, vectors, transforms, curves, surfaces |
| [topology.md](topology.md) | `Kerf.Topology`: shapes, sub-shapes, traversal, history |
| [construction.md](construction.md) | `Kerf.Construction`: primitives, sweeps, lofts, shapes from geometry |
| [booleans.md](booleans.md) | `Kerf.Booleans` |
| [fillets.md](fillets.md) | `Kerf.Fillets`: fillets and chamfers, 3D and 2D |
| [offsets-features.md](offsets-features.md) | `Kerf.Offsets` and `Kerf.Features` |
| [healing.md](healing.md) | `Kerf.Healing`: checking, fixing, sewing, unifying |
| [measurement.md](measurement.md) | `Kerf.Measurement`: mass properties, bounds, distance, classification |
| [meshing.md](meshing.md) | `Kerf.Meshing` |
| [exchange.md](exchange.md) | `Kerf.Exchange`: STEP, STL, glTF, 3MF, OBJ, BREP |
| [classification.tsv](classification.tsv) | Every public type today: public API, engine or removed, with its new name |
| [classification-rules.txt](classification-rules.txt) | The rules that table comes from: by package, with exceptions by type |
| [kerf.map](kerf.map) | The draft name map, in kerf-rename's format |
| [modeling-uses.txt](modeling-uses.txt) | The kernel types the modelling layer uses today |

## 1. The split

[tools/api-classify.cs](../../tools/api-classify.cs) reads the four assemblies' metadata and classifies every public
type by [classification-rules.txt](classification-rules.txt): a rule per OCCT package (246), and exceptions by type
(298). It fails if a type has no rule, if two types would get one name, or if a public type takes a common .NET
type's name. Regenerate with `dotnet run tools/api-classify.cs` (from `kernel/`, after a Release build).

- **Public API:** the type itself becomes a public Kerf type, renamed, and reshaped by 10L (properties, 0-based
  indexing, no setters, no engine types in its signatures).
- **Engine:** the type becomes internal in `Kerf.Engine.<Area>`, renamed by rule (`BOPAlgo_PaveFiller` →
  `Kerf.Engine.Booleans.PaveFiller`), its mathematics unchanged.
- **Removed:** the type goes, replaced by .NET's own (NCollection by `List<T>`, `Dictionary<,>` and spans in 10J;
  `TCollection_HAsciiString` by `string`). Until then it keeps its OCCT name in the namespace `Occt`, which then holds
  only what is going.

| Assembly | Public API | Engine | Removed | Total | Nested (included) |
|---|---:|---:|---:|---:|---:|
| Foundation | 40 | 438 | 38 | 516 | 174 |
| ModelingData | 62 | 585 | 3 | 650 | 50 |
| ModelingAlgorithms | 0 | 1,419 | 0 | 1,419 | 57 |
| DataExchange | 0 | 1,307 | 0 | 1,307 | 15 |
| **Total** | **102** | **3,749** | **41** | **3,892** | 296 |

The metadata count (3,892, nested types included) is lower than the plan's 4,027, which counted declarations in source.
The removed count is low because most of NCollection's uses are aliases (`TColStd_Array1OfReal`, `TopTools_ListOfShape`:
global `using`s, not types); they go with NCollection in 10J.

**The public surface is about 170 types:** the 94 top-level types above (the gp values, Geom's and Geom2d's curves and
surfaces, the TopoDS shapes, four enums), and about 75 new ones that 10L writes over the engine: the operations, their
options and results, `Result<T>` and diagnostics, `BoundingBox`, `ShapeHistory`, the evaluators (`EdgeCurve`,
`FaceSurface`), the mesh and the exchange types. Each area's document lists its own.

By area (top-level types; the engine areas have no public namespace):

| Area | Public API | Engine | Removed | | Area | Public API | Engine | Removed |
|---|---:|---:|---:|---|---|---:|---:|---:|
| Geometry | 78 | 322 | 0 | | Fillets | 0 | 104 | 0 |
| Topology | 13 | 68 | 3 | | Offsets | 0 | 55 | 0 |
| Construction | 0 | 342 | 0 | | Features | 0 | 47 | 0 |
| Booleans | 0 | 195 | 0 | | Healing | 0 | 126 | 0 |
| Measurement | 0 | 64 | 0 | | Meshing | 0 | 117 | 0 |
| Exchange | 0 | 1,315 | 0 | | Intersection | 0 | 274 | 0 |
| Approximation | 0 | 92 | 0 | | Extrema | 0 | 70 | 0 |
| Classification | 0 | 33 | 0 | | HiddenLines | 0 | 102 | 0 |
| Numerics | 0 | 63 | 0 | | Foundation | 0 | 74 | 0 |
| Collections | 0 | 8 | 31 | | | | | |

Most areas have no public types of their own today: their OCCT API classes (`BRepAlgoAPI_Fuse`, `BRepPrimAPI_MakeBox`)
are mutable `Perform`/`IsDone` objects, so they become engine, and the area's public API is new code over them.

### 1.1 What the split means for 10E and 10L

Measured with the tool (its last report lines):

- **Public types expose engine types today.** `gp_Pnt.Coord()` returns `gp_XYZ`, `Geom_BSplineCurve` takes
  `TColgp_Array1OfPnt`, every Geom class derives from `Standard_Transient`. C# rejects a public member whose signature
  names an internal type, so **10E renames and moves, but can't make the engine internal.** Each area's engine types
  become internal at the end of its 10L step, once its public types no longer expose them.
  `Standard_Transient` stays public as `Kerf.Engine.Foundation.Transient`, with no public members, because the public
  Geom classes derive from it (question 55).
- **479 engine types are used from another kernel assembly** (202 of Foundation's, 232 of ModelingData's, 45 of
  ModelingAlgorithms'): the intersection walkers use Foundation's maths, the Booleans ModelingData's adaptors. That is
  not "a small `Engine` surface". The kernel's four assemblies should keep `InternalsVisibleTo` among themselves, as one
  product released together, and lose it to everything else (question 54).
- **The modelling layer uses 237 kernel types** ([modeling-uses.txt](modeling-uses.txt); 176 named in its source): 76
  become public as they are, and 161 are engine types, each with its public replacement in the table's `replacement`
  column, except 20 base classes the modelling layer only reaches through others (`BOPAlgo_Options`,
  `Adaptor3d_Curve`). After 10B it calls OCCT.NET's types directly; until 10M moves it onto this API, it needs
  `InternalsVisibleTo` too, or the engine types it uses stay public until then (question 54).
- **Draw-lite keeps using the engine** through `InternalsVisibleTo`, as it does today: it is the corpus's test harness
  (question 51), its commands are OCCT's, and the renamer keeps them compiling. It is the one non-test assembly KERF0107
  allows.

## 2. Results and diagnostics

One result type for expected failures (plan 2.5). In `Kerf`, in ModelingData, since diagnostics name shapes.

```csharp
namespace Kerf;

public readonly struct Result<T>
{
    public bool IsSuccess { get; }
    public T Value { get; }                               // throws KerfException with the diagnostics on failure
    public ImmutableArray<Diagnostic> Diagnostics { get; } // warnings, and the error on failure
    public Diagnostic? Error { get; }                     // the first error, or null
    public bool TryGetValue([MaybeNullWhen(false)] out T value);
    public T GetValueOrThrow();                           // the same as Value, for readers who want it spelt out
    public T GetValueOrDefault(T fallback);
    public Result<TOut> Map<TOut>(Func<T, TOut> map);    // keeps the warnings
    public Result<TOut> Bind<TOut>(Func<T, Result<TOut>> next);

    public static Result<T> Success(T value, ImmutableArray<Diagnostic> warnings = default);
    public static Result<T> Failure(Diagnostic error, ImmutableArray<Diagnostic> warnings = default);
    public static implicit operator Result<T>(T value);
}

public readonly struct Result { /* the same, without a value: what writers return */ }

public sealed record Diagnostic
{
    public required Severity Severity { get; init; }      // Warning or Error
    public required DiagnosticCode Code { get; init; }    // stable, for programs: "boolean.self-interference"
    public required string Message { get; init; }        // plain English, no OCCT names (design-rules 5)
    public ImmutableArray<Shape> Shapes { get; init; } = []; // the sub-shapes concerned
}

public enum Severity { Warning, Error }

public readonly record struct DiagnosticCode(string Value)   // well-known codes as static properties per area
{
    public static DiagnosticCode InvalidInput { get; }       // "input.invalid"
    public static DiagnosticCode NotConverged { get; }       // "numerics.not-converged"
    public static DiagnosticCode Cancelled { get; }          // "operation.cancelled"
}

public sealed class KerfException : Exception { public ImmutableArray<Diagnostic> Diagnostics { get; } }
```

- **Expected failures** (a fillet that can't be built, a Boolean on invalid input, an unreadable file, a projection
  with no solution) return `Result<T>`. **Misuse** (null, a negative radius, an index out of range) throws .NET's
  argument exceptions.
- **Cancellation** throws `OperationCanceledException`, as .NET does everywhere; it isn't a diagnostic.
- **One model replaces four** at the boundary: OCCT's status enums (`ChFiDS_ErrorStatus`, `BRepBuilderAPI_WireError`),
  `Message_Report` alerts, `IsDone()` and exceptions-as-statuses become diagnostics. Each area lists its codes.
- **Bento:** a failed result becomes `#VALUE!` with `Error.Message`, as `ModelFunctions.Run` does with exceptions
  today; warnings can go to the cell's note. A cell holds the value, never a `Result`.
- `Try…(out T)` where a single bool says enough: `Face.TryGetPlane(out Plane3D)`.

The definitions above compiled as written, with C# 14's extension members below, in a scratch project with warnings as
errors (not committed: tests belong in kernel/tests, which unit 10A holds; 10F adds the types with their tests).

### 2.1 Shape results carry their history

Operations that make shapes return `Result<Built<TShape>>`. `Built` converts to its shape, so the common case stays
short, and history (which the modelling layer uses for its `Select.LAST` and to name new faces) is there when wanted.

```csharp
public sealed record Built<TShape>(TShape Shape, ShapeHistory History) where TShape : struct, IShape<TShape>
{
    public static implicit operator TShape(Built<TShape> built);
}

Solid part = BooleanOperations.Cut(block, hole).Value;       // Built<Solid> converts to Solid
var cut = BooleanOperations.Cut(block, hole);
if (!cut.IsSuccess) return cut.Error!.Message;
IReadOnlyList<Shape> newFaces = cut.Value.History.Generated(holeFace);
```

## 3. Operations, options, cancellation and progress

- **An operation is a static method**, in one static class per area named `<Area>Operations` (`BooleanOperations`,
  `FilletOperations`, `OffsetOperations`, `FeatureOperations`, `HealingOperations`, `MeasureOperations`), plus
  `Primitives`, `Sweeps`, `Mesher` and the exchange formats (`Step`, `Stl`…). design-rules.md's `Booleans.Fuse` can't
  be: a type may not be named after its namespace (question 53).
- **Each is declared once as a C# 14 extension member**, so it reads both ways:
  `BooleanOperations.Fuse(a, b)` and `a.Fuse(b)`; `Solid.Box(…)` for factories (static extension members). Importing
  an area's namespace brings its operations onto the shape types.
- **Signature order:** inputs; an options record (`XxxOptions? options = null`, `null` meaning `XxxOptions.Default`);
  `IProgress<double>? progress = null` where the operation can be long; `CancellationToken cancellationToken = default`.
- **Options are `sealed record`s** with `init` members whose defaults are the common case, and a static `Default`. No
  options record has a mutable member or a collection that isn't immutable.
- **Progress** reports a fraction from 0 to 1. Inside, the engine keeps `Message_ProgressRange`; one adapter makes a
  range from `(IProgress<double>?, CancellationToken)`, and the range's `UserBreak` becomes
  `cancellationToken.ThrowIfCancellationRequested()`.
- **Tolerances:** OCCT's `Precision` becomes `Kerf.Geometry.Tolerances` (`Confusion` 1e-7, `Angular` 1e-12,
  `Parametric`, `Approximation`); options take a `Tolerance` where OCCT takes one.

```csharp
public sealed record BooleanOptions
{
    public static BooleanOptions Default { get; } = new();
    public double FuzzyValue { get; init; }
    public GlueMode Glue { get; init; } = GlueMode.Off;
    public bool Parallel { get; init; } = true;
}

var fused = BooleanOperations.Fuse(a, b, new BooleanOptions { FuzzyValue = 1e-5 }, progress, cancellationToken);
```

## 4. Values: points, vectors, directions, axes, frames, transforms, boxes

**gp's structs become them directly**, renamed and made `readonly struct`s: they are already structs, and the engine
uses them everywhere, so a second set of values would mean a conversion at every boundary. 10L reshapes their members.

| OCCT | Kerf | | OCCT | Kerf |
|---|---|---|---|---|
| `gp_Pnt`, `gp_Pnt2d` | `Point3D`, `Point2D` | | `gp_Trsf`, `gp_Trsf2d` | `Transform3D`, `Transform2D` |
| `gp_Vec`, `gp_Vec2d` | `Vector3D`, `Vector2D` | | `gp_GTrsf`, `gp_GTrsf2d` | `GeneralTransform3D`, `…2D` |
| `gp_Dir`, `gp_Dir2d` | `Direction3D`, `Direction2D` | | `gp_Mat` | `Matrix3x3` |
| `gp_Ax1`, `gp_Ax2d` | `Axis3D`, `Axis2D` | | `gp_Lin`, `gp_Circ`, `gp_Elips`… | `Line3D`, `Circle3D`, `Ellipse3D`… (and 2D) |
| `gp_Ax2`, `gp_Ax22d` | `Frame3D`, `Frame2D` | | `gp_Pln`, `gp_Cylinder`, `gp_Cone`… | `Plane3D`, `Cylinder3D`, `Cone3D`, `Sphere3D`, `Torus3D` |
| `gp_Ax3` | `CoordinateSystem3D` | | `Bnd_Box`, `Bnd_OBB` (classes) | `BoundingBox`, `OrientedBox` (new structs) |

- **Engine, not public:** `gp_XYZ`/`gp_XY` (the coordinates inside points and vectors), `gp_Quaternion` (its name is
  System.Numerics's; rotations are `Transform3D.Rotation(…)` and `GetEulerAngles(…)`), the interpolators, `gp_Mat2d`.
- **Members:** `X`, `Y`, `Z` properties; `DistanceTo`, `AngleTo`; operators (`point + vector`, `point - point`,
  `transform * point`); `WithX(…)`; constants as static properties (`Point3D.Origin`, `Direction3D.Z`,
  `Frame3D.World`) in place of the `gp` class; exact `==`, and tolerant `IsEqual(other, tolerance)`.
- **`default`:** a default `Direction3D` or `Axis3D` is invalid (zero), and every API rejects it with an
  `ArgumentException`; points, vectors, transforms (identity is not default: `Transform3D.Identity`) are valid.
- **System.Numerics:** `ToVector3()`, `ToMatrix4x4()` conversions (single precision, for viewers), not inheritance.
- **The 3D suffix** goes on every gp value that is a 3D shape of space (`Plane3D`, `Cylinder3D`), not only where a
  2D twin exists as in the pilot, so none clashes with System.Numerics's `Plane` or Kerf.Modeling's `Plane`,
  `Cylinder` and `Sphere` (question 57).

## 5. Shapes

**`TopoDS_Shape` is the public `Shape`:** a `readonly struct` of a reference to the shared, immutable shape data (the
`TShape`, engine), a `Placement` and an `Orientation`. The typed sub-shapes stay structs of the same three fields.

```csharp
namespace Kerf.Topology;

public interface IShape<TSelf> where TSelf : struct, IShape<TSelf>
{
    static abstract ShapeType Type { get; }
    Shape AsShape();
}

public readonly struct Shape : IEquatable<Shape>
{
    public ShapeType Type { get; }
    public Orientation Orientation { get; }
    public Placement Placement { get; }
    public bool IsNull { get; }                       // default(Shape)
    public bool IsSame(Shape other);                  // same data and placement, either orientation
    public static bool operator ==(Shape a, Shape b); // IsEqual: same data, placement and orientation
    public bool TryAs<TShape>(out TShape shape) where TShape : struct, IShape<TShape>;
    public TShape As<TShape>() where TShape : struct, IShape<TShape>;   // InvalidCastException on the wrong type
}

public readonly struct Edge : IShape<Edge>, IEquatable<Edge>       // and Vertex, Wire, Face, Shell, Solid, CompSolid, Compound
{
    public static implicit operator Shape(Edge edge);
    public static explicit operator Edge(Shape shape);               // throws on the wrong type
}
```

- A typed sub-shape converts to `Shape` implicitly, and back explicitly or with `TryAs`; pattern matching works on
  `shape.Type`. Generic traversal (`shape.Explore<Face>()`) uses `IShape<T>`'s static `Type`.
- **Comparers:** `ShapeComparer.Same` and `ShapeComparer.Equal` (`IEqualityComparer<Shape>`), for dictionaries and sets;
  OCCTNET0002's rule (name the comparer) stays.
- **Published shapes are immutable.** Nothing in the public API changes one; operations return new shapes that share
  the sub-shapes they didn't change (10H). `Freeze`, `Locked` and `Checked` flags leave the public API.

[topology.md](topology.md) has the members: traversal, geometry access, placement, history.

## 6. Collections

- **0-based everywhere public** (a B-spline's first pole is `Poles[0]`).
- **Returns:** `ReadOnlySpan<T>` for numbers and small values the object owns (poles, knots, weights, mesh nodes), or
  `ReadOnlyMemory<T>` where the caller may keep it; `ImmutableArray<T>` for results built per call (the faces of a
  shape, a projection's solutions); `IReadOnlyList<T>`, `IReadOnlyDictionary`, `IReadOnlySet` where a type is lazy or
  shared. Bento accepts these as they are (question 50).
- **Parameters:** `ReadOnlySpan<T>` (and `params ReadOnlySpan<T>`) for points and numbers; `IEnumerable<T>` for shapes
  enumerated once.
- **2D grids** (a B-spline surface's poles, a grid of points to fit) are `Grid<T>`, a read-only struct over
  `ReadOnlyMemory<T>` with `RowCount`, `ColumnCount` and `this[row, column]`.

## 7. What the modelling layer needs, and the 15 `UnsafeAccessor`s

Every OCCT type the modelling layer uses has a public replacement in its area's mapping table (and in the TSV's
`replacement` column). The 15 `UnsafeAccessor`s become ordinary API:

| Today (reached by `UnsafeAccessor`) | Public replacement |
|---|---|
| `TopoDS_TShape.FreezeGraph` | Not needed: every shape the public API returns is published |
| `TopoDS_Builder` constructor, `MakeWire/Shell/Solid/CompSolid/Compound`, `Add` | `Compound.Create(shapes)`, `Wire.Create(edges)`, `Shell.Create(faces)`, `Solid.Create(shells)`, `CompSolid.Create(solids)`: raw assembly, unchecked (topology.md) |
| `TopoDS_Builder.Remove` | `shape.WithChildren(…)`, `shape.WithoutChild(child)` |
| `BRepTools_History`'s three maps | `ShapeHistory`'s read API (`Entries`, `Modified`, `Generated`, `IsRemoved`) and `ShapeHistory.Compose` |
| `BRep_Builder` constructor and `MakeFace(face, triangulation)` | `Face.FromMesh(FaceMesh)` (meshing.md) |

The 3,300 hand-written runtime lines that 10B moves (mesh extraction, the exchange glue, colours, history access) land
as `Mesher`/`ShapeMesh`, `Kerf.Exchange`'s readers and writers with `ModelNode`, `ColorRgba` and `ShapeHistory`.

## 8. Bento

Bento's 3D models extension uses only the modelling layer. What it needs from the kernel, through that layer:
- **Errors** as `Result<T>` with plain messages (section 2), so `#VALUE!` shows the reason.
- **Meshes** as flat `float`/`int` buffers per face and polylines per edge, with face and edge counts
  (`ShapeModels.Part` builds its `CadMesh` from them): `ShapeMesh` (meshing.md) gives exactly those spans.
- **Exchange** from streams with read limits and cancellation, writes approved later: `Kerf.Exchange` (exchange.md)
  takes `Stream`, `ReadLimits` and `CancellationToken` as `Importers`/`Exporters3D` do today.
- **Colours and names** on assemblies: `ModelNode` and `ColorRgba`.
- **Names:** its spreadsheet names follow Kerf.Modeling's (question 48), so the kernel's names reach it only where the
  modelling layer re-exports a kernel type (`Point3D`, `Axis3D`, `Plane3D`).

## 9. Open questions for the owner

53. **Operations' entry points.** One static class per area named `<Area>Operations` (`BooleanOperations.Fuse(a, b)`),
    each operation declared once as a C# 14 extension member so `a.Fuse(b)` works too; factories as static extension
    members (`Solid.Box(…)`, `Edge.Line(p, q)`). design-rules.md's examples `Booleans.Fuse` and `Fillet.Apply` need
    correcting: the first names a type after its namespace. Alternatives: static classes only, without extension
    members; or shorter class names per operation family (`Fillet.Apply`, `Boolean`-free names such as
    `Combine.Fuse`). Agree `<Area>Operations` plus extension members?
54. **Engine visibility.** 479 engine types cross between the kernel's assemblies. Proposed: keep
    `InternalsVisibleTo` among the four kernel assemblies (one product, one version), and to their tests and Draw-lite;
    give it temporarily to Kerf.Modeling until 10M moves the modelling layer onto the public API. Each area's engine
    becomes internal at the end of its 10L step, not in 10E (section 1.1). The alternative is merging the four
    assemblies into one `Kerf.Kernel.dll` (1.2 million lines; slower incremental builds), which would need no
    `InternalsVisibleTo` at all. Which?
55. **`Standard_Transient` stays public** as `Kerf.Engine.Foundation.Transient`, with no public members, because the
    public Geom classes derive from it and the engine's handles (`Transfer`, the BRep representations) need the common
    base. Alternatively 10L re-bases the Geom classes on a new public `GeometryBase`, a larger change. Accept it?
56. **Kernel and modelling names.** The kernel's `Shape`, `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid` and
    `Compound` (from TopoDS) share their names with build123d.NET's classes, so a file importing both namespaces is
    ambiguous. Proposed: the kernel keeps the names, and 10M has the modelling layer use the kernel's shapes directly,
    adding its conveniences as extension members and keeping only its own typed compounds (`Part`, `Sketch`, `Curve`)
    and placement types; this is the plan's "uses OCCT.NET's types directly, without wrappers". Otherwise the kernel's
    take a prefix (`TopoShape`, `TopoEdge`). Which?
57. **Geometry names.** gp's analytic values take a `3D` suffix throughout (`Plane3D`, `Cylinder3D`, `Sphere3D`), not
    only where a 2D twin exists (the pilot's rule), and `gp_Ax3` is `CoordinateSystem3D`; Geom's curves are
    `…Curve3D`/`…Curve2D` (`CircleCurve3D`, `BSplineCurve3D`) and its surfaces `…Surface` (`PlaneSurface`,
    `RevolutionSurface`). Agree, or prefer a namespace `Kerf.Geometry.Curves` and unsuffixed names?
58. **Removed types keep their OCCT names** in the namespace `Occt` until 10J deletes them, rather than 10E renaming
    what is about to go. Agree?
59. **Meshing leaves shapes alone.** `Mesher.Mesh` returns a separate `ShapeMesh` rather than writing triangulations
    into the faces as OCCT does, which is a change of input; exporters take the mesh. A face that is only a mesh (STL)
    is still a face (`Face.FromMesh`). Agree?
60. **Colour:** `Kerf.Exchange.ColorRgba`, a `readonly record struct` of four `float`s in linear RGB (OCCT's model),
    with `FromSrgb`/`ToSrgb`. `Color` is System.Drawing's name. Agree the name, the home and linear storage?
61. **Diagnostic codes** as strings in a struct (`"boolean.self-interference"`), extensible per area and stable for
    Bento, rather than one large enum. Agree?
62. **`Placement`** (from `TopLoc_Location`) is public, with OCCT's identity-based equality (two placements made from
    equal transforms aren't equal), because shape equality depends on it. The alternative exposes only `Transform3D`
    and keeps `Placement` in the engine, at the cost of `IsSame` becoming harder to explain. Which?
63. **STEP's entity classes become internal**, so 10G's 1,556 STEP setters are engine hygiene rather than public API.
    Drop them from 10G, or keep the `init`-and-builders work there?
64. **No public hidden-line API yet.** HLR (102 engine types) has no public face; nothing in the modelling layer or
    Bento uses it. Add one (2D drawings from 3D) later, as its own review?
65. **New namespaces** beyond question 46's: `Kerf.Fillets`, `Kerf.Offsets`, `Kerf.Features`, `Kerf.Healing` and
    `Kerf.Measurement` (not `Kerf.Properties`, which collides with project `Properties` folders). Agree?
