# API review: offsets and features

*Unit 10D, draft. Namespaces `Kerf.Offsets` and `Kerf.Features`, in ModelingAlgorithms. Engine: `Kerf.Engine.Offsets`
(BRepOffset, BRepOffsetAPI's offsets, BiTgte, MAT, MAT2d, BRepMAT2d, Bisector) and `Kerf.Engine.Features` (BRepFeat,
LocOpe, Draft, BRepOffsetAPI_DraftAngle).*

## Main types

New: `OffsetOperations`, `OffsetOptions`, `OffsetMode` (Skin, Pipe, RectoVerso), `JoinKind` (Arc, Tangent,
Intersection; from `GeomAbs_JoinType`), `Offset2DOptions`; `FeatureOperations`, `DraftOptions`, `ExtrudeUntilOptions`,
`SplitFacesOptions`.

```csharp
public static class OffsetOperations   // extension members on Shape, Face, Wire
{
    // BRepOffset_MakeOffset / BRepOffsetAPI_MakeOffsetShape: every face moved by distance
    public static Result<Built<Shape>> Offset(Shape shape, double distance, OffsetOptions? options = null,
                                              IProgress<double>? progress = null, CancellationToken cancellationToken = default);
    // BRepOffsetAPI_MakeThickSolid: a hollow solid, the given faces removed as openings
    public static Result<Built<Solid>> Hollow(Solid solid, IEnumerable<Face> openings, double thickness, OffsetOptions? options = null, …);
    // BRepOffset_MakeOffset with thickening: a face or shell made a solid
    public static Result<Built<Solid>> Thicken(Shape faceOrShell, double thickness, OffsetOptions? options = null, …);
    // BRepOffsetAPI_MakeOffset: planar wires and faces offset in their plane
    public static Result<Built<Shape>> Offset2D(Shape wireOrFace, double distance, Offset2DOptions? options = null);
}

public sealed record OffsetOptions
{
    public static OffsetOptions Default { get; } = new();
    public double Tolerance { get; init; } = 1e-4;
    public OffsetMode Mode { get; init; } = OffsetMode.Skin;
    public JoinKind Join { get; init; } = JoinKind.Arc;
    public bool Intersection { get; init; }
    public bool SelfIntersection { get; init; }
    public bool RemoveInternalEdges { get; init; }
}

public sealed record Offset2DOptions { public JoinKind Join { get; init; } = JoinKind.Arc; public bool Open { get; init; } }

public static class FeatureOperations
{
    // BRepOffsetAPI_DraftAngle
    public static Result<Built<Shape>> Draft(Shape shape, IEnumerable<Face> faces, Direction3D pull, double angle,
                                             in Plane3D neutralPlane, DraftOptions? options = null);
    // BRepFeat_MakeDPrism: a profile on a face extruded with a taper up to a face, or through all
    public static Result<Built<Shape>> ExtrudeUntil(Shape shape, Face profile, Face sketchFace, double taper,
                                                    ExtrudeUntilOptions options);   // Until: Next, Last, Face; Fuse or Cut
    // BRepFeat_SplitShape: faces split by wires or edges lying on them
    public static Result<Built<Shape>> SplitFaces(Shape shape, IEnumerable<(Face Face, Shape Splitter)> splits);
}
```

- **Diagnostics:** `offset.self-intersection`, `offset.unknown-error`, `offset.not-planar` (2D),
  `offset.c0-geometry` (OCCT's `BRepOffset_Error` values, by name); `draft.face-failed` (with the face),
  `feature.no-limit-face`.

## Examples

```csharp
Solid shelled = OffsetOperations.Hollow(box, [topFace], thickness: -2).Value;   // negative: inwards
```

```csharp
var thick = OffsetOperations.Thicken(surfaceFace, 1.5, new OffsetOptions { Join = JoinKind.Intersection });
```

```csharp
Shape outline = sketchFace.Offset2D(3, new Offset2DOptions { Join = JoinKind.Tangent }).Value;
```

```csharp
var drafted = FeatureOperations.Draft(part, sideFaces, Direction3D.Z, angle: 3 * Math.PI / 180, basePlane);
```

```csharp
var boss = FeatureOperations.ExtrudeUntil(part, profileFace, sketchFace, taper: 0,
                                          new ExtrudeUntilOptions { Until = Until.Next, Fuse = true });
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `BRepOffset_MakeOffset.Initialize(shape, amount, tol, BRepOffset_Skin, inter, selfInter, join…)`, `MakeOffsetShape()` | `OffsetOperations.Offset` with `OffsetOptions` |
| `BRepOffsetAPI_MakeThickSolid.MakeThickSolidByJoin(…)` | `Hollow` |
| `BRepOffset_MakeOffset` with `MakeThickSolid` on a face | `Thicken` |
| `BRepOffsetAPI_MakeOffset` (`Init(join, open)`, `AddWire`, `Perform(amount)`) | `Offset2D` |
| `BRepOffset_Mode`, `GeomAbs_JoinType` | `OffsetMode`, `JoinKind` |
| `BRepOffsetAPI_DraftAngle` (`Add(face, dir, angle, plane)`, `Build`, `ProblematicShape`) | `FeatureOperations.Draft`; the failing face in the diagnostic |
| `BRepFeat_MakeDPrism` (`Perform(until)`, `PerformUntilEnd`, `PerformThruAll`) | `ExtrudeUntil` with `ExtrudeUntilOptions` |
| `BRepFeat_SplitShape` (`Add(wire, face)`, `Build`) | `SplitFaces` |

**Disappears:** `BRepOffset_Error` and `BRepOffset_MakeOffset.Error()`, `BRepFeat_Form`'s status methods,
`LocOpe` (engine only), BiTgte's pipe fillets (engine only).

## Open points

- Two namespaces for one review: offsets change every face; features are local edits. Merge into `Kerf.Features` if
  the owner prefers fewer namespaces.
- `Hollow` is build123d's "offset with openings" and OCCT's "thick solid"; the name follows the result.
