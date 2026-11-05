# One set of shapes (unit 10Q, question 56)

The owner's answer: "the modelling layer uses the kernel's shapes directly (`Shape`, `Edge`, `Face` and so on), and keeps
its own names only for modelling ideas (`Part`, `Sketch`, `Curve`). One set of shapes, not two." Decisions: unit 10Q's
section of `kernel/docs/decisions.md`, and unit 10Q2's for the values and names. The change tables are [10q-bento.md](10q-bento.md) and
[10q2-bento.md](10q2-bento.md).

## The design

- **Topology is the kernel's.** `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `Compound` and `Shape` are
  `Kerf.Topology`'s structs. `Kerf.Modeling` adds build123d's conveniences to them as C# 14 extension members.
- **Modelling objects are classes.** `ModelShape` (new) is the base of `Part`, `Sketch` and `Curve` (and `Box`, `Circle`,
  `Line`…) and of `Assembly` (new). It holds a kernel shape and carries what a topological shape can't: label, colour,
  construction flag, history, assembly children. It converts implicitly to `Shape`, so it goes wherever a shape goes.
- **Small kernel interfaces make one extension block serve both.** `IShape` (non-generic, `AsShape()`), and the markers
  `IShape1D` (`Edge`, `Wire`), `IShape2D` (`Face`, `Shell`) and `IShape3D` (`Solid`, `CompSolid`, `Compound`).
  An extension written `extension<T>(T s) where T : IShape1D` is on an edge and a wire without boxing, and on a `Line`.
  They replace build123d's `Mixin1D`, `Mixin2D` and `Mixin3D`.
- **A shape's provenance moves to the selection.** A struct can't remember where it came from. `ShapeList<T>` does:
  `part.Edges()` is a list whose `Origin` is the part, and every selection on it keeps that. An operation that needs the
  parent reads the list's origin, or takes the parent as an argument. Nothing is hidden on a single shape any more.

## Each type

| Before | After | Why |
|---|---|---|
| `Shape`, `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `Compound` | Dropped: the kernel's structs, with extension members | The owner's answer |
| `Shape1D`, `Shape2D`, `Shape3D` | Dropped: `IShape1D`, `IShape2D`, `IShape3D` in the kernel | Parameters and extension receivers need a type; a kernel struct can't have a base class |
| `Part`, `Sketch`, `Curve` | Kept, as classes over `ModelShape`, no longer compounds by inheritance | Modelling ideas |
| `Compound.Assembly(...)`, `AssemblyChildren` | Moved: `Assembly : ModelShape` | A tree of labelled, coloured children is a modelling idea |
| `BasePartObject`, `BaseSketchObject`, `BaseCurveObject` | Kept | Modelling objects |
| `BaseEdgeObject : Edge`, `BaseLineObject : Wire` | Now derive from `Curve` and hold an edge or wire; convert implicitly to `Edge` or `Wire` | A class can't derive from a struct. A `Line` is a modelling object |
| `ShapeType` (modelling) | Dropped: the kernel's | Same meaning |
| `ShapeHistory` (modelling) | Renamed `ModelHistory` | Adds "already there" and "brought in" over the kernel's `ShapeHistory`; the name clashed |
| `ShapeList<T>`, `ShapeGroups<T, TKey>` | Kept, `T : IShape`; gain `Origin` | Selection is a modelling idea |
| `ShapeLog` | Dropped, with `tools/b3d-differential.ps1` | It logged each new wrapper; there is no wrapper. Native OCCT went in 10B, so the run's value was already the frozen recording, which `Build123dDifferentialTests` still checks |

## Each piece of state

| State | Where it goes | Why |
|---|---|---|
| `Label`, `Color`, `ForConstruction` | `ModelShape` | Attributes of a modelling object. `solid.WithLabel("a")` gives `ModelShape.For(solid)` with the label: a `Part` |
| Material | Doesn't exist in the code | Nothing to move |
| `CreatedOn` (on a face) | Dropped | Nothing read it |
| `TopoPath`, `TopoParent`, `TopoOwner` | Dropped from shapes; `ShapeList<T>.Origin` | See below |
| `History`, `_historyAt` | `ModelShape` | `Select.Last` and `Select.New` are asked of a `Part`, `Sketch` or `Curve`. A struct's operation returns the bare shape |
| Assembly parents and children | `Assembly.Children` | Downward only, as before |
| A subclass's parameters (a `Box`'s `Length`) | The object, kept by `MemberwiseClone` as before | Unchanged |
| The empty "zero of a class" | `default(Solid)` and so on; `IsEmpty` also true of an empty compound | `new Solid()` was the zero; the default struct is. Once it is a plain `Shape` it has no class; a `Part` keeps its own |
| `Shape.KernelShape` | Dropped | A shape *is* the kernel's now; `ModelShape.Shape` holds it for an object |

## Keeping the Python-pinned behaviour

- **`Select.Last`, `Select.New`.** The same record, the same classification. The operations are written once, on kernel
  shapes, and note their record (`shape.MadeBy(record)`) in a `HistoryLog` when one is open: an internal, `AsyncLocal`
  scope, as `Warnings` and `SkipClean` are. An operation of a `Part`, `Sketch` or `Curve` opens one and keeps the record it
  finds on its result. A struct's operation, with none open, computes the record as before and drops it.
  This is the one piece of ambient state in the design. Unit 10Q2 looked again and kept it: `kernel/docs/state.md` says why.
- **Which side of an edge a face is on; convexity.** It used to read the parent from the sub-shape. Now the parent is
  said: `box.ConvexityOf(edge)`, `box.ConvexityOf(vertex, through: face)`; `edges.Where(Convexity.Concave)` reads the
  list's origin. Same code underneath, same answers.
- **`Fillet`, `Chamfer`, `Draft`, `ConnectedEdges`, `FullRound`, topological distance.** The same: from the list's
  origin, or an explicit parent.
- **Result classes.** A reference test that pinned a class (`Assert.IsType<Solid>`) now pins the kernel's `Type`
  (`Check.Is<Solid>`). A composite `Part` of pieces from `solid - solid` is a kernel `Compound`, and the reference data's
  `Part`, `Sketch` and `Curve` map to `Compound`.

## Before and after, for the tutorials

Modelling code hardly changes: objects are still classes, and `Part` still has its operators and selections.

```csharp
// Unchanged
Part plate = new Box(80, 60, 10);
plate -= new GridLocations(60, 40, 2, 2) * new Cylinder(5, 10);
Result<Part> rounded = Fillet(box.Edges().ParallelTo(Axis.Z), 3);
```

Topological code uses the kernel's structs, and says what it means:

```csharp
// Before                                           // After
Solid s = Solid.MakeBox(1, 2, 3);                   Solid s = Solid.MakeBox(1, 2, 3);        // the kernel's Solid
Face f = new Face(outer, [hole]);                   Face f = Face.Make(outer, [hole]);       // no extension constructors
Vertex v = new Vertex(1, 2, 3);                     Vertex v = Vertex.Make(1, 2, 3);
Edge e = s.Edges().OrderBy(Axis.Z).Last();          Edge e = s.Edges().OrderBy(Axis.Z).Last();
Convexity c = e.Convexity;                          Convexity c = s.ConvexityOf(e);
s.WithLabel("bracket").WithColor(red);              s.WithLabel("bracket").WithColor(red);   // now a Part
Compound.Assembly([a, b], "top");                   new Assembly([a, b], "top");
if (shape is Face face) …                           if (shape.TryAs(out Face face)) …
```

**A struct and an object read alike (unit 10Q2).** 10Q gave the layer's forms other names, as a struct's own member won. That left
traps: `part.Edges().Where(e => e.IsClosed)` compiled and read OCCT's stored flag. So the kernel's names moved instead.

| Name | Now | The kernel's other meaning |
|---|---|---|
| `Edges()`, `Faces()`… | The selection, on a struct and an object alike: a `ShapeList`, with its origin, without degenerate edges | `AllEdges()`, `AllFaces()`…: an `ImmutableArray`, with them. They can't be one method, as `ShapeList` is a modelling idea |
| `IsClosed` | Closed in fact, on any shape or object | `ClosedFlag`, internal: nothing outside the engine needs what a builder set |
| `IsInfinite` | The layer's, an axis made into an edge | `InfiniteFlag`, internal |
| `IsOrientable`, `IsConvex` | Not public | `OrientableFlag`, `ConvexFlag`, internal |
| `Orientation` | The kernel's enum, which is what the word means on a shape. The layer's angles stay `OrientationAngles`, the same on both | |
| `Volume`, `Area`, `Length` | The layer's, as properties | `IntegrateVolume()`, `IntegrateArea()`, `IntegrateLength()`: what the faces bound, whatever the shape is, with options |

**Equality.** `==` on a kernel shape includes orientation; `IsSame` doesn't. The layer's old `==` was `IsSame`. Inside
the layer every set and comparison names `ShapeComparer<T>.Same` (new in the kernel), and `ShapeList` compares that way.
`shape.SameAs(other)` is the layer's: two empty shapes are the same nothing.

**A switch's natural type.** `shape.Type switch { ShapeType.Edge => (Edge)shape, … }` assigned to `object` boxes every arm as
`Shape`, as they all convert to it. `TopoUtil.Typed` uses a statement. Likewise
`OfType<Face>()` on shapes finds nothing (CA2021 catches that one; `OfKind<Face>()` is the form that works).

## The duplicate values

| Type | Decision | Why |
|---|---|---|
| `Color` | Replaced by `Kerf.Color`, with `ColorExtensions` | Question 60 |
| `Vector` | Replaced by `Point3D`, `Vector3D` and `Direction3D`, with `PointExtensions` (unit 10Q2) | Below |
| `BoundBox` | Replaced by `BoundingBox`, with `BoundingBoxExtensions` (unit 10Q2) | Below |
| `Matrix` | Replaced by `GeneralTransform3D`, with `TransformExtensions` (unit 10Q2) | Below |
| `Tangency` | Replaced by the kernel's (unit 10Q2) | The same four values |
| `Plane` | Kept | A modelling idea: build123d's named planes, `plane * shape`, `plane * location`, local coordinates. `Plane3D` is the kernel's surface value |
| `Location`, `Rotation`, `Pos` | Kept | A modelling idea: `location * shape`, Euler orientation, build123d's algebra. It wraps `Placement` |
| `Axis` | Kept | Not a duplicate in the audit; a selection key with build123d's named axes |

## Points, vectors and directions (unit 10Q2)

- **By meaning.** A position is a `Point3D`, an offset or a computed direction a `Vector3D`. A `Direction3D` is a unit direction that came
  from the kernel: an axis's, a plane's. `TangentAt` and `NormalAt` stay `Vector3D`s: they are computed, and making them directions would
  normalise them again and move their last bits.
- **Two additions to the kernel make that readable.** A `Direction3D` converts implicitly to a `Vector3D`, as nothing is lost, so one
  parameter takes both; and it scales to one (`plane.ZDir * amount`).
- **Tolerance has a name.** The kernel's `==` is exact and unchanged. What the old `Vector` called equal is `IsNear`, and `PointComparer.Near`
  for sets; `PointOrder`, which keeps CPython's set order, compares with it. The compiler can't find a comparison that needs it, so every
  `==`, `!=` and `/` on the old type was listed by marking those operators obsolete and building (47 in the library, 11 in the tests).
- **The same bits.** The old `/` multiplied by `1.0 / n`, where the kernel divides. Those sites write it out where `n` isn't a power of two.
  A vector's `Normalized`, cross and dot are the same sums. `ToDirection()` makes a direction again from its components, as the old
  `ToDirection3D` did and build123d's `gp_Dir(*v)` does, where a value went through it twice.
- **`Intersect` returns an `IGeometry?`.** It can be a point, axis, plane or location, so it stays one value to pattern-match:
  `a.Intersect(b) is Point3D p`. The marker moved to `Kerf.Geometry`, and `Point3D` implements it. `object` would have lost what the result can be;
  a union type would have added a type for callers to unwrap and left the pattern worse.
- **No two-argument constructor, no indexer.** `new Point3D(1, 2, 0)` says its z. An extension can't add either.

**`BoundBox` is the kernel's `BoundingBox`, and its gap went.** Only a box begun from nothing by `Add(point, tolerance)` had a gap; every
shape's box was made with none, so none reached a result. `Min` and `Max` of an empty box throw, where the layer's gave the origin; the
layer's `Center`, `Diagonal`, `Measure` and `ToAlignOffset` read an empty box as nothing, and `Section` checks. A transformed box's corners
are `BoxCorners`, not a box: build123d keeps them as they land, so `Min` can be above `Max`, and its `test_locate_bb` pins that.

**`Matrix` is the general form, `GeneralTransform3D`.** `Transform3D` can't hold a stretch. `Plane.ForwardTransform` stays general, so a point
goes through the same sums. The layer's product is `Compose`, as the kernel's `*` is OCCT's, which loses a similarity's scale beside a general
matrix; `At` counts from zero, as the kernel's indexer counts from one.

## Left

- **The objects don't have every edge's member.** `line.Length` and the other `IShape1D` members work on a `Line`; an `Edge`-only member
  needs `line.Edge.ArcCenter`.
- **Names kept from the mechanical move.** The dispatch's targets (`CenterOfShape1D`, `IntersectCoreOfFace`) are internal, and read
  oddly. They stay (decision 1668).
- **Other namespaces still meet the layer's names.** `Kerf.Meshing`'s `FaceMesh`, `ShapeMesh` and `Triangle`, and `Kerf.Exchange`'s
  `Step`. A file with both qualifies one, and that stays (decision 1668). The options are nested in `Operations`.
- **`shape.Bounds()` beside the kernel's `shape.BoundingBox()`.** The same type, with different jobs: tight, falling back on a mesh,
  and fast and loose. It was `BoundBox()` until 0.3.0 (decision 1668).
- **The `HistoryLog`.** Kept; see `kernel/docs/state.md`.
