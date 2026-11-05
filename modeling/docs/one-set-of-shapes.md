# One set of shapes (unit 10Q, question 56)

The owner's answer: "the modelling layer uses the kernel's shapes directly (`Shape`, `Edge`, `Face` and so on), and keeps
its own names only for modelling ideas (`Part`, `Sketch`, `Curve`). One set of shapes, not two." Decisions: unit 10Q's
section of `kernel/docs/decisions.md`. The whole change table is [10q-bento.md](10q-bento.md).

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
  This is the one piece of ambient state in the design. The alternative was a `(shape, record)` result threaded through about
  thirty methods; the note exists so that a later unit can change its mind.
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
Edge e = s.Edges().OrderBy(Axis.Z).Last();          Edge e = s.SelectEdges().OrderBy(Axis.Z).Last();
Convexity c = e.Convexity;                          Convexity c = s.ConvexityOf(e);
s.WithLabel("bracket").WithColor(red);              s.WithLabel("bracket").WithColor(red);   // now a Part
Compound.Assembly([a, b], "top");                   new Assembly([a, b], "top");
if (shape is Face face) …                           if (shape.TryAs(out Face face)) …
```

**`SelectEdges`, not `Edges`.** The kernel's instance method `solid.Edges()` wins over any extension, gives an
`ImmutableArray<Edge>` and includes degenerate edges (a sphere's poles). build123d's selection leaves those out and
returns a list with an origin. So on a struct it has its own name (`SelectVertices`, `SelectEdges`, `SelectWires`,
`SelectFaces`, `SelectShells`, `SelectSolids`); on a `ModelShape`, `Edges()` is as it was.

**The same rule moved three other names.** `edge.IsClosed` on a struct is the kernel's stored flag: use the kernel's
`CheckClosed()`. The layer's `IsInfinite` is `IsUnbounded`, and its `Orientation` (angles) `OrientationAngles`.

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
| `Plane` | Kept | A modelling idea: build123d's named planes, `plane * shape`, `plane * location`, local coordinates. `Plane3D` is the kernel's surface value |
| `Location`, `Rotation`, `Pos` | Kept | A modelling idea: `location * shape`, Euler orientation, build123d's algebra. It wraps `Placement` |
| `Axis` | Kept | Not a duplicate in the audit; a selection key with build123d's named axes |
| `Vector`, `BoundBox`, `Matrix` | Not replaced: see below | The brief: the shapes are the commitment |

## Left

- **`Vector` is still `Vector`.** It should become `Vector3D` with extension members; it wasn't reached. It isn't a mechanical
  rename, which is why it is left rather than rushed: about 2,700 uses; it stands for both a point and a direction; its equality
  has a tolerance where the kernel's is exact, and `PointOrder` and the Python-reference tests rest on that; a two-argument
  constructor (`new Vector(1, 2)`) has no extension form; and `Intersect` returns it as an `IGeometry`.
- **`BoundBox` and `Matrix`** follow `Vector`: `BoundBox`'s `Min`, `Max` and `Center()` are vectors, and `Matrix`'s `Multiply` takes
  one. `BoundBox` also keeps a shape's gap, which the kernel's `BoundingBox` doesn't.
- **`Kerf.Measurement` beside `Kerf.Modeling`.** In a file with both, `solid.Volume`, `.Area` and `edge.Length` are ambiguous with the
  kernel's methods `Volume()`, `Area()` and `Length()`. The meanings differ (a face's volume is 0 here; a compound's counts the solids
  directly in it), so the layer's can't simply go. `Kerf.Topology` itself is clean, which is what the owner's answer asked: see
  `OneSetOfShapesTests`, which imports it with `Kerf` and `Kerf.Modeling` at file level.
- **`Kerf.Geometry` beside `Kerf.Modeling`.** `Cylinder`, `Cone`, `Sphere` and `Torus` are the kernel's surface values and the layer's
  objects. Different ideas; a file using both qualifies one.
- **The objects don't have every edge's member.** `line.Length` and the other `IShape1D` members work on a `Line`; an `Edge`-only member
  needs `line.Edge.ArcCenter`.
- **Names kept from the mechanical move.** The dispatch's targets (`CenterOfShape1D`, `IntersectCoreOfFace`) are internal, and read
  oddly.
