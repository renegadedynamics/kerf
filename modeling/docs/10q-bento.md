# Unit 10Q: what Bento sees change

Unit 10Q moved `Kerf.Modeling` onto the kernel's shapes (question 56: "One set of shapes, not two"; kernel decisions for
unit 10Q; [one-set-of-shapes.md](one-set-of-shapes.md)). Results are unchanged: volumes, face counts, STEP bytes and the
Python-reference values are the same, and Bento's benchmark gives the same numbers no slower. This is every change a
caller meets. **A member not listed keeps its name and meaning**, on the kernel's struct as an extension member:
`edge.PositionAt(0.5)`, `face.NormalAt()`, `solid.Fillet(…)`, `Solid.MakeBox(…)`, `Edge.MakeLine(…)`, `Wire.MakeRect(…)`,
`Face.MakeRect(…)`.

Importing: `using Kerf.Topology;` as well as `using Kerf.Modeling;`.

## Types

| Before (`Kerf.Modeling`) | After |
|---|---|
| `Shape`, `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `Compound` | The kernel's structs in `Kerf.Topology` |
| `Shape1D`, `Shape2D`, `Shape3D` | The kernel's interfaces `IShape1D` (`Edge`, `Wire`), `IShape2D` (`Face`, `Shell`), `IShape3D` (`Solid`, `CompSolid`, `Compound`); `IShape` for any shape |
| `ShapeType` | The kernel's `Kerf.Topology.ShapeType` (it adds `Shape`) |
| `ShapeHistory` | `ModelHistory` (the kernel's `ShapeHistory` is what it wraps) |
| `Color` | The kernel's `Kerf.Color`, with `ColorExtensions` |
| `Part`, `Sketch`, `Curve` | Classes over the new `ModelShape`, no longer compounds by inheritance |
| `BasePartObject`, `BaseSketchObject`, `BaseCurveObject`, and `Box`, `Circle`… | The same |
| `BaseEdgeObject : Edge`, `BaseLineObject : Wire` (`Line`, `Polyline`, `ThreePointArc`…) | Derive from `Curve`, hold the edge or wire (`.Edge`, `.Wire`), convert implicitly to `Edge` or `Wire` |
| `Compound` with `AssemblyChildren` | `Assembly : ModelShape`, with `Children` |
| `ShapeOperations` (for `T : Shape`) | `ModelShapeOperations` (for `T : ModelShape`) and `ShapeOperations` (for the structs) |
| (none) | `ModelShape`, `Assembly`, `ShapeOperators`, `VertexOperators`, `ColorExtensions`, `Shape1DExtensions` and the other `*Extensions` classes |

## Making shapes

An extension can't add a constructor, so the constructors of topological shapes are `Make`:

| Before | After |
|---|---|
| `new Vertex(x, y, z)`, `new Vertex(vector)` | `Vertex.Make(x, y, z)`, `Vertex.Make(vector)` |
| `new Edge()`, `new Solid()`… (the zero) | `default(Edge)`, `default(Solid)`… |
| `new Wire(edges)`, `new Wire(edge)`, `new Wire(curve)` | `Wire.Make(edges)`, `Wire.Make(edge)`, `Wire.Make(curve)` |
| `new Face(outer, inner, surface)` | `Face.Make(outer, inner, surface)` |
| `new Shell(faces, tolerance: t)`, `new Shell(face)` | `Shell.Make(faces, t)`, `Shell.Make(face)` |
| `new Solid(shell)` | `Solid.Make(shell)` |
| `new Compound(shapes)` | `Compound.Make(shapes)` |
| `new Edge(kernelEdge)`, `Shape.Cast(kernelShape)` | Nothing to do: it is the kernel's already |
| `Shape.AsShape(geometry)` | `ShapeExtensions.ShapeOf(geometry)` |
| `new Face(…, label: "a", color: c)` and any label or colour argument of a topological shape's constructor | `Face.Make(…).WithLabel("a").WithColor(c)`, an object (below) |
| `Compound.Assembly(children, label, color)` | `new Assembly(children, label, color)` |
| `compound.WithAssemblyChildren(c)` | `assembly.WithChildren(c)` |
| `new Part(shape)`, `new Part(shapes)`, the same for `Sketch` and `Curve` | The same. `new Curve((Shape)edge)` holds an edge |

## Attributes and history

| Before | After |
|---|---|
| `solid.Label`, `.Color`, `.ForConstruction`, `.History` | On `ModelShape` only. `solid.WithLabel("a")`, `.WithColor(c)`, `.WithForConstruction()` give the object of the shape's dimension: `ModelShape.For(solid)`, a `Part`, `Sketch`, `Curve`, or an `Assembly` leaf for a vertex or a mixed compound |
| `solid.Faces(Select.Last)` and `Select.New` on a topological result | `new Part(solid) - tool`, then `.Faces(Select.Last)`: a record is kept by a `Part`, `Sketch` or `Curve`. `Operations`' results carry it as before. `Curve`'s `+` joins as an edge's did |
| `shape.KernelShape` | `model.Shape`; on a struct, the struct |
| `face.CreatedOn`, `face.WithCreatedOn(p)` | Gone: nothing read it |
| Material | Never existed |

## Selections

| Before | After |
|---|---|
| `part.Edges()`, `.Faces()`… on a `Part`, `Sketch`, `Curve`, `Box`… | The same: a `ShapeList<Edge>`, of the kernel's `Edge` |
| `solid.Edges()`, `.Faces()`, `.Vertices()`, `.Wires()`, `.Shells()`, `.Solids()` on a topological shape | `solid.SelectEdges()`… A struct's own `Edges()` is the kernel's, an `ImmutableArray<Edge>` with degenerate edges (a sphere's poles), and it hides an extension of that name |
| `edge.TopoParent`, `.TopoOwner`, `.TopoPath` | `list.Origin`, the shape a list was selected from. Every selection on a list keeps it (`Where`, `OrderBy`, `ParallelTo`, `GroupBy`, slices, `Except`); LINQ's `Select`, `ToList` and a single element don't |
| `ShapeList<T>.Upcast<TBase>()` | `Upcast()`, a `ShapeList<Shape>` |
| `T : Shape` on `ShapeList<T>` and the selections | `T : IShape` |
| `list.Contains(edge)`, `Distinct()`, `IndexOf`, `==` on shapes | `==` and default equality on the kernel's shapes include orientation; the layer's `==` didn't. Use `IsSame`, `shape.SameAs(other)`, `ShapeComparer<Edge>.Same`, `ShapeComparer.Same`. `ShapeList`'s own `Intersect`, `Except` and `==` compare with `Same` |

## What needed the parent on the sub-shape

| Before | After |
|---|---|
| `edge.Convexity`, `vertex.Convexity` | `parent.ConvexityOf(edge)`, `parent.ConvexityOf(vertex, through: face)`. `face.Convexity` reads the face's own surface and stays |
| `edge.IsInterior` | `parent.ConvexityOf(edge) == Convexity.Concave` |
| `edges.Where(Convexity.X)`, `.WhereNot`, `.GroupByConvexity()` | The same on a list with an `Origin`; otherwise `InvalidOperationException` |
| `Fillet(edge, r)`, `Chamfer(edge, l…)`, `Draft(face, plane, a)`, and their vertex forms | `Fillet(parent, edge, r)`, `Chamfer(parent, edge, l…)`, `Draft(parent, face, plane, a)` |
| `Fillet(edges, r)`, `Chamfer(edges, l…)`, `Draft(faces, plane, a)` | The same with a list with an `Origin`; `Fillet(parent, edges, r)` for any other sequence |
| `Fillet(vertices, r)`, `Chamfer(vertices, l)`: `Result<Shape>` | `Result<ModelShape>`: a `Sketch`, or a `Curve` for a wire |
| `FullRound(edge, invert, n)` | `FullRound(edge, parent, invert, n)` |
| `OneD.ConnectedEdges(edge, parent = null)`, `ConnectedFaces` | `parent` is required |
| `TopologyUtils.TopoDistanceTo(shapes)` | `TopoDistanceTo(parent, shapes)`, generic in the shape type; a shape not in the parent is an `ArgumentException` |

## Results that were a class

| Before | After |
|---|---|
| `Operations.BoundingBox`, `Mirror`, `Scale`, `Split`, `Project`, `Offset`, `Sweep` of any shapes: `Compound` (a `Part`, `Sketch` or `Curve` at run time) | `ModelShape`, the same class at run time, and an `Assembly` leaf for a mix of dimensions |
| `Importers.ImportStep`, `ImportGltf`: `Result<Shape>` | `Result<ModelShape>`: an `Assembly` for an assembly, otherwise `Part`, `Sketch`, `Curve` or an `Assembly` leaf holding the shape, with its label and colour |
| `Importers.ImportBrep`, `ImportStl` | `Result<Shape>`, `Result<Face>`, of the kernel's |
| `Exporters3D.Export…(Shape toExport, …)` | `IShape toExport`: anything; an object's label, colour and children are written |
| `solid - solid`, `solid + solid`: a `Part` of pieces where it split | A kernel `Compound` of the pieces. Check `shape.Type` |
| `x is Face f`, `case Face f` on a `Shape` | `shape.TryAs(out Face f)` (a null shape is any type), or `shape.Type` |
| `(Solid)shape`, `(Wire)edgeOrWire` | The kernel's explicit conversions, the same. A `ModelShape` converts explicitly too |
| `offset2D`, `Close()`: `Shape1D` | `IShape1D`. `.AsShape()` for a `Shape` |
| `Solid.SweepMulti(IEnumerable<Shape2D>, …)` | `IEnumerable<Face>` |

## Empty shapes

- The zero of a type is `default(T)`; `shape.IsEmpty` is also true of an empty compound.
- A zero has no class at run time once it is a `Shape`: `default(Solid) - x` is a default `Shape`. A `Part`, `Sketch` or `Curve`
  keeps its class.
- `IsSame` and `IsEqual` of two null shapes are the kernel's: true. `shape.SameAs(other)` is the layer's: two empty shapes are the same
  nothing.
- `shape.ToString()` on a struct is the kernel's (`NULL`, `VERTEX FORWARD`). `shape.Describe()` is the layer's text: `Vertex()`,
  `Vertex(1, 2, 3)`, `Solid at Location(…)`.

## Names that met the kernel's

A struct's own member wins over an extension of the same name, so these moved:

| Before | After | Why |
|---|---|---|
| `edge.IsClosed`, `wire.IsClosed` | `edge.CheckClosed()`, the kernel's. On a `Curve` or an `IShape1D`, `IsClosed` is as it was | The struct's `IsClosed` is the kernel's stored flag, which a full circle's edge doesn't set |
| `edge.IsInfinite` | `edge.IsUnbounded` | The struct's is the kernel's stored flag |
| `shape.Orientation` (a vector of angles) | `shape.OrientationAngles` | The struct's is the kernel's `Orientation` enum |
| `shape.BoundingBox(tolerance, optimal)`: `BoundBox` | `shape.BoundBox(tolerance, optimal)` | The kernel's `shape.BoundingBox()` gives its own `BoundingBox`, with another default |
| `edge.Reversed()` | The same, now the kernel's, which keeps the type (kernel change). An empty edge no longer throws. `edge.Reversed(reconstruct: true)` is as it was | |
| `face.Volume`… in `Kerf.Measurement`'s company | With `using Kerf.Measurement;` in the same file, `solid.Volume`, `.Area` and `edge.Length` are ambiguous with the kernel's methods (`CS9339`). Write `solid.Volume()` for the kernel's, or leave that `using` out | A property and a method of one name can't be chosen between |
| `new Cylinder(…)`, `new Sphere(…)`, `new Cone(…)`, `new Torus(…)` beside `using Kerf.Geometry;` | Ambiguous with the kernel's surface values: qualify one | Different ideas under one name, as before |

## Colour

| Before | After |
|---|---|
| `new Color("red", 0.5)`, `new Color("#ff0000")` | `Color.Parse("red", 0.5)`, `Color.Parse("#ff0000")` |
| `new Color(0xFF0000, 0x80)` | `Color.FromCode(0xFF0000, 0x80)` |
| `new Color(0.1)`, `new Color(0.1, 0.2)` (missing components are 1) | `Color.Rgb(0.1)`, `Color.Rgb(0.1, 0.2)` |
| `new Color(r, g, b, a)` | The same: the kernel's constructor. Doubles only, each from 0 to 1 |
| `new Color()` (white) | `Color.White`: `default(Color)` isn't white |
| `new Color(kerfColor)`, `color.ToKerfColor()` | Nothing to do |
| `color.ToString()` | `color.Describe()` gives the old text; `ToString()` is the kernel's |
| `color.R`, `.G`, `.B`, `.A` rounded to seven places | Unrounded |
| `Color.CategoricalSet(…)` | The same, a static extension |

## Bento's couplings

- **`A1.Volume`-style property reads by reflection** no longer find members on a shape: the conveniences are static methods of the
  `*Extensions` classes (`get_Volume<TSelf>(TSelf)`), and a `Solid` in a cell is a struct. Bento's table of properties calls them
  directly, or holds a `ModelShape`.
- **A cell's value.** A cell holding a shape holds a boxed struct, or a `ModelShape`; `IShape` covers both, and `.AsShape()` gives the
  kernel's shape.
- **The enum text** of `ShapeType` parses the kernel's enum: the same names, plus `Shape`.
- **`Location *`** is unchanged, on objects and structs.
- **The differential log.** `BUILD123D_SHAPE_LOG` and `ShapeLog` went: every operation no longer passes through one constructor.
