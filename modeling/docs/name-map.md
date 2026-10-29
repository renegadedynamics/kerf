# Python → .NET name map

The port uses .NET naming: PascalCase members, properties for values, and methods for computations.
The tables below list only names that differ in more than case. A Python member that changes the
object has an immutable counterpart that returns a new value (see [design.md](design.md)).

## Enums

Members are PascalCase: `Align.MIN` → `Align.Min`, `Mode.SUBTRACT` → `Mode.Subtract`,
`Unit.MM` → `Unit.Millimeter` (`Unit.Millimeter.Symbol()` gives `"mm"`). `Intrinsic` and
`Extrinsic` keep their axis names (`Intrinsic.XYZ`). A parameter that accepts either one is a
`RotationOrder`, which both convert to implicitly.

| Python | .NET |
|---|---|
| `Align2D`, `Align3D` (an `Align` or a tuple of them) | `Alignment`, which converts implicitly from an `Align` or a tuple |
| `to_align_offset(min, max, align, center)` | `Alignment.Offset(min, max, center)` |

## Geometry

| Python | .NET |
|---|---|
| `TOLERANCE`, `TOL` | `Constants.Tolerance`, `Constants.LooseTolerance` |
| `VectorLike` tuples | Implicit conversion from `(x, y)` and `(x, y, z)` |
| `Vector(X=…)`, `vector.X = …` | Constructor arguments; vectors are immutable |
| `vector.to_pnt()`, `.to_dir()`, `.wrapped` | `ToGpPnt()`, `ToGpDir()`, `ToGpVec()`, `ToGpXyz()` (each returns a new OCCT object) |
| `abs(vector)` | `vector.Length` |
| `round(vector, n)` | `vector.Round(n)` |
| `format(v, ".3f")` | `v.ToString(".3f")`. Specs follow Python's; `ToString()` gives Python's `str()` |
| `Axis(origin, end_point=…)` | `Axis.FromPoints(origin, endPoint)` |
| `Axis(edge)` | `new Axis(edge)` |
| `axis & shape`, `plane.intersect(shape)`, … | `axis & shape`, `plane.Intersect(shape)`: a `ShapeList<Shape>` |
| `repr(vector)`, `repr(plane)`, `repr(location)` | `Repr()` (`ToString()` gives Python's `str()`) |
| `axis.position = …`, `axis.direction = …` | `axis.WithPosition(…)`, `axis.WithDirection(…)` |
| `axis & other` (returns a union of types) | `IGeometry?`; pattern-match with `is Vector v` |
| `location.position = …`, `.orientation = …` | `WithPosition(…)`, `WithOrientation(…)` |
| `location ** n` | `location.Pow(n)` |
| `tuple(location)` | `var (position, orientation) = location;` |
| `location.wrapped` | `ToTopLoc()` (shares the location's OCCT identity), `ToGpTrsf()` |
| `Rot` | `Rotation` (or `using Rot = Build123d.Rotation;`) |
| `Plane(origin, x_dir, y_dir=…)` | `Plane.FromXAndYDirections(origin, xDir, yDir)` |
| `Plane([p0, p1, p2])` | `Plane.ThroughPoints(p0, p1, p2)` |
| `Plane.front`, `.isometric`, … | `Plane.Front`, `Plane.Isometric`, … (cached; planes are immutable) |
| `plane.origin = …`, `.x_dir = …`, `.y_dir = …` | `WithOrigin(…)`, `WithXDir(…)`, `WithYDir(…)` |
| `plane.move(loc)` | `plane.Moved(loc)` |
| `plane.wrapped`, `.to_gp_ax3()`, `.to_gp_ax2()` | `ToGpPln()`, `ToGpAx3()`, `ToGpAx2()` (copies) |
| `Matrix([[…], …])` | `new Matrix(new double[,] { … })` |
| `matrix.rotate(axis, angle)` | `matrix.Rotated(axis, angle)` |
| `matrix.wrapped` | `ToGpGTrsf()` (a copy) |
| `BoundBox(bnd_box)` | `BoundBox.FromBndBox(box)`; `BoundBox.Empty` for the box of nothing |
| `BoundBox.find_outside_box_2d`, `.is_inside` (deprecated) | Not ported; use `Contains`, `Within` |
| `Color(color_like=…)`, tuples | Constructor overloads |
| `tuple(color)` | `color.R`, `.G`, `.B`, `.A`, or deconstruction |
| `color.wrapped` | `ToQuantityColorRgba()` |
| `Color.categorical_set(n, starting_hue=0xff0000)` | `Color.CategoricalSet(n, new Color(0xff0000))` |
| `GeomEncoder` (JSON) | Not yet ported |
| `Vector(vertex)` | `new Vector(vertex)` or `vertex.ToVector()` |
| `Plane(face)`, `BoundBox(shape)` | `new Plane(face)`, `shape.BoundingBox()` / `BoundBox.FromShape(shape)` |
| `plane.to_local_coords(shape)` | `plane.ToLocalCoords(shape)` (keeps the shape's class) |

## Topology

| Python | .NET |
|---|---|
| `Mixin1D`, `Mixin2D`, `Mixin3D` | `Shape1D`, `Shape2D`, `Shape3D` (abstract base classes) |
| `shape.wrapped` | `Wrapped` (a new handle sharing the geometry; the library itself never modifies its OCCT shape) |
| `shape.is_null`, `bool(shape)` | `IsEmpty` |
| `vertices()`, `edges()`, … `solids()` | `Vertices()`, `Edges()`, … `Solids()`, each taking `Select` |
| `vertex()`, `edge()`, `wire()`, `face()`, `shell()`, `solid()`, `compound()` | `SingleVertex()`, `SingleEdge()`, … `SingleCompound()` (a method can't share its type's name) |
| `locate(loc)`, `move(loc)`, `shape.location = …` | `Located(loc)`, `Moved(loc)`, `WithLocation(loc)`: copies |
| `shape.label = …`, `.color = …` | `WithLabel(…)`, `WithColor(…)` |
| `relocate`, `clean()`, `fix()`, `mirror()`, `rotate()`, `scale()`, `translate()` | `Clean()`, `Fix()`, `Mirror()`, `Rotate()`, `Scale()`, `Translate()`: copies of the caller's class |
| `a + b`, `a - b`, `a & b` | The same operators, or `Add`, `Subtract`, `CommonWith`; `Fuse`, `Cut`, `Intersect` as in Python |
| `loc * shape`, `[locs] * shape` | The same operators, keeping the shape's class |
| `shape.split(tool, keep=Keep.TOP)` | `Split(tool, Keep.Top)`, returning a `ShapeList` (empty for nothing); `Keep.BOTH` is `SplitBoth(tool)` |
| `edge @ u` | `edge.PositionAt(u)` (C# has no `@` operator); `%` and `^` are kept |
| `edge.trim(a, b)` with floats or points | `Trim(a, b)`: each a `CurvePosition`, from a `double`, a `Vector` or a `Vertex` |
| `ShapeList.first`, `.last` | `First`, `Last` |
| `filter_by(lambda)`, `sort_by(Edge.length)`, `group_by(...)` | `FilterBy(Func)`, `SortBy(e => e.Length)`, `GroupBy(...)`; `GroupByConvexity()` for the enum |
| `filter_by(Edge.is_closed)`, `sort_by(Edge.length)`, `group_by(Face.area)` (a `property`) | also by name: `FilterBy("IsClosed")`, `SortBy("Length")`, `GroupBy("Area")`; build123d's snake_case names work too |
| `list > Axis.Z`, `list | GeomType.LINE`, `list >> Axis.Z` | The same operators |
| `compound.get_type(Solid)` | `ElementsOfType<Solid>()` |
| `Compound(children=[…])` | `Compound.Assembly([…])`; children hold no parent link |
| `Shape.cast(topods)` | `Shape.Cast(topods)` |
| `SkipClean()` context | `using (new SkipClean()) { … }` |
| `warnings.warn` | `using var w = Warnings.Capture();` then `w.Messages`: per operation, not global |
| `topo_explore_connected_edges`, `edges_to_wires`, … | `OneD.ConnectedEdges`, `OneD.EdgesToWires`, …; `Vertex.CommonVertex` |
| `topo_distance_to(shapes)` | `TopologyUtils.TopoDistanceTo(shapes)`, a `Func<Shape, double>` |
| `show_topology("Face")` | `ShowTopology(ShapeType.Face)` |
| `filter_by_value(Edge.length, 3)` | `FilterByValue(e => e.Length, 3)`, `FilterByValue("Length", 3)` or `FilterByValue(SortBy.Length, 3)`; an enum value may be its name (`FilterByValue("GeomType", "Plane")`) |
| `ShapeHistory.merge` (in place) | `Merge` returns a new record |
| `shape.mesh(tol)` (in place) | `Meshed(tol)` returns a meshed copy; `Tessellate` meshes a copy too |
| (ocp-tessellate, for viewers) | `Triangulate(…)` returns a `ShapeMesh` (faces, edges and vertices in the shape's order); `TriangulateTree()` a `MeshNode` tree for assemblies |
| `make_constrained_arcs(a, b, radius=r)`, `(a, b, c)`, `(a, center=p)`, `(a, radius=r, center_on=c)` | `Edge.MakeConstrainedArcs(…)` overloads, each target a `Tangent` (from an `Edge`, `Axis`, `Vector`, `Vertex` or `(edge, Tangency)`) |
| `make_constrained_arcs(a, b, center_on=c)` | `Edge.MakeConstrainedArcsCenteredOn(a, b, c)`: three targets would be ambiguous with the three-tangent form |
| `make_constrained_lines(a, b)`, `(a, axis, angle=…)`, `(a, axis, direction=…)` | `Edge.MakeConstrainedLines(…)` overloads |

## Objects

| Python | .NET |
|---|---|
| `Box(1, 2, 3, align=…, rotation=…)` | `new Box(1, 2, 3, rotation: …, align: …)`; `align: null` is the object's default, `Align.None` no alignment |
| `rotation=(x, y, z)` | A `Rotation`, also from a `(x, y, z)` tuple |
| `box.box_height`, `rect.rectangle_height` | `BoxHeight`, `RectangleHeight` (as Python names them) |
| `Line(p1, p2)`, `Polyline(*pts, close=…)` | `new Line(p1, p2)`, `new Polyline(pts, close: …)` or `new Polyline(p1, p2, …)` |
| `CenterArc(c, r, start, arc_size)` with a float or a limit (`Shape`, `Axis`, …) | Two overloads: `double arcSize`, or a `CurveLimit` (from a shape, axis, location, plane or point); the same for `JernArc`, the conic arcs and `PolarLine`'s length |
| `ConstrainedArcs(…, selector=lambda arcs: …)` | `new ConstrainedArcs(…, selector: arcs => …)`, the selector returning edges; `center_on` with two targets is `ConstrainedArcs.CenteredOn(…)` |
| `GridLocations(…) * Circle(1)` | The same operator over the `LocationList` |

## Operations

| Python | .NET |
|---|---|
| `extrude(face, 5)`, `fillet(edges, 1)`, … | `Operations.Extrude(face, 5)`, `Operations.Fillet(edges, 1)`, …; with `using static Build123d.Operations;` just `Extrude(face, 5)` |
| A union return (`Part \| Sketch \| Curve \| Compound`) | `Compound`, which is a `Part`, `Sketch` or `Curve` at run time by Python's dimension rule |
| `chamfer`/`fillet` of edges, of vertices | `Part`, and `Shape` (a `Sketch`, `Wire` or `Shell`) |
| `project(points, workplane)` | `ProjectPoints(points, workplane)`, returning the points |
| `full_round` (scipy's Voronoi) | `FullRound`, over a small Delaunay triangulation of its own |
| `insert`, `add`, `make_brake_formed`, sheet-metal `thicken` | Not yet ported (builder mode; sheet metal in phase 7) |

## Not yet ported

- Text (`Compound.make_text`, fonts and the `Text` object): phase 7.
- Builder mode (`BuildLine`, `BuildSketch`, `BuildPart` and the objects' and operations' builder behaviour): phase 5.

## Deliberate differences

- `Color(0)` (black as a hex code) is black. build123d treats the 0 as "no colour code" and
  returns white.
- Python's FutureWarning for `Plane.shift_origin((x, y))` isn't ported. That warning covers a
  change in meaning, and the port has only ever had the new meaning: a 2-tuple is a local coordinate.
- Run-time argument-type checks (for example `Rotation("text")` raising TypeError) have no
  counterpart, because the compiler rules those calls out.
- Exceptions: `ValueError` becomes `ArgumentException`, except where the shape being operated on is empty,
  which throws `InvalidOperationException` (the object's state, not an argument, is at fault).
  `RuntimeError` becomes `InvalidOperationException`. The messages are Python's.
- A `Vertex` given as a constrained-arc target or centre is its point. build123d adds the vertex's
  position to its point, which counts the position twice for a moved vertex.
