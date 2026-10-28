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
| `list > Axis.Z`, `list | GeomType.LINE`, `list >> Axis.Z` | The same operators |
| `compound.get_type(Solid)` | `ElementsOfType<Solid>()` |
| `Compound(children=[…])` | `Compound.Assembly([…])`; children hold no parent link |
| `Shape.cast(topods)` | `Shape.Cast(topods)` |
| `SkipClean()` context | `using (new SkipClean()) { … }` |
| `warnings.warn` | `Warnings.Raised` event |
| `topo_explore_connected_edges`, `edges_to_wires`, … | `OneD.ConnectedEdges`, `OneD.EdgesToWires`, …; `Vertex.CommonVertex` |
| `topo_distance_to(shapes)` | `TopologyUtils.TopoDistanceTo(shapes)`, a `Func<Shape, double>` |
| `show_topology("Face")` | `ShowTopology(ShapeType.Face)` |
| `filter_by_value(Edge.length, 3)` | `FilterByValue(e => e.Length, 3)` |
| `ShapeHistory.merge` (in place) | `Merge` returns a new record |

## Not yet ported

- Text (`Compound.make_text`, fonts), constrained arcs and lines, and `Wire.fillet_2d` / `Face.fillet_2d`
  (which use the constrained-arc solvers): phase 6.
- `project_to_viewport` (hidden-line removal): with the SVG and DXF exporters, phase 5.
- Materials and mass (`material`, `mass()`), joints, `uv_face`, `uv_frame`, `wrap`, `unfold`,
  `make_gordon_surface`, `tessellate_with_uvs`: phase 6.
- `Face.length` and `width` of a cylindrical face, which need `uv_face`.

## Deliberate differences

- `Color(0)` (black as a hex code) is black. build123d treats the 0 as "no colour code" and
  returns white.
- Python's FutureWarning for `Plane.shift_origin((x, y))` isn't ported. That warning covers a
  change in meaning, and the port has only ever had the new meaning: a 2-tuple is a local coordinate.
- Run-time argument-type checks (for example `Rotation("text")` raising TypeError) have no
  counterpart, because the compiler rules those calls out.
