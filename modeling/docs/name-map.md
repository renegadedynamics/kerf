# Python → .NET name map

The port uses .NET naming: PascalCase members, properties for values, and methods for computations.
The tables below list only names that differ in more than case. A Python member that changes the
object has an immutable counterpart that returns a new value (see [design.md](design.md)). They map Python build123d
to Kerf.Modeling's names after unit 10M2, which redesigned the API for .NET (design.md, "The .NET API"); 10M2's own
changes are listed for Bento in [10m2-bento.md](10m2-bento.md).

## Enums

Members are PascalCase: `Align.MIN` → `Align.Min`, `Unit.MM` → `Unit.Millimeter` (`Unit.Millimeter.Symbol()` gives `"mm"`). `Intrinsic` and
`Extrinsic` keep their axis names (`Intrinsic.XYZ`). A parameter that accepts either one is a
`RotationOrder`, which both convert to implicitly.

| Python | .NET |
|---|---|
| `Align2D`, `Align3D` (an `Align` or a tuple of them) | `Alignment`: an `Align` converts implicitly; `new Alignment(x, y)` or `new Alignment(x, y, z)` |
| `Mode.ADD`, `SUBTRACT`, … (`mode=` on objects and operations) | Not ported: algebra mode only, combined with operators |
| `SortBy.LENGTH`, … | `ShapeKey.Length`, … |
| `to_align_offset(min, max, align, center)` | `Alignment.Offset(min, max, center)` |

## Geometry

| Python | .NET |
|---|---|
| `TOLERANCE`, `TOL` | `Constants.Tolerance`, `Constants.LooseTolerance` |
| `VectorLike` tuples, `(x, y, z)` | `new Point3D(x, y, z)` for a position, `new Vector3D(x, y, z)` for an offset (no conversion from tuples) |
| `Vector`, `Vector(X=…)`, `vector.X = …` | The kernel's `Point3D`, `Vector3D`, `Direction3D`, with `PointExtensions` (unit 10Q2); constructor arguments, as they are immutable |
| `vector == other` (a tolerance) | `a.IsNear(b)`, `PointComparer.Near`; the kernel's `==` is exact |
| `vector.to_pnt()`, `.to_dir()`, `.wrapped` | `ToPoint()`, `ToDirection()`, `ToVector()` (they are the kernel's values) |
| `abs(vector)` | `vector.Magnitude` |
| `round(vector, n)` | `vector.Round(n)` |
| `format(v, ".3f")` | `v.ToString("F3")`: .NET's numeric formats |
| `Axis(origin, end_point=…)` | `Axis.FromPoints(origin, endPoint)` |
| `Axis(edge)` | `new Axis(edge)` |
| `axis & shape`, `plane.intersect(shape)`, … | `axis.Intersect(shape)`, `plane.Intersect(shape)`: a `ShapeList<Shape>` |
| `repr(vector)`, `repr(plane)`, `repr(location)` | `ToString()` |
| `axis.position = …`, `axis.direction = …` | `axis.WithPosition(…)`, `axis.WithDirection(…)` |
| `axis & other` (returns a union of types) | `axis.Intersect(other)`: an `IGeometry?`; pattern-match with `is Vector v` |
| `location.position = …`, `.orientation = …` | `WithPosition(…)`, `WithOrientation(…)` |
| `location ** n` | `location.Pow(n)` |
| `tuple(location)` | `var (position, orientation) = location;` |
| `location.wrapped` | `ToPlacement()` (the kernel's placement), `ToTransform3D()` |
| `Rot` | `Rotation` (or `using Rot = Kerf.Modeling.Rotation;`) |
| `RotationLike` tuples, `rotation=(x, y, z)` | `new Rotation(x, y, z)` |
| `Plane(origin, x_dir, y_dir=…)` | `Plane.FromXAndYDirections(origin, xDir, yDir)` |
| `Plane([p0, p1, p2])` | `Plane.ThroughPoints(p0, p1, p2)` |
| `Plane.front`, `.isometric`, … | `Plane.Front`, `Plane.Isometric`, … (cached; planes are immutable) |
| `plane.origin = …`, `.x_dir = …`, `.y_dir = …` | `WithOrigin(…)`, `WithXDir(…)`, `WithYDir(…)` |
| `plane.move(loc)` | `plane.Moved(loc)` |
| `plane.wrapped`, `.to_gp_ax3()`, `.to_gp_ax2()` | `ToPlane3D()`, `ToCoordinateSystem()`, `ToFrame3D()` |
| `plane.shift_origin((x, y))` (local coordinates) | `ShiftOriginBy(x, y)`; `ShiftOrigin(point)` takes a global point |
| `Matrix([[…], …])` | `GeneralTransform3D.FromValues(new double[,] { … })`, with `TransformExtensions` (unit 10Q2) |
| `matrix[r, c]`, `a * b` | `matrix.At(r, c)`, from zero; `a.Compose(b)` |
| `matrix.rotate(axis, angle)` | `matrix.Rotated(axis, angle)` |
| `matrix.wrapped` | Nothing: it is the kernel's |
| `BoundBox`, `BoundBox(bnd_box)` | The kernel's `BoundingBox`, with `BoundingBoxExtensions` (unit 10Q2); `BoundingBox.Empty` for the box of nothing |
| `box.add(x, tol)`, `box.intersects(o)` | `box.Including(p)`, `box.Union(o)`; `box.IntersectsNear(o)` |
| `BoundBox.find_outside_box_2d`, `.is_inside` (deprecated) | Not ported; use `Contains`, `Within` |
| `Color(color_like=…)`, tuples | Constructor overloads |
| `tuple(color)` | `color.R`, `.G`, `.B`, `.A`, or deconstruction |
| `color.wrapped` | `ToKerfColor()` |
| `Color.categorical_set(n, starting_hue=0xff0000)` | `Color.CategoricalSet(n, new Color(0xff0000))` |
| `GeomEncoder` (JSON) | Not yet ported |
| `Vector(vertex)` | `vertex.Point` |
| `Plane(face)`, `BoundBox(shape)` | `new Plane(face)`, `shape.Bounds()` |
| `plane.to_local_coords(shape)` | `plane.ToLocalCoords(shape)` (keeps the shape's class) |

## Topology

| Python | .NET |
|---|---|
| `Shape`, `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Solid`, `Compound` | The kernel's `Kerf.Topology` structs, with extension members; constructors are `Vertex.Make(…)`, `Wire.Make(…)`… (unit 10Q) |
| `Mixin1D`, `Mixin2D`, `Mixin3D` | The kernel's `IShape1D`, `IShape2D`, `IShape3D` |
| `Part`, `Sketch`, `Curve` | `Part`, `Sketch`, `Curve`, over `ModelShape` |
| `Line`, `Polyline`… (an `Edge`, a `Wire`) | `Line`, `Polyline`… (a `Curve` holding one) |
| `shape.wrapped` | `model.Shape`; a kernel shape is itself |
| `shape.label`, `.color` on a topological shape | `shape.WithLabel(…)`, `.WithColor(…)` give the object of its dimension, which has them |
| `shape.topo_parent` | `list.Origin`; what needs a parent takes it (`box.ConvexityOf(edge)`, `Fillet(box, edge, r)`) |
| `edge.is_interior`, `shape.convexity` | `parent.ConvexityOf(edge) == Convexity.Concave`; `face.Convexity` |
| `shape.is_null`, `bool(shape)` | `IsEmpty` |
| `vertices()`, `edges()`, … `solids()` | `Vertices()`, `Edges()`, … `Solids()`, a selection, on an object (taking `Select`) or a struct alike. The kernel\'s array, with degenerate edges, is `AllEdges()` and so on |
| `vertex()`, `edge()`, `wire()`, `face()`, `shell()`, `solid()`, `compound()` | `SingleVertex()`, `SingleEdge()`, … `SingleCompound()` (a method can't share its type's name) |
| `locate(loc)`, `move(loc)`, `shape.location = …` | `Located(loc)`, `Moved(loc)`, `WithLocation(loc)`: copies |
| `shape.label = …`, `.color = …` | `WithLabel(…)`, `WithColor(…)` |
| `relocate`, `clean()`, `fix()`, `mirror()`, `rotate()`, `scale()`, `translate()` | `Clean()`, `Fix()`, `Mirror()`, `Rotate()`, `Scale()`, `Translate()`: copies of the caller's class |
| `a + b`, `a - b`, `a & b` on a Part, Sketch or Curve | The same operators, typed (`Part + Shape` is a `Part`), or `Add`, `Subtract`, `Common`; `TryAdd`, `TrySubtract`, `TryCommon` give a `Result<T>` |
| `a + b`, `a - b`, `a & b` on a Solid, Face, Edge… | The same operators, giving the kernel's result as a `Shape` (one solid or a compound, a wire or a compound); `Fuse`, `Cut`, `Intersect` as in Python |
| `shape + None` | Not allowed: a null operand throws `ArgumentNullException`; add nothing with an empty list |
| `loc * shape`, `[locs] * shape` | The same operators, keeping the shape's class |
| `shape.split(tool, keep=Keep.TOP)` | `Split(tool, Keep.Top)`, returning a `ShapeList` (empty for nothing); `Keep.BOTH` is `SplitBoth(tool)` |
| `edge @ u`, `edge % u`, `edge ^ u` | `edge.PositionAt(u)`, `TangentAt(u)`, `LocationAt(u)` (on curves too) |
| `edge.trim(a, b)` with floats or points | `Trim(a, b)`: each a `CurvePosition`, from a `double`, a `Vector` or a `Vertex` |
| `ShapeList.first`, `.last`, `shapes[-1]` | LINQ's `First()`, `Last()`; `shapes[^1]` |
| `filter_by(lambda)`, `sort_by(lambda)` | LINQ's `Where(e => …)`, `OrderBy(e => …)` (then `ToShapeList()` for a list) |
| `filter_by(GeomType.LINE)`, `filter_by(Axis.Z)`, `filter_by(Plane.XY)`, `reverse=True` | `Where(GeomType.Line)`, `ParallelTo(Axis.Z)`, `ParallelTo(Plane.XY)`; `WhereNot(…)`, `NotParallelTo(…)` |
| `filter_by(Edge.is_closed)` (a `property`, or its name) | `Where(e => e.IsClosed)`; names aren't taken (unit 10M2) |
| `filter_by_position(Axis.Z, min, max, inclusive=(a, b))` | `Between(Axis.Z, min, max, includeMinimum: a, includeMaximum: b)` |
| `sort_by(Axis.Z)`, `sort_by(SortBy.AREA)`, `reverse=True` | `OrderBy(Axis.Z)`, `OrderBy(ShapeKey.Area)`; `OrderByDescending(…)` |
| `sort_by_distance(other)` | `OrderByDistanceTo(other)`, `OrderByDescendingDistanceTo(other)` |
| `group_by(Axis.Z)`, `group_by(SortBy.LENGTH)`, `group_by(lambda)` | `GroupBy(Axis.Z)`, `GroupBy(ShapeKey.Length)`, `GroupByValue(e => …)`: a `ShapeGroups<T, TKey>`; `GroupByConvexity()` for the enum |
| `list > Axis.Z`, `list < Axis.Z`, `list | GeomType.LINE`, `list >> Axis.Z`, `list << Axis.Z` | `OrderBy(Axis.Z)`, `OrderByDescending(Axis.Z)`, `Where(GeomType.Line)`, `GroupBy(Axis.Z).Last()`, `GroupBy(Axis.Z).First()` |
| `list + list`, `list - list`, `list & list` | `AddRange`, `Except`, `Intersect` (a `ShapeList`) |
| `compound.get_type(Solid)` | `ElementsOfType<Solid>()` |
| `Compound(children=[…])` | `new Assembly([…])`; children hold no parent link |
| `Shape.cast(topods)` | Nothing: it is the kernel's shape; `ModelShape.For(shape)` for an object |
| `SkipClean()` context | `using (new SkipClean()) { … }` |
| `warnings.warn` | `using var w = Warnings.Capture();` then `w.Messages`: per operation, not global |
| `topo_explore_connected_edges`, `edges_to_wires`, … | `edge.ConnectedEdges`, `edge.ConnectedFaces`, `Wire.Combine`, …; `Vertex.CommonVertex` |
| `topo_distance_to(shapes)` | Internal |
| `show_topology("Face")` | `ShowTopology(ShapeType.Face)` |
| `filter_by_value(Edge.length, 3)` | `WithValue(e => e.Length, 3)` or `WithValue(ShapeKey.Length, 3)`; `WithoutValue(…)` for the rest |
| `ShapeHistory.merge` (in place), on `ModelHistory` | `Merge` returns a new record |
| `shape.mesh(tol)` (in place) | Not ported: a shape holds no mesh; `Triangulate` gives one (unit 10M1) |
| (ocp-tessellate, for viewers) | `Triangulate(…)` returns a `ShapeMesh` (faces, edges and vertices in the shape's order); `TriangulateTree()` a `MeshNode` tree for assemblies |
| `make_constrained_arcs(a, b, radius=r)`, `(a, b, c)`, `(a, center=p)`, `(a, radius=r, center_on=c)` | `Edge.MakeConstrainedArcs(…)` overloads, each target a `Tangent` (from an `Edge`, `Axis`, `Vector` or `Vertex`, or `new Tangent(edge, Tangency.Enclosing)`) |
| `make_constrained_arcs(a, b, center_on=c)` | `Edge.MakeConstrainedArcsCenteredOn(a, b, c)`: three targets would be ambiguous with the three-tangent form |
| `make_constrained_lines(a, b)`, `(a, axis, angle=…)`, `(a, axis, direction=…)` | `Edge.MakeConstrainedLines(…)` overloads |

## Objects

| Python | .NET |
|---|---|
| `Box(1, 2, 3, align=…, rotation=…)` | `new Box(1, 2, 3, rotation: …, align: …)`; `align: null` is the object's default, `Align.None` no alignment |
| `rotation=(x, y, z)` | `rotation: new Rotation(x, y, z)` |
| `mode=Mode.PRIVATE` | Not ported |
| `box.box_height`, `rect.rectangle_height` | `Height`, `Height` (as Python names them) |
| `Line(p1, p2)`, `Polyline(*pts, close=…)` | `new Line(p1, p2)`, `new Polyline(pts, close: …)` or `new Polyline(p1, p2, …)` |
| `CenterArc(c, r, start, arc_size)` with a float or a limit (`Shape`, `Axis`, …) | Two overloads: `double arcSize`, or a `CurveLimit` (from a shape, axis, location, plane or point); the same for `JernArc`, the conic arcs and `PolarLine`'s length |
| `ConstrainedArcs(…, selector=lambda arcs: …)` | `new ConstrainedArcs(…, selector: arcs => …)`, the selector returning edges; `center_on` with two targets is `ConstrainedArcs.CenteredOn(…)` |
| `GridLocations(…) * Circle(1)` | The same operator over the `LocationList` |

## Operations

| Python | .NET |
|---|---|
| `extrude(face, 5)`, `fillet(edges, 1)`, … | `Operations.Extrude(face, 5)`, `Operations.Fillet(edges, 1)`, …; with `using static Kerf.Modeling.Operations;` just `Extrude(face, 5)` |
| `extrude(…, dir=, both=, taper=, clean=)` | `Extrude(face, amount, new ExtrudeOptions { Direction = …, Both = …, Taper = …, Clean = … })` |
| `extrude(…, until=Until.NEXT, target=t)` | `Extrude(face, Until.Next, t, options)` |
| `sweep(…, multisection=, is_frenet=, transition=, normal=, binormal=, clean=)` | `Sweep(sections, path, new SweepOptions { … })` |
| `offset(…, openings=, kind=, side=, closed=, min_edge_length=)` | `Offset(obj, amount, new OffsetOptions { Openings = [..], Kind = …, … })` |
| `thicken(…, normal_override=, both=, clean=)` | `Thicken(faces, amount, new ThickenOptions { … })` |
| A failure (`ValueError`, `RuntimeError` from fillet, chamfer, offset, loft, draft) | A failed `Result<T>` with a code (`ModelingDiagnostics`) from the operation (`Fillet`, `Extrude`, …; unit 10M3) |
| `DraftAngleError` | A failed `Result<Part>`, code `modeling.draft-failed`, the face as the diagnostic's shape |
| A union return (`Part \| Sketch \| Curve \| Compound`) | Typed for a Part, Sketch or Curve given (`Mirror(part)` is a `Part`); `Compound` for a mix, a `Part`, `Sketch` or `Curve` at run time by Python's dimension rule |
| `chamfer`/`fillet` of edges, of vertices | `Part`, and `ModelShape` (a `Sketch` or `Curve`) |
| `project(points, workplane)` | `ProjectPoints(points, workplane)`, returning the points |
| `full_round` (scipy's Voronoi) | `FullRound`, over a small Delaunay triangulation of its own |
| `insert`, `add`, `make_brake_formed`, sheet-metal `thicken` | Not yet ported (builder mode; sheet metal in phase 7) |

## Import and export

| Python | .NET |
|---|---|
| `import_brep(path)` | `Importers.ImportBrep(stream or path, limits, token)` |
| `import_step(path)` | `Importers.ImportStep(stream or path, limits, token)`: the file's tree, assemblies as compounds with `AssemblyChildren`, labels and colours; natively one shape, or a compound of the file's shapes |
| `import_stl(path, model_unit)` | `Importers.ImportStl(stream or path, modelUnit, limits, token)`: a `Face` holding the triangles |
| (none) | `Importers.ImportGltf(stream or path, modelUnit, limits, token)`: parts and assemblies with names and colours |
| `export_brep(shape, path)` | `Exporters3D.ExportBrep(shape, stream or path)` |
| `export_step(shape, path, unit, write_pcurves, precision_mode, timestamp=…)` | `Exporters3D.ExportStep(shape, stream or path, StepExportOptions, token)`: a `string` time stamp (`datetime.isoformat()`'s); assemblies, product names and colours on the the kernel backend, natively the shape's compound alone |
| `export_stl(shape, path, tolerance, angular_tolerance, ascii_format)` | `Exporters3D.ExportStl(shape, stream or path, StlExportOptions, token)` |
| `export_gltf(shape, path, unit, binary, linear_deflection, angular_deflection)` | `Exporters3D.ExportGltf(shape, stream or path, GltfExportOptions, token)`; to a stream, a `.gltf` embeds its buffer |
| `Mesher(unit).add_shape(shape, linear_deflection, angular_deflection)` then `write("x.3mf")` | `Exporters3D.Export3mf(shape, stream or path, ThreeMfExportOptions, token)` |
| `ValueError` from a reader | `ArgumentException`; past an `ImportLimits` limit, `InvalidDataException` |
| `FileNotFoundError` (no file, or `export_stl`'s missing folder) | `FileNotFoundException`, `DirectoryNotFoundException` (every exporter) |

## Not yet ported

- Text (`Compound.make_text`, fonts and the `Text` object): phase 7.
- Builder mode (`BuildLine`, `BuildSketch`, `BuildPart` and the objects' and operations' builder behaviour): phase 5.
- `export_obj`, `export_to_pcbway`, `Mesher` as a class (its metadata, `read`, STL through lib3mf), `import_svg` and
  the 2D exporters (SVG, DXF); STEP's names, colours and assemblies natively (XCAF isn't bound); the glTF
  export's UVs and PBR materials.

## Deliberate differences

- `Color(0)` (black as a hex code) is black. build123d treats the 0 as "no colour code" and
  returns white.
- Python's FutureWarning for `Plane.shift_origin((x, y))` isn't ported. That warning covers a
  change in meaning, and the port has only ever had the new meaning: a 2-tuple is a local coordinate.
- Run-time argument-type checks (for example `Rotation("text")` raising TypeError) have no
  counterpart, because the compiler rules those calls out.
- Exceptions: a failure a caller should expect (a fillet, chamfer, offset, loft, draft or spline that can't be
  built, a Boolean that fails, a file that can't be read) is a failed `Result<T>` with diagnostic codes; the operators
  and constructors, which can't return one, throw `InvalidOperationException` (unit 10M3). Misuse
  stays .NET's: `ValueError` becomes `ArgumentException` (`ArgumentNullException` for a missing input), except where
  the shape being operated on is empty, which throws `InvalidOperationException` (the object's state, not an argument,
  is at fault). `RuntimeError` becomes `InvalidOperationException`. The messages are .NET sentences (unit 10M2).
- A `Vertex` given as a constrained-arc target or centre is its point. build123d adds the vertex's
  position to its point, which counts the position twice for a moved vertex.
