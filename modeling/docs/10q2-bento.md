# Unit 10Q2: what Bento sees change

Unit 10Q2 finished what 10Q left (kernel decisions for unit 10Q2; [one-set-of-shapes.md](one-set-of-shapes.md)). Results are
unchanged: volumes, face counts, the Python-reference values, and BRep and STEP bytes on Bento's examples and 14 others, by
hash. Bento's benchmark is no slower. This is every change a caller meets. **A member not listed keeps its name and meaning.**

Importing: `using Kerf.Geometry;` as well as `using Kerf.Topology;` and `using Kerf.Modeling;`. A file can now import those with
`Kerf` and `Kerf.Measurement`, and qualify nothing common.

## `Vector` goes

A position is a `Point3D`, an offset or a computed direction a `Vector3D`, and a unit direction that came from the kernel a
`Direction3D`. A `Direction3D` converts implicitly to a `Vector3D`, so anything taking a direction takes either.

| Before | After |
|---|---|
| `new Vector(1, 2)`, `new Vector(1, 2, 3)` | `new Point3D(1, 2, 0)`, `new Point3D(1, 2, 3)`, or `new Vector3D(…)` for an offset: z is always given |
| `new Vector(point)`, `new Vector(vec)`, `new Vector(dir)` | The value itself |
| `new Vector([1, 2, 3])`, `new Vector(vertex)`, `vertex.ToVector()` | The constructor; `vertex.Point` |
| `v.ToPoint3D()`, `v.ToVector3D()`, `v.ToDirection3D()` | `v.ToPoint()`, `p.ToVector()`, `v.ToDirection()` |
| `a == b`, `Assert.Equal(a, b)`, `HashSet<Vector>`, `Distinct()` on vectors | `a.IsNear(b)` (five places, as before); `PointComparer.Near` for sets. **`==` still compiles and is now exact** |
| `v.Length`, `v.Reverse()` | `v.Magnitude`, `v.Reversed()` |
| `v.Cross(w)`, `v.Dot(w)`, `GetAngle`, `GetSignedAngle`, `ProjectToLine`, `Rotate`, `Round` | The same on `Vector3D` (`Cross`, `Dot`, the angles also on `Direction3D`). `Direction3D.Crossed` is the kernel's, normalised; `Cross` isn't |
| `p.DistanceToPlane`, `SignedDistanceFromPlane`, `ProjectToPlane`, `Center`, `Intersect(…)` | The same on `Point3D` |
| `v[i]` | `v.X`, `.Y`, `.Z`: there are no extension indexers |
| `p + q`, `p * 2`, `p / 2`, `-p` on positions | `p + offset`, `p - q` (a `Vector3D`). To average: `((p.ToVector() + q.ToVector()) / 2).ToPoint()` |
| `v / n` | The same, but the kernel divides where the old type multiplied by `1.0 / n`. Write `v * (1.0 / n)` for its exact bits |
| `dir * 2` | The same: a `Vector3D` |
| `v.ToString()`, `v.ToString("F3")` | `Describe()`, `Describe("F3")`. `ToString()` is the kernel's (`gp_Pnt(…)`), and `$"{p:F3}"` **ignores the format** |
| `a.Intersect(b) is Vector point`, `Assert.IsType<Vector>` | `is Point3D point`: an `IGeometry?`, which is now in `Kerf.Geometry` |
| `object` arguments: `new Locations(vector)` | A `Point3D` or `Vector3D`. `Intersect(params object[])`, `TrimToOther` and a sweep's binormal take the type meant: a point, and a vector or direction |

Which type each member uses:

| Type | Members |
|---|---|
| `Point3D` | `Center()`, `PositionAt`, `Positions`, `StartPoint`, `EndPoint`, `ArcCenter`, `Position` (shape, location, axis), `Plane.Origin`, `Plane.ToLocalCoords(point)`, the points of `Line`, `Polyline`, `Spline`, arcs, `Bezier`, `Polygon`, `Locations`, `ClosestPoints`, `Tessellate`, `about:` of `Scale`, `center:` |
| `Vector3D` | `TangentAt`, `NormalAt`, `DerivativeAt`, `Translate`, `Extrude(…, direction)`, `Size`, `Orientation` and `OrientationAngles` (degrees), `Rotation(angles)`, scale factors, `xDir:`, `zDir:`, `normal:` and `direction:` arguments |
| `Direction3D` | `Axis.Direction`, `Plane.XDir`, `.YDir`, `.ZDir`, `OrientedBoundBox.XDirection`…, `PrincipalProperties`' axes, `Normal()` of a planar curve |

`new Location(offset)` and `new Pos(point)` take either. `Vertex + Vector3D`, `Vertex - Vector3D` are as they were.

## `BoundBox` goes

| Before | After |
|---|---|
| `BoundBox` | The kernel's `BoundingBox`, a struct. `shape.BoundBox()` keeps its name and returns one |
| `box.Min`, `box.Max` on an empty box: the origin | **They throw `InvalidOperationException`**. Check `box.IsEmpty`. `Size` is zero, and the layer's `Center()`, `Diagonal`, `Measure` and `ToAlignOffset` still treat it as nothing |
| `BoundBox.Empty`, `FromBoundingBox`, `ToBoundingBox`, `FromShape(s)`, `FromTopoDS(s)` | `BoundingBox.Empty`; nothing; nothing; `s.BoundBox()`; `s.BoundBox()` |
| `box.Add(point, tolerance)`, `box.Add(other, tolerance)` | `box.Including(point)`, `box.Union(other)`: no gap (below) |
| `box.Intersects(other, tolerance)` | `box.IntersectsNear(other, tolerance)`. **`box.Intersects(other)` still compiles and is now the kernel's exact test** |
| `Covers`, `CoveredBy`, `Contains(box)`, `ContainsProperly`, `Within`, `Disjoint`, `Touches`, `Overlaps`, `Measure`, `Diagonal`, `Center()`, `ToAlignOffset` | The same, as extension members |
| `box.ToString()` | `box.Describe()` |
| `plane.ToLocalCoords(box)`, `FromLocalCoords(box)`: a `BoundBox` | `BoxCorners`, with `Min` and `Max`: each corner as it lands, so `Min` can be above `Max`, as build123d's tests pin. `.ToBoundingBox()` gives the box around both |

**The gap.** The old box kept a gap for `Add`, as OCCT's does. Only a box begun from nothing by `Add(point, tolerance)` had one: every box of a shape had none, so it reached no result.

## `Matrix` goes

| Before | After |
|---|---|
| `Matrix`, `new Matrix()`, `new Matrix(trsf)` | The kernel's `GeneralTransform3D`, a struct: `new GeneralTransform3D()`, `new GeneralTransform3D(trsf)` |
| `new Matrix(values)` | `GeneralTransform3D.FromValues(values)`, which keeps a similarity's form and scale |
| `m[row, col]`, from zero | `m.At(row, col)`. **`m[row, col]` still compiles and counts from one**, as OCCT does |
| `a * b`, `a.Multiply(b)` | `a.Compose(b)`. **`a * b` still compiles and is OCCT's product**, which loses a similarity's scale factor beside a general matrix |
| `m * point`, `m.Multiply(point)`, `point.Transform(m)` | `m * point`, `point.Transform(m)` |
| `m.Inverse()`, `m.ToGeneralTransform3D()` | `m.Inverted()`, nothing |
| `m.Rotated(axis, degrees)`, `TransposedList()`, `Svd()` | The same, as extension members |
| `m.ToString()` | `m.Describe()` |

## Flags and selections

| Before | After |
|---|---|
| `edge.IsClosed` on a struct: OCCT's stored flag | Closed in fact, on any shape or object: an edge's two vertices the same, a wire with no free vertex, a face or shell with no free edge, every child of a solid or compound. `Where(e => e.IsClosed)` now gives what it reads as |
| `edge.CheckClosed()` | `edge.IsClosed`. On nothing it throws `InvalidOperationException`, where it was `ArgumentException` |
| `shape.IsInfinite`, `IsOrientable` on a struct: OCCT's stored flags | Gone from the public API (internal as `InfiniteFlag`, `OrientableFlag`). `IsClosed`'s is `ClosedFlag`, internal |
| `edge.IsUnbounded` | `edge.IsInfinite`: a line longer than 1e100, an axis made into an edge |
| `solid.SelectEdges()`, `SelectVertices`, `SelectWires`, `SelectFaces`, `SelectShells`, `SelectSolids` | `solid.Edges()` and so on: a `ShapeList`, with its origin, leaving out degenerate edges, as on a `Part` |
| The kernel's `solid.Edges()`, an `ImmutableArray` with degenerate edges (and the other five) | `solid.AllEdges()`, `AllVertices`, `AllWires`, `AllFaces`, `AllShells`, `AllSolids`. **Code that kept `.Edges()` compiles and gets the selection**: `.Length` becomes `.Count` |
| `shape.OrientationAngles` | The same: the kernel's `Orientation` is the enum, which is what the word means on a shape |

## Measurement

| Before | After |
|---|---|
| `solid.Volume`, `.Area`, `edge.Length`, which were ambiguous beside `using Kerf.Measurement;` | The same, and no longer ambiguous |
| The kernel's `solid.Volume(options)`, `.Area(options)`, `.Length(options)` | `IntegrateVolume(options)`, `IntegrateArea(options)`, `IntegrateLength(options)` |

They differ here. `Volume`: a vertex, edge, wire or face has none; a compound's counts the solids in it. `IntegrateVolume` is what
the shape's faces bound, whatever it is: a sphere's lone face gives the sphere's volume, and a compound counts its loose faces. On
nothing, `Volume`, `Area` and `Length` are zero, and the `Integrate` forms throw `ArgumentException`.

## Geometry names

| Before | After |
|---|---|
| `Kerf.Geometry.Cylinder`, `Cone`, `Sphere`, `Torus` (the analytic values) | `Cylinder3D`, `Cone3D`, `Sphere3D`, `Torus3D`. `new Cylinder(1, 2)` is always the object |
| `Kerf.Modeling.Tangency` | The kernel's `Kerf.Geometry.Tangency`: the same four values |
| `IGeometry` in `Kerf.Modeling` | In `Kerf.Geometry`. `Point3D`, `Axis`, `Plane` and `Location` implement it |

## Bento's couplings

- **A cell's value.** A point, vector, box or transform in a cell is now a boxed kernel struct. Bento's table of properties can't
  find `Magnitude`'s neighbours by reflection where they are extension members (10Q's note on `*Extensions`), so it calls them.
- **Text.** `Vector: (X=1, Y=2, Z=3)` is now that for a `Vector3D`'s `Describe()`, `Point: (…)` for a `Point3D`'s and `Direction: (…)` for a
  `Direction3D`'s. An `Axis`, `Plane` or `Location`'s `ToString()` is unchanged.
- **Silent changes to check by hand**, as they still compile: `==` and `Equals` on points; `box.Intersects(other)`; `a * b` and
  `m[r, c]` on transforms; `.Edges()` on a struct; `$"{p:F3}"`.
