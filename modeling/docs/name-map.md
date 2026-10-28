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
| `Axis(edge)` | Comes with the topology port |
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

## Deliberate differences

- `Color(0)` (black as a hex code) is black. build123d treats the 0 as "no colour code" and
  returns white.
- Python's FutureWarning for `Plane.shift_origin((x, y))` isn't ported. That warning covers a
  change in meaning, and the port has only ever had the new meaning: a 2-tuple is a local coordinate.
- Run-time argument-type checks (for example `Rotation("text")` raising TypeError) have no
  counterpart, because the compiler rules those calls out.
