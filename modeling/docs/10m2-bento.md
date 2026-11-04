# Unit 10M2: what Bento sees change

Unit 10M2 redesigned `Kerf.Modeling`'s public API for .NET (kernel decisions 1575 to 1585; design.md, "The .NET API (unit 10M2)"). This is every public member renamed, removed or retyped,
for unit 10N to work from. Found by comparing the public API of `Kerf.Modeling.dll` at kerf main c24cb02e with this
unit's, by reflection: 249 members went or changed, 346 came or changed. Results are unchanged: the same calls, written
the new way, give the same shapes.

## Selections (decision 1575)

Bento's `CAD_SEL_` functions looked members up by name (`FilterBy("IsClosed")`, the property tables): those names are
gone, and Bento maps its own names to these calls (plan 2.9: "Bento's couplings become explicit").

| Before | After |
|---|---|
| `list.FilterBy(Func<T, bool>, reverse)` | LINQ's `Where(e => …)`, then `ToShapeList()` for a `ShapeList<T>`; reverse by negating |
| `list.FilterBy(GeomType, reverse)` | `Where(GeomType)`, `WhereNot(GeomType)` |
| `list.FilterBy(Convexity, reverse)` | `Where(Convexity)`, `WhereNot(Convexity)` |
| `list.FilterBy(Axis, reverse, tolerance)` | `ParallelTo(Axis, tolerance)`, `NotParallelTo(Axis, tolerance)` |
| `list.FilterBy(Plane, reverse, tolerance)` | `ParallelTo(Plane, tolerance)`, `NotParallelTo(Plane, tolerance)` |
| `list.FilterBy(string property, reverse)` | Removed: `Where(e => e.IsClosed)`, or Bento's own table |
| `list.FilterByPosition(axis, min, max, (bool, bool)? inclusive)` | `Between(axis, min, max, includeMinimum, includeMaximum)` |
| `list.FilterByValue(SortBy, value, tolerance, reverse)` | `WithValue(ShapeKey, value, tolerance)`, `WithoutValue(…)` |
| `list.FilterByValue(Func<T, TValue>, value, tolerance, reverse)` | `WithValue(Func<T, TValue>, value, tolerance)`, `WithoutValue(…)` |
| `list.FilterByValue(string measure, object value, …)` | Removed: `WithValue(e => e.Radius, 5.0)`, or `WithValue(ShapeKey.Radius, 5)` |
| `list.SortBy(Axis, reverse)` | `OrderBy(Axis)`, `OrderByDescending(Axis)` |
| `list.SortBy(SortBy, reverse)` | `OrderBy(ShapeKey)`, `OrderByDescending(ShapeKey)` |
| `list.SortBy(Shape1D path, reverse)` | `OrderBy(Shape1D)`, `OrderByDescending(Shape1D)` |
| `list.SortBy(Func<T, TKey>, reverse)` | LINQ's `OrderBy(e => …)`, `OrderByDescending(e => …)` (then `ToShapeList()`) |
| `list.SortBy(string property, reverse)` | Removed |
| `list.SortByDistance(Shape | Vector, reverse)` | `OrderByDistanceTo(…)`, `OrderByDescendingDistanceTo(…)` |
| `list.GroupBy(Axis | SortBy | Shape1D, reverse, tolDigits)` | `GroupBy(Axis | ShapeKey | Shape1D, digits)`; reverse with LINQ's `Reverse()` |
| `list.GroupBy(Func<T, double>, reverse, tolDigits)`, `GroupBy<TKey>(Func<T, TKey>, …)` | `GroupByValue(Func<T, double>, digits)`, `GroupByValue<TKey>(Func<T, TKey>, IComparer<TKey>?)` |
| `list.GroupBy(string property, …)` | Removed |
| `list.GroupByConvexity(reverse)` | `GroupByConvexity()` |
| `GroupBy<T, TKey>` (type) | `ShapeGroups<T, TKey>`; `Keys` is an `IReadOnlyList<TKey>` |
| `GroupBy.First`, `GroupBy.Last`, `ShapeList.First`, `ShapeList.Last` (properties) | LINQ's `First()`, `Last()`, or `[0]`, `[^1]` |
| `SortBy` (enum) | `ShapeKey` (the same members) |
| `list > axis`, `list < axis`, `list > sortBy`, `list < sortBy` | `OrderBy(…)`, `OrderByDescending(…)` |
| `list | Axis`, `| Plane`, `| GeomType`, `| Convexity` | `ParallelTo(…)`, `Where(…)` |
| `list >> axis`, `list << axis` (and by `SortBy`) | `GroupBy(…).Last()`, `GroupBy(…).First()` |
| `list + shape`, `list + shapes`, `list - shapes`, `list & shapes` | `Add`, `AddRange`, `Except`, `Intersect` |
| `Selector.FilterByGeomType`, `FilterByAxis`, `GroupByMeasure(SortBy, …)` (steps) | `Selector.OfGeomType`, `ParallelTo`, `GroupByKey(ShapeKey, …)`; `GroupByAxis` as before |
| `Selector.ToPython()` | Removed; `ToString()` prints the C# (`Faces().GroupBy(Axis.Z).Last()`) |

## Booleans and typed results (decisions 1576, 1577)

| Before | After |
|---|---|
| `Shape.Add(params IEnumerable<Shape?>)`, `Subtract`, `CommonWith(params IEnumerable<object>)` (and their overrides on `Compound`, `Curve`, `Shape1D`, `Shape2D`, `Vertex`) | Internal. On a `Part`, `Sketch` or `Curve`: `Add`, `Subtract`, `Common` (`params IEnumerable<Shape>`), returning the class, and `TryAdd`, `TrySubtract`, `TryCommon` |
| `Shape + Shape?`, `Shape - Shape?`, `Shape & Shape` → `Shape` | Unchanged for the topological shapes (the kernel's result); on a `Part`, `Sketch` or `Curve` the operator returns that class. A null operand throws `ArgumentNullException` (it added nothing) |
| `Operations.Mirror`, `Scale`, `Split`, `Offset` of one shape → `Compound` | Unchanged for any `Shape`; new overloads for a `Part`, `Sketch` or `Curve` return that class |
| `Operations.Sweep(…)` → `Compound` | Unchanged for the general forms; new overloads: a `Shape2D` or `Sketch` section gives a `Part`, a `Shape1D` or `Curve` section a `Sketch` |
| `Curve % position`, `Shape1D % position` | `TangentAt(position)` (`Curve.TangentAt` is new) |
| `Curve ^ position`, `Shape1D ^ position` | `LocationAt(position)` (`Curve.LocationAt` is new) |
| `Axis & …`, `Plane & …`, `Location & …`, `Vector & …` | `Intersect(…)` (the methods were there) |

## Options (decision 1579)

| Before | After |
|---|---|
| `Extrude(Shape | IEnumerable<Face>, double? amount, Vector? dir, Until? until, Shape? target, bool both, double taper, bool clean)` | `Extrude(…, double amount, Operations.ExtrudeOptions? options)` and `Extrude(…, Until until, Shape target, ExtrudeOptions? options)`; `ExtrudeOptions { Direction, Both, Taper, Clean }` |
| `Sweep(sections | section, path, bool multisection, bool isFrenet, Transition transition, Vector? normal, Shape1D? binormal, bool clean)` | `Sweep(…, path, Operations.SweepOptions? options)`; `SweepOptions { Multisection, IsFrenet, Transition, Normal, Binormal, Clean }` |
| `Offset(objects | obj, double amount = 0, IEnumerable<Face>? openings, Kind kind, Side side, bool closed, double? minEdgeLength)` | `Offset(…, double amount, Operations.OffsetOptions? options)`; `OffsetOptions { Openings (ImmutableArray<Face>), Kind, Side, Closed, MinEdgeLength }` |
| `Thicken(Shape | IEnumerable<Face>, double? amount, Vector? normalOverride, bool both, bool clean)` | `Thicken(…, double amount, Operations.ThickenOptions? options)`; `ThickenOptions { NormalOverride, Both, Clean }` |
| `Operations.Scale(…, (double, double, double) by, …)`, `shape.Scale((double, double, double) factors, …)` | `Scale(…, Vector by, …)`, `Scale(Vector factors, …)` |

## Errors (decision 1578)

| Before | After |
|---|---|
| A failed 3D fillet: `ArgumentException` "Failed creating a fillet with radius of 3 …, try a smaller value or use max_fillet() …" | `KerfException`, first error `modeling.fillet-failed`: "The fillet with a radius of 3 … failed; try a smaller radius, or MaxFillet to find the largest that works.", the kernel's diagnostics after it |
| A failed chamfer: `ArgumentException` | `KerfException`, `modeling.chamfer-failed` |
| A 2D fillet's corner that can't be filleted, an unclosed result: `ArgumentException` | `KerfException`, `modeling.fillet-failed` |
| A 2D or face offset that fails: `InvalidOperationException` | `KerfException`, `modeling.offset-failed` |
| A spline that can't be interpolated or approximated: `InvalidOperationException` | `KerfException`, `modeling.spline-failed` |
| `DraftAngleError` (type, with `Face` and `ProblematicShape`) | Removed: `KerfException`, `modeling.draft-failed`, the face and shape as the error's `Shapes` |
| "objects must be provided", "path must be provided", "A face or sketch must be provided" (`ArgumentException`) | `ArgumentNullException` with the parameter's name |
| (none) | `ModelingDiagnostics` (the codes); `Operations.TryFillet`, `TryChamfer`, `TryExtrude` (two), `TryLoft`, `TryRevolve`, `TrySweep`, `TryOffset` (Part, Sketch), `TryThicken`, `TryDraft`, each a `Result<T>` |

Bento's `ModelFunctions.Run` shows any exception's message, so a cell shows the new messages unchanged; with the `Try`
forms it can show `Error.Message` without catching. Messages that named Python (snake_case parameters, `None`,
`BuildSheet`) are .NET sentences now: 32 of them.

## Mode, tuples and names (decisions 1580, 1581, 1583)

| Before | After |
|---|---|
| `Mode` (enum), `IBuilderObject.Mode`, `Base*Object.Mode` | Removed, and `IBuilderObject` with it |
| Every object's `Mode mode` constructor parameter (Airfoil, BSpline, Bezier, BlendCurve, Box, CenterArc, Circle, Cone, ConstrainedArcs, ConstrainedLines, ConvexPolyhedron, CounterBoreHole, CounterSinkHole, Cylinder, DoubleTangentArc, Ellipse, EllipticalCenterArc, EllipticalStartArc, FilletPolyline, Helix, Hole, HyperbolicCenterArc, IntersectingLine, JernArc, Line, ParabolicCenterArc, PolarLine, Polygon, Polyline, RadiusArc, Rectangle, RectangleRounded, RegularPolygon, SagittaArc, SlotArc, SlotCenterPoint, SlotCenterToCenter, SlotCurve, SlotOverall, Sphere, Spline, Superellipse, TangentArc, ThreePointArc, Torus, Trapezoid, Triangle, Wedge) | Removed; the other parameters are as they were |
| `(x, y)`, `(x, y, z)` to `Vector`, `CurveLimit`, `Tangent` (implicit) | `new Vector(x, y, z)` |
| `(x, y, z)` to `Rotation` | `new Rotation(x, y, z)` |
| `(Align, Align)`, `(Align, Align, Align)` to `Alignment` | `new Alignment(x, y)`, `new Alignment(x, y, z)` |
| `(Edge, Tangency)`, `(Axis, Tangency)` to `Tangent` | `new Tangent(edge, tangency)`, `new Tangent(axis, tangency)` |
| `Plane.ShiftOrigin((double, double) local)` | `ShiftOriginBy(double x, double y)` |
| `Shape.Wrapped` | `KernelShape` |
| `Vector.ToGpPnt()`, `ToGpDir()`, `ToGpVec()` | `ToPoint3D()`, `ToDirection3D()`, `ToVector3D()` |
| `Axis.ToGpAx1()` | `ToAxis3D()` |
| `Location.ToTopLoc()`, `ToGpTrsf()` | `ToPlacement()`, `ToTransform3D()` |
| `Plane.ToGpPln()`, `ToGpAx3()`, `ToGpAx2()` | `ToPlane3D()`, `ToCoordinateSystem()`, `ToFrame3D()` |
| `Matrix.ToGpGTrsf()` | `ToGeneralTransform3D()` |

## Bento's couplings

- **Enum text** parsed against C# enum names: `SortBy` is `ShapeKey`, `Mode` is gone; the others are as they were.
- **Selector property names by reflection**: no member takes a name now; Bento's table maps its names to the calls
  above (a measure to a `ShapeKey`, a test such as "is closed" to a delegate).
- **`A1.Volume`-style property reads by reflection**: the shapes' properties are unchanged.
- **The one operator Bento uses**, `Location *`, is unchanged.
