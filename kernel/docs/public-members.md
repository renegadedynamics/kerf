# Public members no test calls

Written by `tools/public-members.cs` from the coverage run of `tools/coverage.ps1` (phase10-plan.md 3.2, question 49:
every public member tested). The public API is every exported type outside `Kerf.Engine` in the kernel's assemblies,
and `Kerf.Modeling`. A member counts as called if a line of it ran in any test run, directly or through another
member; "no data" means coverage has no entry for it. Abstract members, fields and compiler-written members without
lines are not measured, and are counted below.

Measured: 4,470 members; called 4,406; not called 64; no data 0. Not measured: 32 abstract, 6 fields, 844 compiler-written.

| Assembly | Members | Called | Not called | No data |
|---|---:|---:|---:|---:|
| Kerf.Kernel.Foundation | 1,279 | 1,279 | 0 | 0 |
| Kerf.Kernel.ModelingData | 1,010 | 1,010 | 0 | 0 |
| Kerf.Kernel.ModelingAlgorithms | 781 | 781 | 0 | 0 |
| Kerf.Kernel.DataExchange | 114 | 114 | 0 | 0 |
| Kerf.Modeling | 1,286 | 1,222 | 64 | 0 |

## `Kerf.Modeling.BasePartObject`

- `get_Rotation()`

## `Kerf.Modeling.Box`

- `get_BoxHeight()`
- `get_Length()`
- `get_Width()`

## `Kerf.Modeling.BoxCorners`

- `ToBoundingBox()`

## `Kerf.Modeling.Circle`

- `get_Align()`
- `get_ArcSize()`
- `get_Radius()`

## `Kerf.Modeling.Curve`

- `op_Addition(Curve, IEnumerable<ModelShape>)`
- `op_Subtraction(Curve, IEnumerable<ModelShape>)`

## `Kerf.Modeling.CurveLimit`

- `op_Implicit(Compound)`
- `op_Implicit(Face)`
- `op_Implicit(Shape)`
- `op_Implicit(Shell)`
- `op_Implicit(Solid)`
- `op_Implicit(Vertex)`

## `Kerf.Modeling.GltfExportOptions`

- `set_AngularDeflection(double)`
- `set_LinearDeflection(double)`
- `set_Unit(Unit)`

## `Kerf.Modeling.ModelShape`

- `Equals(object)`
- `op_Implicit(Compound)`
- `op_Implicit(Shell)`
- `op_Implicit(Wire)`
- `op_Inequality(ModelShape, ModelShape)`

## `Kerf.Modeling.ModelShapeOperations`

- `ToSplines(T, int, double, bool)`
- `TransformGeometry(T, GeneralTransform3D)`
- `TransformShape(T, GeneralTransform3D)`
- `Transformed(T, Vector3D?, Vector3D?)`
- `Translate(T, Vector3D, bool)`
- `WithOrientation(T, Vector3D)`

## `Kerf.Modeling.Operations`

- `BoundingBox(Shape)`
- `BoundingBox(Sketch)`
- `Mirror(IEnumerable<T>, Plane)`
- `Split(IEnumerable<T>, IShape2D, Keep)`
- `Split(IEnumerable<T>, Plane, Keep)`
- `Sweep(BaseLineObject, Shape, SweepOptions)`
- `Sweep(IEnumerable<T>, IEnumerable<Edge>, SweepOptions)`
- `Sweep(IEnumerable<T>, Shape, SweepOptions)`
- `Sweep(Shell, Shape, SweepOptions)`
- `Sweep(Wire, Shape, SweepOptions)`

## `Kerf.Modeling.Part`

- `Subtract(IEnumerable<T>)`
- `op_Addition(Part, IEnumerable<ModelShape>)`

## `Kerf.Modeling.PointExtensions`

- `Describe(Direction3D)`
- `Round(Point3D, int)`

## `Kerf.Modeling.Pos`

- `.ctor(Vector3D)`

## `Kerf.Modeling.Shape1DExtensions`

- `TangentAt(TSelf)`

## `Kerf.Modeling.ShapeOperations`

- `WithForConstruction(T, bool)`
- `WithPosition(T, Point3D)`
- `op_Multiply(IEnumerable<Location>, T)`
- `op_Multiply(IEnumerable<Plane>, T)`

## `Kerf.Modeling.ShapeOperators`

- `op_Subtraction(T, IEnumerable<T>)`

## `Kerf.Modeling.ShellExtensions`

- `FoldLines(Shell)`
- `Rims(Shell)`

## `Kerf.Modeling.Sketch`

- `op_Addition(Sketch, IEnumerable<ModelShape>)`

## `Kerf.Modeling.SlotOverall`

- `get_SlotHeight()`
- `get_Width()`

## `Kerf.Modeling.StlExportOptions`

- `set_AngularTolerance(double)`
- `set_Tolerance(double)`

## `Kerf.Modeling.ThreeMfExportOptions`

- `set_AngularDeflection(double)`
- `set_LinearDeflection(double)`
- `set_Unit(Unit)`

## `Kerf.Modeling.VertexOperators`

- `op_Addition(Vertex, Vertex)`
- `op_Subtraction(Vertex, Vertex)`

## `Kerf.Modeling.WireExtensions`

- `Make(BaseLineObject)`
