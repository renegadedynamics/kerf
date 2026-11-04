# Unit 10M1: what Bento sees change

Unit 10M1 moved `Kerf.Modeling` onto the kernel's public API alone (kernel decisions 17401 to 17410). Bento isn't
changed in phase 10 until 10N; this is the list of public modelling members it may meet renamed, removed or behaving
differently. Found by comparing the public API of `Kerf.Modeling.dll` at kerf main 9c6086ff with this unit's, by
reflection.

## Removed or renamed

| Before | After | Why |
|---|---|---|
| `Vector(Xyz)` | `Vector(Point3D)`, `Vector(Vector3D)`, `Vector(Direction3D)` | `Xyz` is an engine type |
| `Vector.ToGpXyz()` | `ToGpVec()`, `ToGpPnt()`, `ToGpDir()` | as above |
| `Color(ColorWithAlpha)` | `Color(Kerf.Color)` | `ColorWithAlpha` is an engine type |
| `Color.ToQuantityColorRgba()` | `Color.ToKerfColor()` | as above |
| `BoundBox.FromBndBox(Box)` | `BoundBox.FromBoundingBox(BoundingBox)`, and `ToBoundingBox()` | `Bnd_Box` is an engine type |
| `OrientedBoundBox.FromBndObb(Obb)` | `OrientedBoundBox.FromOrientedBox(OrientedBox)` | `Bnd_OBB` is an engine type |
| `ShapeHistory.FromHistory(History, …)` | `ShapeHistory.FromHistory(Kerf.Topology.ShapeHistory, …)`, and `ToHistory()` | the kernel's public history |
| `ShapeHistory.FromBoolean(BuilderAlgo, …)`, `FromUnify(UnifySameDomain)`, `FromSewing(Sewing, …)`, `FromAlgorithm(MakeShape, …)` | `FromHistory` of the operation's `Built.History` | the engine's algorithms aren't public |
| `Edge.GeomAdaptor()` returning `EdgeCurveAdaptor` | returns the kernel's `EdgeCurve` (`Evaluate`, `Derivatives`, `Range`, `Length`…) | engine adaptor |
| `Wire.GeomAdaptor()` returning `WireCurveAdaptor` | returns the kernel's `WireCurve` | engine adaptor |
| `Shape1D.GeomAdaptor()` (abstract, `CurveAdaptorBase`) | gone; `Edge` and `Wire` each have theirs | engine adaptor |
| `shape.Meshed(tolerance, angularTolerance)` (`ShapeOperations.Meshed`) | gone: `Triangulate` gives the mesh | the kernel never stores a mesh on a shape (decision 17405) |
| `Vector.Repr()`, `Location.Repr()`, `Plane.Repr()` | `ToString()`, `ToString(format)` | Python's `repr` retired (decision 17408) |

## Behaving differently

- **Numbers in text are .NET's invariant text**, not Python's `repr`: `Vertex(0, 0, 0)`, `Color: (1, 0, 0, 1) is
  'RED'`, `bbox: 0 <= x <= 1, …`, `Matrix([[1, 0, 0, 1], …])`, "Failed creating a fillet with radius of 3 on 8 of 12
  edges", "box 0 × 1 × 1 failed: …" (a whole number has no ".0"; an exponent is "1E-07"). `OrientedBoundBox`'s text is
  `OrientedBoundBox(center=(…), size=(…), plane=((…), (…), (…)))`.
- **`ToString(format)` takes .NET numeric formats** on `Vector`, `Axis`, `Location` and `Plane`: `"G6"`, `"F2"` (null
  for `"G6"`), where it took Python's `".6g"` and `".2f"`. A component within tolerance of zero still prints as zero.
- **A kernel refusal is a `KerfException`** in context, as a kernel failure was: a box of zero size gives "box 0 × 1 × 1
  failed: The value must be a positive number", code `kernel.invalid-argument` (it was `engine.domain-error`, from
  OCCT's `Standard_DomainError`). The inner exception is the kernel's (`ArgumentOutOfRangeException`, or a
  `KerfException` for an operation's own failure, as `construction.sweep-failed`), not an engine exception.
- **BREP that isn't BREP** (`Importers.ImportBrep`) throws a `KerfException` with `exchange.malformed`, where it threw
  OCCT's `Storage_StreamFormatError`.
- **STL and glTF** that can't be read throw `InvalidDataException` with the kernel's message (its `KerfException`
  inside).

## The tests rewritten for the new text (one for one)

`ErrorContextTests` `APrimitiveSaysWhatItWasMaking`, `AFeatureSaysWhichItWas` (4 cases: the code is the kernel's),
`ANestedOperationSaysBoth`, `LibraryExceptionsKeepOcctsAsTheirCause`, `AFailedBooleanNamesOcctsReasons` (the kernel
refuses a Boolean without arguments before OCCT; a good one has no errors); `BoundBoxTests.Formatting`,
`BoundBoxWithShapesTests.BoundingBoxRepr`, `ColorTests.ToStringNamesExactMatches`, `ToStringNamesTheNearestOcctColor`,
`ToOcct`, `MatrixTests.CreationAndAccess`, `OrientedBoundBoxWithShapesTests.Repr`, `PlaneTests.Formatting`,
`AxisTests`, `LocationTests` and `VectorTests` formatting (.NET formats), `VectorTests.NormalizingZeroThrows`
(`InvalidOperationException`), `ReferenceTests.ColorsFromNames` and its neighbour (Python's "1.0" read as "1"),
`FilletFailuresTests` `Operation`, `Plate`, `TangentChain`, `GeneratorDuplicatesAndReversedEdges` and `PlainMessage`,
`GroupByTests.Repr`, `VertexTests.VertexStr`; `MalformedInputTests` (the BREP errors' code); `ShapeHistoryTests.
AlgorithmHistoryFromAPrism` (the kernel's extrusion history also generates the top cap's copy).
