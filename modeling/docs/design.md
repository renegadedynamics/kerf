# build123d.NET design

A .NET port of [build123d](https://github.com/gumyr/build123d), the Python CAD library built on
OpenCascade (OCCT).

## Decisions

| Topic | Decision |
|---|---|
| Target framework | .NET 10 |
| Platforms | Windows, Linux and macOS (x64 and arm64) from the start |
| Naming | Idiomatic .NET (PascalCase, `Try*`, properties instead of getters). A Python → .NET name map is kept in `docs/name-map.md`. |
| OCCT | 8.0.x, built from source and linked dynamically (OCCT is LGPL 2.1 with an exception) |
| Bindings | A thin C ABI over the subset of OCCT that build123d uses, generated from a whitelist, with C# `SafeHandle` wrappers on top |

## Principles

### 1. Immutability

All modelling values are immutable. Forking a partly built design is plain assignment: no copies,
no aliasing surprises.

- **Geometry values** (`Vector`, `Axis`, `Plane`, `Location`, `Color`, `BoundBox`, `Matrix`) are
  `readonly struct`s or immutable records.
- **Shapes** (`Vertex` … `Compound`) are immutable classes. Every operation returns a new shape.
  The Python in-place mutators become methods that return a copy:

  | Python (mutates) | .NET (returns new) |
  |---|---|
  | `locate(loc)` / `move(loc)` | `Located(loc)` / `Moved(loc)` |
  | `clean()` | `Clean()` |
  | `shape.location = …`, `.position`, `.orientation` | `WithLocation(…)`, `WithPosition(…)`, `WithOrientation(…)` |
  | `shape.color = …`, `.label`, `.material` | `WithColor(…)`, `WithLabel(…)`, `WithMaterial(…)` |

- **Structural sharing.** A `TopoDS_Shape` is a handle to shared geometry (`TShape`), plus a location
  and an orientation. Because nothing changes existing geometry, every shape derived from a common
  base shares that base's geometry for free.
- **OCCT APIs that modify in place** (`BRep_Builder` edits, some `ShapeFix` / `ShapeUpgrade` calls) are
  reachable only through an internal "copy, then modify" helper.
- **Assemblies** are immutable trees. A child holds no back-reference to its parent; you get parent
  context by navigating from the root. This is the main departure from Python's two-way
  `parent`/`children` links.
- **Builders** (`BuildPart`, `BuildSketch`, `BuildLine`) are mutable accumulators, but they store their
  state as immutable snapshots. That makes `Fork()` O(1): the new builder starts from the same shape
  and the same pending locations.
- **Known exception: triangulation.** Meshing attaches a triangulation cache to the shared `TShape`.
  The cache is harmless, but meshing the same geometry from several threads at once must be
  serialised.

### 2. Thin, generated bindings; idiomatic API above them

`Build123d.Occt` mirrors OCCT class and method names, so Python lines like
`BRepAlgoAPI_Cut(a, b).Shape()` port almost one-to-one. Only `Build123d` itself exposes idiomatic
.NET names. `Shape.Wrapped` remains as an escape hatch to the raw OCCT handle.

#### How the bindings work

`tools/OcctGen` parses the OCCT headers with ClangSharp and binds every class (or namespace) listed in
`bindings/occt.txt`. It emits a C++ shim (`native/shim/generated`, built into `b3docct`) and C# wrappers
(`src/Build123d.Occt/Generated`). The generated code is committed, so building needs no libclang.
`bindings/occt.skipped.txt` lists every member left out, and why.

- **Objects** cross the C ABI as opaque pointers.
  - Value classes (`gp_Pnt`, `TopoDS_Shape`, …) are heap copies that the wrapper owns.
  - Transients (`Standard_Transient` subclasses, held by `occ::handle`) are handed out with one
    reference taken, which the wrapper releases.
  - Wrappers free their object on `Dispose()` or finalisation.
- **Returns** of `const T&` are copied. Returns of a non-const `T&` are skipped, because a copy can't
  honour writes into the object's state.
- **Errors:** each shim function catches C++ exceptions and reports them through a trailing out-parameter,
  which the wrapper turns into `OcctException` (with `OcctType`, e.g. `Standard_DomainError`).
- **Inheritance** mirrors OCCT. A class's C# base is its nearest whitelisted ancestor. Methods of
  ancestors that aren't whitelisted are flattened into the class. Base-class pointers come from generated
  cast functions, which is needed because OCCT uses some multiple inheritance.
- **Default arguments** become one overload per arity, because OCCT defaults are often non-constant.
- **Namespaces of free functions** (OCCT 8 turned `TopoDS` into one) become static classes.

#### Geometry values

- **`Vector`, `Axis` and `Color` are `readonly struct`s.**
  - Plain arithmetic (add, dot, cross, normalise) runs in C#, in the same operation order as OCCT,
    so results are bit-identical.
  - Anything OCCT computes non-trivially goes through short-lived OCCT objects, so the numbers and
    the exceptions match build123d. This covers angles, axis tests, rotation, and sRGB ↔ linear
    colour conversion.
- **`Location`, `Plane`, `Matrix`, `BoundBox` and `OrientedBoundBox` are immutable classes**, each
  holding one private OCCT object that is never modified.
  - A `Location` keeps its `TopLoc_Location` for life. OCCT compares shape locations by identity, so
    shapes placed with the same `Location` instance share their location, as in build123d.
  - Public accessors (`ToGpPln()`, `ToGpGTrsf()`, …) return copies. `ToTopLoc()` returns a
    `TopLoc_Location` that shares the location's identity.
- **`IGeometry`.** Intersections between geometric values can return several types, so they return
  `IGeometry?` for pattern matching.
- **Names** are listed in [name-map.md](name-map.md), with the deliberate differences from build123d.

#### Shapes

- **Classes** follow build123d's: `Shape` ← `Vertex`; `Shape1D` ← `Edge`, `Wire`; `Shape2D` ← `Face`,
  `Shell`; `Shape3D` ← `Solid`, `Compound` ← `Curve`, `Sketch`, `Part`. The `Mixin1D/2D/3D` bases are
  abstract classes here.
- **`Self` return types.** Operations that keep a shape's class (`Moved`, `Located`, `Rotate`, `Clean`,
  `WithLabel`, `Location * shape`, …) are C# 14 extension members generic in the shape's type, so
  `box.Moved(loc)` is still a `Box`. A copy is a `MemberwiseClone` around a new `TopoDS_Shape`, which
  carries a subclass's own fields (a Box's dimensions) as Python's `copy.copy` does.
- **History** (`ShapeHistory`, behind `Select.Last` and `Select.New`) is immutable: `Merge` returns a new
  record built on a copy of the OCCT `BRepTools_History`, so shapes sharing a record never see it change.
- **Collections.** `ShapeList<T>` is an immutable list over Bento.Collections' `IBentoList<T>`. OCCT's
  template collections (`NCollection_List<TopoDS_Shape>` and friends) are bound under their legacy names
  (`TopTools_ListOfShape`, …) and enumerate as `IEnumerable`.

#### Immutability in the kernel

A `TopoDS_Shape` is a handle: shapes made from one another share their `TShape`s, and so do the shapes
in different cells of a spreadsheet. Some OCCT algorithms write into the topology they are given: they
add pcurves to its edges, raise its tolerances or store a mesh on its faces. Without care, that would
change every shape sharing it, and race between threads. So:

- **Meshing works on a private copy** (`TopoUtil.PrivateCopy`): `Tessellate`, and `Meshed`, which
  replaces build123d's in-place `mesh`. Bounding boxes ignore triangulations instead of calling
  `BRepTools::Clean`.
- **Booleans, splits and sections run non-destructive** (`SetNonDestructive`), so OCCT copies an input
  it would otherwise change.
- **Algorithms that edit what they're given get copies:**
  - ShapeFix in `Fix()`, `FixDegenerateEdges` and the 2D chamfer;
  - loft sections;
  - the profile of a tapered extrusion, whose prism shares the profile's face. Its copy's history is
    merged in front, so selection by history still reaches the original.
- **What is shared stays shared** when Python relies on it. A face made from a wire keeps the wire's
  edges, for example. These are only copied if the test below finds a change.
- **The test is the rule.** [InputsUnchangedTests](../tests/Build123d.Tests/Immutability/InputsUnchangedTests.cs)
  snapshots the inputs as BRep text, triangulations included, and checks them after each case.
  - It runs a curated list of operations, a reflection sweep of every public member that takes no
    arguments, and all the cases concurrently on shared inputs.
  - It ignores only OCCT's Free and Checked flags, which are bookkeeping: any shape put into a new
    compound loses Free.
  - A new operation goes into its list.
- **Warnings** are collected per operation by `Warnings.Capture()` scopes that follow the async flow,
  not by a static event.

#### Meshes

- **`Triangulate` is for viewers and picking**, and leaves `Tessellate` as build123d has it. It meshes
  a private copy of the shape. The copy has the shape's structure, so its `Faces()`, `Edges()` and
  `Vertices()` come in the same order: index *i* of the mesh is index *i* of the shape.
- **The shim copies out the whole mesh in two calls**
  ([b3d_mesh.cpp](../native/shim/src/b3d_mesh.cpp)): one gathers every face's nodes, normals and
  triangles and every edge's polyline into flat buffers, and the next copies them out. A face's normals
  come from its surface (`BRepLib_ToolTriangulatedShape::ComputeNormals`). Its winding is flipped for a
  reversed face or a mirroring location. An edge's polyline is its polygon on an adjacent face's
  triangulation, so it lies exactly on the shading. The faces and edges of a `ShapeMesh` are read-only
  slices of those buffers. Copying out 100k triangles takes a few milliseconds; OCCT's meshing dominates.
- **`TriangulateTree`** follows `AssemblyChildren`. It gives a node per child with its label, colour and
  location relative to its parent, and meshes each `TShape` once in its own coordinates. Instances share
  the same `ShapeMesh` object.

#### Errors and memory

- **Faults come back as exceptions.** The shim is built with `/EHa`, so its `catch (...)` stops an
  access violation in OCCT from ending the process. On Windows each shim function also starts with
  `B3D_TRY`, which installs OCCT's structured-exception translator (`OSD::SetThreadLocalSignal`) the
  first time a thread calls in.
  - With the translator, a fault is a typed `Standard_Failure`, such as `OSD_Exception_ACCESS_VIOLATION`
    with the address, so OCCT's own algorithms can catch it and recover as they were designed to.
  - Floating-point traps stay off, as .NET expects.
  - On Linux and macOS, OCCT's handlers would be process-wide signal handlers that take over the
    runtime's own, so nothing is installed there.
  - A stack overflow, or heap corruption that has already happened, can still end the process.
  - `Occt.FaultForTesting` makes a deliberate fault for the tests.
- **Failures say what was being done.** `OcctContext.Doing("extrude")` scopes nest per thread, and an
  `OcctException` thrown inside them starts with them: "box 0.0 × 1.0 × 1.0 failed:
  Standard_DomainError". These scopes are set:
  - on primitives and on extrude, revolve, loft, sweep and thicken;
  - on booleans, as "fuse of a Box with 2 Cylinders";
  - on triangulation.

  A boolean that fails reports OCCT's alerts (`BOPAlgo_Alert…`) instead of a bare `StdFail_NotDone`.
  build123d's own messages (fillet, chamfer, offset) stand, with OCCT's exception inside.
- **Memory pressure goes on results.** `Shape.Result`, called from `MadeBy`, `Rewrapped` and the
  feature factories, gives the result's `TopoDS_Shape` wrapper an estimate of its native memory, and
  calls `GC.AddMemoryPressure` with it. The estimate is one walk over its unique sub-shapes, curves,
  surfaces and triangulations.
  - The pressure is removed when that wrapper is disposed or finalised.
  - Results under 4 KB, and sub-shapes taken out of a result, aren't counted, so shared geometry isn't
    counted many times over.
  - Triangulation frees its private meshed copy as soon as the mesh is copied out, rather than leaving
    it for finalisation.
- **Untrusted BRep.** OCCT's reader trusts its input. At the end of truncated data it loops for ever,
  reusing the last word it read, and it reads words into fixed buffers with no length limit. So
  `ReadBRep` first refuses NULs and words of 200 characters or more, then reads with stream exceptions
  on, and reports any failure as `Storage_StreamFormatError`.
  - A fuzz test mutates BRep data and checks that reading, validating and measuring the result fail
    cleanly or not at all.
  - Meshing or fusing a shape read from corrupted data can still run for a very long time, which is
    left to cancellation (4.7).

#### Objects

- **Objects are shapes.** `Box` is a `Part`, `Circle` a `Sketch`, `Line` an `Edge`, `Polyline` a `Wire`,
  as in build123d. Each computes its shape in a static method and hands it to its base constructor
  (`BasePartObject`, `BaseSketchObject`, `BaseEdgeObject`, `BaseLineObject`, `BaseCurveObject`), which
  aligns and rotates it. An object's parameters are read-only properties, and the self-typed operations
  keep the class, so `box.Moved(loc)` is still a `Box` with its `Length`.
- **Defaults.** `align: null` means the object's own default (centred for most), and `Align.None` means
  none, so a caller can ask for either. A `Mode` is stored on every object for the builders of phase 5;
  algebra mode ignores it.
- **Python's union arguments** become overloads: an arc's size is a `double` or a `CurveLimit` (a shape,
  axis, location, plane or point it's trimmed to). Constrained arcs and lines take `Tangent` targets.

### 3. Behavioural parity is tested, not assumed

- The pytest suite is ported module by module, alongside the code it tests.
- A differential harness runs Python build123d and the port on the same inputs and compares the
  results. For geometry that's [tests/reference](../tests/reference), held to about 1e-12. For shapes
  it will also compare volume, area, bounding box and topology counts within a tolerance.

## Phases

0. **Spike.** Build OCCT 8.0 on all platforms; generate bindings for `gp`, `BRepPrimAPI`, `BRepAlgoAPI`
   and `STEPControl`; make a box minus a cylinder and export it to STEP from C#, in CI.
1. `geometry.py`: Vector, Axis, Plane, Location, BoundBox, Color, Matrix.
2. Topology core: Shape, ShapeList, Vertex, Edge, Wire, Face, Shell, Solid, Compound.
3. Objects and operations in algebra mode, with the constrained arc and line solvers the objects need.
4. **Bento readiness**, for the first user, Bento's 3D-models extension (see
   [bento-integration.md](bento-integration.md)), in its order of priority:
   1. **Packaging.** NuGet packages (`Build123d`, `Build123d.Occt`) on a local folder feed, with every
      native DLL flat under `runtimes/win-x64/native`. A test loads the library through a fresh
      collectible `AssemblyLoadContext` from another folder and tessellates a box. The native set is
      pruned by actual dependencies, and `THIRD-PARTY-NOTICES.md` and a changelog are added.
   2. **No mutation, thread safety.** Meshing works on a copy, not the shared `TShape`. Bounding
      boxes don't `BRepTools::Clean` the shape. `Fix()` and the other ShapeFix calls work on a copy.
      Booleans run non-destructive. Warnings belong to the operation, not a static event. A test runs
      every public operation, from several threads, and checks that the inputs' BRep bytes are
      unchanged. BRep to bytes, needed for it, is brought forward from import and export.
   3. **Tessellation.** A structured, immutable mesh per shape (faces, edges and vertices in the order
      of `Faces()`, `Edges()` and `Vertices()`, with nodes shared within a face, normals and correct
      winding), copied out in bulk by shim functions. It gets default tolerances from the bounding box,
      and supports instances and the assembly tree.
   4. **Errors and memory.** OCCT signal handling on each thread, context on OCCT failures, memory
      pressure for new results and meshes, and malformed inputs as test cases.
   5. **Selectors with plain values,** with the lambda overloads kept.
   6. **Bytes in and out** for BREP, STEP and STL, through streams, with limits for untrusted input.
   7. **Cancellation and progress** (`CancellationToken`, `IProgress<double>`) and the missing measurements.
5. Builders (`build_common`, BuildLine, BuildSketch, BuildPart), including `Fork()`.
6. Import and export: SVG, DXF, 3MF and glTF (STEP, STL and BREP come in phase 4). glTF needs OCCT's
   `TKDEGLTF`, which builds only with RapidJSON, so add RapidJSON to the native superbuild first.
7. Advanced: joints, drafting, text, sheet metal, `brep_from_stl`, `uv_write`.
