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
