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

### 3. Behavioural parity is tested, not assumed

- The pytest suite is ported module by module, alongside the code it tests.
- A differential harness runs the Python examples and their .NET ports, then compares volume, area,
  bounding box and topology counts within a tolerance.

## Phases

0. **Spike.** Build OCCT 8.0 on all platforms; generate bindings for `gp`, `BRepPrimAPI`, `BRepAlgoAPI`
   and `STEPControl`; make a box minus a cylinder and export it to STEP from C#, in CI.
1. `geometry.py`: Vector, Axis, Plane, Location, BoundBox, Color, Matrix.
2. Topology core: Shape, ShapeList, Vertex, Edge, Wire, Face, Shell, Solid, Compound.
3. Objects and operations in algebra mode.
4. Builders (`build_common`, BuildLine, BuildSketch, BuildPart), including `Fork()`.
5. Import and export: STEP, STL and BREP first, then SVG, DXF, 3MF and glTF. glTF needs OCCT's
   `TKDEGLTF`, which builds only with RapidJSON, so add RapidJSON to the native superbuild first.
6. Advanced: joints, drafting, text, sheet metal, `brep_from_stl`, `uv_write`, constrained lines.
