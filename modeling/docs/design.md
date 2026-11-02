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
| Managed backend (the default) | `Build123d` builds on OCCT.NET, the managed port, through a second implementation of `Build123d.Occt`'s public API (`src/Build123d.OcctNet`) that forwards to OCCT.NET and is generated. Native is opt-in with `-p:Build123dBackend=Native`, as the oracle's reference, and is no longer published. See "The OCCT.NET backend" |

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
  - An object that walks another without holding it keeps that one's wrapper alive. OCCT 8's `TopoDS_Iterator`
    holds an iterator into the shape's child list, not the shape, so once the shape's wrapper was unreachable its
    finaliser could free the shape mid-iteration (found by OCCT.NET's oracle under GC pressure). Its wrapper keeps the
    wrapper of the shape it was last given (`KeepShapeAlive` in `CsEmitter`).
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
- **Collections.** `ShapeList<T>` is an immutable list over Schematiq.Collections' `ISchematiqList<T>`. OCCT's
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

#### Selectors

- **Plain values throughout.** Every selector has an overload that takes values rather than a delegate:
  axes, planes, enums and numbers, as before. build123d's `property` arguments are taken by name:
  `FilterBy("IsClosed")`, `FilterByValue("Radius", 5)`, `SortBy("Length")` and `GroupBy("Area")`.
  - A name matches a public property of the shape's own class, or a method whose parameters are all
    optional. Case and underscores are ignored, so build123d's `is_closed` works.
  - Lookups are cached per class and name.
  - `Shape` carries `DynamicallyAccessedMembers`, so trimming keeps what a name can reach.
  - An enum compared by `FilterByValue` may be given by its name.
- **`Selector`** is a selection as a value: faces, edges or vertices, then steps.
  - The steps are `FilterByGeomType`, `FilterByAxis`, `GroupByAxis(axis, index)` and
    `GroupByMeasure(measure, index)`, where a negative index counts from the end.
  - It applies to any shape, prints as C# or Python, and compares by value.
- **`Selector.For(shape, picked)`** finds selectors for a face, edge or vertex picked in a viewer.
  - It does a breadth-first search over sets holding the pick, at most three steps deep. Each step's
    effect comes from keys measured once per candidate, and only steps that keep the pick are tried.
  - Fewest steps come first, then the most robust: end groups along the model's axes, then filters,
    then the smallest or largest of a measure, then inner groups.
  - A selector that holds a shorter one is dropped.
  - Every result is applied to the shape before it is returned, so the search's model of the
    selectors can't give a wrong answer. It takes milliseconds for a part of a few hundred faces.

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

#### The OCCT.NET backend

`Build123d` running on OCCT.NET, the managed C# port of OCCT 8 (class for class), with no native code. A spike
until OCCT.NET's phase 9, which made it the default backend.

- **The switch.** `Build123dBackend` (`OcctNet` by default, or `Native`) chooses which project `Build123d`
  references: `src/Build123d.OcctNet` or `src/Build123d.Occt` (the shim). `OCCTNET` is defined for the tests' few
  backend-specific cases.
- **Where OCCT.NET comes from.** By default the `OcctNet.*` packages on nuget.org, at `OcctNetVersion`
  (`Directory.Build.props`), so a fresh clone builds and tests with a plain `dotnet test`. With `OcctNetRoot` given,
  `Build123d.OcctNet` references that OCCT.NET checkout's projects instead: for working on both together, and for
  OCCT.NET's `tools/pack-nuget.ps1`, which packs `Build123d.OcctNet` and `Build123d` that way (the project references
  become the same package dependencies).
- **Native** (`-p:Build123dBackend=Native`) stays buildable and tested, as the reference: OCCT.NET's oracle tests
  reference `Build123d` and `Build123d.Occt` with `Build123dBackend=Native` on the project reference, whatever the
  default (and restore by the project graph, as NuGet's default walk would restore them on the default backend).
  `Build123dNativeRoot` (this checkout by default) says whose `.deps/install` and `native/out` it copies, so a
  worktree can use another checkout's native build. It isn't published: `Build123d.Occt` and native `Build123d` are
  packable only through `tools/pack.ps1` (which passes the staged natives, `NativeStageDir`), into the local feed,
  versioned `0.1.0-native`, for the load-context test.
- **Output folders.** Each backend builds into folders of its own, so the two sit side by side: the default into
  `bin` and `obj`, native into `bin-native` and `obj-native`. (Before the switch, OCCT.NET built into `bin-occtnet` and
  `obj-occtnet`; those are still excluded from the projects' items, in case an old checkout has them.)
- **Tests on one backend only.** `Build123d.Occt.Tests`, the binding's tests, is a test project on the native backend
  only (`IsTestProject` false otherwise, so `dotnet test` skips it). In `Build123d.Tests`, `NativeFact` skips the
  shim's tests under OCCT.NET.
  The packaging tests skip without their inputs: the native packages (`B3D_PACKAGES`, set by `tools/pack.ps1`) or
  OCCT.NET's build output (`B3D_OCCTNET_LIB`).
- **Why a second `Build123d.Occt`, rather than `Build123d` compiled against `Occt` directly.** OCCT.NET's API differs
  from OCCT's in ways `Build123d` relies on:
  - its shapes and `gp` types are structs, so `TopoDS_Edge` isn't a `TopoDS_Shape` and can't be null or
    pattern-matched;
  - its builders and in-place forms are internal, because it freezes every shape it hands out;
  - its exceptions are `Standard_Failure`s, not `OcctException`s.

  Adapting `Build123d` would touch most of its 18,000 lines and change its semantics. Instead
  `src/Build123d.OcctNet` has the same namespace, types and members as `Build123d.Occt`, so `Build123d` compiles
  unchanged against either.
- **Generated.** `tools/OcctNetGen` reads both assemblies by reflection and writes a wrapper per class.
  - A wrapper holds its OCCT.NET object: by reference for a class, or as a field of its own for a struct, as the
    native wrapper holds a heap copy.
  - Each member is matched to its OCCT.NET counterpart (overloads, optional parameters, `in`/`out`/`ref`, implicit
    shape conversions) and forwards with conversions.
  - Results are wrapped by the native signature's type, as the native binding's are. A collection OCCT returns by
    reference is copied.
  - Where OCCT.NET has only a functional twin of an in-place form on a struct (`gp_Trsf.SetRotation` as
    `WithRotation`, `Bnd_Box.Add` as `Added`), the wrapper's struct is replaced by the twin's result.
  - `bindings/occtnet.unforwarded.txt` lists what can't be forwarded, and why. `bindings/occtnet.handwritten.txt`
    lists what `src/Build123d.OcctNet/Runtime` writes by hand.
- **Errors.** Every forwarding call turns OCCT.NET's `Standard_Failure` into an `OcctException` with the same
  `OcctType` (`StdFail_NotDone`…) and `OcctContext` operation, with OCCT.NET's exception as the inner one. Other
  exceptions, such as an OCCT.NET bug's `NullReferenceException`, pass through unchanged, so they're seen.
- **What isn't ported** throws `NotSupportedException`, with no native fallback:
  - a class OCCT.NET doesn't have yet says "not yet ported to OCCT.NET: …". Since OCCT.NET's phase 8 (STEP, forwarded
    from `OcctNet.DataExchange`) there is none;
  - a member with no public counterpart says "not available through OCCT.NET's public API: …", such as the copy
    constructors of the phase 7 algorithms (`BRepFilletAPI_MakeFillet`, `BRepOffset_MakeOffset`…) and of
    `STEPControl_Reader` and `Writer`, which OCCT.NET doesn't offer.
- **Written by hand** (`Runtime`):
  - the runtime classes;
  - `Extras`, over OCCT.NET's `MeshExtraction`, BRep text I/O and alerts;
  - a port of `Quantity_Color`, which OCCT.NET doesn't have;
  - `TopoDS_Builder`: a shape it makes stays open while it's built, and is frozen the first time anything else reads
    it, so OCCT.NET's frozen-shape guarantee holds for every shape `Build123d` passes on;
  - `BRepMesh_IncrementalMesh`: OCCT.NET meshes a copy and returns it, so the wrapper it's given takes the meshed
    shape, as the native one would carry the mesh (`Build123d` meshes only private copies);
  - `BRepLib.BuildCurves3d`, over its functional twin, the same way;
  - the shape's in-place forms (`Location(loc)`, `Reverse()`…), on the wrapper's own copy;
  - data exchange (`Runtime/Exchange.cs`): the STEP reader's and writer's stream forms, with OCCT.NET's limits and
    cancellation, and the mesh formats (see "Import and export").
- **OCCT.NET internals.** Four things have no public way in OCCT.NET yet, so they're reached with `UnsafeAccessor`
  (no reflection). They're listed for OCCT.NET to make public:
  - building shells and solids from parts: `TopoDS_Builder`'s internal constructor and its `Make*` and `Add`;
  - freezing the result: `FreezeGraph`;
  - copying a `BRepTools_History` as it is;
  - a face holding only a triangulation (STL import): the internal `BRep_Builder`'s `MakeFace(face, triangulation)`,
    named with `UnsafeAccessorType`, as the class itself is internal.
- **Semantics kept.**
  - Immutability: `InputsUnchangedTests` passes, the concurrent cases included.
  - Exception types and messages, and thread safety.
  - Tessellation (`Triangulate`, `ShapeMesh`), which matches native float for float.
  - Native memory pressure doesn't apply: OCCT.NET's memory is managed, so the GC already counts it.
- **Differences from native.**
  - OCCT.NET never writes a caller's shape. Where OCCT adds pcurves to a profile it was given, OCCT.NET's result
    holds a copy instead, so the prism of a circular face doesn't share the face. `Solid.ExtrudeUntil` then finds the
    piece that holds the profile by place.
  - The tests that depend on the native shim (faults, memory pressure) are `NativeFact`s, skipped under OCCT.NET.
- **Packaging.** No native library is needed. `Build123d`, `Build123d.OcctNet` and OCCT.NET's four assemblies load
  and unload through a collectible `AssemblyLoadContext`: `OcctNetBackendTests` in the packaging tests, given the
  build output as `B3D_OCCTNET_LIB`.
- **Measured** (OCCT.NET at be3b97db, with its phase 8 through STEPCAFControl; Windows x64):
  - 1,876 of 1,881 tests pass and 5 are skipped. `Fillet2DTests.FilletRecord` failed until OCCT.NET's phase 9: the 2D
    fillet recorded each edge MakeWire returned as it went, and when the wire closed, OCCT.NET's MakeWire enlarged the
    first vertex's tolerance in a copy where native does it in place, so the first vertex had two modified shapes. The
    fillet now records the finished wire's edges (OCCT.NET's unit 9F2; the same shapes natively). Natively 1,851 of 1,858 pass and 7 are skipped (the mesh formats' tests among them). At 140ceb00
    (STEPControl), 1,856 of 1,862 passed; at 991bbe6, with
    phase 7, 1,813 of 1,819 passed; at bad099b, before it, 1,676 passed and 130 failed, 128 of them on phase 7;
  - the 37 introductory examples (all but 34, text) match native's volume to a part in 10⁹ and its face count
    (`IntroductoryExamplesTests`), the seven with fillets, chamfers, offsets and extrusion until among them;
  - the suite takes 27 s against native's 5 s, 7.6 times summed over the tests both pass.

#### Import and export

build123d's `importers.py` and `exporters3d.py` are `Importers` and `Exporters3D`: `ImportBrep`, `ImportStep`,
`ImportStl` and `ExportBrep`, `ExportStep`, `ExportStl`, `ExportGltf`, with `Export3mf` for build123d's `Mesher` and
`ImportGltf`, which build123d doesn't have.

- **Streams first.** Each function reads or writes a `Stream`; the file forms open the file and call it. Bento does its
  own file access and hands over bytes (bento-integration.md, section 7). An export is made in memory and written
  only when whole, so a failure leaves no partial file.
- **Hostile input.** `ImportLimits` (bytes, STEP records, nesting depth, triangles, glTF nodes; generous defaults)
  is passed to the readers, which stop with an error rather than allocate past it. Data past a limit throws
  `InvalidDataException`; data that isn't the format throws `ArgumentException` (build123d's ValueError), or for
  BREP `OcctException` (`Storage_StreamFormatError`, as before).
- **Cancellation.** Every function takes a `CancellationToken`. On OCCT.NET it becomes a `Message_ProgressIndicator`,
  which OCCT's readers and writers poll as they run; natively it is checked between OCCT's calls.
- **Through the backend's API.** `Build123d` reaches the readers and writers through members both backends have:
  - STEP through `STEPControl_Reader` and `Writer`, with hand-written stream forms (`ReadStream`, `WriteStream`,
    `TransferRoots(token)`, `Transfer(shape, mode, StepWriteSettings, token)`). On OCCT.NET they forward to its stream
    forms with `StepFile_ReadLimits` and a progress range. Natively OCCT's stream forms (`std::istream`) aren't bound,
    so they go through a temporary file: only the byte limit applies, and only OCCT's default writing parameters
    (millimetres, pcurves, average precision). Other units, `writePcurves: false` and other precision modes need
    `Interface_Static`, which isn't bound, and throw `NotSupportedException`;
  - the mesh formats through `Extras` (`WriteStl`, `ReadStl`, `WriteGltf`, `Write3mf`, `ReadGltf`) and
    `ExchangeNode`, a backend-neutral assembly tree (name, colour, shape or children and location).
- **The mesh formats need the OCCT.NET backend** (a decision): natively they throw `NotSupportedException`.
  - Native OCCT's STL (TKDESTL) and mesh writers (TKRWMesh) would need new bindings and shim code, and its glTF
    writer RapidJSON in the superbuild (phase 6), for formats OCCT.NET already has: `RWStl` and `StlAPI_Writer`
    (ported), and `RWGltf_TreeWriter`, `RW3mf_TreeWriter` and `RWGltf_TreeReader` (written fresh over its assembly
    tree, `XdeDoc_Tree`).
  - Writing them in `Build123d` itself would have duplicated OCCT.NET's writers.
  - Bento runs on OCCT.NET, so it has every format.
- **STEP's names, colours and assemblies on the OCCT.NET backend.** build123d writes and reads STEP through XCAF
  (`STEPCAFControl`); the OCCT.NET backend does the same through OCCT.NET's STEPCAFControl over its assembly
  document (`XdeDoc_Document`, frozen into an `XdeDoc_Tree`), reached through `Extras.ReadStep` and `WriteStep` on
  `ExchangeNode`s (`Extras.StepKeepsAssemblies` says which backend has them).
  - `ExportStep` writes a compound with `AssemblyChildren` as an assembly, each child a component at its location, a
    product per part and assembly named by its label and coloured by its colour (a surface colour, or a curve colour
    for a shape without faces). Children of one shape, label and colour share a product, placed once per child, as
    XCAF's `AddShape` shares a compound's equal shapes; a child of the same shape under another label or colour gets
    a copy of the shape, since STEP's writer maps shapes to products and would write the two as one, with the last
    name. Any other shape is one part, even a compound: build123d's
    `Part` is a compound of a solid, which XCAF's `AddShape(shape, makeAssembly: true)` would make an assembly of
    one solid (a decision). The shapes are written unprocessed, as OCCT's STEPCAFControl writes them (OCCT.NET's
    decision 1277).
  - `ImportStep` gives build123d's `import_step` tree: the free shapes (one returned as it is, several in a
    compound), each assembly a compound whose `AssemblyChildren` are its components placed by their locations, each
    part its shape at its place; labels are the products' names (STEP's instance names are the occurrences'
    identifiers), colours the shape's own or its product's (surface, else generic, else curve). An unnamed product
    keeps the name OCCT wrote for it ("Open CASCADE STEP translator 8.0 1"), as in build123d.
  - **External references.** OCCT reads the other files a STEP file refers to from the disk while transferring.
    The file form reads them from the file's folder, as OCCT does; the stream form refuses such data
    (`InvalidDataException`), since a stream has no folder and a hostile file could name any path.
  - **Natively** OCCT's XCAF documents and STEPCAFControl aren't bound (a decision: binding them means new toolkits,
    TKXCAF, TKLCAF and TKCAF, and OCAF's document framework in the shim, for what Bento, the user that needs them,
    gets on OCCT.NET). Native STEP stays shapes alone, through `STEPControl`: an assembly is written as its compound,
    and a file's shapes come back as one shape or a compound of them, without labels.
  - The FILE_NAME header is build123d's on both backends: the shape's label, the time stamp given (for repeatable
    output) and "build123d" as the originating system. Native OCCT's header maker (`APIHeaderSection_MakeHeader`)
    isn't bound, so the record is rewritten in the text, the same way on both.
- **glTF and 3MF keep the tree.** An assembly is a node (3MF: a components object) per child, named by its label,
  with a material per colour. A part is meshed on a private copy with build123d's deflections (relative to each
  edge), as `Shape.mesh` does. glTF turns Z-up into Y-up and the unit into metres, as build123d's does (by the
  writer's coordinate change rather than by moving the shape), and a `.gltf` written to one stream embeds its buffer
  as a `data:` URI. `ImportGltf` gives a part per mesh, a compound of faces holding only triangles, with its name and
  colour, and an assembly per node hierarchy.
- **Compared with native** (`StepNativeTests`, Reference/Exchange): ten shapes built natively and kept as BRep (solids
  of each surface type, a fillet, a loft, a face, a wire, a compound). Their STEP is byte for byte native OCCT's
  STEPControl's (with the time stamp fixed and the product name's index masked, which native counts
  process-wide) on both backends, but for the compound on OCCT.NET, whose STEPCAFControl orders the compound's
  representations differently; the STEP read back gives native's BRep text exactly on both.
- **Compared with OCCT.NET's STEPCAFControl** (`StepAssemblyTests`, OCCT.NET backend): the ten shapes and an assembly
  (nested, placed, rotated, a part placed twice, names, colours, a translucent one) are written as OCCT.NET's
  STEPCAFControl_Writer writes a document built with XCAF's calls (`AddShape`, `NewShape`, `AddComponent`,
  `SetName`, `SetColor`), which OCCT.NET's unit 8G2 compared with native XCAF; the assembly reads back with its
  labels, colours, nesting, places and shared part; an assembly written as three files reads from its file and is
  refused from a stream.
- **Faces holding only triangles** (from STL and glTF) have no surface: `BoundingBox()`, which otherwise ignores
  triangulations, bounds their triangles; `Tessellate` and `Triangulate` give them.

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
6. Import and export: SVG, DXF, 3MF and glTF (STEP, STL and BREP come in phase 4; STL, glTF and 3MF are on the
   OCCT.NET backend already, see "Import and export"). Natively, glTF needs OCCT's
   `TKDEGLTF`, which builds only with RapidJSON, so add RapidJSON to the native superbuild first.
7. Advanced: joints, drafting, text, sheet metal, `brep_from_stl`, `uv_write`.
