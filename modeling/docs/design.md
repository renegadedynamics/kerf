# build123d.NET design

A .NET port of [build123d](https://github.com/gumyr/build123d), the Python CAD library built on
OpenCascade (OCCT).

## Decisions

| Topic | Decision |
|---|---|
| Target framework | .NET 10 |
| Platforms | Windows, Linux and macOS (x64 and arm64) from the start |
| Naming | Idiomatic .NET (PascalCase, `Try*`, properties instead of getters). A Python → .NET name map is kept in `docs/name-map.md`. |
| OCCT | OCCT.NET, the managed port of OCCT 8.0.1 (LGPL 2.1 with OCCT's exception), the repository's `kernel` folder |
| Kernel calls | `Kerf.Modeling` calls the kernel's public API alone (unit 10M1; it called OCCT.NET's engine types directly from unit 10B). The native backend (a C ABI shim over native OCCT, with generated `SafeHandle` wrappers) and the generated forwarding layer that gave OCCT.NET its API went in unit 10B. See "The kernel" |

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

### 2. OCCT's names below; idiomatic API above them

`Build123d` calls OCCT.NET's classes under OCCT's names, so Python lines like `BRepAlgoAPI_Cut(a, b).Shape()` port
almost one-to-one. Only `Build123d` itself exposes idiomatic .NET names. `Shape.Wrapped` remains as an escape hatch to
the raw OCCT shape. Until unit 10B the calls went through `Build123d.Occt`, a binding with OCCT's API: native OCCT
through a generated C shim, or OCCT.NET through a generated forwarding layer (see "The kernel").

#### Geometry values

- **`Vector`, `Axis` and `Color` are `readonly struct`s.**
  - Plain arithmetic (add, dot, cross, normalise) runs in C#, in the same operation order as OCCT,
    so results are bit-identical.
  - Anything OCCT computes non-trivially goes through short-lived OCCT objects, so the numbers and
    the exceptions match build123d. This covers angles, axis tests, rotation, and sRGB ↔ linear
    colour conversion.
- **`Location`, `Plane`, `Matrix`, `BoundBox` and `OrientedBoundBox` are immutable classes**, each
  holding one OCCT value (OCCT.NET's structs, immutable themselves).
  - A `Location` keeps its `TopLoc_Location` for life. OCCT compares shape locations by identity, so
    shapes placed with the same `Location` instance share their location, as in build123d.
  - Public accessors (`ToGpPln()`, `ToGpGTrsf()`, …) return the values. `ToTopLoc()` returns the
    `TopLoc_Location`, which keeps the location's identity.
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
  collections are OCCT.NET's (`NCollection_List<TopoDS_Shape>` and friends, which OCCT names `TopTools_ListOfShape`…),
  maps of shapes made with `TopTools_ShapeMapHasher`, as OCCT's typedefs are.

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
- **The whole mesh is copied out in one call** (OCCT.NET's `MeshExtraction`, once the native shim's b3d_mesh.cpp,
  which it matches float for float): every face's nodes, normals and triangles and every edge's polyline go into flat
  buffers. A face's normals
  come from its surface (`BRepLib_ToolTriangulatedShape::ComputeNormals`). Its winding is flipped for a
  reversed face or a mirroring location. An edge's polyline is its polygon on an adjacent face's
  triangulation, so it lies exactly on the shading. The faces and edges of a `ShapeMesh` are read-only
  slices of those buffers. Copying out 100k triangles takes a few milliseconds; OCCT's meshing dominates.
- **`TriangulateTree`** follows `AssemblyChildren`. It gives a node per child with its label, colour and
  location relative to its parent, and meshes each `TShape` once in its own coordinates. Instances share
  the same `ShapeMesh` object.

#### Errors and memory

- **OCCT's failures are OCCT.NET's exceptions**, `Standard_Failure` and its subclasses (`Standard_DomainError`,
  `StdFail_NotDone`…, with `OcctType`), as OCCT.NET throws them (unit 10B), until results replace expected exceptions
  (unit 10F). The native backend's `OcctException`, with the operation being done ("box 0.0 × 1.0 × 1.0 failed: …",
  from `OcctContext.Doing` scopes), went with the forwarding layer that made it.
  - A boolean that fails reports OCCT's alerts (`BOPAlgo_Alert…`) instead of a bare `StdFail_NotDone`.
  - build123d's own messages (fillet, chamfer, offset) stand, with OCCT's exception inside.
- **No native faults, no memory pressure.** OCCT.NET is managed code: an access violation can't happen inside it, and
  the GC counts its memory. The native backend's fault translation and its `GC.AddMemoryPressure` on results went with
  it. The shim's estimate of a shape's memory, which nothing called, moved to the tests' `EngineShapes` in unit 10M1.
- **Untrusted BRep.** OCCT's reader trusts its input. At the end of truncated data it loops for ever,
  reusing the last word it read, and it reads words into fixed buffers with no length limit. So
  `ReadBRep` first refuses NULs and words of 200 characters or more, then reads through the kernel's `Brep.Read`, and
  reports any failure as a `KerfException` (`exchange.malformed`; unit 10M1, where it was OCCT's
  `Storage_StreamFormatError`).
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

#### The kernel

`Build123d` runs on OCCT.NET, the managed C# port of OCCT 8.0.1 (class for class, the repository's `kernel` folder),
with no native code, and calls its types directly (unit 10B).

- **Where OCCT.NET comes from.** The kernel's projects (`OcctNetRoot`, `../kernel/` by default), which pack as the
  `OcctNet.*` package dependencies (`kernel/tools/pack-nuget.ps1`). `Build123d` uses kernel members the published
  packages (0.1.0-preview.3) don't make public, so it no longer builds against them.
- **OCCT.NET's types as they are.** Shapes and `gp` values are structs: `TopoDS_Edge` converts to `TopoDS_Shape`
  rather than deriving from it, so a shape's kind is asked of it (`IsCompound()`, `TopoDS.Compound(shape)`;
  `Kernel/ShapeExtensions.cs`), and an absent shape is a nullable one or a null shape (`IsNull()`). Values are
  changed through their functional twins (`gp_Trsf.WithRotation`, `Bnd_Box.Added`); outputs are `out` and `ref`
  parameters. Nothing is disposed.
- **Building shapes.** OCCT.NET freezes every shape it hands out. Its builders (`TopoDS_Builder`, `BRep_Builder`'s face
  of a triangulation) build a shape that stays open until `TopoDS_TShape.FreezeGraph`, made public for this in unit
  10B, as they were reached before with `UnsafeAccessor`; `Build123d` mostly uses the builders' frozen forms
  (`TopoDS_Builder.MakeCompound`, `MakeShell`, `MakeSolid`).
- **Meshing** is the kernel's `Mesher`, which meshes in a working overlay and never writes a shape (decision 1349,
  question 59); a shape no longer carries a mesh, so `Meshed` went in 10M1. A mesh read (STL, glTF) keeps the shape read,
  faces holding only triangles (`ShapeMesh.TriangulatedShape`), which `ImportStl` returns as build123d's does.
- **The kernel's public API alone** (unit 10M1). `src/Kerf.Modeling` uses no engine type (`Kerf.Engine.*`, `Occt`): it
  calls the kernel's public operations (`Kerf.Construction`, `Kerf.Booleans`, `Kerf.Healing`, `Kerf.Measurement`,
  `Kerf.Exchange`, `Kerf.Meshing`, `Kerf.Geometry`…), and the kernel gives it no `InternalsVisibleTo`. Where an
  operation's public form differed from the engine call build123d makes, the kernel gained an option rather than the
  layer a difference (a section's pcurves, Booleans unsimplified, a transform's or revolution's copy, an edge on a
  surface without its 3D curve, the order of a wireframe fix). The layer's own glue (`Kernel/Extras.cs`, internal) is
  BRep text as bytes with the reader's checks, and an edge's own curve's planarity. The modelling tests keep
  `InternalsVisibleTo` only to check the layer against the engine itself (`StepAssemblyTests` against
  STEPCAFControl's document, `ShapeMeshTests` timing the mesh copy, `MemoryPressureTests` through `EngineShapes`).
- **Semantics kept.** `InputsUnchangedTests` passes, the concurrent cases included; tessellation matches what native
  OCCT gave float for float; the differential run against native OCCT's recorded log (OCCT.NET's unit 10A) shows no
  open difference.
- **Differences from native OCCT.** OCCT.NET never writes a caller's shape. Where OCCT adds pcurves to a profile it was
  given, OCCT.NET's result holds a copy instead, so the prism of a circular face doesn't share the face.
  `Solid.ExtrudeUntil` then finds the piece that holds the profile by place.
- **Packaging.** `Build123d` and OCCT.NET's four assemblies load and unload through a collectible
  `AssemblyLoadContext` with no native library (`OcctNetBackendTests` in the packaging tests, given the build output
  as `B3D_OCCTNET_LIB`).
- **History.** Until unit 10B, `Build123d` compiled against `Build123d.Occt`, a binding with OCCT's API, on either of
  two backends: native OCCT through a generated C shim (`b3docct`, from `tools/OcctGen`), the reference OCCT.NET's
  oracle compared against, or OCCT.NET through a generated forwarding layer (`Build123d.OcctNet`, from
  `tools/OcctNetGen`, with 3,300 lines written by hand), the default from OCCT.NET's phase 9. Both went in unit 10B:
  about 136,000 lines for native, 71,000 for the forwarding layer. Measured then: 1,876 of 1,881 tests passed on
  OCCT.NET and 5 native-only ones were skipped; the suite took 27 s against native's 5 s.

#### Import and export

build123d's `importers.py` and `exporters3d.py` are `Importers` and `Exporters3D`: `ImportBrep`, `ImportStep`,
`ImportStl` and `ExportBrep`, `ExportStep`, `ExportStl`, `ExportGltf`, with `Export3mf` for build123d's `Mesher` and
`ImportGltf`, which build123d doesn't have.

- **Streams first.** Each function reads or writes a `Stream`; the file forms open the file and call it. Bento does its
  own file access and hands over bytes (bento-integration.md, section 7). An export is made in memory and written
  only when whole, so a failure leaves no partial file.
- **Hostile input.** `ImportLimits` (bytes, STEP records, nesting depth, triangles, glTF nodes; generous defaults)
  is passed to the readers, which stop with an error rather than allocate past it. Data past a limit throws
  `InvalidDataException`; data that isn't the format throws `ArgumentException` (build123d's ValueError), for BREP a
  `KerfException` (`exchange.malformed`), and for STL and glTF an `InvalidDataException`.
- **Cancellation.** Every function takes a `CancellationToken`. It becomes a `Message_ProgressIndicator`, which OCCT's
  readers and writers poll as they run.
- **Through the kernel's `Kerf.Exchange`** (units 10L3 and 10M1): `Brep`, `Step`, `Stl`, `Gltf`, `ThreeMf` over its
  `ModelNode` tree (name, colour, shape or mesh or children, placement). The mesh formats are the engine's: `RWStl` and
  `StlAPI_Writer` (ported), and `RWGltf_TreeWriter`, `RW3mf_TreeWriter` and `RWGltf_TreeReader` (written fresh over its
  assembly tree). The native backend had neither them nor XCAF, which were OCCT.NET's only until it went (unit 10B).
- **STEP's names, colours and assemblies.** build123d writes and reads STEP through XCAF (`STEPCAFControl`);
  `Build123d` does the same through OCCT.NET's STEPCAFControl over its assembly document (`XdeDoc_Document`, frozen
  into an `XdeDoc_Tree`), reached through `Kerf.Exchange.Step` on `ModelNode`s.
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
  - The FILE_NAME header is build123d's: the shape's label, the time stamp given (for repeatable output) and
    "build123d" as the originating system. The record is rewritten in the text, as it was when native OCCT's header
    maker wasn't bound.
- **glTF and 3MF keep the tree.** An assembly is a node (3MF: a components object) per child, named by its label,
  with a material per colour. A part is meshed on a private copy with build123d's deflections (relative to each
  edge), as `Shape.mesh` does. glTF turns Z-up into Y-up and the unit into metres, as build123d's does (by the
  writer's coordinate change rather than by moving the shape), and a `.gltf` written to one stream embeds its buffer
  as a `data:` URI. `ImportGltf` gives a part per mesh, a compound of faces holding only triangles, with its name and
  colour, and an assembly per node hierarchy.
- **Compared with native** (`StepNativeTests`, Reference/Exchange): ten shapes built natively and kept as BRep (solids
  of each surface type, a fillet, a loft, a face, a wire, a compound). Their STEP is byte for byte native OCCT's
  STEPControl's (with the time stamp fixed and the product name's index masked, which native counts
  process-wide), but for the compound, whose STEPCAFControl orders the compound's representations differently; the
  STEP read back gives native's BRep text exactly. The references were written natively and are kept.
- **Compared with OCCT.NET's STEPCAFControl** (`StepAssemblyTests`): the ten shapes and an assembly
  (nested, placed, rotated, a part placed twice, names, colours, a translucent one) are written as OCCT.NET's
  STEPCAFControl_Writer writes a document built with XCAF's calls (`AddShape`, `NewShape`, `AddComponent`,
  `SetName`, `SetColor`), which OCCT.NET's unit 8G2 compared with native XCAF; the assembly reads back with its
  labels, colours, nesting, places and shared part; an assembly written as three files reads from its file and is
  refused from a stream.
- **Faces holding only triangles** (from STL and glTF) have no surface: `BoundingBox()`, which otherwise ignores
  triangulations, bounds their triangles; `Tessellate` and `Triangulate` give them.

#### Left by unit 10M1

Unit 10M1 moved the layer onto the kernel's public API (items 1 and 2 of its brief), retired the emulation of Python's
`repr` and format specs (`PyFormat`: numbers are .NET's invariant text, formats .NET's), and moved the readers onto
`Kerf.Exchange`. Left, for 10M2 or later:

- **`PySet`, `PyComparer` and `PyMath` stay.** Each decides a result, not only its text: `PySet.Distinct` orders the
  points `Shape1D` fits a plane to as a CPython set iterates them, and the plane taken on a tie depends on that order;
  `PyComparer` sorts and groups mixed keys (numbers, tuples, strings, null) as Python compares them; `PyMath` is
  CPython's float modulo and scipy's bounded Brent and Nelder–Mead minimizers, which build123d calls. Replacing them
  with `HashSet`, `Comparer<T>.Default` and a kernel minimizer would change results the Python-reference tests pin.
  They want a decision on whether the Python reference or .NET's own behaviour defines those results.
- **Python's operators and messages.** The operators (`|`, `&`, `^`, `<<`, `>>`, `@`, `%` on shapes, lists and
  vectors) and build123d's messages are the API's shape, which 10M2 redesigns; 10M1 changed only the numbers in the
  messages (".0" no longer printed: "radius of 5", not "radius of 5.0").
- **Helpers that duplicate kernel operations.** `TopoUtil`'s wrappers (`BoolOp`, `Boolean`, `SewFaces`, `Explore`,
  `Unique`) remain as the layer's adapters onto the kernel's operations, keeping the layer's defaults (unsimplified
  Booleans, the removal semantics of its histories); 10M2's API shape decides whether they go.

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
6. Import and export: SVG, DXF, 3MF and glTF (STEP, STL and BREP come in phase 4; STL, glTF and 3MF are OCCT.NET's,
   see "Import and export").
7. Advanced: joints, drafting, text, sheet metal, `brep_from_stl`, `uv_write`.
