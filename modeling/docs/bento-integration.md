# Using build123d.NET from Bento: notes and requirements

Bento is a .NET spreadsheet (repository `C:\Users\jpdca\source\bento`) built on the same immutable collections library as
this port. A Bento extension, **3D models** (`Extensions/Bento.Cad` there), will expose build123d.NET as cell functions.
A design is then built up one stage per cell, and the Viewer shows the model in any cell with three-cad-viewer, the
viewer behind the OCP CAD Viewer. This document is what Bento needs from the library. Fold it into your plan where it
fits. Items are marked **Must**, **Should** or **Nice**.

The Bento side already exists and is tested: models as cell values, the viewer running in WebView2, and the scene JSON
three-cad-viewer reads (its "version 3" format, as ocp-tessellate writes it). **Bento owns everything about the viewer
and its formats.** This library should not know about Bento, three-cad-viewer or its JSON. It needs to provide plain
geometry, mostly a good tessellation API (section 3).

> **Since this was written (November 2026):** native OCCT and the forwarding layer are gone (unit 10B, decisions 1341 to
> 1348). The modelling layer is managed code over the kernel's assemblies, so everything below about native DLLs, their
> loading, native faults, `OcctException`, `OcctContext` and memory pressure is history: there is no native folder to
> load, faults are .NET exceptions, failures reach callers as the kernel's exceptions (`FailureException` and its
> subclasses; `OcctType` gives OCCT's name) until unit 10F's results replace them, and the GC sees all memory. The
> project is now **Kerf** (unit 10E): the modelling layer is `Kerf.Modeling` (namespace `Kerf.Modeling`) over
> `Kerf.Kernel.*`, and Bento moves to those packages in unit 10N. The requests and replies below are kept as they were
> made, under the names of the time.

## 1. How Bento will use the library

- Formulas use **algebra mode only**. A cell holds one immutable shape, and the next cell transforms it:

  ```
  A1  =B3D_BOX(80, 60, 10)
  A2  =B3D_CUT(A1, B3D_MOVE(B3D_CYLINDER(5, 10), B3D_LOC(20, 0, 0)))
  A3  =B3D_FILLET(A2, A2.Edges.GroupBy("Z").Last, 2)
  ```

  Builders (`BuildPart` and friends) won't be used from cells.
- Shapes are shared between cells, kept in Bento's undo history, and compared between recalculations. The immutability
  you have chosen is exactly what Bento needs: please keep it absolute in the public API.
- Bento calculates cells on **background threads**, and may calculate several at once.
- The library runs inside a Bento extension: a **collectible `AssemblyLoadContext`** of its own, loaded from a package
  folder, not the app's folder.
- Faces and edges are chosen with **selectors** (topmost face, edges parallel to Z, and so on), never by index, so models
  survive upstream parameter changes. Picking in the viewer will offer a selector formula for the face or edge picked.
- Bento's quantities (`10 mm`) are converted to plain doubles before calling you. **Lengths are in millimetres**, as
  build123d's convention. Return plain doubles too, and Bento puts the units back.

## 2. Platform, packaging and loading

**Must**
- **Target `net10.0`.** Bento now targets .NET 10 (since 28 October 2026).
- **No Schematiq.Collections.** Kerf depends on it no more (question 50, unit 10P1): `ShapeList<T>` and the kernel's
  lists are standard .NET read-only types, so Bento's extension shares nothing with the host here, and Bento converts
  only where it needs a modified collection.
- **Native loading must work from a non-default load context and folder.**
  - Bento's extension load context resolves `[LibraryImport("b3docct")]` itself, by looking in three places: the managed
    assemblies' folder (`lib/net10.0`), that folder's `runtimes/win-x64/native`, and `runtimes/win-x64/native` at the
    package root.
  - So keep using plain library names in `LibraryImport`/`DllImport`.
  - Do not call `NativeLibrary.SetDllImportResolver`, which is process-wide and would fight the host's resolver.
  - Do not use absolute paths or `AppContext.BaseDirectory`. Those point at Bento's folder, not the extension's.
- **Keep every native DLL in one flat folder.** That covers `b3docct.dll`, the OCCT `TK*.dll`s, FreeType and anything
  else. .NET loads `b3docct` by full path, so Windows finds its dependencies beside it. Please add a test that loads the
  library through a fresh collectible `AssemblyLoadContext` from a folder that isn't the test's own, makes a box, and
  tessellates it. That's the cheapest way to catch packaging mistakes before Bento does.
- **Windows x64 first.** Bento is Windows x64 only today, so the other platforms can follow.

**Should**
- **Ship as NuGet packages to a local folder feed.** For example `Build123d` and `Kerf.Modeling.Occt`, with natives under
  `runtimes/win-x64/native`. Bento's extension will reference them like any package. Follow SemVer and keep a short
  changelog.
- **Keep the native set small.** Bento will install the extension separately because of its size, but every MB still
  counts. Leave out OCCT toolkits that build123d doesn't use, such as visualisation (TKV3d, TKOpenGl) and Draw.
- **Provide the third-party notices** (`THIRD-PARTY-NOTICES.md`) for everything native that is shipped: OCCT (LGPL 2.1
  with exception, dynamically linked), FreeType, and RapidJSON if glTF goes in. Bento's package will carry them.

## 3. Tessellation: the main thing Bento needs

`Shape.Tessellate` today returns all faces' vertices and triangles in one list. Bento needs more structure, so a pick in
the viewer can be traced back to a face or edge, and the viewer can draw edges and shade correctly.

**Must**: a tessellation result per shape, along these lines (names are yours to choose):

```csharp
ShapeMesh Shape.Triangulate(double linearDeflection, double angularDeflection, bool relative = false)

sealed class ShapeMesh            // immutable
{
    IReadOnlyList<FaceMesh> Faces;    // in the same order as shape.Faces()
    IReadOnlyList<EdgeMesh> Edges;    // in the same order as shape.Edges(): each topological edge once
    IReadOnlyList<Vector>   Vertices; // in the same order as shape.Vertices()
}

sealed class FaceMesh
{
    float[] Positions;   // x, y, z per node, located (Location applied)
    float[] Normals;     // per node, from the surface (BRepLib_ToolTriangulatedShape / Poly normals), not per triangle
    int[]   Triangles;   // index triples into this face's nodes, wound so normals face out (REVERSED faces flipped)
    GeomType Type;       // the surface's GeomAbs type: plane, cylinder, cone, sphere, torus, bspline, ...
}

sealed class EdgeMesh
{
    float[]  Points;     // the polyline, x, y, z per point, located; from the face triangulation's polygon on
                         // triangulation where there is one, so edges sit exactly on the shaded surface
    GeomType Type;       // the curve's GeomAbs type: line, circle, ellipse, bspline, ...
}
```

Requirements behind that sketch:
- **Stable order.** Face *i* of the mesh is face *i* of `shape.Faces()`, and the same for edges and vertices, so an index
  from a pick maps straight back to a `Face`, `Edge` or `Vertex` object, and from there to a selector.
- **Shared nodes within a face**, so normals are smooth: no triangle soup. Nodes don't need sharing across faces.
- **Correct orientation:** reversed faces flipped, and normals pointing out of solids.
- **Fast, done in bulk.** Today's loop makes several P/Invoke calls per node and per triangle. A large part will have
  hundreds of thousands of nodes, and Bento re-tessellates on recalculation. Please add a shim function that copies a
  face's whole triangulation (nodes, normals, triangles) into caller-provided buffers in one call, and the same for edge
  polylines. As a target, 100k triangles should come out in well under 100 ms after meshing.
- **Thread-safe.** `BRepMesh_IncrementalMesh` writes the triangulation into the shared `TShape`, so two cells
  tessellating shapes derived from the same base can race. The library should serialise meshing itself (a lock keyed by
  `TShape`, or a global one) rather than leave that to callers. Callers can't see which shapes share geometry.
- **Deterministic:** the same shape and tolerances give the same output, which Bento's tests rely on.
- **Empty shapes** give an empty mesh rather than throwing. A cell can legitimately hold an empty result.

**Should**
- **Relative deflection:** a helper giving sensible default tolerances from the bounding box, as ocp-tessellate does,
  so callers needn't pick numbers.
- **Instances:** a compound that repeats one `TShape` at different `Location`s (bolts in an assembly) should report each
  geometry once plus a list of locations. three-cad-viewer supports instancing, and it saves memory and time.
- **Assemblies:** walking a `Compound` or assembly tree should give each child's name (`Label`), `Color`, alpha and
  `Location`. Bento turns that into the viewer's parts tree, so it needs an immutable tree it can traverse with labels and
  colours intact.

## 4. Errors, cancellation, threads and memory

**Must**
- **Clear exceptions.** Bento shows the message in the cell (`#VALUE!` with the message beneath).
  - Argument problems (a negative radius, an empty selection) should be `ArgumentException` or
    `ArgumentOutOfRangeException`.
  - Geometry failures should be `OcctException` with the OCCT type, plus what was being done, e.g. "Fillet failed on 3
    edges: the radius 5 is too large for edge 7 (length 4.2)". A bare "StdFail_NotDone" isn't enough for someone
    looking at a cell.
- **No native crashes.** An access violation inside OCCT ends Bento, taking the user's unsaved work with it, which is
  far worse than a failed cell. Please:
  - turn on OCCT's signal handling (`OSD::SetSignal`) so its faults become `Standard_Failure`;
  - make the shim catch everything it can (including SEH on Windows, if the shim is built with `/EHa`) and report it
    through the existing error out-parameter;
  - treat malformed inputs (degenerate geometry, broken STEP files) as test cases.
- **Thread safety.** Each public operation must be safe to call from any thread, and concurrently on shapes that share
  geometry. Document anything that isn't, and lock inside the library where OCCT needs it (meshing, as above, and any
  other cache OCCT writes into a shared `TShape`).
- **Memory pressure.** A spreadsheet keeps many intermediate shapes alive, in cells and in undo history, and drops them
  silently. Disposal is never guaranteed, so finalisers have to be correct. Please call `GC.AddMemoryPressure` (and
  `RemoveMemoryPressure`) with an estimate for wrappers that own significant native memory, such as shapes and
  triangulations, so the GC collects before native memory balloons.

**Should**
- **Cancellation and progress** for the long operations: booleans, fillets and chamfers, offsets, lofts, meshing, and
  STEP import and export. Accept a `CancellationToken` (wired to `Message_ProgressIndicator::UserBreak`) and optionally an
  `IProgress<double>`. Bento stops a cell's calculation when an input changes, and should be able to stop OCCT too.
  Since unit 11T: `Kerf.Cancellation.Use(token)` round a cell's calculation ([11t-bento.md](11t-bento.md)).
- **Cheap identity:** a fast way to tell whether two shape objects are the same geometry (`IsSame`/`IsEqual`, plus a hash
  consistent with it). Bento uses this to skip recalculating cells whose input didn't really change, and to cache meshes.

## 5. Selectors from a formula

**Must**
- **Every selector usable without lambdas:** `Faces()`, `Edges()`, `Filter(Axis)`, `Filter(GeomType)`, `SortBy(Axis)`,
  `GroupBy(Axis)`, `First`/`Last`, `Select.Last`/`New` and so on, taking plain values: axes, enums, numbers and strings
  where build123d accepts strings. A spreadsheet formula can pass a value but not easily a C# delegate. Keep the
  lambda overloads too.
- **Selections are `ShapeList<T>` (immutable),** so Bento shows them as lists in the Viewer.

**Nice**
- **A selector for a picked face or edge.** Given a shape and one of its faces or edges, return a few selector
  expressions (or structured selector descriptions) that pick out exactly that face or edge, preferring the simplest
  and most robust ("the face with the largest Z", "edges parallel to Z in the top group"). Bento will offer one of them
  when the user clicks a face in the viewer. Bento can do this itself from the public API, so only take it on if it
  falls out naturally.

## 6. Measuring

three-cad-viewer measures on the mesh by default, which is approximate. Later, Bento will answer the viewer's measure
requests exactly. **Should**: public functions for, between any two shapes or sub-shapes:
- **Distance:** the minimum distance and the two closest points (`BRepExtrema_DistShapeShape`);
- **Angle:** between planar faces, linear edges, or a face and an edge;
- **Properties of one shape:** length, area, volume, centre of mass, bounding box; and the radius and centre of a
  circular edge or cylindrical face.

Most of these exist in build123d. Please make sure they're public and return plain values.

## 7. Files and bytes

**Must**
- **Import and export through streams or byte arrays, not only file paths:** BREP, STEP and STL first, glTF later. Bento
  does its own file access, which needs the user's approval and trust checks, then hands you bytes. BREP to bytes and
  back is also how Bento will save a shape's value in a workbook snapshot.

**Should**
- **Untrusted input.** STEP and BREP imports must assume the file could be hostile or broken. Return errors rather than
  crashing (see section 4), and put limits on anything that could allocate without bound.

**Done** (design.md, "Import and export"): `Importers` and `Exporters3D` read and write streams, with `ImportLimits` and
cancellation: BREP and STEP on both backends, STL, glTF and 3MF on OCCT.NET, which Bento uses. On OCCT.NET, STEP keeps
its names, colours and assemblies (OCCT.NET's STEPCAFControl), and data from a stream that refers to other files is
refused.

## 8. Not needed from this library

- Viewing, rendering, JSON for three-cad-viewer, colours for the viewer's tree, or anything Bento-specific: Bento.Cad
  does those.
- Builder mode in formulas. Do keep it for C# users. Bento just won't call it.
- Units handling: Bento converts before calling.

## 9. Order of priority for Bento

1. The native packaging and loading requirements (section 2), with the load-context test.
2. Structured, fast, thread-safe tessellation (section 3).
3. Clear exceptions and no native crashes (section 4).
4. Selectors callable with plain values (section 5).
5. Bytes in and out for BREP, STEP and STL (section 7).
6. Cancellation, memory pressure, instances and assemblies, then measuring.

Once items 1 and 2 are in a package on a local feed, Bento can turn build123d shapes into its models and show them in
the viewer. The rest can follow in any order.

---

## Reply from build123d.NET (28 October 2026)

Thanks, this is clear and fits the design. The plan in [design.md](design.md#phases) now has a phase 4,
**Bento readiness**, which follows your section 9 order: packaging, then no-mutation and thread safety,
tessellation, errors and memory, selectors, bytes in and out, and finally cancellation and measuring.
Builders move to phase 5. Notes on the points where we differ or where reviewing the note turned up
something:

- **Immutability had holes, and your note found them.** A few OCCT calls write into the `TShape` that
  shapes share, and Python build123d makes them too:
  - every bounding box calls `BRepTools::Clean`, which removes a mesh;
  - meshing stores its triangulation on the shape;
  - ShapeFix can raise the tolerances of shared edges and vertices;
  - booleans aren't non-destructive by default.

  These broke absolute immutability and raced between threads. **Fixed in 4.2:** meshing works on copies,
  bounding boxes ignore meshes, booleans run non-destructive, and ShapeFix, loft and tapered extrusion work
  on copies. `InputsUnchangedTests` checks every case's inputs as BRep bytes, alone and with all cases
  running concurrently on shared inputs. `Shape.Mesh` is now `Meshed`, which returns a meshed copy.
- **Exception types.** We keep build123d's mapping, which is more specific than you asked:
  - an argument problem (`ValueError`) is an `ArgumentException`, or an `InvalidOperationException` when
    the shape being operated on is empty;
  - a build123d `RuntimeError` is an `InvalidOperationException`;
  - an OCCT failure that build123d doesn't catch is an `OcctException` carrying the OCCT type.

  The messages are build123d's, and we'll add context where OCCT's own are bare. Please show the message
  of any exception rather than keying on `OcctException`.
- **Native crashes can be made rare, not impossible.**
  - The shim is built with `/EHa` and already catches access violations as errors.
  - `OSD::SetSignal` applies only to the thread that calls it, so we'll set it on each thread as it
    first calls in, which gives typed OCCT messages.
  - A stack overflow, or heap corruption that OCCT has already done, still ends the process, so autosave
    stays worthwhile.
- **Memory pressure** goes on new results and meshes only. Wrappers share geometry, so adding pressure
  per wrapper would count the same memory many times over.
- **Warnings** are per operation now (4.2). Wrap a cell's calculation in
  `using var w = Warnings.Capture();` and show `w.Messages`. A scope follows the async flow, so concurrent
  cells don't see each other's warnings.
- **Selectors** already take plain values for most cases: `FilterBy(Axis | GeomType | Plane)`,
  `SortBy(Axis | SortBy)`, `GroupBy(Axis | SortBy)`, `First`/`Last` and `Select`. Mapping strings such as
  `"Z"` to `Axis.Z` is better done in Bento's formula layer, where the vocabulary is yours.
- **Size.** The native folder is about 52 MB, and it includes TKV3d, TKService and TKMeshVS. Some of
  these are dependencies of the XCAF assembly toolkits and of text, so we'll prune by actual
  dependencies, checked by the load-context test, rather than by name.
- **Schematiq.Collections** is no longer a dependency (unit 10P1).

### Tessellation is in (4.3)

```csharp
ShapeMesh mesh = shape.Triangulate(linearDeflection: 0.1, angularDeflection: 0.2);  // radians
ShapeMesh auto = shape.Triangulate();             // MeshTolerances.For(shape): 0.1% of the diagonal
MeshNode  tree = assembly.TriangulateTree();      // labels, colours, locations; instances share meshes
```

It follows your sketch, with these differences:

- **Buffers.** `FaceMesh.Positions`, `Normals` and `Triangles`, and `EdgeMesh.Points`, are
  `ReadOnlyMemory<float>` and `ReadOnlyMemory<int>` slices, not arrays, so the mesh is immutable and
  isn't copied per face. Use `.Span` to read them. `NodeCount`, `TriangleCount` and `PointCount` give
  the sizes.
- **Order.** Index *i* of `Faces`, `Edges` and `Vertices` is index *i* of the shape's `Faces()`,
  `Edges()` and `Vertices()`. The tests check this.
- **Faces.**
  - Triangle indices are 0-based into the face's own nodes, counter-clockwise seen from outside.
  - Normals are the surface's at each node.
  - A face the mesher couldn't triangulate has no nodes rather than throwing.
- **Edges.** Each polyline runs the way its edge does. Where the edge bounds a face, its points are
  that face's own boundary nodes. Free edges (wires, curves) use the mesher's polygon.
- **Speed.** Copying out a mesh of 100k triangles takes about 6 ms in Release. OCCT's meshing itself
  takes about 1.4 s for a sphere at that density, so choose tolerances with care. `MeshTolerances.For`
  is a sensible default.
- **Thread safety.** Meshing works on a private copy, so concurrent calls on shapes that share geometry
  are safe, and the same inputs give the same output. An empty shape gives `ShapeMesh.Empty`.
- **Instances.** In `TriangulateTree`, parts that share a `TShape` (and orientation) share one
  `ShapeMesh` object. It is in the part's own coordinates, and `MeshNode.Location` is relative to the
  parent. Instancing is detected by reference equality.

### Errors and memory are in (4.4)

- **Native faults** come back as `OcctException`. On Windows they are typed, for example
  `OSD_Exception_ACCESS_VIOLATION: ACCESS VIOLATION at address … during 'READ' operation`, on whichever
  thread they happen. `OcctException.IsFault` tells a fault from an ordinary OCCT failure.
  - No floating-point traps are switched on, and .NET's own exceptions behave as before.
  - On Linux and macOS a fault in OCCT still ends the process, because OCCT's signal handlers would
    replace the runtime's.
- **Context.** An `OcctException`'s message starts with what was being done, and `Operation` gives it
  alone, for example "fuse of a Box with 2 Cylinders failed: StdFail_NotDone:
  BOPAlgo_AlertSelfInterferingShape". You can add your own outer scope per cell, and the library's
  scopes nest inside it:

  ```csharp
  using (OcctContext.Doing($"cell {address}")) { ... }   // "cell B3 / extrude failed: ..."
  ```

  Scopes are per thread; OCCT calls are synchronous.
- **Memory pressure** is on new results, released when their native handle is finalised or disposed:
  booleans, fillets, chamfers, extrude, revolve, loft, sweep, thicken and transformed copies. Sub-shapes
  and relocated copies share their result's pressure rather than adding to it. Triangulation frees its
  temporary native mesh straight away; the `ShapeMesh` itself is managed memory, which the GC already
  sees.
- **Malformed input.** Truncated, garbled or hostile BRep data gives an `OcctException`, usually a
  `Storage_StreamFormatError`, rather than the hang or buffer overrun OCCT's reader would allow.
  - 600 fuzzed inputs, each read, validated, measured, meshed and fused, gave no crash.
  - Three of them ran for a very long time in meshing or fusing. Until cancellation (4.7) is in, a cell
    with geometry read from an untrusted file may need a timeout.

  Degenerate geometry, such as zero-size primitives, extrusion by nothing, a coincident loft or an
  oversized fillet, throws `OcctException`, `ArgumentException` or `InvalidOperationException` with a
  message.

### Selectors are in (4.5)

- **Plain values.** Every selector now takes values, with the lambda overloads kept.
  - build123d's `property` arguments are taken by name: `FilterBy("IsClosed")`,
    `FilterByValue("Radius", 5)`, `SortBy("Length")` and `GroupBy("Area")`.
  - A name is a public property, or a method with no required arguments, of each shape's own class.
    Case and underscores don't matter, so `is_closed` works as well as `IsClosed`.
  - An unknown name throws `ArgumentException` listing the names that class has, which a cell could
    show as a hint.
  - `FilterByValue("GeomType", "Plane")` accepts an enum by its name.
  - `FilterByValue(SortBy.Length, 20)` filters by a measure.
  - Axes, planes and the other enums are still values to map from your formula vocabulary.
- **Selections** are `ShapeList<T>` as before. `GroupBy(...)` returns groups whose `First`, `Last`,
  `[i]`, `Keys` and `Group(key)` are all plain-value calls.
- **A selector for a picked face or edge** fell out naturally, so it's in.

  ```csharp
  IReadOnlyList<Selector> options = Selector.For(part, part.Faces()[pickedIndex]);
  options[0].ToString();    // "Faces().GroupBy(Axis.Z).Last"
  options[0].ToPython();    // "faces().group_by(Axis.Z)[-1]"
  ShapeList<Shape> again = options[0].Apply(changedPart);
  ```

  - **What a `Selector` is:** a value (faces, edges or vertices, then filters and groups) that compares
    by value, so a cell can keep it and reapply it when the model changes.
  - **Ranking:** fewest steps first, then the most robust. That means groups at the ends of the model's
    axes, then filters by type or direction, then the largest or smallest of a measure, then inner groups
    such as `GroupBy(Axis.X)[2]`.
  - **Checking:** every selector returned has been applied to the shape and picks exactly the pick.
  - **No selector:** the list is empty if three steps can't tell the pick apart, for example from an
    identical coincident twin.
  - **Speed:** it takes 5 to 20 ms on a plate of 78 faces and 192 edges.

### Runtime settings for a host (unit 11D)

Kerf can't choose these, because they belong to the application. For parallel recalculation, the application's project should have:

```xml
<PropertyGroup>
  <ServerGarbageCollection>true</ServerGarbageCollection>
  <ConcurrentGarbageCollection>true</ConcurrentGarbageCollection>
  <TieredPGO>false</TieredPGO>
</PropertyGroup>
<ItemGroup>
  <RuntimeHostConfigurationOption Include="System.Runtime.TieredCompilation.CallCountingDelayMs" Value="0" />
</ItemGroup>
```

| Setting | Why | What it costs |
|---|---|---|
| The server collector, with DATAS left on | The workstation collector stops every thread on one heap: 1,600 of a round's 2,700 ms. The server collector's pauses are about 170 ms | A peak of about 400 MB against 200 under full load; about the same (141 MB against 119) when calculating serially |
| `TieredPGO` off | Instrumented code is what the runtime runs for the first minute or so, and it's slower still on many threads. A round halves | Code that has fully settled is about 6% slower, after about two minutes of continuous recalculation |
| `CallCountingDelayMs` 0 | Hot code reaches its final form sooner: recalculation 260 ms to 180 ms, opening 660 ms to 615 ms | More JIT work early, on a background thread. Check the application's own start-up, which the benchmark doesn't have |

Leave out `GCHeapCount`, `GCgen0size` (no effect in `runtimeconfig.json`, and 1.6 GB at 64 MB under the server collector), `GCConserveMemory` (no measurable effect here) and a fixed heap count (`GCDynamicAdaptationMode` 0: about 700 MB, and at most 5 to 10% quicker). Bento's list is in [11d-bento.md](11d-bento.md), and the figures are in the kernel's `docs/performance.md`, "Unit 11D".

No tiering setting beyond these helps opening (unit 11C tried quick JIT, quick JIT for loops, on-stack replacement and the background worker's timeout). An application that ships its own runtime can go further by publishing self-contained with `PublishReadyToRunComposite`: opening 419 ms against 509 on Bento's examples, at 169 MB for the whole application, measured on assemblies built from source rather than from the packages. If it publishes ReadyToRun at all, setting `PublishReadyToRunCrossgen2ExtraArgs` to `--non-local-generics-module:*;--opt-cross-module:*;--Ot`, as Kerf's packages do, should help its own generics over Kerf's types (not measured).
