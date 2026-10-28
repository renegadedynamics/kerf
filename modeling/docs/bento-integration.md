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
- **Stay binary-compatible with the Bento.Collections Bento ships.** That is currently 0.4.0, and you reference the
  0.4.0 package. Inside Bento, the extension gets the host's copy, never its own, so `ShapeList<T>` must work against
  whatever 0.4.x Bento carries. If you need something newer from Bento.Collections, flag it rather than upgrading
  quietly, so both repositories move together.
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
- **Ship as NuGet packages to a local folder feed.** For example `Build123d` and `Build123d.Occt`, with natives under
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
- **Bento.Collections** stays at 0.4.0. We'll flag any need for something newer.
