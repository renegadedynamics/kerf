# OCCT.NET design

OCCT.NET is a managed port of [Open CASCADE Technology](https://github.com/Open-Cascade-SAS/OCCT) (OCCT) to .NET.
Its first user is Bento, a spreadsheet whose cells hold immutable .NET values, through build123d.NET, which
today wraps native OCCT. A managed kernel removes native crashes, the 52 MB of native libraries, load-context
plumbing, shared-geometry races and finalisers, and lets models run where Bento's engine runs in WebAssembly.

## Decisions

| Topic | Decision |
|---|---|
| Source | OCCT **V8_0_1**, commit `b8f597c677811d1f9f4d8a97f5ae2825c0353a42` (30 July 2026), the latest stable release, and the one build123d.NET builds natively. Recorded in `occt-version.json`. |
| Licence | **LGPL 2.1 with the Open CASCADE exception**, as OCCT. The port is a derivative work. |
| Target | .NET 10. Windows x64 first, but nothing platform-specific: the port is pure managed code. |
| Shape of the port | **Mechanical and class for class.** OCCT's type, method, field and parameter names and its file layout are kept, so code can be compared with OCCT and upstream fixes carried across. See [translation-rules.md](translation-rules.md). |
| Public API | **Functional.** Every value a caller sees is immutable. Mutators exist but are `internal`, for ported algorithms. Mutable builders are allowed. |
| Value types | gp and other by-value classes are structs, passed `in`. An analyzer (OCCTNET0001) stops internal mutators from being called where C# would silently mutate a copy. |
| Collections | Returned collections are Bento.Collections interfaces (`IBentoList<T>` and friends), against the version Bento ships (0.4.0). Numeric arrays are `PackedList<T>`: contiguous, readable as a span, still an `IBentoList<T>`. |
| Floating point | Parity with OCCT is not required everywhere, but the C++ order of operations is kept. In the first package (gp), 66–100% of results are bit-identical to native OCCT, depending on the operation, and the rest agree to within 5e-14. |
| Errors | OCCT's exception hierarchy (`Standard_Failure` and subclasses). OCCT's release builds compile most argument checks out; the port always makes them. |
| Serialisation | OCCT's formats, ported faithfully (BRep text and binary in phase 3, STEP in phase 8), so files round-trip with OCCT and its viewers. |
| Assemblies | `OcctNet.Foundation` (TKernel's Standard and Precision, TKMath), then `OcctNet.ModelingData`, `OcctNet.ModelingAlgorithms`, `OcctNet.DataExchange`. They see each other's internals. |

## Verification

Three layers, all from phase 0:

1. **OCCT's GTests**, ported to xUnit alongside each package (`tests/OcctNet.Foundation.Tests`).
2. **The oracle**: the same random inputs through the port and through native OCCT 8.0.1, via build123d.NET's
   generated bindings (`tests/OcctNet.Oracle.Tests`, [oracle-testing.md](oracle-testing.md)). Each comparison
   reports how many results are bit-identical and the largest difference.
3. **OCCT's Draw corpus**: 17,237 test scripts in OCCT's `tests/`, run by Draw-lite (`tests/OcctNet.DrawLite`), a
   small Tcl interpreter to which modelling commands are added as packages are ported. Its scan lists the
   commands the corpus needs, most-used first.

Progress is measured by `dotnet run tools/port-status.cs`, which writes [port-status.md](port-status.md).

## Phases

| Phase | Contents | Milestone |
|---|---|---|
| **0. Groundwork** (done) | Repository, translation rules, analyzer, oracle harness, Draw-lite skeleton, port status, the gp package | gp ported and tested three ways |
| 1. Foundation and maths | Rest of TKMath: math, PLib, BSplCLib/BSplSLib, ElCLib/ElSLib, Bnd, Poly, BVH, Convert, CSLib; NCollection replacements | Matches the oracle |
| 2. Geometry | TKG2d, TKG3d, TKGeomBase (AppCont's tables generated; AdvApp2Var last). Rules for Handle classes and freezing finalised. | Curves, surfaces, projections, extrema |
| 3. Topology and input/output | TopoDS, TopLoc, BRep, TopExp, BRepTools/BinTools, BRepGProp, BRepBndLib, BRepCheck, BRepExtrema; Draw-lite `restore`, `checkshape`, `checkprops` | Every `.brep` in the corpus round-trips byte-identical; Bento can store, load and measure shapes without native code |
| 4. Meshing | BRepMesh | Bento's viewer fully managed; WebAssembly demo |
| 5. Construction | BRepBuilderAPI, BRepLib, primitives, sweeps, BRepFill/GeomFill, Geom2dGcc, intersections (IntPatch, IntWalk, IntPolyh) | build123d's objects can be built |
| 6. Booleans | IntTools, BOPDS, BOPAlgo, BRepAlgoAPI (not the legacy TopOpeBRep) | build123d.NET's algebra-mode tests pass |
| 7. Finishing | ShapeHealing, fillets and chamfers, offsets, drafts and features, helix, then HLR | build123d parity |
| 8. Data exchange | STEP generated from the EXPRESS schemas, StepData/XSBase/STEPControl ported; immutable assembly tree; STL, glTF, 3MF, OBJ written fresh | Bytes in and out without native code |
| 9. Switch-over | build123d.NET moves onto OCCT.NET, op by op through BRep bytes, then drops native OCCT | |

## Not ported

Visualization (TKV3d, TKOpenGl, TKService, MeshVS, IVtk): Bento has its own viewer. The OCAF application framework:
Bento has its own document model and undo. Draw, except Draw-lite for tests. IGES, VRML and the Deprecated module
unless asked for. The legacy TopOpeBRep Booleans.
