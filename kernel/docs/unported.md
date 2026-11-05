# What's left of OCCT

*5 November 2026, for question 6 of [phase11-plan.md](phase11-plan.md). The figures come from
[port-status.md](port-status.md), and count physical lines of OCCT 8.0.1. The owner's answer for now is not to port more,
so this is a list to choose from, not a plan.*

**Ported: 1,796,086 of 2,897,781 lines (62.0%).** That leaves 1,101,695 lines, and most of them are what Kerf decided
early on not to need (design.md): OCCT's viewer, its document framework and its test console.

| What's left | Lines | What it does | What it might be useful for | Worth porting? |
|---|---:|---|---|---|
| **IGES** (TKDEIGES) | 141,411 | Reads and writes IGES, the older CAD exchange format | Opening files from older CAD systems and suppliers. The one gap a user is likely to hit | **The best candidate**, if a user asks. Reading alone would be smaller |
| **Visualisation** (TKV3d, TKOpenGl, TKService, TKMeshVS and others) | 277,081 | OCCT's 3D viewer on OpenGL: display, selection, materials | A Kerf GUI could borrow selection's ideas, but Avalonia would want its own renderer over Kerf's meshes | No. Write what a GUI needs instead |
| **Test console** (Draw) | 192,386 | OCCT's command-line test harness | Nothing further: Draw-lite already runs OCCT's test scripts | No |
| **Document framework** (OCAF: TKLCAF, TKCAF, TKCDF, TKStd, TKBin, TKXml and others) | 123,021 | Documents with undo, attributes, and OCCT's own file formats | A GUI's document model and undo, though immutable shapes make undo simple without it | No. Design it for .NET if a GUI needs one |
| **The old Boolean engine** (the rest of TKBool: TopOpeBRep and friends) | 88,598 | Boolean operations from before today's engine | Nothing new: the parts other algorithms need, fillets among them, are already ported | No |
| **BRepGraph** (in TKBRep) | 55,984 | New in OCCT 8: topology in flat tables with integer links, for fast traversal and parallel work | **Worth reading during phase 11.** It's aimed at the speed and memory problems 11B and 11D are after, but it would mean a second way of holding shapes | Read it for ideas; a port is a decision for later |
| **Foundation's remainder** (the rest of TKernel) | 100,182 | Operating system wrappers, Unicode tables, memory managers and collections, mostly replaced by .NET. Includes Units and UnitsAPI (7,077 lines), converting between units of measure | Units might be handy, but it's small enough to write afresh | No, apart from anything specific that comes up |
| **Assembly documents** (TKXCAF and its binary and XML forms) | 31,577 | Assemblies, names and colours in an OCAF document | Kerf already has an immutable assembly tree, and STEP's names and colours over it (question 31) | No |
| **VRML** (TKDEVRML) | 22,851 | Reads and writes VRML, an old web 3D format | Occasional old files; glTF has replaced it | Only if asked |
| **Mesh formats** (TKDEGLTF, TKDEOBJ, TKRWMesh, TKDEPLY) | 20,068 | OCCT's readers and writers for meshes | glTF, OBJ, 3MF and STL already work through Kerf's own code. **PLY isn't covered**, and is small | PLY if asked |
| **Expressions** (TKExpress) | 15,888 | Symbolic mathematical expressions | Formula-driven curves. Bento has its own formula language | No |
| **Leftovers in ported areas** (TKXSBase, TKDESTEP, TKDE, TKDESTL and others) | 32,648 | Interactive selection in the STEP and exchange layers, STEP AP209, STEP's tidying step (StepTidy, 2,655 lines), and DE's plugin registry | StepTidy could make written STEP files smaller, so it's worth a look when STEP's header is changed | StepTidy perhaps |

## For a Kerf GUI

If the Avalonia GUI goes ahead, none of this needs porting first. Kerf already has meshes, hidden-line drawing, a managed
renderer and an assembly tree. A GUI would need its own viewport, picking and document model, and these are better
written for .NET than ported from OCCT's viewer and OCAF.
