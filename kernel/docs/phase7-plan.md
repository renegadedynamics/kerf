# Phase 7 plan: finishing

This plan splits phase 7 into work units that can be ported in parallel, as [phase5-6-plan.md](phase5-6-plan.md) did
for phases 5 and 6. Phase 7 is design.md's "finishing" phase:

- the rest of ShapeHealing (ShapeUpgrade's divide and convert tools, ShapeCustom, ShapeProcess);
- 3D fillets and chamfers (TKFillet, but ChFi2d, which phase 5 ported);
- 3D offsets and drafts (TKOffset, but phase 5's wrappers; `BRepFill_Draft`, AdvancedEvolved);
- features (TKFeat);
- helix (TKHelix);
- hidden-line removal (TKHLR).

**The finding that shapes the plan:** OCCT's 3D fillets still build their result through the legacy topological data
structure (TopOpeBRepDS, TopOpeBRepBuild, TopOpeBRepTool). That is about 79,000 lines of TKBool, about 72,000 of them
new, on top of TKFillet's 91,000. Phase 5 ported only the slice of it that the 2D offset needs (2.11 there). Section
1.3 gives the evidence, and section 2.1 the recommendation. Nothing else in phase 7 needs it.

The milestones (section 5) are:

- build123d.NET's fillets, chamfers, hollow solids, 3D offsets, thickening, drafts, tapered extrusions, draft prisms,
  perimeter splits and B-spline conversion reproduce native results through BRep bytes;
- the corpus's healing, fillet, chamfer, offset, draft, feature, helix and HLR cases (about 5,600 newly runnable,
  3,200 with their data) pass at DRAWEXE's rate;
- every native OCCT call build123d.NET makes has a port ("build123d parity", design.md).

The rules are the earlier phases': results freeze on publication, writers copy first, exact copy and `KeepUnchanged`
(decision 416), makers' `Internal*` accessors (455), one parallel helper (`OSD_Parallel`, 522), a parallel run gives
the serial result (490), OCCT's bugs reproduced and logged. Section 2 settles what is specific to phase 7.

All counts here were measured on `main` after phase 6 (3887494), with OCCT V8_0_1's source.

## 1. Scope

### 1.1 What phase 7 starts from

Phases 0 to 6, merged on `main`: 39.5% of OCCT's lines (port-status.md). For phase 7 what matters is:

- **TKGeomAlgo** (99.9%), **TKTopAlgo** (99.3%), **TKBO** (99.9%), **TKPrim**, **TKMesh**.
- **TKShHealing** (75%): ShapeAnalysis, ShapeBuild, ShapeConstruct, ShapeExtend and ShapeFix whole; of ShapeUpgrade
  only UnifySameDomain and RemoveLocations.
- **TKBool** (34%): BRepFill (91%), BRepAlgo (82%), BRepProj; the 2D offset's subset of TopOpeBRepBuild (20%),
  TopOpeBRepTool (29%) and TopOpeBRepDS (7%); none of TopOpeBRep.
- **TKFillet:** ChFi2d and `BRepFilletAPI_MakeFillet2d` only. **TKOffset:** the BRepOffsetAPI wrappers of phase 5
  (MakePipe, MakePipeShell, ThruSections, MakeFilling, MakeOffset, MakeEvolved without its volume mode, MiddlePath).
- Draw-lite with the phase 5 and 6 commands; the oracle harnesses of 6F/6P (Booleans), 6U (UnifySameDomain) and 6M
  (build123d.NET's operations replayed).

### 1.2 Inventory

Thousands of physical lines of OCCT source (`.cxx .hxx .lxx .gxx`, GTests apart), still to port.

| Area | Packages (k lines) | Total |
|---|---|---:|
| **Healing** | ShapeUpgrade's other 71 files 10.9; ShapeCustom 6.1; ShapeProcess 3.0; ShapeAlgo 1.0; SHMessage 0.6; ShapeProcessAPI 0.3; TKernel's `Resource_Manager` 0.8 | 22.7 |
| **Fillets** | ChFi3d 39.0 (Builder 21.8, Builder_0 6.5, FilBuilder 5.5, ChBuilder 3.9, package 1.1); BRepBlend 19.3; BlendFunc 14.7; ChFiKPart 6.0; ChFiDS 5.8; Blend 2.7; FilletSurf 1.7; BRepFilletAPI's rest 1.7 | 90.9 |
| **The legacy data structure** (for fillets) | TopOpeBRepBuild 37.3; TopOpeBRepDS 23.0; TopOpeBRepTool 19.0; about 72 of the 79.3 new | 79.3 |
| **Offsets and drafts** | BRepOffset 30.8 (MakeOffset 15.5, Tool 4.9, Inter2d 2.5, Offset 2.0, Inter3d 1.7, Analyse 1.3); Draft 4.0; BiTgte 3.7; BRepOffsetAPI's rest 2.5; BRepFill_AdvancedEvolved 2.0, Draft and DraftLaw 1.2; BRepAlgo_NormalProjection 0.8 | 45.0 |
| **Features** | BRepFeat 16.8; LocOpe 15.8 | 32.6 |
| **Helix** | HelixGeom 1.3; HelixBRep 0.8 (new in OCCT 8) | 2.1 |
| **HLR** | HLRBRep 26.1; Contap 7.5; HLRAlgo 6.3; HLRTopoBRep 2.7; Intrv 1.1; TopBas, TopCnx, HLRAppli 0.6 | 44.2 |
| **All** | | **≈ 317** (≈ 310 new) |

That is a little smaller than phases 5 and 6 together (395,000 lines), and over half of it is the fillets with their
data structure.

**OCCT's GTests to port** (phase 7's toolkits): `BRepFilletAPI_MakeFillet` (535 lines), `BRepFilletAPI_MakeChamfer`
(249), `BRepOffset_MakeOffset` (1,029; uses `BRepAlgoAPI_Fuse`), `BRepOffsetAPI_MakeThickSolid` (149),
`ShapeUpgrade_FaceDivide` (66), and TKHelix's six files (1,521). TKFeat and TKHLR have none.

### 1.3 The legacy data structure: what needs it

Read from the call sites, not only the includes:

- **Fillets need it, with no other path.** `ChFi3d_Builder_1.cxx:346` creates `myCoup = new
  TopOpeBRepBuild_HBuilder(...)`, and `ChFi3d_Builder::Compute` always calls `myCoup->Perform(myDS)`
  (`ChFi3d_Builder.cxx:471`), then MergeSolid, IsSplit, Splits, Merged and NewFaces. The fillet and chamfer histories
  read the HBuilder (`ChFi3d_FilBuilder.cxx:2606–2624`, `ChFi3d_ChBuilder.cxx:2236–2243`), and every
  `ChFiKPart_ComputeData_*` writes into `TopOpeBRepDS_DataStructure`. `HBuilder` holds a `TopOpeBRepBuild_Builder1`,
  so the whole Builder/Builder1 machinery is on the path.
- **The closure** from TKFillet, hard edges only: TopOpeBRepBuild 29 classes (37,300 lines; Builder 19,300, Builder1
  3,900, BuilderON 3,500), TopOpeBRepDS 49 (23,000), TopOpeBRepTool 21 (19,000). About 3,000 to 5,000 lines are already
  ported (phase 5's slice), about 4,600 sit in `OCCT_DEBUG`/`DRAW` blocks and about 2,000 are trace and dump files.
  The include graph over-approximates; a member trace might cut TopOpeBRepBuild and Tool by 10 to 25%.
- **TopOpeBRep itself (the intersection package, 23,600 lines) is not needed.** No phase 7 package and nothing in the
  closure includes it, and nothing in phase 7 uses `BRepAlgo_BooleanOperation`.
- **Offsets do not need it.** `BRepOffset_Analyse` calls only ChFi3d's package functions `DefineConnectType` and
  `IsTangentFaces`, which need `TopOpeBRepTool_TOOL::Nt`, `ChFi3d_Coefficient` (plain maths) and
  `ChFiDS_TypeOfConcavity`. A file-level count would wrongly add 38,000 lines through `ChFi3d.cxx`'s includes.
- **BiTgte** calls ChFi3d only for chronometers inside `OCCT_DEBUG`. **FilletSurf** builds the HBuilder but never runs
  `Perform`. **BRepFeat, LocOpe, Draft, HLR, helix and healing** have no edge into the legacy data structure.

### 1.4 Other dependencies outside phase 7's packages

Everything else phase 7 reaches is ported, but for:

- `Resource_Manager` (TKernel, 781 lines): ShapeProcess reads its operator sequences from resource files through it.
- `BRepAlgo_NormalProjection` (798): ChFi3d_Builder and `BRepOffsetAPI_NormalProjection`.
- `ShapeCustom_Curve2d`: BRepOffset_Tool uses it, so healing's first unit precedes the offset units.
- `Quantity_Color` and `Quantity_ColorRGBA` (TKernel, small): not phase 7, but the only class outside phase 7 and
  STEP that build123d.NET calls and the port lacks (its `Color`). Unit 7X ports it.

### 1.5 build123d.NET's use of phase 7

From `build123d.net/src/Build123d` (file:line in its Topology folder). Every class here is bound natively in
build123d.NET's shim, so each has a direct oracle.

| OCCT call | build123d.NET API | Options |
|---|---|---|
| `BRepFilletAPI_MakeFillet`: `Add(r, edge)`, `Shape`, the faulty-contour and stripe-status diagnostics | `Shape3D.Fillet`, `MaxFillet` (a binary search, a new builder per probe), the `fillet` operation | constant radius, default shape type; `ChFiDS_*Failure` statuses become messages |
| `BRepFilletAPI_MakeChamfer`: `Add(d1, d2, edge, face)` | `Shape3D.Chamfer`, the `chamfer` operation | two distances |
| `BRepOffsetAPI_MakeThickSolid::MakeThickSolidByJoin` | `Shape3D.Hollow`, `Shape3D.Offset3D`, the `offset` operation | Skin, intersection on, Arc or Intersection joins, remove-intersections on for Offset3D |
| `BRepOffset_MakeOffset`: `Initialize` (9 arguments), `MakeOffsetShape` | `Solid.Thicken` (thickening), `Shape2D.OffsetCore`, `OneD.OffsetTopodsFace` (ExtrudeTaper's fallback) | tolerance 1e-5, Intersection join |
| `BRepOffsetAPI_DraftAngle` | `Solid.Draft`, `Solid.ExtrudeTaper`, the `draft` operation | planes, cylinders, cones; `ProblematicShape` on failure |
| `BRepFeat_MakeDPrism` | `Shape3D.DPrism` | through all, depth, up to a face; taper; fuse or cut |
| `BRepFeat_SplitShape` | `Shape2D.SplitByPerimeter` | a sequence of edges |
| `ShapeCustom::BSplineRestriction` | `Shape.ToSplines` | default restriction parameters |

**Not used by build123d.NET:** HLR, TKHelix (its `Helix` is built from a cylinder, a 2D line and
`BRepLib::BuildCurves3d`, all ported), ShapeProcess, ShapeUpgrade's divide tools, LocOpe directly, BiTgte, FilletSurf,
`BRepFill_Draft`, AdvancedEvolved. They are in phase 7 for the corpus, for Bento and for completeness.

About 25 of build123d.NET's 70 test files exercise phase 7: Shape3DTests (fillet, chamfer, hollow, DPrism,
MaxFillet), SolidTests (taper, draft, thicken), OperationsGenericTests (fillet 20, chamfer 34, offset),
OperationsPartTests (draft, thicken), FaceTests (SplitByPerimeter, Offset), ShapeTests, HistoryTests,
MalformedInputTests, MemoryPressureTests, InputsUnchangedTests, EmptyShapeTests, TopologyReferenceTests.

build123d.NET also depends on behaviour beyond the shapes: exception types (`StdFail_NotDone`), the fillet's
`ChFiDS` statuses and faulty-contour counts, `Generated`/`Modified`/`IsDeleted` on the fillet, chamfer and offset
makers, and a null result after a reported success in Hollow and Offset3D (its comment at Shape3D.cs:363 notes that
`MakeThickSolid` can return reversed solids that `MakeFillet` then misreads). The ports must reproduce all of it.

### 1.6 Draw commands and the corpus

Measured with `DrawCorpus.Scan` over all 21,068 cases (the `cases.list` grids included), with
`FollowsCalledProcsOnly` on (question 29), by removing an area's commands from each case's missing set. "Data" means
the files a case reads exist (checked against real runs on 240 random cases).

**Today:** 10,106 cases runnable, 6,811 with their data (8,347 and 6,107 with `FollowsCalledProcsOnly` off).

| Area | Commands (cases calling them) | Newly runnable (with data) | Top grids |
|---|---|---:|---|
| Healing | `DT_SplitAngle` 802, `DT_ShapeDivide` 796, `expshape` 754, `DT_ShapeConvertRev` 754, `DT_ClosedSplit`, `DT_ToBspl`, `DT_ShapeConvert`, `DT_ApplySeq`, `DT_SplitByNumber`, `directfaces`, `scaleshape`, `offsetcurve`, `offset2dcurve`, `bsplres` | 2,604 (1,618) | heal's split_angle, split_continuity and surface_to_revolution grids, standard 464 each and advanced 289 each |
| Fillets and chamfers | `blend` 428, `chamf` 296, `mkevol`/`updatevol`/`buildevol` 113, `tolblend` 67, `bfuseblend`, `chamf_throat*`, `brollingball`, `checkhist` | 849 (758) | blend/simple 226, blend/buildevol 98, blend/complex 63, chamfer grids about 49 each |
| Offsets and drafts | `offsetload`/`offsetparameter`/`offsetperform` 1,417, `offsetonface` 263, `depouille` 136, `offsetshapesimple` 79, `thickshell` 58, `offsetshape`, `offsetcompshape`, `draft`, `mkoffsetshape` | 1,584 (457) | offset/shape_type_i_c 803, offset/with_intersect_20 and _80, draft/angle 113 |
| Features | `featperform` 229, `featprism` 150, `featperformval`, `featrevol`, `featlf`, `featdprism`, `splitshape`, `fillet`, `featrf`, `addslide`, `hole` | 361 (307) | feat/featprism 138, featrevol 78, featlf 40, featdprism 35 |
| Helix | `comphelix`, `helix`, `spiral`, `comphelix2`, `spiral2`, `helix2`, `setaxis`; and the viewer no-op `vzfit` | 60 (60) | helix/standard 56 |
| HLR | `vcomputehlr` 172, `build3d` 176, the Draw views `top`, `v2d`, `axo`, `front`; HLRTest's `hprj`, `hload`, `hhide`, `houtl`... (5 cases) | 157 (36) | hlr/poly_hlr, hlr/exact_hlr |
| **Phase 7** | | **5,615 (3,236)** | |

After phase 7: 15,721 cases runnable (74.6%), 10,047 with data. The areas hardly overlap: each area's cases are
blocked by that area alone. Offsets gain few cases with data because the offset group's grids mostly read data that
isn't public.

**Cheap gaps outside phase 7** (question 28), as the only blocker after phase 7:

- Draw's core: `lastrep` 240 (an interactive pick reached only through the `disp`/`don`/`del`/`era` procedures'
  joker branch; a stub does), `dversion` 25, `dmode` 13, `XProgress` 11, `meminfo` 8, `removeinternals` 8, the unit
  commands 4.
- Geometry commands over ported packages: `proj` 65, `shape` 51, `project` 51, `convert` 39, `nproject` 27,
  `projonplane` 27, `validrange` 12, `2dintersect` 10, `2dextrema` 8, a few more.
- All of these together: +885 cases (522 with data). The viewer commands would add 1,177 more (508 with data), but
  those cases would run without testing anything, so they stay out.

**Still blocked after phase 7:** 5,347 cases: the viewer (1,177), OCAF (427, and 765 more with phase 8), phase 8's
STEP and IGES (586, and the `de` grids' 1,073), QA test commands (137), and the rest.

## 2. Design questions

### 2.1 The legacy data structure: port the classes the fillets reach

Phase 5 ported a member-level slice of TopOpeBRepBuild, DS and Tool for `WireToFace::MakeFaces` (phase5-6-plan.md
2.11). The fillets need far more of it: `HBuilder::Perform` and `MergeSolid` reach the grid, `ffsfs`, `fctwes`,
`makefaces`/`makesolids`, the regularisations (`SREGU`/`FREGU`) and the KPart files.

**Recommendation: port the classes the fillets reach, whole** (about 79,000 lines, 72,000 new), extending phase 5's
files where they exist, as phase 6 did for ShapeFix (question 16). The alternatives:

- **A member-level slice**, as 2.11. It saves perhaps 10 to 25% of TopOpeBRepBuild and Tool, but every member needs
  a trace, a later caller may need what was left out, and the slice is hard to review against OCCT.
- **Fillets on BOPAlgo instead** (a rewrite of ChFi3d's Compute). Not a port: results would differ from native in
  ways the oracle couldn't attribute, and upstream fixes would no longer carry across.

Either way the data structure's shared state changes form:

- **File-scope and `thread_local` state** (`TopOpeBRepBuild_Builder1`'s `mySDEdgeMap`, the grids' `GLOBAL_DS2d`,
  `GLOBAL_TOOLBOX_hboxtool`, the `thread_local` globals in HBuilder, `ffsfs`, `DS_connex` and `Tool_2d`) becomes
  state of the builder or of a context passed down, as `IntTools_Context` did for the Booleans (decision 580). Where
  OCCT reads a value an earlier call left behind, the port keeps that (`[ThreadStatic]`) and logs it, so results
  match native's.
- **Address-keyed maps** iterate in insertion order (as in phase 6); the sites where that order is observable are
  logged.
- The data structure is a working object, not a frozen value: it is internal to the builders.

### 2.2 Inputs the algorithms write into

All of these must copy at entry (`BRepTools_CopyAtEntry` or `ExactCopyForWrite` plus `KeepUnchanged`) and give back
the caller's own sub-shapes where nothing changed:

| Area | Writes |
|---|---|
| Fillets | `ChFi3d_Builder` raises the tolerance of the input's vertices (`UpdateVertex`, :449, :505; Builder_0 :2333); TopOpeBRepBuild reuses input sub-shapes and calls `Free(true)` on them |
| Offsets | `BRepOffset_MakeOffset::MakeMissingWalls` adds pcurves to the input's boundary edges (:3421, :3512); `UpdateTolerance` calls `Locked(false)` and raises edge and vertex tolerances (:4196–4290); `Free(true)` on edges and shells at six sites; `BRepOffsetAPI_DraftAngle` frees and edits the modifier's result |
| Features | `LocOpe_SplitShape` adds point representations to input vertices (:198–246); `LocOpe_Generator` adds pcurves to input edges; `LocOpe_Gluer` sets continuities; `BRepFeat_MakeLinearForm` raises vertex tolerances. BOP is non-destructive on frozen arguments already (decision 274) |
| Healing | ShapeUpgrade's WireDivide and ShapeConvertToBezier and `ShapeCustom_DirectModification` update edges and vertices, mostly through `ShapeBuild_ReShape` or `BRepTools_Modifier` (which copy); each site is audited as decisions 274–275 |
| HLR | `HLRTopoBRep_DSFiller` (:739–754) and `FaceIsoLiner` (:484–486) add points to the input's vertices |

`Locked(false)` is the one OCCT idiom with no port equivalent: a frozen shape cannot be unlocked. The offset unit
copies first, so its `Locked(false)` acts on the copy.

### 2.3 Process-wide state in healing and HLR

- **ShapeProcess** keeps a static operator registry (`ShapeProcess.cxx:29`), a static mutex with a cached
  `Resource_Manager` (`ShapeProcess_Context.cxx:33–88`) and a static `done` flag (`ShapeProcess_OperLibrary.cxx:984`);
  `ShapeAlgo` a static global `AlgoContainer`. Ported as process-wide state with OCCT's semantics. Draw-lite tests
  that change it join the non-parallel collection, as the mesher's defaults did (decision 496's pattern), and
  DrawCorpus resets it after each case.
- **Resource files** (question 26): ShapeProcess reads `CSF_*Defaults` resource files from environment variables.
  The port embeds OCCT's default resource files and reads a file only when told to.
- **Scratch statics:** `Contap_HContTool`'s `static double uinf, vinf, usup, vsup` and the `thread_local` scratch
  vectors of `BlendFunc_ConstRad`/`EvolRad` and `ChFi3d_Builder_6`. Where OCCT never reads a value across calls they
  become locals; otherwise `[ThreadStatic]`, logged. HLRBRep's global counters and the debug flags of `ChFi3d_Debug`
  and `Blend_Debug` keep OCCT's behaviour (they affect only traces).

### 2.4 Parallelism

There are no `OSD_Parallel` loops in phase 7's own packages. AdvancedEvolved runs Booleans with `SetRunParallel`, and
BRepOffset_MakeOffset runs BOP and MakerVolume: those inherit phase 6's determinism (parallel = serial). The fillet
units must not introduce parallelism.

### 2.5 Generics

In OCCT 8, Blend and BRepBlend are no longer built from `.gxx` files, apart from `BRepBlend_AppSurf_0.cxx`
(`AppBlend_AppSurf`, already a C# generic). HLRBRep (10 files) and Contap (3) instantiate generics phase 3 and 5
already ported (`IntCurve_*Gen`, `IntImp_IntCS`, `IntStart_*`, `IntWalk_IWalking`,
`Intf_InterferencePolygonPolyhedron`). Phase 7 adds no new generic patterns.

### 2.6 History and failure reporting

build123d.NET reads `Generated`, `Modified` and `IsDeleted` from the fillet, chamfer, thick-solid and offset makers,
and reports fillet failures from `NbFaultyContours`, `FaultyContour`, `StripeStatus` and the `ChFiDS_ErrorStatus`
values. These go in the units' oracle as first-class results, compared as phase 6 compared Boolean histories. The
port's exception types match native's (decision 9, as corrected).

### 2.7 Performance

Fillets and offsets are among the slowest calls build123d.NET makes (MaxFillet runs a fillet per probe). Targets as
for the Booleans (question 22): within 2× native serial. Unit 7X adds a fillet, chamfer and offset suite to the
benchmarks, and re-measures phase 6's parallel Booleans on a quiet machine.

### 2.8 What the public API looks like to Bento

As phase 6 (question 21): the makers stay OCCT's working objects with frozen results. The fillet, chamfer,
thick-solid, offset and draft makers each get a functional twin over an equatable options struct, with OCCT's names
and defaults, nothing added (for example `BRepFilletAPI.Fillet(shape, edges, radius, in options, out history)`).

## 3. The oracle

**Bound natively** (direct comparisons from BRep bytes, as phase 6's oracle): `BRepFilletAPI_MakeFillet`,
`MakeChamfer`, `BRepOffsetAPI_MakeThickSolid`, `MakeOffsetShape`, `BRepOffset_MakeOffset`,
`BRepOffsetAPI_DraftAngle`, `BRepFeat_MakeDPrism`, `BRepFeat_SplitShape`, `ShapeCustom::BSplineRestriction`.
Some members are not bound: `MakeFillet`'s law overloads, the `Builder()` accessors, `BRepOffset_MakeOffset::GetAnalyse`,
`ShapeCustom_RestrictionParameters`' setters.

**Not bound** (question 27): ShapeUpgrade's divide and convert tools, ShapeProcess, the other BRepFeat makers, LocOpe,
BiTgte, FilletSurf, `BRepOffsetAPI_MakeDraft`, AdvancedEvolved, HLR, TKHelix. For these the units use a native
recorder (`tools/<name>rec`, built with MSVC against build123d.NET's OCCT, its recordings embedded in the tests), as
phase 5 did for MAT and Bisector, plus the corpus.

The cases:

- **Fillets and chamfers:** every edge of the corpus's blend and chamfer inputs, data/occ shapes and primitives,
  singly and in chains, constant and evolving radii; the faulty-contour diagnostics on radii too large; build123d.NET's
  `MaxFillet` probes.
- **Offsets:** MakeThickSolid and MakeOffsetShape on primitives, Boolean results and data/occ shapes, both joins, both
  signs, with and without intersection; thickening of faces and shells; build123d.NET's Hollow, Offset3D and Thicken.
- **Drafts:** DraftAngle on the draft group's inputs and primitives (planes, cylinders, cones), neutral planes in
  and outside the shape.
- **Features:** DPrism in each mode; SplitShape with edges and wires on faces.
- **Healing:** the heal group's public inputs through each divide and convert tool (recorded).
- **HLR:** exact and polygonal HLR of the hlr group's shapes in fixed projectors (recorded).

**build123d.NET's operations** (unit 7X) extend 6M's replay: each phase 7 call its tests make, rebuilt with its public
API from the Oracle project, compared as phase 6 did (bytes, then canonically where native varies, histories as
sets), with build123d.NET's `Fix()` and `Clean()` after the operation where it applies them.

## 4. Work units and waves

Each unit is one branch and one porter, as in phases 5 and 6. Sizes are thousands of OCCT lines. **Tests:** GT is
OCCT's GTests; Oracle is native comparisons; Corpus is Draw-lite cases (run by 7W, but each unit's oracle and unit
tests can use corpus inputs).

### Wave 1: start at once

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **7A** ShapeCustom (the healing pilot) | ShapeCustom (all: Curve, Curve2d, Surface, BSplineRestriction, ConvertToBSpline, ConvertToRevolution, DirectModification, SweptToElementary, TrsfModification, RestrictionParameters, the package); SHMessage. **Design:** the copy-at-entry audit for healing's modifiers (2.2) | 6.7 | phase 6 | Oracle: `ShapeCustom::BSplineRestriction` (bound) on build123d.NET's `ToSplines` inputs and data/occ shapes. Recorded: the other modifications on the heal group's inputs |
| **7D** Helix | HelixGeom, HelixBRep | 2.1 | phase 6 | GT: TKHelix's six files (1,521 lines). Recorded: `HelixBRep_BuilderHelix` on the helix grid's parameters |
| **7E** Offset foundations (the offset pilot) | BRepOffset but MakeOffset: Analyse, Tool, Inter2d, Inter3d, Offset, Interval, the package, MakeSimpleOffset and its writers; ChFi3d's package functions (`DefineConnectType`, `IsTangentFaces`, `ConcaveSide`...) with `ChFi3d_Coefficient`, `TopOpeBRepTool_TOOL::Nt` and `ChFiDS_TypeOfConcavity`. **Design:** copy at entry for the offset family (2.2, `Locked(false)`) | 16.4 | 7A (ShapeCustom_Curve2d) | Unit tests: BRepOffset_Analyse's edge types on primitives and Boolean results against recorded native types; BRepOffset_Offset's faces. Oracle: indirect (7F) |
| **7H** LocOpe | All of LocOpe (SplitShape, WiresOnShape, Spliter, Generator, Gluer, SplitDrafts, DPrism, Prism, Revol, Pipe, LinearForm, RevolutionForm, the finders and builders) | 15.8 | phase 6 | Recorded: LocOpe_SplitShape and Spliter on faces of data/occ shapes; frozen inputs (2.2) |
| **7J** Blend functions | Blend, BlendFunc, ChFiDS | 23.2 | phase 6 | Unit tests: BlendFunc's functions (ConstRad, EvolRad, Chamfer, ConstThroat, Ruled, CSConstRad...) value and derivatives against finite differences and recorded native values; ChFiDS's data |
| **7L1** TopOpeBRepTool (the legacy-DS pilot) | The 21 classes of TopOpeBRepTool the fillets reach (TOOL, TOPOLOGY, CORRISO, REGUW, REGUS, CurveTool, FuseEdges, PURGE, makeTransition, ShapeTool, CLASSI, 2d, PROJECT, GEOMETRY, tol, HBoxTool, connexity, face, box...), extending phase 5's files. **Design:** the context for the data structure's shared state (2.1) | 19.0 | phase 6 | Unit tests on OCCT's own uses (classification, regularisation of faces and wires from the blend group's inputs) against recorded native results |
| **7P** HLR foundations | Intrv, TopBas, TopCnx, HLRAlgo, Contap | 15.3 | phase 6 | Unit tests: HLRAlgo's projector and edge status; Contap's contours on quadrics and B-splines against recorded native points |

### Wave 2

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **7B** ShapeUpgrade | The rest of ShapeUpgrade (ShapeDivide and its Angle, Area, Closed, ClosedEdges, Continuity variants, FaceDivide, WireDivide, EdgeDivide, the split tools, ConvertCurve2d/3dToBezier, ConvertSurfaceToBezierBasis, ShapeConvertToBezier, FixSmallCurves, FixSmallBezierCurves, RemoveInternalWires, ShellSewing, CombineToCylinder...), ShapeAlgo | 11.9 | 7A | GT: ShapeUpgrade_FaceDivide. Recorded: each divide tool on the heal group's public inputs |
| **7F** The 3D offset | `BRepOffset_MakeOffset` (15,500 lines in four files), `BRepOffsetAPI_MakeOffsetShape`, `MakeThickSolid` | 16.5 | 7E | GT: BRepOffset_MakeOffset (1,029 lines), BRepOffsetAPI_MakeThickSolid. Oracle: MakeThickSolid, MakeOffsetShape, MakeOffset (bound), section 3's cases |
| **7I** BRepFeat | All of BRepFeat (MakePrism, MakeDPrism, MakeRevol, MakePipe, MakeLinearForm, MakeRevolutionForm, MakeCylindricalHole, SplitShape, Gluer, Builder, Form, RibSlot...) | 16.8 | 7H | Oracle: `BRepFeat_MakeDPrism`, `SplitShape` (bound). Recorded: the other makers on the feat group's inputs |
| **7K** BRepBlend | All of BRepBlend (Walking, the lines, the surface and restriction tools, AppSurf/AppFunc, the CS and RS walkers, Extremity, PointOnRst...) | 19.3 | 7J | Unit tests: walking a constant-radius blend between two planes, a plane and a cylinder, two B-splines, step by step against recorded native steps |
| **7L2** TopOpeBRepDS | The 49 classes the fillets reach (DataStructure, HDataStructure, the interferences, Curve/Point/Surface data, the EXPORT, filters, reducers and processors, BuildTool, the iterators) | 23.0 | 7L1 | Unit tests: a data structure built by hand, its interference reduction against recorded native results |
| **7Q1** HLR data | HLRTopoBRep (all), HLRBRep's data model, curves, surfaces and intersector | 14.7 | 7P | Recorded: HLRBRep_Data on the hlr group's shapes |

### Wave 3

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **7C** ShapeProcess | ShapeProcess, ShapeProcessAPI, `Resource_Manager`; the operator library; OCCT's default resource files embedded (2.3) | 4.1 | 7B | Recorded: `ShapeProcessAPI_ApplySequence` with OCCT's default sequences on the heal group's inputs |
| **7G** Drafts and the rest of TKOffset | Draft, `BRepOffsetAPI_DraftAngle`, `MakeDraft`, `NormalProjection`, `FindContigousEdges`; `BRepFill_Draft`, `DraftLaw`, `AdvancedEvolved` (MakeEvolved's volume mode); `BRepAlgo_NormalProjection`; BiTgte | 13.2 | 7E | Oracle: DraftAngle (bound) on section 3's cases and build123d.NET's Draft and ExtrudeTaper inputs. Recorded: MakeDraft, BiTgte_Blend, AdvancedEvolved |
| **7M1** TopOpeBRepBuild's tools | GTopo, GTool, GIter, Pave, PaveSet, PaveClassifier, Area1d/3dBuilder, EdgeBuilder, FaceBuilder, SolidBuilder, ShellFaceSet, ShellFaceClassifier, ShellToSolid, Tools, Tools2d, FuseFace, CorrectFace2d, VertexInfo, extending phase 5's files | 9.7 | 7L2 | Unit tests: face and solid building from hand-made shape sets against recorded native results |
| **7Q2** HLR algorithms | The rest of HLRBRep (Algo, InternalAlgo, PolyAlgo, Hider, HLRToShape, PolyHLRToShape, the shape bounds and tools), HLRAppli | 14.2 | 7Q1 | Recorded: exact and polygonal HLR of the hlr group's shapes; the visible, hidden, outline and iso compounds compared as BRep text |

### Wave 4

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **7M2** TopOpeBRepBuild's builder | Builder (19,300 lines, its grid, merge, section, KPart, regularisation files), Builder1, BuilderON, HBuilder. Probably forked in two | 27.6 | 7M1 | Unit tests: HBuilder's Perform and MergeSolid on data structures recorded from native fillet runs (a recorder dumps the DS before `Perform`), the merged faces and solids compared with native's |
| **7N1** Fillet construction data | ChFiKPart (the analytic cases: plane/plane, plane/cylinder, cylinder/cylinder, cone, torus, sphere...), `ChFi3d_Builder_0` | 12.5 | 7K, 7L2 | Unit tests: each ChFiKPart case's surfaces and data written into the DS against recorded native data |

### Wave 5

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **7N2** The fillet builder | `ChFi3d_Builder` (its `_1`, `_2`, `_6`, `_C1`, `_C2`, `_CnCrn`, `_SpKP` files), `BRepAlgo_NormalProjection` if 7G hasn't merged | 21.8 | 7M2, 7N1 | Oracle: indirect (7N3); unit tests on stripes and corners from recorded native runs |

### Wave 6

| Unit | Content | k lines | Needs | Tests |
|---|---|---:|---|---|
| **7N3** Fillets and chamfers | `ChFi3d_FilBuilder`, `ChFi3d_ChBuilder`, `BRepFilletAPI_MakeFillet`, `MakeChamfer`, `LocalOperation`; FilletSurf | 12.8 | 7N2 | GT: BRepFilletAPI_MakeFillet (535 lines), MakeChamfer (249). Oracle: MakeFillet and MakeChamfer (bound), section 3's cases, histories and diagnostics (2.6) |

### Wave 7: harnesses

| Unit | Content | Needs | Tests |
|---|---|---|---|
| **7W** Draw-lite: phase 7 | BRepTest's fillet, chamfer, offset, draft, feature and helix commands; SWDRAW's ShapeUpgrade, ShapeCustom and ShapeProcess commands (`DT_*`, `expshape`, `directfaces`, `scaleshape`, `bsplres`...); HLRTest and `vcomputehlr`/`build3d` without a viewer (they compute and bind the result shapes); the Draw view commands `top`, `axo`, `front`, `v2d` as no-ops; the cheap gaps of 1.6 (question 28). May start with the healing, feature and helix commands as soon as their units merge | all of phase 7 | **The corpus milestone for phase 7:** per command, OCCT's log check |
| **7X** build123d.NET parity | 6M's replay extended to every phase 7 call build123d.NET makes (1.5); `Quantity_Color` and `Quantity_ColorRGBA`; the fillet, chamfer and offset benchmarks; phase 6's parallel Boolean benchmark re-measured on a quiet machine. May start each part as soon as its unit merges (7A, 7F, 7G, 7I first, 7N3 last) | 7A, 7F, 7G, 7I, 7N3 | **The build123d.NET milestone** (section 5) |

### Totals

| Stage | Units | Lines (k) |
|---|---|---:|
| Healing | 7A, 7B, 7C | 22.7 |
| Helix | 7D | 2.1 |
| Offsets and drafts | 7E, 7F, 7G | 46.1 |
| Features | 7H, 7I | 32.6 |
| Fillets | 7J, 7K, 7N1, 7N2, 7N3 | 89.6 |
| The legacy data structure | 7L1, 7L2, 7M1, 7M2 | 79.3 (≈ 72 new) |
| HLR | 7P, 7Q1, 7Q2 | 44.2 |
| Draw-lite, harnesses | 7W, 7X | ≈ 5 (C#) |

That is about **317,000 lines in 21 units** (about 310,000 new), plus the two harness units.

**The critical path** is the fillets: 7L1 → 7L2 → 7M1 → 7M2 → 7N2 → 7N3 → 7X. 7J → 7K → 7N1 run beside it. It is
also the path to build123d.NET's most used phase 7 call.

- **7A, 7E and 7L1 are the pilots.** Healing's copy-at-entry audit lands in 7A, the offset family's copy at entry
  (with `Locked(false)`) in 7E, and the data structure's context in 7L1. Each should be reviewed before 7B, 7F and
  7L2.
- **Six units can start at once:** 7A, 7D, 7H, 7J, 7L1 and 7P. 7E follows as soon as 7A has ported
  `ShapeCustom_Curve2d`, which 7A does first.
- **Off the critical path** and startable early: everything but the fillets. Healing, features and helix unblock the
  most corpus cases per line (1.6); offsets and drafts the most build123d.NET operations after fillets. HLR is last in
  value (no build123d.NET use, 157 corpus cases) and can run whenever a porter is free.

## 5. Milestones

1. **After each unit: its oracle.** Results byte-identical with native (the Free column apart) wherever native is
   repeatable, otherwise canonically identical or attributed; histories equal; frozen inputs never written; parallel
   equal to serial where the unit runs Booleans.
2. **After 7X: build123d.NET's phase 7 operations reproduce native results through BRep bytes.** Every fillet,
   chamfer, hollow, 3D offset, thickening, draft, tapered extrusion, DPrism, perimeter split and B-spline conversion
   that build123d.NET's tests make is replayed on the port from native's input bytes, with its `Fix` and `Clean`. The
   results, histories and failure diagnostics (faulty contours, stripe statuses, problematic shapes, exception types)
   equal native's. With phases 5 and 6 this is **build123d parity** (design.md): every native OCCT call build123d.NET
   makes has a port, but STEP (not called by build123d.NET today) and the viewer.
3. **After 7W: the corpus.** The 5,615 newly runnable cases (3,236 with data) and the cheap gaps' 885 (522) pass at
   DRAWEXE's rate, OCCT's TODOs apart.
4. **Performance** (2.7): fillets, chamfers and offsets within 2× native serial, or the gap explained.

## 6. Notes for porters

- **The earlier phases' notes apply** (phase3-plan.md section 5, phase4-plan.md section 6, phase5-6-plan.md section
  6, and handover.md's "Conventions that bite").
- **Extend, don't duplicate,** phase 5's TopOpeBRepBuild, DS and Tool files: their `// Port of:` lines name the
  members already ported.
- **The data structure's state** goes through the context of 2.1. Never a C# static for what OCCT keeps in a
  file-scope map.
- **Copy at entry** in every maker of 2.2; the frozen-input tests of phase 6 are the model.
- **Fillet history and diagnostics** are results, not extras (2.6).
- **ChFi3d is long and branchy:** `ChFi3d_Builder_C1.cxx` and `_CnCrn` are thousands of lines of corner cases. Port
  them file for file, keep OCCT's structure, and test them through the blend group's inputs, which reach them.
- **Debug code:** `OCCT_DEBUG` and `DRAW` blocks are left out, as before, with a comment where they were.

## 7. Questions for the owner

These are in [open-questions.md](open-questions.md) as 24 to 30, with the provisional answers the units would follow.

1. **The legacy data structure for fillets** (24). Port the TopOpeBRepBuild, DS and Tool classes the fillets reach,
   whole (about 79,000 lines, 72,000 new)? The alternatives are a member-level slice (perhaps 10 to 25% smaller, as
   2.11 of phases 5 and 6) or fillets rebuilt on BOPAlgo (not a port). Recommended: whole classes.
2. **HLR and helix** (25). build123d.NET uses neither (its helix is built from ported pieces). Keep both in phase 7?
   Recommended: helix yes (small, OCCT 8's GTests, 60 corpus cases); HLR yes but last, and deferrable if Bento has no
   use for drawings.
3. **ShapeProcess's resource files** (26). Embed OCCT's default resource files and read others only when given a path,
   rather than OCCT's environment variables? Recommended: yes.
4. **Bindings to widen the oracle** (27). Bind in build123d.NET (your repository): ShapeUpgrade's ShapeDivideAngle,
   ShapeDivideContinuity, ShapeDivideClosed and ShapeConvertToBezier; `ShapeProcessAPI_ApplySequence`; BRepFeat's
   MakePrism, MakeRevol and MakeLinearForm; `LocOpe_SplitShape`; `HLRBRep_Algo`, `PolyAlgo`, `HLRToShape`;
   `HelixBRep_BuilderHelix`; `BiTgte_Blend`; `FilletSurf_Builder`; `BRepOffsetAPI_MakeDraft`? Without them those are
   checked by native recorders and the corpus.
5. **Draw-lite's cheap gaps** (28). Add the non-phase-7 commands that would unblock 885 corpus cases (`lastrep` as a
   stub, `proj`, `shape`, `project`, `convert`, `projonplane`...) in 7W? Viewer commands stay out but for the setup
   no-ops a few cases need (`vzfit`, the Draw view commands). Recommended: yes.
6. **The corpus scan's default** (29). Make `FollowsCalledProcsOnly` (unit 5W) the default at the start of phase 7?
   It counts a case's procedures only where they are called, so the modalg grids' unused `OFFSETSHAPE` stops blocking
   them; it changes several existing tests' expected counts. This plan's counts assume it. Recommended: yes.
7. **Order of the stages** (30). The fillets are the critical path and build123d.NET's most used phase 7 call, but
   healing, features and helix unblock more corpus cases sooner. Start everything that can start (section 4's wave
   1), and give spare porters to the fillet path? Recommended: yes.
