# OCCT's bugs, triaged

Phase 10, unit 10K (decisions 15201 and 15202; phase10-plan.md 2.8). [occt-issues.md](occt-issues.md) lists every
OCCT bug the port reproduced so its results matched native OCCT. Divergence is now allowed, so each row is given one
of four classes, in a new last column of that file:

- **fix** (`fix K1` to `fix K8`): a user can see it through a public operation: wrong geometry or a wrong property, a
  spurious failure on valid input, a hang or crash, a file written wrong or read with data lost. The batch says which
  unit takes it.
- **touch** (fix when touched): real, but its effect stays inside an algorithm or needs misuse to reach (arrays not
  indexed from 1, a working object reused, invalid input that 10F turns into an error), or it lives in code a later
  unit rewrites anyway: copy at entry (10H), statics (10I), NCollection (10J), reports and exceptions (10F).
- **leave**: cosmetic, documentation, dead or latent code, declared but never defined, uninitialised in OCCT but
  deterministic in the port, native non-determinism the port doesn't have, test scripts, and Draw's command bugs
  (Draw-lite keeps OCCT's commands as the corpus harness, question 51).
- **settled**: the port already doesn't reproduce it: a hang fixed minimally, a guard against untrusted input, a copy
  instead of an in-place write, a port bug fixed. Nothing is left to do; the row stays as history.

The rules, and the cases between classes, are decision 15201's. This file is the triage's result; the row-by-row
verdict is the column in occt-issues.md.

## 1. Counts

1,154 rows in 116 tables. Thirteen rows of units 8D and 8E (lines 1964 and 1970 to 1981) repeat rows of units 8C1
and 8A2 and carry the same verdicts, so the **fix** list holds 133 distinct bugs.

| Area | Rows | fix | touch | leave | settled |
|---|---:|---:|---:|---:|---:|
| Geometry and maths | 204 | 38 | 99 | 55 | 12 |
| Extrema and projection | 25 | 4 | 14 | 7 | 0 |
| Intersections | 76 | 2 | 46 | 21 | 7 |
| Topology, I/O and checks | 81 | 6 | 43 | 12 | 20 |
| Meshing | 23 | 0 | 12 | 10 | 1 |
| Booleans | 63 | 1 | 44 | 7 | 11 |
| Healing and sewing | 95 | 4 | 54 | 17 | 20 |
| Construction | 156 | 14 | 82 | 40 | 20 |
| 2D constraints | 57 | 45 | 6 | 6 | 0 |
| Offsets, drafts and features | 65 | 2 | 38 | 15 | 10 |
| Fillets and chamfers | 67 | 3 | 31 | 29 | 4 |
| Hidden lines | 29 | 0 | 16 | 9 | 4 |
| Data exchange | 152 | 15 | 85 | 27 | 25 |
| Draw-lite and test scripts | 61 | 0 | 2 | 58 | 1 |
| **All** | **1,154** | **134** | **572** | **313** | **135** |

Areas follow the sections of occt-issues.md (each phase unit's section goes to one area), so a few rows sit in the
area of the unit that found them rather than the package they are in (bounding boxes under geometry, gp under
geometry, BRepGProp under topology).

| Batch | Content | Rows | Size |
|---|---|---:|---|
| K1 | Booleans, bounding boxes, distances | 5 | small |
| K2 | Hangs and spurious failures in modelling operations | 16 | medium |
| K3 | Wrong geometry from construction (2D fillets, chamfers, faces, fillings, features) | 10 | small to medium |
| K4 | 2D constraints (GccAna, Geom2dGcc): build123d.NET's constrained lines and arcs | 45 | large |
| K5 | Properties and approximation errors | 14 | small to medium |
| K6 | Extrema and projection | 7 | small |
| K7 | Curve and surface evaluation and conversion, gp | 21 | small each |
| K8 | Data exchange (STEP, STL, binary BRep) | 16 | small to medium |

Sizes: **S** under 20 lines and a test, **M** up to about 100 lines or one function's logic, **L** larger or needing
investigation first. Of open-questions.md item 10's examples, offset-curve D3 is in K7, AdvApp2Var's error estimate and
`GeomLProp::Continuity` in K5, and `Extrema_ExtCC` keeping old solutions in K6.

## 2. The fix list, by user impact

Batches are ordered by how much a user of Bento or build123d.NET would notice, and so is each batch's list. "Line" is
the row's line in occt-issues.md at this triage. "Site" is the `// OCCT` marker in kernel/src (paths from
`OcctNet.<assembly>/`); where none was found, the row says so. "Pinned by" names the tests that assert OCCT's wrong
value and must change with the fix; beyond them, any oracle comparison of the class becomes a 10A recording whose
expectation moves (section 4).

### K1. Booleans, bounding boxes, distances

A box too small lets a Boolean or a distance skip a pair that touches; a Boolean that misjudges a solid gives a wrong
solid. The fixes are small, but every Boolean recording that reaches them moves.

| Line | Bug | What a user sees | Site | Size | Pinned by |
|---:|---|---|---|---|---|
| 1134 | `BOPAlgo_BOP::BuildSolid` scans `i < aNb`, skipping the last face | A fuse, cut or common can keep or drop a solid wrongly when its one face shared with another solid is the map's last | ModelingAlgorithms/BOPAlgo/BOPAlgo_BOP.cs:1081 | S | none by name; Boolean recordings |
| 259 | `BndLib::Add` for parabola and hyperbola arcs: only the ends and the vertex | Edge boxes too small, so Booleans, distances and classification can miss a parabolic or hyperbolic edge | ModelingData/BndLib/BndLib.cs:950, 1015 | M | box recordings (BndLibOracleTests) |
| 263 | `GeomBndLib_Parabola2d::Box` | As above, for 2D arcs | ModelingData/GeomBndLib/GeomBndLib_Parabola2d.cs:89 | S | Parabola2d_TurnedArc_MissesCoordinateExtremum |
| 264 | `GeomBndLib_Parabola::Box` | As above, for `BRepBndLib` on 3D edges | ModelingData/GeomBndLib/GeomBndLib_Parabola.cs:95 | S | TurnedParabola_BoxMissesCoordinateExtremum |
| 265 | `Extrema_ExtCS` line against plane, torus and others reports the origin as the point on the curve | Distance between an edge and a face (BRepExtrema) gives the right distance but a wrong point | ModelingData/Extrema/Extrema_ExtCS.cs:219 | M | extrema recordings |

### K2. Hangs and spurious failures

Each makes a valid modelling call hang, take minutes, or fail. In the port a crash is an exception, which Bento shows
as an error; the user still gets no shape.

| Line | Bug | What a user sees | Site | Size | Pinned by |
|---:|---|---|---|---|---|
| 512 | `ShapeAnalysis_Edge::CheckOverlapping`: step 0 on a degenerated edge | Healing (ShapeFix) hangs on a shape with a zero-length edge near another edge | ModelingAlgorithms/ShapeAnalysis/ShapeAnalysis_Edge.cs:734 | S | not pinned (it hangs); the oracle skips such edges |
| 702 | `ShapeConstruct::ConvertSurfaceToBSpline` retries C0 without end | `ShapeCustom::ConvertToBSpline` and healing operators hang when an approximation keeps failing | ModelingAlgorithms/ShapeConstruct/ShapeConstruct.cs:240 | S | none |
| 1087 | `GeomFill_CorrectedFrenet::InitInterval`: the step has no lower bound | A sweep (MakePipeShell's default trihedron) along an offset curve takes seconds to over ten minutes | no line marker found | M | the oracle leaves those spines out (decision 811) |
| 1181, 1182 | `GeomFill_Profiler` with all-periodic sections of different knots | A loft (ThruSections, approximated) of closed B-spline sections fails; natively it corrupts the heap | no line marker found | M | Profiler_on_periodic_sections_throws_where_native_corrupts_its_heap |
| 1296 | `BRepOffsetAPI_MakeOffset::BuildDomains` (an access violation natively) | 2D offset of a face or wire reports "not done" | ModelingAlgorithms/BRepOffsetAPI/BRepOffsetAPI_MakeOffset.cs:451 (the catch) | M | offset recordings |
| 1121 | `MAT2d_Mat2d::CreateMat` | 2D offset fails in the medial axis | ModelingAlgorithms/MAT2d/MAT2d_Mat2d.cs:942 | M | none |
| 1122 | `MAT2d_Tool2d::BisecFusion` | 2D offset of a holed face with Bézier sides fails | ModelingAlgorithms/MAT2d/MAT2d_Tool2d.cs:914 | M | none |
| 1552, 1763 | `ChFiDS_FilSpine::SetRadius(Law, IinC)` drops the law | A variable-radius fillet given by a law throws at Build | no line marker (doc comment, ChFiDS/ChFiDS_FilSpine.cs:215) | S | TheEvolvingRadii (BRepFilletAPITests) |
| 1240 | `BRepFill_Filling::AddConstraints`: C1 becomes G2, G2 throws | MakeFilling with G2 boundaries always fails; build123d.NET's `MakeSurfacePatch` at its C2 level can't work | no line marker found | S | MakeFilling recordings (10 cases) |
| 1565 | `Draft_Modification`, some drafts of several faces (an access violation natively) | A draft angle (BRepOffsetAPI_DraftAngle) fails in 24 of 870 recorded runs | ModelingAlgorithms/Draft/Draft_Modification_1.cs:680, 924, 1123 | L | draft recordings |
| 1389, 1390 | `ShapeCustom_SweptToElementary::NewCurve2d` | The SweptToElementary healing operator (STEP import's ShapeProcess) fails on shapes with such faces | ModelingAlgorithms/ShapeCustom/ShapeCustom_SweptToElementary.cs:219, 225 | M | none by name |
| 1307 | `BRepFill_Pipe::Edge` through `MakePipe::Generated(spine edge)` | The history of a pipe fails for a vertex profile | ModelingAlgorithms/BRepFill/BRepFill_Pipe.cs:479 | S | pipe recordings |
| 1156 | `GeomFill_Pipe::Init(Path, FirstSect, LastSect)` | A pipe evolving between two sections fails on most 3D paths (27 of 30) | ModelingAlgorithms/GeomFill/GeomFill_Pipe.cs:572 | L | a GeomFill test and the recordings |

### K3. Wrong geometry from construction

| Line | Bug | What a user sees | Site | Size | Pinned by |
|---:|---|---|---|---|---|
| 1204 | `ChFi2d_AnaFilletAlgo::isCW` evaluates the middle point without the edge's location | 2D fillets of a wire (ChFi2d_FilletAPI, build123d.NET's wire fillets) go the wrong way on a moved arc | ModelingAlgorithms/ChFi2d/ChFi2d_AnaFilletAlgo.cs:55 | S | none |
| 1205 | `ChFi2d_AnaFilletAlgo::Perform`: e1's orientation decides e2's ends | Wrong trimming when the two edges' orientations differ | ModelingAlgorithms/ChFi2d/ChFi2d_AnaFilletAlgo.cs:510 | S | none |
| 1198 | `ChFi2d_Builder::BuildNewWire` | A 2D fillet or chamfer of a face with holes returns one wire holding outer and inner edges (build123d.NET chamfers each wire alone to avoid it) | ModelingAlgorithms/ChFi2d/ChFi2d_Builder.cs:565 | M | ChFi2d recordings |
| 1666 | `ChFiKPart_ComputeData_ChAsymPlnCyl::MakeChAsym`: coordinate-wise product for a dot product | A distance-angle chamfer between a plane and a cylinder whose axis isn't along X, Y or Z is built from a wrong point | ModelingAlgorithms/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnCyl.cs:314 | S | none |
| 455 | `BRepTools_Modifier::Rebuild` with NurbsConvert on a meshed shape | NurbsConvert or GTransform of a shape that was meshed (as Bento's are, for display) gives its B-spline and Bézier faces no surface | ModelingData/BRepTools/BRepTools_Modifier.cs:286 | M | modifier recordings |
| 406 | `BRepLib::ExtendFace`, general branch: surface without location | An extended located face loses its placement | ModelingAlgorithms/BRepLib/BRepLib.cs:3258 | S | none |
| 407 | `BRepLib::ExtendFace` tests aSVMax for the VMin extension | VMin is extended when VMax is finite | ModelingAlgorithms/BRepLib/BRepLib.cs:3290 | S | none |
| 1175 | `GeomFill_Coons::Init`, rational form | Surfaces filled from four rational boundaries (GeomFill_BSplineCurves, BezierCurves) get two boundaries' weights swapped | ModelingAlgorithms/GeomFill/GeomFill_Coons.cs:154 | S | a GeomFill_FillingTests case |
| 1586 | `BRepFeat_MakeRevol::PerformUntilAngle`: branches the wrong way round | A revolved feature up to an angle cuts the until face and glues the support | ModelingAlgorithms/BRepFeat/BRepFeat_MakeRevol.cs:595 | S | none |
| 1157 | `GeomFill_SweepSectionGenerator::Perform`, types 1 to 3 | Sections stay where the first one is; only direct users of the class (GeomFill_Pipe uses type 0) | ModelingAlgorithms/GeomFill/GeomFill_SweepSectionGenerator.cs:290 | M | a GeomFill_SweepTests case |

### K4. 2D constraints

build123d.NET's constrained lines and arcs (`ConstrainedLines.cs`: Circ2d3Tan, Circ2d2TanRad, Circ2d2TanOn,
Circ2dTanOnRad, Circ2dTanCen, Lin2d2Tan, Lin2dTanObl) and its wire fillets call Geom2dGcc, which hands lines, circles
and points to GccAna. The rows report wrong qualifiers, missed or wrong solutions, wrong tangency points and centres,
and exceptions (Standard_OutOfRange in 189 of 1,500 random two-circle cases; every generic line-and-circle case of
Circ2dTanOnRad throws). One unit should take the whole family, class by class, with property tests (each solution
tangent to its arguments, centred where asked) replacing the oracle's recordings.

| Lines | Class | Rows | Markers | Size |
|---|---|---:|---:|---|
| 969 to 976 | GccAna_Circ2d3Tan (`_1`, `_3`, `_5`) | 7 | 14 | L |
| 977 to 985 | GccAna_Circ2d2TanOn (`_1` to `_10`) | 9 | 24 | L |
| 987 to 990 | GccAna_Circ2d2TanRad (`_4`) | 4 | 4 | M |
| 991 to 996 | GccAna_Circ2dTanOnRad (`_1` to `_5`) | 6 | 10 | M |
| 997 to 999 | GccAna_Lin2d2Tan, Lin2dTanObl, Lin2dTanPer | 3 | 17 | M |
| 1050 to 1059 | Geom2dGcc_Circ2d2TanOn, Circ2d2TanOnIter, Circ2d2TanOnGeo, Circ2d2TanRad | 6 | 32 | L |
| 1061 to 1064 | Geom2dGcc_Circ2d3TanIter | 4 | 10 | M |
| 1066 to 1071 | Geom2dGcc_Circ2dTanOnRadGeo, Geom2dGcc_Lin2dTanObl | 6 | 10 | M |

Pinned by: the Geom2dGcc and GccAna unit tests (`*_ThrowAsOCCT`, "pinned by a test" in the rows) and
Geom2dGccOracleTests' recordings, which already mask fields native leaves unset.

### K5. Properties and approximation errors

| Line | Bug | What a user sees | Site | Size | Pinned by |
|---:|---|---|---|---|---|
| 216 | `GeomLProp::Continuity` compares too few derivatives at a knot | A C2 junction is reported C1, a C1 one C0 | ModelingData/GeomLProp/GeomLProp.cs:85 | S | Continuity_at_a_BSpline_knot_is_reported_lower_as_in_OCCT |
| 345 | `BRepGProp_Gauss::convert` | Volume properties against a plane (BRepGProp's `VolumeProperties` with a gp_Pln) report a null volume and centre | ModelingAlgorithms/BRepGProp/BRepGProp_Gauss.cs:492 | S | Vinert_by_plane_returns_a_null_mass_as_OCCT_does |
| 346 | `BRepGProp_Face::VKnots` on planes, cylinders, cones | Adaptive (Eps) area and volume of natural faces subdivide V from U values: less accurate | ModelingAlgorithms/BRepGProp/BRepGProp_Face.cs:409 | S | Face_tool_matches_native_OCCT |
| 215 | `LProp_SurfaceUtils`: curvatures asked before `IsCurvatureDefined` | Stale curvatures (0, then the previous point's); `IsUmbilic` likewise | ModelingData/GeomLProp/GeomLProp_SLProps.cs:322; LProp/LProp_SLProps3d.cs:322 | M | none by name |
| 176 | `AdvApp2Var_ApproxF2var::mma2ce1_` writes the U maxima to the V slot | Approximated surfaces (GeomConvert_ApproxSurface, conversions, offsets) report a `MaxError` far below the real error; degree reduction is unchecked | ModelingData/AdvApp2Var/AdvApp2Var_ApproxF2var.cs:3729 | S, wide effect | SinCos_C0_ReproducesOcctsUnderestimatedError; every recording through AdvApp2Var |
| 184 | `AppCont_LeastSquare`: `IsDone` false with free ends | A continuous approximation (Approx_ComputeCLine) with free ends reports "not done" | ModelingData/AppCont/AppCont_LeastSquare.cs:259 | S | AppCont_LeastSquareTests |
| 127 to 134 | `GProp_CelGProps`, `SelGProps`, `VelGProps` (8 rows) | Wrong inertia, centres, cone area and volume, cylinder axial inertia, torus centre. No OCCT algorithm uses them: fix, or keep them out of the public API (10D) and mark the rows leave | ModelingData/GProp/GProp_CelGProps.cs, GProp_SelGProps.cs, GProp_VelGProps.cs (26 markers) | M | none by name |

### K6. Extrema and projection

| Line | Bug | What a user sees | Site | Size | Pinned by |
|---:|---|---|---|---|---|
| 315 | `ProjLib_ComputeApprox`, cylinder bounds test always true | The pcurve of a helix of more than one turn on a cylinder (threads) is wrong by up to the diameter | ModelingData/ProjLib/ProjLib_ComputeApprox.cs:146 | S | Cylinder_HelixOfMoreThanATurn_WrongResult_OcctBug |
| 311 | `ProjLib_CompProjectedCurve::Perform` | Projecting a curve on a surface: a free part after an isoparametric one gets the iso line as its pcurve | ModelingData/ProjLib/ProjLib_CompProjectedCurve.cs:1234 | S | Perform_IsoThenFreePart_ReusesIsoLine_OcctBug |
| 205 | `Extrema_GGenExtCC::Perform` appends to the last run's solutions | Curve-curve extrema reused (Extrema_ExtCC, the 2D twin) report old solutions too | ModelingData/Extrema/Extrema_ECC.cs:103; Extrema_ECC2d.cs:103 | S | ECC_PerformTwice_AccumulatesSolutions_AsInOcct |
| 328 | `GeomAPI_ExtremaCurveCurve::TotalPerform` | `TotalLowerDistance` can be a farther pair's (√8 for √2) | ModelingAlgorithms/GeomAPI/GeomAPI_ExtremaCurveCurve.cs:295 | S | ExtremaCurveCurve_TotalReachesTheEnds |
| 329 | `GeomAPI_ExtremaCurveCurve::Init` with ranges | The total methods use the whole curves' ends | ModelingAlgorithms/GeomAPI/GeomAPI_ExtremaCurveCurve.cs:90 | S | ExtremaCurveCurve_TotalOnRangesReportsTheCurveEnds |
| 97 | `IntAna2d_AnaIntersection` circle/circle on an indirect circle | Wrong parameters of 2D circle intersections | ModelingData/IntAna2d/IntAna2d_AnaIntersection.cs:256 | S | a unit test; intersection recordings |
| 319 | `ProjLib_ProjectOnPlane::Load`, parabola | A parabola projected on a plane gets the wrong focal | ModelingData/ProjLib/ProjLib_ProjectOnPlane.cs:755 | S | Parabola_TiltedAboutItsXAxis_WrongFocal_OcctBug |

### K7. Curve and surface evaluation and conversion

| Line | Bug | What a user sees | Site | Size | Pinned by |
|---:|---|---|---|---|---|
| 70 | `BSplCLib::IncreaseDegree`, periodic curves | Raising the degree of most periodic B-splines changes the curve (lofts' profiles, fillets' spines) | Foundation/BSplCLib/BSplCLib.cs:2586 | M to L | none by name |
| 199 | `GeomAdaptor_SurfaceOfLinearExtrusion::VTrim`, `UTrim` | Trimming an extrusion adaptor trims the wrong direction: intersections and projections on extruded faces | ModelingData/GeomAdaptor/GeomAdaptor_SurfaceOfLinearExtrusion.cs:135 | S | a unit test |
| 110, 232 | Offset curves' D3 (`CalculateD3`) and `EvaluateDN` | Third derivatives of offset curves wrong, and different through the adaptors | ModelingData/Geom/Geom_OffsetCurveUtils.cs:220, 243, 546; Geom2d/Geom2d_OffsetCurveUtils.cs:500, 601 | M | OcctIssue_OffsetCurveEvalD3_* (Geom and Geom2d port tests) |
| 156 | `Adaptor3d_CurveOnSurface::EvalD3` | D3 of a curve on a surface off by u'·Su and more | ModelingData/Adaptor3d/Adaptor3d_CurveOnSurface.cs:90 | S | EvalD3_OcctFormula_DiffersFromDerivative |
| 197 | `Geom_OffsetSurface::TransformParameters` transforms twice | Located offset surfaces under a scaling: parameters off by the scale | ModelingData/Geom/Geom_OffsetSurface.cs:735 | S | TransformParameters_Transforms_Twice_With_An_Equivalent_Surface |
| 24 | `ElSLib::TorusDN`, mixed orders | Some mixed derivatives of a torus have the wrong sign | Foundation/ElSLib/ElSLib.cs:558 | S | TorusDN_reproduces_OCCT_sign_error |
| 84 | `BSplSLib::CacheD2`, rational, both degrees below 2 | Second derivatives (curvature) of rational bilinear patches wrong | no line marker found | S | none |
| 237 | `GeomConvert::SplitBSplineSurface` in V | Reverses U instead of V | ModelingData/GeomConvert/GeomConvert.cs:1570 | S | SplitBSplineSurface_InV_ReversesU_OCCT |
| 238 | `GeomConvert_BSplineSurfaceKnotSplitting` | V splits written into U; the V queries throw | ModelingData/GeomConvert/GeomConvert_BSplineSurfaceKnotSplitting.cs:99, 138 | S | KnotSplitting_SmoothV_ReproducesOcctError |
| 241 | `GeomConvert_CurveToAnaCurve::ComputeEllipse` loses the rotation's sign | Canonical recognition rejects most ellipses | ModelingData/GeomConvert/GeomConvert_CurveToAnaCurve.cs:494 | S | ComputeEllipse_MajorAxisAt60Degrees_ReproducesOcctError |
| 286 | `GeomConvert_SurfToAnaSurf::TryCylinerCone` | A B-spline converted to a cone gets its axis through the wrong point | ModelingData/GeomConvert/GeomConvert_SurfToAnaSurf.cs:181 | S | none |
| 148 | `Geom_ConicalSurface::Coefficients` | Wrong implicit equation unless the cone sits at the origin with radius 0 | ModelingData/Geom/Geom_ConicalSurface.cs:192 | S | a unit test |
| 229 | `Geom2dGridEval_Parabola` ignores the frame's sense | Grid evaluation of an indirect 2D parabola is mirrored | ModelingData/Geom2dGridEval/Geom2dGridEval_Parabola.cs:43, 80 | S | IndirectParabola_GridIsMirrored |
| 15 to 21 | gp: `gp_Dir::Mirror(gp_Ax1)`, `gp_Mat2d` squaring in place, `gp_Trsf2d::Multiply` and `Power`, `gp_Hypr2d` asymptotes and second directrix, `gp_GTrsf::Power` below -1 (7 rows) | Wrong mirrored directions, 2D transformation powers, hyperbola asymptotes and directrix: gp becomes Kerf.Geometry, the public value types | Foundation/gp/gp_Dir.cs:142; gp_Mat2d.cs:230; gp_Trsf2d.cs:360, 717; gp_GTrsf2d.cs:216; gp_Hypr2d.cs:173, 188, 273; gp_GTrsf.cs:278 | S each | gp tests; recordings through gp |

### K8. Data exchange

| Line | Bug | What a user sees | Site | Size | Pinned by |
|---:|---|---|---|---|---|
| 1790 | `RWStl::ReadStream`, `StlAPI_Reader::Read(stream)` | A binary STL whose header starts "solid" (common from other tools) fails to read from a stream | no line marker (doc comment, DataExchange/RWStl/RWStl.cs:18) | S | RWStlTests |
| 1787 | `RWStl_Reader::Read`, multi-domain ASCII | Solids after the first are lost in files under 1 KB; joined numbers at block boundaries | no line marker found | M | RWStlTests |
| 1906, 1912 | `RWStepVisual_RWSurfaceStyleRenderingWithProperties::ReadStep`; reflectance entities 825 and 826 | Rendering methods of 545 corpus records read wrong; reflectance styles can't be read or written | no line marker (generated code: fix in stepgen) | S | none |
| 1908 | `StepVisual_StyledItem::Item` | A style on a representation is written with `$`: colours lost on write (68 records) | no line marker found | M | none |
| 1909 | `RWStepAP214_ReadWriteModule::IsComplex` | Some complex records are written `?()`: an invalid file (30 records) | no line marker found | M | IsComplexMissesTwoComplexCasesAsOcctDoes |
| 1896, 1964 | `RWStepRepr_RWBooleanRepresentationItem::WriteStep` | BOOLEAN_REPRESENTATION_ITEM written 1 or 0, unreadable | no line marker found | S | StepRoundTrip.cs |
| 1932 | `IFSelect_WorkSession::ReadFile` with StepSelect_WorkLibrary | The generic `XSControl_Reader::ReadFile` fails on STEP (STEPControl_Reader works) | DataExchange/IFSelect/IFSelect_WorkSession.cs (8 markers) | M | none |
| 1960 | `GeomToStep_MakeCurve` (Geom2d) | A 2D trimmed curve can't be written (StdFail_NotDone) | DataExchange/GeomToStep/GeomToStep_MakeCurve.cs:148 | S | GeomToStepTests |
| 1961 | `GeomToStep_MakeBoundedCurve` (Geom2d) | A rational 2D Bézier loses its weights on write | DataExchange/GeomToStep/GeomToStep_MakeBoundedCurve.cs:112 | S | none |
| 1962 | `GeomToStep_MakeAxis2Placement2d(gp_Ax2)` | An axis2_placement_2d with 3D coordinates | DataExchange/GeomToStep/GeomToStep_MakeAxis2Placement2d.cs:29 | S | STEP recordings |
| 1963 | `GeomToStep_MakeRectangularTrimmedSurface`, `MakeConicalSurface` | Angular bounds written in degrees read back as radians | DataExchange/GeomToStep/GeomToStep_MakeRectangularTrimmedSurface.cs:44 | S | none |
| 1802 | step.lex: a string of 16,383 bytes or more | Such a STEP file fails to read | DataExchange/StepFile/StepFile_Scanner.cs:624 | M | none |
| 1803 | step.lex: the end of a string | `'a'` followed by a tab and a comma runs on to a later quote: a misread file | DataExchange/StepFile/StepFile_Scanner.cs:209 | S | none |
| 367 | `BinTools_ShapeSet` point on surface | Parameters swapped on every binary BRep round trip | ModelingData/BinTools/BinTools_ShapeSet.cs:669 | S | Point_representations_follow_OCCT |

## 3. What "touch" covers

572 rows. A unit rewriting a module fixes its touch rows as it goes, with the same recording rules (section 4). The
largest groups:

- **Intersection and Boolean internals** (IntPolyh, IntPatch, IntWalk, IntTools, BOPDS, BOPAlgo's pave filler):
  parity-sensitive code whose bugs change which lines and paves are found. A fix moves Boolean results across the
  corpus without a shown user benefit, so these wait for a module rewrite or for a case that shows them.
- **Working objects reused** (a second `Perform`, `Init` or `Load` keeping the first's state): 10L's public API makes
  operations single-use results, which removes most of them.
- **Arrays not indexed from 1**, unchecked indexes and null dereferences on invalid input: 10J's spans and 10F's
  results replace them.
- **Healing** (ShapeAnalysis, ShapeFix, ShapeUpgrade, ShapeCustom): status flags, analysis details and conversions;
  10H4 rewrites healing for the overlay.
- **Approximation internals** (AppParCurves, AppDef, FEmTool, AppBlend): constrained and mixed 2D/3D cases OCCT's
  callers don't reach.
- **Data exchange's framework** (Interface, IFSelect, Transfer, StepData): check lists, selections and graph tools.

A touch row becomes **fix** when a user-visible case is found: a bug report, a corpus case, a build123d.NET test.

## 4. How a fix is recorded

1. **One row, one change.** The fix, its test and its record go in one commit (or a few, for one row), never mixed
   with renames or overlay work (phase10-plan.md 5). A batch is one unit.
2. **The code:** the `// OCCT` marker goes (design-rules.md 11). Where the fix needs explaining, a plain comment
   says what the code does and cites the decision.
3. **The test:** the test that pinned OCCT's value (the "Pinned by" column; also search the class's tests for `Occt`,
   `OCCT`, `Reproduces`, `AsIn`) is rewritten to assert the corrected behaviour against an independent reference: a
   finite difference, an analytic value, an invariant (a box containing sampled points, a solution tangent to its
   arguments). Never against a value recorded from native OCCT.
4. **The recordings** (10A): every recorded comparison the fix moves drops to "geometrically equal" or, where the
   shape changes, "intended difference" with a reason that cites the occt-issues.md row and the decision. A
   Draw-lite corpus case whose OCCT reference changes gets an override with the same reason (plan 3.3). Units
   fixing rows check the recordings of their area before and after, and list what moved.
5. **occt-issues.md:** the row stays, as history; its Triage cell becomes `fixed (decision N)`. Nothing else in the
   row changes.
6. **decisions.md and the CHANGELOG:** a decision per batch (per row where a choice was made), a CHANGELOG paragraph
   per batch in plain words: what users get that they didn't.
7. **build123d.NET:** where its code works around the bug (chamfering each wire alone, K3; `MakeSurfacePatch`'s G2,
   K2; `MakeThickSolid`'s null check, a touch row), the workaround goes in the same unit, with modeling/CHANGELOG.md.
8. **After the rename (10E)** the sites carry .NET names; the applied name map finds them from the OCCT names used
   here. The markers keep their text through the rename.

## 5. The `// OCCT` markers

The code has no distinct marker for a bug. "A comment starting `// OCCT`" also starts porting notes ("OCCT iterates
this map by address…", "OCCT copies the adaptor…"), so markers and rows were matched by class name, and bug notes told
apart from porting notes by their wording. The counts are approximate.

- **Markers:** 1,989 line comments starting `// OCCT` (not `// OCCT.NET`, `// OCCT_DEBUG`, `// OCCT's`, nor doc
  comments) in the repository: 1,617 in kernel/src, 344 in the kernel's tests and Draw-lite, 16 in the modelling
  layer, 1 in stepgen. Of kernel/src's 1,617, about 1,080 read as bug notes and about 540 as porting notes.
- **Rows:** 778 rows have a bug note in a file of a class they name; 116 only porting notes there; 201 name a class
  whose files have no `// OCCT` comment (appendix A: settled rows, rows marked in doc comments, generated STEP code,
  and some whose marker was never placed or has gone); 59 name no class the code has (Draw commands, test scripts,
  file-static functions).
- **Markers without a row:** 126 bug-like markers in 95 kernel/src files whose class occt-issues.md never names
  (appendix B). Many are porting notes worded like bugs, or bugs listed under a caller's or generic's name; the rest
  need a row. Not triaged here: they keep OCCT's behaviour until a unit touching the file decides.

A fixer finds a row's sites by searching the class's files for `// OCCT` and the row's wording, not by these counts.

## Appendices

Generated from occt-issues.md and the code at this triage.

### A. Rows whose marker wasn't found

A row is listed when the class it names has a source file but no `// OCCT` line comment in any of that class's files. `settled` rows are expected here: their marker went with the port's fix.

| Line | Where | Triage |
|---:|---|---|
| 191 | `AppDef_LinearCriteria::Gradient` (AppDef_LinearCriteria.cxx) | settled |
| 210 | `Extrema_GFuncExtPC::Value` and `SearchOfTolerance` (Extrema_GFuncExtPC.hxx) | leave |
| 215 | `LProp_SurfaceUtils::RequireCurvature`, `IsUmbilic` (GeomLProp_SurfaceUtils.hxx), calle… | fix K5 |
| 217 | `LProp_CurveUtils::CentreOfCurvature` (LProp_CurveUtils.hxx) | touch |
| 252 | `Extrema_ExtSS` on parallel surfaces (Extrema_ExtSS.cxx) | touch |
| 253 | `Extrema_ExtSS::Perform` | touch |
| 331 | `GeomAPI_ExtremaCurveSurface::Init` (GeomAPI_ExtremaCurveSurface.cxx) | leave |
| 337 | `BRepTools_GTrsfModification::NewTriangulation` (BRepTools_GTrsfModification.cxx) | touch |
| 338 | `BRepTools_GTrsfModification::NewPolygonOnTriangulation` | touch |
| 340 | `BRepTools_History::AddModified`/`prepareModified` (BRepTools_History.cxx) | touch |
| 358 | `TopTools_LocationSet::Read` (TopTools_LocationSet.cxx) | settled |
| 371 | BinTools readers: `ShapeReader` references and locations, `LocationSet::Read` | settled |
| 390 | `tcopy` (BRepTest_BasicCommands.cxx) | leave |
| 392 | `nexplode` (DBRep.cxx) | leave |
| 435 | `isbbinterf` (BRepTest_BasicCommands.cxx) | leave |
| 436 | `checksection` (BRepTest_CheckCommands.cxx) | leave |
| 437 | `checkshape` structural dump (`StructuralDump`, BRepTest_CheckCommands.cxx) | leave |
| 465 | `BRepPrimAPI_MakeCylinder`, `MakeSphere`, `MakeTorus` (documentation) | touch |
| 468 | `pcone`, `psphere`, `ptorus` (BRepTest_PrimitiveCommands.cxx) | leave |
| 476 | `setorigin` (GeomliteTest_CurveCommands.cxx) | leave |
| 477 | `2dbeziercurve` (GeomliteTest_CurveCommands.cxx) | leave |
| 478 | `extsurf`, `revsurf`, `trim` (GeomliteTest_SurfaceCommands.cxx), `gcarc ... cir` (Geome… | leave |
| 479 | `gcarc` (GeometryTest_ConstraintCommands.cxx) | leave |
| 481 | `visoedge` (BRepTest_CurveCommands.cxx, `isoedge`) | leave |
| 482 | `vertex name P` (BRepTest_CurveCommands.cxx), `mksurface`, `pcurve edge face` (BRepTest… | leave |
| 483 | `length` (GeomliteTest_CurveCommands.cxx) | leave |
| 485 | `proximity`, `selfintersect` (BRepTest_ExtremaCommands.cxx; a property, not a bug) | leave |
| 486 | lowalgos/extss/bug29712_44 (a test, not the library) | leave |
| 497 | `tricheck` (MeshTest_PluginCommands.cxx), the check of the meshes inside the faces | leave |
| 498 | `MeshTest_CheckTopology::Perform`, cross-face errors | leave |
| 500 | `trmergenodes` without `-oneFace` (MeshTest.cxx) | leave |
| 501 | `correctnormals` (MeshTest.cxx) | leave |
| 502 | `triepoints` (MeshTest.cxx; a property, not a bug) | leave |
| 520 | `ShapeAnalysis::TotCross2D` | touch |
| 533 | `NCollection_MapAlgo::Differ` (NCollection_MapAlgo.hxx; a property, not a bug) | leave |
| 552 | `CDelaBella::Triangulate` (delabella.cpp), all points cocircular | touch |
| 553 | `CDelaBella::Triangulate`, 0 points | touch |
| 554 | `CDelaBella::Triangulate`, `assert(0)` and `assert(_f != face_alloc)` | leave |
| 555 | `IDelaBella::Triangulate` (a property, not a bug) | leave |
| 561 | `BRepMesh_CircleInspector::Inspect` (OCCT's own comment) | leave |
| 573 | `BRepMesh_BaseMeshAlgo::Perform` (a property) | touch |
| 582 | `incmesh` (MeshTest.cxx), argument parsing | leave |
| 585 | `StdPrs_ToolTriangulatedShape::GetDeflection` (a property) | leave |
| 586 | mesh group, standard_shading/M4 (a test reference) | leave |
| 597 | BRepLib_ToolTriangulatedShape::ComputeNormals (a property) | leave |
| 648 | `IntStart_SearchOnBoundaries.gxx`, `BoundedArc` (a site, decision 610) | leave |
| 649 | `IntImp_ZerImpFunc`, `IntImp_ZerParFunc`, `IntPatch_CSFunction` (a property, decision 608) | leave |
| 650 | `IntSurf_Transition::SetValue()` and `SetValue(Tangent, Type)` (a property) | leave |
| 664 | BOPDS_FaceInfo::Clear | touch |
| 668 | BOPAlgo_ToolsProvider::SetTools | touch |
| 730 | `Geom2dHatch_Element` default constructor | leave |
| 742 | `IntCyCo`, the `IntAna_Point` case | touch |
| 744 | `WorkWithBoundaries::BoundariesComputing`, `mB < 0` | touch |
| 745 | `WorkWithBoundaries::BoundariesComputing`, `mB > 0` | leave |
| 776 | `IntWalk_PWalking` (the constructor with the bounds; a property) | leave |
| 787 | `ShapeFix_EdgeProjAux::Init2d`, `UpdateParam2d` | settled |
| 788 | `ShapeFix_Edge::FixReversed2d` | settled |
| 793 | `ShapeFix_Edge::FixAddPCurve`, seam branch | settled |
| 827 | `IntTools_TopolTool::SamplePoint` | touch |
| 878 | `ShapeFix_Shape::Perform` | touch |
| 904 | `IntPatch_WLineTool::ComputePurgedWLine` | touch |
| 909 | Draw's `intersect`, curve/surface branch (GeometryTest_CurveCommands.cxx) | leave |
| 923 | `IntTools_CurveRangeLocalizeData::ListRangeOut` (decision 629) | leave |
| 1012 | `BOPAlgo_MPC::Perform` / file-static `UpdateVertices` (PaveFiller_7) | settled |
| 1035 | `TopOpeBRepBuild_Merge.cxx`, `CorrectUnclosedWire` | settled |
| 1125 | `MAT2d_CutCurve` | leave |
| 1139 | `BOPAlgo_ToolsProvider::SetTools` | touch |
| 1181 | `GeomFill_Profiler::UnifyByInsertingAllKnots` | fix K2 |
| 1182 | `GeomFill_Profiler::Perform` | fix K2 |
| 1233 | `Plate_LinearXYZConstraint`, `Plate_LinearScalarConstraint` (from arrays) | leave |
| 1240 | `BRepFill_Filling::AddConstraints` (through MakeFilling's `Add(edge, face, order)`) | fix K2 |
| 1241 | `BRepFill_Filling::Build` | touch |
| 1242 | `BRepFill_Filling::G0Error()` and the others before `Build` | touch |
| 1252 | `BRepTools_History::Merge` | leave |
| 1253 | `BRepAlgoAPI_BuilderAlgo::SimplifyResult` | settled |
| 1263 | `BOPAlgo_CheckerSI` (through PaveFiller_6's `PutPaveOnCurve` and `MakeBlocks`) | settled |
| 1265 | `bopapicheck` (BOPTest_CheckCommands.cxx) | leave |
| 1306 | MakePipeShell with an auxiliary spine and `BRepFill_Contact` on polyline spines (the si… | touch |
| 1318 | `BRepOffsetAPI_MiddlePath::Build` | leave |
| 1332 | `BRepProj_Projection::BuildSection` | settled |
| 1348 | BRepProj_Projection::BuildSection (through build123d.NET's Project and ProjectToSh… | settled |
| 1356 | BOPTest_TolerCommands `ReduceFaceTolerance` (`btolx`) | leave |
| 1357 | BOPTest_TolerCommands `breducetolerance` | leave |
| 1358 | BOPTest_DebugCommands `bopfsd` | leave |
| 1359 | BOPTest_DebugCommands `bopindex` | leave |
| 1360 | BOPTest_DebugCommands `bopwho`, `bopsd`, `bopfon`/`bopfin`/`bopfsc`, `bopfav` | leave |
| 1361 | SWDRAW_ShapeAnalysis `fbclose` | leave |
| 1362 | SWDRAW_ShapeAnalysis `statshape` | leave |
| 1363 | SWDRAW_ShapeAnalysis `fbprops` | leave |
| 1365 | SWDRAW_ShapeFix `fixshape +?` | leave |
| 1366 | SWDRAW_ShapeAnalysis `fbprops` on a 10×10 square | leave |
| 1367 | bugs/heal/bug29382_3 (UnifySameDomain with `+b` on a Bézier prism's common with a box) | leave |
| 1375 | `qcircle`, `qline` (GeometryTest_CurveTanCommands.cxx) | leave |
| 1376 | `geompipe` (BRepTest_SweepCommands.cxx) | leave |
| 1395 | ShapeCustom_Curve2d `SimplifyBSpline2d` | settled |
| 1402 | ShapeExtend::Init with Message_MsgFile::Msg | leave |
| 1410 | bugs/modalg_7/bug26034 (`bopcheck` after `ttranslate Revolve002 0 0 -0.5`) | leave |
| 1411 | `insertpole` (GeomliteTest_CurveCommands.cxx) | leave |
| 1413 | `lastrep` (Draw_VariableCommands.cxx) | leave |
| 1414 | `crvpoints`, `crvtpoints` (GeometryTest_CurveCommands.cxx) | leave |
| 1415 | `2dintersect` (GeomliteTest_API2dCommands.cxx) | leave |
| 1416 | `xbounds`, `removeinternals` (BRepTest_OtherCommands.cxx, DBRep.cxx) | leave |
| 1418 | BRepPrimAPI_MakePrism's history on a sphere band from 0 to 2π (bugs/modalg_7/bug30346_2… | settled |
| 1424 | bugs/modalg_7/bug83 (`bsection` of two planar B-spline sheets from BUC60912_sec_slow.brep) | leave |
| 1438 | `helix2`, `spiral2` (BRepTest_HelixCommands.cxx) | leave |
| 1446 | ShapeUpgrade_SplitSurfaceContinuity `Compute` | settled |
| 1447 | ShapeUpgrade_SplitSurfaceContinuity `Compute` | touch |
| 1448 | ShapeUpgrade::C0BSplineToSequenceOfC1BSplineCurve | settled |
| 1449 | ShapeUpgrade_ShapeConvertToBezier `Perform` | touch |
| 1452 | ShapeUpgrade_ShapeDivide `Perform` | touch |
| 1453 | ShapeUpgrade_ShellSewing `ApplySewing` | touch |
| 1454 | ShapeUpgrade_ShapeDivideClosedEdges `SetNbSplitPoints` | touch |
| 1455 | ShapeUpgrade_SplitCurve2dContinuity `Compute` | touch |
| 1471 | BRepOffset_MakeLoops (three places) | touch |
| 1472 | BRepOffset_Inter3d `ContextIntByInt` | leave |
| 1497 | LocOpe_Revol, LocOpe_RevolutionForm | leave |
| 1498 | LocOpe_LinearForm | leave |
| 1509 | ShapeProcess_Context `LoadResourceManager` | settled |
| 1510 | ShapeProcess_Context `GetReal`, `GetInteger` | touch |
| 1514 | ShapeProcessAPI_ApplySequence `PrepareShape`, `ClearMap` | touch |
| 1515 | SWDRAW `DT_SplitCurve`, `DT_SplitCurve2d`, `DT_SplitSurface` | leave |
| 1528 | BRepOffset_BuildOffsetFaces `BuildSplitsOfFaces` | touch |
| 1549 | BlendFunc_Chamfer, ConstThroat, ChAsym, Ruled (GenChamfer) `GetTolerance` | leave |
| 1552 | ChFiDS_FilSpine `SetRadius(Law, IinC)` | fix K2 |
| 1553 | Blend_CurvPointFuncInv `NbVariables` | leave |
| 1568 | BRepOffsetAPI_FindContigousEdges `NbEdges` | leave |
| 1601 | BRepBlend_CSWalking `IsDone`, `Line` | settled |
| 1611 | BRepBlend_CurvPointRadInv | leave |
| 1623 | TopOpeBRepDS_Interference() | leave |
| 1624 | TopOpeBRepDS_SurfaceExplorer() | leave |
| 1626 | TopOpeBRepDS_Surface | touch |
| 1654 | HLRAlgo_EdgeIterator `InitHidden` | touch |
| 1655 | Intrv_Interval's default constructor | settled |
| 1658 | HLRAlgo_EdgeStatus `Hide` | touch |
| 1666 | ChFiKPart_ComputeData_ChAsymPlnCyl `MakeChAsym` (straight guide) | fix K3 |
| 1667 | ChFiKPart_ComputeData_FilPlnCyl `MakeFillet` (circle, sphere case) | leave |
| 1688 | HLRBRep_Intersector `Perform` (two edges sharing a vertex) | touch |
| 1689 | HLRBRep_ShapeToHLR `ExploreShape` | touch |
| 1691 | HLRBRep_Surface | leave |
| 1699 | BRepFill_Filling `AddConstraints` | settled |
| 1714 | BRepFilletAPI_MakeFillet (all twelve edges of a cube of 10, radius 6) | touch |
| 1715 | TopOpeBRepBuild_HBuilder::Perform (blend/complex A3) | leave |
| 1738 | BRepFilletAPI_MakeFillet (a box of 10, its top edges, radius 6) | leave |
| 1746 | HLRBRep_EdgeBuilder.cxx | touch |
| 1749 | HLRBRep_PolyAlgo `InsertOnOutLine` | leave |
| 1750 | HLRBRep_PolyAlgo's copy constructor | touch |
| 1751 | HLRBRep_HLRToShape `InternalCompound` | touch |
| 1752 | HLRBRep `MakeEdge3d`, HLRAppli_ReflectLines `GetCompoundOf3dEdges` | settled |
| 1763 | ChFiDS_FilSpine::SetRadius(Law_Function, IinC) (BRepFilletAPI_MakeFillet's `Add(L, E)`… | fix K2 |
| 1765 | FilletSurf_Builder::EndSectionStatus | leave |
| 1766 | BRepTest_ChamferCommands.cxx, `chamf` | leave |
| 1767 | BRepTest_FilletCommands.cxx, `buildevol` and `blend` | leave |
| 1775 | ChFi3d (tolblend_simple C4, C7, C8, bug354_1) | leave |
| 1776 | TopOpeBRepBuild (blend/simple V7, tolblend_simple C5, complex A2) | settled |
| 1777 | BRepFilletAPI_MakeFillet `IsDeleted` | touch |
| 1786 | RWStl_Reader::Read | touch |
| 1787 | RWStl_Reader::Read, multi-domain ASCII | fix K8 |
| 1788 | RWStl_Reader::Read, multi-domain | touch |
| 1789 | RWStl_Reader::ReadVertex | leave |
| 1790 | RWStl::ReadStream, StlAPI_Reader::Read(stream) | fix K8 |
| 1791 | RWStl_Reader::ReadBinary | touch |
| 1792 | RWStl_Reader::ReadBinary | settled |
| 1794 | RWStl::WriteAscii(path), StlAPI_Writer::Write(path) | leave |
| 1824 | Interface_EntityList::TypedEntity | touch |
| 1831 | Interface_Category | leave |
| 1886 | Transfer_ActorOfProcessForTransient/ForFinder | leave |
| 1887 | Transfer_ActorDispatch::Transfer, Transfer_TransferDispatch::Copy | settled |
| 1896 | RWStepRepr_RWBooleanRepresentationItem::WriteStep | fix K8 |
| 1906 | RWStepVisual_RWSurfaceStyleRenderingWithProperties::ReadStep | fix K8 |
| 1907 | RWStepVisual_RWComplexTriangulatedFace::WriteStep, RWStepVisual_RWCubicBezierTriangulat… | touch |
| 1908 | StepVisual_StyledItem::Item, RWStepVisual_RWStyledItem::WriteStep | fix K8 |
| 1909 | RWStepAP214_ReadWriteModule::IsComplex | fix K8 |
| 1910 | RWStepAP214_ReadWriteModule::ComplexType and CaseStep (cases 694-697, 705, 706, 800) | leave |
| 1911 | RWStepAP214_GeneralModule::NewVoid case 106 | touch |
| 1912 | RWStepAP214_ReadWriteModule, cases 825 and 826 (SurfaceStyleReflectanceAmbientDiffuse(S… | fix K8 |
| 1913 | StepElement_VolumeElementPurposeMember and StepElement_SurfaceElementPurposeMember, Co… | touch |
| 1914 | StepBasic_SizeMember | touch |
| 1915 | RWStepVisual_RWCharacterizedObjAndRepresentationAndDraughtingModel::WriteStep | touch |
| 1916 | StepAP214_RepItemGroup, StepAP214_ExternallyDefinedClass, StepAP214_ExternallyDefinedGe… | touch |
| 1917 | RWStepVisual_RWSurfaceStyleParameterLine::WriteStep, RWStepVisual_RWTextStyleWithBoxCha… | touch |
| 1918 | StepFEA_SymmetricTensor43d | leave |
| 1938 | IFSelect_EditForm::NbTouched, IFSelect_ContextModif::Search, IFSelect_IntParam::StaticName | leave |
| 1946 | XCAFDoc_ShapeTool::GetShape | touch |
| 1947 | XCAFDoc_ShapeTool::AddComponent | touch |
| 1964 | RWStepRepr_RWBooleanRepresentationItem::WriteStep | fix K8 |
| 1980 | Transfer_ActorOfProcessForTransient/ForFinder | leave |
| 1981 | Transfer_ActorDispatch::Transfer, Transfer_TransferDispatch::Copy | settled |
| 1989 | TDF_Tool::Label (entry) | leave |
| 1990 | XCAFDoc_ShapeTool::RemoveShape, RemoveComponent | settled |
| 1991 | XDEDRAW_Layers.cxx setVisibility; XDEDRAW.cxx XDumpAssemblyGraph | leave |
| 1999 | STEPConstruct_Assembly::MakeRelationship | leave |
| 2000 | STEPControl_Controller (write actor) | settled |
| 2003 | STEPSelections_SelectDerived::Matches | settled |
| 2004 | STEPSelections_SelectGSCurves, SelectInstances | settled |
| 2013 | STEPCAFControl_Controller | touch |
| 2019 | XCAFDoc_ShapeTool::GetExternRefs | touch |
| 2033 | DEBREP_Provider::Write (`write.binary : 0`) | leave |
| 2034 | DE_ConfigurationContext (WhatKindOfLine) | leave |
| 2040 | BRepPrimAPI_MakeSweep's copy for write (OCCT.NET's, `KeepUnchangedGenerating`) | settled |
| 2042 | BRepFill_Filling::BuildWires with frozen boundary edges | settled |
| 2043 | tests/lowalgos/extss/bug29712_44 | leave |

### B. Markers whose class has no row

Bug-like `// OCCT` comments in kernel/src in files whose class occt-issues.md never names. Many are porting notes worded like bugs, or bugs listed under another name (a caller's, a generic's); each needs a look, then a row or a rewording.

| File | Lines |
|---|---|
| OcctNet.DataExchange/GeomToStep/GeomToStep_MakeConicalSurface.cs | 40 |
| OcctNet.DataExchange/IFSelect/IFSelect_SignAncestor.cs | 23 |
| OcctNet.DataExchange/IFSelect/IFSelect_SignType.cs | 37 |
| OcctNet.DataExchange/Interface/Interface_CopyMap.cs | 77 |
| OcctNet.DataExchange/Interface/Interface_FloatWriter.cs | 125, 186 |
| OcctNet.DataExchange/Interface/Interface_StaticSet.cs | 700 |
| OcctNet.DataExchange/Interface/Interface_UndefinedContent.cs | 63 |
| OcctNet.DataExchange/STEPSelections/STEPSelections_Selections.cs | 85, 97 |
| OcctNet.DataExchange/StepData/StepData_FreeFormEntity.cs | 172 |
| OcctNet.DataExchange/StepFile/StepFile_Scanner.cs | 624 |
| OcctNet.DataExchange/StepToTopoDS/StepToTopoDS_Builder.cs | 483, 575 |
| OcctNet.DataExchange/StepToTopoDS/StepToTopoDS_NMTool.cs | 64 |
| OcctNet.DataExchange/TopoDSToStep/TopoDSToStep_MakeFacetedBrep.cs | 107 |
| OcctNet.DataExchange/TopoDSToStep/TopoDSToStep_MakeFacetedBrepAndBrepWithVoids.cs | 134 |
| OcctNet.DataExchange/TopoDSToStep/TopoDSToStep_MakeShellBasedSurfaceModel.cs | 171 |
| OcctNet.DataExchange/TopoDSToStep/TopoDSToStep_WireframeBuilder.cs | 102 |
| OcctNet.DataExchange/Transfer/Transfer_ProcessForFinder.cs | 1295, 1338, 1391 |
| OcctNet.DataExchange/XSControl/XSControl_Utils.cs | 104 |
| OcctNet.Foundation/NCollection/NCollection_UBTree.cs | 155, 253 |
| OcctNet.ModelingAlgorithms/BOPAlgo/BOPAlgo_WireSplitter_1.cs | 399 |
| OcctNet.ModelingAlgorithms/BRepAlgo/BRepAlgo_Loop.cs | 458 |
| OcctNet.ModelingAlgorithms/BRepBlend/BRepBlend_RstRstEvolRad.cs | 56, 305 |
| OcctNet.ModelingAlgorithms/BRepBlend/BRepBlend_SurfRstEvolRad.cs | 51 |
| OcctNet.ModelingAlgorithms/BRepBuilderAPI/BRepBuilderAPI_Transform.cs | 21 |
| OcctNet.ModelingAlgorithms/BRepFeat/BRepFeat_Gluer.cs | 100 |
| OcctNet.ModelingAlgorithms/BRepGProp/BRepGProp_TFunction.cs | 134 |
| OcctNet.ModelingAlgorithms/BRepGProp/BRepGProp_UFunction.cs | 74 |
| OcctNet.ModelingAlgorithms/BRepMesh/BRepMesh_CurveTessellator.cs | 24 |
| OcctNet.ModelingAlgorithms/BRepMesh/BRepMesh_ModelBuilder.cs | 37 |
| OcctNet.ModelingAlgorithms/BRepMesh/delabella.cs | 336, 504 |
| OcctNet.ModelingAlgorithms/ChFi3d/ChFi3d_Builder_C1.cs | 2924 |
| OcctNet.ModelingAlgorithms/ChFi3d/ChFi3d_Builder_C2.cs | 515 |
| OcctNet.ModelingAlgorithms/ChFi3d/ChFi3d_Builder_SpKP.cs | 1249 |
| OcctNet.ModelingAlgorithms/ChFi3d/ChFi3d_ChBuilder_C3.cs | 203 |
| OcctNet.ModelingAlgorithms/ChFi3d/ChFi3d_FilBuilder_C2.cs | 213 |
| OcctNet.ModelingAlgorithms/ChFi3d/ChFi3d_FilBuilder_C3.cs | 248, 288 |
| OcctNet.ModelingAlgorithms/ChFiDS/ChFiDS_SurfData.cs | 36 |
| OcctNet.ModelingAlgorithms/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnCyl.cs | 314 |
| OcctNet.ModelingAlgorithms/Draft/Draft_FaceInfo.cs | 19 |
| OcctNet.ModelingAlgorithms/Geom2dAPI/Geom2dAPI_PointsToBSpline.cs | 235 |
| OcctNet.ModelingAlgorithms/GeomFill/GeomFill_Fixed.cs | 34 |
| OcctNet.ModelingAlgorithms/GeomFill/GeomFill_Frenet.cs | 763 |
| OcctNet.ModelingAlgorithms/GeomFill/GeomFill_FunctionGuide.cs | 179 |
| OcctNet.ModelingAlgorithms/GeomFill/GeomFill_NetworkSurface.cs | 290, 489 |
| OcctNet.ModelingAlgorithms/GeomFill/GeomFill_SectionPlacement.cs | 485 |
| OcctNet.ModelingAlgorithms/HLRBRep/HLRBRep_BCurveTool.cs | 83 |
| OcctNet.ModelingAlgorithms/HLRBRep/HLRBRep_PCLocFOfTheLocateExtPCOfTheProjPCurOfCInter.cs | 39, 140 |
| OcctNet.ModelingAlgorithms/IntCurve/IntCurve_ExactIntersectionPoint.cs | 19, 130 |
| OcctNet.ModelingAlgorithms/IntCurve/IntCurve_IConicTool.cs | 134, 164 |
| OcctNet.ModelingAlgorithms/IntCurve/IntCurve_Polygon2dGen.cs | 240 |
| OcctNet.ModelingAlgorithms/IntImp/IntImp_Int2S.cs | 94 |
| OcctNet.ModelingAlgorithms/IntPatch/IntPatch_ImpImpIntersection_CyCy.cs | 545, 597, 863, 3851 |
| OcctNet.ModelingAlgorithms/IntPatch/IntPatch_ImpImpIntersection_Quadrics.cs | 573, 1136, 1467 |
| OcctNet.ModelingAlgorithms/LocOpe/LocOpe_PntFace.cs | 26 |
| OcctNet.ModelingAlgorithms/ShapeAnalysis/ShapeAnalysis_ShapeContents.cs | 230 |
| OcctNet.ModelingAlgorithms/ShapeConstruct/ShapeConstruct_ProjectCurveOnSurface.cs | 526 |
| OcctNet.ModelingAlgorithms/ShapeFix/ShapeFix_ComposeShell.cs | 848 |
| OcctNet.ModelingAlgorithms/ShapeUpgrade/ShapeUpgrade_ConvertCurve2dToBezier.cs | 262 |
| OcctNet.ModelingAlgorithms/ShapeUpgrade/ShapeUpgrade_ConvertSurfaceToBezierBasis.cs | 402 |
| OcctNet.ModelingAlgorithms/Sweep/Sweep_NumShapeIterator.cs | 19 |
| OcctNet.ModelingAlgorithms/TopOpeBRepBuild/TopOpeBRepBuild_BlockBuilder.cs | 22 |
| OcctNet.ModelingAlgorithms/TopOpeBRepBuild/TopOpeBRepBuild_LoopSet.cs | 14 |
| OcctNet.ModelingAlgorithms/TopOpeBRepDS/TopOpeBRepDS_Transition.cs | 57 |
| OcctNet.ModelingData/Approx/Approx_Curve2d.cs | 93 |
| OcctNet.ModelingData/Approx/Approx_CurveOnSurface.cs | 541 |
| OcctNet.ModelingData/Approx/Approx_SameParameter.cs | 486 |
| OcctNet.ModelingData/BinTools/BinTools.cs | 272, 288 |
| OcctNet.ModelingData/Extrema/Extrema_EPCOfExtPC.cs | 21 |
| OcctNet.ModelingData/Extrema/Extrema_EPCOfExtPC2d.cs | 21 |
| OcctNet.ModelingData/Extrema/Extrema_FuncPSDist.cs | 34, 64 |
| OcctNet.ModelingData/Extrema/Extrema_GlobOptFuncCC.cs | 23, 39 |
| OcctNet.ModelingData/Extrema/Extrema_GlobOptFuncCS.cs | 100, 123, 136 |
| OcctNet.ModelingData/Extrema/Extrema_GlobOptFuncConicS.cs | 173 |
| OcctNet.ModelingData/Extrema/Extrema_PCFOfEPCOfExtPC.cs | 41, 142 |
| OcctNet.ModelingData/Extrema/Extrema_PCFOfEPCOfExtPC2d.cs | 41, 142 |
| OcctNet.ModelingData/Geom2dConvert/Geom2dConvert_CompCurveToBSplineCurve.cs | 103 |
| OcctNet.ModelingData/GeomBndLib/GeomBndLib_Surface.cs | 125 |
| OcctNet.ModelingData/GeomConvert/GeomConvert_CompCurveToBSplineCurve.cs | 104 |
| OcctNet.ModelingData/GeomGridEval/GeomGridEval.cs | 125 |
| OcctNet.ModelingData/GeomGridEval/GeomGridEval_BSplineSurface.cs | 636 |
| OcctNet.ModelingData/GeomGridEval/GeomGridEval_BezierSurface.cs | 283 |
| OcctNet.ModelingData/GeomLProp/GeomLProp_CLProps.cs | 214 |
| OcctNet.ModelingData/GeomLProp/GeomLProp_CLProps2d.cs | 214 |
| OcctNet.ModelingData/GeomLProp/GeomLProp_FuncCurExt2d.cs | 25, 53 |
| OcctNet.ModelingData/GeomLib/GeomLib_MakeCurvefromApprox.cs | 16 |
| OcctNet.ModelingData/GeomTools/GeomTools_CurveSet.cs | 688, 708, 964, 977, 996 |
| OcctNet.ModelingData/LProp/LProp_CLProps3d.cs | 213 |
| OcctNet.ModelingData/TopExp/TopExp.cs | 217, 301 |
| OcctNet.ModelingData/TopoDS/TopoDS_CompSolid.cs | 247 |
| OcctNet.ModelingData/TopoDS/TopoDS_Compound.cs | 247 |
| OcctNet.ModelingData/TopoDS/TopoDS_Face.cs | 247 |
| OcctNet.ModelingData/TopoDS/TopoDS_Shell.cs | 247 |
| OcctNet.ModelingData/TopoDS/TopoDS_Solid.cs | 247 |
| OcctNet.ModelingData/TopoDS/TopoDS_Vertex.cs | 247 |
| OcctNet.ModelingData/TopoDS/TopoDS_Wire.cs | 247 |
