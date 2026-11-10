# OCCT's defects, triaged

*Unit 11S, from 6 November 2026 (decision 1743; decisions 1775 onwards). Kerf ported OCCT 8.0.1 at `b8f597c6`.*

The owner's direction: "We don't need to be bound by OCCT's way of doing things - it has plenty of bugs and unintuitive
behaviour which we can and should get rid of in Kerf." So "native does the same" explains a defect and doesn't excuse it. This
file lists what OCCT knows to be wrong and whether Kerf has it. It's a working list: the unit goes on after 0.3.0.

**What a status means.**

- **already fixed:** Kerf doesn't have it, and the row says why, with a decision where one exists.
- **not relevant:** it can't reach Kerf, such as OCCT's viewer, build or document framework ([unported.md](unported.md)).
- **applies:** Kerf has it. The row says how that was confirmed; "reproduced" means through Kerf's own code, "read" means only
  from the code.
- **can't tell:** Not settled yet. The third pass (decisions 1830 to 1838) fetched what the issues attach and re-expressed their scripts through Draw-lite or a test, reading the files as data; the rows it reached say what was seen. What's left is mostly hidden lines, STEP writing and issues with no model, each saying why. "The public fillet fails" and the like are inferred from the check of decision 1742, not run.

**Counts.** OCCT's 224 open issues: 47 already fixed, 88 not relevant, 42 applies (one of them fixed in Kerf), 2 not reproduced, 45 can't tell (units 12I and 12J, counted from the rows at the merge; 12J moved nine from applies to already fixed, and 15E one more; [corpus-failures.md](corpus-failures.md) sorts the can't-tells by why). The 16 issues closed since 8.0.1: 1 already fixed, 9 not relevant, 4 applies, 2 can't tell. Its 76 commits since 8.0.1: 3 already fixed, 57 not relevant, 10 applies, 6 can't tell.
The other two sources are sorted by group below: the corpus's 502 BAD cases, and the rows of [occt-issues.md](occt-issues.md)
kept only for compatibility.


## The ranked list

What a user would meet, worst first. Sizes: S under 20 lines and a test, M up to about 100 lines, L larger.

| Kind | Where | What | How confirmed | Size |
|---|---|---|---|---|
| Silent wrong answer | #1639 | A fillet of volume 10596 for 29800, depending only on the order of the shell's faces. The public fillet now fails, by the size of its tolerance (decision 1833) | Reproduced | M to L: the corner's 3D curve, 190 from its pcurves |
| Silent wrong answer | #245 | A cut that removes almost nothing, with warnings: now a failure (decision 1866) | Reproduced | L |
| Silent wrong answer | #1570, #1569, #737, #694, #1430 | Fillets that pass the check and intersect themselves: the check doesn't look for that | Reproduced | **For the owner**: `bopcheck` on a fillet's result is too slow to be a default, I think; not measured |
| Invalid result | #1573, #1571, #899 and others | Unify on rotated spheres; fillets. All reported as failures by the public layer | Reproduced | M to L |
| Silent wrong answer | Found by #1675 | Area and volume without a precision, which `Kerf.Modeling` uses, can be up to 3.7% off on swept and B-spline faces: bug5805_21 gives 253.49 where a mesh gives 244.17. The precise path now gives 244.195 | Reproduced | S to change the call; **for the owner**: precise is slower |
| Silent wrong answer | Upstream `8f817f5da5` | Curve-on-surface deviation, narrow extrema, face self-intersection within trimmed domains | Upstream fix, not checked here | L: 346 files |
| Invalid result | #1711 | `FuseEdges` can leave an edge range outside its curve | Read | S to M |
| Failure on an ordinary shape | #1630 | A pipe of a circle along a planar cubic: no crash, but no pipe, with the default corrected Frenet trihedron, also through `Sweeps.Sweep`. Frenet and discrete both work on it, area 217.81. Why the sweep's approximation fails is not found | Reproduced | M; **for the owner** if the fix is to fall back on another trihedron |
| Failure on an ordinary shape | #1177, #172 | A fillet that should reach the opposite edge fails; upstream `f713b1305f` and `cbca11e975` do it | Upstream fix | L: 27 files. **Not ported, the owner's decision** (1892, 1901); done another way for straight edges (1847) |
| Failure on an ordinary shape | corpus: offset/shape_type_i | A cone's offset, with its apex: `Solid.Cone(5, 0, 12).Offset(-1)` with round joins, and a cone with its apex at the bottom inwards with either join or outwards with mitres. Failures since decision 1918; found by 12J, not fixed: with mitres the first is right (107.757, by similar triangles), so a retry with mitres inwards would serve it, but that needs decision 1918's check, which this branch didn't have. The apex at the bottom is the enlarged face keeping its far side, past the base's plane, in `BuildOffsetByInter`; its box reaches 3.8 million | Reproduced | M; the retry S |
| Failure on an ordinary shape | corpus: offset/wire_* | 77 BAD, all OCCT's own known failures (`TODO` in its scripts), on wires of B-splines: 39 a kink inside a B-spline edge ("Offset on C0 curve"), 16 an offset wire left open, 4 `FixHoles`, 18 built but faulty. Not the medial axis: traced by unit 12S ([offset2d-survey.md](offset2d-survey.md), decision 1958) | Corpus, traced | L for a new offset; not recommended. See below |
| Unintuitive API | #1036 | History differs from maker to maker | Read | M |
| Unintuitive API | #1618, the rest | `Point3D` takes NaN, so `Vertex.At` does. `Solid.Box` refuses it, checked through the public layer (and decision 1820 for directions) | Reproduced | S to M |
| Costs time only | #1622, #1697 | Lipschitz estimate on `GlobOptFuncCCC2` too large; a mesh's stated deflection too large, which inflates a box from it | Read | For 11Q |
| Costs time only | #1495 | `StoreData`'s walk could spin | Read | S |

## Questions for the owner

1. **Should `Kerf.Modeling`'s area and volume use the precise path?** The default integrates at a fixed number of points, and was 3.7% off on the worst corpus case. The precise path now matches a fine mesh, at a measured 1.85 times the cost on the issue's set (not measured here). I recommend yes, with a time check first. **Answered yes, done: decision 1802; it costs no more here.**
2. **Fillets to the opposite edge (#1177, #172, upstream `f713b1305f`).** Porting needs 27 upstream files in the fillet code. I recommend it before any rolling-ball rewrite, as it's the most common request in the fillet list. *Since then: on straight edges between planes it's done another way (decision 1847), and upstream's own results for the four corpus cases it changes are still faulty. [fillet-design.md](fillet-design.md) recommended not porting it, and the owner agreed: **it isn't ported** (decisions 1892, 1901).*
3. **The 2D offset's failures (77 corpus cases).** Fixing them means a new way of making offsets, not a patch: I recommend a survey of what Bento needs first. **Answered: the survey, done** ([offset2d-survey.md](offset2d-survey.md), decisions 1956 to 1960). The 77 are OCCT's known failures on digitised B-spline outlines, which Bento's profiles don't reach; what Bento does reach, an offset closing up, is fixed, and an inward offset past a curve's sharpest bend, up to 1% off, is a question in the survey.
4. **A default sweep that fails (#1630).** Should a sweep with the default trihedron retry with another when its approximation fails? I recommend yes until the cause is found; Frenet and discrete both work on the case. **Answered yes, done: decision 1805.**
5. **Should a Boolean check its result's volume?** #1659, #1643 and others give a Cut larger than its object, or a Common of nothing where the input's point classification finds plenty, and no error. A check that a Cut is no larger than its object, a Common no larger than the smaller, and a Fuse no smaller than the larger would turn those into failures carrying the shape, as decision 1742 does for invalid shapes. It wouldn't catch #1629, which comes out at exactly the block's volume, or #1613's kind. It costs a volume per Boolean, not measured here. I recommend yes if a time check allows it. **Answered yes, done: decisions 1860 to 1868,** which do catch #1629 and need more than volumes for the rest.

## For unit 11S: results of no volume reported as success

From decision 1809, which fixed the sweep's: a `Solid.Extrude` in the profile's plane (volume 0, invalid, and no failure); a `Solid.MakeLoft` of two sections in one plane (0, "valid"); the kernel's `Sweeps.Sweep` with `SweepFrame.Fixed` and `WithCorrection`, or along the section's plane. **All fixed by decision 1927;** `WithCorrection` turns the section to the path, so it has its volume, 2000 for a 10 by 10 square along 20, and needs nothing.

## Fixed so far

| Issue or commit | What a user saw | Decision |
|---|---|---|
| #1629 | A Cut with a solid inside out, a Common empty with no error, and a Fuse with an extra groove: now right, made again with a fuzzy value of ten tolerances. The cause is a neck 1e-5 wide in the tool's flank, which the exact cut doesn't split. `Kerf.Modeling`'s cut still fails, carrying the shape | 28601, 28602 |
| #1643 | A Common empty, a Cut that removed nothing and a Fuse missing part of the lip, by the inputs' order: now right, made the other way round | 1937 |
| `1349657592` | A general transform's mesh inside out under a mirror, with wrong normals, and moved twice if located | 1936 |
| #1659 | A Cut that removed nothing and an empty Common, for a section curve stopping 1e-3 short of the next | 1935 |
| #1039 | A section's closed curve lost between a sphere and a slice at a shallow angle, 86898 for 73353 | 1934 |
| #1632, #1431 | A chamfer not saying which contours failed; the same 2D corner chamfered twice an internal fault | 1933 |
| #1039 | A slice's Common with a model, 86898 for about 70000, reported as a success: now a failure | 1932 |
| #1355 | A rotation from a 4 by 4 matrix refusing to convert to a transform | 1931 |
| #572 | A cone from a STEP file with no triangles | 1930 |
| #1444 | A thin torus's mesh 1.42 from the surface for 0.45 asked | 1929 |
| A sphere | A sphere's mesh up to twice its deflection, from the scaling of its parameters; now within it with the grid's own triangles | 1953 |
| `53706f78a8` | A loft's rational section changing shape, for weights that agree part way through being scaled | 1928 |
| Decision 1809's list | `Solid.Extrude`, `Solid.MakeLoft` and `Sweeps.Sweep` reporting success for a solid of no volume | 1927 |
| #1123 | A minimum distance of 0.083 between parts that intersect | 1926 |
| #1629, #1643, #1659, #245 | Booleans valid and wrong, reported as a success: now a failure, causes open | 1860, 1863, 1866 |
| #1543 | A cut by a helical tube removing nothing, by where it left relative to the seam | 1834 |
| #1613 | A fuse dropping a compound of reversed solids | 1832 |
| #994 | A face lost reading a STEP offset surface whose basis is C0 at a knot | 1838 |
| #1541, #1340 | Unify making a valid solid invalid: a box cut by a sphere lost a face | 1831, 1830 |
| #1692 | A reused solid classifier's answer depending on the points before it | 1835 |
| #1124 | The public section losing most of a curve (approximation now on) | 1836 |
| #1310 | Unify leaving a closed pipe a face with no edges, and no mesh | 1837 |
| #1639, #1695, #1701 | A fillet with a tolerance past its radius reported as a success: now a failure, causes open | 1833 |
| #1633 | Booleans of cylinders about 1° from parallel: Common empty, Cut removes nothing, no error | 1783 |
| #1604, #1679 | An optimal box smaller than the shape | 1777 |
| Found by #1675 | `Kerf.Modeling`'s area and volume up to 3.7% off on swept and B-spline faces | 1802 |
| #1630 | A default sweep with no result where other frames work | 1805 |
| #1685 | An area up to 2.7% short, with an estimate of 1e-10, depending on where an edge starts | 1785 |
| #1675 | Precise mass properties never refined along U | 1784 |
| #1681, #1682 | A sweep with a guide distorted, volume 1072.3 for 1080 | 1776 |
| #1566 | Revolving an arc that ends at the axis: invalid, 20% of the volume gone | 1780 |
| #1577 | A surface grid 5 units off just below a knot; slow optimal boxes | 1775 |
| #1617 | A parabola of focal 0 projected wrongly | 1781 |
| #1618 | NaN radii and angles accepted (directions stay: see the decision) | 1782 |
| #1698 | A meshed shape's optimal box included the tolerance when told not to | 1779 |
| #1606, #1452 | A ring of concentric circles couldn't be offset inward; a collapsing circle went to the general algorithm | 1788 |
| `3270746444` | A point mirror's inverse | 1778 |
| `71658f6c8f` | An offset surface on an indirect plane on the wrong side | 1811 |
| `71658f6c8f`, `8f817f5da5` | `ApproxInt`'s caches returning without writing | 1812 |
| occt-issues.md | `Law.BSpline`'s multiplicities and `Segment`; the cylinder range for negative B; `CoonsAlgPatch.Value`, `FunctionDraft`, quasi-angular and `GetCircle` second derivatives | 1813 to 1815 |
| Unit 11R's list | A clean that turned a good Boolean invalid, up to 17% out in volume | 1816 |
| occt-issues.md | `ApproximateBSplines` on a cylinder, and `ToSplines` on a sphere, invalid | 1817 |
| #1567 | A sphere touching a torus along a circle: an empty fuse, with no failure | 1818 |
| #1618 | NaN and infinite directions accepted | 1820 |
| #1461 | A nearly horn torus written as a torus or a revolution by noise | 1821 |
| `53706f78a8` | A closed polygon's self-intersections lost | 1822 |

## The corpus's 502 BAD cases, by group

A BAD case is one whose script expects a failure. Native and the port fail alike unless the row says otherwise; that explains
them and doesn't excuse them. The sort below is from each case's expected messages, not from running each one. Decision 1717's
survey ran the fillet and chamfer ones natively.

| Group | Cases | What they say | Sorted as | What next |
|---|---:|---|---|---|
| offset/wire_* | 77 | "The offset cannot be built" for 2D offsets of closed and open wires: OCCT's own known failures, by four causes, none the medial axis (decision 1958) | **Real**: ordinary input, a clean failure | The largest real group. A question for the owner, below |
| offset/faces_type_i, shape_type_i*, offset/bugs | 34 | 3D offsets in intersection mode: not built, invalid, or wrong volume | **Real**, some invalid | Goes with #1106, #1107, #1706. Large |
| bugs/modalg_2 | 52 | Old Boolean and section cases: counts of entities, area and length differ from a reference | **Mixed**. Many compare with figures from an old engine; 16 are about triangle counts in a mesh | Sort by hand; expect most to be stale references |
| bugs/modalg_1 | 22 | The same | **Mixed** | The same |
| bugs/modalg_7 | 32 | Each with its own issue number: fillets that can't touch, micro edges, bad accuracy | **Real**, one by one | Read in turn: each is small |
| de/step_* | 76 | The dataset's own counts: labels, properties, numbers of shapes, translation messages | **Mostly stale references** (one ticket for all, CR23096), though a few lose colours or layers | Compare with native's log (already equal), and look at the colours and layers |
| thrusection/specific* | 36 | Area differs, or the result is "not Closed shape" (7) | **Stale area for the 29; real for the 7** | The 7 are lofts that don't close |
| heal/split_angle_*, heal/surface_to_* | 38 | Faulty shape from `DivideClosed` and conversions on a file's faces, or area | **Real but narrow**: the input is often faulty already | Check input validity first |
| draft/angle | 11 | Faulty shape or area | **Real** | With 11G's drafts; #303 |
| blend/*, chamfer/* | 25 | With 5 in bugs/, the fillet survey of 30: 26 fail natively at the same step, 3 invalid natively, 1 the port only (corpus-failures.md) | **Real**: "the fillet list" | The owner's question on a rolling-ball rewrite (decision 1724) |
| chamfer corners (FAILED, not BAD) | 4 | Natively invalid at step 2 or 15; the port refuses | **Real**; the port is right to refuse | With the fillet list |
| lowalgos/intss | 13 | Tori give splines, not circles (3); big tolerances; "reproduced" | **Quality**, not wrong answers | Small ones in turn |
| mesh/* | 16 | "Meshing statuses: Failure" on known faces | **Fails rightly** on self-intersecting wires for most; the rest with 11O | Look with 11O |
| sewing/tol_100 | 5 | "Number of faults" at a very large tolerance | **Fails rightly** | Leave |
| mkface/after_extsurf_and_offset | 4 | "Incorrect input parameters are not processed correctly" | **Real, small**: bad parameters accepted | Goes with #1618's checks |
| bugs/moddata_*, bugs/heal, bugs/mesh, bugs/xde, the other heal groups, others | 61 | One-off tickets | **Mixed** | Read in turn |

## Rows of occt-issues.md kept only for compatibility

1,154 rows were triaged by unit 10K (occt-issues-triage.md). Its 134 **fix** rows are done. Of its 572 **touch** rows, those
that would be a silent wrong answer and can be reached from a public operation are in the ranked list by line number; others
are fixed when their code is next changed, as 10K agreed. `ShapeFix_Wire::FixGap3d` on one edge stays: it's harmless, as the
vertex's tolerance then covers it. A second look could raise more of the touch rows, and 744 was tried here.


## OCCT's open issues

| # | What it says | Status | Why |
|---:|---|---|---|
| 1711 | BRepLib_FuseEdges::UpdatePCurve creates an out-of-domain B-spline edge range from valid input (reproduced in 8.0.1) | can't tell | Same code (`FuseEdges.UpdatePCurve`); the shell needs the issue's long sample; not reproduced |
| 1709 | Invalid geometry while sweeping rectangle along a closed path | can't tell | `pipe/specific/Z7`'s data isn't in the public dataset |
| 1706 | fillet -> offset creates invalid geometry | applies | Reproduced from `bug31601`: invalid, bad orientation. The public offset fails |
| 1704 | Data Exchange, STEP Export - Full-turn toroidal fillet face with an indirect frame is written with its FACE_BOUND in the wrong sense | can't tell | Same writer code; needs a full-turn torus with an indirect frame through a write and a raw read; not reproduced |
| 1703 | BRepClass3d_SolidClassifier reads an outside point IN: BRepTopAdaptor_TopolTool undersamples a B-spline with two pole rows | can't tell | Its file can't be read: its locations hold a scale and a form before the matrix, which isn't what 8.0.1's `BinTools` or Kerf's reader expects. Read: the undersampled polyhedron is the same code |
| 1702 | Modeling Algorithms - BRepFilletAPI_MakeFillet returns a self-intersecting wire on a plate crossed by a round bar | can't tell | The gist wasn't fetched. A self-intersecting wire is what the public check reports, so it wouldn't be silent |
| 1701 | Modeling Algorithms - BRepFilletAPI_MakeFillet gives a 4.8 mm vertex tolerance when its radius equals an adjacent fillet's | can't tell | Not run: the gist wasn't fetched. By its figures, the public fillet now fails on it (decision 1833) |
| 1700 | Modeling Algorithms - BRepExtrema_DistShapeShape reports InnerSolution for disjoint solids: IntCurvesFace_Intersector misses a crossing near a B-splin | can't tell | Needs the issue's attached model; not reproduced in this pass (same intersector as #1703) |
| 1699 | Hidden Line Removal - HLRBRep_Algo outline edge runs back on itself and leaves the surface | can't tell | Needs the issue's attached model; not reproduced in this pass (hidden lines) |
| 1698 | BRepBndLib::AddOptimal adds the face tolerance with useShapeTolerance = false when the face has a triangulation | already fixed | Decision 1779; read, then tested |
| 1697 | BRepLib::UpdateDeflection gives 11.8 for an exact mesh of a planar B-spline face, so AddOptimal boxes grow | applies | Read: `UpdateDeflection` measures at the average of the UVs, as the issue says; no reproduction yet |
| 1696 | STEP import: geometric tolerance dropped when its face carrier is also a datum target's feature (FEATURE_FOR_DATUM_TARGET_RELATIONSHIP) | not relevant | GD&T in the assembly document, which isn't ported ([unported.md](unported.md)) |
| 1695 | Modeling Algorithms - BRepFilletAPI_MakeFillet reports success on a two-edge corner with an invalid curve on surface | can't tell | Not run: the gist wasn't fetched. By its figures, the public fillet now fails on it (decision 1833) |
| 1692 | Modeling Algorithms : BRepTopAdaptor_TopolTool keeps the 2D classifier of its first call, so a reused BRepClass3d_SolidClassifier answers after the po | already fixed | Read, and a test on a holed square. Fixed: decision 1835; a classifier for each tolerance |
| 1691 | Crash with sweep with auxiliary spine | already fixed | Tried on its files: its first case isn't done, with the message native had. Its fourth is a null reference at `PipeShell.Add`, which the public layer reports as a failure. The other two pass a compound as the guide, which the typed API can't |
| 1690 | STEP export: material density written with wrong unit exponents (g³·cm²) and an untyped value | applies | Read: same writer; material density units. Small, data exchange |
| 1688 | Crash with loft on closed profile | already fixed | Tried on its files: a valid shell and no overflow. Its `SetMaxDegree(5)` wasn't set |
| 1687 | BRepAlgoAPI_Fuse(S, Copy(S)) returns empty with IsDone=true while Fuse(S, S) succeeds (OCCT 7.9.3) | not reproduced | Fuse with a copy on a box, cylinder, sphere, cone and torus: right volume and faces, as Fuse with itself. Its model wasn't tried, so that's where it could still be |
| 1686 | Data Exchange, STEP Export - Non-ASCII labels in AP214 files are written as raw UTF-8 instead of being encoded | applies, fixed | Decision 1920. It's as the issue says: raw UTF-8 in the file. Now `\\X2\\` and `\\X4\\`, read back the same |
| 1685 | BRepGProp SurfaceProperties: phase-dependent area for a planar periodic B-spline, despite small error estimate | already fixed | Decision 1785; reproduced (22.285 for 22.900) and fixed: the box of a periodic pcurve's arc |
| 1684 | Crash with loft on infinite line | already fixed | Tried on its files: a null reference, which the public `Loft` reports as a failure (by the code, not run) |
| 1682 | BRepOffsetAPI_MakePipeShell with a guide distorts a straight sweep in 8.0.1 (exact in 7.9.3) | already fixed | Decision 1776 (same cause as #1681) |
| 1681 | Modeling - GeomFill_GuideTrihedronAC::D2 returns wrong second derivatives (TN2 typo); since #1182 made D2 return true, auxiliary-spine MakePipeShell d | already fixed | Decision 1776; reproduced and fixed |
| 1680 | BRepBndLib::AddOptimal ignores a trimmed edge on a face of revolution in 8.0.1 (correct in 7.9.3) | can't tell | Needs the issue's attached model; not reproduced in this pass (a box too large, not too small) |
| 1679 | BRepBndLib::AddOptimal returns a box smaller than a smooth ThruSections loft (8.0.1 axis-aligned; 7.9.3 and 8.0.1 when rotated) | already fixed | Decision 1777; reproduced (29.969 for 30) and fixed |
| 1678 | BOPAlgo_PaveFiller::MakeBlocks can pass a section edge with no 3D curve to BOPTools_AlgoTools::MakePCurve, which dereferences it (SIGSEGV) | not relevant | A null dereference natively; the port throws, and BOPAlgo reports a failure |
| 1677 | STEP import: cone loses its apex when the file's uncertainty exceeds 2*pi mm | not reproduced | A written cone read back with an uncertainty of 1, 7 and 10: its faces, its apex and its volume hold. A file from elsewhere could still differ |
| 1676 | Boolean operation between two simple faces takes unreasonable amount of time (12 minutes) | applies | Reproduced on its files: past 150 s. For 11Q |
| 1675 | BRepGProp_Gauss: the precise (Eps) path never refines along U and drops U's error from its estimate, so swept B-splines, full revolutions and areas ar | already fixed | Decision 1784; the issue's patch applied by hand, with a test |
| 1673 | Modeling Algorithms - ChFi3d_FilBuilder::Compute overflows the stack on a three-edge corner of a ThruSections loft (NaN bounds in GeomPlate initial su | applies | Built from its numbers: no overflow, but invalid, volume 1.8e6 from 67183. The public fillet fails |
| 1672 | IntTools_FaceFace drops the last partial turn of a helical section where the walk meets a cylinder face's v bound | can't tell | Needs the issue's attached model; not reproduced in this pass |
| 1671 | IntWalk_PWalking's step is relative to the face's parameter range, so sections on long helical faces lose accuracy and booleans go wrong | can't tell | Needs the issue's attached model; not reproduced in this pass (the step is relative, read) |
| 1665 | Fillet failure on a simple solid | can't tell | Tried on its file: it reads, valid. The issue doesn't say which edges |
| 1661 | Documentation - Add AGENTS.md that points coding assistants to the contribution rules | not relevant | OCCT's documentation or tests only |
| 1659 | Boolean cut silently returns an imprint instead of a cut: plane/B-spline section loop left open | already fixed | Decision 1935: its files, Cut 4903.29 and Common 375.21, adding to 5278.50, 5 faces, a closed section |
| 1658 | `SetColor()` does not recolor seam / degenerated / boundary edges of a shape | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 1652 | Data Exchange, STEP Import - Spherical/toroidal CURVE_BOUNDED_SURFACE with a clockwise OUTER_BOUNDARY_CURVE is read as the complement of the patch sin | can't tell | CURVE_BOUNDED_SURFACE read: same reader; needs a file |
| 1651 | GeomLProp_SLProps: MinCurvature/MaxCurvature/MeanCurvature/GaussianCurvature return the previous value on the first call after construction or SetPara | already fixed | Kerf computes before reading: `SLProps.RequireCurvature` (decision in phase 2); the port's comment says so |
| 1643 | Booleans on two valid lofts that share an end section: Fuse empty, Common zero, Cut larger than its object (7.9.3 and 8.0.1) | already fixed | Decision 1937: right the other way round, which is now tried; the cause in one order isn't found |
| 1639 | Modeling Algorithms - BRepFilletAPI_MakeFillet result depends on the order of the shell's faces | applies | Reproduced on its files: 10596.3 for 29799.9, with an edge's tolerance of 190, and valid. Its other order is right. The public fillet now fails (decision 1833); the cause isn't found |
| 1638 | Segfault in STEPCAFControl_Reader::findReprItems when a representation has no items | not relevant | Null items natively a crash; the port's STEP layer reports (decision 1050s) |
| 1637 | GCC 16 miscompiles BRepClass3d_SClassifier::Perform in 7.9.x, classifying a point on an edge segfaults | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 1633 | BRepAlgoAPI_Common returns an empty result (and Cut removes nothing) for two overlapping cylinders whose axes are about 1 degree apart | already fixed | Decision 1783; reproduced (Common 0 at 1°) and fixed: the walk ran on past both bounds |
| 1632 | Modeling Algorithms - Expose faulty contours and vertices on BRepFilletAPI_MakeChamfer | already fixed | Decision 1933 |
| 1631 | Modeling Algorithms - BRepFilletAPI_MakeFillet loses external radius law and throws in Build | already fixed | Reproduced: works, 7962.8; decision 1676 fixed `ChFiDS_FilSpine.Reset` |
| 1630 | Modeling - BRepOffsetAPI_MakePipe access violation for cubic spine and circular face | applies | Reproduced: no crash, but `ApiMakePipe.Shape()` refuses: a cubic spine and a circle make no pipe |
| 1629 | Boolean cut of a helical sweep fused with a cylinder returns the argument uncut plus an inverted solid, silently, at some depths | already fixed | Reproduced on its files: Cut gives two solids totalling 7705.48, the block's volume, no error. The public Boolean now retries with a fuzzy value and is right (decision 2103); the engine's exact cut is unchanged |
| 1626 | Add a way to disable cout in case of error | not relevant | Kerf doesn't write to the console (the clean-up) |
| 1622 | Extrema_GlobOptFuncCCC2: value is the distance but the gradient is of the squared distance | applies | Read: the same gradient. Costs time only: belongs with 11Q |
| 1619 | Use-after-free in TopOpeBRepBuild `CorrectUnclosedWire` crashes `BRepFilletAPI_MakeFillet::Build` | already fixed | occt-issues.md row for `CorrectUnclosedWire`: the port iterates a copy |
| 1618 | NaN and infinite inputs pass the validity checks of gp/Geom constructors | already fixed | Decision 1782 for radii and angles; directions and points are left: see the ranked list |
| 1617 | Parabola with zero focal distance: evaluation is a line, but projection and parameter inversion are not | already fixed | Decision 1781; reproduced and fixed |
| 1615 | Visualisation: Run-time vs Compile-time paths for X11 / Wayland contexts | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 1613 | BRepAlgoAPI_Fuse silently drops a compound operand of two B-spline solids (fused correctly as separate tools) | already fixed | Reproduced: the fuse gave the cylinder alone, 12566.4, for 12687.3. A reversed solid's splits in a compound were turned inside out. Fixed: decision 1832 |
| 1610 | BOP fuse of two tubes meeting at a shallow angle leaves the section open since 0029093 (IntWalk_PWalking::ExtendLineInCommonZone) | can't tell | Needs the issue's attached model; not reproduced in this pass (a section left open at a shallow angle) |
| 1608 | BRepTools::Read never returns on a truncated text BRep | already fixed | Reproduced: the port throws `StreamFormatErrorException` (the reader's guards, phase 8) |
| 1607 | BRepPrimAPI_MakeRevol: should Generated() report the Solid and planar Faces of a full revolution? | not relevant | A question about what history should say, not a defect |
| 1606 | `BRepOffsetAPI_MakeOffset` crashes (SIGSEGV) offsetting a planar face bounded by two concentric circles | already fixed | Decision 1788; here not a crash but "not done"; fixed |
| 1604 | BRepBndLib::AddOptimal returns a box smaller than the bspline-shape in 8.0.1 (correct in 7.8.1) | already fixed | Decision 1777; reproduced (-19.826 for -20) and fixed |
| 1577 | `GeomGridEval_Surface::EvaluateGrid` returns wrong points just below an interior BSpline knot, makes `BRepBndLib::AddOptimal/AddOBB` ~10-35x slower | already fixed | Decision 1775; reproduced (5.03 off) and fixed |
| 1576 | Crash on chamfer on complex shape | already fixed | Tried on its file: the chamfer fails with no crash |
| 1573 | UnifySameDomain breaks spherical faces with pole edges when the spheres have rotated frames | applies | Reproduced from its numbers: invalid at 37, 90 and 180 degrees, with no crash. Its partial fix isn't enough, by its own account |
| 1571 | Faulty shape with fillet on a rounded cube | applies | Reproduced on its file: invalid, volume 4e29. The public fillet fails |
| 1570 | Faulty shape with fillet beyond a radius | applies | Reproduced on its file: at 5.5 valid, with edges crossing (`bopcheck`), and clean at 5.0. Silent |
| 1569 | Faulty shape with fillet on simple shape | applies | Reproduced on its file: valid and self-interfering. Silent |
| 1568 | Crash on fillet meeting opposite edge | already fixed | Tried on its file: 1.5 fails with no crash, and 1.49 is valid. The fillet to the opposite edge itself is question 2 |
| 1567 | `BRepAlgoAPI_Fuse` of a partial torus and a tangent sphere gives an invalid, empty or wrong-volume result | already fixed | Decision 1818; reproduced (empty) and fixed: 26.7684 |
| 1566 | `BRepPrimAPI_MakeRevol` gives an invalid or wrong-volume solid for a circular profile edge | already fixed | Decision 1780; reproduced (0.5655 for 0.7101) and fixed |
| 1564 | License header of 8 TKGeomBase files says 'commercial software', contradicting LGPL-2.1 | not relevant | OCCT's licence header on 8 files: for the kept header, see NOTICE; nothing to change in behaviour |
| 1561 | HLRBRep_Algo returns no outline for B-spline surfaces with many more U than V knots — Contap_HContTool::SamplePoint swaps its grid indices | can't tell | Read: `Contap_HContTool.SamplePoint` is the same code; hidden lines, not tried |
| 1560 | BRepTools::Read hangs on BRep with a number longer than 31 characters | already fixed | Reproduced with a 46-character number: the port doesn't hang |
| 1559 | STEPCAFControl_Writer segfaults writing a CommonLabel dimension after any STEP read | not relevant | CommonLabel dimension through XCAF document writing; the port throws, and the case is rare |
| 1558 | BRepMesh generates degenerate triangles on a helical pipe with a periodic circular section | can't tell | Needs the issue's attached model; not reproduced in this pass |
| 1556 | IFSelect_WorkSession::GiveFileRoot/GiveFileComplete return a pointer into one process-shared buffer | not relevant | The port has no process-shared buffer (decision on statics, unit 10I) |
| 1546 | HLRBRep_Data: TableauRejection::Set reads one element past a row (bound tested after dereference) | can't tell | Hidden lines, read past a row: in managed code an exception at worst; not checked |
| 1543 | Modeling Algorithms - Boolean Cut with a helical swept tool silently removes nothing when the tool exits through a planar face (IntPatch loses the bou | already fixed | Reproduced with its tool: DZ 16.2, 16.4 and 16.6 removed nothing. Fixed with its two changes, for pieces partly out of the face: decision 1834; 5841.8 at 16.2 |
| 1542 | Division by Machine Precision in Intf_Tool::LinBox | can't tell | Read: `Intf_Tool.LinBox` has the same division; effect only on a nearly axis-parallel box |
| 1541 | Applying `ShapeUpgrade_UnifySameDomain` to a valid boolean cut (box - sphere) produces an invalid shape | already fixed | Reproduced from its numbers: invalid, 0.9543 for 0.9836. Fixed: decision 1831; valid, and the volume stays |
| 1536 | Violation of the the C++ One Definition Rule | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 1534 | Boolean cut on periodic cylindrical/conical face creates disconnected wire | can't tell | Its cutter isn't attached. The file before is valid and the one after invalid, so the Boolean can't be run |
| 1527 | Configuration, CMake - support GNUInstallDirs for Unix installation layout | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 1523 | Bug with DF_Label reading Chinese names | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1521 | Modeling Algorithms - Defeaturing cannot remove a corner round: adjacent-face extension is sized by the feature, not by the distance the faces must sp | can't tell | Needs the issue's attached model; not reproduced in this pass (defeaturing) |
| 1517 | Exact HLR emits occluded edges as visible: boundary edge misclassified as whole-arc restriction contour disables face hiding | can't tell | Needs the issue's attached model; not reproduced in this pass (hidden lines) |
| 1516 | VCPKG build fails for gcc 15+ and clang 22+ | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 1515 | Foundation Classes - OCCT_CLOCALE_POSIX2008 not defined on the BSDs | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 1508 | Public STEP AP242 assembly round-trip fixtures (counts + bbox) | not relevant | A request for a new feature, not a defect |
| 1495 | `ChFi3d_Builder::StoreData` cannot leave its degenerate-normal walk: the exit needs more steps than an `int` can count | applies | Read: the loop in `ChFi3d_Builder.StoreData` is the same; it needs degenerate normals, and could spin |
| 1494 | `BRepFilletAPI_MakeFillet/MakeChamfer::Build` discard their `Message_ProgressRange` | not relevant | Progress on fillets: a feature; the port keeps the parameter unused |
| 1461 | Data Exchange, STEP Export - Torus written as SURFACE_OF_REVOLUTION when MajorRadius - MinorRadius is 7e-12 | applies | Read: the same comparison. Near a horn torus the writer chooses by noise. See #1566 |
| 1460 | Data Exchange, STEP Export - Natural bound written as VERTEX_LOOP drops the seam of a trimmed periodic face | can't tell | Same writer; not tried |
| 1458 | Rename TKCAF and TKMesh-related toolkits to make them fit to the general TK-naming rule | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 1456 | BOPTools_AlgoTools3D::DoSplitSEAMOnFace(edge, face) SIGSEGVs on a null pcurve (unchecked BRep_Tool::CurveOnSurface, unlike its 3-argument sibling) | not relevant | A null dereference natively; here an exception, and BOPAlgo reports it |
| 1444 | Bad meshing of torus-like shapes with some specific curvatures | already fixed | Decision 1929: its file, 0.28 measured for 0.45 asked. The sphere decision 1929 left is fixed by decision 1953 |
| 1434 | GeomTools_Curve2dSet::Add / GeomTools_SurfaceSet::Add accept a null handle and defer the crash to Write(), unlike the CurveSet sibling | not relevant | Natively a crash on a null handle: the port's analyzers refuse null (nullable) |
| 1431 | ChFi2d_Builder::AddChamfer(edge, edge, ...) SIGSEGVs on a repeated edge pair (unchecked FindConnectedEdges status) | already fixed | Decision 1933: `fillet.connexion-error`, by a test |
| 1430 | Fillet crashes and self-intersections | applies | Reproduced on its file: no crash, but four of its five cases are invalid, which the public fillet reports, and the fifth valid and self-interfering |
| 1429 | HLRAlgo_PolyHidingData.hxx:43:5: error: 'gp_XYZ' does not name a type | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 1427 | Failing fillet on tapered surface | applies | Reproduced on its files: `tapered_2_9` fails where `tapered_2_8` works |
| 1421 | Crash on failing fillet | already fixed | Tried on its file: the fillet fails with no crash |
| 1419 | Modeling Algorithms - Missing implementations for exported methods | already fixed | occt-issues.md row: declared, never defined: left out |
| 1409 | Shape Healing - ShapeFix_ComposeShell::Perform/SplitEdges and ShapeUpgrade_WireDivide::Perform SIGSEGV on a null ReShape context | not relevant | A null context natively; the port's healing always makes one |
| 1405 | Static DRAWEXE fails with USE_RAPIDJSON=OFF due to unconditional XSDRAWGLTF registration | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 1398 | Resource_Manager::Debug and Storage_Schema::ICurrentData() race under legitimate multi-instance TDocStd_Application usage | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1396 | CDM_Application::myMetaDataLookUpTable and CDM_MetaData fields unsynchronized (race under concurrent document save/close) | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1393 | CDF_Application-cached storage/retrieval driver instances are not reentrant (SIGSEGV on concurrent Save/SaveAs of the same format) | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1389 | XCAFApp_Application::GetApplication()/CDF_Directory: unsynchronized process-wide singleton races, crash reproducible | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1387 | XCAFDoc_ShapeTool::theAutoNaming global race: RWMesh_CafReader save/modify/restore has no locking | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1385 | BOPAlgo_ArgumentAnalyzer self-interference: unbounded runtime (no checkpoint) + O(n) tangent-zone lookup | can't tell | Self-interference has no checkpoint: read; a slow-case matter for 11Q |
| 1384 | String's keyValues in TDataStd_NamedData have encoding problem | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1371 | MakeFillet on a closed B-spline rim silently returns a self-intersecting solid | can't tell | Needs the issue's attached model; not reproduced in this pass (fillet: goes with the fillet survey, corpus-failures.md) |
| 1368 | Data Exchange, Step Import - Zero value of semantic dimentions | not relevant | GD&T in the assembly document, which isn't ported ([unported.md](unported.md)) |
| 1367 | Foundation -  BVH_BoxSet::Center has improper return type. | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 1366 | Data Exchange - redundant instances of XCAFDoc_GraphNode are created in XCAFDoc_ViewTool | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1365 | Shape Healing - Exception on fixshape call | applies | Reproduced on its file: an evaluation failure from `fixshape`, on an input with bad flags. The public `Fix` reports a failure |
| 1364 | Modeling Algorithms - IntCurvesFace_ShapeIntersector results in a crash when used inside a loop | can't tell | Needs the issue's attached model; not reproduced in this pass (natively a crash) |
| 1363 | Data Exchange - Wire order change on STEP Exchange | applies | Reproduced on its file: edges 2 and 5 of the face swap on a round trip, and its area moves by 0.12 in 2939 |
| 1362 | Coding - IGESToBRep_CurveAndSurface's method has unreachable code | not relevant | IGES: not ported (unported.md) |
| 1361 | Visualisation - AIS_InteractiveContext::setObjectStatus not using ToPropagateVisualState() in AIS_InteractiveContext::setObjectStatus() | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 1360 | Modeling - Access violation using fuzzy boolean operations | can't tell | Needs the issue's attached model; not reproduced in this pass (fuzzy Boolean, natively a crash) |
| 1359 | Modeling - Read a bincad file throws an exception | can't tell | Its attachments are a 7z and a tar.gz, which this pass doesn't unpack |
| 1358 | Visualization - dynamic highlight results are not cleared by AIS_InteractiveContext::Redisplay() | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 1357 | Modeling Algorithms - ASSERT failed in BRepTools_History::AddGenerated() in debug mode | not relevant | A debug-build assertion, which the port doesn't have |
| 1356 | Foundation Classes, OSD_Host::InternetAddress() - possible NULL dereference on copying gethostbyname() result | not relevant | OSD_Host isn't ported |
| 1355 | Foundation - gp_GTrsf::SetVectorialPart sets scale to 0.0 causing non-invertable matrix | already fixed | Decision 1931: conversion now works; inversion already did, as it uses the matrix alone |
| 1354 | Visualization - update links of highlight drawers in SetAttributes | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 1353 | Tests - move tests bugs modalg_5 bug24157_* from group bugs modalg_5 to perf | not relevant | OCCT's documentation or tests only |
| 1352 | Mesh -  tricheck command incorrectly reports error on internal face | not relevant | Draw's own command or harness: Draw-lite has its own |
| 1351 | Data Exchange, XCAF - XCAFDoc_GraphNode doesn't copy backward links | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1350 | Shape Fix - FixMissingSeam produces extra wire with null area | already fixed | Tried on its file: `fixshape` gives a valid face with 17 wires from 18, so no extra wire |
| 1349 | Iges Import - Iges-reader reads invalid shape | not relevant | IGES: not ported (unported.md) |
| 1348 | Step Export - Valid shape is written to STEP-file as invalid (self-intersected) | can't tell | Needs the issue's attached model; not reproduced in this pass (a valid shape written self-intersecting) |
| 1347 | STEP - edge curve with identical vertices is reported on file generated by OCCT | not relevant | Its warnings come from a file written through the assembly document, which isn't ported ([unported.md](unported.md)) |
| 1346 | IGES Import - Parser does not recognize comments in Parameters section | not relevant | IGES: not ported (unported.md) |
| 1345 | STEP - Possible misuse of StepData_StepModel::SetIdentLabel() in StepAP209_Construct | not relevant | StepAP209 isn't ported (unported.md) |
| 1344 | Documentation -  compgraph parameter of STEPControl_Writer::Transfer () is missing | not relevant | OCCT's documentation or tests only |
| 1343 | Modeling - LProp_SLProps::IsNormalDefined() uses unapproptiate method of CSLib to calculate normal | can't tell | Read: same `CSLib.Normal` call; effect is on a tolerance only |
| 1342 | Data Exchange - Regression in DEBUG mode in test case de iges_2 B1 | not relevant | IGES: not ported (unported.md) |
| 1341 | Foundation - gp_Hyper empty constructor raise exception | already fixed | Decision 1789; read and fixed |
| 1340 | Modeling - SameParameter & SameRange flags are set incorrectly in ShapeBuild_Edge::CopyRanges | already fixed | Reproduced on its `min_shell.brep` and `fuse_valid.brep`: both invalid after Unify. Fixed: decision 1830; both valid |
| 1339 | STEP Export - Exception is raised during writing with ExternalReferences mode in STEP  XDE  document. | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1328 | OpenGl_PBREnvironment::bake() — mismatched texture unit in Bind/Unbind pair for environment map | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 1320 | Segmentation violation on exit from Draw after view initialization | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 1315 | BRepOffsetAPI_ThruSections — twisted / self-intersecting result reported valid | can't tell | Needs the issue's attached model; not reproduced in this pass (a twisted loft reported valid) |
| 1310 | ShapeUpgrade_UnifySameDomain returns a non-triangulatable shape for a closed pipe with circular spine and circular profile made of two semicircular ar | already fixed | Reproduced from its numbers: one face with no edges, and no triangles. Fixed: decision 1837 |
| 1297 | Modeling - SIGSEGV (null deref) in BRepFill_CompatibleWires::SameNumberByPolarMethod on mismatched ThruSections profiles | already fixed | Tried from its numbers: the loft reports failure, with no crash |
| 1285 | Data Exchange - USD Import Export | not relevant | A request for a new feature, not a defect |
| 1283 | Incomplete read STEP with special 'COMPOUND_REPRESENTATION_ITEM' | not relevant | A PMI item in the assembly document, which isn't ported ([unported.md](unported.md)) |
| 1279 | Data Exchange - VrmlAPI_Writer does not write all colors | not relevant | VRML: not ported (unported.md) |
| 1276 | HLR from cylinder's axis is extremely slow | can't tell | Needs the issue's attached model; not reproduced in this pass (slowness: belongs with 11Q) |
| 1238 | Entity missing when call 'XCAFDoc_DimTolTool.GetGDTPresentations()' | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1217 | Chamfer fails for bspline corner | can't tell | Needs the issue's attached model; not reproduced in this pass (fillet: goes with the fillet survey, corpus-failures.md) |
| 1177 | Fillet/Chamfer cannot cannot meet opposite edges | applies | Upstream fixed it in #1449, not ported, and it won't be (decision 1901): a fillet can't reach the opposite edge, except as decision 1847 does. `f713b1305f` |
| 1173 | Section of two solids results in a compound of edges | not relevant | A question of what a section returns: it returns edges, by design |
| 1162 | make MAX_TEX_NUMBER settable from code | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 1161 | Poor PSO_Perform performance | not relevant | PSO's speed: 11Q |
| 1138 | Missing shape labels for certain datum labels | not relevant | Datum labels in the assembly document, which isn't ported ([unported.md](unported.md)) |
| 1124 | Incorrectly computed sections | applies | Reproduced on its files: with approximation off, case 4 has 0.004 of its 40.53 length and two loose vertices, and with it on all four close. `BooleanOptions` and `Kerf.Modeling` now approximate (decision 1836). The cause with it off isn't found |
| 1123 | Incorrect minimum distance with intersecting parts | already fixed | Decision 1926: its four files through `Distance`: 0, 0, 8e-15 and 2e-15, and a Common of 2.96, 2.16, 1.48 and 0.92 |
| 1107 | mkoffsetshape bad geometry (wrench.brep) | applies | Reproduced on its file: invalid, two open shells. The public offset fails |
| 1106 | mkoffsetshape bad geometry (pacman.brep) | applies | Reproduced on its file: invalid. The public offset fails |
| 1081 | Conversion STP AP242 to GLB | applies | Reproduced on its file: invalid, and 106 s and 1.08M triangles to mesh on a loaded machine |
| 1062 | BRepOffsetAPI_MakePipeShell holes in the surface | can't tell | Needs the issue's attached model; not reproduced in this pass (pipe) |
| 1055 | Improve exception safety with smart pointers | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 1053 | Reserved identifier violation (header guards) | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 1043 | Unexpected memory footprint differences | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 1041 | Boolean fuse operation fails when rotation is involved | can't tell | Needs the issue's attached model; not reproduced in this pass (Boolean with rotation) |
| 1039 | Missing intersection edges when performing face-model BREP intersection | already fixed | Decisions 1932 and 1934: its files, Common and Cut add up to the slice's area, and the section has no loose ends |
| 1036 | Inconsistent Generated/Modified/IsDeleted implementations across BRepBuilderAPI classes Summary | applies | A real inconsistency in history across makers; a design job, partly done by Kerf.Modeling's own history. Medium |
| 994 | OFFSET_SURFACE did not parse correctly | already fixed | Reproduced on its file: 35 of 36 faces, and the shell open. The basis is C0 at a knot, so no surface was made. Fixed: decision 1838; 36 faces, 4 valid solids, and it meshes |
| 986 | Provide an approach to tracking Topo_Shape after using ShapeFix_Shape | not relevant | A request for a new feature, not a defect |
| 945 | BRepPrimAPI_MakePrism: Shell extrusion produces CompSolid with problematic internal faces | not relevant | By design: a prism of a shell makes one solid a face. Kerf.Modeling's question, not a defect |
| 905 | BOPAlgo_MakerVolume fails to orient shape correctly | applies | Reproduced on its file: one solid for two, with the warning that it can't orient the shape |
| 900 | Faulty shapes with a fillet on a straight base, sweep, and a curve in the opposite direction | applies | Reproduced on its file: invalid, volume 2927 from 9820. The public fillet fails |
| 899 | Faulty shapes with a fillet on a curved base, sweep, and a curve in the opposite direction | applies | Reproduced on its file: invalid, volume 62624 from 17069. The public fillet fails |
| 837 | Add stream output support for STL, GLTF and other file formats | not relevant | A request for a new feature, not a defect |
| 791 | Simplified wrapper for C++, Python and JS/TS | not relevant | A request for a new feature, not a defect |
| 775 | VRML import issue | not relevant | VRML: not ported (unported.md) |
| 737 | Fillet bending upward on one side, downward on the other | applies | Reproduced on its file: valid and self-interfering (F/F). Silent |
| 736 | Fillet operation produces shape with artefact | can't tell | Needs the issue's attached model; not reproduced in this pass (fillet: goes with the fillet survey, corpus-failures.md) |
| 724 | More than 10GB of memory needed for boolean operation between two simple faces | applies | Reproduced on its files: past 150 s, memory not measured. For 11Q |
| 694 | Artifact with a fillet on a straight base, sweep, and a straight edge | applies | Reproduced on its file: valid and self-interfering. Silent |
| 692 | Faulty shapes with a fillet on a curved base, sweep, and a curve in the same direction. | applies | Reproduced on its file: invalid. The public fillet fails |
| 691 | Faulty shapes with a fillet on a curved base, sweep, and a straight edge | applies | Reproduced on its file: invalid, volume 62252 from 17084. The public fillet fails |
| 666 | The result of ShapeFix_Shape do not make ShapeUpgrade_UnifySameDomain work. | applies | Reproduced on its file: Unify gives 7 faces directly and 5 after a STEP round trip. Both valid, so only less merged |
| 654 | Consider providing an official C API | not relevant | A request for a new feature, not a defect |
| 651 | AIS_MultipleConnectedInteractive Color Change | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 639 | Sweeping fail with RightCorner transition mode | applies | Reproduced on its files: not done. A failure, not a wrong shape |
| 588 | Extrema between two circular curves incorrect | already fixed | Tried on five of its cases: the shortest extremum matches `distmini` on each. Its patch's flag for infinitely many isn't compared |
| 575 | Incorrect HLR result | can't tell | Needs the issue's attached model; not reproduced in this pass (hidden lines) |
| 572 | Cone surface cannot be triangulated | already fixed | Decision 1930: its file, 342 triangles at 0.091 for 0.1 asked |
| 568 | OCC+VTK：The program crashed in debug mode | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 553 | ElCLib::AdjustPeriodic() failed to add a period | applies | Read: `AdjustPeriodic` uses the same test; needs a very small Preci. Small but delicate |
| 541 | Origin and color problems with SolidWorks STEP models. | not relevant | Origin and colours through the assembly document, which isn't ported ([unported.md](unported.md)) |
| 512 | STEP import fails if cascade unit it set to M | not relevant | The cascade unit is a global setting of the XS layer; Kerf's `StepReader` takes units from the file |
| 509 | Possible bug with hash code in NCollection_Map | not relevant | Kerf has its own hashes (decision on `ShapeMapComparer`) |
| 507 | Crash / invalid memory access when meshing STEP file | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have (a sanitiser's container overflow) |
| 505 | Documentation - Single documentation for all version (readthedocs.com) | not relevant | OCCT's documentation or tests only |
| 490 | Data Exchange - Structural storage and interface | not relevant | A request for a new feature, not a defect |
| 486 | Improve TDocStd_Document Memory Management via Lazy Loading | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 485 | BRepAlgoAPI_Common returns empty TopoDS_Compound for two faces visually intersecting | not relevant | Tried on its file: Common of two faces is what they share, so empty is right. The curve it wants is the section, which gives three edges |
| 484 | How to fix: Nothing to transfer | applies | Reproduced on its file: no shape, as the issue says. Why isn't looked at |
| 472 | HLR incorrect hidden line | can't tell | Needs the issue's attached model; not reproduced in this pass (hidden lines) |
| 456 | DRAW command reference manual | not relevant | OCCT's documentation or tests only |
| 444 | Use normal find_package/target_link_libraries/modern cmake | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 439 | WebGL Viewer Sample: Emscripten no longer exports malloc | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 437 | 7.9.0 7.8.0 AIS_ViewController::FlushViewEvents line 249 | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 434 | Export dll functions bug | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 430 | The model has defects when converting from step format to wavefront format | can't tell | Needs the issue's attached model; not reproduced in this pass (meshing) |
| 402 | Geom2dAPI_InterCurveCurve has 2 repeated intersection points | can't tell | Needs the issue's attached model; not reproduced in this pass (duplicate intersections) |
| 382 | One surface from step file is wrongly imported or tesselated | applies | Reproduced on its file: invalid, two faces with no triangles |
| 349 | Missing/incorrect faces in displayed STEP file | applies | Reproduced on its file: self-intersecting wires, ten faces with no triangles |
| 344 | Incorrect result when cutting a model with another model | can't tell | Needs the issue's attached model; not reproduced in this pass |
| 305 | GeomAPI_ExtremaCurveCurve fails to detect intersection [30316] | can't tell | Needs the issue's attached model; not reproduced in this pass (extrema of offset curves) |
| 304 | Modeling Algorithms - Sweep with acute angle in spine produces wrong intersection results [24991] | applies | Reproduced on its files: `BRepFill_Sweep::BuildEdge` throws. A failure, not a wrong shape |
| 303 | Error when finding the intersection between a new face made after a draft and the inner face of body [32857] | can't tell | Needs the issue's attached model; not reproduced in this pass (draft) |
| 302 | Modeling Algorithms - TCollection_IndexedDataMap::FindFromKey error when sweeping circle along BezierCurve [25110] | applies | Reproduced on its files, the same as #304 |
| 275 | Intersection produces strange result for given surfaces | can't tell | Tried on its file: one edge in a small box, and no way to judge it without its picture |
| 249 | Multiplatform compilation testing on GH | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 245 | Boolean cut has issues when it involves a common Edge between argument and the tool. | applies | Reproduced on its files: the cut removes about 14,000 of 756 million, with a sphere of 524 million, and warnings. A comment puts the cause in the revolution/sphere intersection at the pole |
| 191 | BRepOffsetAPI_MakeOffset fail | can't tell | Needs the issue's attached model; not reproduced in this pass (offset) |
| 176 | MoveTo() from AIS_InteractiveContext sometimes doesn't redraw the viewer, even if the flag is set to true | not relevant | Visualisation or its OpenGL: not ported (unported.md) |
| 145 | Standard_Macro.hxx conflicts on Windows defines | not relevant | A C++ matter (compiler, headers, ODR, pointers) that managed code can't have |
| 71 | Optimizing String representation for STEP | not relevant | Kerf has its own string handling (decision in phase 8) |

## Issues closed since 8.0.1

| # | What it says | Status | Why |
|---:|---|---|---|
| 1689 | Use map to store attribute for TDF_LabelNode | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1660 | Documentation - Generate llms.txt for the reference manual | not relevant | OCCT's documentation or tests only |
| 1646 | Linking fails on Linux systems without legacy libGL | not relevant | Build, packaging or repository matter: Kerf has its own build |
| 1581 | Data Exchange, STEP - plus/minus tolerance limits written as untyped MEASURE_WITH_UNIT (off-by-one stripping "POSITIVE_") | applies | Upstream fixed it in `71658f6c8f`: plus/minus limits written untyped; not ported. Data exchange, small |
| 1580 | Data Exchange, STEP - RUNOUT_ZONE_DEFINITION written without its orientation, and for every diametral tolerance zone | applies | Upstream fixed it in `71658f6c8f` and `8f95c78c7c`: runout zones; not ported. Small |
| 1579 | Data Exchange, STEP - Complex triangulated face: fan with a repeated index and unmatched normal count give an invalid triangulation | applies | Upstream fixed it in `22a9ead2a0`: triangulated face fan; not ported. Small |
| 1578 | Unresolved STEP reference binds the entity at the record number equal to its identifier | applies | Upstream fixed it in `71658f6c8f`: an unresolved reference binds another entity. Small |
| 1565 | Variant analysis: 3 unfixed sibling safety gaps in OCCT | not relevant | Overflow guards in native code |
| 1563 | CAD Assistant XDE Deletion Bug: Deleting a shared instance purges the geometric prototype | not relevant | OCAF or CDF documents: not ported; Kerf has an immutable assembly tree (unported.md) |
| 1539 | Data Exchange - BinTools cannot serialize Geom2dEval_ArchimedeanSpiralCurve | not relevant | Spiral curves (Geom2dEval) aren't ported |
| 1530 | Update obsolete Unicode license in `NCollection_UtfIterator.lxx` | not relevant | NCollection_UtfIterator isn't ported |
| 1496 | BRepAlgoAPI_Common silently returns empty (or negative) for two geometrically-identical swept solids when they are distinct operands; IsDone() is True | can't tell | Upstream fixed by `8f817f5da5`; needs the issue's swept solids |
| 1488 | Vector scale error in DRAW | not relevant | Draw's own command or harness: Draw-lite has its own |
| 1452 | Crash when atttempting to apply a negative offset to a small diamater circular wire. | already fixed | Decision 1788: `e177d7e68c` ported |
| 1451 | [FreeCAD] OCCT does not support revolved geometry without "parting lines" | not relevant | A request for a new feature, not a defect |
| 172 | Fillet can not round a face | can't tell | Upstream fix is `f713b1305f`, not ported, and it won't be (decision 1901). bugs/modalg_7 OCC25478 cases are BAD for it |

## OCCT's commits since 8.0.1

| Commit | What it says | Status | Why |
|---|---|---|---|
| `53706f78a8` | Modeling - Fix intersection, fitting and profile regressions (#1653) | applies | The closed polygon and `LeastSquare` (decision 1822) and the profile's weights (decision 1928) are ported. Left: Newton's line search and `IntCurveSurface`'s scaled transitions, not seen to matter |
| `8f95c78c7c` | Data Exchange - Support companion streams and improve STEP PMI handling (#1647) | applies | STEP PMI units and references; companion streams are a feature. Small pieces |
| `71658f6c8f` | Modeling - Fix geometry evaluation and data exchange regressions (#1644) | applies | The offset surface and `ApproxInt` are ported (decisions 1811, 1812); rest to read |
| `8f817f5da5` | Modeling - Fix numerical accuracy and geometry differences (#1640) | applies | Ranked above; large |
| `09b2da2c16` | Coding - Fix MSVC warnings (#1636) | not relevant | Compiler warnings |
| `ec0167b409` | Modelling - Fix geometry regressions and reduce repeated evaluations (#1634) | can't tell | Regressions, repeated evaluations: to read piece by piece; the blending index change is for pointers Kerf doesn't have |
| `8f5a737b54` | Foundation Classes - Modernise Unicode strings and iterators (#1531) | not relevant | Unicode strings aren't ported |
| `5804e8ecbb` | Visualization - Restore font compatibility and fix glyph bitmap handling (#1627) | not relevant | Visualisation |
| `8d929b1ae2` | Modeling Data - Defer BRepGraph integration pending API review (#1628) | not relevant | BRepGraph isn't ported |
| `5f146315f9` | Coding - Update Unicode notice in NCollection_UtfIterator (#1532) | not relevant | A notice |
| `3f3d32d9f3` | Modeling Algorithms - Fix continuity, tolerance and feature construction (#1620) | applies | Continuity, tolerance and feature construction: to read; the shallow-corner tangent test looks useful for fillets |
| `88cc61507b` | Modeling - Fix B-spline evaluation performance regression (#1616) | not relevant | Performance of a path Kerf ports differently: 11Q may look |
| `23d68aeb95` | Visualisation - Fix OIT fallback and selection mode reactivation (#1614) | not relevant | Visualisation |
| `22a9ead2a0` | Data Exchange - Add XCAF stream I/O and fix STEP and GD&T handling (#1611) | applies | STEP triangle strips and fans wound wrong; #1579. Small |
| `0089343103` | Modeling Data - Optimize curve-on-surface deviation checking (#1595) | not relevant | Speed of deviation checks: 11Q may look |
| `e3f073d8a4` | Coding - Update Copilot instructions (#1584) | not relevant | Repository |
| `3270746444` | Foundation Classes - Improve numerical robustness and evaluation efficiency (#1585) | applies | Partly ported: a point mirror's inverse, decision 1778. The rest is mostly numerics that can change results |
| `f6a4ec34e5` | Tests - Skip unavailable reference results (#1590) | not relevant | Tests |
| `31dfb16bc1` | Tests - Skip unavailable reference results (#1586) | not relevant | Tests |
| `f2929057ea` | Modeling Algorithms - Fix periodic fillet spine closure continuity (#1501) | applies | A closed fillet spine's C0 knot: to port with the fillet list |
| `36b368da97` | Application Framework - Resolve deep XBF references after partial read (#1528) | not relevant | XBF isn't ported |
| `8e05e086fe` | Configuration - Fix BUILD_SOVERSION_NUMBERS=0 handling (#1199) | not relevant | Build |
| `4b801ad354` | Modeling - Do not always cache `BRepClass3d_SolidClassifier` (#1046) | not relevant | Speed: the classifier's cache |
| `ecfb7ada2c` | Modeling - Lazily instantiate `BRepTopAdaptor_TopolTool` (#1045) | not relevant | Speed |
| `6faa5d6c7c` | Modeling - Add immutable BRepGraph revisions and transactions (#1520) | not relevant | BRepGraph isn't ported |
| `c7ea5bf865` | Coding - Eliminate mutable static state and obsolete code (#1519) | not relevant | Mutable statics: Kerf did this in unit 10I |
| `573b4be651` | Data Exchange - Harden malformed input handling (#1514) | can't tell | Malformed input: STEP spline checks and oriented edge cycles should be compared with the port's guards |
| `8a817a4a19` | Modeling Data - Improve BSpline and Bezier surface grid evaluation (#1511) | already fixed | Grid evaluation rework; the wrong point is fixed by decision 1775 otherwise |
| `8353236d05` | Modeling Data - Improve direct B-spline curve evaluation (#1512) | not relevant | Speed |
| `8cf4d7cea6` | Coding - Eliminate mutable globals and obsolete debug code (#1509) | not relevant | Mutable globals: unit 10I |
| `287b5271d3` | Coding - Fix MSVC warnings (#1510) | not relevant | Compiler warnings |
| `8fa86ae687` | Coding - Eliminate mutable globals and obsolete debug code (#1505) | not relevant | Mutable globals: unit 10I |
| `a935e43812` | Tests - Limit DRAW execution Action (#1504) | not relevant | Tests |
| `c7da6ad918` | Coding - Fix compiler and static-analysis warnings across toolkits (#1502) | not relevant | Warnings |
| `3d097a0328` | Documentation - Fix documentation generation (#1499) | not relevant | Documentation |
| `59d88e7b1c` | Coding - Windows unused font function (#1498) | not relevant | Visualisation |
| `b185eaf66a` | DRAW - misprint in scale and 2dscale commands (#1493) | not relevant | Draw's command |
| `8ab68fe939` | Visualization - Rework BRep font and text construction (#1485) | not relevant | Visualisation |
| `1a2e039ab9` | Coding - Clean up the trace debug files (#1487) | not relevant | Debug files |
| `5b145ea48a` | Coding - Remove obsolete ChFi3d debug instrumentation (#1490) | not relevant | Debug code |
| `9248874be6` | Modeling Algorithms - eliminate global state from exact HLR (#1489) | not relevant | Mutable statics |
| `04082876f3` | Modeling Data - Rework ExtremaPC and add 2D algorithms (#1480) | can't tell | A rework of point/curve extrema; a port could change results: not for now |
| `8c31e671c2` | Modeling Data - Modernize GeomLib storage and evaluation (#1482) | not relevant | Storage |
| `bd81fbfaed` | Modeling Data - Unify Adaptor2d curve evaluation API (#1484) | not relevant | An API change |
| `2fa332705f` | Foundation Classes, TKMath - Rework numerical algorithms (#1479) | can't tell | A rework of numerics: to read |
| `e62f6b2483` | Shape Healing - Restore removed connectivity and sewing tools (#1476) | not relevant | Tools 8.0 removed: Kerf has them or doesn't need them |
| `8f4df11e17` | Draw - Fix C4459 warning in BRepTest_CheckCommands (#1477) | not relevant | Warnings |
| `76667c8d7e` | Coding - Fix high-severity CodeQL multiplication warnings (#1478) | not relevant | Warnings |
| `219d198c5d` | Coding - Restore deprecated type alias headers (#1475) | not relevant | Headers |
| `4478114223` | Application Framework - Remove obsolete UTL package (#1474) | not relevant | Not ported |
| `e4da2bbd16` | Modeling - Modernize BRepGProp_Gauss integration storage (#1473) | not relevant | Storage |
| `3ed71f8914` | Data Exchange, STEP - add FieldKind enum and fix field handling (#1470) | can't tell | STEP field handling: to compare |
| `1349657592` | Modeling - Correct BRepTools transformations (#1472) | already fixed | Decision 1936: the general transform of a mesh. `BRepTools_TrsfModification`'s part is the same but for a mesh with a placement of its own, which nothing here makes |
| `ec6533d50d` | Documentation - Improve Doxygen configuration and build (#1469) | not relevant | Documentation |
| `4719d5fa98` | Modeling Data - Refactor Geom_OsculatingSurface (#1468) | not relevant | A refactoring |
| `4c8d3df98d` | Tests - Fix DRAW test migration (#1464) | not relevant | Tests |
| `8abc975c07` | Revert "Mesh - Fix iterator invalidation in Delaun triangle-kill cascade (#1450)" (#1459) | not relevant | A revert |
| `7d2efad9c8` | Mesh - Fix iterator invalidation in Delaun triangle-kill cascade (#1450) | not relevant | Reverted upstream |
| `6bf468c87c` | Data Exchange, IGES - Fix stream rework for corrupted files (#1455) | not relevant | IGES |
| `07e6d8d40e` | Modeling Data - Fix AdvApp2Var Jacobi maxima written to the wrong workspace slot (#1418) | can't tell | AdvApp2Var Jacobi maxima written to the wrong slot: one line. To check: the regression it cites may postdate the port |
| `e177d7e68c` | Modeling - Fix circular wire handling in BRepOffsetAPI_MakeOffset (#1453) | already fixed | Decision 1788: ported |
| `5eba9aff4f` | Tests - Migrate DRAW tests to Gtests (#1448) | not relevant | Tests |
| `0f7ab882d5` | Foundation Classes, NCollection - optimize iteration workflow (#1446) | not relevant | Speed of collections |
| `f713b1305f` | Modeling Algorithms - Enhance fillet and chamfer handling. Part 2 (#1449) | applies | Fillets to the opposite edge, #1177: large. Not ported, the owner's decision (1901) |
| `cbca11e975` | Modeling - Enhance fillet and chamfer handling. Part 1 (#1293) | applies | As above |
| `0d47342863` | Coding, Modeling Data - Clean up dead headers (#1439) | not relevant | Headers |
| `4c2c356490` | Data Exchange, IGES - Add stream-based I/O support (#1406) | not relevant | IGES |
| `1cb09f481b` | Modeling - Optimize exact face classification fallback (#1400) | not relevant | Speed: 11Q may look |
| `caad597813` | Coding, Modeling Algo - Clean up dead headers (#1442) | not relevant | Headers, tests or repository |
| `e616f301da` | Coding, Data Exchange - Clean up dead headers (#1438) | not relevant | Headers, tests or repository |
| `e521a4a7a7` | Coding, OCAF - Clean up dead headers (#1441) | not relevant | Headers, tests or repository |
| `4bcc7dae06` | Coding, Foundation - Clean up dead headers (#1437) | not relevant | Headers, tests or repository |
| `ea716e3fa4` | Coding, Draw - Clean up dead headers (#1440) | not relevant | Headers, tests or repository |
| `a2a9a8d96f` | Configuration - Clean up unsupported scripts and templates (#1428) | not relevant | Headers, tests or repository |
| `299344fc95` | Coding - Move PR GH template (#1423) | not relevant | Headers, tests or repository |
| `7219e39b2c` | Coding - Bump version to 8.1.0.dev1 (#1422) | not relevant | Headers, tests or repository |
