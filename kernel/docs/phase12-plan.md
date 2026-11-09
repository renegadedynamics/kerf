# Phase 12 plan: after 0.3.0

> **Agreed, 7 November 2026** (decision 1893), from the outline the owner asked for. Section 1 is started. The next
> release is 0.4.0, with no fixed date: when sections 1 to 3 are done and the corpus shows no new failures. Sections 4
> and 5 ship with whatever is ready by then.
>
> **0.4.0 prepared, 9 November 2026** (decision 2002), for the owner to test and publish: units 12A to 12Y as their
> rows say. 12F, 12G, 12J, 12Q and 12R carry on. Phase 12 stays open until the owner closes it.

## The goal

Phase 11 met its speed targets (recalculation 265 ms against native's 685, 126 MB allocated per recalculation against
450 at 0.2.0). The largest risk left is geometry that fails or comes out wrong. So phase 12 is **trustworthy results
first, then speed**.

Phase 11's standing directions still hold: no operation reports success for an invalid result (decision 1742); Kerf isn't
bound by OCCT's behaviour (decision 1743); wasteful cases go on being found and fixed, to the bar of 5% over many different
operations or 20% on a slower or larger one (decisions 1734 and 1829); and look at the worst case, not only the typical.

## 1. The owner's answers (decision 1892)

Small, and first.

| Unit | What | Done when |
|---|---|---|
| **12A** No clean with nothing to merge | Skip the clean after an extrusion when the profile has nothing to merge: about 2% of Bento's bytes, and 4% to 5% on examples of many small solids (performance.md). | Results shown to be the same over far more than Bento's examples, including the order of faces and edges and the history, or the cases that differ are excluded. **Done:** the same shape over 42,344 generated extrusions; a quarter to a half of an extrusion's bytes (decision 1895, performance.md) |
| **12B** Exact planar centres | `Center` and `Plane(face)` take a planar face's centre in closed form, as its volume and area already do (decision 1871). A fifth of the harness's samples. The last digits of what's built from one change, on purpose. | Before and after in time and bytes, and every changed figure in a test checked from the geometry. **Done:** no expected figure changed; a quarter of `Center`'s bytes (decision 1894, performance.md) |
| **12C** Fillet limits | A radius within tolerance of the limit is the limit. Above it, refuse at once from the geometry, where a box just above its limit took 3.4 s. Narrow `MaxFillet`'s search, from (0, twice the box's diagonal). Upstream's `f713b1305f` isn't ported. | The cases in fillet-design.md behave so; probes counted before and after. **Done** (decisions 1898 to 1901; fillet-design.md, section 8) |
| **12D** Bento's load | **Done, for the owner to merge in Bento** (decisions 1896 and 1897, [12d-bento.md](../../modeling/docs/12d-bento.md)): through Bento's real loader, 2,205 ms to 330 ms on a loaded machine. A row in the rig that loads Kerf as Bento does, then a Bento branch that loads Kerf from its files outside the collectible context, so a first operation can approach 170 ms from 2,200 ms. Kerf then doesn't unload with the extension. | The row in the rig; the branch's suites passing, for the owner to merge; what the owner must check by hand listed |

## 2. Wrong or failing results

The main work, with silent errors first.

| Unit | What | Done when |
|---|---|---|
| **12E** The silent Cut | The Cut of cylinders at radius 0.5 that comes out 0.8% wrong and says nothing, as the volume check's margin lets it through. | Right, or a failure carrying the shape. **Done:** right; the cause was the clean's merge, and it wasn't the only one (decisions 1922 and 1924) |
| **12F** The equal tee | Fillets on the equal tee: 514 of 2,508 wrong or failing. | The rate falls, or the cause is written down. **Cause written down:** ChFi3d's walk near the tangency, 514 still (decision 1925) |
| **12G** Radius 500 | Cylinders of radius 500: 21 to 22 of 113 steps fail, all loudly, up from 17 to 18. Why it rose, then the cause. | The same. **Part done:** 13 and 13; an absolute reach was the cause of most; what's left is in reliability.md (decision 1923) |
| **12H** Hang risks (11T) | The suspect loops in reliability.md, and the rest of its ranked list of failure islands. | Each bounded, or shown to be bounded by something other than a user's input. **Done** (decisions 1913 to 1917; reliability.md, section 4b): about 220 loops; 1,000 damaged files, none that a cancel can't stop |
| **12H2** Section 2's loose ends | Meshes out of memory on damaged files; the `while` and `do` loops not read; the two edge-order cases. | Each bounded so it fails loudly, or written down. **Done** (decisions 1942 to 1946; reliability.md, section 4d): the 6 out of memory are refused, 1 false failure in 752; of 3 that main didn't stop in 150 s, 1 is cancelled and 2 still run out of a 1.5 GB heap; 110 loops read, 13 bounded, about 480 left; the edge-order cases stay |
| **12I** The corpus | The 59 FAILED, the 502 BAD, and the 48 issues that couldn't be told. | Each attributed, as in corpus-failures.md, and fixed where the fix is plain. **Attributed; 3 silent faults fixed** (decisions 1918 to 1921; corpus-failures.md). 396 of the BAD stand on their scripts alone |
| **12J** OCCT's defects (11S) | Carry on down occt-defects.md, from the top. | Ongoing. **First pass done** (decisions 1926 to 1936): #1123, #1444, #572, #1039, #1659, #1643 right where they were wrong or failing; solids of no volume, a face against a solid, and a fuse that lost part of an input now fail; #1355, #1632, #1431 and upstream `53706f78a8` and `1349657592` in part. Left at the top: #1629, #245, #1639, the self-interfering fillets, a sphere's mesh (for the owner) and a cone's offset |
| **12X** A user's agent's three reports | `ConvexPolyhedron` throwing on valid points; `MakeConvexHull` sampling 2,000 points an edge; `Renderer` ignoring opacity. | Each fixed at its cause, or written down. **Done** (decisions 1993 to 1999): a hull is a valid solid for any points with volume, one face a flat side, nothing sewn, 640 points 1,272 ms to 25 ms, and `MakeConvexPolyhedron` gives it as a `Result`; the 2D hull exact where its lines touch, 100 to 5,000 times quicker; see-through items sorted at each sample, opaque scenes byte for byte as before. Found, for the owner: sewing overflows the stack on about 1,000 faces, and the kernel's deflection sampling can miss a lobe (occt-issues.md); constructors that can throw on valid input listed (1999) |

## 3. Fillets a user would notice

fillet-design.md's stages, in the order it recommends. They run only after ChFi3d has failed, so nothing that works today
changes. Each result is checked independently of how it was built, by volume, area and validity. **The sizes are
judgements, and could be out by half either way.**

| Unit | What | Size | Gets |
|---|---|---:|---|
| **12K** Rims (stage 1): **done** (177 lines; decision 1904) | Better messages, as for straight edges; limits for circles | 100 | "The most that fits" on a cylinder |
| **12L** Circular edges (stage 3): **done** (290 lines; decisions 1905 to 1907) | Plane and coaxial cylinder or cone: a revolved tool, the same check by Pappus | 300 to 600 | B2, Q3, W7; a disc's rim; full rounds; hemispheres |
| **12M** Corners (stage 2): **done** (about 600 lines; decisions 1908 to 1911) | Spherical patches where three blends of one radius meet; slanted ends | 500 to 1,000 | Cube to sphere; bar to capsule; bug32931, bug25478_2 |
| **12V** Chamfer limits: **done** (about 280 lines; decisions 1985 to 1987; `MaxChamfer` added) | A chamfer past the tightest convex arc or ellipse in a plane beside its edges, proved not to fit, refused before ChFi3d, on the chain ChFi3d carries it round and by the length on that face. The one case here asked before ChFi3d, not after it fails | 100 | "The most that fits" on a rounded outline; 33 of 132 runs past a limit, all by a length and an angle, had come back valid but wrong |
| **12W** Chamfers past tight corners (the owner, 8 November 2026): **done** (about 620 lines; decisions 1988 to 1992; [chamfer-corners.md](chamfer-corners.md)) | `TightCorners.Collapse`: a convex arc tighter than the chamfer collapses to the point where the straight insets meet, the chamfer over it an oblique cone tangent to its neighbours; whole loops of a planar face, sides square to it; the rest refused saying why. Off by default | | A rounded prism's or housing's end past its corners' radius, by the closed form |

**Then stop and look.** General-surface fillets (stages 4 and 5) and replacing ChFi3d (stage 6) wait for failures from
Bento's users that justify them.

## 4. Speed and memory

Ongoing, to the owner's bar.

| Unit | What |
|---|---|
| **12N** Unexplained rises: **explained** (decisions 1938 to 1941) | Extrude until's +3.6 MB, a thick solid's +2.2%, and `Pump_Nut.brep`'s parallel mesh, 5.86 to 6.81 MB. The thick solid's is fixed (rounding taken as crossing, decision 1941); extrude until's is decision 1881's check and the mesh's the rig's noise, both under the bar. With it, decision 1918's cone: round joins at an apex retried with mitres |
| **12O** The mesher (11O): **done, and the flag has gone** (decisions 1953 to 1955) | Meet the deflection at a sensible triangle count, so that `ControlAllSurfaces` can go altogether (decision 1680). Measured from the surface over `data/occ`, Bento's examples and spheres, tori, cylinders and cones: only spheres still missed it, up to 2.06 times, from the scaling of their parameters. Now within it everywhere at an absolute deflection with the grid's own triangles (worst: a torus 0.98); the flag added triangles and made cones 2.6 times out, so it went. A mesh with no face meshed fails; `Delaun`'s dead circles are swept, 28% less allocated over `data/occ`. Left: a relative deflection keeps OCCT's misses |
| **12P** Wasteful cases (11Q): **done** (decisions 1947 to 1951) | Down the slowest list: the corpus's slowest cases, the rig's slowest suites and Bento's examples. With it, `Kerf.Modeling`'s offsets checked and retried (1947, 1948), and the rig's bytes the median of its runs (1949). A curve adaptor keeps a B-spline's ends: `buc60462_2`'s check -35% of samples (1950); the rest written down (1951) |
| **Perf search** (the owner, 8 November 2026: "continue to speculate and investigate"): **done** (decisions 1971 to 1973) | Bento's examples against f5a7bd7a2, 12P's candidates, phase 12's checks and the slowest suites. A placed, meshed shape's validity check -48% of its bytes (`bug26701`, 1971); ShapeFix -5.7% and the STEP import with healing -2.9% (1973); the rest measured and under the bar, and the ideas not tried ranked in performance.md, "The perf search" (1972) |
| **Perf search, second round** (the first round's five ideas): **done** (decisions 1975 and 1976) | Each idea counted over 561 corpus cases and the slowest, kept only over decision 1829's bar and with the same results. A frozen 2D B-spline keeps the boxes of its arcs: `mkvolume` -19% of its bytes over the corpus's 74 cases, 20% to 37% on 15 of the 20 over 150 MB, `bug25232_9` -10% (1975); the other four under the bar (1976) |
| **12U** The mesh guards' cost (decisions 1929, 1953): **done** (decisions 1977 to 1979) | The torus's and sphere's guards cost 14% to 18% in time and 5% to 12% in memory on the bottle and `Axis_of_bearing.brep`, for the same meshes. A quarter of the checks was a set of node pairs growing; a pass now marks its links by index (OCCT's map kept for the passes after the first), and a node's normal is estimated once, not once a link. Over `data/occ`, 5.3% fewer bytes and the angular check's estimates from 8.0% of the samples to 1.8%; `Axis_of_bearing.brep` 10.7% fewer bytes; the same meshes. Not done: skipping a face proved within the deflection, which would change its recorded deflection |
| **12Y** Booleans on ordinary parts (a user's agent: 1.3 to 1.4 times build123d's, its suite twice as long): **done** (decisions 2000 and 2001; performance.md, "Booleans on ordinary parts") | Reproduced on eight extrude-and-cut parts, 145 Booleans, against native OCCT 8.0.1: the kernel's Boolean and clean are 0.5 to 0.7 times native's, and the gap was `Kerf.Modeling`'s two checks (decisions 1742, 1860), about as much again as the Boolean. The volume check kept a result's volume for the next Boolean (its time halved) and a classifier keeps a frozen solid's edge-face map (a second boss's fuse a third fewer bytes): the parts now 0.94 times native at the quietest (from 1.12), the same results. The rest of "twice as long" is a fresh process's JIT and tiering, about 3 s over the first 300 Booleans. The checks are still about 40% of `Kerf.Modeling`'s Boolean time, for the owner |
| **12Z** A fillet's cancel slower in 0.4.0 (Bento's `AnEditStopsACellInsideALongFillet`, 2.5 to 2.9 s against 2 s): **done** (decisions 24201 and 24202), on `release/0.4.0` | Reproduced in Kerf: every edge of a plate of 900 holes, cancelled at 300 ms, returned 1.8 to 2.4 s later (0.3.0: 4 to 28 ms). Every fillet reads its edges' limits before ChFi3d, with no look at the token, and asking whether each hole's far rim was tangent to the plate's top of 904 edges took 2.1 s. Each edge is a check now, as are the builds after ChFi3d fails, and faces turned about a rim's axis and well apart aren't sampled, as a straight edge's weren't: the reading 30 ms with the same answers, and a cancel within 70 ms at 17 points (0.3.0: 78 ms). A test of the cancel on that plate |

## 5. Tests and surveys

Fitted around the rest.

| Unit | What |
|---|---|
| **12Q** Coverage floor (11I): **done for now** (decisions 1961 to 1970) | Hand-written engine packages over the floor, by real tests. Measured the same way before and after (Microsoft's coverage over the kernel's unit tests, the public API's, the modelling layer's, the oracle replays and Draw-lite's default samples; no whole corpus). Over the floor now: FilletSurf 78.6 to 88.0%, Geom2dInt 71.0 to 87.4%, GeomBndLib 78.8 to 85.4%, StepToGeom 65.2 to 84.8%, STEPConstruct 75.3 to 82.0%, TopoDSToStep 75.8 to 81.4%, Geom2dHatch 78.1 to 81.2%, AppParCurves 73.0 to 81.0%, HLRBRep 76.8 to 80.3%, Draft 75.4 to 80.2%. Still under, with reasons in `coverage-gaps.tsv`: BiTgte 66.2 to 74.1%, STEPControl 71.4 to 74.9%, StepToTopoDS 63.5 to 73.8%, BRepApprox 33.3 to 55.6%; IntCurveSurface and Hermit unchanged. Nine fixes found by the tests: the rolling ball's supports and contact (1961), STEP's faceted B-rep of a solid (1962), B-reps with voids written and read whole (1963), boxes of half-infinite conics and slanting rays (1964), a STEP spline its poles can't hold (1966), a curvature point inside a line (1967), a rotation about any axis as yaw, pitch and roll (1968), interior points on 2d curves (1969); more in `occt-issues.md` |
| **12R** Tests of their own (11J): **a slice done** (decisions 1965, 1970) | A direct call and a checked result, area by area. This slice: exchange's STEP and BREP options, read limits, units and models, `Step.Read` from a file; the camera, the drawing and SVG options and line styles; a box's equality; the offset options. Public members with no direct test: 279 to 207 (`direct-members.md`). Found: a STEP file past the limits failed as `exchange.failed`, not `exchange.limit-exceeded` (1965) |
| **12S** 2D offsets: **done** (decisions 1956 to 1960) | The survey: [offset2d-survey.md](offset2d-survey.md). The 77 are OCCT's known failures on B-spline outlines, none the medial axis; Bento's profiles right in a sweep of 1,914, but for an inward offset past a curve's sharpest bend (up to 1% off, for the owner). Fixed: an offset that closes up or splits fails saying so, or keeps its pieces, where it threw or gave an empty sketch |
| **12T** 2D offsets, the owner's decisions: **done** (decisions 1980 to 1984) | The polyline for an edge curving tighter than the distance at 0.001 of it, OCCT's 0.01 where that fails: inward offsets past a curve's sharpest bend within 0.1% (the sweep's silently wrong 12 to 5, the rest the checker's), 22 corpus cases' edge counts intended differences, 1 known; a finer offset with an edge the wrong way round falls back too (decision 1984). `Kind.Tangent` gone (it did nothing useful in 2D or 3D); Bento's `"tangent"` gone on its branch `kerf/offset-kinds`. The 77 left as OCCT's |

## Not in this phase

- General-surface fillets and replacing ChFi3d, as above.
- Coverage of generated STEP entities.
- Bento's caching (11F), which is the owner's with another agent.
- DXF export and text (fonts, `Text`, notes and title blocks in drawings): 0.5.0 (decision 2004).

## How the work runs

As in phase 11: at most three helpers at once, each on a branch and worktree of its own, then the merge gate, one at a
time. Results stay identical unless a unit says otherwise and records why. Timed claims come from a quiet machine, and the
rig compares against `docs/benchmarks/2026-11-07-f5a7bd7a2.tsv` (decision 1902). Nothing is pushed, except by the owner.
