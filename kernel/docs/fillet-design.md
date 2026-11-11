# Fillets: where they stop, and what could replace them

*7 November 2026; section 9 is what units 12K to 12M built, section 10 what unit 15Q found of faces that narrow to a point, section 11 stage 4a, built by unit 15N, section 12 what unit 15T found of stage 4b's corners, section 13 what unit 15R found of why ChFi3d's own fillets are off the ball, and section 14 stage 4c, built by unit 15X, the piece of a sphere where the ball can't move. A study for the owner's decision, who asked on 6 November: "if I have a 10mm cube, I can't apply a 10mm fillet to a single edge - but a 9.999mm fillet is fine... If a rewrite would fix that then that's another reason to consider bringing this work forwards." Decisions 1847 to 1852. No new algorithm is written here. The owner has since agreed its recommendations (decision 1892); [section 8](#8-limits-as-built-unit-12c) has what unit 12C built from them.*

**In short.** The 10 mm case is fixed, without a rewrite (decision 1847). That class of failure is not one failure of ChFi3d's mathematics: ChFi3d computes the right surface and then can't remove a face. **A rewrite of ChFi3d isn't justified yet, but a rolling-ball path that runs only where ChFi3d has failed is,** one analytic stage at a time. Its next stage is circular edges, the largest group among the corpus's fillet failures (8 of 23, 3 of them exactly this limit). Don't port `f713b1305f`.

Everything here says if it's measured, inferred, or from elsewhere. Timings are from a loaded machine.

## 1. How ChFi3d works, and why it fails at a limit

ChFi3d is about 33,000 lines, and with its surface kinds, spines, blend functions and walking about 65,600 (`ChFi3d`, `ChFiKPart`, `ChFiDS`, `BRepBlend`, `Blend`, `BlendFunc`). The steps, for each contour (a chain of tangent edges, the "spine"):

1. **Surfaces.** `PerformSetOfSurf`. For a pair it knows (plane with plane, cylinder or cone, on a straight or circular edge) `ChFiKPart.ComputeData` writes the fillet's surface exactly. For any other pair, `PerformSetOfKGen` walks along a guide line: it finds a first section (`StartSol`, `PerformFirstSection`), then marches, solving for the ball's contact points at each step (`BRepBlend`), and approximates a B-spline through them.
2. **Trimming to faces.** `SplitKPart` asks a hatcher where the analytic surface's contact lines lie inside each face. The walk does the equivalent with its restriction checks.
3. **Corners.** `PerformOneCorner` and the others (`Builder_C1`, `_C2`, `_CnCrn`, `FilBuilder_C2`, `_C3`, `ChBuilder_C3`: about 12,300 lines), case by case on how many fillets meet.
4. **Rebuilding the solid.** The data structure of new surfaces and curves goes to `BOPReconstruction` (Kerf's, in place of OCCT's TopOpeBRepBuild), which trims the old faces against them and assembles a solid.

**The design's assumption is that a fillet's two contact lines lie inside its two faces,** and that each face survives with less of it. Nothing in the steps above has the idea of a face being used up. Contact leaving a face is handled by a further face being found (step 2 of the walk) or by a corner, and not as an event of its own.

### A trace: radius 10 on one edge of a 10 mm cube

Measured, with temporary lines in `SplitKPart`, `StartSol`, `PerformSetOfKPart` (not kept):

| Radius | Step 1 | Hatcher on face 1 | Then | Result |
|---|---|---:|---|---|
| 9 | Analytic: plane and plane | 1 domain, 2 points | | 7 faces |
| 10 - 1e-8 | Analytic | 1 domain, 2 points | | 7 faces, two of them 1e-8 wide |
| **10** | **Analytic, and right** | **0 domains, 2 points** | `SplitKPart` refuses; on to the walk; `StartSol` finds no start | "no starting solution" |
| 11 | Analytic | 0 domains, 0 points | The same, rightly | The same message, which is why it says nothing |

So at exactly 10 the surface is right. The contact line lies along the face's far edge, so the hatcher finds it on the boundary, not in a domain, and the code takes that as "not on the face". The walk then tries a problem whose solution sits on the same boundary, and that fails too. Upstream's commit message and its hatching code (below) say the same.

**Two fillets that meet** take another way: both contact lines hatch properly (1 domain each), and nothing faults a contour, but the two leave the face in between with no area, and the operation fails with "couldn't be built on 2 edge(s)". Measured for r = 5 on two parallel edges of a 10 mm face. Where in the rebuild is inferred, not traced.

### A trace on curved faces

Measured, the same way:

| Case | Step 1 | Hatcher | Then | Result |
|---|---|---:|---|---|
| Disc, R 10, h 5; rim, r = 4.9 | Analytic: cylinder and plane | 1 domain on each | | 4 faces |
| Same, **r = 5** (the side is used up) | Analytic | **0 domains, 2 points** | `SplitKPart` refuses; walk; `StartSol` fails | "no starting solution" |
| Cylinder R 5, h 20; rim, **r = 5** (the top is used up) | Analytic | Its plane side has no curve to hatch: a degenerate contact | | **Works**: 3 faces, right |
| Ellipse's extrusion, 20 by 10, h 5; rim, r = 4.9 | **Walk**, no analytic case | | | 4 faces |
| Same, **r = 5** | Walk | | `StartSol` fails | "no starting solution" |
| Same, r = 5.1 | Walk | | `StartSol` fails | The same |

**So the limit is the same defect on analytic and general surfaces:** the boundary is treated as outside. The one that works, the top used up on a cylinder, works because the contact has collapsed to a point and the code has a case for that.

## 2. What was fixed, and what it shows

Measured: volume, area and validity against the true answer, in `FilletLimitTests`.

| Case | Before | Now |
|---|---|---|
| One edge, r = a face's width (cube, 10 by 20 by 30, plate) | Fails | **Exact** |
| Two fillets meeting (r 5 on a 10 mm face; a plate's end fully rounded) | Fails | **Exact** |
| Four edges to a cylinder; a hexagon's six; a triangle's three | Fails | **Exact** |
| A concave edge up to its limit | Fails | **Exact** |
| Chamfer of a whole width; two meeting; four to a diamond; two lengths | Fails | **Exact** |
| Above the limit | "no starting solution" | Says what fits, and since unit 12C without trying |
| All edges of a cube, r = 5 (a sphere); a bar's (a capsule) | Fails | **Exact,** since unit 12M (section 9) |
| Disc's rim, r = h; both rims, r = h / 2; hemisphere; sphere from a short cylinder | Fails | **Exact,** since unit 12L (section 9) |
| Hole's rim, r = 10 in a 10 mm plate | Fails | **Exact,** since unit 12L |
| Cylinder's rim, r = R, one end or both (a dome, a capsule) | Works | Works, unchanged |

**An independent check:** `blend/simple/Q8` is such a case. Through `FilletOperations` it now gives an area of 11957.1. Upstream's reference after `f713b1305f` is 1.19571e+10, at a scale of 1000: the same to the digits it records. **Its result is valid, where upstream's own TODO for that case says it's faulty.** The corpus's status stays BAD, as Draw's `blend` calls the engine's `MakeFillet`, and the fallback isn't in it.

It costs nothing unless ChFi3d has failed: 7.8 ms for r = 10 against 1.1 ms for r = 9, in a loop on a loaded machine.

## 3. What a new approach would look like

**The formulation.** A ball of radius r touches both faces. Its centre lies on the intersection of the two faces' offset surfaces at r. The spine of the fillet is that curve, and the fillet is the envelope of the ball along it. The contact points follow from the centre and the normals. This is what the blending literature calls a rolling-ball blend, and what ChFi3d's walk solves too, though it solves for contacts, not centres.

**What would be new is events as first-class steps.** March along the centre curve, and at each step ask what stops the current contact: 

| Event | What happens | Today |
|---|---|---|
| A contact reaches a face's edge, and the next face is tangent | Contact carries on to it | Handled in the walk, as a recovery |
| It reaches a face's edge, and the next face turns away | Ball rolls on that edge, or the blend stops at a "cliff" | Not done: `f713b1305f` only accepts the exact limit |
| A whole face has been used up between two contact lines | The face goes | **Not done.** The defect here |
| Two blends meet on a face | They share an edge, tangent | **Not done** |
| Ball meets a third face before the spine ends | Trim, or a corner | Partly, case by case |
| A corner of three blends | A sphere patch, when radii and angles allow, else a filling patch | `_CnCrn`, case by case |
| Ball can't fit (curvature tighter than r) | Fails, and says where | A walking failure |

Then the result comes from cutting or adding by Boolean, on a tool whose faces are the fillet's: the engine is reliable now (unit 7R's reconstruction is a much smaller piece of the same idea), and it's what decision 1847 does.

**From other kernels' documentation.** Downstream products of commercial kernels describe the same list by name, which gives some confidence it's the right list. NX and T-FLEX CAD give an edge blend's "overflow" three named resolutions: *smooth overflow* (the blend deforms to stay tangent to a rounding it runs over), *cliff edge* (a sharp edge is kept, and the blend is a ball touching one face and supported by the edge), and *notch* (the blend is unchanged and the other element's faces are trimmed to it), tried in that order, and with all off, an overflowing blend fails. None of this is in ChFi3d as such. Those are documents for users. They describe what's done, not how, and I've read no kernel's code.

**From the literature** (titles and places checked; the papers themselves not read for this study):

- Rossignac and Requicha, "Constant-radius blending in solid modelling", *Computers in Mechanical Engineering* 3(1), 1984: the offset formulation.
- Vida, Martin and Várady, "A survey of blending methods that use parametric surfaces", *Computer-Aided Design* 26(5), 1994.
- Braid, "Non-local blending of boundary models", *Computer-Aided Design* 29(2), 1997: from the title and its place in the field, the problem of a blend that removes or passes faces. **Worth reading first.**
- US patent 5,615,317, "Method for blending edges of a geometric object in a computer-aided design system", showed up in the search, and others like it are likely. It's old enough that it has probably lapsed. Someone should look before a general implementation; nothing here depends on it.

## 4. What it would fix, and what it wouldn't

**The list has moved since the brief.** corpus-failures.md counted 37 fillet cases failing or BAD. At this head the groups are **OK 749, BAD 35, FAILED 10, IMPROVEMENT 6, SKIPPED 84 of 884; 45 failing or BAD,** as other units have added cases. Of those, 23 are fillets and 22 are chamfers.

The 23 fillets, sorted by the surfaces beside the first step's edges. **Measured:** the kinds and what the public `Fillet` does on the first step of each, with the case's edges as far as explode's order matches. **Inferred:** the last column.

| Kind | Cases | Public fillet now | Would a rolling ball fix it? |
|---|---|---|---|
| **Plane and plane, at a limit, apart** | Q8, bug25478_1 | **Now works** | Done |
| Plane and plane, at a limit, touching at a vertex | bug32931, bug25478_2 | Fails | Yes, with corners: stage 2 |
| Plane and plane, another cause | U1 (invalid), bug28354, bug27711_3 | U1 invalid; others work on the first step | U1 perhaps; the others aren't fillet failures |
| **Cylinder and plane, a circular edge, at a limit** | B2 (r 15, side 15 high), Q3 (1, 1), W7 (10, 10) | Fails | **Yes: stage 3,** as for straight edges |
| Cylinder and plane, circular, past a limit | U7 (r 3, ledge 2 wide), Y7 (r 12, side 10 high), bug602 (r 20, ledge 4.45) | Fails | Only as a *cliff edge*, a ball running on the edge. Otherwise failing is right, and stage 1 says why |
| Cylinder and plane, circular, another cause | H4, bug32930 | Fails | Unknown |
| Mixed, with corners | bug26655 | Fails | Perhaps: stage 4 |
| Bezier or B-spline | A1, A7, G8 | A1 invalid; A7 fails; G8 is the edge-order gap | Stage 5 at best; G8 no |
| Evolved radius | K2 | Not tried | Only if it does variable radii |
| Tori, a fuse | bug28119_1, bug_gh501, bug23902 | Not tried | Unknown |

So of the 23: **2 fixed now; 5 more at exact limits within stages 2 and 3; 3 past a limit that want a new feature, not a fix; 3 that aren't fillet failures at all; and 10 unknown,** a few of which might fall to stage 4 or 5. No claim is stronger than that without trying them. **Of the limit cases, the four upstream's commit changes are exactly B2, Q3, Q8 and W7:** two sets of evidence for the same class.

**The chamfer corners.** 22 cases: 12 `_sequence`, 9 `_complex`, and dist_angle/A1. A rolling ball doesn't apply: a chamfer has no ball, and these fail on corners on open shells, where ChFi3d's curves don't close a loop on a face (corpus-failures.md). They'd need their own treatment, cutting by a plane or ruled tool, which is **simpler than a fillet's** but is its own unit. **None fixed by a rolling ball.** Those that read FAILED are where the port refuses what native makes invalid (decision 1735), which stands.

**Upstream's fillets to the opposite edge (`f713b1305f`, 27 files).** Read from GitHub, not ported. 12 of its source files are TopOpeBRepBuild and TopOpeBRepDS, which Kerf has replaced, so that half is to be done over in `BOPReconstruction`; 5 are ChFi3d (`_0` +131, `SpKP` +148, `_C1` +31); the others are tests. It's the same limit: "Exclude fully consumed faces... Accept tangential contacts while rejecting transversal intersections". **It changes four corpus cases, B2, Q3, Q8 and W7, and all four keep their TODO, now "Faulty shapes":** upstream's own reference says its result is invalid there. So it would take each from failing to invalid, except that Kerf would refuse them, as it does an invalid result. Q8 is now right here without it. **Its reason for being on the list has mostly gone. I recommend not porting it:** that reverses occt-defects.md's advice, which predates this. **The owner agreed: it isn't ported** (decisions 1892, 1901).

**Unit 11T's islands** (its reliability.md, in progress when read):

| Island | Failing | A rolling ball? |
|---|---|---|
| An equal tee, filleted where they meet | 514 of 2,508 | **Plausibly:** tangency at the saddle's crossing points is an event. Cylinder and cylinder: stage 4 |
| Crossing cylinders, all edges | 401 of 401 | Plausibly, as corners. Stage 4 |
| Fillet on a loft | 39 of 2,194 | Perhaps: general surfaces. Stage 5 |
| Chamfer on a loft | 946 of 2,194 | No ball. Own unit |
| Fillets that fail slowly (a box just above its limit 3.4 s; a loft 13.9 s; a pipe 12.5 s) | | **The box's, done** (section 8) |

The box's case now ends at once. The loft's and pipe's needed a limit on the work in `ApproxAFunction`: decision 1884.

**What it wouldn't fix.** Files that Draw-lite can't read or scripts counting edges. Anything to do with `ChFi2d`. The tolerance of imported shapes (a rolling ball on a sloppy import fails too). Self-intersecting results from radii larger than local curvature: they'd fail honestly, which is better, but they wouldn't work.

## 5. The size, honestly, in stages

Every stage runs after ChFi3d fails, until stage 6 says otherwise. Each leaves things better and is useful on its own. Sizes are judgements, in lines of production code, from this unit's 670-line `FilletLimits` and from ChFi3d's own. **They could be out by half either way.**

| Stage | What | Size | Gets |
|---|---|---:|---|
| 0, **done** | Straight edges between planes, apart, square ends | 670 | The owner's case; Q8; bug25478_1 |
| 1 | Better messages for rims, as for straight edges; limits for circles | 100 | "the most that fits" on a cylinder |
| 2 | Corners of planes: spherical patches where three blends of one radius meet; slanted ends. Cube to sphere | 500 to 1,000 | bug32931, bug25478_2. A bar to a capsule |
| 3 | Circular edges, plane and coaxial cylinder: a revolved tool, the same check by Pappus. Cone likewise | 300 to 600 | **B2, Q3, W7**; disc's rim; full round; hemisphere; sphere |
| 4 | Any analytic pair: plane, cylinder, cone, sphere, torus, with centre curves from analytic intersections, and tangent chains | 3,000 to 6,000 | The tee, crossing cylinders, perhaps bug26655 |
| 5 | General surfaces: marched centre curves with events, and fitted surfaces | 8,000 to 15,000 | Lofts, pipes; the hard end |
| 6 | Replacing ChFi3d: run the new path first. Delete the old only if it never loses | | Maybe 40,000 lines fewer. Not soon |

**Stages 0 to 3 are about 2,000 lines, days of work, and carry almost no risk.** Stage 4 is a real project and may not be needed, depending on what Bento's users hit. Stage 5 is the real rewrite, and there's no evidence yet that it's worth doing: on 28 ordinary fillet and chamfer calls, 27 matched native (corpus-failures.md), and what failed on these lists was mostly at a limit.

**Could it start with the commonest cases and fall back to ChFi3d for the rest? Yes:** this unit is that, and it worked without touching ChFi3d, in a few hours.

## 6. What it risks

**Behind ChFi3d, as stages 0 to 5 are: nothing that works,** by construction, as they run only where it has failed. The remaining risks are: a result that's wrong but passes. So every stage needs a check independent of how it builds. Here it's the volume and area from the geometry, and validity. For stage 4 onwards there's no closed form, so the check would have to be the ball itself: sample points on the fillet, and test that a ball there touches both faces and doesn't enter the solid.

**In front of ChFi3d (stage 6):** the corpus's 995 of 1,044 fillet and chamfer steps that match native today (after decision 1735). A new algorithm has to earn them again.

- **How to compare.** The frozen native recordings give area, validity and face counts for each step, so a step that differs by more than 0.1% in area, or goes invalid, is a finding. Face counts may rightly differ (decision 1847's 5 faces where a hair less gives 7), and surfaces will: exact where ChFi3d approximates, so "same" must not mean the same B-spline.
- **What's lost.** OCCT's reference figures stop being a check in the strict sense, where the new result is better than they are (J6 is one already, decision 1784). Edge order changes, as it did with the reconstruction (G8, bug27711_3). History has to be rebuilt for each stage, and `Generated` for corners isn't simple.
- **Bento.** A workbook's fillets would change their face order and possibly count, on a recalculation, if the new path went in front. Behind ChFi3d, only cells that showed an error change.

## 7. Recommendation

1. **No rewrite now.** The owner's case is fixed, and the evidence says the failures are specific.
2. **After 0.3.0: stages 1 and 3,** with the rims first, as they're the largest group among the corpus's fillet failures and the most ordinary thing a user tries after a box: a disc's edge fully rounded. Then stage 2.
3. **Don't port `f713b1305f`.**
4. **Stop there and look.** Stage 4 and beyond should wait for failures from Bento's users that are ChFi3d's own, as beyond-compatibility.md says. By then the event list here will have been tried on small cases, and I'd read Braid first.
5. **A separate small unit for chamfer corners,** when one of them matters to someone.

## 8. Limits as built (unit 12C)

Decisions 1898 to 1901. Times are from a machine shared with other work.

**A radius moved for a narrow strip:** at most 1e-6 of the model's units (decision 1912).

**What's proved.** No tangent fillet fits where a contact line would lie off a rectangular face beside the edge, when the face past its far edge turns by more than ChFi3d calls tangent. **The limit was wrong below that:** see the next section. It's only an upper limit: other things can stop a fillet sooner.

**Does ChFi3d ever return something above the limit?** Yes, in two ways, measured on ChFi3d alone:

| What it returned above the width | Where | So |
|---|---|---|
| **Valid and right** | The next face within 0.1 radians, either way: a ball touching the side and the next face, to 1e-10 by volume, against the section by hand | **The limit was wrong** (decision 1899). There's no limit there now, and it isn't refused |
| **Invalid** | All of a cube's edges, a top's four, an S of two: 97 of 3,890 runs above a limit, such as volume 1,546 for what should be near 524 | Refused: this was the slow case |
| Nothing | The other 3,793 | Refused |
| **Valid, above a limit still claimed** | **None,** in 65 cases at five sizes | |

**On the corpus's real cases.** With a temporary line in Draw's `blend`, over `blend`, `chamfer` and `bugs/modalg_*`: 116 fillets had edges with a limit. None of one radius lay above it. Four of one radius lay exactly on it, and ChFi3d failed on each: Q8, bug25478_1, bug25478_2 and bug32931, as section 4 says. The two with their largest radius at or above the number are of several radii (E3 and A4), which aren't judged, and were built, valid. The corpus's Draw commands call the engine, not `FilletOperations`, so 0 of its 3,140 statuses move, as they shouldn't.

**Near the limit, by the radius.** For each case and size in the survey, from half the limit to five times it (29 radii), through the public `Fillet`:

| Radius | One edge, or edges apart, square ends | Corners, slanted ends |
|---|---|---|
| Below by more than the band | ChFi3d, right by volume | ChFi3d, right by volume |
| Within the band, either way | The limit, exact | "can't be built yet" |
| Above the band | Refused, says what fits | The same |

The 26 cases away from a near-tangent edge go through these in order at all five sizes, on all 29 radii, with nothing out of place: built, then perhaps "can't be built yet", then refused. The band's width is the shape's tolerance on the contact line, so for a radius it's that over the contact per unit of radius.

**How it behaves by size.** The tolerance is absolute, as all of the kernel's are.

| Size | Band, as a part of it | Limit's rounding (9 places) | What else |
|---:|---:|---:|---|
| 0.01 | 1e-5 | 5e-8 | ChFi3d's own tolerances are absolute too, 1e-4 on the spine: a hundredth of the shape. Its corners are right by volume only to 2e-6, where they're 1e-6 or better at the larger sizes |
| 1 | 1e-7 | 5e-10 | |
| 10 | 1e-8 | 5e-11 | |
| 10,000 | 1e-11 | 5e-14 | Still some 50,000 times the rounding of a coordinate there. `MaxFillet`'s defaults throw (decision 1900) |

**What's still "sometimes it works".** Neither reaches a wrong answer silently, and neither has a limit, so neither is refused. Both are ChFi3d's, and in occt-issues.md:

- **A next face within 0.1 radians, just below the width:** the result is invalid over a band up to about 1e-6 wide at small angles and 1e-8 at 5 degrees, and carries the shape. Either side of that it's right.
- **A contact line reaching a tangent edge,** as a far side with a fillet on it: fails from the width to as much as 1e-5 above it, though it's right on both sides.

**The check, separately from how it's built.** `FilletLimitSweepTests`: 16 cases at six sizes, 0.01 to 10,000: boxes, moved and turned, several edges apart, faces at other angles, a concave edge, a slanted end, a chain round a top, all of a cube. The volume and area come from each section's corners by hand (for the slanted end, integrals over the spandrel; for the chain, by layers, volume only). 6 radii below, 7 in the band with the limit itself, 8 above, with the limit claimed checked against one by hand, and a fine sweep of a cube's edge with 100 radii from 1e-12 to 7e-3 either side at four sizes. A shape with its tolerances raised to 1e-3 is in `FilletLimitsTests`.

**Its cost.** About 4% of a working fillet's bytes, and under 3% of its time on a shape of 262 faces. It would have been 8% and 11% without making the rest of `Neighbours` wait until a limit asks.

### Chamfers past a bend (unit 12V)

Decision 1985. A chamfer's contact on a plane is its edge stepped in by the length on that face, so along a convex circle of radius r it's the circle of r − d: it closes to the centre at d = r, and past that every ruling crosses the axis at one point and the face crosses itself. Past an ellipse's tightest curvature, b²/a, the stepped curve doubles back. That's proved, so it's refused before ChFi3d, as a fillet past a width is; it applies to any chamfer whose contour, as ChFi3d carries it round a tangent chain, has such an edge.

| Case | Limit |
|---|---|
| One length, a convex circle in a plane beside the edge, in a chain or alone | r |
| Two lengths | The length on the plane (`Distance` on the reference face) against r |
| A length and an angle | The length, if the plane is the reference; the length times the angle's tangent if the other face is a cylinder square to the plane; otherwise none claimed |
| A convex ellipse | b²/a if the edge passes the end of the major axis, otherwise the curvature at its nearer end |
| A concave circle (a hole's rim), the other face's circle on a cylinder | None: the stepped curve grows, or runs along the axis |
| A B-spline | None claimed: the check is the backstop |
| A throat | None: it isn't measured along the face |

**Does ChFi3d return something above it?** On ChFi3d alone, above each limit on 22 cases and six lengths from 1 + 1e-6 to 2 times it (132 runs): 17 failed, 82 invalid, and 33 valid to the check but wrong, all of a length and an angle, one of 1.01 times the radius leaving more material than one of 0.999 times it. None was right. **At the limit** ChFi3d is still asked: a rim's cone to a point is built; the reported prism's corners fail, and say the edge closes to a point.

**Its cost.** 132 refusals took 37 ms where ChFi3d alone took 1,850 ms (a loaded machine). On every chamfer, an adaptor per contour edge; the map of faces only for a contour with a circle or an ellipse.

## 9. Rims and corners as built (units 12K, 12L, 12M)

Decisions 1904 to 1911. Stages 1, 3 and 2, in that order, and then, as the plan says, stop. Nothing here is timed: the machine was shared, and the counts are what's reported.

**How each runs.** Stage 1's limits are asked before ChFi3d. Stages 2 and 3 run only after ChFi3d has failed, or made something invalid, so what it builds right can't change. `ChFi3d` isn't touched.

| Sizes | Estimate | As built (added, in production code) |
|---|---:|---:|
| Stage 1, limits for circles | 100 | 177 |
| Stage 3, circular edges | 300 to 600 | 290 |
| Stage 2, corners and slanted ends | 500 to 1,000 | about 600 |

Stage 3 came in just below its range, as a rim's section is a straight edge's and most of it was there already. Stage 1 was nearly twice its estimate, for reading three kinds of surface and where each ends.

**What now works that failed, measured through the public `Fillet`, right by closed forms worked out apart from how it's built:**

| Shape | At what | Faces | Surfaces |
|---|---|---:|---|
| Disc's rim | r = h | 3 | A torus with a tube as wide as it, or wider |
| Disc's two rims, a full round | r = h / 2 | 4 | Two tori |
| Hemisphere; sphere from a cylinder | r = R = h; r = R = h / 2 | 2; 1 | Sphere |
| Frustum's rim; ring's rims, one or both | Side used up, or the ring between | 3 to 5 | Torus |
| Hole's rim in a plate, boss's foot, step, pocket's floor | Wall used up | | Torus; concave ones add |
| B2, Q3, W7's edges | As in the corpus | | Torus |
| All of a cube, a bar, a plate, a triangular bar | At the limit | 1; 12; 18; and so on | Sphere and cylinder |
| Three edges at a corner, to an eighth of a sphere | r = the side | 4 | Sphere |
| Slanted end; a chain round a top | At the limit | | Cylinder |
| bug25478_2: two convex, one concave, touching | r = 2.5 | 9 | Cylinder |
| bug32931: an L's top, split in three, which crashed native | r = 1 | 15 | Cylinder, and a torus at its pivot |

Not the corpus's status: Draw's `blend` calls the engine, so those stay BAD or OK as they were. Filtered to `blend`, `chamfer` and `bugs/modalg_*`, 0 of 3,140 statuses differ from unit 11E's run.

**Refusals on rims are as true as on straight edges.** ChFi3d alone, on 12 shapes at six sizes and seven radii above each limit (504 runs, asserted in `FilletRimSweepTests`), failed or made something invalid: never valid. Nothing is claimed for a plane with anything but a coaxial circle or nothing else on it, since the rest of a contact line stays on the face; so Y7 isn't refused, its rim having a tangent edge past its wall, and no limit, and it fails as it did. U7 and bug602 weren't tried. **A plan with a re-entrant corner has no width to read either:** above an L's limit the failure is ChFi3d's, not a refusal.

**Sweeps, at sizes 0.01 to 10,000.** `FilletRimSweepTests`, 12 cases, and `FilletLimitSweepTests`, 20 cases: 6 radii below, 7 in the band, 7 or 8 above, at six sizes; and a fine sweep of each case at four sizes, 60 radii bunching up at the limit from either side, 2,880 and 4,800 runs. **On all of them the outcome goes from built to refused once and never back, and each built result is right by its section** to 1e-6, or 1e-5 at sizes below 0.5, as a shape's tolerance is a larger part there.

**What the sweep caught.** A ring's two rims fail in ChFi3d from 1e-7 to 1e-6 below the limit, and a tool can't leave so thin a strip, so past failure a strip under ten tolerances is the limit (decision 1907). Two hemispheres at the largest size failed, so what's only a ball is made as one. A check of a part in a million refused a right chain at size 0.01 (decision 1911). `FilletLimits`'s check also refused a right three-edge corner, its own mistake (decision 1908). Each of those failed honestly rather than returning something wrong.

**One of section 8's two bands has gone.** A next face within 0.1 radians, just below the width: ChFi3d's invalid results now go to the exact path, and it's right across the band (decision 1910). The other, a contact line reaching a tangent edge, is still ChFi3d's failure, as the path doesn't read tangent chains.

**What this still doesn't do.** Different radii at a corner. Corners of more than three edges, or of concave edges; a concave edge with a slanted end. Mitres between edges of two different angles. Arcs rather than whole circles, and rims with their circle in pieces. A plane and sphere or torus. A bar's capsule stays 12 faces; only a whole ball is one. Chamfers at corners have their refusals, as they had, and no exact path. **Stage 4 and on, as the plan says, wait for failures from Bento's users.**

## 10. Faces that narrow to a point (unit 15Q)

Decisions 2131 and 2132. 15L's check (decision 2125) failed four of the corpus's results whose edges touch at a point; the owner asked whether they were the cube's case. One is, one is something else, and two have no shape to build here.

| Case | What the geometry says | Now |
|---|---|---|
| X4, 30 on an edge beside a half round notch of 30 | At 30 the fillet's line on the top just reaches the notch's deepest point. Below, the fillet ends on the notch's wall; above, ChFi3d runs it on past the notch to the far end. The volume jumps there, 523,251.177 to 517,188.189, both right. At 30 the fillet is two faces meeting at that point | **Built**, with a vertex there: ChFi3d's own surfaces, right to 2.4e-9 by the section |
| B7, 75 on a circle between a cone and a prism leaning as it does | The faces are tangent at one point of the edge, so the fillet has no width there at any radius | **Built** at every radius from 10 to 150, where 30 to 70 failed; ChFi3d itself split the face there at the others |
| C2, the same, the prism leaning 1e-4 further | Tangent at two points 0.71 apart and convex between: the fillet turns convex, at most 1.2e-4 wide, and ChFi3d's two edges cross | **Fails** from 46 up; below that the strip is within tolerance and it builds. For the owner (2132) |
| B4, a chamfer of 0.2 after two of 0.05 | No limit: ChFi3d's corner is invalid from 0.1 up | **Fails**, as it did, at the validity check; a corner path is 15P's |

**How a touch is told from a crossing.** The two edges must bound one face the fillet made, each with one other face, not the same, be within their tolerances of each other, and, out to where they're ten times that apart either side of where they come closest, the face must lie between them in its own parameters. Crossing edges fail that on one side; edges with the face outside them on both. Only then is a vertex put where they come closest and the face cut in two there. Where the face's surface itself has no width at the point (B7's), its two edges reach it at different parameters, and a degenerate edge, all at the vertex, joins them in each part, as at a cone's apex.

**What it doesn't change.** It runs only after the check has failed a result, so nothing that passed moves; it moves no point of any surface or curve; and its result goes through every check again. The cube's cases and 12M's limits are untouched (`FilletLimitsTests`, `FilletLimitSweepTests`).

**Not for chamfers.** A chamfer's width is its lengths, so it doesn't narrow to nothing where faces are tangent, and the notch chamfered at 30 is invalid from ChFi3d. None has been seen to need it.

## 11. Stage 4a: a ball between cylinders that vanishes where they turn tangent (unit 15N)

Decisions 2133 to 2135; rewrite-study.md's first unit (decision 2128). Timings are from a loaded machine.

**What's built, and where it's tried.** `FilletLimits.BuildBlends`, after stages 1 to 3 in `AfterAlgorithm`: so only where ChFi3d has failed or made something invalid, and nothing it builds changes. It takes one constant radius on edges between a plane or a cylinder and a cylinder, in chains: edges of the same two faces, joined at vertices of only those two edges, from a vertex where the faces turn tangent to another (to within the vertex's tolerance at the radius), or one closed edge whose faces are tangent at its vertex. Seams, which ChFi3d passes over, are passed over. That's the equal tee's junction: each half ellipse runs from (5, 0, 0) to (-5, 0, 0), where the two cylinders are tangent, and its fillet narrows to nothing at both. Anything else, a radius that differs, an edge with an end where the fillet has width, a corner, and the path isn't asked.

**How.** No walk: each section is solved where it lies. At a point of the edge the ball's centre is at the radius from both surfaces (inside the solid on a convex edge, outside on a concave one), found by Newton's method in the plane square to the edge, and its two contacts are the feet of the centre on the surfaces; at a chain's ends the faces are tangent and both contacts are the vertex. The section is the circle's arc between the contacts, a rational quadratic. A cubic goes through the poles, weights and contacts' parameters of sections spaced closer at each edge's ends, and the surface, its two contact curves and their curves on the cylinders are read from it. The number of sections grows until, halfway between them, the ball put back on the surface, the curves on their surfaces and on each other, and the surface's tangency along them (as the gap it opens across the section) are all within the kernel's confusion, 1e-7, or 1e-9 of the radius on a larger one: as the error goes as the fourth power of the step, the next count is what that says. The faces keep their vertices: a chain's edges give way to one face, bounded by its two contact edges and a degenerate edge at each end, as at a cone's apex, and each of the chain's two faces gets its contact edge in their place. No Boolean.

**The ball test, apart from how it's built** (decision 1742: a result that fails it is a failure). On the face as it is in the result: at three points of every interval between sections the ball put back along the face's outward normal is within twice that of both surfaces; along both edges the face lies on its surface and is tangent to it, to the same gap across the section; at the middle the ball's centre is at the radius from the result's faces, by `BRepExtrema`'s distance, so the ball reaches into nothing else; and the volume is the input's with the region between the surfaces and the ball added or taken, by the divergence theorem over its three sides on the exact sections, to decision 1911's part in a million and tolerance by area. Then the operation's checks, validity, tolerances and 15L's crossings, as for any fillet. **Apart from the path,** `FilletBlendTests` slice the tee: in each slice x = 5 cos phi the cylinders are two strips meeting at four corners, and a point is added if no ball of the radius clear of both reaches it; the volume the fillet adds agrees with that to the path's tolerance over the fillet's area.

**The islands, before and after** (reliability.md's sweeps through `Kerf.Modeling`, the same probe both sides):

| Sweep | Before | After | Built by stage 4a | Failing the ball test |
|---|---:|---:|---:|---:|
| The equal tee, r = 0.02 to 2.64 by 0.2% (2,508) | 514 fail | **0 fail** | 514 | 0 |
| Crossing cylinders of different radii, every edge at 0.3 (401) | 401 fail | 401 fail | 0 | 0 |

The tee's other 1,994 radii are ChFi3d's as before, the same volume and faces. Each fillet built here took 0.69 s on average (2.4 s at most) where ChFi3d's failure took 0.11 s: 0.1 s for the sections and faces, the rest the checks, most of it 15L's crossing check on the long contact curves, which meet tangent at the pinches. **The crossing cylinders aren't touched:** every one has corners (the rims, the branch's lines in the top), which are stage 4b's.

**ChFi3d's own tee fillets are off the ball** (occt-issues.md). Where ChFi3d builds the tee, its surface misses the ball by 3e-5 at r = 0.3, 1.2e-3 at 1 and 1.6e-2 at 2.5 (0.65% of the radius), and at 2.5 it adds a sliver face of 7e-8; its volume adds 33.236 at 2.5 where the slices give 32.818. Stage 4a's surface at the same radii is within 7e-9, and its volume the slices' to their own error. That's a wrong answer reported as a success, but the brief keeps ChFi3d's successes as they are: for the owner (decision 2134).

**What it doesn't do.** A plane and a cylinder are never tangent at a point, only along a line, so no plane–cylinder edge ends in a pinch: the support is there, but nothing yet reaches it. Ends where the fillet has width (a contact reaching another face's edge, a corner) are stage 4b. A branch flush with the side (a closed edge, tangent at its one vertex) would be a chain, but ChFi3d throws there, and the path isn't asked after a throw.

## 12. Stage 4b: the crossing cylinders' corners (unit 15T)

Decisions 2139 and 2140. Nothing is built: the unit stopped at its stop sign.

**What the corners are.** The crossing cylinders (reliability.md's island 6: a cylinder of radius 5, and one of radius 0.5 to 4.9 through it along y with its axis in the top plane, every edge at 0.3) have four corners, all of one kind: the top rim's arc, convex, ends where the branch's line on the top and its curve through the side, both concave, meet; no two of the three faces are tangent there. So no edge ends in a pinch, stage 4a has no chain, and there are no 4a blends to meet. **The right corner** is the one ChFi3d makes on planes (a box with a rib across its top edge, built valid, with a torus at each such corner): the concave blend runs on over the convex edge's blend, which ends on it. Here that's a blend of the branch's cylinder and the rim's torus, the ball's centre on the cylinder of radius rad + 0.3 and the torus of tube 0.6, with no closed form, and its contact passing from the top to the torus to the side at points that are no vertices of the input. A stretch of one blend over another, not a patch; and each kind of edge alone fails too, as each ends where its fillet has width.

**ChFi3d in two steps** (each cylinder rimmed, fused, then the concave loop, which then has no corner) builds all 21 radii tried valid, but its blend over the torus is off the ball by up to 6.8e-3 at radius 1 and 1.6e-2 at 0.5 (5% of the radius), not tangent to the torus along its contact by up to 0.1 radians; the tori and the plane-to-cylinder stretch are exact. It would fail the ball test, and the kernel has no cylinder and branch apart to ask it of.

**What it would take** (decision 2140), about two units: 4a's sections along a chain whose support changes (plane, torus, cylinder, tangent where they meet), with ends where the blend has width, which also builds the junction's concave edges alone; then the rim's torus trimmed by that blend, the assembly with new vertices where its contact changes face, and the proof with a torus as a support and the volume against slices of the shape opened and then closed by the ball.

## 13. Why ChFi3d's own fillets are off the ball (unit 15R)

Decisions 2141 and 2142. **Native OCCT 8.0.1 does the same,** to the digits: it's OCCT's. **It isn't the walk or the approximation:** at r = 2.5 on the equal tee the walk's own points miss the ball by 1.64e-2, the approximation adds 3e-5 of its own, and the surface is off by the same at the walk's sections and between them. **It's the formulation, `BlendFunc_ConstRad`:** each section's circle lies in the plane square to the edge where the section crosses it, tangent to that plane's sections of the two faces, the faces' normals projected into it. The rolling ball's circle lies in the plane square to the curve of the ball's centres. Where the faces' normals at the contacts are square to the edge (planes and extrusions along it, circles of coaxial revolutions) the two agree; elsewhere the section is a 2D fillet in a tilted plane, and its centre isn't the ball's. A script solving ConstRad's equations for the tee, apart from ChFi3d, gives its miss (4.8e-5, 1.5e-3, 1.7e-2 at 0.3, 1, 2.5), and nothing with the curve of centres as the guide.

**How far it reaches,** the ball test on ChFi3d's results, worst miss at r = 1:

| Faces | Miss at r = 1 |
|---|---:|
| Equal tee (cylinders of 5) | 1.3e-3 |
| Tee, 5 and 4, 3, 2 | 7.6e-4 to 1.9e-3 |
| Branch of 3 at 60 degrees, at 45 | 2.4e-2, 2.0e-3 |
| Branch of 2 offset by 2 | 9.8e-3 |
| Cone into a cylinder | 1.2e-3 |
| Cylinder through a sphere, off its centre | 3.7e-3 |
| Cylinder through a torus's tube | 1.3e-2 |
| Cylinder tilted 30 degrees through a plane | 1.8e-2 |
| Cylinder through a sphere's centre; cone on a cylinder's axis (walked) | 9e-6; 2.2e-5, at every radius |
| Upright cylinder on a plane (`ChFiKPart`'s torus) | 1e-15 |

The miss grows about as the cube of the radius: on the tee 2.1e-6 at 0.1, 3.9e-5 at 0.3, 2.9e-4 at 0.6. **A fix** gives the functions the curve of the ball's centres as their guide, from a first walk, and walks again; it reaches the ends' sections and the corners, and moves every walked fillet's surface. Not done here: for the owner, beside decision 2134's cheaper choices (2142).

## 14. Stage 4c: the ball that can't move (unit 15X)

Decisions 30202 to 30207. Timings are from a loaded machine.

**Where the ball stands still.** Where a fillet's contour crosses a cylinder of exactly its radius, with the ball on the cylinder's inside (a block's rounded corner with its foot filleted at the corner's radius, or a pocket's), the ball's centre has to be on the cylinder's axis and at the radius from the face the contour runs along: one point. Over that stretch the ball doesn't roll; it touches the common face at one point and the cylinder round a circle, and the fillet there is a piece of the sphere, as a ball-end mill leaves in a pocket's corner. Since 15W ChFi3d's walk finds exactly that, and fails, as the fillet's edge on the common face is a point (decision 2159).

**What's built, and where it's tried.** `FilletLimits.BuildSpheres`, last in `AfterAlgorithm`, so only where ChFi3d has failed or made something invalid. It reads ChFi3d's own contours, so one edge brings its tangent chain as in ChFi3d. A contour is taken if its edges share one face (a plane, a cylinder or a face extruded from a curve) and the faces beside it come in runs, one face each, a pinned run between two stretches; an open contour ends in stretches on plane faces. Anything else, and the stage isn't asked.

| Piece | How |
|---|---|
| Pin | The centre on the axis at the radius from the common face, which must be at the radius from the faces either side; the contacts its foot on the common face, its feet on the faces either side (on their edges with the cylinder) and the circle round the axis between them |
| Sphere's piece | Sections the great arcs from the foot to points of that circle, through stage 4a's cubic; a degenerate edge at the foot; its end arcs exactly the stretches' end sections |
| Stretch | 4a's sections, each solved in the plane square to the edge, from the plane through one pin's centre to the next one's, past or short of the edge's vertex |
| Square end | The section at the vertex lies in the end face, and is the end (C5) |
| Slanted end | Each contact runs on to the plane; the fillet is cut by the plane between those two sections, and the plane's edges are cut where the contacts meet them, or carried on past the vertex where the corner is concave (C1) |
| Faces beside | The contour's edges give way to the contact edges, meeting at each pin's foot on the common face; the edges between a pinned cylinder and its neighbours are cut where the ball touches them. No Boolean |

**The proof, apart from how it's built.** As 4a's: the ball put back on every new face touches both supports, along its rails the face lies on them and is tangent to them, the ball at each stretch's middle and each pin's centre reaches nothing nearer than the radius, the shape is valid, and the volume is the input's plus a third of the flux of X − O through what changed: the new faces from their exact sections, the faces beside by Green's theorem round their boundaries, each support being c(u) + v d, the new edges taken along their exact contacts. A result that fails any is a failure. Then the operation's checks, 15L's crossings among them.

**The four, through the public `Fillet`.** Before, each failed honestly (15W); on main before 15W they were off the ball by 0.9 to 1.7 at radius 10, and 1.0 at 2.

| Case | Faces | Volume | Area | OCCT's area |
|---|---:|---:|---:|---:|
| `blend/complex/C1` (a pin of radius 2, its end at 2; a slanted end beyond a concave corner) | 14 | 75,808.3450 | 19,652.3265 | 19,652.4 |
| `C5` (two contours, a wall extruded from a B-spline, square ends, four pins) | 17 | 4,876,176.7540 | 180,859.1874 | 180,881 |
| `F1` (a block's top, a cylinder of 455, its drafted walls' corners of 10) | 18 | 796,640.4254 | 50,534.2309 | 50,544.3 |
| `F2` (the same block's foot) | 18 | 791,489.1380 | 49,421.8940 | 49,413.3 |

Draw's `blend` calls the engine alone, so the corpus's statuses don't move: the filtered corpus (561 cases) is main's.

**Against closed forms** (`FilletSphereTests`): a block's foot with corners of its radius, at three sizes from 0.3 to 300; a pocket's floor, concave; an end face square at both ends; an end face slanted beyond a concave corner at 30 and 55 degrees; each to 1e-6 of the region taken (in fact 5e-8 or better), and a foot on a slope of 5 and 12 degrees, where ChFi3d fails, against the spandrels and each corner's region by rays from the ball's centre, to 1e-5. Where the cylinder's axis is square to the common face ChFi3d builds the same thing itself, an analytic sphere, and agrees with the closed form to 1e-9.

**What it doesn't do.** A contour where two stretches meet without a pin (a corner larger than the radius beside one of it), a pin at a contour's end, an end on anything but a plane, a slanted end whose plane's edge is curved and must be carried on, a cylinder of the radius with the ball outside it (the ball rolls there), and no ball that fits (`blend/simple/Y6`, rightly a failure).

## Sources

From documentation and the web, read on 6 and 7 November 2026; I've read no kernel's source other than OCCT's.

- [T-FLEX CAD: overflow processing of edge blends](https://tflex.com/help/eng/T-FLEX%20CAD/17/blendedges_overflowprocessing.htm)
- [Siemens NX: UF_MODL blend options](https://docs.plm.automation.siemens.com/data_services/resources/nx/11/nx_api/custom/en_US/nxopen_python_ref/NXOpen.UF.ModlFeatures.CreateBlend.html)
- [Braid 1997 in DBLP](https://dblp.dagstuhl.de/rec/journals/cad/Braid97.html)
- [US 5,615,317](https://uspto.report/patent/grant/5615317)
- Upstream's [`f713b1305f`](https://github.com/Open-Cascade-SAS/OCCT/commit/f713b1305f) (#1449), read through GitHub's API.
