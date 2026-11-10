# Rewriting OCCT's algorithms: a study

*Unit 15M, 11 November 2026, decision 2124, for the owner's decision. The owner (decision 2123): "a partial or full rewrite
of fillets would be expensive but might also fix a number of problems along with a potential gain in performance... A
similar thought process may apply to booleans, chamfers, and possibly others." No code is changed here. It builds on two
earlier studies of fillets alone, [corpus-failures.md](corpus-failures.md) (unit 11G, decision 1724) and
[fillet-design.md](fillet-design.md) (decisions 1847 to 1852, agreed in 1892), and brings them up to date with the other
algorithms beside them.*

Every figure says where it came from. **Counted** means read from the repository's files at `a79c2ec94` (main): the corpus
statuses in `corpus-13.txt` (0.5.0's run), the documents named, and the source. **Measured** means a figure an earlier unit
measured, with its decision. **Inferred** and **estimated** are my judgement. Nothing was run here but scripts that count.

## In short

**A rewrite of any of these isn't justified by the evidence, and wouldn't be quicker than what the project does now. One
kind of partial rewrite is: a new path for the common cases, written from the geometry, that runs where OCCT's fails and is
checked independently of how it builds.** The project already has one, for fillets: `FilletLimits`, 4,916 lines (stages 0
to 3 of fillet-design.md), which made the owner's 10 mm cube, rims, full rounds and corners exact without touching ChFi3d,
and the chamfer past tight corners (unit 12W) is another. That's where to carry on, and fillets are where it pays most.

| Candidate | What's wrong today (counted) | What a rewrite would gain | Cost (lines today; estimate) | Recommendation |
|---|---|---|---|---|
| **Fillets** (ChFi3d and its blends) | 22 corpus cases failing or BAD, all OCCT's own; 14 of OCCT's 40 open issues that apply to Kerf, the most of any area; three of reliability.md's islands (the equal tee 514 of 2,508, crossing cylinders all edges 401 of 401, a loft 39 of 2,194); four checks that exist because results can be wrong (1833, 2120 and 15L, 1735, 1898) | Fixes most of the islands and the open issues, *if* it handles events ChFi3d lacks. Little speed: fillets are 0.6% of Bento's recalculation | ~69,700 lines with its walking and blend functions. Next stage 3,000 to 6,000 lines, **4 to 10 units**; a replacement 30 to 60 units, out by half either way | **A new path behind ChFi3d, carried on: stage 4, cylinders first.** No replacement |
| **Chamfers** (ChFi3d's chamfer builders) | 22 corpus cases (14 BAD, 8 FAILED: corners on open shells, natively invalid too); the chamfer on a loft, 946 of 2,194; a client's chamfer at 1.71 (now refused, 12V) | Corners that build, where today they're refused or invalid | ~8,600 lines inside the above. A path by a Boolean with a ruled tool: 1,000 to 2,500 lines, **2 to 4 units** | **Second: a Boolean-tool path for corners of planes and cylinders,** as 12W did for tight corners. Leave the loft's chamfer until a client meets it |
| **Booleans** (BOPAlgo, BOPDS, BOPTools, IntTools) | 3,301 of 3,305 runnable `boolean/` cases OK, the rest Draw-lite's or deliberate; ~50 BAD in `bugs/`, mostly old reference figures; 10 silent wrong answers from OCCT's issues, each now fixed or failing; at radii of 50 to 10,000, 329 of 18,306 near-tangent calls fail (loudly) | Little. The kernel's Boolean is already **0.56 times native's time** on ordinary parts (12Y); what's slow is every Boolean going over the whole of a large solid, and the checks | ~61,000 lines, and ~69,000 more of surface intersection beneath. A replacement **40 to 80 units** without the intersections | **No rewrite.** Fix in place: a Boolean that leaves the faces a small tool can't reach alone (performance, 2 to 4 units); and the tolerance questions below |
| **Offsets and shells** (BRepOffset, the 2D offset) | 111 BAD: 77 2D offsets of B-spline wires, 34 3D offsets in intersection mode; #1106, #1107, #1706 apply; the loft's offset fails 550 of 555; the cone's apex (fenced, 1918) | Possibly the 3D offset's failures; the 2D survey found Bento's profiles right | ~36,000 lines; a replacement 20 to 40 units | **No rewrite yet.** One unit to find whose the loft's "bad orientation" is, the nearest thing to "doesn't work" in this list |
| **Sweeps and lofts** (BRepFill, GeomFill) | 64 BAD, 29 of them stale areas (occt-defects.md); 7 lofts that don't close; #1630 (fenced by a retry, 1805), #639, #304, #302 apply, all loud | Nothing measurable | ~56,000 lines | **Leave.** Fix by issue |
| **The mesher** (BRepMesh) | 16 BAD that fail rightly on bad wires; the 7 FAILED are Draw-lite's or formats'; sphere, torus and deflection faults already fixed (1929, 1953) | Meshing was the largest share of Bento's recalculation (11A), since cut by 11E; no evidence the design is the cause | ~18,000 lines; a replacement 10 to 20 units | **Leave.** It's self-contained and works |
| **Others:** the point classifier and the checks; `UnifySameDomain`; the tolerance model | The checks are 20% to 40% of a modelling Boolean's time (12Y, 13K); the classifier is OCCT's ray casting (15C); Unify had four wrong results (#1541, #1340, #1310, #1573) | A classifier and a check kept with a frozen solid would make repeated questions cheap | Classifier ~2,900 lines, check ~7,100 | **Fix in place, as phase 15 is doing.** The tolerance model is the root of the scale failures and isn't for a rewrite of any one algorithm |

**The first unit I'd suggest:** fillet stage 4a, a rolling-ball blend between two cylinders, and a cylinder and a plane, on a
saddle edge, with the events of fillet-design.md section 3 (a contact reaching a point where the surfaces turn tangent; a
blend vanishing), behind ChFi3d. Its targets are measured already: the equal tee (514 of 2,508 failing) and crossing
cylinders with all edges filleted (401 of 401). It's working if, early on, a fixed sweep of the tee goes from 514 failures
towards none with **no result wrong by an independent check** (the ball test below), and no corpus status moves (it runs
only where ChFi3d failed). If after the first unit the tee's walk can't be made to pass the tangency, stop there: it tells us
stage 4 is harder than estimated, before the larger sums are spent.

## 1. How the evidence was counted

- **The corpus** (`corpus-13.txt`, 17,872 cases): OK 10,476, SKIPPED 6,825 (data missing), BAD 502, FAILED 61, IMPROVEMENT 8.
  A BAD case is one whose OCCT script expects that failure (a `TODO`): it fails natively too. A FAILED case is Kerf's alone,
  and corpus-failures.md attributes all 59 that were there at 12I (two since): 38 Draw-lite's missing commands, 6 left out
  on purpose, 12 OCCT's own run natively, 2 scripts counting edges in another order, 1 a cold start's timing. **None is a
  wrong result of an algorithm in Kerf that native gets right.**
- **The cases sorted by algorithm:** by the group's folder where it says (`blend/`, `chamfer/`, `boolean/`, `offset/`,
  `thrusection/`, `pipe/`, `mesh/` and so on), and for `bugs/` by the commands in each script, the first family in the order
  fillet, chamfer, offset, sweep, draft, Boolean, mesh (as a fillet's script makes its solid by Booleans). A script using
  several is counted once. So these counts are a sort, not a diagnosis.
- **OCCT's open issues** (occt-defects.md, 224): sorted by the words of their titles. 40 apply to Kerf.
- **occt-issues.md** (1,311 table rows, every `// OCCT` mark in the code): sorted by the first column's class names.
- **The decisions** that fence a defect rather than fix it: read from decisions.md.
- **Sizes:** lines of C# under each folder of `kernel/src/Kerf.Kernel.ModelingAlgorithms` and `ModelingData`, comments and
  all.

| Family | Corpus BAD | Corpus FAILED (what) | OCCT issues that apply | occt-issues.md rows (open: touch or fix) |
|---|---:|---:|---:|---:|
| Fillets | 20 | 2 (scripts counting edges) | 14 | 80 with chamfers (42) |
| Chamfers | 14 | 8 (natively invalid; Kerf refuses, 1735) | 0 of their own (#1177 is both) | (above) |
| Booleans | 50 | 10 (Draw-lite 6, OCCT's own 2, deliberate 1, a cold start 1) | 5 (two of them time only) | 158 with intersections (90) |
| Offsets | 126 | 1 | 3 | 67 with drafts (33) |
| Sweeps and lofts | 64 | 1 (Draw-lite) | 4 | 83 (40) |
| Mesh | 30 | 7 (formats, Draw-lite) | 1 (time only) | 43 (13) |
| Drafts | 12 | 0 | | |
| Exchange, healing, other | 186 | 32 | 13 | |

The "open" rows of occt-issues.md are mostly `touch` (fixed when the code is next changed), almost none a wrong answer a user
meets; they measure how much of OCCT's oddity a family carries, not its failures.

## 2. Fillets

### What's wrong today

**The corpus (counted).** The fillet and chamfer groups and the fillets in `bugs/`: 22 fillet cases failing or BAD, 20 of
them BAD (OCCT's own, each run natively by unit 11G or sorted from its script) and 2 a script counting edges. Of the 1,044
fillet and chamfer steps compared with native (11G, after decision 1735), **995 are the same, 31 fail on both sides, 0 are
invalid in Kerf and valid natively.** ChFi3d as ported is OCCT's ChFi3d, faults and all.

fillet-design.md sorted the 23 fillet failures of 7 November by the surfaces beside them. Since then stages 1 to 3 made the
limit cases exact through the public `Fillet` (Q8, bug25478_1 and _2, bug32931, B2, Q3, W7), though the corpus's statuses
don't move, as Draw's `blend` calls the engine. What's left: 3 past a limit that would want a *cliff edge* (a new feature,
not a fix); 3 that aren't fillet failures; 10 of unknown cause (B-splines, tori, a fuse, an evolved radius, mixed corners).

**OCCT's open issues (counted).** 14 fillet issues apply to Kerf, of the 40 that apply at all: #1673, #1571, #1570, #1569,
#1495, #1430, #1427, #1177, #900, #899, #737, #694, #692, #691 (and #1706, a fillet then an offset). **Eleven of them come
out invalid or self-intersecting, six of those on swept or lofted solids.** Each is a failure through the public fillet, by
the check of validity or decision 1833's tolerance, except #1570, the last silent one, which 15L's edge check is to fail by
default.

**reliability.md's islands (measured, 11T and 12G).**

| Island | Rate | Whose |
|---|---:|---|
| An equal tee, filleted where they meet | 514 of 2,508 radii fail | ChFi3d's walk, going towards the saddle's points where the surfaces turn tangent; native fails 451 of 828 where Kerf fails 460, agreeing on 809 (decision 1925) |
| Crossing cylinders of different radii, every edge | 401 of 401 | Corners; the edges where they meet alone work, 441 of 441 |
| A fillet on a loft, square to circle | 39 of 2,194 | "Bad orientation of subshape" 30, walking 9 |
| A fillet that fails slowly (a pipe, a loft) | 12.5 s, 13.9 s | Bounded since decision 1884 at 20,000 sections |

**Fences, not fixes (counted from the decisions).** Each exists because ChFi3d can return something wrong and call it done:

- 1833: a fillet or chamfer whose tolerance passes its size fails (#1639, #1695, #1701; #1639's cause since fixed, 2119).
- 1735: a fillet or chamfer that would lose a face fails (`BOPReconstruction.HasLostFace`).
- 1898, 1985: radii and lengths past a proved limit are refused before ChFi3d is asked, as above them it fails, or returns
  something invalid, or (for a chamfer by a length and an angle, 33 of 132 runs) something valid and wrong.
- 2120 and 15L: the edge-level self-interference check, on by default by the owner's word, for #1570.
- 1742's validity check after every fillet: it's what turns the eleven invalid issues above into failures.

**The clients (counted).** The owner's 10 mm cube (fixed, 1847); a user's agent's 40 by 30 by 30 box chamfered at 1.71
(refused since 12V). The enclosure's fillets and chamfers don't fail; its cost is in Booleans (performance.md, phase 14).

### What a rewrite might gain

**Correctness:** the islands and the issues on swept solids, if the new design has what ChFi3d hasn't: events as steps of
their own (a contact leaving a face, a face used up, a blend vanishing at a tangency, two blends meeting), described in
fillet-design.md section 3. Inferred, not shown: nobody has built stage 4 to see.

**Time and bytes:** little. Fillets and chamfers were 0.6% of the thread time and 4.1% of the bytes in Bento's recalculation
(11A). Where a fillet's time goes is OCCT's support code more than its algorithm: `IsTangentFaces` and its adaptors (10% of
the features suite's samples, 11Q; 22% of a box's chamfer and fillet bytes, 15F, both fixed), `MaxFillet`'s search over real
fillets (plate surfaces 35%, `SameParameter` 22%, validity 20%), and the slow failures, now bounded. **What is the design's:**
a contour's fillets are built in order, each depending on the last, so they can't run in parallel (14F), and a plate of 1,600
holes with every edge filleted took 19 to 26 s (11T). A design that blends independent edges apart could use threads, but
the reconstruction would still meet them all. Unmeasured.

Kerf's own path costs little: about 4% of a working fillet's bytes for the limits (12C), and 7.8 ms against 1.1 ms on the
cube at its limit, where ChFi3d would have failed.

### What it would cost

**Size (counted):** ChFi3d 33,146 lines (of which `BOPReconstruction`, Kerf's own, 1,345), ChFiKPart 3,902, ChFiDS 4,199,
BRepBlend 12,824, BlendFunc 10,320, Blend 1,258, AppBlend 1,370, BRepFilletAPI 1,571, FilletSurf 1,119: **about 69,700**.
The corners alone (`Builder_C1`, `_C2`, `_CnCrn`, `FilBuilder_C2`, `_C3`, `ChBuilder_C3`) are about 12,300. Kerf's own path,
`Fillets/`: 4,916.

**What it depends on and what depends on it:** below it, the surface intersections and walking (IntPatch, IntWalk,
`Approx`), the hatcher, `GeomPlate` for corners, and the Boolean engine for the rebuild (`BOPReconstruction`, which replaced
OCCT's 69,000-line TopOpeBRepBuild in unit 7R). Above it, only `FilletOperations`, `BRepFilletAPI`, Draw-lite and
`Kerf.Modeling`'s `Fillet` and `Chamfer`. **It's a leaf:** nothing else in the kernel builds on ChFi3d (the offset's one
reference aside), so a new path can replace it piece by piece without disturbing anything else.

**What pins it (counted):** 191 test cases in `BRepFilletAPI`'s tests, 38 in `ChFi3d`'s, 57 in `BRepBlend`'s, 17 in
`BlendFunc`'s; the limit, rim and corner sweeps (`FilletLimitSweepTests`, `FilletRimSweepTests`: thousands of runs at six
sizes against closed forms); the corpus's 884 fillet and chamfer cases; 6 oracle tests replaying native's recordings; and
the users' cases.

**Estimate.** This project's units have built new geometry at 300 to 1,800 lines of production code a unit, with its sweeps
and tests: 12C 670, 12K to 12M about 1,070 in three, 12W about 1,800 in one. Taking 1,000 lines a unit, and fillet-design.md's
sizes (themselves out by half either way):

| Stage | What | Lines | Units |
|---|---|---:|---:|
| 4 | Any analytic pair (plane, cylinder, cone, sphere, torus), centre curves from analytic intersections, tangent chains, the events | 3,000 to 6,000 | 4 to 10 |
| 5 | General surfaces: marched centre curves with events, fitted surfaces | 8,000 to 15,000 | 10 to 25 |
| 6 | The new path first, ChFi3d deleted if it never loses; history; the 995 matching steps earned again | | 10 to 25 more |

**A full replacement: 30 to 60 units, with the uncertainty of a factor of two.** Stage 5 is the one most likely to run over:
it's what OCCT's walking already does, and its failures (the loft's) are where the hard cases live.

### The ways in

1. **A new path behind ChFi3d** (what stages 0 to 3 are). Runs only where ChFi3d has failed or made something invalid, so
   nothing that works changes. Proved by: closed forms where there are any (stages 0 to 3 had them); for stage 4 and on,
   **the ball itself**: at sample points of the new fillet, a ball of the radius there touches both faces and doesn't enter
   the solid, plus the volume against a slicing integration, plus validity. And by sweeps of radii and positions, as
   reliability.md's islands were found.
2. **A new path in front for the common cases**, ChFi3d as the fallback. Faster to reach the good cases, but it changes what
   works today: face order and counts change (Bento's cells would show different face numbers), and the 995 matching steps
   have to be earned again. Proved by differential runs against ChFi3d's path on the same inputs: every step where both
   succeed must agree in volume, area and validity, and a difference is a finding to settle by the ball test.
3. **A replacement.** Way 2 carried to the end. Only when way 2 never loses on the corpus and the sweeps.
4. **Fixing in place.** What phases 11 to 15 did: #1639's seam (2119), the face lost (1735), the walk's bound (1884). Cheap
   for each, and it's how ChFi3d's own bugs get fixed. It can't add events ChFi3d's design doesn't have.

**Native OCCT is still at hand for comparison:** the hardware repository's `.venv` holds build123d over native OCCT 8.0.1,
which may be run read-only (common rules); the frozen recordings cover only what the oracle tests asked.

### Risks

- **What ChFi3d handles that a rewrite would relearn:** variable radii (`EvolRad`), chamfers by two lengths, by a length and
  an angle, and by a throat; corners of three or more blends of different radii; tangent chains round a part; seams and
  periodic faces; the 2D curves on every face, to the tolerance the rest of the kernel expects; and `Generated` history for
  each new face. Each of those was a phase of the port.
- **A wrong result that passes.** The new path's check must be independent of how it builds: the ball test, not the
  builder's own idea of the surface.
- **Reference figures.** Where a new result is right and OCCT's isn't, the corpus's figures stop being a check in the strict
  sense (J6 is one already, decision 1784).
- **The literature** (titles and places checked; the papers not read for this study): Rossignac and Requicha, "Constant-radius
  blending in solid modelling", *Computers in Mechanical Engineering* 3(1), 1984; Choi and Ju, "Constant-radius blending in
  surface modelling", *Computer-Aided Design* 21(4), 1989 (the spine as an intersection of offset surfaces); Vida, Martin and
  Várady, "A survey of blending methods that use parametric surfaces", *Computer-Aided Design* 26(5), 1994; and Braid,
  "Non-local blending of boundary models", *Computer-Aided Design* 29(2), 1997, on blends that pass or remove faces, which is
  what the events are; **read first.** Users' documentation of NX and T-FLEX CAD names the overflow cases (smooth overflow,
  cliff edge, notch): it says what's done, not how. No other kernel's code has been read, and none should be copied;
  fillet-design.md notes a patent (US 5,615,317) to look at before a general implementation.

### Recommendation

**Carry on the path behind ChFi3d: stage 4, one surface pair at a time, starting with cylinders.** Not a replacement: the
evidence (995 of 1,044 steps matching native, no client failure that's ChFi3d's own beyond limits now handled) doesn't pay
for one, and way 2 can come later if stage 4 shows its events working. That's also the plan's line, "General-surface fillets
and replacing ChFi3d, until clients' failures justify them".

## 3. Chamfers

### What's wrong today

**The corpus (counted):** 14 BAD and 8 FAILED. The FAILED are the sequences on open shells where ChFi3d's curves don't close a
loop on a face; native's result is invalid at the same step, and Kerf refuses (decision 1735). The BAD are OCCT's corners
(`dist_angle_complex`, `dist_dist_complex`, `equal_dist_complex`), each failing natively.

**The islands (measured):** a chamfer of every edge of a loft, square to circle: **946 of 2,194 fail, the highest rate of any
island**, at small lengths (0.05 and 0.1 fail, 0.5 works). Not looked into.

**Fences:** 1735 (a lost face), 1985 (lengths past a bend refused, as ChFi3d above them was never right in 132 runs, 33 of
them valid and wrong), 1833.

**The clients:** the guide's box chamfered at 1.71 (refused since 12V; build123d gives a result to 1.69); chamfers past tight
corners, asked by the owner and built by Kerf's own path (`TightCorners.Collapse`, 12W), which neither OCCT nor build123d has.

### What a rewrite might gain

A chamfer has no ball: its surface over a line is a plane, over an arc a cone, ruled between two contact curves. **Unit 12W
showed the way:** a tool made exactly of those faces, applied by one Boolean, checked against a volume and area worked out
from the outline alone. Extending that to corners where three chamfers meet on planes and cylinders would build what ChFi3d's
corner code gets wrong. Speed: the users' `chamfer-housing` is milliseconds; 15F cut its bytes by a quarter in place.

### What it would cost

Chamfer-specific code inside ChFi3d's family (counted): `ChBuilder` and `ChBuilder_C3` 2,800, the chamfer blend functions,
`ChFiKPart`'s chamfer cases, `MakeChamfer`: **about 8,600 lines**, sharing everything else with fillets. Kerf's 12W path:
about 1,800. A corner path on planes and cylinders, as 12W built: **1,000 to 2,500 lines, 2 to 4 units.** Pinned by the
corpus's 279 chamfer cases, `TightCorners`' tests and the users' four chamfer cases.

### Ways in, risks, recommendation

The same as fillets': behind ChFi3d, checked by closed forms. **Risk:** the corpus's failures are on open shells, where a
Boolean doesn't apply as it does to a solid; the path would need the splitter, or would leave them to fail as now. The loft's
chamfer is on B-spline faces, where no closed form checks the result.

**Recommendation: second, after the first fillet unit or beside it:** a corner path for chamfers of planes and cylinders, as
12W's. Before the loft's 946, a half unit to find why small chamfers fail there, as it may be one fault, not a design.

## 4. Booleans

### What's wrong today

**The corpus (counted): the Boolean engine is the best-covered and best-behaved part of the port.** `boolean/`: 3,301 OK of
3,305 run, 2 BAD, 1 FAILED (Draw-lite's frozen variables), 1 IMPROVEMENT. In `bugs/`, about 50 BAD by their scripts, 38 of
them in `modalg_1` and `modalg_2`, which occt-defects.md sorts as old cases comparing with an old engine's figures, mostly
stale, and 10 FAILED: 6 Draw-lite's, 2 OCCT's own run natively (bug26034, bug29382_3), one deliberate (bug27928, 1923)
and one a cold start's timing (bug83).

**OCCT's issues (counted):** 5 apply: #905 (one solid for two), #1124 (a section with approximation off), #1573 (Unify on
rotated spheres), and #724 and #1676, Booleans of two faces past 150 s, which cost time only. Fixed so far, each once a
silent wrong answer: **#1629, #1643, #1659, #245, #1039, #1633, #1543, #1613, #1567** (occt-defects.md), each by its own
cause: a neck 1e-5 wide not split, a section stopping 1e-3 short, a seam in a flat face, cylinders 1° from parallel.

**The islands (measured, 11T and 12G):** equal cylinders crossing within a tolerance were 38% wrong or failing in the worst
position, **now none** (decision 1879: treating nearly equal as equal was the fault). What's left is scale: of 18,306 calls at
radii 0.01 to 10,000 and offsets down to 1e-16, **329 fail, all at radii of 0.5 and over and growing with the radius, and 21
are silently over 0.3% out, all at radii of 0.05 and under** (the worst 0.79%). The cause is absolute tolerances meeting a
touch that spans sqrt(2 R tol): the tolerance model, not the Boolean's design.

**Fences (counted):** decision 1860's volume rules, 1866's points in both inputs, 1881's grid for a fuse, 1924 and 2066's
point check on by default, 1937's retry the other way round, 2103's retry with a fuzzy value, and the validity check of 1742.
**Seven checks or retries, each added for a case found wrong.** Since 13K they cost 10% to 12% less, and fired falsely on none
of the corpus's 4,026 Booleans.

**The clients:** no wrong Boolean reported. Two slowdowns (the boss and the detent, 0.4.0), both the checks', fixed in 13A
and 13K.

### What a rewrite might gain

**Little time.** On 12Y's eight ordinary parts, **the kernel's Boolean and clean take 0.56 times native's time**; `Kerf.Modeling`
was 1.12 times native only because of its checks, and is 0.94 times since. In the enclosure the Booleans are 66% of the samples
and 77% of the bytes, and the reason is **a design point, not the port**: every Boolean goes over every face of both inputs,
and 89% of the faces of those results are the object's own, unchanged (8,096 of 9,066). A small cut from the housing costs
about 100 ms and 57 MB, native's 54 to 78 ms. That's BOPAlgo's general fuse as OCCT designed it; it can be changed inside
BOPAlgo, by passing through the faces whose boxes no tool's box meets, without a rewrite (inferred: the pave filler already
skips pairs by their boxes; the builder, the classification of whole faces and the checks don't).

**Little correctness.** The wrong answers above had nine different causes, each in a different corner (intersections, the
tolerance of a neck, seams, nearly parallel axes). A new Boolean would meet the same geometry with the same absolute
tolerances, and would have to rediscover each; it would also need the same checks, by decision 1742.

### What it would cost

**Size (counted):** BOPAlgo 29,246, IntTools 14,743, BOPTools 6,270, BOPDS 4,152, BRepAlgo 3,640, BRepAlgoAPI 1,644, Kerf's
`Booleans/` 1,340: **about 61,000**. Beneath them the surface intersections, IntPatch 34,436, IntWalk 7,587, IntPolyh 6,333,
IntAna 5,955, GeomInt 4,646, ApproxInt 3,794 and the rest: **about 69,000 more.** Above them: fillets (the rebuild), offsets,
features, the checks, healing's Unify, and every `Kerf.Modeling` operator. **The most depended-on algorithm in the kernel.**

**Pinned by (counted):** 294 test cases in `BRepAlgoAPI`'s tests, 174 in `BOPAlgo`'s, 107 in `IntTools`', 147 in `IntPatch`'s;
the corpus's 4,093 Boolean cases and the `bugs/modalg_*` groups; 3,499 checks compared case by case (15B); the users' cases
and the enclosure.

**Estimate:** a replacement of the Boolean layer, keeping the intersections, **40 to 80 units**; with them, more than 100,
which I wouldn't put a number on. A planar and quadric path (exact predicates for planes, cylinders, cones, spheres; OCCT's
as the fallback) 5,000 to 10,000 lines, **8 to 15 units**, for a class of parts the kernel already does faster than native.

### Ways in

1. **Fixing in place** (recommended): a Boolean that leaves alone the faces a tool can't reach, **2 to 4 units**, proved by the
   enclosure's results bit for bit and the corpus's Boolean groups (no status moving); measured on the enclosure's bytes,
   where hot path 1 is 28% of the build's bytes in the housing alone.
2. **A planar and quadric path in front**, checked against OCCT's path differentially on every Boolean of the corpus and the
   users' cases. It would make near-tangent quadrics exact by construction (Hoffmann, Hopcroft and Karasick, "Robust set
   operations on polyhedral solids", *IEEE Computer Graphics and Applications* 9(6), 1989, is the classic of resolving the
   uncertainty by reasoning, for polyhedra). Gain: the scale island at large radii, perhaps. Not enough evidence to start.
3. **A replacement.** Not recommended.

### Risks

What BOPAlgo handles that a rewrite would relearn: the general fuse of many arguments, splitting and the cells builder,
fuzzy values, gluing modes, open shells and compounds, history (`Generated`, `Modified`, `IsDeleted`) that `Kerf.Modeling`'s
names and Bento's selectors depend on, and fifteen years of `bugs/modalg_*` cases. beyond-compatibility.md's warning stands:
"rewriting them is where reimplementations of OCCT have gone wrong before".

### Recommendation

**No rewrite.** The local Boolean, as a performance unit under decision 1829's bar, is the one design change with evidence
behind it. The scale failures are for the tolerance questions (section 8), not for the Boolean.

## 5. Offsets and shells

### What's wrong today

**The corpus (counted):** 126 BAD. 77 are 2D offsets of B-spline wires (`offset/wire_*`), OCCT's own, traced by 12S to four
causes (a kink inside a B-spline edge 39, a wire left open 16, `FixHoles` 4, built but faulty 18); none is a profile Bento
makes (offset2d-survey.md, 1,914 offsets of Bento's profiles right but for 1% inwards past a bend). 34 are 3D offsets in
intersection mode, not built, invalid or of the wrong volume; the rest in `bugs/`.

**OCCT's issues:** #1106 and #1107 (invalid, the public offset fails), #1706 (a fillet then an offset).

**The islands:** **the loft's offset fails 550 of 555, "Bad orientation of subshape"**: outwards invalid, inwards valid
(reliability.md 4b). Whether that's the offset or the loft isn't known. **Since (unit 15O, decision 29201):** the offset's, as native's: a sphere of no area where two faces are tangent at a corner, now left out; 555 of 555 made.

**Fences:** decision 1918 (a solid's offset must be a solid, of a size its input allows), after cones with an apex gave
volumes of 3.7e101 and called them valid.

### Gain, cost, recommendation

A rewrite might fix the intersection mode and the loft; nothing says how many. Size: BRepOffset 27,548, BRepOffsetAPI 5,545,
BiTgte 2,703, Kerf's `Offsets/` 437, and the 2D offset in BRepFill: **about 36,000**; a replacement 20 to 40 units.
Literature: Maekawa, "An overview of offset curves and surfaces", *Computer-Aided Design* 31(3), 1999 (checked). Bento's
`CAD_PT_OFFSET` and shells use it; the bottle is its one example, which works.

**Recommendation: no rewrite yet. One unit to find whose the loft's failure is,** as 550 of 555 is the nearest thing in this
study to an operation that doesn't work on an ordinary shape. The 2D offset: as 12S recommended, none.

## 6. Sweeps and lofts

**Counted:** 64 BAD, 36 of them `thrusection/specific*`, 29 of which occt-defects.md sorts as stale areas and 7 as lofts that
don't close; pipes 5. Issues: #1630 (fenced by a retry with another frame, 1805), #639, #304, #302, all loud failures.
Fixed: #1681/#1682's distorted guide, a rational section's weights, a sweep of no volume (1927). Size: BRepFill 28,720,
GeomFill 22,090, BRepSweep 2,204, Law 3,521: **about 56,000**. No profile names them as costly.

**Recommendation: leave.** Fix by issue, as now. No evidence a rewrite would gain anything.

## 7. The mesher

**Counted:** 30 BAD by the sort, of which `mesh/` 16 fail rightly on faces with self-intersecting wires; the 7 FAILED are the
glTF reader's tree, Draw-lite and a deliberate sphere (1953). Fixed in place: the sphere and torus within their deflection
(1929, 1953), bounds on memory and loops (12H2, 13J); left, 25 faces 1.002 to 1.035 times over (15I). **Time:** meshing and
its tolerance box were the largest share of Bento's recalculation (27.7%, 11A), and 11E cut the box; nothing traced the rest
to the Delaunay design. Size: BRepMesh 16,310 and its data 1,812.

**Recommendation: leave.** It's self-contained, its failures are honest, and its costs have yielded to fixes in place.

## 8. Others the evidence points to

**The point classifier and the checks.** The checks are what decision 1742 costs: 20% to 40% of a modelling Boolean's time
before 13K, and in 15B's enclosure build the validity check 5.9% and the volume check 7.4% of the bytes. Their classifier is
OCCT's ray casting, a point at a time; 15C kept one with a frozen solid (2,291 MB to 4.7 MB for 224 points). The rest is
OCCT's design: a grid of points each projected and intersected alone. A classifier on kept data (a winding number, or rays
batched through a tree) is 2,000 to 4,000 lines, **3 to 5 units**, with a sharp check (15C's 162,458 points, each state the
same). **Worth doing in place, as phase 15 is.**

**`UnifySameDomain`** (`ShapeUpgrade`, 12,945 lines with the rest of the upgrade): three wrong results fixed one by one
(#1541, #1340, #1310) and one open (#1573); about 5% of a modelling Boolean's time (12Y). A clean made for what Kerf's Booleans produce (faces
on one surface, merged by their shared edges) is smaller than OCCT's general one. **A candidate for later,** if more of its
faults appear; no evidence yet that it pays.

**The tolerance model.** Absolute tolerances that grow are behind the scale island (failures rising with the radius,
silent errors at the smallest), several of the Booleans' fixes (fuzzy retries), and the fillets' bands near limits.
beyond-compatibility.md left it alone, "not worth it short of a new kernel", and I agree, but **it's why rewriting any one
algorithm on top of it wouldn't remove these failures:** a new Boolean would meet them too. A narrower question is open in
reliability.md: whether models in metres or at 0.01 should get relative tolerances at the boundaries of an operation.

## 9. Why a rewrite wouldn't be quicker

The owner asked whether a rewrite would "fix a number of problems" at once. Counted from occt-defects.md's "Fixed so far",
48 rows since decision 1775: **each a unit's slice, not a unit.** They had different causes in different
places. The failures that do share a cause are the ones a new path targets: ChFi3d's missing events (the limits, the tee,
the corners) and chamfer corners. Those are exactly what stages 0 to 3 and 12W reached in five units, without touching
OCCT's code. What isn't of that kind (Booleans' near-degenerate inputs, the tolerance model, Draw-lite's gaps, stale
references) a rewrite wouldn't touch.

And a rewrite loses three things the port has: the 995 fillet steps and 3,301 Boolean cases that match OCCT; OCCT's later
fixes (its issues and commits are still triaged against Kerf's code); and "native does X" as a reference in a dispute. A
path behind OCCT's keeps all three.

## 10. Recommendation and order

| Order | Unit | Scope | How to tell early it's working | Size |
|---:|---|---|---|---:|
| 1 | **Fillet stage 4a: cylinders** | Plane and cylinder, cylinder and cylinder, on saddle edges; the events of a contact reaching a tangency and a blend vanishing; behind ChFi3d; the ball test as the check | The equal tee's sweep: from 514 failures down, with **no result failing the ball test** and no corpus status moving. If the walk past the tangency can't be built in the first unit, stop and report | 1 to 2 units, of stage 4's 4 to 10 |
| 2 | **Chamfer corners** | Three chamfers meeting on planes and cylinders, by a ruled tool and a Boolean, as 12W; and why small chamfers fail on the loft | Corner cases of `chamfer/*_complex` built through the public `Chamfer`, right by closed forms; the loft's cause named | 2 to 4 |
| 3 | **The loft's offset** | Whose "bad orientation" it is: the offset's or the loft's | The cause found, with a probe on the 555 | 1 |
| 4 | **The local Boolean** (performance, not a rewrite) | Faces no tool's box meets passed through untouched | The enclosure's bytes, bit for bit the same results, the corpus's Boolean groups unchanged | 2 to 4 |
| 5 | **Fillet stages 4b onward** | Cones, spheres, tori, tangent chains; crossing cylinders' corners | As 1, on the crossing cylinders (401 of 401) | 3 to 8 |
| | **Then stop and look** | Stage 5, way 2 or a replacement only on clients' failures that are ChFi3d's own | | |

**Leave alone:** the Boolean engine's design and the intersections beneath it, sweeps and lofts, the mesher, the 2D offset,
and the tolerance model. Keep fixing their defects one by one as the triage finds them, and keep the checks.

## Sources

- In the repository: [corpus-failures.md](corpus-failures.md), [fillet-design.md](fillet-design.md),
  [chamfer-corners.md](chamfer-corners.md), [reliability.md](reliability.md), [occt-defects.md](occt-defects.md),
  [occt-issues.md](occt-issues.md), [offset2d-survey.md](offset2d-survey.md), [performance.md](performance.md) (11A, 12Y,
  phase 14, 13K, 15B, 15C, 15F), [beyond-compatibility.md](beyond-compatibility.md), [enclosure-feedback.md](enclosure-feedback.md),
  the users' cases' README, decisions 1724, 1735, 1742, 1743, 1833, 1847, 1860, 1866, 1881, 1884, 1892, 1898, 1918, 1925,
  1937, 1985, 2038, 2103, 2116, 2119, 2120, 2123; and `scratchpad/corpus-13.txt`.
- Papers, titles and places checked on 11 November 2026 (the papers not read): [Hoffmann, Hopcroft and Karasick 1989](https://cs.purdue.edu/cgvlab/www/publications/hoffmann1989robust);
  [Choi and Ju 1989](https://portal.mardi4nfdi.de/wiki/Publication:1120949); [Maekawa 1999](https://portal.mardi4nfdi.de/wiki/Publication:1401750);
  and those fillet-design.md checked (Rossignac and Requicha 1984; Vida, Martin and Várady 1994; Braid 1997).
