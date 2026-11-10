# The corpus's failures, and the fillet survey

*Unit 11G, 6 November 2026. Decisions 1719 to 1724. Corrected by the known defects unit the same day, decisions 1735 to 1741: where a figure below was 11G's and has moved, the text says so.*

The last whole corpus run before this unit had 69 FAILED cases of 17,872. This unit fixed 13, all in the fillets. The 56 left
are below, by cause. Of those, only the three reconstruction gaps were the port getting a result wrong.

**Since then:** J5 is fixed (decision 1736). G8 and bug27711_3 are left: only their edge numbers differ (decision 1738). Four cases that wrongly read IMPROVEMENT now read FAILED, as the port refuses what native makes invalid (decision 1735), so there are 59 FAILED: 2 gaps in the numbering, 12 OCCT's own, and the rest as below.

**How this was checked.** Native OCCT 8.0.1 still runs on this machine: a scratch build of build123d.NET's wrappers survives
in the scratchpad, though the repository's oracle tests now only replay recordings. So every fillet and chamfer step below
was run natively as well as on the port. Native's figures for the other cases come from earlier decisions; for those, this
unit reran the case and checked that its log still shows the figures those decisions recorded. The blend, chamfer, draft and
fillet2d grids and the fillet cases elsewhere were rerun after the fixes. The whole corpus wasn't.

## Every failure

| Cause | Cases | Evidence |
|---|---:|---|
| **Port bug, fixed** | 13 | Below |
| **Reconstruction gap, not fixed** | 3 | Native is right on the same input. J5 since fixed (decision 1736) |
| **OCCT's own** | 8 | Native fails or is invalid at the same point |
| **Draw-lite's limit** | 38 | A command, or a check of something Draw-lite doesn't do |
| **Left out or different on purpose** | 6 | Earlier decisions |
| **Speed check on a cold start** | 1 | Warm, it passes |
| **Data** | 0 | Files the public dataset lacks make a case SKIPPED, not FAILED |

### Port bugs, fixed

| Cases | Cause | Decision |
|---|---|---|
| draft/angle A2, B5, C3, C4, C5, F1 | A fillet's edges weren't coded tangent. Native's draft fails on the port's fillet, and the port's draft works on native's | 1719 |
| blend/tolblend_buildvol/A9 | A closed edge with no radius at its one vertex stayed as a second face in a second shell | 1720 |
| blend/simple/V7, blend/tolblend_simple/C5, chamfer/dist_angle_sequence B8 and B9, chamfer/dist_dist_sequence/B8, chamfer/equal_dist_sequence/B8 | The surface the fillet extended was ignored. Two of these, dist_angle B9 and dist_dist B8, are now BAD: OCCT's TODOs | 1721 |

### Reconstruction gaps, not fixed

| Case | What happens | Evidence |
|---|---|---|
| blend/buildevol/J5 | **Fixed (decision 1736).** Shell not closed: one edge where a varying radius reaches past a neighbouring edge isn't shared. 36 edges against 35; every face's area matches | Native on the same input is valid |
| blend/complex/G8, bugs/modalg_7/bug27711_3 | The script names an edge by its explode number in an earlier result, and the reconstruction's edges come in another order, so it picks another edge | Given native's edge (found by position), native succeeds on the port's result, with the same area and volume as on its own, and so does the port (a test; decision 1738). Not fixed: matching the numbers means following much of what decision 992 chose not to port |

### OCCT's own

| Case | Evidence |
|---|---|
| demo/samples/anc101 | Native fails the same way (decision 1310) |
| lowalgos/extss/bug29712_44 | The script's mistake; native gives 0 too (decision 1309) |
| bugs/modalg_7/bug27928 | Unit 12G's reach for a grazing edge (decision 1923) takes a tiny edge of the section `r4`: 12 vertices and 18 edges for 13 and 19. Everything else the script checks passes. A deliberate change, for the owner |
| bugs/heal/bug29382_3 | Native gives the same 6 vertices (occt-issues.md) |
| bugs/mesh/bug31461 | Unit 12O's sphere (decision 1953): `psphere s 10` at 0.01 has 10,150 triangles and 5,127 nodes for 10,108 and 5,106. Native's scaling of the parameters loses 21 of the grid's nodes beside the seam, one a row; Kerf keeps them. Both are within the deflection; the script's other two counts pass. A deliberate change |
| bugs/modalg_7/bug26034 | Native, with exceptions compiled in, gives the same warning (unit 7W) |
| chamfer/dist_angle_sequence/A5, chamfer/equal_dist_sequence/A5 | Native's result is invalid at step 15 too. The port's is invalid with far fewer faces: see "Faces lost silently" |
| chamfer/dist_dist_sequence/B9, chamfer/equal_dist_sequence/B9 | Native's result is invalid at step 2 too, and the same |
| chamfer/dist_dist_sequence/B7, chamfer/equal_dist_sequence B5 and B7 | Native's result is invalid at step 2 too. The port refuses, where OCCT's TODO expects the invalid shape (decision 1735); the cause is ChFi3d's one-stripe corner (decision 29601). dist_angle_sequence/B6 is built since (IMPROVEMENT) |
| chamfer/equal_dist_sequence/B4 | Native's result is invalid at step 3 (OCCT's TODO; on MacOS it fails). The port refuses since decision 29601, for the same corner |

### Draw-lite's limits

| Cases | What's missing |
|---|---|
| demo/samples bottle, cad, cpu, dimensions, dimensionsglsl, dimensionspbr, gehause, logo2019, materials, pencil, penrose, raytrace, snowflake, xde (14); v3d/bugs/bug29020, v3d/materials phong1 and raytrace1 (3) | The viewer, `text2brep`, `DFBrowse` |
| bugs/modalg_7 bug21264, bug30788, bug32330; lowalgos/progress A1 to A4; de_mesh/shape_write_stl/B1 (8) | The progress indicator's text (decision 954) |
| boolean/bcut_complex/F2, bugs/modalg_5/bug23881, bugs/modalg_6/bug26393, bugs/modalg_7 bug29333_1 and _2 (5) | Draw's variables are frozen, so its Booleans are non-destructive (decision 955). Native gives the port's counts when it is non-destructive too |
| hlr/poly_hlr Plate and bug25813_3 (2) | The view's projector is built from the axis, not a camera. OCCT's own TODOs differ by platform, and the port's figures are a third set |
| bugs/mesh/bug32241 | `checkview -display` meshes natively |
| bugs/fclasses/bug1281 | Draw's plugin loader (decision 1311) |
| bugs/modalg_6/bug21727 | Frozen shapes keep no Free flag to clear |
| de_mesh/gltf_write/as1 | `incmesh` can't mesh a frozen document's shapes in place |
| de_mesh/gltf_write/spheres | `XFindComponent` |
| bugs/modalg_7/bug30949 | `Bnd_OBB::DumpJson` |

`bugs/fclasses/bug26922` isn't among the 69. It needs `writevrml`, and VRML isn't ported ([unported.md](unported.md)), so the
corpus doesn't count it as runnable. It fails only when forced to run.

### Left out or different on purpose

| Cases | Why |
|---|---|
| de_mesh/gltf_read boxinterleaved and cubeemb, de_mesh/gltf_write/empty | The glTF reader builds Kerf's tree, not XCAF's layout (decisions 1236, 1237). Triangles and faces match; the structure the script dumps doesn't. **`empty` also keeps nodes with no mesh**, which OCCT's reference doesn't have: for the owner |
| de/step_1/ZQ2, de/step_5/B8 | GD&T isn't read, so its sub-shape labels aren't there (decision 1279) |
| de_wrapper/xcaf/A4 | XBF, OCAF's storage, isn't ported |

### bug83, the speed check

Each `bsection` must take under 0.1 s, and the case now runs in a new process, so it times the first call: 0.41 s and 0.43 s.
Measured warm in one process on a loaded machine, after a dozen rounds: **the port 6 ms and 60 ms, native 16 ms and 55 ms**,
with native's first round 18 ms and 60 ms. So the 6× gap recorded in occt-issues.md is gone, and what's left is the JIT
(decision 1723). The port's and native's calls may not have every option alike, so read these as "about the same".

## The fillet survey

### The corpus

866 cases use the fillet and chamfer commands: OK 744, BAD 30, IMPROVEMENT 9, SKIPPED 76 (no data), FAILED 7. It was OK 733,
BAD 28, FAILED 20 (draft cases included) before this unit.

After the known defects unit: OK 745, BAD 30, IMPROVEMENT 5, SKIPPED 76, FAILED 10. See decisions 1735 and 1736.

The 37 failing or BAD cases, sorted:

| | Cases | Examples |
|---|---:|---|
| **Reconstruction gaps** | 3 | J5, G8, bug27711_3 |
| **Port bugs** | 0 left (13 fixed) | Above |
| **OCCT's own** | 34 | 30 BAD and the 4 FAILED chamfer sequences above |

Every BAD case was checked natively, not only taken from its TODO: 26 fail natively at the same step, 3 are invalid natively
at the same step, and one, dist_angle_sequence/B9, has a TODO that fires whatever the result, with all its steps equal to
native's.

Every step was also compared, because a case's own check can hide things. **Of 1,074 fillet and chamfer steps, run on the port
and natively on the same input:**

| | Steps |
|---|---:|
| Both valid, same faces, area within 0.1% | 1,023 |
| Both fail | 31 |
| Both invalid | 6 |
| **Port invalid, native valid** | **1** (J5; it was 21 before decision 1721). None since decision 1736 |
| Port valid, native invalid | 9 |
| Both valid, but different; or the port succeeds and native fails | 4 |

The last four (blend/simple C8, H5, H4, blend/buildevol/D6) are this comparison's fault, as far as can be seen: it finds native's
edge by its centre, which can pick the wrong one, and the port's results pass OCCT's own reference figures. "Same" means
validity, face count and area, not the geometry point by point.

(After the known defects unit: 1,044 steps, as a sequence that's refused stops sooner. Both valid and the same 995, both fail 31, both invalid 5, port invalid and native valid **0**, port valid and native invalid 3, the port refuses and native is invalid 6, and this comparison's four unchanged.)

Of the nine, two are the port doing better: blend/simple/H8 and blend/buildevol/D8, with native's area but passing the check
native fails (decision 1113). bugs/modalg_7/bug25979 looks real too (decision 1741): the port is valid and mirror-symmetric with 64 faces and area 7926, native invalid
with 65 and 8006, keeping one of the input's two mirrored slivers. The other six are below.

### Faces lost silently

**Fixed so that it fails honestly (decision 1735).** On an open shape the port now refuses these steps, and the four statuses read FAILED. The A5 steps are on a closed shape, where the result stays invalid, so a check says so. As 11G found it:

In six steps of chamfer sequences on open shells, native's result is invalid and the port's is "valid" but has lost faces:
chamfer/equal_dist_sequence/B5 step 2 gives 3 faces and area 0.93, where native gives 6 and 1.67, and the input has 1.63. A
face's split gives no pieces, and `KeptWholeFaces` then drops the whole face beside it. On a solid this would show as a shell
that isn't closed. On an open shell nothing shows. **So four of the nine IMPROVEMENT statuses aren't improvements:**
dist_angle_sequence/B6, dist_dist_sequence/B7, equal_dist_sequence B5 and B7, where OCCT's TODO expects a bad shape and the
port's has merely lost the evidence. The other two steps are the B9 cases above. Steps 15 of
the A5 sequences are the same, with 4 faces against 38. It only appears after an earlier step on the same shape, and only on
open shells, in the corpus. Why: ChFi3d's curves don't close a loop on the face, natively as well, so native's face has an open wire. The chamfer itself isn't fixed, on either side.

### Ordinary shapes, through the modelling layer

28 `Fillet` and `Chamfer` calls on shapes a user might make, in `Kerf.Modeling`, each checked against native on the same input:
a box (all edges at radii 1, 5 and 9.99, the top edges, one edge, chamfers), a boss on a plate (the junction, the top, every
edge, a chamfered junction), a cylinder (both ends, chamfers, a slanted cut), two pipes meeting (unequal and equal), a hole's
rim, a shelled box (rim, outer edges, every edge, chamfered rim), a cone, a fillet on a filleted box, a spherical pocket, a torus
on a plate, a rectangular pocket.

- **27 were valid, with native's area and face count.** Radius 11 on a 20 mm box is refused with a useful message. Native
  returns an invalid shape there, and so does the port's engine (decision 1740).
- Chamfers by two lengths and by an angle ran and were valid, but weren't compared with native.

**Variable radii:** `Kerf.Modeling` has no variable radius. Through Draw on the same shapes, each edge tapered from 0.3 to 0.8
and then every edge at once, 145 steps: **133 identical to native and 12 failing on both sides.** None differed.

These are shapes chosen by this unit, not a sample of what users do, and they're small.

### What a rolling-ball rewrite would and wouldn't fix

This section is judgement, not measurement.

**Might fix:** what ChFi3d itself can't do: the steps failing on both sides (31, two of them the edge-order cases) and the 6
invalid on both. They weren't sorted by geometric cause, so how many a rolling ball would solve isn't known.

**Wouldn't fix:** any of today's other failures. The edge-order cases come from scripts counting edges, and a new algorithm
would only change the order again. Draw-lite's limits and the file formats have nothing to do with it.

**Would risk:** 1,023 corpus steps that match native today, which a new algorithm would have to earn again, with results that
can't match OCCT's reference figures exactly, and a loss of OCCT's fillet surfaces as the comparison.

## Recommendation

**Don't rewrite now** (decision 1724; the owner decides). `beyond-compatibility.md` asked for a list of real failures first. Of
the 20 fillet failures attributed to the reconstruction and its order, 13 turned out to be three small bugs. What's left is one
step of 1,074 where the port is invalid and native isn't, and ordinary shapes match native throughout. The rest are OCCT's own,
and no failure from a user is on the list.

Do these instead, cheapest first:

1. Report faces lost: compare a result's area, or check that a split gave pieces, so that an open shell can't lose faces in
   silence.
2. Fix J5.
3. Give the modelling layer variable radii, which the kernel has.
4. Revisit a rewrite if Bento's users bring real failures that are ChFi3d's own.

The list of real failures this question was waiting for is the 37 cases above, plus whatever users report.

## Since this survey

At the head of 7 November the fillet and chamfer groups read OK 749, BAD 35, FAILED 10, IMPROVEMENT 6, SKIPPED 84. [fillet-design.md](fillet-design.md) sorts the 45 by surfaces and by what a new path would fix.

## Every case attributed (unit 12I)

*Decisions 1918 to 1921.* The same 569 cases as corpus-11e.txt, run again with their logs at the head of this branch. All three statuses came out the same. **The counts don't move: 59, 502 and 8, before and after,** as Draw-lite calls the engine and these fixes are in the public layer. Filtered runs at this branch, over the folders it reaches, against corpus-11e.txt: 5,677, 1,985 and 4,544 cases, 0 differences of status. The whole corpus wasn't run.

| Kind | FAILED | BAD | IMPROVEMENT | How it's known |
|---|---:|---:|---:|---|
| **Draw-lite's gap:** a command or check it doesn't have | 38 | 0 | | The list above |
| **Left out on purpose** | 6 | 0 | | Earlier decisions |
| **The harness:** a speed check on a cold start; a data file missing; a TODO on a script's own text | 1 | 0 | 2 (gdml_private/B5: its file is missing; bug236: `directory`'s name) | Logs |
| **Port:** scripts counting edges, where ours come in another order. The geometry is right given the same edge | 2 (G8, bug27711_3) | 0 | 2 can't be told (bug25206, bug27711_2: the same counting) | Decision 1738 |
| **OCCT's own, run natively:** the 12, and the fillet survey's 30 | 12 | 30 | | Native failed at the same step, or was recorded so |
| **OCCT's own, its log compared:** de/step_* | | 76 | | occt-defects.md: already equal to native's |
| **OCCT's own, by its script alone** | | 396 | | The script's TODO says native fails with that message. **Not run on native** |
| **The port better than native** | | | 4 (D8, H8, bug25979, bug25199: a valid result, within the tolerance the case asks) | Decisions 1113, 1741; the log |
| **Total** | **59** | **502** | **8** | |

**Are there port bugs in the BAD cases?** None found, though only 106 of the 502 are checked against native, so this is the weakest line above. What was looked for, in the logs: an exception or a command native wouldn't give (56 logs: each is what the TODO names), a TODO that's optional or for one platform (6: bug24909_1 and _2 throw `OutOfRange` where OCCT's ticket is a crash, and the rest give Windows' own figures), and figures far from the script's reference (103, below). Native can't be run on them any more: the Draw-lite corpus can only compare with what the scripts say.

**What's silent and wrong, which is what matters here.** A BAD case tells you OCCT knows of a fault in a command, and not what Kerf's public operation does with it, so the ones that give figures were followed through the public API:

| Found | Before | Now |
|---|---|---|
| **A cone offset, with its apex.** `Solid.Cone(5, 0, 12).Offset(-1)`, the default join: a success, "valid", volume **3.7e101**. Inverted, with mitres, 5.7e19 and a box of 3.8 million. 16 of 112 plain offsets of primitives (offset/shape_type_i showed it, with volumes to -2e20) | Success | A failure holding the shape (decision 1918). A solid's offset must hold a solid, of the size its input allows. **The geometry isn't fixed:** with mitres the same cone gives the right 107.757, so that's a place to start |
| **A face with bounds off its surface** (mkface/after_extsurf_and_offset C6 to C9) | Success, invalid, a vertex tolerance of 19.8 | A failure (decision 1919) |
| **STEP labels that aren't ASCII** (#1686) | Raw UTF-8 in a format that doesn't allow it | The format's directives (decision 1920) |
| **boolean/bsection/B6:** two faces whose section should be 76 long come out with none | Success, empty | **Not fixed:** coplanar faces, and no rule known that an empty section breaks |

The 103 figures, against the scripts' references, are about half checks with no reference at all (the script says 0, to print the figure). Thrusection's are sorted as stale in occt-defects.md, which wasn't rechecked. **Left to read one at a time:** bugs/modalg_2's bug5805 group (area half or a sixth of its reference), bugs/moddata_2/bug3721 (the fuse's area twice its reference; a Fuse's volume is checked, not its area), draft/angle F9 and G8. All say so in OCCT's TODOs.

**The 48 issues that couldn't be told:** three were settled, leaving 45.

| Why | Issues |
|---|---:|
| Needs a model attached to the issue: Kerf has the same code, by reading | 27 |
| Read only: same code, no input built | 9 |
| Its data could not be had or read | 8 |
| Tried on its file, and nothing to judge it by | 1 |

**Not done, by kind:** 2 port cases of edge order (not plain: decisions 1738 and 1946); 38 Draw-lite (no user sees these); 396 BAD not checked natively, which can't now be; the bsection, bug5805 and bug3721 rows above; the 45.

## Left

- J5, and the faces lost silently, as above.
- `de_mesh/gltf_write/empty`'s meshless nodes, for the owner.
- The whole corpus hasn't run at this head; the fillet cases have.
- Native wasn't rerun for the non-fillet attributions, only their logs checked against the figures recorded.
- The surviving native build is a scratch artefact, not something the repository can rebuild. If it goes, this comparison goes.
