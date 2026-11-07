# Phase 12 plan: after 0.3.0

> **Agreed, 7 November 2026** (decision 1893), from the outline the owner asked for. Section 1 is started. The next
> release is 0.4.0, with no fixed date: when sections 1 to 3 are done and the corpus shows no new failures. Sections 4
> and 5 ship with whatever is ready by then.

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
| **12I** The corpus | The 59 FAILED, the 502 BAD, and the 48 issues that couldn't be told. | Each attributed, as in corpus-failures.md, and fixed where the fix is plain. **Attributed; 3 silent faults fixed** (decisions 1918 to 1921; corpus-failures.md). 396 of the BAD stand on their scripts alone |
| **12J** OCCT's defects (11S) | Carry on down occt-defects.md, from the top. | Ongoing. **First pass done** (decisions 1926 to 1936): #1123, #1444, #572, #1039, #1659, #1643 right where they were wrong or failing; solids of no volume, a face against a solid, and a fuse that lost part of an input now fail; #1355, #1632, #1431 and upstream `53706f78a8` and `1349657592` in part. Left at the top: #1629, #245, #1639, the self-interfering fillets, a sphere's mesh (for the owner) and a cone's offset |

## 3. Fillets a user would notice

fillet-design.md's stages, in the order it recommends. They run only after ChFi3d has failed, so nothing that works today
changes. Each result is checked independently of how it was built, by volume, area and validity. **The sizes are
judgements, and could be out by half either way.**

| Unit | What | Size | Gets |
|---|---|---:|---|
| **12K** Rims (stage 1): **done** (177 lines; decision 1904) | Better messages, as for straight edges; limits for circles | 100 | "The most that fits" on a cylinder |
| **12L** Circular edges (stage 3): **done** (290 lines; decisions 1905 to 1907) | Plane and coaxial cylinder or cone: a revolved tool, the same check by Pappus | 300 to 600 | B2, Q3, W7; a disc's rim; full rounds; hemispheres |
| **12M** Corners (stage 2): **done** (about 600 lines; decisions 1908 to 1911) | Spherical patches where three blends of one radius meet; slanted ends | 500 to 1,000 | Cube to sphere; bar to capsule; bug32931, bug25478_2 |

**Then stop and look.** General-surface fillets (stages 4 and 5) and replacing ChFi3d (stage 6) wait for failures from
Bento's users that justify them.

## 4. Speed and memory

Ongoing, to the owner's bar.

| Unit | What |
|---|---|
| **12N** Unexplained rises | Extrude until's +3.6 MB, a thick solid's +2.2%, and `Pump_Nut.brep`'s parallel mesh, 5.86 to 6.81 MB |
| **12O** The mesher (11O) | Meet the deflection at a sensible triangle count, so that `ControlAllSurfaces` can go altogether (decision 1680) |
| **12P** Wasteful cases (11Q) | Down the slowest list: the corpus's slowest cases, the rig's slowest suites and Bento's examples |

## 5. Tests and surveys

Fitted around the rest.

| Unit | What |
|---|---|
| **12Q** Coverage floor (11I) | Hand-written engine packages over the floor, by real tests |
| **12R** Tests of their own (11J) | A direct call and a checked result, area by area |
| **12S** 2D offsets | The survey |

## Not in this phase

- General-surface fillets and replacing ChFi3d, as above.
- Coverage of generated STEP entities.
- Bento's caching (11F), which is the owner's with another agent.

## How the work runs

As in phase 11: at most three helpers at once, each on a branch and worktree of its own, then the merge gate, one at a
time. Results stay identical unless a unit says otherwise and records why. Timed claims come from a quiet machine, and the
rig compares against `docs/benchmarks/2026-11-07-f5a7bd7a2.tsv` (decision 1902). Nothing is pushed, except by the owner.
