# 2D offsets: what Bento needs, and the 77 corpus failures

*8 November 2026, unit 12S. The survey the owner asked for before any work on the 2D offset's 77 failures (decision 1790).
Decisions 23201 to 23205.*

**In short.** The 77 aren't the medial axis giving up, and none is a port bug: all 77 are cases OCCT marks as its own
known failures (`TODO OCC23068`, `OCC23748`, `OCC24255` and others in the scripts), on wires made almost entirely of
B-splines, and Kerf fails them as OCCT does. Traced, 39 are a B-spline with a kink inside one edge, which the offset curve
refuses; 16 an offset wire left open; 4 `FixHoles`; 18 built but faulty. Bento's profiles (rectangles, slots, polygons,
fillets, circles, holed plates, open polylines and arcs) come out right in a sweep of 1,914 offsets checked from the
geometry, except two things: **an offset inwards past a curve's sharpest bend is up to 1% of the distance off, silently**
(an ellipse, a spline), and **an offset that closed up threw an exception out of an operation that returns a result**
(213 of the 1,914), or returned an empty sketch (21). The second is fixed here; the first is OCCT's own approximation and
is for the owner. **I recommend no new offset algorithm:** the evidence doesn't call for one.

Everything below says whether it was measured or inferred. Timings are from a loaded machine.

## 1. What Bento calls

Read from Bento's `main` and its `kerf/0.3.0` branch (`Extensions/Bento.Cad/ModelFunctions.cs`), not built or run:

| Bento | Kerf.Modeling | Kernel |
|---|---|---|
| `CAD_SK_OFFSET(sketch or curve, amount, {kind})`, kind `"arc"` (default), `"intersection"` or `"tangent"`; positive outwards | `Operations.Offset(IEnumerable<IShape>, amount, OffsetOptions { Kind })` | `Wire.Offset2D` (`MakeWireOffset`, OCCT's `BRepOffsetAPI_MakeOffset`) |
| `CAD_SK_SLOT_CC`, `_CP`, `_OVERALL`, `_ARC` (slots, and a slot along a curve) | `SlotCenterToCenter` and the others: a line or curve's `Offset2D(height / 2)` | the same |
| `CAD_PT_EXTRUDE` with a taper | `Solid.Extrude`'s taper: the profile's `Offset2D(…, Kind.Intersection)` | the same |
| `CAD_PT_OFFSET` (3D, not this survey) | `Solid.Offset3D` | `BRepOffsetAPI_MakeOffsetShape` |

How the modelling layer offsets a face: its outer wire and each hole separately, as build123d does, then a face from the
outer one with the holes' faces cut away. A wire or edge is offset as one curve, both sides, round ends. Bento has no text
(Kerf.Modeling has no `Text`), so font outlines don't reach the offset yet.

What a spreadsheet user would feed it, from the functions and the introductory examples (one uses `CAD_SK_OFFSET` on a
pentagon, either join): rectangles, slots, regular polygons, filleted corners, circles and ellipses, plates with holes,
open polylines and arcs, splines from `CAD_CRV_SPLINE`, both directions, distances up to the shape's own size and past it.

## 2. How Kerf does on those

**Measured:** 29 profiles, three joins, 11 distances each way from 1e-4 of the profile's size to three times it: 1,914
calls of `Operations.Offset` as `CAD_SK_OFFSET` makes them (`scratchpad/12s/sweep`, kept out of the repository). Each
result is checked against the geometry:

- **Closed forms** where there are any: a convex shape outwards by Steiner's formula, A + P d + &pi; d&sup2;; inwards, while d
  is under the smallest radius of curvature, A &minus; P d + &pi; d&sup2;; a rectangle, a tangential polygon (homothetic), a
  slot, a rounded rectangle, a washer and a plate with a hole by hand; any polygon, while no edge vanishes, by its corners
  (A &plusmn; P d plus d&sup2; times tan or half the angle at each corner, by join and by convex or reflex); an open polyline as
  2 d L + &pi; d&sup2; plus its corners.
- **Otherwise a grid**: the area of {p : distance to the input &le; d} (or, inwards, &ge; d inside it) on 500 by 500 cells
  with partial cells, from the input's edges sampled, nothing of the offset's code. It was 0.01% to 0.03% from the closed
  forms where both apply.
- **And for round joins, the distance itself:** every result point must be d from the input, within 0.1% of d.
- Valid, and a failure where the answer is nothing.

| | Before this unit | After |
|---|---:|---:|
| Right (area, distance, valid) | 1,303 | 1,306 |
| Right: nothing there, and it says so | 19 | 225 |
| **Threw `InvalidOperationException` ("Unexpected result type")** out of `Offset`, which returns a result | **213** | 0 |
| **An empty sketch as a success** | **21** | 3 |
| Failed loudly where there was an answer | 16 | 38 (22 of them the tangent join, below) |
| **Silently wrong** | **10** | **10** |
| Unchecked (mitres on shapes without a closed form) | 332 | 332 |

**The silent ones (measured).** Each is an offset inwards past the input's sharpest bend:

| Profile | Distance | Kerf | Right | Off by |
|---|---:|---:|---:|---|
| Ellipse 10 &times; 5 (sharpest radius 2.5) | &minus;2.5, each join | 55.2597 | 55.6090 (Steiner) | points up to 0.025 too far in |
| | &minus;3 | 39.556 | 40.035 (grid) | 0.029, 1% of d |
| | &minus;4.5, each join | 4.835 | 5.119 | |
| | &minus;4.95, each join | 0.079 | 0.165 | |
| Closed spline through five points | &minus;4.5, round | 58.484 | 58.703 | a point 0.0435 off, 1% of d |

Below the sharpest radius all are exact. The cause (traced): `BRepFill_OffsetWire::Perform`'s `CheckBadEdges` replaces any
edge curving tighter than the distance by a polyline of deflection `0.01 * |offset|`, as OCCT does, before the medial
axis. Its chords lie inside the curve, so the offset lies up to about 1% of d too far in, and nothing says so. With the
deflection at 0.001 the error is 0.1% of d and the ellipse's polyline 3 times the edges (36 to 96 at d = 3); at 0.0001,
0.006% and 10 times. (Inferred: OCCT does the same; the code is the same line.)

**The loud ones** are inwards at exactly the collapse of a part (an L's or T's arm at half its width, a dumbbell's waist at
half its width: an invalid face, said as such) and the mitred dumbbell past its waist ("failed"; round joins give the two
pieces). Not fixed.

**The tangent join** isn't one OCCT documents for 2D offsets (Arc and Intersection), and it behaves as neither: as mitres
on 613 of 638 pairs, but nothing at all outwards from a triangle or a star, at every distance (22 cases, which threw
before and fail now), and it differs from mitres the other way on three. Bento offers `"tangent"`. A question for the owner, below.

**The 3 empty successes left** are a closed polyline (a curve, not a face) offset inwards by exactly half its width: the
kernel gives a wire of no area. Left: it's the limit itself.

### Fixed here (decisions 23201 and 23202)

In `Kerf.Modeling` (`Topology/OneD.cs`, `Operations/OperationsGeneric.cs`):

- **A closed curve that closes up or splits is a failure, `modeling.offset-failed`,** "leaves nothing: the curve closes
  up at that distance" or "splits the curve into N pieces", the pieces carried as the failure's shapes. It was an
  `InvalidOperationException` that escaped `Operations.Offset`. Nothing from an outward offset says the kernel failed.
- **A face whose outer wire splits at a waist gives a face for each piece** (a dumbbell offset inwards past its waist is
  its two discs, 77.22 against a grid's 77.21); **a hole that splits is cut away in pieces.**
- **A face that closes up is left out, with a warning, and a shape that closes up altogether fails** ("the shape closes
  up"), where it gave an empty sketch as a success.
- **A hole whose offset fails for any reason other than closing up fails the offset.** Before, every failure of a hole's
  offset was taken for the hole closing up, and the hole silently left out.

Results that were right are unchanged: the same calls, faces and order (the sweep's 1,303 right ones are all still right,
to the digit). Tests: `OffsetCloseUpTests` (8).

## 3. The 77 corpus failures, by cause

**Measured:** the offset/wire_* grids, filtered (777 cases, 552 run: OK 475, BAD 77, SKIPPED 69 for missing data, at
the branch's base and after). Every one of the 77 is marked `TODO` in OCCT's own script: they are OCCT's failures, which
OCCT's harness counts as passing. Each was then run alone as the harness runs it (`mkoffset` by the wire's length times the
grid's factor), with temporary lines (not kept) at every place the 2D offset can give up:

| Cause | Cases | Input files | Where |
|---|---:|---|---|
| **"Offset on C0 curve"**: a B-spline edge with a kink inside it, which `Geom2d_OffsetCurve` refuses | 39 | 9 (011, 030 to 033, 043, 046, 076, 077) | `BRepFill_OffsetWire::MakeOffset` |
| **"Offset wire is not closed"** | 16 | 5 (018, 021, 040, 041, 081) | the check after `PerformWithBiLo` |
| **`FixHoles`: "Wrong wire"** | 4 | 3 (018, 040, 041) | `BRepFill_OffsetWire::FixHoles` |
| Built, but faulty: self-intersecting (11), a tolerance above the input's (5), a vertex count (1); one not built in the corpus is built alone | 18 | 11 | |

**None came from the medial axis giving up** (`MAT2d_Mat2d::CreateMat`, #1121), bisector fusion (#1122) or
`BuildDomains` (#1296): those trace lines never fired. The defects list's "the medial axis gives up, many with circles" was
a guess from the message; it's corrected there.

**Which a Bento user would hit (inferred).** 21 of the 22 input files are mostly or wholly B-splines, many of 15 to 90
edges: digitised outlines, not sketches. Bento's own profiles are lines, arcs and circles, which the sweep shows right, and
its splines are interpolated (C2) or Béziers, so they have no kink inside an edge. A kinked B-spline reaches Bento only by
import (STEP, BREP). The one file of lines and arcs (offset_wire_001, an open wire) fails only on a vertex count at 0.005
and isn't reproduced alone at 0.075. **So the 77 matter little to Bento today;** the inward error of section 2 matters more,
since an ellipse or a spline offset inwards is an ordinary thing to do.

**Two fallbacks tried, measured, not kept:**

- **Splitting the kinked B-splines first** (`ShapeUpgrade_ShapeDivideContinuity`, C1): 3 edges become 187, 18 become 1,517;
  of the 39, 4 are then right, 23 still fail, and the open wires' results pass through the input (points 100% of d off),
  valid and wrong. Not a cheap fix.
- **OCCT's own `-approx`** (B-splines replaced by arcs and lines first, `MakeWireOffset.SetApprox`, not reachable from the
  public API): on the first three distinct files none was rescued, at about four minutes each, and the fourth ran for half an hour before I stopped it.

## 4. What a fix would take

| Option | What | Size | Risk | For Bento |
|---|---|---|---|---|
| **A. Guards and small fixes** | Done: closing up, splitting, holes (decision 23201). Next: the inward error, a tighter `CheckBadEdges` deflection (0.001 of d, or the model's tolerance) | S | Low; results change only where an edge curves tighter than the distance, with up to 3 times the edges there | The silent error |
| **A2.** Tangent join | Map it to mitres in 2D, or refuse it | S | Low | Bento's `"tangent"` |
| **B. A direct offset for lines and arcs** | Offset each segment, join with arcs or mitres, trim self-intersections; checked against the medial axis where both work | M (1 to 2 weeks) | Medium | Little: the sweep finds those profiles right already |
| **C. A new offset** | e.g. B-splines approximated by biarcs within tolerance, then B; the medial axis kept as a check | L (4 to 6 weeks) | High | The kinked imports and 59 of the 77 |

**My recommendation:** the rest of A (the tighter deflection) and A2 next; B and C only if users bring failures on their
own profiles. The 77 are OCCT's known failures on digitised outlines, and the sweep shows the profiles Bento makes come out
right, except the inward error, which a tighter deflection answers without a rewrite.

## 5. For the owner

1. **The inward offset past a curve's sharpest bend is up to 1% of the distance off**, silently, as in OCCT. Tighten
   the polyline to 0.001 of the distance (0.1% off, 3 times its edges where it applies), or to the model's tolerance (exact
   to within it, more edges)? I recommend 0.001 of the distance, with a time check on the sweep.
2. **The tangent join in 2D:** map it to mitres, or refuse it? I recommend mapping it, with Bento's description saying
   it's the same as `"intersection"`.
3. **The 77:** leave them as OCCT's known problems, I recommend, unless a user brings an outline that fails.
