# Beyond compatibility: what to improve, optimise or rewrite

*Written 1 November 2026, during phase 7, at the owner's request, for reference later. Nothing here is planned yet.*

> **Superseded by [phase10-plan.md](phase10-plan.md) (2 November 2026).** The owner chose .NET conventions and full
> immutability over compatibility: decision point A (retire native build123d.NET) is yes; decision point B (diverge from
> OCCT) is yes everywhere, under that plan's testing rules; OCCT's bugs may be fixed and copy at entry goes. This document
> stays as the reasoning behind the plan.

## The question

What should change in OCCT.NET and build123d.NET if we stopped requiring them to behave exactly like native OCCT and
Python build123d, while still supporting everything Bento's `CAD_` functions need? (Those functions are listed in Bento's
`docs/cad/build123d-in-bento.md`.)

## Why compatibility costs what it does

Phases 0 to 7 port OCCT class for class and check the port against native OCCT, byte for byte where native repeats
itself. That has been the right choice. The oracle catches mistakes no amount of reading would, and OCCT's own fixes
carry across. But much of the code exists only to keep that equivalence while the port's shapes are immutable:

- **Copy at entry.** OCCT's algorithms write into their inputs: tolerances, pcurves, flags, and surfaces changed in
  place. The port's shapes are frozen, so almost every phase 7 unit copies its input first, runs OCCT's code on the
  copy, then maps the history back onto the caller's shapes. Examples: `BRepTools_CopyAtEntry`, `ExactCopyForWrite` and
  `KeepUnchanged` (decisions 416, 973, 1005); LocOpe's working shapes (1013); BRepFeat's re-keyed histories; MakeOffset's
  copy before analysis.
  - Measured cost in time: at most 7% (7X's benchmark).
  - Cost in complexity: large, and it is where subtle bugs have come from:
    - 7I's frozen-sub-shape Boolean change;
    - OCC5805's offset surface, which OCCT widens in place and the port widened on a copy;
    - the prism that doesn't share its profile face with the caller.
- **Masks and attributions.** The oracle masks the Free and Checked flags and accepts canonical rather than byte
  equality where native iterates maps in address order. It records native's own variation from run to run, case by
  case.
- **OCCT's bugs reproduced.** Hundreds of rows in `occt-issues.md`, each marked `// OCCT` in the code.
- **Process-wide state.** Message tables, ShapeProcess's registry, `Resource_Manager`, and scratch values kept as
  `[ThreadStatic]`. These are faithful to OCCT and awkward for a spreadsheet that recalculates cells in parallel. The
  message table's "unknown keyword" caching made tests depend on their order twice.

## The proposals, in the order I'd do them

### 1. build123d.NET directly on OCCT.NET

**Today:**
- build123d.NET reaches OCCT through `Build123d.Occt`, wrappers generated from `bindings/occt.txt` over a native shim.
- The spike added a second implementation, `Build123d.OcctNet`: 4,384 generated forwarding members. 894 of OCCT.NET's
  members couldn't be forwarded.
- It makes OCCT.NET's API look like the native wrappers: struct shapes become handle classes, `Standard_Failure` becomes
  `OcctException`, and some internal members are reached through `UnsafeAccessor`.
- Hand-written parts cover builders, meshing, `Extras` and a private copy of `Quantity_Color`.

**Change:** build123d.NET uses OCCT.NET's types directly, with no wrapper layer:
- shapes as OCCT.NET's frozen structs;
- OCCT.NET's exceptions;
- OCCT.NET's `Quantity_Color` (unit 7X);
- OCCT.NET's public builders (7Y added `TopoDS_Builder.MakeWire`, `MakeShell`, `MakeSolid` and a `BRepTools_History`
  copy constructor).

**Why:**
- Less code.
- No indirection on every call.
- Missing public APIs get designed properly rather than reached around.
- It also retires a real build123d.NET bug on the native side: its `TopoDS_Iterator` wrapper doesn't keep the shape
  alive, the use-after-free 7Y found (decision 1109).

**Cost:** moderate, a unit or two. Mostly mechanical, guided by the compiler. build123d.NET's own test suite (1,808 tests)
is the safety net.

**Prerequisite:** dropping native build123d.NET, or keeping it only as the oracle's reference copy (see "What we lose").

### 2. Performance, guided by a profiler

**Measured:**

| Comparison | Port against native |
|---|---|
| build123d.NET's suite on OCCT.NET | 4.4× native overall |
| 7X's operations, total time | 1.56× (geometric mean 1.09×) |
| Hollow | 2.19× |
| DPrism | 2.31× |
| SplitByPerimeter | 2.59× |
| ToSplines | 2.67× |
| A B-spline Boolean section (bug83) | about 6× |

**Likely causes**, all specific to .NET and none of them OCCT's algorithms:
- **Allocation.** One DPrism call allocates 16 MB. OCCT heap-allocates small vectors and matrices (`math_Vector`,
  `TColgp_Array1OfPnt`) in inner loops; C++ allocators make that cheap, the .NET garbage collector doesn't.
- **Exceptions as control flow.** OCCT throws and catches `Standard_Failure` routinely (`StdFail_NotDone` as a status,
  `Standard_ConstructionError` in trial constructions). A .NET exception costs microseconds, a C++ one far less.
- **Collections.** NCollection maps ported faithfully, with hashing and iteration order kept for byte equality.

**Change:** profile the hot paths (Booleans, offsets, fillets, meshing), then:
- use `Span<T>`, `stackalloc` and pooled buffers for small fixed-size arrays;
- use try-patterns instead of exceptions where OCCT uses them as statuses;
- use .NET collections where iteration order doesn't reach a result.

**Why:** the spreadsheet recalculates on every change, and Bento's WebAssembly target is slower still.

**Cost:** open-ended but incremental: one hot path at a time, each measurable.

**Most of this needs no loss of compatibility.** Allocation and exception changes that keep results bit-identical can be
done now, with the oracle still running. Only changes to iteration order need compatibility dropped.

### 3. Non-destructive algorithms

**Change:** rewrite the main operations to build new shapes rather than edit their inputs. That covers Booleans, offsets,
sweeps, fillets and healing. Then delete the copy-at-entry, KeepUnchanged and history re-keying machinery, and the masks.

**Why:** removes a whole category of code and of bugs. Histories become direct ("this new face came from that old
face") instead of mapped back through copies. Thread safety becomes structural, not something each algorithm has to
achieve.

**Cost:** large, and spread across many modules. Do it module by module, each verified against the oracle before and
after: geometrically equal results, the same histories as sets. Booleans first, since everything uses them.

**Risk:** OCCT's in-place edits are sometimes load-bearing. OC5805's offset surface is the example: OCCT widens a
surface in place and later code relies on that. Each such site needs understanding, not just a copy.

### 4. Errors and shared state

**Today:** OCCT reports failure four ways: exceptions, status enums (`BRepFill_ThruSectionErrorStatus`,
`ChFiDS_ErrorStatus`…), `Message_Report` alerts, and messages in the style of its Draw interpreter.

**Change:**
- **One model.** A result-and-diagnostics type that maps directly onto Bento's `#VALUE!` with a message, and onto C#
  exceptions where the caller wants them.
- **Per-operation state.** Process-wide state becomes a context passed to each operation: the message table,
  ShapeProcess's operator registry, `Resource_Manager` and the scratch values.

**Why:**
- Bento evaluates cells in parallel.
- Today's process-wide state needs locks, `[ThreadStatic]` and careful test isolation.
- Error messages currently depend on which code path failed.

**Cost:** moderate. Mostly API surface, done together with item 1.

### 5. Fillets, beyond option C

Unit 7R already replaced OCCT's 69,000-line TopOpeBRepBuild with about 900 lines on the Boolean engine. Across 120
recorded cases it matches native's geometry, and where it doesn't, the fault so far has always been native's.

The remaining fragility is ChFi3d itself: thousands of lines of corner special cases (`_C1`, `_C2`, `_CnCrn`). Without
compatibility, a different approach becomes possible, such as a rolling-ball fillet built directly on the Boolean
engine with corners as patches.

I'd only do it **after** the port works and the corpus and Bento users have produced a list of real failures to aim at.

### 6. build123d.NET's Python heritage

**Context:** Bento uses algebra mode only.

**Drop or simplify:**
- the builders (`BuildPart`, `BuildSketch`, `BuildLine`) and `Mode`, which aren't ported yet anyway;
- Python's union return types (a `Compound` that is really a Part, Sketch or Curve by dimension);
- the operator overloads that exist only to mirror Python.

**Keep:** what Bento's mapping uses (`docs/cad/build123d-in-bento.md`): objects, operations, locations, selectors,
`Selector.For`, tessellation.

**Cost:** small. Mostly deletion, and leaving things unported.

## What I'd leave alone

- **The core.** The mathematical core (math, GeomAbs/Adaptor, the B-spline libraries), the curve and surface
  intersections (IntPatch, IntWalk, IntCurveSurface…), the Boolean engine's algorithms, and meshing. These hold OCCT's
  real value, the corpus tests them well, and rewriting them is where reimplementations of OCCT have gone wrong before.
  Optimise them (item 2), don't redesign them.
- **The tolerance model.** OCCT's per-entity tolerances, which grow, are the root of many robustness problems. But
  changing the model touches every algorithm and every file format. Not worth it short of a new kernel.

## What we lose

- **The oracle, the strongest test we have.** Every "identical to native" result in phases 3 to 7 depends on it.
  Without it, correctness rests on:
  - the Draw corpus, which judges by OCCT's own checks;
  - geometric invariants;
  - property-based tests;
  - build123d.NET's and Bento's own suites.

  These catch less.
- **Upstream fixes.** Once a module diverges, OCCT's later fixes to it stop carrying across.
- **A reference for disputes.** "Native does X" settles arguments quickly; a rewritten module has no such reference.

## Sequencing and decision points

1. **Finish phase 7 compatibly,** to get a verified baseline. Then decide.
2. **Do now, without losing compatibility:** item 2's allocation and exception work, checked with the oracle, and item 6.
   (Item 6 is a build123d.NET scope decision, not a rewrite.)
3. **Decision point A:** retire native build123d.NET? If yes, do item 1 and item 4's API side together.
4. **Decision point B:** per module, diverge from OCCT? Keep the native comparison running for modules that haven't
   diverged. For those that have, compare geometrically and record each deliberate difference the way `occt-issues.md`
   records OCCT's bugs. Then item 3, Booleans first.
5. **Item 5** only with a list of real fillet failures in hand.

## Evidence to keep collecting meanwhile

- Profiles of build123d.NET's suite and of Bento's example workbook on OCCT.NET. Where the 4.4× actually goes. *Done in unit 9E: docs/performance.md.*
- The fillet corpus's failures after 7N3 (849 cases), sorted into reconstruction gaps, ChFi3d port bugs and OCCT's own. *Done in unit 11G: docs/corpus-failures.md.*
- Which copy-at-entry sites caused bugs, as opposed to the ones that only cost lines.
