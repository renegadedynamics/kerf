# Phase 11 plan: after 0.2.0

> **Proposed, 5 November 2026, for the owner to agree.** Not started. The owner's order: "Let's get these phase 10 units
> done, then we can go ahead and optimise from there." So optimisation comes first, and the points the owner set aside
> until after 0.2.0 follow it.

## The goal

Phases 0 to 10 made Kerf correct and made it a .NET library. Phase 11 makes it fast, measured on what Bento does, and then
closes what phase 10 left, so that a 1.0 can be considered.

## 1. Where the time stands

The last full review is [performance.md](performance.md) (unit 9E, 2 November), against native OCCT on Bento's eleven guide
examples:

| | Native | Kerf at 9E, ReadyToRun |
|---|---:|---:|
| Opening a workbook (first pass), ms | 921 | 2,034 |
| Recalculation (median), ms | 685 | 1,162 |
| Allocated per recalculation, MB | 3 | 1,025 |

Unit 10J since cut "extrude until" from 802 MB to 142 MB a recalculation. Phase 10 changed most of the code above the
engine, and 10Q2's before-and-after only shows it did no harm (936 ms against 949 ms, summing medians). So these figures
are stale, and the first job is to measure again.

Three things to keep in mind:
- **Memory is the widest gap.** Allocation also drives the parallel cost: under load, the garbage collector was the
  largest single loss at 9E.
- **Opening is mostly the JIT.** ReadyToRun took about a third off; what's left needs measuring separately.
- **Native is a yardstick, not the goal.** Its figures are frozen with the recordings, and can't be measured again.

## 2. The optimisation units

In order. Each takes its figures from 11A, and states before and after. Results stay identical unless a unit says
otherwise and records why: the corpus, the recordings and the Python-reference values are the check.

| Unit | What | Done when |
|---|---|---|
| **11A** Benchmark rig | One command that runs a fixed set on a quiet machine and writes a dated table: Bento's examples (first pass, recalculation, allocation), Booleans, fillets, meshing, STEP read and write, parallel throughput. Fails if the machine is busy. Bring `Kerf.Kernel.Bento.Benchmarks` and `Kerf.Kernel.Benchmarks` into a build the gate checks, since 10Q broke one unnoticed. | A baseline table for 0.2.0 is in `performance.md`; a regression check exists |
| **11B** Allocation | Profile by allocation. Likely first: small vectors and matrices on the stack (performance.md's proposal 5), then whatever else the profile names. Take the remaining collections from [collections-plan.md](collections-plan.md) only where they show up: that plan is about 25,000 references, and converting it all isn't the aim. | Allocation per recalculation down by a target set from 11A |
| **11C** Opening | Separate JIT, type loading and first-call cost. Try composite ReadyToRun, tiered settings, and avoiding work at load (static tables, large initialisers). | Opening's figure, with what was tried and didn't help |
| **11D** Parallel | Throughput across cores, with Bento's recalculation as the case. Contention from the 23 locks ([state.md](state.md)), shared caches, and the collector's settings. This is where the locks get looked at, so plan 2.7's "only one-time locks" is settled by evidence. | A scaling curve; locks that cost anything gone |
| **11E** Algorithms | With the profile from 11A, the hot paths: Boolean setup, "extrude until", healing after Booleans. Phase 10 allowed divergence from OCCT, so an algorithm may change, with its results checked as intended differences. | Named, measured wins |
| **11F** Caching in Bento | Bento knows which cells really changed. Cache meshes and shapes across recalculations by the shapes' identity. Immutability makes this safe, and it may help users more than any engine work. On a Bento branch, as usual. | Recalculating an edit to one cell, measured |

**A separate question: fillets.** A rolling-ball rewrite (beyond-compatibility.md item 5) only makes sense with a list of
real failures in hand. **11G** sorts the fillet corpus's failures into reconstruction gaps, port bugs and OCCT's own, then
reports. Any rewrite is a decision after that, not part of this plan.

## 3. What phase 10 left

The owner set these aside until after 0.2.0 (decision 1642). They don't depend on the optimisation, and can run alongside it.

| Unit | What | Note |
|---|---|---|
| **11H** Re-measure coverage | About four hours with `-WithOracle -FullCorpus`. The figures predate 10Q and 10Q2. | Run first, since 11I depends on it |
| **11I** Coverage floor (question 49) | 81.6% overall at the last measure; 81 of 272 packages under 80%, 41 of them STEP's. The corpus adds at most 0.2 points, so all of them are real gaps. | **Owner's decision.** My recommendation: hand-written engine packages first, then decide on generated STEP entities, where the floor buys least |
| **11J** Tests of their own (plan 3.2) | 0 of 4,417 public members are uncalled, but "called" includes indirect calls. Tighten to a direct call and a checked result, area by area, starting from what users meet first. | Large; do it in slices |
| **11K** The remaining exceptions | Operators, constructors and the shapes' own methods still throw (decisions 1589, 1592). | **Owner's decision**, since they left these for now. Operators can't return a result without losing what makes them useful, so my recommendation is to keep them, and add a `Try` form only where Bento shows a need |
| **11L** Sharp edges in the API | From `one-set-of-shapes.md`'s "Left": `shape.BoundBox()` beside `shape.BoundingBox()`, with different defaults; clashes with `Kerf.Meshing`, `Kerf.Exchange` and `Kerf.Offsets`; odd internal names. | Best done before anyone relies on them |
| **11M** Engine tidying | About 3,300 IDE1006 exemptions; the `Transient` base; 572 "touch" rows in the bug triage, fixed when their code is next changed. | Opportunistic: no unit of its own unless something needs it |

## 4. Questions for the owner

1. **Is this order right?** Optimisation first, with the coverage work alongside it.
2. **A target for optimisation?** I'd suggest deciding after 11A's baseline, rather than guessing now. "Recalculation within
   1.2× of native's frozen figure, and allocation down by half" is a possible first target.
3. **The coverage floor (11I):** hand-written engine packages first?
4. **The remaining exceptions (11K):** keep them?
5. **STEP's header.** STEP files written by Kerf still say organisation `Open CASCADE`, an Open CASCADE processor and the
   product name `Open CASCADE STEP translator` (decision 1641). Phase 10 kept STEP's bytes identical, so they stayed. Since
   Kerf is a modified version and isn't endorsed by Open Cascade SAS, I recommend changing them to Kerf. Other tools may
   read those strings, but they shouldn't act on them. It changes every STEP file's bytes, so it needs its fixtures updating
   once, and is simplest done early.
6. **The unported rest of OCCT.** The port status is 62.0%, and nothing in this plan adds to it. I suggest porting more only
   when something needs it.
7. **What would 1.0 mean?** A stable API needs real users to test it. I suggest considering it after phase 11, once Bento has
   run on Kerf for a while.
