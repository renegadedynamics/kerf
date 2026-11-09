# Phase 14 plan: real-world scripts

> **Agreed, 9 November 2026** (the owner; decision 2041). Runs beside phase 13: what's merged by 0.5.0 ships in it. Its
> first script is Renegade Dynamics' blade bumper enclosure; others follow as the owner names them.

## The goal

The owner: "We're going to start looking more at the performance of real world scripts." The rig and the corpus
measure Kerf's operations one by one; a real script is many operations in a pattern no benchmark chose, and 0.4.0's two
slowdowns were found by one (13C). So phase 14 profiles real scripts end to end, finds their hot paths, and makes Kerf
faster on them.

**Trade-offs go to the owner** (decision 2041): "Report back when improving a real-world case has a negative impact on
other test cases." A change that speeds a real script and slows anything else (the rig, the users' cases, the corpus's
timed or measured cases, another script) past decision 1829's bar isn't merged on a helper's judgement or mine: it's
reported to the owner with both sides measured, and waits for their decision.

**The script itself is the owner's to change, through its own agent:** where a script could be quicker written another
way, that's written up as feedback for the owner to pass on, and nothing in the script's repository changes.

The standing directions hold: no success for an invalid result (1742), Kerf isn't bound by OCCT (1743), the bar of
decision 1829, and immutability and a functional style (2039).

## 1. The blade bumper enclosure

The Kerf model at `products/rowing-strain/enclosure/kerf/` in renegadedynamics/hardware, commit `9ab5fb0` (v0.16,
9 November 2026), with its build123d twin `blade_bumper.py` (the reference) and its spec tests. 13C's user case
`enclosure` is the model at `3ae3778` (v0.11).

| Unit | What | Done when |
|---|---|---|
| **14A** Profile | The whole build, every shaft size: wall time cold (a fresh process, as the script runs) and warm, bytes, and where they go, by the script's parts, by Kerf operation (Boolean, fillet, chamfer, hull, sweep, export, mesh, render, drawing) and by kernel phase; against the build123d twin on native OCCT, part by part. The spec tests' own time too. | A ranked list of hot paths, with the gap to native for each, in performance.md. |
| **14B** Kerf's hot paths | Down 14A's list: each fix measured on the enclosure and on everything else (the rig's rows by counting, the users' cases, a filtered corpus). Trade-offs to the owner, as above. | Each hot path faster to the bar or explained; no change merged that slows another case past the bar without the owner's word. |
| **14C** The script | Where the script could do the same work for less in Kerf (caching, order of operations, an operation Kerf has that it hand-builds, work it repeats per size), each with what it would save, measured. | A note for the owner to pass to Renegade's agent. |
| **14D** The user case | 13C's `enclosure` case moved to v0.16 (or a second case beside it), so the release check covers the model as it's built now. | The release check runs it. |

## 2. Vector instructions, parallelism and async (the owner, decision 2046)

| Unit | What | Done when |
|---|---|---|
| **14E** Vector instructions | Led by 14A's profile and the rig's hottest loops: loops over many values of one kind (points classified against a solid, boxes and trees in the classifiers and the mesher, B-spline bases and polynomials over many parameters, the mesher's triangle and circle tests, the renderer, mesh export). Points stay `Point3D` and `Point2D` in `double` (`System.Numerics`' `Vector3` and its kin are `float`, too coarse for CAD's tolerances, and .NET has no `double` versions); where a loop pays, it holds its points as arrays of x, y and z inside and uses `TensorPrimitives` or `Vector256<double>`. Each prototype measured; results bit-identical, or a change in results brought to the owner. | Each hot loop vectorised where it meets the bar, or written down as not worth it. |
| **14F** Parallelism | First, the ambient state (`Cancellation.Current`, `Warnings.Capture`, `SkipClean`: 13P's decision 2045) checked for callers on many threads, and the thread-safety of operations and shapes documented as a guarantee. Then the enclosure built with its independent parts in parallel, as a client would: if it pays, script feedback and a guide section. Then parallelism inside long operations where the profile shows them (a Boolean's face pairs, the checks' points, fillets of many edges, exports). | The guarantee written and tested; each candidate measured on a quiet machine. |
| **14G** Async | No async versions of the modelling operations: they're CPU work, and wrapping them in tasks hides the cost and takes the choice of threads from the caller. Async reading and writing of files and streams (STEP, STL, glTF, 3MF, DXF, SVG), and a guide section on running operations in parallel. | The readers and writers have async forms; the guide says how to parallelise. **Done** (decisions 27701 and 27702): `…Async` forms of every reader and writer of the kernel (BREP, STEP, STL, glTF, 3MF, OBJ, SVG, PNG, fonts) and of the modelling layer's importers, exporters and `ExportDxf`, the I/O asynchronous and the parsing or building synchronous, with the same results and bytes as the synchronous forms; on the dataset's largest STEP file (22.4 MB) reading allocates 22 to 26 MB more (1%), writing 29 to 33 MB more. The guide's section 9 runs operations in parallel, checked by a stress test of the operations it teaches (parts built, meshed, drawn, rendered and exported on several threads, and one part shared by all): the same to the bit. The ambient scopes are per flow and don't leak between callers; a scope disposed on another flow stays open on its own, for 14F. |

14E and 14F start when 14A has reported and the machine is free of phase 13's helpers, and take their final figures in a
quiet window. 14G can start when a helper's slot frees.

## How the work runs

As in phase 13, and alongside it: helpers on branches and worktrees of their own, the merge gate one at a time.
Timing comes from a quiet machine; where the machine is shared, judge by bytes, samples and counts, and say so. The
hardware repository is read only through `git show` and `git archive` of committed revisions: another agent works in
its checkouts.
