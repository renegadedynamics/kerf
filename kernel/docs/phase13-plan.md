# Phase 13 plan: towards 0.5.0

> **Agreed, 9 November 2026** (decisions 2007 and 2008), with the owner's answers to the questions below. The release is 0.5.0,
> with no fixed date: when sections 1 to 3 are done, the corpus shows no new failures, and the user cases of 13C build no
> slower than in 0.4.0. Sections 4 and 5 ship with whatever is ready by then.

## The goal

0.4.0 made results trustworthy first, and a user's agent then found two things it made slower: fusing a small part onto
a large one, 4 to 5 times 0.3.0's time, and a chamfer that collapses tight corners, 30 to 80 times one that doesn't.
Neither showed on the rig, which has no part shaped like theirs. So phase 13 is **put right what 0.4.0 made slower, and
make sure a release can't do that again; then text and DXF, so drawings can be finished in Kerf; then failures as
`Result`s**.

The standing directions hold: no success for an invalid result (decision 1742); Kerf isn't bound by OCCT (1743);
wasteful cases found and fixed to the bar of 5% over many operations or 20% on a slower or larger one (1829); and look at
the worst case, not only the typical.

## 1. The user's agent's two reports

First, and small. Repros: the operands as STEP files and the scripts, from the agent's folder, copied into the tests.

| Unit | What | Done when |
|---|---|---|
| **13A** Small parts fused onto a large one | A screw boss (a cylinder, r 3, 5.5 long) onto an enclosure's door: native 11 ms, 0.3.0 18 ms, 0.4.0 86 ms. A 1.5 × 0.5 × 1.5 bump on a slanted facet: 11, 59 and 225 ms. Same results on all three. **Suspected:** decision 1937's proof that a fuse lost nothing, which classifies 9 × 9 points on every face of the smaller input against the result, on every fuse; or decision 1932's side check. Measure first: time and samples per step, at 0.3.0 and 0.4.0. | Within 10% of 0.3.0's time on both, or the cost left is named and under the bar, with every check still catching what it was added for (#1643's lip, the corpus's 3,997 Booleans counted as in decision 1937). **Done** (decisions 2034 to 2038; performance.md, "Small parts fused onto a large one"): all of 0.4.0's rise was decision 1937's points, on faces already on the result's boundary. Those are passed over unclassified, and a classifier's ray skips the faces it misses. The bump now takes 0.6 to 0.7 times 0.3.0's time (351 to 24 MB). The boss takes 1.3 to 1.4 times (85 to 17 MB, 0.3.0's 14), and the cost left is named: its 150 points inside the door, about 5 to 7 ms. #1643 is still proved lost; over the corpus's 3,997 Booleans, none fired and the checks' time fell by about a third. The pattern: 1924 doesn't run, 1985 is under the bar, and 1932 and 1866 still cost more than their Booleans on this shape, as in 0.3.0 (for 13K). |
| **13B** The collapsing chamfer *(done: decisions 2025 to 2027)* | `TightCorners.Collapse`: the guide's box 256 ms against 3 ms without it, the enclosure's housing end (20 edges) 428 ms against 7 ms. It builds with one Boolean against a tool of the chamfer's faces, then checks by sections and Gauss-Legendre (decisions 1990, 1991). Measure which part costs it. | Each case under 50 ms, or the cost explained and as low as the method allows, with the same results to the checks' tolerance. |
| **13C** User cases in the release check | The rig missed both. Add rows built from users' parts: the two fuses, the collapse, and the enclosure's whole build (question 1: agreed). A release candidate runs them against the last release's packages, warm and median of 9, and a row more than 20% slower stops the release until it's explained. | The rows exist, run in the release script, and would have stopped 0.4.0. **Done** (decisions 2015 to 2017): seven cases in `benchmarks/Kerf.UserCases` with fixed results and their test, `tools/release-check.ps1`, and the rig's `user-cases` suite. 0.3.0 against 0.4.0 stops on both fuses (4.0 and 4.9 times) and both collapses (35 and 38 times); 0.4.0 against main passes. Found on the way, for 13G: a kernel's refusal can escape as a bare `ArgumentException` once the runtime has optimised the call (decision 2017). |

**Then look at the pattern.** Phase 12 added several checks that run on every operation (decisions 1924, 1932, 1937,
1985). Each is counted on a fuse or chamfer of something small against something large, the shape these two reports
share, and any over the bar is made cheaper or confined to where it's needed.

## 2. Text and DXF (decision 2004)

The gaps for drawing sheets: title blocks and notes in the model, and a file a laser cutter or another CAD program reads.

| Unit | What | Size | Done when |
|---|---|---:|---|
| **13D** Fonts | A managed reader for TrueType (`glyf`, quadratic) and OpenType (`CFF`, cubic) outlines, with `cmap`, metrics and pair kerning (`kern`, and GPOS's simple pairs). Fonts found by family and style on Windows, Linux and macOS, plus a default font shipped in the packages, Liberation Sans (question 2). OCCT reads fonts through FreeType, which isn't managed, so this is new code, not a port. Damaged font files are refused with a limit, as STEP is. | 1,500 to 2,500 lines | Glyph outlines match FreeType's for a set of fonts to 1e-6 of the em; fuzzed and damaged files fail cleanly. **Done** (decisions 2018 to 2023): `Kerf.Fonts` in the kernel; 595,931 glyphs of 377 fonts (CID-keyed CFF and collections among them) agree with FreeType 2.13.2 to 4.4e-7 em; 36,000 damaged fonts fail cleanly; Liberation Sans 2.1.5 shipped, its licence in THIRD-PARTY-NOTICES.md. |
| **13E** `Text` | build123d's `Text`: a sketch of a string's faces, with size, font, style, alignment and text on a path, as in its `Text(txt, font_size, font, font_path, font_style, text_align, align, path, position_on_path, single_line_width, rotation, mode)`. `single_line_width` draws single-stroke fonts for engraving, as strokes of that width. Extrude it to emboss or engrave. Name and behaviour from build123d, measured against it. | 400 to 700 | build123d's text examples give the same faces, areas to 1e-6, and the same bounding boxes. |
| **13F** DXF export | build123d's `ExportDXF`: lines, arcs, circles, ellipses and splines from a `LineDrawing` or sketch, with layers, colours, line types and units, and text as outlines by default (question 3). build123d writes DXF 2013 through `ezdxf` by default; Kerf writes the same, with an older version as an option if laser-cutter software needs one. | 800 to 1,200 | Every drawing in the guide written, read back by Kerf's own test reader and by `ezdxf`'s audit with no errors, and opened in LibreCAD by hand once. **Done** (decisions 2009 to 2012): `ExportDxf` in `Kerf.Modeling`, 2013 and R12; the reader, the audit (69 files, no errors or fixes) and build123d's own output (23 shapes, to 1e-9) agree. Left: LibreCAD by hand, the owner's. |
| **13F2** Text in drawings | Notes and a title block on a drawing, written to SVG and DXF as outlines by default, or as text as an option (question 3). | 300 to 500 | The guide has a sheet with a title block, in both formats. **Done** (decisions 2030 to 2033): `DrawingNote` on a `LineDrawing` and `Sheet` in `Kerf.Drawing`; SVG and DXF write notes as outlines (filled paths, closed polylines) or as text (`<text>`, TEXT with a style per font); every file valid XML or through ezdxf's audit with no errors or fixes (77 files), and read back by the test reader. |

## 3. Failures as `Result`s (decision 1999)

| Unit | What | Done when |
|---|---|---|
| **13G** Throwing constructors | Decision 1999 lists about 20 public constructors in `Kerf.Modeling` that throw on input a caller would think valid, with two exception types. As the owner decided (question 4): one exception type with the error's code, and a `Make…` operation returning a `Result` beside each, as `MakeConvexPolyhedron` is. | Every one on the list is done, and a test walks the public constructors to find any that throw something else. **Done** (decision 2013): all 66 constructors of the 48 objects were found to fail on values that look valid; they throw `ModelingException` (the layer's failures all are) and each has a `Make…`; `ConstructorFailureTests` walks them. The kernel's argument checks now throw from a frame of their own, as the JIT's inlining hid them from the layer (2014). |

## 4. Carried over

| Unit | What |
|---|---|
| **13H** Phase 12's open units | 12F (the equal tee, ChFi3d's walk: only if Bento's users hit it), 12G (radius 500), 12J (on down occt-defects.md), 12Q and 12R (coverage and tests of their own). |
| **13I** Chamfer follow-ups | `MaxChamfer` gives 1.594 where 1.699 builds, as its search stops at 0.1 like build123d's `max_fillet`: give the proved limit where 12V has one. Collapse beside another arc, on B-spline outlines and on drafted sides, which are refused today (chamfer-corners.md). |
| **13J** Loose ends | The cause of A1's changed offset (12T: the new result is right, the reason isn't written down). The two lofts damaged to 1e300 that run out of memory rather than fail. |
| **13M** Kernel refusals by type | The layer told a kernel's refusal of a value from its own misuse by the assembly that threw, which the JIT's inlining changes once a call is hot (decisions 2014, 2017): "sometimes it works". Make it not depend on where code was compiled, remove 13C's child process, and make tiny primitives that come out invalid fail. **Done** (decisions 2028, 2029): the kernel's 403 argument-exception sites throw its own three types (derived from .NET's), told by `Diagnostic.IsKernelArgumentError`, and KERF0004 keeps it so. 13G's `Refuse` is gone, and the users' cases run in the test process. On main 5 of 65 failing cases changed once warm; here none do, and a deterministic test that fails on main passes. A cylinder, hole, sphere, torus, wedge or rectangle too small to be valid fails, carrying the shape. Modelling suite: 2,575 passed, three runs on 2 threads. |

## 5. Speed

| Unit | What |
|---|---|
| **13K** The checks' share of a Boolean | After 12Y, Kerf's checks of each result are about 40% of a Boolean's time (0.56 times native without them, 0.94 with). Find which parts of the check could be cheaper without missing what they catch. All stay on by default (question 5). With it, from 13A: the boss fuse, 1.3 to 1.4 times 0.3.0's time (decision 2038), and decisions 1932's and 1866's checks, which cost more than their Booleans on a small part fused onto a large one. And **decision 1924's point check, in a cheaper form, on by default** (the owner, decision 2039). |
| **13L** Wasteful cases | Carries on to decision 1829's bar, with 13C's user cases in the list. |
| **13N** Filled text in DXF | Notes written as outlines are their contours alone in DXF, so a viewer draws them hollow, where the SVG fills them (the owner, in LibreCAD). Fill them with solid hatches in DXF 2013, contours alone in R12 (decision 2039). **Done** (decision 2042): in 2013 each outer contour is a solid HATCH with its holes, under the contours, which stay; `fillOutlines: false` for the contours alone; R12 unchanged. LibreCAD's `dxf2png` draws the sheet's text filled, where before it was hollow. |
| **13O** Relative deflections | The torus's and sphere's meshes keep to the deflection only when it's absolute (decisions 1929, 1953); with a relative one they're OCCT's, which can miss it. Extend the fixes (the owner, decision 2039). |

## The owner's answers (decision 2008)

1. **The enclosure as a rig row:** yes, the blade bumper's enclosure model from renegadedynamics/hardware may be included.
2. **A default font:** ship one, suitable for CAD. **Liberation Sans** (SIL Open Font Licence 1.1): the same advance
   widths as Arial, build123d's default, so text built from build123d's examples comes out the same size; its four styles
   for build123d's `FontStyle`. The ISO 3098 lettering fonts aren't as free to ship (osifont is GPL with a font exception).
   Its licence goes in the packages' notices.
3. **Text in SVG and DXF:** outlines by default, text as an option.
4. **Constructors:** one exception type, plus `Result` versions.
5. **Checks:** on by default; making them cheaper is investigated first (13A, 13K).
6. Still open from phase 12: Bento's `tight_corners` option (12w-bento.md), the point check (decision 1924),
   `bug27928` as a known difference, relative deflections, and closing phase 12.

**Later answers (decision 2039):** the point check in a cheaper form, on by default (13K); `bug27928` kept as a known
difference; the mesh fixes extended to relative deflections (13O); phase 12 closed (decision 2040). And Kerf has several
clients now, Bento one of them: Kerf's work no longer changes Bento or writes notes for it. Each client takes what's new
from the CHANGELOGs, the guide and the API docs, so those must say everything a client needs.

## Not in this phase

- SVG and DXF import.
- General-surface fillets and replacing ChFi3d, until Bento's users' failures justify them.
- Bento's caching (11F), which is the owner's with another agent.

## How the work runs

As in phase 12: at most three helpers, each on a branch and worktree of its own, then the merge gate, one at a time.
Results stay identical unless a unit says otherwise and records why. Kerf keeps to immutability and a functional style:
values that don't change once made, operations that return new values, and no hidden shared state (the owner, decision 2039). Timed claims come from a quiet machine; the rig
compares against `docs/benchmarks/2026-11-09-2a330b9b7.tsv` (decision 2003), and from 13C also against the last
release's packages on users' cases. Nothing is pushed, except by the owner.
