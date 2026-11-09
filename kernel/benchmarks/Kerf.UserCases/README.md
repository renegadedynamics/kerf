# Users' cases

Parts shaped like users', each with a fixed result, so a release can't be slower on them, or change their answer,
without anyone noticing (unit 13C, decision 25301). 0.4.0 shipped two slowdowns a user's agent found on the first day,
which the rig had no part shaped like.

| Case | What | From |
|---|---|---|
| `fuse-boss` | a screw boss (a cylinder, r 3, 5.5 long) fused onto an enclosure's door; the fuse and its volume timed | the agent's report on 0.4.0: 18 ms on 0.3.0, 86 ms on 0.4.0 |
| `fuse-detent` | a 1.5 x 0.5 x 1.5 bump fused onto a slanted facet of the door | the same: 59 and 225 ms |
| `chamfer-guide-box` | the guide's box (40 x 30 x 30, its vertical edges filleted 1.7), its top's 8 edges chamfered 1.4 | the guide; the plain chamfer beside the next |
| `collapse-guide-box` | the same edges chamfered 3 with `TightCorners.Collapse` | the agent's second report: 256 ms against 3 ms |
| `chamfer-housing` | a coil housing's section extruded, the 20 edges round its end chamfered 1.4 | the enclosure's housing |
| `collapse-housing` | the same edges chamfered 3 with `TightCorners.Collapse` | the same report: 428 ms against 7 ms |
| `enclosure` | the blade bumper's enclosure built whole: 13 printed parts, a print aid and 34 placed components, 48 shapes | `enclosure/`, with the owner's permission (decision 2008) |

Each case's result is every shape's volume (to 1e-6 relative), validity and face count, or the failure it returned;
`Expected.cs` holds them, written by the program's `expected` mode. A change that moves one is a bug, or a fix with a
decision that says why, which rewrites `Expected.cs`.

The STEP files in `data/` are the operands the agent saved from the enclosure model just before each fuse, and the
housing's section extruded (`housing-prism.step`). The agent's scripts timed the same operations, warm, on 0.3.0, 0.4.0
and native OCCT through build123d.

## Who runs them

- **The release check**, `tools/release-check.ps1`: the last release's packages from nuget.org against the candidate,
  warm, interleaved, median of 9; a case more than 20% slower, or with a different result, stops the release
  (docs/performance.md, "The release check").
- **The rig**, `tools/bench.ps1`: the `user-cases` suite, group `user cases`, a time, the bytes allocated and the
  result for each.
- **Their test**, `UserCasesTests` in Kerf.Modeling.Tests: the program's `check` in a process of its own, each case
  twice from the same operands, against its fixed result on the tree.

## Versions

The same source builds against this tree (by default) or a released version's packages (`-p:KerfPackageVersion=0.3.0`).
A case that uses a later API is compiled only from the version that has it, by the symbols the project file defines
(`KERF_0_4_OR_LATER`): before 0.4.0 the collapsing chamfers run without `TightCorners`, which those versions refuse, and
the enclosure isn't built (the release check says it has no baseline). When a release changes an API the cases use, the
change goes in the same way, with a symbol for the new version, so the cases still build against the last release.

By hand, from the kernel folder: `dotnet run -c Release --project benchmarks/Kerf.UserCases -- check` runs each once
against its fixed result; `Program.cs` lists the other modes.
