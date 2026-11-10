# Phase 15 plan: towards 0.6.0

> **Proposed, 10 November 2026** (decision 2091), for the owner to agree. Phases 13 and 14 close with 0.5.0 (decision 2090):
> their open units, 13H, 13L and 14B's larger items, move here, beside a unit for each loose end left at the release.
> The release is 0.6.0, with no fixed date: when section 1 is done and the corpus and the users' cases show nothing
> worse than 0.5.0. Sections 2 to 4 ship with whatever is ready by then.

## The goal

The owner, 10 November 2026: "Let's put parallel volumes into 0.6.0. Make a plan to sweep up the remaining phase 13 and
14 units, and add units for everything else here." So 0.6.0 brings the parallel volume integration that 0.5.0 left off
(decision 2088) as something a caller chooses, finishes the hot paths the enclosure's profile named, and clears the
loose ends the release left: a rig crash, an unexplained rise in bytes, meshes a little over their deflection with
nothing said, and a gate that runs every suite for every change.

The standing directions hold: no success for an invalid result (1742), Kerf isn't bound by OCCT (1743), the bar of
decision 1829, immutability and a functional style (2039), and a real-world speed-up that slows another case past the
bar goes to the owner (2041). Kerf has several clients: what's new reaches them through the CHANGELOGs, the guide and
the API documentation, and no client is changed here (2039).

## 1. Parallel volumes, chosen by the caller (decision 2089)

| Unit | What | Done when |
|---|---|---|
| **15A** Parallel volumes as an option | Decision 2081's integration of a volume's faces on several threads, off since decision 2088, offered to callers: a caller building one large part gains about 20% of its wall time, and a caller already running builds side by side loses processor time and waits, so it's off unless asked for. The option goes where the volumes are measured for the caller: the Boolean's options (its volume check, decision 1860), `Volume` and the mass properties, and a scope for code that doesn't pass options, following `Cancellation.Use` (it flows as decision 2076's scopes do). The owner chose both, a setting and a scope (decision 2092). | The option on, the same values to the bit as off (the 8,724 properties of decision 2081 and its test); off, the rig's figures as 0.5.0's; on, the enclosure built by one caller measured in a quiet window; the guide's section 9 says when to use it; the public API record, both CHANGELOGs. **Done** (decision 2101), the quiet window's figures left to `quiet-15a.ps1`: `BooleanOptions.ParallelVolumes` and `MassOptions.ParallelVolumes` (`bool?`, null following the scope), the scope `ParallelVolumes.Use()` on decision 2076's machinery, now shared with `Cancellation`, and `parallelVolumes` on Kerf.Modeling's `Cut`, `Fuse`, `Intersect` and `ComputeVolume`; 8,728 properties on and off, 0 differing; off, the bytes as 0.5.0's within the processes' noise. |

## 2. The hot paths left from phase 14 (14B)

Each measured on the enclosure, the users' cases and the rig, by bytes and counts where the machine is shared and by
time in a quiet window; anything that slows another case past the bar waits for the owner.

| Unit | What | Done when |
|---|---|---|
| **15B** The Boolean's memory per face | What decision 1801's checks keep for each face, and what a frozen solid keeps with its data, measured on the enclosure's Booleans; and the clearing in `GaussWork.Rent`, which clears work vectors that the integration then fills. | **Done** (decisions 2095 to 2098): measured on the enclosure, each explained; the integration's work set no longer zeroed on every rent (a fifth of the integration's samples), the check's cleanness found once and its map sized by the faces to check, the shell's edge map built once; the same bits on 8,730 properties, the same statuses on every check of the suites, the same fingerprints. Below the bar in bytes, kept as 2077 was: for the owner to drop. |
| **15C** Points classified one by one | `IsInside` builds a classifier for each point it's asked; and decision 1937's proof that a fuse lost nothing still classifies about 150 points of the boss's faces (13R): a classifier built once for many points, and a quicker one for points that the boxes already decide. | **Done** (decisions 2110 to 2114): a frozen shape's classifier kept for the next point and `IsInside` one classification (the housing's 224 points 2,291 to 4.7 MB); line and circle edges measured before their extrema; the Boolean's builder no longer fills a tree for the point at infinity (115 MB a build). The enclosure −7% in bytes, the boss −5% (14.1 MB, under 0.3.0's), the detent −3%, Bento's examples the same; 162,458 points of the dataset the same states; OCCT's stale parallel flag found (2111). What's left in 2114. |
| **15D** More real-world scripts | Phase 14's method (14A's profile, 14C's feedback, 14D's user case) on the next scripts the owner names. | Each script profiled, its user case in the release check, feedback for its agent; starts when the owner names one. |

## 3. Carried over from phase 13

| Unit | What | Done when |
|---|---|---|
| **15E** Phase 12's open units (13H) | 12G (cylinders of radius 500: the steps that still fail, loudly), 12J (on down occt-defects.md from where decision 1936 left it), and slices of 12Q and 12R (coverage and tests of their own). 12F (the equal tee) only if a client hits it. | Ongoing, as each was; each slice recorded. **Slice 1 done** (decisions 2103, 2104): 12J's #1629 right through the public Boolean (a retry with a fuzzy value of ten tolerances, its cause found), and an empty common it made no longer a success. Stopped there; the next slice starts with `Kerf.Modeling`'s Booleans, which have neither retry, then #245, #1639 and the self-interfering fillets; 12G not reached. |
| **15F** Wasteful cases (13L) | On to decision 1829's bar, with the users' cases and the enclosure in the list. | Ongoing. **First slice stopped** (decisions 2105 to 2109): the housing's chamfer, Bento's chamfer and fillet, and Bento's point lists chosen; curves on planes kept in a table that no longer loses them to collisions, planes further apart than `IsTangentFaces`' angle answered without its samples, and four small wastes in the mesher and the extrema. The chamfer and fillet −20% bytes, the housing's chamfer −17% to −28%, the boss's fuse −19.5%, Bento's examples together −5.2%, the point lists −4.5% (under the bar); every result the same, the filtered corpus as `corpus-13.txt`. Time left for a quiet window (`quiet-15f.sh`). Next: the point lists' meshing (the face checker's tree nodes, the cell filter's and the node map's growth). |

## 4. Loose ends from 0.5.0

| Unit | What | Done when |
|---|---|---|
| **15G** The rig's "first-load bytes" crash | The rig sometimes stops on its first-load bytes measurement; found, then fixed or made to report the measurement it couldn't take. | The rig runs it many times over without a crash. **Done** (decisions 2099 and 2100): the rig let go of its collectible context after loading the probe, the runtime finalised and so unloaded it on a collection between two of Kerf's loads, and the next load was refused; now held until the work is done. 10 of 10 processes failed with a 1 MB gen0 budget and 0 of 10 after, 0 of 20 plain; not Kerf's, and not Bento's, whose loader holds its context. A process that fails is now noted and the run goes on. |
| **15H** Extrude-until's picture | The rig's extrude-until PNG case uses 17% more bytes than at decision 2069's run, unexplained. | The cause found; fixed if it's a waste, recorded if it isn't. **Done** (decisions 2093 and 2094): not a change between the runs; the row was the last pass, 7.2 or 8.5 MB by whether a generation 2 collection had lost the mesher's weakly held lists (decision 1762) since the pass before, 1.3 MB grown again on the half torus. Under the bar and decision 1877's limit, so it stays; the rig's images rows now start each pass with a full collection and take the median of the passes: extrude until 8.383 MB, as 0.5.0's run. |
| **15I** Meshes a little over their deflection | Decision 2071's warning finds every face over its deflection by more than 3.5%; 25 faces between 1.002 and 1.035 times over get no warning. Either the measure that finds them, at a cost in bytes the rig shows, or the meshes kept within it. | Each of the 25 warned of or within its deflection; the meshes otherwise unchanged; the cost measured. **Done** (decision 2102): the guard measured each triangle at its centre and its links' middles only, and a skewed triangle on a torus is furthest from it at a median's midpoint. The torus's guard now measures there once the control has nothing left to add, and refines: 14 of the 25 within, 11 warned (beside a frontier link), every face over now warned of; four of the survey's 448 meshes change (+0.025% triangles), the rest and every absolute mesh the same; the rig's meshing files +0.04% bytes, the images cases unchanged. |
| **15J** A gate that runs what a change touches | Item 2 of decision 2085's list: the gate chooses its suites from the paths a change touches. First a test that the kernel's projects can't depend on Kerf.Modeling, so a change to the modelling layer can skip the kernel's suites. | The test in the gate; the choice of suites in the merge script, with a full run before each release; the time saved measured on a few merges. |
| **15K** The status page | The status page published at 0.4.0 moved to 0.5.0 and phase 15. | Republished. **Done**, 10 November 2026, at the same link. |

## Not in this phase

- SVG and DXF import.
- General-surface fillets and replacing ChFi3d, until clients' failures justify them.
- Changes to any client (decision 2039), or to a real-world script (decision 2041): feedback goes to the owner.

## How the work runs

As in phases 13 and 14: at most three helpers, each on a branch and worktree of its own, then the merge gate, one at
a time. 15A and 15K first; 15B, 15C, 15G and 15H next, with 15E, 15F and 15I as slots free; 15J when the gate isn't in
use; 15D when the owner names a script. Results stay identical unless a unit says otherwise and records why. Timed
claims come from a quiet machine; the rig compares against `docs/benchmarks/2026-11-10-33b1e7dc7.tsv` (decision 2087),
and the release check against 0.5.0's packages. Nothing is pushed, except by the owner.

## Questions for the owner

1. **15A:** an option on the operations' options records plus a scope, as above, or the scope alone? **Both** (the owner,
   decision 2092).
2. **15D:** which script next, and when?
