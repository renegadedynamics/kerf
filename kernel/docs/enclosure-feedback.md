# The blade bumper enclosure: what the script could do for less

*For the owner to pass to the script's agent (unit 14C, decision 27105). The script: renegadedynamics/hardware `9ab5fb0`
(v0.16), `products/rowing-strain/enclosure/kerf/`. Nothing in that repository was changed.*

Measured on a busy shared machine, so bytes allocated are the steady figure and times are rough (processor cycles,
median of 3, each way taken in turn). Size 35, warm, on Kerf 0.4.0 as the script uses it; main in brackets where it
differs. "The same part": the same volume to 1e-12, the same validity, faces, edges and solids, and the same bounding
box to 1e-9 mm, so the script's reference tests (0.5% and 0.1 mm against build123d) pass as they do now.

## 1. One Boolean with all the tools, not one per tool

Each `x -= tool` (or `+=`) is a whole Boolean: Kerf goes over every face of `x` each time, however small the tool. Kerf's
`-` and `+` take a list (`part - new List<Part> { a, b, c }`), and do it in one go. build123d's `-` takes a list too, so
the twin can do the same.

| Part | As written, MB | One Boolean per step, MB | Cycles | The same part |
|---|---:|---:|---:|---|
| housing: its 12 cuts as 1 | 1,100 (1,081) | 358 | −66% | yes |
| housing: only the 4 nut shafts and 4 bolt holes as 1 | 1,104 | 700 | −45% | yes |
| door: the cavity's 2 cuts, 4 bosses, 4 insert holes, 10 lips and ribs, each group as 1 | 1,455 (414) | 168 | −82% | yes |
| body: its 15 cuts as 1 | 297 | 123 | −69% | yes |
| clamp, housing clamp: their holes as 1 | 46, 107 | 22, 57 | −49%, −47% | yes |
| each sled: its boxes as 1 fuse | 93, 102 | 30, 35 | −59%, −53% | yes |
| body bumper: its cuts as 1 | 207 | 158 | −27% | yes |
| boot: its thick middle and ribs as 1 fuse | 190 | 137 | −45% | yes |

Together the parts go from about 4,280 MB to about 1,770 on 0.4.0, and from about 2,990 to about 1,530 (−49%) on Kerf's main, where the housing goes 1,089 to 357 MB, the door 415 to 168 and the body 293 to 121: the largest saving available. Order still matters
where a later step must see an earlier one (the door's bosses before their insert holes): group only steps that don't
touch each other's results, as above. Kerf's unit 14C has the code of each, if the agent wants it.

## 2. The boot's print file: a compound, not a fuse

`PrintPose("boot", ...) + PrintPose("boot_tower", ...)` fuses two solids that never touch, only to write one STL. A
compound (`new Part([boot.Shape, tower.Shape])`) writes the same STL, byte for byte, and skips the Boolean: 153 MB and
about 0.2 s less. The twin already uses a `Compound`.

## 3. Publishing: don't cut what the slice misses

The section renders cut every part and component (47) by each of the 5 slices: 235 Booleans, 900 MB. 166 of them miss
the slice entirely (their bounding boxes don't meet it). Skipping those (compare `Bounds()` first) gives the same
pictures for 716 MB, 20% less. Render's own `Volume > 0` filter already drops the empty ones afterwards.

## 4. The housing's ramp: points every 0.25 mm make 237 faces

`Ring` samples the sections' arcs every 0.25 mm, and the ramp's hull has a facet between each pair, so the housing has
342 faces and every later Boolean on it pays for them (item 1 is the bigger lever). Sampling every 1 mm gives 202 faces
and takes the housing and its envelope from 1,472 to 854 MB, but it is **not the same part**: the volume moves by 8e-5
(2.2 mm³), within the reference tests but a change of shape. Every 0.5 mm: 247 faces, 1,058 MB, 2.5e-5. A design
choice for the owner, not a free saving.

## 5. The spec tests' point checks

`Housing_nuts_drop_in_from_the_top_onto_a_ledge` and `Screw_heads_can_be_reached_from_the_arm_side` classify 224 points
against the housing for each size, about 10 MB and several milliseconds each, as Kerf builds a classifier for every
`IsInside`. Fewer points (the shaft every 4 mm rather than every 1) would do the same check for a quarter of that; Kerf
may make repeated points cheap later (decision 27105).

## What's not worth changing

Building per size, the cached sections and `PerSize` are right: nothing is built twice. The STL deflection of 0.01 is
coarser than build123d's default, and the exports are a small share (STEP and STL about 13% of the build's bytes). The
`ConvexPolyhedron` ramp and the collapsing chamfer are each well under 1 s.
