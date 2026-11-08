# Unit 12V: chamfer limits, for Bento

**Nothing has to change for Bento to build.** `ModelFunctions.Run` already turns the failure into `#VALUE!` with its message. A few cells show something else (decision 1985 in the kernel's `docs/decisions.md`):

| Cell | Before | Now |
|---|---|---|
| A chamfer longer than the tightest convex arc or ellipse in a face beside its edges, such as a rounded outline's corners | `#VALUE!` with `result.invalid` ("Not closed ..."), and the shape carried | `#VALUE!` with `modeling.chamfer-failed`: "The chamfer with a length of 1.71 is too large for these 8 edges: the most that fits is 1.7, the radius of the tightest convex arc in a face beside them, where its edge on that face closes to a point." No shape carried, as none was made |
| The same by a length and an angle | Sometimes a shape, valid to the check but wrong | `#VALUE!` with the same message |
| A chamfer exactly at such a radius | `#VALUE!` with the engine's failure | `#VALUE!` saying the edge closes to a point there: try a slightly smaller length |

**New:** `MaxChamfer(edges)`, beside `MaxFillet`, if Bento wants a cell for it; nothing uses it yet.

**To check by hand:** search Bento's samples for a chamfer by a length and an angle on a rounded outline whose length on the outline's face is more than its corners' radius. Such a cell gave a shape before and gives `#VALUE!` now; the shape was wrong.
