# Unit 12C: fillet limits, for Bento

**Nothing has to change for Bento to build.** `ModelFunctions.Run` already turns the failure into `#VALUE!` with its message. A few cells show something else (decisions 1898 to 1900 in the kernel's `docs/decisions.md`):

| Cell | Before | Now |
|---|---|---|
| A fillet past a limit, where the engine made something invalid (all of a box's edges, past half its side) | `#VALUE!` with `result.invalid`, and the shape carried | `#VALUE!` with `modeling.fillet-failed`: "The fillet with a radius of 6 is too large for these 12 edges: the most that fits is 5, ... It failed on 12 of 12 edges (too large: ...)". No shape carried, as none was made |
| A fillet past a limit otherwise | `#VALUE!`, after what could be seconds | The same message, in under 2 ms |
| A fillet just below a limit, by under the shape's tolerance (1e-7 on most) | A shape with faces that narrow | The limit's own result, with those faces gone: 5 faces where it was 7. On corners and slanted ends, `#VALUE!` saying it can't be built yet |
| `MaxFillet` | | The same number, with fewer tries |

**To check by hand:** search Bento's samples for cells that test for `result.invalid` from a fillet. There shouldn't be any, as the shape in that failure was of no use to a cell. A cell recalculating past a limit should feel no slower, which is the only thing a user could notice otherwise.
