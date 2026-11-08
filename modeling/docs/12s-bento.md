# Unit 12S: 2D offsets, for Bento

**Nothing has to change for Bento to build.** `CAD_SK_OFFSET` calls `Operations.Offset`, whose failures `ModelFunctions.Run`
already shows as `#VALUE!` with the message. Some cells show something else (decisions 1956 and 1957 in the kernel's
`docs/decisions.md`; the survey is the kernel's `docs/offset2d-survey.md`):

| Cell | Before | Now |
|---|---|---|
| `=CAD_SK_OFFSET(CAD_SK_CIRCLE(5), -6)`, or any sketch offset inwards past its size | An `InvalidOperationException`, "Unexpected result type", thrown out of an operation that returns a result | `#VALUE!` with `modeling.offset-failed`: "The offset by -6 in the plane leaves nothing: the shape closes up at that distance." (for a closed curve, "the curve closes up") |
| A washer, or a plate with holes, offset inwards until nothing is left | An empty sketch, as a success | `#VALUE!`: "... leaves nothing: the shape closes up at that distance." |
| A shape with a waist (two discs joined by a bar) offset inwards past the waist | The exception above | The pieces, one face each |
| A sketch of several faces, one of which closes up | The exception above | The rest, with a warning: "1 of the 2 faces close up when offset by -1 and are left out." |
| A hole whose offset fails for another reason | The face without that hole, as a success | `#VALUE!` with the offset's failure |
| `{kind: "tangent"}` outwards on a triangle or a star | The exception | `#VALUE!`: "... failed: it made nothing." |

**For Bento's description of `CAD_SK_OFFSET` (inferred from the survey, for the owner to decide):** `"tangent"` isn't a
2D join OCCT supports; it gives mitres' results almost everywhere and nothing at sharp corners outwards. Decision 1960
asks whether Kerf should map it to `"intersection"`; until then the description could say to use `"intersection"`.

**What Bento users may still meet:** an offset inwards past a curve's sharpest bend (an ellipse or a spline, inwards by
more than its tightest radius) is up to 1% of the distance off, with no warning (decision 1959, a question for the owner).
