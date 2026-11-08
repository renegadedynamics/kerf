# Unit 12T: 2D offsets' accuracy and kinds, for Bento

**Bento's change is made, on its branch `kerf/offset-kinds`** (from `kerf/load-from-files`, for the owner to merge): the
kinds table (`ModelNames.Kind`) no longer has `"tangent"`, nor does `CAD_SK_OFFSET`'s description or its line in
`Extensions/Bento.Cad/docs/3d-models.html`. `{kind: "tangent"}`, in `CAD_SK_OFFSET` or `CAD_PT_OFFSET`, now gives the
usual error for a word not in a table: `kind: tangent is not one of "arc", "intersection".` Bento.Cad's tests pass (95,
one more check in `ModelFunctionTests`).

**What Bento needs to see Kerf's change:** Bento takes Kerf as packages (`KerfVersion` 0.3.0 in
`Extensions/Bento.Cad/Bento.Cad.csproj`), so nothing here reaches it until a Kerf release after 0.3.0 and that property
raised. The branch builds against 0.3.0 as it is, and against the next release, since it no longer names `Kind.Tangent`.

| Cell | With 0.3.0 | With the next Kerf |
|---|---|---|
| `=CAD_SK_OFFSET(CAD_SK_ELLIPSE(10, 5), -3)` (an ellipse or spline inwards past its sharpest bend) | up to 1% of the distance too far in, silently: area 39.556 | within 0.1% of the distance: 40.004 (the true area is 40.035) |
| A closed spline offset inwards to where it just closes up, when the kernel gives an open piece | `#VALUE!` from an exception thrown out of `Offset` | `#VALUE!`, `modeling.offset-failed`: "the offset of a closed curve came out open" |
| `{kind: "tangent"}` | `#VALUE!` on the branch already (the table) | the same |

Decisions 23601 to 23604 in the kernel's `docs/decisions.md`.
