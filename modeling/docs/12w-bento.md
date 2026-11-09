# Unit 12W: chamfers past tight corners, for Bento

**Nothing in Bento has to change:** the option is off by default, and a chamfer without it is what it was. Bento takes
Kerf as packages (`KerfVersion` in `Extensions/Bento.Cad/Bento.Cad.csproj`), so nothing here reaches it until a Kerf
release after 0.3.0.

**To offer it, for the owner to decide:** `CAD_PT_CHAMFER` (`Extensions/Bento.Cad/ModelFunctions.cs`) takes options
`{length2, angle}`. One more, `tight_corners`, read as the other enums are (`options.Enum("tight_corners",
TightCorners.Refuse)`, with a table `"refuse"`, `"collapse"` beside `ModelNames.Kind`), passed as
`Chamfer(…, options: new ChamferOptions { TightCorners = … })`, and a line in the function's description and in
`docs/3d-models.html`. Not done here: Bento is read only to this unit.

| Cell | Now | With the option, from the next Kerf |
|---|---|---|
| A 40 by 30 by 30 box with 1.7 corners, its top loop chamfered 3 | `#VALUE!`: invalid (unit 12V: refused, saying the most that fits) | `{tight_corners: "collapse"}`: volume 35336.538, the corners collapsed to points |
| The same at 1.69 | 35734.872 | the same, with the option or without |

Decisions 1988 to 1991 in the kernel's `docs/decisions.md`; the geometry in its `docs/chamfer-corners.md`.
