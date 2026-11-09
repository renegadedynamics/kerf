# Unit 13F2: notes and drawing sheets, for Bento

**Nothing in Bento has to change:** notes and sheets are new, and a drawing without notes is written as before, by
`Svg` and by `ExportDxf`, to the byte. Kerf.Modeling's unused `PageSize` enum has gone, to `Kerf.Drawing.PageSize`;
Bento (only read for this unit) names no `PageSize` of Kerf's: its `PrintLayout.PageSize` is its own.

**What notes and sheets would need, for the owner to decide:**

- **Notes on a drawing:** `CAD_LINE_DRAWING` could take notes as an option, `{notes: [[text, x, y, size], …]}`, made
  with `DrawingNote.Make(text, new Point2D(x, y), new NoteStyle { Size, Horizontal, Vertical, Rotation })` and added
  with `drawing.WithNotes(…)`. `Make` returns a `Result`: a character the font hasn't fails as `text.missing-glyph`,
  naming it, which a cell shows as its error. Bento's `CadDrawing` keeps the SVG text; written with notes it needs
  nothing more.
- **Outlines or text:** `SvgOptions.TextOutput` and `ExportDxf(textOutput: …)`, a cell option `{text: "outlines"}` or
  `"text"`. Outlines are the default and look the same everywhere, which suits a spreadsheet whose pictures should not
  depend on the viewer's fonts; text is smaller and editable in another CAD program.
- **A sheet:** `CAD_SHEET(size, views, [titleBlock])`, `size` a name read as the other enums are (`"a4"`, `"a3"`,
  `"letter"`, `"ansi_c"`, with `"portrait"`), `views` a list of `[drawing, x, y, scale]`, `titleBlock`
  `{title, partNumber, scale, units, date, author, sheet}`. `new Sheet(…) { TitleBlock, Views }.Draw()` gives one
  `LineDrawing`, so it goes to `CAD_IO_EXPORT_SVG` with `sheet.PageOptions()` for a page of the paper's size, and to the
  DXF export of 13f-bento.md unchanged. Its warnings (`drawing.sheet-overflow`: a view off the paper or over the title
  block, an entry too long for its cell) are worth showing as the other warnings are.
- **The date:** a sheet doesn't read the clock, so the same cells give the same file; a `TODAY()` cell feeds the date.
- **Caching:** a note and a sheet are immutable and safe to share between threads; a note draws its glyphs once, when
  it's made, and a sheet's `Draw` makes its title block's notes again each time, so a cell can cache its value on its
  arguments.

Decisions 2030 to 2033 in the kernel's `docs/decisions.md`; the kernel's `docs/api/drawing.md` ("Notes", "Sheets");
the guide's section 8, "Drawing sheets".
