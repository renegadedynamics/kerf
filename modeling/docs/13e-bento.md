# Unit 13E: text, for Bento

**Nothing in Bento has to change:** Kerf.Modeling's `Text` is new, and Bento takes Kerf as packages, so nothing here
reaches it until a Kerf release after 0.4.0. The kernel's THIRD-PARTY-NOTICES.md, which Bento already copies as
`Kerf.Kernel-THIRD-PARTY-NOTICES.md`, now carries the licence of the font Kerf ships, Liberation Sans (SIL Open Font
License 1.1); Bento's own notices need only say that the Kerf packages include it.

**What a `CAD_TEXT` would need, for the owner to decide** (Bento was only read for this unit):

- **The function:** `CAD_SK_TEXT(text, size, [options])`, beside `CAD_SK_RECTANGLE` in
  `Extensions/Bento.Cad/ModelFunctions.cs`, returning `new Text(…)`: options `{font, style, textAlign, align, rotation,
  path, positionOnPath, singleLineWidth}`, with `style` read as the other enums are (`"regular"`, `"bold"`, `"italic"`,
  `"bold_italic"` to `Kerf.Fonts.FontStyle`) and `textAlign` as a pair of names (`"left"`, `"center"`, `"right"`;
  `"bottom"`, `"center"`, `"top"`, `"top_first_line"`) to `(TextAlign, TextAlign)`, `path` a curve cell. Its faces
  extrude as any sketch's do, so embossing and engraving need nothing more.
- **Failures as cell errors:** use `Operations.MakeText(…)`, whose failure has a code (`text.missing-glyph`, naming
  the characters, `text.empty`, `input.invalid` for a size of 0, `font.malformed` for a damaged font file), as the
  other `Make…` operations' do; and its warning `font.substituted` when the font named isn't installed, which a cell
  should show, since the text's widths then come from Liberation Sans.
- **Fonts and reproducibility:** a model that names a font draws differently on a machine without it. Liberation Sans
  is in the packages, so `font: "Liberation Sans"` is the same everywhere; Bento's documentation could say so, and
  could default `CAD_SK_TEXT`'s font to it rather than build123d's Arial. In WebAssembly there are no system fonts:
  `FontLibrary.System` is empty and every family is Liberation Sans, with the warning.
- **Fonts from cells:** a font file the user brings could be a cell's bytes, read with `Font.Read(stream)` and made a
  library with `FontLibrary.FromFonts(…)`; `Text` takes a path today, so this would want an overload taking a `Font`,
  a small addition to the kernel's `TextShapes.Make`, which already takes one.
- **Caching and recalculation:** a `Font` is immutable and safe to share between threads; the default fonts and the
  system's list are read once a process (the list the first time a font is asked for, reading only each file's
  names: 70 ms for this machine's 353 font files, its cache warm). The same text, size and font give the same shape, so a cell's
  value can be cached on its arguments; a font installed after Bento starts isn't seen until it restarts.
- **Cancellation:** `TextShapes.Make` obeys the cancellation in force, as the operations do.

Decisions 2018 to 2024 in the kernel's `docs/decisions.md`; the kernel's `docs/api/fonts.md`.
