# Kerf.Fonts: fonts and text

*Unit 13D, 9 November 2026; decisions 2018 to 2023.*

OCCT reads fonts through FreeType (`Font_FTFont`, `Font_FontMgr`) and draws text as shapes with `StdPrs_BRepFont`,
`Font_TextFormatter` and `StdPrs_BRepTextBuilder`, in its Visualization module, which Kerf doesn't port. Kerf reads fonts
with a reader of its own, managed and bounds-checked, and draws text with a port of those three classes. They are in
`Kerf.Kernel.ModelingAlgorithms`, the lowest assembly that makes faces, so `Kerf.Drawing` can set notes and title
blocks (13F2) and the kernel's users can draw text without Kerf.Modeling.

| Type | What |
|---|---|
| `Font` | A TrueType or OpenType font: `Read`, `ReadCollection` (`.ttc`) and their async forms `ReadAsync`, `ReadCollectionAsync` (the stream read into memory asynchronously, then read; unit 14G), its names, style, weight and metrics, `GlyphOf(codePoint)`, `AdvanceWidth(glyph)`, `Kerning(left, right)`, `Outline(glyph)`. Immutable; reads its bytes for each request |
| `GlyphOutline`, `OutlineSegment`, `OutlineSegmentKind` | A glyph's contours of lines, quadratics (TrueType) and cubics (CFF), in font units, as FreeType decomposes them |
| `FontReadLimits` | What the reader refuses: bytes, fonts in a collection, points to a glyph, components' depth, charstring operations |
| `FontLibrary`, `FontEntry` | Fonts found by family and style: `System` (the machine's and the user's folders), `Scan(folders)`, `FromFonts(fonts)`, and `Default(style)`, Liberation Sans, which Kerf ships; `Find` puts Liberation Sans in place of what it hasn't, with a `font.substituted` warning |
| `FontStyle` | Regular, Bold, Italic, BoldItalic (OCCT's `Font_FontAspect`, build123d's `FontStyle`) |
| `TextShapes`, `TextOptions`, `TextAlign` | `TextShapes.Make(font, text, size, options)`: a compound of a face or faces per character, in the XY plane facing +Z, holes as holes; lines, tabs, kerning, alignment as OCCT's formatter gives them; a slant for an italic a font hasn't; single-stroke fonts as open wires |

## What is read

| Table | What |
|---|---|
| `cmap` | Formats 0, 4, 6, 12 and 13; the Unicode map FreeType chooses, and a symbol font's at U+F000 |
| `head`, `hhea`, `hmtx`, `maxp`, `OS/2`, `post`, `name` | Units per em, FreeType's face metrics (OS/2's typographic ones where `USE_TYPO_METRICS` says), advances and bearings, the glyph count, weight and style, italic angle, the names (IDs 1, 2, 4, 6, 16, 17) |
| `loca`, `glyf` | Quadratic outlines; composites scaled (one scale, x and y, 2×2), with scaled offsets where flagged, matched points, `USE_MY_METRICS`, nested |
| `CFF ` | Type 2 charstrings: every path operator, flex, hint masks read past, the arithmetic operators, local and global subroutines; CID-keyed fonts through FDSelect (formats 0 and 3); a font matrix |
| `GPOS`, `kern` | Pair kerning: GPOS's pair adjustments (formats 1 and 2, in extension lookups too) of the `kern` features, else the kern table's format 0 pairs |
| `.ttc`, `.otc` | Collections |

Not read: WOFF and WOFF2, CFF2 and variable fonts, bitmap-only fonts, Type 1, CFF's `seac` accents, vertical metrics, and
GSUB (text is set glyph by glyph from the character map, as OCCT sets it).

## Diagnostics

| Code | Meaning |
|---|---|
| `font.malformed` | Not a font, or damaged past reading: an offset or count past its table, a subroutine nested past 10 |
| `font.limit-exceeded` | Past a `FontReadLimits` limit |
| `font.unsupported` | WOFF, CFF2, Type 1, bitmaps only, a `seac` accent |
| `font.substituted` | A warning: the family, style or file wasn't found or couldn't be read, and another is used |
| `text.missing-glyph` | A character the font has no glyph for, named with its code point |
| `text.empty` | Nothing to draw |
