# Unit 13F: DXF export, for Bento

**Nothing in Bento has to change:** `ExportDxf` is new, and nothing else moved. Bento takes Kerf as packages, so it
reaches Bento with the first release after 0.4.0. Not done here: Bento is read only to this unit.

**What an export to DXF would need, for the owner to decide.** Two functions, as Bento has for SVG and STEP:

- **`CAD_IO_EXPORT_DXF(shape, path, [options])`**, for a sketch, a face, a part's edges or a list of them, to cut or
  engrave: one more `Export(r, …, "DXF", …)` in `Extensions/Bento.Cad/IoFunctions.cs`, beside `CAD_IO_EXPORT_STEP`,
  whose writer is `(shape, options, full, stream, cancel) => new ExportDxf(version, Unit(options, "unit"), …)
  .AddShape(shape).Write(stream, cancel)`. Options `{unit, version: "2013" or "R12", layer, color, lineWeight,
  lineType, tolerance}`, read as the others are, with tables for `ColorIndex` and `LineType` beside `ModelNames`
  (their names are build123d's: `"red"`, `"hidden"`, `"iso_dash"`). A model whose parts have names could put each part
  on a layer of its name; that's a choice, not something Kerf decides.
- **`CAD_IO_EXPORT_DXF(drawing, path)`** for a line drawing from `CAD_LINE_DRAWING`, beside `CAD_IO_EXPORT_SVG`
  (`Extensions/Bento.Cad/PictureFunctions.cs`). Bento's `CadDrawing` keeps the drawing's SVG text and its counts, not
  the `LineDrawing`; it would keep the `LineDrawing` too (it's immutable, so it can be shared), and the export would be
  `new ExportDxf().AddDrawing(drawing.Lines).WriteToString()` through `IoFunctions.RequestWrite`, as the SVG is. The
  DXF is full size in model units, where the SVG's page options (margin, scale, page size) don't apply.

`Write` returns a `Result` as the 3D exporters do; its one warning, `modeling.dxf-not-planar`, says edges off the XY
plane were projected onto it (a part's edges seen from above), which Bento could show as it shows other warnings.

Decisions 2009 to 2012 in the kernel's `docs/decisions.md`; the guide's section 8, "DXF".
