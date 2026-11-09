# Kerf.Drawing: line drawings, SVG, pictures and PNG

*Unit 10L4, 4 November 2026: decisions 1552 to 1557 (placeholders). Decision 1349, question 64: the owner wants
Kerf, and later Bento, to make both SVG line drawings and PNG shaded pictures.*

`Kerf.Drawing` is in ModelingAlgorithms (its model loader, `ModelNode.ToScene`, is in DataExchange). It has two halves
that share one camera:

| Type | What |
|---|---|
| `Camera`, `Projection` | A view: eye, target, up; orthographic or perspective. `Top`, `Front`, `Right`, `Isometric`, `LookAt`, `Along` |
| `DrawingOperations` | `ProjectLines`: hidden-line removal of shapes, giving a `LineDrawing` |
| `LineDrawing`, `DrawnEdge`, `LineKind`, `DrawingOptions`, `HiddenLineMethod` | The drawing: 2D curves, visible or hidden, by kind, with their sources |
| `DrawingNote`, `NoteStyle`, `NoteLine`, `TextOutput` | Notes: text on a drawing, set in a font, written as outlines or as text (unit 13F2) |
| `Sheet`, `SheetView`, `TitleBlock`, `PageSize`, `PageOrientation` | A drawing sheet: paper, border, title block and views (unit 13F2) |
| `Svg`, `SvgOptions`, `LineStyle`, `SvgUnit` | The SVG writer |
| `Renderer`, `Scene`, `SceneItem`, `RenderOptions`, `Shading`, `EdgeDisplay`, `DirectionalLight` | The software renderer of meshes |
| `RgbaImage` | A picture, 8-bit RGBA |
| `Png`, `PngOptions`, `PngCompression` | The PNG writer |

## The camera

The view's frame is OCCT's projector's (HLRAlgo_Projector): origin the target, Z from the target towards the eye, X to
the right, Y up (the up direction made perpendicular). Orthographic projection keeps (X, Y); perspective divides by
1 − Z/d, d being the eye's distance from the target, so the target's plane keeps its scale. The same arithmetic serves
the drawings and the pictures, so a drawing and a picture from one camera line up. `camera.Project(point)` gives a
point's 2D position. build123d's `project_to_viewport(viewport_origin, viewport_up, look_at)` is
`Camera.LookAt(viewport_origin, look_at, viewport_up)`.

An eye at its target, or an up direction along the view, is misuse: `ArgumentException`.

## Line drawings

```csharp
Result<LineDrawing> drawing = part.ProjectLines(Camera.Isometric);                    // or DrawingOperations.ProjectLines(part, …)
Result<LineDrawing> both = DrawingOperations.ProjectLines([part, fixture], Camera.Front, // several shapes hide each other
    new DrawingOptions { Method = HiddenLineMethod.Polygonal, IncludeHidden = false });
foreach (DrawnEdge edge in drawing.Value.Visible) { /* edge.Curve, edge.Kind, edge.Source */ }
```

- **Two algorithms,** as OCCT: `Exact` (HLRBRep_Algo, the default) is slower and gives true curves; `Polygonal`
  (HLRBRep_PolyAlgo) hides the edges of a mesh (made in the mesher's overlay, the shape untouched) and gives polylines.
  The polygonal one is orthographic only: OCCT's results conversion ignores the perspective.
- **Kinds** (OCCT's `HLRBRep_TypeOfResultingEdge`): `Sharp`, `Smooth` (G1 between faces), `Sewn` (CN inside a surface,
  such as a seam), `Outline` (silhouettes), `IsoLine` (`IsoLineCount` per face direction, exact only). OCCT's rules are
  kept: an edge lying on an outline is drawn as sharp.
- **Curves** are public 2D curves, bounded: `TrimmedCurve2D` of `LineCurve2D`, `CircleCurve2D` or `EllipseCurve2D`,
  `BSplineCurve2D` and `BezierCurve2D` where OCCT projects exactly (orthographic conics and splines), and degree-1
  B-splines (polylines) within `Tolerance` otherwise (a circle seen edge on or in perspective). OCCT's own edge builder
  uses 15 points for those; this samples until within the tolerance.
- **Sources:** each line's `Source` is the caller's edge it is part of, or the face an outline or isoline is drawn on.
  The outliner splits edges and rebuilds faces; its records (`NewSOldS`) map them back, so in practice every line has
  one. A null source means the engine didn't know.
- **Order:** by kind (sharp, smooth, sewn, outline, isoline), visible before hidden, then the engine's. The same input
  gives the same drawing, on any thread.
- **Results:** an engine failure is `drawing.failed` (or the engine's code); a polygonal mesh missing faces gives the
  warning `drawing.faces-not-meshed`. Nothing to draw is an empty drawing. Cancellation throws.
- `new LineDrawing(camera, edges)` makes or completes a drawing by hand (title blocks, dimensions, another source).

## SVG

```csharp
string svg = Svg.WriteToString(drawing, new SvgOptions { PageWidth = 297, PageHeight = 210, Margin = 10 });
Result written = Svg.Write(drawing, stream);
```

- Two layers: `<g id="hidden">` (drawn first, under) and `<g id="visible">`, each with its stroke (colour as
  `Kerf.Color`, width, dashes); hidden lines dashed grey by default, or left out (`IncludeHidden = false`). One
  `<path>` per edge, its kind as its `class` (`sharp`, `outline`…) for styling.
- Lines are `L`; arcs of circles and ellipses `A` (a whole circle two arcs); polynomial B-splines and Bézier curves of
  degree 1 to 3 Bézier segments (`L`, `Q`, `C`: the spline's interior knots inserted to the degree's multiplicity);
  anything else (rational curves, higher degrees, offsets) a polyline within `Tolerance` page units.
- **Page:** sizes in `Unit`s (mm by default; cm, in, pt, px). With no page size and no scale, full size (1 page unit a
  model unit) on a page fitting the drawing and margins; with a page size, scaled to fit inside the margins; with a
  `Scale`, at that scale. Always centred. The view's Y up becomes SVG's Y down.
- **Bytes:** UTF-8 without a byte order mark, `\n` line ends, numbers rounded to `Decimals` (3) under the invariant
  culture, no negative zero: the same drawing and options give the same bytes on every machine.
- **Notes** are a third layer, `<g id="notes">`, over the lines: each note a `<path class="note">` of its glyphs'
  outlines, one closed subpath a contour, filled in `NoteColor` by the non-zero rule; or, with `TextOutput.Text`, a
  `<text>` a line, at its anchor, in the font's family (Liberation Sans with Arial, which has its widths, after it),
  size, weight and style, `text-anchor` for its alignment, `rotate` for its rotation and `xml:space="preserve"`.
- **`Area`** fixes the part of the view plane the page shows in place of the drawing's extent: a sheet's page.
- A stream that can't be written is `svg.write-failed`.
- `await Svg.WriteAsync(drawing, stream, options, token)` writes the same bytes asynchronously: the SVG made in memory
  on the calling thread, then written and the stream flushed without blocking a thread (unit 14G, exchange.md).

## Notes

*Unit 13F2, 9 November 2026: decisions 2030 to 2033 (placeholders). Decision 2008: text in SVG and DXF as outlines by
default, as text as an option.*

```csharp
Result<DrawingNote> note = DrawingNote.Make("Ø8 through", new Point2D(20, 15),
    new NoteStyle { Size = 3.5, Horizontal = TextAlign.Center, Rotation = 90 });   // Kerf.Fonts' TextAlign
LineDrawing annotated = drawing.WithNotes(note.Value);
string svg = Svg.WriteToString(annotated, new SvgOptions { TextOutput = TextOutput.Text });
```

- A note is text at a point of the view plane: `Size` the font's em, as `TextShapes.Make` and build123d's `Text` size it
  (a capital is about 0.7 of it), `Font` (Liberation Sans by default), `Horizontal` and `Vertical` alignment on its
  position as `TextOptions` has them, and `Rotation` in degrees about it. `Make` draws it once with `TextShapes.Make`
  and fails as that does (`text.empty`, `text.missing-glyph`, `result.invalid`).
- **`Contours`** are its outlines, placed: closed loops of lines and Bézier curves, outer contours counter-clockwise and
  holes clockwise. **`Lines`** are its lines for writing as text: each line's characters (a tab as the spaces it
  advances) and its anchor, the point on its baseline where the alignment puts the line's start, centre or end, so a
  reader's own text lands where the outlines do. **`CapHeight`** is the height of the font's H, which DXF's TEXT takes.
- **In DXF** (Kerf.Modeling's `ExportDxf`), outlines are each contour as a closed polyline within the exporter's
  tolerance; in DXF 2013 they are also filled, as the SVG fills them: a solid `HATCH` for each outer contour, its
  boundary that contour's polyline and the holes in it, under the contours, in the notes' layer and its colour
  (`fillOutlines: false` leaves the fill out). R12 has no `HATCH`, so its notes are their contours alone (unit 13N).
- A drawing keeps its notes (`LineDrawing.Notes`, `WithNotes`, or the constructor), and `GetBounds` counts them, so a
  page fitted to the drawing fits its notes too. A drawing without notes is written as before.

## Sheets

```csharp
var sheet = new Sheet(PageSize.A3)            // landscape; PageOrientation.Portrait
{
    TitleBlock = new TitleBlock { Title = "Bracket", PartNumber = "KF-0042", Date = "2026-11-09", Author = "J. Davies" },
    Views = [new SheetView(part.ProjectLines(Camera.Front).Value, new Point2D(120, 180), 0.5)],   // 1:2, centred there
};
Result<LineDrawing> drawn = sheet.Draw();
string svg = Svg.WriteToString(drawn.Value, sheet.PageOptions());
```

- **Paper:** ISO A0 to A10 and US letter, legal and ledger (build123d's `PageSize`), with ANSI C, D and E; `AnsiA` is
  `Letter` and `AnsiB` `Ledger`. Either way up.
- **Plain, not a drafting standard:** a border 20 mm from the left edge, for filing, as ISO 5457 has it, and 10 mm from
  the others (less in proportion on paper smaller than A4); a title block in the frame's bottom right corner, 180 × 25 mm (smaller in proportion where the frame is
  narrower), of a title and part number above and scale, units, date, author and sheet below, each cell its label and the
  caller's text. The scale is the views' (`1:2`, `2:1`), "As shown" where they differ, or what `TitleBlock.Scale` says.
- **Views:** each a line drawing, the centre of its extent placed at a point of the paper, drawn at a scale (paper
  millimetres to the drawing's unit); its curves are transformed, its notes made again at the scale.
- **`Draw`** gives the sheet as one `LineDrawing` in millimetres from the paper's bottom left corner, so it is written
  as any drawing is: `Svg` on a page of the paper's size with `PageOptions` (or any `SvgOptions` with `PageOptions(style)`),
  Kerf.Modeling's `ExportDxf.AddDrawing` as it is. A view outside the border or over the title block, and an entry wider
  than its cell, are warnings (`drawing.sheet-overflow`); an entry the default font can't set fails.

## Pictures

```csharp
Result<RgbaImage> image = part.Render(Camera.Isometric);                 // meshed at a thousandth of its size
Result<RgbaImage> picture = Renderer.Render(new Scene(
    new SceneItem(mesh, new Color(0.2, 0.4, 0.8)).WithFaceColor(top, new Color(1, 0, 0)),
    new SceneItem(other) { Transform = Transform3D.Translation(new Vector3D(50, 0, 0)) }),
    Camera.LookAt(eye, target, projection: Projection.Perspective),
    new RenderOptions { Width = 1024, Height = 768, Background = new Color(1, 1, 1, 0) });
Result<Scene> scene = model.ToScene();                                   // a ModelNode's parts, placed and coloured
byte[] png = Png.Encode(picture.Value);
```

- **The renderer** is a software rasteriser over `ShapeMesh`, in managed code: no native or GPU dependency. It projects
  every node with the camera, fits the scene in the picture inside `Margin` (or uses `Scale`, pixels a unit, centred
  on the target), and rasterises triangles with a depth buffer (nearer wins, whatever the order), into a buffer
  `Supersampling`² times larger (2 × 2 by default), averaged down with the colours weighted by opacity.
- **Shading:** `Smooth` interpolates the mesh's normals; `Flat` uses each triangle's. The light is `Ambient` plus each
  `DirectionalLight`'s intensity times the cosine, the lights given in the view's frame so they follow the camera; the
  colour is the face's sRGB colour times the light, clamped. A triangle seen from behind is lit as if it faced the eye.
- **See-through:** a colour's opacity under 1 (`new Color(r, g, b, a)`) makes those faces, and the item's edges,
  see-through. Each sample keeps the see-through faces and edges in front of the nearest opaque face, sorts them by
  depth and lays them over it from the back, so the order of the items doesn't matter and a see-through solid shows
  its far side; shared edges of a see-through face's triangles are laid once (the top-left rule). A scene with nothing
  see-through is drawn exactly as before (decision 1998).
- **Colours:** `SceneItem.Color` (light grey by default), `FaceColors` by face index (`WithFaceColor(face, colour)` finds
  the index), and a model's colours inherited down its tree (`ModelNode.ToScene`).
- **Edges** are drawn over the faces with a depth test (a small bias, so edges on a face show and edges behind it
  don't). `Feature` (the default) draws the shape's edges where its faces' normals meet at more than `FeatureAngle`
  (20°), and free edges: not seams nor tangent edges. It reads the normals at each edge's points from the faces that
  have a node at all of them (a seam: one face, twice). `All` draws every edge; `None` none. Meshes read from files
  have no edges.
- **Determinism:** IEEE double arithmetic with no library function but `Math.Sqrt`, and bands of 16 rows rasterised in
  parallel, each owning its pixels: the same pixels on every machine, serial or parallel.
- Points at or behind a perspective eye are left out with their triangles; there is no near plane beyond that.
- `render.failed` is a scene whose coordinates aren't finite; the meshing's failures come through `part.Render` and
  `ToScene`. An empty scene is the background.

## PNG

`Png.Encode(image)` and `Png.Write(image, stream)`: 8-bit RGBA, not interlaced, each row's filter (none, sub, up,
average, Paeth) chosen by the smallest sum of magnitudes, CRC-32 per chunk, IHDR, one IDAT, IEND. The deflate is Kerf's
own by default (`PngCompression.Deterministic`: one block of fixed Huffman codes, greedy LZ77 on a hash chain of 32,
Adler-32): .NET's `ZLibStream` uses the platform's zlib (zlib-ng), whose bytes aren't promised to be the same on every
machine and version. `PngCompression.Smallest` uses it anyway, for smaller files. Rendered parts compress to 20 to 50 KB
at 800 × 600 either way. `Png.WriteAsync(image, stream, options, token)` encodes on the calling thread and writes the
same bytes asynchronously (unit 14G).

## Measured

Bento's examples (`Kerf.Kernel.Bento.Benchmarks -- images 5`, Release, a machine shared with other units, median of
the passes after the first): the exact drawing, isometric, and its SVG take 2.4 to 22 ms (the bottle 86 ms, 176 edges;
0.4 to 14 MB allocated); meshing, rendering at 800 × 600 supersampled 2 × 2 and encoding the PNG take 38 to 93 ms
(rendering alone 16 to 65 ms; 22 to 51 MB allocated, most of it the supersampled colour and depth buffers, 15 MB).

## Left

- Dimensions, leaders and section views on a sheet; a title block of a standard's layout (ISO 7200's fields).
- Section views (a cut by a plane, hatched).
- Feature edges for meshes without a shape (STL): crease detection from the triangles.
- Shadows, and anti-aliasing other than supersampling.
- Bento's spreadsheet functions making both (10N).
