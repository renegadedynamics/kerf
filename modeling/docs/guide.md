# Kerf.Modeling: a user guide

Kerf.Modeling is a .NET 10 CAD modelling library: build123d's API, in C#, over Kerf's kernel (a managed port of
Open CASCADE). This guide gets you from nothing to a part, a mesh and a STEP file, and covers what most often
trips people up. If you know build123d, [name-map.md](name-map.md) lists every name that changed.

Every example here was compiled and run against Kerf 0.4.0, and its volumes checked by hand.

## 1. Setting up

You need the .NET 10 SDK (`dotnet --version` shows 10.0 or later). The quickest start is a single C# file, run with
`dotnet run`: no project file needed.

```csharp
#:package Kerf.Modeling@0.4.0
#:property PublishAot=false
using Kerf;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Modeling;
using static Kerf.Modeling.Operations;

Part plate = new Box(80, 60, 10);
plate -= new GridLocations(60, 40, 2, 2) * new Cylinder(5, 10);
Console.WriteLine($"Volume {plate.Volume:F3}");          // 44858.407
Exporters3D.ExportStep(plate, "plate.step");
```

Save it as `plate.cs` and run `dotnet run plate.cs`. The first run restores the packages and takes a while; later
runs are quick.

- `#:property PublishAot=false` stops a file-based app from pulling in the native-compilation package, which isn't
  needed and gives a NuGet warning.
- In a project instead: `dotnet add package Kerf.Modeling --version 0.4.0`. It brings the four kernel packages with it.
- The namespaces: `Kerf` (`Result<T>`, `Diagnostic`), `Kerf.Geometry` (`Point3D`, `Vector3D`, `Direction3D`),
  `Kerf.Topology` (the kernel's `Shape`, `Face`, `Edge`…), `Kerf.Modeling` (everything in this guide), and
  `using static Kerf.Modeling.Operations;` so that `Extrude(…)`, `Fillet(…)` and the rest read as in build123d.

## 2. The model in five rules

1. **Algebra mode only.** There's no `BuildPart`/`BuildSketch` builder mode. You combine objects with operators:
   `a + b` (fuse), `a - b` (cut), `a & b` (common). `Location * shape` places a shape; `locations * shape` copies it
   to each.
2. **Shapes never change.** Every operation returns a new shape. `part.Moved(…)`, `part.Rotate(…)`, `WithLabel(…)`
   are copies; there's no `shape.location = …`.
3. **Operations that can fail return `Result<T>`.** Fillet, chamfer, offset, loft, sweep, extrude, revolve, draft,
   thicken, imports and exports return a `Result`, never throw for a geometric failure. Check it (section 6).
4. **Operators and constructors can't return a `Result`,** so `a - b` throws `ModelingException` if the Boolean
   fails, and `new Triangle(…)` if no triangle has those sides. Use the methods `a.Subtract(b)`, `a.Add(b)` and
   `a.Common(b)`, which return a `Result<Part>` (or `Sketch`, `Curve`), and each object's `Make…` twin
   (`MakeTriangle(…)`, `MakeSpline(…)`), where a failure is possible and you want to handle it.
5. **Python names become .NET names:** `fillet` → `Fillet`, `Align.MIN` → `Align.Min`, `edges()` → `Edges()`,
   `sort_by(Axis.Z)` → `OrderBy(Axis.Z)`, `group_by(Axis.Z)[-1]` → `GroupBy(Axis.Z).Last()`. See
   [name-map.md](name-map.md).

The three object kinds are `Part` (3D), `Sketch` (2D faces) and `Curve` (1D), as in build123d. `part.Shape` is the
kernel's shape underneath, when you need the kernel's own API (section 8).

## 3. Making shapes

```csharp
// Primitives: Box, Cylinder, Sphere, Cone, Torus, Wedge; holes: Hole, CounterBoreHole, CounterSinkHole
Part block = new Box(40, 30, 20);
Part withHole = block - new Cylinder(5, 20);            // 24000 - 1570.796 = 22429.204

// A sketch, extruded
Sketch outline = new Rectangle(50, 20) - new Circle(4);
Result<Part> bar = Extrude(outline, 8);                 // 7597.876

// A profile on another plane, revolved about Z
Sketch profile = new Location(Plane.XZ) * new Rectangle(5, 10, align: new Alignment(Align.Min, Align.Min));
Result<Part> ring = Revolve(profile.Moved(new Location(new Vector3D(10, 0, 0))), Axis.Z);   // π(15² − 10²)·10

// A loft between sections
Sketch bottom = new Rectangle(20, 20);
Sketch top = new Location(new Vector3D(0, 0, 30)) * new Circle(6);
Result<Part> loft = Loft([bottom, top]);

// The convex hull of points: one face for each flat side (points closer than 1e-6 count as one)
Part hull = new ConvexPolyhedron(points);
Result<Part> checkedHull = MakeConvexPolyhedron(points);   // modeling.hull-failed if the points are flat or too few

// Patterns: spacing first, then counts
Part plate = new Box(100, 60, 5) - new GridLocations(40, 40, 3, 2) * new Cylinder(3, 5);   // six holes
```

Sketch objects: `Rectangle`, `RectangleRounded`, `Circle`, `Ellipse`, `Polygon`, `RegularPolygon`, `Trapezoid`,
`Triangle`, the `Slot…` family, `Superellipse`. Curves: `Line`, `Polyline`, `CenterArc`, `ThreePointArc`,
`TangentArc`, `RadiusArc`, `Spline`, `Bezier`, `Helix`, `FilletPolyline` and others, as in build123d.

**An object that can't be made from its values throws `ModelingException`,** carrying the error a `Result` would
(`Error`, with its `Code`, and `Diagnostics`): no triangle has sides 1, 1 and 3, no arc of radius 1 joins points 4
apart, no spline passes through points closer than 1e-6. It's an `InvalidOperationException`. **Each object has a twin
in `Operations` returning a `Result`,** `Make` and its name, with the same parameters, for values that may not make a
shape:

```csharp
Result<Sketch> tri = MakeTriangle(sideA: 1, sideB: 1, sideC: 3);   // modeling.no-solution: No such triangle
Result<Curve> arc = MakeRadiusArc(new Point3D(0, 0, 0), new Point3D(4, 0, 0), 1);   // modeling.no-solution
try { var s = new Spline(points); }
catch (ModelingException e) { Console.WriteLine(e.Error.Code); }   // modeling.spline-failed
```

An argument that is wrong on its own (a null, too few points, a missing angle, a negative radius the object checks)
is still an `ArgumentException`, from the constructor and its `Make…` alike.

Objects are centred by default, as in build123d. `align: new Alignment(Align.Min, Align.Min, Align.Min)` puts the
minimum corner at the origin; `GridLocations(xSpacing, ySpacing, xCount, yCount)` takes the spacing between
centres, not the overall size.

Options are records, not keyword arguments:

```csharp
Extrude(face, 10, new ExtrudeOptions { Both = true, Taper = 5 });
Extrude(face, Until.Next, target);
Sweep([section], path, new SweepOptions { IsFrenet = true });
```

### Text: embossing and engraving

`Text` is a sketch of a string's faces, the holes in letters as holes, as build123d's is. Extrude it up from a face to
emboss it, or down into one to engrave it:

```csharp
using Kerf.Fonts;   // FontStyle, TextAlign

Part plate = new Box(80, 30, 4);
Sketch label = new Text("KERF 0.5", 10, font: "Liberation Sans", fontStyle: FontStyle.Bold);
Sketch onTop = label.Moved(new Location(new Vector3D(0, 0, 2)));   // on the plate's top face
Part embossed = plate + Extrude(onTop, 1).Value;                   // 9600 + 129.622 = 9729.622
Part engraved = plate - Extrude(onTop, -0.5).Value;                // 9600 - 129.622 / 2 = 9535.189

// A note in lines, its box's top left at the origin
Sketch note = new Text("Line one\nLine two", 5, "Liberation Sans", textAlign: (TextAlign.Left, TextAlign.Top),
                       align: new Alignment(Align.Min, Align.Max));

// Along a path: each character upright on the curve
Sketch arc = new Text("ROUND THE ARC", 6, "Liberation Sans", textAlign: (TextAlign.Left, TextAlign.Bottom),
                      path: Edge.MakeCircle(30, startAngle: 0, endAngle: 180), positionOnPath: 0.05);
```

- The size is the font's em in model units, as in build123d: a capital is about 0.7 of it.
- Fonts are found by family among those installed for everyone and for you, on Windows, macOS and Linux, or read
  from `fontPath`. build123d's default is Arial; where Arial, or any family asked for, isn't installed, Kerf's own
  Liberation Sans stands in, which has Arial's widths, with a warning (`Warnings.Capture()`, or `font.substituted` on
  `MakeText`'s result). Name `"Liberation Sans"` to get the same text on every machine.
- `MakeText(…)` gives the same as a `Result<Text>`: a character the font has no glyph for fails as
  `text.missing-glyph`, naming it, where build123d draws an empty box; an empty text as `text.empty`.
- A single-stroke engraving font (one whose family starts with "olf " or says "SingleLine") draws its strokes at
  `singleLineWidth`, 4% of the size by default.

## 4. Selecting faces and edges

Selections are lists you filter, order and group, with LINQ alongside:

```csharp
Part box = new Box(30, 30, 10);
var topEdges = box.Edges().GroupBy(Axis.Z).Last();       // the four edges at the top
var vertical = box.Edges().ParallelTo(Axis.Z);           // the four vertical edges
Face top = box.Faces().OrderBy(Axis.Z).Last();           // the top face
var circles = part.Edges().Where(GeomType.Circle);
var longest = part.Edges().OrderBy(ShapeKey.Length).Last();
var near = part.Faces().Between(Axis.Z, 0, 5);
```

`Where(GeomType.…)`, `ParallelTo(…)`, `OrderBy(Axis…)` and `OrderBy(ShapeKey…)`, `GroupBy(…)`, `Between(…)` and
their `Not…`/`…Descending` forms cover build123d's `filter_by`, `sort_by`, `group_by` and `filter_by_position`.
Ordinary LINQ (`Where(e => e.Length > 5)`, `First()`, `Last()`) works too; `ToShapeList()` makes a list again.

## 5. Fillets, chamfers and shells

```csharp
Result<Part> rounded = Fillet(box, topEdges, 2);                   // 8900.059
Result<Part> chamfered = Chamfer(box, vertical, 1.5);              // 9000 − 4 · 1.5²/2 · 10 = 8955
Result<Part> cup = Offset(box, -1.5, new OffsetOptions { Openings = [top] });   // 9000 − 27 · 27 · 8.5 = 2803.5
```

Pass the part as the first argument (`Fillet(box, edges, r)`): the edges alone don't know which solid they came
from. **A chamfer can't be longer than the tightest convex arc in a face beside its edges:** on that face
its edge is the outline stepped in by the length, and past the arc's radius that closes up and crosses itself. So
`Chamfer` refuses such a length at once, saying what fits (`modeling.chamfer-failed`: "… the most that fits is 1.7,
the radius of the tightest convex arc in a face beside them …"), as `Fillet` does for a radius wider than a face.
With two lengths, or a length and an angle, it's the length on that face that counts. A chain of tangent edges is
chamfered all round, so one straight edge of a rounded outline is held to the arcs too. `MaxChamfer(edges)`, as
`MaxFillet`, finds the largest single length that builds: the proved limit where it builds, or else just under it, by
twice the shape's tolerance there, where that builds (1.6999998 on the box above, whose corners close to a point at
1.7), and otherwise a search to within 0.1. The kernel's `ChamferSizeLimit` and `FilletSizeLimit` give the limit with
that tolerance. A shell is `Offset` with `Openings`, negative to go inwards. 2D offsets of sketches and curves are `Offset`
too, with `Kind` (`Arc`, `Intersection`) and `Side` in `OffsetOptions`.

**Chamfers past tight corners** (a Kerf addition: build123d and OCCT have nothing like it). A chamfer longer than the
radius of a convex arc on its face can't step that arc in, and is refused. With
`new ChamferOptions { TightCorners = TightCorners.Collapse }` the corner collapses to a point instead: the straight
edges' insets meet, and the chamfer over the arc is a cone from there, tangent to the planes beside it, as the convex
hull a user would otherwise build by hand. It's built for whole loops of a planar face, their sides square to it or
drafted by one angle (the end of a rounded prism, a housing, a moulding), with the length down a drafted side measured
along its slope: tight arcs beside lines or convex arcs (a convex arc's end beside the corner runs to its point too),
corners of several arcs together, and corners that are B-splines or ellipses where the collapse swallows them whole. A
tight arc beside a concave arc, a curve the corner swallows only in part, and sides drafted by different angles are
refused, saying why:

```csharp
Part block = new Box(40, 30, 30);
Part body = Fillet(block, block.Edges().ParallelTo(Axis.Z), 1.7).Value;
var end = body.Edges().GroupBy(Axis.Z).Last();
Result<Part> refused = Chamfer(body, end, 3);                        // longer than the 1.7 corners
Result<Part> past = Chamfer(body, end, 3, options: new ChamferOptions { TightCorners = TightCorners.Collapse });   // 35336.538
```

Below every corner's radius the option changes nothing. `MaxChamfer` and the kernel's `ChamferLimit` take no options
and give the default's limit, the arcs' radius; with `Collapse` the arcs set no limit, and what fits is set by the
face's width, which they don't work out.

## 6. Results, failures and warnings

An operation that can fail gives a `Result<T>`:

```csharp
Result<Part> tooBig = Fillet(box, topEdges, 20);
if (tooBig.IsFailure)
    Console.WriteLine(tooBig.Error);
// error modeling.fillet-failed: The fillet with a radius of 20 is too large for these 4 edges: the most that fits is
// 10, where it uses up the whole width of a face beside the edge. …

if (Fillet(box, topEdges, 2).TryGetValue(out Part? part)) { /* use part */ }
Part p = Fillet(box, topEdges, 2).Value;    // throws InvalidOperationException, with the error, if it failed
```

- `IsSuccess`, `IsFailure`, `Value`, `TryGetValue`, `GetValueOrDefault`, `Error` (the first error) and `Diagnostics`
  (every error and warning). `Map`, `Bind` and LINQ's `from … select` chain results.
- **Read the error's message:** it says what failed and usually why (here, the largest radius that fits). Its `Code`
  is stable, for code to test: `modeling.fillet-failed`, `modeling.offset-failed`, `meshing.failed` and so on.
- **A success can carry warnings** in `Diagnostics`, for example when a result was built a different way from the
  one asked. Check them when a result matters.
- **Kerf doesn't report success for a result it knows is wrong.** A Boolean, offset or fillet whose result fails its
  checks (wrong volume, not a valid solid) is a failure, not a broken shape. `TryGetInvalid` gives the broken shape,
  if one was made, to look at.
- `part.IsValid` checks any shape, as build123d's `is_valid`. Check what you import.

## 7. Meshing

Two ways in. For a quick look, the modelling layer's `Triangulate`:

```csharp
var mesh = rounded.Value.Triangulate(0.05);   // linear deflection 0.05, angular 0.2 rad
// mesh.Faces[i].Positions, .Normals, .Triangles; mesh.Edges[i].Points; in the order of Faces() and Edges()
```

`Triangulate` throws `InvalidOperationException` if the shape can't be meshed. **When meshing might fail, or you
need to know which faces did, use the kernel's mesher,** which returns a `Result`:

```csharp
using Kerf.Meshing;

Result<Kerf.Meshing.ShapeMesh> result = part.Shape.Mesh(new MeshOptions { LinearDeflection = 0.05, AngularDeflection = 0.3 });
if (result.IsFailure) Console.WriteLine(result.Error);           // meshing.failed: why
foreach (var d in result.Diagnostics) Console.WriteLine(d);      // meshing.faces-not-meshed: which faces were left out
var m = result.Value;                                            // Positions, Normals, Triangles, Faces, Edges
```

- `Kerf.Modeling` and `Kerf.Meshing` both have a `ShapeMesh`: write `Kerf.Meshing.ShapeMesh` or use `var`.
- **Deflection is a distance in the model's units,** the most a triangle may be from the surface. Pick it from the
  part's size: `MeshTolerances.For(part.Shape)` gives 0.1% of the bounding box's diagonal. A tiny deflection on a
  large part asks for millions of triangles. `MeshOptions.MaxTriangles` (10 million by default) refuses a mesh it
  expects to go past the limit, as `meshing.too-large`.
- `MeshOptions { Relative = true }` makes the deflection a proportion of each edge's size, as OCCT's relative mode.
- Exports mesh for you: `ExportStl`, `ExportGltf` and `Export3mf` take the deflections in their options.
- **A mesh further from the surface than the deflection says so,** with the warning `meshing.deflection-exceeded`:
  spheres and tori are measured as they're meshed and refined until they keep to it, and a face that can't be refined
  further is left as it is and named. It's met at relative deflections, on small fillet tori beside a long edge, up to
  about 1.14 times out. The exports give it in their results' `Diagnostics`; `Triangulate`, `Tessellate` and
  `TriangulateTree` raise it to `Warnings` (section 6); the kernel's `Mesh` in its `Diagnostics`,
  with the faces in `Shapes`. Other kinds of surface aren't measured, so nothing is said of them.

```csharp
var written = Exporters3D.ExportStl(part, "part.stl");                    // relative 0.001 by default
foreach (var d in written.Diagnostics) Console.WriteLine(d);              // warning meshing.deflection-exceeded: Face 2 is …
```

## 8. Import and export

```csharp
Exporters3D.ExportStep(part, "part.step");
Exporters3D.ExportStl(part, "part.stl", new StlExportOptions { LinearDeflection = 0.01 });
Exporters3D.ExportGltf(part, "part.glb", new GltfExportOptions { Binary = true });
Exporters3D.Export3mf(part, "part.3mf");
Exporters3D.ExportBrep(part, "part.brep");

Result<ModelShape> step = Importers.ImportStep("part.step");     // assemblies, labels and colours
Result<Shape> brep = Importers.ImportBrep("part.brep");
```

Every exporter and importer returns a `Result`. A missing folder or file is `DirectoryNotFoundException` or
`FileNotFoundException`, as everywhere in .NET; a file past `ImportLimits` (size, nesting) is
`InvalidDataException`.

**Async forms.** Every importer and exporter, and `ExportDxf.Write`, has an async form named `…Async`, taking a
`CancellationToken` and returning a `Task<Result>` or `Task<Result<…>>`; so do the kernel's readers and writers
(`Step.ReadAsync`, `Stl.WriteAsync`, `Svg.WriteAsync`, `Png.WriteAsync`, `Font.ReadAsync` and the rest). Only the I/O
is asynchronous: parsing and building a file are CPU work, which Kerf doesn't hide in a task.

```csharp
Result<ModelShape> read = await Importers.ImportStepAsync("part.step", cancellationToken: token);
Result written = await Exporters3D.ExportStepAsync(part, "part.step", cancellationToken: token);
```

- A reader reads the whole file or stream into memory without blocking a thread, then parses it on the thread the
  read finished on. In a UI, call it inside `Task.Run` to keep the parsing off the UI thread.
- A writer builds the file in memory on the calling thread, then writes it without blocking one, and flushes the
  stream. Nothing is written if it fails.
- The results, diagnostics and bytes are the synchronous forms'. The cost is memory: the file's bytes, held while it
  is parsed (STEP's synchronous reader parses as it reads) or before it is written. For the largest STEP file of
  OCCT's test data, 22 MB, that is 22 to 26 MB more allocated beside the 2.3 GB that reading it allocates.
- A missing file or folder, or a unit that isn't a length, throws before the task starts, as the synchronous forms
  throw; a cancelled token cancels the task.

### Drawings: hidden lines and SVG

build123d's `ExportSVG` class isn't ported (`ExportDxf` is, below), but the kernel's `Kerf.Drawing` does the same job: a hidden-line
projection of any shape through a camera, written as SVG with the hidden lines in their own dashed layer.

```csharp
using Kerf.Drawing;

Result<LineDrawing> drawing = part.Shape.ProjectLines(Camera.Isometric);   // or Top, Front, Right, Camera.LookAt(...)
using (var file = File.Create("part.svg"))
    Console.WriteLine(Svg.Write(drawing.Value, file));                       // a Result

string front = Svg.WriteToString(part.Shape.ProjectLines(Camera.Front).Value,
                                 new SvgOptions { IncludeHidden = false, Margin = 2 });
```

- The page is full size in millimetres with a 5 mm margin unless `SvgOptions` says otherwise (`PageWidth`,
  `PageHeight`, `Scale`, `Unit`, `VisibleStyle`, `HiddenStyle`, `Background`).
- `DrawingOptions` chooses the method (exact, or the faster polygonal one) and whether hidden lines are computed at all.
- `DrawingOperations.ProjectLines(shapes, camera)` draws several shapes together, each hiding the others' lines.

### DXF: for other CAD programs and laser cutters

`ExportDxf` is build123d's `ExportDXF`: layers with colours, line types and weights, then shapes or drawings on them.

```csharp
// A sketch's outline to cut, and a hole's to engrave, in millimetres
Result written = new ExportDxf(unit: Unit.Millimeter)
    .AddLayer("cut", ColorIndex.Red)
    .AddLayer("engrave", ColorIndex.Blue, lineWeight: 0.25)
    .AddShape(new Rectangle(50, 20) - new Circle(4), "cut")
    .AddShape(new Pos(18, 0) * new Circle(2), "engrave")
    .Write("plate.dxf");

// A drawing: the visible lines on "visible", the hidden ones on "hidden" (grey, dashed)
Result<LineDrawing> drawing = part.Shape.ProjectLines(Camera.Isometric);
new ExportDxf().AddDrawing(drawing.Value).Write("part.dxf");
```

- **Each call returns a new exporter;** chain them, or keep the result: `dxf.AddShape(s);` on its own adds nothing.
- Edges are drawn in the XY plane: lines, circles, arcs, ellipses and splines exactly (a rational spline with its
  weights), anything else as a polyline within `tolerance` (1e-3). An edge off the plane is projected onto it, and
  `Write` warns (`modeling.dxf-not-planar`). Coordinates aren't scaled: `unit` names them in the header.
- `new ExportDxf(DxfVersion.R12)` writes the old R12 format, which some laser cutters' software reads better:
  ellipses and splines become polylines, and the header has no units.
- A layer must be added before a shape goes on it; `""` is the default layer, `"0"`. Line weights are DXF's standard
  ones (0.05, 0.09, 0.13 … 2.11 mm).
- `Text`'s faces are written as their edges, as any other shape's. A drawing's notes (next) are written as closed
  polylines of their outlines by default, filled in DXF 2013 by a solid HATCH under each letter's contours (holes
  kept), so they look as in the SVG; `new ExportDxf(fillOutlines: false)` writes the contours alone, for a laser cutter
  to engrave the outlines, say. R12 has no HATCH: its notes are their contours either way. Or notes can be TEXT, with
  `new ExportDxf(textOutput: TextOutput.Text)`.

### Drawing sheets

A sheet is paper of a standard size with a border, a title block and views of a part at chosen scales: a `Kerf.Drawing`
addition, as build123d has no drawing sheets (its `TechnicalDrawing` is a sketch of a frame and text faces). The sheet
is a line drawing in millimetres, so it goes to SVG and to DXF as any drawing does.

```csharp
using Kerf.Drawing;
using Kerf.Fonts;   // TextAlign, for notes

Part plate = new Box(30, 20, 10) - new Cylinder(4, 10);
var sheet = new Sheet(PageSize.A4)          // landscape by default; ISO A0 to A10, Letter, Legal, Ledger, ANSI C to E
{
    TitleBlock = new TitleBlock { Title = "Plate", PartNumber = "KF-0001", Date = "2026-11-09", Author = "Kerf" },
    Views =
    [
        new SheetView(plate.Shape.ProjectLines(Camera.Top).Value, new Point2D(90, 140), 2),    // 2:1, centred there
        new SheetView(plate.Shape.ProjectLines(Camera.Front).Value, new Point2D(200, 140), 2),
        new SheetView(plate.Shape.ProjectLines(Camera.Isometric).Value, new Point2D(90, 60), 1),
    ],
};
LineDrawing drawn = sheet.Draw().Value;      // a Result: it warns of a view or an entry that doesn't fit
File.WriteAllText("plate.svg", Svg.WriteToString(drawn, sheet.PageOptions()));   // a page of the paper's size
new ExportDxf().AddDrawing(drawn).Write("plate.dxf");                             // millimetres from its corner

// A note on any drawing: text at a point, set in a font
DrawingNote note = DrawingNote.Make("Ø8 through", new Point2D(20, 15), new NoteStyle { Size = 3.5 }).Value;
LineDrawing annotated = plate.Shape.ProjectLines(Camera.Top).Value.WithNotes(note);
string asText = Svg.WriteToString(annotated, new SvgOptions { TextOutput = TextOutput.Text });
new ExportDxf(textOutput: TextOutput.Text).AddDrawing(annotated).Write("plate-top.dxf");
```

- **Text is outlines by default,** in both formats: SVG fills each note's glyph outlines, DXF writes each contour as a
  closed polyline within the exporter's `tolerance` and, in DXF 2013, fills it with a solid HATCH (`fillOutlines:
  false` for the contours alone), so the file looks the same everywhere and a laser cutter can engrave it. **`TextOutput.Text`** writes text instead: SVG's `<text>` in the font's family, DXF's TEXT with a text style for
  each font (`LiberationSans-Regular.ttf`), smaller and editable but drawn in whatever font the reader has. R12's text is
  ASCII, so a note that isn't is written as outlines there, with a warning (`modeling.dxf-text-as-outlines`).
- A note's size is the font's em, as `Text`'s is; `NoteStyle` sets the font, the alignment on the note's position
  (`TextAlign.Left`, `Center`, `Right`; `Bottom`, `Center`, `Top`, `TopFirstLine`) and a rotation in degrees.
  `DrawingNote.Make` fails as `MakeText` does (`text.missing-glyph`, `text.empty`).
- The title block's fields are plain text the caller fills: title, part number, scale (the views' own, `2:1`, unless
  given), units (`mm`), date, author and sheet (`1 of 1`). Nothing reads the clock, so the same sheet is the same file.
- A view is placed by the centre of its drawing's extent, at a scale of paper millimetres to the model's unit; a view
  outside the border or over the title block, or an entry too long for its cell, is a warning (`drawing.sheet-overflow`).
- `ExportDxf.AddDrawing` puts notes on a layer `"notes"` (`notesLayer:` to change it, null to leave them out);
  `AddNotes(notes, layer)` adds notes alone.

### Pictures: shaded PNG

```csharp
using Kerf.Drawing;
using Kerf.Meshing;

var mesh = part.Shape.Mesh(Renderer.DefaultMeshOptions(part.Shape)).Value;
var scene = new Scene(new SceneItem(mesh), new SceneItem(pinMesh, new Color(0.2, 0.4, 0.8)));
Result<RgbaImage> image = Renderer.Render(scene, Camera.Isometric, new RenderOptions { Width = 640, Height = 480 });
using (var file = File.Create("part.png"))
    Console.WriteLine(Png.Write(image.Value, file));
```

- A `SceneItem` has a colour, colours for single faces (`WithFaceColor`) and a `Transform`.
- `RenderOptions` sets the size, the background (an opacity of 0 makes it transparent), flat or smooth shading, the
  lights and which edges are drawn over the faces.
- A colour with an opacity under 1, `new Color(0.2, 0.5, 0.9, 0.35)`, makes an item (its faces and edges) see-through,
  whatever the order of the items.

## 9. Running operations in parallel

The modelling operations have no async forms: they're CPU work, and you choose the threads they run on. Reading and
writing files have async forms (section 8).

- **Shapes and parts are immutable,** so one can be shared between threads without a lock: every thread can measure
  it, select from it, fillet it, cut from it, mesh it, draw it or export it at once, and gets what it would alone. The
  options records, locations, meshes, drawings, fonts and exporters (`ExportDxf`) are immutable too.
- **Operations run on many threads at once,** each giving exactly what it gives on one: the same parts to the bit,
  the same meshes and the same files. Kerf's tests build parts with fillets, chamfers, shells, extrusions,
  revolutions, lofts, sweeps, hulls and text on several threads, and mesh, draw, render and export them, against the
  same run on one thread.
- **What Kerf keeps between operations is safe too.** A solid's measured volume and the faces a check found valid are
  kept with the shape's immutable data, for the next Boolean on it: two threads reading or keeping them at once
  get the same answers, and the worst a race does is measure or check something again. Nothing kept changes a
  result, and nothing keeps a shape alive.
- Independent parts are the natural unit of work: one part a thread. Meshing, and a few of the kernel's algorithms,
  use .NET's thread pool themselves; many parts at once share the pool rather than multiply its threads.

```csharp
int[] sizes = [40, 50, 60, 70];

// Each part built and written on a thread of its own
Parallel.ForEach(sizes, size =>
{
    Part plate = new Box(size, 30, 8) - new GridLocations(size / 2.0, 15, 2, 2) * new Cylinder(3, 8);
    Part part = Fillet(plate, plate.Edges().GroupBy(Axis.Z).Last(), 1.5).Value;
    Exporters3D.ExportStep(part, $"plate-{size}.step");
});

// Or as tasks: each part built on the thread pool, then written asynchronously
Result[] written = await Task.WhenAll(sizes.Select(async size =>
{
    Part part = await Task.Run(() => (Part)(new Box(size, 30, 8) - new Cylinder(3, 8)));
    return await Exporters3D.ExportStepAsync(part, $"block-{size}.step");
}));
```

**Parts built in parallel pay** (decision 2080): the blade bumper enclosure's 13 parts built with `Parallel.For`, the
shared sections in `Lazy<T>` built once by whichever part needs them first, came out the same to the bit, for the same
bytes, in 1.5 to 1.7 times less wall time on 4 processors (a loaded machine; the largest part bounds it on more).
Keep what the parts share in `Lazy<T>` (thread-safe by default) or build it first; don't keep the part being built in a
static the parts read, such as a size chosen by a setter: give each part what it needs.

**The runtime's settings belong to your application** (`.csproj` or `runtimeconfig.json`), and matter once many
operations run at once:

- **`TieredPGO` off and `CallCountingDelayMs` 0** cut the processor time of a parallel run of Kerf by 35% to 45%,
  whichever collector, and suit any application that runs Kerf for more than a moment (kernel README, "Runtime
  settings for a host").
- **The server collector** (`ServerGarbageCollection`, with DATAS left on) does half the collections and, on a machine
  the application has to itself, took a parallel round from 1,862 to 546 ms with the settings above. **On a machine
  other busy processes share** (a build agent, a desktop running other heavy work), its pauses ran to 5 to 14 s a
  round, once 118 s, as each heap's thread waits for its own processor; the workstation collector's stayed under 1.5 s.
  There, keep the workstation collector (the default), or try `GCNoAffinitize`.
- `GCConserveMemory` and a heap mask over part of the processors didn't help.

**Cancellation, warnings and `SkipClean` follow the flow of control,** on a thread and through `await` alike (each
keeps its state in an `AsyncLocal`):

- `using (Cancellation.Use(token)) { … }`, `using (var warnings = Warnings.Capture()) { … }` and
  `using (new SkipClean()) { … }` apply to the code inside: on its thread, after its `await`s, and in the tasks and
  `Parallel` loops it starts. A token passed to an operation overrides the one in force.
- Other callers don't see them: two cells computed at once on two threads each collect their own warnings, and one
  that skips the clean, or is cancelled, doesn't change the other.
- A scope collects the warnings of a `Parallel` loop started inside it, from every thread. A cancelled operation in
  a loop's body throws `OperationCanceledException`, which `Parallel` gathers into an `AggregateException`.
- An `async` method's scopes end with it, even one it never disposed: its caller doesn't see them.
- **A scope lasts from its opening until it is disposed, wherever the `Dispose` runs.** Disposed in another task, it
  ends on the flow that opened it too. A task started inside a scope that outlives it doesn't keep it: once the
  scope is disposed the task cleans again, obeys the token outside, and its warnings go to the enclosing scope still
  open, if any. So await what you start inside a scope before it ends, if it's to apply to them; `using` does the
  rest.

## 10. Common mistakes

- **Not checking a `Result`.** `.Value` on a failure throws; use `IsFailure` or `TryGetValue` and print `Error`.
- **`ShapeMesh` is ambiguous** with both `Kerf.Modeling` and `Kerf.Meshing` imported: qualify it or use `var`.
- **`Shape` is ambiguous** if another library you use has one: Kerf's is `Kerf.Topology.Shape`.
- **Points and vectors are different types.** `new Point3D(x, y, z)` for a position, `new Vector3D(x, y, z)` for an
  offset or direction; tuples don't convert.
- **`==` on points is exact.** Use `a.IsNear(b)` to compare with a tolerance.
- **A fillet or chamfer without its part:** `Fillet(edges, r)` works only when the edges came from one part; pass the
  part first.
- **Angles are degrees** in objects and operations (`Rotation`, `Revolve`, `Taper`), as in build123d; mesh angular
  deflections are radians.
- **Not ported yet:** text, builder mode, `import_svg`. SVG drawings come from `Kerf.Drawing`, DXF from `ExportDxf`
  (section 8). [name-map.md](name-map.md) lists the rest.

## 11. Coming from build123d

```python
# build123d
with BuildPart() as p:
    Box(30, 30, 10)
    fillet(p.edges().group_by(Axis.Z)[-1], radius=2)
export_step(p.part, "part.step")
```

```csharp
// Kerf.Modeling
Part box = new Box(30, 30, 10);
Part part = Fillet(box, box.Edges().GroupBy(Axis.Z).Last(), 2).Value;
Exporters3D.ExportStep(part, "part.step");
```

Kerf differs from build123d on purpose where build123d is wrong or unclear; those are listed under "Deliberate
differences" in [name-map.md](name-map.md). Where a result differs from OCCT's, Kerf's decision log
(`kernel/docs/decisions.md`) says why.
