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
4. **Operators can't return a `Result`,** so `a - b` throws `InvalidOperationException` if the Boolean fails. Use
   the methods `a.Subtract(b)`, `a.Add(b)` and `a.Common(b)`, which return a `Result<Part>` (or `Sketch`, `Curve`),
   where a failure is possible and you want to handle it.
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

Objects are centred by default, as in build123d. `align: new Alignment(Align.Min, Align.Min, Align.Min)` puts the
minimum corner at the origin; `GridLocations(xSpacing, ySpacing, xCount, yCount)` takes the spacing between
centres, not the overall size.

Options are records, not keyword arguments:

```csharp
Extrude(face, 10, new ExtrudeOptions { Both = true, Taper = 5 });
Extrude(face, Until.Next, target);
Sweep([section], path, new SweepOptions { IsFrenet = true });
```

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
`MaxFillet`, finds the largest single length that builds. A shell is `Offset` with `Openings`, negative to go inwards. 2D offsets of sketches and curves are `Offset`
too, with `Kind` (`Arc`, `Intersection`) and `Side` in `OffsetOptions`.

**Chamfers past tight corners** (a Kerf addition: build123d and OCCT have nothing like it). A chamfer longer than the
radius of a convex arc on its face can't step that arc in, and is refused. With
`new ChamferOptions { TightCorners = TightCorners.Collapse }` the corner collapses to a point instead: the straight
edges' insets meet, and the chamfer over the arc is a cone from there, tangent to the planes beside it, as the convex
hull a user would otherwise build by hand. It's built for whole loops of a planar face of lines and arcs, their sides
square to it (the end of a rounded prism or a housing), and refused, saying why, otherwise:

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
- Text, when it comes (`Text`), is written as its outlines, as any other shape's edges.

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

## 9. Common mistakes

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

## 10. Coming from build123d

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
