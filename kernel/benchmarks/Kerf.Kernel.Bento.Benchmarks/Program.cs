// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Bento's CAD workload (unit 9E, docs/performance.md): examples from Bento's guide to its CAD functions
// (docs/cad/build123d-in-bento.md, section 9), the introductory examples of build123d written as Bento's formulas, here
// through build123d.NET's algebra API as the CAD_ functions call it. Each is built, triangulated with build123d's
// tolerances as Bento's viewer does (ShapeModels.Part), and its volume and face count read, as a cell shows them.
//   dotnet run -c Release --project benchmarks/Kerf.Kernel.Bento.Benchmarks -p:KerfModelingRoot=<build123d.NET>/ [-- passes]
// and on native OCCT, for comparison: --project benchmarks/Kerf.Kernel.Bento.Benchmarks/Native (the same arguments).
// The first pass is a workbook opening (it includes the JIT's work on OCCT.NET); the others are recalculations, of
// which the median is shown.

using System.Diagnostics;
using System.Globalization;
using Kerf.Modeling;
using static Kerf.Modeling.Operations;

const double Length = 80.0, Width = 60.0, Thickness = 10.0;

var examples = new (string Name, Func<Shape> Build)[]
{
    // A1 =CAD_PT_BOX(80, 60, 10); A2 =CAD_CUT(A1, CAD_PT_CYLINDER(22 / 2, 10))
    ("2 plate with hole", () => new Box(Length, Width, Thickness) - new Cylinder(22.0 / 2, Thickness)),
    // A1 =CAD_CUT(CAD_SK_CIRCLE(60), CAD_SK_RECT(80 / 2, 60 / 2)); A2 =CAD_PT_EXTRUDE(A1, 2 * 10)
    ("3 prismatic solid", () => Extrude(new Circle(Width) - new Rectangle(Length / 2, Width / 2), amount: 2 * Thickness)),
    // A1 =CAD_CUT(CAD_SK_CIRCLE(90), CAD_MOVE(CAD_SK_RECT(15, 15), CAD_LOC_POS(45, 0)), CAD_MOVE(CAD_SK_CIRCLE(7.5), CAD_LOC_POS(0, 45)))
    // A2 =CAD_PT_EXTRUDE(A1, 15)
    ("5 working point", () => Extrude(new Circle(90) - new Pos(45, 0.0) * new Rectangle(15, 15) - new Pos(0.0, 45) * new Circle(7.5), 15)),
    // A1 =CAD_CUT(CAD_SK_CIRCLE(80), CAD_MOVE(CAD_SK_CIRCLE(10), CAD_LOC_LIST({60, 0; 0, 60; -60, 0; 0, -60}))); A2 =CAD_PT_EXTRUDE(A1, 10)
    ("6 point lists", () => Extrude(new Circle(80) - new Locations(new Vector(60, 0), new Vector(0, 60), new Vector(-60, 0), new Vector(0, -60))
        .Select(loc => (Shape?)(loc * new Circle(10))), 10)),
    // A1 =CAD_PT_BOX(80, 60, 10); A4 =CAD_PT_CHAMFER(A1, A1.Edges().GroupBy("Z")[-1], 4); A5 =CAD_PT_FILLET(A4, A4.Edges().FilterBy("Z"), 5)
    ("9 chamfer, fillet", () =>
    {
        Shape a1 = new Box(Length, Width, Thickness);
        Shape a4 = Chamfer(a1.Edges().GroupBy(Axis.Z)[^1], length: 4);
        return Fillet(a4.Edges().FilterBy(Axis.Z), radius: 5);
    }),
    // A1 =CAD_PT_BOX(80, 60, 10); A2 =CAD_CUT(A1, CAD_PT_HOLE(60 / 4, 10)); A3 =CAD_PT_FILLET(A2, CAD_SEL_LAST(CAD_SEL_GROUP(CAD_SEL_NEW(A2, A1, {type: "edge"}), "Z")), 2)
    ("10 select last, hole", () =>
    {
        Shape a1 = new Box(Length, Width, Thickness);
        Shape a2 = a1 - new Hole(Width / 4, depth: Thickness);
        return Fillet((a2.Edges() - a1.Edges()).GroupBy(Axis.Z)[^1], 2);
    }),
    // The classic OCC bottle (section 13): a profile, extruded, filleted, a neck added, and hollowed with offset
    ("29 bottle", () =>
    {
        double l = 60.0, w = 18.0, t = 9.0, b = 0.9, h = 90.0, n = 8.0;
        var l1 = new Line((0, 0), (0, w / 2));
        var l2 = new ThreePointArc(l1.PositionAt(1), (l / 2.0, w / 2.0 + t), (l, w / 2.0));
        var l3 = new Line(l2.PositionAt(1), (l2.PositionAt(1).X, 0, 0));
        Shape profile = l1 + l2 + l3;
        profile += Mirror(profile);
        Shape body = Extrude(MakeFace(profile), -(h + b));
        body = Fillet(body.Edges(), radius: w / 6);
        body += Extrude(new Plane(body.Faces().SortBy(Axis.Z).Last) * new Circle(t), n);
        return Offset(body, -b, openings: [body.Faces().SortBy(Axis.Z).Last]);
    }),
    // Nested locations: CAD_LOC_COMBINE of a polar and a grid placement, fused and extruded (164 faces)
    ("31 nesting locations", () =>
    {
        Shape sk = new Rotation(z: 30) * new RegularPolygon(15, 6);
        var cell = new RegularPolygon(5, 4) + new GridLocations(15, 15, 2, 2).Select(loc => (Shape?)(loc * new RegularPolygon(5, 3)));
        sk += new PolarLocations(40, 6).Select(loc => (Shape?)(loc * cell));
        return Extrude(sk, 3);
    }),
    // A1 =CAD_FUSE(CAD_SK_REG_POLYGON(20, 6, {rotation: 30}), CAD_MOVE(CAD_SK_REG_POLYGON(10, 4), CAD_LOC_POLAR(40, 6)))
    // A2 =LET(faces, A1.Faces(), CAD_FUSE(MAP(SEQUENCE(LIST_COUNT(faces)), LAMBDA(i, CAD_PT_EXTRUDE(faces[i], 1 + 3 * (i - 1))))))
    ("32 for-loop", () =>
    {
        Shape a1 = new RegularPolygon(20, 6, rotation: 30);
        a1 += new PolarLocations(40, 6).Select(loc => (Shape?)(loc * new RegularPolygon(10, 4)));
        return new Part().Add(a1.Faces().Select((face, i) => (Shape?)Extrude(face, 1 + 3 * i)));
    }),
    // A1 =LET(locs, CAD_LOC_POLAR(40, 6), CAD_FUSE(MAP(SEQUENCE(6), LAMBDA(i, CAD_PT_EXTRUDE(SQUARE(5 + 2 * (i - 1), locs[i]), 1 + 2 * (i - 1))))))
    ("33 function, for-loop", () =>
        new Part().Add(new PolarLocations(40, 6).Select((loc, i) => (Shape?)Extrude(loc * new RegularPolygon(5 + 2 * i, 4), 1 + 2 * i)))),
    // A1 =CAD_PT_REVOLVE(CAD_MOVE(CAD_SK_CIRCLE(6), CAD_LOC_POS(0, 50)), "X", {angle: 180})
    // A2 =CAD_FUSE(A1, CAD_PT_EXTRUDE(CAD_SK_RECT(6, 50), , {until: "next", target: A1}))
    ("36 extrude until", () =>
    {
        Shape a1 = Revolve(new Pos(0, 50) * new Circle(6), Axis.X, revolutionArc: 180);
        return a1 + Extrude(new Rectangle(6, 50), until: Until.Next, target: a1);
    }),
};

int passes = args.Length > 0 ? int.Parse(args[0], CultureInfo.InvariantCulture) : 7;
var times = new double[examples.Length][];
var bytes = new long[examples.Length];
for (int i = 0; i < examples.Length; i++)
{
    times[i] = new double[passes];
}

string[] results = new string[examples.Length];
var total = Stopwatch.StartNew();
for (int pass = 0; pass < passes; pass++)
{
    for (int i = 0; i < examples.Length; i++)
    {
        long before = GC.GetAllocatedBytesForCurrentThread();
        var clock = Stopwatch.StartNew();
        Shape shape = examples[i].Build();
        ShapeMesh mesh = shape.Triangulate();
        results[i] = string.Create(CultureInfo.InvariantCulture,
            $"volume {shape.Volume:F3}, {shape.Faces().Count} faces, {mesh.TriangleCount} triangles");
        times[i][pass] = clock.Elapsed.TotalMilliseconds;
        if (pass == passes - 1)
        {
            bytes[i] = GC.GetAllocatedBytesForCurrentThread() - before;
        }
    }
}

Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
    $"Bento's examples on OCCT.NET, {passes} passes, {total.Elapsed.TotalSeconds:F1} s"));
Console.WriteLine("example                    first ms  median ms   alloc MB  result");
double firstSum = 0, medianSum = 0;
for (int i = 0; i < examples.Length; i++)
{
    double first = times[i][0];
    double median = passes > 1 ? times[i][1..].Order().ElementAt((passes - 1) / 2) : first;
    firstSum += first;
    medianSum += median;
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
        $"{examples[i].Name,-24} {first,10:F1} {median,10:F1} {bytes[i] / 1048576.0,10:F1}  {results[i]}"));
}

Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{"total",-24} {firstSum,10:F1} {medianSum,10:F1}"));
Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
    $"GC: {GC.CollectionCount(0)}/{GC.CollectionCount(1)}/{GC.CollectionCount(2)} collections, pause {GC.GetTotalPauseDuration().TotalMilliseconds:F0} ms; JIT {System.Runtime.JitInfo.GetCompilationTime().TotalMilliseconds:F0} ms"));
return 0;
