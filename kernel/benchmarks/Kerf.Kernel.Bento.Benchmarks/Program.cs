// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Bento's CAD workload (unit 9E, docs/performance.md): examples from Bento's guide to its CAD functions
// (docs/cad/build123d-in-bento.md, section 9), the introductory examples of build123d written as Bento's formulas, here
// through Kerf.Modeling's algebra API as the CAD_ functions call it. Each is built, triangulated with build123d's
// tolerances as Bento's viewer does (ShapeModels.Part), and its volume and face count read, as a cell shows them.
// tools/bench.ps1 runs it in fresh processes, ReadyToRun, and keeps the figures (docs/performance.md). By hand, from the
// kernel folder:
//   dotnet run -c Release --project benchmarks/Kerf.Kernel.Bento.Benchmarks [-- passes [copies]]
//   (or [-- measures [passes]] for what a volume and an area cost by each integration; or [-- images [passes]] for the SVG drawings and PNG pictures of unit 10L4; or [-- retained] for what a caller keeps; or [-- thread-sets [copies [seconds idle]]] for what each thread keeps)
// (copies: how many of each example the parallel rounds run at once; 0 skips them; the default is a processor's worth)
// tools/parallel-curve.ps1 varies the processors it has, and the collector's settings
// The first pass is a workbook opening (it includes the JIT's work on Kerf); passes 2 to 9 are recalculations, of which
// the median is shown, as native's frozen figures were taken (9 passes); with 12 or more passes the median of the last
// half is the recalculation once the runtime has settled.

using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using Kerf.Geometry;
using Kerf.Kernel.Benchmarks;
using Kerf.Measurement;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

const double Length = 80.0, Width = 60.0, Thickness = 10.0;

var examples = new (string Name, Func<Shape> Build)[]
{
    // A1 =CAD_PT_BOX(80, 60, 10); A2 =CAD_CUT(A1, CAD_PT_CYLINDER(22 / 2, 10))
    ("2 plate with hole", () => new Box(Length, Width, Thickness) - new Cylinder(22.0 / 2, Thickness)),
    // A1 =CAD_CUT(CAD_SK_CIRCLE(60), CAD_SK_RECT(80 / 2, 60 / 2)); A2 =CAD_PT_EXTRUDE(A1, 2 * 10)
    ("3 prismatic solid", () => Extrude(new Circle(Width) - new Rectangle(Length / 2, Width / 2), 2 * Thickness).Value),
    // A1 =CAD_CUT(CAD_SK_CIRCLE(90), CAD_MOVE(CAD_SK_RECT(15, 15), CAD_LOC_POS(45, 0)), CAD_MOVE(CAD_SK_CIRCLE(7.5), CAD_LOC_POS(0, 45)))
    // A2 =CAD_PT_EXTRUDE(A1, 15)
    ("5 working point", () => Extrude(new Circle(90) - new Pos(45, 0.0) * new Rectangle(15, 15) - new Pos(0.0, 45) * new Circle(7.5), 15).Value),
    // A1 =CAD_CUT(CAD_SK_CIRCLE(80), CAD_MOVE(CAD_SK_CIRCLE(10), CAD_LOC_LIST({60, 0; 0, 60; -60, 0; 0, -60}))); A2 =CAD_PT_EXTRUDE(A1, 10)
    ("6 point lists", () => Extrude(new Circle(80) - new Locations(new Point3D(60, 0, 0), new Point3D(0, 60, 0), new Point3D(-60, 0, 0), new Point3D(0, -60, 0))
        .Select(loc => (Shape)(loc * new Circle(10))), 10).Value),
    // A1 =CAD_PT_BOX(80, 60, 10); A4 =CAD_PT_CHAMFER(A1, A1.Edges().GroupBy("Z")[-1], 4); A5 =CAD_PT_FILLET(A4, A4.Edges().FilterBy("Z"), 5)
    ("9 chamfer, fillet", () =>
    {
        Part a1 = new Box(Length, Width, Thickness);
        Part a4 = Chamfer(a1.Edges().GroupBy(Axis.Z)[^1], length: 4).Value;
        return Fillet(a4.Edges().ParallelTo(Axis.Z), radius: 5).Value;
    }),
    // A1 =CAD_PT_BOX(80, 60, 10); A2 =CAD_CUT(A1, CAD_PT_HOLE(60 / 4, 10)); A3 =CAD_PT_FILLET(A2, CAD_SEL_LAST(CAD_SEL_GROUP(CAD_SEL_NEW(A2, A1, {type: "edge"}), "Z")), 2)
    ("10 select last, hole", () =>
    {
        Part a1 = new Box(Length, Width, Thickness);
        Part a2 = a1 - new Hole(Width / 4, depth: Thickness);
        return Fillet(a2.Edges().Except(a1.Edges()).GroupBy(Axis.Z)[^1], 2).Value;
    }),
    // The classic OCC bottle (section 13): a profile, extruded, filleted, a neck added, and hollowed with offset
    ("29 bottle", () =>
    {
        double l = 60.0, w = 18.0, t = 9.0, b = 0.9, h = 90.0, n = 8.0;
        var l1 = new Line(new Point3D(0, 0, 0), new Point3D(0, w / 2, 0));
        var l2 = new ThreePointArc(l1.PositionAt(1), new Point3D(l / 2.0, w / 2.0 + t, 0), new Point3D(l, w / 2.0, 0));
        var l3 = new Line(l2.PositionAt(1), new Point3D(l2.PositionAt(1).X, 0, 0));
        Curve profile = l1 + l2 + l3;
        profile += Mirror(profile);
        Part body = Extrude(MakeFace(profile), -(h + b)).Value;
        body = Fillet(body.Edges(), radius: w / 6).Value;
        body += Extrude(new Plane(body.Faces().OrderBy(Axis.Z).Last()) * new Circle(t), n).Value;
        return Offset(body, -b, new OffsetOptions { Openings = [body.Faces().OrderBy(Axis.Z).Last()] }).Value;
    }),
    // Nested locations: CAD_LOC_COMBINE of a polar and a grid placement, fused and extruded (164 faces)
    ("31 nesting locations", () =>
    {
        Sketch sk = new Rotation(z: 30) * new RegularPolygon(15, 6);
        var cell = new RegularPolygon(5, 4) + new GridLocations(15, 15, 2, 2).Select(loc => (Shape)(loc * new RegularPolygon(5, 3)));
        sk += new PolarLocations(40, 6).Select(loc => (Shape)(loc * cell));
        return Extrude(sk, 3).Value;
    }),
    // A1 =CAD_FUSE(CAD_SK_REG_POLYGON(20, 6, {rotation: 30}), CAD_MOVE(CAD_SK_REG_POLYGON(10, 4), CAD_LOC_POLAR(40, 6)))
    // A2 =LET(faces, A1.Faces(), CAD_FUSE(MAP(SEQUENCE(LIST_COUNT(faces)), LAMBDA(i, CAD_PT_EXTRUDE(faces[i], 1 + 3 * (i - 1))))))
    ("32 for-loop", () =>
    {
        Sketch a1 = new RegularPolygon(20, 6, rotation: 30);
        a1 += new PolarLocations(40, 6).Select(loc => (Shape)(loc * new RegularPolygon(10, 4)));
        return new Part().Add(a1.Faces().Select((face, i) => (Shape)Extrude(face, 1 + 3 * i).Value)).Value;
    }),
    // A1 =LET(locs, CAD_LOC_POLAR(40, 6), CAD_FUSE(MAP(SEQUENCE(6), LAMBDA(i, CAD_PT_EXTRUDE(SQUARE(5 + 2 * (i - 1), locs[i]), 1 + 2 * (i - 1))))))
    ("33 function, for-loop", () =>
        new Part().Add(new PolarLocations(40, 6).Select((loc, i) => (Shape)Extrude(loc * new RegularPolygon(5 + 2 * i, 4), 1 + 2 * i).Value)).Value),
    // A1 =CAD_PT_REVOLVE(CAD_MOVE(CAD_SK_CIRCLE(6), CAD_LOC_POS(0, 50)), "X", {angle: 180})
    // A2 =CAD_FUSE(A1, CAD_PT_EXTRUDE(CAD_SK_RECT(6, 50), , {until: "next", target: A1}))
    ("36 extrude until", () =>
    {
        Shape a1 = Revolve(new Pos(0, 50) * new Circle(6), Axis.X, revolutionArc: 180).Value;
        return a1 + Extrude(new Rectangle(6, 50), Until.Next, a1).Value;
    }),
};

// Images (unit 10L4): `-- images [passes]` draws each example's lines (exact hidden lines, isometric) as SVG and renders
// it at 800 x 600 (meshed for rendering, supersampled 2 x 2) as PNG, and times both
if (args.Length > 0 && args[0] == "images")
{
    int imagePasses = args.Length > 1 ? int.Parse(args[1], CultureInfo.InvariantCulture) : 5;
    Console.WriteLine("example                   svg ms  svg MB  edges   svg KB  png ms  png MB  render ms  png KB");
    foreach ((string name, Func<Shape> build) in examples)
    {
        global::Kerf.Topology.Shape part = build();
        var svgTimes = new double[imagePasses];
        var pngTimes = new double[imagePasses];
        var renderTimes = new double[imagePasses];
        long svgBytes = 0, pngBytes = 0;
        int edges = 0, svgLength = 0, pngLength = 0;
        for (int pass = 0; pass < imagePasses; pass++)
        {
            long before = GC.GetAllocatedBytesForCurrentThread();
            var clock = Stopwatch.StartNew();
            var drawing = global::Kerf.Drawing.DrawingOperations.ProjectLines([part], global::Kerf.Drawing.Camera.Isometric).Value;
            string svg = global::Kerf.Drawing.Svg.WriteToString(drawing);
            svgTimes[pass] = clock.Elapsed.TotalMilliseconds;
            svgBytes = GC.GetAllocatedBytesForCurrentThread() - before;
            (edges, svgLength) = (drawing.Edges.Length, svg.Length);

            before = GC.GetTotalAllocatedBytes(true);
            clock.Restart();
            var image = global::Kerf.Drawing.Renderer.Render(part, global::Kerf.Drawing.Camera.Isometric).Value;
            renderTimes[pass] = clock.Elapsed.TotalMilliseconds;
            byte[] png = global::Kerf.Drawing.Png.Encode(image);
            pngTimes[pass] = clock.Elapsed.TotalMilliseconds;
            pngBytes = GC.GetTotalAllocatedBytes(true) - before;
            pngLength = png.Length;
        }

        static double Median(double[] t) => t.Length > 1 ? t[1..].Order().ElementAt((t.Length - 1) / 2) : t[0];
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{name,-24} {Median(svgTimes),7:F1} {svgBytes / 1048576.0,7:F1} {edges,6} {svgLength / 1024.0,8:F1} {Median(pngTimes),7:F1} {pngBytes / 1048576.0,7:F1} {Median(renderTimes),10:F1} {pngLength / 1024.0,7:F1}"));
        BenchRows.Add("images", name, "svg ms", Median(svgTimes));
        BenchRows.Add("images", name, "svg MB", svgBytes / 1048576.0);
        BenchRows.Add("images", name, "png ms", Median(pngTimes));
        BenchRows.Add("images", name, "png MB", pngBytes / 1048576.0);
        BenchRows.Add("images", name, "result", $"{edges} edges, {svgLength} characters of SVG, {pngLength} bytes of PNG");
    }

    return 0;
}

// What a caller keeps (unit 11Q): `-- retained` holds each example's shape and mesh, and reads how much the collector can't free,
// against what building them allocated. The difference is everything the work threw away (performance.md, "What allocation is made of").
if (args.Length > 0 && args[0] == "retained")
{
    const int held = 8;
    Console.WriteLine("example                  allocated MB  kept KB  kept %");
    double allocatedSum = 0, keptSum = 0;
    foreach (var (name, build) in examples)
    {
        for (int warm = 0; warm < 3; warm++)
        {
            GC.KeepAlive(build().Triangulate());
        }
        var keep = new (Shape Shape, ShapeMesh Mesh, double Volume)[held];
        long live = GC.GetTotalMemory(true);
        long start = GC.GetTotalAllocatedBytes(true);
        for (int i = 0; i < held; i++)
        {
            Shape shape = build();
            keep[i] = (shape, shape.Triangulate(), shape.Volume);
        }
        double allocated = (GC.GetTotalAllocatedBytes(true) - start) / (double)held;
        double kept = (GC.GetTotalMemory(true) - live) / (double)held;
        GC.KeepAlive(keep);
        allocatedSum += allocated;
        keptSum += kept;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{name,-24} {allocated / 1048576.0,12:F1} {kept / 1024.0,8:F0} {100.0 * kept / allocated,7:F2}"));
    }
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
        $"{"total",-24} {allocatedSum / 1048576.0,12:F1} {keptSum / 1024.0,8:F0} {100.0 * keptSum / allocatedSum,7:F2}"));
    return 0;
}

// What each thread keeps (unit 11Q): `-- thread-sets [copies]` runs the parallel round, then drops every thread's sets one field at a
// time, and reads what each lets the collector free (performance.md, "What the per-thread sets hold").
if (args.Length > 0 && args[0] == "thread-sets")
{
    int setCopies = args.Length > 1 ? int.Parse(args[1], CultureInfo.InvariantCulture) : Environment.ProcessorCount;
    long liveAtStart = GC.GetTotalMemory(true);
    for (int round = 0; round < 3; round++)
    {
        Parallel.For(0, setCopies * examples.Length, k =>
        {
            Shape shape = examples[k % examples.Length].Build();
            GC.KeepAlive(shape.Triangulate());
            GC.KeepAlive(shape.Volume);
        });
    }
    if (args.Length > 2)
    {
        // The pool lets an idle thread go after about 20 s, and its sets with it
        Console.WriteLine($"{ThreadPool.ThreadCount} pool threads after the rounds");
        Thread.Sleep(TimeSpan.FromSeconds(int.Parse(args[2], CultureInfo.InvariantCulture)));
    }
    ThreadSets.Report(liveAtStart);
    return 0;
}

// What a measure costs (the owner's answers, item 1): `-- measures [passes]` builds each example once, then times, by
// processor time, its volume and area by fixed Gauss integration and adaptively to several precisions, beside one
// build and mesh of it. A shape's volume is what a cell reads, so its share of a recalculation is the ratio here.
if (args.Length > 0 && args[0] == "measures")
{
    int measurePasses = args.Length > 1 ? int.Parse(args[1], CultureInfo.InvariantCulture) : 200;
    double[] precisions = [0, 1e-4, 1e-5, 1e-6, 1e-7, 1e-9];
    static double Cpu(Action act, int repeats)
    {
        TimeSpan before = Process.GetCurrentProcess().TotalProcessorTime;
        for (int r = 0; r < repeats; r++)
        {
            act();
        }

        return (Process.GetCurrentProcess().TotalProcessorTime - before).TotalMilliseconds / repeats;
    }

    // Besides the examples, faces the default is known to get wrong (OCCT #1675) and parts of other sizes
    Shape Swept(double size) => Sweep(new Plane(new Point3D(size, 0, 0), zDir: new Vector3D(0, 1, 0)) * new Circle(size / 10),
                                      new Helix(size / 2, size * 2, size), new SweepOptions { IsFrenet = true }).Value;
    Shape Lofted() => Loft(new Shape[]
    {
        new Plane(new Point3D(0, 0, 0)) * new Rectangle(20, 10),
        new Plane(new Point3D(2, 3, 20)) * new Ellipse(12, 6),
        new Plane(new Point3D(-3, 0, 40)) * new Rectangle(8, 16),
    }).Value;
    (string Name, Func<Shape> Build)[] others =
    [
        ("helical sweep, size 10", () => Swept(10)),
        ("helical sweep, size 0.01", () => Swept(0.01)),
        ("helical sweep, size 10000", () => Swept(10000)),
        ("loft", Lofted),
    ];
    // Files named after the passes: a free-form part, say, from a STEP file
    foreach (string file in args.Skip(2))
    {
        others = [.. others, (Path.GetFileName(file), () => Importers.ImportStep(file).Value.Shape)];
    }

    foreach (var (name, build) in examples.Concat(others))
    {
        build();
        double buildMs = Cpu(() => build().Triangulate(), 5);
        Shape shape = build();
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{name}: build and mesh {buildMs:F2} ms, {shape.Faces().Count} faces"));
        foreach (double eps in precisions)
        {
            var options = new MassOptions { Tolerance = eps };
            double volume = 0, area = 0;
            // Warmed first, so the order of the precisions doesn't favour one with the JIT's work
            Cpu(() => shape.IntegrateVolume(options), 50);
            Cpu(() => shape.IntegrateArea(options), 50);
            double volumeMs = Cpu(() => volume = shape.IntegrateVolume(options), measurePasses);
            double areaMs = Cpu(() => area = shape.IntegrateArea(options), measurePasses);
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"  eps {eps:G3}: volume {volume:F6} in {volumeMs:F3} ms, area {area:F6} in {areaMs:F3} ms"));
        }
    }

    return 0;
}

int passes = args.Length > 0 ? int.Parse(args[0], CultureInfo.InvariantCulture) : 9;
var times = new double[examples.Length][];
var bytes = new long[examples.Length];
for (int i = 0; i < examples.Length; i++)
{
    times[i] = new double[passes];
}

string[] results = new string[examples.Length];
var total = Stopwatch.StartNew();
var passCpu = new double[passes];
for (int pass = 0; pass < passes; pass++)
{
    TimeSpan passCpuBefore = Process.GetCurrentProcess().TotalProcessorTime;
    for (int i = 0; i < examples.Length; i++)
    {
        long before = GC.GetTotalAllocatedBytes(true);
        Marks.Log?.Mark(pass == 0 ? examples[i].Name : $"pass {pass + 1}: {examples[i].Name}");
        var clock = Stopwatch.StartNew();
        Shape shape = examples[i].Build();
        ShapeMesh mesh = shape.Triangulate();
        results[i] = string.Create(CultureInfo.InvariantCulture,
            $"volume {shape.Volume:F3}, {shape.Faces().Count} faces, {mesh.TriangleCount} triangles");
        times[i][pass] = clock.Elapsed.TotalMilliseconds;
        if (pass == passes - 1)
        {
            bytes[i] = GC.GetTotalAllocatedBytes(true) - before;
        }
    }
    passCpu[pass] = (Process.GetCurrentProcess().TotalProcessorTime - passCpuBefore).TotalMilliseconds;
    if (pass == 0)
    {
        Marks.Log?.Mark("end of first pass");
        // Opening: from the process's start, so with the runtime's own start-up, and how much of it the JIT was
        double sinceStart = (DateTime.Now - Process.GetCurrentProcess().StartTime).TotalMilliseconds;
        BenchRows.Add("opening", "from the process's start to the end of the first pass", "ms", sinceStart);
        BenchRows.Add("opening", "the JIT's time in that", "ms", System.Runtime.JitInfo.GetCompilationTime().TotalMilliseconds);
        BenchRows.Add("opening", "methods the JIT compiled in that", "count", System.Runtime.JitInfo.GetCompiledMethodCount());
    }
}

Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
    $"Bento's examples on Kerf, {passes} passes, {total.Elapsed.TotalSeconds:F1} s"));
Console.WriteLine("example                    first ms  median ms   alloc MB  result");
double firstSum = 0, medianSum = 0, settledSum = 0;
long bytesSum = 0;
for (int i = 0; i < examples.Length; i++)
{
    double first = times[i][0];
    // Passes 2 to 9, as native's frozen figures were taken
    double[] recalculations = times[i][1..Math.Min(passes, 9)];
    double median = recalculations.Length > 0 ? recalculations.Order().ElementAt(recalculations.Length / 2) : first;
    firstSum += first;
    medianSum += median;
    bytesSum += bytes[i];
    BenchRows.Add("bento", examples[i].Name, "first ms", first);
    BenchRows.Add("bento", examples[i].Name, "recalc ms", median);
    BenchRows.Add("bento", examples[i].Name, "alloc MB", bytes[i] / 1048576.0);
    BenchRows.Add("bento", examples[i].Name, "result", results[i]);
    if (passes >= 12)
    {
        double settled = times[i][(passes / 2)..].Order().ElementAt((passes - passes / 2) / 2);
        settledSum += settled;
        BenchRows.Add("bento", examples[i].Name, "settled ms", settled);
    }
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
        $"{examples[i].Name,-24} {first,10:F1} {median,10:F1} {bytes[i] / 1048576.0,10:F1}  {results[i]}"));
}

Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{"total",-24} {firstSum,10:F1} {medianSum,10:F1} {bytesSum / 1048576.0,10:F1}"));
BenchRows.Add("bento", "total", "first ms", firstSum);
BenchRows.Add("bento", "total", "recalc ms", medianSum);
BenchRows.Add("bento", "total", "alloc MB", bytesSum / 1048576.0);
BenchRows.Add("bento", "total", "processor time in a recalculation ms", passes > 1 ? passCpu[1..Math.Min(passes, 9)].Order().ElementAt(Math.Min(passes - 1, 8) / 2) : passCpu[0]);
if (passes >= 12)
{
    BenchRows.Add("bento", "total", "settled ms", settledSum);
}

// Independent workbooks recalculating at once (unit 10I): every example, `copies` times, on the thread pool; the
// wall time of a round (the median of 3) and the bytes it allocates, each result checked against the serial one.
int copies = args.Length > 1 ? int.Parse(args[1], CultureInfo.InvariantCulture) : Environment.ProcessorCount;
if (copies > 0)
{
    var rounds = new double[3];
    var roundPauses = new double[3];
    var roundCpu = new double[3];
    var roundContentions = new long[3];
    long roundBytes = 0;
    int mismatches = 0;
    using Process self = Process.GetCurrentProcess();
    for (int round = 0; round < rounds.Length; round++)
    {
        long allocatedBefore = GC.GetTotalAllocatedBytes(true);
        TimeSpan pausedBefore = GC.GetTotalPauseDuration();
        self.Refresh();
        TimeSpan cpuBefore = self.TotalProcessorTime;
        long contentionsBefore = Monitor.LockContentionCount;
        var clock = Stopwatch.StartNew();
        Parallel.For(0, copies * examples.Length, k =>
        {
            int i = k % examples.Length;
            Shape shape = examples[i].Build();
            ShapeMesh mesh = shape.Triangulate();
            string result = string.Create(CultureInfo.InvariantCulture,
                $"volume {shape.Volume:F3}, {shape.Faces().Count} faces, {mesh.TriangleCount} triangles");
            if (result != results[i])
            {
                Interlocked.Increment(ref mismatches);
            }
        });
        rounds[round] = clock.Elapsed.TotalMilliseconds;
        roundBytes = GC.GetTotalAllocatedBytes(true) - allocatedBefore;
        roundPauses[round] = (GC.GetTotalPauseDuration() - pausedBefore).TotalMilliseconds;
        self.Refresh();
        roundCpu[round] = (self.TotalProcessorTime - cpuBefore).TotalMilliseconds;
        roundContentions[round] = Monitor.LockContentionCount - contentionsBefore;
    }
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
        $"parallel: {copies} copies of every example on {Environment.ProcessorCount} processors, median round {rounds.Order().ElementAt(1):F1} ms "
        + $"({medianSum * copies:F1} ms of serial work), {roundBytes / 1048576.0:F1} MB a round, {mismatches} results differing from the serial run"));
    double medianRound = rounds.Order().ElementAt(1);
    self.Refresh();
    double peakMB = self.PeakWorkingSet64 / 1048576.0;
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
        $"rounds: pause {string.Join(' ', roundPauses.Select(p => p.ToString("F0", CultureInfo.InvariantCulture)))} ms, processor time {string.Join(' ', roundCpu.Select(p => p.ToString("F0", CultureInfo.InvariantCulture)))} ms, contended locks {string.Join(' ', roundContentions)}; peak working set {peakMB:F0} MB; all rounds {string.Join(' ', rounds.Select(p => p.ToString("F0", CultureInfo.InvariantCulture)))} ms"));
    string roundName = $"{copies} copies of every example on {Environment.ProcessorCount} processors";
    BenchRows.Add("parallel", roundName, "round ms", medianRound);
    BenchRows.Add("parallel", roundName, "serial work ms", medianSum * copies);
    BenchRows.Add("parallel", roundName, "speed-up x", medianSum * copies / medianRound);
    BenchRows.Add("parallel", roundName, "alloc MB", roundBytes / 1048576.0);
    BenchRows.Add("parallel", roundName, "collector's pause in a round ms", roundPauses.Order().ElementAt(1));
    BenchRows.Add("parallel", roundName, "processor time in a round ms", roundCpu.Order().ElementAt(1));
    BenchRows.Add("parallel", roundName, "contended locks in a round count", roundContentions.Order().ElementAt(1));
    BenchRows.Add("parallel", roundName, "peak working set in MB, count", peakMB);
    BenchRows.Add("parallel", roundName, "result", $"{mismatches} differing from the serial run");
}
BenchRows.Add("process", copies > 0 ? "with the parallel rounds" : "the serial passes", "collector's pause ms", GC.GetTotalPauseDuration().TotalMilliseconds);
BenchRows.Add("process", copies > 0 ? "with the parallel rounds" : "the serial passes", "generation 0 count", GC.CollectionCount(0));
BenchRows.Add("process", copies > 0 ? "with the parallel rounds" : "the serial passes", "generation 2 count", GC.CollectionCount(2));
Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
    $"GC: {GC.CollectionCount(0)}/{GC.CollectionCount(1)}/{GC.CollectionCount(2)} collections, pause {GC.GetTotalPauseDuration().TotalMilliseconds:F0} ms; JIT {System.Runtime.JitInfo.GetCompilationTime().TotalMilliseconds:F0} ms"));
Marks.Log?.Mark("end");
return 0;

// Every [ThreadStatic] reference in Kerf's assemblies, dropped on every thread that could hold it: the pool's, and the caller's, which
// takes part in a Parallel.For.
internal static class ThreadSets
{
    public static void Report(long liveAtStart)
    {
        var fields = (from assembly in AppDomain.CurrentDomain.GetAssemblies()
                      where assembly.GetName().Name!.StartsWith("Kerf.", StringComparison.Ordinal)
                      from type in assembly.GetTypes()
                      where !type.ContainsGenericParameters
                      from field in type.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic)
                      where field.IsDefined(typeof(ThreadStaticAttribute), false) && !field.FieldType.IsValueType
                      select field).ToArray();
        long liveBefore = GC.GetTotalMemory(true);
        var info = GC.GetGCMemoryInfo(GCKind.FullBlocking);
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{ThreadPool.ThreadCount} pool threads and the caller; live {liveAtStart / 1048576.0:F2} MB at the start, {liveBefore / 1048576.0:F2} MB after the rounds; committed {info.TotalCommittedBytes / 1048576.0:F1} MB; peak working set {Process.GetCurrentProcess().PeakWorkingSet64 / 1048576.0:F0} MB"));
        Console.WriteLine("field                                                   threads holding  MB in all  KB a thread");
        long previous = liveBefore;
        foreach (FieldInfo field in fields)
        {
            int holding = OnEveryThread(() =>
            {
                bool held = field.GetValue(null) is not null;
                field.SetValue(null, null);
                return held;
            });
            long now = GC.GetTotalMemory(true);
            double freed = previous - now;
            previous = now;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{field.DeclaringType!.FullName + "." + field.Name,-55} {holding,15} {freed / 1048576.0,10:F2} {(holding > 0 ? freed / 1024.0 / holding : 0.0),12:F1}"));
        }
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{fields.Length} fields; all of them {(liveBefore - previous) / 1048576.0:F2} MB; {previous / 1048576.0:F2} MB live without them"));
    }

    // Each work item waits at the barrier for all the others, so each runs on a thread of its own; a Task could be inlined in the caller
    private static int OnEveryThread(Func<bool> action)
    {
        int threads = ThreadPool.ThreadCount;
        ThreadPool.SetMinThreads(threads, threads);
        using var barrier = new Barrier(threads);
        using var finished = new CountdownEvent(threads);
        int holding = action() ? 1 : 0;
        for (int i = 0; i < threads; i++)
        {
            ThreadPool.QueueUserWorkItem(_ =>
            {
                barrier.SignalAndWait();
                if (action())
                {
                    Interlocked.Increment(ref holding);
                }
                finished.Signal();
            });
        }
        finished.Wait();
        return holding;
    }
}

// Marks in an EventPipe trace, so tools/opening-report.cs can cut it at each example. Created only when asked for, so the rig's
// runs don't pay for it.
[System.Diagnostics.Tracing.EventSource(Name = "Kerf-Bench")]
internal sealed class Marks : System.Diagnostics.Tracing.EventSource
{
    public static readonly Marks? Log = Environment.GetEnvironmentVariable("KERF_BENCH_MARKS") is null ? null : new Marks();

    [System.Diagnostics.Tracing.Event(1)]
    public void Mark(string name) => WriteEvent(1, name);
}
