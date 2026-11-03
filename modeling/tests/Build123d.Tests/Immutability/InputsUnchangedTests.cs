using System.Collections.Concurrent;
using System.Reflection;
using Build123d.Occt;
using Ops = Build123d.Operations;

namespace Build123d.Tests.Immutability;

/// <summary>
/// No operation changes its inputs, not even the OCCT data they share with other shapes (tolerances, meshes,
/// pcurves), whether run alone or from several threads at once: a shape is a value, as Bento relies on.
/// </summary>
public class InputsUnchangedTests
{
    /// <summary>An operation on some inputs.</summary>
    public sealed record Case(string Name, Shape[] Inputs, Func<object?> Run)
    {
        public override string ToString() => Name;
    }

    // Inputs, built once and shared by every case (moved copies share their TShape with the original)
    private static readonly Box Box = new(10, 10, 10);
    private static readonly Box MovedBox = Box.Moved(new Location((3.0, 3.0, 3.0)));
    private static readonly Cylinder Cylinder = new(3, 14);
    private static readonly Sphere Sphere = new(4);
    private static readonly Face Plate = Face.MakeRect(10, 6);
    private static readonly Circle Circle = new(2);
    private static readonly Wire RectWire = Wire.MakeRect(8, 4);
    private static readonly Edge Spline = Edge.MakeSpline([(0.0, 0.0), (3.0, 2.0), (6.0, -1.0), (9.0, 1.0)]);
    private static readonly Edge Path = Edge.MakeThreePointArc((0, 0, 0), (5, 0, 5), (10, 0, 0));
    private static readonly Edge ArcEdge = Edge.MakeCircle(3);
    private static readonly Edge LineEdge = Edge.MakeLine((-5, 1, 0), (5, 1, 0));
    private static readonly Part Fused = (Part)(Box + Cylinder);
    private static readonly SlotOverall Slot = new(10, 4);

    public static TheoryData<Case> Cases()
    {
        var data = new TheoryData<Case>();
        foreach (var c in AllCases())
            data.Add(c);
        return data;
    }

    private static List<Case> AllCases()
    {
        Case C(string name, Shape[] inputs, Func<object?> run) => new(name, inputs, run);
        var cases = new List<Case>
        {
            // Booleans
            C("cut", [Box, Cylinder], () => Box - Cylinder),
            C("fuse", [Box, Cylinder], () => Box + Cylinder),
            C("common", [Box, Sphere], () => Box & Sphere),
            C("cut shared", [Box, MovedBox], () => Box - MovedBox),
            C("fuse glue", [Box, MovedBox], () => Box.Fuse([MovedBox], glue: true)),
            C("intersect axis", [Box], () => Box.Intersect(Axis.Z)),
            C("intersect edge", [ArcEdge, LineEdge], () => ArcEdge.Intersect(LineEdge)),
            C("split plane", [Box], () => Box.Split(Plane.XY)),
            C("split both", [Sphere], () => Sphere.SplitBoth(Plane.XZ)),
            C("split by face", [Box, Plate], () => Box.Split(Plate.Moved(new Location((0.0, 0.0, 0.0), (90.0, 0.0, 0.0))))),
            C("touch", [Box, MovedBox], () => ((Solid)Box.Solids()[0]).Touch(MovedBox)),
            C("section", [Box], () => Ops.Section(Box)),
            // Features
            C("fillet", [Box], () => Ops.Fillet(Box.Edges(), 1)),
            C("fillet top", [Fused], () => Ops.Fillet(Fused.Edges().GroupBy(Axis.Z).Last, 1)),
            C("chamfer", [Box], () => Ops.Chamfer(Box.Edges().GroupBy(Axis.Z).Last, 1)),
            C("offset", [Box], () => Ops.Offset(Box, 1)),
            C("shell", [Box], () => Ops.Offset(Box, -1, openings: [Box.Faces().SortBy(Axis.Z).Last])),
            C("hollow", [Box], () => ((Solid)Box.Solids()[0]).Hollow(Box.Faces().SortBy(Axis.Z)[^1..], 1)),
            C("draft", [Box], () => Ops.Draft(Box.Faces().FilterBy(Axis.Z, reverse: true), Plane.XY.Offset(-5), 5)),
            C("mirror", [Box], () => Ops.Mirror(Box, Plane.YZ.Offset(5))),
            C("scale", [Box], () => Ops.Scale(Box, 2)),
            C("scale xyz", [Sphere], () => Ops.Scale(Sphere, (1.0, 2.0, 3.0))),
            // Sketch to part
            C("extrude", [Plate], () => Ops.Extrude(Plate, 5)),
            C("extrude taper", [Circle], () => Ops.Extrude(Circle, 5, taper: 10)),
            C("extrude until", [Plate, Sphere], () => Ops.Extrude(Plate.Moved(new Location((0.0, 0.0, -10.0))), dir: (0, 0, 1), until: Until.Next, target: Sphere)),
            C("revolve", [Plate], () => Ops.Revolve(Plate.Moved(new Location((10.0, 0.0, 0.0))), Axis.Y, 180)),
            C("loft", [Circle, Plate], () => Ops.Loft([Circle, Plate.Moved(new Location((0.0, 0.0, 5.0)))])),
            C("sweep", [Circle, Path], () => Ops.Sweep(Circle.Moved(new Location(Path.LocationAt(0))), Path)),
            C("thicken", [Plate], () => Ops.Thicken(Plate, 1)),
            C("make face", [RectWire], () => Ops.MakeFace(RectWire.Edges())),
            C("make hull", [ArcEdge, LineEdge], () => Ops.MakeHull([ArcEdge, LineEdge])),
            C("trace", [Spline], () => Ops.Trace([Spline], 0.5)),
            C("full round", [Slot], () => Ops.FullRound(Slot.Edges().SortBy(Axis.X)[0])),
            // 2D
            C("face fillet 2d", [Plate], () => Plate.Fillet2D(1, Plate.Vertices())),
            C("face chamfer 2d", [Plate], () => Plate.Chamfer2D(1, 1, Plate.Vertices())),
            C("wire fillet 2d", [RectWire], () => RectWire.Fillet2D(1, RectWire.Vertices())),
            C("wire offset 2d", [RectWire], () => RectWire.Offset2D(1)),
            C("constrained arcs", [ArcEdge, LineEdge], () => Edge.MakeConstrainedArcs(ArcEdge, LineEdge, 1.0)),
            C("constrained lines", [ArcEdge], () => Edge.MakeConstrainedLines(ArcEdge, new Vector(6, 6))),
            C("fillet polyline", [], () => new FilletPolyline([(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)], 1)),
            C("rounded rectangle", [], () => new RectangleRounded(6, 4, 1)),
            // Curves
            C("trim to other", [Spline, LineEdge], () => Spline.TrimToOther(LineEdge)),
            C("project to shape", [ArcEdge, Box], () => ArcEdge.Moved(new Location((5.0, 5.0, 20.0))).ProjectToShape(Box, new Vector(0, 0, -1))),
            C("find intersections", [Spline, LineEdge], () => Spline.FindIntersectionPoints(LineEdge)),
            // Measures, meshes and the rest
            C("bounding box", [Box], () => Box.BoundingBox()),
            C("bounding box loose", [Sphere], () => Sphere.BoundingBox(optimal: false)),
            C("tessellate", [Box], () => Box.Tessellate(0.1)),
            C("tessellate moved", [MovedBox], () => MovedBox.Tessellate(0.01)),
            C("triangulate", [Fused], () => Fused.Triangulate(0.01)),
            C("triangulate free edge", [Spline], () => Spline.Triangulate(0.01)),
            C("triangulate tree", [Box, MovedBox], () => Compound.Assembly([Box, MovedBox]).TriangulateTree()),
            C("selectors by name", [Fused], () => Fused.Faces().SortBy("Area").GroupBy("GeomType").Count + Fused.Edges().FilterBy("IsClosed").Count),
            C("selector for a pick", [Fused], () => Selector.For(Fused, Fused.Faces()[0])),
            C("distance", [Box, Sphere], () => Box.DistanceToWithClosestPoints(Sphere.Moved(new Location((20.0, 0.0, 0.0))))),
            C("surface intersections", [Sphere], () => Sphere.FindIntersectionPoints(Axis.X)),
            C("clean", [Fused], () => Fused.Clean()),
            C("fix", [Fused], () => Fused.Fix()),
            C("is valid", [Fused], () => Fused.IsValid),
            C("show topology", [Box], () => Box.ShowTopology()),
            C("is inside", [Sphere], () => ((Solid)Sphere.Solids()[0]).IsInside(new Vector(1, 1, 1))),
            // Import and export
            C("export brep", [Fused], () => Exporters3D.ExportBrep(Fused, new MemoryStream())),
            C("export step", [Fused, MovedBox], () => Exporters3D.ExportStep(Compound.Assembly([Fused, MovedBox]), new MemoryStream())),
        };
        // The mesh formats are OCCT.NET's (the native binding doesn't bind them)
        cases.AddRange(
        [
            C("export stl", [Fused], () => Exporters3D.ExportStl(Fused, new MemoryStream())),
            C("export gltf", [Fused, MovedBox], () => Exporters3D.ExportGltf(Compound.Assembly([Fused, MovedBox.WithColor(new Color("red"))]), new MemoryStream(), binary: true)),
            C("export 3mf", [Fused, Sphere], () => Exporters3D.Export3mf(Compound.Assembly([Fused, Sphere]), new MemoryStream())),
        ]);
        return cases;
    }

    /// <summary>
    /// Every input, as BRep text with its triangulations: any change to it, or to what it shares, shows. Two of each
    /// TShape's flags are left out, being OCCT's bookkeeping rather than the shape: Free, which OCCT clears on any shape
    /// put into another (every new compound does it), and Checked, a cache of a validity check.
    /// </summary>
    private static byte[][] Snapshot(Shape[] inputs) => [.. inputs.Select(s => WithoutBookkeeping(Extras.WriteBRep(s.TopoShape, withTriangles: true)))];

    private static readonly System.Text.RegularExpressions.Regex Flags = new(@"^[01]([01])[01]([01]{4})\r?$", System.Text.RegularExpressions.RegexOptions.Multiline);

    private static byte[] WithoutBookkeeping(byte[] brep) =>
        System.Text.Encoding.ASCII.GetBytes(Flags.Replace(System.Text.Encoding.ASCII.GetString(brep), "*$1*$2"));

    private static void AssertUnchanged(string name, Shape[] inputs, byte[][] before)
    {
        var after = Snapshot(inputs);
        for (int i = 0; i < inputs.Length; i++)
            Assert.True(before[i].AsSpan().SequenceEqual(after[i]), $"{name} changed its input {i} ({inputs[i].GetType().Name}): {FirstDifference(before[i], after[i])}");
    }

    /// <summary>The first line of BRep text that differs, with the section it's in.</summary>
    private static string FirstDifference(byte[] before, byte[] after)
    {
        var a = System.Text.Encoding.ASCII.GetString(before).Split('\n');
        var b = System.Text.Encoding.ASCII.GetString(after).Split('\n');
        string section = "";
        for (int i = 0; i < Math.Min(a.Length, b.Length); i++)
        {
            if (a[i].Length > 0 && char.IsLetter(a[i][0]))
                section = a[i].Trim();
            if (a[i] != b[i])
                return $"line {i} in '{section}': '{a[i].Trim()}' became '{b[i].Trim()}'";
        }
        return $"{a.Length} lines became {b.Length}";
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void AnOperationLeavesItsInputsAlone(Case c)
    {
        var before = Snapshot(c.Inputs);
        try
        {
            c.Run();
        }
        catch (Exception e) when (e is ArgumentException or InvalidOperationException or OcctException)
        {
            // Failing is allowed; changing the inputs isn't
        }
        AssertUnchanged(c.Name, c.Inputs, before);
    }

    /// <summary>Every public member that needs no arguments, on each kind of shape.</summary>
    [Fact]
    public void NoParameterlessMemberChangesItsShape()
    {
        Shape[] shapes = [Box, MovedBox, Fused, Plate, Circle, RectWire, Spline, ArcEdge, new Vertex(1, 2, 3), Box.Shells()[0], Slot];
        var failures = new List<string>();
        foreach (var shape in shapes)
        {
            var before = Snapshot([shape]);
            var members = shape.GetType().GetMethods(BindingFlags.Public | BindingFlags.Instance)
                .Where(m => !m.IsSpecialName || m.Name.StartsWith("get_"))
                .Where(m => !m.ContainsGenericParameters && m.ReturnType != typeof(void) && m.GetParameters().All(p => p.IsOptional))
                .Where(m => m.Name is not ("GetType" or "GetHashCode" or "ToString" or "Equals" or "MemberwiseClone" or "get_Wrapped"));
            foreach (var member in members)
            {
                try
                {
                    member.Invoke(shape, [.. member.GetParameters().Select(p => p.DefaultValue)]);
                }
                catch (TargetInvocationException)
                {
                    // Failing is allowed
                }
                if (!before[0].AsSpan().SequenceEqual(Snapshot([shape])[0]))
                {
                    failures.Add($"{shape.GetType().Name}.{member.Name} ({FirstDifference(before[0], Snapshot([shape])[0])})");
                    before = Snapshot([shape]);
                }
            }
        }
        Assert.True(failures.Count == 0, "changed their shape: " + string.Join(", ", failures));
    }

    /// <summary>
    /// Every case at once, several times over, on the same inputs: each gives what it gives alone (or fails as it does
    /// alone), and the inputs are unchanged.
    /// </summary>
    [Fact]
    public void OperationsOnSharedInputsRunConcurrently()
    {
        var cases = AllCases();
        var inputs = cases.SelectMany(c => c.Inputs).Distinct().ToArray();
        var before = Snapshot(inputs);
        var expected = cases.ToDictionary(c => c.Name, c => Summary(c.Run));

        var differences = new ConcurrentBag<string>();
        Parallel.For(0, cases.Count * 6, new ParallelOptions { MaxDegreeOfParallelism = Math.Max(4, Environment.ProcessorCount) }, i =>
        {
            var c = cases[i % cases.Count];
            var summary = Summary(c.Run);
            if (summary != expected[c.Name])
                differences.Add($"{c.Name}: {summary} != {expected[c.Name]}");
        });
        Assert.True(differences.IsEmpty, string.Join("\n", differences.Distinct()));
        AssertUnchanged("the concurrent run", inputs, before);
    }

    /// <summary>What an operation gave, enough to tell results apart: a shape's measures, else the value, or the failure.</summary>
    private static string Summary(Func<object?> run)
    {
        try
        {
            return run() switch
            {
                Shape s when s.IsEmpty => "empty " + s.GetType().Name,
                Shape s => $"{s.GetType().Name} V={s.Volume:G9} A={Area(s):G9} E={s.Edges().Count} F={s.Faces().Count}",
                System.Collections.IEnumerable list and not string => $"{list.Cast<object>().Count()} items",
                var value => value?.ToString() ?? "null",
            };
        }
        catch (Exception e) when (e is ArgumentException or InvalidOperationException or OcctException)
        {
            return e.GetType().Name + ": " + e.Message;
        }
    }

    private static double Area(Shape s) => s.Faces().Sum(f => f.Area);
}
