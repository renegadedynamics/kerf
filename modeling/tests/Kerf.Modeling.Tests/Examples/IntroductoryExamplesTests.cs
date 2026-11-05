using static Kerf.Modeling.Operations;
using Rot = Kerf.Modeling.Rotation;

namespace Build123d.Tests.Examples;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// build123d's introductory examples (https://build123d.readthedocs.io/en/latest/introductory_examples.html), their
/// algebra-mode forms (docs/general_examples_algebra.py) in C#: all but 34, which needs text. Each result's volume and
/// face count are native OCCT's, recorded here, so the same test checks the OCCT.NET backend against native. The seven
/// that need OCCT.NET's phase 7 (3D fillets and chamfers, offsets, extrusion until: 9, 10, 11, 18, 26, 29, 36) were
/// added with it.
/// </summary>
public class IntroductoryExamplesTests
{
    // Native OCCT's results (Build123dBackend=Native): volume and face count
    private static readonly Dictionary<int, (double Volume, int Faces)> Native = new()
    {
        [1] = (48000, 6),
        [2] = (44198.67288915635, 7),
        [3] = (202194.6710584651, 7),
        [4] = (69504.6716490322, 6),
        [5] = (375677.7886096935, 8),
        [6] = (188495.55921538753, 7),
        [7] = (21401.923788646673, 18),
        [8] = (5799.999999999997, 14),
        [9] = (45706.9022894405, 14),
        [10] = (40848.10405858136, 8),
        [11] = (36177.365057283976, 36),
        [12] = (14627.106295339287, 6),
        [13] = (70872.25969468076, 23),
        [14] = (91398.2236861551, 10),
        [15] = (79999.99999999999, 10),
        [16] = (1297854.872805332, 75),
        [17] = (85595.08646656378, 10),
        [18] = (44706.90228944052, 18),
        [19] = (41538.56826564553, 11),
        [20] = (123398.22368615503, 9),
        [21] = (9091.444627430465, 5),
        [22] = (46778.13736363046, 10),
        [23] = (88619.09277001217, 7),
        [24] = (102969.87958520795, 12),
        [25] = (24387.59273282052, 26),
        [26] = (13952, 11),
        [27] = (20465.708264711477, 8),
        [28] = (251188.19571967638, 7),
        [29] = (15893.314536718122, 56),
        [30] = (64463.381199800904, 9),
        [31] = (4991.970032881474, 164),
        [32] = (14839.230484541327, 44),
        [33] = (10840, 36),
        [35] = (49219.02754903829, 18),
        [36] = (30298.935241110394, 8),
        [37] = (5.53429173528852, 8),
        [38] = (2304, 18),
    };

    public static TheoryData<int> Numbers =>
        [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
         33, 35, 36, 37, 38];

    [Theory]
    [MemberData(nameof(Numbers))]
    public void MatchesNative(int number)
    {
        var shape = Build(number);
        var (volume, faces) = Native[number];
        Assert.True(shape.IsValid, $"example {number} isn't valid");
        Assert.Equal(faces, shape.SelectFaces().Count);
        Assert.Equal(volume, shape.Volume, volume * 1e-9);
    }

    internal static Shape Build(int number) => number switch
    {
        1 => Ex1(), 2 => Ex2(), 3 => Ex3(), 4 => Ex4(), 5 => Ex5(), 6 => Ex6(), 7 => Ex7(), 8 => Ex8(),
        9 => Ex9(), 10 => Ex10(), 11 => Ex11(), 12 => Ex12(), 13 => Ex13(), 14 => Ex14(), 15 => Ex15(),
        16 => Ex16(), 17 => Ex17(), 18 => Ex18(), 19 => Ex19(), 20 => Ex20(), 21 => Ex21(), 22 => Ex22(),
        23 => Ex23(), 24 => Ex24(), 25 => Ex25(), 26 => Ex26(), 27 => Ex27(), 28 => Ex28(), 29 => Ex29(),
        30 => Ex30(), 31 => Ex31(), 32 => Ex32(), 33 => Ex33(), 35 => Ex35(), 36 => Ex36(), 37 => Ex37(),
        38 => Ex38(),
        _ => throw new ArgumentOutOfRangeException(nameof(number)),
    };

    private const double Length = 80.0, Width = 60.0, Thickness = 10.0;

    // 1. Simple rectangular plate
    private static Shape Ex1() => new Box(Length, Width, Thickness);

    // 2. Plate with hole
    private static Shape Ex2() => new Box(Length, Width, Thickness) - new Cylinder(22.0 / 2, Thickness);

    // 3. An extruded prismatic solid
    private static Shape Ex3() => Extrude(new Circle(Width) - new Rectangle(Length / 2, Width / 2), 2 * Thickness).Value;

    // 4. Building profiles using lines and arcs
    private static Shape Ex4()
    {
        var lines = new Curve().Add([
            new Line(new Point3D(0, 0, 0), new Point3D(Length, 0, 0)),
            new Line(new Point3D(Length, 0, 0), new Point3D(Length, Width, 0)),
            new ThreePointArc(new Point3D(Length, Width, 0), new Point3D(Width, Width * 1.5, 0), new Point3D(0.0, Width, 0)),
            new Line(new Point3D(0.0, Width, 0), new Point3D(0, 0, 0)),
        ]).Value;
        return Extrude(MakeFace(lines), Thickness).Value;
    }

    // 5. Moving the current working point
    private static Shape Ex5()
    {
        double a = 90, b = 45, c = 15, d = 7.5;
        var sk5 = new Circle(a) - new Pos(b, 0.0) * new Rectangle(c, c) - new Pos(0.0, b) * new Circle(d);
        return Extrude(sk5, c).Value;
    }

    // 6. Using point lists
    private static Shape Ex6()
    {
        double a = 80, b = 60, c = 10;
        var sk6 = new Locations(new Point3D(b, 0, 0), new Point3D(0, b, 0), new Point3D(-b, 0, 0), new Point3D(0, -b, 0)).Select(loc => (Shape)(loc * new Circle(c)));
        return Extrude(new Circle(a) - sk6, c).Value;
    }

    // 7. Polygons
    private static Shape Ex7()
    {
        double a = 60, b = 80, c = 5;
        var polygons = new Locations(new Point3D(0, 3 * c, 0), new Point3D(0, -3 * c, 0)).Select(loc => (Shape)(loc * new RegularPolygon(2 * c, 6)));
        return Extrude(new Rectangle(a, b) - polygons, c).Value;
    }

    // 8. Polylines
    private static Shape Ex8()
    {
        double l = 100.0, h = 20.0, w = 20.0, t = 1.0;
        Point3D[] pts =
        [
            new Point3D(0, h / 2.0, 0), new Point3D(w / 2.0, h / 2.0, 0), new Point3D(w / 2.0, h / 2.0 - t, 0), new Point3D(t / 2.0, h / 2.0 - t, 0),
            new Point3D(t / 2.0, t - h / 2.0, 0), new Point3D(w / 2.0, t - h / 2.0, 0), new Point3D(w / 2.0, h / -2.0, 0), new Point3D(0, h / -2.0, 0),
        ];
        Shape ln = new Polyline(pts);
        ln += Mirror(ln, Plane.YZ);
        var sk8 = MakeFace(Plane.YZ * ln);
        return Extrude(sk8, -l).Value.Clean();
    }

    // 9. Selectors, fillets and chamfers
    private static Shape Ex9()
    {
        Shape ex9 = new Box(Length, Width, Thickness);
        ex9 = Chamfer(ex9.SelectEdges().GroupBy(Axis.Z)[^1], length: 4).Value;
        return Fillet(ex9.SelectEdges().ParallelTo(Axis.Z), radius: 5).Value;
    }

    // 10. Select last edges and hole
    private static Shape Ex10()
    {
        Shape ex10 = new Box(Length, Width, Thickness);
        var snapshot = ex10.SelectEdges();
        ex10 -= new Hole(Width / 4, depth: Thickness);
        var lastEdges = ex10.SelectEdges().Except(snapshot);
        return Fillet(lastEdges.GroupBy(Axis.Z)[^1], 2).Value;
    }

    // 11. A face as a workplane, grid locations
    private static Shape Ex11()
    {
        Shape ex11 = new Box(Length, Width, Thickness);
        ex11 = Chamfer(ex11.SelectEdges().GroupBy(Axis.Z)[^1], 4).Value;
        ex11 = Fillet(ex11.SelectEdges().ParallelTo(Axis.Z), 5).Value;
        var last = ex11.SelectEdges();
        ex11 -= new Hole(Width / 4, depth: Thickness);
        ex11 = Fillet(ex11, ex11.SelectEdges().Except(last).OrderBy(Axis.Z).Last(), 2).Value;
        var plane = new Plane(ex11.SelectFaces().OrderBy(Axis.Z).Last());
        var polygons = new Sketch().Add(new GridLocations(Length / 2, Width / 2, 2, 2)
            .Select(loc => (Shape)(plane * loc * new RegularPolygon(5, 5)))).Value;
        ex11 -= Extrude(polygons, -Thickness).Value;
        return ex11;
    }

    // 12. Defining an edge with a spline
    private static Shape Ex12()
    {
        var l1 = new Spline(new Point3D(55, 30, 0), new Point3D(50, 35, 0), new Point3D(40, 30, 0), new Point3D(30, 20, 0), new Point3D(20, 25, 0), new Point3D(10, 20, 0), new Point3D(0, 20, 0));
        var l2 = new Line(l1.PositionAt(0), new Point3D(60, 0, 0));
        var l3 = new Line(l2.PositionAt(1), new Point3D(0, 0, 0));
        var l4 = new Line(l3.PositionAt(1), l1.PositionAt(1));
        return Extrude(MakeFace([l1, l2, l3, l4]), 10).Value;
    }

    // 13. Counterbore and countersink holes, polar locations
    private static Shape Ex13()
    {
        double a = 40, b = 4;
        Shape ex13 = new Cylinder(50, 10);
        var plane = new Plane(ex13.SelectFaces().OrderBy(Axis.Z).Last());
        ex13 -= (plane * new PolarLocations(a, 4)).Select(loc => (Shape)(loc * new CounterSinkHole(b, 2 * b, depth: 10)));
        ex13 -= (plane * new PolarLocations(a, 4, startAngle: 45, angularRange: 360))
            .Select(loc => (Shape)(loc * new CounterBoreHole(b, 2 * b, b, depth: 10)));
        return ex13;
    }

    // 14. Position on a line with @ and %, sweep
    private static Shape Ex14()
    {
        double a = 40, b = 20;
        var l1 = new JernArc(new Point3D(0, 0, 0), new Vector3D(0, 1, 0), a, 180);
        var l2 = new JernArc(l1.PositionAt(1), l1.TangentAt(1), a, -90);
        var l3 = new Line(l2.PositionAt(1), l2.PositionAt(1) + new Vector3D(-a, a, 0));
        var path = l1 + l2 + l3;
        var sk14 = Plane.XZ * new Rectangle(b, b);
        return Sweep(sk14, path).Value;
    }

    // 15. Mirroring symmetric geometry
    private static Shape Ex15()
    {
        double a = 80, b = 40, c = 20;
        var l1 = new Line(new Point3D(0, 0, 0), new Point3D(a, 0, 0));
        var l2 = new Line(l1.PositionAt(1), l1.PositionAt(1) + new Vector3D(0, b, 0));
        var l3 = new Line(l2.PositionAt(1), l2.PositionAt(1) + new Vector3D(-c, 0, 0));
        var l4 = new Line(l3.PositionAt(1), l3.PositionAt(1) + new Vector3D(0, -c, 0));
        var l5 = new Line(l4.PositionAt(1), new Point3D(0, l4.PositionAt(1).Y, 0));
        Shape ln = new Curve().Add([l1, l2, l3, l4, l5]).Value;
        ln += Mirror(ln, Plane.YZ);
        return Extrude(MakeFace(ln), c).Value;
    }

    // 16. Mirroring 3D objects
    private static Shape Ex16()
    {
        Shape sk16 = new Rectangle(Length, Width);
        sk16 = Fillet(sk16.SelectVertices(), Length / 10).Value;
        var circles = new GridLocations(Length / 4, 0, 3, 1).Select(loc => (Shape)(loc * new Circle(Length / 12)));
        sk16 = sk16 - circles - new Rectangle(Length, Width, align: new Alignment(Align.Min, Align.Min));
        var single = Extrude(Plane.XZ * sk16, Length).Value;
        Plane[] planes = [Plane.XY.Offset(Width), Plane.YX.Offset(Width), Plane.YZ.Offset(Width), Plane.YZ.Offset(-Width)];
        return single + planes.Select(p => (Shape)Mirror(single, p));
    }

    // 17. Mirroring from faces
    private static Shape Ex17()
    {
        double a = 30, b = 20;
        Shape ex17 = Extrude(new RegularPolygon(a, 5), b).Value;
        ex17 += Mirror(ex17, new Plane(ex17.SelectFaces().OrderBy(Axis.Y).First()));
        return ex17;
    }

    // 18. Creating workplanes on faces
    private static Shape Ex18()
    {
        double a = 4, b = 5;
        Shape ex18 = new Box(Length, Width, Thickness);
        ex18 = Chamfer(ex18.SelectEdges().GroupBy(Axis.Z)[^1], a).Value;
        ex18 = Fillet(ex18.SelectEdges().ParallelTo(Axis.Z), b).Value;
        var sk18 = new Plane(ex18.SelectFaces().OrderBy(Axis.Z).First()) * new Rectangle(2 * b, 2 * b);
        ex18 -= Extrude(sk18, -Thickness).Value;
        return ex18;
    }

    // 19. Locating a workplane on a vertex
    private static Shape Ex19()
    {
        double length = 80.0, thickness = 10.0;
        Shape ex19 = Extrude(new RegularPolygon(length / 2, 7), thickness).Value;
        var topf = ex19.SelectFaces().OrderBy(Axis.Z).Last();
        var vtx = topf.SelectVertices().GroupBy(Axis.X)[^1][0];
        var vtx2 = topf.SelectVertices().OrderBy(new Axis(new Point3D(0, 0, 0), new Vector3D(-1, -0.5, 0)))[^1];
        var circle = new Circle(length / 8);
        var sk2 = new Pos(vtx.X, vtx.Y) * circle + new Pos(vtx2.X, vtx2.Y) * circle;
        ex19 -= Extrude(sk2, thickness).Value;
        return ex19;
    }

    // 20. Offset sketch workplane
    private static Shape Ex20()
    {
        Shape ex20 = new Box(Length, Width, Thickness);
        var plane = new Plane(ex20.SelectFaces().OrderBy(Axis.X).First()).Offset(2 * Thickness);
        ex20 += Extrude(plane * new Circle(Width / 3), Width).Value;
        return ex20;
    }

    // 21. Copying workplanes
    private static Shape Ex21()
    {
        double width = 10.0, length = 60.0;
        Shape ex21 = Extrude(new Circle(width / 2), length).Value;
        var plane = new Plane(ex21.Center(), zDir: new Vector3D(-1, 0, 0));
        ex21 += plane * Extrude(new Circle(width / 2), length).Value;
        return ex21;
    }

    // 22. Rotated workplanes
    private static Shape Ex22()
    {
        Shape ex22 = new Box(Length, Width, Thickness);
        var plane = new Plane(ex22.SelectFaces().GroupBy(Axis.Z)[0][0]) * new Rot(0, 50, 0);
        var holes = new Sketch().Add(new GridLocations(Length / 4, Width / 4, 2, 2)
            .Select(loc => (Shape)(new Location(plane) * loc * new Circle(Thickness / 4)))).Value;
        ex22 -= Extrude(holes, -100, new ExtrudeOptions { Both = true }).Value;
        return ex22;
    }

    // 23. Revolve
    private static Shape Ex23()
    {
        var l1 = new Polyline(new Point3D(-25, 35, 0), new Point3D(-25, 0, 0), new Point3D(-20, 0, 0), new Point3D(-20, 5, 0), new Point3D(-15, 10, 0), new Point3D(-15, 35, 0));
        var l2 = new Line(l1.PositionAt(1), l1.PositionAt(0));
        Shape sk23 = MakeFace([l1, l2]);
        sk23 += new Pos(0, 35) * new Circle(25);
        sk23 = Plane.XZ * Split(sk23, bisectBy: Plane.ZY);
        return Revolve(sk23, Axis.Z).Value;
    }

    // 24. Lofts
    private static Shape Ex24()
    {
        Shape ex24 = new Box(Length, Length, Thickness);
        var plane = new Plane(ex24.SelectFaces().OrderBy(Axis.Z).Last());
        var faces = new Sketch().Add([plane * new Circle(Length / 3), plane.Offset(Length / 2) * new Rectangle(Length / 6, Width / 6)]).Value;
        ex24 += Loft([faces]).Value;
        return ex24;
    }

    // 25. Offset sketch
    private static Shape Ex25()
    {
        double rad = 50, offs = 10;
        var sk1 = new RegularPolygon(rad, 5);
        var sk2 = Offset(Plane.XY.Offset(15) * new RegularPolygon(rad, 5), offs).Value;
        var sk3 = Offset(Plane.XY.Offset(30) * new RegularPolygon(rad, 5), offs, new OffsetOptions { Kind = Kind.Intersection }).Value;
        var sk25 = new Sketch().Add([sk1, sk2, sk3]).Value;
        return Extrude(sk25, 1).Value;
    }

    // 26. Offset part to create thin features
    private static Shape Ex26()
    {
        double wall = 2.0;
        Shape ex26 = new Box(Length, Width, Thickness);
        var topf = ex26.SelectFaces().OrderBy(Axis.Z).Last();
        return Offset(ex26, -wall, new OffsetOptions { Openings = [topf] }).Value;
    }

    // 27. Splitting an object
    private static Shape Ex27()
    {
        Shape ex27 = new Box(Length, Width, Thickness);
        var sk27 = new Plane(ex27.SelectFaces().OrderBy(Axis.Z).First()) * new Circle(Width / 4);
        ex27 -= Extrude(sk27, -Thickness).Value;
        return Split(ex27, new Plane(ex27.SelectFaces().OrderBy(Axis.Y).Last()).Offset(-Width / 2));
    }

    // 28. Locating features based on faces
    private static Shape Ex28()
    {
        double width = 80.0, thickness = 10.0;
        var tmp28 = Extrude(new RegularPolygon(width / 4, 3), thickness).Value;
        Shape ex28 = new Sphere(width / 2);
        foreach (var p in tmp28.Faces().GroupBy(Axis.Z)[1].Select(f => new Plane(f)))
            ex28 -= p * new Hole(thickness / 2, depth: width);
        return ex28;
    }

    // 29. The classic OCC bottle
    private static Shape Ex29()
    {
        double l = 60.0, w = 18.0, t = 9.0, b = 0.9, h = 90.0, n = 8.0;
        var l1 = new Line(new Point3D(0, 0, 0), new Point3D(0, w / 2, 0));
        var l2 = new ThreePointArc(l1.PositionAt(1), new Point3D(l / 2.0, w / 2.0 + t, 0), new Point3D(l, w / 2.0, 0));
        var l3 = new Line(l2.PositionAt(1), new Point3D(l2.PositionAt(1).X, 0, 0));
        Shape ln29 = l1 + l2 + l3;
        ln29 += Mirror(ln29);
        var sk29 = MakeFace(ln29);
        Shape ex29 = Extrude(sk29, -(h + b)).Value;
        ex29 = Fillet(ex29.SelectEdges(), radius: w / 6).Value;
        var neck = new Plane(ex29.SelectFaces().OrderBy(Axis.Z).Last()) * new Circle(t);
        ex29 += Extrude(neck, n).Value;
        var necktopf = ex29.SelectFaces().OrderBy(Axis.Z).Last();
        return Offset(ex29, -b, new OffsetOptions { Openings = [necktopf] }).Value;
    }

    // 30. Bezier curve
    private static Shape Ex30()
    {
        Point3D[] pts = [new Point3D(0, 0, 0), new Point3D(20, 20, 0), new Point3D(40, 0, 0), new Point3D(0, -40, 0), new Point3D(-60, 0, 0), new Point3D(0, 100, 0), new Point3D(100, 0, 0)];
        double[] wts = [1.0, 1.0, 2.0, 3.0, 4.0, 2.0, 1.0];
        var ln = new Polyline(pts) + new Bezier(pts, wts);
        return Extrude(MakeFace(ln), -10).Value;
    }

    // 31. Nesting locations
    private static Shape Ex31()
    {
        double a = 80.0, b = 5.0;
        Shape ex31 = new Rot(z: 30) * new RegularPolygon(3 * b, 6);
        var cell = new RegularPolygon(b, 4) + new GridLocations(3 * b, 3 * b, 2, 2).Select(loc => (Shape)(loc * new RegularPolygon(b, 3)));
        ex31 += new PolarLocations(a / 2, 6).Select(loc => (Shape)(loc * cell));
        return Extrude(ex31, 3).Value;
    }

    // 32. Python for-loop
    private static Shape Ex32()
    {
        double a = 80.0, b = 10.0, c = 1.0;
        Shape sk = new RegularPolygon(2 * b, 6, rotation: 30);
        sk += new PolarLocations(a / 2, 6).Select(loc => (Shape)(loc * new RegularPolygon(b, 4)));
        return new Part().Add(sk.SelectFaces().Select((face, idx) => (Shape)Extrude(face, c + 3 * idx).Value)).Value;
    }

    // 33. Python function and for-loop
    private static Shape Ex33()
    {
        double a = 80.0, b = 5.0, c = 1.0;
        static Shape Square(double rad, Location loc) => loc * new RegularPolygon(rad, 4);
        return new Part().Add(new PolarLocations(a / 2, 6).Select((loc, i) => (Shape)Extrude(Square(b + 2 * i, loc), c + 2 * i).Value)).Value;
    }

    // 35. Slots
    private static Shape Ex35()
    {
        Shape ex35 = new Box(Length, Length, Thickness);
        var plane = new Plane(ex35.SelectFaces().OrderBy(Axis.Z).Last());
        Shape sk = new SlotCenterToCenter(Width / 2, 10);
        var ln = new RadiusArc(new Point3D(-Width / 2, 0, 0), new Point3D(0, Width / 2, 0), Width / 2);
        sk += new SlotCurve(ln.Edges()[0], Thickness);
        var ln2 = new RadiusArc(new Point3D(0, -Width / 2, 0), new Point3D(Width / 2, 0, 0), -Width / 2);
        sk += new SlotCurve(ln2.Edges()[0], Thickness);
        ex35 -= Extrude(plane * sk, -Thickness).Value;
        return ex35;
    }

    // 36. Extrude until
    private static Shape Ex36()
    {
        double rad = 6, rev = 50;
        var sk36 = new Pos(0, rev) * new Circle(rad);
        Shape ex36 = Revolve(sk36, Axis.X, revolutionArc: 180).Value;
        var sk36b = new Rectangle(rad, rev);
        ex36 += Extrude(sk36b, Until.Next, ex36).Value;
        return ex36;
    }

    // 37. Positioning sketches within a plane (builder mode in build123d: the three sketches' faces, extruded)
    private static Shape Ex37()
    {
        var sk1 = new Rectangle(1, 2, align: new Alignment(Align.Center, Align.Min));
        var origin = sk1.Vertices().GroupBy(Axis.Y)[0].OrderBy(Axis.X)[0];
        var sk2 = Plane.XY.ShiftOrigin(origin) * new Circle(1);
        var sk3 = new Plane(new Point3D(0.5, 2, 0)) * new Ellipse(0.5, 1);
        return Extrude(new Sketch().Add([sk1, sk2, sk3]).Value, 1).Value;
    }

    // 38. Applying rotations
    private static Shape Ex38()
    {
        double length = 24, width = 8, height = 4, spacing = 30;
        var aboutAxis = new Box(length, width, height).Rotate(new Axis(new Point3D(-length / 2, -width / 2, 0), new Vector3D(0, 0, 1)), 20);
        var withConstructor = new Pos(y: spacing) * new Box(length, width, height, rotation: new Rotation(0, 0, 40));
        var withLocation = new Pos(y: 2 * spacing) * new Rot(z: 60) * new Box(length, width, height);
        return new Part().Add([aboutAxis, withConstructor, withLocation]).Value;
    }
}
