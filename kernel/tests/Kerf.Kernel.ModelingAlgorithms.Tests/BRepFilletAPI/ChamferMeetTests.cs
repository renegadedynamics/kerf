// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// Chamfers whose corner ChFi3d can't build, built with their faces carried on until they meet (unit 15U): the corpus's
/// shapes, each ChFi3d's failure first, then the public chamfer's result and its proof, read from the result alone.
/// </summary>
public class ChamferMeetTests
{
    // The tests of what the path builds run without the wall-clock limit, so a loaded machine (the gate's suites side by
    // side) can't turn a build into fillet.time-limit; the limit's own tests set theirs
    private static readonly ChamferOptions NoLimit = new() { TimeLimit = Timeout.InfiniteTimeSpan };

    private static Shape Read(string name)
    {
        using Stream stream = typeof(ChamferMeetTests).Assembly.GetManifestResourceStream($"BRepFilletAPI.Issues.{name}.brep")
                              ?? throw new InvalidOperationException(name);
        BrepTools.Read(out Shape shape, new StreamReader(stream));
        return shape;
    }

    private static Edge EdgeBetween(Shape shape, Point3D a, Point3D b) =>
        shape.AllEdges().Single(e => e.AllVertices().Length == 2
                                     && e.AllVertices().Any(v => v.Point.DistanceTo(a) < 1e-5)
                                     && e.AllVertices().Any(v => v.Point.DistanceTo(b) < 1e-5));

    // OCCT's 8_J1.draw: an open shell of three Bézier faces, a floor at about z 0.3 with a quadrant cut out, and two walls
    // standing on the cut's edges, meeting at a convex corner. The two concave edges where the walls stand are chamfered;
    // ChFi3d fails at the corner, where the chamfers wrap round the walls' corner and meet (chamfer/*_complex/B5 to B7)
    private static readonly Point3D Corner = new(-0.0722033, -0.0230087, 0.307218);

    private static (Shape Shell, Edge Along, Edge Across, Face Floor) EightJ1()
    {
        Shape shell = Read("chamfer-8_J1");
        Edge along = EdgeBetween(shell, new Point3D(-0.0752354, -0.514541, 0.322338), Corner);
        Edge across = EdgeBetween(shell, new Point3D(-0.606778, 0.0116437, 0.226719), Corner);
        Face floor = shell.AllFaces().Single(f => f.AllEdges().Any(e => e.IsSame(along)) && f.AllEdges().Any(e => e.IsSame(across)));
        return (shell, along, across, floor);
    }

    // The corpus's seven, six of them different: a distance and an angle, two distances, and one distance (equal_dist_complex/B7
    // is B5 again), the distances first on the floor
    public static TheoryData<string, double, double?, double?, double, double?, double?> Corners => new()
    {
        { "dist_angle_complex/B5", 0.1, null, 60, 0.06, null, 60 },
        { "dist_angle_complex/B6", 0.08, null, 60, 0.06, null, 60 },
        { "dist_angle_complex/B7", 0.1, null, 30, 0.08, null, 30 },
        { "dist_dist_complex/B6", 0.1, 0.2, null, 0.1, 0.2, null },
        { "dist_dist_complex/B7", 0.1, 0.11, null, 0.2, 0.21, null },
        { "equal_dist_complex/B5", 0.1, null, null, 0.2, null, null },
    };

    [Theory]
    [MemberData(nameof(Corners))]
    public void Two_chamfers_round_a_corner_of_an_open_shell_are_carried_on_until_they_meet(string corpusCase, double d1, double? e1, double? a1,
                                                                                       double d2, double? e2, double? a2)
    {
        (Shape shell, Edge along, Edge across, Face floor) = EightJ1();
        ChamferSpec[] specs =
        [
            Spec(along, d1, e1, a1, floor),
            Spec(across, d2, e2, a2, floor),
        ];

        // ChFi3d alone fails at the corner
        MakeChamfer native = Native(shell, specs);
        native.Build();
        Assert.False(native.IsDone, corpusCase);
        Assert.Equal(1, native.InternalBuilder().FaultyVertexCount);

        Shape made = shell.Chamfer(specs, NoLimit).Value.Shape;
        Assert.Equal(ShapeType.Shell, made.Type);
        Assert.True(made.Validate().IsValid, corpusCase);
        Assert.Equal(5, made.AllFaces().Length); // the three faces cut back, and the two chamfers
        // the proof, of the result alone: its free edges the input's or a chamfer's ends, the contacts at the specs
        Assert.True(ChamferMeet.Proved(shell, made, specs, [floor, floor], native, 1e-4), corpusCase);
        Assert.Null(EdgeCrossings.Problem(shell, made, "The chamfer's result", FilletOperations.SelfIntersecting));
    }

    // chamfer/equal_dist_sequence/B4, its third step: a chamfer of 0.2 whose end runs over the two chamfers of 0.05 made before
    // it and the face at the end; ChFi3d built it invalid, and since unit 15S fails at its corner
    [Fact]
    public void A_chamfer_whose_end_runs_over_three_faces_is_built()
    {
        Shape shape = Read("chamfer-B4-step3");
        Edge edge = EdgeBetween(shape, new Point3D(-0.0210811, -0.603933, -1.23162), new Point3D(-0.0206116, -0.587177, -0.554159));
        ChamferSpec[] specs = [new(edge, 0.2)];
        MakeChamfer native = Native(shape, specs);
        native.Build();
        Assert.False(native.IsDone);

        Shape made = shape.Chamfer(specs, NoLimit).Value.Shape;
        Assert.True(made.Validate().IsValid);
        Assert.Equal(6, made.AllFaces().Length);
        Face first = shape.AllFaces().First(f => f.AllEdges().Any(e => e.IsSame(edge)));
        Assert.True(ChamferMeet.Proved(shape, made, specs, [first], native, 1e-4));
    }

    // chamfer/dist_dist_sequence/B7 and equal_dist_sequence/B5, their second step on 8_J1: the first wall's foot was chamfered
    // in the first step, and now the second wall's foot, whose contact on the wall runs onto the walls' convex edge before
    // the corner, where the corner's other edges are one convex and one concave (decision 2144). ChFi3d fails; unit 15U2
    // builds it from the faces' formula there, floor or else (the wall or this chamfer) and (the other wall or its chamfer),
    // with the other wall and its chamfer carried on past their edges, so the new chamfer ends against both
    [Theory]
    [InlineData(0.2, 0.21)]
    [InlineData(0.2, null)]
    public void A_corner_of_convex_and_concave_edges_is_built_with_the_faces_beside_it_carried_on(double distance, double? second)
    {
        Shape shape = Read("chamfer-B5-step2");
        Edge edge = EdgeBetween(shape, new Point3D(-0.606778, 0.0116437, 0.226719), Corner);
        Face floor = shape.AllFaces().Single(f => f.AllVertices().Any(v => v.Point.DistanceTo(new Point3D(0.473048, 0.5374, 0.308257)) < 1e-5));
        ChamferSpec[] specs = [new ChamferSpec(edge, distance, second, ReferenceFace: second is null ? null : floor)];
        MakeChamfer native = Native(shape, specs);
        native.Build();
        Assert.False(native.IsDone);

        Result<Built<Shape>> made = shape.Chamfer(specs, NoLimit);
        Assert.True(made.IsSuccess, made.Error?.ToString());
        Shape result = made.Value.Shape;
        Assert.Equal(ShapeType.Shell, result.Type);
        Assert.True(result.Validate().IsValid);
        Assert.Equal(5, result.AllFaces().Length); // the four faces, two of them carried on, and the chamfer
        Assert.True(ChamferMeet.Proved(shape, result, specs, [floor], native, 1e-4));
        Assert.Null(EdgeCrossings.Problem(shape, result, "The chamfer's result", FilletOperations.SelfIntersecting));
        AssertCarriedOn(shape, result, edge);
    }

    // chamfer/dist_dist_sequence/B9 and equal_dist_sequence/B9, their second step on OCCT's 4_L1.draw: a tilted floor, two walls
    // meeting at a convex edge, the foot of one chamfered in the first step; now the other's foot, a chamfer of 1 on the floor
    // and 1.2 on the wall (B9) or 1.1 (equal), whose contact on the wall runs onto the walls' edge before the corner
    [Theory]
    [InlineData(1.0, 1.2)]
    [InlineData(1.1, null)]
    public void The_same_corner_on_4_L1_is_built(double distance, double? second)
    {
        Shape shape = Read("chamfer-4_L1-step2");
        Edge edge = EdgeBetween(shape, new Point3D(3.33333, 5, 0.5), new Point3D(3.33333, 10, 1));
        Face floor = shape.AllFaces().Single(f => f.AllVertices().Any(v => v.Point.DistanceTo(new Point3D(10, 0, 0)) < 1e-5));
        ChamferSpec[] specs = [new ChamferSpec(edge, distance, second, ReferenceFace: second is null ? null : floor)];
        MakeChamfer native = Native(shape, specs);
        native.Build();
        Assert.False(native.IsDone);

        Result<Built<Shape>> made = shape.Chamfer(specs, NoLimit);
        Assert.True(made.IsSuccess, made.Error?.ToString());
        Shape result = made.Value.Shape;
        Assert.True(result.Validate().IsValid);
        Assert.Equal(5, result.AllFaces().Length);
        Assert.True(ChamferMeet.Proved(shape, result, specs, [floor], native, 1e-4));
        Assert.Null(EdgeCrossings.Problem(shape, result, "The chamfer's result", FilletOperations.SelfIntersecting));
        AssertCarriedOn(shape, result, edge);
    }

    // The two faces at the corner that aren't the chamfered edge's are carried on past their edges: each is one face of the
    // result, on a surface through all of the input face's vertices, and larger than in the input, as the chamfer ends against it
    private static void AssertCarriedOn(Shape input, Shape result, Edge edge)
    {
        Face[] beside = [.. input.AllFaces().Where(f => !f.AllEdges().Any(e => e.IsSame(edge)))];
        Assert.Equal(2, beside.Length);
        foreach (Face face in beside)
        {
            Face[] same = [.. result.AllFaces().Where(f => face.AllVertices().All(v => OnSurface(f, v.Point)))];
            Assert.Single(same);
            Assert.True(same[0].IntegrateArea() > face.IntegrateArea() * (1 + 1e-4), $"{same[0].IntegrateArea()} against {face.IntegrateArea()}");
        }
    }

    private static bool OnSurface(Face face, Point3D p)
    {
        var projection = new ProjectPointOnSurface(p, Tool.Surface(face)!);
        return projection.NbPoints() > 0 && projection.LowerDistance() < 1e-6;
    }

    // chamfer/dist_dist_sequence/A5, its failing step: a solid of 39 faces, mostly planes, an edge between two of them chamfered
    // by 0.03 and 0.06. ChFiKPart makes the chamfer's stripes planes, and at the edge's foot, where an earlier chamfer's face
    // rises out of one of the two across a concave edge, ChFi3d fails. Unit 15U3 takes the planes as patches over their
    // contacts, builds the foot from its faces' formula, and proves the solid by its volume, the foot's box by that formula
    [Fact]
    public void A_chamfer_whose_stripes_are_planes_is_built_on_a_solid_at_a_corner_of_convex_and_concave_edges()
    {
        Shape shape = Read("chamfer-A5-step");
        Edge edge = EdgeBetween(shape, new Point3D(0.365923, 0.301923, 0.365334), new Point3D(0.365923, 0.301923, -0.185155));
        Face[] two = [.. shape.AllFaces().Where(f => f.AllEdges().Any(e => e.IsSame(edge)))];
        Face reference = two.Single(f => f.AllVertices().Any(v => v.Point.Y > 0.32)); // the face the 0.03 is on
        ChamferSpec[] specs = [new ChamferSpec(edge, 0.03, 0.06, ReferenceFace: reference)];
        MakeChamfer native = Native(shape, specs);
        native.Build();
        Assert.False(native.IsDone);

        Result<Built<Shape>> made = shape.Chamfer(specs, NoLimit);
        Assert.True(made.IsSuccess, made.Error?.ToString());
        Shape result = made.Value.Shape;
        Assert.Equal(ShapeType.Solid, result.AllSolids().Single().Type);
        Assert.True(result.Validate().IsValid);
        Assert.Equal(39, result.AllFaces().Length); // the chamfer's face, and one of the input's gone under it
        Assert.True(ChamferMeet.Proved(shape, result, specs, [reference], native, 1e-4));
        Assert.Null(EdgeCrossings.Problem(shape, result, "The chamfer's result", FilletOperations.SelfIntersecting));

        // What's taken off is the prism of the two distances along the edge and the one that runs on from it to the foot,
        // 0.5805 long together (5.2244e-4), less what the earlier chamfer's material keeps of it over the last 0.069 at the foot
        var exact = new MassOptions { Tolerance = 1e-8 };
        double removed = MeasureOperations.IntegrateVolume(shape, exact) - MeasureOperations.IntegrateVolume(result, exact);
        Assert.Equal(5.01745e-4, removed, 1e-8);
    }

    // A cube's top chamfered on its four edges by more than half its width: each plane takes the others' contacts on the top
    // away. What the fuse would bound, a pyramid on a block, isn't the chamfer asked, and the path isn't taken (unit 15U3)
    [Theory]
    [InlineData(5.0)]
    [InlineData(6.0)]
    public void Planes_that_take_each_others_contacts_away_arent_built(double length)
    {
        Shape cube = new MakeBox(10, 10, 10).Shape();
        ChamferSpec[] specs = [.. cube.AllEdges().Where(e => e.AllVertices().All(v => Math.Abs(v.Point.Z - 10) < 1e-9)).Select(e => new ChamferSpec(e, length))];
        Assert.Equal(4, specs.Length);
        Assert.False(cube.Chamfer(specs, NoLimit).IsSuccess);
    }

    // The volume proof's sections for a chamfer of two distances, and of a distance and an angle (unit 15U3), on a box's edge:
    // a triangle of the two distances, or of the distance and its tangent's share of it, along the edge
    [Theory]
    [InlineData(0.3, 0.7, null)]
    [InlineData(0.7, 0.3, null)]
    [InlineData(0.4, null, 30.0)]
    [InlineData(0.4, null, 60.0)]
    public void The_sections_along_an_edge_take_two_distances_or_a_distance_and_an_angle(double distance, double? second, double? degrees)
    {
        Shape box = new MakeBox(10, 10, 10).Shape();
        Face[] faces = [.. box.AllFaces()];
        Edge edge = EdgeBetween(box, new Point3D(0, 0, 0), new Point3D(0, 0, 10));
        Face[] two = [.. faces.Where(f => f.AllEdges().Any(e => e.IsSame(edge)))];
        double other = second ?? distance * Math.Tan(degrees!.Value * Math.PI / 180);
        foreach (Face reference in two)
        {
            double? swept = ChamferMeet.SweptVolume(faces, edge, distance, second, degrees * Math.PI / 180, reference);
            Assert.Equal(0.5 * distance * other * 10, swept!.Value, 1e-9);
        }
    }

    // A caller who leaves the check out asks for what ChFi3d makes, and it made nothing
    [Fact]
    public void Without_the_check_the_path_isnt_taken()
    {
        (Shape shell, Edge along, Edge across, Face floor) = EightJ1();
        Result<Built<Shape>> made = shell.Chamfer([new ChamferSpec(along, 0.1, Angle: Math.PI / 3, ReferenceFace: floor),
                                                   new ChamferSpec(across, 0.06, Angle: Math.PI / 3, ReferenceFace: floor)],
                                                  new ChamferOptions { CheckResult = false });
        Assert.False(made.IsSuccess);
    }

    // A loft from a 10 by 10 square to a circle of radius 4, 10 above (reliability.md's island 3), every edge chamfered. At
    // each top vertex the rim's two arcs are tangent and so are the side faces: ChFi3d's corner of three chamfers fails, and
    // its side stripes are off the length where they meet their faces at a narrow angle. Unit 15V's bands of Kerf's own (the
    // rim one band through its vertices, the sides solved where they lie) build it, valid, at the lengths, without crossings
    // and of the volume the sections give, and the public chamfer returns it (decision 2179). Its 11 to 14 s can pass the
    // default 20 on a loaded machine, so the limit is raised (decision 2185)
    [Theory]
    [InlineData(0.05, 18)]
    [InlineData(0.2, 18)]
    public void A_loft_whose_side_faces_turn_tangent_at_its_top_corners_is_built_with_kerfs_own_bands(double length, int faces)
    {
        Shape loft = Loft();
        Edge[] edges = [.. loft.AllEdges()];
        ChamferSpec[] specs = [.. edges.Select(e => new ChamferSpec(e, length))];
        Face[] references = [.. edges.Select(e => loft.AllFaces().First(f => f.AllEdges().Any(x => x.IsSame(e))))];
        MakeChamfer native = Native(loft, specs);
        native.Build();
        Assert.True(!native.IsDone || !native.Shape().Validate().IsValid);

        Result<Built<Shape>> chamfered = loft.Chamfer(edges, length, new ChamferOptions { TimeLimit = TimeSpan.FromMinutes(10) });
        Assert.True(chamfered.IsSuccess, chamfered.Error?.ToString());
        Shape made = chamfered.Value.Shape;
        Assert.Equal(ShapeType.Solid, made.Type);
        Assert.True(made.Validate().IsValid);
        Assert.Equal(faces, made.AllFaces().Length);
        Assert.True(ChamferMeet.Proved(loft, made, specs, references, native, 1e-4));
        Assert.Null(EdgeCrossings.Problem(loft, made, "The chamfer's result", FilletOperations.SelfIntersecting));
    }

    // Near 0.01 the loft's fuse ran 36 to 82 s and its pieces still weren't a chamfer (unit 15Y, decisions 2182 and 2183).
    // The time was each side edge against its own chamfer's band, 0.007 away along its whole length: the edge/face
    // intersection's localisation listed up to 99,873 pairs of samples, now given up at 2,000. The pieces were ChFi3d's bands
    // carried on across by a length, their parameter across running 0 to 10 over a width of 0.02: the fuse's curves on them were
    // up to 0.5 off their surfaces. With that parameter by the band's width they're within 1e-5, and the loft builds as at
    // 0.05; at 0.01, where ChFi3d can't walk one stripe, that stripe takes a band of Kerf's own. Asked of the construction
    // itself with no limit (Timeout.InfiniteTimeSpan), so that a loaded machine can't fail it
    [Theory]
    [InlineData(0.01, 17, 740.964618)]
    [InlineData(0.0122, 18, 740.962334)]
    public void The_lofts_chamfers_near_a_hundredth_are_built(double length, int faces, double volume)
    {
        Shape loft = Loft();
        Edge[] edges = [.. loft.AllEdges()];
        ChamferSpec[] specs = [.. edges.Select(e => new ChamferSpec(e, length))];
        Face[] references = [.. edges.Select(e => loft.AllFaces().First(f => f.AllEdges().Any(x => x.IsSame(e))))];
        MakeChamfer native = Native(loft, specs);
        native.Build();
        Assert.True(!native.IsDone || !native.Shape().Validate().IsValid);

        EngineOutcome? met = ChamferMeet.Build(loft, native, specs, references, new ChamferOptions { TimeLimit = Timeout.InfiniteTimeSpan }, out bool late);
        Assert.False(late);
        Assert.NotNull(met);
        Shape made = met.Value.Shape();
        Assert.Equal(ShapeType.Solid, made.Type);
        Assert.True(made.Validate().IsValid);
        Assert.Equal(faces, made.AllFaces().Length);
        Assert.True(ChamferMeet.Proved(loft, made, specs, references, native, 1e-4));
        Assert.Null(EdgeCrossings.Problem(loft, made, "The chamfer's result", FilletOperations.SelfIntersecting));
        Assert.Equal(volume, Kerf.Measurement.MeasureOperations.IntegrateVolume(made, new Kerf.Measurement.MassOptions { Tolerance = 1e-8 }), 1e-5);
    }

    // A chamfer whose construction runs past the caller's limit stops there: the operation fails with fillet.time-limit, whose
    // message names the limit and how to change it, and ChFi3d's own failure after it (decisions 2180, 2185). No chamfer
    // known takes the default 20 s any more (the loft's near 0.01 did, before unit 15Y), so the limit is set short here
    [Fact]
    public void A_chamfer_past_its_time_limit_stops_there_and_says_so()
    {
        Shape loft = Loft();
        TimeSpan limit = TimeSpan.FromSeconds(1);
        var watch = System.Diagnostics.Stopwatch.StartNew();
        Result<Built<Shape>> chamfered = loft.Chamfer(loft.AllEdges(), 0.05, new ChamferOptions { TimeLimit = limit });
        watch.Stop();
        Assert.False(chamfered.IsSuccess);
        Assert.Equal(FilletOperations.TimeLimit, chamfered.Error!.Code);
        Assert.Contains("limit of 1 s (ChamferOptions.TimeLimit; Timeout.InfiniteTimeSpan for none)", chamfered.Error.Message, StringComparison.Ordinal);
        Assert.True(chamfered.Diagnostics.Length > 1); // ChFi3d's own failure, or its invalid result's, after it
        Assert.True(watch.Elapsed < limit + Margin, $"{watch.Elapsed}");
    }

    // ChFi3d's part of the loft's chamfer, and the construction's slowest step between two looks at its token, with room for
    // a loaded machine
    private static readonly TimeSpan Margin = TimeSpan.FromSeconds(5);

    // The default is the owner's 20 s (decision 2179), named in the message; a short limit is given to the millisecond
    [Fact]
    public void The_time_limit_is_20_s_by_default_and_named_in_its_failure()
    {
        Assert.Equal(TimeSpan.FromSeconds(20), ChamferOptions.Default.TimeLimit);
        Assert.Contains("limit of 20 s ", FilletOperations.LateFailure(ChamferOptions.Default.TimeLimit).Message, StringComparison.Ordinal);
        Assert.Contains("limit of 0.25 s ", FilletOperations.LateFailure(TimeSpan.FromMilliseconds(250)).Message, StringComparison.Ordinal);
    }

    // A limit that isn't positive, other than Timeout.InfiniteTimeSpan, or that a deadline can't hold, is refused as an argument
    public static TheoryData<TimeSpan> BadLimits => new()
    {
        TimeSpan.Zero,
        TimeSpan.FromMilliseconds(-2),
        TimeSpan.FromSeconds(-1),
        TimeSpan.MinValue,
        TimeSpan.FromDays(50),
        TimeSpan.MaxValue,
    };

    [Theory]
    [MemberData(nameof(BadLimits))]
    public void A_time_limit_that_isnt_positive_or_off_is_refused(TimeSpan limit)
    {
        Shape box = new MakeBox(10.0, 10.0, 10.0).Shape();
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => box.Chamfer(box.AllEdges(), 1, new ChamferOptions { TimeLimit = limit }));
    }

    // The caller's own cancellation still stops the construction, and as a cancellation, not the limit
    [Fact]
    public void A_callers_cancellation_stops_the_construction_before_its_limit()
    {
        Shape loft = Loft();
        using var cancel = new CancellationTokenSource(TimeSpan.FromSeconds(3));
        var watch = System.Diagnostics.Stopwatch.StartNew();
        Assert.ThrowsAny<OperationCanceledException>(() => loft.Chamfer(loft.AllEdges(), 0.0105, cancellationToken: cancel.Token));
        Assert.True(watch.Elapsed < ChamferOptions.Default.TimeLimit, $"{watch.Elapsed}");
    }

    // The volume proof's sections along an edge (unit 15V) against Pappus: a cylinder's top rim chamfered by d takes off a
    // triangle of d squared over two whose centroid turns at the radius less d over three
    [Theory]
    [InlineData(0.05)]
    [InlineData(0.3)]
    [InlineData(0.8)]
    public void The_sections_along_an_edge_give_pappus_on_a_cylinders_rim(double length)
    {
        Shape cylinder = new ApiMakeCylinder(4.0, 10.0).Shape();
        Face[] faces = [.. cylinder.AllFaces()];
        Edge rim = cylinder.AllEdges().Single(e => !Tool.Degenerated(e) && faces.Count(f => f.AllEdges().Any(x => x.IsSame(e))) == 2
                                                   && e.AllVertices().All(v => Math.Abs(v.Point.Z - 10) < 1e-9));
        double exact = 2 * Math.PI * (4 - (length / 3)) * length * length / 2;
        Assert.Equal(exact, ChamferMeet.SweptVolume(faces, rim, length)!.Value, 1e-7 * exact);
    }

    private static Shape Loft()
    {
        var square = new MakePolygon(new Point3D(-5, -5, 0), new Point3D(5, -5, 0), new Point3D(5, 5, 0), new Point3D(-5, 5, 0), true);
        var circle = new MakeWire();
        circle.Add(new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 10), new Direction3D(0, 0, 1)), 4)).Edge());
        var loft = new ThruSections(true, false);
        loft.AddWire(square.Wire());
        loft.AddWire(circle.Wire());
        loft.Build();
        return loft.Shape();
    }

    private static ChamferSpec Spec(Edge edge, double distance, double? second, double? degrees, Face reference) =>
        new(edge, distance, second, degrees is { } a ? a * Math.PI / 180 : null, second is null && degrees is null ? null : reference);

    private static MakeChamfer Native(Shape shape, ChamferSpec[] specs)
    {
        var native = new MakeChamfer(shape);
        foreach (ChamferSpec spec in specs)
        {
            if (spec.Angle is { } angle)
            {
                native.AddDA(spec.Distance, angle, spec.Edge, spec.ReferenceFace!.Value);
            }
            else if (spec.Distance2 is { } second)
            {
                native.Add(spec.Distance, second, spec.Edge, spec.ReferenceFace!.Value);
            }
            else
            {
                native.Add(spec.Distance, spec.Edge);
            }
        }
        return native;
    }
}
