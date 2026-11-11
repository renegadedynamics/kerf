// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// Chamfers whose corner ChFi3d can't build, built with their faces carried on until they meet (unit 15U): the corpus's
/// shapes, each ChFi3d's failure first, then the public chamfer's result and its proof, read from the result alone.
/// </summary>
public class ChamferMeetTests
{
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

        Shape made = shell.Chamfer(specs).Value.Shape;
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

        Shape made = shape.Chamfer(specs).Value.Shape;
        Assert.True(made.Validate().IsValid);
        Assert.Equal(6, made.AllFaces().Length);
        Face first = shape.AllFaces().First(f => f.AllEdges().Any(e => e.IsSame(edge)));
        Assert.True(ChamferMeet.Proved(shape, made, specs, [first], native, 1e-4));
    }

    // chamfer/dist_dist_sequence/B7, its second step: the stripe runs onto another edge of its face before the corner, whose
    // other edges are one convex and one concave (decision 2144). Not built: the corner would need a face of the input
    // carried on too, so ChFi3d's failure stands
    [Fact]
    public void A_corner_of_convex_and_concave_edges_still_fails()
    {
        Shape shape = Read("chamfer-B5-step2");
        Edge edge = EdgeBetween(shape, new Point3D(-0.606778, 0.0116437, 0.226719), Corner);
        Face floor = shape.AllFaces().Single(f => f.AllVertices().Any(v => v.Point.DistanceTo(new Point3D(0.473048, 0.5374, 0.308257)) < 1e-5));
        Result<Built<Shape>> made = shape.Chamfer([new ChamferSpec(edge, 0.2, 0.21, ReferenceFace: floor)]);
        Assert.False(made.IsSuccess);
        Assert.Equal(FilletOperations.Failed, made.Error!.Code);
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
    // and of the volume the sections give, and the public chamfer returns it (decision 2179)
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

        Result<Built<Shape>> chamfered = loft.Chamfer(edges, length);
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
    // itself with a long limit, so that a loaded machine can't fail it
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

        EngineOutcome? met = ChamferMeet.Build(loft, native, specs, references, ChamferOptions.Default, out bool late, TimeSpan.FromMinutes(10));
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

    // A construction that runs past its limit stops there and says it was late, and the operation's failure for that names
    // the limit. No chamfer known takes the 20 s any more (the loft's near 0.01 did, before unit 15Y), so the limit is set
    // short here
    [Fact]
    public void A_construction_that_runs_past_its_limit_stops_there_and_says_so()
    {
        Shape loft = Loft();
        Edge[] edges = [.. loft.AllEdges()];
        ChamferSpec[] specs = [.. edges.Select(e => new ChamferSpec(e, 0.05))];
        Face[] references = [.. edges.Select(e => loft.AllFaces().First(f => f.AllEdges().Any(x => x.IsSame(e))))];
        MakeChamfer native = Native(loft, specs);
        native.Build();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        EngineOutcome? met = ChamferMeet.Build(loft, native, specs, references, ChamferOptions.Default, out bool late, TimeSpan.FromSeconds(1));
        watch.Stop();
        Assert.Null(met);
        Assert.True(late);
        Assert.True(watch.Elapsed < TimeSpan.FromSeconds(11), $"{watch.Elapsed}");

        Diagnostic failure = FilletOperations.LateFailure();
        Assert.Equal(FilletOperations.TimeLimit, failure.Code);
        Assert.Contains("20 s", failure.Message, StringComparison.Ordinal);
    }

    // The caller's own cancellation still stops the construction, and as a cancellation, not the limit
    [Fact]
    public void A_callers_cancellation_stops_the_construction_before_its_limit()
    {
        Shape loft = Loft();
        using var cancel = new CancellationTokenSource(TimeSpan.FromSeconds(3));
        var watch = System.Diagnostics.Stopwatch.StartNew();
        Assert.ThrowsAny<OperationCanceledException>(() => loft.Chamfer(loft.AllEdges(), 0.0105, cancellationToken: cancel.Token));
        Assert.True(watch.Elapsed < ChamferMeet.Limit, $"{watch.Elapsed}");
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
