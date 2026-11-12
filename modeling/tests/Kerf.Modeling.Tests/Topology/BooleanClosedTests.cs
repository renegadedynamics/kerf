using Kerf.Booleans;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Healing;
using Kerf.Measurement;

namespace Kerf.Modeling.Tests;

/// <summary>
/// Booleans of two closed solids that came out open inside a vertex, wrong, or invalid, and are now closed and right
/// (kernel decisions 33701 to 33707), each on the family of decision 2257's sweep as the sweep makes it: equal cylinders, the
/// tool's axis in the other's end plane, a little short of crossing its axis or a little past.
/// </summary>
public class BooleanClosedTests
{
    private static (Part Cylinder, Part Tool) Inputs(double radius, double offset) =>
        (new Cylinder(radius, 2 * radius),
         new Location(new Point3D(offset, 0, radius)) * (new Rotation(Axis.X, 90) * new Cylinder(radius, 4 * radius)));

    // The gaps the faces' boundaries have in their parameters inside a vertex's tolerance
    private static int Gaps(Shape shape)
    {
        int gaps = 0;
        foreach (var face in shape.AllFaces())
            gaps += GapClosures.Of(ShapeCasts.Face(face.AsShape()))?.Count ?? 0;
        return gaps;
    }

    // The volume as OCCT reads it, with no gap closed
    private static double AsOcct(Shape shape)
    {
        var properties = new Properties();
        MassPropertiesBuilder.VolumeProperties(shape, properties, 1e-9, massOnly: true, closeGaps: false);
        return properties.Mass();
    }

    // One cylinder, and half of two equal cylinders' common part, 16/3 r^3, which these offsets change by less than 1e-8 of it
    private static (double One, double Common) Volumes(double radius) =>
        (Math.PI * radius * radius * 2 * radius, 8.0 / 3 * radius * radius * radius);

    // The common or the cut as the engine makes it, with crossings held apart at so many tolerances (0: OCCT's)
    private static Shape Engine(bool common, Shape cylinder, Shape tool, double apart, bool touches = true, bool smallPieces = false)
    {
        Kerf.Engine.Booleans.Api.BooleanOperation operation = common ? new Kerf.Engine.Booleans.Api.Common() : new Kerf.Engine.Booleans.Api.Cut();
        var arguments = new Kerf.Engine.NCollection_List<Shape>();
        arguments.Append(cylinder);
        var tools = new Kerf.Engine.NCollection_List<Shape>();
        tools.Append(tool);
        operation.SetArguments(arguments);
        operation.SetTools(tools);
        operation.SetNonDestructive(true);
        operation.SetCrossingsApart(apart, touches, smallPieces);
        operation.Build();
        Assert.True(operation.IsDone);
        return operation.Shape();
    }

    private static readonly BooleanOptions AsMade = new() { Simplify = false, CheckResult = false };

    private static readonly UnifyOptions Clean = new() { Edges = true, Faces = true, ConcatBSplines = true, AllowInternalEdges = false, CheckResult = false };

    // Radius 50000, 1e-7 short: as OCCT's vertices have it the cut removes nothing and the common is empty, the other way
    // round and at each fuzzy value too, and both failed. Made with the crossings held apart, two ways agree, valid and
    // passing the check; the first result, being wrong, can't vouch for them (decision 33704)
    [Fact]
    public void A_Boolean_proved_wrong_is_made_again_with_its_crossings_apart()
    {
        const double Radius = 50000;
        (Part cylinder, Part tool) = Inputs(Radius, -1e-7);
        (double one, double common) = Volumes(Radius);

        // What the test stands on: the cut as first made is the cylinder whole, and proved wrong
        Shape first = BooleanOperations.Cut([cylinder], [tool], AsMade).Value.Shape;
        Assert.NotNull(BooleanVolumes.Check(BooleanVolumeRule.Cut, [cylinder], [tool], first, 0));

        var checkedCut = BooleanOperations.Cut([cylinder], [tool], new BooleanOptions { Simplify = false });
        Assert.True(checkedCut.IsSuccess);
        Assert.Contains(checkedCut.Diagnostics, static d => d.Code == BooleanOperations.BuiltWithCrossingsApart);

        Shape cut = cylinder - tool;
        Assert.True(cut.IsValid);
        Assert.Equal(0, Gaps(cut));
        Assert.Equal(one - common, cut.Volume, 1e-9 * one);
        Assert.Equal(one - common, AsOcct(cut), 1e-9 * one);

        Shape shared = cylinder & tool;
        Assert.True(shared.IsValid);
        Assert.Equal(0, Gaps(shared));
        Assert.Equal(common, AsOcct(shared), 1e-9 * one);
    }

    // Radius 50000, 3e-4 past: the cut as first made is invalid, and failed for it. Made again as a wrong one is, it's
    // valid, passes the check and is right (decision 33706)
    [Fact]
    public void An_invalid_first_result_is_made_again_as_a_wrong_one_is()
    {
        const double Radius = 50000;
        (Part cylinder, Part tool) = Inputs(Radius, 3e-4);
        (double one, double common) = Volumes(Radius);

        Shape first = BooleanOperations.Cut([cylinder], [tool], AsMade).Value.Shape;
        Assert.False(first.Validate().IsValid);

        Shape cut = cylinder - tool;
        Assert.True(cut.IsValid);
        Assert.Equal(one - common, cut.Volume, 1e-7 * one);
        Shape shared = cylinder & tool;
        Assert.True(shared.IsValid);
        Assert.Equal(common, shared.Volume, 1e-7 * one);
    }

    // Radius 500, 1e-13 short, fused: the first is 16% out, and the one made the other way round is right and open inside
    // a vertex. Held apart the first way round each is 16% out again, so the ways are made the other way round, as the
    // result they stand in for was, and close it (decision 33705)
    [Fact]
    public void A_result_made_the_other_way_round_is_closed_the_other_way_round()
    {
        const double Radius = 500;
        (Part cylinder, Part tool) = Inputs(Radius, -1e-13);
        (double one, double common) = Volumes(Radius);
        Shape fuse = cylinder + tool;
        Assert.True(fuse.IsValid);
        Assert.Equal(0, Gaps(fuse));
        Assert.Equal(3 * one - common, AsOcct(fuse), 1e-9 * one);
    }

    // A result closed at the source stays closed when its edges are merged (decisions 33702 and 33703). Three ways the
    // merge opened one: the merged circle built between its vertices' own points, where the pieces had ended at another
    // point of a wide vertex (radius 500, 3e-13 short); the merged curve on a face made again from the curve in space,
    // where the pieces' had been brought to the face's other edges (radius 500, 2e-11 short, on the tool's seam); and merged
    // lines on a face taken as the first carried on, which a first piece 1.8e-4 long, tilted by rounding, takes 2.6e-7 off
    // over a chain of 0.03 (radius 50000, 1e-8 short)
    [Theory]
    [InlineData(500, -3e-13, true, true)]
    [InlineData(500, -2e-11, true, true)]
    [InlineData(50000, -1e-8, false, false)]
    public void Merging_a_closed_results_edges_leaves_it_closed(double radius, double offset, bool common, bool touches)
    {
        (Part cylinder, Part tool) = Inputs(radius, offset);
        // As the engine makes it with its crossings held apart, the way the public Booleans had it
        Shape made = Engine(common, cylinder, tool, 10, touches);
        Assert.Equal(0, Gaps(made));

        Shape merged = made.Unify(Clean).Value.Shape;
        // Edges were merged: the faces' boundaries have fewer between them
        Assert.True(merged.AllFaces().Sum(static f => f.AsShape().AllEdges().Length) < made.AllFaces().Sum(static f => f.AsShape().AllEdges().Length));
        Assert.Equal(0, Gaps(merged));
        Assert.True(merged.Validate().IsValid);
        Assert.True(merged.AllVertices().Max(static v => v.Tolerance) <= made.AllVertices().Max(static v => v.Tolerance));
        double read = made.IntegrateVolume(new MassOptions { Tolerance = 1e-9 });
        Assert.Equal(read, AsOcct(merged), 1e-9 * read);

        Shape modelled = common ? cylinder & tool : cylinder - tool;
        Assert.Equal(0, Gaps(modelled));
    }

    // Radius 0.5, 1e-14 short: the sections' curves end 1e-7 either side of the touch on the tool's seam, each within the
    // default tolerance of the vertex and so left where it was, 2e-7 from the other: a gap. A curve is now brought to its
    // vertex from half that, and the common, 8.2e-8 out read across the gap, is right to 1e-8 (decision 33707)
    [Fact]
    public void Curves_that_end_a_default_tolerance_either_side_of_a_vertex_are_brought_to_it()
    {
        const double Radius = 0.5;
        (Part cylinder, Part tool) = Inputs(Radius, -1e-14);
        (double one, double common) = Volumes(Radius);
        foreach ((Shape made, double expected) in new (Shape, double)[]
                 {
                     (cylinder - tool, one - common),
                     (cylinder & tool, common),
                     (cylinder + tool, 3 * one - common),
                 })
        {
            Assert.True(made.IsValid);
            Assert.Equal(0, Gaps(made));
            Assert.Equal(expected, AsOcct(made), 1e-8 * expected);
        }
    }

    // Radius 50000, 2e-6 short: the walk's tolerance there is 0.6 and one vertex, 1.05 wide, covers both crossings, 0.45
    // either side of the touch, and the end circle's own vertex between them. The circle's two pieces from its own vertex to
    // that one are no longer than the tolerances, and the common, which has neither, was left with the circle open by 0.9
    // inside the vertex. The last way of making it again takes each into the piece next to it (decision 33708)
    [Fact]
    public void A_piece_too_small_to_keep_is_taken_into_its_neighbour()
    {
        const double Radius = 50000;
        (Part cylinder, Part tool) = Inputs(Radius, -2e-6);
        (double one, double common) = Volumes(Radius);

        // What the test stands on: open as OCCT makes it, and still open with its vertices and the other closing steps
        Assert.True(Gaps(Engine(true, cylinder, tool, 0)) > 0);
        Assert.True(Gaps(Engine(true, cylinder, tool, double.PositiveInfinity, false)) > 0);
        Assert.Equal(0, Gaps(Engine(true, cylinder, tool, double.PositiveInfinity, false, true)));

        Shape shared = cylinder & tool;
        Assert.True(shared.IsValid);
        Assert.Equal(0, Gaps(shared));
        Assert.Equal(common, AsOcct(shared), 1e-8 * common);
        Assert.Equal(common, shared.Volume, 1e-8 * common);
    }
}
