using Kerf.Fillets;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// A chamfer longer than the tightest convex arc in a face beside its edges is refused before it's built, saying what fits
/// (decision 1985). The case reported: a prism with corners of 1.7, its end's whole outline chamfered.
/// </summary>
public class ChamferLimitTests
{
    private static Part Prism()
    {
        var box = new Box(40, 30, 30);
        return Fillet(box, box.Edges().Where(e => Math.Abs(e.TangentAt(0.5).Z) > 0.5), 1.7).Value;
    }

    private static ShapeList<Edge> Outline(Part prism) => prism.Faces().OrderBy(f => f.Center().Z).Last().Edges();

    [Fact]
    public void The_reported_prism_at_169_170_171_and_3()
    {
        Part prism = Prism();
        ShapeList<Edge> outline = Outline(prism);
        Assert.Equal(8, outline.Count);

        // As build123d gives it
        Part made = Chamfer(prism, outline, 1.69).Value;
        Assert.True(made.IsValid);
        Assert.Equal(35734.871591, made.Volume, 1e-6);

        // At the arcs' radius the outline closes to a point at each corner: ChFi3d is asked, and fails, saying so
        Result<Part> at = Chamfer(prism, outline, 1.7);
        Assert.True(at.IsFailure);
        Assert.Contains("to a point", at.Error!.Message, StringComparison.Ordinal);
        Assert.Contains("try a slightly smaller length", at.Error.Message, StringComparison.Ordinal);

        // Past it, refused before anything is built: a failure, not a broken shape
        foreach (double length in new[] { 1.71, 3 })
        {
            Result<Part> refused = Chamfer(prism, outline, length);
            Assert.True(refused.IsFailure);
            Assert.False(refused.TryGetInvalid(out _));
            Assert.Equal(ModelingDiagnostics.ChamferFailed, refused.Error!.Code);
            Assert.Equal($"The chamfer with a length of {length} is too large for these 8 edges: the most that fits is 1.7, the radius of the "
                         + "tightest convex arc in a face beside them, where its edge on that face closes to a point.", refused.Error.Message);
        }
    }

    [Fact]
    public void A_broken_result_says_the_chamfer_couldnt_be_built_at_its_length()
    {
        // Not past a bend, so left to the check: a cube's top at 5, where ChFi3d makes something not closed
        var box = new Box(10, 10, 10);
        Result<Part> made = Chamfer(box, box.Faces().OrderBy(f => f.Center().Z).Last().Edges(), 5);
        Assert.True(made.TryGetInvalid(out _));
        Assert.EndsWith("isn't valid: Not closed. The chamfer couldn't be built at a length of 5: try a smaller one.", made.Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void MaxChamfer_takes_the_limit_where_it_builds_and_searches_below_it_where_it_doesnt()
    {
        // A box's edge: the width of a face, 10, built; a disc's rim: its radius, the top closing to a point, built
        var box = new Box(10, 10, 10);
        Edge edge = box.Edges().First(e => Math.Abs(e.TangentAt(0.5).Z) > 0.5);
        Assert.Equal(10, box.AsShape().ChamferLimit([edge]));
        Assert.Equal(10, box.MaxChamfer([edge]));
        var disc = new Cylinder(5, 10);
        Edge rim = disc.Edges().OrderBy(e => e.Center().Z).Last();
        Assert.Equal(5, disc.MaxChamfer([rim]));

        // The prism: 1.7 doesn't build, as the arcs' edge on the end closes to a point, but the largest length below it by more than
        // the shape's tolerance does, a cone of radius 2e-7 at each corner; it was the halving's 1.594 (decision 2047)
        Part prism = Prism();
        ShapeList<Edge> outline = Outline(prism);
        Assert.Equal(1.7, prism.AsShape().ChamferLimit(outline));
        SizeLimit limit = prism.AsShape().ChamferSizeLimit(outline)!.Value;
        Assert.Equal(1.7, limit.Value);
        Assert.Equal(1e-7, limit.Tolerance, 1e-20);
        double most = prism.MaxChamfer(outline);
        Assert.Equal(limit.JustBelow, most);
        Assert.Equal(1.6999998, most, 1e-15);
        Result<Part> made = Chamfer(prism, outline, most);
        Assert.True(made.IsSuccess);
        Assert.True(made.Value.IsValid);
        Assert.True(Chamfer(prism, outline, 1.7).IsFailure);
    }

    [Fact]
    public void MaxChamfer_doesnt_try_past_the_limit()
    {
        // The prism's tries, counted: the limit, then just below it, which builds; none past 1.7 asked of ChFi3d
        Part prism = Prism();
        ShapeList<Edge> outline = Outline(prism);
        var asked = new List<double>();
        bool Works(double d)
        {
            asked.Add(d);
            return prism.AsShape().Chamfer(outline, d).IsSuccess;
        }
        double window = 2 * prism.Bounds().Diagonal;
        double most = Shape3DExtensions.Largest(new SizeLimit(1.7, 1e-7), Works, window, 0.1, 10);
        Assert.Equal(prism.MaxChamfer(outline), most);
        Assert.Equal([1.7, 1.7 - 2e-7], asked);

        // Without it, the halving's answer, within 0.1 of a length that works, from 10 tries, each past 1.7 refused at once by the
        // chamfer itself
        asked.Clear();
        double searched = Shape3DExtensions.Largest(null, Works, window, 0.1, 10);
        Assert.InRange(searched, 1.7 - 0.11, most);
        Assert.Equal(10, asked.Count);
    }

    [Fact]
    public void The_shorthand_throws_the_same()
    {
        Part prism = Prism();
        var failure = Assert.Throws<ModelingException>(() => prism.Chamfer(1.71, null, Outline(prism)));
        Assert.Contains("the most that fits is 1.7", failure.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Two_lengths_are_judged_by_the_one_on_the_end()
    {
        // build123d's length goes on the reference face, here the end, and length2 down the side, which is straight
        Part prism = Prism();
        Face end = prism.Faces().OrderBy(f => f.Center().Z).Last();
        Assert.True(Chamfer(prism, Outline(prism), 1, 3, reference: end).IsSuccess);
        Assert.Contains("the most that fits is 1.7", Chamfer(prism, Outline(prism), 1.71, 1, reference: end).Error!.Message, StringComparison.Ordinal);

        // By an angle, length2 is length times its tangent: 1 on the end and 1.71 down the side fits; 1.71 on the end doesn't
        Assert.True(Chamfer(prism, Outline(prism), 1, angle: Math.Atan(1.71) * 180 / Math.PI, reference: end).IsSuccess);
        Assert.Contains("the most that fits is 1.7", Chamfer(prism, Outline(prism), 1.71, angle: 30, reference: end).Error!.Message, StringComparison.Ordinal);
    }
}
