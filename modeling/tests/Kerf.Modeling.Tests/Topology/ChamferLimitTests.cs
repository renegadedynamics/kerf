using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// A chamfer longer than the tightest convex arc in a face beside its edges is refused before it's built, saying what fits
/// (decision 23801). The case reported: a prism with corners of 1.7, its end's whole outline chamfered.
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
    public void The_shorthand_throws_the_same()
    {
        Part prism = Prism();
        var failure = Assert.Throws<InvalidOperationException>(() => prism.Chamfer(1.71, null, Outline(prism)));
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
