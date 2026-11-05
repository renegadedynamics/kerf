using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Immutability;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// Offsets, shells, thickenings, fillets, chamfers and drafts of parts taken from elsewhere (Kerf unit 10H5): a part
/// made by another operation, a face of another part. The part is untouched, BRep text for BRep text, and the result
/// shares what the kernel left unchanged: the faces away from a fillet or a chamfer, the bottom of a shelled part.
/// </summary>
public class FeatureInputsUnchangedTests
{
    // A part made by another operation: a box with a boss fused on its top
    private static Part Fused() => (Part)(new Box(10, 6, 4) + new Box(2, 2, 6));

    private static byte[] Text(Shape shape) => Extras.WriteBRep(shape);

    private static int SharedFaces(Shape result, Shape part) =>
        part.SelectFaces().Count(f => result.SelectFaces().Any(r => r.IsSame(f)));

    private static Shape Untouched(Shape part, Func<Shape> operation)
    {
        byte[] before = Text(part);
        Shape result = operation();
        Assert.True(before.AsSpan().SequenceEqual(Text(part)), "the part changed");
        Assert.True(result.IsValid);
        return result;
    }

    [Fact]
    public void FilletingAndChamferingAnEdgeOfAPartKeepTheFacesAwayFromIt()
    {
        Part part = Fused();
        Edge edge = part.Edges().GroupBy(Axis.Z)[0][0];
        Shape fillet = Untouched(part, () => Ops.Fillet(part, [edge], 0.5).Value);
        Shape chamfer = Untouched(part, () => Ops.Chamfer(part, [edge], 0.5).Value);
        int faces = part.Faces().Count;
        Assert.True(SharedFaces(fillet, part) >= faces - 4);
        Assert.True(SharedFaces(chamfer, part) >= faces - 4);
    }

    [Fact]
    public void OffsettingAndShellingAPartLeaveItAlone()
    {
        Part part = Fused();
        Untouched(part, () => Ops.Offset(part, 0.5).Value);
        Shape shell = Untouched(part, () => Ops.Offset(part, -0.5, new Ops.OffsetOptions { Openings = [part.Faces().OrderBy(Axis.Z).Last()] }).Value);
        // the bottom face, away from the opening, is the part's own
        Face bottom = part.Faces().OrderBy(Axis.Z)[0];
        Assert.Contains(shell.SelectFaces(), f => f.IsSame(bottom));
    }

    [Fact]
    public void ThickeningAFaceOfAPartLeavesThePartAlone()
    {
        Part part = Fused();
        Face top = part.Faces().OrderBy(Axis.Z)[0];
        Untouched(part, () => Ops.Thicken(top, 1).Value);
        Untouched(part, () => Ops.Thicken(top, -1).Value);
    }

    [Fact]
    public void DraftingTheSidesOfAPartLeavesItAlone()
    {
        Box box = new(10, 6, 4);
        Untouched(box, () => Ops.Draft(box.Faces().NotParallelTo(Axis.Z), Plane.XY.Offset(-2), 5).Value);
    }
}
