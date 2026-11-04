using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Immutability;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// Extrusions, revolutions, sweeps and lofts of faces taken from other parts (Kerf unit 10H3): the part the face came
/// from is untouched, BRep text for BRep text, and the result shares what the kernel's sweep leaves unchanged: the
/// extruded face itself and its edges (decisions 468 and 1102).
/// </summary>
public class ConstructionInputsUnchangedTests
{
    private static Box Part() => new(10, 6, 4);

    private static Face Top(Shape part) => part.Faces().OrderBy(Axis.Z).Last();

    private static Face Side(Shape part) => part.Faces().OrderBy(Axis.X).Last();

    private static byte[] Text(Shape shape) => Extras.WriteBRep(shape.KernelShape);

    private static bool Holds(Shape result, Shape sub) =>
        sub switch
        {
            Face => result.Faces().Any(f => f.KernelShape.IsSame(sub.KernelShape)),
            Vertex => result.Vertices().Any(v => v.KernelShape.IsSame(sub.KernelShape)),
            _ => result.Edges().Any(e => e.KernelShape.IsSame(sub.KernelShape)),
        };

    private static Shape Untouched(Shape part, Func<Shape> construction)
    {
        byte[] before = Text(part);
        Shape result = construction();
        Assert.True(before.AsSpan().SequenceEqual(Text(part)), "the part the face came from changed");
        Assert.True(result.Volume > 0);
        return result;
    }

    [Fact]
    public void ExtrudingAFaceOfAPartSharesTheFace()
    {
        Box part = Part();
        Face top = Top(part);
        Shape result = Untouched(part, () => Ops.Extrude(top, 3, new Ops.ExtrudeOptions { Clean = false }).Value);
        Assert.True(Holds(result, top));
        Assert.All(top.Edges(), e => Assert.True(Holds(result, e)));
        Shape solid = Untouched(part, () => Solid.Extrude(top, new Vector(0, 0, 3)));
        Assert.True(Holds(solid, top));
    }

    [Fact]
    public void RevolvingAFaceOfAPartLeavesThePartAlone()
    {
        // build123d revolves with copy (BRepPrimAPI_MakeRevol's last argument): the solid shares nothing with the face
        Box part = Part();
        Face side = Side(part);
        Untouched(part, () => Ops.Revolve(side, new Axis(new Vector(20.0, 0.0, 0.0), new Vector(0.0, 0.0, 1.0)), 90, clean: false).Value);
    }

    [Fact]
    public void SweepingAFaceOfAPartLeavesThePartAlone()
    {
        Box part = Part();
        Face top = Top(part);
        Edge path = Edge.MakeThreePointArc(new Vector(5, 3, 4), new Vector(5, 8, 9), new Vector(5, 13, 4));
        Untouched(part, () => Ops.Sweep(top, path, new Ops.SweepOptions { Clean = false }).Value);
    }

    [Fact]
    public void LoftingFacesOfPartsLeavesThePartsAlone()
    {
        Box part = Part();
        Box other = new(4, 4, 4);
        Shape moved = other.Moved(new Location(new Vector(0.0, 0.0, 10.0)));
        byte[] otherBefore = Text(other);
        Untouched(part, () => Ops.Loft([Top(part), Top(moved)], clean: false).Value);
        Assert.True(otherBefore.AsSpan().SequenceEqual(Text(other)));
        Untouched(part, () => Ops.Loft([Top(part), Top(moved)], ruled: true, clean: false).Value);
    }
}
