using Kerf.Engine.Construction.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntTools_ShrunkRange. The expected values follow OCCT's algorithm: the vertices' tolerances are raised to the
/// edge's and increased by Precision::Confusion(), and BRepLib::FindValidRange finds the parameters where the curve
/// leaves those spheres to within its parametric epsilon (max(Resolution(TolE) / 10, PConfusion)).
/// </summary>
public class IntTools_ShrunkRangeTests
{
    private static EdgeShrunkRange Perform(in Edge theEdge)
    {
        Exploration.Vertices(theEdge, out Vertex aV1, out Vertex aV2);
        Tool.Range(theEdge, out double aT1, out double aT2);
        EdgeShrunkRange aSR = new();
        aSR.SetData(theEdge, aT1, aT2, aV1, aV2);
        aSR.SetContext(new IntersectionContext());
        aSR.Perform();
        return aSR;
    }

    [Fact]
    public void Defaults()
    {
        EdgeShrunkRange aSR = new();
        aSR.ShrunkRange(out double aT1, out double aT2);
        Assert.Equal((-99.0, -99.0), (aT1, aT2));
        Assert.False(aSR.IsDone);
        Assert.False(aSR.IsSplittable);
        Assert.Equal(0.0, aSR.Length());
        Assert.True(aSR.BndBox().IsVoid);
        Assert.Null(aSR.Context());
    }

    [Fact]
    public void StraightEdge()
    {
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        EdgeShrunkRange aSR = Perform(anEdge);
        Assert.True(aSR.IsDone);
        Assert.True(aSR.IsSplittable);
        Assert.True(aSR.Edge().IsSame(anEdge));
        Assert.NotNull(aSR.Context());

        // tolerance spheres of 1e-7 + 1e-7 at each end; the line's parameter is its length
        aSR.ShrunkRange(out double aTS1, out double aTS2);
        Assert.Equal(2.0e-7, aTS1, 1.0e-8);
        Assert.Equal(10.0 - 2.0e-7, aTS2, 1.0e-8);
        Assert.Equal(aTS2 - aTS1, aSR.Length(), 1.0e-12);

        // the box of the shrunk range, enlarged by TolE + Confusion
        aSR.BndBox().Get(out double xmin, out double ymin, out _, out double xmax, out double ymax, out _);
        Assert.True(xmin <= aTS1 - 2.0e-7 && xmin > aTS1 - 1.0e-6);
        Assert.True(xmax >= aTS2 + 2.0e-7 && xmax < aTS2 + 1.0e-6);
        Assert.True(ymin <= -2.0e-7 && ymax >= 2.0e-7);
    }

    [Fact]
    public void CircularEdge()
    {
        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        Edge anEdge = new MakeEdge(aCirc, 0.0, Math.PI).Edge();
        EdgeShrunkRange aSR = Perform(anEdge);
        Assert.True(aSR.IsDone);
        Assert.True(aSR.IsSplittable);

        // a sphere of radius 2e-7 at each end cuts the circle of radius 2 at the angle 2e-7 / 2
        aSR.ShrunkRange(out double aTS1, out double aTS2);
        Assert.Equal(1.0e-7, aTS1, 1.0e-8);
        Assert.Equal(Math.PI - 1.0e-7, aTS2, 1.0e-8);
        Assert.Equal(2.0 * (aTS2 - aTS1), aSR.Length(), 1.0e-6);
    }

    [Fact]
    public void ShortEdgeWithBigTolerance_DoneButNotSplittable()
    {
        // a 1e-3 edge of tolerance 2.6e-4: the shrunk range is 1e-3 - 2 * (2.6e-4 + 1e-7) = 4.798e-4 long, less than
        // the 2 * TolE + 2 * Confusion = 5.202e-4 needed to split it once
        MakeEdge aMaker = new(new Point3D(0, 0, 0), new Point3D(1.0e-3, 0, 0));
        Edge anEdge = aMaker.InternalEdge();
        new BrepBuilder().UpdateEdge(anEdge, 2.6e-4);
        EdgeShrunkRange aSR = Perform(anEdge);
        Assert.True(aSR.IsDone);
        Assert.False(aSR.IsSplittable);
        // FindValidRange stops within its epsilon, Resolution(TolE) / 10 = 2.6e-5, of each sphere
        Assert.Equal(4.798e-4, aSR.Length(), 6.0e-5);
    }

    [Fact]
    public void ShortEdgeWithSmallerTolerance_Splittable()
    {
        MakeEdge aMaker = new(new Point3D(0, 0, 0), new Point3D(1.0e-3, 0, 0));
        Edge anEdge = aMaker.InternalEdge();
        new BrepBuilder().UpdateEdge(anEdge, 2.0e-4);
        EdgeShrunkRange aSR = Perform(anEdge);
        Assert.True(aSR.IsDone);
        Assert.True(aSR.IsSplittable);
    }

    [Fact]
    public void EdgeCoveredByItsVertices_NotDone()
    {
        MakeEdge aMaker = new(new Point3D(0, 0, 0), new Point3D(1.0e-3, 0, 0));
        Edge anEdge = aMaker.InternalEdge();
        new BrepBuilder().UpdateEdge(anEdge, 6.0e-4);
        EdgeShrunkRange aSR = Perform(anEdge);
        Assert.False(aSR.IsDone);
        Assert.False(aSR.IsSplittable);
        Assert.True(aSR.BndBox().IsVoid);
    }

    [Fact]
    public void TooShortRange_NotDone()
    {
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
        EdgeShrunkRange aSR = new();
        aSR.SetData(anEdge, 5.0, 5.0 + 1.0e-10, aV1, aV2);
        aSR.Perform();
        Assert.False(aSR.IsDone);
    }

    [Fact]
    public void SetShrunkRange_AddsToTheBox()
    {
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
        EdgeShrunkRange aSR = new();
        aSR.SetData(anEdge, 0.0, 10.0, aV1, aV2);
        aSR.SetShrunkRange(2.0, 3.0);
        aSR.ShrunkRange(out double aT1, out double aT2);
        Assert.Equal((2.0, 3.0), (aT1, aT2));
        aSR.BndBox().Get(out double xmin, out _, out _, out double xmax, out _, out _);
        Assert.Equal(2.0, xmin, 1.0e-6);
        Assert.Equal(3.0, xmax, 1.0e-6);

        // as in OCCT, the box isn't reset: it grows with the next range
        aSR.SetShrunkRange(5.0, 6.0);
        aSR.BndBox().Get(out xmin, out _, out _, out xmax, out _, out _);
        Assert.Equal(2.0, xmin, 1.0e-6);
        Assert.Equal(6.0, xmax, 1.0e-6);
    }
}
