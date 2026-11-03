using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Values;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>The statics of the IntTools package.</summary>
public class IntToolsTests
{
    private static NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot> Roots(params (double Root, int Type)[] theRoots)
    {
        NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot> aSeq = new();
        foreach ((double aRoot, int aType) in theRoots)
        {
            aSeq.Append(new global::Kerf.Engine.Intersection.Tools.IntersectionRoot(aRoot, aType));
        }
        return aSeq;
    }

    [Fact]
    public void SortRoots_IncreasingOrder()
    {
        NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot> aSeq = Roots((3.0, 0), (1.0, 0), (2.0, 0), (-1.0, 0));
        global::Kerf.Engine.Intersection.Tools.IntTools.SortRoots(aSeq, 1.0e-7);
        Assert.Equal(new[] { -1.0, 1.0, 2.0, 3.0 }, aSeq.Select(r => r.Root()));
        global::Kerf.Engine.Intersection.Tools.IntTools.SortRoots(new NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot>(), 1.0e-7); // empty
    }

    [Fact]
    public void SortRoots_TiesKeepTheirOrderUpTo32Roots()
    {
        // MSVC's std::sort sorts up to 32 elements by insertion: tied roots keep their order (decision 491)
        var aRandom = new Random(3);
        var anInput = new (double, int)[32];
        for (int i = 0; i < anInput.Length; ++i)
        {
            anInput[i] = (aRandom.Next(4), i); // the type records the input position
        }
        NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot> aSeq = Roots(anInput);
        global::Kerf.Engine.Intersection.Tools.IntTools.SortRoots(aSeq, 0.0);
        Assert.Equal(anInput.OrderBy(r => r.Item1).ThenBy(r => r.Item2).Select(r => r.Item2), aSeq.Select(r => r.Type()));
    }

    [Fact]
    public void SortRoots_ManyRoots()
    {
        var aRandom = new Random(5);
        var anInput = Enumerable.Range(0, 200).Select(i => (Math.Round(aRandom.NextDouble(), 1), i)).ToArray();
        NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot> aSeq = Roots(anInput);
        global::Kerf.Engine.Intersection.Tools.IntTools.SortRoots(aSeq, 0.0);
        Assert.Equal(200, aSeq.Length());
        for (int i = 2; i <= aSeq.Length(); ++i)
        {
            Assert.True(aSeq.Value(i - 1).Root() <= aSeq.Value(i).Root());
        }
        Assert.Equal(Enumerable.Range(0, 200), aSeq.Select(r => r.Type()).Order());
    }

    [Fact]
    public void RemoveIdenticalRoots_RemovesCloseRoots()
    {
        NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot> aSeq = Roots((1.0, 0), (2.0, 0), (1.1, 0), (3.0, 0));
        global::Kerf.Engine.Intersection.Tools.IntTools.RemoveIdenticalRoots(aSeq, 0.4); // |ti - tj| < 0.2
        Assert.Equal(new[] { 1.0, 2.0, 3.0 }, aSeq.Select(r => r.Root()));
    }

    [Fact]
    public void RemoveIdenticalRoots_OcctBug_SkipsTheRootAfterARemoval()
    {
        // OCCT doesn't step back after removing the k-th root, so the next one isn't compared: of three roots within
        // the tolerance of the first, the third stays
        NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot> aSeq = Roots((1.0, 0), (1.0001, 0), (1.0002, 0));
        global::Kerf.Engine.Intersection.Tools.IntTools.RemoveIdenticalRoots(aSeq, 1.0);
        Assert.Equal(new[] { 1.0, 1.0002 }, aSeq.Select(r => r.Root()));
    }

    [Fact]
    public void FindRootStates_ByType()
    {
        NCollection_Sequence<global::Kerf.Engine.Intersection.Tools.IntersectionRoot> aSeq = new();
        void Add(int theType, double f1, double f2) =>
            aSeq.Append(new global::Kerf.Engine.Intersection.Tools.IntersectionRoot(0.5, theType).WithInterval(0.0, 1.0, f1, f2));
        Add(0, 1.0, -1.0);   // simple, + to -
        Add(0, -1.0, 1.0);   // simple, otherwise
        Add(1, 0.0, 0.0);    // pure zero
        Add(2, 1.0, 0.0);    // smart, f2 null, f1 > 0
        Add(2, -1.0, 0.0);   // smart, f2 null, f1 <= 0
        Add(2, 0.0, 1.0);    // smart, f2 > 0
        Add(2, 0.0, -1.0);   // smart, f2 < 0
        Add(-1, 1.0, 1.0);   // unknown type: unchanged
        global::Kerf.Engine.Intersection.Tools.IntTools.FindRootStates(aSeq, 1.0e-10);

        (Containment, Containment)[] anExpected =
        [
            (Out, In), (In, Out), (On, On), (Out, On),
            (In, On), (On, Out), (On, In), (Unknown, Unknown),
        ];
        Assert.Equal(anExpected, aSeq.Select(r => (r.StateBefore(), r.StateAfter())));
    }

    [Fact]
    public void Length_OfEdges()
    {
        Edge aLine = new MakeEdge(new Point3D(0, 0, 0), new Point3D(3, 4, 0)).Edge();
        Assert.Equal(5.0, global::Kerf.Engine.Intersection.Tools.IntTools.Length(aLine), 1.0e-12);

        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        Edge anArc = new MakeEdge(aCirc, 0.0, Math.PI / 2).Edge();
        Assert.Equal(Math.PI, global::Kerf.Engine.Intersection.Tools.IntTools.Length(anArc), 1.0e-9);

    }

    [Fact]
    public void Parameter_ByCurveType()
    {
        double aPar = -1.0;
        LineCurve3D aLine = new(new Point3D(1, 0, 0), new Direction3D(1, 0, 0));
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.Parameter(new Point3D(4, 1, 0), aLine, ref aPar));
        Assert.Equal(3.0, aPar, 1.0e-15);

        CircleCurve3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.Parameter(new Point3D(0, 5, 0), aCircle, ref aPar));
        Assert.Equal(Math.PI / 2, aPar, 1.0e-15);

        EllipseCurve3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0, 1.0);
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.Parameter(new Point3D(-3, 0, 0), anEllipse, ref aPar));
        Assert.Equal(Math.PI, aPar, 1.0e-12);

        NCollection_Array1<Point3D> aPoles = NCollection_Array1<Point3D>.From([new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, 0, 0)]);
        BezierCurve3D aBezier = new(aPoles);
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.Parameter(aBezier.Value(0.3), aBezier, ref aPar));
        Assert.Equal(0.3, aPar, 1.0e-9);

        aPar = -1.0;
        OffsetCurve3D anOffset = new(aCircle, 1.0, new Direction3D(0, 0, 1));
        Assert.Equal(1, global::Kerf.Engine.Intersection.Tools.IntTools.Parameter(new Point3D(3, 0, 0), anOffset, ref aPar));
        Assert.Equal(-1.0, aPar); // untouched
    }

    [Fact]
    public void GetRadius_LineCircleAndThreePoints()
    {
        Edge aLine = new MakeEdge(new Point3D(0, 0, 0), new Point3D(3, 4, 0)).Edge();
        Assert.Equal(1, global::Kerf.Engine.Intersection.Tools.IntTools.GetRadius(new EdgeCurveAdaptor(aLine), 0.0, 1.0, out _));

        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        Edge anArc = new MakeEdge(aCirc, 0.0, Math.PI).Edge();
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.GetRadius(new EdgeCurveAdaptor(anArc), 0.0, 1.0, out double aR));
        Assert.Equal(2.0, aR);

        // an ellipse: the circle through three points of it
        Ellipse3D anElips = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0, 1.0);
        Edge anEllipseEdge = new MakeEdge(anElips, 0.0, Math.PI).Edge();
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.GetRadius(new EdgeCurveAdaptor(anEllipseEdge), -0.1, 0.1, out aR));
        MakeCirc aCheck = new(global::Kerf.Engine.Geometry.ElCLib.Value(-0.1, anElips), global::Kerf.Engine.Geometry.ElCLib.Value(0.0, anElips), global::Kerf.Engine.Geometry.ElCLib.Value(0.1, anElips));
        Assert.Equal(aCheck.Value().Radius(), aR);

        // three coincident points: gce_MakeCirc makes a circle of radius 0
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.GetRadius(new EdgeCurveAdaptor(anEllipseEdge), 0.5, 0.5, out aR));
        Assert.Equal(0.0, aR);
    }

    [Fact]
    public void PrepareArgs_LineAndCircle()
    {
        NCollection_Array1<double> anArgs = new(1, 1);
        Edge aLine = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.PrepareArgs(new EdgeCurveAdaptor(aLine), 10.0, 0.0, 4, 0.01, anArgs));
        Assert.Equal(0, anArgs.Lower());
        Assert.Equal(new[] { 0.0, 2.5, 5.0, 7.5, 10.0 }, Enumerable.Range(0, anArgs.Length()).Select(anArgs.Value));

        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        Edge anArc = new MakeEdge(aCirc, 0.0, Math.PI).Edge();
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.PrepareArgs(new EdgeCurveAdaptor(anArc), Math.PI, 0.0, 2, 0.001, anArgs));
        double[] aValues = Enumerable.Range(0, anArgs.Length()).Select(anArgs.Value).ToArray();
        Assert.True(aValues.Length > 3); // refined by the deflection
        Assert.Equal(0.0, aValues[0]);
        Assert.Equal(Math.PI, aValues[^1]);
        Assert.Contains(Math.PI / 2, aValues);
        for (int i = 1; i < aValues.Length; ++i)
        {
            Assert.True(aValues[i - 1] < aValues[i]);
        }

        // a step below 1e-5 isn't refined
        Assert.Equal(0, global::Kerf.Engine.Intersection.Tools.IntTools.PrepareArgs(new EdgeCurveAdaptor(anArc), 1.0e-5, 0.0, 2, 0.001, anArgs));
        Assert.Equal(3, anArgs.Length());
    }
}
