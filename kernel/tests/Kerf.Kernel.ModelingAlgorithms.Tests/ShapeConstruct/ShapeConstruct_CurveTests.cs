// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Construct;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeConstruct_Curve's other forms (lines by the segment form, 2D B-splines end by end, parts of 2D curves, knots
/// too close), and ShapeConstruct's joins and surface conversions with reversed curves, seams and revolutions.
/// </summary>
public class ShapeConstruct_CurveTests
{
    private const double Tol = 1e-9;

    private static BSplineCurve2D Spline2d() =>
        new(new[] { new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, 2), new Point2D(3, 0) }, [0.0, 1.0], [4, 4], 3);

    private static BSplineCurve3D Spline3d() =>
        new(new[] { new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, 2, 1), new Point3D(3, 0, 1) }, [0.0, 1.0], [4, 4], 3);

    [Fact]
    public void AdjustCurve_TakingNeitherEnd_LeavesTheCurve()
    {
        var aSpline = (BSplineCurve3D)Spline3d().Copy();
        Assert.True(new ConstructCurve().AdjustCurve(aSpline, new Point3D(9, 9, 9), new Point3D(9, 9, 9), false, false));
        Assert.True(aSpline.Value(0).IsEqual(new Point3D(0, 0, 0), Tol));
        Assert.True(aSpline.Value(1).IsEqual(new Point3D(3, 0, 1), Tol));

        BSplineCurve3D aSame = Spline3d();
        Assert.Same(aSame, new ConstructCurve().CurveAdjusted(aSame, new Point3D(), new Point3D(), out bool isDone, false, false));
        Assert.True(isDone);

        BSplineCurve2D aSame2d = Spline2d();
        Assert.Same(aSame2d, new ConstructCurve().Curve2dAdjusted(aSame2d, new Point2D(), new Point2D(), out isDone, false, false));
        Assert.True(isDone);
        Assert.True(new ConstructCurve().AdjustCurve2d(aSame2d, new Point2D(9, 9), new Point2D(9, 9), false, false));
        Assert.True(aSame2d.Value(0).IsEqual(new Point2D(0, 0), Tol));
    }

    [Fact]
    public void AdjustCurve2d_OnASpline_MovesOnlyTheEndsAskedFor()
    {
        BSplineCurve2D aOne = Spline2d();
        Assert.True(new ConstructCurve().AdjustCurve2d(aOne, new Point2D(-1, -1), new Point2D(9, 9), true, false));
        Assert.True(aOne.Value(0).IsEqual(new Point2D(-1, -1), Tol));
        Assert.True(aOne.Value(1).IsEqual(new Point2D(3, 0), Tol));

        BSplineCurve2D aOther = Spline2d();
        Assert.True(new ConstructCurve().AdjustCurve2d(aOther, new Point2D(9, 9), new Point2D(4, 1), false, true));
        Assert.True(aOther.Value(0).IsEqual(new Point2D(0, 0), Tol));
        Assert.True(aOther.Value(1).IsEqual(new Point2D(4, 1), Tol));

        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);
        Assert.False(new ConstructCurve().AdjustCurve2d(aCircle, new Point2D(0, 0), new Point2D(1, 1)));
        Assert.Same(aCircle, new ConstructCurve().Curve2dAdjusted(aCircle, new Point2D(0, 0), new Point2D(1, 1), out bool isDone));
        Assert.False(isDone);
    }

    [Fact]
    public void AdjustCurveSegment_OnALine_RunsItThroughBothPoints()
    {
        var aLine = new LineCurve3D(new Point3D(2, 0, 0), new Direction3D(1, 0, 0));
        Assert.True(new ConstructCurve().AdjustCurveSegment(aLine, new Point3D(0, 1, 0), new Point3D(1, 2, 0), 0, 1));
        Assert.True(aLine.Line.Direction.IsEqual(new Direction3D(1, 1, 0), 1e-12));
        // through the first point, whatever its origin
        Point3D aOrigin = aLine.Line.Location;
        Assert.Equal(aOrigin.X + 1, aOrigin.Y, 12);
    }

    [Fact]
    public void AdjustCurveSegment_RefusesAnEmptyRange_AndOtherCurves()
    {
        var aSpline = (BSplineCurve3D)Spline3d().Copy();
        Assert.False(new ConstructCurve().AdjustCurveSegment(aSpline, new Point3D(), new Point3D(1, 0, 0), 0.5, 0.5));
        Assert.Equal(1.0, aSpline.LastParameter, Tol);

        var aCircle = new CircleCurve3D(new Frame3D(), 1);
        Assert.False(new ConstructCurve().AdjustCurveSegment(aCircle, new Point3D(), new Point3D(1, 0, 0), 0, 1));
        Assert.Same(aCircle, new ConstructCurve().CurveSegmentAdjusted(aCircle, new Point3D(), new Point3D(1, 0, 0), 0, 1, out bool isDone));
        Assert.False(isDone);
    }

    [Fact]
    public void AdjustCurveSegment_KeepsWithinTheSplinesRange()
    {
        var aSpline = (BSplineCurve3D)Spline3d().Copy();
        Assert.True(new ConstructCurve().AdjustCurveSegment(aSpline, new Point3D(-1, 0, 0), new Point3D(5, 0, 0), -2, 0.5));
        Assert.Equal(0, aSpline.FirstParameter, Tol);
        Assert.Equal(0.5, aSpline.LastParameter, Tol);
        Assert.True(aSpline.Value(0).IsEqual(new Point3D(-1, 0, 0), Tol));
        Assert.True(aSpline.Value(0.5).IsEqual(new Point3D(5, 0, 0), Tol));
    }

    [Theory]
    [InlineData(0.25, 1.0)]
    [InlineData(0.0, 0.75)]
    [InlineData(0.25, 0.75)]
    public void ConvertToBSpline_OfPartOfASpline2d_IsThatPart(double theF, double theL)
    {
        BSplineCurve2D aSpline = Spline2d();
        BSplineCurve2D? aPart = new ConstructCurve().ConvertToBSpline(aSpline, theF, theL, 1e-5);
        Assert.NotNull(aPart);
        Assert.NotSame(aSpline, aPart);
        Assert.Equal(theF, aPart.FirstParameter, Tol);
        Assert.Equal(theL, aPart.LastParameter, Tol);
        Assert.True(aPart.Value(theF).IsEqual(aSpline.Value(theF), Tol));
        Assert.True(aPart.Value((theF + theL) / 2).IsEqual(aSpline.Value((theF + theL) / 2), Tol));
        Assert.True(aPart.Value(theL).IsEqual(aSpline.Value(theL), Tol));
        // the whole curve is itself
        Assert.Same(aSpline, new ConstructCurve().ConvertToBSpline(aSpline, 0, 1, 1e-5));
    }

    [Theory]
    [InlineData(0.25, 1.0)]
    [InlineData(0.0, 0.75)]
    public void ConvertToBSpline_OfPartOfASpline3d_IsThatPart(double theF, double theL)
    {
        BSplineCurve3D aSpline = Spline3d();
        BSplineCurve3D? aPart = new ConstructCurve().ConvertToBSpline(aSpline, theF, theL, 1e-5);
        Assert.NotNull(aPart);
        Assert.Equal(theF, aPart.FirstParameter, Tol);
        Assert.Equal(theL, aPart.LastParameter, Tol);
        Assert.True(aPart.Value(theF).IsEqual(aSpline.Value(theF), Tol));
        Assert.True(aPart.Value(theL).IsEqual(aSpline.Value(theL), Tol));
    }

    public static TheoryData<Curve2D> Curves2d() =>
    [
        new LineCurve2D(new Point2D(1, 1), new Direction2D(1, 0)),
        new BezierCurve2D(new[] { new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, 0) }),
        new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2),
    ];

    [Theory]
    [MemberData(nameof(Curves2d))]
    public void ConvertToBSpline_OfOtherCurves2d_FollowsTheCurve(Curve2D theCurve)
    {
        BSplineCurve2D? aSpline = new ConstructCurve().ConvertToBSpline(theCurve, 0.1, 0.9, 1e-6);
        Assert.NotNull(aSpline);
        for (int i = 0; i <= 8; i++)
        {
            double t = 0.1 + i * 0.1;
            Assert.True(aSpline.Value(t).DistanceTo(theCurve.Value(t)) < 1e-4, $"at {t}"); // for a Bezier, OCCT trims it twice (decision 19401)
        }
    }

    [Fact]
    public void ConvertToBSpline_OfPartOfABezier3d_IsThatPart()
    {
        // OCCT trims it twice (decision 19401)
        var aBezier = new BezierCurve3D(new[] { new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, 0, 1) });
        BSplineCurve3D? aPart = new ConstructCurve().ConvertToBSpline(aBezier, 0.25, 0.75, 1e-6);
        Assert.NotNull(aPart);
        Assert.Equal(0.25, aPart.FirstParameter, Tol);
        Assert.Equal(0.75, aPart.LastParameter, Tol);
        for (int i = 0; i <= 4; i++)
        {
            double t = 0.25 + i * 0.125;
            Assert.True(aPart.Value(t).DistanceTo(aBezier.Value(t)) < 1e-9, $"at {t}");
        }
    }

    [Fact]
    public void ConvertToBSpline_OfACircle3d_FollowsIt()
    {
        var aCircle = new CircleCurve3D(new Frame3D(), 2);
        BSplineCurve3D? aSpline = new ConstructCurve().ConvertToBSpline(aCircle, 0.1, 0.9, 1e-6);
        Assert.NotNull(aSpline);
        Assert.True(aSpline.Value(0.5).DistanceTo(aCircle.Value(0.5)) < 1e-4);
    }

    [Fact]
    public void FixKnots_SeparatesKnotsToClose_InBothForms()
    {
        var aH = new NCollection_HArray1<double>(1, 3);
        aH.SetValue(1, 1.0);
        aH.SetValue(2, 1.0);
        aH.SetValue(3, 2.0);
        Assert.True(ConstructCurve.FixKnots(aH));
        Assert.True(aH.Value(2) > aH.Value(1));
        Assert.Equal(2.0, aH.Value(3));
        // nothing further to do
        Assert.False(ConstructCurve.FixKnots(aH));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void JoinCurves2d_OfACurveRunningAway_RunsAlongTheJoin(bool theSayReversed)
    {
        // the first curve runs away from the join: said so, it's reversed at once; not said, the ends show it
        var aL1 = new BSplineCurve2D(new[] { new Point2D(1, 0), new Point2D(0, 0) }, [0.0, 1.0], [2, 2], 1);
        var aL2 = new BSplineCurve2D(new[] { new Point2D(1, 0), new Point2D(2, 0) }, [0.0, 1.0], [2, 2], 1);
        double aF1 = 0, aL1r = 1, aF2 = 0, aL2r = 1;
        Assert.True(ShapeConstruct.JoinCurves(aL1, aL2, theSayReversed ? Reversed : Forward, Forward, ref aF1, ref aL1r, ref aF2, ref aL2r, out Curve2D? aJoined, out bool aRev1, out bool aRev2));
        Assert.NotNull(aJoined);
        Assert.Equal(!theSayReversed, aRev1);
        Assert.False(aRev2);
        Assert.True(aJoined.Value(aJoined.FirstParameter).DistanceTo(new Point2D(0, 0)) < 1e-9);
        Assert.True(aJoined.Value(aJoined.LastParameter).DistanceTo(new Point2D(2, 0)) < 1e-9);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void JoinCurves3d_WithBothRunningTheWrongWay_RunsAlongTheJoin(bool theSayReversed)
    {
        var aC1 = new BSplineCurve3D(new[] { new Point3D(1, 0, 0), new Point3D(0, 0, 0) }, [0.0, 1.0], [2, 2], 1);
        var aC2 = new BSplineCurve3D(new[] { new Point3D(2, 0, 0), new Point3D(1, 0, 0) }, [0.0, 1.0], [2, 2], 1);
        Orientation anO = theSayReversed ? Reversed : Forward;
        double aF1 = 0, aL1 = 1, aF2 = 0, aL2 = 1;
        Assert.True(ShapeConstruct.JoinCurves(aC1, aC2, anO, anO, ref aF1, ref aL1, ref aF2, ref aL2, out Curve3D? aJoined, out bool aRev1, out bool aRev2));
        Assert.NotNull(aJoined);
        Assert.Equal(!theSayReversed, aRev1);
        Assert.Equal(!theSayReversed, aRev2);
        Assert.Equal(2, aJoined.Value(aJoined.FirstParameter).DistanceTo(aJoined.Value(aJoined.LastParameter)), 9);
    }

    [Fact]
    public void JoinCurves_OfCurvesThatDontMeet_BridgesThem()
    {
        // far apart: both ends move to the point between them, and it's one curve from end to end
        var aC1 = new BSplineCurve3D(new[] { new Point3D(0, 0, 0), new Point3D(1, 0, 0) }, [0.0, 1.0], [2, 2], 1);
        var aC2 = new BSplineCurve3D(new[] { new Point3D(5, 0, 0), new Point3D(6, 0, 0) }, [0.0, 1.0], [2, 2], 1);
        double aF1 = 0, aL1 = 1, aF2 = 0, aL2 = 1;
        bool aDone = ShapeConstruct.JoinCurves(aC1, aC2, Forward, Forward, ref aF1, ref aL1, ref aF2, ref aL2, out Curve3D? aJoined, out _, out _);
        Assert.True(aDone);
        Assert.True(aJoined!.Value(aJoined.FirstParameter).DistanceTo(new Point3D(0, 0, 0)) < 1e-7);
        Assert.True(aJoined.Value(aJoined.LastParameter).DistanceTo(new Point3D(6, 0, 0)) < 1e-7);
    }

    [Fact]
    public void ConvertSurfaceToBSpline_OfARevolution_KeepsTheSurfacesPoints()
    {
        var aBasis = new BezierCurve3D(new[] { new Point3D(2, 0, 0), new Point3D(3, 0, 1), new Point3D(2, 0, 2) });
        var aRevol = new RevolutionSurface(aBasis, new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        BSplineSurface? aB = ShapeConstruct.ConvertSurfaceToBSpline(aRevol, 0, 2, 0, 1, 1e-4, C1, 9, 10);
        Assert.NotNull(aB);
        for (int i = 0; i <= 4; i++)
        {
            double u = i * 0.5, v = i * 0.25;
            Assert.True(aB.Value(u, v).DistanceTo(aRevol.Value(u, v)) < 1e-2, $"at {u} {v}");
        }
    }

    [Fact]
    public void ConvertSurfaceToBSpline_OfATrimmedSurface_ConvertsItsBasis()
    {
        var aCylinder = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3);
        var aTrimmed = new TrimmedSurface(aCylinder, 0, 1, 0, 2);
        BSplineSurface? aB = ShapeConstruct.ConvertSurfaceToBSpline(aTrimmed, 0, 1, 0, 2, 1e-4, C1, 9, 10);
        Assert.NotNull(aB);
        Point3D aP = aB.Value(0.5, 1);
        Assert.Equal(3, Math.Sqrt(aP.X * aP.X + aP.Y * aP.Y), 2);
    }

    [Fact]
    public void ConvertCurveToBSpline_OfATrimmedOffset2d_FollowsIt()
    {
        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2);
        var aOffset = new OffsetCurve2D(aCircle, 1);
        var aTrimmed = new TrimmedCurve2D(aOffset, 0, 1);
        BSplineCurve2D? aB = ShapeConstruct.ConvertCurveToBSpline(aTrimmed, 0, 1, 1e-5, C1, 100, 9);
        Assert.NotNull(aB);
        Assert.True(aB.Value(0.5).DistanceTo(aTrimmed.Value(0.5)) < 1e-3);
    }

    [Fact]
    public void JoinPCurves_ASeamInTwoPieces_JoinsBothItsPCurves()
    {
        // both pieces of a seam of a cylinder, each with its two pcurves: the joined edge is a seam over the whole range
        var aCylinder = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5);
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, aCylinder, 1e-7);
        var aSide = new LineCurve3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1));
        Edge Piece(double aFrom, double aTo)
        {
            Edge anE = new MakeEdge(aSide, aFrom, aTo).InternalEdge();
            var aAt0 = new BSplineCurve2D(new[] { new Point2D(0, aFrom), new Point2D(0, aTo) }, [aFrom, aTo], [2, 2], 1);
            var aAt2Pi = new BSplineCurve2D(new[] { new Point2D(2 * Math.PI, aFrom), new Point2D(2 * Math.PI, aTo) }, [aFrom, aTo], [2, 2], 1);
            aB.UpdateEdge(anE, aAt0, aAt2Pi, aF, 1e-7);
            aB.Range(anE, aF, aFrom, aTo);
            return anE;
        }
        Edge aE1 = Piece(0, 2), aE2 = Piece(2, 5);
        var anAll = new MakeEdge(aSide, 0, 5).InternalEdge();
        var anEdges = new NCollection_HSequence<Shape>();
        anEdges.Append(aE1);
        anEdges.Append(aE2);

        Edge aJoined = ShapeConstruct.PCurvesJoined(anEdges, TestShapes.Frozen(aF), TestShapes.Frozen(anAll), out _);
        Assert.True(Tool.IsClosed(aJoined, aF));
    }
}
