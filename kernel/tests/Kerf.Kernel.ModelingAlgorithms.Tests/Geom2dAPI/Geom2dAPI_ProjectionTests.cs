// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;
using Occt;
using Xunit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Known answers of Geom2dAPI_ProjectPointOnCurve and Geom2dAPI_ExtremaCurveCurve.</summary>
public class Geom2dAPI_ProjectionTests
{
    [Fact]
    public void ProjectPointOnCircle()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(1, 1), new Direction2D(1, 0)), 2.0);
        ProjectPointOnCurve2D aProj = new(new Point2D(4, 1), aCircle);
        Assert.Equal(2, aProj.NbPoints());
        Assert.Equal(1.0, aProj.LowerDistance(), 1e-12);
        Assert.Equal(0.0, aProj.LowerDistanceParameter(), 1e-10);
        Assert.Equal(3.0, aProj.NearestPoint().X, 1e-12);
        Assert.Equal(1.0, aProj.NearestPoint().Y, 1e-12);
        Assert.Equal(1.0, (double)aProj, 1e-12);
        Assert.Equal(2, (int)aProj);
        double aMax = Math.Max(aProj.Distance(1), aProj.Distance(2));
        Assert.Equal(5.0, aMax, 1e-12);
    }

    [Fact]
    public void ProjectPointOnLine_WithRange()
    {
        LineCurve2D aLine = new(new Point2D(0, 0), new Direction2D(1, 0));
        ProjectPointOnCurve2D aProj = new(new Point2D(3, 2), aLine, -10, 10);
        Assert.Equal(1, aProj.NbPoints());
        Assert.Equal(3.0, aProj.Parameter(1), 1e-12);
        aProj.Parameter(1, out double aU);
        Assert.Equal(3.0, aU, 1e-12);
        Assert.Equal(2.0, aProj.Distance(1), 1e-12);

        // Outside the range: no orthogonal projection.
        aProj.Init(new Point2D(30, 2), aLine, -10, 10);
        Assert.Equal(0, aProj.NbPoints());
        EngineAssert.ThrowsAny<NotDoneException>(() => aProj.NearestPoint());
        EngineAssert.ThrowsAny<OutOfRangeException>(() => aProj.Point(1));
    }

    [Fact]
    public void ProjectPointOnBSpline()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 2);
        aPoles[3] = new Point2D(3, 2);
        aPoles[4] = new Point2D(4, 0);
        BezierCurve2D aBez = new(aPoles);
        Point2D aP = new(2, 3);
        ProjectPointOnCurve2D aProj = new(aP, aBez);
        Assert.True(aProj.NbPoints() >= 1);
        double aU = aProj.LowerDistanceParameter();
        // The symmetric curve's apex (t = 1/2) is the nearest point: (2, 1.5).
        Assert.Equal(0.5, aU, 1e-9);
        Assert.Equal(1.5, aProj.LowerDistance(), 1e-9);
        Assert.Same(aProj.Extrema(), aProj.Extrema());
    }

    [Fact]
    public void ProjectPoint_Empty()
    {
        ProjectPointOnCurve2D aProj = new();
        Assert.Equal(0, aProj.NbPoints());
        EngineAssert.ThrowsAny<NotDoneException>(() => aProj.LowerDistance());
    }

    [Fact]
    public void ExtremaCircleLine()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        LineCurve2D aLine = new(new Point2D(-10, 3), new Direction2D(1, 0));
        ExtremaCurveCurve2D anExt = new(aCircle, aLine, 0, 2 * Math.PI, -20, 20);
        Assert.True(anExt.NbExtrema() >= 1);
        Assert.Equal(2.0, anExt.LowerDistance(), 1e-12);
        anExt.NearestPoints(out Point2D aP1, out Point2D aP2);
        Assert.Equal(0.0, aP1.X, 1e-10);
        Assert.Equal(1.0, aP1.Y, 1e-10);
        Assert.Equal(0.0, aP2.X, 1e-10);
        Assert.Equal(3.0, aP2.Y, 1e-10);
        anExt.LowerDistanceParameters(out double aU1, out double aU2);
        Assert.Equal(Math.PI / 2, aU1, 1e-10);
        Assert.Equal(10.0, aU2, 1e-10);
        Assert.Equal(2.0, (double)anExt, 1e-12);
        Assert.Equal(anExt.NbExtrema(), (int)anExt);
    }

    [Fact]
    public void ExtremaBSplineBSpline()
    {
        NCollection_Array1<Point2D> aPoles1 = new(1, 3);
        aPoles1[1] = new Point2D(0, 0);
        aPoles1[2] = new Point2D(1, 1);
        aPoles1[3] = new Point2D(2, 0);
        NCollection_Array1<Point2D> aPoles2 = new(1, 3);
        aPoles2[1] = new Point2D(0, 3);
        aPoles2[2] = new Point2D(1, 2);
        aPoles2[3] = new Point2D(2, 3);
        BezierCurve2D aC1 = new(aPoles1);
        BezierCurve2D aC2 = new(aPoles2);
        ExtremaCurveCurve2D anExt = new(aC1, aC2, 0, 1, 0, 1);
        Assert.True(anExt.NbExtrema() >= 1);
        // Apexes at (1, 0.5) and (1, 2.5).
        Assert.Equal(2.0, anExt.LowerDistance(), 1e-9);
        anExt.LowerDistanceParameters(out double aU1, out double aU2);
        Assert.Equal(0.5, aU1, 1e-7);
        Assert.Equal(0.5, aU2, 1e-7);
        EngineAssert.ThrowsAny<OutOfRangeException>(() => anExt.Distance(anExt.NbExtrema() + 1));
    }
}
