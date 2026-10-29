// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using Xunit;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of Geom2dAPI_ProjectPointOnCurve and Geom2dAPI_ExtremaCurveCurve.</summary>
public class Geom2dAPI_ProjectionTests
{
    [Fact]
    public void ProjectPointOnCircle()
    {
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 0)), 2.0);
        Geom2dAPI_ProjectPointOnCurve aProj = new(new gp_Pnt2d(4, 1), aCircle);
        Assert.Equal(2, aProj.NbPoints());
        Assert.Equal(1.0, aProj.LowerDistance(), 1e-12);
        Assert.Equal(0.0, aProj.LowerDistanceParameter(), 1e-10);
        Assert.Equal(3.0, aProj.NearestPoint().X(), 1e-12);
        Assert.Equal(1.0, aProj.NearestPoint().Y(), 1e-12);
        Assert.Equal(1.0, (double)aProj, 1e-12);
        Assert.Equal(2, (int)aProj);
        double aMax = Math.Max(aProj.Distance(1), aProj.Distance(2));
        Assert.Equal(5.0, aMax, 1e-12);
    }

    [Fact]
    public void ProjectPointOnLine_WithRange()
    {
        Geom2d_Line aLine = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        Geom2dAPI_ProjectPointOnCurve aProj = new(new gp_Pnt2d(3, 2), aLine, -10, 10);
        Assert.Equal(1, aProj.NbPoints());
        Assert.Equal(3.0, aProj.Parameter(1), 1e-12);
        aProj.Parameter(1, out double aU);
        Assert.Equal(3.0, aU, 1e-12);
        Assert.Equal(2.0, aProj.Distance(1), 1e-12);

        // Outside the range: no orthogonal projection.
        aProj.Init(new gp_Pnt2d(30, 2), aLine, -10, 10);
        Assert.Equal(0, aProj.NbPoints());
        Assert.ThrowsAny<StdFail_NotDone>(() => aProj.NearestPoint());
        Assert.ThrowsAny<Standard_OutOfRange>(() => aProj.Point(1));
    }

    [Fact]
    public void ProjectPointOnBSpline()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt2d(0, 0);
        aPoles[2] = new gp_Pnt2d(1, 2);
        aPoles[3] = new gp_Pnt2d(3, 2);
        aPoles[4] = new gp_Pnt2d(4, 0);
        Geom2d_BezierCurve aBez = new(aPoles);
        gp_Pnt2d aP = new(2, 3);
        Geom2dAPI_ProjectPointOnCurve aProj = new(aP, aBez);
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
        Geom2dAPI_ProjectPointOnCurve aProj = new();
        Assert.Equal(0, aProj.NbPoints());
        Assert.ThrowsAny<StdFail_NotDone>(() => aProj.LowerDistance());
    }

    [Fact]
    public void ExtremaCircleLine()
    {
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.0);
        Geom2d_Line aLine = new(new gp_Pnt2d(-10, 3), new gp_Dir2d(1, 0));
        Geom2dAPI_ExtremaCurveCurve anExt = new(aCircle, aLine, 0, 2 * Math.PI, -20, 20);
        Assert.True(anExt.NbExtrema() >= 1);
        Assert.Equal(2.0, anExt.LowerDistance(), 1e-12);
        anExt.NearestPoints(out gp_Pnt2d aP1, out gp_Pnt2d aP2);
        Assert.Equal(0.0, aP1.X(), 1e-10);
        Assert.Equal(1.0, aP1.Y(), 1e-10);
        Assert.Equal(0.0, aP2.X(), 1e-10);
        Assert.Equal(3.0, aP2.Y(), 1e-10);
        anExt.LowerDistanceParameters(out double aU1, out double aU2);
        Assert.Equal(Math.PI / 2, aU1, 1e-10);
        Assert.Equal(10.0, aU2, 1e-10);
        Assert.Equal(2.0, (double)anExt, 1e-12);
        Assert.Equal(anExt.NbExtrema(), (int)anExt);
    }

    [Fact]
    public void ExtremaBSplineBSpline()
    {
        NCollection_Array1<gp_Pnt2d> aPoles1 = new(1, 3);
        aPoles1[1] = new gp_Pnt2d(0, 0);
        aPoles1[2] = new gp_Pnt2d(1, 1);
        aPoles1[3] = new gp_Pnt2d(2, 0);
        NCollection_Array1<gp_Pnt2d> aPoles2 = new(1, 3);
        aPoles2[1] = new gp_Pnt2d(0, 3);
        aPoles2[2] = new gp_Pnt2d(1, 2);
        aPoles2[3] = new gp_Pnt2d(2, 3);
        Geom2d_BezierCurve aC1 = new(aPoles1);
        Geom2d_BezierCurve aC2 = new(aPoles2);
        Geom2dAPI_ExtremaCurveCurve anExt = new(aC1, aC2, 0, 1, 0, 1);
        Assert.True(anExt.NbExtrema() >= 1);
        // Apexes at (1, 0.5) and (1, 2.5).
        Assert.Equal(2.0, anExt.LowerDistance(), 1e-9);
        anExt.LowerDistanceParameters(out double aU1, out double aU2);
        Assert.Equal(0.5, aU1, 1e-7);
        Assert.Equal(0.5, aU2, 1e-7);
        Assert.ThrowsAny<Standard_OutOfRange>(() => anExt.Distance(anExt.NbExtrema() + 1));
    }
}
