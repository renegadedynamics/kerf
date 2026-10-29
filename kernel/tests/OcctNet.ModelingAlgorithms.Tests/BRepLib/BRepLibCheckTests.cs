// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepLib_ValidateEdge and BRepLib_CheckCurveOnSurface on edges with exact and offset pcurves.</summary>
public class BRepLibCheckTests
{
    private const double Tol = 1e-7;

    /// <summary>
    /// A segment from (0, 0, 0) to (2, 0, 0) on the plane z = 0 (located by <paramref name="theLoc"/>, curve and
    /// surface alike), with its pcurve moved by <paramref name="theOffset"/> in V.
    /// </summary>
    private static (TopoDS_Edge Edge, TopoDS_Face Face) EdgeOnPlane(double theOffset, TopLoc_Location? theLoc = null)
    {
        TopLoc_Location aLoc = theLoc ?? new TopLoc_Location();
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, aLoc, Tol);
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE, new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), aLoc, Tol);
        aB.Range(anE, 0.0, 2.0);
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, theOffset), new gp_Dir2d(1, 0)), aF, Tol);
        return (anE, aF);
    }

    private static BRepLib_ValidateEdge Validate(double theOffset, bool theSameParameter, double theOtherShift = 0.0)
    {
        var aRef = new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0.0, 2.0);
        var aPC = new Geom2dAdaptor_Curve(new Geom2d_Line(new gp_Pnt2d(-theOtherShift, theOffset), new gp_Dir2d(1, 0)),
                                          0.0 + theOtherShift, 2.0 + theOtherShift);
        var aCOS = new Adaptor3d_CurveOnSurface(aPC, new GeomAdaptor_Surface(new Geom_Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1))));
        return new BRepLib_ValidateEdge(aRef, aCOS, theSameParameter);
    }

    [Fact]
    public void CheckCurveOnSurface_ExactPCurve()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = EdgeOnPlane(0.0);
        var aCheck = new BRepLib_CheckCurveOnSurface(anE, aF);
        aCheck.Perform();
        Assert.True(aCheck.IsDone());
        Assert.Equal(0, aCheck.ErrorStatus());
        Assert.Equal(0.0, aCheck.MaxDistance(), 1e-15);
    }

    [Fact]
    public void CheckCurveOnSurface_OffsetPCurve()
    {
        foreach (bool isParallel in new[] { false, true })
        {
            (TopoDS_Edge anE, TopoDS_Face aF) = EdgeOnPlane(0.01);
            var aCheck = new BRepLib_CheckCurveOnSurface();
            aCheck.SetParallel(isParallel);
            Assert.Equal(isParallel, aCheck.IsParallel());
            aCheck.Init(anE, aF);
            aCheck.Perform();
            Assert.True(aCheck.IsDone());
            Assert.Equal(0.01, aCheck.MaxDistance(), 1e-12);
            Assert.InRange(aCheck.MaxParameter(), 0.0, 2.0);
        }
    }

    [Fact]
    public void CheckCurveOnSurface_LocatedEdgeAndFace()
    {
        var aT = new gp_Trsf();
        aT.SetRotation(new gp_Ax1(new gp_Pnt(1, 2, 3), new gp_Dir(1, 1, 0)), 0.7);
        aT.SetTranslationPart(new gp_Vec(5, -3, 2));
        (TopoDS_Edge anE, TopoDS_Face aF) = EdgeOnPlane(0.02, new TopLoc_Location(aT));
        var aCheck = new BRepLib_CheckCurveOnSurface(anE, aF);
        aCheck.Perform();
        Assert.True(aCheck.IsDone());
        Assert.Equal(0.02, aCheck.MaxDistance(), 1e-12);
    }

    [Fact]
    public void CheckCurveOnSurface_SeamOfACylinder()
    {
        TopoDS_Face aF = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            var aCheck = new BRepLib_CheckCurveOnSurface(TopoDS.Edge(anExp.Current()), aF);
            aCheck.Perform();
            Assert.True(aCheck.IsDone());
            Assert.True(aCheck.MaxDistance() < 1e-12, $"distance {aCheck.MaxDistance()}");
        }
    }

    [Fact]
    public void CheckCurveOnSurface_NothingToDo()
    {
        // null shapes leave the algorithm without data: status 1 (null curve)
        var aCheck = new BRepLib_CheckCurveOnSurface(new TopoDS_Edge(), new TopoDS_Face());
        aCheck.Perform();
        Assert.False(aCheck.IsDone());
        Assert.Equal(1, aCheck.ErrorStatus());
    }

    [Fact]
    public void ValidateEdge_ApproxAndExact()
    {
        foreach (bool isExact in new[] { false, true })
        {
            BRepLib_ValidateEdge aValidate = Validate(0.001, true);
            aValidate.SetExactMethod(isExact);
            Assert.Equal(isExact, aValidate.IsExactMethod());
            aValidate.Process();
            Assert.True(aValidate.IsDone());
            Assert.Equal(0.001 * 1.00001, aValidate.GetMaxDistance(), 1e-15);
            Assert.True(aValidate.CheckTolerance(0.0011));
            Assert.False(aValidate.CheckTolerance(0.0009));
            double aTol = 1e-7;
            aValidate.UpdateTolerance(ref aTol);
            Assert.Equal(0.001 * 1.00001, aTol, 1e-15);
            double aBig = 1.0;
            aValidate.UpdateTolerance(ref aBig);
            Assert.Equal(1.0, aBig);
        }
    }

    [Fact]
    public void ValidateEdge_ExactPCurve()
    {
        BRepLib_ValidateEdge aValidate = Validate(0.0, true);
        aValidate.Process();
        Assert.True(aValidate.IsDone());
        Assert.Equal(0.0, aValidate.GetMaxDistance());
        Assert.True(aValidate.CheckTolerance(1e-7));
    }

    [Fact]
    public void ValidateEdge_ProjectionWhenNotSameParameter()
    {
        // ranges differ: the distances are found by projection (Extrema_LocateExtPC), also when SameParameter
        foreach (bool isSameParameter in new[] { false, true })
        {
            BRepLib_ValidateEdge aValidate = Validate(0.003, isSameParameter, 0.5);
            aValidate.SetControlPointsNumber(10);
            aValidate.Process();
            Assert.True(aValidate.IsDone());
            Assert.Equal(0.003 * 1.00001, aValidate.GetMaxDistance(), 1e-12);
        }

        // the exact method (only for SameParameter) needs equal ranges: GeomLib_CheckCurveOnSurface fails (status 2)
        BRepLib_ValidateEdge anExact = Validate(0.003, true, 0.5);
        anExact.SetExactMethod(true);
        anExact.Process();
        Assert.False(anExact.IsDone());
        Assert.Equal(0.0, anExact.GetMaxDistance());

        // and is ignored for an edge that isn't SameParameter
        BRepLib_ValidateEdge anIgnored = Validate(0.003, false, 0.5);
        anIgnored.SetExactMethod(true);
        anIgnored.Process();
        Assert.True(anIgnored.IsDone());
        Assert.Equal(0.003 * 1.00001, anIgnored.GetMaxDistance(), 1e-12);
    }

    [Fact]
    public void ValidateEdge_ExitIfToleranceExceeded()
    {
        BRepLib_ValidateEdge aValidate = Validate(0.01, true);
        aValidate.SetParallel(true);
        Assert.True(aValidate.IsParallel());
        aValidate.SetExitIfToleranceExceeded(1e-3);
        aValidate.Process();
        Assert.True(aValidate.IsDone());
        Assert.False(aValidate.CheckTolerance(1e-3));
        Assert.Equal(0.01 * 1.00001, aValidate.GetMaxDistance(), 1e-15);

        // fewer than one control point counts as one
        BRepLib_ValidateEdge aOne = Validate(0.01, true);
        aOne.SetControlPointsNumber(0);
        aOne.Process();
        Assert.Equal(0.01 * 1.00001, aOne.GetMaxDistance(), 1e-15);
    }

    [Fact]
    public void ValidateEdge_OnAnEdgeOfACone()
    {
        // the cone's numbers widen the tolerance by Epsilon(apex coordinates, radius)
        var aCone = new Geom_ConicalSurface(new gp_Ax3(new gp_Pnt(0, 0, 1e6), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 0.5, 3.0);
        var aPC = new Geom2dAdaptor_Curve(new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 0.0, 2 * Math.PI);
        var aCOS = new Adaptor3d_CurveOnSurface(aPC, new GeomAdaptor_Surface(aCone));
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 1e6), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 3.0);
        var aValidate = new BRepLib_ValidateEdge(new GeomAdaptor_Curve(aCircle, 0.0, 2 * Math.PI), aCOS, true);
        aValidate.Process();
        Assert.True(aValidate.IsDone());
        Assert.True(aValidate.GetMaxDistance() < 1e-9);
        double aDelta = Epsilon(1e6);
        Assert.True(aValidate.CheckTolerance(aValidate.GetMaxDistance() - aDelta / 2));
    }
}
