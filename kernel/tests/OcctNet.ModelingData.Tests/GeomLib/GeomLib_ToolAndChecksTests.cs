// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of GeomLib_Tool, GeomLib_CheckBSplineCurve, GeomLib_Check2dBSplineCurve,
/// GeomLib_Interpolate and the paths of GeomLib_CheckCurveOnSurface the GTests don't reach.
/// </summary>
public class GeomLib_ToolAndChecksTests
{
    private static Geom_BSplineCurve Wavy3d()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 7);
        for (int i = 1; i <= 7; i++)
        {
            aPoles[i] = new gp_Pnt(i, Math.Sin(i), 0.3 * Math.Cos(2.0 * i));
        }
        NCollection_Array1<double> aKnots = new(1, 5);
        NCollection_Array1<int> aMults = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aKnots[i] = i - 1;
            aMults[i] = 1;
        }
        aMults[1] = 4;
        aMults[5] = 4;
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    [Fact]
    public void Tool_Parameter_Curve_FindsPointOnCurve()
    {
        Geom_BSplineCurve aCurve = Wavy3d();
        for (double u = 0.0; u <= 4.0; u += 0.37)
        {
            gp_Pnt aP = aCurve.Value(u);
            Assert.True(GeomLib_Tool.Parameter(aCurve, aP, 1e-7, out double aU));
            Assert.Equal(u, aU, 1e-7);
        }
        // Too far
        Assert.False(GeomLib_Tool.Parameter(aCurve, new gp_Pnt(2.0, 50.0, 0.0), 1.0, out double aFar));
        Assert.Equal(0.0, aFar);
        Assert.False(GeomLib_Tool.Parameter((Geom_Curve?)null, new gp_Pnt(), 1.0, out _));
    }

    [Fact]
    public void Tool_Parameter_Curve2d_FindsPointOnCircle()
    {
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0)), 3.0);
        Assert.True(GeomLib_Tool.Parameter(aCircle, aCircle.Value(2.5), 1e-7, out double aU));
        Assert.Equal(2.5, aU, 1e-9);
        // A point 0.1 off the circle: found within 0.2, not within 0.05
        gp_Pnt2d anOff = new(1.0 + 3.1 * Math.Cos(1.0), 2.0 + 3.1 * Math.Sin(1.0));
        Assert.True(GeomLib_Tool.Parameter(aCircle, anOff, 0.2, out aU));
        Assert.Equal(1.0, aU, 1e-9);
        Assert.False(GeomLib_Tool.Parameter(aCircle, anOff, 0.05, out _));
    }

    [Fact]
    public void Tool_Parameters_Surface_FindsPointOnSphere()
    {
        Geom_SphericalSurface aSphere = new(new gp_Ax3(new gp_Pnt(1, 1, 1), new gp_Dir(0, 0, 1)), 2.0);
        gp_Pnt aP = aSphere.Value(1.2, 0.4);
        Assert.True(GeomLib_Tool.Parameters(aSphere, aP, 1e-7, out double aU, out double aV));
        Assert.Equal(1.2, aU, 1e-8);
        Assert.Equal(0.4, aV, 1e-8);
        Assert.False(GeomLib_Tool.Parameters(aSphere, new gp_Pnt(10, 10, 10), 1.0, out aU, out aV));
        Assert.Equal(0.0, aU);
        Assert.Equal(0.0, aV);
    }

    [Fact]
    public void Tool_ComputeDeviation_OfCircularArc()
    {
        // Arc of radius 1 from -a to a: the sagitta 1 - cos(a) is at parameter 0.
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 1.0);
        Geom2dAdaptor_Curve anAC = new(aCircle);
        double a = 0.8;
        double aPrm = -10.0;
        double aRough = GeomLib_Tool.ComputeDeviation(anAC, -a, a, 5, 10, ref aPrm);
        Assert.True(aRough > 0.0);
        Assert.InRange(aPrm, -a, a);

        double aPrmOn = 0.0;
        gp_Pnt2d aPt = new();
        gp_Vec2d aVec = new();
        gp_Lin2d aLin = new();
        double aDefl = GeomLib_Tool.ComputeDeviation(anAC, -a, a, aPrm, 100, ref aPrmOn, ref aPt, ref aVec, ref aLin);
        Assert.Equal(1.0 - Math.Cos(a), aDefl, 1e-12);
        Assert.Equal(0.0, aPrmOn, 1e-7);
        Assert.Equal(1.0, aPt.X(), 1e-12);
        Assert.Equal(-(1.0 - Math.Cos(a)), aVec.X(), 1e-12);
        Assert.Equal(Math.Cos(a), aLin.Location().X(), 1e-15);
        Assert.Equal(1.0, Math.Abs(aLin.Direction().Y()), 1e-15);
        Assert.Equal(aDefl, GeomLib_Tool.ComputeDeviation(anAC, -a, a, aPrm));

        // Start outside the range, or a degenerate chord
        Assert.Equal(-1.0, GeomLib_Tool.ComputeDeviation(anAC, -a, a, 2.0));
        Assert.Equal(-1.0, GeomLib_Tool.ComputeDeviation(anAC, 0.0, 2.0 * Math.PI, 1.0));
        Assert.Equal(-1.0, GeomLib_Tool.ComputeDeviation(anAC, 0.0, 2.0 * Math.PI, 4));
    }

    [Fact]
    public void CheckBSplineCurve_DetectsAndFixesReversedTangents()
    {
        // The second pole lies behind the first, the third ahead on the same line: reversed start tangent.
        NCollection_Array1<gp_Pnt> aPoles = new(1, 6);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(-0.1, 0, 0);
        aPoles[3] = new gp_Pnt(1, 0, 0);
        aPoles[4] = new gp_Pnt(2, 1, 0);
        aPoles[5] = new gp_Pnt(3, 1, 0);
        aPoles[6] = new gp_Pnt(4, 1, 0);
        NCollection_Array1<double> aKnots = new(1, 4);
        NCollection_Array1<int> aMults = new(1, 4);
        aKnots[1] = 0; aKnots[2] = 1; aKnots[3] = 2; aKnots[4] = 3;
        aMults[1] = 4; aMults[2] = 1; aMults[3] = 1; aMults[4] = 4;
        Geom_BSplineCurve aCurve = new(aPoles, aKnots, aMults, 3);

        GeomLib_CheckBSplineCurve aCheck = new(aCurve, 1e-7, 1e-7);
        Assert.False(aCheck.IsDone());
        aCheck.NeedTangentFix(out bool aFirst, out bool aLast);
        Assert.True(aFirst);
        Assert.False(aLast);

        Assert.Null(aCheck.FixedTangent(false, true));
        Geom_BSplineCurve? aFixed = aCheck.FixedTangent(true, true);
        Assert.NotNull(aFixed);
        Assert.True(aCheck.IsDone());
        // Pole 2 is moved halfway between poles 1 and 3; the input is untouched.
        Assert.Equal(new gp_Pnt(0.5, 0, 0), aFixed!.Pole(2));
        Assert.Equal(new gp_Pnt(-0.1, 0, 0), aCurve.Pole(2));

        // In place (internal, as IntTools does), on a curve the caller has just built
        aCheck.FixTangent(true, true);
        Assert.Equal(new gp_Pnt(0.5, 0, 0), aCurve.Pole(2));

        // A frozen curve can't be fixed in place
        Geom_BSplineCurve aFrozen = (Geom_BSplineCurve)aFixed.Copy();
        aFrozen.SetPole(2, new gp_Pnt(-0.1, 0, 0));
        aFrozen.Freeze();
        GeomLib_CheckBSplineCurve aCheckFrozen = new(aFrozen, 1e-7, 1e-7);
        Assert.Throws<Standard_ImmutableObject>(() => aCheckFrozen.FixTangent(true, true));

        // A curve without reversed tangents needs no fix
        GeomLib_CheckBSplineCurve aWavy = new(Wavy3d(), 1e-7, 1e-7);
        aWavy.NeedTangentFix(out aFirst, out aLast);
        Assert.False(aFirst || aLast);
        Assert.Null(aWavy.FixedTangent(true, true));
    }

    [Fact]
    public void CheckBSplineCurve_PeriodicOrShortCurves_AreDone()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(-1, 0, 0);
        aPoles[3] = new gp_Pnt(1, 0, 0);
        NCollection_Array1<double> aKnots = new(1, 2);
        NCollection_Array1<int> aMults = new(1, 2);
        aKnots[1] = 0; aKnots[2] = 1;
        aMults[1] = 3; aMults[2] = 3;
        Assert.True(new GeomLib_CheckBSplineCurve(new Geom_BSplineCurve(aPoles, aKnots, aMults, 2), 1e-7, 1e-7).IsDone());
    }

    [Fact]
    public void Check2dBSplineCurve_DetectsReversedLastTangent()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 5);
        aPoles[1] = new gp_Pnt2d(0, 0);
        aPoles[2] = new gp_Pnt2d(1, 1);
        aPoles[3] = new gp_Pnt2d(2, 0);
        aPoles[4] = new gp_Pnt2d(3.2, 0);
        aPoles[5] = new gp_Pnt2d(3, 0);
        NCollection_Array1<double> aKnots = new(1, 3);
        NCollection_Array1<int> aMults = new(1, 3);
        aKnots[1] = 0; aKnots[2] = 1; aKnots[3] = 2;
        aMults[1] = 4; aMults[2] = 1; aMults[3] = 4;
        Geom2d_BSplineCurve aCurve = new(aPoles, aKnots, aMults, 3);

        GeomLib_Check2dBSplineCurve aCheck = new(aCurve, 1e-7, 1e-7);
        aCheck.NeedTangentFix(out bool aFirst, out bool aLast);
        Assert.False(aFirst);
        Assert.True(aLast);
        Geom2d_BSplineCurve? aFixed = aCheck.FixedTangent(false, true);
        Assert.NotNull(aFixed);
        // The pre-last pole is moved halfway between the last pole and pole 3.
        Assert.Equal(new gp_Pnt2d(2.5, 0), aFixed!.Pole(4));
        Assert.Equal(new gp_Pnt2d(3.2, 0), aCurve.Pole(4));
    }

    [Fact]
    public void Interpolate_PassesThroughPoints()
    {
        int aNb = 6;
        NCollection_Array1<gp_Pnt> aPoints = new(1, aNb);
        NCollection_Array1<double> aParams = new(1, aNb);
        for (int i = 1; i <= aNb; i++)
        {
            aParams[i] = (i - 1) * 0.5;
            aPoints[i] = new gp_Pnt(i, Math.Sin(i), i * i * 0.1);
        }
        foreach (int aDegree in new[] { 3, 4, 5 })
        {
            GeomLib_Interpolate anInterp = new(aDegree, aNb, aPoints, aParams);
            Assert.True(anInterp.IsDone());
            Assert.Equal(GeomLib_InterpolationErrors.GeomLib_NoError, anInterp.Error());
            Geom_BSplineCurve aCurve = anInterp.Curve()!;
            Assert.True(aCurve.IsFrozen());
            Assert.Same(aCurve, anInterp.Curve());
            // An even degree gives degree - 1
            Assert.Equal(aDegree % 2 == 0 ? aDegree - 1 : aDegree, aCurve.Degree());
            for (int i = 1; i <= aNb; i++)
            {
                Assert.True(aCurve.Value(aParams[i]).Distance(aPoints[i]) < 1e-10);
            }
        }

        Assert.Equal(GeomLib_InterpolationErrors.GeomLib_DegreeSmallerThan3,
                     new GeomLib_Interpolate(2, aNb, aPoints, aParams).Error());
        GeomLib_Interpolate aTooFew = new(7, aNb, aPoints, aParams);
        Assert.False(aTooFew.IsDone());
        Assert.Equal(GeomLib_InterpolationErrors.GeomLib_NotEnoughtPoints, aTooFew.Error());
        Assert.Null(aTooFew.Curve());
    }

    [Fact]
    public void CheckCurveOnSurface_BSplines_ParallelMatchesSequential()
    {
        // A B-spline 3D curve against the image of a B-spline 2D curve on a plane: the general (PSO + Newton) path.
        Geom_BSplineCurve aC3 = Wavy3d();
        NCollection_Array1<gp_Pnt2d> aPoles2d = new(1, 7);
        for (int i = 1; i <= 7; i++)
        {
            aPoles2d[i] = new gp_Pnt2d(i, Math.Sin(i) + 0.05 * Math.Cos(3.0 * i));
        }
        NCollection_Array1<double> aKnots = new(1, 5);
        NCollection_Array1<int> aMults = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aKnots[i] = i - 1;
            aMults[i] = 1;
        }
        aMults[1] = 4;
        aMults[5] = 4;
        Geom2d_BSplineCurve aC2 = new(aPoles2d, aKnots, aMults, 3);
        Geom_Plane aPlane = new(new gp_Ax3(gp.XOY()));
        Adaptor3d_CurveOnSurface aCoS = new(new Geom2dAdaptor_Curve(aC2), new GeomAdaptor_Surface(aPlane));
        GeomAdaptor_Curve aGC = new(aC3);

        GeomLib_CheckCurveOnSurface aSeq = new(aGC);
        aSeq.Perform(aCoS);
        Assert.True(aSeq.IsDone());

        // Dense sampling bounds the maximum from below.
        double aSampled = 0.0;
        for (int i = 0; i <= 4000; i++)
        {
            double t = 4.0 * i / 4000;
            aSampled = Math.Max(aSampled, aC3.Value(t).Distance(aCoS.Value(t)));
        }
        Assert.True(aSeq.MaxDistance() >= aSampled - 1e-9);
        Assert.True(aSeq.MaxDistance() <= aSampled + 1e-4);
        double aPrm = aSeq.MaxParameter();
        Assert.Equal(aSeq.MaxDistance(), aC3.Value(aPrm).Distance(aCoS.Value(aPrm)), 1e-12);

        GeomLib_CheckCurveOnSurface aPar = new(aGC);
        aPar.SetParallel(true);
        Assert.True(aPar.IsParallel());
        aPar.Perform(aCoS);
        Assert.True(aPar.IsDone());
        Assert.Equal(aSeq.MaxDistance(), aPar.MaxDistance());
        Assert.Equal(aSeq.MaxParameter(), aPar.MaxParameter());
    }

    [Fact]
    public void CheckCurveOnSurface_ErrorStatuses()
    {
        GeomLib_CheckCurveOnSurface anEmpty = new();
        anEmpty.Perform(null);
        Assert.Equal(1, anEmpty.ErrorStatus());
        Assert.False(anEmpty.IsDone());

        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        Geom2d_Line aLine2d = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        Adaptor3d_CurveOnSurface aCoS = new(new Geom2dAdaptor_Curve(aLine2d, 0.0, 1.0),
                                            new GeomAdaptor_Surface(new Geom_Plane(new gp_Ax3(gp.XOY()))));

        // The 3D range doesn't cover the curve on surface's
        GeomLib_CheckCurveOnSurface aCheck = new(new GeomAdaptor_Curve(aLine, 0.0, 0.5));
        aCheck.Perform(aCoS);
        Assert.Equal(2, aCheck.ErrorStatus());
        Assert.Equal(RealLast(), aCheck.MaxDistance());

        aCheck.Init(new GeomAdaptor_Curve(aLine, 0.0, 1.0));
        aCheck.Perform(aCoS);
        Assert.True(aCheck.IsDone());
        Assert.Equal(0.0, aCheck.MaxDistance());

        aCheck.Init();
        aCheck.Perform(aCoS);
        Assert.Equal(1, aCheck.ErrorStatus());
    }
}
