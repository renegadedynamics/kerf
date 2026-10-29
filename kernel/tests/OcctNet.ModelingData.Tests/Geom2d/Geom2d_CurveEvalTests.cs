// Port of: src/ModelingData/TKG2d/GTests/Geom2d_CurveEval_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_CurveEvalTest
{
    // OCCT.NET: OCCT builds the B-splines below with Geom2dAPI_PointsToBSpline, which isn't ported yet; the port's
    // tests use a cubic B-spline through the same kind of points, built from poles directly.
    private static Geom2d_BSplineCurve MakeBSpline(params gp_Pnt2d[] thePoles)
    {
        NCollection_Array1<gp_Pnt2d> aPoles = NCollection_Array1<gp_Pnt2d>.From(thePoles);
        int aDeg = Math.Min(3, thePoles.Length - 1);
        int aNbKnots = thePoles.Length - aDeg + 1;
        NCollection_Array1<double> aKnots = new NCollection_Array1<double>(1, aNbKnots);
        NCollection_Array1<int> aMults = new NCollection_Array1<int>(1, aNbKnots);
        for (int i = 1; i <= aNbKnots; ++i)
        {
            aKnots[i] = i - 1;
            aMults[i] = 1;
        }
        aMults[1] = aDeg + 1;
        aMults[aNbKnots] = aDeg + 1;
        return new Geom2d_BSplineCurve(aPoles, aKnots, aMults, aDeg);
    }

    // Circle 2D tests

    [Fact]
    public void Circle_EvalD0D1_ValidResults()
    {
        gp_Circ2d aCirc = new gp_Circ2d(new gp_Ax22d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 4.0);
        Geom2d_Circle aCurve = new Geom2d_Circle(aCirc);
        double aU = M_PI / 4.0;

        // EvalD0
        gp_Pnt2d aD0 = aCurve.EvalD0(aU);
        double aDist = aD0.Distance(new gp_Pnt2d(0.0, 0.0));
        Assert.Equal(4.0, aDist, Precision.Confusion());

        // EvalD1
        Geom2d_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        Assert.Equal(0.0, aD1.Point.Distance(aD0), Precision.Confusion());
        // D1 should be perpendicular to radius
        gp_Vec2d aRadial = new gp_Vec2d(new gp_Pnt2d(0.0, 0.0), aD1.Point);
        Assert.Equal(0.0, aRadial.Dot(aD1.D1), 1.0e-10);
    }

    // OffsetCurve 2D failure-path test

    [Fact]
    public void OffsetCurve_EvalD0_ValidAtDegenerateCenter()
    {
        // Create a circle with offset = -radius to get a degenerate (zero-radius) curve
        gp_Circ2d aCirc = new gp_Circ2d(new gp_Ax22d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 3.0);
        Geom2d_Circle aBasis = new Geom2d_Circle(aCirc);

        // Offset by -radius creates a collapsed curve at the center
        Geom2d_OffsetCurve anOffset = new Geom2d_OffsetCurve(aBasis, -3.0);

        gp_Pnt2d aResult = anOffset.EvalD0(0.0);
        Assert.Equal(0.0, aResult.Distance(new gp_Pnt2d(0.0, 0.0)), Precision.Confusion());
    }

    // BSplineCurve 2D consistency tests

    [Fact]
    public void BSplineCurve_EvalD1_ConsistentWithOldAPI()
    {
        // OCCT.NET: built from poles (Geom2dAPI_PointsToBSpline not ported)
        Geom2d_BSplineCurve aCurve = MakeBSpline(new gp_Pnt2d(0.0, 0.0),
                                                 new gp_Pnt2d(1.0, 2.0),
                                                 new gp_Pnt2d(3.0, 1.0),
                                                 new gp_Pnt2d(5.0, 3.0),
                                                 new gp_Pnt2d(7.0, 0.0));

        double aFirst = aCurve.FirstParameter();
        double aLast = aCurve.LastParameter();
        double aMid = (aFirst + aLast) / 2.0;

        // Compare EvalD1 with old D1 wrapper
        Geom2d_Curve.ResD1 aEvalResult = aCurve.EvalD1(aMid);
        aCurve.D1(aMid, out gp_Pnt2d aOldP, out gp_Vec2d aOldV1);
        Assert.Equal(0.0, aEvalResult.Point.Distance(aOldP), Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D1 - aOldV1).Magnitude(), Precision.Confusion());
    }

    // Ellipse 2D tests

    [Fact]
    public void Ellipse_EvalD2D3_ValidResults()
    {
        gp_Elips2d anElips = new gp_Elips2d(new gp_Ax22d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0, 3.0);
        Geom2d_Ellipse aCurve = new Geom2d_Ellipse(anElips);
        double aU = M_PI / 4.0;

        Geom2d_Curve.ResD2 aD2 = aCurve.EvalD2(aU);
        Assert.True(aD2.D2.Magnitude() > 0.0);

        Geom2d_Curve.ResD3 aD3 = aCurve.EvalD3(aU);
        Assert.True(aD3.D3.Magnitude() > 0.0);
    }

    // Line 2D tests

    [Fact]
    public void Line_EvalDN_ValidResults()
    {
        Geom2d_Line aCurve = new Geom2d_Line(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0));

        // DN(U, 1) should return the direction
        gp_Vec2d aDN1 = aCurve.EvalDN(5.0, 1);
        Assert.Equal(1.0, aDN1.X(), Precision.Confusion());
        Assert.Equal(0.0, aDN1.Y(), Precision.Confusion());

        // DN(U, 2) should return zero for a line
        gp_Vec2d aDN2 = aCurve.EvalDN(5.0, 2);
        Assert.Equal(0.0, aDN2.Magnitude(), Precision.Confusion());
    }

    // BSplineCurve 2D EvalDN invalid N test

    [Fact]
    public void BSplineCurve_EvalDN_InvalidN_Throws()
    {
        // OCCT.NET: built from poles (Geom2dAPI_PointsToBSpline not ported)
        Geom2d_BSplineCurve aCurve =
            MakeBSpline(new gp_Pnt2d(0.0, 0.0), new gp_Pnt2d(1.0, 1.0), new gp_Pnt2d(2.0, 0.0));

        double aMid = (aCurve.FirstParameter() + aCurve.LastParameter()) / 2.0;
        Assert.ThrowsAny<Geom2d_UndefinedDerivative>(() => aCurve.EvalDN(aMid, 0));
        Assert.ThrowsAny<Geom2d_UndefinedDerivative>(() => aCurve.EvalDN(aMid, -1));
    }

    // OffsetCurve 2D EvalD1 singular test

    [Fact]
    public void OffsetCurve_EvalD1_DegenerateAtCollapsed()
    {
        // Create a circle with offset = -radius to get a degenerate (zero-radius) curve
        gp_Circ2d aCirc = new gp_Circ2d(new gp_Ax22d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 3.0);
        Geom2d_Circle aBasis = new Geom2d_Circle(aCirc);

        // The basis circle has valid tangents everywhere, so EvalD1 succeeds
        // but produces a degenerate result (all points map to the center).
        Geom2d_OffsetCurve anOffset = new Geom2d_OffsetCurve(aBasis, -3.0);

        Geom2d_Curve.ResD1 aResult = anOffset.EvalD1(0.0);
        // The point should be near the center
        Assert.Equal(0.0, aResult.Point.Distance(new gp_Pnt2d(0.0, 0.0)), Precision.Confusion());
    }
}
