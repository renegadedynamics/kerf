// Port of: src/ModelingData/TKG2d/GTests/Geom2d_CurveEval_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_CurveEvalTest
{
    // OCCT.NET: OCCT builds the B-splines below with Geom2dAPI_PointsToBSpline, which isn't ported yet; the port's
    // tests use a cubic B-spline through the same kind of points, built from poles directly.
    private static BSplineCurve2D MakeBSpline(params Point2D[] thePoles)
    {
        NCollection_Array1<Point2D> aPoles = NCollection_Array1<Point2D>.From(thePoles);
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
        return new BSplineCurve2D(aPoles, aKnots, aMults, aDeg);
    }

    // Circle 2D tests

    [Fact]
    public void Circle_EvalD0D1_ValidResults()
    {
        Circle2D aCirc = new Circle2D(new Frame2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 4.0);
        CircleCurve2D aCurve = new CircleCurve2D(aCirc);
        double aU = M_PI / 4.0;

        // EvalD0
        Point2D aD0 = aCurve.EvalD0(aU);
        double aDist = aD0.Distance(new Point2D(0.0, 0.0));
        Assert.Equal(4.0, aDist, global::Kerf.Engine.Foundation.Precision.Confusion());

        // EvalD1
        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Assert.Equal(0.0, aD1.Point.Distance(aD0), global::Kerf.Engine.Foundation.Precision.Confusion());
        // D1 should be perpendicular to radius
        Vector2D aRadial = new Vector2D(new Point2D(0.0, 0.0), aD1.Point);
        Assert.Equal(0.0, aRadial.Dot(aD1.D1), 1.0e-10);
    }

    // OffsetCurve 2D failure-path test

    [Fact]
    public void OffsetCurve_EvalD0_ValidAtDegenerateCenter()
    {
        // Create a circle with offset = -radius to get a degenerate (zero-radius) curve
        Circle2D aCirc = new Circle2D(new Frame2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 3.0);
        CircleCurve2D aBasis = new CircleCurve2D(aCirc);

        // Offset by -radius creates a collapsed curve at the center
        OffsetCurve2D anOffset = new OffsetCurve2D(aBasis, -3.0);

        Point2D aResult = anOffset.EvalD0(0.0);
        Assert.Equal(0.0, aResult.Distance(new Point2D(0.0, 0.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // BSplineCurve 2D consistency tests

    [Fact]
    public void BSplineCurve_EvalD1_ConsistentWithOldAPI()
    {
        // OCCT.NET: built from poles (Geom2dAPI_PointsToBSpline not ported)
        BSplineCurve2D aCurve = MakeBSpline(new Point2D(0.0, 0.0),
                                                 new Point2D(1.0, 2.0),
                                                 new Point2D(3.0, 1.0),
                                                 new Point2D(5.0, 3.0),
                                                 new Point2D(7.0, 0.0));

        double aFirst = aCurve.FirstParameter();
        double aLast = aCurve.LastParameter();
        double aMid = (aFirst + aLast) / 2.0;

        // Compare EvalD1 with old D1 wrapper
        Curve2D.ResD1 aEvalResult = aCurve.EvalD1(aMid);
        aCurve.D1(aMid, out Point2D aOldP, out Vector2D aOldV1);
        Assert.Equal(0.0, aEvalResult.Point.Distance(aOldP), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, (aEvalResult.D1 - aOldV1).Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Ellipse 2D tests

    [Fact]
    public void Ellipse_EvalD2D3_ValidResults()
    {
        Ellipse2D anElips = new Ellipse2D(new Frame2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0, 3.0);
        EllipseCurve2D aCurve = new EllipseCurve2D(anElips);
        double aU = M_PI / 4.0;

        Curve2D.ResD2 aD2 = aCurve.EvalD2(aU);
        Assert.True(aD2.D2.Magnitude() > 0.0);

        Curve2D.ResD3 aD3 = aCurve.EvalD3(aU);
        Assert.True(aD3.D3.Magnitude() > 0.0);
    }

    // Line 2D tests

    [Fact]
    public void Line_EvalDN_ValidResults()
    {
        LineCurve2D aCurve = new LineCurve2D(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0));

        // DN(U, 1) should return the direction
        Vector2D aDN1 = aCurve.EvalDN(5.0, 1);
        Assert.Equal(1.0, aDN1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDN1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // DN(U, 2) should return zero for a line
        Vector2D aDN2 = aCurve.EvalDN(5.0, 2);
        Assert.Equal(0.0, aDN2.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // BSplineCurve 2D EvalDN invalid N test

    [Fact]
    public void BSplineCurve_EvalDN_InvalidN_Throws()
    {
        // OCCT.NET: built from poles (Geom2dAPI_PointsToBSpline not ported)
        BSplineCurve2D aCurve =
            MakeBSpline(new Point2D(0.0, 0.0), new Point2D(1.0, 1.0), new Point2D(2.0, 0.0));

        double aMid = (aCurve.FirstParameter() + aCurve.LastParameter()) / 2.0;
        Assert.ThrowsAny<UndefinedDerivative2DException>(() => aCurve.EvalDN(aMid, 0));
        Assert.ThrowsAny<UndefinedDerivative2DException>(() => aCurve.EvalDN(aMid, -1));
    }

    // OffsetCurve 2D EvalD1 singular test

    [Fact]
    public void OffsetCurve_EvalD1_DegenerateAtCollapsed()
    {
        // Create a circle with offset = -radius to get a degenerate (zero-radius) curve
        Circle2D aCirc = new Circle2D(new Frame2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 3.0);
        CircleCurve2D aBasis = new CircleCurve2D(aCirc);

        // The basis circle has valid tangents everywhere, so EvalD1 succeeds
        // but produces a degenerate result (all points map to the center).
        OffsetCurve2D anOffset = new OffsetCurve2D(aBasis, -3.0);

        Curve2D.ResD1 aResult = anOffset.EvalD1(0.0);
        // The point should be near the center
        Assert.Equal(0.0, aResult.Point.Distance(new Point2D(0.0, 0.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
