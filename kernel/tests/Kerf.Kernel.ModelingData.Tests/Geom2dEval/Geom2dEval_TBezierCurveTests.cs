// Port of: src/ModelingData/TKG2d/GTests/Geom2dEval_TBezierCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// Left out: DumpJson_NoCrash, DumpJson_Rational_NoCrash (DumpJson is not ported).
public class Geom2dEval_TBezierCurveTest
{
    private const double THE_FD_TOL_D1 = 1e-5;
    private const double THE_FD_TOL_D2 = 1e-4;
    private const double THE_FD_TOL_D3 = 1e-3;

    // Helper: T-Bezier unit semicircle C(t) = (cos(t), sin(t)), t in [0, Pi].
    private static TBezierCurve2D createSemicircle2d()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(0.0, 1.0));
        aPoles.SetValue(3, new Point2D(1.0, 0.0));
        return new TBezierCurve2D(aPoles, 1.0);
    }

    // Helper: T-Bezier semi-ellipse (a=3, b=2): C(t) = (3*cos(t), 2*sin(t)).
    private static TBezierCurve2D createSemiEllipse2d()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(0.0, 2.0));
        aPoles.SetValue(3, new Point2D(3.0, 0.0));
        return new TBezierCurve2D(aPoles, 1.0);
    }

    // Helper to create a simple 3-pole (order 1) non-rational T-Bezier curve
    private static TBezierCurve2D createSimpleCurve()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        return new TBezierCurve2D(aPoles, 1.0);
    }

    // Helper to create a 5-pole (order 2) non-rational T-Bezier curve
    private static TBezierCurve2D createQuadraticCurve()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 5);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 2.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        aPoles.SetValue(4, new Point2D(3.0, -1.0));
        aPoles.SetValue(5, new Point2D(4.0, 0.5));
        return new TBezierCurve2D(aPoles, 0.5);
    }

    [Fact]
    public void Construction_Linear()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        Assert.Equal(3, aCurve.PoleCount);
        Assert.Equal(1, aCurve.Order());
        Assert.Equal(1.0, aCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(aCurve.IsRational);
    }

    [Fact]
    public void Construction_Quadratic()
    {
        TBezierCurve2D aCurve = createQuadraticCurve();
        Assert.Equal(5, aCurve.PoleCount);
        Assert.Equal(2, aCurve.Order());
        Assert.Equal(0.5, aCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(aCurve.IsRational);
    }

    [Fact]
    public void Construction_InvalidParams_Throws()
    {
        // Even number of poles (not odd)
        NCollection_Array1<Point2D> aPoles2 = new(1, 2);
        aPoles2.SetValue(1, new Point2D(0.0, 0.0));
        aPoles2.SetValue(2, new Point2D(1.0, 0.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve2D(aPoles2, 1.0));

        // Too few poles
        NCollection_Array1<Point2D> aPoles1 = new(1, 1);
        aPoles1.SetValue(1, new Point2D(0.0, 0.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve2D(aPoles1, 1.0));

        // Invalid alpha
        NCollection_Array1<Point2D> aPoles3 = new(1, 3);
        aPoles3.SetValue(1, new Point2D(0.0, 0.0));
        aPoles3.SetValue(2, new Point2D(1.0, 0.0));
        aPoles3.SetValue(3, new Point2D(2.0, 0.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve2D(aPoles3, 0.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve2D(aPoles3, -1.0));
    }

    [Fact]
    public void EvalD0_Endpoints()
    {
        TBezierCurve2D aCurve = createSimpleCurve();

        Point2D aStart = aCurve.EvalD0(aCurve.FirstParameter);
        Point2D aEnd = aCurve.EvalD0(aCurve.LastParameter);

        Assert.Equal(0.0, aStart.DistanceTo(aCurve.StartPoint), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aEnd.DistanceTo(aCurve.EndPoint), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_Midpoint()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        double aMid = (aCurve.FirstParameter + aCurve.LastParameter) / 2.0;
        Point2D aPt = aCurve.EvalD0(aMid);

        // Verify the point is finite and within a reasonable range
        Assert.True(double.IsFinite(aPt.X));
        Assert.True(double.IsFinite(aPt.Y));
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        double aU = M_PI / 3.0;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Point2D aP1 = aCurve.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point2D aP2 = aCurve.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aP1.XY() - aP2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD1.D1.X, THE_FD_TOL_D1);
        Assert.Equal(aFD.Y, aD1.D1.Y, THE_FD_TOL_D1);
    }

    [Fact]
    public void EvalD1_QuadraticConsistentWithD0()
    {
        TBezierCurve2D aCurve = createQuadraticCurve();
        double aU = M_PI / 4.0;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Point2D aP1 = aCurve.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point2D aP2 = aCurve.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aP1.XY() - aP2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD1.D1.X, THE_FD_TOL_D1);
        Assert.Equal(aFD.Y, aD1.D1.Y, THE_FD_TOL_D1);
    }

    [Fact]
    public void Bounds()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        Assert.Equal(0.0, aCurve.FirstParameter, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(M_PI / 1.0, aCurve.LastParameter, global::Kerf.Engine.Foundation.Precision.Confusion());

        TBezierCurve2D aCurve2 = createQuadraticCurve();
        Assert.Equal(0.0, aCurve2.FirstParameter, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(M_PI / 0.5, aCurve2.LastParameter, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void IsRational_NonRational()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        Assert.False(aCurve.IsRational);
    }

    [Fact]
    public void IsRational_WithWeights()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 2.0);
        aWeights.SetValue(3, 1.0);
        TBezierCurve2D aCurve = new(aPoles, aWeights, 1.0);
        Assert.True(aCurve.IsRational);
        Assert.Equal(3, aCurve.Weights().Count);
    }

    [Fact]
    public void Construction_Rational_MismatchedWeights_Throws()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aWeights.SetValue(i, 1.0);
        }
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve2D(aPoles, aWeights, 1.0));
    }

    [Fact]
    public void Construction_Rational_NegativeWeight_Throws()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, -1.0);
        aWeights.SetValue(3, 1.0);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve2D(aPoles, aWeights, 1.0));
    }

    [Fact]
    public void NbPoles_Alpha()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        Assert.Equal(3, aCurve.PoleCount);
        Assert.Equal(1.0, aCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());

        TBezierCurve2D aCurve2 = createQuadraticCurve();
        Assert.Equal(5, aCurve2.PoleCount);
        Assert.Equal(0.5, aCurve2.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(1.0, 2.0));

        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transform(aTrsf));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        Geometry2D aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        TBezierCurve2D? aCopyCurve = aCopy as TBezierCurve2D;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(3, aCopyCurve.PoleCount);
        Assert.Equal(1.0, aCopyCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Verify copy evaluates the same
        double aU = M_PI / 4.0;
        Point2D aOrigPt = aCurve.EvalD0(aU);
        Point2D aCopyPt = aCopyCurve.EvalD0(aU);
        Assert.Equal(0.0, aOrigPt.DistanceTo(aCopyPt), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy_Modification_Independent()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        Geometry2D aCopy = aCurve.Copy();
        TBezierCurve2D? aCopyCurve = aCopy as TBezierCurve2D;
        Assert.NotNull(aCopyCurve);

        // Transform is unsupported on the copy as well.
        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(10.0, 10.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCopyCurve.Transform(aTrsf));

        // Copy and original remain identical.
        double aU = M_PI / 4.0;
        Point2D aOrigPt = aCurve.EvalD0(aU);
        Point2D aCopyPt = aCopyCurve.EvalD0(aU);
        Assert.Equal(0.0, aOrigPt.DistanceTo(aCopyPt), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void PeriodicityAndContinuity()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        Assert.False(aCurve.IsPeriodic);
        Assert.Equal(Continuity.CN, aCurve.Continuity);
        Assert.True(aCurve.IsCN(0));
        Assert.True(aCurve.IsCN(10));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Reverse());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void EvalD1_Rational_ConsistentWithD0()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 3.0);
        aWeights.SetValue(3, 1.0);
        TBezierCurve2D aCurve = new(aPoles, aWeights, 1.0);

        double aU = M_PI / 3.0;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Point2D aP1 = aCurve.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point2D aP2 = aCurve.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aP1.XY() - aP2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD1.D1.X, THE_FD_TOL_D1);
        Assert.Equal(aFD.Y, aD1.D1.Y, THE_FD_TOL_D1);
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        double aU = M_PI / 4.0;

        Curve2D.ResD2 aD2 = aCurve.EvalD2(aU);
        Curve2D.ResD1 aD1Plus = aCurve.EvalD1(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve2D.ResD1 aD1Minus = aCurve.EvalD1(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aD1Plus.D1.XY() - aD1Minus.D1.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD2.D2.X, THE_FD_TOL_D2);
        Assert.Equal(aFD.Y, aD2.D2.Y, THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        double aU = M_PI / 3.0;

        Curve2D.ResD3 aD3 = aCurve.EvalD3(aU);
        Curve2D.ResD2 aD2Plus = aCurve.EvalD2(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve2D.ResD2 aD2Minus = aCurve.EvalD2(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aD2Plus.D2.XY() - aD2Minus.D2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD3.D3.X, THE_FD_TOL_D3);
        Assert.Equal(aFD.Y, aD3.D3.Y, THE_FD_TOL_D3);
    }

    [Fact]
    public void EvalDN_ConsistentWithD1()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        double aU = M_PI / 6.0;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Vector2D aDN = aCurve.EvalDN(aU, 1);

        Assert.Equal(aDN.X, aD1.D1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aDN.Y, aD1.D1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_MultiplePoints()
    {
        TBezierCurve2D aCurve = createSimpleCurve();
        double aFirst = aCurve.FirstParameter;
        double aLast = aCurve.LastParameter;
        int aNbPoints = 6;

        for (int i = 0; i < aNbPoints; ++i)
        {
            double aU = aFirst + (aLast - aFirst) * i / (aNbPoints - 1);
            Point2D aPt = aCurve.EvalD0(aU);
            Assert.True(double.IsFinite(aPt.X));
            Assert.True(double.IsFinite(aPt.Y));
        }
    }

    [Fact]
    public void EvalD3_MatchesCircle()
    {
        TBezierCurve2D aTBez = createSemicircle2d();
        // Geom2d_Circle (unit 1B) evaluates through ElCLib; the circle is used directly.
        Circle2D aCircle = new(new Frame2D(), 1.0);

        double[] aParams = [M_PI / 6.0, M_PI / 3.0, M_PI / 2.0, 2.0 * M_PI / 3.0];
        foreach (double aU in aParams)
        {
            Curve2D.ResD3 aDT = aTBez.EvalD3(aU);
            Curve2D.ResD3 aDC;
            global::Kerf.Engine.Geometry.ElCLib.D3(aU, aCircle, out aDC.Point, out aDC.D1, out aDC.D2, out aDC.D3);

            Assert.Equal(0.0, aDT.Point.DistanceTo(aDC.Point), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.X, aDT.D1.X, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.Y, aDT.D1.Y, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.X, aDT.D2.X, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.Y, aDT.D2.Y, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D3.X, aDT.D3.X, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D3.Y, aDT.D3.Y, global::Kerf.Engine.Foundation.Precision.Angular());
        }
    }

    [Fact]
    public void EvalD2_MatchesEllipse()
    {
        TBezierCurve2D aTBez = createSemiEllipse2d();
        // Geom2d_Ellipse (unit 1B) evaluates through ElCLib; the ellipse is used directly.
        Ellipse2D anEllipse = new(new Frame2D(), 3.0, 2.0);

        double[] aParams = [M_PI / 6.0, M_PI / 4.0, M_PI / 2.0, 3.0 * M_PI / 4.0];
        foreach (double aU in aParams)
        {
            Curve2D.ResD2 aDT = aTBez.EvalD2(aU);
            Curve2D.ResD2 aDC;
            global::Kerf.Engine.Geometry.ElCLib.D2(aU, anEllipse, out aDC.Point, out aDC.D1, out aDC.D2);

            Assert.Equal(0.0, aDT.Point.DistanceTo(aDC.Point), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.X, aDT.D1.X, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.Y, aDT.D1.Y, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.X, aDT.D2.X, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.Y, aDT.D2.Y, global::Kerf.Engine.Foundation.Precision.Angular());
        }
    }

    // OCCT.NET addition: every derivative against central differences of the one below, along the curve,
    // for non-rational and rational curves of orders 1 to 3.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 7);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 2.0));
        aPoles.SetValue(3, new Point2D(2.0, -0.5));
        aPoles.SetValue(4, new Point2D(3.0, -1.0));
        aPoles.SetValue(5, new Point2D(4.0, 0.5));
        aPoles.SetValue(6, new Point2D(-1.0, 1.5));
        aPoles.SetValue(7, new Point2D(0.5, 0.25));
        NCollection_Array1<double> aWeights = new(1, 7);
        double[] aW = [3.0, 1.2, 0.7, 2.0, 1.5, 0.9, 1.1];
        for (int i = 1; i <= 7; ++i)
        {
            aWeights.SetValue(i, aW[i - 1]);
        }

        foreach (int aNb in new[] { 3, 5, 7 })
        {
            NCollection_Array1<Point2D> aP = new(aPoles, 1, 1, aNb);
            NCollection_Array1<double> aWt = new(aWeights, 1, 1, aNb);
            TBezierCurve2D aCurve = new(aP, 0.8);
            TBezierCurve2D aRational = new(aP, aWt, 0.8);
            for (double aU = 0.05; aU < aCurve.LastParameter; aU += 0.31)
            {
                EvalTestUtils.CheckCurveDerivatives(aCurve, aU);
                EvalTestUtils.CheckCurveDerivatives(aRational, aU, 6);
            }
        }
    }

    // OCCT.NET addition: Poles() and Weights() are 0-based copies.
    [Fact]
    public void Poles_Weights_AreCopies()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.Init(2.0);
        TBezierCurve2D aCurve = new(aPoles, aWeights, 1.0);
        aPoles.SetValue(2, new Point2D(5.0, 5.0));
        aWeights.SetValue(2, 7.0);
        Assert.Equal(new Point2D(1.0, 1.0), aCurve.Poles()[1]);
        Assert.Equal(2.0, aCurve.Weights()[1]);
        Assert.Empty(createSimpleCurve().Weights());
    }
}
