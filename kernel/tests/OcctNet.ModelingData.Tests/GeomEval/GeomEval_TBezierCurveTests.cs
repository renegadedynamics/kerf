// Port of: src/ModelingData/TKG3d/GTests/GeomEval_TBezierCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class GeomEval_TBezierCurveTest
{
    private const double THE_FD_TOL_D1 = 1e-5;
    private const double THE_FD_TOL_D2 = 1e-4;

    // Helper to create a T-Bezier that represents a unit semicircle in XY plane.
    // C(t) = P_0 + P_1*sin(t) + P_2*cos(t) = (cos(t), sin(t), 0), t in [0, Pi].
    private static TBezierCurve createSemicircle()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(0.0, 1.0, 0.0));
        aPoles.SetValue(3, new Point3D(1.0, 0.0, 0.0));
        return new TBezierCurve(aPoles, 1.0);
    }

    // Helper to create a T-Bezier that represents a semi-ellipse (a=3, b=2) in XY plane.
    // C(t) = P_0 + P_1*sin(t) + P_2*cos(t) = (3*cos(t), 2*sin(t), 0), t in [0, Pi].
    private static TBezierCurve createSemiEllipse()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(0.0, 2.0, 0.0));
        aPoles.SetValue(3, new Point3D(3.0, 0.0, 0.0));
        return new TBezierCurve(aPoles, 1.0);
    }

    // Helper to create a simple 3-pole (order 1) T-Bezier curve
    private static TBezierCurve createSimpleCurve()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(1.0, 1.0, 0.0));
        aPoles.SetValue(3, new Point3D(2.0, 0.0, 0.0));
        return new TBezierCurve(aPoles, 1.0);
    }


    [Fact]
    public void Construction_ValidParams()
    {
        TBezierCurve aCurve = createSimpleCurve();
        Assert.Equal(3, aCurve.PoleCount);
        Assert.Equal(1, aCurve.Order());
        Assert.Equal(1.0, aCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(aCurve.IsRational);
    }

    [Fact]
    public void Construction_InvalidParams_Throws()
    {
        // Even number of poles (not odd)
        NCollection_Array1<Point3D> aPoles2 = new(1, 2);
        aPoles2.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles2.SetValue(2, new Point3D(1.0, 0.0, 0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve(aPoles2, 1.0));

        // Invalid alpha
        NCollection_Array1<Point3D> aPoles3 = new(1, 3);
        aPoles3.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles3.SetValue(2, new Point3D(1.0, 0.0, 0.0));
        aPoles3.SetValue(3, new Point3D(2.0, 0.0, 0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve(aPoles3, 0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierCurve(aPoles3, -1.0));
    }

    [Fact]
    public void EvalD0_Endpoints()
    {
        TBezierCurve aCurve = createSimpleCurve();

        Point3D aStart = aCurve.EvalD0(aCurve.FirstParameter());
        Point3D aEnd = aCurve.EvalD0(aCurve.LastParameter());

        Assert.Equal(0.0, aStart.Distance(aCurve.StartPoint()), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aEnd.Distance(aCurve.EndPoint()), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ParameterRange()
    {
        TBezierCurve aCurve = createSimpleCurve();
        Assert.Equal(0.0, aCurve.FirstParameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(M_PI / 1.0, aCurve.LastParameter(), global::Kerf.Engine.Foundation.Precision.Confusion()); // Pi/alpha
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        TBezierCurve aCurve = createSimpleCurve();
        double aU = M_PI / 3.0;

        Curve3D.ResD1 aD1 = aCurve.EvalD1(aU);
        Point3D aP1 = aCurve.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D aP2 = aCurve.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector3D aFD = new((aP1.XYZ() - aP2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X(), aD1.D1.X(), THE_FD_TOL_D1);
        Assert.Equal(aFD.Y(), aD1.D1.Y(), THE_FD_TOL_D1);
        Assert.Equal(aFD.Z(), aD1.D1.Z(), THE_FD_TOL_D1);
    }

    [Fact]
    public void PeriodicityAndClosure()
    {
        TBezierCurve aCurve = createSimpleCurve();
        Assert.False(aCurve.IsPeriodic);
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        TBezierCurve aCurve = createSimpleCurve();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Reverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void Construction_Rational()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(1.0, 1.0, 0.0));
        aPoles.SetValue(3, new Point3D(2.0, 0.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 2.0);
        aWeights.SetValue(3, 1.0);
        TBezierCurve aCurve = new(aPoles, aWeights, 1.0);
        Assert.True(aCurve.IsRational);
        Assert.Equal(3, aCurve.Weights().Count);
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        TBezierCurve aCurve = createSimpleCurve();
        Transform3D aTrsf = new();
        aTrsf.SetTranslation(new Vector3D(1.0, 2.0, 3.0));

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transform(aTrsf));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        TBezierCurve aCurve = createSimpleCurve();
        Geometry3D aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        TBezierCurve? aCopyCurve = aCopy as TBezierCurve;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(3, aCopyCurve.PoleCount);
        Assert.Equal(1.0, aCopyCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        TBezierCurve aCurve = createSimpleCurve();
        double aU = M_PI / 4.0;

        Curve3D.ResD2 aD2 = aCurve.EvalD2(aU);
        Curve3D.ResD1 aD1Plus = aCurve.EvalD1(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve3D.ResD1 aD1Minus = aCurve.EvalD1(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector3D aFD = new((aD1Plus.D1.XYZ() - aD1Minus.D1.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X(), aD2.D2.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Y(), aD2.D2.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Z(), aD2.D2.Z(), THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        TBezierCurve aCurve = createSimpleCurve();
        double aU = M_PI / 3.0;

        Curve3D.ResD3 aD3 = aCurve.EvalD3(aU);
        Curve3D.ResD2 aD2Plus = aCurve.EvalD2(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve3D.ResD2 aD2Minus = aCurve.EvalD2(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector3D aFD = new((aD2Plus.D2.XYZ() - aD2Minus.D2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X(), aD3.D3.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Y(), aD3.D3.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Z(), aD3.D3.Z(), THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalDN_ConsistentWithD1()
    {
        TBezierCurve aCurve = createSimpleCurve();
        double aU = M_PI / 6.0;

        Curve3D.ResD1 aD1 = aCurve.EvalD1(aU);
        Vector3D aDN = aCurve.EvalDN(aU, 1);

        Assert.Equal(aDN.X(), aD1.D1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aDN.Y(), aD1.D1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aDN.Z(), aD1.D1.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_MatchesCircle()
    {
        TBezierCurve aTBez = createSemicircle();
        // Geom_Circle (unit 1A) evaluates through ElCLib; the circle is used directly.
        Circle3D aCircle = new(new Frame3D(), 1.0);

        double[] aParams =
            [0.0, M_PI / 6.0, M_PI / 4.0, M_PI / 3.0, M_PI / 2.0, 2.0 * M_PI / 3.0, 3.0 * M_PI / 4.0, M_PI];
        foreach (double aU in aParams)
        {
            Point3D aPT = aTBez.EvalD0(aU);
            Point3D aPC = global::Kerf.Engine.Geometry.ElCLib.Value(aU, aCircle);
            Assert.Equal(0.0, aPT.Distance(aPC), global::Kerf.Engine.Foundation.Precision.Angular());
        }
    }

    [Fact]
    public void EvalD3_MatchesCircle()
    {
        TBezierCurve aTBez = createSemicircle();
        // Geom_Circle (unit 1A) evaluates through ElCLib; the circle is used directly.
        Circle3D aCircle = new(new Frame3D(), 1.0);

        double[] aParams = [M_PI / 6.0, M_PI / 3.0, M_PI / 2.0, 2.0 * M_PI / 3.0];
        foreach (double aU in aParams)
        {
            Curve3D.ResD3 aDT = aTBez.EvalD3(aU);
            Curve3D.ResD3 aDC;
            global::Kerf.Engine.Geometry.ElCLib.D3(aU, aCircle, out aDC.Point, out aDC.D1, out aDC.D2, out aDC.D3);

            Assert.Equal(0.0, aDT.Point.Distance(aDC.Point), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.X(), aDT.D1.X(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.Y(), aDT.D1.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.Z(), aDT.D1.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.X(), aDT.D2.X(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.Y(), aDT.D2.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.Z(), aDT.D2.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D3.X(), aDT.D3.X(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D3.Y(), aDT.D3.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D3.Z(), aDT.D3.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
        }
    }

    [Fact]
    public void EvalD2_MatchesEllipse()
    {
        TBezierCurve aTBez = createSemiEllipse();
        // Geom_Ellipse (unit 1A) evaluates through ElCLib; the ellipse is used directly.
        Ellipse3D anEllipse = new(new Frame3D(), 3.0, 2.0);

        double[] aParams = [M_PI / 6.0, M_PI / 4.0, M_PI / 2.0, 3.0 * M_PI / 4.0];
        foreach (double aU in aParams)
        {
            Curve3D.ResD2 aDT = aTBez.EvalD2(aU);
            Curve3D.ResD2 aDC;
            global::Kerf.Engine.Geometry.ElCLib.D2(aU, anEllipse, out aDC.Point, out aDC.D1, out aDC.D2);

            Assert.Equal(0.0, aDT.Point.Distance(aDC.Point), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.X(), aDT.D1.X(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.Y(), aDT.D1.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D1.Z(), aDT.D1.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.X(), aDT.D2.X(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.Y(), aDT.D2.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC.D2.Z(), aDT.D2.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
        }
    }

    [Fact]
    public void EvalD0_MultiplePoints()
    {
        TBezierCurve aCurve = createSimpleCurve();
        double aFirst = aCurve.FirstParameter();
        double aLast = aCurve.LastParameter();
        int aNbSamples = 10;
        double aStep = (aLast - aFirst) / aNbSamples;

        Point3D aPrev = aCurve.EvalD0(aFirst);
        for (int i = 1; i <= aNbSamples; ++i)
        {
            double aU = aFirst + aStep * i;
            Point3D aPnt = aCurve.EvalD0(aU);

            // Verify the point has finite coordinates
            Assert.True(double.IsFinite(aPnt.X()));
            Assert.True(double.IsFinite(aPnt.Y()));
            Assert.True(double.IsFinite(aPnt.Z()));

            // Verify consecutive points don't jump too far (smoothness check)
            double aDist = aPrev.Distance(aPnt);
            Assert.True(aDist < 10.0 * aStep);

            aPrev = aPnt;
        }
    }

    // OCCT.NET addition: derivatives up to D6 against finite differences, non-rational and rational, orders 1 to 3
    // (the rational EvalDN recurrence runs for N > 3).
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        for (int anOrder = 1; anOrder <= 3; ++anOrder)
        {
            int aNb = 2 * anOrder + 1;
            NCollection_Array1<Point3D> aPoles = new(1, aNb);
            NCollection_Array1<double> aWeights = new(1, aNb);
            for (int i = 1; i <= aNb; ++i)
            {
                aPoles.SetValue(i, new Point3D(Math.Cos(i * 1.3), 0.5 * i, Math.Sin(0.7 * i) - 0.2 * i));
                aWeights.SetValue(i, i == 1 ? 3.0 : 0.6 + 0.1 * i);
            }
            TBezierCurve aCurve = new(aPoles, 0.8);
            TBezierCurve aRational = new(aPoles, aWeights, 0.8);
            for (double aU = 0.1; aU < aCurve.LastParameter(); aU += 0.43)
            {
                EvalTestUtils.CheckCurveDerivatives(aCurve, aU, 6);
                EvalTestUtils.CheckCurveDerivatives(aRational, aU, 6);
            }
        }
    }
}
