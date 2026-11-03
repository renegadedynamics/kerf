// Port of OCCT's src/ModelingData/TKGeomBase/GTests/GeomBndLib_Curve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Kernel.ModelingData.Tests.GeomBndLibSurfaceTestUtils;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomBndLib_CurveTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    /// <summary>Helper: verify that the new box conservatively encloses the old box.</summary>
    private static void ExpectConservativeEnvelope(in Box theNew, in Box theOld, double theTol)
    {
        var (aXminN, aXmaxN, aYminN, aYmaxN, aZminN, aZmaxN) = theNew.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO, aZminO, aZmaxO) = theOld.Get();
        ExpectLE(aXminN, aXminO + theTol, "Xmin");
        ExpectGE(aXmaxN, aXmaxO - theTol, "Xmax");
        ExpectLE(aYminN, aYminO + theTol, "Ymin");
        ExpectGE(aYmaxN, aYmaxO - theTol, "Ymax");
        ExpectLE(aZminN, aZminO + theTol, "Zmin");
        ExpectGE(aZmaxN, aZmaxO - theTol, "Zmax");
    }

    // =========================================================================
    // Line
    // =========================================================================

    [Fact]
    public void Line_FiniteSegment()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 1.0, 0.0));
        CurveBounds aCurve = new(aLine);
        Assert.Equal(Line, aCurve.GetType());

        Box aBox = new();
        aCurve.Add(0.0, 10.0, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        // Line from (0,0,0) in direction (1,1,0) normalized, parameter 10
        // Endpoint: (10/sqrt(2), 10/sqrt(2), 0)
        double aLen = 10.0 / Math.Sqrt(2.0);
        Assert.Equal(0.0, aXmin, Conf);
        Assert.Equal(0.0, aYmin, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(aLen, aXmax, Conf);
        Assert.Equal(aLen, aYmax, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void Line_CompareWithBndLib()
    {
        LineCurve3D aLine = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0));
        CurveAdaptor anAdaptor = new(aLine);

        Box aNewBox = new();
        new CurveBounds(aLine).Add(0.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, 0.0, 5.0, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Circle
    // =========================================================================

    [Fact]
    public void Circle_Full()
    {
        const double aRadius = 5.0;
        CircleCurve3D aCircle = new(Constants.XOY(), aRadius);
        CurveBounds aCurve = new(aCircle);
        Assert.Equal(Circle, aCurve.GetType());

        Box aBox = new();
        aCurve.Add(0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        Assert.Equal(-aRadius, aXmin, Conf);
        Assert.Equal(-aRadius, aYmin, Conf);
        Assert.Equal(aRadius, aXmax, Conf);
        Assert.Equal(aRadius, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void Circle_Arc()
    {
        const double aRadius = 10.0;
        CircleCurve3D aCircle = new(Constants.XOY(), aRadius);
        CurveBounds aCurve = new(aCircle);

        Box aBox = new();
        // Pass 0 tolerance for precise reference comparison
        aCurve.Add(0.0, M_PI / 2.0, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        Assert.Equal(0.0, aXmin, Conf);
        Assert.Equal(0.0, aYmin, Conf);
        Assert.Equal(aRadius, aXmax, Conf);
        Assert.Equal(aRadius, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void Circle_CompareWithBndLib()
    {
        CircleCurve3D aCircle = new(Constants.XOY(), 7.5);
        CurveAdaptor anAdaptor = new(aCircle);

        Box aNewBox = new();
        new CurveBounds(aCircle).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void Circle_Arc_CompareWithBndLib()
    {
        CircleCurve3D aCircle = new(Constants.XOY(), 7.5);
        CurveAdaptor anAdaptor = new(aCircle);

        Box aNewBox = new();
        new CurveBounds(aCircle).Add(0.5, 2.5, Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, 0.5, 2.5, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Ellipse
    // =========================================================================

    [Fact]
    public void Ellipse_Full()
    {
        const double aMajR = 10.0;
        const double aMinR = 5.0;
        EllipseCurve3D anEllipse = new(Constants.XOY(), aMajR, aMinR);
        CurveBounds aCurve = new(anEllipse);
        Assert.Equal(Ellipse, aCurve.GetType());

        Box aBox = new();
        aCurve.Add(0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        Assert.Equal(-aMajR, aXmin, Conf);
        Assert.Equal(aMajR, aXmax, Conf);
        Assert.Equal(-aMinR, aYmin, Conf);
        Assert.Equal(aMinR, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void Ellipse_CompareWithBndLib()
    {
        EllipseCurve3D anEllipse = new(Constants.XOY(), 8.0, 3.0);
        CurveAdaptor anAdaptor = new(anEllipse);

        Box aNewBox = new();
        new CurveBounds(anEllipse).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void Ellipse_Arc_CompareWithBndLib()
    {
        EllipseCurve3D anEllipse = new(Constants.XOY(), 8.0, 3.0);
        CurveAdaptor anAdaptor = new(anEllipse);

        Box aNewBox = new();
        new CurveBounds(anEllipse).Add(0.3, 2.0, Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, 0.3, 2.0, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Hyperbola
    // =========================================================================

    [Fact]
    public void Hyperbola_FiniteArc()
    {
        const double aMajR = 5.0;
        const double aMinR = 3.0;
        HyperbolaCurve3D aHyp = new(Constants.XOY(), aMajR, aMinR);
        CurveBounds aCurve = new(aHyp);
        Assert.Equal(Hyperbola, aCurve.GetType());

        Box aBox = new();
        const double aU = 1.0;
        aCurve.Add(-aU, aU, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, _, _) = aBox.Get();

        // Hyperbola: x = aMajR * cosh(t), y = aMinR * sinh(t)
        double aXEnd = aMajR * Math.Cosh(aU);
        double aYEnd = aMinR * Math.Sinh(aU);

        // Box should contain the vertex (aMajR, 0) and endpoints
        ExpectLE(aXmin, aMajR + Conf, "Xmin");
        ExpectGE(aXmax, aXEnd - Conf, "Xmax");
        ExpectLE(aYmin, -aYEnd + Conf, "Ymin");
        ExpectGE(aYmax, aYEnd - Conf, "Ymax");
    }

    [Fact]
    public void Hyperbola_CompareWithBndLib()
    {
        HyperbolaCurve3D aHyp = new(Constants.XOY(), 5.0, 3.0);
        CurveAdaptor anAdaptor = new(aHyp);

        Box aNewBox = new();
        new CurveBounds(aHyp).Add(-1.0, 1.0, Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, -1.0, 1.0, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Parabola
    // =========================================================================

    [Fact]
    public void Parabola_FiniteArc()
    {
        const double aFocal = 2.0;
        ParabolaCurve3D aParab = new(Constants.XOY(), aFocal);
        CurveBounds aCurve = new(aParab);
        Assert.Equal(Parabola, aCurve.GetType());

        Box aBox = new();
        const double aU = 5.0;
        aCurve.Add(-aU, aU, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, _, _) = aBox.Get();

        // Parabola: x = t^2/(4*focal), y = t
        double aXEnd = aU * aU / (4.0 * aFocal);

        ExpectLE(aXmin, Conf, "Xmin");
        ExpectGE(aXmax, aXEnd - Conf, "Xmax");
        ExpectLE(aYmin, -aU + Conf, "Ymin");
        ExpectGE(aYmax, aU - Conf, "Ymax");
    }

    [Fact]
    public void Parabola_CompareWithBndLib()
    {
        ParabolaCurve3D aParab = new(Constants.XOY(), 2.0);
        CurveAdaptor anAdaptor = new(aParab);

        Box aNewBox = new();
        new CurveBounds(aParab).Add(-5.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, -5.0, 5.0, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // BezierCurve
    // =========================================================================

    [Fact]
    public void BezierCurve_Simple()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(5.0, 10.0, 0.0));
        aPoles.SetValue(3, new Point3D(10.0, 0.0, 0.0));

        BezierCurve3D aBez = new(aPoles);
        CurveBounds aCurve = new(aBez);
        Assert.Equal(BezierCurve, aCurve.GetType());

        Box aBox = new();
        aCurve.Add(Conf, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        // Bezier convex hull: x in [0,10], y in [0, <=10], z = 0
        ExpectLE(aXmin, Conf, "Xmin");
        ExpectGE(aXmax, 10.0 - Conf, "Xmax");
        ExpectLE(aYmin, Conf, "Ymin");
        ExpectGE(aYmax, 0.0, "Ymax");
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void BezierCurve_CompareWithBndLib()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(3.0, 8.0, 1.0));
        aPoles.SetValue(3, new Point3D(7.0, 2.0, -1.0));
        aPoles.SetValue(4, new Point3D(10.0, 5.0, 0.0));

        BezierCurve3D aBez = new(aPoles);
        CurveAdaptor anAdaptor = new(aBez);

        Box aNewBox = new();
        new CurveBounds(aBez).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // BSplineCurve
    // =========================================================================

    private static BSplineCurve3D QuadraticBSpline(double theZ2, double theZ3)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(2.0, 5.0, theZ2));
        aPoles.SetValue(3, new Point3D(8.0, 5.0, theZ3));
        aPoles.SetValue(4, new Point3D(10.0, 0.0, 0.0));

        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 0.5);
        aKnots.SetValue(3, 1.0);

        NCollection_Array1<int> aMults = new(1, 3);
        aMults.SetValue(1, 3);
        aMults.SetValue(2, 1);
        aMults.SetValue(3, 3);

        return new BSplineCurve3D(aPoles, aKnots, aMults, 2);
    }

    [Fact]
    public void BSplineCurve_Simple()
    {
        BSplineCurve3D aBSpl = QuadraticBSpline(0.0, 0.0);
        CurveBounds aCurve = new(aBSpl);
        Assert.Equal(BSplineCurve, aCurve.GetType());

        Box aBox = new();
        aCurve.Add(Conf, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        ExpectLE(aXmin, Conf, "Xmin");
        ExpectGE(aXmax, 10.0 - Conf, "Xmax");
        ExpectLE(aYmin, Conf, "Ymin");
        ExpectGE(aYmax, 0.0, "Ymax");
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void BSplineCurve_CompareWithBndLib()
    {
        BSplineCurve3D aBSpl = QuadraticBSpline(1.0, -1.0);
        CurveAdaptor anAdaptor = new(aBSpl);

        Box aNewBox = new();
        new CurveBounds(aBSpl).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void BSplineCurve_TrimmedRange_CompareWithBndLib()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 7);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(10.0, 20.0, 0.0));
        aPoles.SetValue(3, new Point3D(20.0, -10.0, 5.0));
        aPoles.SetValue(4, new Point3D(30.0, 40.0, 0.0));
        aPoles.SetValue(5, new Point3D(40.0, 5.0, -5.0));
        aPoles.SetValue(6, new Point3D(50.0, 15.0, 0.0));
        aPoles.SetValue(7, new Point3D(60.0, 0.0, 0.0));

        NCollection_Array1<double> aKnots = new(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aKnots.SetValue(i, i - 1.0);
        }

        NCollection_Array1<int> aMults = new(1, 5);
        aMults.SetValue(1, 4);
        aMults.SetValue(2, 1);
        aMults.SetValue(3, 1);
        aMults.SetValue(4, 1);
        aMults.SetValue(5, 4);

        BSplineCurve3D aBSpl = new(aPoles, aKnots, aMults, 3);
        CurveAdaptor anAdaptor = new(aBSpl);

        const double aU1 = 1.82;
        const double aU2 = 2.04;

        Box aNewBox = new();
        new CurveBounds(aBSpl).Add(aU1, aU2, Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, aU1, aU2, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Adaptor constructor
    // =========================================================================

    [Fact]
    public void AdaptorConstructor_Circle()
    {
        const double aRadius = 5.0;
        CircleCurve3D aCircle = new(Constants.XOY(), aRadius);

        // Construct from handle
        CurveBounds aCurveH = new(aCircle);
        Box aBoxH = new();
        aCurveH.Add(Conf, ref aBoxH);

        // Construct from adaptor
        CurveAdaptor anAdaptor = new(aCircle);
        CurveBounds aCurveA = new(anAdaptor);
        Assert.Equal(Circle, aCurveA.GetType());

        Box aBoxA = new();
        aCurveA.Add(Conf, ref aBoxA);

        CompareBoxes(aBoxH, aBoxA, Conf);
    }

    // =========================================================================
    // AddOptimal
    // =========================================================================

    [Fact]
    public void AddOptimal_Circle()
    {
        const double aRadius = 5.0;
        CircleCurve3D aCircle = new(Constants.XOY(), aRadius);
        CurveBounds aCurve = new(aCircle);

        Box aBox = new();
        aCurve.Add(Conf, ref aBox);

        Box aBoxOpt = new();
        aCurve.AddOptimal(Conf, ref aBoxOpt);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO, aZminO, aZmaxO) = aBoxOpt.Get();

        // AddOptimal should produce a box no larger than Add
        ExpectGE(aXminO, aXmin - Conf, "Xmin");
        ExpectGE(aYminO, aYmin - Conf, "Ymin");
        ExpectGE(aZminO, aZmin - Conf, "Zmin");
        ExpectLE(aXmaxO, aXmax + Conf, "Xmax");
        ExpectLE(aYmaxO, aYmax + Conf, "Ymax");
        ExpectLE(aZmaxO, aZmax + Conf, "Zmax");
    }

    [Fact]
    public void AddOptimal_CompareWithBndLib()
    {
        EllipseCurve3D anEllipse = new(Constants.XOY(), 8.0, 3.0);
        CurveAdaptor anAdaptor = new(anEllipse);

        Box aNewBox = new();
        new CurveBounds(anEllipse).AddOptimal(Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.AddOptimal(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void AdaptorConstructor_TrimmedRangeUsedForFullAdd()
    {
        CircleCurve3D aCircle = new(Constants.XOY(), 10.0);
        CurveAdaptor anAdaptor = new(aCircle, 0.0, M_PI / 2.0);

        Box aNewBox = new();
        new CurveBounds(anAdaptor).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void TrimmedCurveHandle_Fallback_CompareWithBndLib()
    {
        CircleCurve3D aCircle = new(Constants.XOY(), 10.0);
        TrimmedCurve3D aTrim = new(aCircle, 0.0, M_PI / 2.0);
        CurveAdaptor anAdaptor = new(aTrim);

        Box aNewBox = new();
        new CurveBounds(aTrim).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, Conf, ref anOldBox);

        Assert.False(aNewBox.IsVoid);
        ExpectConservativeEnvelope(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void TrimmedCurveHandle_UsesBasisSpecialization()
    {
        CircleCurve3D aCircle = new(Constants.XOY(), 10.0);
        TrimmedCurve3D aTrim = new(aCircle, 0.0, M_PI / 2.0);
        CurveBounds aCurve = new(aTrim);
        Assert.Equal(Circle, aCurve.GetType());

        CurveAdaptor anAdaptor = new(aTrim);
        Box aNewBox = new();
        aCurve.Add(Conf, ref aNewBox);
        Box anOldBox = new();
        Add3dCurve.Add(anAdaptor, Conf, ref anOldBox);
        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void SamplingHelpers3d_MatchesLegacyPolicy()
    {
        CircleCurve3D aCircle = new(Constants.XOY(), 10.0);
        CurveAdaptor aCircleGA = new(aCircle);
        Assert.Equal(33, SamplingHelpers.ComputeNbSamples(aCircleGA, 0.0, 1.0));

        const int aDegree = 3;
        const int aNbKnots = 40;
        const int aNbPoles = 42;
        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; ++i)
        {
            double t = (double)(i - 1) / (double)(aNbPoles - 1);
            aPoles.SetValue(i, new Point3D(10.0 * t, t, 0.0));
        }
        NCollection_Array1<double> aKnots = new(1, aNbKnots);
        for (int i = 1; i <= aNbKnots; ++i)
        {
            aKnots.SetValue(i, (double)(i - 1));
        }
        NCollection_Array1<int> aMults = new(1, aNbKnots);
        aMults.SetValue(1, aDegree + 1);
        for (int i = 2; i < aNbKnots; ++i)
        {
            aMults.SetValue(i, 1);
        }
        aMults.SetValue(aNbKnots, aDegree + 1);

        BSplineCurve3D aBSpl = new(aPoles, aKnots, aMults, aDegree);
        CurveAdaptor aBSplGA = new(aBSpl);
        Assert.Equal(312, SamplingHelpers.ComputeNbSamples(aBSplGA, aBSpl.FirstParameter, aBSpl.LastParameter));
    }
}
