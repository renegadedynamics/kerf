// Port of OCCT's src/ModelingData/TKGeomBase/GTests/GeomBndLib_Curve2d_Test.cxx (V8_0_1).
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Assertions shared by the GeomBndLib 2D curve tests (the GTests' anonymous-namespace helpers).</summary>
internal static class GeomBndLibCurve2dTestUtils
{
    internal static void ExpectGE(double theA, double theB, string theWhat = "")
        => Assert.True(theA >= theB, $"{theWhat}: expected {theA:R} >= {theB:R}");

    internal static void ExpectLE(double theA, double theB, string theWhat = "")
        => Assert.True(theA <= theB, $"{theWhat}: expected {theA:R} <= {theB:R}");

    /// <summary>Helper: compare two Bnd_Box2d results within tolerance.</summary>
    internal static void CompareBoxes2d(in Box2d theNew, in Box2d theOld, double theTol)
    {
        var (aXminN, aXmaxN, aYminN, aYmaxN) = theNew.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO) = theOld.Get();
        Assert.Equal(aXminO, aXminN, theTol);
        Assert.Equal(aXmaxO, aXmaxN, theTol);
        Assert.Equal(aYminO, aYminN, theTol);
        Assert.Equal(aYmaxO, aYmaxN, theTol);
    }

    /// <summary>Helper: verify that the new box conservatively encloses the old box.</summary>
    internal static void ExpectConservativeEnvelope2d(in Box2d theNew, in Box2d theOld, double theTol)
    {
        var (aXminN, aXmaxN, aYminN, aYmaxN) = theNew.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO) = theOld.Get();
        ExpectLE(aXminN, aXminO + theTol, "Xmin");
        ExpectGE(aXmaxN, aXmaxO - theTol, "Xmax");
        ExpectLE(aYminN, aYminO + theTol, "Ymin");
        ExpectGE(aYmaxN, aYmaxO - theTol, "Ymax");
    }

    /// <summary>Helper: verify new 2D box is no larger than old box (tighter or equal).</summary>
    internal static void ExpectNoLarger2d(in Box2d theNew, in Box2d theOld, double theTol)
    {
        var (aXminN, aXmaxN, aYminN, aYmaxN) = theNew.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO) = theOld.Get();
        ExpectGE(aXminN, aXminO - theTol, "New Xmin is smaller than old");
        ExpectLE(aXmaxN, aXmaxO + theTol, "New Xmax is larger than old");
        ExpectGE(aYminN, aYminO - theTol, "New Ymin is smaller than old");
        ExpectLE(aYmaxN, aYmaxO + theTol, "New Ymax is larger than old");
    }
}

public class GeomBndLib_Curve2dTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    private static void CompareBoxes2d(in Box2d theNew, in Box2d theOld, double theTol)
        => GeomBndLibCurve2dTestUtils.CompareBoxes2d(theNew, theOld, theTol);

    private static void ExpectGE(double theA, double theB) => GeomBndLibCurve2dTestUtils.ExpectGE(theA, theB);

    private static void ExpectLE(double theA, double theB) => GeomBndLibCurve2dTestUtils.ExpectLE(theA, theB);

    private static CircleCurve2D Circle(double theX, double theY, double theR)
        => new(new Axis2D(new Point2D(theX, theY), new Direction2D(1.0, 0.0)), theR);

    private static Frame2D XOY22d() => new(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0), new Direction2D(0.0, 1.0));

    // =========================================================================
    // Line2d
    // =========================================================================

    [Fact]
    public void Line_FiniteSegment()
    {
        LineCurve2D aLine = new(new Point2D(0.0, 0.0), new Direction2D(1.0, 1.0));
        Curve2dBounds aCurve = new(aLine);
        Assert.Equal(Line, aCurve.GetType());

        Box2d aBox = new();
        aCurve.Add(0.0, 10.0, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();

        // Line from (0,0) in direction (1,1) normalized, parameter 10
        double aLen = 10.0 / Math.Sqrt(2.0);
        Assert.Equal(0.0, aXmin, Conf);
        Assert.Equal(0.0, aYmin, Conf);
        Assert.Equal(aLen, aXmax, Conf);
        Assert.Equal(aLen, aYmax, Conf);
    }

    [Fact]
    public void Line_CompareWithBndLib()
    {
        LineCurve2D aLine = new(new Point2D(1.0, 2.0), new Direction2D(0.0, 1.0));
        CurveAdaptor2D anAdaptor = new(aLine);

        Box2d aNewBox = new();
        new Curve2dBounds(aLine).Add(0.0, 5.0, Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, 0.0, 5.0, Conf, ref anOldBox);

        CompareBoxes2d(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Circle2d
    // =========================================================================

    [Fact]
    public void Circle_Full()
    {
        double aRadius = 5.0;
        CircleCurve2D aCircle = Circle(0.0, 0.0, aRadius);
        Curve2dBounds aCurve = new(aCircle);
        Assert.Equal(global::Kerf.Geometry.CurveKind.Circle, aCurve.GetType());

        Box2d aBox = new();
        aCurve.Add(0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();

        Assert.Equal(-aRadius, aXmin, Conf);
        Assert.Equal(-aRadius, aYmin, Conf);
        Assert.Equal(aRadius, aXmax, Conf);
        Assert.Equal(aRadius, aYmax, Conf);
    }

    [Fact]
    public void Circle_Arc()
    {
        double aRadius = 10.0;
        CircleCurve2D aCircle = Circle(0.0, 0.0, aRadius);
        Curve2dBounds aCurve = new(aCircle);

        Box2d aBox = new();
        // Pass 0 tolerance for precise reference comparison
        aCurve.Add(0.0, M_PI / 2.0, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();

        Assert.Equal(0.0, aXmin, Conf);
        Assert.Equal(0.0, aYmin, Conf);
        Assert.Equal(aRadius, aXmax, Conf);
        Assert.Equal(aRadius, aYmax, Conf);
    }

    [Fact]
    public void Circle_CompareWithBndLib()
    {
        CircleCurve2D aCircle = Circle(0.0, 0.0, 7.5);
        CurveAdaptor2D anAdaptor = new(aCircle);

        Box2d aNewBox = new();
        new Curve2dBounds(aCircle).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes2d(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Ellipse2d
    // =========================================================================

    [Fact]
    public void Ellipse_Full()
    {
        double aMajR = 10.0;
        double aMinR = 5.0;
        EllipseCurve2D anEllipse = new(XOY22d(), aMajR, aMinR);
        Curve2dBounds aCurve = new(anEllipse);
        Assert.Equal(Ellipse, aCurve.GetType());

        Box2d aBox = new();
        aCurve.Add(0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();

        Assert.Equal(-aMajR, aXmin, Conf);
        Assert.Equal(aMajR, aXmax, Conf);
        Assert.Equal(-aMinR, aYmin, Conf);
        Assert.Equal(aMinR, aYmax, Conf);
    }

    [Fact]
    public void Ellipse_CompareWithBndLib()
    {
        EllipseCurve2D anEllipse = new(XOY22d(), 8.0, 3.0);
        CurveAdaptor2D anAdaptor = new(anEllipse);

        Box2d aNewBox = new();
        new Curve2dBounds(anEllipse).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes2d(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Hyperbola2d
    // =========================================================================

    [Fact]
    public void Hyperbola_FiniteArc()
    {
        double aMajR = 5.0;
        double aMinR = 3.0;
        HyperbolaCurve2D aHyp = new(XOY22d(), aMajR, aMinR);
        Curve2dBounds aCurve = new(aHyp);
        Assert.Equal(Hyperbola, aCurve.GetType());

        Box2d aBox = new();
        double aU = 1.0;
        aCurve.Add(-aU, aU, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();

        double aXEnd = aMajR * Math.Cosh(aU);
        double aYEnd = aMinR * Math.Sinh(aU);

        ExpectLE(aXmin, aMajR + Conf);
        ExpectGE(aXmax, aXEnd - Conf);
        ExpectLE(aYmin, -aYEnd + Conf);
        ExpectGE(aYmax, aYEnd - Conf);
    }

    [Fact]
    public void Hyperbola_CompareWithBndLib()
    {
        HyperbolaCurve2D aHyp = new(XOY22d(), 5.0, 3.0);
        CurveAdaptor2D anAdaptor = new(aHyp);

        Box2d aNewBox = new();
        new Curve2dBounds(aHyp).Add(-1.0, 1.0, Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, -1.0, 1.0, Conf, ref anOldBox);

        CompareBoxes2d(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Parabola2d
    // =========================================================================

    [Fact]
    public void Parabola_FiniteArc()
    {
        double aFocal = 2.0;
        ParabolaCurve2D aParab = new(XOY22d(), aFocal);
        Curve2dBounds aCurve = new(aParab);
        Assert.Equal(Parabola, aCurve.GetType());

        Box2d aBox = new();
        double aU = 5.0;
        aCurve.Add(-aU, aU, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();

        double aXEnd = aU * aU / (4.0 * aFocal);

        ExpectLE(aXmin, Conf);
        ExpectGE(aXmax, aXEnd - Conf);
        ExpectLE(aYmin, -aU + Conf);
        ExpectGE(aYmax, aU - Conf);
    }

    [Fact]
    public void Parabola_CompareWithBndLib()
    {
        ParabolaCurve2D aParab = new(XOY22d(), 2.0);
        CurveAdaptor2D anAdaptor = new(aParab);

        Box2d aNewBox = new();
        new Curve2dBounds(aParab).Add(-5.0, 5.0, Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, -5.0, 5.0, Conf, ref anOldBox);

        CompareBoxes2d(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // BezierCurve2d
    // =========================================================================

    [Fact]
    public void BezierCurve_Simple()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(5.0, 10.0));
        aPoles.SetValue(3, new Point2D(10.0, 0.0));

        BezierCurve2D aBez = new(aPoles);
        Curve2dBounds aCurve = new(aBez);
        Assert.Equal(BezierCurve, aCurve.GetType());

        Box2d aBox = new();
        aCurve.Add(Conf, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();

        ExpectLE(aXmin, Conf);
        ExpectGE(aXmax, 10.0 - Conf);
        ExpectLE(aYmin, Conf);
        ExpectGE(aYmax, 0.0);
    }

    [Fact]
    public void BezierCurve_CompareWithBndLib()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(3.0, 8.0));
        aPoles.SetValue(3, new Point2D(7.0, 2.0));
        aPoles.SetValue(4, new Point2D(10.0, 5.0));

        BezierCurve2D aBez = new(aPoles);
        CurveAdaptor2D anAdaptor = new(aBez);

        Box2d aNewBox = new();
        new Curve2dBounds(aBez).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        // New box should be no larger than old box (may be tighter due to convex hull approach)
        var (aXminN, aXmaxN, aYminN, aYmaxN) = aNewBox.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO) = anOldBox.Get();
        ExpectGE(aXminN, aXminO - Conf);
        ExpectLE(aXmaxN, aXmaxO + Conf);
        ExpectGE(aYminN, aYminO - Conf);
        ExpectLE(aYmaxN, aYmaxO + Conf);
    }

    // =========================================================================
    // BSplineCurve2d
    // =========================================================================

    private static BSplineCurve2D SimpleBSpline()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(2.0, 5.0));
        aPoles.SetValue(3, new Point2D(8.0, 5.0));
        aPoles.SetValue(4, new Point2D(10.0, 0.0));

        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 0.5);
        aKnots.SetValue(3, 1.0);

        NCollection_Array1<int> aMults = new(1, 3);
        aMults.SetValue(1, 3);
        aMults.SetValue(2, 1);
        aMults.SetValue(3, 3);

        return new BSplineCurve2D(aPoles, aKnots, aMults, 2);
    }

    [Fact]
    public void BSplineCurve_Simple()
    {
        BSplineCurve2D aBSpl = SimpleBSpline();
        Curve2dBounds aCurve = new(aBSpl);
        Assert.Equal(BSplineCurve, aCurve.GetType());

        Box2d aBox = new();
        aCurve.Add(Conf, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();

        ExpectLE(aXmin, Conf);
        ExpectGE(aXmax, 10.0 - Conf);
        ExpectLE(aYmin, Conf);
        ExpectGE(aYmax, 0.0);
    }

    [Fact]
    public void BSplineCurve_CompareWithBndLib()
    {
        BSplineCurve2D aBSpl = SimpleBSpline();
        CurveAdaptor2D anAdaptor = new(aBSpl);

        Box2d aNewBox = new();
        new Curve2dBounds(aBSpl).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes2d(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Adaptor constructor
    // =========================================================================

    [Fact]
    public void AdaptorConstructor_Circle()
    {
        double aRadius = 5.0;
        CircleCurve2D aCircle = Circle(0.0, 0.0, aRadius);

        // Construct from handle
        Curve2dBounds aCurveH = new(aCircle);
        Box2d aBoxH = new();
        aCurveH.Add(Conf, ref aBoxH);

        // Construct from adaptor
        CurveAdaptor2D anAdaptor = new(aCircle);
        Curve2dBounds aCurveA = new(anAdaptor);
        Assert.Equal(global::Kerf.Geometry.CurveKind.Circle, aCurveA.GetType());

        Box2d aBoxA = new();
        aCurveA.Add(Conf, ref aBoxA);

        CompareBoxes2d(aBoxH, aBoxA, Conf);
    }

    // =========================================================================
    // AddOptimal
    // =========================================================================

    [Fact]
    public void AddOptimal_Circle()
    {
        double aRadius = 5.0;
        CircleCurve2D aCircle = Circle(0.0, 0.0, aRadius);
        Curve2dBounds aCurve = new(aCircle);

        Box2d aBoxOpt = new();
        aCurve.AddOptimal(0.0, ref aBoxOpt);

        var (aXminO, aXmaxO, aYminO, aYmaxO) = aBoxOpt.Get();

        // Analytical circle should give precise box
        Assert.Equal(-aRadius, aXminO, Conf);
        Assert.Equal(-aRadius, aYminO, Conf);
        Assert.Equal(aRadius, aXmaxO, Conf);
        Assert.Equal(aRadius, aYmaxO, Conf);
    }

    [Fact]
    public void AdaptorConstructor_TrimmedRangeUsedForFullAdd()
    {
        CircleCurve2D aCircle = Circle(0.0, 0.0, 10.0);
        CurveAdaptor2D anAdaptor = new(aCircle, 0.0, M_PI / 2.0);

        Box2d aNewBox = new();
        new Curve2dBounds(anAdaptor).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes2d(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void TrimmedCurveHandle_Fallback_CompareWithBndLib()
    {
        CircleCurve2D aCircle = Circle(0.0, 0.0, 10.0);
        TrimmedCurve2D aTrim = new(aCircle, 0.0, M_PI / 2.0);
        CurveAdaptor2D anAdaptor = new(aTrim);

        Box2d aNewBox = new();
        new Curve2dBounds(aTrim).Add(Conf, ref aNewBox);

        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);

        Assert.False(aNewBox.IsVoid);
        GeomBndLibCurve2dTestUtils.ExpectConservativeEnvelope2d(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void TrimmedCurveHandle_UsesBasisSpecialization()
    {
        CircleCurve2D aCircle = Circle(0.0, 0.0, 10.0);
        TrimmedCurve2D aTrim = new(aCircle, 0.0, M_PI / 2.0);
        Curve2dBounds aCurve = new(aTrim);
        Assert.Equal(global::Kerf.Geometry.CurveKind.Circle, aCurve.GetType());

        CurveAdaptor2D anAdaptor = new(aTrim);
        Box2d aNewBox = new();
        aCurve.Add(Conf, ref aNewBox);
        Box2d anOldBox = new();
        Add2dCurve.Add(anAdaptor, Conf, ref anOldBox);
        CompareBoxes2d(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void SamplingHelpers2d_MatchesLegacyPolicy()
    {
        CircleCurve2D aCircle = Circle(0.0, 0.0, 10.0);
        CurveAdaptor2D aCircleGA = new(aCircle);
        Assert.Equal(17, SamplingHelpers.ComputeNbSamples2d(aCircleGA, 0.0, 1.0));

        const int aDegree = 3;
        const int aNbKnots = 40;
        const int aNbPoles = 42;
        NCollection_Array1<Point2D> aPoles = new(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; ++i)
        {
            double t = (double)(i - 1) / (double)(aNbPoles - 1);
            aPoles.SetValue(i, new Point2D(10.0 * t, t));
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

        BSplineCurve2D aBSpl = new(aPoles, aKnots, aMults, aDegree);
        CurveAdaptor2D aBSplGA = new(aBSpl);
        Assert.Equal(23, SamplingHelpers.ComputeNbSamples2d(aBSplGA, aBSpl.FirstParameter, aBSpl.LastParameter));
    }
}
