// Port of: src/ModelingData/TKG2d/GTests/Geom2dHash_CurveHasher_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2dHash_CurveHasherTest
{
    private readonly Geom2dHash_CurveHasher myHasher = new();

    // ============================================================================
    // Line Tests
    // ============================================================================

    [Fact]
    public void Line_CopiedLines_SameHash()
    {
        gp_Pnt2d aLoc = new(1.0, 2.0);
        gp_Dir2d aDir = new(1.0, 0.0);
        Geom2d_Line aLine1 = new Geom2d_Line(aLoc, aDir);
        Geom2d_Line aLine2 = (Geom2d_Line)aLine1.Copy();

        Assert.Equal(myHasher.GetHashCode(aLine1), myHasher.GetHashCode(aLine2));
        Assert.True(myHasher.Equals(aLine1, aLine2));
    }

    [Fact]
    public void Line_DifferentLines_DifferentHash()
    {
        Geom2d_Line aLine1 = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_Line aLine2 = new Geom2d_Line(new gp_Pnt2d(1.0, 0.0), new gp_Dir2d(1.0, 0.0));

        Assert.NotEqual(myHasher.GetHashCode(aLine1), myHasher.GetHashCode(aLine2));
        Assert.False(myHasher.Equals(aLine1, aLine2));
    }

    // ============================================================================
    // Circle Tests
    // ============================================================================

    [Fact]
    public void Circle_CopiedCircles_SameHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle aCircle1 = new Geom2d_Circle(anAxis, 5.0);
        Geom2d_Circle aCircle2 = (Geom2d_Circle)aCircle1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.True(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Circle_DifferentRadius_DifferentHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle aCircle1 = new Geom2d_Circle(anAxis, 5.0);
        Geom2d_Circle aCircle2 = new Geom2d_Circle(anAxis, 10.0);

        Assert.NotEqual(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.False(myHasher.Equals(aCircle1, aCircle2));
    }

    // ============================================================================
    // Ellipse Tests
    // ============================================================================

    [Fact]
    public void Ellipse_CopiedEllipses_SameHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Ellipse anEllipse1 = new Geom2d_Ellipse(anAxis, 10.0, 5.0);
        Geom2d_Ellipse anEllipse2 = (Geom2d_Ellipse)anEllipse1.Copy();

        Assert.Equal(myHasher.GetHashCode(anEllipse1), myHasher.GetHashCode(anEllipse2));
        Assert.True(myHasher.Equals(anEllipse1, anEllipse2));
    }

    [Fact]
    public void Ellipse_DifferentRadii_DifferentHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Ellipse anEllipse1 = new Geom2d_Ellipse(anAxis, 10.0, 5.0);
        Geom2d_Ellipse anEllipse2 = new Geom2d_Ellipse(anAxis, 10.0, 7.0);

        Assert.NotEqual(myHasher.GetHashCode(anEllipse1), myHasher.GetHashCode(anEllipse2));
        Assert.False(myHasher.Equals(anEllipse1, anEllipse2));
    }

    // ============================================================================
    // Hyperbola Tests
    // ============================================================================

    [Fact]
    public void Hyperbola_CopiedHyperbolas_SameHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Hyperbola aHyp1 = new Geom2d_Hyperbola(anAxis, 5.0, 3.0);
        Geom2d_Hyperbola aHyp2 = (Geom2d_Hyperbola)aHyp1.Copy();

        Assert.Equal(myHasher.GetHashCode(aHyp1), myHasher.GetHashCode(aHyp2));
        Assert.True(myHasher.Equals(aHyp1, aHyp2));
    }

    [Fact]
    public void Hyperbola_DifferentRadii_DifferentHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Hyperbola aHyp1 = new Geom2d_Hyperbola(anAxis, 5.0, 3.0);
        Geom2d_Hyperbola aHyp2 = new Geom2d_Hyperbola(anAxis, 5.0, 4.0);

        Assert.NotEqual(myHasher.GetHashCode(aHyp1), myHasher.GetHashCode(aHyp2));
        Assert.False(myHasher.Equals(aHyp1, aHyp2));
    }

    // ============================================================================
    // Parabola Tests
    // ============================================================================

    [Fact]
    public void Parabola_CopiedParabolas_SameHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Parabola aPar1 = new Geom2d_Parabola(anAxis, 2.0);
        Geom2d_Parabola aPar2 = (Geom2d_Parabola)aPar1.Copy();

        Assert.Equal(myHasher.GetHashCode(aPar1), myHasher.GetHashCode(aPar2));
        Assert.True(myHasher.Equals(aPar1, aPar2));
    }

    [Fact]
    public void Parabola_DifferentFocal_DifferentHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Parabola aPar1 = new Geom2d_Parabola(anAxis, 2.0);
        Geom2d_Parabola aPar2 = new Geom2d_Parabola(anAxis, 3.0);

        Assert.NotEqual(myHasher.GetHashCode(aPar1), myHasher.GetHashCode(aPar2));
        Assert.False(myHasher.Equals(aPar1, aPar2));
    }

    // ============================================================================
    // BezierCurve Tests
    // ============================================================================

    [Fact]
    public void BezierCurve_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 2.0);
        aPoles[3] = new gp_Pnt2d(3.0, 2.0);
        aPoles[4] = new gp_Pnt2d(4.0, 0.0);

        Geom2d_BezierCurve aCurve1 = new Geom2d_BezierCurve(aPoles);
        Geom2d_BezierCurve aCurve2 = (Geom2d_BezierCurve)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    [Fact]
    public void BezierCurve_DifferentPoles_DifferentComparison()
    {
        var aPoles1 = new NCollection_Array1<gp_Pnt2d>(1, 3);
        aPoles1[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles1[2] = new gp_Pnt2d(1.0, 2.0);
        aPoles1[3] = new gp_Pnt2d(2.0, 0.0);

        var aPoles2 = new NCollection_Array1<gp_Pnt2d>(1, 3);
        aPoles2[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles2[2] = new gp_Pnt2d(1.0, 3.0); // Different
        aPoles2[3] = new gp_Pnt2d(2.0, 0.0);

        Geom2d_BezierCurve aCurve1 = new Geom2d_BezierCurve(aPoles1);
        Geom2d_BezierCurve aCurve2 = new Geom2d_BezierCurve(aPoles2);

        // Hash may be same (metadata only), but comparison should differ
        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // BSplineCurve Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 1.0);
        aPoles[3] = new gp_Pnt2d(2.0, 1.0);
        aPoles[4] = new gp_Pnt2d(3.0, 0.0);

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        Geom2d_BSplineCurve aCurve1 = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3);
        Geom2d_BSplineCurve aCurve2 = (Geom2d_BSplineCurve)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // TrimmedCurve Tests
    // ============================================================================

    [Fact]
    public void TrimmedCurve_CopiedCurves_SameHash()
    {
        Geom2d_Line         aLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_TrimmedCurve aTrimmed1 = new Geom2d_TrimmedCurve(aLine, 0.0, 10.0);
        Geom2d_TrimmedCurve aTrimmed2 =
            (Geom2d_TrimmedCurve)aTrimmed1.Copy();

        Assert.Equal(myHasher.GetHashCode(aTrimmed1), myHasher.GetHashCode(aTrimmed2));
        Assert.True(myHasher.Equals(aTrimmed1, aTrimmed2));
    }

    [Fact]
    public void TrimmedCurve_DifferentBounds_DifferentHash()
    {
        Geom2d_Line         aLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_TrimmedCurve aTrimmed1 = new Geom2d_TrimmedCurve(aLine, 0.0, 10.0);
        Geom2d_TrimmedCurve aTrimmed2 = new Geom2d_TrimmedCurve(aLine, 0.0, 20.0);

        Assert.NotEqual(myHasher.GetHashCode(aTrimmed1), myHasher.GetHashCode(aTrimmed2));
        Assert.False(myHasher.Equals(aTrimmed1, aTrimmed2));
    }

    // ============================================================================
    // OffsetCurve Tests
    // ============================================================================

    [Fact]
    public void OffsetCurve_CopiedCurves_SameHash()
    {
        Geom2d_Line        aLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_OffsetCurve anOffset1 = new Geom2d_OffsetCurve(aLine, 5.0);
        Geom2d_OffsetCurve anOffset2 = (Geom2d_OffsetCurve)anOffset1.Copy();

        Assert.Equal(myHasher.GetHashCode(anOffset1), myHasher.GetHashCode(anOffset2));
        Assert.True(myHasher.Equals(anOffset1, anOffset2));
    }

    [Fact]
    public void OffsetCurve_DifferentOffset_DifferentHash()
    {
        Geom2d_Line        aLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_OffsetCurve anOffset1 = new Geom2d_OffsetCurve(aLine, 5.0);
        Geom2d_OffsetCurve anOffset2 = new Geom2d_OffsetCurve(aLine, 10.0);

        Assert.NotEqual(myHasher.GetHashCode(anOffset1), myHasher.GetHashCode(anOffset2));
        Assert.False(myHasher.Equals(anOffset1, anOffset2));
    }

    // ============================================================================
    // Cross-type Tests
    // ============================================================================

    [Fact]
    public void DifferentTypes_DifferentComparison()
    {
        Geom2d_Line   aLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle aCircle = new Geom2d_Circle(anAxis, 5.0);

        Assert.False(myHasher.Equals(aLine, aCircle));
    }

    [Fact]
    public void NullCurves_HandledCorrectly()
    {
        Geom2d_Curve? aNullCurve = null;
        Geom2d_Line  aLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));

        Assert.Equal(0, myHasher.GetHashCode(aNullCurve));
        Assert.True(myHasher.Equals(aNullCurve, aNullCurve));
        Assert.False(myHasher.Equals(aLine, aNullCurve));
    }

    [Fact]
    public void SameObject_Equal()
    {
        Geom2d_Line aLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Assert.True(myHasher.Equals(aLine, aLine));
    }

    // ============================================================================
    // Weighted BSpline Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_Weighted_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 1.0);
        aPoles[3] = new gp_Pnt2d(2.0, 1.0);
        aPoles[4] = new gp_Pnt2d(3.0, 0.0);

        var aWeights = new NCollection_Array1<double>(1, 4);
        aWeights[1] = 1.0;
        aWeights[2] = 2.0;
        aWeights[3] = 2.0;
        aWeights[4] = 1.0;

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        Geom2d_BSplineCurve aCurve1 =
            new Geom2d_BSplineCurve(aPoles, aWeights, aKnots, aMults, 3);
        Geom2d_BSplineCurve aCurve2 = (Geom2d_BSplineCurve)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    [Fact]
    public void BSplineCurve_DifferentWeights_DifferentComparison()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 1.0);
        aPoles[3] = new gp_Pnt2d(2.0, 1.0);
        aPoles[4] = new gp_Pnt2d(3.0, 0.0);

        var aWeights1 = new NCollection_Array1<double>(1, 4);
        aWeights1[1] = 1.0;
        aWeights1[2] = 2.0;
        aWeights1[3] = 2.0;
        aWeights1[4] = 1.0;

        var aWeights2 = new NCollection_Array1<double>(1, 4);
        aWeights2[1] = 1.0;
        aWeights2[2] = 3.0; // Different
        aWeights2[3] = 2.0;
        aWeights2[4] = 1.0;

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        Geom2d_BSplineCurve aCurve1 =
            new Geom2d_BSplineCurve(aPoles, aWeights1, aKnots, aMults, 3);
        Geom2d_BSplineCurve aCurve2 =
            new Geom2d_BSplineCurve(aPoles, aWeights2, aKnots, aMults, 3);

        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Weighted Bezier Tests
    // ============================================================================

    [Fact]
    public void BezierCurve_Weighted_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 3);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 2.0);
        aPoles[3] = new gp_Pnt2d(2.0, 0.0);

        var aWeights = new NCollection_Array1<double>(1, 3);
        aWeights[1] = 1.0;
        aWeights[2] = 2.0;
        aWeights[3] = 1.0;

        Geom2d_BezierCurve aCurve1 = new Geom2d_BezierCurve(aPoles, aWeights);
        Geom2d_BezierCurve aCurve2 = (Geom2d_BezierCurve)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Axis Orientation Tests
    // ============================================================================

    [Fact]
    public void Circle_DifferentAxisOrientation_DifferentHash()
    {
        gp_Ax22d anAxis1 = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        gp_Ax22d anAxis2 = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(0.0, 1.0), new gp_Dir2d(-1.0, 0.0));
        Geom2d_Circle aCircle1 = new Geom2d_Circle(anAxis1, 5.0);
        Geom2d_Circle aCircle2 = new Geom2d_Circle(anAxis2, 5.0);

        Assert.NotEqual(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.False(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Line_DifferentDirection_DifferentHash()
    {
        Geom2d_Line aLine1 = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_Line aLine2 = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(0.0, 1.0));

        Assert.NotEqual(myHasher.GetHashCode(aLine1), myHasher.GetHashCode(aLine2));
        Assert.False(myHasher.Equals(aLine1, aLine2));
    }

    // ============================================================================
    // BSpline with Different Knots Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_DifferentKnots_DifferentComparison()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 1.0);
        aPoles[3] = new gp_Pnt2d(2.0, 1.0);
        aPoles[4] = new gp_Pnt2d(3.0, 0.0);

        var aKnots1 = new NCollection_Array1<double>(1, 2);
        aKnots1[1] = 0.0;
        aKnots1[2] = 1.0;

        var aKnots2 = new NCollection_Array1<double>(1, 2);
        aKnots2[1] = 0.0;
        aKnots2[2] = 2.0; // Different

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        Geom2d_BSplineCurve aCurve1 = new Geom2d_BSplineCurve(aPoles, aKnots1, aMults, 3);
        Geom2d_BSplineCurve aCurve2 = new Geom2d_BSplineCurve(aPoles, aKnots2, aMults, 3);

        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Periodic BSpline Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_QuadraticBezierEquivalent_CopiedCurves_SameHash()
    {
        // Simple quadratic B-spline (Bezier-like, single span)
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 3);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 2.0);
        aPoles[3] = new gp_Pnt2d(2.0, 0.0);

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;

        Geom2d_BSplineCurve aCurve1 = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 2);
        Geom2d_BSplineCurve aCurve2 = (Geom2d_BSplineCurve)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Reversed Curve Tests
    // ============================================================================

    [Fact]
    public void Line_Reversed_DifferentComparison()
    {
        Geom2d_Line aLine1 = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_Line aLine2 = (Geom2d_Line)aLine1.Reversed();

        Assert.False(myHasher.Equals(aLine1, aLine2));
    }

    [Fact]
    public void BezierCurve_Reversed_DifferentComparison()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 3);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 2.0);
        aPoles[3] = new gp_Pnt2d(2.0, 0.0);

        Geom2d_BezierCurve aCurve1 = new Geom2d_BezierCurve(aPoles);
        Geom2d_BezierCurve aCurve2 = (Geom2d_BezierCurve)aCurve1.Reversed();

        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Transformed Curve Tests
    // ============================================================================

    [Fact]
    public void Circle_Translated_DifferentHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle aCircle1 = new Geom2d_Circle(anAxis, 5.0);
        Geom2d_Circle aCircle2 = (Geom2d_Circle)aCircle1.Copy();
        aCircle2.Translate(new gp_Vec2d(1.0, 0.0));

        Assert.NotEqual(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.False(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Circle_Scaled_DifferentHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle aCircle1 = new Geom2d_Circle(anAxis, 5.0);
        Geom2d_Circle aCircle2 = (Geom2d_Circle)aCircle1.Copy();
        aCircle2.Scale(new gp_Pnt2d(0.0, 0.0), 2.0);

        Assert.NotEqual(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.False(myHasher.Equals(aCircle1, aCircle2));
    }

    // ============================================================================
    // Higher Degree BSpline Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_HigherDegree_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 6);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 1.0);
        aPoles[3] = new gp_Pnt2d(2.0, 0.5);
        aPoles[4] = new gp_Pnt2d(3.0, 1.5);
        aPoles[5] = new gp_Pnt2d(4.0, 0.0);
        aPoles[6] = new gp_Pnt2d(5.0, 1.0);

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 6;
        aMults[2] = 6;

        Geom2d_BSplineCurve aCurve1 = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 5);
        Geom2d_BSplineCurve aCurve2 = (Geom2d_BSplineCurve)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Multiple Knot Spans Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_LinearMultipleSpans_CopiedCurves_SameHash()
    {
        // Linear B-spline with multiple knots (piecewise linear)
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 1.0);
        aPoles[3] = new gp_Pnt2d(2.0, 0.0);
        aPoles[4] = new gp_Pnt2d(3.0, 1.0);

        var aKnots = new NCollection_Array1<double>(1, 4);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        aKnots[3] = 2.0;
        aKnots[4] = 3.0;

        var aMults = new NCollection_Array1<int>(1, 4);
        aMults[1] = 2;
        aMults[2] = 1;
        aMults[3] = 1;
        aMults[4] = 2;

        Geom2d_BSplineCurve aCurve1 = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 1);
        Geom2d_BSplineCurve aCurve2 = (Geom2d_BSplineCurve)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Cross-type Conic Tests
    // ============================================================================

    [Fact]
    public void Ellipse_vs_Hyperbola_DifferentComparison()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Ellipse   anEllipse  = new Geom2d_Ellipse(anAxis, 5.0, 3.0);
        Geom2d_Hyperbola aHyperbola = new Geom2d_Hyperbola(anAxis, 5.0, 3.0);

        Assert.False(myHasher.Equals(anEllipse, aHyperbola));
    }

    [Fact]
    public void Circle_vs_Ellipse_DifferentComparison()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle  aCircle   = new Geom2d_Circle(anAxis, 5.0);
        Geom2d_Ellipse anEllipse = new Geom2d_Ellipse(anAxis, 5.0, 5.0);

        Assert.False(myHasher.Equals(aCircle, anEllipse));
    }

    // ============================================================================
    // Trimmed vs Base Curve Tests
    // ============================================================================

    [Fact]
    public void TrimmedCurve_vs_BaseCurve_DifferentComparison()
    {
        Geom2d_Line         aLine = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_TrimmedCurve aTrimmed = new Geom2d_TrimmedCurve(aLine, 0.0, 10.0);

        Assert.False(myHasher.Equals(aLine, aTrimmed));
    }

    // ============================================================================
    // Edge Cases - Degenerate/Special Values
    // ============================================================================

    [Fact]
    public void Circle_VerySmallRadius_CopiedCircles_SameHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle aCircle1 = new Geom2d_Circle(anAxis, 1e-10);
        Geom2d_Circle aCircle2 = (Geom2d_Circle)aCircle1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.True(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Circle_VeryLargeRadius_CopiedCircles_SameHash()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Circle aCircle1 = new Geom2d_Circle(anAxis, 1e10);
        Geom2d_Circle aCircle2 = (Geom2d_Circle)aCircle1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.True(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Line_AtOrigin_vs_FarFromOrigin_DifferentHash()
    {
        Geom2d_Line aLine1 = new Geom2d_Line(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_Line aLine2 = new Geom2d_Line(new gp_Pnt2d(1e10, 1e10), new gp_Dir2d(1.0, 0.0));

        Assert.NotEqual(myHasher.GetHashCode(aLine1), myHasher.GetHashCode(aLine2));
        Assert.False(myHasher.Equals(aLine1, aLine2));
    }
}
