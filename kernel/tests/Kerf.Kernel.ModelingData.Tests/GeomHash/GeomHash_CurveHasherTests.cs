// Port of: src/ModelingData/TKG3d/GTests/GeomHash_CurveHasher_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Hashing;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomHash_CurveHasherTest
{
    private readonly CurveHasher myHasher = new();

    // ============================================================================
    // Line Tests
    // ============================================================================

    [Fact]
    public void Line_CopiedLines_SameHash()
    {
        Point3D aLoc = new(1.0, 2.0, 3.0);
        Direction3D aDir = new(1.0, 0.0, 0.0);
        LineCurve3D aLine1 = new LineCurve3D(aLoc, aDir);
        LineCurve3D aLine2 = (LineCurve3D)aLine1.Copy();

        Assert.Equal(myHasher.GetHashCode(aLine1), myHasher.GetHashCode(aLine2));
        Assert.True(myHasher.Equals(aLine1, aLine2));
    }

    [Fact]
    public void Line_DifferentLines_DifferentHash()
    {
        LineCurve3D aLine1 = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        LineCurve3D aLine2 = new LineCurve3D(new Point3D(1.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));

        Assert.NotEqual(myHasher.GetHashCode(aLine1), myHasher.GetHashCode(aLine2));
        Assert.False(myHasher.Equals(aLine1, aLine2));
    }

    // ============================================================================
    // Circle Tests
    // ============================================================================

    [Fact]
    public void Circle_CopiedCircles_SameHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CircleCurve3D aCircle1 = new CircleCurve3D(anAxis, 5.0);
        CircleCurve3D aCircle2 = (CircleCurve3D)aCircle1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.True(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Circle_DifferentRadius_DifferentHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CircleCurve3D aCircle1 = new CircleCurve3D(anAxis, 5.0);
        CircleCurve3D aCircle2 = new CircleCurve3D(anAxis, 10.0);

        Assert.NotEqual(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.False(myHasher.Equals(aCircle1, aCircle2));
    }

    // ============================================================================
    // Ellipse Tests
    // ============================================================================

    [Fact]
    public void Ellipse_CopiedEllipses_SameHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        EllipseCurve3D anEllipse1 = new EllipseCurve3D(anAxis, 10.0, 5.0);
        EllipseCurve3D anEllipse2 = (EllipseCurve3D)anEllipse1.Copy();

        Assert.Equal(myHasher.GetHashCode(anEllipse1), myHasher.GetHashCode(anEllipse2));
        Assert.True(myHasher.Equals(anEllipse1, anEllipse2));
    }

    [Fact]
    public void Ellipse_DifferentRadii_DifferentHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        EllipseCurve3D anEllipse1 = new EllipseCurve3D(anAxis, 10.0, 5.0);
        EllipseCurve3D anEllipse2 = new EllipseCurve3D(anAxis, 10.0, 7.0);

        Assert.NotEqual(myHasher.GetHashCode(anEllipse1), myHasher.GetHashCode(anEllipse2));
        Assert.False(myHasher.Equals(anEllipse1, anEllipse2));
    }

    // ============================================================================
    // Hyperbola Tests
    // ============================================================================

    [Fact]
    public void Hyperbola_CopiedHyperbolas_SameHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        HyperbolaCurve3D aHyp1 = new HyperbolaCurve3D(anAxis, 5.0, 3.0);
        HyperbolaCurve3D aHyp2 = (HyperbolaCurve3D)aHyp1.Copy();

        Assert.Equal(myHasher.GetHashCode(aHyp1), myHasher.GetHashCode(aHyp2));
        Assert.True(myHasher.Equals(aHyp1, aHyp2));
    }

    [Fact]
    public void Hyperbola_DifferentRadii_DifferentHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        HyperbolaCurve3D aHyp1 = new HyperbolaCurve3D(anAxis, 5.0, 3.0);
        HyperbolaCurve3D aHyp2 = new HyperbolaCurve3D(anAxis, 5.0, 4.0);

        Assert.NotEqual(myHasher.GetHashCode(aHyp1), myHasher.GetHashCode(aHyp2));
        Assert.False(myHasher.Equals(aHyp1, aHyp2));
    }

    // ============================================================================
    // Parabola Tests
    // ============================================================================

    [Fact]
    public void Parabola_CopiedParabolas_SameHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        ParabolaCurve3D aPar1 = new ParabolaCurve3D(anAxis, 2.0);
        ParabolaCurve3D aPar2 = (ParabolaCurve3D)aPar1.Copy();

        Assert.Equal(myHasher.GetHashCode(aPar1), myHasher.GetHashCode(aPar2));
        Assert.True(myHasher.Equals(aPar1, aPar2));
    }

    [Fact]
    public void Parabola_DifferentFocal_DifferentHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        ParabolaCurve3D aPar1 = new ParabolaCurve3D(anAxis, 2.0);
        ParabolaCurve3D aPar2 = new ParabolaCurve3D(anAxis, 3.0);

        Assert.NotEqual(myHasher.GetHashCode(aPar1), myHasher.GetHashCode(aPar2));
        Assert.False(myHasher.Equals(aPar1, aPar2));
    }

    // ============================================================================
    // BezierCurve Tests
    // ============================================================================

    [Fact]
    public void BezierCurve_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 2.0, 0.0);
        aPoles[3] = new Point3D(3.0, 2.0, 0.0);
        aPoles[4] = new Point3D(4.0, 0.0, 0.0);

        BezierCurve3D aCurve1 = new BezierCurve3D(aPoles);
        BezierCurve3D aCurve2 = (BezierCurve3D)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    [Fact]
    public void BezierCurve_DifferentPoles_DifferentComparison()
    {
        var aPoles1 = new NCollection_Array1<Point3D>(1, 3);
        aPoles1[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles1[2] = new Point3D(1.0, 2.0, 0.0);
        aPoles1[3] = new Point3D(2.0, 0.0, 0.0);

        var aPoles2 = new NCollection_Array1<Point3D>(1, 3);
        aPoles2[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles2[2] = new Point3D(1.0, 3.0, 0.0); // Different
        aPoles2[3] = new Point3D(2.0, 0.0, 0.0);

        BezierCurve3D aCurve1 = new BezierCurve3D(aPoles1);
        BezierCurve3D aCurve2 = new BezierCurve3D(aPoles2);

        // Hash may be same (metadata only), but comparison should differ
        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // BSplineCurve Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 1.0, 0.0);
        aPoles[4] = new Point3D(3.0, 0.0, 0.0);

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        BSplineCurve3D aCurve1 = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        BSplineCurve3D aCurve2 = (BSplineCurve3D)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // TrimmedCurve Tests
    // ============================================================================

    [Fact]
    public void TrimmedCurve_CopiedCurves_SameHash()
    {
        LineCurve3D aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        TrimmedCurve3D aTrimmed1 = new TrimmedCurve3D(aLine, 0.0, 10.0);
        TrimmedCurve3D aTrimmed2 = (TrimmedCurve3D)aTrimmed1.Copy();

        Assert.Equal(myHasher.GetHashCode(aTrimmed1), myHasher.GetHashCode(aTrimmed2));
        Assert.True(myHasher.Equals(aTrimmed1, aTrimmed2));
    }

    [Fact]
    public void TrimmedCurve_DifferentBounds_DifferentHash()
    {
        LineCurve3D aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        TrimmedCurve3D aTrimmed1 = new TrimmedCurve3D(aLine, 0.0, 10.0);
        TrimmedCurve3D aTrimmed2 = new TrimmedCurve3D(aLine, 0.0, 20.0);

        Assert.NotEqual(myHasher.GetHashCode(aTrimmed1), myHasher.GetHashCode(aTrimmed2));
        Assert.False(myHasher.Equals(aTrimmed1, aTrimmed2));
    }

    // ============================================================================
    // OffsetCurve Tests
    // ============================================================================

    [Fact]
    public void OffsetCurve_CopiedCurves_SameHash()
    {
        LineCurve3D        aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Direction3D aRefDir = new(0.0, 0.0, 1.0);
        OffsetCurve3D anOffset1 = new OffsetCurve3D(aLine, 5.0, aRefDir);
        OffsetCurve3D anOffset2 = (OffsetCurve3D)anOffset1.Copy();

        Assert.Equal(myHasher.GetHashCode(anOffset1), myHasher.GetHashCode(anOffset2));
        Assert.True(myHasher.Equals(anOffset1, anOffset2));
    }

    [Fact]
    public void OffsetCurve_DifferentOffset_DifferentHash()
    {
        LineCurve3D        aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Direction3D aRefDir = new(0.0, 0.0, 1.0);
        OffsetCurve3D anOffset1 = new OffsetCurve3D(aLine, 5.0, aRefDir);
        OffsetCurve3D anOffset2 = new OffsetCurve3D(aLine, 10.0, aRefDir);

        Assert.NotEqual(myHasher.GetHashCode(anOffset1), myHasher.GetHashCode(anOffset2));
        Assert.False(myHasher.Equals(anOffset1, anOffset2));
    }

    // ============================================================================
    // Cross-type Tests
    // ============================================================================

    [Fact]
    public void DifferentTypes_DifferentComparison()
    {
        LineCurve3D   aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CircleCurve3D aCircle = new CircleCurve3D(anAxis, 5.0);

        Assert.False(myHasher.Equals(aLine, aCircle));
    }

    [Fact]
    public void NullCurves_HandledCorrectly()
    {
        Curve3D? aNullCurve = null;
        LineCurve3D  aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));

        Assert.Equal(0, myHasher.GetHashCode(aNullCurve));
        Assert.True(myHasher.Equals(aNullCurve, aNullCurve));
        Assert.False(myHasher.Equals(aLine, aNullCurve));
    }

    [Fact]
    public void SameObject_Equal()
    {
        LineCurve3D aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Assert.True(myHasher.Equals(aLine, aLine));
    }

    // ============================================================================
    // Weighted BSpline Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_Weighted_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 1.0, 0.0);
        aPoles[4] = new Point3D(3.0, 0.0, 0.0);

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

        BSplineCurve3D aCurve1 =
            new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 3);
        BSplineCurve3D aCurve2 = (BSplineCurve3D)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    [Fact]
    public void BSplineCurve_DifferentWeights_DifferentComparison()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 1.0, 0.0);
        aPoles[4] = new Point3D(3.0, 0.0, 0.0);

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

        BSplineCurve3D aCurve1 =
            new BSplineCurve3D(aPoles, aWeights1, aKnots, aMults, 3);
        BSplineCurve3D aCurve2 =
            new BSplineCurve3D(aPoles, aWeights2, aKnots, aMults, 3);

        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Weighted Bezier Tests
    // ============================================================================

    [Fact]
    public void BezierCurve_Weighted_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 2.0, 0.0);
        aPoles[3] = new Point3D(2.0, 0.0, 0.0);

        var aWeights = new NCollection_Array1<double>(1, 3);
        aWeights[1] = 1.0;
        aWeights[2] = 2.0;
        aWeights[3] = 1.0;

        BezierCurve3D aCurve1 = new BezierCurve3D(aPoles, aWeights);
        BezierCurve3D aCurve2 = (BezierCurve3D)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Axis Orientation Tests
    // ============================================================================

    [Fact]
    public void Circle_DifferentAxisOrientation_DifferentHash()
    {
        Frame3D anAxis1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Frame3D anAxis2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));
        CircleCurve3D aCircle1 = new CircleCurve3D(anAxis1, 5.0);
        CircleCurve3D aCircle2 = new CircleCurve3D(anAxis2, 5.0);

        Assert.NotEqual(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.False(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Line_DifferentDirection_DifferentHash()
    {
        LineCurve3D aLine1 = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        LineCurve3D aLine2 = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));

        Assert.NotEqual(myHasher.GetHashCode(aLine1), myHasher.GetHashCode(aLine2));
        Assert.False(myHasher.Equals(aLine1, aLine2));
    }

    // ============================================================================
    // BSpline with Different Knots Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_DifferentKnots_DifferentComparison()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 1.0, 0.0);
        aPoles[4] = new Point3D(3.0, 0.0, 0.0);

        var aKnots1 = new NCollection_Array1<double>(1, 2);
        aKnots1[1] = 0.0;
        aKnots1[2] = 1.0;

        var aKnots2 = new NCollection_Array1<double>(1, 2);
        aKnots2[1] = 0.0;
        aKnots2[2] = 2.0; // Different

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        BSplineCurve3D aCurve1 = new BSplineCurve3D(aPoles, aKnots1, aMults, 3);
        BSplineCurve3D aCurve2 = new BSplineCurve3D(aPoles, aKnots2, aMults, 3);

        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Periodic BSpline Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_QuadraticBezierEquivalent_CopiedCurves_SameHash()
    {
        // Simple quadratic B-spline (Bezier-like, single span)
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 2.0, 0.0);
        aPoles[3] = new Point3D(2.0, 0.0, 0.0);

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;

        BSplineCurve3D aCurve1 = new BSplineCurve3D(aPoles, aKnots, aMults, 2);
        BSplineCurve3D aCurve2 = (BSplineCurve3D)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Reversed Curve Tests
    // ============================================================================

    [Fact]
    public void Line_Reversed_DifferentComparison()
    {
        LineCurve3D aLine1 = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        LineCurve3D aLine2 = (LineCurve3D)aLine1.Reversed();

        // Reversed line has opposite direction
        Assert.False(myHasher.Equals(aLine1, aLine2));
    }

    [Fact]
    public void BezierCurve_Reversed_DifferentComparison()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 2.0, 0.0);
        aPoles[3] = new Point3D(2.0, 0.0, 0.0);

        BezierCurve3D aCurve1 = new BezierCurve3D(aPoles);
        BezierCurve3D aCurve2 = (BezierCurve3D)aCurve1.Reversed();

        // Reversed curve has poles in different order
        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Transformed Curve Tests
    // ============================================================================

    [Fact]
    public void Circle_Translated_DifferentHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CircleCurve3D aCircle1 = new CircleCurve3D(anAxis, 5.0);
        CircleCurve3D aCircle2 = (CircleCurve3D)aCircle1.Copy();
        aCircle2.Translate(new Vector3D(1.0, 0.0, 0.0));

        Assert.NotEqual(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.False(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Circle_Scaled_DifferentHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CircleCurve3D aCircle1 = new CircleCurve3D(anAxis, 5.0);
        CircleCurve3D aCircle2 = (CircleCurve3D)aCircle1.Copy();
        aCircle2.Scale(new Point3D(0.0, 0.0, 0.0), 2.0);

        Assert.NotEqual(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.False(myHasher.Equals(aCircle1, aCircle2));
    }

    // ============================================================================
    // Higher Degree BSpline Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_HigherDegree_CopiedCurves_SameHash()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 6);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 0.5, 0.0);
        aPoles[4] = new Point3D(3.0, 1.5, 0.0);
        aPoles[5] = new Point3D(4.0, 0.0, 0.0);
        aPoles[6] = new Point3D(5.0, 1.0, 0.0);

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 6;
        aMults[2] = 6;

        // Degree 5 curve
        BSplineCurve3D aCurve1 = new BSplineCurve3D(aPoles, aKnots, aMults, 5);
        BSplineCurve3D aCurve2 = (BSplineCurve3D)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    [Fact]
    public void BSplineCurve_DifferentDegree_DifferentComparison()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 1.0, 0.0);
        aPoles[4] = new Point3D(3.0, 0.0, 0.0);

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        var aMults1 = new NCollection_Array1<int>(1, 2);
        aMults1[1] = 4;
        aMults1[2] = 4;

        var aMults2 = new NCollection_Array1<int>(1, 2);
        aMults2[1] = 3;
        aMults2[2] = 3;

        // Different degrees: 3 vs 2
        BSplineCurve3D aCurve1 = new BSplineCurve3D(aPoles, aKnots, aMults1, 3);

        var aPoles2 = new NCollection_Array1<Point3D>(1, 3);
        aPoles2[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles2[2] = new Point3D(1.5, 1.0, 0.0);
        aPoles2[3] = new Point3D(3.0, 0.0, 0.0);

        BSplineCurve3D aCurve2 = new BSplineCurve3D(aPoles2, aKnots, aMults2, 2);

        Assert.False(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Multiple Knot Spans Tests
    // ============================================================================

    [Fact]
    public void BSplineCurve_LinearMultipleSpans_CopiedCurves_SameHash()
    {
        // Linear B-spline with multiple knots (piecewise linear)
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles[3] = new Point3D(2.0, 0.0, 0.0);
        aPoles[4] = new Point3D(3.0, 1.0, 0.0);

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

        BSplineCurve3D aCurve1 = new BSplineCurve3D(aPoles, aKnots, aMults, 1);
        BSplineCurve3D aCurve2 = (BSplineCurve3D)aCurve1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCurve1), myHasher.GetHashCode(aCurve2));
        Assert.True(myHasher.Equals(aCurve1, aCurve2));
    }

    // ============================================================================
    // Cross-type Conic Tests
    // ============================================================================

    [Fact]
    public void Ellipse_vs_Hyperbola_DifferentComparison()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        EllipseCurve3D   anEllipse  = new EllipseCurve3D(anAxis, 5.0, 3.0);
        HyperbolaCurve3D aHyperbola = new HyperbolaCurve3D(anAxis, 5.0, 3.0);

        Assert.False(myHasher.Equals(anEllipse, aHyperbola));
    }

    [Fact]
    public void Circle_vs_Ellipse_DifferentComparison()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CircleCurve3D  aCircle   = new CircleCurve3D(anAxis, 5.0);
        EllipseCurve3D anEllipse = new EllipseCurve3D(anAxis, 5.0, 5.0);

        Assert.False(myHasher.Equals(aCircle, anEllipse));
    }

    // ============================================================================
    // Trimmed vs Base Curve Tests
    // ============================================================================

    [Fact]
    public void TrimmedCurve_vs_BaseCurve_DifferentComparison()
    {
        LineCurve3D aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        TrimmedCurve3D aTrimmed = new TrimmedCurve3D(aLine, 0.0, 10.0);

        // Trimmed curve is a different type from base curve
        Assert.False(myHasher.Equals(aLine, aTrimmed));
    }

    // ============================================================================
    // Edge Cases - Degenerate/Special Values
    // ============================================================================

    [Fact]
    public void Circle_VerySmallRadius_CopiedCircles_SameHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CircleCurve3D aCircle1 = new CircleCurve3D(anAxis, 1e-10);
        CircleCurve3D aCircle2 = (CircleCurve3D)aCircle1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.True(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Circle_VeryLargeRadius_CopiedCircles_SameHash()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CircleCurve3D aCircle1 = new CircleCurve3D(anAxis, 1e10);
        CircleCurve3D aCircle2 = (CircleCurve3D)aCircle1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCircle1), myHasher.GetHashCode(aCircle2));
        Assert.True(myHasher.Equals(aCircle1, aCircle2));
    }

    [Fact]
    public void Line_AtOrigin_vs_FarFromOrigin_DifferentHash()
    {
        LineCurve3D aLine1 = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        LineCurve3D aLine2 = new LineCurve3D(new Point3D(1e10, 1e10, 1e10), new Direction3D(1.0, 0.0, 0.0));

        Assert.NotEqual(myHasher.GetHashCode(aLine1), myHasher.GetHashCode(aLine2));
        Assert.False(myHasher.Equals(aLine1, aLine2));
    }
}
