// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Circ2d2TanRad: OCCT's GTest (src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dGcc_Circ2d2TanRad_Test.cxx),
/// the analytic dispatch against GccAna_Circ2d2TanRad (the swapped line/circle case included), the geometric solver on
/// an ellipse and a B-spline, and the accessors' exceptions.
/// </summary>
public class Geom2dGcc_Circ2d2TanRadTests
{
    private static readonly Direction2D DX = new(Direction2D.D.X);

    // Test BUC60897: Geom2dGcc_Circ2d2TanRad finds circles tangent to a line and
    // a Bezier curve. Verifies that each tangency point lies at the circle radius
    // from the circle center (within 1% relative tolerance).
    [Fact]
    public void BUC60897_TangentToLineAndBezier()
    {
        LineCurve2D aLine = new(new Point2D(100, 0), new Direction2D(Direction2D.D.NX));

        NCollection_Array1<Point2D> aPoints = new(1, 3);
        aPoints.SetValue(1, new Point2D(0, 0));
        aPoints.SetValue(2, new Point2D(50, 50));
        aPoints.SetValue(3, new Point2D(0, 100));
        BezierCurve2D aCurve = new(aPoints);

        CurveAdaptor2D      aCLine        = new(aLine);
        CurveAdaptor2D      aCCurve       = new(aCurve);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQualifCurve1 = new(aCLine, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQualifCurve2 = new(aCCurve, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside);

        const double            aRadius    = 10.0;
        const double            aTolerance = 1e-7;
        global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad aGccCirc2d = new(aQualifCurve1, aQualifCurve2, aRadius, aTolerance);

        Assert.True(aGccCirc2d.IsDone);
        Assert.True(aGccCirc2d.SolutionCount > 0);

        const double aMaxDeltaPercent = 1.0; // 1% tolerance as used in the original Draw test
        for (int i = 1; i <= aGccCirc2d.SolutionCount; i++)
        {
            Circle2D aCirc2d = aGccCirc2d.ThisSolution(i);
            Point2D  aCenter = aCirc2d.Location;
            double    aR      = aCirc2d.Radius;

            aGccCirc2d.Tangency1(i, out _, out _, out Point2D aPntSol1);
            aGccCirc2d.Tangency2(i, out _, out _, out Point2D aPntSol2);

            double aDelta1 = Math.Abs(aPntSol1.DistanceTo(aCenter) - aR) / aR * 100.0;
            Assert.True(aDelta1 <= aMaxDeltaPercent, $"Solution {i}: tangency1 distance error {aDelta1}% exceeds 1%");

            double aDelta2 = Math.Abs(aPntSol2.DistanceTo(aCenter) - aR) / aR * 100.0;
            Assert.True(aDelta2 <= aMaxDeltaPercent, $"Solution {i}: tangency2 distance error {aDelta2}% exceeds 1%");
        }
    }

    private static void AssertSameAsAna(global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad theCirc, AnalyticCirc2d2TanRad theAna)
    {
        Assert.True(theCirc.IsDone);
        Assert.Equal(theAna.SolutionCount, theCirc.SolutionCount);
        for (int i = 1; i <= theCirc.SolutionCount; i++)
        {
            Assert.Equal(theAna.ThisSolution(i), theCirc.ThisSolution(i));
            theCirc.Tangency1(i, out double p1, out double a1, out Point2D t1);
            theAna.Tangency1(i, out double q1, out double b1, out Point2D u1);
            Assert.Equal((q1, b1, u1), (p1, a1, t1));
            theCirc.Tangency2(i, out double p2, out double a2, out Point2D t2);
            theAna.Tangency2(i, out double q2, out double b2, out Point2D u2);
            Assert.Equal((q2, b2, u2), (p2, a2, t2));
            theCirc.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            theAna.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aR1, out global::Kerf.Engine.Construction.Constraints.Qualifier aR2);
            Assert.Equal((aR1, aR2), (aQ1, aQ2));
        }
    }

    [Fact]
    public void CircleAndLine_GiveGccAnasSolutions()
    {
        Circle2D c1 = new(new Axis2D(new Point2D(1, 2), DX), 3);
        Line2D  l2 = new(new Point2D(0, -2), new Direction2D(1, 0.25));
        global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad aCirc = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new CircleCurve2D(c1))),
                                            global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new LineCurve2D(l2))), 1.5, 1e-9);
        AnalyticCirc2d2TanRad anAna = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l2), 1.5, 1e-9);
        Assert.True(aCirc.SolutionCount > 0);
        AssertSameAsAna(aCirc, anAna);
    }

    [Fact]
    public void LineAndCircle_AreSwappedBack()
    {
        // The line comes first: GccAna is called with (circle, line), and the accessors swap the arguments back.
        Circle2D c2 = new(new Axis2D(new Point2D(1, 2), DX), 3);
        Line2D  l1 = new(new Point2D(0, -2), new Direction2D(1, 0.25));
        global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad aCirc = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new LineCurve2D(l1))),
                                            global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new CircleCurve2D(c2))), 1.5, 1e-9);
        AnalyticCirc2d2TanRad anAna = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c2), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(l1), 1.5, 1e-9);
        Assert.True(aCirc.IsDone);
        Assert.Equal(anAna.SolutionCount, aCirc.SolutionCount);
        Assert.True(aCirc.SolutionCount > 0);
        for (int i = 1; i <= aCirc.SolutionCount; i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aCirc.ThisSolution(i));
            aCirc.Tangency1(i, out double p1, out double a1, out Point2D t1);
            anAna.Tangency2(i, out double q2, out double b2, out Point2D u2);
            Assert.Equal((q2, b2, u2), (p1, a1, t1));
            aCirc.Tangency2(i, out double p2, out double a2, out Point2D t2);
            anAna.Tangency1(i, out double q1, out double b1, out Point2D u1);
            Assert.Equal((q1, b1, u1), (p2, a2, t2));
            aCirc.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            anAna.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aR1, out global::Kerf.Engine.Construction.Constraints.Qualifier aR2);
            Assert.Equal((aR2, aR1), (aQ1, aQ2));
        }
    }

    [Fact]
    public void TwoPoints_GiveGccAnasSolutions()
    {
        Point2D p1 = new(0, 0), p2 = new(2, 1);
        global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad aCirc = new(new CartesianPoint2(p1), new CartesianPoint2(p2), 2, 1e-9);
        AssertSameAsAna(aCirc, new AnalyticCirc2d2TanRad(p1, p2, 2, 1e-9));
        Assert.Equal(2, aCirc.SolutionCount);
    }

    private static void AssertTangent(Curve2DAdaptorBase theCurve, in Circle2D theCirc, double theParArg, in Point2D thePnt)
    {
        theCurve.D1(theParArg, out Point2D aP, out Vector2D aV);
        Assert.True(aP.DistanceTo(thePnt) < 1e-6);
        Assert.Equal(theCirc.Radius, thePnt.DistanceTo(theCirc.Location), 1e-6);
        // The radius at the tangency point is normal to the curve.
        Vector2D aRad = new(theCirc.Location, thePnt);
        Assert.True(Math.Abs(aRad.Normalized().Dot(aV.Normalized())) < 1e-5);
    }

    [Fact]
    public void EllipseAndCircle_GiveTangentCircles()
    {
        CurveAdaptor2D anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 4, 2));
        Circle2D c1 = new(new Axis2D(new Point2D(7, 0), DX), 1);
        global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad aCirc = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new CircleCurve2D(c1))),
                                            global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Outside(anEll), 1, 1e-7);
        Assert.True(aCirc.IsDone);
        Assert.True(aCirc.SolutionCount > 0);
        for (int i = 1; i <= aCirc.SolutionCount; i++)
        {
            Circle2D aSol = aCirc.ThisSolution(i);
            Assert.Equal(1.0, aSol.Radius);
            aCirc.Tangency2(i, out _, out double a2, out Point2D t2);
            AssertTangent(anEll, aSol, a2, t2);
            aCirc.Tangency1(i, out _, out _, out Point2D t1);
            Assert.Equal(1.0, t1.DistanceTo(c1.Location), 1e-6);
            aCirc.WhichQualifier(i, out _, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ2);
        }
    }

    [Fact]
    public void BSplineAndLine_SwappedThroughGeo_GiveTangentCircles()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(-5, 3);
        aPoles[2] = new Point2D(-2, 6);
        aPoles[3] = new Point2D(2, 6);
        aPoles[4] = new Point2D(5, 3);
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        CurveAdaptor2D aBS = new(new BSplineCurve2D(aPoles, aKnots, aMults, 3));
        Line2D l2 = new(new Point2D(0, 0), DX);
        global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad aCirc = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(aBS),
                                            global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new LineCurve2D(l2))), 2, 1e-7);
        Assert.True(aCirc.IsDone);
        Assert.True(aCirc.SolutionCount > 0);
        for (int i = 1; i <= aCirc.SolutionCount; i++)
        {
            Circle2D aSol = aCirc.ThisSolution(i);
            aCirc.Tangency1(i, out _, out double a1, out Point2D t1);
            AssertTangent(aBS, aSol, a1, t1);
            aCirc.Tangency2(i, out _, out _, out Point2D t2);
            Assert.Equal(2.0, Math.Abs(aSol.Location.Y), 1e-9);
            Assert.Equal(0.0, t2.Y, 1e-9);
        }
    }

    [Fact]
    public void Accessors_ThrowAsOcct()
    {
        EngineAssert.Throws<NegativeValueException>(() => new global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad(new CartesianPoint2(0, 0),
                                                                               new CartesianPoint2(1, 0), -1, 1e-9));
        CurveAdaptor2D aLine = new(new LineCurve2D(new Point2D(0, 0), DX));
        CurveAdaptor2D anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 3), DX), 4, 2));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(() => new global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Enclosing(aLine), global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), 1, 1e-7));

        // Points too far apart for the radius: done, no solution.
        global::Kerf.Engine.Construction.Constraints.Circ2d2TanRad aFar = new(new CartesianPoint2(0, 0), new CartesianPoint2(10, 0), 1, 1e-9);
        Assert.True(aFar.IsDone);
        Assert.Equal(0, aFar.SolutionCount);
        EngineAssert.Throws<OutOfRangeException>(() => aFar.ThisSolution(1));
        EngineAssert.Throws<OutOfRangeException>(() => aFar.Tangency1(0, out _, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => aFar.WhichQualifier(1, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => aFar.IsTheSame2(1));
    }
}
