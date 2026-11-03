// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

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
        Geom2d_Line aLine = new(new Point2D(100, 0), new Direction2D(Direction2D.D.NX));

        NCollection_Array1<Point2D> aPoints = new(1, 3);
        aPoints.SetValue(1, new Point2D(0, 0));
        aPoints.SetValue(2, new Point2D(50, 50));
        aPoints.SetValue(3, new Point2D(0, 100));
        Geom2d_BezierCurve aCurve = new(aPoints);

        Geom2dAdaptor_Curve      aCLine        = new(aLine);
        Geom2dAdaptor_Curve      aCCurve       = new(aCurve);
        Geom2dGcc_QualifiedCurve aQualifCurve1 = new(aCLine, GccEnt_outside);
        Geom2dGcc_QualifiedCurve aQualifCurve2 = new(aCCurve, GccEnt_outside);

        const double            aRadius    = 10.0;
        const double            aTolerance = 1e-7;
        Geom2dGcc_Circ2d2TanRad aGccCirc2d = new(aQualifCurve1, aQualifCurve2, aRadius, aTolerance);

        Assert.True(aGccCirc2d.IsDone());
        Assert.True(aGccCirc2d.NbSolutions() > 0);

        const double aMaxDeltaPercent = 1.0; // 1% tolerance as used in the original Draw test
        for (int i = 1; i <= aGccCirc2d.NbSolutions(); i++)
        {
            Circle2D aCirc2d = aGccCirc2d.ThisSolution(i);
            Point2D  aCenter = aCirc2d.Location();
            double    aR      = aCirc2d.Radius();

            aGccCirc2d.Tangency1(i, out _, out _, out Point2D aPntSol1);
            aGccCirc2d.Tangency2(i, out _, out _, out Point2D aPntSol2);

            double aDelta1 = Math.Abs(aPntSol1.Distance(aCenter) - aR) / aR * 100.0;
            Assert.True(aDelta1 <= aMaxDeltaPercent, $"Solution {i}: tangency1 distance error {aDelta1}% exceeds 1%");

            double aDelta2 = Math.Abs(aPntSol2.Distance(aCenter) - aR) / aR * 100.0;
            Assert.True(aDelta2 <= aMaxDeltaPercent, $"Solution {i}: tangency2 distance error {aDelta2}% exceeds 1%");
        }
    }

    private static void AssertSameAsAna(Geom2dGcc_Circ2d2TanRad theCirc, GccAna_Circ2d2TanRad theAna)
    {
        Assert.True(theCirc.IsDone());
        Assert.Equal(theAna.NbSolutions(), theCirc.NbSolutions());
        for (int i = 1; i <= theCirc.NbSolutions(); i++)
        {
            Assert.Equal(theAna.ThisSolution(i), theCirc.ThisSolution(i));
            theCirc.Tangency1(i, out double p1, out double a1, out Point2D t1);
            theAna.Tangency1(i, out double q1, out double b1, out Point2D u1);
            Assert.Equal((q1, b1, u1), (p1, a1, t1));
            theCirc.Tangency2(i, out double p2, out double a2, out Point2D t2);
            theAna.Tangency2(i, out double q2, out double b2, out Point2D u2);
            Assert.Equal((q2, b2, u2), (p2, a2, t2));
            theCirc.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            theAna.WhichQualifier(i, out GccEnt_Position aR1, out GccEnt_Position aR2);
            Assert.Equal((aR1, aR2), (aQ1, aQ2));
        }
    }

    [Fact]
    public void CircleAndLine_GiveGccAnasSolutions()
    {
        Circle2D c1 = new(new Axis2D(new Point2D(1, 2), DX), 3);
        Line2D  l2 = new(new Point2D(0, -2), new Direction2D(1, 0.25));
        Geom2dGcc_Circ2d2TanRad aCirc = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Circle(c1))),
                                            Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Line(l2))), 1.5, 1e-9);
        GccAna_Circ2d2TanRad anAna = new(GccEnt.Unqualified(c1), GccEnt.Unqualified(l2), 1.5, 1e-9);
        Assert.True(aCirc.NbSolutions() > 0);
        AssertSameAsAna(aCirc, anAna);
    }

    [Fact]
    public void LineAndCircle_AreSwappedBack()
    {
        // The line comes first: GccAna is called with (circle, line), and the accessors swap the arguments back.
        Circle2D c2 = new(new Axis2D(new Point2D(1, 2), DX), 3);
        Line2D  l1 = new(new Point2D(0, -2), new Direction2D(1, 0.25));
        Geom2dGcc_Circ2d2TanRad aCirc = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Line(l1))),
                                            Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Circle(c2))), 1.5, 1e-9);
        GccAna_Circ2d2TanRad anAna = new(GccEnt.Unqualified(c2), GccEnt.Unqualified(l1), 1.5, 1e-9);
        Assert.True(aCirc.IsDone());
        Assert.Equal(anAna.NbSolutions(), aCirc.NbSolutions());
        Assert.True(aCirc.NbSolutions() > 0);
        for (int i = 1; i <= aCirc.NbSolutions(); i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aCirc.ThisSolution(i));
            aCirc.Tangency1(i, out double p1, out double a1, out Point2D t1);
            anAna.Tangency2(i, out double q2, out double b2, out Point2D u2);
            Assert.Equal((q2, b2, u2), (p1, a1, t1));
            aCirc.Tangency2(i, out double p2, out double a2, out Point2D t2);
            anAna.Tangency1(i, out double q1, out double b1, out Point2D u1);
            Assert.Equal((q1, b1, u1), (p2, a2, t2));
            aCirc.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            anAna.WhichQualifier(i, out GccEnt_Position aR1, out GccEnt_Position aR2);
            Assert.Equal((aR2, aR1), (aQ1, aQ2));
        }
    }

    [Fact]
    public void TwoPoints_GiveGccAnasSolutions()
    {
        Point2D p1 = new(0, 0), p2 = new(2, 1);
        Geom2dGcc_Circ2d2TanRad aCirc = new(new Geom2d_CartesianPoint(p1), new Geom2d_CartesianPoint(p2), 2, 1e-9);
        AssertSameAsAna(aCirc, new GccAna_Circ2d2TanRad(p1, p2, 2, 1e-9));
        Assert.Equal(2, aCirc.NbSolutions());
    }

    private static void AssertTangent(Adaptor2d_Curve2d theCurve, in Circle2D theCirc, double theParArg, in Point2D thePnt)
    {
        theCurve.D1(theParArg, out Point2D aP, out Vector2D aV);
        Assert.True(aP.Distance(thePnt) < 1e-6);
        Assert.Equal(theCirc.Radius(), thePnt.Distance(theCirc.Location()), 1e-6);
        // The radius at the tangency point is normal to the curve.
        Vector2D aRad = new(theCirc.Location(), thePnt);
        Assert.True(Math.Abs(aRad.Normalized().Dot(aV.Normalized())) < 1e-5);
    }

    [Fact]
    public void EllipseAndCircle_GiveTangentCircles()
    {
        Geom2dAdaptor_Curve anEll = new(new Geom2d_Ellipse(new Axis2D(new Point2D(0, 0), DX), 4, 2));
        Circle2D c1 = new(new Axis2D(new Point2D(7, 0), DX), 1);
        Geom2dGcc_Circ2d2TanRad aCirc = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Circle(c1))),
                                            Geom2dGcc.Outside(anEll), 1, 1e-7);
        Assert.True(aCirc.IsDone());
        Assert.True(aCirc.NbSolutions() > 0);
        for (int i = 1; i <= aCirc.NbSolutions(); i++)
        {
            Circle2D aSol = aCirc.ThisSolution(i);
            Assert.Equal(1.0, aSol.Radius());
            aCirc.Tangency2(i, out _, out double a2, out Point2D t2);
            AssertTangent(anEll, aSol, a2, t2);
            aCirc.Tangency1(i, out _, out _, out Point2D t1);
            Assert.Equal(1.0, t1.Distance(c1.Location()), 1e-6);
            aCirc.WhichQualifier(i, out _, out GccEnt_Position aQ2);
            Assert.Equal(GccEnt_outside, aQ2);
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
        Geom2dAdaptor_Curve aBS = new(new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3));
        Line2D l2 = new(new Point2D(0, 0), DX);
        Geom2dGcc_Circ2d2TanRad aCirc = new(Geom2dGcc.Unqualified(aBS),
                                            Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Line(l2))), 2, 1e-7);
        Assert.True(aCirc.IsDone());
        Assert.True(aCirc.NbSolutions() > 0);
        for (int i = 1; i <= aCirc.NbSolutions(); i++)
        {
            Circle2D aSol = aCirc.ThisSolution(i);
            aCirc.Tangency1(i, out _, out double a1, out Point2D t1);
            AssertTangent(aBS, aSol, a1, t1);
            aCirc.Tangency2(i, out _, out _, out Point2D t2);
            Assert.Equal(2.0, Math.Abs(aSol.Location().Y()), 1e-9);
            Assert.Equal(0.0, t2.Y(), 1e-9);
        }
    }

    [Fact]
    public void Accessors_ThrowAsOcct()
    {
        Assert.Throws<NegativeValueException>(() => new Geom2dGcc_Circ2d2TanRad(new Geom2d_CartesianPoint(0, 0),
                                                                               new Geom2d_CartesianPoint(1, 0), -1, 1e-9));
        Geom2dAdaptor_Curve aLine = new(new Geom2d_Line(new Point2D(0, 0), DX));
        Geom2dAdaptor_Curve anEll = new(new Geom2d_Ellipse(new Axis2D(new Point2D(0, 3), DX), 4, 2));
        Assert.Throws<GccEnt_BadQualifier>(() => new Geom2dGcc_Circ2d2TanRad(Geom2dGcc.Enclosing(aLine), Geom2dGcc.Unqualified(anEll), 1, 1e-7));

        // Points too far apart for the radius: done, no solution.
        Geom2dGcc_Circ2d2TanRad aFar = new(new Geom2d_CartesianPoint(0, 0), new Geom2d_CartesianPoint(10, 0), 1, 1e-9);
        Assert.True(aFar.IsDone());
        Assert.Equal(0, aFar.NbSolutions());
        Assert.Throws<OutOfRangeException>(() => aFar.ThisSolution(1));
        Assert.Throws<OutOfRangeException>(() => aFar.Tangency1(0, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => aFar.WhichQualifier(1, out _, out _));
        Assert.Throws<OutOfRangeException>(() => aFar.IsTheSame2(1));
    }
}
