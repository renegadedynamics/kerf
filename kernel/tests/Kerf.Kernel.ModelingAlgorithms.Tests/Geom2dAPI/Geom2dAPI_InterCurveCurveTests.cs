// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dAPI_InterCurveCurve_Test.cxx
// Port of: src/ModelingData/TKG2d/GTests/Geom2dAPI_InterCurveCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using Xunit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of Geom2dAPI_InterCurveCurve (TKGeomAlgo's and TKG2d's), and known answers.</summary>
public class Geom2dAPI_InterCurveCurveTests
{
    [Fact]
    public void OCC29289_EllipseIntersectionNewtonRoot()
    {
        // Create two ellipses
        Ellipse2D e1 = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(Direction2D.D.X)), 2.0, 1.0);
        EllipseCurve2D Ge1 = new(e1);
        Ellipse2D e2 = new(new Axis2D(new Point2D(0.5, 0.5), new Direction2D(1.0, 1.0)), 2.0, 1.0);
        EllipseCurve2D Ge2 = new(e2);

        // Find intersection points
        CurveCurveIntersection2D Intersector = new();
        Intersector.Init(Ge1, Ge2, 1.0e-7);
        Assert.True(Intersector.NbPoints() > 0, "Error: intersector found no points");

        // Setup trigonometric equation: A*std::cos(x) + B*Sin(x) + C*std::cos(2*x) + D*Sin(2*x) + E
        double A = 1.875, B = -0.75, C = -0.5, D = -0.25, E = -0.25;
        TrigonometricEquationFunction MyF = new(A, B, C, D, E);

        double Tol1 = 1.0e-15;
        double Eps = 1.5e-12;
        int[] Nit = [5, 6, 7, 6];

        // For each intersection point, verify Newton root finding
        double TetaPrev = 0.0;
        for (int i = 1; i <= Intersector.NbPoints(); i++)
        {
            double Teta = Intersector.Intersector().Point(i).ParamOnFirst();
            double X = Teta - 0.1 * (Teta - TetaPrev);
            TetaPrev = Teta;

            NewtonFunctionRoot Resol = new(MyF, X, Tol1, Eps, Nit[i - 1]);
            Assert.True(Resol.IsDone, $"Error: Newton is not done for {Teta}");

            double TetaNewton = Resol.Root();
            Assert.True(Math.Abs(Teta - TetaNewton) <= 1.0e-7, $"Error: Newton root is wrong for {Teta}");
        }
    }

    [Fact]
    public void PointRejectsZeroIndex()
    {
        Ellipse2D anEllipse1 = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 2.0, 1.0);
        EllipseCurve2D aCurve1 = new(anEllipse1);
        Ellipse2D anEllipse2 = new(new Axis2D(new Point2D(0.5, 0.5), new Direction2D(1.0, 1.0)), 2.0, 1.0);
        EllipseCurve2D aCurve2 = new(anEllipse2);

        CurveCurveIntersection2D anIntersector = new(aCurve1, aCurve2, 1.0e-7);
        Assert.True(anIntersector.NbPoints() > 0);

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.OutOfRangeException>(() => anIntersector.Point(0));
    }

    // Regression test for bug #11: Init with null handle must raise Standard_NullObject.
    [Fact]
    public void Init_NullHandle_RaisesException()
    {
        Curve2D aNullCurve = null!;
        Ellipse2D anEllipse = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 2.0, 1.0);
        Curve2D aValidCurve = new EllipseCurve2D(anEllipse);

        // Both null.
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NullObjectException>(() => new CurveCurveIntersection2D(aNullCurve, aNullCurve, 1.0e-7));

        // First null.
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NullObjectException>(() => new CurveCurveIntersection2D(aNullCurve, aValidCurve, 1.0e-7));

        // Second null.
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NullObjectException>(() => new CurveCurveIntersection2D(aValidCurve, aNullCurve, 1.0e-7));

        // Single-curve Init with null.
        CurveCurveIntersection2D anInter = new();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NullObjectException>(() => anInter.Init(aNullCurve, 1.0e-7));
    }

    [Fact]
    public void OCC24889_IntersectionParameterWithinLimits()
    {
        // Bug OCC24889: Geom2dAPI_InterCurveCurve produces result with parameter outside the curve limits

        // Create two circles
        CircleCurve2D aCircle1 = new(new Frame2D(new Point2D(25, -25), new Direction2D(Direction2D.D.X),
                                                  new Direction2D(-0.0, 1)), 155);
        CircleCurve2D aCircle2 = new(new Frame2D(new Point2D(25, 25), new Direction2D(Direction2D.D.X),
                                                  new Direction2D(-0.0, 1)), 155);

        // Create trimmed curves
        TrimmedCurve2D aTrim1 = new(aCircle1, 1.57079632679490, 2.97959469729228);
        TrimmedCurve2D aTrim2 = new(aCircle2, 3.30359060633978, 4.71238898038469);

        // Perform intersection
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        CurveCurveIntersection2D aIntTool = new(aTrim1, aTrim2, aTol);

        Assert.True(aIntTool.NbPoints() > 0, "Intersection should find at least one point");

        IntersectionPoint2D aIntPnt = aIntTool.Intersector().Point(1);

        double aParOnC1 = aIntPnt.ParamOnFirst();
        double aParOnC2 = aIntPnt.ParamOnSecond();

        // Verify that intersection parameters are within the curve limits
        Assert.True(aParOnC1 >= aTrim1.FirstParameter);
        Assert.True(aParOnC1 <= aTrim1.LastParameter);
        Assert.True(aParOnC2 >= aTrim2.FirstParameter);
        Assert.True(aParOnC2 <= aTrim2.LastParameter);

        // Verify that both curves evaluate to the same point at the intersection parameters
        Point2D aP1 = aTrim1.Value(aParOnC1);
        Point2D aP2 = aTrim2.Value(aParOnC2);
        Assert.True(aP1.SquareDistanceTo(aP2) < 1.0e-14, "Points on both curves at intersection parameters should coincide");
    }

    // ---------------------------------------------------------------------------------------------
    // Known answers

    private static BSplineCurve2D Polyline(params (double X, double Y)[] thePnts)
    {
        NCollection_Array1<Point2D> aPoles = new(1, thePnts.Length);
        for (int i = 0; i < thePnts.Length; i++)
        {
            aPoles[i + 1] = new Point2D(thePnts[i].X, thePnts[i].Y);
        }
        NCollection_Array1<double> aKnots = new(1, thePnts.Length);
        NCollection_Array1<int> aMults = new(1, thePnts.Length);
        for (int i = 1; i <= thePnts.Length; i++)
        {
            aKnots[i] = i - 1;
            aMults[i] = 1;
        }
        aMults[1] = 2;
        aMults[thePnts.Length] = 2;
        return new BSplineCurve2D(aPoles, aKnots, aMults, 1);
    }

    private static void AssertOnBoth(CurveCurveIntersection2D theInter, Curve2D theC1, Curve2D theC2,
                                     double theTol)
    {
        for (int i = 1; i <= theInter.NbPoints(); i++)
        {
            IntersectionPoint2D aP = theInter.Intersector().Point(i);
            Assert.True(theC1.Value(aP.ParamOnFirst()).DistanceTo(aP.Value()) < theTol);
            Assert.True(theC2.Value(aP.ParamOnSecond()).DistanceTo(aP.Value()) < theTol);
        }
    }

    [Fact]
    public void LineCircle_TwoPoints()
    {
        LineCurve2D aLine = new(new Point2D(-5, 0.5), new Direction2D(1, 0));
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        CurveCurveIntersection2D anInter = new(aLine, aCircle, 1.0e-7);
        Assert.Equal(2, anInter.NbPoints());
        Assert.Equal(0, anInter.NbSegments());
        double aX = Math.Sqrt(1 - 0.25);
        double[] aXs = [anInter.Point(1).X, anInter.Point(2).X];
        Array.Sort(aXs);
        Assert.Equal(-aX, aXs[0], 1e-12);
        Assert.Equal(aX, aXs[1], 1e-12);
        Assert.Equal(0.5, anInter.Point(1).Y, 1e-12);
        AssertOnBoth(anInter, aLine, aCircle, 1e-9);
    }

    [Fact]
    public void LineCircle_Tangent()
    {
        LineCurve2D aLine = new(new Point2D(-5, 2), new Direction2D(1, 0));
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        CurveCurveIntersection2D anInter = new(aLine, aCircle, 1.0e-7);
        Assert.True(anInter.NbPoints() + anInter.NbSegments() >= 1);
        if (anInter.NbPoints() > 0)
        {
            Assert.Equal(0.0, anInter.Point(1).X, 1e-3);
            Assert.Equal(2.0, anInter.Point(1).Y, 1e-6);
        }
    }

    [Fact]
    public void CircleCircle_Secant()
    {
        CircleCurve2D aC1 = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        CircleCurve2D aC2 = new(new Axis2D(new Point2D(1, 0), new Direction2D(1, 0)), 1.0);
        CurveCurveIntersection2D anInter = new(aC1, aC2, 1.0e-7);
        Assert.Equal(2, anInter.NbPoints());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(0.5, anInter.Point(i).X, 1e-12);
            Assert.Equal(Math.Sqrt(0.75), Math.Abs(anInter.Point(i).Y), 1e-12);
        }
        AssertOnBoth(anInter, aC1, aC2, 1e-9);
    }

    [Fact]
    public void CircleCircle_Identical_GivesSegment()
    {
        CircleCurve2D aC1 = new(new Axis2D(new Point2D(1, 2), new Direction2D(1, 0)), 3.0);
        CircleCurve2D aC2 = new(new Axis2D(new Point2D(1, 2), new Direction2D(1, 0)), 3.0);
        CurveCurveIntersection2D anInter = new(aC1, aC2, 1.0e-7);
        Assert.Equal(0, anInter.NbPoints());
        Assert.True(anInter.NbSegments() >= 1);
        anInter.Segment(1, out Curve2D aS1, out Curve2D aS2);
        Assert.IsType<TrimmedCurve2D>(aS1);
        Assert.True(aS1.Value(aS1.FirstParameter).DistanceTo(aS2.Value(aS2.FirstParameter)) < 1e-7
                    || aS1.Value(aS1.FirstParameter).DistanceTo(aS2.Value(aS2.LastParameter)) < 1e-7);
    }

    [Fact]
    public void EllipseLine_OnTrimmedCurves()
    {
        EllipseCurve2D anE = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 2.0);
        TrimmedCurve2D aSeg = new(new LineCurve2D(new Point2D(0, -5), new Direction2D(0, 1)), 0.0, 4.0);
        CurveCurveIntersection2D anInter = new(anE, aSeg, 1.0e-7);
        Assert.Equal(1, anInter.NbPoints());
        Assert.Equal(0.0, anInter.Point(1).X, 1e-12);
        Assert.Equal(-2.0, anInter.Point(1).Y, 1e-12);
    }

    [Fact]
    public void BSplineBSpline_PolylineCrossings()
    {
        BSplineCurve2D aC1 = Polyline((0, 0), (4, 4), (8, 0));
        BSplineCurve2D aC2 = Polyline((0, 1), (8, 1));
        CurveCurveIntersection2D anInter = new(aC1, aC2, 1.0e-7);
        Assert.Equal(2, anInter.NbPoints());
        double[] aXs = [anInter.Point(1).X, anInter.Point(2).X];
        Array.Sort(aXs);
        Assert.Equal(1.0, aXs[0], 1e-9);
        Assert.Equal(7.0, aXs[1], 1e-9);
        AssertOnBoth(anInter, aC1, aC2, 1e-7);
    }

    [Fact]
    public void BSplineCircle_CubicCrossings()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(-2, -1);
        aPoles[2] = new Point2D(-1, 2);
        aPoles[3] = new Point2D(1, -2);
        aPoles[4] = new Point2D(2, 1);
        BezierCurve2D aBez = new(aPoles);
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        CurveCurveIntersection2D anInter = new(aBez, aCircle, 1.0e-7);
        Assert.True(anInter.NbPoints() >= 2);
        for (int i = 1; i <= anInter.NbPoints(); i++)
        {
            Assert.Equal(1.0, anInter.Point(i).DistanceTo(new Point2D(0, 0)), 1e-7);
        }
        AssertOnBoth(anInter, aBez, aCircle, 1e-7);
    }

    [Fact]
    public void SelfIntersection_Loop()
    {
        // A polyline crossing itself once at (2, 2).
        BSplineCurve2D aC = Polyline((0, 0), (4, 4), (4, 0), (0, 4));
        CurveCurveIntersection2D anInter = new(aC, 1.0e-7);
        Assert.Equal(1, anInter.NbPoints());
        Assert.Equal(2.0, anInter.Point(1).X, 1e-9);
        Assert.Equal(2.0, anInter.Point(1).Y, 1e-9);
    }

    [Fact]
    public void SelfIntersection_OfConic_IsEmpty()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        CurveCurveIntersection2D anInter = new(aCircle, 1.0e-7);
        Assert.True(anInter.Intersector().IsDone);
        Assert.Equal(0, anInter.NbPoints());
        Assert.Equal(0, anInter.NbSegments());
    }

    [Fact]
    public void Overlap_GivesSegment()
    {
        BSplineCurve2D aC1 = Polyline((0, 0), (4, 0));
        BSplineCurve2D aC2 = Polyline((2, 0), (6, 0));
        CurveCurveIntersection2D anInter = new(aC1, aC2, 1.0e-7);
        Assert.Equal(1, anInter.NbSegments());
        anInter.Segment(1, out Curve2D aS1, out Curve2D aS2);
        Assert.Equal(2.0, aS1.Value(aS1.FirstParameter).X, 1e-7);
        Assert.Equal(4.0, aS1.Value(aS1.LastParameter).X, 1e-7);
        Assert.Equal(2.0, aS2.Value(aS2.FirstParameter).X, 1e-7);
        Assert.Equal(4.0, aS2.Value(aS2.LastParameter).X, 1e-7);
    }

    [Fact]
    public void NoIntersection()
    {
        CircleCurve2D aC1 = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        CircleCurve2D aC2 = new(new Axis2D(new Point2D(5, 0), new Direction2D(1, 0)), 1.0);
        CurveCurveIntersection2D anInter = new(aC1, aC2);
        Assert.Equal(0, anInter.NbPoints());
        Assert.Equal(0, anInter.NbSegments());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.OutOfRangeException>(() => anInter.Segment(1, out _, out _));
    }

    [Fact]
    public void Empty_HasNoResults()
    {
        CurveCurveIntersection2D anInter = new();
        Assert.Equal(0, anInter.NbPoints());
        Assert.Equal(0, anInter.NbSegments());
    }
}
