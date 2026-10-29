// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dAPI_InterCurveCurve_Test.cxx
// Port of: src/ModelingData/TKG2d/GTests/Geom2dAPI_InterCurveCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using Xunit;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of Geom2dAPI_InterCurveCurve (TKGeomAlgo's and TKG2d's), and known answers.</summary>
public class Geom2dAPI_InterCurveCurveTests
{
    [Fact]
    public void OCC29289_EllipseIntersectionNewtonRoot()
    {
        // Create two ellipses
        gp_Elips2d e1 = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(gp_Dir2d.D.X)), 2.0, 1.0);
        Geom2d_Ellipse Ge1 = new(e1);
        gp_Elips2d e2 = new(new gp_Ax2d(new gp_Pnt2d(0.5, 0.5), new gp_Dir2d(1.0, 1.0)), 2.0, 1.0);
        Geom2d_Ellipse Ge2 = new(e2);

        // Find intersection points
        Geom2dAPI_InterCurveCurve Intersector = new();
        Intersector.Init(Ge1, Ge2, 1.0e-7);
        Assert.True(Intersector.NbPoints() > 0, "Error: intersector found no points");

        // Setup trigonometric equation: A*std::cos(x) + B*Sin(x) + C*std::cos(2*x) + D*Sin(2*x) + E
        double A = 1.875, B = -0.75, C = -0.5, D = -0.25, E = -0.25;
        math_TrigonometricEquationFunction MyF = new(A, B, C, D, E);

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

            math_NewtonFunctionRoot Resol = new(MyF, X, Tol1, Eps, Nit[i - 1]);
            Assert.True(Resol.IsDone(), $"Error: Newton is not done for {Teta}");

            double TetaNewton = Resol.Root();
            Assert.True(Math.Abs(Teta - TetaNewton) <= 1.0e-7, $"Error: Newton root is wrong for {Teta}");
        }
    }

    [Fact]
    public void PointRejectsZeroIndex()
    {
        gp_Elips2d anEllipse1 = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 2.0, 1.0);
        Geom2d_Ellipse aCurve1 = new(anEllipse1);
        gp_Elips2d anEllipse2 = new(new gp_Ax2d(new gp_Pnt2d(0.5, 0.5), new gp_Dir2d(1.0, 1.0)), 2.0, 1.0);
        Geom2d_Ellipse aCurve2 = new(anEllipse2);

        Geom2dAPI_InterCurveCurve anIntersector = new(aCurve1, aCurve2, 1.0e-7);
        Assert.True(anIntersector.NbPoints() > 0);

        Assert.ThrowsAny<Standard_OutOfRange>(() => anIntersector.Point(0));
    }

    // Regression test for bug #11: Init with null handle must raise Standard_NullObject.
    [Fact]
    public void Init_NullHandle_RaisesException()
    {
        Geom2d_Curve aNullCurve = null!;
        gp_Elips2d anEllipse = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 2.0, 1.0);
        Geom2d_Curve aValidCurve = new Geom2d_Ellipse(anEllipse);

        // Both null.
        Assert.ThrowsAny<Standard_NullObject>(() => new Geom2dAPI_InterCurveCurve(aNullCurve, aNullCurve, 1.0e-7));

        // First null.
        Assert.ThrowsAny<Standard_NullObject>(() => new Geom2dAPI_InterCurveCurve(aNullCurve, aValidCurve, 1.0e-7));

        // Second null.
        Assert.ThrowsAny<Standard_NullObject>(() => new Geom2dAPI_InterCurveCurve(aValidCurve, aNullCurve, 1.0e-7));

        // Single-curve Init with null.
        Geom2dAPI_InterCurveCurve anInter = new();
        Assert.ThrowsAny<Standard_NullObject>(() => anInter.Init(aNullCurve, 1.0e-7));
    }

    [Fact]
    public void OCC24889_IntersectionParameterWithinLimits()
    {
        // Bug OCC24889: Geom2dAPI_InterCurveCurve produces result with parameter outside the curve limits

        // Create two circles
        Geom2d_Circle aCircle1 = new(new gp_Ax22d(new gp_Pnt2d(25, -25), new gp_Dir2d(gp_Dir2d.D.X),
                                                  new gp_Dir2d(-0.0, 1)), 155);
        Geom2d_Circle aCircle2 = new(new gp_Ax22d(new gp_Pnt2d(25, 25), new gp_Dir2d(gp_Dir2d.D.X),
                                                  new gp_Dir2d(-0.0, 1)), 155);

        // Create trimmed curves
        Geom2d_TrimmedCurve aTrim1 = new(aCircle1, 1.57079632679490, 2.97959469729228);
        Geom2d_TrimmedCurve aTrim2 = new(aCircle2, 3.30359060633978, 4.71238898038469);

        // Perform intersection
        double aTol = Precision.Confusion();
        Geom2dAPI_InterCurveCurve aIntTool = new(aTrim1, aTrim2, aTol);

        Assert.True(aIntTool.NbPoints() > 0, "Intersection should find at least one point");

        IntRes2d_IntersectionPoint aIntPnt = aIntTool.Intersector().Point(1);

        double aParOnC1 = aIntPnt.ParamOnFirst();
        double aParOnC2 = aIntPnt.ParamOnSecond();

        // Verify that intersection parameters are within the curve limits
        Assert.True(aParOnC1 >= aTrim1.FirstParameter());
        Assert.True(aParOnC1 <= aTrim1.LastParameter());
        Assert.True(aParOnC2 >= aTrim2.FirstParameter());
        Assert.True(aParOnC2 <= aTrim2.LastParameter());

        // Verify that both curves evaluate to the same point at the intersection parameters
        gp_Pnt2d aP1 = aTrim1.Value(aParOnC1);
        gp_Pnt2d aP2 = aTrim2.Value(aParOnC2);
        Assert.True(aP1.SquareDistance(aP2) < 1.0e-14, "Points on both curves at intersection parameters should coincide");
    }

    // ---------------------------------------------------------------------------------------------
    // Known answers

    private static Geom2d_BSplineCurve Polyline(params (double X, double Y)[] thePnts)
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, thePnts.Length);
        for (int i = 0; i < thePnts.Length; i++)
        {
            aPoles[i + 1] = new gp_Pnt2d(thePnts[i].X, thePnts[i].Y);
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
        return new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 1);
    }

    private static void AssertOnBoth(Geom2dAPI_InterCurveCurve theInter, Geom2d_Curve theC1, Geom2d_Curve theC2,
                                     double theTol)
    {
        for (int i = 1; i <= theInter.NbPoints(); i++)
        {
            IntRes2d_IntersectionPoint aP = theInter.Intersector().Point(i);
            Assert.True(theC1.Value(aP.ParamOnFirst()).Distance(aP.Value()) < theTol);
            Assert.True(theC2.Value(aP.ParamOnSecond()).Distance(aP.Value()) < theTol);
        }
    }

    [Fact]
    public void LineCircle_TwoPoints()
    {
        Geom2d_Line aLine = new(new gp_Pnt2d(-5, 0.5), new gp_Dir2d(1, 0));
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.0);
        Geom2dAPI_InterCurveCurve anInter = new(aLine, aCircle, 1.0e-7);
        Assert.Equal(2, anInter.NbPoints());
        Assert.Equal(0, anInter.NbSegments());
        double aX = Math.Sqrt(1 - 0.25);
        double[] aXs = [anInter.Point(1).X(), anInter.Point(2).X()];
        Array.Sort(aXs);
        Assert.Equal(-aX, aXs[0], 1e-12);
        Assert.Equal(aX, aXs[1], 1e-12);
        Assert.Equal(0.5, anInter.Point(1).Y(), 1e-12);
        AssertOnBoth(anInter, aLine, aCircle, 1e-9);
    }

    [Fact]
    public void LineCircle_Tangent()
    {
        Geom2d_Line aLine = new(new gp_Pnt2d(-5, 2), new gp_Dir2d(1, 0));
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        Geom2dAPI_InterCurveCurve anInter = new(aLine, aCircle, 1.0e-7);
        Assert.True(anInter.NbPoints() + anInter.NbSegments() >= 1);
        if (anInter.NbPoints() > 0)
        {
            Assert.Equal(0.0, anInter.Point(1).X(), 1e-3);
            Assert.Equal(2.0, anInter.Point(1).Y(), 1e-6);
        }
    }

    [Fact]
    public void CircleCircle_Secant()
    {
        Geom2d_Circle aC1 = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.0);
        Geom2d_Circle aC2 = new(new gp_Ax2d(new gp_Pnt2d(1, 0), new gp_Dir2d(1, 0)), 1.0);
        Geom2dAPI_InterCurveCurve anInter = new(aC1, aC2, 1.0e-7);
        Assert.Equal(2, anInter.NbPoints());
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(0.5, anInter.Point(i).X(), 1e-12);
            Assert.Equal(Math.Sqrt(0.75), Math.Abs(anInter.Point(i).Y()), 1e-12);
        }
        AssertOnBoth(anInter, aC1, aC2, 1e-9);
    }

    [Fact]
    public void CircleCircle_Identical_GivesSegment()
    {
        Geom2d_Circle aC1 = new(new gp_Ax2d(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 0)), 3.0);
        Geom2d_Circle aC2 = new(new gp_Ax2d(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 0)), 3.0);
        Geom2dAPI_InterCurveCurve anInter = new(aC1, aC2, 1.0e-7);
        Assert.Equal(0, anInter.NbPoints());
        Assert.True(anInter.NbSegments() >= 1);
        anInter.Segment(1, out Geom2d_Curve aS1, out Geom2d_Curve aS2);
        Assert.IsType<Geom2d_TrimmedCurve>(aS1);
        Assert.True(aS1.Value(aS1.FirstParameter()).Distance(aS2.Value(aS2.FirstParameter())) < 1e-7
                    || aS1.Value(aS1.FirstParameter()).Distance(aS2.Value(aS2.LastParameter())) < 1e-7);
    }

    [Fact]
    public void EllipseLine_OnTrimmedCurves()
    {
        Geom2d_Ellipse anE = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0, 2.0);
        Geom2d_TrimmedCurve aSeg = new(new Geom2d_Line(new gp_Pnt2d(0, -5), new gp_Dir2d(0, 1)), 0.0, 4.0);
        Geom2dAPI_InterCurveCurve anInter = new(anE, aSeg, 1.0e-7);
        Assert.Equal(1, anInter.NbPoints());
        Assert.Equal(0.0, anInter.Point(1).X(), 1e-12);
        Assert.Equal(-2.0, anInter.Point(1).Y(), 1e-12);
    }

    [Fact]
    public void BSplineBSpline_PolylineCrossings()
    {
        Geom2d_BSplineCurve aC1 = Polyline((0, 0), (4, 4), (8, 0));
        Geom2d_BSplineCurve aC2 = Polyline((0, 1), (8, 1));
        Geom2dAPI_InterCurveCurve anInter = new(aC1, aC2, 1.0e-7);
        Assert.Equal(2, anInter.NbPoints());
        double[] aXs = [anInter.Point(1).X(), anInter.Point(2).X()];
        Array.Sort(aXs);
        Assert.Equal(1.0, aXs[0], 1e-9);
        Assert.Equal(7.0, aXs[1], 1e-9);
        AssertOnBoth(anInter, aC1, aC2, 1e-7);
    }

    [Fact]
    public void BSplineCircle_CubicCrossings()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt2d(-2, -1);
        aPoles[2] = new gp_Pnt2d(-1, 2);
        aPoles[3] = new gp_Pnt2d(1, -2);
        aPoles[4] = new gp_Pnt2d(2, 1);
        Geom2d_BezierCurve aBez = new(aPoles);
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.0);
        Geom2dAPI_InterCurveCurve anInter = new(aBez, aCircle, 1.0e-7);
        Assert.True(anInter.NbPoints() >= 2);
        for (int i = 1; i <= anInter.NbPoints(); i++)
        {
            Assert.Equal(1.0, anInter.Point(i).Distance(new gp_Pnt2d(0, 0)), 1e-7);
        }
        AssertOnBoth(anInter, aBez, aCircle, 1e-7);
    }

    [Fact]
    public void SelfIntersection_Loop()
    {
        // A polyline crossing itself once at (2, 2).
        Geom2d_BSplineCurve aC = Polyline((0, 0), (4, 4), (4, 0), (0, 4));
        Geom2dAPI_InterCurveCurve anInter = new(aC, 1.0e-7);
        Assert.Equal(1, anInter.NbPoints());
        Assert.Equal(2.0, anInter.Point(1).X(), 1e-9);
        Assert.Equal(2.0, anInter.Point(1).Y(), 1e-9);
    }

    [Fact]
    public void SelfIntersection_OfConic_IsEmpty()
    {
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.0);
        Geom2dAPI_InterCurveCurve anInter = new(aCircle, 1.0e-7);
        Assert.True(anInter.Intersector().IsDone());
        Assert.Equal(0, anInter.NbPoints());
        Assert.Equal(0, anInter.NbSegments());
    }

    [Fact]
    public void Overlap_GivesSegment()
    {
        Geom2d_BSplineCurve aC1 = Polyline((0, 0), (4, 0));
        Geom2d_BSplineCurve aC2 = Polyline((2, 0), (6, 0));
        Geom2dAPI_InterCurveCurve anInter = new(aC1, aC2, 1.0e-7);
        Assert.Equal(1, anInter.NbSegments());
        anInter.Segment(1, out Geom2d_Curve aS1, out Geom2d_Curve aS2);
        Assert.Equal(2.0, aS1.Value(aS1.FirstParameter()).X(), 1e-7);
        Assert.Equal(4.0, aS1.Value(aS1.LastParameter()).X(), 1e-7);
        Assert.Equal(2.0, aS2.Value(aS2.FirstParameter()).X(), 1e-7);
        Assert.Equal(4.0, aS2.Value(aS2.LastParameter()).X(), 1e-7);
    }

    [Fact]
    public void NoIntersection()
    {
        Geom2d_Circle aC1 = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.0);
        Geom2d_Circle aC2 = new(new gp_Ax2d(new gp_Pnt2d(5, 0), new gp_Dir2d(1, 0)), 1.0);
        Geom2dAPI_InterCurveCurve anInter = new(aC1, aC2);
        Assert.Equal(0, anInter.NbPoints());
        Assert.Equal(0, anInter.NbSegments());
        Assert.ThrowsAny<Standard_OutOfRange>(() => anInter.Segment(1, out _, out _));
    }

    [Fact]
    public void Empty_HasNoResults()
    {
        Geom2dAPI_InterCurveCurve anInter = new();
        Assert.Equal(0, anInter.NbPoints());
        Assert.Equal(0, anInter.NbSegments());
    }
}
