// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntCurve's parametric/parametric generics through their Geom2dInt instantiations: the polygon, the distance
/// function, the exact intersection point and the polygon/polygon intersection (two curves and self-intersection),
/// on B-spline, Bezier and trimmed conic curves with known answers.
/// </summary>
public class IntCurvePolyPolyTests
{
    private const double TolConf = 1.0e-7;
    private const double Tol = 1.0e-7;

    // A degree-1 B-spline through the points, its parameter i - 1 at the point i.
    private static CurveAdaptor2D Polyline(params (double X, double Y)[] thePnts)
    {
        int n = thePnts.Length;
        NCollection_Array1<Point2D> aPoles = new(1, n);
        NCollection_Array1<double> aKnots = new(1, n);
        NCollection_Array1<int> aMults = new(1, n);
        for (int i = 1; i <= n; i++)
        {
            aPoles[i] = new Point2D(thePnts[i - 1].X, thePnts[i - 1].Y);
            aKnots[i] = i - 1;
            aMults[i] = (i == 1 || i == n) ? 2 : 1;
        }
        return new CurveAdaptor2D(new BSplineCurve2D(aPoles, aKnots, aMults, 1));
    }

    private static CurveAdaptor2D Bezier(params (double X, double Y)[] thePnts)
    {
        NCollection_Array1<Point2D> aPoles = new(1, thePnts.Length);
        for (int i = 1; i <= thePnts.Length; i++)
        {
            aPoles[i] = new Point2D(thePnts[i - 1].X, thePnts[i - 1].Y);
        }
        return new CurveAdaptor2D(new BezierCurve2D(aPoles));
    }

    // A cubic B-spline with two spans (interior knot of multiplicity 1).
    private static CurveAdaptor2D CubicBSpline(params (double X, double Y)[] thePnts)
    {
        Assert.Equal(5, thePnts.Length);
        NCollection_Array1<Point2D> aPoles = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i] = new Point2D(thePnts[i - 1].X, thePnts[i - 1].Y);
        }
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.5;
        aKnots[3] = 1.0;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        return new CurveAdaptor2D(new BSplineCurve2D(aPoles, aKnots, aMults, 3));
    }

    private static TwoDDomain Domain(Curve2DAdaptorBase C, double theTol = TolConf)
    {
        double u0 = C.FirstParameter(), u1 = C.LastParameter();
        return new TwoDDomain(C.Value(u0), u0, theTol, C.Value(u1), u1, theTol);
    }

    private static TheIntPCurvePCurveOfGInter Intersect(Curve2DAdaptorBase C1, Curve2DAdaptorBase C2)
    {
        TheIntPCurvePCurveOfGInter anInter = new();
        anInter.Perform(C1, Domain(C1), C2, Domain(C2), TolConf, Tol);
        Assert.True(anInter.IsDone);
        return anInter;
    }

    // Every point must be on both curves at its parameters.
    private static void CheckOnCurves(IntersectionPoint2D P, Curve2DAdaptorBase C1, Curve2DAdaptorBase C2,
                                      double theTol = 1.0e-6)
    {
        Assert.True(C1.Value(P.ParamOnFirst()).Distance(P.Value()) < theTol);
        Assert.True(C2.Value(P.ParamOnSecond()).Distance(P.Value()) < theTol);
    }

    private static List<IntersectionPoint2D> SortedByX(Intersection2D I)
    {
        List<IntersectionPoint2D> aList = [];
        for (int i = 1; i <= I.PointCount; i++)
        {
            aList.Add(I.Point(i));
        }
        aList.Sort((a, b) => a.Value().X().CompareTo(b.Value().X()));
        return aList;
    }

    [Fact]
    public void Segments_Crossing_GiveOnePointWithOppositeTransitions()
    {
        CurveAdaptor2D c1 = Polyline((0, 0), (2, 2));
        CurveAdaptor2D c2 = Polyline((0, 2), (2, 0));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        Assert.Equal(1, anInter.PointCount);
        Assert.Equal(0, anInter.SegmentCount);
        IntersectionPoint2D P = anInter.Point(1);
        Assert.Equal(1.0, P.Value().X(), 1e-9);
        Assert.Equal(1.0, P.Value().Y(), 1e-9);
        Assert.Equal(0.5, P.ParamOnFirst(), 1e-9);
        Assert.Equal(0.5, P.ParamOnSecond(), 1e-9);
        Assert.Equal(Position2D.Middle, P.TransitionOfFirst().PositionOnCurve());
        Assert.Equal(Position2D.Middle, P.TransitionOfSecond().PositionOnCurve());
        // C1 x C2 = (1,1) x (1,-1) < 0: the first curve goes in, the second out.
        Assert.Equal(TypeTrans.In, P.TransitionOfFirst().TransitionType());
        Assert.Equal(TypeTrans.Out, P.TransitionOfSecond().TransitionType());
    }

    [Fact]
    public void Zigzag_CrossesLine_ThreeTimes()
    {
        CurveAdaptor2D c1 = Polyline((0, 0), (1, 2), (2, 0), (3, 2));
        CurveAdaptor2D c2 = Polyline((-1, 1), (4, 1));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        Assert.Equal(3, anInter.PointCount);
        List<IntersectionPoint2D> aPnts = SortedByX(anInter);
        double[] anX = [0.5, 1.5, 2.5];
        for (int i = 0; i < 3; i++)
        {
            Assert.Equal(anX[i], aPnts[i].Value().X(), 1e-9);
            Assert.Equal(1.0, aPnts[i].Value().Y(), 1e-9);
            Assert.Equal(anX[i], aPnts[i].ParamOnFirst(), 1e-9);
            Assert.Equal((anX[i] + 1.0) / 5.0, aPnts[i].ParamOnSecond(), 1e-9);
            CheckOnCurves(aPnts[i], c1, c2);
        }
    }

    [Fact]
    public void BezierCubic_CrossesLine_AtItsEndsAndMiddle()
    {
        // y(t) = 6t(1-t)(1-2t), x(t) = 3t: zeros at t = 0, 0.5, 1.
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, -2), (3, 0));
        CurveAdaptor2D c2 = Polyline((-1, 0), (4, 0));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        Assert.Equal(3, anInter.PointCount);
        List<IntersectionPoint2D> aPnts = SortedByX(anInter);
        double[] aT = [0.0, 0.5, 1.0];
        Position2D[] aPos = [Position2D.Head, Position2D.Middle, Position2D.End];
        for (int i = 0; i < 3; i++)
        {
            Assert.Equal(3.0 * aT[i], aPnts[i].Value().X(), 1e-8);
            Assert.Equal(0.0, aPnts[i].Value().Y(), 1e-8);
            Assert.Equal(aT[i], aPnts[i].ParamOnFirst(), 1e-8);
            Assert.Equal(aPos[i], aPnts[i].TransitionOfFirst().PositionOnCurve());
            Assert.Equal(Position2D.Middle, aPnts[i].TransitionOfSecond().PositionOnCurve());
            CheckOnCurves(aPnts[i], c1, c2);
        }
    }

    [Fact]
    public void TwoBezierParabolas_CrossTwice()
    {
        // x = 2t on both; 4t - 4t^2 = 1.5 - 4t + 4t^2 at t = 0.25 and 0.75.
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, 0));
        CurveAdaptor2D c2 = Bezier((0, 1.5), (1, -0.5), (2, 1.5));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        Assert.Equal(2, anInter.PointCount);
        List<IntersectionPoint2D> aPnts = SortedByX(anInter);
        double[] aT = [0.25, 0.75];
        for (int i = 0; i < 2; i++)
        {
            Assert.Equal(2.0 * aT[i], aPnts[i].Value().X(), 1e-8);
            Assert.Equal(0.75, aPnts[i].Value().Y(), 1e-8);
            Assert.Equal(aT[i], aPnts[i].ParamOnFirst(), 1e-8);
            Assert.Equal(aT[i], aPnts[i].ParamOnSecond(), 1e-8);
            CheckOnCurves(aPnts[i], c1, c2);
        }
        // Opposite crossings: the transitions alternate.
        Assert.NotEqual(aPnts[0].TransitionOfFirst().TransitionType(), aPnts[1].TransitionOfFirst().TransitionType());
    }

    [Fact]
    public void CubicBSplines_CrossingsMatchDenseSampling()
    {
        CurveAdaptor2D c1 = CubicBSpline((0, 0), (1, 3), (2, -3), (3, 3), (4, 0));
        CurveAdaptor2D c2 = CubicBSpline((0, 0.5), (1, -1), (2, 1.5), (3, -1), (4, 0.5));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        // Both x(t) are the same function of t (equally spaced poles, same knots): count the sign changes of
        // y1(t) - y2(t).
        int aChanges = 0;
        double aPrev = c1.Value(0).Y() - c2.Value(0).Y();
        for (int i = 1; i <= 10000; i++)
        {
            double t = i / 10000.0;
            double d = c1.Value(t).Y() - c2.Value(t).Y();
            if (d * aPrev < 0)
            {
                aChanges++;
            }
            aPrev = d;
        }
        Assert.True(aChanges >= 2);
        Assert.Equal(aChanges, anInter.PointCount);
        for (int i = 1; i <= anInter.PointCount; i++)
        {
            IntersectionPoint2D P = anInter.Point(i);
            CheckOnCurves(P, c1, c2, 1e-7);
            Assert.Equal(P.ParamOnFirst(), P.ParamOnSecond(), 1e-7);
        }
    }

    [Fact]
    public void TrimmedCircles_TreatedAsParametric_CrossAtKnownPoints()
    {
        CircleCurve2D aC1 = new(new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0));
        CircleCurve2D aC2 = new(new Circle2D(new Axis2D(new Point2D(1, 0), new Direction2D(1, 0)), 1.0));
        CurveAdaptor2D c1 = new(new TrimmedCurve2D(aC1, -2.0, 2.0));
        CurveAdaptor2D c2 = new(new TrimmedCurve2D(aC2, 1.0, 5.0));
        TheIntPCurvePCurveOfGInter anInter = new();
        anInter.Perform(c1, Domain(c1), c2, Domain(c2), TolConf, Tol);

        Assert.Equal(2, anInter.PointCount);
        double aY = Math.Sqrt(3.0) / 2.0;
        for (int i = 1; i <= 2; i++)
        {
            IntersectionPoint2D P = anInter.Point(i);
            Assert.Equal(0.5, P.Value().X(), 1e-8);
            Assert.Equal(aY, Math.Abs(P.Value().Y()), 1e-8);
            double anExpected1 = P.Value().Y() > 0 ? Math.PI / 3 : -Math.PI / 3;
            double anExpected2 = P.Value().Y() > 0 ? 2 * Math.PI / 3 : 4 * Math.PI / 3;
            // The trimmed curves' parameters are the circles' angles, up to the period (OCCT adjusts the trim).
            Assert.Equal(0.0, Math.IEEERemainder(P.ParamOnFirst() - anExpected1, 2 * Math.PI), 1e-8);
            Assert.Equal(0.0, Math.IEEERemainder(P.ParamOnSecond() - anExpected2, 2 * Math.PI), 1e-8);
            CheckOnCurves(P, c1, c2, 1e-8);
        }
    }

    [Fact]
    public void Parabola_TouchingLine_GivesATangentSolution()
    {
        // y = 4t(1-t) reaches 1 at t = 0.5, x = 1.
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, 0));
        CurveAdaptor2D c2 = Polyline((0, 1), (2, 1));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        Assert.True(anInter.PointCount + anInter.SegmentCount >= 1);
        for (int i = 1; i <= anInter.PointCount; i++)
        {
            IntersectionPoint2D P = anInter.Point(i);
            Assert.True(P.Value().Distance(new Point2D(1, 1)) < 1e-3);
        }
        for (int i = 1; i <= anInter.SegmentCount; i++)
        {
            IntersectionSegment2D S = anInter.Segment(i);
            Assert.True(S.FirstPoint().Value().Distance(new Point2D(1, 1)) < 1e-3);
            Assert.True(S.LastPoint().Value().Distance(new Point2D(1, 1)) < 1e-3);
        }
    }

    [Fact]
    public void OverlappingSegments_GiveOneSegment()
    {
        CurveAdaptor2D c1 = Polyline((0, 0), (2, 0));
        CurveAdaptor2D c2 = Polyline((1, 0), (3, 0));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        Assert.Equal(1, anInter.SegmentCount);
        IntersectionSegment2D S = anInter.Segment(1);
        double x0 = S.FirstPoint().Value().X(), x1 = S.LastPoint().Value().X();
        Assert.Equal(1.0, Math.Min(x0, x1), 1e-7);
        Assert.Equal(2.0, Math.Max(x0, x1), 1e-7);
        Assert.False(S.IsOpposite);
        for (int i = 1; i <= anInter.PointCount; i++)
        {
            CheckOnCurves(anInter.Point(i), c1, c2);
        }
    }

    [Fact]
    public void OverlappingOppositeSegments_GiveAnOppositeSegment()
    {
        CurveAdaptor2D c1 = Polyline((0, 0), (2, 0));
        CurveAdaptor2D c2 = Polyline((3, 0), (1, 0));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        Assert.Equal(1, anInter.SegmentCount);
        Assert.True(anInter.Segment(1).IsOpposite);
    }

    [Fact]
    public void SegmentsMeetingEndToHead_GiveAnEndPoint()
    {
        CurveAdaptor2D c1 = Polyline((0, 0), (1, 0));
        CurveAdaptor2D c2 = Polyline((1, 0), (1, 1));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);

        Assert.Equal(1, anInter.PointCount);
        IntersectionPoint2D P = anInter.Point(1);
        Assert.Equal(1.0, P.Value().X(), 1e-9);
        Assert.Equal(0.0, P.Value().Y(), 1e-9);
        Assert.Equal(Position2D.End, P.TransitionOfFirst().PositionOnCurve());
        Assert.Equal(Position2D.Head, P.TransitionOfSecond().PositionOnCurve());
        Assert.Equal(1.0, P.ParamOnFirst(), 1e-9);
        Assert.Equal(0.0, P.ParamOnSecond(), 1e-9);
    }

    [Fact]
    public void DisjointCurves_GiveNothing()
    {
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, 0));
        CurveAdaptor2D c2 = Bezier((0, 3), (1, 5), (2, 3));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);
        Assert.True(anInter.IsEmpty);
    }

    [Fact]
    public void NearlyTouchingCurves_OutsideTolerance_GiveNothing()
    {
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, 0));
        CurveAdaptor2D c2 = Polyline((0, 1.001), (2, 1.001));
        TheIntPCurvePCurveOfGInter anInter = Intersect(c1, c2);
        Assert.True(anInter.IsEmpty);
    }

    [Fact]
    public void SelfIntersection_OfPolylineFigure_FindsTheCrossing()
    {
        // (0,0)-(2,2) and (2,0)-(0,2) cross at (1,1): parameters 0.5 and 2.5.
        CurveAdaptor2D c = Polyline((0, 0), (2, 2), (2, 0), (0, 2));
        TheIntPCurvePCurveOfGInter anInter = new();
        anInter.Perform(c, Domain(c), TolConf, Tol);

        Assert.True(anInter.IsDone);
        Assert.Equal(1, anInter.PointCount);
        IntersectionPoint2D P = anInter.Point(1);
        Assert.Equal(1.0, P.Value().X(), 1e-9);
        Assert.Equal(1.0, P.Value().Y(), 1e-9);
        double u = Math.Min(P.ParamOnFirst(), P.ParamOnSecond());
        double v = Math.Max(P.ParamOnFirst(), P.ParamOnSecond());
        Assert.Equal(0.5, u, 1e-9);
        Assert.Equal(2.5, v, 1e-9);
    }

    [Fact]
    public void SelfIntersection_OfBezierLoop_FindsTheDoublePoint()
    {
        // x(t) - x(1-t) = (1-2t)(-10t^2 + 10t - 1) and y(t) = y(1-t): the loop closes at t = 1/2 -+ sqrt(60)/20.
        CurveAdaptor2D c = Bezier((0, 0), (2, 1), (-1, 1), (1, 0));
        TheIntPCurvePCurveOfGInter anInter = new();
        anInter.Perform(c, Domain(c), TolConf, Tol);

        Assert.True(anInter.IsDone);
        Assert.Equal(1, anInter.PointCount);
        IntersectionPoint2D P = anInter.Point(1);
        double u = Math.Min(P.ParamOnFirst(), P.ParamOnSecond());
        double v = Math.Max(P.ParamOnFirst(), P.ParamOnSecond());
        double d = Math.Sqrt(60.0) / 20.0;
        Assert.Equal(0.5 - d, u, 1e-8);
        Assert.Equal(0.5 + d, v, 1e-8);
        Assert.True(c.Value(u).Distance(c.Value(v)) < 1e-8);
    }

    [Fact]
    public void SelfIntersection_OfSimpleArc_FindsNothing()
    {
        CurveAdaptor2D c = Bezier((0, 0), (1, 2), (2, 0));
        TheIntPCurvePCurveOfGInter anInter = new();
        anInter.Perform(c, Domain(c), TolConf, Tol);
        Assert.True(anInter.IsDone);
        Assert.True(anInter.IsEmpty);
    }

    [Fact]
    public void MinNbSamples_DefaultsTo20_AndCanBeSet()
    {
        TheIntPCurvePCurveOfGInter anInter = new();
        Assert.Equal(20, anInter.GetMinNbSamples());
        anInter.SetMinNbSamples(50);
        Assert.Equal(50, anInter.GetMinNbSamples());

        CurveAdaptor2D c1 = Polyline((0, 0), (2, 2));
        CurveAdaptor2D c2 = Polyline((0, 2), (2, 0));
        anInter.Perform(c1, Domain(c1), c2, Domain(c2), TolConf, Tol);
        Assert.Equal(1, anInter.PointCount);
    }

    [Fact]
    public void NotDone_BeforePerform()
    {
        TheIntPCurvePCurveOfGInter anInter = new();
        Assert.False(anInter.IsDone);
        Assert.ThrowsAny<NotDoneException>(() => anInter.PointCount);
    }

    [Fact]
    public void Polygon_SamplesTheDomain()
    {
        CurveAdaptor2D c = Bezier((0, 0), (1, 2), (2, 0));
        TwoDDomain D = new(c.Value(0.2), 0.2, TolConf, c.Value(0.9), 0.9, TolConf);
        ThePolygon2dOfTheIntPCurvePCurveOfGInter aPoly = new(c, 15, D, Tol);

        Assert.Equal(14, aPoly.SegmentCount);
        Assert.False(aPoly.Closed());
        Assert.Equal(0.2, aPoly.InfParameter(), 1e-15);
        Assert.Equal(0.9, aPoly.SupParameter(), 1e-12);
        Assert.Equal(0.2, aPoly.ApproxParamOnCurve(1, 0.0), 1e-15);
        Assert.Equal(0.9, aPoly.ApproxParamOnCurve(14, 1.0), 1e-12);
        Assert.Equal(0.9, aPoly.ApproxParamOnCurve(15, 0.0), 1e-12);
        Assert.Equal(0.2 + 0.25 * 0.05, aPoly.ApproxParamOnCurve(1, 0.25), 1e-12);

        aPoly.Segment(1, out Point2D aBegin, out Point2D anEnd);
        Assert.True(aBegin.Distance(c.Value(0.2)) < 1e-15);
        Assert.True(anEnd.Distance(c.Value(0.25)) < 1e-12);
        Assert.ThrowsAny<OutOfRangeException>(() => aPoly.Segment(15, out _, out _));

        // The deflection bounds the distance between the curve and its polygon, and the box holds the curve.
        Box2d aBox = aPoly.Bounding();
        for (int i = 0; i <= 700; i++)
        {
            double u = 0.2 + i * 0.001;
            Assert.False(aBox.IsOut(c.Value(u)));
        }
        double aMaxDev = 0.0;
        for (int s = 1; s <= 14; s++)
        {
            aPoly.Segment(s, out Point2D a, out Point2D b);
            Line2D L = new(a, new Direction2D(new Vector2D(a, b)));
            aMaxDev = Math.Max(aMaxDev, L.Distance(c.Value(0.2 + (s - 0.5) * 0.05)));
        }
        Assert.True(aMaxDev > 0.0);
        Assert.True(aPoly.DeflectionOverEstimation() >= aMaxDev);
    }

    [Fact]
    public void Polygon_ClosedAndDeflection()
    {
        CurveAdaptor2D c = Polyline((0, 0), (4, 0));
        ThePolygon2dOfTheIntPCurvePCurveOfGInter aPoly = new(c, 5, Domain(c), Tol);
        Assert.Equal(4, aPoly.SegmentCount);
        Assert.Equal(Math.Min(0.000000001, Tol / 100.0), aPoly.DeflectionOverEstimation());
        aPoly.Closed(true);
        Assert.True(aPoly.Closed());
        Assert.Equal(5, aPoly.SegmentCount);
        aPoly.Segment(5, out Point2D aBegin, out Point2D anEnd);
        Assert.Equal(4.0, aBegin.X(), 1e-15);
        Assert.Equal(0.0, anEnd.X(), 1e-15);
        aPoly.SetDeflectionOverEstimation(0.5);
        Assert.Equal(0.5, aPoly.DeflectionOverEstimation());
        Assert.False(aPoly.Bounding().IsOut(new Point2D(2, 0.4)));
    }

    [Fact]
    public void Polygon_FewPoints_UsesThree()
    {
        CurveAdaptor2D c = Polyline((0, 0), (4, 0));
        ThePolygon2dOfTheIntPCurvePCurveOfGInter aPoly = new(c, 1, Domain(c), Tol);
        Assert.Equal(2, aPoly.SegmentCount);
    }

    [Fact]
    public void Polygon_AutoIntersectionIsPossible()
    {
        CurveAdaptor2D aLine = Polyline((0, 0), (4, 1));
        Assert.False(new ThePolygon2dOfTheIntPCurvePCurveOfGInter(aLine, 10, Domain(aLine), Tol)
                         .AutoIntersectionIsPossible());
        CurveAdaptor2D aLoop = Bezier((0, 0), (2, 1), (-1, 1), (1, 0));
        Assert.True(new ThePolygon2dOfTheIntPCurvePCurveOfGInter(aLoop, 20, Domain(aLoop), Tol)
                        .AutoIntersectionIsPossible());
    }

    [Fact]
    public void Polygon_ComputeWithBox_KeepsThePartNearTheBox()
    {
        CurveAdaptor2D c = Bezier((0, 0), (1, 2), (2, 0));
        ThePolygon2dOfTheIntPCurvePCurveOfGInter aPoly = new(c, 40, Domain(c), Tol);
        Box2d anOther = Box2d.FromPoint(new Point2D(0.1, 0.0)).Added(new Point2D(0.3, 0.6));
        aPoly.ComputeWithBox(c, anOther);
        Assert.True(aPoly.SegmentCount < 39);
        Assert.True(aPoly.SegmentCount >= 1);
        // The kept part covers the curve inside the box.
        Assert.True(aPoly.InfParameter() <= 0.06);

        ThePolygon2dOfTheIntPCurvePCurveOfGInter aFar = new(c, 40, Domain(c), Tol);
        aFar.ComputeWithBox(c, Box2d.FromPoint(new Point2D(10, 10)).Added(new Point2D(11, 11)));
        Assert.Equal(1, aFar.SegmentCount);
        Assert.True(aFar.Bounding().IsVoid);
    }

    [Fact]
    public void Polygon_CalculRegion()
    {
        CurveAdaptor2D c = Polyline((0, 0), (4, 0));
        ThePolygon2dOfTheIntPCurvePCurveOfGInter aPoly = new(c, 5, Domain(c), Tol);
        Assert.Equal(0, aPoly.CalculRegion(0.5, 0.5, 0, 1, 0, 1));
        Assert.Equal(1, aPoly.CalculRegion(-1, 0.5, 0, 1, 0, 1));
        Assert.Equal(2 | 8, aPoly.CalculRegion(2, 2, 0, 1, 0, 1));
        Assert.Equal(1 | 4, aPoly.CalculRegion(-2, -2, 0, 1, 0, 1));
    }

    [Fact]
    public void DistBetweenPCurves_ValuesAndDerivatives()
    {
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, -2), (3, 0));
        CurveAdaptor2D c2 = CubicBSpline((0, 0.5), (1, -1), (2, 1.5), (3, -1), (4, 0.5));
        TheDistBetweenPCurvesOfTheIntPCurvePCurveOfGInter F = new(c1, c2);
        Assert.Equal(2, F.NbVariables());
        Assert.Equal(2, F.NbEquations());

        math_Vector X = new(1, 2);
        X[1] = 0.3;
        X[2] = 0.7;
        math_Vector V = new(1, 2);
        Assert.True(F.Value(X, V));
        Point2D P1 = c1.Value(0.3), P2 = c2.Value(0.7);
        Assert.Equal(P1.X() - P2.X(), V[1], 1e-15);
        Assert.Equal(P1.Y() - P2.Y(), V[2], 1e-15);

        Matrix D = new(1, 2, 1, 2);
        Assert.True(F.Derivatives(X, D));
        const double h = 1e-6;
        for (int j = 1; j <= 2; j++)
        {
            math_Vector Xp = new(1, 2), Xm = new(1, 2), Vp = new(1, 2), Vm = new(1, 2);
            Xp[1] = X[1];
            Xp[2] = X[2];
            Xm[1] = X[1];
            Xm[2] = X[2];
            Xp[j] += h;
            Xm[j] -= h;
            F.Value(Xp, Vp);
            F.Value(Xm, Vm);
            for (int i = 1; i <= 2; i++)
            {
                Assert.Equal((Vp[i] - Vm[i]) / (2 * h), D[i, j], 1e-6);
            }
        }

        math_Vector V2 = new(1, 2);
        Matrix D2 = new(1, 2, 1, 2);
        Assert.True(F.Values(X, V2, D2));
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(V[i], V2[i]);
            for (int j = 1; j <= 2; j++)
            {
                Assert.Equal(D[i, j], D2[i, j]);
            }
        }
    }

    [Fact]
    public void ExactIntersectionPoint_ConvergesFromANearbyStart()
    {
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, 0));
        CurveAdaptor2D c2 = Bezier((0, 1.5), (1, -0.5), (2, 1.5));
        ExactIntersectionPointOfTheIntPCurvePCurveOfGInter E = new(c1, c2, TolConf);
        E.Perform(0.3, 0.2, 0.0, 0.0, 0.5, 0.5);
        Assert.Equal(1, E.RootCount);
        Assert.False(E.AnErrorOccurred());
        E.Roots(out double U, out double V);
        Assert.Equal(0.25, U, 1e-9);
        Assert.Equal(0.25, V, 1e-9);

        E.Perform(0.7, 0.8, 0.5, 0.5, 1.0, 1.0);
        Assert.Equal(1, E.RootCount);
        E.Roots(out U, out V);
        Assert.Equal(0.75, U, 1e-9);
        Assert.Equal(0.75, V, 1e-9);
    }

    [Fact]
    public void ExactIntersectionPoint_FindsNoRootBetweenDisjointCurves()
    {
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, 0));
        CurveAdaptor2D c2 = Bezier((0, 3), (1, 5), (2, 3));
        ExactIntersectionPointOfTheIntPCurvePCurveOfGInter E = new(c1, c2, TolConf);
        E.Perform(0.5, 0.5, 0.0, 0.0, 1.0, 1.0);
        Assert.Equal(0, E.RootCount);
    }

    [Fact]
    public void ExactIntersectionPoint_FromPolygonSegments()
    {
        CurveAdaptor2D c1 = Bezier((0, 0), (1, 2), (2, 0));
        CurveAdaptor2D c2 = Bezier((0, 1.5), (1, -0.5), (2, 1.5));
        ThePolygon2dOfTheIntPCurvePCurveOfGInter aPoly1 = new(c1, 21, Domain(c1), Tol);
        ThePolygon2dOfTheIntPCurvePCurveOfGInter aPoly2 = new(c2, 21, Domain(c2), Tol);
        ExactIntersectionPointOfTheIntPCurvePCurveOfGInter E = new(c1, c2, TolConf);
        // The crossing at t = 0.25 is in the 6th segment of both polygons (steps of 0.05).
        int aSeg1 = 6, aSeg2 = 6;
        double aPar1 = 0.0, aPar2 = 0.0;
        E.Perform(aPoly1, aPoly2, ref aSeg1, ref aSeg2, ref aPar1, ref aPar2);
        Assert.Equal(1, E.RootCount);
        E.Roots(out double U, out double V);
        Assert.Equal(0.25, U, 1e-9);
        Assert.Equal(0.25, V, 1e-9);

        // Segment indices at the ends are moved inside the polygons.
        int aLast1 = 20, aLast2 = 0;
        double aP1 = 0.0, aP2 = 0.5;
        E.Perform(aPoly1, aPoly2, ref aLast1, ref aLast2, ref aP1, ref aP2);
        Assert.Equal(19, aLast1);
        Assert.Equal(1.0, aP1);
        Assert.Equal(1, aLast2);
        Assert.Equal(0.0, aP2);
    }
}
