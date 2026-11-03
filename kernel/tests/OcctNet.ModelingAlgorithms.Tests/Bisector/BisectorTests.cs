// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The Bisector package on simple configurations: each bisector's points are equidistant from the two elements, the
/// kind of bisector chosen for lines, circles, points and other curves, the polygon and point values, and the
/// freezing rules (decisions of unit 5M). OCCT has no GTests for Bisector.
/// </summary>
public class BisectorTests
{
    private const double Tol = 1.0e-7;

    private static Geom2d_TrimmedCurve Segment(double x1, double y1, double x2, double y2)
    {
        Point2D p1 = new(x1, y1), p2 = new(x2, y2);
        return new Geom2d_TrimmedCurve(new Geom2d_Line(p1, new Direction2D(x2 - x1, y2 - y1)), 0.0, p1.Distance(p2));
    }

    private static Geom2d_TrimmedCurve Arc(double cx, double cy, double r, double u1, double u2) =>
        new(new Geom2d_Circle(new Circle2D(new Axis2D(new Point2D(cx, cy), new Direction2D(1.0, 0.0)), r)), u1, u2);

    private static Geom2d_BSplineCurve Spline(params double[] xy)
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aPoles[i] = new Point2D(xy[2 * i - 2], xy[2 * i - 1]);
        }
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    private static double Dist(Geom2d_Curve theC, in Point2D theP)
    {
        Geom2dAPI_ProjectPointOnCurve aProj = new(theP, theC, theC.FirstParameter(), theC.LastParameter());
        double aD = Math.Min(theP.Distance(theC.Value(theC.FirstParameter())),
                             theP.Distance(theC.Value(theC.LastParameter())));
        return aProj.NbPoints() > 0 ? Math.Min(aD, aProj.LowerDistance()) : aD;
    }

    [Fact]
    public void SquareCorner_IsTheDiagonal()
    {
        // The corner (2, 0) of the square [0, 2]^2 travelled anticlockwise: the bisector goes inside along (-1, 1).
        Geom2d_TrimmedCurve a = Segment(0, 0, 2, 0), b = Segment(2, 0, 2, 2);
        Bisector_Bisec aBis = new();
        aBis.Perform(a, b, new Point2D(2, 0), new Vector2D(1, 0), new Vector2D(0, -1), 1.0, JoinKind.Arc,
                     Tol);
        Geom2d_TrimmedCurve aCurve = aBis.Value();
        Bisector_BisecAna anAna = Assert.IsType<Bisector_BisecAna>(aCurve.BasisCurve());
        Assert.IsType<Geom2d_Line>(anAna.Geom2dCurve());
        Assert.Equal(1, anAna.NbIntervals());
        Assert.False(anAna.IsExtendAtStart() || anAna.IsExtendAtEnd());
        Point2D aStart = aCurve.Value(aCurve.FirstParameter());
        Assert.True(aStart.Distance(new Point2D(2, 0)) < Tol);
        Point2D aP = aCurve.Value(aCurve.FirstParameter() + Math.Sqrt(2.0));
        Assert.Equal(1.0, aP.X(), 1e-12);
        Assert.Equal(1.0, aP.Y(), 1e-12);
        Assert.Equal(aCurve.FirstParameter() + 1.0, anAna.Parameter(aCurve.Value(aCurve.FirstParameter() + 1.0)), 1e-12);
        Assert.ThrowsAny<OutOfRangeException>(() => anAna.IntervalFirst(2));
    }

    [Fact]
    public void CirclesAndLines_AreEquidistant()
    {
        // A line then a quarter circle tangent to it; a point and a circle; two points.
        Geom2d_TrimmedCurve a = Segment(-2, 1, 0, 1);
        Geom2d_TrimmedCurve c = Arc(0, 0, 1, Math.PI / 2, 3.0);
        Bisector_Bisec aBis = new();
        aBis.Perform(a, c, new Point2D(-3, -2), new Vector2D(1, 1), new Vector2D(-1, 1), 1.0,
                     JoinKind.Arc, Tol, false);
        Geom2d_TrimmedCurve aCurve = aBis.Value();
        Assert.IsType<Bisector_BisecAna>(aCurve.BasisCurve());
        Geom2d_Line aL = new(new Point2D(-2, 1), new Direction2D(1, 0));
        Geom2d_Circle aC = new(new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0));
        for (int k = 1; k <= 5; k++)
        {
            Point2D aP = aCurve.Value(aCurve.FirstParameter() + 0.3 * k);
            double d1 = aL.Lin2d().Distance(aP);
            double d2 = Math.Abs(aC.Circ2d().Distance(aP));
            Assert.Equal(d1, d2, 1e-9);
        }

        Bisector_Bisec aPP = new();
        aPP.Perform(new Geom2d_CartesianPoint(0, 0), new Geom2d_CartesianPoint(2, 0), new Point2D(1, 0),
                    new Vector2D(0, 1), new Vector2D(0, -1), 1.0);
        Point2D aQ = aPP.Value().Value(aPP.Value().FirstParameter() + 3.0);
        Assert.Equal(1.0, aQ.X(), 1e-12);
    }

    [Fact]
    public void CoincidentPoints_Throw()
    {
        Bisector_Bisec aPP = new();
        Assert.ThrowsAny<GccAna_NoSolution>(() => aPP.Perform(new Geom2d_CartesianPoint(1, 1),
                                                                 new Geom2d_CartesianPoint(1, 1), new Point2D(1, 1),
                                                                 new Vector2D(1, 0), new Vector2D(0, 1), 1.0));
    }

    [Fact]
    public void PointAndSpline_IsBisecPC()
    {
        Geom2d_BSplineCurve aS = Spline(0, 0, 1, 1, 2, 1, 3, 0);
        Bisector_BisecPC aPC = new(aS, new Point2D(1.5, -2.0), -1.0);
        Assert.False(aPC.IsEmpty());
        Assert.False(aPC.IsPeriodic());
        Assert.Equal(aPC.FirstParameter() + aPC.LastParameter() - 0.25, aPC.ReversedParameter(0.25));
        for (int k = 1; k < 10; k++)
        {
            double u = aPC.FirstParameter() + k * (aPC.LastParameter() - aPC.FirstParameter()) / 10.0;
            Point2D aP = aPC.Value(u);
            Assert.Equal(aP.Distance(new Point2D(1.5, -2.0)), Dist(aS, aP), 1e-7);
            Assert.Equal(aP.SquareDistance(new Point2D(1.5, -2.0)), aPC.Distance(u), 1e-9);
            Assert.Equal(u, aPC.LinkCurveBis(aPC.LinkBisCurve(u)));
        }
        Assert.ThrowsAny<MethodNotImplementedException>(() => aPC.Reversed());
        Assert.ThrowsAny<Geom2d_UndefinedDerivative>(() => aPC.DN(0.5, 4));
    }

    [Fact]
    public void TwoSplines_IsBisecCC_WithChangeGuide()
    {
        // Two arcs of B-splines facing each other: the bisector between them, and the same parameterised by the second.
        Geom2d_BSplineCurve aS1 = Spline(0, 0, 1, 0.5, 2, 0.5, 3, 0);
        Geom2d_BSplineCurve aS2 = Spline(3, 3, 2, 2.5, 1, 2.5, 0, 3);
        Bisector_BisecCC aCC = new(aS1, aS2, 1.0, 1.0, new Point2D(0.0, 1.5));
        Assert.False(aCC.IsEmpty());
        Assert.True(aCC.Polygon().Length() > 2);
        for (int k = 1; k < 10; k++)
        {
            double u = aCC.FirstParameter() + k * (aCC.LastParameter() - aCC.FirstParameter()) / 10.0;
            Point2D aP = aCC.Value(u);
            Assert.Equal(Dist(aS1, aP), Dist(aS2, aP), 1e-6);
        }
        Bisector_BisecCC aGuide = aCC.ChangeGuide();
        Assert.Same(aCC.Curve(2), aGuide.Curve(1));
        Assert.Equal(aCC.Polygon().Length(), aGuide.Polygon().Length());
        Assert.ThrowsAny<OutOfRangeException>(() => aCC.Curve(3));

        // Copy owns copies of everything.
        Bisector_BisecCC aCopy = (Bisector_BisecCC)aCC.Copy();
        Assert.NotSame(aCC.Curve(1), aCopy.Curve(1));
        Assert.NotSame(aCC.Polygon(), aCopy.Polygon());
        Assert.Equal(aCC.Value(aCC.FirstParameter()), aCopy.Value(aCopy.FirstParameter()));
    }

    [Fact]
    public void Inter_OfNeighbouringCorners()
    {
        // The square [0, 2]^2: the bisectors of the corners (2, 0) and (2, 2) meet at the centre.
        Geom2d_TrimmedCurve a = Segment(0, 0, 2, 0), b = Segment(2, 0, 2, 2), c = Segment(2, 2, 0, 2);
        Bisector_Bisec aB1 = new(), aB2 = new();
        aB1.Perform(a, b, new Point2D(2, 0), new Vector2D(1, 0), new Vector2D(0, -1), 1.0, JoinKind.Arc,
                    Tol);
        aB2.Perform(b, c, new Point2D(2, 2), new Vector2D(0, 1), new Vector2D(1, 0), 1.0, JoinKind.Arc,
                    Tol);
        Bisector_Inter anInter = new(aB1, BisectorRecordings.Domain(aB1.Value(), Tol), aB2,
                                     BisectorRecordings.Domain(aB2.Value(), Tol), Tol, Tol, true);
        Assert.True(anInter.IsDone());
        Assert.Equal(1, anInter.NbPoints());
        Point2D aP = anInter.Point(1).Value();
        Assert.Equal(1.0, aP.X(), 1e-9);
        Assert.Equal(1.0, aP.Y(), 1e-9);
    }

    [Fact]
    public void IsConvex_BySide()
    {
        Geom2d_TrimmedCurve anArc = Arc(0, 0, 1, 0, 1); // anticlockwise: the centre is on its left
        Assert.True(Bisector.IsConvex(anArc, -1.0));
        Assert.False(Bisector.IsConvex(anArc, 1.0));
        Assert.True(Bisector.IsConvex(Segment(0, 0, 1, 0), 1.0));
    }

    [Fact]
    public void PolyBis_AndPointOnBis()
    {
        Bisector_PointOnBis aDefault = new();
        Assert.Equal(default, aDefault);
        Bisector_PointOnBis aP = new(1.0, 2.0, 3.0, 4.0, new Point2D(5, 6));
        Assert.Equal(3.0, aP.ParamOnBis());
        Assert.False(aP.IsInfinite());
        Assert.True(aP.WithIsInfinite(true).IsInfinite());
        Assert.Equal(7.0, aP.WithParamOnBis(7.0).ParamOnBis());
        Assert.Equal(3.0, aP.ParamOnBis());
        Assert.NotEqual(aP, aP.WithDistance(0.0));

        Bisector_PolyBis aPoly = new();
        Assert.True(aPoly.IsEmpty());
        for (int i = 0; i <= 10; i++)
        {
            aPoly.Append(new Bisector_PointOnBis(i, i, i * 0.5, 0.0, new Point2D(i, 0)));
        }
        Assert.Equal(11, aPoly.Length());
        Assert.Equal(0.0, aPoly.First().ParamOnBis());
        Assert.Equal(5.0, aPoly.Last().ParamOnBis());
        Assert.Equal(3, aPoly.Interval(1.2));
        Assert.Equal(10, aPoly.Interval(5.0));
        Bisector_PolyBis aCopy = new(aPoly);
        aPoly.Transform(new Transform2D());
        Assert.Equal(aPoly.Value(4), aCopy.Value(4));
    }

    [Fact]
    public void Freezing_FollowsOcct()
    {
        Geom2d_TrimmedCurve a = Segment(0, 0, 2, 0), b = Segment(2, 0, 2, 2);
        Bisector_Bisec aBis = new();
        aBis.Perform(a, b, new Point2D(2, 0), new Vector2D(1, 0), new Vector2D(0, -1), 1.0, JoinKind.Arc,
                     Tol);
        // The bisector is a working object: Value and ChangeValue hand out the curve itself, unfrozen, and reading it
        // in Bisector_Inter doesn't freeze its basis, which MAT2d re-trims in place.
        Assert.Same(aBis.Value(), aBis.ChangeValue());
        Assert.False(aBis.Value().IsFrozen);
        _ = new Bisector_Inter(aBis, BisectorRecordings.Domain(aBis.Value(), Tol), aBis,
                               BisectorRecordings.Domain(aBis.Value(), Tol), Tol, Tol, true);
        Bisector_BisecAna anAna = (Bisector_BisecAna)aBis.Value().ChangeBasisCurve();
        Assert.False(anAna.IsFrozen);
        anAna.SetTrim(anAna.FirstParameter(), anAna.FirstParameter() + 1.0);
        Assert.Equal(anAna.FirstParameter() + 1.0, anAna.LastParameter());

        // Geom2dCurve hands out the conic frozen; the bisector still re-trims (its trimmed curve is its own).
        Geom2d_Curve aConic = anAna.Geom2dCurve();
        Assert.True(aConic.IsFrozen);
        anAna.SetTrim(anAna.FirstParameter(), anAna.FirstParameter() + 2.0);
        Assert.Equal(anAna.FirstParameter() + 2.0, anAna.LastParameter());

        // A frozen bisector refuses to change.
        Bisector_BisecAna aFrozen = (Bisector_BisecAna)anAna.Copy();
        aFrozen.Freeze();
        Assert.ThrowsAny<ImmutableObjectException>(() => aFrozen.SetTrim(0.0, 1.0));
    }
}
