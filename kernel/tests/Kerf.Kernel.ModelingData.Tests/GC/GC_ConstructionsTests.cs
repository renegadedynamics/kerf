// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Each of GC's constructions against the geometry it's documented to make (GC_Make*.hxx), by known points on hand-worked
/// conics. Unit 11I: GC_PortTests has the failures and freezing, OCCT's GTests little beyond the statuses.
/// </summary>
public class GC_ConstructionsTests
{
    private const double Tol = 1e-12;

    private static readonly Point3D O = new(0, 0, 0);
    private static readonly Point2D O2 = new(0, 0);
    private static readonly Direction3D Z = new(0, 0, 1);
    private static readonly Direction3D X = new(1, 0, 0);
    private static readonly Direction2D X2 = new(1, 0);
    private static readonly Frame3D XY = new(O, Z, X);
    private static readonly Frame2D XY2 = new(O2, X2, new Direction2D(0, 1));
    private static readonly Axis3D ZAxis = new(O, Z);
    private static readonly Axis2D XAxis2 = new(O2, X2);

    private static void Near(Point3D theExpected, Point3D theActual) => Assert.True(theExpected.IsEqual(theActual, Tol), $"expected {theExpected.X} {theExpected.Y} {theExpected.Z}, got {theActual.X} {theActual.Y} {theActual.Z}");
    private static void Near(Point2D theExpected, Point2D theActual) => Assert.True(theExpected.IsEqual(theActual, Tol), $"expected {theExpected.X} {theExpected.Y}, got {theActual.X} {theActual.Y}");
    private static void Near(double theExpected, double theActual) => Assert.Equal(theExpected, theActual, Tol);

    // Circles: radius 5 in the plane z = 0, so u = pi/2 is (0, 5)

    [Fact]
    public void Circle_every_form_gives_the_documented_circle()
    {
        Circle3D aCircle = new(XY, 5);

        Near(new Point3D(0, 5, 0), new MakeCircle(aCircle).Value().Value(M_PI / 2));
        Near(new Point3D(0, 5, 0), new MakeCircle(XY, 5).Value().Value(M_PI / 2));
        Near(5, ((CircleCurve3D)new MakeCircle(ZAxis, 5)).Radius);

        // coaxial: through a point at distance 7 of the axis, whatever its height; 5 + 2 = 7 by distance
        CircleCurve3D aThrough = new MakeCircle(aCircle, new Point3D(7, 0, 9)).Value();
        Near(7, aThrough.Radius);
        Near(0, aThrough.Value(0).Z);
        Near(7, new MakeCircle(aCircle, 2).Value().Radius);
        Near(3, new MakeCircle(aCircle, -2).Value().Radius);
        Assert.Equal(NegativeRadius, new MakeCircle(aCircle, -6).Status());

        // three points on x^2 + y^2 = 25 at height 4
        CircleCurve3D aThree = new MakeCircle(new Point3D(5, 0, 4), new Point3D(3, 4, 4), new Point3D(-5, 0, 4)).Value();
        Near(5, aThree.Radius);
        Near(new Point3D(0, 0, 4), aThree.Circle.Location);
        Assert.Equal(ColinearPoints, new MakeCircle(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0)).Status());

        // a centre and a normal; a centre and a point on the axis: both planes z = 3 about (1, 1, 3)
        Point3D aCentre = new(1, 1, 3);
        CircleCurve3D aNormal = new MakeCircle(aCentre, Z, 2).Value();
        Near(aCentre, aNormal.Circle.Location);
        Near(2, aNormal.Radius);
        Assert.True(aNormal.Circle.Axis.Direction.IsParallel(Z, 1e-12));
        CircleCurve3D anAxis = new MakeCircle(aCentre, new Point3D(1, 1, 8), 2).Value();
        Near(2, anAxis.Radius);
        Assert.True(anAxis.Circle.Axis.Direction.IsParallel(Z, 1e-12));
        Assert.Equal(NegativeRadius, new MakeCircle(aCentre, Z, -2).Status());
        Assert.Equal(NegativeRadius, new MakeCircle(ZAxis, -2).Status());
        Assert.Equal(NegativeRadius, new MakeCircle(aCentre, new Point3D(1, 1, 8), -2).Status());
    }

    [Fact]
    public void Circle2d_every_form_gives_the_documented_circle_and_sense()
    {
        Circle2D aCircle = new(XY2, 5);

        Near(new Point2D(0, 5), ((CircleCurve2D)new MakeCircle2d(aCircle)).Value(M_PI / 2));
        Near(new Point2D(0, 5), new MakeCircle2d(XY2, 5).Value().Value(M_PI / 2));
        // a sense of false turns the circle clockwise, so a quarter on is (0, -5)
        Near(new Point2D(0, 5), new MakeCircle2d(XAxis2, 5).Value().Value(M_PI / 2));
        Near(new Point2D(0, -5), new MakeCircle2d(XAxis2, 5, false).Value().Value(M_PI / 2));
        Near(new Point2D(0, -3), new MakeCircle2d(O2, 3, false).Value().Value(M_PI / 2));
        Near(new Point2D(0, 3), new MakeCircle2d(O2, 3).Value().Value(M_PI / 2));
        Near(4, new MakeCircle2d(O2, new Point2D(0, 4)).Value().Radius);

        Near(7, new MakeCircle2d(aCircle, 2).Value().Radius);
        Near(13, new MakeCircle2d(aCircle, new Point2D(5, 12)).Value().Radius);
        Near(5, new MakeCircle2d(new Point2D(5, 0), new Point2D(3, 4), new Point2D(-5, 0)).Value().Radius);

        Assert.Equal(NegativeRadius, new MakeCircle2d(XY2, -1).Status());
        Assert.Equal(NegativeRadius, new MakeCircle2d(XAxis2, -1).Status());
        Assert.Equal(NegativeRadius, new MakeCircle2d(O2, -1).Status());
        // gce_MakeCirc2d takes the absolute value here, where the 3D gce_MakeCirc reports a negative radius
        Near(1, new MakeCircle2d(aCircle, -6).Value().Radius);
        Assert.False(new MakeCircle2d(O2, new Point2D(1, 1), new Point2D(2, 2)).IsDone);
    }

    // Arcs: start and end points, and which way round from a point of the middle

    [Fact]
    public void ArcOfCircle_from_a_circle_goes_the_asked_way_round()
    {
        Circle3D aCircle = new(XY, 2);
        TrimmedCurve3D aForward = new MakeArcOfCircle(aCircle, 0, M_PI / 2, true).Value();
        Near(new Point3D(2, 0, 0), aForward.StartPoint);
        Near(new Point3D(0, 2, 0), aForward.EndPoint);
        Near(new Point3D(Math.Sqrt(2), Math.Sqrt(2), 0), aForward.Value(M_PI / 4));

        // a sense of false is the same quarter, reversed (Geom_TrimmedCurve::SetTrim trims, then Reverse)
        TrimmedCurve3D aBack = (TrimmedCurve3D)new MakeArcOfCircle(aCircle, 0, M_PI / 2, false);
        Near(new Point3D(0, 2, 0), aBack.StartPoint);
        Near(new Point3D(2, 0, 0), aBack.EndPoint);
        Near(M_PI / 2, aBack.LastParameter - aBack.FirstParameter);

        TrimmedCurve3D aFromPoint = new MakeArcOfCircle(aCircle, new Point3D(0, 2, 0), M_PI, true).Value();
        Near(new Point3D(0, 2, 0), aFromPoint.StartPoint);
        Near(new Point3D(-2, 0, 0), aFromPoint.EndPoint);

        TrimmedCurve3D aPoints = new MakeArcOfCircle(aCircle, new Point3D(0, 2, 0), new Point3D(0, -2, 0), true).Value();
        Near(new Point3D(-2, 0, 0), aPoints.Value((aPoints.FirstParameter + aPoints.LastParameter) / 2));
    }

    [Fact]
    public void ArcOfCircle_through_three_points_and_from_a_tangent()
    {
        Point3D aP1 = new(2, 0, 1), aP2 = new(0, 2, 1), aP3 = new(-2, 0, 1);
        TrimmedCurve3D aThree = new MakeArcOfCircle(aP1, aP2, aP3).Value();
        Near(aP1, aThree.StartPoint);
        Near(aP3, aThree.EndPoint);
        Near(aP2, aThree.Value((aThree.FirstParameter + aThree.LastParameter) / 2));
        Assert.Equal(ColinearPoints, new MakeArcOfCircle(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0)).Status());

        // from (2, 0) heading +y to (-2, 0): the half through (0, 2), not (0, -2)
        TrimmedCurve3D aTangent = new MakeArcOfCircle(new Point3D(2, 0, 1), new Vector3D(0, 1, 0), new Point3D(-2, 0, 1)).Value();
        Near(aP1, aTangent.StartPoint);
        Near(aP3, aTangent.EndPoint);
        Near(aP2, aTangent.Value((aTangent.FirstParameter + aTangent.LastParameter) / 2));
        // the same chord heading -y: the half through (0, -2)
        TrimmedCurve3D aBelow = new MakeArcOfCircle(new Point3D(2, 0, 1), new Vector3D(0, -1, 0), new Point3D(-2, 0, 1)).Value();
        Near(new Point3D(0, -2, 1), aBelow.Value((aBelow.FirstParameter + aBelow.LastParameter) / 2));
    }

    [Fact]
    public void ArcOfCircle2d_every_form()
    {
        Circle2D aCircle = new(XY2, 2);
        TrimmedCurve2D aForward = new MakeArcOfCircle2d(aCircle, 0, M_PI / 2).Value();
        Near(new Point2D(2, 0), aForward.StartPoint);
        Near(new Point2D(0, 2), aForward.EndPoint);

        TrimmedCurve2D aBack = (TrimmedCurve2D)new MakeArcOfCircle2d(aCircle, 0, M_PI / 2, false);
        Near(new Point2D(0, 2), aBack.StartPoint);
        Near(new Point2D(2, 0), aBack.EndPoint);
        Near(M_PI / 2, aBack.LastParameter - aBack.FirstParameter);

        TrimmedCurve2D aFromPoint = new MakeArcOfCircle2d(aCircle, new Point2D(0, 2), M_PI).Value();
        Near(new Point2D(0, 2), aFromPoint.StartPoint);
        Near(new Point2D(-2, 0), aFromPoint.EndPoint);

        TrimmedCurve2D aPoints = new MakeArcOfCircle2d(aCircle, new Point2D(0, 2), new Point2D(0, -2)).Value();
        Near(new Point2D(-2, 0), aPoints.Value((aPoints.FirstParameter + aPoints.LastParameter) / 2));

        TrimmedCurve2D aThree = new MakeArcOfCircle2d(new Point2D(2, 0), new Point2D(0, 2), new Point2D(-2, 0)).Value();
        Near(new Point2D(2, 0), aThree.StartPoint);
        Near(new Point2D(-2, 0), aThree.EndPoint);
        Near(new Point2D(0, 2), aThree.Value((aThree.FirstParameter + aThree.LastParameter) / 2));
        Assert.False(new MakeArcOfCircle2d(new Point2D(0, 0), new Point2D(1, 0), new Point2D(2, 0)).IsDone);
    }

    [Fact]
    public void ArcOfEllipse_from_parameters_and_points()
    {
        // x = 3 cos u, y = 2 sin u
        Ellipse3D anEllipse = new(XY, 3, 2);
        Point3D aRight = new(3, 0, 0), aTop = new(0, 2, 0), aLeft = new(-3, 0, 0);

        TrimmedCurve3D aForward = new MakeArcOfEllipse(anEllipse, 0, M_PI / 2, true).Value();
        Near(aRight, aForward.StartPoint);
        Near(aTop, aForward.EndPoint);
        TrimmedCurve3D aBack = (TrimmedCurve3D)new MakeArcOfEllipse(anEllipse, 0, M_PI / 2, false);
        Near(aTop, aBack.StartPoint);
        Near(aRight, aBack.EndPoint);

        TrimmedCurve3D aFromPoint = new MakeArcOfEllipse(anEllipse, aTop, M_PI, true).Value();
        Near(aTop, aFromPoint.StartPoint);
        Near(aLeft, aFromPoint.EndPoint);
        TrimmedCurve3D aPoints = new MakeArcOfEllipse(anEllipse, aRight, aLeft, true).Value();
        Near(aTop, aPoints.Value((aPoints.FirstParameter + aPoints.LastParameter) / 2));
    }

    [Fact]
    public void ArcOfEllipse2d_from_parameters_and_points()
    {
        Ellipse2D anEllipse = new(XY2, 3, 2);
        Point2D aRight = new(3, 0), aTop = new(0, 2), aLeft = new(-3, 0);

        TrimmedCurve2D aForward = new MakeArcOfEllipse2d(anEllipse, 0, M_PI / 2).Value();
        Near(aRight, aForward.StartPoint);
        Near(aTop, aForward.EndPoint);
        TrimmedCurve2D aBack = (TrimmedCurve2D)new MakeArcOfEllipse2d(anEllipse, 0, M_PI / 2, false);
        Near(aTop, aBack.StartPoint);
        Near(aRight, aBack.EndPoint);

        Near(aLeft, new MakeArcOfEllipse2d(anEllipse, aTop, M_PI).Value().EndPoint);
        TrimmedCurve2D aPoints = new MakeArcOfEllipse2d(anEllipse, aRight, aLeft).Value();
        Near(aTop, aPoints.Value((aPoints.FirstParameter + aPoints.LastParameter) / 2));
    }

    [Fact]
    public void ArcOfHyperbola_from_parameters_and_points()
    {
        // x = 3 cosh u, y = 2 sinh u
        Hyperbola3D aHyperbola = new(XY, 3, 2);
        Point3D aVertex = new(3, 0, 0), anAtOne = new(3 * Math.Cosh(1), 2 * Math.Sinh(1), 0);

        TrimmedCurve3D aForward = new MakeArcOfHyperbola(aHyperbola, 0, 1, true).Value();
        Near(aVertex, aForward.StartPoint);
        Near(anAtOne, aForward.EndPoint);
        Near(anAtOne, ((TrimmedCurve3D)new MakeArcOfHyperbola(aHyperbola, aVertex, 1, true)).EndPoint);
        TrimmedCurve3D aPoints = new MakeArcOfHyperbola(aHyperbola, aVertex, anAtOne, true).Value();
        Near(0, aPoints.FirstParameter);
        Near(1, aPoints.LastParameter);
    }

    [Fact]
    public void ArcOfHyperbola2d_from_parameters_and_points()
    {
        Hyperbola2D aHyperbola = new(XY2, 3, 2);
        Point2D aVertex = new(3, 0), anAtOne = new(3 * Math.Cosh(1), 2 * Math.Sinh(1));

        Near(anAtOne, new MakeArcOfHyperbola2d(aHyperbola, 0, 1).Value().EndPoint);
        Near(anAtOne, ((TrimmedCurve2D)new MakeArcOfHyperbola2d(aHyperbola, aVertex, 1)).EndPoint);
        TrimmedCurve2D aPoints = new MakeArcOfHyperbola2d(aHyperbola, aVertex, anAtOne).Value();
        Near(aVertex, aPoints.StartPoint);
        Near(1, aPoints.LastParameter);
    }

    [Fact]
    public void ArcOfParabola_from_parameters_and_points()
    {
        // focal 2: x = u^2 / 8, y = u
        Parabola3D aParabola = new(XY, 2);
        Point3D anApex = new(0, 0, 0), anAtFour = new(2, 4, 0);

        TrimmedCurve3D aForward = new MakeArcOfParabola(aParabola, 0, 4, true).Value();
        Near(anApex, aForward.StartPoint);
        Near(anAtFour, aForward.EndPoint);
        Near(anAtFour, ((TrimmedCurve3D)new MakeArcOfParabola(aParabola, anApex, 4, true)).EndPoint);
        TrimmedCurve3D aPoints = new MakeArcOfParabola(aParabola, anApex, anAtFour, true).Value();
        Near(0, aPoints.FirstParameter);
        Near(4, aPoints.LastParameter);
    }

    [Fact]
    public void ArcOfParabola2d_from_parameters_and_points()
    {
        Parabola2D aParabola = new(XY2, 2);
        Point2D anApex = new(0, 0), anAtFour = new(2, 4);

        Near(anAtFour, new MakeArcOfParabola2d(aParabola, 0, 4).Value().EndPoint);
        Near(anAtFour, ((TrimmedCurve2D)new MakeArcOfParabola2d(aParabola, anApex, 4)).EndPoint);
        TrimmedCurve2D aPoints = new MakeArcOfParabola2d(aParabola, anApex, anAtFour).Value();
        Near(anApex, aPoints.StartPoint);
        Near(4, aPoints.LastParameter);
    }

    // Conics

    [Fact]
    public void Ellipse_every_form()
    {
        Near(new Point3D(0, 2, 0), new MakeEllipse(new Ellipse3D(XY, 3, 2)).Value().Value(M_PI / 2));
        Near(2, new MakeEllipse(XY, 3, 2).Value().MinorRadius);

        // S1 on the major axis, S2 the distance of the minor radius from it
        EllipseCurve3D aThree = (EllipseCurve3D)new MakeEllipse(new Point3D(5, 1, 0), new Point3D(1, 4, 0), new Point3D(1, 1, 0));
        Near(4, aThree.MajorRadius);
        Near(3, aThree.MinorRadius);
        Near(new Point3D(5, 1, 0), aThree.Value(0));
        Assert.Equal(InvertAxis, new MakeEllipse(new Point3D(2, 0, 0), new Point3D(0, 3, 0), O).Status());
    }

    [Fact]
    public void Ellipse2d_every_form()
    {
        Near(new Point2D(0, 2), new MakeEllipse2d(new Ellipse2D(XY2, 3, 2)).Value().Value(M_PI / 2));
        Near(new Point2D(0, 2), new MakeEllipse2d(XY2, 3, 2).Value().Value(M_PI / 2));
        Near(new Point2D(0, 2), new MakeEllipse2d(XAxis2, 3, 2).Value().Value(M_PI / 2));
        Near(new Point2D(0, -2), ((EllipseCurve2D)new MakeEllipse2d(XAxis2, 3, 2, false)).Value(M_PI / 2));

        EllipseCurve2D aThree = new MakeEllipse2d(new Point2D(5, 1), new Point2D(1, 4), new Point2D(1, 1)).Value();
        Near(4, aThree.MajorRadius);
        Near(3, aThree.MinorRadius);

        Assert.Equal(InvertRadius, new MakeEllipse2d(XY2, 2, 3).Status());
        Assert.Equal(NegativeRadius, new MakeEllipse2d(XY2, -3, -4).Status());
        // gce_MakeElips2d doesn't check a negative minor radius itself: gp_Elips2d refuses it
        EngineAssert.Throws<ConstructionErrorException>(() => new MakeEllipse2d(XY2, 3, -2));
        Assert.Equal(InvertRadius, new MakeEllipse2d(XAxis2, 2, 3).Status());
        Assert.Equal(NegativeRadius, new MakeEllipse2d(XAxis2, -3, -4).Status());
    }

    [Fact]
    public void Hyperbola_every_form()
    {
        Point3D anAtOne = new(3 * Math.Cosh(1), 2 * Math.Sinh(1), 0);
        Near(anAtOne, new MakeHyperbola(new Hyperbola3D(XY, 3, 2)).Value().Value(1));
        Near(anAtOne, ((HyperbolaCurve3D)new MakeHyperbola(XY, 3, 2)).Value(1));

        HyperbolaCurve3D aThree = new MakeHyperbola(new Point3D(5, 1, 0), new Point3D(1, 4, 0), new Point3D(1, 1, 0)).Value();
        Near(4, aThree.MajorRadius);
        Near(3, aThree.MinorRadius);
    }

    [Fact]
    public void Hyperbola2d_every_form()
    {
        Point2D anAtOne = new(3 * Math.Cosh(1), 2 * Math.Sinh(1));
        Near(anAtOne, new MakeHyperbola2d(new Hyperbola2D(XY2, 3, 2)).Value().Value(1));
        Near(anAtOne, new MakeHyperbola2d(XY2, 3, 2).Value().Value(1));
        Near(anAtOne, new MakeHyperbola2d(XAxis2, 3, 2, true).Value().Value(1));
        Near(new Point2D(anAtOne.X, -anAtOne.Y), ((HyperbolaCurve2D)new MakeHyperbola2d(XAxis2, 3, 2, false)).Value(1));

        HyperbolaCurve2D aThree = new MakeHyperbola2d(new Point2D(5, 1), new Point2D(1, 4), new Point2D(1, 1)).Value();
        Near(4, aThree.MajorRadius);
        Near(3, aThree.MinorRadius);

        Assert.Equal(NegativeRadius, new MakeHyperbola2d(XY2, -3, 2).Status());
        Assert.Equal(NegativeRadius, new MakeHyperbola2d(XAxis2, 3, -2, true).Status());
    }

    [Fact]
    public void Parabola2d_every_form()
    {
        // focal 2: x = u^2 / 8, y = u; so u = 4 is (2, 4)
        Point2D anAtFour = new(2, 4);
        Near(anAtFour, new MakeParabola2d(new Parabola2D(XY2, 2)).Value().Value(4));
        Near(anAtFour, new MakeParabola2d(XY2, 2).Value().Value(4));
        Near(anAtFour, ((ParabolaCurve2D)new MakeParabola2d(XAxis2, 2, true)).Value(4));
        Near(new Point2D(2, -4), new MakeParabola2d(XAxis2, 2, false).Value().Value(4));

        // a directrix and a focus: the focal length is half their distance, the apex midway
        ParabolaCurve2D aDirectrix = new MakeParabola2d(new Axis2D(new Point2D(-2, 0), new Direction2D(0, 1)), new Point2D(2, 0)).Value();
        Near(2, aDirectrix.Focal);
        Near(O2, aDirectrix.Value(0));

        // a focus and an apex
        ParabolaCurve2D aFocus = new MakeParabola2d(new Point2D(2, 0), O2).Value();
        Near(2, aFocus.Focal);
        Near(O2, aFocus.Value(0));

        Assert.Equal(NullFocusLength, new MakeParabola2d(XY2, -2).Status());
    }

    // Lines and segments

    [Fact]
    public void Line_every_form()
    {
        Near(new Point3D(0, 0, 2), new MakeLine(ZAxis).Value().Value(2));
        Line3D aLine = new(O, Z);
        Near(new Point3D(0, 0, 2), ((LineCurve3D)new MakeLine(aLine)).Value(2));
        Near(new Point3D(1, 2, 2), new MakeLine(aLine, new Point3D(1, 2, 0)).Value().Value(2));
        Near(new Point3D(1, 2, 2), new MakeLine(new Point3D(1, 2, 0), Z).Value().Value(2));
        Near(new Point3D(0, 4, 0), new MakeLine(O, new Point3D(0, 7, 0)).Value().Value(4));
    }

    [Fact]
    public void Line2d_every_form_with_a_positive_distance_to_the_left()
    {
        Near(new Point2D(2, 0), new MakeLine2d(XAxis2).Value().Value(2));
        Line2D aLine = new(O2, X2);
        Near(new Point2D(2, 0), ((LineCurve2D)new MakeLine2d(aLine)).Value(2));
        Near(new Point2D(2, 3), new MakeLine2d(aLine, new Point2D(0, 3)).Value().Value(2));
        // gce_MakeLin2d.cxx adds Dist * (-dy, dx), to the left, though its header says the right (occt-issues.md)
        Near(new Point2D(2, 3), new MakeLine2d(aLine, 3).Value().Value(2));
        Near(new Point2D(2, -3), new MakeLine2d(aLine, -3).Value().Value(2));
        Near(new Point2D(3, 1), new MakeLine2d(new Point2D(1, 1), X2).Value().Value(2));
        Near(new Point2D(0, 4), new MakeLine2d(O2, new Point2D(0, 7)).Value().Value(4));
        Assert.Equal(ConfusedPoints, new MakeLine2d(O2, O2).Status());
    }

    [Fact]
    public void Segment_every_form()
    {
        Line3D aLine = new(O, X);
        TrimmedCurve3D aPointU = new MakeSegment(aLine, new Point3D(2, 0, 0), 5).Value();
        Near(new Point3D(2, 0, 0), aPointU.StartPoint);
        Near(new Point3D(5, 0, 0), aPointU.EndPoint);
        TrimmedCurve3D aPoints = new MakeSegment(aLine, new Point3D(1, 0, 0), new Point3D(4, 0, 0)).Value();
        Near(3, aPoints.LastParameter - aPoints.FirstParameter);
        TrimmedCurve3D aParameters = (TrimmedCurve3D)new MakeSegment(aLine, -1, 1);
        Near(new Point3D(-1, 0, 0), aParameters.StartPoint);
        Near(new Point3D(1, 0, 0), aParameters.EndPoint);
    }

    [Fact]
    public void Segment2d_every_form()
    {
        Line2D aLine = new(O2, X2);
        Near(new Point2D(5, 0), new MakeSegment2d(aLine, new Point2D(2, 0), 5).Value().EndPoint);
        Near(new Point2D(4, 0), new MakeSegment2d(aLine, new Point2D(1, 0), new Point2D(4, 0)).Value().EndPoint);
        TrimmedCurve2D aParameters = (TrimmedCurve2D)new MakeSegment2d(aLine, -1, 1);
        Near(new Point2D(-1, 0), aParameters.StartPoint);
        Near(new Point2D(1, 0), aParameters.EndPoint);
        // from P1 along V to the parameter of P2's projection on that line
        TrimmedCurve2D aDirection = new MakeSegment2d(new Point2D(1, 1), X2, new Point2D(4, 9)).Value();
        Near(new Point2D(1, 1), aDirection.StartPoint);
        Near(new Point2D(4, 1), aDirection.EndPoint);
    }

    // Planes and surfaces

    [Fact]
    public void Plane_every_form()
    {
        // ax + by + cz + d = 0: z = 3
        PlaneSurface anEquation = new MakePlane(0, 0, 1, -3).Value();
        Near(3, anEquation.Plane.Location.Z);
        Assert.True(anEquation.Plane.Axis.Direction.IsParallel(Z, 1e-12));

        Plane3D aPlane = new(new CoordinateSystem(XY));
        Near(0, ((PlaneSurface)new MakePlane(aPlane)).Plane.Location.Z);
        Near(2, new MakePlane(aPlane, new Point3D(5, 5, 2)).Value().Plane.DistanceTo(O));
        // a signed distance along the plane's normal
        Near(new Point3D(0, 0, 4).Z, new MakePlane(aPlane, 4).Value().Plane.Location.Z);
        Near(-4, new MakePlane(aPlane, -4).Value().Plane.Location.Z);
        Near(new Point3D(1, 2, 3), new MakePlane(new Point3D(1, 2, 3), Z).Value().Plane.Location);
        Near(new Point3D(0, 0, 0), new MakePlane(ZAxis).Value().Plane.Location);

        // three points of z = 7, anticlockwise from above
        Plane3D aThree = new MakePlane(new Point3D(0, 0, 7), new Point3D(1, 0, 7), new Point3D(0, 1, 7)).Value().Plane;
        Near(7, aThree.DistanceTo(O));
        Assert.True(aThree.Axis.Direction.IsEqual(Z, 1e-12));
        Assert.Equal(ColinearPoints, new MakePlane(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0)).Status());
    }

    [Fact]
    public void Cylinder_every_form()
    {
        Near(2, ((CylindricalSurface)new MakeCylindricalSurface(new Cylinder3D(new CoordinateSystem(XY), 2))).Radius);
        Near(2, new MakeCylindricalSurface(XY, 2).Value().Radius);
        Near(2, new MakeCylindricalSurface(ZAxis, 2).Value().Radius);
        Near(2, new MakeCylindricalSurface(new Circle3D(XY, 2)).Value().Radius);
        // the axis P1P2, through P3
        CylindricalSurface aThree = new MakeCylindricalSurface(O, new Point3D(0, 0, 5), new Point3D(3, 4, 9)).Value();
        Near(5, aThree.Radius);
        Assert.Equal(NegativeRadius, new MakeCylindricalSurface(ZAxis, -2).Status());
        Assert.Equal(ConfusedPoints, new MakeCylindricalSurface(O, O, new Point3D(3, 4, 9)).Status());
    }

    [Fact]
    public void Cone_every_form()
    {
        Cone3D aCone = new(new CoordinateSystem(XY), 0.5, 2);
        Near(0.5, new MakeConicalSurface(aCone).Value().SemiAngle);
        Near(0.5, ((ConicalSurface)new MakeConicalSurface(XY, 0.5, 2)).SemiAngle);

        // radius 3 at P1 to radius 1 at P2, 4 along: the semi-angle closes towards P2
        ConicalSurface aRadii = new MakeConicalSurface(O, new Point3D(0, 0, 4), 3, 1).Value();
        Near(3, aRadii.RefRadius);
        Near(Math.Atan(-2.0 / 4.0), aRadii.SemiAngle);

        // the same cone by the points at those radii
        ConicalSurface aPoints = new MakeConicalSurface(O, new Point3D(0, 0, 4), new Point3D(3, 0, 0), new Point3D(0, 1, 4)).Value();
        Near(3, aPoints.RefRadius);
        Near(Math.Abs(Math.Atan(-2.0 / 4.0)), Math.Abs(aPoints.SemiAngle));
        Near(1, aPoints.Value(0, Math.Sqrt(20)).DistanceTo(new Point3D(0, 0, 4)));

        Assert.False(new MakeConicalSurface(O, O, 3, 1).IsDone);
        Assert.False(new MakeConicalSurface(O, new Point3D(0, 0, 4), 2, 2).IsDone);
    }

    [Fact]
    public void Trimmed_cylinder_and_cone_have_the_bounds_of_their_heights()
    {
        void Bounds(TrimmedSurface theSurface, double theV2)
        {
            theSurface.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
            Near(0, aU1);
            Near(2 * M_PI, aU2);
            Near(0, aV1);
            Near(theV2, aV2);
        }

        // radius 5 from a point off the axis, 3 along it
        TrimmedSurface aThree = new MakeTrimmedCylinder(O, new Point3D(0, 0, 3), new Point3D(3, 4, 0)).Value();
        Bounds(aThree, 3);
        Near(new Point3D(5, 0, 3).Z, aThree.Value(0, 3).Z);
        Near(5, new Point3D(aThree.Value(0, 3).X, aThree.Value(0, 3).Y, 0).DistanceTo(O));
        Bounds((TrimmedSurface)new MakeTrimmedCylinder(new Circle3D(XY, 2), 6), 6);

        TrimmedSurface aCone = new MakeTrimmedCone(O, new Point3D(0, 0, 4), new Point3D(3, 0, 0), new Point3D(0, 1, 4)).Value();
        aCone.ParameterBounds(out _, out _, out _, out double aConeV2);
        Near(Math.Sqrt(4.0 * 4.0 + 2.0 * 2.0), aConeV2);
    }

    // Transformations, by what they do to (1, 2, 3)

    [Fact]
    public void Mirrors_every_form()
    {
        Point3D aP = new(1, 2, 3);
        Point3D Mirror(MakeMirror theMaker) => aP.Transformed(((GeometryTransformation)theMaker).Trsf());

        Near(new Point3D(-1, -2, -3), Mirror(new MakeMirror(O)));
        // about the z axis, in each of its forms
        Near(new Point3D(-1, -2, 3), Mirror(new MakeMirror(ZAxis)));
        Near(new Point3D(-1, -2, 3), Mirror(new MakeMirror(new Line3D(O, Z))));
        Near(new Point3D(-1, -2, 3), Mirror(new MakeMirror(O, Z)));
        // about the plane z = 0
        Near(new Point3D(1, 2, -3), Mirror(new MakeMirror(new Plane3D(new CoordinateSystem(XY)))));
        Near(new Point3D(1, 2, -3), Mirror(new MakeMirror(XY)));

        Point2D aP2 = new(1, 2);
        Point2D Mirror2d(MakeMirror2d theMaker) => aP2.Transformed(((Transformation2)theMaker).Trsf2d());
        Near(new Point2D(-1, -2), Mirror2d(new MakeMirror2d(O2)));
        Near(new Point2D(1, -2), Mirror2d(new MakeMirror2d(XAxis2)));
        Near(new Point2D(1, -2), Mirror2d(new MakeMirror2d(new Line2D(O2, X2))));
        Near(new Point2D(1, -2), Mirror2d(new MakeMirror2d(O2, X2)));
    }

    [Fact]
    public void Rotations_scales_and_translations_every_form()
    {
        Point3D aP = new(1, 2, 3);
        // a quarter turn about z: (x, y) to (-y, x)
        Point3D aTurned = new(-2, 1, 3);
        Near(aTurned, aP.Transformed(new MakeRotation(ZAxis, M_PI / 2).Value().Trsf()));
        Near(aTurned, aP.Transformed(((GeometryTransformation)new MakeRotation(new Line3D(O, Z), M_PI / 2)).Trsf()));
        Near(aTurned, aP.Transformed(new MakeRotation(O, Z, M_PI / 2).Value().Trsf()));
        Near(new Point2D(-2, 1), new Point2D(1, 2).Transformed(((Transformation2)new MakeRotation2d(O2, M_PI / 2)).Trsf2d()));

        Near(new Point3D(2, 4, 6), aP.Transformed(((GeometryTransformation)new MakeScale(O, 2)).Trsf()));
        Near(new Point2D(2, 4), new Point2D(1, 2).Transformed(((Transformation2)new MakeScale2d(O2, 2)).Trsf2d()));
        Near(new Point2D(2, 4), new Point2D(1, 2).Transformed(new MakeScale2d(O2, 2).Value().Trsf2d()));

        Near(new Point3D(2, 3, 4), aP.Transformed(((GeometryTransformation)new MakeTranslation(new Vector3D(1, 1, 1))).Trsf()));
        Near(new Point2D(2, 3), new Point2D(1, 2).Transformed(((Transformation2)new MakeTranslation2d(new Vector2D(1, 1))).Trsf2d()));
        Near(new Point2D(2, 3), new Point2D(1, 2).Transformed(new MakeTranslation2d(O2, new Point2D(1, 1)).Value().Trsf2d()));
    }

    // What every Value() does once its construction has failed

    [Fact]
    public void A_failed_construction_has_no_value_in_either_form()
    {
        EngineAssert.Throws<NotDoneException>(() => new MakeCircle2d(XY2, -1).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeEllipse2d(XY2, 2, 3).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeHyperbola2d(XY2, -3, 2).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeParabola2d(XY2, -2).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeLine2d(O2, O2).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeLine(O, O).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakePlane(0, 0, 0, 1).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeCylindricalSurface(ZAxis, -2).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeConicalSurface(XY, 0.5, -2).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeHyperbola(XY, -3, 2).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeEllipse(XY, 2, 3).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeTrimmedCylinder(ZAxis, -2, 3).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeTrimmedCone(O, O, 3, 1).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeArcOfCircle(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0)).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeArcOfCircle2d(new Point2D(0, 0), new Point2D(1, 0), new Point2D(2, 0)).Value());
    }
}
