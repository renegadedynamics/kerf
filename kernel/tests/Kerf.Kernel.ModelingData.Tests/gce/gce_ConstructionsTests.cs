// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// gce's constructions that GC doesn't pass through, against the geometry and statuses gce_Make*.hxx document (unit 11I):
/// the value on success, the status on each failure.
/// </summary>
public class gce_ConstructionsTests
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

    [Fact]
    public void Directions_every_form()
    {
        Assert.Equal(Z, new MakeDir(new Vector3D(0, 0, 3)).Value());
        Assert.Equal(Z, new MakeDir(new Xyz(0, 0, 3)).Value());
        Assert.Equal(Z, new MakeDir(0, 0, 3).Value());
        Assert.Equal(Z, new MakeDir(O, new Point3D(0, 0, 3)).Value());
        Assert.Equal(NullVector, new MakeDir(new Vector3D(0, 0, 0)).Status());
        Assert.Equal(NullVector, new MakeDir(new Xyz(0, 0, 0)).Status());
        Assert.Equal(ConfusedPoints, new MakeDir(O, O).Status());

        Assert.Equal(X2, new MakeDir2d(new Vector2D(3, 0)).Value());
        Assert.Equal(X2, new MakeDir2d(new Xy(3, 0)).Value());
        Assert.Equal(X2, new MakeDir2d(3, 0).Value());
        Assert.Equal(X2, new MakeDir2d(O2, new Point2D(3, 0)).Value());
        Assert.Equal(NullVector, new MakeDir2d(new Vector2D(0, 0)).Status());
        Assert.Equal(NullVector, new MakeDir2d(new Xy(0, 0)).Status());
        Assert.Equal(NullVector, new MakeDir2d(0, 0).Status());
        EngineAssert.Throws<NotDoneException>(() => new MakeDir(0, 0, 0).Value());
        EngineAssert.Throws<NotDoneException>(() => new MakeDir2d(0, 0).Value());
    }

    [Fact]
    public void Lines_every_form()
    {
        Assert.Equal(Z, new MakeLin(ZAxis).Value().Direction);
        Assert.Equal(new Point3D(1, 2, 3), new MakeLin(new Point3D(1, 2, 3), Z).Value().Location);
        Line3D aThrough = new MakeLin(new Line3D(O, Z), new Point3D(1, 2, 3)).Value();
        Assert.Equal(Z, aThrough.Direction);
        Assert.Equal(new Point3D(1, 2, 3), aThrough.Location);

        Assert.Equal(X2, new MakeLin2d(XAxis2).Value().Direction);
        Assert.Equal(new Point2D(1, 2), new MakeLin2d(new Point2D(1, 2), X2).Value().Location);
        Assert.Equal(2.0, new MakeLin2d(new Line2D(O2, X2), new Point2D(5, 2)).Value().DistanceTo(O2), Tol);
        // 3x + 4y - 5 = 0 is at distance 1 from the origin
        Assert.Equal(1.0, new MakeLin2d(3, 4, -5).Value().DistanceTo(O2), Tol);
    }

    [Fact]
    public void Circles_every_form()
    {
        Assert.Equal(2.0, new MakeCirc(XY, 2).Value().Radius);
        Assert.Equal(NegativeRadius, new MakeCirc(XY, -2).Status());
        Assert.Equal(2.0, new MakeCirc(ZAxis, 2).Value().Radius);
        Assert.Equal(NegativeRadius, new MakeCirc(ZAxis, -2).Status());

        // in a plane: its normal is the circle's
        Plane3D aPlane = new(new CoordinateSystem(XY));
        Circle3D anInPlane = new MakeCirc(new Point3D(1, 1, 0), aPlane, 2).Value();
        Assert.Equal(new Point3D(1, 1, 0), anInPlane.Location);
        Assert.True(anInPlane.Axis.Direction.IsParallel(Z, Tol));
        Assert.Equal(NegativeRadius, new MakeCirc(O, aPlane, -2).Status());

        Assert.Equal(NullAxis, new MakeCirc(O, O, 2).Status());
        Assert.Equal(3.0, new MakeCirc(new Circle3D(XY, 2), new Point3D(3, 0, 0)).Value().Radius, Tol);

        Assert.Equal(2.0, new MakeCirc2d(XAxis2, 2).Value().Radius);
        Assert.Equal(NegativeRadius, new MakeCirc2d(XAxis2, -2).Status());
        Assert.Equal(2.0, new MakeCirc2d(XY2, 2).Value().Radius);
        Assert.Equal(NegativeRadius, new MakeCirc2d(XY2, -2).Status());
        Assert.Equal(5.0, new MakeCirc2d(O2, new Point2D(3, 4)).Value().Radius, Tol);
        Assert.Equal(NegativeRadius, new MakeCirc2d(O2, -2).Status());
        // three points on a circle of radius 5 about (1, 1)
        Circle2D aThree = new MakeCirc2d(new Point2D(6, 1), new Point2D(4, 5), new Point2D(-4, 1)).Value();
        Assert.Equal(5.0, aThree.Radius, 1e-9);
        Assert.True(aThree.Location.IsEqual(new Point2D(1, 1), 1e-9));
        // three coincident points are a point circle; two coincident points leave no circle
        Assert.Equal(0.0, new MakeCirc2d(O2, O2, O2).Value().Radius);
        Assert.False(new MakeCirc2d(O2, O2, new Point2D(1, 0)).IsDone);
        EngineAssert.Throws<NotDoneException>(() => new MakeCirc2d(XY2, -2).Value());
    }

    [Fact]
    public void Cones_every_form()
    {
        Assert.Equal(0.5, new MakeCone(XY, 0.5, 2).Value().SemiAngle);
        Assert.Equal(NegativeRadius, new MakeCone(XY, 0.5, -2).Status());
        Assert.Equal(BadAngle, new MakeCone(XY, 0, 2).Status());
        Assert.Equal(BadAngle, new MakeCone(XY, M_PI / 2, 2).Status());

        // the axis P1P2, P3 the base radius, P4 another section's
        Cone3D aPoints = new MakeCone(O, new Point3D(0, 0, 4), new Point3D(3, 0, 0), new Point3D(1, 0, 4)).Value();
        Assert.Equal(3.0, aPoints.RefRadius, Tol);
        Assert.Equal(-Math.Atan(0.5), aPoints.SemiAngle, Tol);
        Assert.Equal(ConfusedPoints, new MakeCone(O, O, new Point3D(3, 0, 0), new Point3D(1, 0, 4)).Status());
        Assert.Equal(ConfusedPoints, new MakeCone(O, new Point3D(0, 0, 4), new Point3D(3, 0, 0), new Point3D(3, 0, 0)).Status());
        // P3 and P4 in one section: no angle
        Assert.Equal(NullAngle, new MakeCone(O, new Point3D(0, 0, 4), new Point3D(3, 0, 1), new Point3D(1, 0, 1)).Status());

        // by an axis and two points, as an axis, or as a line
        Cone3D anAxis = new MakeCone(ZAxis, new Point3D(3, 0, 0), new Point3D(1, 0, 4)).Value();
        Assert.Equal(Math.Atan(0.5), Math.Abs(anAxis.SemiAngle), Tol);
        Cone3D aLine = new MakeCone(new Line3D(O, Z), new Point3D(3, 0, 0), new Point3D(1, 0, 4)).Value();
        Assert.Equal(Math.Atan(0.5), Math.Abs(aLine.SemiAngle), Tol);

        // two radii: 3 at P1, 1 at P2
        Cone3D aRadii = new MakeCone(O, new Point3D(0, 0, 4), 3, 1).Value();
        Assert.Equal(3.0, aRadii.RefRadius, Tol);
        Assert.Equal(-Math.Atan(0.5), aRadii.SemiAngle, Tol);
        Cone3D anOpening = new MakeCone(O, new Point3D(0, 0, 4), 1, 3).Value();
        Assert.Equal(Math.Atan(0.5), anOpening.SemiAngle, Tol);
        Assert.Equal(NullAxis, new MakeCone(O, O, 3, 1).Status());
        Assert.Equal(NegativeRadius, new MakeCone(O, new Point3D(0, 0, 4), -3, 1).Status());
        Assert.Equal(NegativeRadius, new MakeCone(O, new Point3D(0, 0, 4), 3, -1).Status());
        Assert.False(new MakeCone(O, new Point3D(0, 0, 4), 2, 2).IsDone);

        // coaxial, through a point and at a distance: the angle stays, the radius moves
        Cone3D aCone = new(new CoordinateSystem(XY), 0.5, 2);
        Assert.Equal(0.5, new MakeCone(aCone, new Point3D(5, 0, 0)).Value().SemiAngle, Tol);
        EngineAssert.Throws<NotDoneException>(() => new MakeCone(XY, 0.5, -2).Value());
        // the distance is normal to the surface: the radius moves by 1 / cos(0.5)
        Assert.Equal(2.0 + 1.0 / Math.Cos(0.5), new MakeCone(aCone, 1).Value().RefRadius, Tol);
        Assert.Equal(NegativeRadius, new MakeCone(aCone, -3).Status());
    }

    [Fact]
    public void Cylinders_every_form()
    {
        Assert.Equal(2.0, new MakeCylinder(XY, 2).Value().Radius);
        Assert.Equal(NegativeRadius, new MakeCylinder(XY, -2).Status());
        Assert.Equal(2.0, new MakeCylinder(ZAxis, 2).Value().Radius);
        Assert.Equal(NegativeRadius, new MakeCylinder(ZAxis, -2).Status());
        Assert.Equal(2.0, new MakeCylinder(new Circle3D(XY, 2)).Value().Radius);

        Cylinder3D aCylinder = new(new CoordinateSystem(XY), 2);
        Assert.Equal(5.0, new MakeCylinder(aCylinder, new Point3D(3, 4, 9)).Value().Radius, Tol);
        Assert.Equal(3.0, new MakeCylinder(aCylinder, 1).Value().Radius, Tol);
        Assert.Equal(1.0, new MakeCylinder(aCylinder, -1).Value().Radius, Tol);
        Assert.Equal(NegativeRadius, new MakeCylinder(aCylinder, -3).Status());

        Assert.Equal(5.0, new MakeCylinder(O, new Point3D(0, 0, 1), new Point3D(3, 4, 7)).Value().Radius, Tol);
        Assert.Equal(ConfusedPoints, new MakeCylinder(O, O, new Point3D(3, 4, 7)).Status());
        EngineAssert.Throws<NotDoneException>(() => new MakeCylinder(XY, -2).Value());
    }

    [Fact]
    public void Planes_every_form()
    {
        Assert.Equal(new Point3D(0, 0, 0), new MakePln(XY).Value().Location);
        Assert.Equal(Z, new MakePln(O, Z).Value().Axis.Direction);
        Assert.Equal(Z, new MakePln(ZAxis).Value().Axis.Direction);
        Assert.Equal(4.0, new MakePln(new Plane3D(new CoordinateSystem(XY)), 4).Value().DistanceTo(O), Tol);
        Assert.Equal(new Point3D(1, 1, 1), new MakePln(new Plane3D(new CoordinateSystem(XY)), new Point3D(1, 1, 1)).Value().Location);

        // two points: through the first, normal to the line to the second
        Plane3D aTwo = new MakePln(new Point3D(0, 0, 2), new Point3D(0, 0, 9)).Value();
        Assert.Equal(2.0, aTwo.DistanceTo(O), Tol);
        Assert.Equal(ConfusedPoints, new MakePln(O, O).Status());

        // 2x + 2y + z - 3 = 0 is at distance 1 from the origin
        Assert.Equal(1.0, new MakePln(2, 2, 1, -3).Value().DistanceTo(O), Tol);
        Assert.Equal(BadEquation, new MakePln(0, 0, 0, -3).Status());
        Assert.Equal(ColinearPoints, new MakePln(O, O, new Point3D(1, 0, 0)).Status());
        Assert.Equal(ColinearPoints, new MakePln(O, new Point3D(1, 0, 0), new Point3D(2, 0, 0)).Status());
        EngineAssert.Throws<NotDoneException>(() => new MakePln(O, O).Value());
    }

    [Fact]
    public void Ellipses_every_form()
    {
        Assert.Equal(3.0, new MakeElips(XY, 3, 2).Value().MajorRadius);
        Assert.Equal(InvertRadius, new MakeElips(XY, 2, 3).Status());

        Ellipse3D aThree = new MakeElips(new Point3D(4, 0, 0), new Point3D(0, 3, 0), O).Value();
        Assert.Equal(4.0, aThree.MajorRadius, Tol);
        Assert.Equal(3.0, aThree.MinorRadius, Tol);
        Assert.Equal(NullAxis, new MakeElips(O, new Point3D(0, 3, 0), O).Status());
        Assert.Equal(InvertAxis, new MakeElips(new Point3D(2, 0, 0), new Point3D(0, 3, 0), O).Status());
        EngineAssert.Throws<NotDoneException>(() => new MakeElips(XY, 2, 3).Value());

        Assert.Equal(2.0, new MakeElips2d(XY2, 3, 2).Value().MinorRadius);
        Assert.Equal(NegativeRadius, new MakeElips2d(XY2, -3, -4).Status());
        Assert.Equal(2.0, new MakeElips2d(XAxis2, 3, 2).Value().MinorRadius);
        Assert.Equal(InvertRadius, new MakeElips2d(XAxis2, 2, 3).Status());
        Assert.Equal(4.0, new MakeElips2d(new Point2D(4, 0), new Point2D(0, 3), O2).Value().MajorRadius, Tol);
        Assert.Equal(InvertAxis, new MakeElips2d(new Point2D(2, 0), new Point2D(0, 3), O2).Status());
    }

    [Fact]
    public void Hyperbolas_every_form()
    {
        Assert.Equal(3.0, new MakeHypr(XY, 3, 2).Value().MajorRadius);
        Assert.Equal(NegativeRadius, new MakeHypr(XY, -3, 2).Status());
        Assert.Equal(NegativeRadius, new MakeHypr(XY, 3, -2).Status());

        Hyperbola3D aThree = new MakeHypr(new Point3D(4, 0, 0), new Point3D(1, 3, 0), O).Value();
        Assert.Equal(4.0, aThree.MajorRadius, Tol);
        Assert.Equal(3.0, aThree.MinorRadius, Tol);
        Assert.Equal(ConfusedPoints, new MakeHypr(O, new Point3D(1, 3, 0), O).Status());
        Assert.Equal(ConfusedPoints, new MakeHypr(new Point3D(4, 0, 0), new Point3D(4, 0, 0), O).Status());
        // S2 on the major axis: there's no minor radius
        Assert.Equal(ColinearPoints, new MakeHypr(new Point3D(4, 0, 0), new Point3D(2, 0, 0), O).Status());
        EngineAssert.Throws<NotDoneException>(() => new MakeHypr(XY, -3, 2).Value());

        Assert.Equal(2.0, new MakeHypr2d(XY2, 3, 2).Value().MinorRadius);
        Assert.Equal(NegativeRadius, new MakeHypr2d(XY2, -3, 2).Status());
        Assert.Equal(2.0, new MakeHypr2d(XAxis2, 3, 2, true).Value().MinorRadius);
        Assert.Equal(NegativeRadius, new MakeHypr2d(XAxis2, 3, -2, true).Status());
        Assert.Equal(4.0, new MakeHypr2d(new Point2D(4, 0), new Point2D(1, 3), O2).Value().MajorRadius, Tol);
    }

    [Fact]
    public void Parabolas_every_form()
    {
        Assert.Equal(2.0, new MakeParab(XY, 2).Value().Focal);
        Assert.Equal(NullFocusLength, new MakeParab(XY, -2).Status());
        // a directrix and a focus: half the distance between them
        Parabola3D aFocus = new MakeParab(new Axis3D(new Point3D(-2, 0, 0), new Direction3D(0, 1, 0)), new Point3D(2, 0, 0)).Value();
        Assert.Equal(2.0, aFocus.Focal, Tol);
        Assert.True(aFocus.Location.IsEqual(O, Tol));

        Assert.Equal(2.0, new MakeParab2d(XY2, 2).Value().Focal);
        Assert.Equal(NullFocusLength, new MakeParab2d(XY2, -2).Status());
        Assert.Equal(2.0, new MakeParab2d(XAxis2, 2).Value().Focal);
        Assert.Equal(NullFocusLength, new MakeParab2d(XAxis2, -2).Status());
        Assert.Equal(2.0, new MakeParab2d(new Axis2D(new Point2D(-2, 0), new Direction2D(0, 1)), new Point2D(2, 0)).Value().Focal, Tol);
        // a focus and the centre
        Assert.Equal(2.0, new MakeParab2d(new Point2D(2, 0), O2).Value().Focal, Tol);
        EngineAssert.Throws<NotDoneException>(() => new MakeParab(XY, -2).Value());
    }

    [Fact]
    public void Transformations_every_form_by_what_they_do()
    {
        Point3D aP = new(1, 2, 3);
        Point2D aP2 = new(1, 2);
        void Is(Point3D theExpected, Transform3D theTrsf) => Assert.True(theExpected.IsEqual(aP.Transformed(theTrsf), Tol));
        void Is2(Point2D theExpected, Transform2D theTrsf) => Assert.True(theExpected.IsEqual(aP2.Transformed(theTrsf), Tol));

        Is(new Point3D(-1, -2, -3), new MakeMirror(O).Value());
        Is(new Point3D(-1, -2, 3), new MakeMirror(ZAxis).Value());
        Is(new Point3D(-1, -2, 3), new MakeMirror(new Line3D(O, Z)).Value());
        Is(new Point3D(-1, -2, 3), new MakeMirror(O, Z).Value());
        Is(new Point3D(1, 2, -3), new MakeMirror(XY).Value());
        Is(new Point3D(1, 2, -3), new MakeMirror(new Plane3D(new CoordinateSystem(XY))).Value());
        Is2(new Point2D(-1, -2), new MakeMirror2d(O2).Value());
        Is2(new Point2D(1, -2), new MakeMirror2d(XAxis2).Value());
        Is2(new Point2D(1, -2), new MakeMirror2d(new Line2D(O2, X2)).Value());
        Is2(new Point2D(1, -2), new MakeMirror2d(O2, X2).Value());

        Is(new Point3D(-2, 1, 3), new MakeRotation(ZAxis, M_PI / 2).Value());
        Is(new Point3D(-2, 1, 3), new MakeRotation(new Line3D(O, Z), M_PI / 2).Value());
        Is(new Point3D(-2, 1, 3), new MakeRotation(O, Z, M_PI / 2).Value());
        Is2(new Point2D(-2, 1), new MakeRotation2d(O2, M_PI / 2).Value());

        Is(new Point3D(2, 4, 6), new MakeScale(O, 2).Value());
        Is2(new Point2D(2, 4), new MakeScale2d(O2, 2).Value());

        Is(new Point3D(2, 3, 4), new MakeTranslation(new Vector3D(1, 1, 1)).Value());
        Is(new Point3D(2, 3, 4), new MakeTranslation(O, new Point3D(1, 1, 1)).Value());
        Is2(new Point2D(2, 3), new MakeTranslation2d(new Vector2D(1, 1)).Value());
        Is2(new Point2D(2, 3), new MakeTranslation2d(O2, new Point2D(1, 1)).Value());
    }
}
