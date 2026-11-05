// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Kerf's own checks of gce: the classes OCCT's GTests leave out and the port's decisions.</summary>
public class gce_PortTest
{
    private const double Tol = 1e-12;

    [Fact]
    public void Value_of_a_failed_construction_throws()
    {
        var aMaker = new MakeLin(new Point3D(1, 2, 3), new Point3D(1, 2, 3));
        Assert.True(aMaker.IsError);
        Assert.Equal(ConfusedPoints, aMaker.Status());
        EngineAssert.Throws<NotDoneException>(() => aMaker.Value());
        EngineAssert.Throws<NotDoneException>(() => (Line3D)aMaker);
    }

    [Fact]
    public void Lines_and_directions()
    {
        Line3D aLin = (Line3D)new MakeLin(new Point3D(1, 0, 0), new Point3D(1, 2, 0));
        Assert.Equal(new Direction3D(0, 1, 0), aLin.Direction);

        Line2D aLin2d = new MakeLin2d(new Line2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0).Value();
        Assert.Equal(new Point2D(0, 2), aLin2d.Location); // to the left of the line for a positive distance
        Assert.Equal(NullAxis, new MakeLin2d(0, 0, 1).Status());
        Assert.Equal(1.0, new MakeLin2d(0, 1, -1).Value().DistanceTo(new Point2D(5, 0)), Tol);

        Assert.Equal(NullVector, new MakeDir(0, 0, 0).Status());
        Assert.Equal(new Direction3D(0, 0, 1), new MakeDir(new Point3D(1, 1, 1), new Point3D(1, 1, 4)).Value());
        Assert.Equal(ConfusedPoints, new MakeDir2d(new Point2D(1, 1), new Point2D(1, 1)).Status());
        Assert.Equal(new Direction2D(0, -1), new MakeDir2d(new Vector2D(0, -3)).Value());
    }

    [Fact]
    public void Circle_through_three_points()
    {
        var aMaker = new MakeCirc(new Point3D(1, 0, 0), new Point3D(0, 1, 0), new Point3D(-1, 0, 0));
        Assert.True(aMaker.IsDone);
        Circle3D aCirc = aMaker.Value();
        Assert.Equal(1.0, aCirc.Radius, Tol);
        Assert.Equal(0.0, aCirc.Location.DistanceTo(new Point3D(0, 0, 0)), Tol);

        Assert.Equal(ColinearPoints, new MakeCirc(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0)).Status());
        Assert.Equal(ConfusedPoints, new MakeCirc(new Point3D(0, 0, 0), new Point3D(0, 0, 0), new Point3D(2, 0, 0)).Status());

        // OCCT leaves the status unset for three coincident points; the port's is gce_Done (decision 118)
        var aPoint = new MakeCirc(new Point3D(1, 2, 3), new Point3D(1, 2, 3), new Point3D(1, 2, 3));
        Assert.True(aPoint.IsDone);
        Assert.Equal(0.0, aPoint.Value().Radius);
    }

    [Fact]
    public void Circle_from_center_plane_and_negative_radius_reports_the_status()
    {
        // OCCT's release build returns the status; the port must not throw from C.Value() (decision 119)
        var aMaker = new MakeCirc(new Point3D(0, 0, 0), new Plane3D(), -1.0);
        Assert.Equal(NegativeRadius, aMaker.Status());
        Assert.Equal(2.0, new MakeCirc(new Point3D(0, 0, 0), new Plane3D(), 2.0).Value().Radius);
    }

    [Fact]
    public void Circle2d_through_collinear_points_fails()
    {
        // OCCT leaves the status unset; the port reports gce_IntersectionError (decision 118)
        var aMaker = new MakeCirc2d(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 2));
        Assert.False(aMaker.IsDone);
        Assert.Equal(IntersectionError, aMaker.Status());

        var aCirc = new MakeCirc2d(new Point2D(1, 0), new Point2D(0, 1), new Point2D(-1, 0)).Value();
        Assert.Equal(1.0, aCirc.Radius, Tol);
        Assert.True(aCirc.IsDirect);
        var aReversed = new MakeCirc2d(new Point2D(-1, 0), new Point2D(0, 1), new Point2D(1, 0)).Value();
        Assert.False(aReversed.IsDirect);
    }

    [Fact]
    public void Planes()
    {
        Assert.Equal(ColinearPoints, new MakePln(new Point3D(0, 0, 0), new Point3D(1, 1, 1), new Point3D(2, 2, 2)).Status());
        Assert.Equal(BadEquation, new MakePln(0, 0, 0, 1).Status());
        Plane3D aPln = new MakePln(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 1, 0)).Value();
        Assert.Equal(new Direction3D(0, 0, 1), aPln.Axis.Direction);
        Assert.Equal(3.0, new MakePln(aPln, 3.0).Value().Location.Z, Tol);
        Assert.Equal(ConfusedPoints, new MakePln(new Point3D(1, 1, 1), new Point3D(1, 1, 1)).Status());
    }

    [Fact]
    public void Conics_status()
    {
        Assert.Equal(InvertRadius, new MakeElips(new Frame3D(), 1, 2).Status());
        Assert.Equal(NegativeRadius, new MakeElips(new Frame3D(), 1, -2).Status());
        Assert.Equal(InvertRadius, new MakeElips2d(new Axis2D(), 1, 2).Status());
        Assert.Equal(NullFocusLength, new MakeParab(new Frame3D(), -1).Status());
        Assert.Equal(NullAxis, new MakeParab2d(new Point2D(1, 1), new Point2D(1, 1)).Status());
        Assert.Equal(2.0, new MakeParab2d(new Point2D(2, 0), new Point2D(0, 0)).Value().Focal, Tol);
        Assert.Equal(BadAngle, new MakeCone(new Frame3D(), 0.0, 1.0).Status());
        Assert.Equal(NullAxis, new MakeCone(new Point3D(0, 0, 0), new Point3D(0, 0, 0), 1.0, 2.0).Status());
        Assert.Equal(NegativeRadius, new MakeCylinder(new Cylinder3D(new CoordinateSystem(), 1.0), -2.0).Status());
        Assert.Equal(3.0, new MakeCylinder(new Circle3D(new Frame3D(), 3.0)).Value().Radius);
    }

    [Fact]
    public void Transformations_match_gp_Trsf()
    {
        Point3D aP = new(1, 2, 3);
        Axis3D anAx = new(aP, new Direction3D(0, 1, 0));
        Assert.Equal(Transform3D.Rotation(anAx, 0.5), new MakeRotation(anAx, 0.5).Value());
        Assert.Equal(Transform3D.Rotation(anAx, 0.5), (Transform3D)new MakeRotation(new Line3D(anAx), 0.5));
        Assert.Equal(Transform3D.Scale(aP, 2.0), new MakeScale(aP, 2.0).Value());
        Assert.Equal(Transform3D.Translation(new Vector3D(1, 1, 1)), new MakeTranslation(new Point3D(0, 0, 0), new Point3D(1, 1, 1)).Value());
        Assert.Equal(Transform3D.Mirror(anAx), new MakeMirror(new Line3D(anAx)).Value());
        Assert.Equal(Transform3D.Mirror(new Frame3D(aP, new Direction3D(0, 0, 1))), new MakeMirror(new Plane3D(aP, new Direction3D(0, 0, 1))).Value());

        Point2D aP2 = new(1, 2);
        Assert.Equal(Transform2D.Rotation(aP2, 0.5), new MakeRotation2d(aP2, 0.5).Value());
        Assert.Equal(Transform2D.Scale(aP2, 3.0), new MakeScale2d(aP2, 3.0).Value());
        Assert.Equal(Transform2D.Translation(new Vector2D(1, 2)), new MakeTranslation2d(new Vector2D(1, 2)).Value());
        Assert.Equal(Transform2D.Mirror(aP2), new MakeMirror2d(aP2).Value());
    }
}
