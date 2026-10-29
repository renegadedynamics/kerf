// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace OcctNet.ModelingData.Tests;

/// <summary>OCCT.NET's own checks of gce: the classes OCCT's GTests leave out and the port's decisions.</summary>
public class gce_PortTest
{
    private const double Tol = 1e-12;

    [Fact]
    public void Value_of_a_failed_construction_throws()
    {
        var aMaker = new gce_MakeLin(new gp_Pnt(1, 2, 3), new gp_Pnt(1, 2, 3));
        Assert.True(aMaker.IsError());
        Assert.Equal(gce_ConfusedPoints, aMaker.Status());
        Assert.Throws<StdFail_NotDone>(() => aMaker.Value());
        Assert.Throws<StdFail_NotDone>(() => (gp_Lin)aMaker);
    }

    [Fact]
    public void Lines_and_directions()
    {
        gp_Lin aLin = (gp_Lin)new gce_MakeLin(new gp_Pnt(1, 0, 0), new gp_Pnt(1, 2, 0));
        Assert.Equal(new gp_Dir(0, 1, 0), aLin.Direction());

        gp_Lin2d aLin2d = new gce_MakeLin2d(new gp_Lin2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0).Value();
        Assert.Equal(new gp_Pnt2d(0, 2), aLin2d.Location()); // to the left of the line for a positive distance
        Assert.Equal(gce_NullAxis, new gce_MakeLin2d(0, 0, 1).Status());
        Assert.Equal(1.0, new gce_MakeLin2d(0, 1, -1).Value().Distance(new gp_Pnt2d(5, 0)), Tol);

        Assert.Equal(gce_NullVector, new gce_MakeDir(0, 0, 0).Status());
        Assert.Equal(new gp_Dir(0, 0, 1), new gce_MakeDir(new gp_Pnt(1, 1, 1), new gp_Pnt(1, 1, 4)).Value());
        Assert.Equal(gce_ConfusedPoints, new gce_MakeDir2d(new gp_Pnt2d(1, 1), new gp_Pnt2d(1, 1)).Status());
        Assert.Equal(new gp_Dir2d(0, -1), new gce_MakeDir2d(new gp_Vec2d(0, -3)).Value());
    }

    [Fact]
    public void Circle_through_three_points()
    {
        var aMaker = new gce_MakeCirc(new gp_Pnt(1, 0, 0), new gp_Pnt(0, 1, 0), new gp_Pnt(-1, 0, 0));
        Assert.True(aMaker.IsDone());
        gp_Circ aCirc = aMaker.Value();
        Assert.Equal(1.0, aCirc.Radius(), Tol);
        Assert.Equal(0.0, aCirc.Location().Distance(new gp_Pnt(0, 0, 0)), Tol);

        Assert.Equal(gce_ColinearPoints, new gce_MakeCirc(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(2, 0, 0)).Status());
        Assert.Equal(gce_ConfusedPoints, new gce_MakeCirc(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 0, 0), new gp_Pnt(2, 0, 0)).Status());

        // OCCT leaves the status unset for three coincident points; the port's is gce_Done (decision 118)
        var aPoint = new gce_MakeCirc(new gp_Pnt(1, 2, 3), new gp_Pnt(1, 2, 3), new gp_Pnt(1, 2, 3));
        Assert.True(aPoint.IsDone());
        Assert.Equal(0.0, aPoint.Value().Radius());
    }

    [Fact]
    public void Circle_from_center_plane_and_negative_radius_reports_the_status()
    {
        // OCCT's release build returns the status; the port must not throw from C.Value() (decision 119)
        var aMaker = new gce_MakeCirc(new gp_Pnt(0, 0, 0), new gp_Pln(), -1.0);
        Assert.Equal(gce_NegativeRadius, aMaker.Status());
        Assert.Equal(2.0, new gce_MakeCirc(new gp_Pnt(0, 0, 0), new gp_Pln(), 2.0).Value().Radius());
    }

    [Fact]
    public void Circle2d_through_collinear_points_fails()
    {
        // OCCT leaves the status unset; the port reports gce_IntersectionError (decision 118)
        var aMaker = new gce_MakeCirc2d(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 1), new gp_Pnt2d(2, 2));
        Assert.False(aMaker.IsDone());
        Assert.Equal(gce_IntersectionError, aMaker.Status());

        var aCirc = new gce_MakeCirc2d(new gp_Pnt2d(1, 0), new gp_Pnt2d(0, 1), new gp_Pnt2d(-1, 0)).Value();
        Assert.Equal(1.0, aCirc.Radius(), Tol);
        Assert.True(aCirc.IsDirect());
        var aReversed = new gce_MakeCirc2d(new gp_Pnt2d(-1, 0), new gp_Pnt2d(0, 1), new gp_Pnt2d(1, 0)).Value();
        Assert.False(aReversed.IsDirect());
    }

    [Fact]
    public void Planes()
    {
        Assert.Equal(gce_ColinearPoints, new gce_MakePln(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 1), new gp_Pnt(2, 2, 2)).Status());
        Assert.Equal(gce_BadEquation, new gce_MakePln(0, 0, 0, 1).Status());
        gp_Pln aPln = new gce_MakePln(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(0, 1, 0)).Value();
        Assert.Equal(new gp_Dir(0, 0, 1), aPln.Axis().Direction());
        Assert.Equal(3.0, new gce_MakePln(aPln, 3.0).Value().Location().Z(), Tol);
        Assert.Equal(gce_ConfusedPoints, new gce_MakePln(new gp_Pnt(1, 1, 1), new gp_Pnt(1, 1, 1)).Status());
    }

    [Fact]
    public void Conics_status()
    {
        Assert.Equal(gce_InvertRadius, new gce_MakeElips(new gp_Ax2(), 1, 2).Status());
        Assert.Equal(gce_NegativeRadius, new gce_MakeElips(new gp_Ax2(), 1, -2).Status());
        Assert.Equal(gce_InvertRadius, new gce_MakeElips2d(new gp_Ax2d(), 1, 2).Status());
        Assert.Equal(gce_NullFocusLength, new gce_MakeParab(new gp_Ax2(), -1).Status());
        Assert.Equal(gce_NullAxis, new gce_MakeParab2d(new gp_Pnt2d(1, 1), new gp_Pnt2d(1, 1)).Status());
        Assert.Equal(2.0, new gce_MakeParab2d(new gp_Pnt2d(2, 0), new gp_Pnt2d(0, 0)).Value().Focal(), Tol);
        Assert.Equal(gce_BadAngle, new gce_MakeCone(new gp_Ax2(), 0.0, 1.0).Status());
        Assert.Equal(gce_NullAxis, new gce_MakeCone(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 0, 0), 1.0, 2.0).Status());
        Assert.Equal(gce_NegativeRadius, new gce_MakeCylinder(new gp_Cylinder(new gp_Ax3(), 1.0), -2.0).Status());
        Assert.Equal(3.0, new gce_MakeCylinder(new gp_Circ(new gp_Ax2(), 3.0)).Value().Radius());
    }

    [Fact]
    public void Transformations_match_gp_Trsf()
    {
        gp_Pnt aP = new(1, 2, 3);
        gp_Ax1 anAx = new(aP, new gp_Dir(0, 1, 0));
        Assert.Equal(gp_Trsf.Rotation(anAx, 0.5), new gce_MakeRotation(anAx, 0.5).Value());
        Assert.Equal(gp_Trsf.Rotation(anAx, 0.5), (gp_Trsf)new gce_MakeRotation(new gp_Lin(anAx), 0.5));
        Assert.Equal(gp_Trsf.Scale(aP, 2.0), new gce_MakeScale(aP, 2.0).Value());
        Assert.Equal(gp_Trsf.Translation(new gp_Vec(1, 1, 1)), new gce_MakeTranslation(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 1)).Value());
        Assert.Equal(gp_Trsf.Mirror(anAx), new gce_MakeMirror(new gp_Lin(anAx)).Value());
        Assert.Equal(gp_Trsf.Mirror(new gp_Ax2(aP, new gp_Dir(0, 0, 1))), new gce_MakeMirror(new gp_Pln(aP, new gp_Dir(0, 0, 1))).Value());

        gp_Pnt2d aP2 = new(1, 2);
        Assert.Equal(gp_Trsf2d.Rotation(aP2, 0.5), new gce_MakeRotation2d(aP2, 0.5).Value());
        Assert.Equal(gp_Trsf2d.Scale(aP2, 3.0), new gce_MakeScale2d(aP2, 3.0).Value());
        Assert.Equal(gp_Trsf2d.Translation(new gp_Vec2d(1, 2)), new gce_MakeTranslation2d(new gp_Vec2d(1, 2)).Value());
        Assert.Equal(gp_Trsf2d.Mirror(aP2), new gce_MakeMirror2d(aP2).Value());
    }
}
