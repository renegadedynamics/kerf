// Port of: src/ModelingData/TKG2d/GTests/Geom2d_AxisPlacement_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_AxisPlacementTest
{
    [Fact]
    public void ConstructFromAx2d()
    {
        gp_Ax2d                           anAxis = new(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_AxisPlacement anAP = new Geom2d_AxisPlacement(anAxis);

        Assert.Equal(1.0, anAP.Location().X(), Precision.Confusion());
        Assert.Equal(2.0, anAP.Location().Y(), Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().X(), Precision.Confusion());
        Assert.Equal(0.0, anAP.Direction().Y(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromPointAndDir()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(3.0, 4.0), new gp_Dir2d(0.0, 1.0));

        Assert.Equal(3.0, anAP.Location().X(), Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().Y(), Precision.Confusion());
    }

    [Fact]
    public void SetLocation()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        anAP.SetLocation(new gp_Pnt2d(5.0, 6.0));

        Assert.Equal(5.0, anAP.Location().X(), Precision.Confusion());
        Assert.Equal(6.0, anAP.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void SetDirection()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        anAP.SetDirection(new gp_Dir2d(0.0, 1.0));

        Assert.Equal(0.0, anAP.Direction().X(), Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().Y(), Precision.Confusion());
    }

    [Fact]
    public void SetAxis()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));

        gp_Ax2d aNewAxis = new(new gp_Pnt2d(10.0, 20.0), new gp_Dir2d(0.0, 1.0));
        anAP.SetAxis(aNewAxis);

        Assert.Equal(10.0, anAP.Location().X(), Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().Y(), Precision.Confusion());
    }

    [Fact]
    public void Ax2d()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0));

        gp_Ax2d anAxis = anAP.Ax2d();
        Assert.Equal(1.0, anAxis.Location().X(), Precision.Confusion());
        Assert.Equal(1.0, anAxis.Direction().X(), Precision.Confusion());
    }

    [Fact]
    public void Angle_Perpendicular()
    {
        Geom2d_AxisPlacement anAP1 =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_AxisPlacement anAP2 =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(0.0, 1.0));

        Assert.Equal(M_PI / 2.0, anAP1.Angle(anAP2), Precision.Confusion());
    }

    [Fact]
    public void Angle_Parallel()
    {
        Geom2d_AxisPlacement anAP1 =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_AxisPlacement anAP2 =
            new Geom2d_AxisPlacement(new gp_Pnt2d(5.0, 5.0), new gp_Dir2d(1.0, 0.0));

        Assert.Equal(0.0, anAP1.Angle(anAP2), Precision.Confusion());
    }

    [Fact]
    public void Angle_Opposite()
    {
        Geom2d_AxisPlacement anAP1 =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_AxisPlacement anAP2 =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(-1.0, 0.0));

        Assert.Equal(M_PI, Math.Abs(anAP1.Angle(anAP2)), Precision.Confusion());
    }

    [Fact]
    public void Reverse()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        anAP.Reverse();

        Assert.Equal(-1.0, anAP.Direction().X(), Precision.Confusion());
        Assert.Equal(0.0, anAP.Direction().Y(), Precision.Confusion());
        // Location unchanged
        Assert.Equal(0.0, anAP.Location().X(), Precision.Confusion());
    }

    [Fact]
    public void Reversed()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        Geom2d_AxisPlacement aReversed = anAP.Reversed();

        Assert.Equal(-1.0, aReversed.Direction().X(), Precision.Confusion());
        // Original unchanged
        Assert.Equal(1.0, anAP.Direction().X(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0));

        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec2d(10.0, 20.0));
        anAP.Transform(aTrsf);

        Assert.Equal(11.0, anAP.Location().X(), Precision.Confusion());
        Assert.Equal(22.0, anAP.Location().Y(), Precision.Confusion());
        // Direction unchanged
        Assert.Equal(1.0, anAP.Direction().X(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(1.0, 0.0), new gp_Dir2d(1.0, 0.0));

        gp_Trsf2d aTrsf = new();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 2.0);
        anAP.Transform(aTrsf);

        Assert.Equal(0.0, anAP.Location().X(), Precision.Confusion());
        Assert.Equal(1.0, anAP.Location().Y(), Precision.Confusion());
        Assert.Equal(0.0, anAP.Direction().X(), Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().Y(), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(0.0, 1.0));
        Geom2d_Geometry      aCopyGeom = anAP.Copy();
        Geom2d_AxisPlacement aCopy     = (aCopyGeom as Geom2d_AxisPlacement)!;

        Assert.False(aCopy is null);
        Assert.Equal(1.0, aCopy.Location().X(), Precision.Confusion());
        Assert.Equal(1.0, aCopy.Direction().Y(), Precision.Confusion());

        // Verify independence
        aCopy.SetLocation(new gp_Pnt2d(99.0, 99.0));
        Assert.Equal(1.0, anAP.Location().X(), Precision.Confusion());
    }
}
