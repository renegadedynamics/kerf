// Port of: src/ModelingData/TKG2d/GTests/Geom2d_CartesianPoint_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_CartesianPointTest
{
    [Fact]
    public void ConstructFromCoords()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(3.0, 4.0);
        Assert.Equal(3.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(4.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromPnt2d()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(new gp_Pnt2d(5.0, 6.0));
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(6.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void SetCoord()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(0.0, 0.0);
        aPnt.SetCoord(7.0, 8.0);
        Assert.Equal(7.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(8.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void SetPnt2d()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(0.0, 0.0);
        aPnt.SetPnt2d(new gp_Pnt2d(1.0, 2.0));
        Assert.Equal(1.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(2.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void SetX_SetY()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(0.0, 0.0);
        aPnt.SetX(10.0);
        aPnt.SetY(20.0);
        Assert.Equal(10.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(20.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Coord()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(3.0, 4.0);
        double                             aX = 0.0, aY = 0.0;
        aPnt.Coord(out aX, out aY);
        Assert.Equal(3.0, aX, Precision.Confusion());
        Assert.Equal(4.0, aY, Precision.Confusion());
    }

    [Fact]
    public void Pnt2d()
    {
        Geom2d_CartesianPoint aPnt   = new Geom2d_CartesianPoint(3.0, 4.0);
        gp_Pnt2d                     aGpPnt = aPnt.Pnt2d();
        Assert.Equal(3.0, aGpPnt.X(), Precision.Confusion());
        Assert.Equal(4.0, aGpPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Distance()
    {
        Geom2d_CartesianPoint aP1 = new Geom2d_CartesianPoint(0.0, 0.0);
        Geom2d_CartesianPoint aP2 = new Geom2d_CartesianPoint(3.0, 4.0);
        Assert.Equal(5.0, aP1.Distance(aP2), Precision.Confusion());
    }

    [Fact]
    public void SquareDistance()
    {
        Geom2d_CartesianPoint aP1 = new Geom2d_CartesianPoint(0.0, 0.0);
        Geom2d_CartesianPoint aP2 = new Geom2d_CartesianPoint(3.0, 4.0);
        Assert.Equal(25.0, aP1.SquareDistance(aP2), Precision.Confusion());
    }

    [Fact]
    public void Distance_SamePoint()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(5.0, 5.0);
        Assert.Equal(0.0, aPnt.Distance(aPnt), Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(1.0, 2.0);

        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec2d(3.0, 4.0));
        aPnt.Transform(aTrsf);

        Assert.Equal(4.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(6.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        Geom2d_CartesianPoint aPnt = new Geom2d_CartesianPoint(1.0, 0.0);

        gp_Trsf2d aTrsf = new();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 2.0);
        aPnt.Transform(aTrsf);

        Assert.Equal(0.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(1.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_CartesianPoint aPnt      = new Geom2d_CartesianPoint(3.0, 4.0);
        Geom2d_Geometry       aCopyGeom = aPnt.Copy();
        Geom2d_CartesianPoint aCopy     = (aCopyGeom as Geom2d_CartesianPoint)!;

        Assert.False(aCopy is null);
        Assert.Equal(3.0, aCopy.X(), Precision.Confusion());
        Assert.Equal(4.0, aCopy.Y(), Precision.Confusion());

        aCopy.SetCoord(0.0, 0.0);
        Assert.Equal(3.0, aPnt.X(), Precision.Confusion());
    }
}
