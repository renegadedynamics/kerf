// Port of: src/ModelingData/TKG2d/GTests/Geom2d_CartesianPoint_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_CartesianPointTest
{
    [Fact]
    public void ConstructFromCoords()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(3.0, 4.0);
        Assert.Equal(3.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ConstructFromPnt2d()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(new Point2D(5.0, 6.0));
        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(6.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetCoord()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(0.0, 0.0);
        aPnt.SetCoord(7.0, 8.0);
        Assert.Equal(7.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetPnt2d()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(0.0, 0.0);
        aPnt.SetPnt2d(new Point2D(1.0, 2.0));
        Assert.Equal(1.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetX_SetY()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(0.0, 0.0);
        aPnt.SetX(10.0);
        aPnt.SetY(20.0);
        Assert.Equal(10.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(20.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Coord()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(3.0, 4.0);
        double                             aX = 0.0, aY = 0.0;
        aPnt.Coord(out aX, out aY);
        Assert.Equal(3.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Pnt2d()
    {
        CartesianPoint2 aPnt   = new CartesianPoint2(3.0, 4.0);
        Point2D                     aGpPnt = aPnt.Pnt2d();
        Assert.Equal(3.0, aGpPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aGpPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Distance()
    {
        CartesianPoint2 aP1 = new CartesianPoint2(0.0, 0.0);
        CartesianPoint2 aP2 = new CartesianPoint2(3.0, 4.0);
        Assert.Equal(5.0, aP1.Distance(aP2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SquareDistance()
    {
        CartesianPoint2 aP1 = new CartesianPoint2(0.0, 0.0);
        CartesianPoint2 aP2 = new CartesianPoint2(3.0, 4.0);
        Assert.Equal(25.0, aP1.SquareDistance(aP2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Distance_SamePoint()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(5.0, 5.0);
        Assert.Equal(0.0, aPnt.Distance(aPnt), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(1.0, 2.0);

        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(3.0, 4.0));
        aPnt.Transform(aTrsf);

        Assert.Equal(4.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(6.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        CartesianPoint2 aPnt = new CartesianPoint2(1.0, 0.0);

        Transform2D aTrsf = new Transform2D().WithRotation(new Point2D(0.0, 0.0), M_PI / 2.0);
        aPnt.Transform(aTrsf);

        Assert.Equal(0.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        CartesianPoint2 aPnt      = new CartesianPoint2(3.0, 4.0);
        Geometry2D       aCopyGeom = aPnt.Copy();
        CartesianPoint2 aCopy     = (aCopyGeom as CartesianPoint2)!;

        Assert.False(aCopy is null);
        Assert.Equal(3.0, aCopy.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aCopy.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        aCopy.SetCoord(0.0, 0.0);
        Assert.Equal(3.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
