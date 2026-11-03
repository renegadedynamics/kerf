// Port of: src/ModelingData/TKG2d/GTests/Geom2d_AxisPlacement_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class Geom2d_AxisPlacementTest
{
    [Fact]
    public void ConstructFromAx2d()
    {
        Axis2D                           anAxis = new(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0));
        Geom2d_AxisPlacement anAP = new Geom2d_AxisPlacement(anAxis);

        Assert.Equal(1.0, anAP.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, anAP.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, anAP.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ConstructFromPointAndDir()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(3.0, 4.0), new Direction2D(0.0, 1.0));

        Assert.Equal(3.0, anAP.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetLocation()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        anAP.SetLocation(new Point2D(5.0, 6.0));

        Assert.Equal(5.0, anAP.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(6.0, anAP.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetDirection()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        anAP.SetDirection(new Direction2D(0.0, 1.0));

        Assert.Equal(0.0, anAP.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetAxis()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));

        Axis2D aNewAxis = new(new Point2D(10.0, 20.0), new Direction2D(0.0, 1.0));
        anAP.SetAxis(aNewAxis);

        Assert.Equal(10.0, anAP.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Ax2d()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0));

        Axis2D anAxis = anAP.Ax2d();
        Assert.Equal(1.0, anAxis.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, anAxis.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Angle_Perpendicular()
    {
        Geom2d_AxisPlacement anAP1 =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        Geom2d_AxisPlacement anAP2 =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(0.0, 1.0));

        Assert.Equal(M_PI / 2.0, anAP1.Angle(anAP2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Angle_Parallel()
    {
        Geom2d_AxisPlacement anAP1 =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        Geom2d_AxisPlacement anAP2 =
            new Geom2d_AxisPlacement(new Point2D(5.0, 5.0), new Direction2D(1.0, 0.0));

        Assert.Equal(0.0, anAP1.Angle(anAP2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Angle_Opposite()
    {
        Geom2d_AxisPlacement anAP1 =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        Geom2d_AxisPlacement anAP2 =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(-1.0, 0.0));

        Assert.Equal(M_PI, Math.Abs(anAP1.Angle(anAP2)), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Reverse()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        anAP.Reverse();

        Assert.Equal(-1.0, anAP.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, anAP.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        // Location unchanged
        Assert.Equal(0.0, anAP.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Reversed()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        Geom2d_AxisPlacement aReversed = anAP.Reversed();

        Assert.Equal(-1.0, aReversed.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        // Original unchanged
        Assert.Equal(1.0, anAP.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0));

        Transform2D aTrsf = new();
        aTrsf.SetTranslation(new Vector2D(10.0, 20.0));
        anAP.Transform(aTrsf);

        Assert.Equal(11.0, anAP.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(22.0, anAP.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        // Direction unchanged
        Assert.Equal(1.0, anAP.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(1.0, 0.0), new Direction2D(1.0, 0.0));

        Transform2D aTrsf = new();
        aTrsf.SetRotation(new Point2D(0.0, 0.0), M_PI / 2.0);
        anAP.Transform(aTrsf);

        Assert.Equal(0.0, anAP.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, anAP.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, anAP.Direction().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, anAP.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_AxisPlacement anAP =
            new Geom2d_AxisPlacement(new Point2D(1.0, 2.0), new Direction2D(0.0, 1.0));
        Geom2d_Geometry      aCopyGeom = anAP.Copy();
        Geom2d_AxisPlacement aCopy     = (aCopyGeom as Geom2d_AxisPlacement)!;

        Assert.False(aCopy is null);
        Assert.Equal(1.0, aCopy.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aCopy.Direction().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Verify independence
        aCopy.SetLocation(new Point2D(99.0, 99.0));
        Assert.Equal(1.0, anAP.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
