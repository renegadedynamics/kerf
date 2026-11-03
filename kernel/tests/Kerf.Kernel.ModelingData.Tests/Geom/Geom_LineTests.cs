// Port of: src/ModelingData/TKG3d/GTests/Geom_Line_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom_LineTest
{
    [Fact]
    public void ConstructFromAx1()
    {
        Axis3D anAx1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        LineCurve3D aLine = new(anAx1);
        Point3D aLoc = aLine.Lin().Location();
        Assert.True(aLoc.IsEqual(new Point3D(0.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void ConstructFromPointAndDir()
    {
        LineCurve3D aLine = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0));
        Point3D aLoc = aLine.Lin().Location();
        Assert.True(aLoc.IsEqual(new Point3D(1.0, 2.0, 3.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.True(aLine.Lin().Direction().IsEqual(new Direction3D(0.0, 0.0, 1.0), global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void D0Evaluation()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        aLine.D0(5.0, out Point3D aPnt);
        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D1Evaluation()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));
        aLine.D1(3.0, out Point3D aPnt, out Vector3D aV1);
        Assert.Equal(3.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        // First derivative is the direction vector (constant for a line)
        Assert.Equal(0.0, aV1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aV1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aV1.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D2Evaluation()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        aLine.D2(1.0, out _, out _, out Vector3D aV2);
        // Second derivative of a line is zero
        Assert.Equal(0.0, aV2.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void InfiniteParameters()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), aLine.FirstParameter());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aLine.LastParameter());
    }

    [Fact]
    public void Reverse()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        aLine.Reverse();
        Assert.True(aLine.Lin().Direction().IsEqual(new Direction3D(-1.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void Reversed()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Curve3D aReversed = aLine.Reversed();
        LineCurve3D? aRevLine = aReversed as LineCurve3D;
        Assert.NotNull(aRevLine);
        Assert.True(aRevLine.Lin().Direction().IsEqual(new Direction3D(-1.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Angular()));
    }

    [Fact]
    public void Transform()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Transform3D aTrsf = new();
        aTrsf.SetTranslation(new Vector3D(0.0, 0.0, 5.0));
        aLine.Transform(aTrsf);
        Assert.Equal(5.0, aLine.Lin().Location().Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        LineCurve3D aLine = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0));
        Geometry3D aCopy = aLine.Copy();
        LineCurve3D? aCopyLine = aCopy as LineCurve3D;
        Assert.NotNull(aCopyLine);
        Assert.True(aCopyLine.Lin().Location().IsEqual(new Point3D(1.0, 2.0, 3.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
    }
}
