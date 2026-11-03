// Port of: src/FoundationClasses/TKMath/GTests/gp_Circ_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_CircTest
{
    [Fact]
    public void Constructor()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Circle3D aCirc = new(anAxis, 5.0);
        Assert.Equal(5.0, aCirc.Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Area()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Circle3D aCirc = new(anAxis, 1.0);
        Assert.Equal(M_PI, aCirc.Area(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Length()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Circle3D aCirc = new(anAxis, 1.0);
        Assert.Equal(2.0 * M_PI, aCirc.Length(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Center()
    {
        Point3D aCenter = new(1.0, 2.0, 3.0);
        Frame3D anAxis = new(aCenter, new Direction3D(0.0, 0.0, 1.0));
        Circle3D aCirc = new(anAxis, 5.0);
        Assert.True(aCirc.Location().IsEqual(aCenter, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void Translate()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Circle3D aCirc = new(anAxis, 5.0);
        Vector3D aVec = new(10.0, 20.0, 30.0);
        Circle3D aTranslated = aCirc.Translated(aVec);
        Assert.Equal(10.0, aTranslated.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(20.0, aTranslated.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(30.0, aTranslated.Location().Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aTranslated.Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        Frame3D anAxis = new(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Circle3D aCirc = new(anAxis, 2.0);
        Axis3D aRotAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Circle3D aRotated = aCirc.Rotated(aRotAxis, M_PI_2);
        Assert.Equal(0.0, aRotated.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aRotated.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aRotated.Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror()
    {
        Frame3D anAxis = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0));
        Circle3D aCirc = new(anAxis, 5.0);
        Point3D aMirrorPnt = new(0.0, 0.0, 0.0);
        Circle3D aMirrored = aCirc.Mirrored(aMirrorPnt);
        Assert.Equal(-1.0, aMirrored.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-2.0, aMirrored.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-3.0, aMirrored.Location().Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        Frame3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Circle3D aCirc = new(anAxis, 3.0);
        Transform3D aTrsf = new();
        aTrsf = aTrsf.WithScale(new Point3D(0.0, 0.0, 0.0), 2.0);
        Circle3D aTransformed = aCirc.Transformed(aTrsf);
        Assert.Equal(6.0, aTransformed.Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
