// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Direction_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_DirectionTest
{
    [Fact]
    public void ConstructFromCoords()
    {
        Geom2d_Direction aDir = new Geom2d_Direction(3.0, 4.0);
        // Should be normalized
        Assert.Equal(1.0, aDir.Magnitude(), Precision.Confusion());
        Assert.Equal(0.6, aDir.X(), Precision.Confusion());
        Assert.Equal(0.8, aDir.Y(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromDir2d()
    {
        Geom2d_Direction aDir = new Geom2d_Direction(new gp_Dir2d(0.0, 1.0));
        Assert.Equal(0.0, aDir.X(), Precision.Confusion());
        Assert.Equal(1.0, aDir.Y(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromZero_Throws()
    {
        // The port always checks (No_Exception is never defined)
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2d_Direction(0.0, 0.0));
    }

    [Fact]
    public void MagnitudeAlwaysOne()
    {
        Geom2d_Direction aDir = new Geom2d_Direction(100.0, 200.0);
        Assert.Equal(1.0, aDir.Magnitude(), Precision.Confusion());
        Assert.Equal(1.0, aDir.SquareMagnitude(), Precision.Confusion());
    }

    [Fact]
    public void SetCoord()
    {
        Geom2d_Direction aDir = new Geom2d_Direction(1.0, 0.0);
        aDir.SetCoord(0.0, 5.0);
        Assert.Equal(0.0, aDir.X(), Precision.Confusion());
        Assert.Equal(1.0, aDir.Y(), Precision.Confusion());
    }

    [Fact]
    public void SetCoord_Zero_Throws()
    {
        // The port always checks (No_Exception is never defined)
        Geom2d_Direction aDir = new Geom2d_Direction(1.0, 0.0);
        Assert.ThrowsAny<Standard_ConstructionError>(() => aDir.SetCoord(0.0, 0.0));
    }

    [Fact]
    public void Dir2d()
    {
        Geom2d_Direction aDir   = new Geom2d_Direction(1.0, 0.0);
        gp_Dir2d                aGpDir = aDir.Dir2d();
        Assert.Equal(1.0, aGpDir.X(), Precision.Confusion());
        Assert.Equal(0.0, aGpDir.Y(), Precision.Confusion());
    }

    [Fact]
    public void Crossed()
    {
        Geom2d_Direction aDirX = new Geom2d_Direction(1.0, 0.0);
        Geom2d_Direction aDirY = new Geom2d_Direction(0.0, 1.0);

        // (1,0) x (0,1) = 1
        Assert.Equal(1.0, aDirX.Crossed(aDirY), Precision.Confusion());
        // (0,1) x (1,0) = -1
        Assert.Equal(-1.0, aDirY.Crossed(aDirX), Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        Geom2d_Direction aDirX = new Geom2d_Direction(1.0, 0.0);
        Geom2d_Direction aDirY = new Geom2d_Direction(0.0, 1.0);

        Assert.Equal(M_PI / 2.0, aDirX.Angle(aDirY), Precision.Confusion());
    }

    [Fact]
    public void Angle_OppositeDirections()
    {
        Geom2d_Direction aDirPos = new Geom2d_Direction(1.0, 0.0);
        Geom2d_Direction aDirNeg = new Geom2d_Direction(-1.0, 0.0);

        Assert.Equal(M_PI, Math.Abs(aDirPos.Angle(aDirNeg)), Precision.Confusion());
    }

    [Fact]
    public void Dot()
    {
        Geom2d_Direction aDirX = new Geom2d_Direction(1.0, 0.0);
        Geom2d_Direction aDirY = new Geom2d_Direction(0.0, 1.0);

        Assert.Equal(0.0, aDirX.Dot(aDirY), Precision.Confusion());
        Assert.Equal(1.0, aDirX.Dot(aDirX), Precision.Confusion());
    }

    [Fact]
    public void Reverse()
    {
        Geom2d_Direction aDir = new Geom2d_Direction(1.0, 0.0);
        aDir.Reverse();
        Assert.Equal(-1.0, aDir.X(), Precision.Confusion());
        Assert.Equal(0.0, aDir.Y(), Precision.Confusion());
    }

    [Fact]
    public void CrossedWithVectorWithMagnitude()
    {
        Geom2d_Direction           aDir = new Geom2d_Direction(1.0, 0.0);
        Geom2d_VectorWithMagnitude aVec = new Geom2d_VectorWithMagnitude(0.0, 5.0);

        Assert.Equal(5.0, aDir.Crossed(aVec), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_Direction aDir      = new Geom2d_Direction(1.0, 0.0);
        Geom2d_Geometry  aCopyGeom = aDir.Copy();
        Geom2d_Direction aCopy     = (aCopyGeom as Geom2d_Direction)!;

        Assert.False(aCopy is null);
        Assert.Equal(1.0, aCopy.X(), Precision.Confusion());

        aCopy.SetCoord(0.0, 1.0);
        Assert.Equal(1.0, aDir.X(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        gp_Trsf2d aTrsf = new();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 2.0);

        Geom2d_Direction aDir = new Geom2d_Direction(1.0, 0.0);
        aDir.Transform(aTrsf);

        Assert.Equal(0.0, aDir.X(), Precision.Confusion());
        Assert.Equal(1.0, aDir.Y(), Precision.Confusion());
        Assert.Equal(1.0, aDir.Magnitude(), Precision.Confusion());
    }
}
