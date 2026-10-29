// Port of: src/ModelingData/TKG2d/GTests/Geom2d_VectorWithMagnitude_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_VectorWithMagnitudeTest
{
    private readonly Geom2d_VectorWithMagnitude myVec1;
    private readonly Geom2d_VectorWithMagnitude myVec2;

    public Geom2d_VectorWithMagnitudeTest()
    {
        myVec1 = new Geom2d_VectorWithMagnitude(3.0, 4.0);
        myVec2 = new Geom2d_VectorWithMagnitude(1.0, 0.0);
    }

    [Fact]
    public void ConstructFromCoords()
    {
        Assert.Equal(3.0, myVec1.X(), Precision.Confusion());
        Assert.Equal(4.0, myVec1.Y(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromVec2d()
    {
        Geom2d_VectorWithMagnitude aVec =
            new Geom2d_VectorWithMagnitude(new gp_Vec2d(5.0, 12.0));
        Assert.Equal(13.0, aVec.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void ConstructFromTwoPoints()
    {
        Geom2d_VectorWithMagnitude aVec =
            new Geom2d_VectorWithMagnitude(new gp_Pnt2d(1.0, 2.0), new gp_Pnt2d(4.0, 6.0));
        Assert.Equal(3.0, aVec.X(), Precision.Confusion());
        Assert.Equal(4.0, aVec.Y(), Precision.Confusion());
        Assert.Equal(5.0, aVec.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void Magnitude()
    {
        Assert.Equal(5.0, myVec1.Magnitude(), Precision.Confusion());
        Assert.Equal(1.0, myVec2.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void SquareMagnitude()
    {
        Assert.Equal(25.0, myVec1.SquareMagnitude(), Precision.Confusion());
    }

    [Fact]
    public void Added()
    {
        Geom2d_VectorWithMagnitude aSum = myVec1.Added(myVec2);
        Assert.Equal(4.0, aSum.X(), Precision.Confusion());
        Assert.Equal(4.0, aSum.Y(), Precision.Confusion());
    }

    [Fact]
    public void Subtracted()
    {
        Geom2d_VectorWithMagnitude aDiff = myVec1.Subtracted(myVec2);
        Assert.Equal(2.0, aDiff.X(), Precision.Confusion());
        Assert.Equal(4.0, aDiff.Y(), Precision.Confusion());
    }

    [Fact]
    public void Multiplied()
    {
        Geom2d_VectorWithMagnitude aScaled = myVec1.Multiplied(2.0);
        Assert.Equal(6.0, aScaled.X(), Precision.Confusion());
        Assert.Equal(8.0, aScaled.Y(), Precision.Confusion());
        Assert.Equal(10.0, aScaled.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void Divided()
    {
        Geom2d_VectorWithMagnitude aScaled = myVec1.Divided(5.0);
        Assert.Equal(0.6, aScaled.X(), Precision.Confusion());
        Assert.Equal(0.8, aScaled.Y(), Precision.Confusion());
    }

    [Fact]
    public void Normalize()
    {
        Geom2d_VectorWithMagnitude aVec = new Geom2d_VectorWithMagnitude(3.0, 4.0);
        aVec.Normalize();
        Assert.Equal(1.0, aVec.Magnitude(), Precision.Confusion());
        Assert.Equal(0.6, aVec.X(), Precision.Confusion());
        Assert.Equal(0.8, aVec.Y(), Precision.Confusion());
    }

    [Fact]
    public void Normalized()
    {
        Geom2d_VectorWithMagnitude aNorm = myVec1.Normalized();
        Assert.Equal(1.0, aNorm.Magnitude(), Precision.Confusion());
        // Original unchanged
        Assert.Equal(5.0, myVec1.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void Crossed()
    {
        // Cross product in 2D is scalar: (3,4) x (1,0) = 3*0 - 4*1 = -4
        double aCross = myVec1.Crossed(myVec2);
        Assert.Equal(-4.0, aCross, Precision.Confusion());
    }

    [Fact]
    public void Dot()
    {
        // (3,4) . (1,0) = 3
        double aDot = myVec1.Dot(myVec2);
        Assert.Equal(3.0, aDot, Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        // Angle from (1,0) to (0,1) = PI/2
        Geom2d_VectorWithMagnitude aVecY   = new Geom2d_VectorWithMagnitude(0.0, 1.0);
        double                            anAngle = myVec2.Angle(aVecY);
        Assert.Equal(M_PI / 2.0, anAngle, Precision.Confusion());
    }

    [Fact]
    public void Angle_Negative()
    {
        // Angle from (0,1) to (1,0) = -PI/2
        Geom2d_VectorWithMagnitude aVecY   = new Geom2d_VectorWithMagnitude(0.0, 1.0);
        double                            anAngle = aVecY.Angle(myVec2);
        Assert.Equal(-M_PI / 2.0, anAngle, Precision.Confusion());
    }

    [Fact]
    public void Reverse()
    {
        Geom2d_VectorWithMagnitude aVec = new Geom2d_VectorWithMagnitude(3.0, 4.0);
        aVec.Reverse();
        Assert.Equal(-3.0, aVec.X(), Precision.Confusion());
        Assert.Equal(-4.0, aVec.Y(), Precision.Confusion());
    }

    [Fact]
    public void SetCoord()
    {
        myVec1.SetCoord(7.0, 8.0);
        Assert.Equal(7.0, myVec1.X(), Precision.Confusion());
        Assert.Equal(8.0, myVec1.Y(), Precision.Confusion());
    }

    [Fact]
    public void Vec2d()
    {
        gp_Vec2d aVec = myVec1.Vec2d();
        Assert.Equal(3.0, aVec.X(), Precision.Confusion());
        Assert.Equal(4.0, aVec.Y(), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_Geometry            aCopyGeom = myVec1.Copy();
        Geom2d_VectorWithMagnitude aCopy =
            (aCopyGeom as Geom2d_VectorWithMagnitude)!;

        Assert.False(aCopy is null);
        Assert.Equal(3.0, aCopy.X(), Precision.Confusion());
        Assert.Equal(4.0, aCopy.Y(), Precision.Confusion());

        aCopy.SetCoord(0.0, 0.0);
        Assert.Equal(3.0, myVec1.X(), Precision.Confusion());
    }

    [Fact]
    public void Add_InPlace()
    {
        myVec1.Add(myVec2);
        Assert.Equal(4.0, myVec1.X(), Precision.Confusion());
        Assert.Equal(4.0, myVec1.Y(), Precision.Confusion());
    }

    [Fact]
    public void Subtract_InPlace()
    {
        myVec1.Subtract(myVec2);
        Assert.Equal(2.0, myVec1.X(), Precision.Confusion());
        Assert.Equal(4.0, myVec1.Y(), Precision.Confusion());
    }

    [Fact]
    public void Multiply_InPlace()
    {
        myVec1.Multiply(3.0);
        Assert.Equal(9.0, myVec1.X(), Precision.Confusion());
        Assert.Equal(12.0, myVec1.Y(), Precision.Confusion());
    }

    [Fact]
    public void Divide_InPlace()
    {
        myVec1.Divide(5.0);
        Assert.Equal(0.6, myVec1.X(), Precision.Confusion());
        Assert.Equal(0.8, myVec1.Y(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        gp_Trsf2d aTrsf = new();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 2.0);

        Geom2d_VectorWithMagnitude aVec = new Geom2d_VectorWithMagnitude(1.0, 0.0);
        aVec.Transform(aTrsf);

        Assert.Equal(0.0, aVec.X(), Precision.Confusion());
        Assert.Equal(1.0, aVec.Y(), Precision.Confusion());
    }
}
