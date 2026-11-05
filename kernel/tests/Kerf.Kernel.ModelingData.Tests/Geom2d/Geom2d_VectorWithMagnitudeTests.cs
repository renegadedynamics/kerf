// Port of: src/ModelingData/TKG2d/GTests/Geom2d_VectorWithMagnitude_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_VectorWithMagnitudeTest
{
    private readonly VectorWithMagnitude2 myVec1;
    private readonly VectorWithMagnitude2 myVec2;

    public Geom2d_VectorWithMagnitudeTest()
    {
        myVec1 = new VectorWithMagnitude2(3.0, 4.0);
        myVec2 = new VectorWithMagnitude2(1.0, 0.0);
    }

    [Fact]
    public void ConstructFromCoords()
    {
        Assert.Equal(3.0, myVec1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, myVec1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ConstructFromVec2d()
    {
        VectorWithMagnitude2 aVec =
            new VectorWithMagnitude2(new Vector2D(5.0, 12.0));
        Assert.Equal(13.0, aVec.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ConstructFromTwoPoints()
    {
        VectorWithMagnitude2 aVec =
            new VectorWithMagnitude2(new Point2D(1.0, 2.0), new Point2D(4.0, 6.0));
        Assert.Equal(3.0, aVec.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aVec.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aVec.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Magnitude()
    {
        Assert.Equal(5.0, myVec1.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, myVec2.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SquareMagnitude()
    {
        Assert.Equal(25.0, myVec1.SquareMagnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Added()
    {
        VectorWithMagnitude2 aSum = myVec1.Added(myVec2);
        Assert.Equal(4.0, aSum.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aSum.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Subtracted()
    {
        VectorWithMagnitude2 aDiff = myVec1.Subtracted(myVec2);
        Assert.Equal(2.0, aDiff.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aDiff.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Multiplied()
    {
        VectorWithMagnitude2 aScaled = myVec1.Multiplied(2.0);
        Assert.Equal(6.0, aScaled.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aScaled.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aScaled.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Divided()
    {
        VectorWithMagnitude2 aScaled = myVec1.Divided(5.0);
        Assert.Equal(0.6, aScaled.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.8, aScaled.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Normalize()
    {
        VectorWithMagnitude2 aVec = new VectorWithMagnitude2(3.0, 4.0);
        aVec.Normalize();
        Assert.Equal(1.0, aVec.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.6, aVec.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.8, aVec.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Normalized()
    {
        VectorWithMagnitude2 aNorm = myVec1.Normalized();
        Assert.Equal(1.0, aNorm.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        // Original unchanged
        Assert.Equal(5.0, myVec1.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Crossed()
    {
        // Cross product in 2D is scalar: (3,4) x (1,0) = 3*0 - 4*1 = -4
        double aCross = myVec1.Crossed(myVec2);
        Assert.Equal(-4.0, aCross, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Dot()
    {
        // (3,4) . (1,0) = 3
        double aDot = myVec1.Dot(myVec2);
        Assert.Equal(3.0, aDot, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Angle()
    {
        // Angle from (1,0) to (0,1) = PI/2
        VectorWithMagnitude2 aVecY   = new VectorWithMagnitude2(0.0, 1.0);
        double                            anAngle = myVec2.Angle(aVecY);
        Assert.Equal(M_PI / 2.0, anAngle, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Angle_Negative()
    {
        // Angle from (0,1) to (1,0) = -PI/2
        VectorWithMagnitude2 aVecY   = new VectorWithMagnitude2(0.0, 1.0);
        double                            anAngle = aVecY.Angle(myVec2);
        Assert.Equal(-M_PI / 2.0, anAngle, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Reverse()
    {
        VectorWithMagnitude2 aVec = new VectorWithMagnitude2(3.0, 4.0);
        aVec.Reverse();
        Assert.Equal(-3.0, aVec.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aVec.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetCoord()
    {
        myVec1.SetCoord(7.0, 8.0);
        Assert.Equal(7.0, myVec1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, myVec1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Vec2d()
    {
        Vector2D aVec = myVec1.Vec2d();
        Assert.Equal(3.0, aVec.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aVec.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geometry2D            aCopyGeom = myVec1.Copy();
        VectorWithMagnitude2 aCopy =
            (aCopyGeom as VectorWithMagnitude2)!;

        Assert.False(aCopy is null);
        Assert.Equal(3.0, aCopy.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aCopy.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());

        aCopy.SetCoord(0.0, 0.0);
        Assert.Equal(3.0, myVec1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Add_InPlace()
    {
        myVec1.Add(myVec2);
        Assert.Equal(4.0, myVec1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, myVec1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Subtract_InPlace()
    {
        myVec1.Subtract(myVec2);
        Assert.Equal(2.0, myVec1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, myVec1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Multiply_InPlace()
    {
        myVec1.Multiply(3.0);
        Assert.Equal(9.0, myVec1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(12.0, myVec1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Divide_InPlace()
    {
        myVec1.Divide(5.0);
        Assert.Equal(0.6, myVec1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.8, myVec1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Rotation()
    {
        Transform2D aTrsf = new Transform2D().WithRotation(new Point2D(0.0, 0.0), M_PI / 2.0);

        VectorWithMagnitude2 aVec = new VectorWithMagnitude2(1.0, 0.0);
        aVec.Transform(aTrsf);

        Assert.Equal(0.0, aVec.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aVec.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
