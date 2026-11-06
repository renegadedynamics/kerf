// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Transformation_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_TransformationTest
{
    [Fact]
    public void DefaultConstructor_Identity()
    {
        Transformation2 aTrsf = new Transformation2();

        Assert.Equal(Identity, aTrsf.Form());
        Assert.Equal(1.0, aTrsf.ScaleFactor(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(aTrsf.IsNegative);
    }

    [Fact]
    public void ConstructFromTrsf2d()
    {
        Transform2D aGpTrsf = new Transform2D().WithTranslation(new Vector2D(3.0, 4.0));

        Transformation2 aTrsf = new Transformation2(aGpTrsf);
        Assert.Equal(Translation, aTrsf.Form());
    }

    [Fact]
    public void SetTranslation_Vector()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetTranslation(new Vector2D(5.0, 10.0));

        Assert.Equal(Translation, aTrsf.Form());

        double aX = 1.0, aY = 2.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(6.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(12.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetTranslation_TwoPoints()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetTranslation(new Point2D(1.0, 1.0), new Point2D(4.0, 5.0));

        double aX = 0.0, aY = 0.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(3.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetRotation()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetRotation(new Point2D(0.0, 0.0), M_PI / 2.0);

        Assert.Equal(Rotation, aTrsf.Form());

        double aX = 1.0, aY = 0.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(0.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetRotation_180Degrees()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetRotation(new Point2D(0.0, 0.0), M_PI);

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(-3.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetScale()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetScale(new Point2D(0.0, 0.0), 2.0);

        Assert.Equal(Scale, aTrsf.Form());
        Assert.Equal(2.0, aTrsf.ScaleFactor(), global::Kerf.Engine.Foundation.Precision.Confusion());

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(6.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetScale_Negative()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetScale(new Point2D(0.0, 0.0), -1.0);

        // In 2D, IsNegative checks determinant sign; scale(-1) has det = (-1)*(-1) = 1 > 0
        Assert.False(aTrsf.IsNegative);
        Assert.Equal(-1.0, aTrsf.ScaleFactor(), global::Kerf.Engine.Foundation.Precision.Confusion());

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(-3.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetMirror_Point()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetMirror(new Point2D(0.0, 0.0));

        Assert.Equal(PointMirror, aTrsf.Form());
        // In 2D, point mirror has det = (-1)*(-1) = 1 > 0, so IsNegative is false
        Assert.False(aTrsf.IsNegative);

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(-3.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetMirror_Axis()
    {
        // Mirror about X axis
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetMirror(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)));

        Assert.Equal(AxisMirror, aTrsf.Form());

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(3.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Inverted()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetTranslation(new Vector2D(5.0, 10.0));

        Transformation2 anInv = aTrsf.Inverted();

        // Applying both should be identity
        double aX = 7.0, aY = 3.0;
        aTrsf.Transforms(ref aX, ref aY);
        anInv.Transforms(ref aX, ref aY);
        Assert.Equal(7.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Multiplied()
    {
        // Translation + Rotation
        Transformation2 aTrans = new Transformation2();
        aTrans.SetTranslation(new Vector2D(1.0, 0.0));

        Transformation2 aRot = new Transformation2();
        aRot.SetRotation(new Point2D(0.0, 0.0), M_PI / 2.0);

        // aRot * aTrans: first translate, then rotate
        Transformation2 aComposed = aRot.Multiplied(aTrans);

        double aX = 0.0, aY = 0.0;
        aComposed.Transforms(ref aX, ref aY);
        // (0,0) . translate to (1,0) . rotate 90deg to (0,1)
        Assert.Equal(0.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Power_Zero_IsIdentity()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetRotation(new Point2D(0.0, 0.0), M_PI / 4.0);
        aTrsf.Power(0);

        Assert.Equal(Identity, aTrsf.Form());
    }

    [Fact]
    public void Power_Positive()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetRotation(new Point2D(0.0, 0.0), M_PI / 2.0);
        aTrsf.Power(2); // Two 90-degree rotations = 180 degrees

        double aX = 1.0, aY = 0.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(-1.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Value_Matrix()
    {
        Transformation2 aTrsf = new Transformation2();
        // Identity matrix values
        Assert.Equal(1.0, aTrsf.Value(1, 1), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aTrsf.Value(1, 2), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aTrsf.Value(2, 1), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aTrsf.Value(2, 2), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Trsf2d_RoundTrip()
    {
        Transform2D aGpTrsf = new Transform2D().WithRotation(new Point2D(1.0, 2.0), M_PI / 3.0);

        Transformation2 aTrsf = new Transformation2(aGpTrsf);
        Transform2D                          aBack = aTrsf.Trsf2d();

        // Verify the round-trip preserves the transformation
        double aX1 = 5.0, aY1 = 7.0;
        double aX2 = 5.0, aY2 = 7.0;
        aGpTrsf.Transforms(ref aX1, ref aY1);
        aBack.Transforms(ref aX2, ref aY2);
        Assert.Equal(aX2, aX1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aY2, aY1, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetScale(new Point2D(0.0, 0.0), 3.0);

        Transformation2 aCopy = aTrsf.Copy();
        Assert.Equal(3.0, aCopy.ScaleFactor(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Verify independence
        aCopy.SetScale(new Point2D(0.0, 0.0), 5.0);
        Assert.Equal(3.0, aTrsf.ScaleFactor(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetScale_WithCenter()
    {
        // Scaling with center at (2, 3) by factor 2
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetScale(new Point2D(2.0, 3.0), 2.0);

        // The center should remain fixed
        double aX = 2.0, aY = 3.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(2.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());

        // A different point should scale from the center
        aX = 4.0;
        aY = 3.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(6.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion()); // 2 + 2*(4-2) = 6
        Assert.Equal(3.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion()); // 3 + 2*(3-3) = 3
    }

    [Fact]
    public void SetRotation_WithCenter()
    {
        // 90-degree rotation about (1, 1)
        Transformation2 aTrsf = new Transformation2();
        aTrsf.SetRotation(new Point2D(1.0, 1.0), M_PI / 2.0);

        // Center stays fixed
        double aX = 1.0, aY = 1.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(1.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());

        // (2, 1) relative to center is (1, 0), rotated 90 deg is (0, 1), so result is (1, 2)
        aX = 2.0;
        aY = 1.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(1.0, aX, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aY, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
