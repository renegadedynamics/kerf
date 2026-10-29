// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Transformation_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace OcctNet.ModelingData.Tests;

public class Geom2d_TransformationTest
{
    [Fact]
    public void DefaultConstructor_Identity()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();

        Assert.Equal(gp_Identity, aTrsf.Form());
        Assert.Equal(1.0, aTrsf.ScaleFactor(), Precision.Confusion());
        Assert.False(aTrsf.IsNegative());
    }

    [Fact]
    public void ConstructFromTrsf2d()
    {
        gp_Trsf2d aGpTrsf = new();
        aGpTrsf.SetTranslation(new gp_Vec2d(3.0, 4.0));

        Geom2d_Transformation aTrsf = new Geom2d_Transformation(aGpTrsf);
        Assert.Equal(gp_Translation, aTrsf.Form());
    }

    [Fact]
    public void SetTranslation_Vector()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetTranslation(new gp_Vec2d(5.0, 10.0));

        Assert.Equal(gp_Translation, aTrsf.Form());

        double aX = 1.0, aY = 2.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(6.0, aX, Precision.Confusion());
        Assert.Equal(12.0, aY, Precision.Confusion());
    }

    [Fact]
    public void SetTranslation_TwoPoints()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetTranslation(new gp_Pnt2d(1.0, 1.0), new gp_Pnt2d(4.0, 5.0));

        double aX = 0.0, aY = 0.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(3.0, aX, Precision.Confusion());
        Assert.Equal(4.0, aY, Precision.Confusion());
    }

    [Fact]
    public void SetRotation()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 2.0);

        Assert.Equal(gp_Rotation, aTrsf.Form());

        double aX = 1.0, aY = 0.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(0.0, aX, Precision.Confusion());
        Assert.Equal(1.0, aY, Precision.Confusion());
    }

    [Fact]
    public void SetRotation_180Degrees()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI);

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(-3.0, aX, Precision.Confusion());
        Assert.Equal(-4.0, aY, Precision.Confusion());
    }

    [Fact]
    public void SetScale()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetScale(new gp_Pnt2d(0.0, 0.0), 2.0);

        Assert.Equal(gp_Scale, aTrsf.Form());
        Assert.Equal(2.0, aTrsf.ScaleFactor(), Precision.Confusion());

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(6.0, aX, Precision.Confusion());
        Assert.Equal(8.0, aY, Precision.Confusion());
    }

    [Fact]
    public void SetScale_Negative()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetScale(new gp_Pnt2d(0.0, 0.0), -1.0);

        // In 2D, IsNegative checks determinant sign; scale(-1) has det = (-1)*(-1) = 1 > 0
        Assert.False(aTrsf.IsNegative());
        Assert.Equal(-1.0, aTrsf.ScaleFactor(), Precision.Confusion());

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(-3.0, aX, Precision.Confusion());
        Assert.Equal(-4.0, aY, Precision.Confusion());
    }

    [Fact]
    public void SetMirror_Point()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetMirror(new gp_Pnt2d(0.0, 0.0));

        Assert.Equal(gp_PntMirror, aTrsf.Form());
        // In 2D, point mirror has det = (-1)*(-1) = 1 > 0, so IsNegative is false
        Assert.False(aTrsf.IsNegative());

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(-3.0, aX, Precision.Confusion());
        Assert.Equal(-4.0, aY, Precision.Confusion());
    }

    [Fact]
    public void SetMirror_Axis()
    {
        // Mirror about X axis
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetMirror(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)));

        Assert.Equal(gp_Ax1Mirror, aTrsf.Form());

        double aX = 3.0, aY = 4.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(3.0, aX, Precision.Confusion());
        Assert.Equal(-4.0, aY, Precision.Confusion());
    }

    [Fact]
    public void Inverted()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetTranslation(new gp_Vec2d(5.0, 10.0));

        Geom2d_Transformation anInv = aTrsf.Inverted();

        // Applying both should be identity
        double aX = 7.0, aY = 3.0;
        aTrsf.Transforms(ref aX, ref aY);
        anInv.Transforms(ref aX, ref aY);
        Assert.Equal(7.0, aX, Precision.Confusion());
        Assert.Equal(3.0, aY, Precision.Confusion());
    }

    [Fact]
    public void Multiplied()
    {
        // Translation + Rotation
        Geom2d_Transformation aTrans = new Geom2d_Transformation();
        aTrans.SetTranslation(new gp_Vec2d(1.0, 0.0));

        Geom2d_Transformation aRot = new Geom2d_Transformation();
        aRot.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 2.0);

        // aRot * aTrans: first translate, then rotate
        Geom2d_Transformation aComposed = aRot.Multiplied(aTrans);

        double aX = 0.0, aY = 0.0;
        aComposed.Transforms(ref aX, ref aY);
        // (0,0) . translate to (1,0) . rotate 90deg to (0,1)
        Assert.Equal(0.0, aX, Precision.Confusion());
        Assert.Equal(1.0, aY, Precision.Confusion());
    }

    [Fact]
    public void Power_Zero_IsIdentity()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 4.0);
        aTrsf.Power(0);

        Assert.Equal(gp_Identity, aTrsf.Form());
    }

    [Fact]
    public void Power_Positive()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetRotation(new gp_Pnt2d(0.0, 0.0), M_PI / 2.0);
        aTrsf.Power(2); // Two 90-degree rotations = 180 degrees

        double aX = 1.0, aY = 0.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(-1.0, aX, Precision.Confusion());
        Assert.Equal(0.0, aY, Precision.Confusion());
    }

    [Fact]
    public void Value_Matrix()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        // Identity matrix values
        Assert.Equal(1.0, aTrsf.Value(1, 1), Precision.Confusion());
        Assert.Equal(0.0, aTrsf.Value(1, 2), Precision.Confusion());
        Assert.Equal(0.0, aTrsf.Value(2, 1), Precision.Confusion());
        Assert.Equal(1.0, aTrsf.Value(2, 2), Precision.Confusion());
    }

    [Fact]
    public void Trsf2d_RoundTrip()
    {
        gp_Trsf2d aGpTrsf = new();
        aGpTrsf.SetRotation(new gp_Pnt2d(1.0, 2.0), M_PI / 3.0);

        Geom2d_Transformation aTrsf = new Geom2d_Transformation(aGpTrsf);
        gp_Trsf2d                          aBack = aTrsf.Trsf2d();

        // Verify the round-trip preserves the transformation
        double aX1 = 5.0, aY1 = 7.0;
        double aX2 = 5.0, aY2 = 7.0;
        aGpTrsf.Transforms(ref aX1, ref aY1);
        aBack.Transforms(ref aX2, ref aY2);
        Assert.Equal(aX2, aX1, Precision.Confusion());
        Assert.Equal(aY2, aY1, Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetScale(new gp_Pnt2d(0.0, 0.0), 3.0);

        Geom2d_Transformation aCopy = aTrsf.Copy();
        Assert.Equal(3.0, aCopy.ScaleFactor(), Precision.Confusion());

        // Verify independence
        aCopy.SetScale(new gp_Pnt2d(0.0, 0.0), 5.0);
        Assert.Equal(3.0, aTrsf.ScaleFactor(), Precision.Confusion());
    }

    [Fact]
    public void SetScale_WithCenter()
    {
        // Scaling with center at (2, 3) by factor 2
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetScale(new gp_Pnt2d(2.0, 3.0), 2.0);

        // The center should remain fixed
        double aX = 2.0, aY = 3.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(2.0, aX, Precision.Confusion());
        Assert.Equal(3.0, aY, Precision.Confusion());

        // A different point should scale from the center
        aX = 4.0;
        aY = 3.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(6.0, aX, Precision.Confusion()); // 2 + 2*(4-2) = 6
        Assert.Equal(3.0, aY, Precision.Confusion()); // 3 + 2*(3-3) = 3
    }

    [Fact]
    public void SetRotation_WithCenter()
    {
        // 90-degree rotation about (1, 1)
        Geom2d_Transformation aTrsf = new Geom2d_Transformation();
        aTrsf.SetRotation(new gp_Pnt2d(1.0, 1.0), M_PI / 2.0);

        // Center stays fixed
        double aX = 1.0, aY = 1.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(1.0, aX, Precision.Confusion());
        Assert.Equal(1.0, aY, Precision.Confusion());

        // (2, 1) relative to center is (1, 0), rotated 90 deg is (0, 1), so result is (1, 2)
        aX = 2.0;
        aY = 1.0;
        aTrsf.Transforms(ref aX, ref aY);
        Assert.Equal(1.0, aX, Precision.Confusion());
        Assert.Equal(2.0, aY, Precision.Confusion());
    }
}
