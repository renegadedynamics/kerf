// Port of: src/FoundationClasses/TKMath/GTests/Bnd_B3_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class Bnd_B3dTest
{
    [Fact]
    public void DefaultConstructor()
    {
        Bnd_B3d aBox = new();
        Assert.True(aBox.IsVoid());
    }

    [Fact]
    public void ConstructorWithCenterAndHSize()
    {
        gp_XYZ aCenter = new(5.0, 10.0, 15.0);
        gp_XYZ aHSize = new(2.0, 3.0, 4.0);

        Bnd_B3d aBox = new(aCenter, aHSize);

        Assert.False(aBox.IsVoid());

        gp_XYZ aMin = aBox.CornerMin();
        gp_XYZ aMax = aBox.CornerMax();

        Assert.Equal(3.0, aMin.X(), Precision.Confusion());
        Assert.Equal(7.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(11.0, aMin.Z(), Precision.Confusion());
        Assert.Equal(7.0, aMax.X(), Precision.Confusion());
        Assert.Equal(13.0, aMax.Y(), Precision.Confusion());
        Assert.Equal(19.0, aMax.Z(), Precision.Confusion());
    }

    [Fact]
    public void Clear()
    {
        Bnd_B3d aBox = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));
        Assert.False(aBox.IsVoid());

        aBox.Clear();
        Assert.True(aBox.IsVoid());
    }

    [Fact]
    public void AddPoint()
    {
        Bnd_B3d aBox = new();

        aBox.Add(new gp_XYZ(1.0, 2.0, 3.0));
        Assert.False(aBox.IsVoid());

        gp_XYZ aMin = aBox.CornerMin();
        gp_XYZ aMax = aBox.CornerMax();
        Assert.Equal(1.0, aMin.X(), Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(3.0, aMin.Z(), Precision.Confusion());
        Assert.Equal(1.0, aMax.X(), Precision.Confusion());
        Assert.Equal(2.0, aMax.Y(), Precision.Confusion());
        Assert.Equal(3.0, aMax.Z(), Precision.Confusion());

        aBox.Add(new gp_XYZ(4.0, 5.0, 6.0));
        aMin = aBox.CornerMin();
        aMax = aBox.CornerMax();
        Assert.Equal(1.0, aMin.X(), Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(3.0, aMin.Z(), Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), Precision.Confusion());
        Assert.Equal(6.0, aMax.Z(), Precision.Confusion());
    }

    [Fact]
    public void AddPnt()
    {
        Bnd_B3d aBox = new();
        aBox.Add(new gp_Pnt(1.0, 2.0, 3.0));

        Assert.False(aBox.IsVoid());
        gp_XYZ aMin = aBox.CornerMin();
        Assert.Equal(1.0, aMin.X(), Precision.Confusion());
        Assert.Equal(2.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(3.0, aMin.Z(), Precision.Confusion());
    }

    [Fact]
    public void AddBox()
    {
        Bnd_B3d aBox1 = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));
        Bnd_B3d aBox2 = new(new gp_XYZ(3.0, 3.0, 3.0), new gp_XYZ(1.0, 1.0, 1.0));

        aBox1.Add(aBox2);

        gp_XYZ aMin = aBox1.CornerMin();
        gp_XYZ aMax = aBox1.CornerMax();
        Assert.Equal(-1.0, aMin.X(), Precision.Confusion());
        Assert.Equal(-1.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(-1.0, aMin.Z(), Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), Precision.Confusion());
        Assert.Equal(4.0, aMax.Y(), Precision.Confusion());
        Assert.Equal(4.0, aMax.Z(), Precision.Confusion());
    }

    [Fact]
    public void SquareExtent()
    {
        Bnd_B3d aBox = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(3.0, 4.0, 5.0));
        double  aSqExtent = aBox.SquareExtent();

        // Square diagonal = 4 * (3^2 + 4^2 + 5^2) = 4 * 50 = 200
        Assert.Equal(200.0, aSqExtent, Precision.Confusion());
    }

    [Fact]
    public void Enlarge()
    {
        Bnd_B3d aBox = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));
        aBox.Enlarge(0.5);

        gp_XYZ aMin = aBox.CornerMin();
        gp_XYZ aMax = aBox.CornerMax();
        Assert.Equal(-1.5, aMin.X(), Precision.Confusion());
        Assert.Equal(-1.5, aMin.Y(), Precision.Confusion());
        Assert.Equal(-1.5, aMin.Z(), Precision.Confusion());
        Assert.Equal(1.5, aMax.X(), Precision.Confusion());
        Assert.Equal(1.5, aMax.Y(), Precision.Confusion());
        Assert.Equal(1.5, aMax.Z(), Precision.Confusion());
    }

    [Fact]
    public void Limit()
    {
        // Test limiting a large box by a smaller box inside it
        // aBox1: (-5, -5, -5) to (5, 5, 5)
        // aBox2: (0, 0, 0) to (4, 4, 4)
        Bnd_B3d aBox1 = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(5.0, 5.0, 5.0));
        Bnd_B3d aBox2 = new(new gp_XYZ(2.0, 2.0, 2.0), new gp_XYZ(2.0, 2.0, 2.0));

        bool isLimited = aBox1.Limit(aBox2);
        Assert.True(isLimited);

        // After limiting, aBox1's min corner should align with aBox2's min corner
        gp_XYZ aMin = aBox1.CornerMin();
        gp_XYZ aMax = aBox1.CornerMax();
        Assert.Equal(0.0, aMin.X(), Precision.Confusion());
        Assert.Equal(0.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(0.0, aMin.Z(), Precision.Confusion());
        Assert.Equal(5.0, aMax.X(), Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), Precision.Confusion());
        Assert.Equal(5.0, aMax.Z(), Precision.Confusion());

        // Test with non-intersecting boxes
        Bnd_B3d aBox3 = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));
        Bnd_B3d aBox4 = new(new gp_XYZ(10.0, 10.0, 10.0), new gp_XYZ(1.0, 1.0, 1.0));
        bool    isLimited2 = aBox3.Limit(aBox4);
        Assert.False(isLimited2);
    }

    [Fact]
    public void IsOutPoint()
    {
        Bnd_B3d aBox = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));

        Assert.False(aBox.IsOut(new gp_XYZ(0.0, 0.0, 0.0)));
        Assert.False(aBox.IsOut(new gp_XYZ(0.5, 0.5, 0.5)));
        Assert.True(aBox.IsOut(new gp_XYZ(2.0, 0.0, 0.0)));
        Assert.True(aBox.IsOut(new gp_XYZ(0.0, 2.0, 0.0)));
        Assert.True(aBox.IsOut(new gp_XYZ(0.0, 0.0, 2.0)));
    }

    [Fact]
    public void IsOutSphere()
    {
        Bnd_B3d aBox = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));

        // Sphere at (0, 0, 0) with small radius - should intersect
        Assert.False(aBox.IsOut(new gp_XYZ(0.0, 0.0, 0.0), 0.5));

        // Sphere far away - should not intersect
        Assert.True(aBox.IsOut(new gp_XYZ(10.0, 10.0, 10.0), 1.0));
    }

    [Fact]
    public void IsOutBox()
    {
        Bnd_B3d aBox1 = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));
        Bnd_B3d aBox2 = new(new gp_XYZ(0.5, 0.5, 0.5), new gp_XYZ(1.0, 1.0, 1.0));
        Bnd_B3d aBox3 = new(new gp_XYZ(5.0, 5.0, 5.0), new gp_XYZ(1.0, 1.0, 1.0));

        Assert.False(aBox1.IsOut(aBox2));
        Assert.True(aBox1.IsOut(aBox3));
    }

    [Fact]
    public void IsOutLine()
    {
        Bnd_B3d aBox = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));

        // Line passing through box
        gp_Ax1 aLine1 = new(new gp_Pnt(-2.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        Assert.False(aBox.IsOut(aLine1));

        // Line not intersecting box
        gp_Ax1 aLine2 = new(new gp_Pnt(-2.0, 5.0, 5.0), new gp_Dir(1.0, 0.0, 0.0));
        Assert.True(aBox.IsOut(aLine2));
    }

    [Fact]
    public void IsOutPlane()
    {
        Bnd_B3d aBox = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));

        // Plane passing through box
        gp_Ax3 aPlane1 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        Assert.False(aBox.IsOut(aPlane1));

        // Plane not intersecting box
        gp_Ax3 aPlane2 = new(new gp_Pnt(0.0, 0.0, 5.0), new gp_Dir(0.0, 0.0, 1.0));
        Assert.True(aBox.IsOut(aPlane2));
    }

    [Fact]
    public void IsInBox()
    {
        Bnd_B3d aBox1 = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(0.5, 0.5, 0.5));
        Bnd_B3d aBox2 = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(2.0, 2.0, 2.0));

        Assert.True(aBox1.IsIn(aBox2));
        Assert.False(aBox2.IsIn(aBox1));
    }

    [Fact]
    public void Transformed()
    {
        Bnd_B3d aBox = new(new gp_XYZ(1.0, 1.0, 1.0), new gp_XYZ(1.0, 1.0, 1.0));

        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(2.0, 3.0, 4.0));

        Bnd_B3d aTransformedBox = aBox.Transformed(aTrsf);

        gp_XYZ aMin = aTransformedBox.CornerMin();
        gp_XYZ aMax = aTransformedBox.CornerMax();

        Assert.Equal(2.0, aMin.X(), Precision.Confusion());
        Assert.Equal(3.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(4.0, aMin.Z(), Precision.Confusion());
        Assert.Equal(4.0, aMax.X(), Precision.Confusion());
        Assert.Equal(5.0, aMax.Y(), Precision.Confusion());
        Assert.Equal(6.0, aMax.Z(), Precision.Confusion());
    }

    [Fact]
    public void TransformedWithRotation()
    {
        Bnd_B3d aBox = new(new gp_XYZ(1.0, 0.0, 0.0), new gp_XYZ(0.5, 0.5, 0.5));

        gp_Trsf aTrsf = new();
        aTrsf.SetRotation(new gp_Ax1(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), M_PI / 2.0);

        Bnd_B3d aTransformedBox = aBox.Transformed(aTrsf);

        // After 90 degree rotation around Z-axis, the center should be at (0, 1, 0)
        gp_XYZ aMin = aTransformedBox.CornerMin();
        gp_XYZ aMax = aTransformedBox.CornerMax();

        // The rotated box should have its center near (0, 1, 0)
        // and half-size should remain approximately the same (with some expansion for rotation)
        Assert.Equal(0.0, (aMin.X() + aMax.X()) / 2.0, 1e-10);
        Assert.Equal(1.0, (aMin.Y() + aMax.Y()) / 2.0, 1e-10);
        Assert.Equal(0.0, (aMin.Z() + aMax.Z()) / 2.0, 1e-10);
    }

    [Fact]
    public void SetCenterAndHSize()
    {
        Bnd_B3d aBox = new();

        aBox.SetCenter(new gp_XYZ(5.0, 10.0, 15.0));
        aBox.SetHSize(new gp_XYZ(2.0, 3.0, 4.0));

        gp_XYZ aMin = aBox.CornerMin();
        gp_XYZ aMax = aBox.CornerMax();

        Assert.Equal(3.0, aMin.X(), Precision.Confusion());
        Assert.Equal(7.0, aMin.Y(), Precision.Confusion());
        Assert.Equal(11.0, aMin.Z(), Precision.Confusion());
        Assert.Equal(7.0, aMax.X(), Precision.Confusion());
        Assert.Equal(13.0, aMax.Y(), Precision.Confusion());
        Assert.Equal(19.0, aMax.Z(), Precision.Confusion());
    }

    [Fact]
    public void IsOutRay()
    {
        Bnd_B3d aBox = new(new gp_XYZ(1.0, 1.0, 1.0), new gp_XYZ(1.0, 1.0, 1.0));

        // Ray starting before the box and pointing towards it
        gp_Ax1 aRay1 = new(new gp_Pnt(-1.0, 1.0, 1.0), new gp_Dir(1.0, 0.0, 0.0));
        Assert.False(aBox.IsOut(aRay1, true));

        // Ray starting inside the box
        gp_Ax1 aRay2 = new(new gp_Pnt(1.0, 1.0, 1.0), new gp_Dir(1.0, 0.0, 0.0));
        Assert.False(aBox.IsOut(aRay2, true));

        // Ray pointing away from the box
        gp_Ax1 aRay3 = new(new gp_Pnt(-1.0, 1.0, 1.0), new gp_Dir(-1.0, 0.0, 0.0));
        Assert.True(aBox.IsOut(aRay3, true));
    }

    [Fact]
    public void IsOutLineWithOverthickness()
    {
        Bnd_B3d aBox = new(new gp_XYZ(0.0, 0.0, 0.0), new gp_XYZ(1.0, 1.0, 1.0));

        // Line passing just outside the box, but within overthickness
        gp_Ax1 aLine = new(new gp_Pnt(-2.0, 1.2, 0.0), new gp_Dir(1.0, 0.0, 0.0));

        // Without overthickness, should be out
        Assert.True(aBox.IsOut(aLine, false, 0.0));

        // With sufficient overthickness, should intersect
        Assert.False(aBox.IsOut(aLine, false, 0.3));
    }
}

public class Bnd_B3fTest
{
    [Fact]
    public void FloatPrecision()
    {
        Bnd_B3f aBox = new(new gp_XYZ(1.0f, 2.0f, 3.0f), new gp_XYZ(0.5f, 0.5f, 0.5f));

        Assert.False(aBox.IsVoid());

        gp_XYZ aMin = aBox.CornerMin();
        gp_XYZ aMax = aBox.CornerMax();

        Assert.Equal(0.5, aMin.X(), 1e-5);
        Assert.Equal(1.5, aMin.Y(), 1e-5);
        Assert.Equal(2.5, aMin.Z(), 1e-5);
        Assert.Equal(1.5, aMax.X(), 1e-5);
        Assert.Equal(2.5, aMax.Y(), 1e-5);
        Assert.Equal(3.5, aMax.Z(), 1e-5);
    }
}
