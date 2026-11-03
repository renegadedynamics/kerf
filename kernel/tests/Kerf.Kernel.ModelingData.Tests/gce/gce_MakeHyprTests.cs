// Port of: src/ModelingData/TKGeomBase/GTests/gce_MakeHypr_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Kernel.ModelingData.Tests;

public class gce_MakeHyprTest
{
    [Fact]
    public void FromAxis_ValidRadii_Done()
    {
        MakeHypr aMaker = new(new Frame3D(), 10.0, 5.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Hyperbola3D aHypr = aMaker.Value();
        Assert.Equal(10.0, aHypr.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aHypr.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromAxis_MajorLessThanMinor_Done()
    {
        MakeHypr aMaker = new(new Frame3D(), 3.0, 7.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Hyperbola3D aHypr = aMaker.Value();
        Assert.Equal(3.0, aHypr.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aHypr.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromAxis_EqualRadii_Done()
    {
        MakeHypr aMaker = new(new Frame3D(), 5.0, 5.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
    }

    [Fact]
    public void FromAxis_ZeroRadii_Done()
    {
        MakeHypr aMaker = new(new Frame3D(), 0.0, 0.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
    }

    [Fact]
    public void FromAxis_NegativeMajor_NegativeRadius()
    {
        MakeHypr aMaker = new(new Frame3D(), -1.0, 5.0);
        Assert.False(aMaker.IsDone);
        Assert.Equal(NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromAxis_NegativeMinor_NegativeRadius()
    {
        MakeHypr aMaker = new(new Frame3D(), 5.0, -1.0);
        Assert.False(aMaker.IsDone);
        Assert.Equal(NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromAxis_BothNegative_NegativeRadius()
    {
        MakeHypr aMaker = new(new Frame3D(), -3.0, -2.0);
        Assert.False(aMaker.IsDone);
        Assert.Equal(NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromPoints_Valid_Done()
    {
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aS1 = new(10.0, 0.0, 0.0);
        Point3D aS2 = new(0.0, 5.0, 0.0);

        MakeHypr aMaker = new(aS1, aS2, aCenter);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Hyperbola3D aHypr = aMaker.Value();
        Assert.Equal(10.0, aHypr.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aHypr.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromPoints_MajorLessThanMinor_Done()
    {
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aS1 = new(3.0, 0.0, 0.0);
        Point3D aS2 = new(0.0, 7.0, 0.0);

        MakeHypr aMaker = new(aS1, aS2, aCenter);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Hyperbola3D aHypr = aMaker.Value();
        Assert.Equal(3.0, aHypr.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aHypr.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromPoints_ConfusedS1Center_ConfusedPoints()
    {
        Point3D aCenter = new(1.0, 2.0, 3.0);
        Point3D aS1 = new(1.0, 2.0, 3.0);
        Point3D aS2 = new(4.0, 5.0, 6.0);

        MakeHypr aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone);
        Assert.Equal(ConfusedPoints, aMaker.Status());
    }

    [Fact]
    public void FromPoints_ConfusedS2Center_ConfusedPoints()
    {
        Point3D aCenter = new(1.0, 2.0, 3.0);
        Point3D aS1 = new(4.0, 5.0, 6.0);
        Point3D aS2 = new(1.0, 2.0, 3.0);

        MakeHypr aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone);
        Assert.Equal(ConfusedPoints, aMaker.Status());
    }

    [Fact]
    public void FromPoints_ConfusedS1S2_ConfusedPoints()
    {
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aS1 = new(5.0, 5.0, 5.0);
        Point3D aS2 = new(5.0, 5.0, 5.0);

        MakeHypr aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone);
        Assert.Equal(ConfusedPoints, aMaker.Status());
    }

    [Fact]
    public void FromPoints_Collinear_ColinearPoints()
    {
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aS1 = new(10.0, 0.0, 0.0);
        Point3D aS2 = new(5.0, 0.0, 0.0);

        MakeHypr aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone);
        Assert.Equal(ColinearPoints, aMaker.Status());
    }

    [Fact]
    public void FromPoints_TinyScale_ConfusedPoints()
    {
        double aRes = Constants.Resolution();
        Point3D aCenter = new(0.0, 0.0, 0.0);
        Point3D aS1 = new(2.0 * aRes, 0.0, 0.0);
        Point3D aS2 = new(0.0, 1.1 * aRes, 0.0);

        MakeHypr aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone);
        Assert.Equal(ConfusedPoints, aMaker.Status());
    }
}

public class gce_MakeHypr2dTest
{
    [Fact]
    public void FromMajorAxis_ValidRadii_Done()
    {
        MakeHypr2d aMaker = new(new Axis2D(), 10.0, 5.0, true);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Hyperbola2D aHypr = aMaker.Value();
        Assert.Equal(10.0, aHypr.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aHypr.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromMajorAxis_MajorLessThanMinor_Done()
    {
        MakeHypr2d aMaker = new(new Axis2D(), 3.0, 7.0, true);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Hyperbola2D aHypr = aMaker.Value();
        Assert.Equal(3.0, aHypr.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aHypr.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromMajorAxis_NegativeRadius_NegativeRadius()
    {
        MakeHypr2d aMaker = new(new Axis2D(), -1.0, 5.0, true);
        Assert.False(aMaker.IsDone);
        Assert.Equal(NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromAx22d_ValidRadii_Done()
    {
        MakeHypr2d aMaker = new(new Frame2D(), 8.0, 4.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
    }

    [Fact]
    public void FromAx22d_NegativeRadius_NegativeRadius()
    {
        MakeHypr2d aMaker = new(new Frame2D(), 5.0, -2.0);
        Assert.False(aMaker.IsDone);
        Assert.Equal(NegativeRadius, aMaker.Status());
    }

    [Fact]
    public void FromPoints_Valid_Done()
    {
        Point2D aCenter = new(0.0, 0.0);
        Point2D aS1 = new(10.0, 0.0);
        Point2D aS2 = new(0.0, 5.0);

        MakeHypr2d aMaker = new(aS1, aS2, aCenter);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Hyperbola2D aHypr = aMaker.Value();
        Assert.Equal(10.0, aHypr.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aHypr.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromPoints_MajorLessThanMinor_Done()
    {
        Point2D aCenter = new(0.0, 0.0);
        Point2D aS1 = new(3.0, 0.0);
        Point2D aS2 = new(0.0, 7.0);

        MakeHypr2d aMaker = new(aS1, aS2, aCenter);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());

        Hyperbola2D aHypr = aMaker.Value();
        Assert.Equal(3.0, aHypr.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aHypr.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromPoints_ConfusedS1Center_ConfusedPoints()
    {
        Point2D aCenter = new(1.0, 2.0);
        Point2D aS1 = new(1.0, 2.0);
        Point2D aS2 = new(4.0, 5.0);

        MakeHypr2d aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone);
        Assert.Equal(ConfusedPoints, aMaker.Status());
    }

    [Fact]
    public void FromPoints_ConfusedS2Center_ConfusedPoints()
    {
        Point2D aCenter = new(1.0, 2.0);
        Point2D aS1 = new(4.0, 5.0);
        Point2D aS2 = new(1.0, 2.0);

        MakeHypr2d aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone);
        Assert.Equal(ConfusedPoints, aMaker.Status());
    }

    [Fact]
    public void FromPoints_Collinear_ColinearPoints()
    {
        Point2D aCenter = new(0.0, 0.0);
        Point2D aS1 = new(10.0, 0.0);
        Point2D aS2 = new(5.0, 0.0);

        MakeHypr2d aMaker = new(aS1, aS2, aCenter);
        Assert.False(aMaker.IsDone);
        Assert.Equal(ColinearPoints, aMaker.Status());
    }
}
