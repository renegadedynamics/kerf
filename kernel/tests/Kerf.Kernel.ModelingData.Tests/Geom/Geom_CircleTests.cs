// Port of: src/ModelingData/TKG3d/GTests/Geom_Circle_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom_CircleTest
{
    private static Frame3D XY(double x = 0, double y = 0, double z = 0) =>
        new(new Point3D(x, y, z), new Direction3D(0.0, 0.0, 1.0));

    [Fact]
    public void Constructor()
    {
        CircleCurve3D aCircle = new(XY(), 5.0);
        Assert.Equal(5.0, aCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Center()
    {
        Point3D aCenter = new(1.0, 2.0, 3.0);
        CircleCurve3D aCircle = new(new Frame3D(aCenter, new Direction3D(0.0, 0.0, 1.0)), 5.0);
        Assert.True(aCircle.Circle.Location.IsEqual(aCenter, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void D0Evaluation()
    {
        CircleCurve3D aCircle = new(XY(), 1.0);
        // At parameter 0: point should be (1,0,0) for unit circle in XY plane
        aCircle.D0(0.0, out Point3D aPnt);
        Assert.Equal(1.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
        // At parameter PI/2: point should be (0,1,0)
        aCircle.D0(M_PI_2, out aPnt);
        Assert.Equal(0.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
        // At parameter PI: point should be (-1,0,0)
        aCircle.D0(M_PI, out aPnt);
        Assert.Equal(-1.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        // At parameter 3*PI/2: point should be (0,-1,0)
        aCircle.D0(3.0 * M_PI_2, out aPnt);
        Assert.Equal(0.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-1.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D1Evaluation()
    {
        CircleCurve3D aCircle = new(XY(), 1.0);
        // At parameter 0: tangent should be (0,1,0) for unit circle
        aCircle.D1(0.0, out _, out Vector3D aV1);
        Assert.Equal(0.0, aV1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aV1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aV1.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D2Evaluation()
    {
        CircleCurve3D aCircle = new(XY(), 1.0);
        // At parameter 0: second derivative should be (-1,0,0) pointing toward center
        aCircle.D2(0.0, out _, out _, out Vector3D aV2);
        Assert.Equal(-1.0, aV2.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aV2.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aV2.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Reverse()
    {
        CircleCurve3D aCircle = new(XY(), 1.0);
        double aParam = M_PI_4;
        double aReversedParam = aCircle.ReversedParameter(aParam);
        // For a circle: reversed parameter = 2*PI - U
        Assert.Equal(2.0 * M_PI - aParam, aReversedParam, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        CircleCurve3D aCircle = new(XY(), 5.0);
        Transform3D aTrsf = new Transform3D().WithScale(new Point3D(0.0, 0.0, 0.0), 2.0);
        aCircle.Transform(aTrsf);
        Assert.Equal(10.0, aCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        CircleCurve3D aCircle = new(XY(1.0, 2.0, 3.0), 7.0);
        CircleCurve3D? aCopyCircle = aCircle.Copy() as CircleCurve3D;
        Assert.NotNull(aCopyCircle);
        Assert.Equal(7.0, aCopyCircle.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aCopyCircle.Circle.Location.IsEqual(new Point3D(1.0, 2.0, 3.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
    }
}
