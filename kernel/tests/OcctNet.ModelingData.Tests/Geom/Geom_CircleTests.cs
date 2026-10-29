// Port of: src/ModelingData/TKG3d/GTests/Geom_Circle_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom_CircleTest
{
    private static gp_Ax2 XY(double x = 0, double y = 0, double z = 0) =>
        new(new gp_Pnt(x, y, z), new gp_Dir(0.0, 0.0, 1.0));

    [Fact]
    public void Constructor()
    {
        Geom_Circle aCircle = new(XY(), 5.0);
        Assert.Equal(5.0, aCircle.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Center()
    {
        gp_Pnt aCenter = new(1.0, 2.0, 3.0);
        Geom_Circle aCircle = new(new gp_Ax2(aCenter, new gp_Dir(0.0, 0.0, 1.0)), 5.0);
        Assert.True(aCircle.Circ().Location().IsEqual(aCenter, Precision.Confusion()));
    }

    [Fact]
    public void D0Evaluation()
    {
        Geom_Circle aCircle = new(XY(), 1.0);
        // At parameter 0: point should be (1,0,0) for unit circle in XY plane
        aCircle.D0(0.0, out gp_Pnt aPnt);
        Assert.Equal(1.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Z(), Precision.Confusion());
        // At parameter PI/2: point should be (0,1,0)
        aCircle.D0(M_PI_2, out aPnt);
        Assert.Equal(0.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(1.0, aPnt.Y(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Z(), Precision.Confusion());
        // At parameter PI: point should be (-1,0,0)
        aCircle.D0(M_PI, out aPnt);
        Assert.Equal(-1.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
        // At parameter 3*PI/2: point should be (0,-1,0)
        aCircle.D0(3.0 * M_PI_2, out aPnt);
        Assert.Equal(0.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(-1.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void D1Evaluation()
    {
        Geom_Circle aCircle = new(XY(), 1.0);
        // At parameter 0: tangent should be (0,1,0) for unit circle
        aCircle.D1(0.0, out _, out gp_Vec aV1);
        Assert.Equal(0.0, aV1.X(), Precision.Confusion());
        Assert.Equal(1.0, aV1.Y(), Precision.Confusion());
        Assert.Equal(0.0, aV1.Z(), Precision.Confusion());
    }

    [Fact]
    public void D2Evaluation()
    {
        Geom_Circle aCircle = new(XY(), 1.0);
        // At parameter 0: second derivative should be (-1,0,0) pointing toward center
        aCircle.D2(0.0, out _, out _, out gp_Vec aV2);
        Assert.Equal(-1.0, aV2.X(), Precision.Confusion());
        Assert.Equal(0.0, aV2.Y(), Precision.Confusion());
        Assert.Equal(0.0, aV2.Z(), Precision.Confusion());
    }

    [Fact]
    public void Reverse()
    {
        Geom_Circle aCircle = new(XY(), 1.0);
        double aParam = M_PI_4;
        double aReversedParam = aCircle.ReversedParameter(aParam);
        // For a circle: reversed parameter = 2*PI - U
        Assert.Equal(2.0 * M_PI - aParam, aReversedParam, Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        Geom_Circle aCircle = new(XY(), 5.0);
        gp_Trsf aTrsf = new();
        aTrsf.SetScale(new gp_Pnt(0.0, 0.0, 0.0), 2.0);
        aCircle.Transform(aTrsf);
        Assert.Equal(10.0, aCircle.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom_Circle aCircle = new(XY(1.0, 2.0, 3.0), 7.0);
        Geom_Circle? aCopyCircle = aCircle.Copy() as Geom_Circle;
        Assert.NotNull(aCopyCircle);
        Assert.Equal(7.0, aCopyCircle.Radius(), Precision.Confusion());
        Assert.True(aCopyCircle.Circ().Location().IsEqual(new gp_Pnt(1.0, 2.0, 3.0), Precision.Confusion()));
    }
}
