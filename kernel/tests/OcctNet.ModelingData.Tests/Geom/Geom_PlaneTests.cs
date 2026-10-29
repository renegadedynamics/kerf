// Port of OCCT's src/ModelingData/TKG3d/GTests/Geom_Plane_Test.cxx (V8_0_1), plus OCCT.NET's own checks.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Geom_Plane. Transform is internal: the port's test goes through Transformed.
/// </summary>
public class Geom_PlaneTests
{
    [Fact]
    public void ConstructFromAx3()
    {
        gp_Ax3 anAx3 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        Geom_Plane aPlane = new(anAx3);
        Assert.NotNull(aPlane);
        Assert.True(aPlane.Location().IsEqual(new gp_Pnt(0.0, 0.0, 0.0), Precision.Confusion()));
    }

    [Fact]
    public void ConstructFromPointAndDir()
    {
        Geom_Plane aPlane = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 0.0, 1.0));
        Assert.True(aPlane.Location().IsEqual(new gp_Pnt(1.0, 2.0, 3.0), Precision.Confusion()));
    }

    [Fact]
    public void D0Evaluation()
    {
        Geom_Plane aPlane = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));

        // XY plane at origin: D0(U,V) = (U,V,0) for default axis
        aPlane.D0(3.0, 4.0, out gp_Pnt aPnt);
        Assert.Equal(3.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(4.0, aPnt.Y(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Z(), Precision.Confusion());
    }

    [Fact]
    public void Coefficients()
    {
        Geom_Plane aPlane = new(new gp_Pnt(0.0, 0.0, 5.0), new gp_Dir(0.0, 0.0, 1.0));
        aPlane.Coefficients(out double anA, out double aB, out double aC, out double aD);
        Assert.Equal(0.0, anA, Precision.Confusion());
        Assert.Equal(0.0, aB, Precision.Confusion());
        Assert.Equal(1.0, aC, Precision.Confusion());
        Assert.Equal(-5.0, aD, Precision.Confusion());
    }

    [Fact]
    public void UIso()
    {
        Geom_Plane aPlane = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        Geom_Curve aUIso = aPlane.UIso(2.0);
        Assert.NotNull(aUIso);

        // UIso at U=2 on XY plane should be a line at x=2, varying in Y
        Geom_Line? aLine = aUIso as Geom_Line;
        Assert.NotNull(aLine);

        aLine.D0(0.0, out gp_Pnt aPnt);
        Assert.Equal(2.0, aPnt.X(), Precision.Confusion());
    }

    [Fact]
    public void VIso()
    {
        Geom_Plane aPlane = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        Geom_Curve aVIso = aPlane.VIso(3.0);
        Assert.NotNull(aVIso);

        // VIso at V=3 on XY plane should be a line at y=3, varying in X
        Geom_Line? aLine = aVIso as Geom_Line;
        Assert.NotNull(aLine);

        aLine.D0(0.0, out gp_Pnt aPnt);
        Assert.Equal(3.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        Geom_Plane aPlane = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(0.0, 0.0, 10.0));
        Geom_Plane aMoved = (Geom_Plane)aPlane.Transformed(aTrsf);
        Assert.Equal(10.0, aMoved.Location().Z(), Precision.Confusion());
        Assert.Equal(0.0, aPlane.Location().Z());
    }

    [Fact]
    public void Copy()
    {
        Geom_Plane aPlane = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 0.0, 1.0));
        Geom_Geometry aCopy = aPlane.Copy();
        Geom_Plane? aCopyPlane = aCopy as Geom_Plane;
        Assert.NotNull(aCopyPlane);
        Assert.True(aCopyPlane.Location().IsEqual(new gp_Pnt(1.0, 2.0, 3.0), Precision.Confusion()));
    }

    // OCCT.NET's own checks

    [Fact]
    public void Reversal_And_Parameters()
    {
        Geom_Plane aPlane = new(new gp_Ax3(gp.XOY()));
        Geom_Plane aU = (Geom_Plane)aPlane.UReversed();
        Assert.Equal(aPlane.Value(2, 3), aU.Value(aPlane.UReversedParameter(2), 3));
        Geom_Plane aV = (Geom_Plane)aPlane.VReversed();
        Assert.Equal(aPlane.Value(2, 3), aV.Value(2, aPlane.VReversedParameter(3)));
        // The parametric normal D1U ^ D1V flips with either reversal; the main direction doesn't (left-handed axes)
        gp_Vec aN = aPlane.DN(0, 0, 1, 0).Crossed(aPlane.DN(0, 0, 0, 1));
        Assert.True(aU.DN(0, 0, 1, 0).Crossed(aU.DN(0, 0, 0, 1)).IsOpposite(aN, 1e-15));
        Assert.True(aV.DN(0, 0, 1, 0).Crossed(aV.DN(0, 0, 0, 1)).IsOpposite(aN, 1e-15));
        Assert.Equal(aPlane.Axis().Direction(), aU.Axis().Direction());
    }

    [Fact]
    public void TransformParameters_Matches_Transformed_Surface()
    {
        Geom_Plane aPlane = new(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(1, 1, 1)));
        gp_Trsf T = gp_Trsf.Scale(new gp_Pnt(4, 5, 6), -2.5);
        T = T.Multiplied(gp_Trsf.Rotation(new gp_Ax1(new gp_Pnt(), new gp_Dir(0, 1, 1)), 0.7));
        Geom_Plane aMoved = (Geom_Plane)aPlane.Transformed(T);
        double U = 1.5, V = -0.25;
        double U2 = U, V2 = V;
        aPlane.TransformParameters(ref U2, ref V2, T);
        Assert.True(aMoved.Value(U2, V2).IsEqual(aPlane.Value(U, V).Transformed(T), 1e-12));

        double aPU = U, aPV = V;
        aPlane.ParametricTransformation(T).Transforms(ref aPU, ref aPV);
        Assert.Equal(U2, aPU, 1e-14);
        Assert.Equal(V2, aPV, 1e-14);

        double aInf = Precision.Infinite();
        double V3 = aInf;
        aPlane.TransformParameters(ref U2, ref V3, T);
        Assert.Equal(aInf, V3);
    }

    [Fact]
    public void Derivatives()
    {
        Geom_Plane aPlane = new(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1), new gp_Dir(0, 1, 0)));
        Assert.Equal(new gp_Vec(0, 1, 0), aPlane.DN(1, 1, 1, 0));
        Assert.Equal(new gp_Vec(aPlane.Position().YDirection()), aPlane.DN(1, 1, 0, 1));
        Assert.Equal(new gp_Vec(0, 0, 0), aPlane.DN(1, 1, 1, 1));
        Assert.Throws<Geom_UndefinedDerivative>(() => aPlane.DN(1, 1, 0, 0));
        Assert.Throws<Geom_UndefinedDerivative>(() => aPlane.DN(1, 1, -1, 2));
        aPlane.D3(1, 1, out _, out _, out _, out gp_Vec D2U, out _, out _, out gp_Vec D3U, out _, out _, out gp_Vec D3UVV);
        Assert.Equal(new gp_Vec(0, 0, 0), D2U);
        Assert.Equal(new gp_Vec(0, 0, 0), D3U);
        Assert.Equal(new gp_Vec(0, 0, 0), D3UVV);
    }

    [Fact]
    public void Bounds_And_Flags()
    {
        Geom_Plane aPlane = new(1, 2, 3, 4);
        aPlane.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal(-Precision.Infinite(), U1);
        Assert.Equal(Precision.Infinite(), U2);
        Assert.Equal(-Precision.Infinite(), V1);
        Assert.Equal(Precision.Infinite(), V2);
        Assert.False(aPlane.IsUClosed() || aPlane.IsVClosed() || aPlane.IsUPeriodic() || aPlane.IsVPeriodic());
        Assert.Equal(GeomAbs_Shape.GeomAbs_CN, aPlane.Continuity());
        Assert.Throws<Standard_ConstructionError>(() => new Geom_Plane(0, 0, 0, 1));
    }

    [Fact]
    public void Frozen_Plane_Refuses_Mutation_But_Twins_Work()
    {
        Geom_Plane aPlane = new(new gp_Ax3(gp.XOY()));
        aPlane.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aPlane.SetPln(new gp_Pln(new gp_Ax3(gp.YOZ()))));
        Assert.Throws<Standard_ImmutableObject>(() => aPlane.Translate(new gp_Vec(1, 0, 0)));
        Assert.Throws<Standard_ImmutableObject>(() => aPlane.UReverse());
        Geom_Plane aOther = aPlane.WithPln(new gp_Pln(new gp_Ax3(gp.YOZ())));
        Assert.False(aOther.IsFrozen());
        Assert.Equal(gp.YOZ().Direction(), aOther.Axis().Direction());
        Assert.Equal(gp.XOY().Direction(), aPlane.Axis().Direction());
        Geom_ElementarySurface aLocated = aPlane.WithLocation(new gp_Pnt(0, 0, 7));
        Assert.Equal(7.0, aLocated.Location().Z());
    }
}
