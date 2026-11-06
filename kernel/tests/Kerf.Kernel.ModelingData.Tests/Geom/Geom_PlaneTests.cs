// Port of OCCT's src/ModelingData/TKG3d/GTests/Geom_Plane_Test.cxx (V8_0_1), plus Kerf's own checks.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Geom_Plane. Transform is internal: the port's test goes through Transformed.
/// </summary>
public class Geom_PlaneTests
{
    [Fact]
    public void ConstructFromAx3()
    {
        CoordinateSystem anAx3 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface aPlane = new(anAx3);
        Assert.NotNull(aPlane);
        Assert.True(aPlane.Location.IsEqual(new Point3D(0.0, 0.0, 0.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void ConstructFromPointAndDir()
    {
        PlaneSurface aPlane = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0));
        Assert.True(aPlane.Location.IsEqual(new Point3D(1.0, 2.0, 3.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void D0Evaluation()
    {
        PlaneSurface aPlane = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

        // XY plane at origin: D0(U,V) = (U,V,0) for default axis
        aPlane.D0(3.0, 4.0, out Point3D aPnt);
        Assert.Equal(3.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Coefficients()
    {
        PlaneSurface aPlane = new(new Point3D(0.0, 0.0, 5.0), new Direction3D(0.0, 0.0, 1.0));
        aPlane.Coefficients(out double anA, out double aB, out double aC, out double aD);
        Assert.Equal(0.0, anA, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aB, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aC, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.0, aD, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void UIso()
    {
        PlaneSurface aPlane = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Curve3D aUIso = aPlane.UIso(2.0);
        Assert.NotNull(aUIso);

        // UIso at U=2 on XY plane should be a line at x=2, varying in Y
        LineCurve3D? aLine = aUIso as LineCurve3D;
        Assert.NotNull(aLine);

        aLine.D0(0.0, out Point3D aPnt);
        Assert.Equal(2.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void VIso()
    {
        PlaneSurface aPlane = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Curve3D aVIso = aPlane.VIso(3.0);
        Assert.NotNull(aVIso);

        // VIso at V=3 on XY plane should be a line at y=3, varying in X
        LineCurve3D? aLine = aVIso as LineCurve3D;
        Assert.NotNull(aLine);

        aLine.D0(0.0, out Point3D aPnt);
        Assert.Equal(3.0, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        PlaneSurface aPlane = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0.0, 0.0, 10.0));
        PlaneSurface aMoved = (PlaneSurface)aPlane.Transformed(aTrsf);
        Assert.Equal(10.0, aMoved.Location.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPlane.Location.Z);
    }

    [Fact]
    public void Copy()
    {
        PlaneSurface aPlane = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0));
        Geometry3D aCopy = aPlane.Copy();
        PlaneSurface? aCopyPlane = aCopy as PlaneSurface;
        Assert.NotNull(aCopyPlane);
        Assert.True(aCopyPlane.Location.IsEqual(new Point3D(1.0, 2.0, 3.0), global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    // Kerf's own checks

    [Fact]
    public void Reversal_And_Parameters()
    {
        PlaneSurface aPlane = new(new CoordinateSystem(Constants.XOY()));
        PlaneSurface aU = (PlaneSurface)aPlane.UReversed();
        Assert.Equal(aPlane.Value(2, 3), aU.Value(aPlane.UReversedParameter(2), 3));
        PlaneSurface aV = (PlaneSurface)aPlane.VReversed();
        Assert.Equal(aPlane.Value(2, 3), aV.Value(2, aPlane.VReversedParameter(3)));
        // The parametric normal D1U ^ D1V flips with either reversal; the main direction doesn't (left-handed axes)
        Vector3D aN = aPlane.DN(0, 0, 1, 0).Crossed(aPlane.DN(0, 0, 0, 1));
        Assert.True(aU.DN(0, 0, 1, 0).Crossed(aU.DN(0, 0, 0, 1)).IsOpposite(aN, 1e-15));
        Assert.True(aV.DN(0, 0, 1, 0).Crossed(aV.DN(0, 0, 0, 1)).IsOpposite(aN, 1e-15));
        Assert.Equal(aPlane.Axis.Direction, aU.Axis.Direction);
    }

    [Fact]
    public void TransformParameters_Matches_Transformed_Surface()
    {
        PlaneSurface aPlane = new(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(1, 1, 1)));
        Transform3D T = Transform3D.Scale(new Point3D(4, 5, 6), -2.5);
        T = T.Multiplied(Transform3D.Rotation(new Axis3D(new Point3D(), new Direction3D(0, 1, 1)), 0.7));
        PlaneSurface aMoved = (PlaneSurface)aPlane.Transformed(T);
        double U = 1.5, V = -0.25;
        double U2 = U, V2 = V;
        aPlane.TransformParameters(ref U2, ref V2, T);
        Assert.True(aMoved.Value(U2, V2).IsEqual(aPlane.Value(U, V).Transformed(T), 1e-12));

        double aPU = U, aPV = V;
        aPlane.ParametricTransformation(T).Transforms(ref aPU, ref aPV);
        Assert.Equal(U2, aPU, 1e-14);
        Assert.Equal(V2, aPV, 1e-14);

        double aInf = global::Kerf.Engine.Foundation.Precision.Infinite();
        double V3 = aInf;
        aPlane.TransformParameters(ref U2, ref V3, T);
        Assert.Equal(aInf, V3);
    }

    [Fact]
    public void Derivatives()
    {
        PlaneSurface aPlane = new(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(0, 1, 0)));
        Assert.Equal(new Vector3D(0, 1, 0), aPlane.DN(1, 1, 1, 0));
        Assert.Equal(new Vector3D(aPlane.Position.YDirection), aPlane.DN(1, 1, 0, 1));
        Assert.Equal(new Vector3D(0, 0, 0), aPlane.DN(1, 1, 1, 1));
        EngineAssert.Throws<UndefinedDerivative3DException>(() => aPlane.DN(1, 1, 0, 0));
        EngineAssert.Throws<UndefinedDerivative3DException>(() => aPlane.DN(1, 1, -1, 2));
        aPlane.D3(1, 1, out _, out _, out _, out Vector3D D2U, out _, out _, out Vector3D D3U, out _, out _, out Vector3D D3UVV);
        Assert.Equal(new Vector3D(0, 0, 0), D2U);
        Assert.Equal(new Vector3D(0, 0, 0), D3U);
        Assert.Equal(new Vector3D(0, 0, 0), D3UVV);
    }

    [Fact]
    public void Bounds_And_Flags()
    {
        PlaneSurface aPlane = new(1, 2, 3, 4);
        aPlane.ParameterBounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), U1);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), U2);
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), V1);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), V2);
        Assert.False(aPlane.IsUClosed || aPlane.IsVClosed || aPlane.IsUPeriodic || aPlane.IsVPeriodic);
        Assert.Equal(Continuity.CN, aPlane.Continuity);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new PlaneSurface(0, 0, 0, 1));
    }

    [Fact]
    public void Frozen_Plane_Refuses_Mutation_But_Twins_Work()
    {
        PlaneSurface aPlane = new(new CoordinateSystem(Constants.XOY()));
        aPlane.Freeze();
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aPlane.SetPln(new Plane3D(new CoordinateSystem(Constants.YOZ()))));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aPlane.Translate(new Vector3D(1, 0, 0)));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aPlane.UReverse());
        PlaneSurface aOther = aPlane.WithPlane(new Plane3D(new CoordinateSystem(Constants.YOZ())));
        Assert.False(aOther.IsFrozen);
        Assert.Equal(Constants.YOZ().Direction, aOther.Axis.Direction);
        Assert.Equal(Constants.XOY().Direction, aPlane.Axis.Direction);
        ElementarySurface aLocated = aPlane.WithLocation(new Point3D(0, 0, 7));
        Assert.Equal(7.0, aLocated.Location.Z);
    }
}
