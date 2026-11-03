// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own checks of Geom_SurfaceOfRevolution, Geom_SurfaceOfLinearExtrusion and
/// Geom_RectangularTrimmedSurface (OCCT has no GTests for them beyond Geom_SurfaceEval_Test).
/// </summary>
public class Geom_SweptAndTrimmedSurfacesTests
{
    private static CircleCurve3D Circle() => new(new Frame3D(new Point3D(3, 0, 0), new Direction3D(0, 1, 0)), 1.0);

    private static RevolutionSurface Torus() => new(Circle(), new Axis3D(new Point3D(), new Direction3D(0, 0, 1)));

    [Fact]
    public void Revolution_Of_A_Circle_Is_A_Torus()
    {
        RevolutionSurface S = Torus();
        ToroidalSurface aRef = new(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1)), 3, 1);
        Assert.True(S.IsUPeriodic && S.IsUClosed && S.IsVPeriodic && S.IsVClosed);
        S.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.0, 2 * M_PI, 0.0, 2 * M_PI), (U1, U2, V1, V2));
        foreach ((double U, double V) in new[] { (0.3, 0.2), (2.0, 4.1), (5.5, 0.9) })
        {
            Point3D P = S.Value(U, V);
            // Distance to the core circle is the minor radius
            double aRho = Math.Sqrt(P.X() * P.X() + P.Y() * P.Y());
            Assert.Equal(1.0, Math.Sqrt((aRho - 3) * (aRho - 3) + P.Z() * P.Z()), 1e-12);
            // Same U iso-plane as the torus's
            Point3D aT = aRef.Value(U, 0.0);
            Assert.Equal(0.0, Math.Abs(P.X() * aT.Y() - P.Y() * aT.X()), 1e-12);
        }
        Assert.Equal(S.Axis().Direction(), new Direction3D(0, 0, 1));
        Assert.Throws<MethodNotImplementedException>(() => S.ReferencePlane());
    }

    [Fact]
    public void Revolution_Derivatives_Are_Consistent()
    {
        RevolutionSurface S = Torus();
        double U = 0.7, V = 1.3;
        S.D3(U, V, out Point3D P, out Vector3D D1U, out Vector3D D1V, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV,
             out Vector3D D3U, out Vector3D D3V, out Vector3D D3UUV, out Vector3D D3UVV);
        Assert.Equal(P, S.Value(U, V));
        Assert.True(S.DN(U, V, 1, 0).IsEqual(D1U, 1e-12, 1e-12));
        Assert.True(S.DN(U, V, 0, 1).IsEqual(D1V, 1e-12, 1e-12));
        Assert.True(S.DN(U, V, 2, 0).IsEqual(D2U, 1e-12, 1e-12));
        Assert.True(S.DN(U, V, 0, 2).IsEqual(D2V, 1e-12, 1e-12));
        Assert.True(S.DN(U, V, 1, 1).IsEqual(D2UV, 1e-12, 1e-12));
        Assert.True(S.DN(U, V, 3, 0).IsEqual(D3U, 1e-12, 1e-12));
        Assert.True(S.DN(U, V, 0, 3).IsEqual(D3V, 1e-12, 1e-12));
        Assert.True(S.DN(U, V, 2, 1).IsEqual(D3UUV, 1e-12, 1e-12));
        Assert.True(S.DN(U, V, 1, 2).IsEqual(D3UVV, 1e-12, 1e-12));
        // The fourth U derivative is the first one's opposite's opposite: back to the radial vector
        Assert.True(S.DN(U, V, 4, 0).IsEqual(new Vector3D(new Point3D(0, 0, P.Z()), P), 1e-12, 1e-12));
        double h = 1e-6;
        Assert.True((new Vector3D(S.Value(U - h, V), S.Value(U + h, V)) / (2 * h)).IsEqual(D1U, 1e-6, 1e-6));
        Assert.True((new Vector3D(S.Value(U, V - h), S.Value(U, V + h)) / (2 * h)).IsEqual(D1V, 1e-6, 1e-6));
        Assert.Throws<UndefinedDerivative3DException>(() => S.DN(U, V, 0, 0));
    }

    [Fact]
    public void Revolution_On_The_Axis_Has_No_U_Derivative()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 1));
        RevolutionSurface S = new(aLine, new Axis3D(new Point3D(), new Direction3D(0, 0, 1)));
        Assert.Equal(new Vector3D(0, 0, 0), S.EvalD1(1.0, 0.0).D1U);
    }

    [Fact]
    public void Revolution_UIso_Rotates_The_Basis_And_Reversal_Keeps_The_Points()
    {
        RevolutionSurface S = Torus();
        Curve3D aIso = S.UIso(0.8);
        Assert.True(aIso.Value(1.1).IsEqual(S.Value(0.8, 1.1), 1e-12));
        // The V iso-curve: the circle described by the basis curve's point, centred on the axis
        CircleCurve3D aVIso = Assert.IsType<CircleCurve3D>(S.VIso(0.5));
        Assert.True(aVIso.Location().IsEqual(new Point3D(0, 0, Circle().Value(0.5).Z()), 1e-15));
        Assert.True(aVIso.Axis().Direction().IsEqual(S.Axis().Direction(), 0.0));
        foreach (double U in new[] { 0.0, 0.8, 4.0 })
        {
            Assert.True(aVIso.Value(U).IsEqual(S.Value(U, 0.5), 1e-12));
        }
        // A point on the axis: a circle of radius 0 at the point
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 1));
        RevolutionSurface aCone = new(aLine, new Axis3D(new Point3D(), new Direction3D(0, 0, 1)));
        Assert.Equal(0.0, ((CircleCurve3D)aCone.VIso(0.0)).Radius());

        Surface aU = S.UReversed();
        Surface aV = S.VReversed();
        Assert.True(aU.Value(S.UReversedParameter(0.8), 1.1).IsEqual(S.Value(0.8, 1.1), 1e-12));
        Assert.True(aV.Value(0.8, S.VReversedParameter(1.1)).IsEqual(S.Value(0.8, 1.1), 1e-12));
    }

    [Fact]
    public void Revolution_Transform_With_Mirror_Reverses_U()
    {
        RevolutionSurface S = Torus();
        Transform3D T = new();
        T.SetMirror(new Frame3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 0)));
        RevolutionSurface M = (RevolutionSurface)S.Transformed(T);
        // A mirror turns the rotation the other way; UReverse (the axis reversed) restores the parametrization
        double U = 0.8, V = 1.1;
        Assert.True(M.Value(U, V).IsEqual(S.Value(U, V).Transformed(T), 1e-12));
        Assert.True(M.Axis().Direction().IsOpposite(S.Axis().Direction().Transformed(T), 1e-15));
    }

    [Fact]
    public void Extrusion_Of_A_Circle_Is_A_Cylinder()
    {
        CircleCurve3D C = new(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 2.0);
        ExtrusionSurface S = new(C, new Direction3D(0, 0, 1));
        CylindricalSurface aCyl = new(new CoordinateSystem(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1))), 2.0);
        foreach ((double U, double V) in new[] { (0.3, 0.2), (2.0, -4.1), (5.5, 0.9) })
        {
            Assert.True(S.Value(U, V).IsEqual(aCyl.Value(U, V), 1e-12));
            Assert.True(S.DN(U, V, 2, 0).IsEqual(aCyl.DN(U, V, 2, 0), 1e-12, 1e-12));
        }
        Assert.True(S.IsUClosed && S.IsUPeriodic && !S.IsVClosed && !S.IsVPeriodic);
        Assert.Equal(new Vector3D(0, 0, 1), S.DN(1, 1, 0, 1));
        Assert.Equal(new Vector3D(0, 0, 0), S.DN(1, 1, 1, 1));
        Assert.Equal(new Vector3D(0, 0, 0), S.DN(1, 1, 0, 2));

        Curve3D aVIso = S.VIso(4.0);
        Assert.True(aVIso.Value(1.0).IsEqual(S.Value(1.0, 4.0), 1e-12));
        LineCurve3D aUIso = Assert.IsType<LineCurve3D>(S.UIso(0.5));
        Assert.Equal(new Direction3D(0, 0, 1), aUIso.Position().Direction());
        Assert.True(aUIso.Value(-2.5).IsEqual(S.Value(0.5, -2.5), 1e-12));

        double U2 = 1.0, V2 = 3.0;
        Transform3D T = Transform3D.Scale(new Point3D(), 3.0);
        S.TransformParameters(ref U2, ref V2, T);
        Assert.Equal((1.0, 9.0), (U2, V2));
        Surface aMoved = (Surface)S.Transformed(T);
        Assert.True(aMoved.Value(U2, V2).IsEqual(S.Value(1.0, 3.0).Transformed(T), 1e-12));
    }

    [Fact]
    public void Swept_Surfaces_Copy_Their_Basis_And_Freeze()
    {
        CircleCurve3D C = Circle();
        ExtrusionSurface E = new(C, new Direction3D(0, 0, 1));
        RevolutionSurface R = new(C, new Axis3D(new Point3D(), new Direction3D(0, 0, 1)));
        Assert.NotSame(C, E.BasisCurve());
        Assert.NotSame(C, R.BasisCurve());
        Assert.Equal(Continuity.CN, E.Continuity());

        E.Freeze();
        R.Freeze();
        Assert.Throws<ImmutableObjectException>(() => E.SetDirection(new Direction3D(1, 0, 0)));
        Assert.Throws<ImmutableObjectException>(() => R.SetAxis(new Axis3D()));
        Assert.Throws<ImmutableObjectException>(() => R.SetLocation(new Point3D()));
        Assert.Throws<ImmutableObjectException>(() => E.UReverse());
        Assert.Throws<ImmutableObjectException>(() => R.VReverse());

        Assert.Equal(new Direction3D(1, 0, 0), E.WithDirection(new Direction3D(1, 0, 0)).Direction());
        Assert.Equal(new Point3D(1, 1, 1), R.WithLocation(new Point3D(1, 1, 1)).Location());
        Assert.Equal(new Direction3D(0, 1, 0), R.WithDirection(new Direction3D(0, 1, 0)).Direction());
        Assert.Equal(new Direction3D(1, 0, 0), R.WithAxis(new Axis3D(new Point3D(), new Direction3D(1, 0, 0))).Axis().Direction());
        LineCurve3D L = new(new Point3D(), new Direction3D(1, 0, 0));
        Assert.IsType<LineCurve3D>(E.WithBasisCurve(L).BasisCurve());
        Assert.IsType<LineCurve3D>(R.WithBasisCurve(L).BasisCurve());
        Assert.Equal(new Direction3D(0, 0, 1), E.Direction());
    }

    [Fact]
    public void Trimmed_Surface_Trims_And_Orients()
    {
        SphericalSurface aSph = new(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1)), 2.0);
        TrimmedSurface T = new(aSph, 0.5, 2.0, -0.5, 1.0);
        T.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.5, 2.0, -0.5, 1.0), (U1, U2, V1, V2));
        Assert.False(T.IsUClosed);
        Assert.False(T.IsUPeriodic);
        Assert.Equal(2 * M_PI, T.UPeriod());
        Assert.Equal(aSph.Value(1, 0.2), T.Value(1, 0.2));
        Assert.NotSame(aSph, T.BasisSurface());

        // Periodic direction: U1 > U2 is adjusted by a period, not reversed
        TrimmedSurface T2 = new(aSph, 6.0, 1.0, true);
        T2.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal(6.0, U1, 1e-15);
        Assert.Equal(1.0 + 2 * M_PI, U2, 1e-15);
        Assert.Equal((-M_PI / 2, M_PI / 2), (V1, V2));

        // Non-periodic direction with V1 > V2: the surface is V-reversed
        TrimmedSurface T3 = new(aSph, 1.0, -0.5, false);
        T3.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((-1.0, 0.5), (V1, V2));
        Assert.True(T3.Value(1.0, T3.VReversedParameter(0.3)).IsEqual(aSph.Value(1.0, 0.3), 1e-12));

        // A full period is closed and periodic again
        TrimmedSurface T4 = new(aSph, 1.0, 1.0 + 2 * M_PI, true);
        Assert.True(T4.IsUClosed);
        Assert.True(T4.IsUPeriodic);

        Assert.Throws<ConstructionErrorException>(() => new TrimmedSurface(aSph, 1.0, 1.0, true));
        Assert.Throws<ConstructionErrorException>(() => new TrimmedSurface(aSph, 0.0, 2.0, false));
    }

    [Fact]
    public void Trimmed_Of_Trimmed_Keeps_The_Other_Direction()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1)), 1.0);
        TrimmedSurface T = new(aCyl, -1.0, 3.0, false);
        TrimmedSurface T2 = new(T, 0.5, 1.5, true);
        T2.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.5, 1.5, -1.0, 3.0), (U1, U2, V1, V2));
        Assert.IsType<CylindricalSurface>(T2.BasisSurface());

        TrimmedSurface aCopy = (TrimmedSurface)T2.Copy();
        aCopy.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((0.5, 1.5, -1.0, 3.0), (U1, U2, V1, V2));
        TrimmedSurface aVOnly = (TrimmedSurface)T.Copy();
        aVOnly.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((0.0, 2 * M_PI, -1.0, 3.0), (U1, U2, V1, V2));

    }

    [Fact]
    public void Trimmed_Transform_And_Reverse_Move_The_Trims()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1)), 1.0);
        TrimmedSurface T = new(aCyl, 0.5, 2.0, -1.0, 3.0);
        TrimmedSurface aScaled = (TrimmedSurface)T.Transformed(Transform3D.Scale(new Point3D(), 2.0));
        aScaled.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.5, 2.0, -2.0, 6.0), (U1, U2, V1, V2));

        TrimmedSurface aU = (TrimmedSurface)T.UReversed();
        aU.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal(2 * M_PI - 2.0, U1, 1e-14);
        Assert.Equal(2 * M_PI - 0.5, U2, 1e-14);
        Assert.True(aU.Value(T.UReversedParameter(1.0), 1.0).IsEqual(T.Value(1.0, 1.0), 1e-12));

        TrimmedSurface aV = (TrimmedSurface)T.VReversed();
        aV.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((-3.0, 1.0), (V1, V2));

        T.Freeze();
        Assert.Throws<ImmutableObjectException>(() => T.SetTrim(0.1, 0.2, true));
        TrimmedSurface aRetrimmed = T.WithTrim(0.1, 0.2, true);
        aRetrimmed.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((0.1, 0.2, -1.0, 3.0), (U1, U2, V1, V2));
        TrimmedSurface aBoth = T.WithTrim(0.1, 0.2, 0.3, 0.4);
        aBoth.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((0.1, 0.2, 0.3, 0.4), (U1, U2, V1, V2));
    }

    [Fact]
    public void Trimmed_IsoCurves_Are_Trimmed_In_A_Trimmed_Direction()
    {
        RevolutionSurface S = Torus();
        TrimmedSurface T = new(S, 0.5, 2.0, true);
        TrimmedCurve3D aVIso = Assert.IsType<TrimmedCurve3D>(T.VIso(0.3));
        Assert.IsType<CircleCurve3D>(aVIso.BasisCurve());
        Assert.Equal((0.5, 2.0), (aVIso.FirstParameter(), aVIso.LastParameter()));
        Assert.True(aVIso.Value(1.2).IsEqual(S.Value(1.2, 0.3), 1e-12));
        Curve3D aIso = T.UIso(1.0); // V is not trimmed: the basis surface's iso-curve
        Assert.IsType<CircleCurve3D>(aIso);
        Assert.True(aIso.Value(0.4).IsEqual(S.Value(1.0, 0.4), 1e-12));

        TrimmedSurface T2 = new(S, 0.5, 2.0, 1.0, 3.0);
        TrimmedCurve3D aUIso = Assert.IsType<TrimmedCurve3D>(T2.UIso(1.0));
        Assert.Equal((1.0, 3.0), (aUIso.FirstParameter(), aUIso.LastParameter()));
        Assert.True(aUIso.Value(2.5).IsEqual(S.Value(1.0, 2.5), 1e-12));
    }

    //=============================================================================================
    // The shared basis (decision 159): handed out frozen, copied by the owner's own mutators
    //=============================================================================================

    [Fact]
    public void Trimmed_BasisSurface_Is_Frozen_And_Shared()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1)), 1.0);
        TrimmedSurface T = new(aCyl, 0.5, 2.0, -1.0, 3.0);
        Surface aBasis = T.BasisSurface();
        Assert.True(aBasis.IsFrozen);
        Assert.Same(aBasis, T.BasisSurface());
        Assert.False(T.IsFrozen);
        Assert.False(aCyl.IsFrozen);
        Assert.Throws<ImmutableObjectException>(() => aBasis.UReverse());

        // A trimmed surface of T copies T's basis without freezing it
        TrimmedSurface aT2 = new(T, 0.6, 1.0, true);
        Assert.False(new TrimmedSurface(aT2, 0.0, 1.0, false).IsFrozen);
        Assert.NotSame(aBasis, aT2.BasisSurface());

        // The owner's mutators copy the frozen basis; the handed-out one doesn't change
        Point3D aBefore = aBasis.Value(1.0, 1.0);
        double aU = T.UReversedParameter(1.0), aV = T.VReversedParameter(1.0);
        Point3D aExpected = T.Value(1.0, 1.0);
        T.UReverse();
        Assert.NotSame(aBasis, T.BasisSurface());
        Assert.True(T.Value(aU, 1.0).IsEqual(aExpected, 1e-12));
        T.VReverse();
        Assert.True(T.Value(aU, aV).IsEqual(aExpected, 1e-12));
        Transform3D aMove = Transform3D.Translation(new Vector3D(1, 2, 3));
        Surface aBasis2 = T.BasisSurface();
        T.Transform(aMove);
        Assert.NotSame(aBasis2, T.BasisSurface());
        Assert.True(T.Value(aU, aV).IsEqual(aExpected.Translated(new Vector3D(1, 2, 3)), 1e-12));
        Assert.Equal(aBefore, aBasis.Value(1.0, 1.0));
    }

    [Fact]
    public void Swept_BasisCurve_Is_Frozen_And_Shared()
    {
        CircleCurve3D C = Circle();
        RevolutionSurface R = new(C, new Axis3D(new Point3D(), new Direction3D(0, 0, 1)));
        ExtrusionSurface E = new(C, new Direction3D(0, 0, 1));
        Curve3D aRBasis = R.BasisCurve();
        Curve3D aEBasis = E.BasisCurve();
        Assert.True(aRBasis.IsFrozen && aEBasis.IsFrozen);
        Assert.Same(aRBasis, R.BasisCurve());
        Assert.Same(aEBasis, E.BasisCurve());
        Assert.False(C.IsFrozen || R.IsFrozen || E.IsFrozen);
        Assert.Throws<ImmutableObjectException>(() => aRBasis.Reverse());

        Point3D aRBefore = aRBasis.Value(1.0);
        Point3D aEBefore = aEBasis.Value(1.0);
        Point3D aRExpected = R.Value(0.7, 1.0);
        Point3D aEExpected = E.Value(1.0, 0.7);

        // Revolution: VReverse and Transform change the basis curve
        R.VReverse();
        Assert.NotSame(aRBasis, R.BasisCurve());
        Assert.True(R.Value(0.7, R.VReversedParameter(1.0)).IsEqual(aRExpected, 1e-12));
        Curve3D aRBasis2 = R.BasisCurve();
        Transform3D aMirror = new();
        aMirror.SetMirror(new Frame3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 0)));
        R.Transform(aMirror);
        Assert.NotSame(aRBasis2, R.BasisCurve());
        Assert.Equal(aRBefore, aRBasis.Value(1.0));

        // Extrusion: UReverse and Transform change the basis curve
        E.UReverse();
        Assert.NotSame(aEBasis, E.BasisCurve());
        Assert.True(E.Value(E.UReversedParameter(1.0), 0.7).IsEqual(aEExpected, 1e-12));
        Curve3D aEBasis2 = E.BasisCurve();
        E.Transform(Transform3D.Scale(new Point3D(), 2.0));
        Assert.NotSame(aEBasis2, E.BasisCurve());
        Assert.Equal(aEBefore, aEBasis.Value(1.0));

        // The iso-curves copy the basis curve: they are not frozen, and don't share it
        Curve3D aUIso = R.UIso(0.3);
        Assert.False(aUIso.IsFrozen);
        Assert.NotSame(R.BasisCurve(), aUIso);
        Assert.False(E.VIso(0.3).IsFrozen);

    }
}
