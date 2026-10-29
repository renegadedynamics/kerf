// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own checks of Geom_SurfaceOfRevolution, Geom_SurfaceOfLinearExtrusion and
/// Geom_RectangularTrimmedSurface (OCCT has no GTests for them beyond Geom_SurfaceEval_Test), on TestCircle and
/// TestLine as basis curves.
/// </summary>
public class Geom_SweptAndTrimmedSurfacesTests
{
    private static TestCircle Circle() => new(new gp_Ax2(new gp_Pnt(3, 0, 0), new gp_Dir(0, 1, 0)), 1.0);

    private static Geom_SurfaceOfRevolution Torus() => new(Circle(), new gp_Ax1(new gp_Pnt(), new gp_Dir(0, 0, 1)));

    [Fact]
    public void Revolution_Of_A_Circle_Is_A_Torus()
    {
        Geom_SurfaceOfRevolution S = Torus();
        Geom_ToroidalSurface aRef = new(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1)), 3, 1);
        Assert.True(S.IsUPeriodic() && S.IsUClosed() && S.IsVPeriodic() && S.IsVClosed());
        S.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.0, 2 * M_PI, 0.0, 2 * M_PI), (U1, U2, V1, V2));
        foreach ((double U, double V) in new[] { (0.3, 0.2), (2.0, 4.1), (5.5, 0.9) })
        {
            gp_Pnt P = S.Value(U, V);
            // Distance to the core circle is the minor radius
            double aRho = Math.Sqrt(P.X() * P.X() + P.Y() * P.Y());
            Assert.Equal(1.0, Math.Sqrt((aRho - 3) * (aRho - 3) + P.Z() * P.Z()), 1e-12);
            // Same U iso-plane as the torus's
            gp_Pnt aT = aRef.Value(U, 0.0);
            Assert.Equal(0.0, Math.Abs(P.X() * aT.Y() - P.Y() * aT.X()), 1e-12);
        }
        Assert.Equal(S.Axis().Direction(), new gp_Dir(0, 0, 1));
        Assert.Throws<Standard_NotImplemented>(() => S.ReferencePlane());
    }

    [Fact]
    public void Revolution_Derivatives_Are_Consistent()
    {
        Geom_SurfaceOfRevolution S = Torus();
        double U = 0.7, V = 1.3;
        S.D3(U, V, out gp_Pnt P, out gp_Vec D1U, out gp_Vec D1V, out gp_Vec D2U, out gp_Vec D2V, out gp_Vec D2UV,
             out gp_Vec D3U, out gp_Vec D3V, out gp_Vec D3UUV, out gp_Vec D3UVV);
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
        Assert.True(S.DN(U, V, 4, 0).IsEqual(new gp_Vec(new gp_Pnt(0, 0, P.Z()), P), 1e-12, 1e-12));
        double h = 1e-6;
        Assert.True((new gp_Vec(S.Value(U - h, V), S.Value(U + h, V)) / (2 * h)).IsEqual(D1U, 1e-6, 1e-6));
        Assert.True((new gp_Vec(S.Value(U, V - h), S.Value(U, V + h)) / (2 * h)).IsEqual(D1V, 1e-6, 1e-6));
        Assert.Throws<Geom_UndefinedDerivative>(() => S.DN(U, V, 0, 0));
    }

    [Fact]
    public void Revolution_On_The_Axis_Has_No_U_Derivative()
    {
        TestLine aLine = new(new gp_Pnt(0, 0, 0), new gp_Vec(1, 0, 1));
        Geom_SurfaceOfRevolution S = new(aLine, new gp_Ax1(new gp_Pnt(), new gp_Dir(0, 0, 1)));
        Assert.Equal(new gp_Vec(0, 0, 0), S.EvalD1(1.0, 0.0).D1U);
    }

    [Fact]
    public void Revolution_UIso_Rotates_The_Basis_And_Reversal_Keeps_The_Points()
    {
        Geom_SurfaceOfRevolution S = Torus();
        Geom_Curve aIso = S.UIso(0.8);
        Assert.True(aIso.Value(1.1).IsEqual(S.Value(0.8, 1.1), 1e-12));
        Assert.Throws<Standard_NotImplemented>(() => S.VIso(0.5)); // TODO(1A): Geom_Circle

        Geom_Surface aU = S.UReversed();
        Geom_Surface aV = S.VReversed();
        Assert.True(aU.Value(S.UReversedParameter(0.8), 1.1).IsEqual(S.Value(0.8, 1.1), 1e-12));
        Assert.True(aV.Value(0.8, S.VReversedParameter(1.1)).IsEqual(S.Value(0.8, 1.1), 1e-12));
    }

    [Fact]
    public void Revolution_Transform_With_Mirror_Reverses_U()
    {
        Geom_SurfaceOfRevolution S = Torus();
        gp_Trsf T = new();
        T.SetMirror(new gp_Ax2(new gp_Pnt(0, 0, 1), new gp_Dir(1, 0, 0)));
        Geom_SurfaceOfRevolution M = (Geom_SurfaceOfRevolution)S.Transformed(T);
        // A mirror turns the rotation the other way; UReverse (the axis reversed) restores the parametrization
        double U = 0.8, V = 1.1;
        Assert.True(M.Value(U, V).IsEqual(S.Value(U, V).Transformed(T), 1e-12));
        Assert.True(M.Axis().Direction().IsOpposite(S.Axis().Direction().Transformed(T), 1e-15));
    }

    [Fact]
    public void Extrusion_Of_A_Circle_Is_A_Cylinder()
    {
        TestCircle C = new(new gp_Ax2(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 2.0);
        Geom_SurfaceOfLinearExtrusion S = new(C, new gp_Dir(0, 0, 1));
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Ax2(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1))), 2.0);
        foreach ((double U, double V) in new[] { (0.3, 0.2), (2.0, -4.1), (5.5, 0.9) })
        {
            Assert.True(S.Value(U, V).IsEqual(aCyl.Value(U, V), 1e-12));
            Assert.True(S.DN(U, V, 2, 0).IsEqual(aCyl.DN(U, V, 2, 0), 1e-12, 1e-12));
        }
        Assert.True(S.IsUClosed() && S.IsUPeriodic() && !S.IsVClosed() && !S.IsVPeriodic());
        Assert.Equal(new gp_Vec(0, 0, 1), S.DN(1, 1, 0, 1));
        Assert.Equal(new gp_Vec(0, 0, 0), S.DN(1, 1, 1, 1));
        Assert.Equal(new gp_Vec(0, 0, 0), S.DN(1, 1, 0, 2));

        Geom_Curve aVIso = S.VIso(4.0);
        Assert.True(aVIso.Value(1.0).IsEqual(S.Value(1.0, 4.0), 1e-12));
        Assert.Throws<Standard_NotImplemented>(() => S.UIso(0.5)); // TODO(1A): Geom_Line

        double U2 = 1.0, V2 = 3.0;
        gp_Trsf T = gp_Trsf.Scale(new gp_Pnt(), 3.0);
        S.TransformParameters(ref U2, ref V2, T);
        Assert.Equal((1.0, 9.0), (U2, V2));
        Geom_Surface aMoved = (Geom_Surface)S.Transformed(T);
        Assert.True(aMoved.Value(U2, V2).IsEqual(S.Value(1.0, 3.0).Transformed(T), 1e-12));
    }

    [Fact]
    public void Swept_Surfaces_Copy_Their_Basis_And_Freeze()
    {
        TestCircle C = Circle();
        Geom_SurfaceOfLinearExtrusion E = new(C, new gp_Dir(0, 0, 1));
        Geom_SurfaceOfRevolution R = new(C, new gp_Ax1(new gp_Pnt(), new gp_Dir(0, 0, 1)));
        Assert.NotSame(C, E.BasisCurve());
        Assert.NotSame(C, R.BasisCurve());
        Assert.Equal(GeomAbs_Shape.GeomAbs_CN, E.Continuity());

        E.Freeze();
        R.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => E.SetDirection(new gp_Dir(1, 0, 0)));
        Assert.Throws<Standard_ImmutableObject>(() => R.SetAxis(new gp_Ax1()));
        Assert.Throws<Standard_ImmutableObject>(() => R.SetLocation(new gp_Pnt()));
        Assert.Throws<Standard_ImmutableObject>(() => E.UReverse());
        Assert.Throws<Standard_ImmutableObject>(() => R.VReverse());

        Assert.Equal(new gp_Dir(1, 0, 0), E.WithDirection(new gp_Dir(1, 0, 0)).Direction());
        Assert.Equal(new gp_Pnt(1, 1, 1), R.WithLocation(new gp_Pnt(1, 1, 1)).Location());
        Assert.Equal(new gp_Dir(0, 1, 0), R.WithDirection(new gp_Dir(0, 1, 0)).Direction());
        Assert.Equal(new gp_Dir(1, 0, 0), R.WithAxis(new gp_Ax1(new gp_Pnt(), new gp_Dir(1, 0, 0))).Axis().Direction());
        TestLine L = new(new gp_Pnt(), new gp_Vec(1, 0, 0));
        Assert.IsType<TestLine>(E.WithBasisCurve(L).BasisCurve());
        Assert.IsType<TestLine>(R.WithBasisCurve(L).BasisCurve());
        Assert.Equal(new gp_Dir(0, 0, 1), E.Direction());
    }

    [Fact]
    public void Trimmed_Surface_Trims_And_Orients()
    {
        Geom_SphericalSurface aSph = new(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1)), 2.0);
        Geom_RectangularTrimmedSurface T = new(aSph, 0.5, 2.0, -0.5, 1.0);
        T.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.5, 2.0, -0.5, 1.0), (U1, U2, V1, V2));
        Assert.False(T.IsUClosed());
        Assert.False(T.IsUPeriodic());
        Assert.Equal(2 * M_PI, T.UPeriod());
        Assert.Equal(aSph.Value(1, 0.2), T.Value(1, 0.2));
        Assert.NotSame(aSph, T.BasisSurface());

        // Periodic direction: U1 > U2 is adjusted by a period, not reversed
        Geom_RectangularTrimmedSurface T2 = new(aSph, 6.0, 1.0, true);
        T2.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal(6.0, U1, 1e-15);
        Assert.Equal(1.0 + 2 * M_PI, U2, 1e-15);
        Assert.Equal((-M_PI / 2, M_PI / 2), (V1, V2));

        // Non-periodic direction with V1 > V2: the surface is V-reversed
        Geom_RectangularTrimmedSurface T3 = new(aSph, 1.0, -0.5, false);
        T3.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((-1.0, 0.5), (V1, V2));
        Assert.True(T3.Value(1.0, T3.VReversedParameter(0.3)).IsEqual(aSph.Value(1.0, 0.3), 1e-12));

        // A full period is closed and periodic again
        Geom_RectangularTrimmedSurface T4 = new(aSph, 1.0, 1.0 + 2 * M_PI, true);
        Assert.True(T4.IsUClosed());
        Assert.True(T4.IsUPeriodic());

        Assert.Throws<Standard_ConstructionError>(() => new Geom_RectangularTrimmedSurface(aSph, 1.0, 1.0, true));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_RectangularTrimmedSurface(aSph, 0.0, 2.0, false));
    }

    [Fact]
    public void Trimmed_Of_Trimmed_Keeps_The_Other_Direction()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1)), 1.0);
        Geom_RectangularTrimmedSurface T = new(aCyl, -1.0, 3.0, false);
        Geom_RectangularTrimmedSurface T2 = new(T, 0.5, 1.5, true);
        T2.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.5, 1.5, -1.0, 3.0), (U1, U2, V1, V2));
        Assert.IsType<Geom_CylindricalSurface>(T2.BasisSurface());

        Geom_RectangularTrimmedSurface aCopy = (Geom_RectangularTrimmedSurface)T2.Copy();
        aCopy.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((0.5, 1.5, -1.0, 3.0), (U1, U2, V1, V2));
        Geom_RectangularTrimmedSurface aVOnly = (Geom_RectangularTrimmedSurface)T.Copy();
        aVOnly.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((0.0, 2 * M_PI, -1.0, 3.0), (U1, U2, V1, V2));

    }

    [Fact]
    public void Trimmed_Transform_And_Reverse_Move_The_Trims()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1)), 1.0);
        Geom_RectangularTrimmedSurface T = new(aCyl, 0.5, 2.0, -1.0, 3.0);
        Geom_RectangularTrimmedSurface aScaled = (Geom_RectangularTrimmedSurface)T.Transformed(gp_Trsf.Scale(new gp_Pnt(), 2.0));
        aScaled.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.5, 2.0, -2.0, 6.0), (U1, U2, V1, V2));

        Geom_RectangularTrimmedSurface aU = (Geom_RectangularTrimmedSurface)T.UReversed();
        aU.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal(2 * M_PI - 2.0, U1, 1e-14);
        Assert.Equal(2 * M_PI - 0.5, U2, 1e-14);
        Assert.True(aU.Value(T.UReversedParameter(1.0), 1.0).IsEqual(T.Value(1.0, 1.0), 1e-12));

        Geom_RectangularTrimmedSurface aV = (Geom_RectangularTrimmedSurface)T.VReversed();
        aV.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((-3.0, 1.0), (V1, V2));

        T.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => T.SetTrim(0.1, 0.2, true));
        Geom_RectangularTrimmedSurface aRetrimmed = T.WithTrim(0.1, 0.2, true);
        aRetrimmed.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((0.1, 0.2, -1.0, 3.0), (U1, U2, V1, V2));
        Geom_RectangularTrimmedSurface aBoth = T.WithTrim(0.1, 0.2, 0.3, 0.4);
        aBoth.Bounds(out U1, out U2, out V1, out V2);
        Assert.Equal((0.1, 0.2, 0.3, 0.4), (U1, U2, V1, V2));
    }

    [Fact]
    public void Trimmed_IsoCurves_NotImplementedUntil1A()
    {
        // TODO(1A): the iso-curves of a trimmed direction are Geom_TrimmedCurve
        Geom_SurfaceOfRevolution S = Torus();
        Geom_RectangularTrimmedSurface T = new(S, 0.5, 2.0, true);
        Assert.Throws<Standard_NotImplemented>(() => T.VIso(0.3));
        Geom_Curve aIso = T.UIso(1.0); // V is not trimmed: the basis surface's iso-curve
        Assert.True(aIso.Value(0.4).IsEqual(S.Value(1.0, 0.4), 1e-12));
    }
}
