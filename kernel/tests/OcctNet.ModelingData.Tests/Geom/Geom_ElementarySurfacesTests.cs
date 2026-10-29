// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own checks of Geom_CylindricalSurface, Geom_ConicalSurface, Geom_SphericalSurface and
/// Geom_ToroidalSurface (OCCT has no GTests for them; the oracle tests compare them with native OCCT).
/// </summary>
public class Geom_ElementarySurfacesTests
{
    private static readonly gp_Ax3 Frame = new(new gp_Pnt(1, -2, 3), new gp_Dir(1, 2, 2), new gp_Dir(2, -1, 0));

    private static Geom_ElementarySurface[] All() =>
    [
        new Geom_CylindricalSurface(Frame, 2.5),
        new Geom_ConicalSurface(Frame, 0.4, 1.5),
        new Geom_SphericalSurface(Frame, 3.0),
        new Geom_ToroidalSurface(Frame, 5.0, 1.25),
    ];

    [Fact]
    public void Constructors_Check_Their_Arguments()
    {
        Assert.Throws<Standard_ConstructionError>(() => new Geom_CylindricalSurface(Frame, -1));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_ConicalSurface(Frame, 0.4, -1));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_ConicalSurface(Frame, 0.0, 1));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_ConicalSurface(Frame, M_PI / 2, 1));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_SphericalSurface(Frame, -1));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_ToroidalSurface(Frame, -1, 1));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_ToroidalSurface(Frame, 1, -1));
        // OCCT's constructor accepts MinorRadius > MajorRadius; SetMajorRadius and SetMinorRadius don't
        Geom_ToroidalSurface aTorus = new(Frame, 1, 2);
        Assert.Throws<Standard_ConstructionError>(() => aTorus.WithMajorRadius(1.5));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_ToroidalSurface(Frame, 3, 1).WithMinorRadius(3));
    }

    [Fact]
    public void From_gp_And_Back()
    {
        gp_Cylinder aCyl = new(Frame, 2);
        Assert.Equal(2.0, new Geom_CylindricalSurface(aCyl).Cylinder().Radius());
        gp_Cone aCone = new(Frame, 0.3, 4);
        Geom_ConicalSurface aGCone = new(aCone);
        Assert.Equal(0.3, aGCone.Cone().SemiAngle());
        Assert.Equal(4.0, aGCone.RefRadius());
        Assert.True(aGCone.Apex().IsEqual(aCone.Apex(), 1e-12));
        Assert.Equal(7.0, new Geom_SphericalSurface(new gp_Sphere(Frame, 7)).Sphere().Radius());
        Geom_ToroidalSurface aTor = new(new gp_Torus(Frame, 6, 2));
        Assert.Equal(6.0, aTor.Torus().MajorRadius());
        Assert.Equal(2.0, aTor.MinorRadius());
        Assert.Equal(4.0 * M_PI * M_PI * 12.0, aTor.Area(), 1e-12);
        Assert.Equal(M_PI * 4.0 * 2.0 * M_PI * 6.0, aTor.Volume(), 1e-12);
        Geom_SphericalSurface aSph = new(Frame, 2);
        Assert.Equal(16.0 * M_PI, aSph.Area(), 1e-12);
        Assert.Equal(32.0 * M_PI / 3.0, aSph.Volume(), 1e-12);
    }

    [Fact]
    public void Points_Satisfy_The_Implicit_Equations()
    {
        Geom_CylindricalSurface aCyl = new(Frame, 2.5);
        Geom_ConicalSurface aCone = new(Frame, 0.4, 1.5);
        Geom_SphericalSurface aSph = new(Frame, 3.0);
        foreach ((double U, double V) in new[] { (0.3, 0.2), (2.0, -1.1), (5.5, 0.9) })
        {
            Assert.Equal(0.0, Quadric(aCyl.Coefficients, aCyl.Value(U, V)), 1e-11);
            Assert.Equal(0.0, Quadric(aSph.Coefficients, aSph.Value(U, V)), 1e-11);
        }

        // OCCT: Geom_ConicalSurface::Coefficients has wrong linear and constant terms (occt-issues.md); they are
        // right only when the local origin is the global one and the reference radius is 0, where gp_Cone agrees.
        Geom_ConicalSurface aBadCone = new(Frame, 0.4, 1.5);
        Assert.True(Math.Abs(Quadric(aBadCone.Coefficients, aBadCone.Value(0.3, 0.2))) > 1e-3);
        Geom_ConicalSurface aApexCone = new(new gp_Ax3(new gp_Pnt(), new gp_Dir(1, 2, 2), new gp_Dir(2, -1, 0)), 0.4, 0.0);
        Assert.Equal(0.0, Quadric(aApexCone.Coefficients, aApexCone.Value(0.3, 0.2)), 1e-12);
        gp_Cone aGpCone = aBadCone.Cone();
        Assert.Equal(0.0, Quadric(aGpCone.Coefficients, aBadCone.Value(0.3, 0.2)), 1e-11);

        Geom_ToroidalSurface aTor = new(Frame, 5.0, 1.25);
        NCollection_Array1<double> aCoef = new(1, 35);
        aTor.Coefficients(aCoef);
        double[] aExpected = new double[35];
        aTor.Torus().Coefficients(aExpected);
        Assert.Equal(aExpected, aCoef.AsSpan().ToArray());
    }

    private delegate void QuadricCoefficients(out double A1, out double A2, out double A3, out double B1, out double B2,
                                              out double B3, out double C1, out double C2, out double C3, out double D);

    private static double Quadric(QuadricCoefficients theCoefs, gp_Pnt P)
    {
        theCoefs(out double A1, out double A2, out double A3, out double B1, out double B2, out double B3,
                 out double C1, out double C2, out double C3, out double D);
        double X = P.X(), Y = P.Y(), Z = P.Z();
        return A1 * X * X + A2 * Y * Y + A3 * Z * Z + 2 * (B1 * X * Y + B2 * X * Z + B3 * Y * Z)
               + 2 * (C1 * X + C2 * Y + C3 * Z) + D;
    }

    [Fact]
    public void Bounds_Periodicity_And_Reversed_Parameters()
    {
        foreach (Geom_ElementarySurface S in All())
        {
            S.Bounds(out double U1, out double U2, out double V1, out double V2);
            Assert.Equal(0.0, U1);
            Assert.Equal(2 * M_PI, U2);
            Assert.True(S.IsUClosed());
            Assert.True(S.IsUPeriodic());
            Assert.Equal(2 * M_PI, S.UPeriod());
            Assert.Equal(S is Geom_ToroidalSurface, S.IsVPeriodic());
            Assert.Equal(S is Geom_ToroidalSurface, S.IsVClosed());
            if (S is Geom_SphericalSurface)
            {
                Assert.Equal(-M_PI / 2, V1);
                Assert.Equal(M_PI / 2, V2);
            }

            // Reversing a direction keeps the point set: (U, V) maps to the reversed parameters
            Geom_Surface aU = S.UReversed();
            Geom_Surface aV = S.VReversed();
            foreach ((double U, double V) in new[] { (0.3, 0.2), (2.0, -1.1), (5.5, 0.9) })
            {
                Assert.True(S.Value(U, V).IsEqual(aU.Value(S.UReversedParameter(U), V), 1e-12));
                Assert.True(S.Value(U, V).IsEqual(aV.Value(U, S.VReversedParameter(V)), 1e-12));
            }
        }
    }

    [Fact]
    public void Cone_VReverse_Flips_The_SemiAngle()
    {
        Geom_ConicalSurface aCone = new(Frame, 0.4, 1.5);
        Geom_ConicalSurface aRev = (Geom_ConicalSurface)aCone.VReversed();
        Assert.Equal(-0.4, aRev.SemiAngle());
        Assert.True(aRev.Apex().IsEqual(aCone.Apex(), 1e-12));
    }

    [Fact]
    public void Derivatives_Are_Consistent()
    {
        foreach (Geom_ElementarySurface S in All())
        {
            double U = 0.7, V = 0.3;
            S.D3(U, V, out gp_Pnt P, out gp_Vec D1U, out gp_Vec D1V, out gp_Vec D2U, out gp_Vec D2V, out gp_Vec D2UV,
                 out gp_Vec D3U, out gp_Vec D3V, out gp_Vec D3UUV, out gp_Vec D3UVV);
            Assert.Equal(P, S.Value(U, V));
            Assert.True(S.DN(U, V, 1, 0).IsEqual(D1U, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 0, 1).IsEqual(D1V, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 2, 0).IsEqual(D2U, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 1, 1).IsEqual(D2UV, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 3, 0).IsEqual(D3U, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 2, 1).IsEqual(D3UUV, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 1, 2).IsEqual(D3UVV, 1e-12, 1e-12));
            if (S is not Geom_CylindricalSurface and not Geom_ConicalSurface)
            {
                Assert.True(S.DN(U, V, 0, 2).IsEqual(D2V, 1e-12, 1e-12));
                Assert.True(S.DN(U, V, 0, 3).IsEqual(D3V, 1e-12, 1e-12));
            }
            else
            {
                Assert.Equal(new gp_Vec(0, 0, 0), S.DN(U, V, 0, 2));
            }

            // Finite differences of the point
            double h = 1e-6;
            gp_Vec aFdU = new gp_Vec(S.Value(U - h, V), S.Value(U + h, V)) / (2 * h);
            Assert.True(aFdU.IsEqual(D1U, 1e-6, 1e-6));
            Assert.Throws<Geom_UndefinedDerivative>(() => S.DN(U, V, 0, 0));
        }
    }

    [Fact]
    public void Transform_Scales_The_Radii_And_TransformParameters_Follows()
    {
        gp_Trsf T = gp_Trsf.Scale(new gp_Pnt(1, 1, 1), -2.0);
        foreach (Geom_ElementarySurface S in All())
        {
            Geom_Surface aMoved = (Geom_Surface)S.Transformed(T);
            double U = 0.7, V = 0.3;
            double U2 = U, V2 = V;
            S.TransformParameters(ref U2, ref V2, T);
            Assert.True(aMoved.Value(U2, V2).IsEqual(S.Value(U, V).Transformed(T), 1e-12));
            double aPU = U, aPV = V;
            S.ParametricTransformation(T).Transforms(ref aPU, ref aPV);
            Assert.Equal(U2, aPU, 1e-14);
            Assert.Equal(V2, aPV, 1e-14);
        }
        Assert.Equal(5.0, ((Geom_CylindricalSurface)new Geom_CylindricalSurface(Frame, 2.5).Transformed(T)).Radius());
        Geom_ToroidalSurface aTor = (Geom_ToroidalSurface)new Geom_ToroidalSurface(Frame, 5.0, 1.25).Transformed(T);
        Assert.Equal(10.0, aTor.MajorRadius());
        Assert.Equal(2.5, aTor.MinorRadius());
    }

    [Fact]
    public void Frozen_Surfaces_Refuse_Mutation_But_Twins_Work()
    {
        Geom_CylindricalSurface aCyl = new(Frame, 2.5);
        Geom_ConicalSurface aCone = new(Frame, 0.4, 1.5);
        Geom_SphericalSurface aSph = new(Frame, 3.0);
        Geom_ToroidalSurface aTor = new(Frame, 5.0, 1.25);
        foreach (Geom_ElementarySurface S in new Geom_ElementarySurface[] { aCyl, aCone, aSph, aTor })
        {
            S.Freeze();
            Assert.Throws<Standard_ImmutableObject>(() => S.Transform(new gp_Trsf()));
            Assert.Throws<Standard_ImmutableObject>(() => S.UReverse());
            Assert.Throws<Standard_ImmutableObject>(() => S.VReverse());
            Assert.Throws<Standard_ImmutableObject>(() => S.SetLocation(new gp_Pnt()));
            Assert.False(S.Copy().IsFrozen());
        }
        Assert.Throws<Standard_ImmutableObject>(() => aCyl.SetRadius(1));
        Assert.Throws<Standard_ImmutableObject>(() => aCone.SetSemiAngle(0.2));
        Assert.Throws<Standard_ImmutableObject>(() => aSph.SetRadius(1));
        Assert.Throws<Standard_ImmutableObject>(() => aTor.SetMinorRadius(1));

        Assert.Equal(1.0, aCyl.WithRadius(1).Radius());
        Assert.Equal(2.0, aCyl.WithCylinder(new gp_Cylinder(Frame, 2)).Radius());
        Assert.Equal(0.2, aCone.WithSemiAngle(0.2).SemiAngle());
        Assert.Equal(0.5, aCone.WithRadius(0.5).RefRadius());
        Assert.Equal(0.1, aCone.WithCone(new gp_Cone(Frame, 0.1, 1)).SemiAngle());
        Assert.Equal(1.0, aSph.WithRadius(1).Radius());
        Assert.Equal(4.0, aSph.WithSphere(new gp_Sphere(Frame, 4)).Radius());
        Assert.Equal(1.0, aTor.WithMinorRadius(1).MinorRadius());
        Assert.Equal(6.0, aTor.WithMajorRadius(6).MajorRadius());
        Assert.Equal(3.0, aTor.WithTorus(new gp_Torus(Frame, 7, 3)).MinorRadius());
        Assert.Equal(2.5, aCyl.Radius());
    }

    [Fact]
    public void IsoCurves_NotImplementedUntil1A()
    {
        // TODO(1A): UIso/VIso return Geom_Line, Geom_Circle and Geom_TrimmedCurve
        foreach (Geom_ElementarySurface S in All())
        {
            Assert.Throws<Standard_NotImplemented>(() => S.UIso(0.5));
            Assert.Throws<Standard_NotImplemented>(() => S.VIso(0.5));
        }
    }
}
