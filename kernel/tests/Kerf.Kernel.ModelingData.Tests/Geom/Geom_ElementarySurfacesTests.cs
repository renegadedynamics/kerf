// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own checks of Geom_CylindricalSurface, Geom_ConicalSurface, Geom_SphericalSurface and
/// Geom_ToroidalSurface (OCCT has no GTests for them; the oracle tests compare them with native OCCT).
/// </summary>
public class Geom_ElementarySurfacesTests
{
    private static readonly CoordinateSystem Frame = new(new Point3D(1, -2, 3), new Direction3D(1, 2, 2), new Direction3D(2, -1, 0));

    private static ElementarySurface[] All() =>
    [
        new CylindricalSurface(Frame, 2.5),
        new ConicalSurface(Frame, 0.4, 1.5),
        new SphericalSurface(Frame, 3.0),
        new ToroidalSurface(Frame, 5.0, 1.25),
    ];

    [Fact]
    public void Constructors_Check_Their_Arguments()
    {
        Assert.Throws<ConstructionErrorException>(() => new CylindricalSurface(Frame, -1));
        Assert.Throws<ConstructionErrorException>(() => new ConicalSurface(Frame, 0.4, -1));
        Assert.Throws<ConstructionErrorException>(() => new ConicalSurface(Frame, 0.0, 1));
        Assert.Throws<ConstructionErrorException>(() => new ConicalSurface(Frame, M_PI / 2, 1));
        Assert.Throws<ConstructionErrorException>(() => new SphericalSurface(Frame, -1));
        Assert.Throws<ConstructionErrorException>(() => new ToroidalSurface(Frame, -1, 1));
        Assert.Throws<ConstructionErrorException>(() => new ToroidalSurface(Frame, 1, -1));
        // OCCT's constructor accepts MinorRadius > MajorRadius; SetMajorRadius and SetMinorRadius don't
        ToroidalSurface aTorus = new(Frame, 1, 2);
        Assert.Throws<ConstructionErrorException>(() => aTorus.WithMajorRadius(1.5));
        Assert.Throws<ConstructionErrorException>(() => new ToroidalSurface(Frame, 3, 1).WithMinorRadius(3));
    }

    [Fact]
    public void From_gp_And_Back()
    {
        Cylinder aCyl = new(Frame, 2);
        Assert.Equal(2.0, new CylindricalSurface(aCyl).Cylinder.Radius);
        Cone aCone = new(Frame, 0.3, 4);
        ConicalSurface aGCone = new(aCone);
        Assert.Equal(0.3, aGCone.Cone.SemiAngle);
        Assert.Equal(4.0, aGCone.RefRadius);
        Assert.True(aGCone.Apex.IsEqual(aCone.Apex, 1e-12));
        Assert.Equal(7.0, new SphericalSurface(new Sphere(Frame, 7)).Sphere.Radius);
        ToroidalSurface aTor = new(new Torus(Frame, 6, 2));
        Assert.Equal(6.0, aTor.Torus.MajorRadius);
        Assert.Equal(2.0, aTor.MinorRadius);
        Assert.Equal(4.0 * M_PI * M_PI * 12.0, aTor.Area, 1e-12);
        Assert.Equal(M_PI * 4.0 * 2.0 * M_PI * 6.0, aTor.Volume, 1e-12);
        SphericalSurface aSph = new(Frame, 2);
        Assert.Equal(16.0 * M_PI, aSph.Area, 1e-12);
        Assert.Equal(32.0 * M_PI / 3.0, aSph.Volume, 1e-12);
    }

    [Fact]
    public void Points_Satisfy_The_Implicit_Equations()
    {
        CylindricalSurface aCyl = new(Frame, 2.5);
        ConicalSurface aCone = new(Frame, 0.4, 1.5);
        SphericalSurface aSph = new(Frame, 3.0);
        foreach ((double U, double V) in new[] { (0.3, 0.2), (2.0, -1.1), (5.5, 0.9) })
        {
            Assert.Equal(0.0, Quadric(aCyl.Coefficients, aCyl.Value(U, V)), 1e-11);
            Assert.Equal(0.0, Quadric(aSph.Coefficients, aSph.Value(U, V)), 1e-11);
        }

        // OCCT: Geom_ConicalSurface::Coefficients has wrong linear and constant terms (occt-issues.md); they are
        // right only when the local origin is the global one and the reference radius is 0, where gp_Cone agrees.
        ConicalSurface aBadCone = new(Frame, 0.4, 1.5);
        Assert.True(Math.Abs(Quadric(aBadCone.Coefficients, aBadCone.Value(0.3, 0.2))) > 1e-3);
        ConicalSurface aApexCone = new(new CoordinateSystem(new Point3D(), new Direction3D(1, 2, 2), new Direction3D(2, -1, 0)), 0.4, 0.0);
        Assert.Equal(0.0, Quadric(aApexCone.Coefficients, aApexCone.Value(0.3, 0.2)), 1e-12);
        Cone aGpCone = aBadCone.Cone;
        Assert.Equal(0.0, Quadric(aGpCone.Coefficients, aBadCone.Value(0.3, 0.2)), 1e-11);

        ToroidalSurface aTor = new(Frame, 5.0, 1.25);
        NCollection_Array1<double> aCoef = new(1, 35);
        aTor.Coefficients(aCoef);
        double[] aExpected = new double[35];
        aTor.Torus.Coefficients(aExpected);
        Assert.Equal(aExpected, aCoef.AsSpan().ToArray());
    }

    private delegate void QuadricCoefficients(out double A1, out double A2, out double A3, out double B1, out double B2,
                                              out double B3, out double C1, out double C2, out double C3, out double D);

    private static double Quadric(QuadricCoefficients theCoefs, Point3D P)
    {
        theCoefs(out double A1, out double A2, out double A3, out double B1, out double B2, out double B3,
                 out double C1, out double C2, out double C3, out double D);
        double X = P.X, Y = P.Y, Z = P.Z;
        return A1 * X * X + A2 * Y * Y + A3 * Z * Z + 2 * (B1 * X * Y + B2 * X * Z + B3 * Y * Z)
               + 2 * (C1 * X + C2 * Y + C3 * Z) + D;
    }

    [Fact]
    public void Bounds_Periodicity_And_Reversed_Parameters()
    {
        foreach (ElementarySurface S in All())
        {
            S.ParameterBounds(out double U1, out double U2, out double V1, out double V2);
            Assert.Equal(0.0, U1);
            Assert.Equal(2 * M_PI, U2);
            Assert.True(S.IsUClosed);
            Assert.True(S.IsUPeriodic);
            Assert.Equal(2 * M_PI, S.UPeriod);
            Assert.Equal(S is ToroidalSurface, S.IsVPeriodic);
            Assert.Equal(S is ToroidalSurface, S.IsVClosed);
            if (S is SphericalSurface)
            {
                Assert.Equal(-M_PI / 2, V1);
                Assert.Equal(M_PI / 2, V2);
            }

            // Reversing a direction keeps the point set: (U, V) maps to the reversed parameters
            Surface aU = S.UReversed();
            Surface aV = S.VReversed();
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
        ConicalSurface aCone = new(Frame, 0.4, 1.5);
        ConicalSurface aRev = (ConicalSurface)aCone.VReversed();
        Assert.Equal(-0.4, aRev.SemiAngle);
        Assert.True(aRev.Apex.IsEqual(aCone.Apex, 1e-12));
    }

    [Fact]
    public void Derivatives_Are_Consistent()
    {
        foreach (ElementarySurface S in All())
        {
            double U = 0.7, V = 0.3;
            S.D3(U, V, out Point3D P, out Vector3D D1U, out Vector3D D1V, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV,
                 out Vector3D D3U, out Vector3D D3V, out Vector3D D3UUV, out Vector3D D3UVV);
            Assert.Equal(P, S.Value(U, V));
            Assert.True(S.DN(U, V, 1, 0).IsEqual(D1U, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 0, 1).IsEqual(D1V, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 2, 0).IsEqual(D2U, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 1, 1).IsEqual(D2UV, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 3, 0).IsEqual(D3U, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 2, 1).IsEqual(D3UUV, 1e-12, 1e-12));
            Assert.True(S.DN(U, V, 1, 2).IsEqual(D3UVV, 1e-12, 1e-12));
            if (S is not CylindricalSurface and not ConicalSurface)
            {
                Assert.True(S.DN(U, V, 0, 2).IsEqual(D2V, 1e-12, 1e-12));
                Assert.True(S.DN(U, V, 0, 3).IsEqual(D3V, 1e-12, 1e-12));
            }
            else
            {
                Assert.Equal(new Vector3D(0, 0, 0), S.DN(U, V, 0, 2));
            }

            // Finite differences of the point
            double h = 1e-6;
            Vector3D aFdU = new Vector3D(S.Value(U - h, V), S.Value(U + h, V)) / (2 * h);
            Assert.True(aFdU.IsEqual(D1U, 1e-6, 1e-6));
            Assert.Throws<UndefinedDerivative3DException>(() => S.DN(U, V, 0, 0));
        }
    }

    [Fact]
    public void Transform_Scales_The_Radii_And_TransformParameters_Follows()
    {
        Transform3D T = Transform3D.Scale(new Point3D(1, 1, 1), -2.0);
        foreach (ElementarySurface S in All())
        {
            Surface aMoved = (Surface)S.Transformed(T);
            double U = 0.7, V = 0.3;
            double U2 = U, V2 = V;
            S.TransformParameters(ref U2, ref V2, T);
            Assert.True(aMoved.Value(U2, V2).IsEqual(S.Value(U, V).Transformed(T), 1e-12));
            double aPU = U, aPV = V;
            S.ParametricTransformation(T).Transforms(ref aPU, ref aPV);
            Assert.Equal(U2, aPU, 1e-14);
            Assert.Equal(V2, aPV, 1e-14);
        }
        Assert.Equal(5.0, ((CylindricalSurface)new CylindricalSurface(Frame, 2.5).Transformed(T)).Radius);
        ToroidalSurface aTor = (ToroidalSurface)new ToroidalSurface(Frame, 5.0, 1.25).Transformed(T);
        Assert.Equal(10.0, aTor.MajorRadius);
        Assert.Equal(2.5, aTor.MinorRadius);
    }

    [Fact]
    public void Frozen_Surfaces_Refuse_Mutation_But_Twins_Work()
    {
        CylindricalSurface aCyl = new(Frame, 2.5);
        ConicalSurface aCone = new(Frame, 0.4, 1.5);
        SphericalSurface aSph = new(Frame, 3.0);
        ToroidalSurface aTor = new(Frame, 5.0, 1.25);
        foreach (ElementarySurface S in new ElementarySurface[] { aCyl, aCone, aSph, aTor })
        {
            S.Freeze();
            Assert.Throws<ImmutableObjectException>(() => S.Transform(new Transform3D()));
            Assert.Throws<ImmutableObjectException>(() => S.UReverse());
            Assert.Throws<ImmutableObjectException>(() => S.VReverse());
            Assert.Throws<ImmutableObjectException>(() => S.SetLocation(new Point3D()));
            Assert.False(S.Copy().IsFrozen);
        }
        Assert.Throws<ImmutableObjectException>(() => aCyl.SetRadius(1));
        Assert.Throws<ImmutableObjectException>(() => aCone.SetSemiAngle(0.2));
        Assert.Throws<ImmutableObjectException>(() => aSph.SetRadius(1));
        Assert.Throws<ImmutableObjectException>(() => aTor.SetMinorRadius(1));

        Assert.Equal(1.0, aCyl.WithRadius(1).Radius);
        Assert.Equal(2.0, aCyl.WithCylinder(new Cylinder(Frame, 2)).Radius);
        Assert.Equal(0.2, aCone.WithSemiAngle(0.2).SemiAngle);
        Assert.Equal(0.5, aCone.WithRadius(0.5).RefRadius);
        Assert.Equal(0.1, aCone.WithCone(new Cone(Frame, 0.1, 1)).SemiAngle);
        Assert.Equal(1.0, aSph.WithRadius(1).Radius);
        Assert.Equal(4.0, aSph.WithSphere(new Sphere(Frame, 4)).Radius);
        Assert.Equal(1.0, aTor.WithMinorRadius(1).MinorRadius);
        Assert.Equal(6.0, aTor.WithMajorRadius(6).MajorRadius);
        Assert.Equal(3.0, aTor.WithTorus(new Torus(Frame, 7, 3)).MinorRadius);
        Assert.Equal(2.5, aCyl.Radius);
    }

    [Fact]
    public void IsoCurves_Are_Lines_Circles_And_Half_Circles()
    {
        ElementarySurface[] aSurfaces = All();
        Assert.IsType<LineCurve3D>(aSurfaces[0].UIso(0.5));
        Assert.IsType<CircleCurve3D>(aSurfaces[0].VIso(0.5));
        Assert.IsType<LineCurve3D>(aSurfaces[1].UIso(0.5));
        Assert.IsType<CircleCurve3D>(aSurfaces[1].VIso(0.5));
        TrimmedCurve3D aHalf = Assert.IsType<TrimmedCurve3D>(aSurfaces[2].UIso(0.5));
        Assert.IsType<CircleCurve3D>(aHalf.BasisCurve);
        // [-PI/2, PI/2], brought into the circle's first period by Geom_TrimmedCurve
        Assert.Equal(1.5 * M_PI, aHalf.FirstParameter, 1e-15);
        Assert.Equal(2.5 * M_PI, aHalf.LastParameter, 1e-15);
        Assert.IsType<CircleCurve3D>(aSurfaces[2].VIso(0.5));
        Assert.IsType<CircleCurve3D>(aSurfaces[3].UIso(0.5));
        Assert.IsType<CircleCurve3D>(aSurfaces[3].VIso(0.5));

        // Each iso-curve has the surface's parametrization in the other direction
        foreach (ElementarySurface S in aSurfaces)
        {
            foreach (double P in new[] { -0.7, 0.5, 1.2 })
            {
                foreach (double T in new[] { -1.1, 0.25, 1.4 })
                {
                    Assert.True(S.UIso(P).Value(T).IsEqual(S.Value(P, T), 1e-12));
                    Assert.True(S.VIso(P).Value(T).IsEqual(S.Value(T, P), 1e-12));
                }
            }
        }
    }
}
