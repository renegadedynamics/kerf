// Port of: src/ModelingData/TKGeomBase/GProp/GProp.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Global properties (length, area, volume, centre of mass, inertia) of sets of points, curves, surfaces, solids
/// and compound systems.
/// </summary>
internal static class GProp
{
    /// <summary>
    /// The "Huyghens operator" of a system of mass <paramref name="mass"/> and centre of mass <paramref name="g"/>
    /// at the point <paramref name="q"/>: Inertia/Q = Inertia/G + HOperator(G, Q, Mass).
    /// </summary>
    public static void HOperator(in Point3D g, in Point3D q, double mass, out Matrix3x3 Operator)
    {
        Xyz qg = g.XYZ() - q.XYZ();
        double ixx = qg.Y() * qg.Y() + qg.Z() * qg.Z();
        double iyy = qg.X() * qg.X() + qg.Z() * qg.Z();
        double izz = qg.Y() * qg.Y() + qg.X() * qg.X();
        double ixy = -qg.X() * qg.Y();
        double iyz = -qg.Y() * qg.Z();
        double ixz = -qg.X() * qg.Z();
        Operator = new Matrix3x3();
        Operator = Operator.WithCols(new Xyz(ixx, ixy, ixz), new Xyz(ixy, iyy, iyz), new Xyz(ixz, iyz, izz));
        Operator = Operator.Multiplied(mass);
    }

    // The elementary classes (CelGProps, SelGProps, VelGProps) compute their properties in closed form from the
    // moments of the item in its own frame (decision 1471). OCCT built their matrices of inertia as P V L (each
    // eigenvector scaled by its value) where P V L V^T P^T was meant, kept the centre of mass absolute where
    // GProp_GProps reads it relative to the location point, and had wrong cone areas and volumes, cylinder axial
    // moments, line Izz and torus centres.

    /// <summary>
    /// The centre of mass relative to <paramref name="loc"/> and the matrix of inertia at <paramref name="loc"/> (as
    /// <see cref="Properties"/> keeps them) of an item of mass <paramref name="m0"/> whose first moment is
    /// <paramref name="m1"/> (the integral of q) and whose second moments are <paramref name="m2"/> (the integrals of
    /// xx, yy, zz, xy, xz and yz), q being its points relative to <paramref name="origin"/>, in the axes
    /// <paramref name="x"/>, <paramref name="y"/>, <paramref name="z"/>.
    /// </summary>
    internal static void FromLocalMoments(in Point3D origin, in Xyz x, in Xyz y, in Xyz z, double m0, in Xyz m1,
                                          ReadOnlySpan<double> m2, in Point3D loc, out Point3D g, out Matrix3x3 inertia)
    {
        Span<double> p = [x.X(), y.X(), z.X(), x.Y(), y.Y(), z.Y(), x.Z(), y.Z(), z.Z()]; // P[i, k] = axis k's i
        Span<double> l = [m2[0], m2[3], m2[4], m2[3], m2[1], m2[5], m2[4], m2[5], m2[2]];
        Span<double> m1g = stackalloc double[3];
        Span<double> d = [origin.X - loc.X, origin.Y - loc.Y, origin.Z - loc.Z];
        Span<double> s = stackalloc double[9];
        for (int i = 0; i < 3; i++)
        {
            m1g[i] = p[3 * i] * m1.X() + p[3 * i + 1] * m1.Y() + p[3 * i + 2] * m1.Z();
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                double aSum = 0.0;
                for (int k = 0; k < 3; k++)
                {
                    for (int n = 0; n < 3; n++)
                    {
                        aSum += p[3 * i + k] * l[3 * k + n] * p[3 * j + n];
                    }
                }
                // the second moments about loc
                s[3 * i + j] = aSum + d[i] * m1g[j] + m1g[i] * d[j] + m0 * d[i] * d[j];
            }
        }
        double aTrace = s[0] + s[4] + s[8];
        inertia = new Matrix3x3(aTrace - s[0], -s[1], -s[2], -s[3], aTrace - s[4], -s[5], -s[6], -s[7], aTrace - s[8]);
        g = m0 != 0.0 ? new Point3D(d[0] + m1g[0] / m0, d[1] + m1g[1] / m0, d[2] + m1g[2] / m0) : new Point3D(d[0], d[1], d[2]);
    }

    /// <summary>
    /// <see cref="FromLocalMoments"/> for an item of revolution about the frame's Z axis, angle u in
    /// [<paramref name="u1"/>, <paramref name="u2"/>]: its points are (w cos u, w sin u, h) with the measure J du dm,
    /// and <paramref name="a0"/> to <paramref name="b2"/> are the integrals over the meridian of J, J w, J w^2, J h,
    /// J w h and J h^2. Returns the mass.
    /// </summary>
    internal static double FromRevolution(in Point3D origin, in Xyz x, in Xyz y, in Xyz z, double u1, double u2,
                                          double a0, double a1, double a2, double b1, double c, double b2,
                                          in Point3D loc, out Point3D g, out Matrix3x3 inertia)
    {
        double i0 = TrigMoment(0, 0, u1, u2), ic = TrigMoment(1, 0, u1, u2), @is = TrigMoment(0, 1, u1, u2);
        double icc = TrigMoment(2, 0, u1, u2), iss = TrigMoment(0, 2, u1, u2), ics = TrigMoment(1, 1, u1, u2);
        double m0 = i0 * a0;
        FromLocalMoments(origin, x, y, z, m0, new Xyz(ic * a1, @is * a1, i0 * b1),
                         [icc * a2, iss * a2, i0 * b2, ics * a2, ic * c, @is * c], loc, out g, out inertia);
        return m0;
    }

    /// <summary>The integral of cos^k(v) sin^b(v) over [<paramref name="v1"/>, <paramref name="v2"/>], for k up to 3 and b up to 2.</summary>
    internal static double TrigMoment(int k, int b, double v1, double v2) => TrigPrimitive(k, b, v2) - TrigPrimitive(k, b, v1);

    private static double TrigPrimitive(int k, int b, double v)
    {
        double s = Math.Sin(v), c = Math.Cos(v);
        return (k, b) switch
        {
            (0, 0) => v,
            (1, 0) => s,
            (2, 0) => (v + s * c) / 2.0,
            (3, 0) => s - s * s * s / 3.0,
            (_, 1) => -Math.Pow(c, k + 1) / (k + 1),
            (0, 2) => (v - s * c) / 2.0,
            (1, 2) => s * s * s / 3.0,
            (2, 2) => (v - Math.Sin(4.0 * v) / 4.0) / 8.0,
            (3, 2) => s * s * s / 3.0 - s * s * s * s * s / 5.0,
            _ => throw new KernelArgumentOutOfRangeException(nameof(k)),
        };
    }

    /// <summary>The integral of (r + s v)^a v^b over [<paramref name="v1"/>, <paramref name="v2"/>].</summary>
    internal static double PolyMoment(double r, double s, int a, int b, double v1, double v2)
    {
        double aSum = 0.0, aBinom = 1.0;
        for (int k = 0; k <= a; k++)
        {
            int n = k + b + 1;
            aSum += aBinom * Math.Pow(r, a - k) * Math.Pow(s, k) * (Math.Pow(v2, n) - Math.Pow(v1, n)) / n;
            aBinom = aBinom * (a - k) / (k + 1);
        }
        return aSum;
    }

    /// <summary>
    /// The integral over v in [<paramref name="v1"/>, <paramref name="v2"/>] of rho^e (R + rho cos v)^n sin^b v, with
    /// rho = <paramref name="rho"/> (a torus' surface), or of the same times rho over rho in [0, <paramref name="rho"/>]
    /// when <paramref name="solid"/> (its solid, in the measure rho drho dv).
    /// </summary>
    internal static double TorusMoment(double rMajor, double rho, int e, int n, int b, double v1, double v2, bool solid)
    {
        double aSum = 0.0, aBinom = 1.0;
        for (int j = 0; j <= n; j++)
        {
            int p = j + e; // the power of rho
            double aRho = solid ? Math.Pow(rho, p + 2) / (p + 2) : Math.Pow(rho, p);
            aSum += aBinom * Math.Pow(rMajor, n - j) * aRho * TrigMoment(j, b, v1, v2);
            aBinom = aBinom * (n - j) / (j + 1);
        }
        return aSum;
    }
}
