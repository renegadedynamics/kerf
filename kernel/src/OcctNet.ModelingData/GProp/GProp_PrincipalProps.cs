// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PrincipalProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PrincipalProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The principal properties of inertia of a system, as computed by <see cref="GProp_GProps.PrincipalProperties"/>:
/// the principal moments (eigenvalues of the matrix of inertia), the principal axes (its eigenvectors, through the
/// centre of mass) and the principal radii of gyration.
/// </summary>
/// <remarks>
/// A value: OCCT's class has only const members and is copied. <c>default(GProp_PrincipalProps)</c> is all zeros;
/// OCCT's "undefined" value, <c>new GProp_PrincipalProps()</c>, has moments, radii and centre RealLast() and the
/// absolute axes.
/// </remarks>
public partial struct GProp_PrincipalProps : IEquatable<GProp_PrincipalProps>
{
    private double i1;
    private double i2;
    private double i3;
    private double r1;
    private double r2;
    private double r3;
    private gp_Vec v1;
    private gp_Vec v2;
    private gp_Vec v3;
    private gp_Pnt g;

    /// <summary>An undefined PrincipalProps.</summary>
    public GProp_PrincipalProps()
    {
        i1 = i2 = i3 = RealLast();
        r1 = r2 = r3 = RealLast();
        v1 = new gp_Vec(1.0, 0.0, 0.0);
        v2 = new gp_Vec(0.0, 1.0, 0.0);
        v3 = new gp_Vec(0.0, 0.0, 1.0);
        g = new gp_Pnt(RealLast(), RealLast(), RealLast());
    }

    // OCCT: private, a friend of GProp_GProps::PrincipalProperties.
    internal GProp_PrincipalProps(double Ixx, double Iyy, double Izz, double Rxx, double Ryy, double Rzz,
                                  in gp_Vec Vxx, in gp_Vec Vyy, in gp_Vec Vzz, in gp_Pnt G)
    {
        i1 = Ixx;
        i2 = Iyy;
        i3 = Izz;
        r1 = Rxx;
        r2 = Ryy;
        r3 = Rzz;
        v1 = Vxx;
        v2 = Vyy;
        v3 = Vzz;
        g = G;
    }

    /// <summary>True if the system has an axis of symmetry (moments compared with relative tolerance 1e-10).</summary>
    public readonly bool HasSymmetryAxis()
    {
        const double aRelTol = 1.0e-10;
        double Eps1 = Math.Abs(i1) * aRelTol;
        double Eps2 = Math.Abs(i2) * aRelTol;
        return (Math.Abs(i1 - i2) <= Eps1 || Math.Abs(i1 - i3) <= Eps1 || Math.Abs(i2 - i3) <= Eps2);
    }

    /// <summary>
    /// True if the system has an axis of symmetry, with relative tolerance <paramref name="aTol"/> (about 1e-16
    /// when it is 0).
    /// </summary>
    public readonly bool HasSymmetryAxis(double aTol)
    {
        double Eps1 = Math.Abs(i1 * aTol) + Math.Abs(Epsilon(i1));
        double Eps2 = Math.Abs(i2 * aTol) + Math.Abs(Epsilon(i2));
        return (Math.Abs(i1 - i2) <= Eps1 || Math.Abs(i1 - i3) <= Eps1 || Math.Abs(i2 - i3) <= Eps2);
    }

    /// <summary>True if the system has a point of symmetry (moments compared with relative tolerance 1e-10).</summary>
    public readonly bool HasSymmetryPoint()
    {
        const double aRelTol = 1.0e-10;
        double Eps1 = Math.Abs(i1) * aRelTol;
        return (Math.Abs(i1 - i2) <= Eps1 && Math.Abs(i1 - i3) <= Eps1);
    }

    /// <summary>
    /// True if the system has a point of symmetry, with relative tolerance <paramref name="aTol"/> (about 1e-16
    /// when it is 0).
    /// </summary>
    public readonly bool HasSymmetryPoint(double aTol)
    {
        double Eps1 = Math.Abs(i1 * aTol) + Math.Abs(Epsilon(i1));
        return (Math.Abs(i1 - i2) <= Eps1 && Math.Abs(i1 - i3) <= Eps1);
    }

    /// <summary>
    /// The principal moments of inertia. Two are equal when the system has an axis of symmetry, all three when it
    /// has a centre of symmetry.
    /// </summary>
    public readonly void Moments(out double Ixx, out double Iyy, out double Izz)
    {
        Ixx = i1;
        Iyy = i2;
        Izz = i3;
    }

    /// <summary>The first axis of inertia (undefined if the system has a point of symmetry).</summary>
    public readonly gp_Vec FirstAxisOfInertia() => v1;

    /// <summary>The second axis of inertia (undefined if the system has a point or an axis of symmetry).</summary>
    public readonly gp_Vec SecondAxisOfInertia() => v2;

    /// <summary>
    /// The third axis of inertia (undefined if the system has a point or an axis of symmetry). The three axes are
    /// the eigenvectors of the matrix of inertia; the principal axes pass through the centre of mass, parallel to
    /// them.
    /// </summary>
    public readonly gp_Vec ThirdAxisOfInertia() => v3;

    /// <summary>The principal radii of gyration, about the three principal axes.</summary>
    public readonly void RadiusOfGyration(out double Rxx, out double Ryy, out double Rzz)
    {
        Rxx = r1;
        Ryy = r2;
        Rzz = r3;
    }

    public readonly bool Equals(GProp_PrincipalProps theOther)
        => i1.Equals(theOther.i1) && i2.Equals(theOther.i2) && i3.Equals(theOther.i3)
           && r1.Equals(theOther.r1) && r2.Equals(theOther.r2) && r3.Equals(theOther.r3)
           && v1.Equals(theOther.v1) && v2.Equals(theOther.v2) && v3.Equals(theOther.v3) && g.Equals(theOther.g);

    public override readonly bool Equals(object? theObj) => theObj is GProp_PrincipalProps aOther && Equals(aOther);

    public override readonly int GetHashCode()
        => HashCode.Combine(HashCode.Combine(i1, i2, i3, r1, r2, r3), v1, v2, v3, g);

    public static bool operator ==(in GProp_PrincipalProps theLeft, in GProp_PrincipalProps theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in GProp_PrincipalProps theLeft, in GProp_PrincipalProps theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => gp.Format("GProp_PrincipalProps", i1, i2, i3, r1, r2, r3, v1.X(), v1.Y(), v1.Z(), v2.X(), v2.Y(), v2.Z(),
                     v3.X(), v3.Y(), v3.Z(), g.X(), g.Y(), g.Z());
}
