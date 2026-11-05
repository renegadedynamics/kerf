// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PrincipalProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_PrincipalProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The principal properties of inertia of a system, as computed by <see cref="Properties.PrincipalProperties"/>:
/// the principal moments (eigenvalues of the matrix of inertia), the principal axes (its eigenvectors, through the
/// centre of mass) and the principal radii of gyration.
/// </summary>
/// <remarks>
/// A value: OCCT's class has only const members and is copied. <c>default(GProp_PrincipalProps)</c> is all zeros;
/// OCCT's "undefined" value, <c>new GProp_PrincipalProps()</c>, has moments, radii and centre RealLast() and the
/// absolute axes.
/// </remarks>
internal partial struct PrincipalProperties : IEquatable<PrincipalProperties>
{
    private double _i1;
    private double _i2;
    private double _i3;
    private double _r1;
    private double _r2;
    private double _r3;
    private Vector3D _v1;
    private Vector3D _v2;
    private Vector3D _v3;
    private Point3D _g;

    /// <summary>An undefined PrincipalProps.</summary>
    public PrincipalProperties()
    {
        _i1 = _i2 = _i3 = RealLast();
        _r1 = _r2 = _r3 = RealLast();
        _v1 = new Vector3D(1.0, 0.0, 0.0);
        _v2 = new Vector3D(0.0, 1.0, 0.0);
        _v3 = new Vector3D(0.0, 0.0, 1.0);
        _g = new Point3D(RealLast(), RealLast(), RealLast());
    }

    // OCCT: private, a friend of GProp_GProps::PrincipalProperties.
    internal PrincipalProperties(double ixx, double iyy, double izz, double rxx, double ryy, double rzz,
                                  in Vector3D vxx, in Vector3D vyy, in Vector3D vzz, in Point3D g)
    {
        _i1 = ixx;
        _i2 = iyy;
        _i3 = izz;
        _r1 = rxx;
        _r2 = ryy;
        _r3 = rzz;
        _v1 = vxx;
        _v2 = vyy;
        _v3 = vzz;
        _g = g;
    }

    /// <summary>True if the system has an axis of symmetry (moments compared with relative tolerance 1e-10).</summary>
    public readonly bool HasSymmetryAxis()
    {
        const double relTol = 1.0e-10;
        double eps1 = Math.Abs(_i1) * relTol;
        double eps2 = Math.Abs(_i2) * relTol;
        return (Math.Abs(_i1 - _i2) <= eps1 || Math.Abs(_i1 - _i3) <= eps1 || Math.Abs(_i2 - _i3) <= eps2);
    }

    /// <summary>
    /// True if the system has an axis of symmetry, with relative tolerance <paramref name="aTol"/> (about 1e-16
    /// when it is 0).
    /// </summary>
    public readonly bool HasSymmetryAxis(double aTol)
    {
        double eps1 = Math.Abs(_i1 * aTol) + Math.Abs(Epsilon(_i1));
        double eps2 = Math.Abs(_i2 * aTol) + Math.Abs(Epsilon(_i2));
        return (Math.Abs(_i1 - _i2) <= eps1 || Math.Abs(_i1 - _i3) <= eps1 || Math.Abs(_i2 - _i3) <= eps2);
    }

    /// <summary>True if the system has a point of symmetry (moments compared with relative tolerance 1e-10).</summary>
    public readonly bool HasSymmetryPoint()
    {
        const double relTol = 1.0e-10;
        double eps1 = Math.Abs(_i1) * relTol;
        return (Math.Abs(_i1 - _i2) <= eps1 && Math.Abs(_i1 - _i3) <= eps1);
    }

    /// <summary>
    /// True if the system has a point of symmetry, with relative tolerance <paramref name="aTol"/> (about 1e-16
    /// when it is 0).
    /// </summary>
    public readonly bool HasSymmetryPoint(double aTol)
    {
        double eps1 = Math.Abs(_i1 * aTol) + Math.Abs(Epsilon(_i1));
        return (Math.Abs(_i1 - _i2) <= eps1 && Math.Abs(_i1 - _i3) <= eps1);
    }

    /// <summary>
    /// The principal moments of inertia. Two are equal when the system has an axis of symmetry, all three when it
    /// has a centre of symmetry.
    /// </summary>
    public readonly void Moments(out double ixx, out double iyy, out double izz)
    {
        ixx = _i1;
        iyy = _i2;
        izz = _i3;
    }

    /// <summary>The first axis of inertia (undefined if the system has a point of symmetry).</summary>
    public readonly Vector3D FirstAxisOfInertia() => _v1;

    /// <summary>The second axis of inertia (undefined if the system has a point or an axis of symmetry).</summary>
    public readonly Vector3D SecondAxisOfInertia() => _v2;

    /// <summary>
    /// The third axis of inertia (undefined if the system has a point or an axis of symmetry). The three axes are
    /// the eigenvectors of the matrix of inertia; the principal axes pass through the centre of mass, parallel to
    /// them.
    /// </summary>
    public readonly Vector3D ThirdAxisOfInertia() => _v3;

    /// <summary>The principal radii of gyration, about the three principal axes.</summary>
    public readonly void RadiusOfGyration(out double rxx, out double ryy, out double rzz)
    {
        rxx = _r1;
        ryy = _r2;
        rzz = _r3;
    }

    public readonly bool Equals(PrincipalProperties other)
        => _i1.Equals(other._i1) && _i2.Equals(other._i2) && _i3.Equals(other._i3)
           && _r1.Equals(other._r1) && _r2.Equals(other._r2) && _r3.Equals(other._r3)
           && _v1.Equals(other._v1) && _v2.Equals(other._v2) && _v3.Equals(other._v3) && _g.Equals(other._g);

    public override readonly bool Equals(object? obj) => obj is PrincipalProperties other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(HashCode.Combine(_i1, _i2, _i3, _r1, _r2, _r3), _v1, _v2, _v3, _g);

    public static bool operator ==(in PrincipalProperties left, in PrincipalProperties right)
        => left.Equals(right);

    public static bool operator !=(in PrincipalProperties left, in PrincipalProperties right)
        => !left.Equals(right);

    public override readonly string ToString()
        => Constants.Format("GProp_PrincipalProps", _i1, _i2, _i3, _r1, _r2, _r3, _v1.X, _v1.Y, _v1.Z, _v2.X, _v2.Y, _v2.Z,
                     _v3.X, _v3.Y, _v3.Z, _g.X, _g.Y, _g.Z);
}
