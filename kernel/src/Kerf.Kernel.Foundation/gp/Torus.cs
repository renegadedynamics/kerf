// Port of: src/FoundationClasses/TKMath/gp/gp_Torus.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A torus, defined by its major and minor radii and positioned by a coordinate system whose origin is the
/// centre: the surface is swept by a circle of the minor radius, in the plane (origin, X direction, main
/// direction) and centred on the X axis at the major radius, rotating about the main direction.
/// </summary>
/// <remarks>
/// <c>new gp_Torus()</c> is OCCT's indefinite torus (major radius <see cref="Standard.RealLast"/>, minor radius
/// <see cref="Standard.RealSmall"/>); <c>default(gp_Torus)</c> is not valid.
/// </remarks>
public readonly partial struct Torus : IEquatable<Torus>
{
    private readonly CoordinateSystem _pos;
    private readonly double _majorRadius;
    private readonly double _minorRadius;

    public Torus()
    {
        _pos = new CoordinateSystem();
        _majorRadius = RealLast();
        _minorRadius = RealSmall();
    }

    /// <remarks>A torus with both radii 0 is allowed.</remarks>
    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public Torus(in CoordinateSystem a3, double majorRadius, double minorRadius)
    {
        _pos = a3;
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        Standard_ConstructionError_Raise_if(minorRadius < 0.0 || majorRadius < 0.0,
                                            "gp_Torus() - invalid construction parameters");
    }

    // Copies

    public readonly Torus WithAxis(in Axis3D a1)
    {
        CoordinateSystem pos = _pos;
        pos = pos.WithAxis(a1);
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly Torus WithLocation(in Point3D loc)
    {
        CoordinateSystem pos = _pos;
        pos = pos.WithLocation(loc);
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly Torus WithMajorRadius(double majorRadius)
    {
        double newMajorRadius = _majorRadius;
        Standard_ConstructionError_Raise_if(majorRadius - _minorRadius <= Constants.Resolution(),
                                            "gp_Torus::SetMajorRadius() - invalid input parameters");
        newMajorRadius = majorRadius;
        return new Torus(_pos, newMajorRadius, _minorRadius);
    }

    public readonly Torus WithMinorRadius(double minorRadius)
    {
        double newMinorRadius = _minorRadius;
        Standard_ConstructionError_Raise_if(minorRadius < 0.0
                                            || _majorRadius - minorRadius <= Constants.Resolution(),
                                            "gp_Torus::SetMinorRadius() - invalid input parameters");
        newMinorRadius = minorRadius;
        return new Torus(_pos, _majorRadius, newMinorRadius);
    }

    public readonly Torus WithPosition(in CoordinateSystem a3)
    {
        CoordinateSystem pos = _pos;
        pos = a3;
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly Torus UReversed()
    {
        CoordinateSystem pos = _pos;
        pos = pos.YReversed();
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly Torus VReversed()
    {
        CoordinateSystem pos = _pos;
        pos = pos.ZReversed();
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    // Queries

    public readonly double Area() => 4.0 * M_PI * M_PI * _minorRadius * _majorRadius;

    /// <summary>True when the coordinate system is right-handed.</summary>
    public readonly bool Direct() => _pos.Direct();

    /// <summary>The symmetry axis.</summary>
    public readonly Axis3D Axis() => _pos.Axis();

    /// <summary>
    /// The 35 coefficients of the implicit quartic equation in the global system, in OCCT's order:
    /// X^4, Y^4, Z^4, X^3.Y, X^3.Z, Y^3.X, Y^3.Z, Z^3.X, Z^3.Y, X^2.Y^2, X^2.Z^2, Y^2.Z^2, X^2.Y.Z, X.Y^2.Z,
    /// X.Y.Z^2, X^3, Y^3, Z^3, X^2.Y, X^2.Z, Y^2.X, Y^2.Z, Z^2.X, Z^2.Y, X.Y.Z, X^2, Y^2, Z^2, X.Y, X.Z, Y.Z,
    /// X, Y, Z, 1.
    /// </summary>
    /// <remarks>
    /// OCCT fills an <c>NCollection_Array1</c> from its lower bound; the port fills a span from index 0, so
    /// OCCT's <c>Coef(Lower() + i)</c> is <c>theCoef[i]</c>.
    /// </remarks>
    /// <exception cref="DimensionErrorException"><paramref name="coef"/> has fewer than 35 elements.</exception>
    public readonly void Coefficients(Span<double> coef)
    {
        //  R = majorRadius;
        //  r = minorRadius.
        //  X = (R + r*cos(V))*cos(U)
        //  Y = (R + r*cos(V))*sin(U)
        //  Z = r*sin(V)
        // Therefore,
        //   4*R*R*(r*r - Z*Z) = (X*X + Y*Y + Z*Z - R*R - r*r)^2
        // Or
        //   X^4+Y^4+Z^4+
        //   2*((X*Y)^2+(X*Z)^2+(Y*Z)^2)-
        //   2*(R^2+r^2)*(X^2+Y^2)+
        //   2*(R^2-r^2)*Z^2+(R^2-r^2)^2 = 0.0
        const int lowIndex = 0;
        Standard_DimensionError_Raise_if(coef.Length < 35, "gp_Torus::theCoefficients(): Dimension mismatch");

        Transform3D tr = new Transform3D().WithTransformation(_pos);
        double t11 = tr.Value(1, 1);
        double t12 = tr.Value(1, 2);
        double t13 = tr.Value(1, 3);
        double t14 = tr.Value(1, 4);
        double t21 = tr.Value(2, 1);
        double t22 = tr.Value(2, 2);
        double t23 = tr.Value(2, 3);
        double t24 = tr.Value(2, 4);
        double t31 = tr.Value(3, 1);
        double t32 = tr.Value(3, 2);
        double t33 = tr.Value(3, 3);
        double t34 = tr.Value(3, 4);

        double tcol1sq = t11 * t11 + t21 * t21 + t31 * t31;
        double tcol2sq = t12 * t12 + t22 * t22 + t32 * t32;
        double tcol3sq = t13 * t13 + t23 * t23 + t33 * t33;
        double tcol4sq = t14 * t14 + t24 * t24 + t34 * t34;
        double tcol1Tcol2 = t11 * t12 + t21 * t22 + t31 * t32;
        double tcol1Tcol3 = t11 * t13 + t21 * t23 + t31 * t33;
        double tcol2Tcol3 = t12 * t13 + t22 * t23 + t32 * t33;
        double tcol1Tcol4 = t11 * t14 + t21 * t24 + t31 * t34;
        double tcol2Tcol4 = t12 * t14 + t22 * t24 + t32 * t34;
        double tcol3Tcol4 = t13 * t14 + t23 * t24 + t33 * t34;

        double sumRadius = (_majorRadius * _majorRadius + _minorRadius * _minorRadius);
        double subRadius = (_majorRadius * _majorRadius - _minorRadius * _minorRadius);

        // After substitution
        // Transpose([X Y Z 1]) = aTr*Transpose([X Y Z 1])
        // we will obtain:
        coef[lowIndex] = tcol1sq * tcol1sq; // X^4
        coef[lowIndex + 1] = tcol2sq * tcol2sq; // Y^4
        coef[lowIndex + 2] = tcol3sq * tcol3sq; // Z^4
        coef[lowIndex + 3] = 4.0 * tcol1sq * tcol1Tcol2; // X^3*Y
        coef[lowIndex + 4] = 4.0 * tcol1sq * tcol1Tcol3; // X^3*Z
        coef[lowIndex + 5] = 4.0 * tcol2sq * tcol1Tcol2; // X*Y^3
        coef[lowIndex + 6] = 4.0 * tcol2sq * tcol2Tcol3; // Y^3*Z
        coef[lowIndex + 7] = 4.0 * tcol3sq * tcol1Tcol3; // X*Z^3
        coef[lowIndex + 8] = 4.0 * tcol3sq * tcol2Tcol3; // Y*Z^3
        coef[lowIndex + 9] = 2.0 * (tcol1sq * tcol2sq + 2.0 * tcol1Tcol2 * tcol1Tcol2); // X^2*Y^2
        coef[lowIndex + 10] =
            2.0 * (tcol1sq * tcol3sq + 2.0 * tcol1Tcol3 * tcol1Tcol3); // X^2*Z^2
        coef[lowIndex + 11] =
            2.0 * (tcol2sq * tcol3sq + 2.0 * tcol2Tcol3 * tcol2Tcol3); // Y^2*Z^2
        coef[lowIndex + 12] =
            4.0 * (tcol1sq * tcol2Tcol3 + 2.0 * tcol1Tcol2 * tcol1Tcol3); // X^2*Y*Z
        coef[lowIndex + 13] =
            4.0 * (tcol2sq * tcol1Tcol3 + 2.0 * tcol1Tcol2 * tcol2Tcol3); // X*Y^2*Z
        coef[lowIndex + 14] =
            4.0 * (tcol3sq * tcol1Tcol2 + 2.0 * tcol1Tcol3 * tcol2Tcol3); // X*Y*Z^2

        coef[lowIndex + 15] = 4.0 * tcol1sq * tcol1Tcol4; // X^3
        coef[lowIndex + 16] = 4.0 * tcol2sq * tcol2Tcol4; // Y^3
        coef[lowIndex + 17] = 4.0 * tcol3sq * tcol3Tcol4; // Z^3
        coef[lowIndex + 18] =
            4.0 * (tcol1sq * tcol2Tcol4 + 2.0 * tcol1Tcol4 * tcol1Tcol2); // X^2*Y
        coef[lowIndex + 19] =
            4.0 * (tcol1sq * tcol3Tcol4 + 2.0 * tcol1Tcol4 * tcol1Tcol3); // X^2*Z
        coef[lowIndex + 20] =
            4.0 * (tcol2sq * tcol1Tcol4 + 2.0 * tcol2Tcol4 * tcol1Tcol2); // X*Y^2
        coef[lowIndex + 21] =
            4.0 * (tcol2sq * tcol3Tcol4 + 2.0 * tcol2Tcol4 * tcol2Tcol3); // Y^2*Z
        coef[lowIndex + 22] =
            4.0 * (tcol3sq * tcol1Tcol4 + 2.0 * tcol3Tcol4 * tcol1Tcol3); // X*Z^2
        coef[lowIndex + 23] =
            4.0 * (tcol3sq * tcol2Tcol4 + 2.0 * tcol3Tcol4 * tcol2Tcol3); // Y*Z^2
        coef[lowIndex + 24] =
            8.0
            * (tcol1Tcol2 * tcol3Tcol4 + tcol2Tcol3 * tcol1Tcol4 + tcol2Tcol4 * tcol1Tcol3); // X*Y*Z

        coef[lowIndex + 25] = 2.0
                                  * (subRadius * t31 * t31 - sumRadius * (t11 * t11 + t21 * t21)
                                     + tcol4sq * tcol1sq + 2.0 * tcol1Tcol4 * tcol1Tcol4); // X^2
        coef[lowIndex + 26] = 2.0
                                  * (subRadius * t32 * t32 - sumRadius * (t12 * t12 + t22 * t22)
                                     + tcol4sq * tcol2sq + 2.0 * tcol2Tcol4 * tcol2Tcol4); // Y^2
        coef[lowIndex + 27] = 2.0
                                  * (subRadius * t33 * t33 - sumRadius * (t13 * t13 + t23 * t23)
                                     + tcol4sq * tcol3sq + 2.0 * tcol3Tcol4 * tcol3Tcol4); // Z^2
        coef[lowIndex + 28] = 4.0
                                  * (subRadius * t31 * t32 - sumRadius * (t11 * t12 + t21 * t22)
                                     + tcol4sq * tcol1Tcol2 + 2.0 * tcol1Tcol4 * tcol2Tcol4); // X*Y
        coef[lowIndex + 29] = 4.0
                                  * (subRadius * t31 * t33 - sumRadius * (t11 * t13 + t21 * t23)
                                     + tcol4sq * tcol1Tcol3 + 2.0 * tcol1Tcol4 * tcol3Tcol4); // X*Z
        coef[lowIndex + 30] = 4.0
                                  * (subRadius * t32 * t33 - sumRadius * (t12 * t13 + t22 * t23)
                                     + tcol4sq * tcol2Tcol3 + 2.0 * tcol2Tcol4 * tcol3Tcol4); // Y*Z

        coef[lowIndex + 31] = 4.0
                                  * (tcol4sq * tcol1Tcol4 + subRadius * t31 * t34
                                     - sumRadius * (t11 * t14 + t21 * t24)); // X
        coef[lowIndex + 32] = 4.0
                                  * (tcol4sq * tcol2Tcol4 + subRadius * t32 * t34
                                     - sumRadius * (t12 * t14 + t22 * t24)); // Y
        coef[lowIndex + 33] = 4.0
                                  * (tcol4sq * tcol3Tcol4 + subRadius * t33 * t34
                                     - sumRadius * (t13 * t14 + t23 * t24)); // Z;

        coef[lowIndex + 34] = 2.0 * subRadius * t34 * t34
                                  - 2.0 * sumRadius * (t14 * t14 + t24 * t24) + tcol4sq * tcol4sq
                                  + subRadius * subRadius;
    }

    public readonly Point3D Location() => _pos.Location();

    public readonly CoordinateSystem Position() => _pos;

    public readonly double MajorRadius() => _majorRadius;

    public readonly double MinorRadius() => _minorRadius;

    public readonly double Volume() => (M_PI * _minorRadius * _minorRadius) * (2.0 * M_PI * _majorRadius);

    public readonly Axis3D XAxis() => new(_pos.Location(), _pos.XDirection());

    public readonly Axis3D YAxis() => new(_pos.Location(), _pos.YDirection());

    public readonly Torus Mirrored(in Point3D p)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(p);
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly Torus Mirrored(in Axis3D a1)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(a1);
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly Torus Mirrored(in Frame3D a2)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(a2);
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly Torus Rotated(in Axis3D a1, double ang)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Rotated(a1, ang);
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    /// <summary>The torus scaled by |<paramref name="s"/>| about <paramref name="p"/>.</summary>
    public readonly Torus Scaled(in Point3D p, double s)
    {
        CoordinateSystem pos = _pos;
        double majorRadius = _majorRadius;
        double minorRadius = _minorRadius;
        pos = pos.Scaled(p, s);
        majorRadius *= s;
        if (majorRadius < 0)
        {
            majorRadius = -majorRadius;
        }

        minorRadius *= s;
        if (minorRadius < 0)
        {
            minorRadius = -minorRadius;
        }

        return new Torus(pos, majorRadius, minorRadius);
    }

    public readonly Torus Transformed(in Transform3D t)
    {
        CoordinateSystem pos = _pos;
        double majorRadius = _majorRadius;
        double minorRadius = _minorRadius;
        pos = pos.Transformed(t);
        majorRadius *= t.ScaleFactor();
        if (majorRadius < 0)
        {
            majorRadius = -majorRadius;
        }

        minorRadius *= t.ScaleFactor();
        if (minorRadius < 0)
        {
            minorRadius = -minorRadius;
        }

        return new Torus(pos, majorRadius, minorRadius);
    }

    public readonly Torus Translated(in Vector3D v)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Translated(v);
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly Torus Translated(in Point3D p1, in Point3D p2)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Translated(p1, p2);
        return new Torus(pos, _majorRadius, _minorRadius);
    }

    public readonly bool Equals(Torus other)
        => _pos.Equals(other._pos) && _majorRadius.Equals(other._majorRadius) && _minorRadius.Equals(other._minorRadius);

    public override readonly bool Equals(object? obj) => obj is Torus other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _majorRadius, _minorRadius);

    public static bool operator ==(in Torus left, in Torus right) => left.Equals(right);

    public static bool operator !=(in Torus left, in Torus right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Torus({_pos}, {_majorRadius:R}, {_minorRadius:R})");
}
