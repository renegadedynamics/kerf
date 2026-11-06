// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_POnCurv.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Extrema;

/// <summary>A point on a curve: its parameter on the curve and the point.</summary>
/// <remarks>
/// <c>new Extrema_POnCurv()</c> and <c>default(Extrema_POnCurv)</c> are the same "indefinite" point (parameter 0,
/// the origin). The constructor is the twin of the internal <c>SetValues</c>.
/// </remarks>
internal partial struct POnCurv : IEquatable<POnCurv>
{
    private double _u;
    private Point3D _p;

    /// <summary>An indefinite point on curve.</summary>
    public POnCurv()
    {
        _u = 0.0;
        _p = new Point3D();
    }

    /// <summary>A point on curve with its parameter value on the curve.</summary>
    public POnCurv(double u, in Point3D p)
    {
        _u = u;
        _p = p;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the point and parameter values.</summary>
    internal void SetValues(double u, in Point3D p)
    {
        _u = u;
        _p = p;
    }

    // Queries

    /// <summary>The point.</summary>
    public readonly Point3D Value() => _p;

    /// <summary>The parameter on the curve.</summary>
    public readonly double Parameter() => _u;

    public readonly bool Equals(POnCurv other) => _u.Equals(other._u) && _p.Equals(other._p);

    public override readonly bool Equals(object? obj) => obj is POnCurv other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_u, _p);

    public static bool operator ==(in POnCurv left, in POnCurv right) => left.Equals(right);

    public static bool operator !=(in POnCurv left, in POnCurv right) => !left.Equals(right);

    public override readonly string ToString() => $"Extrema_POnCurv({_u.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, {_p})";
}
