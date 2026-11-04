// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_POnSurf.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Extrema;

/// <summary>A point on a surface: its parameters on the surface and the point.</summary>
/// <remarks>
/// <c>new Extrema_POnSurf()</c> is OCCT's "indefinite" point, with parameters and coordinates at
/// <see cref="Precision.Infinite"/>; <c>default(Extrema_POnSurf)</c> is all zeros. The constructor is the twin of the
/// internal <c>SetParameters</c>.
/// </remarks>
internal partial struct POnSurf : IEquatable<POnSurf>
{
    private double _u;
    private double _v;
    private Point3D _p;

    /// <summary>An indefinite point on surface.</summary>
    public POnSurf()
    {
        _u = global::Kerf.Engine.Foundation.Precision.Infinite();
        _v = global::Kerf.Engine.Foundation.Precision.Infinite();
        _p = new Point3D(global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());
    }

    /// <summary>A point on surface with its parameter values on the surface.</summary>
    public POnSurf(double u, double v, in Point3D p)
    {
        _u = u;
        _v = v;
        _p = p;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the parameters and the point.</summary>
    internal void SetParameters(double u, double v, in Point3D pnt)
    {
        _u = u;
        _v = v;
        _p = pnt;
    }

    // Queries

    /// <summary>The 3D point.</summary>
    public readonly Point3D Value() => _p;

    /// <summary>The parameter values on the surface.</summary>
    public readonly void Parameter(out double u, out double v)
    {
        u = _u;
        v = _v;
    }

    public readonly bool Equals(POnSurf other) => _u.Equals(other._u) && _v.Equals(other._v) && _p.Equals(other._p);

    public override readonly bool Equals(object? obj) => obj is POnSurf other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_u, _v, _p);

    public static bool operator ==(in POnSurf left, in POnSurf right) => left.Equals(right);

    public static bool operator !=(in POnSurf left, in POnSurf right) => !left.Equals(right);

    public override readonly string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Extrema_POnSurf({_u:R}, {_v:R}, {_p})");
}
