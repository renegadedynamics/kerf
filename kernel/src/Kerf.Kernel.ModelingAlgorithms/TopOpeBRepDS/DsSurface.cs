// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Surface.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>A surface of the legacy data structure: a surface with a tolerance, and whether it is kept.</summary>
/// <remarks>
/// A C++ value class, so a struct. Unlike points and curves, a new surface is <em>not</em> kept (OCCT's default):
/// <see cref="DsSurfaceExplorer"/> with its default <c>FindOnlyKeep</c> skips it until it is.
/// </remarks>
internal partial struct DsSurface : IEquatable<DsSurface>
{
    private Surface? _surface;
    private double _tolerance;
    private bool _keep;

    /// <summary>An empty surface, not kept.</summary>
    public DsSurface()
    {
        _surface = null;
        _tolerance = 0.0;
        _keep = false;
    }

    /// <summary>The surface <paramref name="surface"/> with the tolerance <paramref name="tolerance"/>, not kept.</summary>
    public DsSurface(Surface? surface, double tolerance)
    {
        _surface = surface;
        _tolerance = tolerance;
        _keep = false;
    }

    /// <summary>A copy of <paramref name="other"/>.</summary>
    public DsSurface(in DsSurface other)
    {
        _surface = other._surface;
        _tolerance = other._tolerance;
        _keep = other._keep;
    }

    internal void Assign(in DsSurface other)
    {
        _surface = other._surface;
        _tolerance = other._tolerance;
        _keep = other._keep;
    }

    /// <summary>The surface.</summary>
    public readonly Surface? Surface() => _surface;

    /// <summary>The tolerance.</summary>
    public readonly double Tolerance() => _tolerance;

    internal void Tolerance(double tol)
    {
        _tolerance = tol;
    }

    /// <summary>Whether the surface is kept.</summary>
    public readonly bool Keep() => _keep;

    internal void ChangeKeep(bool toKeep)
    {
        _keep = toKeep;
    }

    /// <summary>A copy with the tolerance <paramref name="tol"/>.</summary>
    public readonly DsSurface WithTolerance(double tol)
    {
        DsSurface copy = this;
        copy.Tolerance(tol);
        return copy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(DsSurface other)
        => ReferenceEquals(_surface, other._surface) && _tolerance.Equals(other._tolerance) && _keep == other._keep;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is DsSurface other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode()
        => HashCode.Combine(_surface is null ? 0 : System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(_surface),
                            _tolerance, _keep);

    public static bool operator ==(in DsSurface left, in DsSurface right) => left.Equals(right);

    public static bool operator !=(in DsSurface left, in DsSurface right) => !left.Equals(right);

    /// <inheritdoc/>
    public override readonly string ToString()
        => "TopOpeBRepDS_Surface(" + (_surface is null ? "null" : global::Kerf.Engine.Foundation.OcctTypeNames.Of(_surface.GetType())) + ", " + _tolerance.ToString("R", System.Globalization.CultureInfo.InvariantCulture)
           + (_keep ? ", kept)" : ")");
}
