// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Point.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>A point of the legacy data structure: a 3D point with a tolerance, and whether it is kept.</summary>
/// <remarks>
/// A C++ value class, so a struct. The data structure keeps its own copy (<see cref="DsDataStructure.AddPoint"/>)
/// and gives it for writing by <c>ref</c> (<see cref="DsDataStructure.ChangePoint"/>).
/// </remarks>
internal partial struct DsPoint : IEquatable<DsPoint>
{
    private Point3D _point;
    private double _tolerance;
    private bool _keep;

    /// <summary>An empty point, kept.</summary>
    /// <remarks>OCCT leaves the tolerance unset; the port's is 0.</remarks>
    public DsPoint()
    {
        _point = new Point3D();
        _tolerance = 0.0;
        _keep = true;
    }

    /// <summary>The point <paramref name="p"/> with the tolerance <paramref name="t"/>, kept.</summary>
    public DsPoint(in Point3D p, double t)
    {
        _point = p;
        _tolerance = t;
        _keep = true;
    }

    /// <summary>The point and tolerance of the vertex <paramref name="s"/>.</summary>
    /// <exception cref="ProgramErrorException"><paramref name="s"/> is not a vertex.</exception>
    public DsPoint(in Shape s)
    {
        _point = ToolShapeTool.Pnt(s);
        _tolerance = ToolShapeTool.Tolerance(s);
        // OCCT leaves myKeep unset here (an uninitialised member); the port keeps the point, as the other constructors
        _keep = true;
    }

    /// <summary>Whether the two points are equal within the larger of their tolerances.</summary>
    public readonly bool IsEqual(in DsPoint p)
    {
        double t = Max(_tolerance, p.Tolerance());
        bool b = _point.IsEqual(p.Point(), t);
        return b;
    }

    /// <summary>The 3D point.</summary>
    public readonly Point3D Point() => _point;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Point3D ChangePoint() => ref _point;

    /// <summary>The tolerance.</summary>
    public readonly double Tolerance() => _tolerance;

    internal void Tolerance(double tol)
    {
        _tolerance = tol;
    }

    /// <summary>Whether the point is kept (not removed from the data structure).</summary>
    public readonly bool Keep() => _keep;

    internal void ChangeKeep(bool b)
    {
        _keep = b;
    }

    /// <summary>A copy with the tolerance <paramref name="tol"/>.</summary>
    public readonly DsPoint WithTolerance(double tol)
    {
        DsPoint copy = this;
        copy.Tolerance(tol);
        return copy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(DsPoint other)
        => _point.Equals(other._point) && _tolerance.Equals(other._tolerance) && _keep == other._keep;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is DsPoint other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => HashCode.Combine(_point, _tolerance, _keep);

    public static bool operator ==(in DsPoint left, in DsPoint right) => left.Equals(right);

    public static bool operator !=(in DsPoint left, in DsPoint right) => !left.Equals(right);

    /// <inheritdoc/>
    public override readonly string ToString()
        => "TopOpeBRepDS_Point(" + _point.ToString() + ", " + _tolerance.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + (_keep ? ")" : ", not kept)");
}
