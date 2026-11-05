// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ThePathPointOfTheSOnBounds.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ThePathPointOfTheSOnBounds_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// A solution point of the boundary search (<see cref="TheSOnBounds"/>): the point, its tolerance, the arc
/// and the parameter on it, and the vertex of the domain when the point is one (not new).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 609). <c>new()</c> is OCCT's empty constructor (a new point);
/// <c>default</c> is not new, so ported code writes <c>new()</c>. The <c>SetValue</c> overloads are internal
/// mutators (reached by the boundary search through <see cref="IPathPoint{TSelf}"/>); the constructors are
/// their twins. The arc and the vertex are compared by reference (OCCT's handles).
/// </remarks>
internal partial struct ThePathPointOfTheSOnBounds
    : IPathPoint<ThePathPointOfTheSOnBounds>, IEquatable<ThePathPointOfTheSOnBounds>
{
    private Point3D _point;
    private double _tol;
    private bool _isnew;
    private HVertex? _vtx;
    private Curve2DAdaptorBase? _arc;
    private double _param;

    public ThePathPointOfTheSOnBounds()
    {
        _point = new Point3D();
        _tol = 0.0;
        _isnew = true;
        _vtx = null;
        _arc = null;
        _param = 0.0;
    }

    /// <summary>A point on the vertex <paramref name="v"/> of the arc <paramref name="a"/>.</summary>
    public ThePathPointOfTheSOnBounds(in Point3D p, double tol, HVertex v, Curve2DAdaptorBase a,
                                               double parameter)
    {
        _point = p;
        _tol = tol;
        _isnew = false;
        _vtx = v;
        _arc = a;
        _param = parameter;
    }

    /// <summary>A new point on the arc <paramref name="a"/>.</summary>
    public ThePathPointOfTheSOnBounds(in Point3D p, double tol, Curve2DAdaptorBase a, double parameter)
    {
        _point = p;
        _tol = tol;
        _isnew = true;
        _vtx = null;
        _arc = a;
        _param = parameter;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in Point3D p, double tol, HVertex v, Curve2DAdaptorBase a, double parameter)
    {
        _isnew = false;
        _point = p;
        _tol = tol;
        _vtx = v;
        _arc = a;
        _param = parameter;
    }

    /// <summary>Sets a new point (the vertex field is kept, as in OCCT; it is not readable on a new point).</summary>
    internal void SetValue(in Point3D p, double tol, Curve2DAdaptorBase a, double parameter)
    {
        _isnew = true;
        _point = p;
        _tol = tol;
        _arc = a;
        _param = parameter;
    }

    // Queries

    public readonly Point3D Value() => _point;

    public readonly double Tolerance() => _tol;

    /// <summary>True if the point is not a vertex of the domain.</summary>
    public readonly bool IsNew() => _isnew;

    /// <summary>The vertex; throws Standard_DomainError on a new point.</summary>
    public readonly HVertex Vertex()
    {
        if (_isnew)
        {
            throw new DomainErrorException();
        }
        return _vtx!;
    }

    public readonly Curve2DAdaptorBase Arc() => _arc!;

    public readonly double Parameter() => _param;

    static ThePathPointOfTheSOnBounds IPathPoint<ThePathPointOfTheSOnBounds>.Create(
        in Point3D p, double tol, HVertex v, Curve2DAdaptorBase a, double parameter) => new(p, tol, v, a, parameter);

    static ThePathPointOfTheSOnBounds IPathPoint<ThePathPointOfTheSOnBounds>.Create(
        in Point3D p, double tol, Curve2DAdaptorBase a, double parameter) => new(p, tol, a, parameter);

    void IPathPoint<ThePathPointOfTheSOnBounds>.SetValue(in Point3D p, double tol, HVertex v,
                                                                             Curve2DAdaptorBase a, double parameter) =>
        SetValue(p, tol, v, a, parameter);

    void IPathPoint<ThePathPointOfTheSOnBounds>.SetValue(in Point3D p, double tol, Curve2DAdaptorBase a,
                                                                             double parameter) =>
        SetValue(p, tol, a, parameter);

    public readonly bool Equals(ThePathPointOfTheSOnBounds other)
        => _point.Equals(other._point) && _tol.Equals(other._tol) && _isnew == other._isnew && ReferenceEquals(_vtx, other._vtx)
           && ReferenceEquals(_arc, other._arc) && _param.Equals(other._param);

    public override readonly bool Equals(object? obj) => obj is ThePathPointOfTheSOnBounds other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_point, _tol, _isnew, _vtx, _arc, _param);

    public static bool operator ==(in ThePathPointOfTheSOnBounds left,
                                   in ThePathPointOfTheSOnBounds right) => left.Equals(right);

    public static bool operator !=(in ThePathPointOfTheSOnBounds left,
                                   in ThePathPointOfTheSOnBounds right) => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("IntPatch_ThePathPointOfTheSOnBounds", _point.X, _point.Y, _point.Z, _tol, _param)} new {_isnew}";
}
