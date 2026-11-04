// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_InteriorPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_InteriorPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_InteriorPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// A solution point of the intersection between an implicit and a parametric surface inside the domain: a passing
/// point of an intersection line, or a starting point of a closed line on the parametric surface. It holds the 3D
/// point, its parameters on the parametric surface and the tangents of the line in 3D and in the parametric space.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same. <see cref="SetValue"/> is an internal
/// mutator; the constructor is its twin.
/// </remarks>
internal partial struct InteriorPoint : IEquatable<InteriorPoint>
{
    private Point3D _point;
    private double _paramu;
    private double _paramv;
    private Vector3D _direc;
    private Vector2D _direc2d;

    public InteriorPoint()
    {
        _point = new Point3D();
        _paramu = 0.0;
        _paramv = 0.0;
        _direc = new Vector3D();
        _direc2d = new Vector2D();
    }

    public InteriorPoint(in Point3D p, double u, double v, in Vector3D direc, in Vector2D direc2d)
    {
        _point = p;
        _paramu = u;
        _paramv = v;
        _direc = direc;
        _direc2d = direc2d;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in Point3D p, double u, double v, in Vector3D direc, in Vector2D direc2d)
    {
        _point = p;
        _paramu = u;
        _paramv = v;
        _direc = direc;
        _direc2d = direc2d;
    }

    // Queries

    /// <summary>The 3D point.</summary>
    public readonly Point3D Value() => _point;

    /// <summary>The parameters on the parametric surface.</summary>
    public readonly void Parameters(out double u, out double v)
    {
        u = _paramu;
        v = _paramv;
    }

    public readonly double UParameter() => _paramu;

    public readonly double VParameter() => _paramv;

    /// <summary>The tangent of the intersection in 3D.</summary>
    public readonly Vector3D Direction() => _direc;

    /// <summary>The tangent of the intersection in the parametric space of the parametric surface.</summary>
    public readonly Vector2D Direction2d() => _direc2d;

    public readonly bool Equals(InteriorPoint other)
        => _point.Equals(other._point) && _paramu.Equals(other._paramu) && _paramv.Equals(other._paramv)
           && _direc.Equals(other._direc) && _direc2d.Equals(other._direc2d);

    public override readonly bool Equals(object? obj) => obj is InteriorPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_point, _paramu, _paramv, _direc, _direc2d);

    public static bool operator ==(in InteriorPoint left, in InteriorPoint right) => left.Equals(right);

    public static bool operator !=(in InteriorPoint left, in InteriorPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => Constants.Format("IntSurf_InteriorPoint", _point.X, _point.Y, _point.Z, _paramu, _paramv, _direc.X, _direc.Y,
                     _direc.Z, _direc2d.X, _direc2d.Y);
}
