// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PntOn2S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PntOn2S.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PntOn2S.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// An intersection point between two surfaces: its 3D coordinates and its parameters (U1, V1) on the first surface
/// and (U2, V2) on the second.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct: OCCT copies it into lines and vertices. <c>new()</c> and <c>default</c> are the
/// same (the origin, zero parameters). The <c>SetValue</c> overloads are internal mutators (translation rules,
/// section 3); <see cref="WithValue(in Point3D, double, double, double, double)"/> is the functional twin of the one
/// that sets everything.
/// </remarks>
internal partial struct PntOn2S : IEquatable<PntOn2S>
{
    private Point3D _pt;
    private double _u1;
    private double _v1;
    private double _u2;
    private double _v2;

    /// <summary>Empty constructor: the origin, zero parameters.</summary>
    public PntOn2S()
    {
        _pt = new Point3D(0, 0, 0);
        _u1 = 0;
        _v1 = 0;
        _u2 = 0;
        _v2 = 0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the point in 3D space.</summary>
    internal void SetValue(in Point3D pt) => _pt = pt;

    /// <summary>Sets the point in 3D space and in the parametric space of one of the surfaces.</summary>
    internal void SetValue(in Point3D pt, bool onFirst, double u, double v)
    {
        _pt = pt;
        if (onFirst)
        {
            _u1 = u;
            _v1 = v;
        }
        else
        {
            _u2 = u;
            _v2 = v;
        }
    }

    /// <summary>Sets the point in 3D space and in the parametric space of each surface.</summary>
    internal void SetValue(in Point3D pt, double u1, double v1, double u2, double v2)
    {
        _pt = pt;
        _u1 = u1;
        _v1 = v1;
        _u2 = u2;
        _v2 = v2;
    }

    /// <summary>Sets the point in the parametric space of one of the surfaces.</summary>
    internal void SetValue(bool onFirst, double u, double v)
    {
        if (onFirst)
        {
            _u1 = u;
            _v1 = v;
        }
        else
        {
            _u2 = u;
            _v2 = v;
        }
    }

    /// <summary>Sets the point in the parametric space of each surface.</summary>
    internal void SetValue(double u1, double v1, double u2, double v2)
    {
        _u1 = u1;
        _v1 = v1;
        _u2 = u2;
        _v2 = v2;
    }

    /// <summary>A copy with the point and both pairs of parameters set.</summary>
    public readonly PntOn2S WithValue(in Point3D pt, double u1, double v1, double u2, double v2)
    {
        PntOn2S copy = this;
        copy.SetValue(pt, u1, v1, u2, v2);
        return copy;
    }

    // Queries

    /// <summary>The point in 3D space.</summary>
    public readonly Point3D Value() => _pt;

    /// <summary>The point in the parametric space of one of the surfaces.</summary>
    public readonly Point2D ValueOnSurface(bool onFirst)
    {
        Point2D pointOnSurf = new();
        if (onFirst)
        {
            pointOnSurf = pointOnSurf.WithCoord(_u1, _v1);
        }
        else
        {
            pointOnSurf = pointOnSurf.WithCoord(_u2, _v2);
        }
        return pointOnSurf;
    }

    /// <summary>The parameters on the first surface.</summary>
    public readonly void ParametersOnS1(out double u1, out double v1)
    {
        u1 = _u1;
        v1 = _v1;
    }

    /// <summary>The parameters on the second surface.</summary>
    public readonly void ParametersOnS2(out double u2, out double v2)
    {
        u2 = _u2;
        v2 = _v2;
    }

    /// <summary>The parameters on one of the surfaces.</summary>
    public readonly void ParametersOnSurface(bool onFirst, out double u, out double v)
    {
        if (onFirst)
        {
            u = _u1;
            v = _v1;
        }
        else
        {
            u = _u2;
            v = _v2;
        }
    }

    /// <summary>The parameters on both surfaces.</summary>
    public readonly void Parameters(out double u1, out double v1, out double u2, out double v2)
    {
        u1 = _u1;
        v1 = _v1;
        u2 = _u2;
        v2 = _v2;
    }

    /// <summary>
    /// True if the 3D points are within <paramref name="tol3D"/> and, unless <paramref name="tol2D"/> is
    /// negative, the parameters on each surface within <paramref name="tol2D"/>.
    /// </summary>
    public readonly bool IsSame(in PntOn2S otherPoint, double tol3D = 0.0, double tol2D = -1.0)
    {
        if (_pt.SquareDistanceTo(otherPoint.Value()) > tol3D * tol3D)
        {
            return false;
        }

        if (tol2D < 0.0)
        { // We need not compare 2D-coordinates of the points
            return true;
        }

        otherPoint.Parameters(out double u1, out double v1, out double u2, out double v2);

        Point2D p1 = new(_u1, _v1), p2 = new(u1, v1);

        if (!p1.IsEqual(p2, tol2D))
        {
            return false;
        }

        p1 = p1.WithCoord(_u2, _v2);
        p2 = p2.WithCoord(u2, v2);

        return p1.IsEqual(p2, tol2D);
    }

    public readonly bool Equals(PntOn2S other)
        => _pt.Equals(other._pt) && _u1.Equals(other._u1) && _v1.Equals(other._v1) && _u2.Equals(other._u2)
           && _v2.Equals(other._v2);

    public override readonly bool Equals(object? obj) => obj is PntOn2S other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pt, _u1, _v1, _u2, _v2);

    public static bool operator ==(in PntOn2S left, in PntOn2S right) => left.Equals(right);

    public static bool operator !=(in PntOn2S left, in PntOn2S right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("IntSurf_PntOn2S", _pt.X, _pt.Y, _pt.Z, _u1, _v1, _u2, _v2);
}
