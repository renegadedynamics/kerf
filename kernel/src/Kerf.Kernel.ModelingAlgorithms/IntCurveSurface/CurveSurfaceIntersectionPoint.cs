// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// An intersection point between a curve and a surface: the point, its parameters (U, V) on the surface and W on the
/// curve, and the transition of the curve through the surface.
/// </summary>
/// <remarks>
/// <c>new IntCurveSurface_IntersectionPoint()</c> and <c>default</c> are the same: the origin, zero parameters and a
/// tangent transition (OCCT's default). <c>SetValues</c> is an internal mutator; the constructor is its twin.
/// OCCT's <c>Dump</c> (to std::cout) is <see cref="ToString"/>.
/// </remarks>
internal partial struct CurveSurfaceIntersectionPoint : IEquatable<CurveSurfaceIntersectionPoint>
{
    private Point3D _p;
    private double _uSurf;
    private double _vSurf;
    private double _uCurv;
    private TransitionOnCurve _trOnCurv;

    public CurveSurfaceIntersectionPoint()
    {
        _p = new Point3D();
        _uSurf = 0.0;
        _vSurf = 0.0;
        _uCurv = 0.0;
        _trOnCurv = TransitionOnCurve.Tangent;
    }

    public CurveSurfaceIntersectionPoint(in Point3D p, double uSurf, double vSurf, double uCurv,
                                             TransitionOnCurve trOnCurv)
    {
        _p = p;
        _uSurf = uSurf;
        _vSurf = vSurf;
        _uCurv = uCurv;
        _trOnCurv = trOnCurv;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(in Point3D p, double uSurf, double vSurf, double uCurv,
                            TransitionOnCurve trOnCurv)
    {
        _p = p;
        _uSurf = uSurf;
        _vSurf = vSurf;
        _uCurv = uCurv;
        _trOnCurv = trOnCurv;
    }

    // Queries

    /// <summary>All the values of the point.</summary>
    public readonly void Values(out Point3D p, out double uSurf, out double vSurf, out double uCurv,
                                out TransitionOnCurve trOnCurv)
    {
        p = _p;
        uSurf = _uSurf;
        vSurf = _vSurf;
        uCurv = _uCurv;
        trOnCurv = _trOnCurv;
    }

    /// <summary>The point.</summary>
    public readonly Point3D Pnt() => _p;

    /// <summary>The U parameter on the surface.</summary>
    public readonly double U() => _uSurf;

    /// <summary>The V parameter on the surface.</summary>
    public readonly double V() => _vSurf;

    /// <summary>The parameter on the curve.</summary>
    public readonly double W() => _uCurv;

    /// <summary>The transition of the curve through the surface.</summary>
    public readonly TransitionOnCurve Transition() => _trOnCurv;

    public readonly bool Equals(CurveSurfaceIntersectionPoint other)
        => _p.Equals(other._p) && _uSurf.Equals(other._uSurf) && _vSurf.Equals(other._vSurf)
           && _uCurv.Equals(other._uCurv) && _trOnCurv == other._trOnCurv;

    public override readonly bool Equals(object? obj) => obj is CurveSurfaceIntersectionPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_p, _uSurf, _vSurf, _uCurv, _trOnCurv);

    public static bool operator ==(in CurveSurfaceIntersectionPoint left,
                                   in CurveSurfaceIntersectionPoint right) => left.Equals(right);

    public static bool operator !=(in CurveSurfaceIntersectionPoint left,
                                   in CurveSurfaceIntersectionPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("IntCurveSurface_IntersectionPoint", _p.X, _p.Y, _p.Z, _uSurf, _vSurf, _uCurv)} {_trOnCurv}";
}
