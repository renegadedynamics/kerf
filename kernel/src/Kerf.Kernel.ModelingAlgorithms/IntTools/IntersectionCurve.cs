// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Curve.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// One 3D curve, its two 2D curves and two tolerances: a result of the face/face intersection. The 3D curve is the
/// intersection curve, the 2D curves its p-curves on the faces; the tolerance is the largest deviation between the 3D
/// curve and the 2D curves (or the surfaces), the tangential tolerance the largest distance from the 3D curve to the
/// end of the tangential zone between the faces.
/// </summary>
/// <remarks>
/// A struct, as OCCT's by-value class: a copy shares the curves (handles), as in C++. The setters are internal
/// (translation rules, section 3).
/// </remarks>
public partial struct IntersectionCurve
{
    private Curve3D? _my3dCurve;
    private Curve2D? _my2dCurve1;
    private Curve2D? _my2dCurve2;
    private double _tolerance;
    private double _tangentialTolerance;

    /// <summary>An empty curve.</summary>
    public IntersectionCurve()
    {
        _tolerance = 0.0;
        _tangentialTolerance = 0.0;
    }

    /// <summary>A 3D curve, two 2D curves and two tolerances.</summary>
    public IntersectionCurve(Curve3D? the3dCurve, Curve2D? the2dCurve1, Curve2D? the2dCurve2,
                          double tolerance = 0.0, double tangentialTolerance = 0.0)
    {
        _tolerance = tolerance;
        _tangentialTolerance = tangentialTolerance;
        SetCurves(the3dCurve, the2dCurve1, the2dCurve2);
    }

    internal void SetCurves(Curve3D? the3dCurve, Curve2D? the2dCurve1, Curve2D? the2dCurve2)
    {
        _my3dCurve = the3dCurve;
        _my2dCurve1 = the2dCurve1;
        _my2dCurve2 = the2dCurve2;
    }

    internal void SetCurve(Curve3D? the3dCurve) => _my3dCurve = the3dCurve;

    internal void SetFirstCurve2d(Curve2D? the2dCurve1) => _my2dCurve1 = the2dCurve1;

    internal void SetSecondCurve2d(Curve2D? the2dCurve2) => _my2dCurve2 = the2dCurve2;

    internal void SetTolerance(double tolerance) => _tolerance = tolerance;

    internal void SetTangentialTolerance(double tangentialTolerance) => _tangentialTolerance = tangentialTolerance;

    /// <summary>The 3D curve.</summary>
    public readonly Curve3D? Curve() => _my3dCurve;

    /// <summary>The first 2D curve.</summary>
    public readonly Curve2D? FirstCurve2d() => _my2dCurve1;

    /// <summary>The second 2D curve.</summary>
    public readonly Curve2D? SecondCurve2d() => _my2dCurve2;

    public readonly double Tolerance() => _tolerance;

    public readonly double TangentialTolerance() => _tangentialTolerance;

    /// <summary>True if the 3D curve is a bounded curve.</summary>
    public readonly bool HasBounds()
    {
        bool bIsBounded = _my3dCurve is BoundedCurve3D;
        return bIsBounded;
    }

    /// <summary>
    /// If the 3D curve is bounded, its first and last parameters and points, and true; otherwise false, the outputs
    /// left as they were.
    /// </summary>
    public readonly bool Bounds(ref double first, ref double last, ref Point3D firstPnt, ref Point3D lastPnt)
    {
        bool bIsBounded = HasBounds();
        if (bIsBounded)
        {
            first = _my3dCurve!.FirstParameter;
            last = _my3dCurve.LastParameter;
            _my3dCurve.D0(first, out firstPnt);
            _my3dCurve.D0(last, out lastPnt);
        }
        return bIsBounded;
    }

    /// <summary>
    /// The 3D point at the parameter, and true, if the parameter is inside the curve's bounds; otherwise false, the
    /// point left as it was.
    /// </summary>
    public readonly bool D0(double par, ref Point3D pnt)
    {
        // OCCT: "||" where "&&" is meant, so every parameter is inside (occt-issues.md, unit 6A)
        bool bInside = par >= _my3dCurve!.FirstParameter || par <= _my3dCurve.LastParameter;
        if (bInside)
        {
            _my3dCurve.D0(par, out pnt);
        }
        return bInside;
    }

    /// <summary>The type of the 3D curve.</summary>
    public readonly CurveKind Type()
    {
        CurveAdaptor gac = new(_my3dCurve!);
        CurveKind type = gac.GetType();
        return type;
    }

    // ---- OCCT.NET additions: the setters' non-mutating twins

    public readonly IntersectionCurve WithCurves(Curve3D? the3dCurve, Curve2D? the2dCurve1, Curve2D? the2dCurve2)
    {
        IntersectionCurve copy = this;
        copy.SetCurves(the3dCurve, the2dCurve1, the2dCurve2);
        return copy;
    }

    public readonly IntersectionCurve WithTolerance(double tolerance)
    {
        IntersectionCurve copy = this;
        copy.SetTolerance(tolerance);
        return copy;
    }

    public readonly IntersectionCurve WithTangentialTolerance(double tangentialTolerance)
    {
        IntersectionCurve copy = this;
        copy.SetTangentialTolerance(tangentialTolerance);
        return copy;
    }
}
