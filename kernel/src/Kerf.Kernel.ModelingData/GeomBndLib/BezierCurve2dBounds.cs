// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierCurve2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 2D Bezier curve (Geom2d_BezierCurve): the poles of the curve, segmented to the range
/// when it is a strict sub-range.
/// </summary>
public sealed class BezierCurve2dBounds : Curve2dBounds.ICurve2dEvaluator
{
    private readonly BezierCurve2D _geom;

    public BezierCurve2dBounds(BezierCurve2D curve)
    {
        _geom = curve;
    }

    public BezierCurve2D Geometry() => _geom;

    Curve2D? Curve2dBounds.ICurve2dEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double u1, double u2, double tol)
    {
        BezierCurve2D curve = _geom;
        double aU1 = curve.FirstParameter;
        double aU2 = curve.LastParameter;
        double trim1 = Max(u1, aU1);
        double trim2 = Min(u2, aU2);
        double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (Math.Abs(aU1 - trim1) > eps || Math.Abs(aU2 - trim2) > eps)
        {
            Geometry2D copy = curve.Copy();
            curve = (BezierCurve2D)copy;
            curve.Segment(trim1, trim2);
        }

        Box2d box = new();
        for (int idx = 1; idx <= curve.PoleCount; ++idx)
        {
            box.Add(curve.Pole(idx));
        }
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    public Box2d BoxOptimal(double u1, double u2, double tol)
    {
        CurveAdaptor2D gaCurve = new(_geom);
        return SplineHelpers.CurveBoxOptimal(gaCurve, u1, u2, tol);
    }
}
