// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D Bezier curve (Geom_BezierCurve): the poles' convex hull reduced by sampling with a
/// deflection estimate.
/// </summary>
internal sealed class BezierCurveBounds : CurveBounds.ICurveEvaluator
{
    private readonly BezierCurve3D _geom;

    public BezierCurveBounds(BezierCurve3D curve)
    {
        _geom = curve;
    }

    public BezierCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol)
    {
        const double weakness = 1.5;
        CurveAdaptor gaCurve = new(_geom);
        Box sampledBox = new();
        double deflection = SplineHelpers.FillBox(ref sampledBox, gaCurve, u1, u2, _geom.Degree);
        sampledBox.Enlarge(weakness * deflection);
        Box box = new();
        SplineHelpers.ReduceSplineBox(_geom.InternalPoles(), sampledBox, ref box);
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    public Box BoxOptimal(double u1, double u2, double tol)
    {
        CurveAdaptor gaCurve = new(_geom);
        return SplineHelpers.CurveBoxOptimal(gaCurve, u1, u2, tol);
    }
}
