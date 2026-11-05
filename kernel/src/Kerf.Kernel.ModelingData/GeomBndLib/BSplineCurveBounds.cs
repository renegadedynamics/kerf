// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a 3D B-spline curve (Geom_BSplineCurve): the curve is segmented to the range, sampled
/// span by span with a deflection estimate, and the result reduced by the poles' convex hull.
/// </summary>
internal sealed class BSplineCurveBounds : CurveBounds.ICurveEvaluator
{
    private readonly BSplineCurve3D _geom;

    public BSplineCurveBounds(BSplineCurve3D curve)
    {
        _geom = curve;
    }

    public BSplineCurve3D Geometry() => _geom;

    Curve3D? CurveBounds.ICurveEvaluator.CurveGeometry() => _geom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol) => Box(_geom.FirstParameter, _geom.LastParameter, tol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol)
    {
        const double weakness = 1.5;
        BSplineCurve3D curve = _geom;
        double aU1 = u1;
        double aU2 = u2;

        if (Math.Abs(curve.FirstParameter - aU1) > global::Kerf.Engine.Foundation.Precision.Parametric(tol)
            || Math.Abs(curve.LastParameter - aU2) > global::Kerf.Engine.Foundation.Precision.Parametric(tol))
        {
            Geometry3D geometry = curve.Copy();
            BSplineCurve3D segmentedCurve = (BSplineCurve3D)geometry;
            if (segmentedCurve.IsPeriodic)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(segmentedCurve.FirstParameter,
                                      segmentedCurve.LastParameter,
                                      global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                      ref aU1,
                                      ref aU2);
            }
            else
            {
                if (segmentedCurve.FirstParameter > aU1)
                {
                    aU1 = segmentedCurve.FirstParameter;
                }
                if (segmentedCurve.LastParameter < aU2)
                {
                    aU2 = segmentedCurve.LastParameter;
                }
            }

            double segmentTol = 2.0 * global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (segmentedCurve.IsPeriodic)
            {
                double period = segmentedCurve.LastParameter - segmentedCurve.FirstParameter;
                double directDiff = Math.Abs(aU2 - aU1);
                double crossPeriodDiff1 = Math.Abs(aU2 - period - aU1);
                double crossPeriodDiff2 = Math.Abs(aU1 - period - aU2);
                double minDiff = Min(directDiff, Min(crossPeriodDiff1, crossPeriodDiff2));
                if (minDiff < segmentTol)
                {
                    // OCCT: aMinDiff is 0 when U2 ends a whole period from U1 to the last bit (U2 = Last - an ulp);
                    // Segment with a tolerance of 0 then leaves a pole unwritten, which OCCT reads uninitialised and
                    // the port reads as 0 (occt-issues.md).
                    segmentTol = minDiff * 0.01;
                }
            }
            else if (Math.Abs(aU2 - aU1) < segmentTol)
            {
                segmentTol = Math.Abs(aU2 - aU1) * 0.01;
            }
            segmentedCurve.Segment(aU1, aU2, segmentTol);
            curve = segmentedCurve;
        }

        Box sampledBox = new();
        int knotFirst = curve.FirstUKnotIndex();
        int knotLast = curve.LastUKnotIndex();
        int degree = curve.Degree;
        NCollection_Array1<double> knots = curve.InternalKnots();
        CurveAdaptor gaCurve = new(curve);
        double maxDeflection = 0.0;
        double first = knots[knotFirst];
        for (int knot = knotFirst + 1; knot <= knotLast; ++knot)
        {
            double last = knots[knot];
            maxDeflection = Max(SplineHelpers.FillBox(ref sampledBox, gaCurve, first, last, degree),
                                 maxDeflection);
            first = last;
        }

        Box box = new();
        if (!sampledBox.IsVoid)
        {
            sampledBox.Enlarge(weakness * maxDeflection);
            // The poles of the whole curve, as OCCT does, also when the curve was segmented.
            SplineHelpers.ReduceSplineBox(_geom.InternalPoles(), sampledBox, ref box);
            box.Enlarge(tol);
        }
        return box;
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    public Box BoxOptimal(double u1, double u2, double tol)
    {
        CurveAdaptor gaCurve = new(_geom);
        return SplineHelpers.CurveBoxOptimal(gaCurve, u1, u2, tol);
    }
}
