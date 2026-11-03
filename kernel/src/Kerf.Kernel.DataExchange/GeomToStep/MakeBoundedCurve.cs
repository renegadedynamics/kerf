// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedCurve.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry.Conversion;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>
/// The mapping of BoundedCurve from Geom and Geom2d to a STEP bounded_curve: B-spline curves (periodic ones made
/// non-periodic on a copy) and Bezier curves (converted to B-splines).
/// </summary>
public sealed class MakeBoundedCurve : GeometryRoot
{
    private StepBoundedCurve _boundedCurve;

    //=============================================================================
    // Creation d' une BoundedCurve de prostep a partir d' une BoundedCurve de Geom
    //=============================================================================

    /// <summary>A 3D bounded curve.</summary>
    public MakeBoundedCurve(BoundedCurve3D c, in Factors localFactors)
    {
        _done = true;

        if (c is BSplineCurve3D)
        {
            BSplineCurve3D bspli = c as BSplineCurve3D;
            // UPDATE FMA 1-04-96
            if (c.IsPeriodic)
            {
                BSplineCurve3D newBspli = bspli.Copy() as BSplineCurve3D;
                newBspli.SetNotPeriodic();
                bspli = newBspli;
            }
            if (bspli.IsRational)
            {
                MakeBSplineCurveWithKnotsAndRationalBSplineCurve mkRatBSplineC =
                    new MakeBSplineCurveWithKnotsAndRationalBSplineCurve(bspli, localFactors);
                _boundedCurve = mkRatBSplineC.Value();
            }
            else
            {
                MakeBSplineCurveWithKnots mkBSplineC = new MakeBSplineCurveWithKnots(bspli, localFactors);
                _boundedCurve = mkBSplineC.Value();
            }
        }
        else if (c is BezierCurve3D)
        {
            BezierCurve3D cur = c as BezierCurve3D;
            BSplineCurve3D bspli = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(cur);
            if (bspli.IsRational)
            {
                MakeBSplineCurveWithKnotsAndRationalBSplineCurve mkRatBSplineC =
                    new MakeBSplineCurveWithKnotsAndRationalBSplineCurve(bspli, localFactors);
                _boundedCurve = mkRatBSplineC.Value();
            }
            else
            {
                MakeBSplineCurveWithKnots mkBSplineC = new MakeBSplineCurveWithKnots(bspli, localFactors);
                _boundedCurve = mkBSplineC.Value();
            }
        }
        else
        {
            // OCCT_DEBUG output left out
            _done = false;
        }
    }

    /// <summary>A 3D bounded curve (default factors).</summary>
    public MakeBoundedCurve(BoundedCurve3D c) : this(c, new Factors())
    {
    }

    //=============================================================================
    // Creation d' une BoundedCurve de prostep a partir d' une BoundedCurve de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D bounded curve.</summary>
    public MakeBoundedCurve(BoundedCurve2D c, in Factors localFactors)
    {
        _done = true;
        if (c is BSplineCurve2D)
        {
            BSplineCurve2D bspli = c as BSplineCurve2D;
            // UPDATE FMA 1-04-96
            if (c.IsPeriodic)
            {
                BSplineCurve2D newBspli = bspli.Copy() as BSplineCurve2D;
                newBspli.SetNotPeriodic();
                bspli = newBspli;
            }
            if (bspli.IsRational)
            {
                MakeBSplineCurveWithKnotsAndRationalBSplineCurve mkRatBSplineC =
                    new MakeBSplineCurveWithKnotsAndRationalBSplineCurve(bspli, localFactors);
                _boundedCurve = mkRatBSplineC.Value();
            }
            else
            {
                MakeBSplineCurveWithKnots mkBSplineC = new MakeBSplineCurveWithKnots(bspli, localFactors);
                _boundedCurve = mkBSplineC.Value();
            }
        }
        else if (c is BezierCurve2D)
        {
            BezierCurve2D cur = c as BezierCurve2D;
            BSplineCurve2D bspli = Geom2dConvert2D.CurveToBSplineCurve(cur);
            // a rational one with its weights, as the 3D case and the 2D B-splines (OCCT lost them; Kerf decision 16824)
            if (bspli.IsRational)
            {
                MakeBSplineCurveWithKnotsAndRationalBSplineCurve mkRatBSplineC =
                    new MakeBSplineCurveWithKnotsAndRationalBSplineCurve(bspli, localFactors);
                _boundedCurve = mkRatBSplineC.Value();
            }
            else
            {
                MakeBSplineCurveWithKnots mkBSplineC = new MakeBSplineCurveWithKnots(bspli, localFactors);
                _boundedCurve = mkBSplineC.Value();
            }
        }
        else
        {
            _done = false;
        }
    }

    /// <summary>A 2D bounded curve.</summary>
    public MakeBoundedCurve(BoundedCurve2D c) : this(c, new Factors())
    {
    }

    /// <summary>The bounded_curve made.</summary>
    public StepBoundedCurve Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeBoundedCurve::Value() - no result");
        return _boundedCurve;
    }
}
