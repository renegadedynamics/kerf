// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedCurve.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Geometry.Conversion;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The mapping of BoundedCurve from Geom and Geom2d to a STEP bounded_curve: B-spline curves (periodic ones made
/// non-periodic on a copy) and Bezier curves (converted to B-splines).
/// </summary>
public sealed class GeomToStep_MakeBoundedCurve : GeomToStep_Root
{
    private StepGeom_BoundedCurve theBoundedCurve;

    //=============================================================================
    // Creation d' une BoundedCurve de prostep a partir d' une BoundedCurve de Geom
    //=============================================================================

    /// <summary>A 3D bounded curve.</summary>
    public GeomToStep_MakeBoundedCurve(BoundedCurve3D C, in StepData_Factors theLocalFactors)
    {
        done = true;

        if (C is BSplineCurve3D)
        {
            BSplineCurve3D Bspli = C as BSplineCurve3D;
            // UPDATE FMA 1-04-96
            if (C.IsPeriodic)
            {
                BSplineCurve3D newBspli = Bspli.Copy() as BSplineCurve3D;
                newBspli.SetNotPeriodic();
                Bspli = newBspli;
            }
            if (Bspli.IsRational)
            {
                GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve MkRatBSplineC =
                    new GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(Bspli, theLocalFactors);
                theBoundedCurve = MkRatBSplineC.Value();
            }
            else
            {
                GeomToStep_MakeBSplineCurveWithKnots MkBSplineC = new GeomToStep_MakeBSplineCurveWithKnots(Bspli, theLocalFactors);
                theBoundedCurve = MkBSplineC.Value();
            }
        }
        else if (C is BezierCurve3D)
        {
            BezierCurve3D Cur = C as BezierCurve3D;
            BSplineCurve3D Bspli = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(Cur);
            if (Bspli.IsRational)
            {
                GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve MkRatBSplineC =
                    new GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(Bspli, theLocalFactors);
                theBoundedCurve = MkRatBSplineC.Value();
            }
            else
            {
                GeomToStep_MakeBSplineCurveWithKnots MkBSplineC = new GeomToStep_MakeBSplineCurveWithKnots(Bspli, theLocalFactors);
                theBoundedCurve = MkBSplineC.Value();
            }
        }
        else
        {
            // OCCT_DEBUG output left out
            done = false;
        }
    }

    /// <summary>A 3D bounded curve (default factors).</summary>
    public GeomToStep_MakeBoundedCurve(BoundedCurve3D C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une BoundedCurve de prostep a partir d' une BoundedCurve de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D bounded curve (a 2D Bezier curve is converted without its weights, as OCCT does).</summary>
    public GeomToStep_MakeBoundedCurve(BoundedCurve2D C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is BSplineCurve2D)
        {
            BSplineCurve2D Bspli = C as BSplineCurve2D;
            // UPDATE FMA 1-04-96
            if (C.IsPeriodic)
            {
                BSplineCurve2D newBspli = Bspli.Copy() as BSplineCurve2D;
                newBspli.SetNotPeriodic();
                Bspli = newBspli;
            }
            if (Bspli.IsRational)
            {
                GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve MkRatBSplineC =
                    new GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(Bspli, theLocalFactors);
                theBoundedCurve = MkRatBSplineC.Value();
            }
            else
            {
                GeomToStep_MakeBSplineCurveWithKnots MkBSplineC = new GeomToStep_MakeBSplineCurveWithKnots(Bspli, theLocalFactors);
                theBoundedCurve = MkBSplineC.Value();
            }
        }
        else if (C is BezierCurve2D)
        {
            BezierCurve2D Cur = C as BezierCurve2D;
            BSplineCurve2D Bspli = Geom2dConvert2D.CurveToBSplineCurve(Cur);
            // OCCT: the rational case isn't distinguished here (the weights are lost; occt-issues.md, 8D)
            GeomToStep_MakeBSplineCurveWithKnots MkBSplineC = new GeomToStep_MakeBSplineCurveWithKnots(Bspli, theLocalFactors);
            theBoundedCurve = MkBSplineC.Value();
        }
        else
        {
            done = false;
        }
    }

    /// <summary>A 2D bounded curve.</summary>
    public GeomToStep_MakeBoundedCurve(BoundedCurve2D C) : this(C, new StepData_Factors())
    {
    }

    /// <summary>The bounded_curve made.</summary>
    public StepGeom_BoundedCurve Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeBoundedCurve::Value() - no result");
        return theBoundedCurve;
    }
}
