// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedCurve.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBoundedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

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
    public GeomToStep_MakeBoundedCurve(Geom_BoundedCurve C, in StepData_Factors theLocalFactors)
    {
        done = true;

        if (C is Geom_BSplineCurve)
        {
            Geom_BSplineCurve Bspli = C as Geom_BSplineCurve;
            // UPDATE FMA 1-04-96
            if (C.IsPeriodic())
            {
                Geom_BSplineCurve newBspli = Bspli.Copy() as Geom_BSplineCurve;
                newBspli.SetNotPeriodic();
                Bspli = newBspli;
            }
            if (Bspli.IsRational())
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
        else if (C is Geom_BezierCurve)
        {
            Geom_BezierCurve Cur = C as Geom_BezierCurve;
            Geom_BSplineCurve Bspli = GeomConvert.CurveToBSplineCurve(Cur);
            if (Bspli.IsRational())
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
    public GeomToStep_MakeBoundedCurve(Geom_BoundedCurve C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une BoundedCurve de prostep a partir d' une BoundedCurve de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D bounded curve (a 2D Bezier curve is converted without its weights, as OCCT does).</summary>
    public GeomToStep_MakeBoundedCurve(Geom2d_BoundedCurve C, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (C is Geom2d_BSplineCurve)
        {
            Geom2d_BSplineCurve Bspli = C as Geom2d_BSplineCurve;
            // UPDATE FMA 1-04-96
            if (C.IsPeriodic())
            {
                Geom2d_BSplineCurve newBspli = Bspli.Copy() as Geom2d_BSplineCurve;
                newBspli.SetNotPeriodic();
                Bspli = newBspli;
            }
            if (Bspli.IsRational())
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
        else if (C is Geom2d_BezierCurve)
        {
            Geom2d_BezierCurve Cur = C as Geom2d_BezierCurve;
            Geom2d_BSplineCurve Bspli = Geom2dConvert.CurveToBSplineCurve(Cur);
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
    public GeomToStep_MakeBoundedCurve(Geom2d_BoundedCurve C) : this(C, new StepData_Factors())
    {
    }

    /// <summary>The bounded_curve made.</summary>
    public StepGeom_BoundedCurve Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeBoundedCurve::Value() - no result");
        return theBoundedCurve;
    }
}
