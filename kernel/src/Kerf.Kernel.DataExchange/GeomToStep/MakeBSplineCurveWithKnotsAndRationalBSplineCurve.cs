// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Kerf.Engine.Exchange.Step.Data.Logical;
using static Kerf.Engine.Exchange.Step.Geometry.StepBSplineCurveForm;
using static Kerf.Engine.Exchange.Step.Geometry.StepKnotType;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>
/// The mapping of BSplineCurve from Geom and Geom2d to a STEP rational b_spline_curve_with_knots (the complex
/// entity BSplineCurveWithKnotsAndRationalBSplineCurve).
/// </summary>
internal sealed class MakeBSplineCurveWithKnotsAndRationalBSplineCurve : GeometryRoot
{
    private StepBSplineCurveWithKnotsAndRationalBSplineCurve _bSplineCurveWithKnotsAndRationalBSplineCurve;

    //=============================================================================
    // Creation d' une bspline_curve_with_knots_and_rational_bspline_curve de
    // prostep a partir d' une BSplineCurve de Geom
    //=============================================================================

    /// <summary>A 3D B-spline curve with its weights, its poles divided by the length factor.</summary>
    public MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BSplineCurve3D bs, in Factors localFactors)
    {
        int n = bs.PoleCount;
        NCollection_Array1<Point3D> p = bs.InternalPoles();
        NCollection_HArray1<StepCartesianPoint> listpoints = new NCollection_HArray1<StepCartesianPoint>(1, n);
        for (int i = p.Lower(); i <= p.Upper(); i++)
        {
            MakeCartesianPoint mkPoint = new MakeCartesianPoint(p.Value(i), localFactors.LengthFactor());
            listpoints.SetValue(i, mkPoint.Value());
        }
        Make(bs.Degree, n, listpoints, bs.IsClosed, bs.KnotCount, bs.InternalMultiplicities(), bs.InternalKnots(),
             bs.KnotDistribution, bs.InternalWeightsArray());
    }

    /// <summary>A 3D B-spline curve with its weights (default factors).</summary>
    public MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BSplineCurve3D bs) : this(bs, new Factors())
    {
    }

    //=============================================================================
    // Creation d' une bspline_curve_with_knots_and_rational_bspline_curve de
    // prostep a partir d' une BSplineCurve de Geom2d
    //=============================================================================

    /// <summary>A 2D B-spline curve with its weights.</summary>
    public MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BSplineCurve2D bs, in Factors localFactors)
    {
        int n = bs.PoleCount;
        NCollection_Array1<Point2D> p = bs.InternalPoles();
        NCollection_HArray1<StepCartesianPoint> listpoints = new NCollection_HArray1<StepCartesianPoint>(1, n);
        for (int i = p.Lower(); i <= p.Upper(); i++)
        {
            MakeCartesianPoint mkPoint = new MakeCartesianPoint(p.Value(i), localFactors.LengthFactor());
            listpoints.SetValue(i, mkPoint.Value());
        }
        Make(bs.Degree, n, listpoints, bs.IsClosed, bs.KnotCount, bs.InternalMultiplicities(), bs.InternalKnots(),
             bs.KnotDistribution, bs.InternalWeightsArray());
    }

    /// <summary>A 2D B-spline curve with its weights.</summary>
    public MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BSplineCurve2D bs) : this(bs, new Factors())
    {
    }

    // GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve_gen.pxx, the body OCCT includes for both curve
    // types (its poles made first, in the same order)
    private void Make(int deg,
                      int n,
                      NCollection_HArray1<StepCartesianPoint> listpoints,
                      bool isClosed,
                      int nknots,
                      NCollection_Array1<int> m,
                      NCollection_Array1<double> k,
                      BSplKnotDistribution distribution,
                      NCollection_Array1<double> w)
    {
        StepBSplineCurveWithKnotsAndRationalBSplineCurve bswk;
        int i, itampon;
        double rtampon;
        StepBSplineCurveForm form;
        Logical fermeture, selfinter;
        NCollection_HArray1<int> mult;
        NCollection_HArray1<double> listKnots, listWeights;
        StepKnotType knotSpec;

        form = StepBSplineCurveForm.BscfUnspecified;
        if (isClosed)
        {
            fermeture = Logical.LTrue;
        }
        else
        {
            fermeture = Logical.LFalse;
        }
        selfinter = Logical.LFalse;
        mult = new NCollection_HArray1<int>(1, nknots);
        for (i = m.Lower(); i <= m.Upper(); i++)
        {
            itampon = m.Value(i);
            mult.SetValue(i, itampon);
        }
        listKnots = new NCollection_HArray1<double>(1, nknots);
        for (i = k.Lower(); i <= k.Upper(); i++)
        {
            rtampon = k.Value(i);
            listKnots.SetValue(i, rtampon);
        }
        if (distribution == NonUniform)
        {
            knotSpec = StepKnotType.KtUnspecified;
        }
        else if (distribution == Uniform)
        {
            knotSpec = StepKnotType.KtUniformKnots;
        }
        else if (distribution == QuasiUniform)
        {
            knotSpec = StepKnotType.KtQuasiUniformKnots;
        }
        else
        {
            knotSpec = StepKnotType.KtPiecewiseBezierKnots;
        }
        listWeights = new NCollection_HArray1<double>(1, n);
        for (i = w.Lower(); i <= w.Upper(); i++)
        {
            listWeights.SetValue(i, w.Value(i));
        }

        bswk = new StepBSplineCurveWithKnotsAndRationalBSplineCurve();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        bswk.Init(name, deg, listpoints, form, fermeture, selfinter, mult, listKnots, knotSpec, listWeights);
        _bSplineCurveWithKnotsAndRationalBSplineCurve = bswk;
        _done = true;
    }

    /// <summary>The rational b_spline_curve_with_knots made.</summary>
    public StepBSplineCurveWithKnotsAndRationalBSplineCurve Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve::Value() - no result");
        return _bSplineCurveWithKnotsAndRationalBSplineCurve;
    }
}
