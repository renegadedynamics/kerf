// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Occt.StepData_Logical;
using static Occt.StepGeom_BSplineCurveForm;
using static Occt.StepGeom_KnotType;

namespace Occt;

/// <summary>
/// The mapping of BSplineCurve from Geom and Geom2d to a STEP rational b_spline_curve_with_knots (the complex
/// entity BSplineCurveWithKnotsAndRationalBSplineCurve).
/// </summary>
public sealed class GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve : GeomToStep_Root
{
    private StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve theBSplineCurveWithKnotsAndRationalBSplineCurve;

    //=============================================================================
    // Creation d' une bspline_curve_with_knots_and_rational_bspline_curve de
    // prostep a partir d' une BSplineCurve de Geom
    //=============================================================================

    /// <summary>A 3D B-spline curve with its weights, its poles divided by the length factor.</summary>
    public GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BSplineCurve3D BS, in StepData_Factors theLocalFactors)
    {
        int N = BS.PoleCount;
        NCollection_Array1<Point3D> P = BS.InternalPoles();
        NCollection_HArray1<StepGeom_CartesianPoint> Listpoints = new NCollection_HArray1<StepGeom_CartesianPoint>(1, N);
        for (int i = P.Lower(); i <= P.Upper(); i++)
        {
            GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(P.Value(i), theLocalFactors.LengthFactor());
            Listpoints.SetValue(i, MkPoint.Value());
        }
        Make(BS.Degree(), N, Listpoints, BS.IsClosed, BS.KnotCount, BS.InternalMultiplicities(), BS.InternalKnots(),
             BS.KnotDistribution(), BS.InternalWeightsArray());
    }

    /// <summary>A 3D B-spline curve with its weights (default factors).</summary>
    public GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BSplineCurve3D BS) : this(BS, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une bspline_curve_with_knots_and_rational_bspline_curve de
    // prostep a partir d' une BSplineCurve de Geom2d
    //=============================================================================

    /// <summary>A 2D B-spline curve with its weights.</summary>
    public GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BSplineCurve2D BS, in StepData_Factors theLocalFactors)
    {
        int N = BS.PoleCount;
        NCollection_Array1<Point2D> P = BS.InternalPoles();
        NCollection_HArray1<StepGeom_CartesianPoint> Listpoints = new NCollection_HArray1<StepGeom_CartesianPoint>(1, N);
        for (int i = P.Lower(); i <= P.Upper(); i++)
        {
            GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(P.Value(i), theLocalFactors.LengthFactor());
            Listpoints.SetValue(i, MkPoint.Value());
        }
        Make(BS.Degree(), N, Listpoints, BS.IsClosed, BS.KnotCount, BS.InternalMultiplicities(), BS.InternalKnots(),
             BS.KnotDistribution(), BS.InternalWeightsArray());
    }

    /// <summary>A 2D B-spline curve with its weights.</summary>
    public GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BSplineCurve2D BS) : this(BS, new StepData_Factors())
    {
    }

    // GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve_gen.pxx, the body OCCT includes for both curve
    // types (its poles made first, in the same order)
    private void Make(int Deg,
                      int N,
                      NCollection_HArray1<StepGeom_CartesianPoint> Listpoints,
                      bool isClosed,
                      int Nknots,
                      NCollection_Array1<int> M,
                      NCollection_Array1<double> K,
                      BSplKnotDistribution Distribution,
                      NCollection_Array1<double> W)
    {
        StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve BSWK;
        int i, itampon;
        double rtampon;
        StepGeom_BSplineCurveForm Form;
        StepData_Logical Fermeture, Selfinter;
        NCollection_HArray1<int> Mult;
        NCollection_HArray1<double> ListKnots, ListWeights;
        StepGeom_KnotType KnotSpec;

        Form = StepGeom_bscfUnspecified;
        if (isClosed)
        {
            Fermeture = StepData_LTrue;
        }
        else
        {
            Fermeture = StepData_LFalse;
        }
        Selfinter = StepData_LFalse;
        Mult = new NCollection_HArray1<int>(1, Nknots);
        for (i = M.Lower(); i <= M.Upper(); i++)
        {
            itampon = M.Value(i);
            Mult.SetValue(i, itampon);
        }
        ListKnots = new NCollection_HArray1<double>(1, Nknots);
        for (i = K.Lower(); i <= K.Upper(); i++)
        {
            rtampon = K.Value(i);
            ListKnots.SetValue(i, rtampon);
        }
        if (Distribution == NonUniform)
        {
            KnotSpec = StepGeom_ktUnspecified;
        }
        else if (Distribution == Uniform)
        {
            KnotSpec = StepGeom_ktUniformKnots;
        }
        else if (Distribution == QuasiUniform)
        {
            KnotSpec = StepGeom_ktQuasiUniformKnots;
        }
        else
        {
            KnotSpec = StepGeom_ktPiecewiseBezierKnots;
        }
        ListWeights = new NCollection_HArray1<double>(1, N);
        for (i = W.Lower(); i <= W.Upper(); i++)
        {
            ListWeights.SetValue(i, W.Value(i));
        }

        BSWK = new StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        BSWK.Init(name, Deg, Listpoints, Form, Fermeture, Selfinter, Mult, ListKnots, KnotSpec, ListWeights);
        theBSplineCurveWithKnotsAndRationalBSplineCurve = BSWK;
        done = true;
    }

    /// <summary>The rational b_spline_curve_with_knots made.</summary>
    public StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve::Value() - no result");
        return theBSplineCurveWithKnotsAndRationalBSplineCurve;
    }
}
