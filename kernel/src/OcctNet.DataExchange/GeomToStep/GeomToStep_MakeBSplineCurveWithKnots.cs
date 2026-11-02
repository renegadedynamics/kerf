// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnots.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnots.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineCurveWithKnots_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.GeomAbs_BSplKnotDistribution;
using static Occt.StepData_Logical;
using static Occt.StepGeom_BSplineCurveForm;
using static Occt.StepGeom_KnotType;

namespace Occt;

/// <summary>The mapping of BSplineCurve from Geom and Geom2d to a STEP b_spline_curve_with_knots.</summary>
public sealed class GeomToStep_MakeBSplineCurveWithKnots : GeomToStep_Root
{
    private StepGeom_BSplineCurveWithKnots theBSplineCurveWithKnots;

    //=============================================================================
    // Creation d' une bspline_curve_with_knots de
    // prostep a partir d' une BSplineCurve de Geom
    //=============================================================================

    /// <summary>A 3D B-spline curve (its weights ignored), its poles divided by the length factor.</summary>
    public GeomToStep_MakeBSplineCurveWithKnots(Geom_BSplineCurve BS, in StepData_Factors theLocalFactors)
    {
        NCollection_Array1<gp_Pnt> P = BS.InternalPoles();
        NCollection_HArray1<StepGeom_CartesianPoint> Listpoints = new NCollection_HArray1<StepGeom_CartesianPoint>(1, BS.NbPoles());
        for (int i = P.Lower(); i <= P.Upper(); i++)
        {
            GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(P.Value(i), theLocalFactors.LengthFactor());
            Listpoints.SetValue(i, MkPoint.Value());
        }
        Make(BS.Degree(), Listpoints, BS.IsClosed(), BS.NbKnots(), BS.InternalMultiplicities(), BS.InternalKnots(), BS.KnotDistribution());
    }

    /// <summary>A 3D B-spline curve (default factors).</summary>
    public GeomToStep_MakeBSplineCurveWithKnots(Geom_BSplineCurve BS) : this(BS, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une bspline_curve_with_knots de
    // prostep a partir d' une BSplineCurve de Geom2d
    //=============================================================================

    /// <summary>A 2D B-spline curve (its weights ignored).</summary>
    public GeomToStep_MakeBSplineCurveWithKnots(Geom2d_BSplineCurve BS, in StepData_Factors theLocalFactors)
    {
        NCollection_Array1<gp_Pnt2d> P = BS.InternalPoles();
        NCollection_HArray1<StepGeom_CartesianPoint> Listpoints = new NCollection_HArray1<StepGeom_CartesianPoint>(1, BS.NbPoles());
        for (int i = P.Lower(); i <= P.Upper(); i++)
        {
            GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(P.Value(i), theLocalFactors.LengthFactor());
            Listpoints.SetValue(i, MkPoint.Value());
        }
        Make(BS.Degree(), Listpoints, BS.IsClosed(), BS.NbKnots(), BS.InternalMultiplicities(), BS.InternalKnots(), BS.KnotDistribution());
    }

    /// <summary>A 2D B-spline curve.</summary>
    public GeomToStep_MakeBSplineCurveWithKnots(Geom2d_BSplineCurve BS) : this(BS, new StepData_Factors())
    {
    }

    // GeomToStep_MakeBSplineCurveWithKnots_gen.pxx, the body OCCT includes for both curve types (its poles made
    // first, in the same order)
    private void Make(int Deg,
                      NCollection_HArray1<StepGeom_CartesianPoint> Listpoints,
                      bool isClosed,
                      int Nknots,
                      NCollection_Array1<int> M,
                      NCollection_Array1<double> K,
                      GeomAbs_BSplKnotDistribution Distribution)
    {
        StepGeom_BSplineCurveWithKnots BSWK;
        int i, itampon;
        double rtampon;
        StepGeom_BSplineCurveForm Form;
        StepData_Logical Fermeture, Selfinter;
        NCollection_HArray1<int> Mult;
        NCollection_HArray1<double> ListKnots;
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
        if (Distribution == GeomAbs_NonUniform)
        {
            KnotSpec = StepGeom_ktUnspecified;
        }
        else if (Distribution == GeomAbs_Uniform)
        {
            KnotSpec = StepGeom_ktUniformKnots;
        }
        else if (Distribution == GeomAbs_QuasiUniform)
        {
            KnotSpec = StepGeom_ktQuasiUniformKnots;
        }
        else
        {
            KnotSpec = StepGeom_ktPiecewiseBezierKnots;
        }
        BSWK = new StepGeom_BSplineCurveWithKnots();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        BSWK.Init(name, Deg, Listpoints, Form, Fermeture, Selfinter, Mult, ListKnots, KnotSpec);
        theBSplineCurveWithKnots = BSWK;
        done = true;
    }

    /// <summary>The b_spline_curve_with_knots made.</summary>
    public StepGeom_BSplineCurveWithKnots Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeBSplineCurveWithKnots::Value() - no result");
        return theBSplineCurveWithKnots;
    }
}
