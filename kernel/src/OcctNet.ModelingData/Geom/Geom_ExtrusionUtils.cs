// Port of: src/ModelingData/TKG3d/Geom/Geom_ExtrusionUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Evaluation of surfaces of linear extrusion, P(U, V) = BasisCurve(U) + V * Direction, shared by
/// Geom_SurfaceOfLinearExtrusion, GeomAdaptor_Surface and GeomGridEval: direct calculations from pre-computed
/// curve values, and evaluators over a basis curve.
/// </summary>
/// <remarks>
/// OCCT's evaluators are templates over the curve type; OCCT instantiates them only with Adaptor3d_Curve
/// (GeomAdaptor_Surface), so they are ported for that type (rules, section 9).
/// </remarks>
internal static class Geom_ExtrusionUtils
{
    /// <summary>The point of the surface from the curve point C(U).</summary>
    internal static void CalculateD0(in gp_Pnt theCurvePt,
                                     double theV,
                                     in gp_XYZ theDir,
                                     out gp_Pnt theP)
    {
        theP = new gp_Pnt();
        theP.SetXYZ(theCurvePt.XYZ() + theV * theDir);
    }

    /// <summary>The point and first derivatives from C(U) and C'(U).</summary>
    internal static void CalculateD1(in gp_Pnt theCurvePt,
                                     in gp_Vec theCurveD1,
                                     double theV,
                                     in gp_XYZ theDir,
                                     out gp_Pnt theP,
                                     out gp_Vec theD1U,
                                     out gp_Vec theD1V)
    {
        theP = new gp_Pnt();
        theP.SetXYZ(theCurvePt.XYZ() + theV * theDir);
        theD1U = theCurveD1;
        theD1V = new gp_Vec();
        theD1V.SetXYZ(theDir);
    }

    /// <summary>The point, first and second derivatives from C(U), C'(U) and C''(U).</summary>
    internal static void CalculateD2(in gp_Pnt theCurvePt,
                                     in gp_Vec theCurveD1,
                                     in gp_Vec theCurveD2,
                                     double theV,
                                     in gp_XYZ theDir,
                                     out gp_Pnt theP,
                                     out gp_Vec theD1U,
                                     out gp_Vec theD1V,
                                     out gp_Vec theD2U,
                                     out gp_Vec theD2V,
                                     out gp_Vec theD2UV)
    {
        theP = new gp_Pnt();
        theP.SetXYZ(theCurvePt.XYZ() + theV * theDir);
        theD1U = theCurveD1;
        theD1V = new gp_Vec();
        theD1V.SetXYZ(theDir);
        theD2U = theCurveD2;
        theD2V = new gp_Vec(0.0, 0.0, 0.0);
        theD2UV = new gp_Vec(0.0, 0.0, 0.0);
    }

    /// <summary>The point and the derivatives up to the third order from C(U) to C'''(U).</summary>
    internal static void CalculateD3(in gp_Pnt theCurvePt,
                                     in gp_Vec theCurveD1,
                                     in gp_Vec theCurveD2,
                                     in gp_Vec theCurveD3,
                                     double theV,
                                     in gp_XYZ theDir,
                                     out gp_Pnt theP,
                                     out gp_Vec theD1U,
                                     out gp_Vec theD1V,
                                     out gp_Vec theD2U,
                                     out gp_Vec theD2V,
                                     out gp_Vec theD2UV,
                                     out gp_Vec theD3U,
                                     out gp_Vec theD3V,
                                     out gp_Vec theD3UUV,
                                     out gp_Vec theD3UVV)
    {
        theP = new gp_Pnt();
        theP.SetXYZ(theCurvePt.XYZ() + theV * theDir);
        theD1U = theCurveD1;
        theD1V = new gp_Vec();
        theD1V.SetXYZ(theDir);
        theD2U = theCurveD2;
        theD2V = new gp_Vec(0.0, 0.0, 0.0);
        theD2UV = new gp_Vec(0.0, 0.0, 0.0);
        theD3U = theCurveD3;
        theD3V = new gp_Vec(0.0, 0.0, 0.0);
        theD3UUV = new gp_Vec(0.0, 0.0, 0.0);
        theD3UVV = new gp_Vec(0.0, 0.0, 0.0);
    }

    /// <summary>
    /// The derivative of orders (theDerU, theDerV): the curve derivative theCurveDN when theDerV is 0, the
    /// direction for (0, 1), zero otherwise.
    /// </summary>
    internal static gp_Vec CalculateDN(in gp_Vec theCurveDN,
                                       in gp_XYZ theDir,
                                       int theDerU,
                                       int theDerV)
    {
        if (theDerV == 0)
        {
            return theCurveDN;
        }
        else if (theDerU == 0 && theDerV == 1)
        {
            return new gp_Vec(theDir);
        }
        return new gp_Vec(0.0, 0.0, 0.0);
    }

    /// <summary>The point of parameters (theU, theV) on the extrusion of theBasis along theDir.</summary>
    internal static bool D0(double theU,
                            double theV,
                            Adaptor3d_Curve theBasis,
                            in gp_XYZ theDir,
                            out gp_Pnt theP)
    {
        gp_Pnt aCurveD0 = theBasis.EvalD0(theU);
        CalculateD0(aCurveD0, theV, theDir, out theP);
        return true;
    }

    /// <summary>The point and first derivatives.</summary>
    internal static bool D1(double theU,
                            double theV,
                            Adaptor3d_Curve theBasis,
                            in gp_XYZ theDir,
                            out gp_Pnt theP,
                            out gp_Vec theD1U,
                            out gp_Vec theD1V)
    {
        Geom_Curve.ResD1 aCurveD1 = theBasis.EvalD1(theU);
        CalculateD1(aCurveD1.Point, aCurveD1.D1, theV, theDir, out theP, out theD1U, out theD1V);
        return true;
    }

    /// <summary>The point, first and second derivatives.</summary>
    internal static bool D2(double theU,
                            double theV,
                            Adaptor3d_Curve theBasis,
                            in gp_XYZ theDir,
                            out gp_Pnt theP,
                            out gp_Vec theD1U,
                            out gp_Vec theD1V,
                            out gp_Vec theD2U,
                            out gp_Vec theD2V,
                            out gp_Vec theD2UV)
    {
        Geom_Curve.ResD2 aCurveD2 = theBasis.EvalD2(theU);
        CalculateD2(aCurveD2.Point,
                    aCurveD2.D1,
                    aCurveD2.D2,
                    theV,
                    theDir,
                    out theP,
                    out theD1U,
                    out theD1V,
                    out theD2U,
                    out theD2V,
                    out theD2UV);
        return true;
    }

    /// <summary>The point and the derivatives up to the third order.</summary>
    internal static bool D3(double theU,
                            double theV,
                            Adaptor3d_Curve theBasis,
                            in gp_XYZ theDir,
                            out gp_Pnt theP,
                            out gp_Vec theD1U,
                            out gp_Vec theD1V,
                            out gp_Vec theD2U,
                            out gp_Vec theD2V,
                            out gp_Vec theD2UV,
                            out gp_Vec theD3U,
                            out gp_Vec theD3V,
                            out gp_Vec theD3UUV,
                            out gp_Vec theD3UVV)
    {
        Geom_Curve.ResD3 aCurveD3 = theBasis.EvalD3(theU);
        CalculateD3(aCurveD3.Point,
                    aCurveD3.D1,
                    aCurveD3.D2,
                    aCurveD3.D3,
                    theV,
                    theDir,
                    out theP,
                    out theD1U,
                    out theD1V,
                    out theD2U,
                    out theD2V,
                    out theD2UV,
                    out theD3U,
                    out theD3V,
                    out theD3UUV,
                    out theD3UVV);
        return true;
    }

    /// <summary>The derivative of orders (theDerU, theDerV); the curve is evaluated only when theDerV is 0.</summary>
    internal static bool DN(double theU,
                            Adaptor3d_Curve theBasis,
                            in gp_XYZ theDir,
                            int theDerU,
                            int theDerV,
                            out gp_Vec theDN)
    {
        gp_Vec aCurveDN = new gp_Vec();
        if (theDerV == 0)
        {
            aCurveDN = theBasis.EvalDN(theU, theDerU);
        }
        theDN = CalculateDN(aCurveDN, theDir, theDerU, theDerV);
        return true;
    }
}
