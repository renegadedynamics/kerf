// Port of: src/ModelingData/TKG3d/Geom/Geom_RevolutionUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Evaluation of surfaces of revolution, P(U, V) = Rotation(Axis, U) * BasisCurve(V), shared by
/// Geom_SurfaceOfRevolution, GeomAdaptor_Surface and GeomGridEval: direct calculations from pre-computed curve
/// values, and evaluators over a basis curve.
/// </summary>
/// <remarks>
/// OCCT's evaluators are templates over the curve type; OCCT instantiates them only with Adaptor3d_Curve
/// (GeomAdaptor_Surface), so they are ported for that type (rules, section 9).
/// </remarks>
internal static class Geom_RevolutionUtils
{
    /// <summary>The point of the surface from the curve point C(V) and the rotation angle theU.</summary>
    internal static void CalculateD0(in gp_Pnt theCurvePt,
                                     double theU,
                                     in gp_Ax1 theAxis,
                                     out gp_Pnt theP)
    {
        theP = theCurvePt;
        gp_Trsf aRotation = new gp_Trsf();
        aRotation.SetRotation(theAxis, theU);
        theP.Transform(aRotation);
    }

    /// <summary>The point and first derivatives from C(V) and C'(V).</summary>
    internal static void CalculateD1(in gp_Pnt theCurvePt,
                                     in gp_Vec theCurveD1,
                                     double theU,
                                     in gp_Ax1 theAxis,
                                     out gp_Pnt theP,
                                     out gp_Vec theD1U,
                                     out gp_Vec theD1V)
    {
        theP = theCurvePt;

        // Vector from center of rotation to the point on rotated curve
        gp_XYZ aCQ = theCurvePt.XYZ() - theAxis.Location().XYZ();
        theD1U = new gp_Vec(theAxis.Direction().XYZ().Crossed(aCQ));

        // If the point is placed on the axis of revolution then derivatives on U are undefined.
        // Manually set them to zero.
        if (theD1U.SquareMagnitude() < Precision.SquareConfusion())
        {
            theD1U.SetCoord(0.0, 0.0, 0.0);
        }

        gp_Trsf aRotation = new gp_Trsf();
        aRotation.SetRotation(theAxis, theU);
        theP.Transform(aRotation);
        theD1U.Transform(aRotation);
        theD1V = theCurveD1;
        theD1V.Transform(aRotation);
    }

    /// <summary>The point, first and second derivatives from C(V), C'(V) and C''(V).</summary>
    internal static void CalculateD2(in gp_Pnt theCurvePt,
                                     in gp_Vec theCurveD1,
                                     in gp_Vec theCurveD2,
                                     double theU,
                                     in gp_Ax1 theAxis,
                                     out gp_Pnt theP,
                                     out gp_Vec theD1U,
                                     out gp_Vec theD1V,
                                     out gp_Vec theD2U,
                                     out gp_Vec theD2V,
                                     out gp_Vec theD2UV)
    {
        theP = theCurvePt;

        // Vector from center of rotation to the point on rotated curve
        gp_XYZ aCQ = theCurvePt.XYZ() - theAxis.Location().XYZ();
        gp_XYZ aDir = theAxis.Direction().XYZ();
        theD1U = new gp_Vec(aDir.Crossed(aCQ));

        // If the point is placed on the axis of revolution then derivatives on U are undefined.
        // Manually set them to zero.
        if (theD1U.SquareMagnitude() < Precision.SquareConfusion())
        {
            theD1U.SetCoord(0.0, 0.0, 0.0);
        }

        theD2U = new gp_Vec(aDir.Dot(aCQ) * aDir - aCQ);
        theD2UV = new gp_Vec(aDir.Crossed(theCurveD1.XYZ()));

        gp_Trsf aRotation = new gp_Trsf();
        aRotation.SetRotation(theAxis, theU);
        theP.Transform(aRotation);
        theD1U.Transform(aRotation);
        theD1V = theCurveD1;
        theD1V.Transform(aRotation);
        theD2U.Transform(aRotation);
        theD2V = theCurveD2;
        theD2V.Transform(aRotation);
        theD2UV.Transform(aRotation);
    }

    /// <summary>The point and the derivatives up to the third order from C(V) to C'''(V).</summary>
    internal static void CalculateD3(in gp_Pnt theCurvePt,
                                     in gp_Vec theCurveD1,
                                     in gp_Vec theCurveD2,
                                     in gp_Vec theCurveD3,
                                     double theU,
                                     in gp_Ax1 theAxis,
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
        theP = theCurvePt;

        // Vector from center of rotation to the point on rotated curve
        gp_XYZ aCQ = theCurvePt.XYZ() - theAxis.Location().XYZ();
        gp_XYZ aDir = theAxis.Direction().XYZ();
        theD1U = new gp_Vec(aDir.Crossed(aCQ));

        // If the point is placed on the axis of revolution then derivatives on U are undefined.
        // Manually set them to zero.
        if (theD1U.SquareMagnitude() < Precision.SquareConfusion())
        {
            theD1U.SetCoord(0.0, 0.0, 0.0);
        }

        theD2U = new gp_Vec(aDir.Dot(aCQ) * aDir - aCQ);
        theD2UV = new gp_Vec(aDir.Crossed(theCurveD1.XYZ()));
        theD3U = -theD1U;
        theD3UUV = new gp_Vec(aDir.Dot(theCurveD1.XYZ()) * aDir - theCurveD1.XYZ());
        theD3UVV = new gp_Vec(aDir.Crossed(theCurveD2.XYZ()));

        gp_Trsf aRotation = new gp_Trsf();
        aRotation.SetRotation(theAxis, theU);
        theP.Transform(aRotation);
        theD1U.Transform(aRotation);
        theD1V = theCurveD1;
        theD1V.Transform(aRotation);
        theD2U.Transform(aRotation);
        theD2V = theCurveD2;
        theD2V.Transform(aRotation);
        theD2UV.Transform(aRotation);
        theD3U.Transform(aRotation);
        theD3V = theCurveD3;
        theD3V.Transform(aRotation);
        theD3UUV.Transform(aRotation);
        theD3UVV.Transform(aRotation);
    }

    /// <summary>
    /// The derivative of orders (theDerU, theDerV). theCurvePtOrDN is C(V) - axis location when theDerV is 0,
    /// and the curve derivative of order theDerV otherwise.
    /// </summary>
    internal static gp_Vec CalculateDN(in gp_Vec theCurvePtOrDN,
                                       double theU,
                                       in gp_Ax1 theAxis,
                                       int theDerU,
                                       int theDerV)
    {
        // theDerV is part of the interface contract - caller provides different data based on its value
        _ = theDerV;

        gp_Trsf aRotation = new gp_Trsf();
        aRotation.SetRotation(theAxis, theU);

        gp_Vec aResult;
        if (theDerU == 0)
        {
            // Pure V derivative: just rotate the curve derivative
            aResult = theCurvePtOrDN;
        }
        else
        {
            // For theDerV == 0: theCurvePtOrDN is (P - AxisLocation) as a vector
            // For theDerV > 0: theCurvePtOrDN is the curve derivative
            gp_XYZ aDir = theAxis.Direction().XYZ();
            if (theDerU % 4 == 1)
            {
                aResult = new gp_Vec(aDir.Crossed(theCurvePtOrDN.XYZ()));
            }
            else if (theDerU % 4 == 2)
            {
                aResult = new gp_Vec(aDir.Dot(theCurvePtOrDN.XYZ()) * aDir - theCurvePtOrDN.XYZ());
            }
            else if (theDerU % 4 == 3)
            {
                aResult = new gp_Vec(aDir.Crossed(theCurvePtOrDN.XYZ())) * (-1.0);
            }
            else
            {
                aResult = new gp_Vec(theCurvePtOrDN.XYZ() - aDir.Dot(theCurvePtOrDN.XYZ()) * aDir);
            }
        }

        aResult.Transform(aRotation);
        return aResult;
    }

    /// <summary>The point of parameters (theU, theV) on the revolution of theBasis about theAxis.</summary>
    internal static bool D0(double theU,
                            double theV,
                            Adaptor3d_Curve theBasis,
                            in gp_Ax1 theAxis,
                            out gp_Pnt theP)
    {
        gp_Pnt aCurveD0 = theBasis.EvalD0(theV);
        CalculateD0(aCurveD0, theU, theAxis, out theP);
        return true;
    }

    /// <summary>The point and first derivatives.</summary>
    internal static bool D1(double theU,
                            double theV,
                            Adaptor3d_Curve theBasis,
                            in gp_Ax1 theAxis,
                            out gp_Pnt theP,
                            out gp_Vec theD1U,
                            out gp_Vec theD1V)
    {
        Geom_Curve.ResD1 aCurveD1 = theBasis.EvalD1(theV);
        CalculateD1(aCurveD1.Point, aCurveD1.D1, theU, theAxis, out theP, out theD1U, out theD1V);
        return true;
    }

    /// <summary>The point, first and second derivatives.</summary>
    internal static bool D2(double theU,
                            double theV,
                            Adaptor3d_Curve theBasis,
                            in gp_Ax1 theAxis,
                            out gp_Pnt theP,
                            out gp_Vec theD1U,
                            out gp_Vec theD1V,
                            out gp_Vec theD2U,
                            out gp_Vec theD2V,
                            out gp_Vec theD2UV)
    {
        Geom_Curve.ResD2 aCurveD2 = theBasis.EvalD2(theV);
        CalculateD2(aCurveD2.Point,
                    aCurveD2.D1,
                    aCurveD2.D2,
                    theU,
                    theAxis,
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
                            in gp_Ax1 theAxis,
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
        Geom_Curve.ResD3 aCurveD3 = theBasis.EvalD3(theV);
        CalculateD3(aCurveD3.Point,
                    aCurveD3.D1,
                    aCurveD3.D2,
                    aCurveD3.D3,
                    theU,
                    theAxis,
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

    /// <summary>The derivative of orders (theDerU, theDerV).</summary>
    internal static bool DN(double theU,
                            double theV,
                            Adaptor3d_Curve theBasis,
                            in gp_Ax1 theAxis,
                            int theDerU,
                            int theDerV,
                            out gp_Vec theDN)
    {
        gp_Vec aCurvePtOrDN;
        if (theDerU == 0)
        {
            aCurvePtOrDN = theBasis.EvalDN(theV, theDerV);
        }
        else if (theDerV == 0)
        {
            gp_Pnt aP = theBasis.EvalD0(theV);
            aCurvePtOrDN = new gp_Vec(aP.XYZ() - theAxis.Location().XYZ());
        }
        else
        {
            aCurvePtOrDN = theBasis.EvalDN(theV, theDerV);
        }
        theDN = CalculateDN(aCurvePtOrDN, theU, theAxis, theDerU, theDerV);
        return true;
    }
}
