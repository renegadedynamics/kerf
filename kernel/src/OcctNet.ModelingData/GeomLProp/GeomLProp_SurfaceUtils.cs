// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_SurfaceUtils.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.CSLib_DerivativeStatus;
using static Occt.LProp_Status;

namespace Occt;

/// <summary>
/// The computations shared by the surface local-property classes (OCCT's namespace <c>LProp_SurfaceUtils</c>, in
/// GeomLProp_SurfaceUtils.hxx): the order of the first significant derivative, the normal, and the principal
/// curvatures and directions from the fundamental forms.
/// </summary>
/// <remarks>
/// OCCT's templates over the surface type are ported per instantiation (rules, section 9): the helpers that don't
/// depend on the surface are here; the access policies, derivative evaluation, bounds and the wrappers that take
/// the properties object are written out in <see cref="GeomLProp_SLProps"/> and <see cref="LProp_SLProps3d"/>.
/// </remarks>
internal static class LProp_SurfaceUtils
{
    /// <summary>
    /// The order (1 or 2) of the first derivative in one direction longer than the tolerance; the status is
    /// LProp_Defined if there is one, LProp_Undefined otherwise.
    /// </summary>
    internal static bool FindSurfTangentOrder(in gp_Vec theD1,
                                              in gp_Vec theD2,
                                              int theCN,
                                              double theTolSq,
                                              out int theOrder,
                                              out LProp_Status theStatus)
    {
        theOrder = 0;

        while (theOrder < 3)
        {
            theOrder++;
            if (theCN >= theOrder)
            {
                // OCCT's array of pointers {&theD1, &theD2}
                if (theOrder <= 2 && (theOrder == 1 ? theD1 : theD2).SquareMagnitude() > theTolSq)
                {
                    theStatus = LProp_Defined;
                    return true;
                }
            }
            else
            {
                theStatus = LProp_Undefined;
                return false;
            }
        }

        theStatus = LProp_Undefined;
        return false;
    }

    /// <summary>The normal by CSLib::Normal; true if it is defined.</summary>
    internal static bool ComputeSurfNormal(in gp_Vec theD1u, in gp_Vec theD1v, double theLinTol, ref gp_Dir theNormal)
    {
        CSLib_DerivativeStatus aStatus;
        CSLib.Normal(theD1u, theD1v, theLinTol, out aStatus, ref theNormal);
        return aStatus == CSLib_Done;
    }

    /// <summary>
    /// The principal curvatures and directions, the mean and the Gaussian curvatures, from the first and second
    /// fundamental forms; false if they can't be computed (the outputs may then be partly written, as in OCCT).
    /// </summary>
    internal static bool ComputeSurfCurvatures(in gp_Vec theD1u,
                                               in gp_Vec theD1v,
                                               in gp_Vec theD2u,
                                               in gp_Vec theD2v,
                                               in gp_Vec theDuv,
                                               in gp_Dir theNormal,
                                               ref double theMinCurv,
                                               ref double theMaxCurv,
                                               ref gp_Dir theDirMin,
                                               ref gp_Dir theDirMax,
                                               ref double theMeanCurv,
                                               ref double theGausCurv)
    {
        gp_Vec aNorm = new(theNormal);

        double anE = theD1u.SquareMagnitude();
        double anF = theD1u.Dot(theD1v);
        double aG = theD1v.SquareMagnitude();

        double aL = aNorm.Dot(theD2u);
        double aM = aNorm.Dot(theDuv);
        double aN = aNorm.Dot(theD2v);

        double anA0 = anE * aM - anF * aL;
        double aB0 = anE * aN - aG * aL;
        double aC0 = anF * aN - aG * aM;

        double aMaxABC = Max(Max(Math.Abs(anA0), Math.Abs(aB0)), Math.Abs(aC0));
        if (aMaxABC < RealEpsilon())
        {
            // Umbilic point
            if (aG < RealEpsilon())
            {
                return false;
            }

            theMinCurv = aN / aG;
            theMaxCurv = theMinCurv;
            theDirMin = new gp_Dir(theD1u);
            theDirMax = new gp_Dir(theD1u.Crossed(aNorm));
            theMeanCurv = theMinCurv;
            theGausCurv = theMinCurv * theMinCurv;
            return true;
        }

        double anA = anA0 / aMaxABC;
        double aB = aB0 / aMaxABC;
        double aC = aC0 / aMaxABC;

        double aCurv1, aCurv2;
        gp_Vec aVectCurv1, aVectCurv2;

        if (Math.Abs(anA) > RealEpsilon())
        {
            var aRoot = new math_DirectPolynomialRoots(anA, aB, aC);
            if (aRoot.NbSolutions() != 2)
            {
                return false;
            }

            double aRoot1 = aRoot.Value(1);
            double aRoot2 = aRoot.Value(2);
            aCurv1 = ((aL * aRoot1 + 2.0 * aM) * aRoot1 + aN) / ((anE * aRoot1 + 2.0 * anF) * aRoot1 + aG);
            aCurv2 = ((aL * aRoot2 + 2.0 * aM) * aRoot2 + aN) / ((anE * aRoot2 + 2.0 * anF) * aRoot2 + aG);
            aVectCurv1 = aRoot1 * theD1u + theD1v;
            aVectCurv2 = aRoot2 * theD1u + theD1v;
        }
        else if (Math.Abs(aC) > RealEpsilon())
        {
            var aRoot = new math_DirectPolynomialRoots(aC, aB, anA);
            if (aRoot.NbSolutions() != 2)
            {
                return false;
            }

            double aRoot1 = aRoot.Value(1);
            double aRoot2 = aRoot.Value(2);
            aCurv1 = ((aN * aRoot1 + 2.0 * aM) * aRoot1 + aL) / ((aG * aRoot1 + 2.0 * anF) * aRoot1 + anE);
            aCurv2 = ((aN * aRoot2 + 2.0 * aM) * aRoot2 + aL) / ((aG * aRoot2 + 2.0 * anF) * aRoot2 + anE);
            aVectCurv1 = theD1u + aRoot1 * theD1v;
            aVectCurv2 = theD1u + aRoot2 * theD1v;
        }
        else
        {
            aCurv1 = aL / anE;
            aCurv2 = aN / aG;
            aVectCurv1 = theD1u;
            aVectCurv2 = theD1v;
        }

        if (aCurv1 < aCurv2)
        {
            theMinCurv = aCurv1;
            theMaxCurv = aCurv2;
            theDirMin = new gp_Dir(aVectCurv1);
            theDirMax = new gp_Dir(aVectCurv2);
        }
        else
        {
            theMinCurv = aCurv2;
            theMaxCurv = aCurv1;
            theDirMin = new gp_Dir(aVectCurv2);
            theDirMax = new gp_Dir(aVectCurv1);
        }

        double anEG_FF = (anE * aG) - (anF * anF);
        theMeanCurv = ((aN * anE) - (2.0 * aM * anF) + (aL * aG)) / (2.0 * anEG_FF);
        theGausCurv = ((aL * aN) - (aM * aM)) / anEG_FF;
        return true;
    }

    /// <summary>The normal status, computing the normal if it is undecided.</summary>
    internal static bool IsNormalDefined(in gp_Vec theD1u,
                                         in gp_Vec theD1v,
                                         double theLinTol,
                                         ref gp_Dir theNormal,
                                         ref LProp_Status theNormStatus)
    {
        if (theNormStatus == LProp_Undefined)
        {
            return false;
        }

        if (theNormStatus >= LProp_Defined)
        {
            return true;
        }

        if (ComputeSurfNormal(theD1u, theD1v, theLinTol, ref theNormal))
        {
            theNormStatus = LProp_Computed;
            return true;
        }

        theNormStatus = LProp_Undefined;
        return false;
    }
}
