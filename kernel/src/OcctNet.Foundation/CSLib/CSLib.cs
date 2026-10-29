// Port of: src/FoundationClasses/TKMath/CSLib/CSLib.hxx
// Port of: src/FoundationClasses/TKMath/CSLib/CSLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.CSLib_DerivativeStatus;
using static Occt.CSLib_NormalStatus;

namespace Occt;

/// <summary>
/// Computes the normal of a surface from its derivatives, including at singular points, where the normal is
/// found from higher-order derivatives; and the derivatives of the normal itself.
/// </summary>
/// <remarks>
/// <c>theNormal</c> is a <c>ref</c> parameter: as in OCCT, it is left unchanged when the status says that no
/// normal was computed.
/// </remarks>
public static class CSLib
{
    // Angular tolerance for parallel/opposite vector detection (radians).
    private const double THE_PARALLEL_ANGULAR_TOL = 1e-6;

    // Maximum number of iterations for root finding algorithm.
    private const int THE_MAX_ROOT_ITERATIONS = 200;

    // Tolerance for root finding algorithm.
    private const double THE_ROOT_FINDING_TOL = 1e-5;

    /// <summary>
    /// The normal D1U ^ D1V, normalized. <paramref name="theStatus"/> is <c>CSLib_Done</c> when it is computed;
    /// otherwise a derivative is null or D1U and D1V are parallel (the sine of their angle is below
    /// <paramref name="theSinTol"/>).
    /// </summary>
    public static void Normal(in gp_Vec theD1U, in gp_Vec theD1V, double theSinTol,
                              out CSLib_DerivativeStatus theStatus, ref gp_Dir theNormal)
    {
        double aD1UMag  = theD1U.SquareMagnitude();
        double aD1VMag  = theD1V.SquareMagnitude();
        gp_Vec aD1UxD1V = theD1U.Crossed(theD1V);

        if (aD1UMag <= gp.Resolution() && aD1VMag <= gp.Resolution())
        {
            theStatus = CSLib_D1IsNull;
        }
        else if (aD1UMag <= gp.Resolution())
        {
            theStatus = CSLib_D1uIsNull;
        }
        else if (aD1VMag <= gp.Resolution())
        {
            theStatus = CSLib_D1vIsNull;
        }
        else
        {
            double aSin2 = aD1UxD1V.SquareMagnitude() / (aD1UMag * aD1VMag);
            if (aSin2 < theSinTol * theSinTol)
            {
                theStatus = CSLib_D1uIsParallelD1v;
            }
            else
            {
                theNormal = new gp_Dir(aD1UxD1V);
                theStatus = CSLib_Done;
            }
        }
    }

    /// <summary>
    /// The normal at a point where D1U ^ D1V is null, approximated from the second derivatives (Taylor expansion
    /// of the normal). <paramref name="theDone"/> is true when <paramref name="theNormal"/> is computed.
    /// </summary>
    public static void Normal(in gp_Vec theD1U, in gp_Vec theD1V, in gp_Vec theD2U, in gp_Vec theD2V,
                              in gp_Vec theD2UV, double theSinTol, out bool theDone,
                              out CSLib_NormalStatus theStatus, ref gp_Dir theNormal)
    {
        // Compute approximate normal using Taylor expansion:
        // N(u0+du, v0+dv) = N0 + dN/du * du + dN/dv * dv + O(du^2, dv^2)
        // where N = D1U ^ D1V, so dN/du = D2U ^ D1V + D1U ^ D2UV
        // and dN/dv = D2UV ^ D1V + D1U ^ D2V

        gp_Vec aD1Nu = theD2U.Crossed(theD1V);
        aD1Nu.Add(theD1U.Crossed(theD2UV));

        gp_Vec aD1Nv = theD2UV.Crossed(theD1V);
        aD1Nv.Add(theD1U.Crossed(theD2V));

        double aLD1Nu = aD1Nu.SquareMagnitude();
        double aLD1Nv = aD1Nv.SquareMagnitude();

        if (aLD1Nu <= RealEpsilon() && aLD1Nv <= RealEpsilon())
        {
            theStatus = CSLib_D1NIsNull;
            theDone   = false;
        }
        else if (aLD1Nu < RealEpsilon())
        {
            theStatus = CSLib_D1NuIsNull;
            theDone   = true;
            theNormal = new gp_Dir(aD1Nv);
        }
        else if (aLD1Nv < RealEpsilon())
        {
            theStatus = CSLib_D1NvIsNull;
            theDone   = true;
            theNormal = new gp_Dir(aD1Nu);
        }
        else if ((aLD1Nv / aLD1Nu) <= RealEpsilon())
        {
            theStatus = CSLib_D1NvNuRatioIsNull;
            theDone   = false;
        }
        else if ((aLD1Nu / aLD1Nv) <= RealEpsilon())
        {
            theStatus = CSLib_D1NuNvRatioIsNull;
            theDone   = false;
        }
        else
        {
            gp_Vec aD1NCross = aD1Nu.Crossed(aD1Nv);
            double aSin2     = aD1NCross.SquareMagnitude() / (aLD1Nu * aLD1Nv);

            if (aSin2 < theSinTol * theSinTol)
            {
                theStatus = CSLib_D1NuIsParallelD1Nv;
                theDone   = true;
                theNormal = new gp_Dir(aD1Nu);
            }
            else
            {
                theStatus = CSLib_InfinityOfSolutions;
                theDone   = false;
            }
        }
    }

    /// <summary>
    /// The normal from the normalized first derivatives. <paramref name="theStatus"/> is <c>CSLib_Singular</c>
    /// when a derivative or their cross product is shorter than <paramref name="theMagTol"/>.
    /// </summary>
    public static void Normal(in gp_Vec theD1U, in gp_Vec theD1V, double theMagTol,
                              out CSLib_NormalStatus theStatus, ref gp_Dir theNormal)
    {
        double aD1UMag  = theD1U.Magnitude();
        double aD1VMag  = theD1V.Magnitude();
        gp_Vec aD1UxD1V = theD1U.Crossed(theD1V);
        double aNMag    = aD1UxD1V.Magnitude();

        if (aNMag <= theMagTol || aD1UMag <= theMagTol || aD1VMag <= theMagTol)
        {
            theStatus = CSLib_Singular;
        }
        else
        {
            // Normalize tangent vectors first for better numerical stability.
            gp_Dir aD1UDir = new(theD1U);
            gp_Dir aD1VDir = new(theD1V);
            theNormal = aD1UDir.Crossed(aD1VDir);
            theStatus = CSLib_Defined;
        }
    }

    /// <summary>
    /// The normal at a singular point, from the derivatives of N = dS/du ^ dS/dv up to order
    /// <paramref name="theMaxOrder"/>: <paramref name="theDerNUV"/>(i, j) is the derivative of N of order i in U
    /// and j in V. The parameter bounds tell whether the point is inside the domain, on an edge or at a corner.
    /// <paramref name="theOrderU"/> and <paramref name="theOrderV"/> are the orders of the first non-null
    /// derivative.
    /// </summary>
    public static void Normal(int theMaxOrder, NCollection_Array2<gp_Vec> theDerNUV, double theSinTol,
                              double theU, double theV, double theUmin, double theUmax, double theVmin,
                              double theVmax, out CSLib_NormalStatus theStatus, ref gp_Dir theNormal,
                              out int theOrderU, out int theOrderV)
    {
        int    anOrder    = -1;
        int    aFoundUIdx = 0;
        bool   aFound     = false;
        double aNorme;
        gp_Vec aD = new();

        // Find k0 such that all derivatives N = dS/du ^ dS/dv are null till order k0-1.
        while (!aFound && anOrder < theMaxOrder)
        {
            ++anOrder;
            aFoundUIdx = anOrder;
            while (aFoundUIdx >= 0 && !aFound)
            {
                int aVIdx = anOrder - aFoundUIdx;
                aD        = theDerNUV.Value(aFoundUIdx, aVIdx);
                aNorme    = aD.Magnitude();
                aFound    = (aNorme >= theSinTol);
                --aFoundUIdx;
            }
        }

        theOrderU = aFoundUIdx + 1;
        theOrderV = anOrder - theOrderU;

        // Vk0 is the first non-null derivative of N: the reference vector.
        if (!aFound)
        {
            theStatus = CSLib_Singular;
            return;
        }

        if (anOrder == 0)
        {
            theStatus = CSLib_Defined;
            theNormal = new gp_Dir(aD.Normalized());
            return;
        }

        gp_Vec                     aVk0   = theDerNUV.Value(theOrderU, theOrderV);
        NCollection_Array1<double> aRatio = new(0, anOrder);

        // Calculate lambda_i ratios for each derivative at this order.
        int  aRatioIdx = 0;
        bool isDefined = false;
        while (aRatioIdx <= anOrder && !isDefined)
        {
            gp_Vec aDerVec = theDerNUV.Value(aRatioIdx, anOrder - aRatioIdx);
            if (aDerVec.Magnitude() <= theSinTol)
            {
                aRatio[aRatioIdx] = 0.0;
            }
            else if (aDerVec.IsParallel(aVk0, THE_PARALLEL_ANGULAR_TOL))
            {
                double aMagnitudeRatio = aDerVec.Magnitude() / aVk0.Magnitude();
                if (aDerVec.IsOpposite(aVk0, THE_PARALLEL_ANGULAR_TOL))
                {
                    aMagnitudeRatio = -aMagnitudeRatio;
                }
                aRatio[aRatioIdx] = aMagnitudeRatio;
            }
            else
            {
                isDefined = true;
            }
            ++aRatioIdx;
        }

        if (isDefined)
        {
            theStatus = CSLib_Defined;
            theNormal = new gp_Dir(aD.Normalized());
            return;
        }

        // All lambda_i exist - analyze the polynomial sign.
        double aInf = -M_PI;
        double aSup = M_PI;

        // Determine domain based on position (interior, edge, corner).
        bool isFU = (Math.Abs(theU - theUmin) < Precision.PConfusion());
        bool isLU = (Math.Abs(theU - theUmax) < Precision.PConfusion());
        bool isFV = (Math.Abs(theV - theVmin) < Precision.PConfusion());
        bool isLV = (Math.Abs(theV - theVmax) < Precision.PConfusion());

        if (isLU)
        {
            aInf = M_PI / 2.0;
            aSup = 3.0 * M_PI / 2.0;
            if (isLV)
            {
                aInf = M_PI;
            }
            if (isFV)
            {
                aSup = M_PI;
            }
        }
        else if (isFU)
        {
            aSup = M_PI / 2.0;
            aInf = -M_PI / 2.0;
            if (isLV)
            {
                aSup = 0.0;
            }
            if (isFV)
            {
                aInf = 0.0;
            }
        }
        else if (isLV)
        {
            aInf = -M_PI;
            aSup = 0.0;
        }
        else if (isFV)
        {
            aInf = 0.0;
            aSup = M_PI;
        }

        bool   aChangesSign = false;
        double aVprec       = 0.0;
        double aVsuiv       = 0.0;

        // Create polynomial and find its roots.
        CSLib_NormalPolyDef aPoly      = new(anOrder, aRatio);
        math_FunctionRoots  aFindRoots = new(aPoly,
                                             aInf,
                                             aSup,
                                             THE_MAX_ROOT_ITERATIONS,
                                             THE_ROOT_FINDING_TOL,
                                             Precision.Confusion(),
                                             Precision.Confusion());

        if (aFindRoots.IsDone() && aFindRoots.NbSolutions() > 0)
        {
            // Sort roots in ascending order.
            int                        aNbSol = aFindRoots.NbSolutions();
            NCollection_Array1<double> aSol   = new(0, aNbSol + 1);

            for (int aRootIdx = 1; aRootIdx <= aNbSol; ++aRootIdx)
            {
                aSol[aRootIdx] = aFindRoots.Value(aRootIdx);
            }
            aSol.AsSpan(1)[..aNbSol].Sort();

            // Add domain limits.
            aSol[0]          = aInf;
            aSol[aNbSol + 1] = aSup;

            // Check for sign changes between consecutive roots.
            int aFirst = 0;
            for (int aIntervalIdx = 0; aIntervalIdx <= aNbSol; ++aIntervalIdx)
            {
                if (Math.Abs(aSol[aIntervalIdx + 1] - aSol[aIntervalIdx]) > Precision.PConfusion())
                {
                    aPoly.Value((aSol[aIntervalIdx] + aSol[aIntervalIdx + 1]) / 2.0, out aVsuiv);
                    // OCCT: aFirst is set to the interval index, which is 0 for the first interval, so the
                    // "first interval" branch runs again for the next one and the sign change between the first
                    // two intervals is never tested.
                    if (aFirst == 0)
                    {
                        aFirst       = aIntervalIdx;
                        aChangesSign = false;
                        aVprec       = aVsuiv;
                    }
                    else
                    {
                        aChangesSign = aChangesSign || (aVprec * aVsuiv) < 0.0;
                        aVprec       = aVsuiv;
                    }
                }
            }
        }
        else
        {
            // No roots found, polynomial doesn't change sign.
            aChangesSign = false;
            aPoly.Value(aInf, out aVsuiv);
        }

        // Determine status based on polynomial sign.
        if (aChangesSign)
        {
            theStatus = CSLib_InfinityOfSolutions;
        }
        else
        {
            theStatus = CSLib_Defined;
            int aSign = (aVsuiv > 0.0) ? 1 : -1;
            theNormal = new gp_Dir(aSign * aVk0.Normalized());
        }
    }

    /// <summary>
    /// The derivative of order (<paramref name="theNu"/>, <paramref name="theNv"/>) of the non-normalized normal
    /// dS/du ^ dS/dv; <paramref name="theDerSurf"/>(i, j) holds the derivatives of the surface.
    /// </summary>
    public static gp_Vec DNNUV(int theNu, int theNv, NCollection_Array2<gp_Vec> theDerSurf)
    {
        gp_Vec aResult = new(0.0, 0.0, 0.0);

        for (int i = 0; i <= theNu; ++i)
        {
            for (int j = 0; j <= theNv; ++j)
            {
                gp_Vec aVG      = theDerSurf.Value(i + 1, j);
                gp_Vec aVD      = theDerSurf.Value(theNu - i, theNv + 1 - j);
                gp_Vec aCross   = aVG.Crossed(aVD);
                double aBinCoef = PLib.Bin(theNu, i) * PLib.Bin(theNv, j);
                aResult.Add(aBinCoef * aCross);
            }
        }

        return aResult;
    }

    /// <summary>
    /// The derivative of order (<paramref name="theNu"/>, <paramref name="theNv"/>) of dS1/du ^ dS2/dv, for two
    /// surfaces with the derivatives <paramref name="theDerSurf1"/> and <paramref name="theDerSurf2"/>.
    /// </summary>
    public static gp_Vec DNNUV(int theNu, int theNv, NCollection_Array2<gp_Vec> theDerSurf1,
                               NCollection_Array2<gp_Vec> theDerSurf2)
    {
        gp_Vec aResult = new(0.0, 0.0, 0.0);

        for (int i = 0; i <= theNu; ++i)
        {
            for (int j = 0; j <= theNv; ++j)
            {
                gp_Vec aVG      = theDerSurf1.Value(i + 1, j);
                gp_Vec aVD      = theDerSurf2.Value(theNu - i, theNv + 1 - j);
                gp_Vec aCross   = aVG.Crossed(aVD);
                double aBinCoef = PLib.Bin(theNu, i) * PLib.Bin(theNv, j);
                aResult.Add(aBinCoef * aCross);
            }
        }

        return aResult;
    }

    /// <summary>
    /// The derivative of order (<paramref name="theNu"/>, <paramref name="theNv"/>) of the normalized normal,
    /// from the derivatives <paramref name="theDerNUV"/> of the non-normalized normal; at a singular point,
    /// (<paramref name="theIduref"/>, <paramref name="theIdvref"/>) are the orders of its first non-null
    /// derivative.
    /// </summary>
    public static gp_Vec DNNormal(int theNu, int theNv, NCollection_Array2<gp_Vec> theDerNUV,
                                  int theIduref = 0, int theIdvref = 0)
    {
        int aKderiv = theNu + theNv;

        NCollection_Array2<gp_Vec> aDerVecNor = new(0, aKderiv, 0, aKderiv);
        NCollection_Array2<double> aTabScal   = new(0, aKderiv, 0, aKderiv);
        NCollection_Array2<double> aTabNorm   = new(0, aKderiv, 0, aKderiv);

        gp_Vec aDerNor = theDerNUV.Value(theIduref, theIdvref).Normalized();
        aDerVecNor.SetValue(0, 0, aDerNor);

        double aDnorm0 = theDerNUV.Value(theIduref, theIdvref).Dot(aDerVecNor.Value(0, 0));
        aTabNorm.SetValue(0, 0, aDnorm0);
        aTabScal.SetValue(0, 0, 0.0);

        for (int aMderiv = 1; aMderiv <= aKderiv; ++aMderiv)
        {
            for (int aPderiv = 0; aPderiv <= aMderiv; ++aPderiv)
            {
                int aQderiv = aMderiv - aPderiv;
                if (aPderiv > theNu || aQderiv > theNv)
                {
                    continue;
                }

                // Compute n . derivative(p,q) of n
                double aScal = 0.0;
                if (aPderiv > aQderiv)
                {
                    for (int aJderiv = 1; aJderiv <= aQderiv; ++aJderiv)
                    {
                        aScal -= PLib.Bin(aQderiv, aJderiv)
                                 * aDerVecNor.Value(0, aJderiv).Dot(aDerVecNor.Value(aPderiv, aQderiv - aJderiv));
                    }

                    for (int aJderiv = 0; aJderiv < aQderiv; ++aJderiv)
                    {
                        aScal -= PLib.Bin(aQderiv, aJderiv)
                                 * aDerVecNor.Value(aPderiv, aJderiv).Dot(aDerVecNor.Value(0, aQderiv - aJderiv));
                    }

                    for (int aIderiv = 1; aIderiv < aPderiv; ++aIderiv)
                    {
                        for (int aJderiv = 0; aJderiv <= aQderiv; ++aJderiv)
                        {
                            aScal -= PLib.Bin(aPderiv, aIderiv) * PLib.Bin(aQderiv, aJderiv)
                                     * aDerVecNor.Value(aIderiv, aJderiv)
                                         .Dot(aDerVecNor.Value(aPderiv - aIderiv, aQderiv - aJderiv));
                        }
                    }
                }
                else
                {
                    for (int aIderiv = 1; aIderiv <= aPderiv; ++aIderiv)
                    {
                        aScal -= PLib.Bin(aPderiv, aIderiv)
                                 * aDerVecNor.Value(aIderiv, 0).Dot(aDerVecNor.Value(aPderiv - aIderiv, aQderiv));
                    }

                    for (int aIderiv = 0; aIderiv < aPderiv; ++aIderiv)
                    {
                        aScal -= PLib.Bin(aPderiv, aIderiv)
                                 * aDerVecNor.Value(aIderiv, aQderiv).Dot(aDerVecNor.Value(aPderiv - aIderiv, 0));
                    }

                    for (int aIderiv = 0; aIderiv <= aPderiv; ++aIderiv)
                    {
                        for (int aJderiv = 1; aJderiv < aQderiv; ++aJderiv)
                        {
                            aScal -= PLib.Bin(aPderiv, aIderiv) * PLib.Bin(aQderiv, aJderiv)
                                     * aDerVecNor.Value(aIderiv, aJderiv)
                                         .Dot(aDerVecNor.Value(aPderiv - aIderiv, aQderiv - aJderiv));
                        }
                    }
                }
                aTabScal.SetValue(aPderiv, aQderiv, aScal / 2.0);

                // Compute the derivative (n,p) of NUV Length.
                double aDnorm =
                  theDerNUV.Value(aPderiv + theIduref, aQderiv + theIdvref).Dot(aDerVecNor.Value(0, 0));

                for (int aJderiv = 0; aJderiv < aQderiv; ++aJderiv)
                {
                    aDnorm -= PLib.Bin(aQderiv + theIdvref, aJderiv + theIdvref)
                              * aTabNorm.Value(aPderiv, aJderiv) * aTabScal.Value(0, aQderiv - aJderiv);
                }

                for (int aIderiv = 0; aIderiv < aPderiv; ++aIderiv)
                {
                    for (int aJderiv = 0; aJderiv <= aQderiv; ++aJderiv)
                    {
                        aDnorm -= PLib.Bin(aPderiv + theIduref, aIderiv + theIduref)
                                  * PLib.Bin(aQderiv + theIdvref, aJderiv + theIdvref)
                                  * aTabNorm.Value(aIderiv, aJderiv)
                                  * aTabScal.Value(aPderiv - aIderiv, aQderiv - aJderiv);
                    }
                }
                aTabNorm.SetValue(aPderiv, aQderiv, aDnorm);

                // Compute derivative (p,q) of n.
                aDerNor = theDerNUV.Value(aPderiv + theIduref, aQderiv + theIdvref);

                for (int aJderiv = 1; aJderiv <= aQderiv; ++aJderiv)
                {
                    aDerNor.Subtract(PLib.Bin(aPderiv + theIduref, theIduref)
                                     * PLib.Bin(aQderiv + theIdvref, aJderiv + theIdvref)
                                     * aTabNorm.Value(0, aJderiv)
                                     * aDerVecNor.Value(aPderiv, aQderiv - aJderiv));
                }

                for (int aIderiv = 1; aIderiv <= aPderiv; ++aIderiv)
                {
                    for (int aJderiv = 0; aJderiv <= aQderiv; ++aJderiv)
                    {
                        aDerNor.Subtract(PLib.Bin(aPderiv + theIduref, aIderiv + theIduref)
                                         * PLib.Bin(aQderiv + theIdvref, aJderiv + theIdvref)
                                         * aTabNorm.Value(aIderiv, aJderiv)
                                         * aDerVecNor.Value(aPderiv - aIderiv, aQderiv - aJderiv));
                    }
                }

                aDerNor.Divide(PLib.Bin(aPderiv + theIduref, theIduref)
                               * PLib.Bin(aQderiv + theIdvref, theIdvref) * aTabNorm.Value(0, 0));
                aDerVecNor.SetValue(aPderiv, aQderiv, aDerNor);
            }
        }

        return aDerVecNor.Value(theNu, theNv);
    }
}
