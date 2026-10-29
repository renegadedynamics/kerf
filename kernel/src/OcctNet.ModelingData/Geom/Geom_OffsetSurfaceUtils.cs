// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetSurfaceUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.CSLib_NormalStatus;

namespace Occt;

/// <summary>
/// The point and derivatives of an offset surface from those of its basis surface:
/// P(u, v) = p(u, v) + Offset * N / ||N||, where N = dp/du ^ dp/dv is the normal. Handles the non-singular case
/// (the normal from the cross product) and the singular one (higher derivatives of the normal, and the
/// osculating surfaces of <see cref="Geom_OsculatingSurface"/>). An internal C++ namespace in OCCT, used by
/// Geom_OffsetSurface and GeomAdaptor_Surface.
/// </summary>
/// <remarks>
/// OCCT's function templates over the basis surface and the osculating-surface query are generic methods over
/// the basis (decision 212): <see cref="GeomSurfaceBasis"/> wraps a <see cref="Geom_Surface"/>
/// (Geom_OffsetSurface's instantiation) and <see cref="AdaptorBasis"/> a <see cref="GeomAdaptor_Surface"/>
/// (GeomAdaptor_Surface's); the bodies are written once. Both of OCCT's query types end in
/// Geom_OsculatingSurface (Geom_OffsetSurface's query returns false without one), so the query is a nullable
/// <see cref="Geom_OsculatingSurface"/>. OCCT's stack buffers (NCollection_LocalArray under an
/// NCollection_Array2) are NCollection_Array2 allocations here (decision 78).
/// </remarks>
internal static class Geom_OffsetSurfaceUtils
{
    /// <summary>The evaluation services of a basis surface that the functions use.</summary>
    internal interface IBasisSurface
    {
        void Bounds(out double theU1, out double theU2, out double theV1, out double theV2);

        bool IsUPeriodic();

        bool IsVPeriodic();

        Geom_Surface.ResD1 EvalD1(double theU, double theV);

        Geom_Surface.ResD2 EvalD2(double theU, double theV);

        Geom_Surface.ResD3 EvalD3(double theU, double theV);

        gp_Vec EvalDN(double theU, double theV, int theNu, int theNv);
    }

    /// <summary>A Geom surface as basis (Geom_OffsetSurface).</summary>
    internal readonly struct GeomSurfaceBasis : IBasisSurface
    {
        private readonly Geom_Surface mySurf;

        public GeomSurfaceBasis(Geom_Surface theSurf) => mySurf = theSurf;

        public void Bounds(out double theU1, out double theU2, out double theV1, out double theV2) =>
            mySurf.Bounds(out theU1, out theU2, out theV1, out theV2);

        public bool IsUPeriodic() => mySurf.IsUPeriodic();

        public bool IsVPeriodic() => mySurf.IsVPeriodic();

        public Geom_Surface.ResD1 EvalD1(double theU, double theV) => mySurf.EvalD1(theU, theV);

        public Geom_Surface.ResD2 EvalD2(double theU, double theV) => mySurf.EvalD2(theU, theV);

        public Geom_Surface.ResD3 EvalD3(double theU, double theV) => mySurf.EvalD3(theU, theV);

        public gp_Vec EvalDN(double theU, double theV, int theNu, int theNv) => mySurf.EvalDN(theU, theV, theNu, theNv);
    }

    /// <summary>A surface adaptor as basis (GeomAdaptor_Surface, bounded by the adaptor's parameter range).</summary>
    internal readonly struct AdaptorBasis : IBasisSurface
    {
        private readonly GeomAdaptor_Surface mySurf;

        public AdaptorBasis(GeomAdaptor_Surface theSurf) => mySurf = theSurf;

        public void Bounds(out double theU1, out double theU2, out double theV1, out double theV2) =>
            mySurf.Bounds(out theU1, out theU2, out theV1, out theV2);

        public bool IsUPeriodic() => mySurf.IsUPeriodic();

        public bool IsVPeriodic() => mySurf.IsVPeriodic();

        public Geom_Surface.ResD1 EvalD1(double theU, double theV) => mySurf.EvalD1(theU, theV);

        public Geom_Surface.ResD2 EvalD2(double theU, double theV) => mySurf.EvalD2(theU, theV);

        public Geom_Surface.ResD3 EvalD3(double theU, double theV) => mySurf.EvalD3(theU, theV);

        public gp_Vec EvalDN(double theU, double theV, int theNu, int theNv) => mySurf.EvalDN(theU, theV, theNu, theNv);
    }

    /// <summary>Default tolerance for normal magnitude check.</summary>
    internal const double THE_D1_MAGNITUDE_TOL = 1.0e-9;

    /// <summary>Osculating surface query results.</summary>
    private struct OsculatingInfo
    {
        /// <summary>True if osculating along U direction.</summary>
        public bool AlongU;

        /// <summary>True if osculating along V direction.</summary>
        public bool AlongV;

        /// <summary>True if normal direction should be reversed.</summary>
        public bool IsOpposite;

        /// <summary>The sign factor for offset calculation.</summary>
        public readonly double Sign() => ((AlongU || AlongV) && IsOpposite) ? -1.0 : 1.0;

        /// <summary>True if osculating surface is available.</summary>
        public readonly bool HasOsculating() => AlongU || AlongV;
    }

    /// <summary>True if any coordinate is infinite.</summary>
    internal static bool IsInfiniteCoord(in gp_Vec theVec)
    {
        return Precision.IsInfinite(theVec.X()) || Precision.IsInfinite(theVec.Y()) || Precision.IsInfinite(theVec.Z());
    }

    /// <summary>True if the normal magnitude is below the tolerance (singular case).</summary>
    internal static bool IsSingular(in gp_Vec theD1U, in gp_Vec theD1V, double theTol = THE_D1_MAGNITUDE_TOL)
    {
        // Normalize derivatives before normal calculation for stability
        gp_Vec aD1U = theD1U;
        gp_Vec aD1V = theD1V;
        double aD1UNorm2 = aD1U.SquareMagnitude();
        double aD1VNorm2 = aD1V.SquareMagnitude();
        if (aD1UNorm2 > 1.0)
        {
            aD1U /= Math.Sqrt(aD1UNorm2);
        }
        if (aD1VNorm2 > 1.0)
        {
            aD1V /= Math.Sqrt(aD1VNorm2);
        }

        gp_Vec aNorm = aD1U.Crossed(aD1V);
        return aNorm.SquareMagnitude() <= theTol * theTol;
    }

    /// <summary>The normalized normal in the non-singular case; false if singular.</summary>
    internal static bool ComputeNormal(in gp_Vec theD1U, in gp_Vec theD1V, out gp_Vec theNormal,
                                       double theTol = THE_D1_MAGNITUDE_TOL)
    {
        // Normalize derivatives before normal calculation for stability
        gp_Vec aD1U = theD1U;
        gp_Vec aD1V = theD1V;
        double aD1UNorm2 = aD1U.SquareMagnitude();
        double aD1VNorm2 = aD1V.SquareMagnitude();
        if (aD1UNorm2 > 1.0)
        {
            aD1U /= Math.Sqrt(aD1UNorm2);
        }
        if (aD1VNorm2 > 1.0)
        {
            aD1V /= Math.Sqrt(aD1VNorm2);
        }

        theNormal = aD1U.Crossed(aD1V);
        if (theNormal.SquareMagnitude() <= theTol * theTol)
        {
            return false;
        }
        theNormal.Normalize();
        return true;
    }

    /// <summary>dN/du for a non-singular offset surface.</summary>
    internal static gp_Vec ComputeDNormalU(in gp_Vec theD1U, in gp_Vec theD1V, in gp_Vec theD2U, in gp_Vec theD2UV,
                                           in gp_Vec theNormal)
    {
        double aScale = (theD1U ^ theD1V).Dot(theNormal);

        gp_Vec aN1U = new gp_Vec();
        aN1U.SetX(theD2U.Y() * theD1V.Z() + theD1U.Y() * theD2UV.Z() - theD2U.Z() * theD1V.Y()
                  - theD1U.Z() * theD2UV.Y());
        aN1U.SetY(-(theD2U.X() * theD1V.Z() + theD1U.X() * theD2UV.Z() - theD2U.Z() * theD1V.X()
                    - theD1U.Z() * theD2UV.X()));
        aN1U.SetZ(theD2U.X() * theD1V.Y() + theD1U.X() * theD2UV.Y() - theD2U.Y() * theD1V.X()
                  - theD1U.Y() * theD2UV.X());
        double aScaleU = aN1U.Dot(theNormal);
        aN1U.Subtract(aScaleU * theNormal);
        aN1U /= aScale;

        return aN1U;
    }

    /// <summary>dN/dv for a non-singular offset surface.</summary>
    internal static gp_Vec ComputeDNormalV(in gp_Vec theD1U, in gp_Vec theD1V, in gp_Vec theD2V, in gp_Vec theD2UV,
                                           in gp_Vec theNormal)
    {
        double aScale = (theD1U ^ theD1V).Dot(theNormal);

        gp_Vec aN1V = new gp_Vec();
        aN1V.SetX(theD2UV.Y() * theD1V.Z() + theD2V.Z() * theD1U.Y() - theD2UV.Z() * theD1V.Y()
                  - theD2V.Y() * theD1U.Z());
        aN1V.SetY(-(theD2UV.X() * theD1V.Z() + theD2V.Z() * theD1U.X() - theD2UV.Z() * theD1V.X()
                    - theD2V.X() * theD1U.Z()));
        aN1V.SetZ(theD2UV.X() * theD1V.Y() + theD2V.Y() * theD1U.X() - theD2UV.Y() * theD1V.X()
                  - theD2V.X() * theD1U.Y());
        double aScaleV = aN1V.Dot(theNormal);
        aN1V.Subtract(aScaleV * theNormal);
        aN1V /= aScale;

        return aN1V;
    }

    /// <summary>
    /// The offset point in the non-singular case: theValue is the basis point on input. False if singular.
    /// </summary>
    internal static bool CalculateD0(ref gp_Pnt theValue, in gp_Vec theD1U, in gp_Vec theD1V, double theOffset,
                                     double theSign = 1.0)
    {
        if (!ComputeNormal(theD1U, theD1V, out gp_Vec aNorm))
        {
            return false;
        }
        theValue.SetXYZ(theValue.XYZ() + theOffset * theSign * aNorm.XYZ());
        return true;
    }

    /// <summary>
    /// The offset point and first derivatives in the non-singular case: the inputs are the basis surface's.
    /// False if singular.
    /// </summary>
    internal static bool CalculateD1(ref gp_Pnt theValue, ref gp_Vec theD1U, ref gp_Vec theD1V, in gp_Vec theD2U,
                                     in gp_Vec theD2V, in gp_Vec theD2UV, double theOffset, double theSign = 1.0)
    {
        if (!ComputeNormal(theD1U, theD1V, out gp_Vec aNorm))
        {
            return false;
        }

        // Compute offset point
        theValue.SetXYZ(theValue.XYZ() + theOffset * theSign * aNorm.XYZ());

        // Compute normal derivatives
        gp_Vec aN1U = ComputeDNormalU(theD1U, theD1V, theD2U, theD2UV, aNorm);
        gp_Vec aN1V = ComputeDNormalV(theD1U, theD1V, theD2V, theD2UV, aNorm);

        theD1U += theOffset * theSign * aN1U;
        theD1V += theOffset * theSign * aN1V;

        return true;
    }

    /// <summary>
    /// The derivatives of the normal at singular points (theDerNUV), from the derivatives of the basis surface
    /// (theDerSurf, completed here) and, along a degenerate iso-curve, of the osculating surface.
    /// </summary>
    internal static bool ComputeDerivatives<TBasis>(int theMaxOrder,
                                                    int theMinOrder,
                                                    double theU,
                                                    double theV,
                                                    in TBasis theBasisSurf,
                                                    int theNU,
                                                    int theNV,
                                                    bool theAlongU,
                                                    bool theAlongV,
                                                    Geom_BSplineSurface? theOscSurf,
                                                    NCollection_Array2<gp_Vec> theDerNUV,
                                                    NCollection_Array2<gp_Vec> theDerSurf)
        where TBasis : struct, IBasisSurface
    {
        if (theAlongU || theAlongV)
        {
            theMaxOrder = 0;
            NCollection_Array2<gp_Vec> DerSurfL = new(0, theMaxOrder + theNU + 1, 0, theMaxOrder + theNV + 1);
            switch (theMinOrder)
            {
                case 1:
                {
                    Geom_Surface.ResD1 aD1 = theOscSurf!.EvalD1(theU, theV);
                    DerSurfL.SetValue(1, 0, aD1.D1U);
                    DerSurfL.SetValue(0, 1, aD1.D1V);
                    break;
                }
                case 2:
                {
                    Geom_Surface.ResD2 aD2 = theOscSurf!.EvalD2(theU, theV);
                    DerSurfL.SetValue(1, 0, aD2.D1U);
                    DerSurfL.SetValue(0, 1, aD2.D1V);
                    DerSurfL.SetValue(1, 1, aD2.D2UV);
                    DerSurfL.SetValue(2, 0, aD2.D2U);
                    DerSurfL.SetValue(0, 2, aD2.D2V);
                    break;
                }
                case 3:
                {
                    Geom_Surface.ResD3 aD3 = theOscSurf!.EvalD3(theU, theV);
                    DerSurfL.SetValue(1, 0, aD3.D1U);
                    DerSurfL.SetValue(0, 1, aD3.D1V);
                    DerSurfL.SetValue(1, 1, aD3.D2UV);
                    DerSurfL.SetValue(2, 0, aD3.D2U);
                    DerSurfL.SetValue(0, 2, aD3.D2V);
                    DerSurfL.SetValue(3, 0, aD3.D3U);
                    DerSurfL.SetValue(2, 1, aD3.D3UUV);
                    DerSurfL.SetValue(1, 2, aD3.D3UVV);
                    DerSurfL.SetValue(0, 3, aD3.D3V);
                    break;
                }
                default:
                    break;
            }

            if (theNU <= theNV)
            {
                for (int i = 0; i <= theMaxOrder + 1 + theNU; i++)
                {
                    for (int j = i; j <= theMaxOrder + theNV + 1; j++)
                    {
                        if (i + j > theMinOrder)
                        {
                            gp_Vec aOscDN = theOscSurf!.EvalDN(theU, theV, i, j);
                            gp_Vec aBasisDN = theBasisSurf.EvalDN(theU, theV, i, j);
                            DerSurfL.SetValue(i, j, aOscDN);
                            theDerSurf.SetValue(i, j, aBasisDN);
                            if (i != j && j <= theNU + 1)
                            {
                                gp_Vec aBasisDNji = theBasisSurf.EvalDN(theU, theV, j, i);
                                gp_Vec aOscDNji = theOscSurf.EvalDN(theU, theV, j, i);
                                theDerSurf.SetValue(j, i, aBasisDNji);
                                DerSurfL.SetValue(j, i, aOscDNji);
                            }
                        }
                    }
                }
            }
            else
            {
                for (int j = 0; j <= theMaxOrder + 1 + theNV; j++)
                {
                    for (int i = j; i <= theMaxOrder + theNU + 1; i++)
                    {
                        if (i + j > theMinOrder)
                        {
                            gp_Vec aOscDN = theOscSurf!.EvalDN(theU, theV, i, j);
                            gp_Vec aBasisDN = theBasisSurf.EvalDN(theU, theV, i, j);
                            DerSurfL.SetValue(i, j, aOscDN);
                            theDerSurf.SetValue(i, j, aBasisDN);
                            if (i != j && i <= theNV + 1)
                            {
                                gp_Vec aBasisDNji = theBasisSurf.EvalDN(theU, theV, j, i);
                                gp_Vec aOscDNji = theOscSurf.EvalDN(theU, theV, j, i);
                                theDerSurf.SetValue(j, i, aBasisDNji);
                                DerSurfL.SetValue(j, i, aOscDNji);
                            }
                        }
                    }
                }
            }
            for (int i = 0; i <= theMaxOrder + theNU; i++)
            {
                for (int j = 0; j <= theMaxOrder + theNV; j++)
                {
                    if (theAlongU)
                    {
                        theDerNUV.SetValue(i, j, CSLib.DNNUV(i, j, DerSurfL, theDerSurf));
                    }
                    if (theAlongV)
                    {
                        theDerNUV.SetValue(i, j, CSLib.DNNUV(i, j, theDerSurf, DerSurfL));
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i <= theMaxOrder + theNU + 1; i++)
            {
                for (int j = i; j <= theMaxOrder + theNV + 1; j++)
                {
                    if (i + j > theMinOrder)
                    {
                        gp_Vec aDN = theBasisSurf.EvalDN(theU, theV, i, j);
                        theDerSurf.SetValue(i, j, aDN);
                        if (i != j && j <= theDerSurf.UpperRow() && i <= theDerSurf.UpperCol())
                        {
                            gp_Vec aDNji = theBasisSurf.EvalDN(theU, theV, j, i);
                            theDerSurf.SetValue(j, i, aDNji);
                        }
                    }
                }
            }
            for (int i = 0; i <= theMaxOrder + theNU; i++)
            {
                for (int j = 0; j <= theMaxOrder + theNV; j++)
                {
                    theDerNUV.SetValue(i, j, CSLib.DNNUV(i, j, theDerSurf));
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Replaces a zero derivative (CSLib_InfinityOfSolutions: exactly one of theDU, theDV is zero) by the one
    /// evaluated a small step away. True if replaced.
    /// </summary>
    internal static bool ReplaceDerivative<TBasis>(double theU,
                                                   double theV,
                                                   double theUMin,
                                                   double theUMax,
                                                   double theVMin,
                                                   double theVMax,
                                                   ref gp_Vec theDU,
                                                   ref gp_Vec theDV,
                                                   double theSquareTol,
                                                   in TBasis theBasisSurf)
        where TBasis : struct, IBasisSurface
    {
        bool isReplaceDU = theDU.SquareMagnitude() < theSquareTol;
        bool isReplaceDV = theDV.SquareMagnitude() < theSquareTol;
        bool isReplaced = false;

        // Only handle case where exactly one derivative is zero
        if (isReplaceDU != isReplaceDV)
        {
            // Calculate step along non-zero derivative
            double aStep;
            if (isReplaceDV)
            {
                aStep = Precision.Confusion() * theDU.Magnitude();
                if (aStep > theUMax - theUMin)
                {
                    aStep = (theUMax - theUMin) / 100.0;
                }
            }
            else
            {
                aStep = Precision.Confusion() * theDV.Magnitude();
                if (aStep > theVMax - theVMin)
                {
                    aStep = (theVMax - theVMin) / 100.0;
                }
            }

            // Step away from current parametric coordinates and calculate derivatives once again.
            // Replace zero derivative by the obtained.
            for (double aStepSign = -1.0; aStepSign <= 1.0 && !isReplaced; aStepSign += 2.0)
            {
                double aU = theU;
                double aV = theV;

                if (isReplaceDV)
                {
                    aU = theU + aStepSign * aStep;
                    if (aU < theUMin || aU > theUMax)
                    {
                        continue;
                    }
                }
                else
                {
                    aV = theV + aStepSign * aStep;
                    if (aV < theVMin || aV > theVMax)
                    {
                        continue;
                    }
                }

                Geom_Surface.ResD1 aD1Result = theBasisSurf.EvalD1(aU, aV);

                if (isReplaceDU && aD1Result.D1U.SquareMagnitude() > theSquareTol)
                {
                    theDU = aD1Result.D1U;
                    isReplaced = true;
                }
                if (isReplaceDV && aD1Result.D1V.SquareMagnitude() > theSquareTol)
                {
                    theDV = aD1Result.D1V;
                    isReplaced = true;
                }
            }
        }
        return isReplaced;
    }

    /// <summary>
    /// Shifts the evaluation point towards the center of the parametric space (each time doubling its distance
    /// from the start point), for singular points near the bounds. False when the center is reached.
    /// </summary>
    internal static bool ShiftPoint(double theUStart,
                                    double theVStart,
                                    ref double theU,
                                    ref double theV,
                                    double theUMin,
                                    double theUMax,
                                    double theVMin,
                                    double theVMax,
                                    bool theIsUPeriodic,
                                    bool theIsVPeriodic,
                                    in gp_Vec theD1U,
                                    in gp_Vec theD1V)
    {
        // Check if either U or V is singular (normally one of them is)
        bool isUSingular = (theD1U.SquareMagnitude() < THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL);
        bool isVSingular = (theD1V.SquareMagnitude() < THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL);

        // Compute vector to shift from start point to center of the surface;
        // if surface is periodic or singular in some direction, take shift in that direction zero
        double aDirU =
            (theIsUPeriodic || (isUSingular && !isVSingular) ? 0.0 : 0.5 * (theUMin + theUMax) - theUStart);
        double aDirV =
            (theIsVPeriodic || (isVSingular && !isUSingular) ? 0.0 : 0.5 * (theVMin + theVMax) - theVStart);
        double aDist = Math.Sqrt(aDirU * aDirU + aDirV * aDirV);

        // Shift current point from its current position towards center, by value of twice
        // current distance from it to start (but not less than Precision::PConfusion());
        // fail if center is overpassed.
        double aDU = theU - theUStart;
        double aDV = theV - theVStart;
        double aStep = Math.Max(2.0 * Math.Sqrt(aDU * aDU + aDV * aDV), Precision.PConfusion());
        if (aStep >= aDist)
        {
            return false;
        }

        aStep /= aDist;
        theU += aDirU * aStep;
        theV += aDirV * aStep;

        return true;
    }

    // Queries the osculating surfaces at (U, V) as OCCT's OscSurfQueryType does.
    private static void queryOsculating(Geom_OsculatingSurface? theOscQuery, double theU, double theV,
                                        ref OsculatingInfo theInfo, ref Geom_BSplineSurface? theOscSurf)
    {
        if (theOscQuery is not null)
        {
            theInfo.AlongU = theOscQuery.UOsculatingSurface(theU, theV, ref theInfo.IsOpposite, ref theOscSurf);
            theInfo.AlongV = theOscQuery.VOsculatingSurface(theU, theV, ref theInfo.IsOpposite, ref theOscSurf);
        }
    }

    /// <summary>
    /// The offset point from the basis point (theValue on input) and first derivatives at (theU, theV); at singular
    /// points, retries closer to the center of the parameter space. False if it fails.
    /// </summary>
    internal static bool EvaluateD0<TBasis>(double theU,
                                            double theV,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            ref gp_Pnt theValue,
                                            in gp_Vec theD1U,
                                            in gp_Vec theD1V)
        where TBasis : struct, IBasisSurface
    {
        double aUStart = theU;
        double aVStart = theV;
        theBasisSurf.Bounds(out double aUMin, out double aUMax, out double aVMin, out double aVMax);
        bool isUPer = theBasisSurf.IsUPeriodic();
        bool isVPer = theBasisSurf.IsVPeriodic();

        // Use pre-computed D1 for first iteration
        gp_Vec aD1U = theD1U;
        gp_Vec aD1V = theD1V;
        bool isFirstIteration = true;

        for (;;)
        {
            // For subsequent iterations, recompute D1 at shifted point
            if (!isFirstIteration)
            {
                Geom_Surface.ResD1 aD1Result = theBasisSurf.EvalD1(theU, theV);
                theValue = aD1Result.Point;
                aD1U = aD1Result.D1U;
                aD1V = aD1Result.D1V;
            }
            isFirstIteration = false;

            if (IsInfiniteCoord(aD1U) || IsInfiniteCoord(aD1V))
            {
                return false;
            }

            // Try non-singular case first
            if (CalculateD0(ref theValue, aD1U, aD1V, theOffset))
            {
                return true;
            }

            // Singular case - query osculating surface and use higher order derivatives
            const int aMaxOrder = 3;

            OsculatingInfo aOscInfo = default;
            Geom_BSplineSurface? aOscSurf = null;
            queryOsculating(theOscQuery, theU, theV, ref aOscInfo, ref aOscSurf);

            NCollection_Array2<gp_Vec> aDerNUV = new(0, aMaxOrder, 0, aMaxOrder);
            NCollection_Array2<gp_Vec> aDerSurf = new(0, aMaxOrder + 1, 0, aMaxOrder + 1);

            aDerSurf.SetValue(1, 0, aD1U);
            aDerSurf.SetValue(0, 1, aD1V);

            // Use ComputeDerivatives which handles osculating surface properly
            if (aOscInfo.HasOsculating() && aOscSurf is not null)
            {
                if (!ComputeDerivatives(aMaxOrder, 1, theU, theV, theBasisSurf, 0, 0, aOscInfo.AlongU,
                                        aOscInfo.AlongV, aOscSurf, aDerNUV, aDerSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(aMaxOrder, 1, theU, theV, theBasisSurf, 0, 0, false, false, null, aDerNUV,
                                        aDerSurf))
                {
                    return false;
                }
            }

            gp_Dir aNormal = new gp_Dir();
            CSLib.Normal(aMaxOrder,
                         aDerNUV,
                         THE_D1_MAGNITUDE_TOL,
                         theU,
                         theV,
                         aUMin,
                         aUMax,
                         aVMin,
                         aVMax,
                         out CSLib_NormalStatus aNStatus,
                         ref aNormal,
                         out _,
                         out _);

            // Handle CSLib_InfinityOfSolutions by replacing zero derivative
            if (aNStatus == CSLib_InfinityOfSolutions)
            {
                gp_Vec aNewDU = aD1U;
                gp_Vec aNewDV = aD1V;
                if (ReplaceDerivative(theU,
                                      theV,
                                      aUMin,
                                      aUMax,
                                      aVMin,
                                      aVMax,
                                      ref aNewDU,
                                      ref aNewDV,
                                      THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL,
                                      theBasisSurf))
                {
                    CSLib.Normal(aNewDU, aNewDV, THE_D1_MAGNITUDE_TOL, out aNStatus, ref aNormal);
                }
            }

            if (aNStatus == CSLib_Defined)
            {
                theValue.SetXYZ(theValue.XYZ() + theOffset * aOscInfo.Sign() * aNormal.XYZ());
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(aUStart, aVStart, ref theU, ref theV, aUMin, aUMax, aVMin, aVMax, isUPer, isVPer, aD1U,
                            aD1V))
            {
                return false;
            }
        }
    }

    /// <summary>The offset point at (theU, theV); false if it fails at a singular point.</summary>
    internal static bool EvaluateD0<TBasis>(double theU,
                                            double theV,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            out gp_Pnt theValue)
        where TBasis : struct, IBasisSurface
    {
        Geom_Surface.ResD1 aBasisD1 = theBasisSurf.EvalD1(theU, theV);
        theValue = aBasisD1.Point;
        return EvaluateD0(theU, theV, theBasisSurf, theOffset, theOscQuery, ref theValue, aBasisD1.D1U, aBasisD1.D1V);
    }

    /// <summary>
    /// The offset point and first derivatives from the basis point and first derivatives (on input) and second
    /// derivatives; at singular points, retries closer to the center. False if it fails.
    /// </summary>
    internal static bool EvaluateD1<TBasis>(double theU,
                                            double theV,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            ref gp_Pnt theValue,
                                            ref gp_Vec theD1U,
                                            ref gp_Vec theD1V,
                                            in gp_Vec theD2U,
                                            in gp_Vec theD2V,
                                            in gp_Vec theD2UV)
        where TBasis : struct, IBasisSurface
    {
        double aUStart = theU;
        double aVStart = theV;
        theBasisSurf.Bounds(out double aUMin, out double aUMax, out double aVMin, out double aVMax);
        bool isUPer = theBasisSurf.IsUPeriodic();
        bool isVPer = theBasisSurf.IsVPeriodic();

        // Use pre-computed D2 for first iteration
        gp_Vec aD2U = theD2U;
        gp_Vec aD2V = theD2V;
        gp_Vec aD2UV = theD2UV;
        bool isFirstIteration = true;

        for (;;)
        {
            // For subsequent iterations, recompute D2 at shifted point
            if (!isFirstIteration)
            {
                Geom_Surface.ResD2 aD2Result = theBasisSurf.EvalD2(theU, theV);
                theValue = aD2Result.Point;
                theD1U = aD2Result.D1U;
                theD1V = aD2Result.D1V;
                aD2U = aD2Result.D2U;
                aD2V = aD2Result.D2V;
                aD2UV = aD2Result.D2UV;
            }
            isFirstIteration = false;

            if (IsInfiniteCoord(theD1U) || IsInfiniteCoord(theD1V))
            {
                return false;
            }

            // Check if singular by normalizing derivatives and computing cross product
            gp_Vec aD1U = theD1U;
            gp_Vec aD1V = theD1V;
            double aD1UNorm2 = aD1U.SquareMagnitude();
            double aD1VNorm2 = aD1V.SquareMagnitude();
            if (aD1UNorm2 > 1.0)
            {
                aD1U /= Math.Sqrt(aD1UNorm2);
            }
            if (aD1VNorm2 > 1.0)
            {
                aD1V /= Math.Sqrt(aD1VNorm2);
            }

            bool isSingular = false;
            const int aMaxOrder = 3;
            gp_Vec aNorm = aD1U.Crossed(aD1V);

            // Query osculating surface only if singular
            OsculatingInfo aOscInfo = default;
            Geom_BSplineSurface? aOscSurf = null;
            if (aNorm.SquareMagnitude() <= THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL)
            {
                queryOsculating(theOscQuery, theU, theV, ref aOscInfo, ref aOscSurf);
                isSingular = true;
            }

            // Compute sign factor
            double aSign = aOscInfo.Sign();

            // Non-singular case: use direct formulas
            if (!isSingular)
            {
                aNorm.Normalize();
                theValue.SetXYZ(theValue.XYZ() + theOffset * aSign * aNorm.XYZ());

                // Compute normal derivatives using inline formulas
                gp_Vec aN0 = new gp_Vec(aNorm.XYZ()), aN1U = new gp_Vec(), aN1V = new gp_Vec();
                double aScale = (theD1U ^ theD1V).Dot(aN0);
                aN1U.SetX(aD2U.Y() * theD1V.Z() + theD1U.Y() * aD2UV.Z() - aD2U.Z() * theD1V.Y()
                          - theD1U.Z() * aD2UV.Y());
                aN1U.SetY((aD2U.X() * theD1V.Z() + theD1U.X() * aD2UV.Z() - aD2U.Z() * theD1V.X()
                           - theD1U.Z() * aD2UV.X())
                          * -1.0);
                aN1U.SetZ(aD2U.X() * theD1V.Y() + theD1U.X() * aD2UV.Y() - aD2U.Y() * theD1V.X()
                          - theD1U.Y() * aD2UV.X());
                double aScaleU = aN1U.Dot(aN0);
                aN1U.Subtract(aScaleU * aN0);
                aN1U /= aScale;

                aN1V.SetX(aD2UV.Y() * theD1V.Z() + aD2V.Z() * theD1U.Y() - aD2UV.Z() * theD1V.Y()
                          - aD2V.Y() * theD1U.Z());
                aN1V.SetY((aD2UV.X() * theD1V.Z() + aD2V.Z() * theD1U.X() - aD2UV.Z() * theD1V.X()
                           - aD2V.X() * theD1U.Z())
                          * -1.0);
                aN1V.SetZ(aD2UV.X() * theD1V.Y() + aD2V.Y() * theD1U.X() - aD2UV.Y() * theD1V.X()
                          - aD2V.X() * theD1U.Y());
                double aScaleV = aN1V.Dot(aN0);
                aN1V.Subtract(aScaleV * aN0);
                aN1V /= aScale;

                theD1U += theOffset * aSign * aN1U;
                theD1V += theOffset * aSign * aN1V;

                return true;
            }

            // Singular case - use higher order derivatives
            NCollection_Array2<gp_Vec> aDerNUV = new(0, aMaxOrder + 1, 0, aMaxOrder + 1);
            NCollection_Array2<gp_Vec> aDerSurf = new(0, aMaxOrder + 2, 0, aMaxOrder + 2);

            aDerSurf.SetValue(1, 0, theD1U);
            aDerSurf.SetValue(0, 1, theD1V);
            aDerSurf.SetValue(1, 1, aD2UV);
            aDerSurf.SetValue(2, 0, aD2U);
            aDerSurf.SetValue(0, 2, aD2V);

            if (aOscInfo.HasOsculating() && aOscSurf is not null)
            {
                if (!ComputeDerivatives(aMaxOrder, 2, theU, theV, theBasisSurf, 1, 1, aOscInfo.AlongU,
                                        aOscInfo.AlongV, aOscSurf, aDerNUV, aDerSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(aMaxOrder, 2, theU, theV, theBasisSurf, 1, 1, false, false, null, aDerNUV,
                                        aDerSurf))
                {
                    return false;
                }
            }

            gp_Dir aNormal = new gp_Dir();
            CSLib.Normal(aMaxOrder,
                         aDerNUV,
                         THE_D1_MAGNITUDE_TOL,
                         theU,
                         theV,
                         aUMin,
                         aUMax,
                         aVMin,
                         aVMax,
                         out CSLib_NormalStatus aNStatus,
                         ref aNormal,
                         out int aOrderU,
                         out int aOrderV);

            // Handle CSLib_InfinityOfSolutions by replacing zero derivative
            if (aNStatus == CSLib_InfinityOfSolutions)
            {
                gp_Vec aNewDU = theD1U;
                gp_Vec aNewDV = theD1V;
                if (ReplaceDerivative(theU,
                                      theV,
                                      aUMin,
                                      aUMax,
                                      aVMin,
                                      aVMax,
                                      ref aNewDU,
                                      ref aNewDV,
                                      THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL,
                                      theBasisSurf))
                {
                    // Re-compute with replaced derivatives
                    aDerSurf.SetValue(1, 0, aNewDU);
                    aDerSurf.SetValue(0, 1, aNewDV);
                    if (aOscInfo.HasOsculating() && aOscSurf is not null)
                    {
                        if (!ComputeDerivatives(aMaxOrder, 2, theU, theV, theBasisSurf, 1, 1, aOscInfo.AlongU,
                                                aOscInfo.AlongV, aOscSurf, aDerNUV, aDerSurf))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        if (!ComputeDerivatives(aMaxOrder, 2, theU, theV, theBasisSurf, 1, 1, false, false, null,
                                                aDerNUV, aDerSurf))
                        {
                            return false;
                        }
                    }
                    CSLib.Normal(aMaxOrder,
                                 aDerNUV,
                                 THE_D1_MAGNITUDE_TOL,
                                 theU,
                                 theV,
                                 aUMin,
                                 aUMax,
                                 aVMin,
                                 aVMax,
                                 out aNStatus,
                                 ref aNormal,
                                 out aOrderU,
                                 out aOrderV);
                }
            }

            if (aNStatus == CSLib_Defined)
            {
                // Compute offset point
                theValue.SetXYZ(theValue.XYZ() + theOffset * aSign * aNormal.XYZ());
                // Compute D1 using CSLib
                theD1U = CSLib.DNNormal(1, 0, aDerNUV, aOrderU, aOrderV);
                theD1V = CSLib.DNNormal(0, 1, aDerNUV, aOrderU, aOrderV);

                theD1U.Multiply(theOffset * aSign);
                theD1U.Add(aDerSurf[1, 0]);
                theD1V.Multiply(theOffset * aSign);
                theD1V.Add(aDerSurf[0, 1]);
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(aUStart, aVStart, ref theU, ref theV, aUMin, aUMax, aVMin, aVMax, isUPer, isVPer, theD1U,
                            theD1V))
            {
                return false;
            }
        }
    }

    /// <summary>The offset point and first derivatives at (theU, theV); false if it fails at a singular point.</summary>
    internal static bool EvaluateD1<TBasis>(double theU,
                                            double theV,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            out gp_Pnt theValue,
                                            out gp_Vec theD1U,
                                            out gp_Vec theD1V)
        where TBasis : struct, IBasisSurface
    {
        Geom_Surface.ResD2 aBasisD2 = theBasisSurf.EvalD2(theU, theV);
        theValue = aBasisD2.Point;
        theD1U = aBasisD2.D1U;
        theD1V = aBasisD2.D1V;
        return EvaluateD1(theU,
                          theV,
                          theBasisSurf,
                          theOffset,
                          theOscQuery,
                          ref theValue,
                          ref theD1U,
                          ref theD1V,
                          aBasisD2.D2U,
                          aBasisD2.D2V,
                          aBasisD2.D2UV);
    }

    /// <summary>
    /// The offset point and derivatives up to the second order from the basis ones (on input) and third
    /// derivatives; at singular points, retries closer to the center. False if it fails.
    /// </summary>
    internal static bool EvaluateD2<TBasis>(double theU,
                                            double theV,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            ref gp_Pnt theValue,
                                            ref gp_Vec theD1U,
                                            ref gp_Vec theD1V,
                                            ref gp_Vec theD2U,
                                            ref gp_Vec theD2V,
                                            ref gp_Vec theD2UV,
                                            in gp_Vec theD3U,
                                            in gp_Vec theD3V,
                                            in gp_Vec theD3UUV,
                                            in gp_Vec theD3UVV)
        where TBasis : struct, IBasisSurface
    {
        double aUStart = theU;
        double aVStart = theV;
        theBasisSurf.Bounds(out double aUMin, out double aUMax, out double aVMin, out double aVMax);
        bool isUPer = theBasisSurf.IsUPeriodic();
        bool isVPer = theBasisSurf.IsVPeriodic();

        // Use pre-computed D3 for first iteration
        gp_Vec aD3U = theD3U;
        gp_Vec aD3V = theD3V;
        gp_Vec aD3UUV = theD3UUV;
        gp_Vec aD3UVV = theD3UVV;
        bool isFirstIteration = true;

        for (;;)
        {
            // For subsequent iterations, recompute D3 at shifted point
            if (!isFirstIteration)
            {
                Geom_Surface.ResD3 aD3Result = theBasisSurf.EvalD3(theU, theV);
                theValue = aD3Result.Point;
                theD1U = aD3Result.D1U;
                theD1V = aD3Result.D1V;
                theD2U = aD3Result.D2U;
                theD2V = aD3Result.D2V;
                theD2UV = aD3Result.D2UV;
                aD3U = aD3Result.D3U;
                aD3V = aD3Result.D3V;
                aD3UUV = aD3Result.D3UUV;
                aD3UVV = aD3Result.D3UVV;
            }
            isFirstIteration = false;

            if (IsInfiniteCoord(theD1U) || IsInfiniteCoord(theD1V))
            {
                return false;
            }

            // Check if singular using CSLib::Normal
            gp_Dir aNormal = new gp_Dir();
            CSLib.Normal(theD1U, theD1V, THE_D1_MAGNITUDE_TOL, out CSLib_NormalStatus aNStatus, ref aNormal);

            // MaxOrder = 0 for non-singular, 3 for singular
            int aMaxOrder = (aNStatus == CSLib_Defined) ? 0 : 3;

            // Get osculating surface info
            OsculatingInfo aOscInfo = default;
            Geom_BSplineSurface? aOscSurf = null;
            if (aNStatus != CSLib_Defined)
            {
                queryOsculating(theOscQuery, theU, theV, ref aOscInfo, ref aOscSurf);
            }

            // Setup derivative arrays
            NCollection_Array2<gp_Vec> aDerNUV = new(0, aMaxOrder + 2, 0, aMaxOrder + 2);
            NCollection_Array2<gp_Vec> aDerSurf = new(0, aMaxOrder + 3, 0, aMaxOrder + 3);

            aDerSurf.SetValue(1, 0, theD1U);
            aDerSurf.SetValue(0, 1, theD1V);
            aDerSurf.SetValue(1, 1, theD2UV);
            aDerSurf.SetValue(2, 0, theD2U);
            aDerSurf.SetValue(0, 2, theD2V);
            aDerSurf.SetValue(3, 0, aD3U);
            aDerSurf.SetValue(2, 1, aD3UUV);
            aDerSurf.SetValue(1, 2, aD3UVV);
            aDerSurf.SetValue(0, 3, aD3V);

            // Use ComputeDerivatives to populate DerNUV
            if (aOscInfo.HasOsculating() && aOscSurf is not null)
            {
                if (!ComputeDerivatives(aMaxOrder, 3, theU, theV, theBasisSurf, 2, 2, aOscInfo.AlongU,
                                        aOscInfo.AlongV, aOscSurf, aDerNUV, aDerSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(aMaxOrder, 3, theU, theV, theBasisSurf, 2, 2, false, false, null, aDerNUV,
                                        aDerSurf))
                {
                    return false;
                }
            }

            // Compute normal using CSLib (second call with MaxOrder)
            CSLib.Normal(aMaxOrder,
                         aDerNUV,
                         THE_D1_MAGNITUDE_TOL,
                         theU,
                         theV,
                         aUMin,
                         aUMax,
                         aVMin,
                         aVMax,
                         out aNStatus,
                         ref aNormal,
                         out int aOrderU,
                         out int aOrderV);

            if (aNStatus == CSLib_Defined)
            {
                double aSign = theOffset * aOscInfo.Sign();

                // Compute offset point
                theValue.SetXYZ(theValue.XYZ() + aSign * aNormal.XYZ());

                // Compute D1 using CSLib::DNNormal
                theD1U = aDerSurf[1, 0].Added(CSLib.DNNormal(1, 0, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD1V = aDerSurf[0, 1].Added(CSLib.DNNormal(0, 1, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));

                // For D2, re-fetch from basis surface
                gp_Vec aDN20 = theBasisSurf.EvalDN(theU, theV, 2, 0);
                gp_Vec aDN02 = theBasisSurf.EvalDN(theU, theV, 0, 2);
                gp_Vec aDN11 = theBasisSurf.EvalDN(theU, theV, 1, 1);
                theD2U = aDN20;
                theD2V = aDN02;
                theD2UV = aDN11;

                // Add offset corrections using CSLib::DNNormal
                theD2U.Add(CSLib.DNNormal(2, 0, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD2V.Add(CSLib.DNNormal(0, 2, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD2UV.Add(CSLib.DNNormal(1, 1, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(aUStart, aVStart, ref theU, ref theV, aUMin, aUMax, aVMin, aVMax, isUPer, isVPer, theD1U,
                            theD1V))
            {
                return false;
            }
        }
    }

    /// <summary>
    /// The offset point and derivatives up to the second order at (theU, theV); false if it fails at a singular
    /// point.
    /// </summary>
    internal static bool EvaluateD2<TBasis>(double theU,
                                            double theV,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            out gp_Pnt theValue,
                                            out gp_Vec theD1U,
                                            out gp_Vec theD1V,
                                            out gp_Vec theD2U,
                                            out gp_Vec theD2V,
                                            out gp_Vec theD2UV)
        where TBasis : struct, IBasisSurface
    {
        Geom_Surface.ResD3 aBasisD3 = theBasisSurf.EvalD3(theU, theV);
        theValue = aBasisD3.Point;
        theD1U = aBasisD3.D1U;
        theD1V = aBasisD3.D1V;
        theD2U = aBasisD3.D2U;
        theD2V = aBasisD3.D2V;
        theD2UV = aBasisD3.D2UV;
        return EvaluateD2(theU,
                          theV,
                          theBasisSurf,
                          theOffset,
                          theOscQuery,
                          ref theValue,
                          ref theD1U,
                          ref theD1V,
                          ref theD2U,
                          ref theD2V,
                          ref theD2UV,
                          aBasisD3.D3U,
                          aBasisD3.D3V,
                          aBasisD3.D3UUV,
                          aBasisD3.D3UVV);
    }

    /// <summary>
    /// The offset point and derivatives up to the third order at (theU, theV); at singular points, retries closer
    /// to the center. False if it fails.
    /// </summary>
    internal static bool EvaluateD3<TBasis>(double theU,
                                            double theV,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            out gp_Pnt theValue,
                                            out gp_Vec theD1U,
                                            out gp_Vec theD1V,
                                            out gp_Vec theD2U,
                                            out gp_Vec theD2V,
                                            out gp_Vec theD2UV,
                                            out gp_Vec theD3U,
                                            out gp_Vec theD3V,
                                            out gp_Vec theD3UUV,
                                            out gp_Vec theD3UVV)
        where TBasis : struct, IBasisSurface
    {
        double aUStart = theU;
        double aVStart = theV;
        theBasisSurf.Bounds(out double aUMin, out double aUMax, out double aVMin, out double aVMax);
        bool isUPer = theBasisSurf.IsUPeriodic();
        bool isVPer = theBasisSurf.IsVPeriodic();

        for (;;)
        {
            Geom_Surface.ResD3 aD3Result = theBasisSurf.EvalD3(theU, theV);
            theValue = aD3Result.Point;
            theD1U = aD3Result.D1U;
            theD1V = aD3Result.D1V;
            theD2U = aD3Result.D2U;
            theD2V = aD3Result.D2V;
            theD2UV = aD3Result.D2UV;
            theD3U = aD3Result.D3U;
            theD3V = aD3Result.D3V;
            theD3UUV = aD3Result.D3UUV;
            theD3UVV = aD3Result.D3UVV;

            if (IsInfiniteCoord(theD1U) || IsInfiniteCoord(theD1V))
            {
                return false;
            }

            // Check if singular using CSLib::Normal
            gp_Dir aNormal = new gp_Dir();
            CSLib.Normal(theD1U, theD1V, THE_D1_MAGNITUDE_TOL, out CSLib_NormalStatus aNStatus, ref aNormal);

            // MaxOrder = 0 for non-singular, 3 for singular
            int aMaxOrder = (aNStatus == CSLib_Defined) ? 0 : 3;

            // Get osculating surface info
            OsculatingInfo aOscInfo = default;
            Geom_BSplineSurface? aOscSurf = null;
            if (aNStatus != CSLib_Defined)
            {
                queryOsculating(theOscQuery, theU, theV, ref aOscInfo, ref aOscSurf);
            }

            // Setup derivative arrays
            // For D3: DerNUV needs (aMaxOrder + 4), DerSurf needs (aMaxOrder + 5)
            NCollection_Array2<gp_Vec> aDerNUV = new(0, aMaxOrder + 3, 0, aMaxOrder + 3);
            NCollection_Array2<gp_Vec> aDerSurf = new(0, aMaxOrder + 4, 0, aMaxOrder + 4);

            aDerSurf.SetValue(1, 0, theD1U);
            aDerSurf.SetValue(0, 1, theD1V);
            aDerSurf.SetValue(1, 1, theD2UV);
            aDerSurf.SetValue(2, 0, theD2U);
            aDerSurf.SetValue(0, 2, theD2V);
            aDerSurf.SetValue(3, 0, theD3U);
            aDerSurf.SetValue(2, 1, theD3UUV);
            aDerSurf.SetValue(1, 2, theD3UVV);
            aDerSurf.SetValue(0, 3, theD3V);

            // Use ComputeDerivatives to populate DerNUV
            if (aOscInfo.HasOsculating() && aOscSurf is not null)
            {
                if (!ComputeDerivatives(aMaxOrder, 3, theU, theV, theBasisSurf, 3, 3, aOscInfo.AlongU,
                                        aOscInfo.AlongV, aOscSurf, aDerNUV, aDerSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(aMaxOrder, 3, theU, theV, theBasisSurf, 3, 3, false, false, null, aDerNUV,
                                        aDerSurf))
                {
                    return false;
                }
            }

            // Compute normal using CSLib (second call with MaxOrder)
            CSLib.Normal(aMaxOrder,
                         aDerNUV,
                         THE_D1_MAGNITUDE_TOL,
                         theU,
                         theV,
                         aUMin,
                         aUMax,
                         aVMin,
                         aVMax,
                         out aNStatus,
                         ref aNormal,
                         out int aOrderU,
                         out int aOrderV);

            if (aNStatus == CSLib_Defined)
            {
                double aSign = theOffset * aOscInfo.Sign();

                // Compute offset point
                theValue.SetXYZ(theValue.XYZ() + aSign * aNormal.XYZ());

                // Compute D1 using CSLib::DNNormal
                theD1U = aDerSurf[1, 0].Added(CSLib.DNNormal(1, 0, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD1V = aDerSurf[0, 1].Added(CSLib.DNNormal(0, 1, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));

                // For D2 and D3, re-fetch from basis surface
                gp_Vec aDN20 = theBasisSurf.EvalDN(theU, theV, 2, 0);
                gp_Vec aDN02 = theBasisSurf.EvalDN(theU, theV, 0, 2);
                gp_Vec aDN11 = theBasisSurf.EvalDN(theU, theV, 1, 1);
                gp_Vec aDN30 = theBasisSurf.EvalDN(theU, theV, 3, 0);
                gp_Vec aDN03 = theBasisSurf.EvalDN(theU, theV, 0, 3);
                gp_Vec aDN21 = theBasisSurf.EvalDN(theU, theV, 2, 1);
                gp_Vec aDN12 = theBasisSurf.EvalDN(theU, theV, 1, 2);
                theD2U = aDN20;
                theD2V = aDN02;
                theD2UV = aDN11;
                theD3U = aDN30;
                theD3V = aDN03;
                theD3UUV = aDN21;
                theD3UVV = aDN12;

                // Add offset corrections using CSLib::DNNormal
                theD2U.Add(CSLib.DNNormal(2, 0, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD2V.Add(CSLib.DNNormal(0, 2, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD2UV.Add(CSLib.DNNormal(1, 1, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD3U.Add(CSLib.DNNormal(3, 0, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD3V.Add(CSLib.DNNormal(0, 3, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD3UUV.Add(CSLib.DNNormal(2, 1, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                theD3UVV.Add(CSLib.DNNormal(1, 2, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(aUStart, aVStart, ref theU, ref theV, aUMin, aUMax, aVMin, aVMax, isUPer, isVPer, theD1U,
                            theD1V))
            {
                return false;
            }
        }
    }

    /// <summary>
    /// The derivative of order (theNu, theNv) from the basis first derivatives at (theU, theV); at singular points,
    /// retries closer to the center. False if it fails.
    /// </summary>
    internal static bool EvaluateDN<TBasis>(double theU,
                                            double theV,
                                            int theNu,
                                            int theNv,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            out gp_Vec theResult,
                                            in gp_Vec theD1U,
                                            in gp_Vec theD1V)
        where TBasis : struct, IBasisSurface
    {
        theResult = new gp_Vec();
        double aUStart = theU;
        double aVStart = theV;
        theBasisSurf.Bounds(out double aUMin, out double aUMax, out double aVMin, out double aVMax);
        bool isUPer = theBasisSurf.IsUPeriodic();
        bool isVPer = theBasisSurf.IsVPeriodic();

        // Use pre-computed D1 for first iteration
        gp_Vec aD1U = theD1U;
        gp_Vec aD1V = theD1V;
        bool isFirstIteration = true;

        for (;;)
        {
            // For subsequent iterations, recompute D1 at shifted point
            if (!isFirstIteration)
            {
                Geom_Surface.ResD1 aD1Result = theBasisSurf.EvalD1(theU, theV);
                aD1U = aD1Result.D1U;
                aD1V = aD1Result.D1V;
            }
            isFirstIteration = false;

            if (IsInfiniteCoord(aD1U) || IsInfiniteCoord(aD1V))
            {
                return false;
            }

            // Check if singular to determine MaxOrder
            gp_Dir aNormal = new gp_Dir();
            CSLib.Normal(aD1U, aD1V, THE_D1_MAGNITUDE_TOL, out CSLib_NormalStatus aNStatus, ref aNormal);

            int aMaxOrder = (aNStatus == CSLib_Defined) ? 0 : 3;

            NCollection_Array2<gp_Vec> aDerNUV = new(0, aMaxOrder + theNu, 0, aMaxOrder + theNv);
            NCollection_Array2<gp_Vec> aDerSurf = new(0, aMaxOrder + theNu + 1, 0, aMaxOrder + theNv + 1);

            aDerSurf.SetValue(1, 0, aD1U);
            aDerSurf.SetValue(0, 1, aD1V);

            // Check osculating surface only in singular case
            OsculatingInfo aOscInfo = default;
            Geom_BSplineSurface? aOscSurf = null;
            if (aNStatus != CSLib_Defined)
            {
                queryOsculating(theOscQuery, theU, theV, ref aOscInfo, ref aOscSurf);
            }

            // Use ComputeDerivatives
            if (aOscInfo.HasOsculating() && aOscSurf is not null)
            {
                if (!ComputeDerivatives(aMaxOrder, 1, theU, theV, theBasisSurf, theNu, theNv, aOscInfo.AlongU,
                                        aOscInfo.AlongV, aOscSurf, aDerNUV, aDerSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(aMaxOrder, 1, theU, theV, theBasisSurf, theNu, theNv, false, false, null,
                                        aDerNUV, aDerSurf))
                {
                    return false;
                }
            }

            // Compute normal with CSLib
            CSLib.Normal(aMaxOrder,
                         aDerNUV,
                         THE_D1_MAGNITUDE_TOL,
                         theU,
                         theV,
                         aUMin,
                         aUMax,
                         aVMin,
                         aVMax,
                         out aNStatus,
                         ref aNormal,
                         out int aOrderU,
                         out int aOrderV);

            if (aNStatus == CSLib_Defined)
            {
                double aSign = theOffset * aOscInfo.Sign();

                // Compute DN result: basis DN + offset * DNNormal
                gp_Vec aBasisDN = theBasisSurf.EvalDN(theU, theV, theNu, theNv);
                theResult = aBasisDN;
                theResult.Add(CSLib.DNNormal(theNu, theNv, aDerNUV, aOrderU, aOrderV).Multiplied(aSign));
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(aUStart, aVStart, ref theU, ref theV, aUMin, aUMax, aVMin, aVMax, isUPer, isVPer, aD1U,
                            aD1V))
            {
                return false;
            }
        }
    }

    /// <summary>The derivative of order (theNu, theNv) at (theU, theV); false if it fails at a singular point.</summary>
    internal static bool EvaluateDN<TBasis>(double theU,
                                            double theV,
                                            int theNu,
                                            int theNv,
                                            in TBasis theBasisSurf,
                                            double theOffset,
                                            Geom_OsculatingSurface? theOscQuery,
                                            out gp_Vec theResult)
        where TBasis : struct, IBasisSurface
    {
        Geom_Surface.ResD1 aBasisD1 = theBasisSurf.EvalD1(theU, theV);
        return EvaluateDN(theU,
                          theV,
                          theNu,
                          theNv,
                          theBasisSurf,
                          theOffset,
                          theOscQuery,
                          out theResult,
                          aBasisD1.D1U,
                          aBasisD1.D1V);
    }
}
