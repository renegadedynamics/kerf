// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_KnotTools.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_KnotTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.ParametrizationType;

namespace Occt;

/// <summary>
/// The knots of the approximation of a walking line: indices of points where the discrete curvature of the point
/// set changes, filtered so that every knot interval has a reasonable number of points; and the parametrisation
/// to use for a walking line.
/// </summary>
/// <remarks>OCCT's class of static functions. The APPROXINT_KNOTTOOLS_DEBUG prints are left out.</remarks>
public static class ApproxInt_KnotTools
{
    private const double aSinCoeff2 = 0.09549150281252627; // aSinCoeff^2 = (3. - std::sqrt(5.)) / 8.
    private const int aMaxPntCoeff = 15;

    //=================================================================================================
    // function : EvalCurv
    // purpose  : Evaluate curvature in dim-dimension point.
    //=================================================================================================

    private static double EvalCurv(double dim, ReadOnlySpan<double> V1, ReadOnlySpan<double> V2)
    {
        // Really V1 and V2 are arrays of size dim;
        // V1 is first derivative, V2 is second derivative
        // of n-dimension curve
        // Curvature is curv = |V1^V2|/|V1|^3
        // V1^V2 is outer product of two vectors:
        // P(i,j) = V1(i)*V2(j) - V1(j)*V2(i);
        double mp = 0.0;
        int i, j;
        double p;
        for (i = 1; i < dim; ++i)
        {
            for (j = 0; j < i; ++j)
            {
                p = V1[i] * V2[j] - V1[j] * V2[i];
                mp += p * p;
            }
        }
        //
        double q = 0.0;
        for (i = 0; i < dim; ++i)
        {
            q += V1[i] * V1[i];
        }

        if (q < 1 / global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            // Indeed, if q is small then we can
            // obtain equivocation of "0/0" type.
            // In this case, local curvature can be
            // not equal to 0 or Infinity.
            // However, it is good solution to insert
            // knot in the place with such singularity.
            // Therefore, we need imitation of curvature
            // jumping. Return of Precision::Infinite() is
            // enough for it.

            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        q = Min(q, global::Kerf.Engine.Foundation.Precision.Infinite());
        q *= q * q;

        //
        double curv = Math.Sqrt(mp / q);

        return curv;
    }

    /// <summary>Builds the discrete curvature of the points <paramref name="theCoords"/> of dimension <paramref name="theDim"/>.</summary>
    public static void BuildCurvature(NCollection_LocalArray<double> theCoords, int theDim, math_Vector thePars,
                                      NCollection_Array1<double> theCurv, out double theMaxCurv)
    {
        // Arrays are allocated for max theDim = 7: 1 3d curve + 2 2d curves.
        Span<double> Val = stackalloc double[21];
        Span<double> Par = stackalloc double[3];
        Span<double> Res = stackalloc double[21];
        int i, j, m, ic;
        int dim = theDim;
        //
        theMaxCurv = 0.0;
        if (theCurv.Length() < 3)
        {
            theCurv.Init(0.0);
            return;
        }
        i = theCurv.Lower();
        for (j = 0; j < 3; ++j)
        {
            int k = i + j;
            ic = (k - theCurv.Lower()) * dim;
            int l = dim * j;
            for (m = 0; m < dim; ++m)
            {
                Val[l + m] = theCoords[ic + m];
            }
            Par[j] = thePars[k];
        }
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(Par[0], 2, 2, dim, Val, Par, Res);
        //
        theCurv[i] = EvalCurv(dim, Res[dim..], Res[(2 * dim)..]);
        //
        if (theCurv[i] > theMaxCurv)
        {
            theMaxCurv = theCurv[i];
        }
        //
        for (i = theCurv.Lower() + 1; i < theCurv.Upper(); ++i)
        {
            for (j = 0; j < 3; ++j)
            {
                int k = i + j - 1;
                ic = (k - theCurv.Lower()) * dim;
                int l = dim * j;
                for (m = 0; m < dim; ++m)
                {
                    Val[l + m] = theCoords[ic + m];
                }
                Par[j] = thePars[k];
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(Par[1], 2, 2, dim, Val, Par, Res);
            //
            theCurv[i] = EvalCurv(dim, Res[dim..], Res[(2 * dim)..]);
            if (theCurv[i] > theMaxCurv)
            {
                theMaxCurv = theCurv[i];
            }
        }
        //
        i = theCurv.Upper();
        for (j = 0; j < 3; ++j)
        {
            int k = i + j - 2;
            ic = (k - theCurv.Lower()) * dim;
            int l = dim * j;
            for (m = 0; m < dim; ++m)
            {
                Val[l + m] = theCoords[ic + m];
            }
            Par[j] = thePars[k];
        }
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(Par[2], 2, 2, dim, Val, Par, Res);
        //
        theCurv[i] = EvalCurv(dim, Res[dim..], Res[(2 * dim)..]);
        if (theCurv[i] > theMaxCurv)
        {
            theMaxCurv = theCurv[i];
        }
    }

    //! Compute indices of knots:
    //!
    //! I: Build discrete curvature in points set,
    //! using outer product of two vectors.
    //!
    //! II: Put knots in points which has extremity on discrete curvature.
    //!
    //! III: Put knots in monotone intervals of curvature.
    //!
    //! IV: Put additional knots near extrema points.
    private static void ComputeKnotInds(NCollection_LocalArray<double> theCoords, int theDim, math_Vector thePars,
                                        NCollection_Sequence<int> theInds)
    {
        // I: Create discrete curvature.
        NCollection_Sequence<int> aFeatureInds = new();
        NCollection_Array1<double> aCurv = new(thePars.Lower(), thePars.Upper());
        BuildCurvature(theCoords, theDim, thePars, aCurv, out double aMaxCurv);
        //
        int i, j, dim = theDim;

        theInds.Append(aCurv.Lower());
        if (aMaxCurv <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // Linear case.
            theInds.Append(aCurv.Upper());
            return;
        }

        // II: Find extremas of curvature.
        // Not used Precision::PConfusion, by different from "param space" eps nature.
        double eps = 1.0e-9, eps1 = 1.0e3 * eps;
        for (i = aCurv.Lower() + 1; i < aCurv.Upper(); ++i)
        {
            double d1 = aCurv[i] - aCurv[i - 1], d2 = aCurv[i] - aCurv[i + 1], ad1 = Math.Abs(d1),
                   ad2 = Math.Abs(d2);

            if (d1 * d2 > 0.0 && ad1 > eps && ad2 > eps)
            {
                if (i != theInds.Last())
                {
                    theInds.Append(i);
                    aFeatureInds.Append(i);
                }
            }
            else if ((ad1 < eps && ad2 > eps1) || (ad1 > eps1 && ad2 < eps))
            {
                if (i != theInds.Last())
                {
                    theInds.Append(i);
                    aFeatureInds.Append(i);
                }
            }
        }
        if (aCurv.Upper() != theInds.Last())
        {
            theInds.Append(aCurv.Upper());
        }

        // III: Put knots in monotone intervals of curvature.
        bool Ok;
        i = 1;
        do
        {
            i++;
            //
            Ok = InsKnotBefI(i, aCurv, theCoords, dim, theInds, true);
            if (Ok)
            {
                i--;
            }
        } while (i < theInds.Length());

        // IV: Checking feature points.
        j = 2;
        for (i = 1; i <= aFeatureInds.Length(); ++i)
        {
            int anInd = aFeatureInds[i];
            for (; j <= theInds.Length() - 1;)
            {
                if (theInds[j] == anInd)
                {
                    int anIndPrev = theInds[j - 1];
                    int anIndNext = theInds[j + 1];
                    int ici = (anIndPrev - aCurv.Lower()) * theDim, ici1 = (anIndNext - aCurv.Lower()) * theDim,
                        icm = (anInd - aCurv.Lower()) * theDim;
                    NCollection_LocalArray<double> V1 = new(theDim), V2 = new(theDim);
                    double mp = 0.0, m1 = 0.0, m2 = 0.0;
                    double p;
                    for (int k = 0; k < theDim; ++k)
                    {
                        V1[k] = theCoords[icm + k] - theCoords[ici + k];
                        m1 += V1[k] * V1[k];
                        V2[k] = theCoords[ici1 + k] - theCoords[icm + k];
                        m2 += V2[k] * V2[k];
                    }
                    for (int k = 1; k < theDim; ++k)
                    {
                        for (int l = 0; l < k; ++l)
                        {
                            p = V1[k] * V2[l] - V1[l] * V2[k];
                            mp += p * p;
                        }
                    }
                    // mp *= 2.; //P(j,i) = -P(i,j);
                    //

                    if (mp > aSinCoeff2 * m1 * m2) // std::sqrt(mp/(m1*m2)) > aSinCoeff
                    {
                        // Insert new knots
                        double d1 = Math.Abs(aCurv[anInd] - aCurv[anIndPrev]);
                        double d2 = Math.Abs(aCurv[anInd] - aCurv[anIndNext]);
                        if (d1 > d2)
                        {
                            Ok = InsKnotBefI(j, aCurv, theCoords, dim, theInds, false);
                            if (Ok)
                            {
                                j++;
                            }
                            else
                            {
                                break;
                            }
                        }
                        else
                        {
                            Ok = InsKnotBefI(j + 1, aCurv, theCoords, dim, theInds, false);
                            if (!Ok)
                            {
                                break;
                            }
                        }
                    }
                    else
                    {
                        j++;
                        break;
                    }
                }
                else
                {
                    j++;
                }
            }
        }
        //
    }

    //! Perform knots filtration.
    //!
    //! I: Filter too big number of points per knot interval.
    //!
    //! II: Filter points with too small amount of points per knot interval.
    //!
    //! III: Fill Last Knot.
    private static void FilterKnots(NCollection_Sequence<int> theInds, int theMinNbPnts,
                                    NCollection_DynamicArray<int> theLKnots)
    {
        // Maximum number of points per knot interval.
        int aMaxNbPnts = aMaxPntCoeff * theMinNbPnts;
        int i = 1;
        int aMinNbStep = theMinNbPnts / 2;

        // I: Filter too big number of points per knot interval.
        while (i < theInds.Length())
        {
            int nbint = theInds[i + 1] - theInds[i] + 1;
            if (nbint <= aMaxNbPnts)
            {
                ++i;
                continue;
            }
            else
            {
                int ind = theInds[i] + nbint / 2;
                theInds.InsertAfter(i, ind);
            }
        }

        // II: Filter points with too small amount of points per knot interval.
        i = 1;
        theLKnots.Append(theInds[i]);
        int anIndsPrev = theInds[i];
        for (i = 2; i <= theInds.Length(); ++i)
        {
            if (theInds[i] - anIndsPrev <= theMinNbPnts)
            {
                if (i != theInds.Length())
                {
                    int anIdx = i + 1;
                    for (; anIdx <= theInds.Length(); ++anIdx)
                    {
                        if (theInds[anIdx] - anIndsPrev >= theMinNbPnts)
                        {
                            break;
                        }
                    }
                    anIdx--;

                    int aMidIdx = (theInds[anIdx] + anIndsPrev) / 2;
                    if (aMidIdx - anIndsPrev < theMinNbPnts && aMidIdx - theInds[anIdx] < theMinNbPnts
                        && theInds[anIdx] - anIndsPrev >= aMinNbStep)
                    {
                        if (theInds[anIdx] - anIndsPrev > 2 * theMinNbPnts)
                        {
                            // Bad distribution points merge into one knot interval.
                            theLKnots.Append(anIndsPrev + theMinNbPnts);
                            anIndsPrev = anIndsPrev + theMinNbPnts;
                            i = anIdx - 1;
                        }
                        else
                        {
                            if (theInds[anIdx - 1] - anIndsPrev >= theMinNbPnts / 2)
                            {
                                // Bad distribution points merge into one knot interval.
                                theLKnots.Append(theInds[anIdx - 1]);
                                anIndsPrev = theInds[anIdx - 1];
                                i = anIdx - 1;
                                if (theInds[anIdx] - theInds[anIdx - 1] <= theMinNbPnts / 2)
                                {
                                    theLKnots.SetValue(theLKnots.Upper(), theInds[anIdx]);
                                    anIndsPrev = theInds[anIdx];
                                    i = anIdx;
                                }
                            }
                            else
                            {
                                // Bad distribution points merge into one knot interval.
                                theLKnots.Append(theInds[anIdx]);
                                anIndsPrev = theInds[anIdx];
                                i = anIdx;
                            }
                        }
                    }
                    else if (anIdx == theInds.Upper() && // Last point obtained.
                             theLKnots.Length() >= 2)    // It is possible to modify last item.
                    {
                        // Current bad interval from i to last.
                        // Trying to add knot to divide sequence on two parts:
                        // Last good index -> Last index - theMinNbPnts -> Last index
                        int aLastGoodIdx = theLKnots.Value(theLKnots.Upper() - 1);
                        if (theInds.Last() - 2 * theMinNbPnts >= aLastGoodIdx)
                        {
                            theLKnots[theLKnots.Upper()] = theInds.Last() - theMinNbPnts;
                            theLKnots.Append(theInds.Last());
                            anIndsPrev = theInds[anIdx];
                            i = anIdx;
                        }
                    }
                } // if (i != theInds.Length())
                continue;
            }
            else
            {
                theLKnots.Append(theInds[i]);
                anIndsPrev = theInds[i];
            }
        }

        // III: Fill Last Knot.
        if (theLKnots.Length() < 2)
        {
            theLKnots.Append(theInds.Last());
        }
        else
        {
            if (theLKnots.Last() < theInds.Last())
            {
                theLKnots[theLKnots.Upper()] = theInds.Last();
            }
        }
    }

    //! Insert knots before index I.
    //!
    //! I: Check curvature change:
    //! if ( maxCurvature / minCurvature ) of current interval greater than
    //! threshold value, then stop and use upper index as knot.
    //!
    //! II: Check midpoint criteria:
    //! If exist point between two knot indices with angle greater than
    //! threshold value, then stop and put this index as knot.
    private static bool InsKnotBefI(int theI, NCollection_Array1<double> theCurv, NCollection_LocalArray<double> theCoords,
                                    int theDim, NCollection_Sequence<int> theInds, bool ChkCurv)
    {
        int anInd1 = theInds[theI];
        int anInd = theInds[theI - 1];
        //
        if ((anInd1 - anInd) == 1)
        {
            return false;
        }
        //
        double curv = 0.5 * (theCurv[anInd] + theCurv[anInd1]);
        int mid = 0, j, jj;
        const double aLimitCurvatureChange = 3.0;
        for (j = anInd + 1; j < anInd1; ++j)
        {
            mid = 0;

            // I: Curvature change criteria:
            // Non-null curvature.
            if (theCurv[j] > global::Kerf.Engine.Foundation.Precision.Confusion() && theCurv[anInd] > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (theCurv[j] / theCurv[anInd] > aLimitCurvatureChange
                    || theCurv[j] / theCurv[anInd] < 1.0 / aLimitCurvatureChange)
                {
                    // Curvature on current interval changed more than 3 times.
                    mid = j;
                    theInds.InsertBefore(theI, mid);
                    return true;
                }
            }

            // II: Angular criteria:
            double ac = theCurv[j - 1], ac1 = theCurv[j];
            if ((curv >= ac && curv <= ac1) || (curv >= ac1 && curv <= ac))
            {
                if (Math.Abs(curv - ac) < Math.Abs(curv - ac1))
                {
                    mid = j - 1;
                }
                else
                {
                    mid = j;
                }
            }
            if (mid == anInd)
            {
                mid++;
            }
            if (mid == anInd1)
            {
                mid--;
            }
            if (mid > 0)
            {
                if (ChkCurv)
                {
                    int ici = (anInd - theCurv.Lower()) * theDim, ici1 = (anInd1 - theCurv.Lower()) * theDim,
                        icm = (mid - theCurv.Lower()) * theDim;
                    NCollection_LocalArray<double> V1 = new(theDim), V2 = new(theDim);
                    int i;
                    double mp = 0.0, m1 = 0.0, m2 = 0.0;
                    double p;
                    for (i = 0; i < theDim; ++i)
                    {
                        V1[i] = theCoords[icm + i] - theCoords[ici + i];
                        m1 += V1[i] * V1[i];
                        V2[i] = theCoords[ici1 + i] - theCoords[icm + i];
                        m2 += V2[i] * V2[i];
                    }
                    for (i = 1; i < theDim; ++i)
                    {
                        for (jj = 0; jj < i; ++jj)
                        {
                            p = V1[i] * V2[jj] - V1[jj] * V2[i];
                            mp += p * p;
                        }
                    }
                    // mp *= 2.; //P(j,i) = -P(i,j);
                    //

                    if (mp > aSinCoeff2 * m1 * m2) // std::sqrt(mp / m1m2) > aSinCoeff
                    {
                        theInds.InsertBefore(theI, mid);
                        return true;
                    }
                }
                else
                {
                    theInds.InsertBefore(theI, mid);
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>
    /// Builds the optimal knot sequence (indices of points) of a point set: at least one of the sets
    /// <paramref name="thePntsXYZ"/>, <paramref name="thePntsU1V1"/>, <paramref name="thePntsU2V2"/> (flagged by
    /// <paramref name="theApproxXYZ"/>, ...) must exist; <paramref name="thePars"/> are the expected parameters;
    /// <paramref name="theMinNbPnts"/> is the minimal number of points per knot interval.
    /// </summary>
    public static void BuildKnots(NCollection_Array1<Point3D> thePntsXYZ, NCollection_Array1<Point2D> thePntsU1V1,
                                  NCollection_Array1<Point2D> thePntsU2V2, math_Vector thePars, bool theApproxXYZ,
                                  bool theApproxU1V1, bool theApproxU2V2, int theMinNbPnts,
                                  NCollection_DynamicArray<int> theKnots)
    {
        NCollection_Sequence<int> aKnots = new();
        int aDim = 0;

        // I: Convert input data to the corresponding format.
        if (theApproxXYZ)
        {
            aDim += 3;
        }
        if (theApproxU1V1)
        {
            aDim += 2;
        }
        if (theApproxU2V2)
        {
            aDim += 2;
        }

        if (aDim == 0 || thePars.Length() < 2)
        {
            theKnots.Clear();
            return;
        }

        NCollection_LocalArray<double> aCoords = new(thePars.Length() * aDim);
        int i, j;
        for (i = thePars.Lower(); i <= thePars.Upper(); ++i)
        {
            j = (i - thePars.Lower()) * aDim;
            if (theApproxXYZ)
            {
                aCoords[j] = thePntsXYZ.Value(i).X();
                ++j;
                aCoords[j] = thePntsXYZ.Value(i).Y();
                ++j;
                aCoords[j] = thePntsXYZ.Value(i).Z();
                ++j;
            }
            if (theApproxU1V1)
            {
                aCoords[j] = thePntsU1V1.Value(i).X();
                ++j;
                aCoords[j] = thePntsU1V1.Value(i).Y();
                ++j;
            }
            if (theApproxU2V2)
            {
                aCoords[j] = thePntsU2V2.Value(i).X();
                ++j;
                aCoords[j] = thePntsU2V2.Value(i).Y();
                ++j;
            }
        }

        // II: Build draft knot sequence.
        ComputeKnotInds(aCoords, aDim, thePars, aKnots);

        // III: Build output knot sequence.
        FilterKnots(aKnots, theMinNbPnts, theKnots);
    }

    //=================================================================================================

    private static double MaxParamRatio(math_Vector thePars)
    {
        int i;
        double aMaxRatio = 0.0;
        //
        for (i = thePars.Lower() + 1; i < thePars.Upper(); ++i)
        {
            double aDenom = thePars[i] - thePars[i - 1];
            if (Math.Abs(aDenom) < global::Kerf.Engine.Foundation.Precision.Computational())
            {
                continue;
            }
            double aRat = (thePars[i + 1] - thePars[i]) / aDenom;
            if (aRat > 0.0 && aRat < 1.0)
            {
                aRat = 1.0 / aRat;
            }

            aMaxRatio = Max(aMaxRatio, aRat);
        }
        return aMaxRatio;
    }

    /// <summary>
    /// The preferable parametrisation of the points <paramref name="theFpar"/> to <paramref name="theLpar"/> of
    /// <paramref name="theWL"/>: centripetal where the curvature varies much, else chord length.
    /// </summary>
    public static ParametrizationType DefineParType(IntPatch_WLine theWL, int theFpar, int theLpar,
                                                           bool theApproxXYZ, bool theApproxU1V1, bool theApproxU2V2)
    {
        if (theLpar - theFpar == 1)
        {
            return IsoParametric;
        }

        int nbp3d = theApproxXYZ ? 1 : 0, nbp2d = (theApproxU1V1 ? 1 : 0) + (theApproxU2V2 ? 1 : 0);

        GeomInt_TheMultiLineOfWLApprox aTestLine = new(theWL, nbp3d, nbp2d, theApproxU1V1, theApproxU2V2, 0.0, 0.0,
                                                       0.0, 0.0, 0.0, 0.0, 0.0, theApproxU1V1, theFpar, theLpar);

        NCollection_Array1<Point3D> aTabPnt3d = new(1, Max(1, nbp3d));
        NCollection_Array1<Point2D> aTabPnt2d = new(1, Max(1, nbp2d));
        NCollection_Array1<Point3D> aPntXYZ = new(theFpar, theLpar);
        NCollection_Array1<Point2D> aPntU1V1 = new(theFpar, theLpar);
        NCollection_Array1<Point2D> aPntU2V2 = new(theFpar, theLpar);

        int i, j;

        for (i = theFpar; i <= theLpar; ++i)
        {
            if (nbp3d != 0 && nbp2d != 0)
            {
                aTestLine.Value(i, aTabPnt3d, aTabPnt2d);
            }
            else if (nbp2d != 0)
            {
                aTestLine.Value(i, aTabPnt2d);
            }
            else if (nbp3d != 0)
            {
                aTestLine.Value(i, aTabPnt3d);
            }
            //
            if (nbp3d > 0)
            {
                aPntXYZ[i] = aTabPnt3d[1];
            }
            if (nbp2d > 1)
            {
                aPntU1V1[i] = aTabPnt2d[1];
                aPntU2V2[i] = aTabPnt2d[2];
            }
            else if (nbp2d > 0)
            {
                if (theApproxU1V1)
                {
                    aPntU1V1[i] = aTabPnt2d[1];
                }
                else
                {
                    aPntU2V2[i] = aTabPnt2d[1];
                }
            }
        }

        int aDim = 0;

        if (theApproxXYZ)
        {
            aDim += 3;
        }
        if (theApproxU1V1)
        {
            aDim += 2;
        }
        if (theApproxU2V2)
        {
            aDim += 2;
        }

        int aLength = theLpar - theFpar + 1;
        NCollection_LocalArray<double> aCoords = new(aLength * aDim);
        for (i = theFpar; i <= theLpar; ++i)
        {
            j = (i - theFpar) * aDim;
            if (theApproxXYZ)
            {
                aCoords[j] = aPntXYZ.Value(i).X();
                ++j;
                aCoords[j] = aPntXYZ.Value(i).Y();
                ++j;
                aCoords[j] = aPntXYZ.Value(i).Z();
                ++j;
            }
            if (theApproxU1V1)
            {
                aCoords[j] = aPntU1V1.Value(i).X();
                ++j;
                aCoords[j] = aPntU1V1.Value(i).Y();
                ++j;
            }
            if (theApproxU2V2)
            {
                aCoords[j] = aPntU2V2.Value(i).X();
                ++j;
                aCoords[j] = aPntU2V2.Value(i).Y();
                ++j;
            }
        }

        // Analysis of curvature
        const double aCritRat = 500.0;
        const double aCritParRat = 100.0;
        math_Vector aPars = new(theFpar, theLpar);
        ParametrizationType aParType = ChordLength;
        GeomInt_WLApprox.Parameters(aTestLine, theFpar, theLpar, aParType, aPars);
        NCollection_Array1<double> aCurv = new(aPars.Lower(), aPars.Upper());
        BuildCurvature(aCoords, aDim, aPars, aCurv, out double aMaxCurv);

        if (aMaxCurv < global::Kerf.Engine.Foundation.Precision.PConfusion() || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aMaxCurv))
        {
            // Linear case
            return aParType;
        }

        double aMidCurv = 0.0;
        double eps = Epsilon(1.0);
        j = 0;
        for (i = aCurv.Lower(); i <= aCurv.Upper(); ++i)
        {
            if (aMaxCurv - aCurv[i] < eps)
            {
                continue;
            }
            ++j;
            aMidCurv += aCurv[i];
        }

        if (j > 1)
        {
            aMidCurv /= j;
        }

        if (aMidCurv <= eps)
        {
            return aParType;
        }

        double aRat = aMaxCurv / aMidCurv;

        if (aRat > aCritRat)
        {
            if (aRat > 5.0 * aCritRat)
            {
                aParType = Centripetal;
            }
            else
            {
                double aParRat = MaxParamRatio(aPars);
                if (aParRat > aCritParRat)
                {
                    aParType = Centripetal;
                }
            }
        }

        return aParType;
    }
}
