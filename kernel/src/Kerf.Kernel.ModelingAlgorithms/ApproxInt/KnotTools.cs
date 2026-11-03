// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_KnotTools.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_KnotTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Collections;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.ParametrizationType;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// The knots of the approximation of a walking line: indices of points where the discrete curvature of the point
/// set changes, filtered so that every knot interval has a reasonable number of points; and the parametrisation
/// to use for a walking line.
/// </summary>
/// <remarks>OCCT's class of static functions. The APPROXINT_KNOTTOOLS_DEBUG prints are left out.</remarks>
public static class KnotTools
{
    private const double ASinCoeff2 = 0.09549150281252627; // aSinCoeff^2 = (3. - std::sqrt(5.)) / 8.
    private const int AMaxPntCoeff = 15;

    //=================================================================================================
    // function : EvalCurv
    // purpose  : Evaluate curvature in dim-dimension point.
    //=================================================================================================

    private static double EvalCurv(double dim, ReadOnlySpan<double> v1, ReadOnlySpan<double> v2)
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
                p = v1[i] * v2[j] - v1[j] * v2[i];
                mp += p * p;
            }
        }
        //
        double q = 0.0;
        for (i = 0; i < dim; ++i)
        {
            q += v1[i] * v1[i];
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

    /// <summary>Builds the discrete curvature of the points <paramref name="coords"/> of dimension <paramref name="theDim"/>.</summary>
    public static void BuildCurvature(NCollection_LocalArray<double> coords, int theDim, math_Vector pars,
                                      NCollection_Array1<double> curv, out double maxCurv)
    {
        // Arrays are allocated for max theDim = 7: 1 3d curve + 2 2d curves.
        Span<double> val = stackalloc double[21];
        Span<double> par = stackalloc double[3];
        Span<double> res = stackalloc double[21];
        int i, j, m, ic;
        int dim = theDim;
        //
        maxCurv = 0.0;
        if (curv.Length() < 3)
        {
            curv.Init(0.0);
            return;
        }
        i = curv.Lower();
        for (j = 0; j < 3; ++j)
        {
            int k = i + j;
            ic = (k - curv.Lower()) * dim;
            int l = dim * j;
            for (m = 0; m < dim; ++m)
            {
                val[l + m] = coords[ic + m];
            }
            par[j] = pars[k];
        }
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(par[0], 2, 2, dim, val, par, res);
        //
        curv[i] = EvalCurv(dim, res[dim..], res[(2 * dim)..]);
        //
        if (curv[i] > maxCurv)
        {
            maxCurv = curv[i];
        }
        //
        for (i = curv.Lower() + 1; i < curv.Upper(); ++i)
        {
            for (j = 0; j < 3; ++j)
            {
                int k = i + j - 1;
                ic = (k - curv.Lower()) * dim;
                int l = dim * j;
                for (m = 0; m < dim; ++m)
                {
                    val[l + m] = coords[ic + m];
                }
                par[j] = pars[k];
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(par[1], 2, 2, dim, val, par, res);
            //
            curv[i] = EvalCurv(dim, res[dim..], res[(2 * dim)..]);
            if (curv[i] > maxCurv)
            {
                maxCurv = curv[i];
            }
        }
        //
        i = curv.Upper();
        for (j = 0; j < 3; ++j)
        {
            int k = i + j - 2;
            ic = (k - curv.Lower()) * dim;
            int l = dim * j;
            for (m = 0; m < dim; ++m)
            {
                val[l + m] = coords[ic + m];
            }
            par[j] = pars[k];
        }
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(par[2], 2, 2, dim, val, par, res);
        //
        curv[i] = EvalCurv(dim, res[dim..], res[(2 * dim)..]);
        if (curv[i] > maxCurv)
        {
            maxCurv = curv[i];
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
    private static void ComputeKnotInds(NCollection_LocalArray<double> coords, int theDim, math_Vector pars,
                                        NCollection_Sequence<int> inds)
    {
        // I: Create discrete curvature.
        NCollection_Sequence<int> featureInds = new();
        NCollection_Array1<double> curv = new(pars.Lower(), pars.Upper());
        BuildCurvature(coords, theDim, pars, curv, out double maxCurv);
        //
        int i, j, dim = theDim;

        inds.Append(curv.Lower());
        if (maxCurv <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // Linear case.
            inds.Append(curv.Upper());
            return;
        }

        // II: Find extremas of curvature.
        // Not used Precision::PConfusion, by different from "param space" eps nature.
        double eps = 1.0e-9, eps1 = 1.0e3 * eps;
        for (i = curv.Lower() + 1; i < curv.Upper(); ++i)
        {
            double d1 = curv[i] - curv[i - 1], d2 = curv[i] - curv[i + 1], ad1 = Math.Abs(d1),
                   ad2 = Math.Abs(d2);

            if (d1 * d2 > 0.0 && ad1 > eps && ad2 > eps)
            {
                if (i != inds.Last())
                {
                    inds.Append(i);
                    featureInds.Append(i);
                }
            }
            else if ((ad1 < eps && ad2 > eps1) || (ad1 > eps1 && ad2 < eps))
            {
                if (i != inds.Last())
                {
                    inds.Append(i);
                    featureInds.Append(i);
                }
            }
        }
        if (curv.Upper() != inds.Last())
        {
            inds.Append(curv.Upper());
        }

        // III: Put knots in monotone intervals of curvature.
        bool ok;
        i = 1;
        do
        {
            i++;
            //
            ok = InsKnotBefI(i, curv, coords, dim, inds, true);
            if (ok)
            {
                i--;
            }
        } while (i < inds.Length());

        // IV: Checking feature points.
        j = 2;
        for (i = 1; i <= featureInds.Length(); ++i)
        {
            int ind = featureInds[i];
            for (; j <= inds.Length() - 1;)
            {
                if (inds[j] == ind)
                {
                    int indPrev = inds[j - 1];
                    int indNext = inds[j + 1];
                    int ici = (indPrev - curv.Lower()) * theDim, ici1 = (indNext - curv.Lower()) * theDim,
                        icm = (ind - curv.Lower()) * theDim;
                    NCollection_LocalArray<double> v1 = new(theDim), v2 = new(theDim);
                    double mp = 0.0, m1 = 0.0, m2 = 0.0;
                    double p;
                    for (int k = 0; k < theDim; ++k)
                    {
                        v1[k] = coords[icm + k] - coords[ici + k];
                        m1 += v1[k] * v1[k];
                        v2[k] = coords[ici1 + k] - coords[icm + k];
                        m2 += v2[k] * v2[k];
                    }
                    for (int k = 1; k < theDim; ++k)
                    {
                        for (int l = 0; l < k; ++l)
                        {
                            p = v1[k] * v2[l] - v1[l] * v2[k];
                            mp += p * p;
                        }
                    }
                    // mp *= 2.; //P(j,i) = -P(i,j);
                    //

                    if (mp > ASinCoeff2 * m1 * m2) // std::sqrt(mp/(m1*m2)) > aSinCoeff
                    {
                        // Insert new knots
                        double d1 = Math.Abs(curv[ind] - curv[indPrev]);
                        double d2 = Math.Abs(curv[ind] - curv[indNext]);
                        if (d1 > d2)
                        {
                            ok = InsKnotBefI(j, curv, coords, dim, inds, false);
                            if (ok)
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
                            ok = InsKnotBefI(j + 1, curv, coords, dim, inds, false);
                            if (!ok)
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
    private static void FilterKnots(NCollection_Sequence<int> inds, int minNbPnts,
                                    List<int> lKnots)
    {
        // Maximum number of points per knot interval.
        int maxNbPnts = AMaxPntCoeff * minNbPnts;
        int i = 1;
        int minNbStep = minNbPnts / 2;

        // I: Filter too big number of points per knot interval.
        while (i < inds.Length())
        {
            int nbint = inds[i + 1] - inds[i] + 1;
            if (nbint <= maxNbPnts)
            {
                ++i;
                continue;
            }
            else
            {
                int ind = inds[i] + nbint / 2;
                inds.InsertAfter(i, ind);
            }
        }

        // II: Filter points with too small amount of points per knot interval.
        i = 1;
        lKnots.Add(inds[i]);
        int indsPrev = inds[i];
        for (i = 2; i <= inds.Length(); ++i)
        {
            if (inds[i] - indsPrev <= minNbPnts)
            {
                if (i != inds.Length())
                {
                    int idx = i + 1;
                    for (; idx <= inds.Length(); ++idx)
                    {
                        if (inds[idx] - indsPrev >= minNbPnts)
                        {
                            break;
                        }
                    }
                    idx--;

                    int midIdx = (inds[idx] + indsPrev) / 2;
                    if (midIdx - indsPrev < minNbPnts && midIdx - inds[idx] < minNbPnts
                        && inds[idx] - indsPrev >= minNbStep)
                    {
                        if (inds[idx] - indsPrev > 2 * minNbPnts)
                        {
                            // Bad distribution points merge into one knot interval.
                            lKnots.Add(indsPrev + minNbPnts);
                            indsPrev = indsPrev + minNbPnts;
                            i = idx - 1;
                        }
                        else
                        {
                            if (inds[idx - 1] - indsPrev >= minNbPnts / 2)
                            {
                                // Bad distribution points merge into one knot interval.
                                lKnots.Add(inds[idx - 1]);
                                indsPrev = inds[idx - 1];
                                i = idx - 1;
                                if (inds[idx] - inds[idx - 1] <= minNbPnts / 2)
                                {
                                    lKnots.SetAt(lKnots.Count - 1, inds[idx]);
                                    indsPrev = inds[idx];
                                    i = idx;
                                }
                            }
                            else
                            {
                                // Bad distribution points merge into one knot interval.
                                lKnots.Add(inds[idx]);
                                indsPrev = inds[idx];
                                i = idx;
                            }
                        }
                    }
                    else if (idx == inds.Upper() && // Last point obtained.
                             lKnots.Count >= 2)    // It is possible to modify last item.
                    {
                        // Current bad interval from i to last.
                        // Trying to add knot to divide sequence on two parts:
                        // Last good index -> Last index - theMinNbPnts -> Last index
                        int lastGoodIdx = lKnots[(lKnots.Count - 1) - 1];
                        if (inds.Last() - 2 * minNbPnts >= lastGoodIdx)
                        {
                            lKnots[lKnots.Count - 1] = inds.Last() - minNbPnts;
                            lKnots.Add(inds.Last());
                            indsPrev = inds[idx];
                            i = idx;
                        }
                    }
                } // if (i != theInds.Length())
                continue;
            }
            else
            {
                lKnots.Add(inds[i]);
                indsPrev = inds[i];
            }
        }

        // III: Fill Last Knot.
        if (lKnots.Count < 2)
        {
            lKnots.Add(inds.Last());
        }
        else
        {
            if (lKnots[^1] < inds.Last())
            {
                lKnots[lKnots.Count - 1] = inds.Last();
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
    private static bool InsKnotBefI(int theI, NCollection_Array1<double> theCurv, NCollection_LocalArray<double> coords,
                                    int dim, NCollection_Sequence<int> inds, bool chkCurv)
    {
        int ind1 = inds[theI];
        int ind = inds[theI - 1];
        //
        if ((ind1 - ind) == 1)
        {
            return false;
        }
        //
        double curv = 0.5 * (theCurv[ind] + theCurv[ind1]);
        int mid = 0, j, jj;
        const double limitCurvatureChange = 3.0;
        for (j = ind + 1; j < ind1; ++j)
        {
            mid = 0;

            // I: Curvature change criteria:
            // Non-null curvature.
            if (theCurv[j] > global::Kerf.Engine.Foundation.Precision.Confusion() && theCurv[ind] > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (theCurv[j] / theCurv[ind] > limitCurvatureChange
                    || theCurv[j] / theCurv[ind] < 1.0 / limitCurvatureChange)
                {
                    // Curvature on current interval changed more than 3 times.
                    mid = j;
                    inds.InsertBefore(theI, mid);
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
            if (mid == ind)
            {
                mid++;
            }
            if (mid == ind1)
            {
                mid--;
            }
            if (mid > 0)
            {
                if (chkCurv)
                {
                    int ici = (ind - theCurv.Lower()) * dim, ici1 = (ind1 - theCurv.Lower()) * dim,
                        icm = (mid - theCurv.Lower()) * dim;
                    NCollection_LocalArray<double> v1 = new(dim), v2 = new(dim);
                    int i;
                    double mp = 0.0, m1 = 0.0, m2 = 0.0;
                    double p;
                    for (i = 0; i < dim; ++i)
                    {
                        v1[i] = coords[icm + i] - coords[ici + i];
                        m1 += v1[i] * v1[i];
                        v2[i] = coords[ici1 + i] - coords[icm + i];
                        m2 += v2[i] * v2[i];
                    }
                    for (i = 1; i < dim; ++i)
                    {
                        for (jj = 0; jj < i; ++jj)
                        {
                            p = v1[i] * v2[jj] - v1[jj] * v2[i];
                            mp += p * p;
                        }
                    }
                    // mp *= 2.; //P(j,i) = -P(i,j);
                    //

                    if (mp > ASinCoeff2 * m1 * m2) // std::sqrt(mp / m1m2) > aSinCoeff
                    {
                        inds.InsertBefore(theI, mid);
                        return true;
                    }
                }
                else
                {
                    inds.InsertBefore(theI, mid);
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>
    /// Builds the optimal knot sequence (indices of points) of a point set: at least one of the sets
    /// <paramref name="pntsXYZ"/>, <paramref name="pntsU1V1"/>, <paramref name="pntsU2V2"/> (flagged by
    /// <paramref name="approxXYZ"/>, ...) must exist; <paramref name="pars"/> are the expected parameters;
    /// <paramref name="minNbPnts"/> is the minimal number of points per knot interval.
    /// </summary>
    public static void BuildKnots(NCollection_Array1<Point3D> pntsXYZ, NCollection_Array1<Point2D> pntsU1V1,
                                  NCollection_Array1<Point2D> pntsU2V2, math_Vector pars, bool approxXYZ,
                                  bool approxU1V1, bool approxU2V2, int minNbPnts,
                                  List<int> knots)
    {
        NCollection_Sequence<int> aKnots = new();
        int dim = 0;

        // I: Convert input data to the corresponding format.
        if (approxXYZ)
        {
            dim += 3;
        }
        if (approxU1V1)
        {
            dim += 2;
        }
        if (approxU2V2)
        {
            dim += 2;
        }

        if (dim == 0 || pars.Length() < 2)
        {
            knots.Clear();
            return;
        }

        NCollection_LocalArray<double> coords = new(pars.Length() * dim);
        int i, j;
        for (i = pars.Lower(); i <= pars.Upper(); ++i)
        {
            j = (i - pars.Lower()) * dim;
            if (approxXYZ)
            {
                coords[j] = pntsXYZ.Value(i).X;
                ++j;
                coords[j] = pntsXYZ.Value(i).Y;
                ++j;
                coords[j] = pntsXYZ.Value(i).Z;
                ++j;
            }
            if (approxU1V1)
            {
                coords[j] = pntsU1V1.Value(i).X;
                ++j;
                coords[j] = pntsU1V1.Value(i).Y;
                ++j;
            }
            if (approxU2V2)
            {
                coords[j] = pntsU2V2.Value(i).X;
                ++j;
                coords[j] = pntsU2V2.Value(i).Y;
                ++j;
            }
        }

        // II: Build draft knot sequence.
        ComputeKnotInds(coords, dim, pars, aKnots);

        // III: Build output knot sequence.
        FilterKnots(aKnots, minNbPnts, knots);
    }

    //=================================================================================================

    private static double MaxParamRatio(math_Vector pars)
    {
        int i;
        double maxRatio = 0.0;
        //
        for (i = pars.Lower() + 1; i < pars.Upper(); ++i)
        {
            double denom = pars[i] - pars[i - 1];
            if (Math.Abs(denom) < global::Kerf.Engine.Foundation.Precision.Computational())
            {
                continue;
            }
            double rat = (pars[i + 1] - pars[i]) / denom;
            if (rat > 0.0 && rat < 1.0)
            {
                rat = 1.0 / rat;
            }

            maxRatio = Max(maxRatio, rat);
        }
        return maxRatio;
    }

    /// <summary>
    /// The preferable parametrisation of the points <paramref name="fpar"/> to <paramref name="lpar"/> of
    /// <paramref name="wl"/>: centripetal where the curvature varies much, else chord length.
    /// </summary>
    public static ParametrizationType DefineParType(WLine wl, int fpar, int lpar,
                                                           bool approxXYZ, bool approxU1V1, bool approxU2V2)
    {
        if (lpar - fpar == 1)
        {
            return IsoParametric;
        }

        int nbp3d = approxXYZ ? 1 : 0, nbp2d = (approxU1V1 ? 1 : 0) + (approxU2V2 ? 1 : 0);

        TheMultiLineOfWLApprox testLine = new(wl, nbp3d, nbp2d, approxU1V1, approxU2V2, 0.0, 0.0,
                                                       0.0, 0.0, 0.0, 0.0, 0.0, approxU1V1, fpar, lpar);

        NCollection_Array1<Point3D> tabPnt3d = new(1, Max(1, nbp3d));
        NCollection_Array1<Point2D> tabPnt2d = new(1, Max(1, nbp2d));
        NCollection_Array1<Point3D> pntXYZ = new(fpar, lpar);
        NCollection_Array1<Point2D> pntU1V1 = new(fpar, lpar);
        NCollection_Array1<Point2D> pntU2V2 = new(fpar, lpar);

        int i, j;

        for (i = fpar; i <= lpar; ++i)
        {
            if (nbp3d != 0 && nbp2d != 0)
            {
                testLine.Value(i, tabPnt3d, tabPnt2d);
            }
            else if (nbp2d != 0)
            {
                testLine.Value(i, tabPnt2d);
            }
            else if (nbp3d != 0)
            {
                testLine.Value(i, tabPnt3d);
            }
            //
            if (nbp3d > 0)
            {
                pntXYZ[i] = tabPnt3d[1];
            }
            if (nbp2d > 1)
            {
                pntU1V1[i] = tabPnt2d[1];
                pntU2V2[i] = tabPnt2d[2];
            }
            else if (nbp2d > 0)
            {
                if (approxU1V1)
                {
                    pntU1V1[i] = tabPnt2d[1];
                }
                else
                {
                    pntU2V2[i] = tabPnt2d[1];
                }
            }
        }

        int dim = 0;

        if (approxXYZ)
        {
            dim += 3;
        }
        if (approxU1V1)
        {
            dim += 2;
        }
        if (approxU2V2)
        {
            dim += 2;
        }

        int length = lpar - fpar + 1;
        NCollection_LocalArray<double> coords = new(length * dim);
        for (i = fpar; i <= lpar; ++i)
        {
            j = (i - fpar) * dim;
            if (approxXYZ)
            {
                coords[j] = pntXYZ.Value(i).X;
                ++j;
                coords[j] = pntXYZ.Value(i).Y;
                ++j;
                coords[j] = pntXYZ.Value(i).Z;
                ++j;
            }
            if (approxU1V1)
            {
                coords[j] = pntU1V1.Value(i).X;
                ++j;
                coords[j] = pntU1V1.Value(i).Y;
                ++j;
            }
            if (approxU2V2)
            {
                coords[j] = pntU2V2.Value(i).X;
                ++j;
                coords[j] = pntU2V2.Value(i).Y;
                ++j;
            }
        }

        // Analysis of curvature
        const double critRat = 500.0;
        const double critParRat = 100.0;
        math_Vector pars = new(fpar, lpar);
        ParametrizationType parType = ChordLength;
        WLApprox.Parameters(testLine, fpar, lpar, parType, pars);
        NCollection_Array1<double> curv = new(pars.Lower(), pars.Upper());
        BuildCurvature(coords, dim, pars, curv, out double maxCurv);

        if (maxCurv < global::Kerf.Engine.Foundation.Precision.PConfusion() || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(maxCurv))
        {
            // Linear case
            return parType;
        }

        double midCurv = 0.0;
        double eps = Epsilon(1.0);
        j = 0;
        for (i = curv.Lower(); i <= curv.Upper(); ++i)
        {
            if (maxCurv - curv[i] < eps)
            {
                continue;
            }
            ++j;
            midCurv += curv[i];
        }

        if (j > 1)
        {
            midCurv /= j;
        }

        if (midCurv <= eps)
        {
            return parType;
        }

        double rat = maxCurv / midCurv;

        if (rat > critRat)
        {
            if (rat > 5.0 * critRat)
            {
                parType = Centripetal;
            }
            else
            {
                double parRat = MaxParamRatio(pars);
                if (parRat > critParRat)
                {
                    parType = Centripetal;
                }
            }
        }

        return parType;
    }
}
