// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib.hxx
// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib.lxx
// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified RLE 9 Sep 1993
// pmn : modified 28-01-97  : fixed a mistake in LocateParameter (PRO6973)
// pmn : modified 4-11-96   : fixed a mistake in BuildKnots (PRO6124)
// pmn : modified 28-Jun-96 : fixed a mistake in AntiBoorScheme
// xab : modified 15-Jun-95 : fixed a mistake in IsRational
// xab : modified 15-Mar-95 : removed Epsilon comparison in IsRational
//                            added RationalDerivatives.
// xab : 30-Mar-95 : fixed coupling with lti in RationalDerivatives
// xab : 15-Mar-96 : fixed a typo in Eval with extrapolation
// jct : 15-Apr-97 : added TangExtendToConstraint
// jct : 24-Apr-97 : correction on computation of Tbord and NewFlatKnots
//                   in TangExtendToConstraint; Continuity can be equal to 0

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Splines.KnotDistribution;
using static Kerf.Engine.Geometry.Splines.MultDistribution;
using static Kerf.Engine.Geometry.BSplKnotDistribution;

namespace Kerf.Engine.Geometry.Splines;

/// <summary>
/// B-spline curve library: management of knots and multiplicities, multi-dimensional spline methods (the poles are
/// an array of reals and a dimension; global methods modify the whole set of poles, local methods work on a local
/// array of poles that they modify), and 2D and 3D spline curve methods, rational or not, built on the
/// multi-dimensional ones (a 3D rational curve is processed as a curve of dimension 4).
/// </summary>
/// <remarks>
/// A curve is given by its poles, weights, knots, multiplicities, degree and periodicity. The bounds of Poles and
/// Weights should be the same, as should those of Knots and Mults. Weights and multiplicities are optional for some
/// functions: <see cref="NoWeights"/> (null) means a non-rational curve, <see cref="NoMults"/> (null) means the knots
/// are "flat" knots. OCCT's <c>double&amp;</c> array starts are spans (<see cref="ReadOnlySpan{T}"/> where the
/// function only reads, <see cref="Span{T}"/> where it writes).
/// </remarks>
public static partial class BSplCLib
{
    //=======================================================================
    // class : BSplCLib_LocalMatrix
    // purpose: Auxiliary class optimizing creation of matrix buffer for
    //         evaluation of bspline (using stack allocation for main matrix)
    //=======================================================================
    // Note: OCCT places the matrix on a zeroed 27 x 27 stack buffer; the port allocates a math_Matrix (also zeroed)
    private static Matrix BSplCLib_LocalMatrix(int derivativeRequest, int order)
    {
        Standard_OutOfRange_Raise_if(derivativeRequest > global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree()
                                       || order > global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1
                                       || global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() > 25,
                                     "BSplCLib: bspline degree is greater than maximum supported");
        return new Matrix(1, derivativeRequest + 1, 1, order);
    }

    /// <summary>
    /// Searches the position of <paramref name="x"/> in the increasing values <paramref name="array"/> by
    /// bisection. Out of range, returns theArray.Lower()-1 or theArray.Upper()+1. Used to locate a knot.
    /// </summary>
    public static void Hunt(NCollection_Array1<double> array, double x, out int xPos)
    {
        // replaced by simple dichotomy (RLE)
        if (array.First() > x)
        {
            xPos = array.Lower() - 1;
            return;
        }
        else if (array.Last() < x)
        {
            xPos = array.Upper() + 1;
            return;
        }

        xPos = array.Lower();
        if (array.Length() <= 1)
        {
            return;
        }

        int hi = array.Upper();
        while (hi - xPos != 1)
        {
            int mid = (hi + xPos) / 2;
            if (array.Value(mid) < x)
            {
                xPos = mid;
            }
            else
            {
                hi = mid;
            }
        }
    }

    /// <summary>The index of the knot of the first parameter (first knot whose cumulated multiplicity exceeds Degree).</summary>
    public static int FirstUKnotIndex(int degree, NCollection_Array1<int> mults)
    {
        int index = mults.Lower();
        int sigmaMult = mults[index];

        while (sigmaMult <= degree)
        {
            index++;
            sigmaMult += mults[index];
        }
        return index;
    }

    /// <summary>The index of the knot of the last parameter.</summary>
    public static int LastUKnotIndex(int degree, NCollection_Array1<int> mults)
    {
        int index = mults.Upper();
        int sigmaMult = mults[index];

        while (sigmaMult <= degree)
        {
            index--;
            sigmaMult += mults.Value(index);
        }
        return index;
    }

    /// <summary>Computes the index of the knot <paramref name="Index"/> in the flat knot sequence.</summary>
    public static int FlatIndex(int degree, int Index, NCollection_Array1<int> mults, bool periodic)
    {
        int i, index = Index;
        int mLower = mults.Lower();
        ReadOnlySpan<int> pmu = mults.AsSpan(); // pmu[i - MLower]

        for (i = mLower + 1; i <= Index; i++)
        {
            index += pmu[i - mLower] - 1;
        }
        if (periodic)
        {
            index += degree;
        }
        else
        {
            index += pmu[mLower - mLower] - 1;
        }
        return index;
    }

    /// <summary>
    /// Locates the parametric value <paramref name="u"/> in the knot sequence between the knots FromK1 and ToK2:
    /// Knots(KnotIndex) &lt;= U &lt; Knots(KnotIndex + 1); for a periodic curve, U is first brought into the
    /// period (NewU).
    /// </summary>
    public static void LocateParameter(int degree, NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                                       double u, bool isPeriodic, int fromK1, int toK2, out int knotIndex,
                                       out double newU)
    {
        double uf = 0, ul = 1;
        if (isPeriodic)
        {
            uf = knots[knots.Lower()];
            ul = knots[knots.Upper()];
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(knots, u, isPeriodic, fromK1, toK2, out knotIndex, out newU, uf, ul);
    }

    /// <summary>
    /// Locates the parametric value <paramref name="u"/> in the flat knot sequence between the knots FromK1 and
    /// ToK2; for a periodic curve the period runs from Knots(Lower + Degree) to Knots(Upper - Degree).
    /// </summary>
    public static void LocateParameter(int degree, NCollection_Array1<double> knots, double u, bool isPeriodic,
                                       int fromK1, int toK2, out int knotIndex, out double newU)
    {
        if (isPeriodic)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(knots,
                                     u,
                                     isPeriodic,
                                     fromK1,
                                     toK2,
                                     out knotIndex,
                                     out newU,
                                     knots[knots.Lower() + degree],
                                     knots[knots.Upper() - degree]);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(knots, u, isPeriodic, fromK1, toK2, out knotIndex, out newU, 0.0, 1.0);
        }
    }

    private static void LocateParameter(NCollection_Array1<double> Knots, double u, bool isPeriodic, int fromK1,
                                        int toK2, out int knotIndex, out double newU, double uFirst, double uLast)
    {
        /*
        Let Knots are distributed as follows (the array is sorted in ascending order):

            K1, K1,..., K1, K1, K2, K2,..., K2, K2,..., Kn, Kn,..., Kn
                 M1 times             M2 times             Mn times

        NbKnots = sum(M1+M2+...+Mn)
        If U <= K1 then KnotIndex should be equal to M1.
        If U >= Kn then KnotIndex should be equal to NbKnots-Mn-1.
        If Ki <= U < K(i+1) then KnotIndex should be equal to sum (M1+M2+...+Mi).
        */

        int first, last;
        if (fromK1 < toK2)
        {
            first = fromK1;
            last = toK2;
        }
        else
        {
            first = toK2;
            last = fromK1;
        }
        int last1 = last - 1;
        newU = u;
        if (isPeriodic && (newU < uFirst || newU > uLast))
        {
            newU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(newU, uFirst, uLast);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(Knots, newU, out knotIndex);

        double val;
        int kLower = Knots.Lower(), kUpper = Knots.Upper();

        double eps = Epsilon(Min(Math.Abs(Knots[kUpper]), Math.Abs(u)));

        ReadOnlySpan<double> knots = Knots.AsSpan(); // knots[i - KLower]
        if (knotIndex < Knots.Upper())
        {
            val = newU - knots[knotIndex + 1 - kLower];
            if (val < 0)
            {
                val = -val;
            }
            // <= to be coherent with Segment where Eps corresponds to a bit of error.
            if (val <= eps)
            {
                knotIndex++;
            }
        }
        if (knotIndex < first)
        {
            knotIndex = first;
        }
        if (knotIndex > last1)
        {
            knotIndex = last1;
        }

        if (knotIndex != last1)
        {
            double k1 = knots[knotIndex - kLower];
            double k2 = knots[knotIndex + 1 - kLower];
            val = k2 - k1;
            if (val < 0)
            {
                val = -val;
            }

            while (val <= eps)
            {
                knotIndex++;

                if (knotIndex >= Knots.Upper())
                {
                    break;
                }

                k1 = k2;
                k2 = knots[knotIndex + 1 - kLower];
                val = k2 - k1;
                if (val < 0)
                {
                    val = -val;
                }
            }
        }
    }

    /// <summary>
    /// Locates <paramref name="u"/> in the knots, unless <paramref name="index"/> already designates a knot of the
    /// curve's range (then NewU = U). Without multiplicities the knots are flat knots.
    /// </summary>
    public static void LocateParameter(int degree, NCollection_Array1<double> knots, NCollection_Array1<int>? mults,
                                       double u, bool periodic, ref int index, out double newU)
    {
        int knotIndex = index;
        int first, last;
        if (mults != null)
        {
            if (periodic)
            {
                first = knots.Lower();
                last = knots.Upper();
            }
            else
            {
                first = FirstUKnotIndex(degree, mults);
                last = LastUKnotIndex(degree, mults);
            }
        }
        else
        {
            first = knots.Lower() + degree;
            last = knots.Upper() - degree;
        }
        if (knotIndex < first || knotIndex > last)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(knots,
                                     u,
                                     periodic,
                                     first,
                                     last,
                                     out knotIndex,
                                     out newU,
                                     knots[first],
                                     knots[last]);
        }
        else
        {
            newU = u;
        }
        index = knotIndex;
    }

    /// <summary>The greatest multiplicity in Mults between FromK1 and ToK2.</summary>
    public static int MaxKnotMult(NCollection_Array1<int> mults, int fromK1, int toK2)
    {
        int maxMult = mults[fromK1];
        for (int index = fromK1 + 1; index <= toK2; ++index)
        {
            maxMult = Max(maxMult, mults[index]);
        }
        return maxMult;
    }

    /// <summary>The smallest multiplicity in Mults between FromK1 and ToK2.</summary>
    public static int MinKnotMult(NCollection_Array1<int> mults, int fromK1, int toK2)
    {
        int minMult = mults[fromK1];
        for (int index = fromK1 + 1; index <= toK2; ++index)
        {
            minMult = Min(minMult, mults[index]);
        }
        return minMult;
    }

    /// <summary>
    /// The number of poles of the curve, or 0 if the multiplicities are inconsistent (a multiplicity &lt;= 0 or
    /// greater than Degree inside, ends greater than Degree + 1, or unequal ends of a periodic curve).
    /// </summary>
    public static int NbPoles(int degree, bool periodic, NCollection_Array1<int> mults)
    {
        int f = mults.Lower();
        int l = mults.Upper();
        ReadOnlySpan<int> pmu = mults.AsSpan();
        int mf = pmu[0];
        int ml = pmu[l - f];
        if (mf <= 0)
        {
            return 0;
        }
        if (ml <= 0)
        {
            return 0;
        }
        int sigma;
        if (periodic)
        {
            if (mf > degree)
            {
                return 0;
            }
            if (ml > degree)
            {
                return 0;
            }
            if (mf != ml)
            {
                return 0;
            }
            sigma = mf;
        }
        else
        {
            int deg1 = degree + 1;
            if (mf > deg1)
            {
                return 0;
            }
            if (ml > deg1)
            {
                return 0;
            }
            sigma = mf + ml - deg1;
        }

        for (int i = 1; i < l - f; i++)
        {
            if (pmu[i] <= 0)
            {
                return 0;
            }
            if (pmu[i] > degree)
            {
                return 0;
            }
            sigma += pmu[i];
        }
        return sigma;
    }

    /// <summary>The length of the flat knot sequence (the sum of the multiplicities, plus the periodic extension).</summary>
    public static int KnotSequenceLength(NCollection_Array1<int> mults, int degree, bool periodic)
    {
        int i, l = 0;
        int mLower = mults.Lower();
        int mUpper = mults.Upper();
        ReadOnlySpan<int> pmu = mults.AsSpan(); // pmu[i - MLower]

        for (i = mLower; i <= mUpper; i++)
        {
            l += pmu[i - mLower];
        }
        if (periodic)
        {
            l += 2 * (degree + 1 - pmu[0]);
        }
        return l;
    }

    /// <summary>Computes the flat knot sequence (each knot repeated by its multiplicity) into KnotSeq.</summary>
    public static void KnotSequence(NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                                    NCollection_Array1<double> knotSeq, bool periodic = false)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(knots, mults, 0, periodic, knotSeq);
    }

    /// <summary>Computes the flat knot sequence into KnotSeq; for a periodic curve, extended by the period on both sides.</summary>
    public static void KnotSequence(NCollection_Array1<double> knots, NCollection_Array1<int> mults, int degree,
                                    bool periodic, NCollection_Array1<double> knotSeq)
    {
        double k;
        int mult;
        int mLower = mults.Lower();
        ReadOnlySpan<int> pmu = mults.AsSpan(); // pmu[i - MLower]
        int kLower = knots.Lower();
        int kUpper = knots.Upper();
        ReadOnlySpan<double> pkn = knots.AsSpan(); // pkn[i - KLower]
        int m1 = degree + 1 - pmu[0]; // for periodic
        int i, j, index = periodic ? m1 + 1 : 1;

        for (i = kLower; i <= kUpper; i++)
        {
            mult = pmu[i - mLower];
            k = pkn[i - kLower];

            for (j = 1; j <= mult; j++)
            {
                knotSeq[index] = k;
                index++;
            }
        }
        if (periodic)
        {
            double period = pkn[kUpper - kLower] - pkn[0];
            int m;
            m = 1;
            j = kUpper - 1;

            for (i = m1; i >= 1; i--)
            {
                knotSeq[i] = pkn[j - kLower] - period;
                m++;
                if (m > pmu[j - mLower])
                {
                    j--;
                    m = 1;
                }
            }
            m = 1;
            j = kLower + 1;

            for (i = index; i <= knotSeq.Upper(); i++)
            {
                knotSeq[i] = pkn[j - kLower] + period;
                m++;
                if (m > pmu[j - mLower])
                {
                    j++;
                    m = 1;
                }
            }
        }
    }

    /// <summary>The number of distinct knots in a flat knot sequence (indexed from 1).</summary>
    public static int KnotsLength(NCollection_Array1<double> knotSeq, bool periodic = false)
    {
        NCollection_Array1<double> seqKnots = knotSeq;
        int sizeMult = 1;
        double val = seqKnots[1];
        for (int jj = 2; jj <= seqKnots.Length(); jj++)
        {
            // test on strict equality on nodes
            if (seqKnots[jj] != val)
            {
                val = seqKnots[jj];
                sizeMult++;
            }
        }
        return sizeMult;
    }

    /// <summary>Computes the knots and multiplicities (indexed from 1) of a flat knot sequence (indexed from 1).</summary>
    public static void Knots(NCollection_Array1<double> knotSeq, NCollection_Array1<double> Knots,
                             NCollection_Array1<int> mults, bool periodic = false)
    {
        NCollection_Array1<double> seqKnots = knotSeq;
        NCollection_Array1<double> knots = Knots;
        NCollection_Array1<int> mult = mults;
        double val = seqKnots[1];
        int kk = 1;
        knots[kk] = val;
        mult[kk] = 1;

        for (int jj = 2; jj <= seqKnots.Length(); jj++)
        {
            // test on strict equality on nodes
            if (seqKnots[jj] != val)
            {
                val = seqKnots[jj];
                kk++;
                knots[kk] = val;
                mult[kk] = 1;
            }
            else
            {
                mult[kk]++;
            }
        }
    }

    /// <summary>Analyses the distribution of the knots between FromK1 and ToK2: uniform or not.</summary>
    public static KnotDistribution KnotForm(NCollection_Array1<double> knots, int fromK1, int toK2)
    {
        if (fromK1 + 1 > knots.Upper())
        {
            return KnotDistribution.Uniform;
        }

        double ui = Math.Abs(knots[fromK1]);
        double uj = Math.Abs(knots[fromK1 + 1]);
        double du0 = Math.Abs(uj - ui);
        double eps = Epsilon(ui) + Epsilon(uj) + Epsilon(du0);

        for (int i = fromK1 + 1; i < toK2; i++)
        {
            ui = Math.Abs(knots[i]);
            uj = Math.Abs(knots[i + 1]);
            double du1 = Math.Abs(uj - ui);

            if (Math.Abs(du1 - du0) > eps)
            {
                return KnotDistribution.NonUniform;
            }

            du0 = du1;
            eps = Epsilon(ui) + Epsilon(uj) + Epsilon(du0);
        }

        return KnotDistribution.Uniform;
    }

    /// <summary>
    /// Analyses the distribution of the multiplicities between FromK1 and ToK2: constant, quasi-constant
    /// (constant except the first and last) or non-constant.
    /// </summary>
    public static MultDistribution MultForm(NCollection_Array1<int> mults, int fromK1, int toK2)
    {
        int first = Min(fromK1, toK2), last = Max(fromK1, toK2);

        if (first + 1 > mults.Upper())
        {
            return Constant;
        }

        int firstMult = mults[first];
        MultDistribution form = Constant;
        int mult = mults[first + 1];

        for (int i = first + 1; i <= last && form != NonConstant; i++)
        {
            if (i == first + 1)
            {
                if (mult != firstMult)
                {
                    form = QuasiConstant;
                }
            }
            else if (i == last)
            {
                if (form == QuasiConstant)
                {
                    if (firstMult != mults[i])
                    {
                        form = NonConstant;
                    }
                }
                else
                {
                    if (mult != mults[i])
                    {
                        form = NonConstant;
                    }
                }
            }
            else
            {
                if (mult != mults[i])
                {
                    form = NonConstant;
                }
                mult = mults[i];
            }
        }

        return form;
    }

    /// <summary>
    /// Analyses the knots and multiplicities of a curve: its knot distribution (uniform, quasi-uniform,
    /// piecewise Bezier or non-uniform) and the greatest multiplicity of its interior knots.
    /// </summary>
    public static void KnotAnalysis(int degree, bool periodic, NCollection_Array1<double> cKnots,
                                    NCollection_Array1<int> cMults, out BSplKnotDistribution knotForm,
                                    out int maxKnotMult)
    {
        knotForm = BSplKnotDistribution.NonUniform;

        KnotDistribution kSet = global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotForm(cKnots, 1, cKnots.Length());

        if (kSet == KnotDistribution.Uniform)
        {
            MultDistribution mSet = global::Kerf.Engine.Geometry.Splines.BSplCLib.MultForm(cMults, 1, cMults.Length());
            switch (mSet)
            {
                case NonConstant:
                    break;
                case Constant:
                    if (cKnots.Length() == 2)
                    {
                        knotForm = PiecewiseBezier;
                    }
                    else
                    {
                        if (cMults[1] == 1)
                        {
                            knotForm = BSplKnotDistribution.Uniform;
                        }
                    }
                    break;
                case QuasiConstant:
                    if (cMults[1] == degree + 1)
                    {
                        double m = cMults[2];
                        if (m == degree)
                        {
                            knotForm = PiecewiseBezier;
                        }
                        else if (m == 1)
                        {
                            knotForm = QuasiUniform;
                        }
                    }
                    break;
            }
        }

        int firstKM = periodic ? cKnots.Lower() : global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(degree, cMults);
        int lastKM = periodic ? cKnots.Upper() : global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(degree, cMults);
        maxKnotMult = 0;
        if (lastKM - firstKM != 1)
        {
            int multi;
            for (int i = firstKM + 1; i < lastKM; i++)
            {
                multi = cMults[i];
                maxKnotMult = Max(maxKnotMult, multi);
            }
        }
    }

    /// <summary>Reparametrises the knots to the range [U1, U2] (in either order), keeping their relative spacing.</summary>
    public static void Reparametrize(double u1, double u2, NCollection_Array1<double> knots)
    {
        int lower = knots.Lower();
        int upper = knots.Upper();
        double uFirst = Min(u1, u2);
        double uLast = Max(u1, u2);
        double newLength = uLast - uFirst;
        KnotDistribution kSet = global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotForm(knots, lower, upper);
        if (kSet == KnotDistribution.Uniform)
        {
            double du = newLength / (upper - lower);
            knots[lower] = uFirst;

            for (int i = lower + 1; i <= upper; i++)
            {
                knots[i] = knots[i - 1] + du;
            }
        }
        else
        {
            double k2;
            double ratio;
            double k1 = knots[lower];
            double length = knots[upper] - knots[lower];
            knots[lower] = uFirst;

            for (int i = lower + 1; i <= upper; i++)
            {
                k2 = knots[i];
                ratio = (k2 - k1) / length;
                knots[i] = knots[i - 1] + (newLength * ratio);

                // for CheckCurveData
                double eps = Epsilon(Math.Abs(knots[i - 1]));
                if (knots[i] - knots[i - 1] <= eps)
                {
                    knots[i] = NextAfter(knots[i - 1] + eps, RealLast());
                }

                k1 = k2;
            }
        }
    }

    /// <summary>Reverses the orientation of the knots (the parametrisation is reversed, the range kept).</summary>
    public static void Reverse(NCollection_Array1<double> knots)
    {
        int first = knots.Lower();
        int last = knots.Upper();
        double kfirst = knots[first];
        double klast = knots[last];
        double tfirst = kfirst;
        double tlast = klast;
        first++;
        last--;

        while (first <= last)
        {
            tfirst += klast - knots[last];
            tlast -= knots[first] - kfirst;
            kfirst = knots[first];
            klast = knots[last];
            knots[first] = tfirst;
            knots[last] = tlast;
            first++;
            last--;
        }
    }

    /// <summary>Reverses the order of the multiplicities.</summary>
    public static void Reverse(NCollection_Array1<int> mults)
    {
        mults.AsSpan().Reverse();
    }

    /// <summary>Reverses the array of weights, <paramref name="last"/> being the index of the new first element.</summary>
    public static void Reverse(NCollection_Array1<double> weights, int last)
    {
        CurveComputation.BSplCLib_Reverse(weights, last);
    }

    /// <summary>
    /// Returns false if all the weights between I1 and I2 are equal (compared exactly; Epsilon is unused);
    /// indices wrap around the array.
    /// </summary>
    public static bool IsRational(NCollection_Array1<double> weights, int i1, int i2, double epsilon = 0.0)
    {
        int i, f = weights.Lower(), l = weights.Length();
        int i3 = i2 - f;
        ReadOnlySpan<double> wg = weights.AsSpan(); // WG[f + x] -> WG[x]

        for (i = i1 - f; i < i3; i++)
        {
            if (wg[(i % l)] != wg[((i + 1) % l)])
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>The maximum degree of the B-splines: 25.</summary>
    public static int MaxDegree() => 25;

    /// <summary>
    /// Performs the de Boor algorithm at <paramref name="u"/> on the local poles (Degree + 1 poles of dimension
    /// Dimension) with the 2 * Degree local knots; the value is left in the first pole.
    /// </summary>
    public static void Eval(double u, int degree, ReadOnlySpan<double> Knots, int dimension, Span<double> Poles)
    {
        int step, i, dms, dm1, dpi, sti;
        double x, y;
        int poles;
        ReadOnlySpan<double> knots = Knots;
        dm1 = dms = degree;
        dm1--;
        dms++;
        switch (dimension)
        {

            case 1:
                {

                    for (step = -1; step < dm1; step++)
                    {
                        dms--;
                        poles = 0;
                        dpi = dm1;
                        sti = step;

                        for (i = 0; i < dms; i++)
                        {
                            dpi++;
                            sti++;
                            x = (knots[dpi] - u) / (knots[dpi] - knots[sti]);
                            y = 1 - x;
                            Poles[poles + 0] *= x;
                            Poles[poles + 0] += y * Poles[poles + 1];
                            poles += 1;
                        }
                    }
                    break;
                }
            case 2:
                {

                    for (step = -1; step < dm1; step++)
                    {
                        dms--;
                        poles = 0;
                        dpi = dm1;
                        sti = step;

                        for (i = 0; i < dms; i++)
                        {
                            dpi++;
                            sti++;
                            x = (knots[dpi] - u) / (knots[dpi] - knots[sti]);
                            y = 1 - x;
                            Poles[poles + 0] *= x;
                            Poles[poles + 0] += y * Poles[poles + 2];
                            Poles[poles + 1] *= x;
                            Poles[poles + 1] += y * Poles[poles + 3];
                            poles += 2;
                        }
                    }
                    break;
                }
            case 3:
                {

                    for (step = -1; step < dm1; step++)
                    {
                        dms--;
                        poles = 0;
                        dpi = dm1;
                        sti = step;

                        for (i = 0; i < dms; i++)
                        {
                            dpi++;
                            sti++;
                            x = (knots[dpi] - u) / (knots[dpi] - knots[sti]);
                            y = 1 - x;
                            Poles[poles + 0] *= x;
                            Poles[poles + 0] += y * Poles[poles + 3];
                            Poles[poles + 1] *= x;
                            Poles[poles + 1] += y * Poles[poles + 4];
                            Poles[poles + 2] *= x;
                            Poles[poles + 2] += y * Poles[poles + 5];
                            poles += 3;
                        }
                    }
                    break;
                }
            case 4:
                {

                    for (step = -1; step < dm1; step++)
                    {
                        dms--;
                        poles = 0;
                        dpi = dm1;
                        sti = step;

                        for (i = 0; i < dms; i++)
                        {
                            dpi++;
                            sti++;
                            x = (knots[dpi] - u) / (knots[dpi] - knots[sti]);
                            y = 1 - x;
                            Poles[poles + 0] *= x;
                            Poles[poles + 0] += y * Poles[poles + 4];
                            Poles[poles + 1] *= x;
                            Poles[poles + 1] += y * Poles[poles + 5];
                            Poles[poles + 2] *= x;
                            Poles[poles + 2] += y * Poles[poles + 6];
                            Poles[poles + 3] *= x;
                            Poles[poles + 3] += y * Poles[poles + 7];
                            poles += 4;
                        }
                    }
                    break;
                }
            default:
                {
                    int k;

                    for (step = -1; step < dm1; step++)
                    {
                        dms--;
                        poles = 0;
                        dpi = dm1;
                        sti = step;

                        for (i = 0; i < dms; i++)
                        {
                            dpi++;
                            sti++;
                            x = (knots[dpi] - u) / (knots[dpi] - knots[sti]);
                            y = 1 - x;

                            for (k = 0; k < dimension; k++)
                            {
                                Poles[poles + k] *= x;
                                Poles[poles + k] += y * Poles[poles + k + dimension];
                            }
                            poles += dimension;
                        }
                    }
                    break;
                }
        }
    }

    /// <summary>
    /// Performs the de Boor scheme at <paramref name="u"/> to depth <paramref name="depth"/> on poles stored
    /// alternately in even positions (see <see cref="BuildBoor"/>), with the local knots of
    /// <see cref="BuildKnots"/>; the intermediate values fill the odd positions. Used for knot insertion.
    /// </summary>
    public static void BoorScheme(double u, int degree, ReadOnlySpan<double> Knots, int dimension, Span<double> poles,
                                  int depth, int length)
    {
        //
        // Compute the values
        //
        //  P(i,j) (U).
        //
        // for i = 0 to Depth,
        // j = 0 to Length - i
        //
        //  The Boor scheme is :
        //
        //  P(0,j) = Pole(j)
        //  P(i,j) = x * P(i-1,j) + (1-x) * P(i-1,j+1)
        //
        //    where x = (knot(i+j+Degree) - U) / (knot(i+j+Degree) - knot(i+j))
        //
        //
        //  The values are stored in the array Poles
        //  They are alternatively written if the odd and even positions.
        //
        //  The successives contents of the array are
        //   ***** means uninitialised, l = Degree + Length
        //
        //  P(0,0) ****** P(0,1) ...... P(0,l-1) ******** P(0,l)
        //  P(0,0) P(1,0) P(0,1) ...... P(0,l-1) P(1,l-1) P(0,l)
        //  P(0,0) P(1,0) P(2,0) ...... P(2,l-1) P(1,l-1) P(0,l)
        //

        int i, k, step;
        ReadOnlySpan<double> knots = Knots;
        int pole, firstpole = -2 * dimension; // indexes in Poles
        // the steps of recursion

        for (step = 0; step < depth; step++)
        {
            firstpole += dimension;
            pole = firstpole;
            // compute the new row of poles

            for (i = step; i < length; i++)
            {
                pole += 2 * dimension;
                // coefficient
                double x = (knots[i + degree - step] - u) / (knots[i + degree - step] - knots[i]);
                double y = 1.0 - x;
                // Value
                // P(i,j) = X * P(i-1,j) + (1-X) * P(i-1,j+1)

                for (k = 0; k < dimension; k++)
                {
                    poles[pole + k] = x * poles[pole + k - dimension] + y * poles[pole + k + dimension];
                }
            }
        }
    }

    /// <summary>
    /// The de Boor scheme reverted, for knot removal: computes the poles before insertion from the poles after
    /// insertion. Returns false if the removal is not possible within <paramref name="tolerance"/>.
    /// </summary>
    public static bool AntiBoorScheme(double u, int degree, ReadOnlySpan<double> Knots, int dimension,
                                      Span<double> poles, int depth, int length, double tolerance)
    {
        // do the Boor scheme reverted.

        int i, k, step, half_length;
        ReadOnlySpan<double> knots = Knots;
        double z, x, y;
        int pole, firstpole = (depth - 1) * dimension; // indexes in Poles

        // Test the special case length = 1
        // only verification of the central point

        if (length == 1)
        {
            x = (knots[degree] - u) / (knots[degree] - knots[0]);
            y = 1.0 - x;

            for (k = 0; k < dimension; k++)
            {
                z = x * poles[firstpole + k] + y * poles[firstpole + k + 2 * dimension];
                if (Math.Abs(z - poles[firstpole + k + dimension]) > tolerance)
                {
                    return false;
                }
            }
            return true;
        }

        // General case
        // the steps of recursion

        for (step = depth - 1; step >= 0; step--)
        {
            firstpole -= dimension;
            pole = firstpole;

            // first step from left to right

            for (i = step; i < length - 1; i++)
            {
                pole += 2 * dimension;

                x = (knots[i + degree - step] - u) / (knots[i + degree - step] - knots[i]);
                y = 1.0 - x;

                for (k = 0; k < dimension; k++)
                {
                    poles[pole + k + dimension] = (poles[pole + k] - x * poles[pole + k - dimension]) / y;
                }
            }

            // second step from right to left
            pole += 4 * dimension;
            half_length = (length - 1 + step) / 2;
            //
            // only do half of the way from right to left
            // otherwise it start degenerating because of
            // overflows
            //

            for (i = length - 1; i > half_length; i--)
            {
                pole -= 2 * dimension;

                // coefficient
                x = (knots[i + degree - step] - u) / (knots[i + degree - step] - knots[i]);
                y = 1.0 - x;

                for (k = 0; k < dimension; k++)
                {
                    z = (poles[pole + k] - y * poles[pole + k + dimension]) / x;
                    if (Math.Abs(z - poles[pole + k - dimension]) > tolerance)
                    {
                        return false;
                    }
                    poles[pole + k - dimension] += z;
                    poles[pole + k - dimension] /= 2.0;
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Computes the poles of the derivatives up to <paramref name="order"/> of the local B-spline, in place
    /// (Length poles of dimension Dimension).
    /// </summary>
    public static void Derivative(int degree, ReadOnlySpan<double> knots, int dimension, int length, int order,
                                  Span<double> poles)
    {
        int i, k, step, span = degree;
        ReadOnlySpan<double> knot = knots;

        for (step = 1; step <= order; step++)
        {
            int pole = 0;

            for (i = step; i < length; i++)
            {
                double coef = -span / (knot[i + span] - knot[i]);

                for (k = 0; k < dimension; k++)
                {
                    poles[pole + k] -= poles[pole + k + dimension];
                    poles[pole + k] *= coef;
                }
                pole += dimension;
            }
            span--;
        }
    }

    /// <summary>
    /// Performs the Bohm algorithm at <paramref name="u"/>: computes the value and the derivatives up to
    /// <paramref name="n"/> of the local B-spline, in place in <paramref name="poles"/> (value, first derivative,
    /// ..., each Dimension values). Derivatives above Degree are not computed.
    /// </summary>
    public static void Bohm(double u, int degree, int n, ReadOnlySpan<double> knots, int dimension, Span<double> poles)
    {
        // First phase independent of U, compute the poles of the derivatives
        int i, j, iDim, min, dmi, dDmi, jDmi, degm1;
        ReadOnlySpan<double> knot = knots;
        Span<double> p = poles;
        double coef;
        int pole, tbis, psDD, psDDmDim; // indexes in Poles
        const int psav = 0;
        if (n < degree)
        {
            min = n;
        }
        else
        {
            min = degree;
        }
        degm1 = degree - 1;
        dDmi = (degree << 1) + 1;
        switch (dimension)
        {
            case 1:
                {
                    psDD = psav + degree;
                    psDDmDim = psDD - 1;

                    for (i = 0; i < degree; i++)
                    {
                        dDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = dDmi;

                        for (j = degm1; j >= i; j--)
                        {
                            jDmi--;
                            p[pole] -= p[tbis];
                            p[pole] = (knot[jDmi] == knot[j]) ? 0.0 : p[pole] / (knot[jDmi] - knot[j]);
                            pole--;
                            tbis--;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -1;

                    for (i = 0; i < degree; i++)
                    {
                        iDim += 1;
                        pole = psav + iDim;
                        tbis = pole + 1;
                        coef = u - knot[i];

                        for (j = i; j >= 0; j--)
                        {
                            p[pole] += coef * p[tbis];
                            pole--;
                            tbis--;
                        }
                    }
                    // multiply by the degrees
                    coef = degree;
                    dmi = degree;
                    pole = psav + 1;

                    for (i = 1; i <= min; i++)
                    {
                        p[pole] *= coef;
                        pole++;
                        dmi--;
                        coef *= dmi;
                    }
                    break;
                }
            case 2:
                {
                    psDD = psav + (degree << 1);
                    psDDmDim = psDD - 2;

                    for (i = 0; i < degree; i++)
                    {
                        dDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = dDmi;

                        for (j = degm1; j >= i; j--)
                        {
                            jDmi--;
                            coef = (knot[jDmi] == knot[j]) ? 0.0 : 1.0 / (knot[jDmi] - knot[j]);
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole++;
                            tbis++;
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole -= 3;
                            tbis -= 3;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -2;

                    for (i = 0; i < degree; i++)
                    {
                        iDim += 2;
                        pole = psav + iDim;
                        tbis = pole + 2;
                        coef = u - knot[i];

                        for (j = i; j >= 0; j--)
                        {
                            p[pole] += coef * p[tbis];
                            pole++;
                            tbis++;
                            p[pole] += coef * p[tbis];
                            pole -= 3;
                            tbis -= 3;
                        }
                    }
                    // multiply by the degrees
                    coef = degree;
                    dmi = degree;
                    pole = psav + 2;

                    for (i = 1; i <= min; i++)
                    {
                        p[pole] *= coef;
                        pole++;
                        p[pole] *= coef;
                        pole++;
                        dmi--;
                        coef *= dmi;
                    }
                    break;
                }
            case 3:
                {
                    psDD = psav + (degree << 1) + degree;
                    psDDmDim = psDD - 3;

                    for (i = 0; i < degree; i++)
                    {
                        dDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = dDmi;

                        for (j = degm1; j >= i; j--)
                        {
                            jDmi--;
                            coef = (knot[jDmi] == knot[j]) ? 0.0 : 1.0 / (knot[jDmi] - knot[j]);
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole++;
                            tbis++;
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole++;
                            tbis++;
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole -= 5;
                            tbis -= 5;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -3;

                    for (i = 0; i < degree; i++)
                    {
                        iDim += 3;
                        pole = psav + iDim;
                        tbis = pole + 3;
                        coef = u - knot[i];

                        for (j = i; j >= 0; j--)
                        {
                            p[pole] += coef * p[tbis];
                            pole++;
                            tbis++;
                            p[pole] += coef * p[tbis];
                            pole++;
                            tbis++;
                            p[pole] += coef * p[tbis];
                            pole -= 5;
                            tbis -= 5;
                        }
                    }
                    // multiply by the degrees
                    coef = degree;
                    dmi = degree;
                    pole = psav + 3;

                    for (i = 1; i <= min; i++)
                    {
                        p[pole] *= coef;
                        pole++;
                        p[pole] *= coef;
                        pole++;
                        p[pole] *= coef;
                        pole++;
                        dmi--;
                        coef *= dmi;
                    }
                    break;
                }
            case 4:
                {
                    psDD = psav + (degree << 2);
                    psDDmDim = psDD - 4;

                    for (i = 0; i < degree; i++)
                    {
                        dDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = dDmi;

                        for (j = degm1; j >= i; j--)
                        {
                            jDmi--;
                            coef = (knot[jDmi] == knot[j]) ? 0.0 : 1.0 / (knot[jDmi] - knot[j]);
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole++;
                            tbis++;
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole++;
                            tbis++;
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole++;
                            tbis++;
                            p[pole] -= p[tbis];
                            p[pole] *= coef;
                            pole -= 7;
                            tbis -= 7;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -4;

                    for (i = 0; i < degree; i++)
                    {
                        iDim += 4;
                        pole = psav + iDim;
                        tbis = pole + 4;
                        coef = u - knot[i];

                        for (j = i; j >= 0; j--)
                        {
                            p[pole] += coef * p[tbis];
                            pole++;
                            tbis++;
                            p[pole] += coef * p[tbis];
                            pole++;
                            tbis++;
                            p[pole] += coef * p[tbis];
                            pole++;
                            tbis++;
                            p[pole] += coef * p[tbis];
                            pole -= 7;
                            tbis -= 7;
                        }
                    }
                    // multiply by the degrees
                    coef = degree;
                    dmi = degree;
                    pole = psav + 4;

                    for (i = 1; i <= min; i++)
                    {
                        p[pole] *= coef;
                        pole++;
                        p[pole] *= coef;
                        pole++;
                        p[pole] *= coef;
                        pole++;
                        p[pole] *= coef;
                        pole++;
                        dmi--;
                        coef *= dmi;
                    }
                    break;
                }
            default:
                {
                    int k;
                    int dim2 = dimension << 1;
                    psDD = psav + degree * dimension;
                    psDDmDim = psDD - dimension;

                    for (i = 0; i < degree; i++)
                    {
                        dDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = dDmi;

                        for (j = degm1; j >= i; j--)
                        {
                            jDmi--;
                            coef = (knot[jDmi] == knot[j]) ? 0.0 : 1.0 / (knot[jDmi] - knot[j]);

                            for (k = 0; k < dimension; k++)
                            {
                                p[pole] -= p[tbis];
                                p[pole] *= coef;
                                pole++;
                                tbis++;
                            }
                            pole -= dim2;
                            tbis -= dim2;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -dimension;

                    for (i = 0; i < degree; i++)
                    {
                        iDim += dimension;
                        pole = psav + iDim;
                        tbis = pole + dimension;
                        coef = u - knot[i];

                        for (j = i; j >= 0; j--)
                        {

                            for (k = 0; k < dimension; k++)
                            {
                                p[pole] += coef * p[tbis];
                                pole++;
                                tbis++;
                            }
                            pole -= dim2;
                            tbis -= dim2;
                        }
                    }
                    // multiply by the degrees
                    coef = degree;
                    dmi = degree;
                    pole = psav + dimension;

                    for (i = 1; i <= min; i++)
                    {

                        for (k = 0; k < dimension; k++)
                        {
                            p[pole] *= coef;
                            pole++;
                        }
                        dmi--;
                        coef *= dmi;
                    }
                    break;
                }
        }
    }

    /// <summary>No weights: the argument for non-rational curves (null).</summary>
    public static NCollection_Array1<double>? NoWeights() => null;

    /// <summary>No multiplicities: the argument for flat knots (null).</summary>
    public static NCollection_Array1<int>? NoMults() => null;

    /// <summary>The size of the shared array of unit weights, <see cref="UnitWeightsData"/>.</summary>
    public static int MaxUnitWeightsSize() => 2049;

    /// <summary>
    /// Stores in <paramref name="lk"/> the 2 * Degree knots around the knot <paramref name="index"/> needed for
    /// the local evaluation of the span. Without <paramref name="mults"/> the knots are flat knots.
    /// </summary>
    public static void BuildKnots(int degree, int index, bool periodic, NCollection_Array1<double> knots,
                                  NCollection_Array1<int>? mults, Span<double> lk)
    {
        int kLower = knots.Lower();
        ReadOnlySpan<double> pkn = knots.AsSpan(); // pkn[i - KLower]
        Span<double> knot = lk;
        if (mults == null)
        {
            switch (degree)
            {
                case 1:
                    {
                        int j = index - kLower;
                        knot[0] = pkn[j];
                        j++;
                        knot[1] = pkn[j];
                        break;
                    }
                case 2:
                    {
                        int j = index - 1 - kLower;
                        knot[0] = pkn[j];
                        j++;
                        knot[1] = pkn[j];
                        j++;
                        knot[2] = pkn[j];
                        j++;
                        knot[3] = pkn[j];
                        break;
                    }
                case 3:
                    {
                        int j = index - 2 - kLower;
                        knot[0] = pkn[j];
                        j++;
                        knot[1] = pkn[j];
                        j++;
                        knot[2] = pkn[j];
                        j++;
                        knot[3] = pkn[j];
                        j++;
                        knot[4] = pkn[j];
                        j++;
                        knot[5] = pkn[j];
                        break;
                    }
                case 4:
                    {
                        int j = index - 3 - kLower;
                        knot[0] = pkn[j];
                        j++;
                        knot[1] = pkn[j];
                        j++;
                        knot[2] = pkn[j];
                        j++;
                        knot[3] = pkn[j];
                        j++;
                        knot[4] = pkn[j];
                        j++;
                        knot[5] = pkn[j];
                        j++;
                        knot[6] = pkn[j];
                        j++;
                        knot[7] = pkn[j];
                        break;
                    }
                case 5:
                    {
                        int j = index - 4 - kLower;
                        knot[0] = pkn[j];
                        j++;
                        knot[1] = pkn[j];
                        j++;
                        knot[2] = pkn[j];
                        j++;
                        knot[3] = pkn[j];
                        j++;
                        knot[4] = pkn[j];
                        j++;
                        knot[5] = pkn[j];
                        j++;
                        knot[6] = pkn[j];
                        j++;
                        knot[7] = pkn[j];
                        j++;
                        knot[8] = pkn[j];
                        j++;
                        knot[9] = pkn[j];
                        break;
                    }
                case 6:
                    {
                        int j = index - 5 - kLower;
                        knot[0] = pkn[j];
                        j++;
                        knot[1] = pkn[j];
                        j++;
                        knot[2] = pkn[j];
                        j++;
                        knot[3] = pkn[j];
                        j++;
                        knot[4] = pkn[j];
                        j++;
                        knot[5] = pkn[j];
                        j++;
                        knot[6] = pkn[j];
                        j++;
                        knot[7] = pkn[j];
                        j++;
                        knot[8] = pkn[j];
                        j++;
                        knot[9] = pkn[j];
                        j++;
                        knot[10] = pkn[j];
                        j++;
                        knot[11] = pkn[j];
                        break;
                    }
                default:
                    {
                        int i, j;
                        int deg2 = degree << 1;
                        j = index - degree - kLower;

                        for (i = 0; i < deg2; i++)
                        {
                            j++;
                            knot[i] = pkn[j];
                        }
                        break;
                    }
            }
        }
        else
        {
            int i;
            int deg1 = degree - 1;
            int kUpper = knots.Upper();
            int mLower = mults.Lower();
            int mUpper = mults.Upper();
            ReadOnlySpan<int> pmu = mults.AsSpan(); // pmu[i - MLower]
            double dknot = 0;
            int ilow = index, mlow = 0;
            int iupp = index + 1, mupp = 0;
            double loffset = 0.0, uoffset = 0.0;
            bool getlow = true, getupp = true;
            if (periodic)
            {
                dknot = pkn[kUpper - kLower] - pkn[0];
                if (iupp > mUpper)
                {
                    iupp = mLower + 1;
                    uoffset = dknot;
                }
            }
            // Find the knots around Index

            for (i = 0; i < degree; i++)
            {
                if (getlow)
                {
                    mlow++;
                    if (mlow > pmu[ilow - mLower])
                    {
                        mlow = 1;
                        ilow--;
                        getlow = (ilow >= mLower);
                        if (periodic && !getlow)
                        {
                            ilow = mUpper - 1;
                            loffset = dknot;
                            getlow = true;
                        }
                    }
                    if (getlow)
                    {
                        knot[deg1 - i] = pkn[ilow - kLower] - loffset;
                    }
                }
                if (getupp)
                {
                    mupp++;
                    if (mupp > pmu[iupp - mLower])
                    {
                        mupp = 1;
                        iupp++;
                        getupp = (iupp <= mUpper);
                        if (periodic && !getupp)
                        {
                            iupp = mLower + 1;
                            uoffset = dknot;
                            getupp = true;
                        }
                    }
                    if (getupp)
                    {
                        knot[degree + i] = pkn[iupp - kLower] + uoffset;
                    }
                }
            }
        }
    }

    /// <summary>Returns the index of the first pole of the span starting at the knot <paramref name="index"/>.</summary>
    public static int PoleIndex(int degree, int index, bool periodic, NCollection_Array1<int> mults)
    {
        int i, pindex = 0;

        for (i = mults.Lower(); i <= index; i++)
        {
            pindex += mults[i];
        }
        if (periodic)
        {
            pindex -= mults[mults.Lower()];
        }
        else
        {
            pindex -= degree + 1;
        }

        return pindex;
    }

    /// <summary>
    /// Copies in <paramref name="lp"/> the Length + 1 poles starting at pole <paramref name="index"/>, in the even
    /// positions (one free slot of Dimension values between two poles), for <see cref="BoorScheme"/>. Wraps around
    /// the poles.
    /// </summary>
    public static void BuildBoor(int index, int length, int dimension, NCollection_Array1<double> Poles, Span<double> lp)
    {
        int poles = 0;
        int i, k, ip = Poles.Lower() + index * dimension;

        for (i = 0; i < length + 1; i++)
        {

            for (k = 0; k < dimension; k++)
            {
                lp[poles + k] = Poles[ip];
                ip++;
                if (ip > Poles.Upper())
                {
                    ip = Poles.Lower();
                }
            }
            poles += 2 * dimension;
        }
    }

    /// <summary>The index in the Boor scheme array of the pole <paramref name="index"/> (from 0 to Length + Depth).</summary>
    public static int BoorIndex(int index, int length, int depth)
    {
        if (index <= depth)
        {
            return index;
        }
        if (index <= length)
        {
            return 2 * index - depth;
        }
        return length + index - depth;
    }

    /// <summary>
    /// Copies the pole <paramref name="index"/> of the Boor scheme array <paramref name="locPoles"/> into
    /// <paramref name="Pole"/> at <paramref name="position"/>, which is advanced (and wraps around).
    /// </summary>
    public static void GetPole(int index, int length, int depth, int dimension, Span<double> locPoles,
                               ref int position, NCollection_Array1<double> Pole)
    {
        int k;
        int pole = BoorIndex(index, length, depth) * dimension;

        for (k = 0; k < dimension; k++)
        {
            Pole[position] = locPoles[pole + k];
            position++;
        }
        if (position > Pole.Upper())
        {
            position = Pole.Lower();
        }
    }

    /// <summary>
    /// Computes the number of poles and knots of the curve after inserting AddKnots (with AddMults, or
    /// multiplicity 1 each when null) for <see cref="InsertKnots(int, bool, int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{int}?, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, double, bool)"/>.
    /// Returns false if the knots to add are not increasing or are outside the range. With Add false, the
    /// multiplicities of existing knots are set, not increased.
    /// </summary>
    public static bool PrepareInsertKnots(int degree, bool periodic, NCollection_Array1<double> knots,
                                          NCollection_Array1<int> mults, NCollection_Array1<double> addKnots,
                                          NCollection_Array1<int>? addMults, out int nbPoles, out int nbKnots,
                                          double tolerance, bool add = true)
    {
        bool addflat = addMults == null;
        // Note: OCCT leaves NbPoles and NbKnots unchanged when it returns false
        nbPoles = 0;
        nbKnots = 0;

        int first, last;
        if (periodic)
        {
            first = knots.Lower();
            last = knots.Upper();
        }
        else
        {
            first = FirstUKnotIndex(degree, mults);
            last = LastUKnotIndex(degree, mults);
        }
        double adeltaK1 = knots[first] - addKnots[addKnots.Lower()];
        double adeltaK2 = addKnots[addKnots.Upper()] - knots[last];
        if (adeltaK1 > tolerance)
        {
            return false;
        }
        if (adeltaK2 > tolerance)
        {
            return false;
        }

        int sigma = 0, mult, amult;
        nbKnots = 0;
        int k = knots.Lower() - 1;
        int ak = addKnots.Lower();

        if (periodic && addKnots.Length() > 1)
        {
            // gka for case when segments was produced on full period only one knot
            // was added in the end of curve
            if (Math.Abs(adeltaK1) <= Constants.Resolution() && Math.Abs(adeltaK2) <= Constants.Resolution())
            {
                ak++;
            }
        }

        int lastKnotMult = mults[knots.Upper()];
        double au, oldau = addKnots[ak], eps;

        while (ak <= addKnots.Upper())
        {
            au = addKnots[ak];
            if (au < oldau)
            {
                return false;
            }
            oldau = au;

            eps = Max(tolerance, Epsilon(au));

            while ((k < knots.Upper()) && (knots[k + 1] - au <= eps))
            {
                k++;
                nbKnots++;
                sigma += mults[k];
            }

            if (addflat)
            {
                amult = 1;
            }
            else
            {
                amult = Max(0, addMults![ak]);
            }

            // OCCT: with Add false, equal entries of AddKnots count once with the first one's multiplicity here, but
            // InsertKnots processes each of them, so the two can disagree on the result
            while ((ak < addKnots.Upper()) && (Math.Abs(au - addKnots[ak + 1]) <= eps))
            {
                ak++;
                if (add)
                {
                    if (addflat)
                    {
                        amult++;
                    }
                    else
                    {
                        amult += Max(0, addMults![ak]);
                    }
                }
            }

            if (Math.Abs(au - knots[k]) <= eps)
            {
                // identic to existing knot
                mult = mults[k];
                if (add)
                {
                    if (mult + amult > degree)
                    {
                        amult = Max(0, degree - mult);
                    }
                    sigma += amult;
                }
                else if (amult > mult)
                {
                    if (amult > degree)
                    {
                        amult = degree;
                    }
                    if (k == knots.Upper() && periodic)
                    {
                        lastKnotMult = Max(amult, mult);
                        sigma += 2 * (lastKnotMult - mult);
                    }
                    else
                    {
                        sigma += amult - mult;
                    }
                }
                /*
                // on periodic curves if this is the last knot
                // the multiplicity is added twice to account for the first knot
                if (k == Knots.Upper() && Periodic) {
              if (Add)
                sigma += amult;
              else
                sigma += amult - mult;
                }
                */
            }
            else
            {
                // not identic to existing knot
                if (amult > 0)
                {
                    if (amult > degree)
                    {
                        amult = degree;
                    }
                    nbKnots++;
                    sigma += amult;
                }
            }

            ak++;
        }

        // count the last knots
        while (k < knots.Upper())
        {
            k++;
            nbKnots++;
            sigma += mults[k];
        }

        if (periodic)
        {
            // for periodic B-Spline the requirement is that multiplicities of the first
            // and last knots must be equal (see Geom_BSplineCurve constructor for
            // instance);
            // respectively AddMults() must meet this requirement if AddKnots() contains
            // knot(s) coincident with first or last
            nbPoles = sigma - lastKnotMult;
        }
        else
        {
            nbPoles = sigma - degree - 1;
        }

        return true;
    }

    private static void Copy(int nbPoles, ref int oldFirst, NCollection_Array1<double> oldPoles, ref int newFirst,
                             NCollection_Array1<double> newPoles)
    {
        // reset the index in the range for periodicity

        oldFirst =
          oldPoles.Lower() + (oldFirst - oldPoles.Lower()) % (oldPoles.Upper() - oldPoles.Lower() + 1);

        newFirst =
          newPoles.Lower() + (newFirst - newPoles.Lower()) % (newPoles.Upper() - newPoles.Lower() + 1);

        // copy
        int i;

        for (i = 1; i <= nbPoles; i++)
        {
            newPoles[newFirst] = oldPoles[oldFirst];
            oldFirst++;
            if (oldFirst > oldPoles.Upper())
            {
                oldFirst = oldPoles.Lower();
            }
            newFirst++;
            if (newFirst > newPoles.Upper())
            {
                newFirst = newPoles.Lower();
            }
        }
    }

    /// <summary>
    /// Inserts the knots AddKnots (with AddMults, or multiplicity 1 each when null) in the curve of dimension
    /// <paramref name="dimension"/>; the new arrays must be sized with <see cref="PrepareInsertKnots"/>. Knots
    /// closer than Epsilon to an existing knot raise its multiplicity (Add) or set it (not Add); multiplicities
    /// never exceed Degree.
    /// </summary>
    public static void InsertKnots(int degree, bool periodic, int dimension, NCollection_Array1<double> Poles,
                                   NCollection_Array1<double> Knots, NCollection_Array1<int> mults,
                                   NCollection_Array1<double> addKnots, NCollection_Array1<int>? addMults,
                                   NCollection_Array1<double> newPoles, NCollection_Array1<double> newKnots,
                                   NCollection_Array1<int> newMults, double tolerance, bool add = true)
    {
        bool addflat = addMults == null;

        int i, k, mult, firstmult;
        int index, kn, curnk, curk;
        int p, np, curp, curnp, length, depth;
        double u;
        int need;
        double eps;

        // -------------------
        // create local arrays
        // -------------------
        // Use stack-based allocation for small arrays (MaxDegree=25, typical Dimension=4)
        int knotsSize = 2 * degree;
        int polesSize = (2 * degree + 1) * dimension;
        Span<double> knots = knotsSize <= 64 ? stackalloc double[knotsSize] : new double[knotsSize];
        Span<double> poles = polesSize <= 256 ? stackalloc double[polesSize] : new double[polesSize];

        //----------------------------
        // loop on the knots to insert
        //----------------------------

        curk = Knots.Lower() - 1;    // current position in Knots
        curnk = newKnots.Lower() - 1; // current position in NewKnots
        curp = Poles.Lower();        // current position in Poles
        curnp = newPoles.Lower();     // current position in NewPoles

        // NewKnots, NewMults, NewPoles contains the current state of the curve

        // index is the first pole of the current curve for insertion schema

        if (periodic)
        {
            index = -mults[mults.Lower()];
        }
        else
        {
            index = -degree - 1;
        }

        // on Periodic curves the first knot and the last knot are inserted later
        // (they are the same knot)
        firstmult = 0; // multiplicity of the first-last knot for periodic

        // kn current knot to insert in AddKnots

        for (kn = addKnots.Lower(); kn <= addKnots.Upper(); kn++)
        {

            u = addKnots[kn];
            eps = Max(tolerance, Epsilon(u));

            //-----------------------------------
            // find the position in the old knots
            // and copy to the new knots
            //-----------------------------------

            while (curk < Knots.Upper() && Knots[curk + 1] - u <= eps)
            {
                curk++;
                curnk++;
                newKnots[curnk] = Knots[curk];
                index += newMults[curnk] = mults[curk];
            }

            //-----------------------------------
            // Slice the knots and the mults
            // to the current size of the new curve
            //-----------------------------------

            i = curnk + Knots.Upper() - curk;
            NCollection_Array1<double> nknots = new(newKnots, newKnots.Lower(), newKnots.Lower(), i);
            NCollection_Array1<int> nmults = new(newMults, newMults.Lower(), newMults.Lower(), i);

            //-----------------------------------
            // copy enough knots
            // to compute the insertion schema
            //-----------------------------------

            k = curk;
            i = curnk;
            mult = 0;

            while (mult < degree && k < Knots.Upper())
            {
                k++;
                i++;
                nknots[i] = Knots[k];
                mult += nmults[i] = mults[k];
            }

            // copy knots at the end for periodic curve
            if (periodic)
            {
                mult = 0;
                k = Knots.Upper();
                i = nknots.Upper();

                while (mult < degree && i > curnk)
                {
                    nknots[i] = Knots[k];
                    mult += nmults[i] = mults[k];
                    k--;
                    i--;
                }
                nmults[nmults.Upper()] = nmults[nmults.Lower()];
            }

            //------------------------------------
            // do the boor scheme on the new curve
            // to insert the new knot
            //------------------------------------

            bool sameknot = (Math.Abs(u - newKnots[curnk]) <= eps);

            if (sameknot)
            {
                length = Max(0, degree - newMults[curnk]);
            }
            else
            {
                length = degree;
            }

            if (addflat)
            {
                depth = 1;
            }
            else
            {
                depth = Min(degree, addMults![kn]);
            }

            if (sameknot)
            {
                if (add)
                {
                    if ((newMults[curnk] + depth) > degree)
                    {
                        depth = degree - newMults[curnk];
                    }
                }
                else
                {
                    depth = Max(0, depth - newMults[curnk]);
                }

                if (periodic)
                {
                    // on periodic curve the first and last knot are delayed to the end
                    if (curk == Knots.Lower() || (curk == Knots.Upper()))
                    {
                        if (firstmult == 0)
                        { // do that only once
                            firstmult += depth;
                        }
                        depth = 0;
                    }
                }
            }
            if (depth <= 0)
            {
                continue;
            }

            BuildKnots(degree, curnk, periodic, nknots, nmults, knots);

            // copy the poles

            need = newPoles.Lower() + (index + length + 1) * dimension - curnp;
            need = Min(need, Poles.Upper() - curp + 1);

            p = curp;
            np = curnp;
            Copy(need, ref p, Poles, ref np, newPoles);
            curp += need;
            curnp += need;

            // slice the poles to the current number of poles in case of periodic
            NCollection_Array1<double> npoles = new(newPoles, newPoles.Lower(), newPoles.Lower(), curnp - 1);

            BuildBoor(index, length, dimension, npoles, poles);
            BoorScheme(u, degree, knots, dimension, poles, depth, length);

            //-------------------
            // copy the new poles
            //-------------------

            curnp += depth * dimension; // number of poles is increased by depth
            NCollection_Array1<double> thePoles = new(newPoles, newPoles.Lower(), newPoles.Lower(), curnp - 1);
            np = newKnots.Lower() + (index + 1) * dimension; // OCCT: NewKnots.Lower() where NewPoles.Lower() is meant

            for (i = 1; i <= length + depth; i++)
            {
                GetPole(i, length, depth, dimension, poles, ref np, thePoles);
            }

            //-------------------
            // insert the knot
            //-------------------

            index += depth;
            if (sameknot)
            {
                newMults[curnk] += depth;
            }
            else
            {
                curnk++;
                newKnots[curnk] = u;
                newMults[curnk] = depth;
            }
        }

        //------------------------------
        // copy the last poles and knots
        //------------------------------

        Copy(Poles.Upper() - curp + 1, ref curp, Poles, ref curnp, newPoles);

        while (curk < Knots.Upper())
        {
            curk++;
            curnk++;
            newKnots[curnk] = Knots[curk];
            newMults[curnk] = mults[curk];
        }

        //------------------------------
        // process the first-last knot
        // on periodic curves
        //------------------------------

        if (firstmult > 0)
        {
            curnk = newKnots.Lower();
            if (newMults[curnk] + firstmult > degree)
            {
                firstmult = degree - newMults[curnk];
            }
            if (firstmult > 0)
            {

                length = degree - newMults[curnk];
                depth = firstmult;

                BuildKnots(degree, curnk, periodic, newKnots, newMults, knots);
                NCollection_Array1<double> npoles = new(newPoles,
                                                        newPoles.Lower(),
                                                        newPoles.Lower(),
                                                        newPoles.Upper() - depth * dimension);
                BuildBoor(0, length, dimension, npoles, poles);
                BoorScheme(newKnots[curnk], degree, knots, dimension, poles, depth, length);

                //---------------------------
                // copy the new poles
                // but rotate them with depth
                //---------------------------

                np = newPoles.Lower();

                for (i = depth; i < length + depth; i++)
                {
                    GetPole(i, length, depth, dimension, poles, ref np, newPoles);
                }

                np = newPoles.Upper() - depth * dimension + 1;

                for (i = 0; i < depth; i++)
                {
                    GetPole(i, length, depth, dimension, poles, ref np, newPoles);
                }

                newMults[newMults.Lower()] += depth;
                newMults[newMults.Upper()] += depth;
            }
        }
    }

    /// <summary>
    /// Decrements the multiplicity of the knot <paramref name="Index"/> to <paramref name="mult"/> (0 removes
    /// it) if the curve stays within <paramref name="tolerance"/>. Returns false if not possible.
    /// </summary>
    public static bool RemoveKnot(int Index, int mult, int degree, bool periodic, int dimension,
                                  NCollection_Array1<double> Poles, NCollection_Array1<double> Knots,
                                  NCollection_Array1<int> mults, NCollection_Array1<double> newPoles,
                                  NCollection_Array1<double> newKnots, NCollection_Array1<int> newMults,
                                  double tolerance)
    {
        int index, i, j, k, p, np;

        int theIndex = Index;

        // protection
        int first, last;
        if (periodic)
        {
            first = Knots.Lower();
            last = Knots.Upper();
        }
        else
        {
            first = global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(degree, mults) + 1;
            last = global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(degree, mults) - 1;
        }
        if (Index < first)
        {
            return false;
        }
        if (Index > last)
        {
            return false;
        }

        if (periodic && (Index == first))
        {
            theIndex = last;
        }

        int depth = mults[theIndex] - mult;
        int length = degree - mult;

        // -------------------
        // create local arrays
        // -------------------
        // Use stack-based allocation for small arrays (MaxDegree=25, typical Dimension=4)
        int knotsSize = 4 * degree;
        int polesSize = (2 * degree + 1) * dimension;
        Span<double> knots = knotsSize <= 128 ? stackalloc double[knotsSize] : new double[knotsSize];
        Span<double> poles = polesSize <= 256 ? stackalloc double[polesSize] : new double[polesSize];

        // ------------------------------------
        // build the knots for anti Boor Scheme
        // ------------------------------------

        // the new sequence of knots
        // is obtained from the knots at Index-1 and Index

        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(degree, theIndex - 1, periodic, Knots, mults, knots);
        index = PoleIndex(degree, theIndex - 1, periodic, mults);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(degree, theIndex, periodic, Knots, mults, knots.Slice(2 * degree));

        index += mult;

        for (i = 0; i < degree - mult; i++)
        {
            knots[i] = knots[i + mult];
        }

        for (i = degree - mult; i < 2 * degree; i++)
        {
            knots[i] = knots[2 * degree + i];
        }

        // ------------------------------------
        // build the poles for anti Boor Scheme
        // ------------------------------------

        p = Poles.Lower() + index * dimension;

        for (i = 0; i <= length + depth; i++)
        {
            j = dimension * BoorIndex(i, length, depth);

            for (k = 0; k < dimension; k++)
            {
                poles[j + k] = Poles[p + k];
            }
            p += dimension;
            if (p > Poles.Upper())
            {
                p = Poles.Lower();
            }
        }

        // ----------------
        // Anti Boor Scheme
        // ----------------

        // OCCT: AntiBoorScheme can step outside its local arrays (seen through GeomLib::ExtendSurfByLength in unit 7F's
        // offsets of data/occ's shapes): OCCT reads and writes the stack around its NCollection_LocalArray there, and the
        // tolerance test then almost surely fails. The port, whose spans are bounded, treats such a removal as impossible
        // (occt-issues.md, unit 7F).
        bool result;
        try
        {
            result = AntiBoorScheme(Knots[theIndex], degree, knots, dimension, poles, depth, length, tolerance);
        }
        catch (IndexOutOfRangeException)
        {
            result = false;
        }

        // ----------------
        // copy the results
        // ----------------

        if (result)
        {

            // poles

            p = Poles.Lower();
            np = newPoles.Lower();

            // unmodified poles before
            Copy((index + 1) * dimension, ref p, Poles, ref np, newPoles);

            // modified

            for (i = 1; i <= length; i++)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.GetPole(i, length, 0, dimension, poles, ref np, newPoles);
            }
            p += (length + depth) * dimension;

            // unmodified poles after
            if (p != Poles.Lower())
            {
                i = Poles.Upper() - p + 1;
                Copy(i, ref p, Poles, ref np, newPoles);
            }

            // knots and mults

            if (mult > 0)
            {
                newKnots.Assign(Knots);
                newMults.Assign(mults);
                newMults[theIndex] = mult;
                if (periodic)
                {
                    if (theIndex == first)
                    {
                        newMults[last] = mult;
                    }
                    if (theIndex == last)
                    {
                        newMults[first] = mult;
                    }
                }
            }
            else
            {
                if (!periodic || (theIndex != first && theIndex != last))
                {

                    for (i = Knots.Lower(); i < theIndex; i++)
                    {
                        newKnots[i] = Knots[i];
                        newMults[i] = mults[i];
                    }

                    for (i = theIndex + 1; i <= Knots.Upper(); i++)
                    {
                        newKnots[i - 1] = Knots[i];
                        newMults[i - 1] = mults[i];
                    }
                }
                else
                {
                    // The interesting case of a Periodic curve
                    // where the first and last knot is removed.

                    for (i = first; i < last - 1; i++)
                    {
                        newKnots[i] = Knots[i + 1];
                        newMults[i] = mults[i + 1];
                    }
                    newKnots[last - 1] = newKnots[first] + Knots[last] - Knots[first];
                    newMults[last - 1] = newMults[first];
                }
            }
        }

        return result;
    }

    /// <summary>Returns the number of knots of the curve after raising its degree to NewDegree.</summary>
    public static int IncreaseDegreeCountKnots(int degree, int newDegree, bool periodic, NCollection_Array1<int> mults)
    {
        if (periodic)
        {
            return mults.Length();
        }
        int f = FirstUKnotIndex(degree, mults);
        int l = LastUKnotIndex(degree, mults);
        int m, i, removed = 0, step = newDegree - degree;

        i = mults.Lower();
        m = degree + (f - i + 1) * step + 1;

        while (m > newDegree + 1)
        {
            removed++;
            m -= mults[i] + step;
            i++;
        }
        if (m < newDegree + 1)
        {
            removed--;
        }

        i = mults.Upper();
        m = degree + (i - l + 1) * step + 1;

        while (m > newDegree + 1)
        {
            removed++;
            m -= mults[i] + step;
            i--;
        }
        if (m < newDegree + 1)
        {
            removed--;
        }

        return mults.Length() - removed;
    }

    /// <summary>
    /// Raises the degree of the curve of dimension <paramref name="dimension"/> to NewDegree. The new knot count
    /// is given by <see cref="IncreaseDegreeCountKnots"/>; the arrays are expected indexed from 1.
    /// </summary>
    public static void IncreaseDegree(int degree, int newDegree, bool periodic, int dimension,
                                      NCollection_Array1<double> poles, NCollection_Array1<double> knots,
                                      NCollection_Array1<int> mults, NCollection_Array1<double> newPoles,
                                      NCollection_Array1<double> newKnots, NCollection_Array1<int> newMults)
    {
        // Degree elevation of a BSpline Curve

        // This algorithms loops on degree incrementation from Degree to NewDegree.
        // The variable curDeg is the current degree to increment.

        // Before degree incrementations a "working curve" is created.
        // It has the same knot, poles and multiplicities.

        // If the curve is periodic knots are added on the working curve before
        // and after the existing knots to make it a non-periodic curves.
        // The poles are also copied.

        // The first and last multiplicity of the working curve are set to Degree+1,
        // null poles are  added if necessary.

        // Then the degree is incremented on the working curve.
        // The knots are unchanged but all multiplicities will be incremented.

        // Each degree incrementation is achieved by averaging curDeg+1 curves.

        // See : Degree elevation of B-spline curves
        //       Hartmut PRAUTZSCH
        //       CAGD 1 (1984)

        //-------------------------
        // create the working curve
        //-------------------------

        int i, k, f, l, m, pf, pl, firstknot;

        pf = 0; // number of null poles added at beginning
        pl = 0; // number of null poles added at end

        int nbwknots = knots.Length();
        f = FirstUKnotIndex(degree, mults);
        l = LastUKnotIndex(degree, mults);

        if (periodic)
        {
            // Periodic curves are transformed in non-periodic curves

            // The working curve starts with the knots l to Upper - 1 less the period, so the null poles before the
            // first pole are counted from the multiplicities of l to Upper, and those after the last from Lower to f.
            // (OCCT counts each from the other end, which differs unless the two ends' multiplicities match, and
            // then changes the curve; Kerf decision 16801.)

            nbwknots += f - mults.Lower();

            pf = -degree - 1;

            for (i = l; i <= mults.Upper(); i++)
            {
                pf += mults[i];
            }

            nbwknots += mults.Upper() - l;

            pl = -degree - 1;

            for (i = mults.Lower(); i <= f; i++)
            {
                pl += mults[i];
            }
        }

        // copy the knots and multiplicities
        NCollection_Array1<double> wknots = new(1, nbwknots);
        NCollection_Array1<int> wmults = new(1, nbwknots);
        if (!periodic)
        {
            wknots.Assign(knots);
            wmults.Assign(mults);
        }
        else
        {
            // copy the knots for a periodic curve
            double period = knots[knots.Upper()] - knots[knots.Lower()];
            i = 0;

            for (k = l; k < knots.Upper(); k++)
            {
                i++;
                wknots[i] = knots[k] - period;
                wmults[i] = mults[k];
            }

            for (k = knots.Lower(); k <= knots.Upper(); k++)
            {
                i++;
                wknots[i] = knots[k];
                wmults[i] = mults[k];
            }

            for (k = knots.Lower() + 1; k <= f; k++)
            {
                i++;
                wknots[i] = knots[k] + period;
                wmults[i] = mults[k];
            }
        }

        // set the first and last mults to Degree+1
        // and add null poles

        pf += degree + 1 - wmults[1];
        wmults[1] = degree + 1;
        pl += degree + 1 - wmults[nbwknots];
        wmults[nbwknots] = degree + 1;

        //---------------------------
        // poles of the working curve
        //---------------------------

        int nbwpoles = 0;

        for (i = 1; i <= nbwknots; i++)
        {
            nbwpoles += wmults[i];
        }
        nbwpoles -= degree + 1;

        // we provide space for degree elevation
        NCollection_Array1<double> wpoles = new(1,
                                                (nbwpoles + (nbwknots - 1) * (newDegree - degree)) * dimension);

        for (i = 1; i <= pf * dimension; i++)
        {
            wpoles[i] = 0;
        }

        k = poles.Lower();

        for (i = pf * dimension + 1; i <= (nbwpoles - pl) * dimension; i++)
        {
            wpoles[i] = poles[k];
            k++;
            if (k > poles.Upper())
            {
                k = poles.Lower();
            }
        }

        for (i = (nbwpoles - pl) * dimension + 1; i <= nbwpoles * dimension; i++)
        {
            wpoles[i] = 0;
        }

        //-----------------------------------------------------------
        // Declare the temporary arrays used in degree incrementation
        //-----------------------------------------------------------

        // Maximum number of poles after all degree elevations
        int nbwpMax = nbwpoles + (nbwknots - 1) * (newDegree - degree);
        // Arrays for storing the temporary curves
        NCollection_Array1<double> tempc1 = new(1, nbwpMax * dimension);
        NCollection_Array1<double> tempc2 = new(1, nbwpMax * dimension);

        // Array for storing the knots to insert
        NCollection_Array1<double> iknots = new(1, nbwknots);

        // Arrays for receiving the knots after insertion
        NCollection_Array1<double> nknots = new(1, nbwknots);

        // Pre-allocate nwpoles with maximum size to avoid repeated heap allocations in loop
        NCollection_Array1<double> nwpoles = new(1, nbwpMax * dimension);

        //------------------------------
        // Loop on degree incrementation
        //------------------------------

        int step, curDeg;
        int nbp = nbwpoles;
        int nbwp = nbp;

        for (curDeg = degree; curDeg < newDegree; curDeg++)
        {

            nbp = nbwp;               // current number of poles
            nbwp = nbp + nbwknots - 1; // new number of poles

            // Initialize the portion of nwpoles that will be used this iteration
            for (int idx = 1; idx <= nbwp * dimension; idx++)
            {
                nwpoles[idx] = 0.0;
            }

            for (step = 0; step <= curDeg; step++)
            {

                // Compute the bspline of rank step.

                // if not the first time, decrement the multiplicities back
                if (step != 0)
                {
                    for (i = 1; i <= nbwknots; i++)
                    {
                        wmults[i]--;
                    }
                }

                // Poles are the current poles
                // but the poles congruent to step are duplicated.

                int offset = 0;

                for (i = 0; i < nbp; i++)
                {
                    offset++;

                    for (k = 0; k < dimension; k++)
                    {
                        // OCCT indexes wpoles (from 1) with NewPoles.Lower()
                        tempc1[(offset - 1) * dimension + k + 1] = wpoles[newPoles.Lower() + i * dimension + k];
                    }
                    if (i % (curDeg + 1) == step)
                    {
                        offset++;

                        for (k = 0; k < dimension; k++)
                        {
                            tempc1[(offset - 1) * dimension + k + 1] = wpoles[newPoles.Lower() + i * dimension + k];
                        }
                    }
                }

                // Knots multiplicities are increased
                // For each knot where the sum of multiplicities is congruent to step

                int stepmult = step + 1;
                int nbknots = 0;
                int smult = 0;

                for (k = 1; k <= nbwknots; k++)
                {
                    smult += wmults[k];
                    if (smult >= stepmult)
                    {
                        // this knot is increased
                        stepmult += curDeg + 1;
                        wmults[k]++;
                    }
                    else
                    {
                        // this knot is inserted
                        nbknots++;
                        iknots[nbknots] = wknots[k];
                    }
                }

                // the curve is obtained by inserting the knots
                // to raise the multiplicities

                // we build "slices" of the arrays to set the correct size
                if (nbknots > 0)
                {
                    NCollection_Array1<double> aknots = new(iknots, 1, 1, nbknots);
                    NCollection_Array1<double> curve = new(tempc1, 1, 1, offset * dimension);
                    NCollection_Array1<double> ncurve = new(tempc2, 1, 1, nbwp * dimension);
                    //	InsertKnots(curDeg+1,false,Dimension,curve,wknots,wmults,
                    //		    aknots,NoMults(),ncurve,nknots,wmults,Epsilon(1.));

                    InsertKnots(curDeg + 1,
                                false,
                                dimension,
                                curve,
                                wknots,
                                wmults,
                                aknots,
                                NoMults(),
                                ncurve,
                                nknots,
                                wmults,
                                0.0);

                    // add to the average

                    for (i = 1; i <= nbwp * dimension; i++)
                    {
                        nwpoles[i] += ncurve[i];
                    }
                }
                else
                {
                    // add to the average

                    for (i = 1; i <= nbwp * dimension; i++)
                    {
                        nwpoles[i] += tempc1[i];
                    }
                }
            }

            // The result is the average

            for (i = 1; i <= nbwp * dimension; i++)
            {
                wpoles[i] = nwpoles[i] / (curDeg + 1);
            }
        }

        //-----------------
        // Copy the results
        //-----------------

        // index in new knots of the first knot of the curve
        if (periodic)
        {
            firstknot = mults.Upper() - l + 1;
        }
        else
        {
            firstknot = f;
        }

        // the new curve starts at index firstknot
        // so we must remove knots until the sum of multiplicities
        // from the first to the start is NewDegree+1

        // m is the current sum of multiplicities
        m = 0;

        for (k = 1; k <= firstknot; k++)
        {
            m += wmults[k];
        }

        // compute the new first knot (k), pf will be the index of the first pole
        k = 1;
        pf = 0;

        while (m > newDegree + 1)
        {
            k++;
            m -= wmults[k];
            pf += wmults[k];
        }
        if (m < newDegree + 1)
        {
            k--;
            wmults[k] += m - newDegree - 1;
            pf += m - newDegree - 1;
        }

        // on a periodic curve the knots start with firstknot
        if (periodic)
        {
            k = firstknot;
        }

        // copy knots

        for (i = newKnots.Lower(); i <= newKnots.Upper(); i++)
        {
            newKnots[i] = wknots[k];
            newMults[i] = wmults[k];
            k++;
        }

        // copy poles
        pf *= dimension;

        for (i = newPoles.Lower(); i <= newPoles.Upper(); i++)
        {
            pf++;
            newPoles[i] = wpoles[pf];
        }
    }

    /// <summary>Computes the number of knots and poles of the curve once made non-periodic (<see cref="Unperiodize(int, int, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{double})"/>).</summary>
    public static void PrepareUnperiodize(int degree, NCollection_Array1<int> mults, out int nbKnots, out int nbPoles)
    {
        int i;
        // initialize NbKnots and NbPoles
        nbKnots = mults.Length();
        nbPoles = -degree - 1;

        for (i = mults.Lower(); i <= mults.Upper(); i++)
        {
            nbPoles += mults[i];
        }

        int sigma, k;
        // Add knots at the beginning of the curve to raise Multiplicities
        // to Degre + 1;
        sigma = mults[mults.Lower()];
        k = mults.Upper() - 1;

        while (sigma < degree + 1)
        {
            sigma += mults[k];
            nbPoles += mults[k];
            k--;
            nbKnots++;
        }
        // We must add exactly until Degree + 1 ->
        //    Suppress the excedent.
        if (sigma > degree + 1)
        {
            nbPoles -= sigma - degree - 1;
        }

        // Add knots at the end of the curve to raise Multiplicities
        // to Degre + 1;
        sigma = mults[mults.Upper()];
        k = mults.Lower() + 1;

        while (sigma < degree + 1)
        {
            sigma += mults[k];
            nbPoles += mults[k];
            k++;
            nbKnots++;
        }
        // We must add exactly until Degree + 1 ->
        //    Suppress the excedent.
        if (sigma > degree + 1)
        {
            nbPoles -= sigma - degree - 1;
        }
    }

    /// <summary>
    /// Makes a periodic curve of dimension <paramref name="dimension"/> non-periodic (arrays sized with
    /// <see cref="PrepareUnperiodize"/>, indexed from 1).
    /// </summary>
    public static void Unperiodize(int degree, int dimension, NCollection_Array1<int> mults,
                                   NCollection_Array1<double> knots, NCollection_Array1<double> poles,
                                   NCollection_Array1<int> newMults, NCollection_Array1<double> newKnots,
                                   NCollection_Array1<double> newPoles)
    {
        int sigma, k, index = 0;
        // evaluation of index : number of knots to insert before knot(1) to
        // raise sum of multiplicities to <Degree + 1>
        sigma = mults[mults.Lower()];
        k = mults.Upper() - 1;

        while (sigma < degree + 1)
        {
            sigma += mults[k];
            k--;
            index++;
        }

        double period = knots[knots.Upper()] - knots[knots.Lower()];

        // set the 'interior' knots;

        for (k = 1; k <= knots.Length(); k++)
        {
            newKnots[k + index] = knots[k];
            newMults[k + index] = mults[k];
        }

        // set the 'starting' knots;

        for (k = 1; k <= index; k++)
        {
            newKnots[k] = newKnots[k + knots.Length() - 1] - period;
            newMults[k] = newMults[k + knots.Length() - 1];
        }
        newMults[1] -= sigma - degree - 1;

        // set the 'ending' knots;
        sigma = newMults[index + knots.Length()];

        for (k = knots.Length() + index + 1; k <= newKnots.Length(); k++)
        {
            newKnots[k] = newKnots[k - knots.Length() + 1] + period;
            newMults[k] = newMults[k - knots.Length() + 1];
            sigma += newMults[k - knots.Length() + 1];
        }
        newMults[newMults.Length()] -= sigma - degree - 1;

        for (k = 1; k <= newPoles.Length(); k++)
        {
            newPoles[k] = poles[(k - 1) % poles.Length() + 1];
        }
    }

    /// <summary>Computes the number of knots and poles of the curve trimmed to [U1, U2] (<see cref="Trimming(int, bool, int, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double}, double, double, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double})"/>).</summary>
    public static void PrepareTrimming(int degree, bool periodic, NCollection_Array1<double> knots,
                                       NCollection_Array1<int> mults, double u1, double u2, out int nbKnots,
                                       out int nbPoles)
    {
        int i;
        double newU1, newU2;
        int index1 = 0, index2 = 0;

        // Eval index1, index2 : position of U1 and U2 in the Array Knots
        // such as Knots(index1-1) <= U1 < Knots(index1)
        //         Knots(index2-1) <= U2 < Knots(index2)
        LocateParameter(degree, knots, mults, u1, periodic, knots.Lower(), knots.Upper(), out index1, out newU1);
        LocateParameter(degree, knots, mults, u2, periodic, knots.Lower(), knots.Upper(), out index2, out newU2);
        index1++;
        if (Math.Abs(knots[index2] - u2) <= Epsilon(u1))
        {
            index2--;
        }

        // eval NbKnots:
        nbKnots = index2 - index1 + 3;

        // eval NbPoles:
        nbPoles = degree + 1;

        for (i = index1; i <= index2; i++)
        {
            nbPoles += mults[i];
        }
    }

    /// <summary>
    /// Trims the curve of dimension <paramref name="dimension"/> to [U1, U2] (arrays sized with
    /// <see cref="PrepareTrimming"/>, indexed from 1). Throws <see cref="OutOfRangeException"/> if U1 or U2 is
    /// outside the curve.
    /// </summary>
    public static void Trimming(int degree, bool periodic, int dimension, NCollection_Array1<double> knots,
                                NCollection_Array1<int> mults, NCollection_Array1<double> poles, double u1, double u2,
                                NCollection_Array1<double> newKnots, NCollection_Array1<int> newMults,
                                NCollection_Array1<double> newPoles)
    {
        int i, nbpoles = 0, nbknots = 0;
        double[] kk = [u1, u2];
        int[] mm = [degree, degree];
        NCollection_Array1<double> k = new(kk, 0, 1, 2);
        NCollection_Array1<int> m = new(mm, 0, 1, 2);
        if (!PrepareInsertKnots(degree,
                                periodic,
                                knots,
                                mults,
                                k,
                                m,
                                out nbpoles,
                                out nbknots,
                                Epsilon(u1),
                                false))
        {
            throw new OutOfRangeException();
        }

        NCollection_Array1<double> tempPoles = new(1, nbpoles * dimension);
        NCollection_Array1<double> tempKnots = new(1, nbknots);
        NCollection_Array1<int> tempMults = new(1, nbknots);

        //
        // do not allow the multiplicities to Add : they must be less than Degree
        //
        InsertKnots(degree,
                    periodic,
                    dimension,
                    poles,
                    knots,
                    mults,
                    k,
                    m,
                    tempPoles,
                    tempKnots,
                    tempMults,
                    Epsilon(u1),
                    false);

        // find in TempPoles the index of the pole corresponding to U1
        int kindex = 0, pindex;
        double newU1;
        LocateParameter(degree,
                        tempKnots,
                        tempMults,
                        u1,
                        periodic,
                        tempKnots.Lower(),
                        tempKnots.Upper(),
                        out kindex,
                        out newU1);
        pindex = PoleIndex(degree, kindex, periodic, tempMults);
        pindex *= dimension;

        for (i = 1; i <= newPoles.Length(); i++)
        {
            newPoles[i] = tempPoles[pindex + i];
        }

        for (i = 1; i <= newKnots.Length(); i++)
        {
            newKnots[i] = tempKnots[kindex + i - 1];
            newMults[i] = tempMults[kindex + i - 1];
        }
        newMults[1] = Min(degree, newMults[1]) + 1;
        newMults[newMults.Length()] = Min(degree, newMults[newMults.Length()]) + 1;
    }

    /// <summary>
    /// Solves the banded system factored by <see cref="FactorBandedMatrix"/> for <paramref name="array"/>, an array
    /// of points of dimension ArrayDimension (one per row), in place. Returns 0, or 1 on a bad matrix or a null pivot.
    /// </summary>
    public static int SolveBandedSystem(Matrix matrix, int upperBandWidth, int lowerBandWidth, int arrayDimension,
                                        Span<double> array)
    {
        if (matrix.LowerCol() != 1 || matrix.UpperCol() != upperBandWidth + lowerBandWidth + 1)
        {
            return 1;
        }

        Span<double> polesArray = array;
        int lRow = matrix.LowerRow();
        int nCols = matrix.ColNumber();
        ReadOnlySpan<double> matData = matrix.AsSpan(); // from (aLRow, 1), row-major

        for (int ii = lRow + 1; ii <= matrix.UpperRow(); ii++)
        {
            int minIndex = (ii - lowerBandWidth >= lRow ? ii - lowerBandWidth : lRow);
            ReadOnlySpan<double> rowII = matData.Slice((ii - lRow) * nCols, nCols);

            for (int jj = minIndex; jj < ii; jj++)
            {
                double coeff = rowII[jj - ii + lowerBandWidth];
                for (int kk = 0; kk < arrayDimension; kk++)
                {
                    polesArray[(ii - 1) * arrayDimension + kk] +=
                      polesArray[(jj - 1) * arrayDimension + kk] * coeff;
                }
            }
        }

        for (int ii = matrix.UpperRow(); ii >= lRow; ii--)
        {
            int maxIndex =
              (ii + upperBandWidth <= matrix.UpperRow() ? ii + upperBandWidth : matrix.UpperRow());
            ReadOnlySpan<double> rowII = matData.Slice((ii - lRow) * nCols, nCols);

            for (int jj = maxIndex; jj > ii; jj--)
            {
                double coeff = rowII[jj - ii + lowerBandWidth];
                for (int kk = 0; kk < arrayDimension; kk++)
                {
                    polesArray[(ii - 1) * arrayDimension + kk] -=
                      polesArray[(jj - 1) * arrayDimension + kk] * coeff;
                }
            }

            // Fixing a bug PRO18577 to avoid division by zero
            double divisor = rowII[lowerBandWidth];
            const double THE_TOLERANCE = 1.0e-16;
            if (Math.Abs(divisor) <= THE_TOLERANCE)
            {
                return 1;
            }
            double inverse = 1.0 / divisor;

            for (int kk = 0; kk < arrayDimension; kk++)
            {
                polesArray[(ii - 1) * arrayDimension + kk] *= inverse;
            }
        }

        return 0;
    }

    /// <summary>
    /// Solves the banded system for rational poles: if <paramref name="homogenousFlag"/> is false the poles are
    /// first multiplied by the weights and divided back after. Returns 0, or 2 or 3 if solving the poles or the
    /// weights failed (1 on a bad matrix).
    /// </summary>
    public static int SolveBandedSystem(Matrix matrix, int upperBandWidth, int lowerBandWidth, bool homogenousFlag,
                                        int arrayDimension, Span<double> array, Span<double> weights)
    {
        bool homogeneousFlag = homogenousFlag;
        if (matrix.LowerCol() != 1 || matrix.UpperCol() != upperBandWidth + lowerBandWidth + 1)
        {
            return 1;
        }

        Span<double> polesArray = array;
        Span<double> weightsArray = weights;
        int numRows = matrix.UpperRow() - matrix.LowerRow() + 1;

        if (!homogeneousFlag)
        {
            for (int ii = 0; ii < numRows; ii++)
            {
                for (int kk = 0; kk < arrayDimension; kk++)
                {
                    polesArray[ii * arrayDimension + kk] *= weightsArray[ii];
                }
            }
        }

        int errorCode =
          global::Kerf.Engine.Geometry.Splines.BSplCLib.SolveBandedSystem(matrix, upperBandWidth, lowerBandWidth, arrayDimension, array);
        if (errorCode != 0)
        {
            return 2;
        }

        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.SolveBandedSystem(matrix, upperBandWidth, lowerBandWidth, 1, weights);
        if (errorCode != 0)
        {
            return 3;
        }

        if (!homogeneousFlag)
        {
            for (int ii = 0; ii < numRows; ii++)
            {
                double inverse = 1.0 / weightsArray[ii];
                for (int kk = 0; kk < arrayDimension; kk++)
                {
                    polesArray[ii * arrayDimension + kk] *= inverse;
                }
            }
        }

        return 0;
    }

    /// <summary>Computes the Schoenberg points (the averages of Degree consecutive flat knots) into Parameters.</summary>
    public static void BuildSchoenbergPoints(int degree, NCollection_Array1<double> flatKnots,
                                             NCollection_Array1<double> parameters)
    {
        int ii, jj;
        double inverse;
        inverse = 1.0e0 / (double)degree;

        for (ii = parameters.Lower(); ii <= parameters.Upper(); ii++)
        {
            parameters[ii] = 0.0e0;

            for (jj = 1; jj <= degree; jj++)
            {
                parameters[ii] += flatKnots[jj + ii];
            }
            parameters[ii] *= inverse;
        }
    }

    /// <summary>
    /// Interpolates the points (and derivatives, by ContactOrderArray) of dimension ArrayDimension stored in
    /// <paramref name="poles"/> at Parameters with a B-spline on FlatKnots; Poles receives the poles.
    /// InversionProblem is the pivot index of a singular matrix (0 otherwise). Throws
    /// <see cref="OutOfRangeException"/> if the system can't be built or solved.
    /// </summary>
    public static void Interpolate(int degree, NCollection_Array1<double> flatKnots, NCollection_Array1<double> parameters,
                                   NCollection_Array1<int> contactOrderArray, int arrayDimension, Span<double> poles,
                                   out int inversionProblem)
    {
        int errorCode, upperBandWidth, lowerBandWidth;
        // Note: OCCT uses a stack buffer of up to 2048 values for the matrix
        Matrix interpolationMatrix = new(1, parameters.Length(), 1, 2 * degree + 1);
        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildBSpMatrix(parameters,
                                            contactOrderArray,
                                            flatKnots,
                                            degree,
                                            interpolationMatrix,
                                            out upperBandWidth,
                                            out lowerBandWidth);
        if (errorCode != 0)
        {
            throw new OutOfRangeException("BSplCLib::Interpolate");
        }

        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.FactorBandedMatrix(interpolationMatrix,
                                                upperBandWidth,
                                                lowerBandWidth,
                                                out inversionProblem);
        if (errorCode != 0)
        {
            throw new OutOfRangeException("BSplCLib::Interpolate");
        }

        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.SolveBandedSystem(interpolationMatrix,
                                               upperBandWidth,
                                               lowerBandWidth,
                                               arrayDimension,
                                               poles);
        if (errorCode != 0)
        {
            throw new OutOfRangeException("BSplCLib::Interpolate");
        }
    }

    /// <summary>
    /// Rational interpolation: as <see cref="Interpolate(int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, int, Span{double}, out int)"/>
    /// with the weights in <paramref name="weights"/> (non-homogeneous poles).
    /// </summary>
    public static void Interpolate(int degree, NCollection_Array1<double> flatKnots, NCollection_Array1<double> parameters,
                                   NCollection_Array1<int> contactOrderArray, int arrayDimension, Span<double> poles,
                                   Span<double> weights, out int inversionProblem)
    {
        int errorCode, upperBandWidth, lowerBandWidth;
        // Note: OCCT uses a stack buffer of up to 2048 values for the matrix
        Matrix interpolationMatrix = new(1, parameters.Length(), 1, 2 * degree + 1);
        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildBSpMatrix(parameters,
                                            contactOrderArray,
                                            flatKnots,
                                            degree,
                                            interpolationMatrix,
                                            out upperBandWidth,
                                            out lowerBandWidth);
        if (errorCode != 0)
        {
            throw new OutOfRangeException("BSplCLib::Interpolate");
        }

        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.FactorBandedMatrix(interpolationMatrix,
                                                upperBandWidth,
                                                lowerBandWidth,
                                                out inversionProblem);
        if (errorCode != 0)
        {
            throw new OutOfRangeException("BSplCLib::Interpolate");
        }

        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.SolveBandedSystem(interpolationMatrix,
                                               upperBandWidth,
                                               lowerBandWidth,
                                               false,
                                               arrayDimension,
                                               poles,
                                               weights);
        if (errorCode != 0)
        {
            throw new OutOfRangeException("BSplCLib::Interpolate");
        }
    }

    /// <summary>
    /// Evaluates the rational B-spline (poles of dimension ArrayDimension, non-homogeneous, and weights) and its
    /// derivatives up to DerivativeRequest at <paramref name="u"/>, into PolesResult (homogeneous) and
    /// WeightsResult. <paramref name="extrapMode"/> holds two values: the degree of extrapolation before the
    /// first and after the last knot (a Taylor expansion of that order is used beyond the ends).
    /// </summary>
    public static void Eval(double u, bool periodicFlag, int derivativeRequest, ReadOnlySpan<int> extrapMode, int degree,
                            NCollection_Array1<double> flatKnots, int arrayDimension, ReadOnlySpan<double> poles,
                            ReadOnlySpan<double> weights, Span<double> polesResult, Span<double> weightsResult)
    {
        double parameter = u;
        int ii, jj, kk = 0, index, index1, index2, modulus, newRequest,
                    errorCode, order = degree + 1, firstNonZeroBsplineIndex,
                    localRequest = derivativeRequest;
        Span<int> extrapolatingFlag = stackalloc int[2];
        double localParameter, period, inverse, delta;
        ReadOnlySpan<double> polesArray = poles;
        ReadOnlySpan<double> weightsArray = weights;
        ReadOnlySpan<int> extrapModeArray = extrapMode;
        Span<double> pResultArray = polesResult;
        Span<double> wResultArray = weightsResult;
        localParameter = parameter;
        extrapolatingFlag[0] = extrapolatingFlag[1] = 0;
        //
        // check if we are extrapolating to a degree which is smaller than
        // the degree of the Bspline
        //
        if (periodicFlag)
        {
            period = flatKnots[flatKnots.Upper() - 1] - flatKnots[2];

            while (localParameter > flatKnots[flatKnots.Upper() - 1])
            {
                localParameter -= period;
            }

            while (localParameter < flatKnots[2])
            {
                localParameter += period;
            }
        }
        if (parameter < flatKnots[2] && localRequest < extrapModeArray[0] && extrapModeArray[0] < degree)
        {
            localRequest = extrapModeArray[0];
            localParameter = flatKnots[2];
            extrapolatingFlag[0] = 1;
        }
        if (parameter > flatKnots[flatKnots.Upper() - 1] && localRequest < extrapModeArray[1]
            && extrapModeArray[1] < degree)
        {
            localRequest = extrapModeArray[1];
            localParameter = flatKnots[flatKnots.Upper() - 1];
            extrapolatingFlag[1] = 1;
        }
        delta = parameter - localParameter;
        if (localRequest >= order)
        {
            localRequest = degree;
        }
        if (periodicFlag)
        {
            modulus = flatKnots.Length() - degree - 1;
        }
        else
        {
            modulus = flatKnots.Length() - degree;
        }

        Matrix bsplineBasis = BSplCLib_LocalMatrix(localRequest, order);
        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(localRequest,
                                              order,
                                              flatKnots,
                                              localParameter,
                                              out firstNonZeroBsplineIndex,
                                              bsplineBasis);
        if (errorCode != 0)
        {
            return;
        }
        if (extrapolatingFlag[0] == 0 && extrapolatingFlag[1] == 0)
        {
            index = 0;
            index2 = 0;

            for (ii = 1; ii <= localRequest + 1; ii++)
            {
                index1 = firstNonZeroBsplineIndex;

                for (kk = 0; kk < arrayDimension; kk++)
                {
                    pResultArray[index + kk] = 0.0e0;
                }
                wResultArray[index] = 0.0e0;

                for (jj = 1; jj <= order; jj++)
                {

                    for (kk = 0; kk < arrayDimension; kk++)
                    {
                        pResultArray[index + kk] += polesArray[(index1 - 1) * arrayDimension + kk]
                                                    * weightsArray[index1 - 1] * bsplineBasis[ii, jj];
                    }
                    wResultArray[index2] += weightsArray[index1 - 1] * bsplineBasis[ii, jj];

                    index1 = index1 % modulus;
                    index1 += 1;
                }
                index += arrayDimension;
                index2 += 1;
            }
        }
        else
        {
            //
            //  store Taylor expansion in LocalRealArray
            //
            newRequest = derivativeRequest;
            if (newRequest > degree)
            {
                newRequest = degree;
            }
            // OCCT sizes LocalRealArray (LocalRequest + 1) * ArrayDimension, then evaluates a polynomial of degree
            // Degree (> LocalRequest here) on it, reading past the Taylor coefficients into uninitialised memory.
            // The port sizes it for Degree + 1 rows; the extra rows are zero.
            int localSize = Max(localRequest + 1, degree + 1) * arrayDimension;
            Span<double> localRealArray = localSize <= 128 ? stackalloc double[localSize] : new double[localSize];
            index = 0;
            inverse = 1.0e0;

            for (ii = 1; ii <= localRequest + 1; ii++)
            {
                index1 = firstNonZeroBsplineIndex;

                for (kk = 0; kk < arrayDimension; kk++)
                {
                    localRealArray[index + kk] = 0.0e0;
                }

                for (jj = 1; jj <= order; jj++)
                {

                    for (kk = 0; kk < arrayDimension; kk++)
                    {
                        localRealArray[index + kk] += polesArray[(index1 - 1) * arrayDimension + kk]
                                                      * weightsArray[index1 - 1] * bsplineBasis[ii, jj];
                    }
                    index1 = index1 % modulus;
                    index1 += 1;
                }

                for (kk = 0; kk < arrayDimension; kk++)
                {
                    localRealArray[index + kk] *= inverse;
                }
                index += arrayDimension;
                inverse /= (double)ii;
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(delta, newRequest, degree, arrayDimension, localRealArray, polesResult); // OCCT: see above
            index = 0;
            inverse = 1.0e0;

            for (ii = 1; ii <= localRequest + 1; ii++)
            {
                index1 = firstNonZeroBsplineIndex;
                localRealArray[index] = 0.0e0;

                for (jj = 1; jj <= order; jj++)
                {
                    localRealArray[index] += weightsArray[index1 - 1] * bsplineBasis[ii, jj];
                    index1 = index1 % modulus;
                    index1 += 1;
                }
                localRealArray[index] *= inverse;
                index += 1;
                inverse /= (double)ii;
            }
            // OCCT: here the coefficients past LocalRequest are the poles' values left from above
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(delta, newRequest, degree, 1, localRealArray, weightsResult);
        }
    }

    /// <summary>
    /// Evaluates the B-spline (poles of dimension ArrayDimension) and its derivatives up to DerivativeRequest at
    /// <paramref name="u"/> into Result. <paramref name="extrapMode"/> holds two values: the degree of
    /// extrapolation before the first and after the last knot.
    /// </summary>
    public static void Eval(double u, bool periodicFlag, int derivativeRequest, ReadOnlySpan<int> extrapMode, int degree,
                            NCollection_Array1<double> flatKnots, int arrayDimension, ReadOnlySpan<double> poles,
                            Span<double> result)
    {
        double parameter = u;
        int ii, jj, kk, index, index1, modulus, newRequest,
          errorCode, order = degree + 1, firstNonZeroBsplineIndex, localRequest = derivativeRequest;
        Span<int> extrapolatingFlag = stackalloc int[2];

        double localParameter, period, inverse, delta;

        ReadOnlySpan<double> polesArray = poles;
        ReadOnlySpan<int> extrapModeArray = extrapMode;
        Span<double> resultArray = result;
        localParameter = parameter;
        extrapolatingFlag[0] = extrapolatingFlag[1] = 0;
        //
        // check if we are extrapolating to a degree which is smaller than
        // the degree of the Bspline
        //
        if (periodicFlag)
        {
            period = flatKnots[flatKnots.Upper() - 1] - flatKnots[2];

            while (localParameter > flatKnots[flatKnots.Upper() - 1])
            {
                localParameter -= period;
            }

            while (localParameter < flatKnots[2])
            {
                localParameter += period;
            }
        }
        if (parameter < flatKnots[2] && localRequest < extrapModeArray[0] && extrapModeArray[0] < degree)
        {
            localRequest = extrapModeArray[0];
            localParameter = flatKnots[2];
            extrapolatingFlag[0] = 1;
        }
        if (parameter > flatKnots[flatKnots.Upper() - 1] && localRequest < extrapModeArray[1]
            && extrapModeArray[1] < degree)
        {
            localRequest = extrapModeArray[1];
            localParameter = flatKnots[flatKnots.Upper() - 1];
            extrapolatingFlag[1] = 1;
        }
        delta = parameter - localParameter;
        if (localRequest >= order)
        {
            localRequest = degree;
        }

        if (periodicFlag)
        {
            modulus = flatKnots.Length() - degree - 1;
        }
        else
        {
            modulus = flatKnots.Length() - degree;
        }

        Matrix bsplineBasis = BSplCLib_LocalMatrix(localRequest, order);

        errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(localRequest,
                                              order,
                                              flatKnots,
                                              localParameter,
                                              out firstNonZeroBsplineIndex,
                                              bsplineBasis);
        if (errorCode != 0)
        {
            return;
        }
        if (extrapolatingFlag[0] == 0 && extrapolatingFlag[1] == 0)
        {
            index = 0;

            for (ii = 1; ii <= localRequest + 1; ii++)
            {
                index1 = firstNonZeroBsplineIndex;

                for (kk = 0; kk < arrayDimension; kk++)
                {
                    resultArray[index + kk] = 0.0e0;
                }

                for (jj = 1; jj <= order; jj++)
                {

                    for (kk = 0; kk < arrayDimension; kk++)
                    {
                        resultArray[index + kk] +=
                          polesArray[(index1 - 1) * arrayDimension + kk] * bsplineBasis[ii, jj];
                    }
                    index1 = index1 % modulus;
                    index1 += 1;
                }
                index += arrayDimension;
            }
        }
        else
        {
            //
            //  store Taylor expansion in LocalRealArray
            //
            newRequest = derivativeRequest;
            if (newRequest > degree)
            {
                newRequest = degree;
            }
            // OCCT sizes LocalRealArray (LocalRequest + 1) * ArrayDimension, then evaluates a polynomial of degree
            // Degree (> LocalRequest here) on it, reading past the Taylor coefficients into uninitialised memory.
            // The port sizes it for Degree + 1 rows; the extra rows are zero.
            int localSize = Max(localRequest + 1, degree + 1) * arrayDimension;
            Span<double> localRealArray = localSize <= 128 ? stackalloc double[localSize] : new double[localSize];

            index = 0;
            inverse = 1.0e0;

            for (ii = 1; ii <= localRequest + 1; ii++)
            {
                index1 = firstNonZeroBsplineIndex;

                for (kk = 0; kk < arrayDimension; kk++)
                {
                    localRealArray[index + kk] = 0.0e0;
                }

                for (jj = 1; jj <= order; jj++)
                {

                    for (kk = 0; kk < arrayDimension; kk++)
                    {
                        localRealArray[index + kk] +=
                          polesArray[(index1 - 1) * arrayDimension + kk] * bsplineBasis[ii, jj];
                    }
                    index1 = index1 % modulus;
                    index1 += 1;
                }

                for (kk = 0; kk < arrayDimension; kk++)
                {
                    localRealArray[index + kk] *= inverse;
                }
                index += arrayDimension;
                inverse /= (double)ii;
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(delta, newRequest, degree, arrayDimension, localRealArray, result); // OCCT: see above
        }
    }

    /// <summary>
    /// Extends a B-spline (flat knots, NumPoles poles of dimension Dimension) to the point ConstraintPoint, after
    /// its end (<paramref name="after"/>) or before its start, with continuity <paramref name="continuity"/> at the
    /// junction; C1Coefficient scales the tangent. Writes the result's flat knots and poles into KnotsResult and
    /// PolesResult (which must be large enough) and their counts into NbKnotsRsult and NbPolesResult.
    /// </summary>
    public static void TangExtendToConstraint(NCollection_Array1<double> flatKnots, double c1Coefficient, int numPoles,
                                              ReadOnlySpan<double> poles, int dimension, int degree,
                                              NCollection_Array1<double> constraintPoint, int continuity, bool after,
                                              out int nbPolesResult, out int nbKnotsRsult, Span<double> knotsResult,
                                              Span<double> polesResult)
    {
        int cDimension = dimension, cDegree = degree;
        ReadOnlySpan<double> padr = poles;
        Span<double> kRadr = knotsResult;
        Span<double> pRadr = polesResult;

        ////////////////////////////////////////////////////////////////////////
        //
        //    1. calculation of extension nD
        //
        ////////////////////////////////////////////////////////////////////////

        //  Hermite matrix
        int csize = continuity + 2;
        Matrix matCoefs = new(1, csize, 1, csize);
        if (after)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(0,
                                     1, // Limits
                                     continuity,
                                     0, // Orders of constraints
                                     matCoefs);
        }
        else
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(0,
                                     1, // Limits
                                     0,
                                     continuity, // Orders of constraints
                                     matCoefs);
        }

        //  position at the node of connection
        double tbord;
        if (after)
        {
            tbord = flatKnots[flatKnots.Upper() - cDegree];
        }
        else
        {
            tbord = flatKnots[flatKnots.Lower() + cDegree];
        }
        bool periodic_flag = false;
        int ipos, derivative_request = Max(continuity, 1);
        Span<int> extrap_mode = stackalloc int[2];
        extrap_mode[0] = extrap_mode[1] = cDegree;
        // EvalBS over a plain array, so that it can be passed as an array start
        double[] evalBuffer = new double[cDimension * (derivative_request + 1)];
        math_Vector evalBS = new(evalBuffer, 1, cDimension * (derivative_request + 1));
        Span<double> eadr = evalBuffer;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(tbord,
                      periodic_flag,
                      derivative_request,
                      extrap_mode,
                      cDegree,
                      flatKnots,
                      cDimension,
                      poles,
                      eadr);

        //  norm of the tangent at the node of connection
        math_Vector tgte = new(1, cDimension);

        for (ipos = 1; ipos <= cDimension; ipos++)
        {
            tgte[ipos] = evalBS[ipos + cDimension];
        }
        double l1 = tgte.Norm();

        //  matrix of constraints
        Matrix contraintes = new(1, csize, 1, cDimension);
        // Precompute powers of C1Coefficient for efficiency (avoid std::pow overhead)
        double c1Coeff2 = c1Coefficient * c1Coefficient;
        double c1Coeff3 = c1Coeff2 * c1Coefficient;
        if (after)
        {
            for (ipos = 1; ipos <= cDimension; ipos++)
            {
                contraintes[1, ipos] = evalBS[ipos];
                contraintes[2, ipos] = c1Coefficient * evalBS[ipos + cDimension];
                if (continuity >= 2)
                {
                    contraintes[3, ipos] = evalBS[ipos + 2 * cDimension] * c1Coeff2;
                }
                if (continuity >= 3)
                {
                    contraintes[4, ipos] = evalBS[ipos + 3 * cDimension] * c1Coeff3;
                }
                contraintes[continuity + 2, ipos] = constraintPoint[ipos];
            }
        }
        else
        {
            for (ipos = 1; ipos <= cDimension; ipos++)
            {
                contraintes[1, ipos] = constraintPoint[ipos];
                contraintes[2, ipos] = evalBS[ipos];
                if (continuity >= 1)
                {
                    contraintes[3, ipos] = c1Coefficient * evalBS[ipos + cDimension];
                }
                if (continuity >= 2)
                {
                    contraintes[4, ipos] = evalBS[ipos + 2 * cDimension] * c1Coeff2;
                }
                if (continuity >= 3)
                {
                    contraintes[5, ipos] = evalBS[ipos + 3 * cDimension] * c1Coeff3;
                }
            }
        }

        //  calculate the coefficients of extension
        int ii, jj, kk;
        NCollection_Array1<double> extraCoeffs = new(1, csize * cDimension);
        extraCoeffs.Init(0.0);

        for (ii = 1; ii <= csize; ii++)
        {

            for (jj = 1; jj <= csize; jj++)
            {

                for (kk = 1; kk <= cDimension; kk++)
                {
                    extraCoeffs[kk + (jj - 1) * cDimension] += matCoefs[ii, jj] * contraintes[ii, kk];
                }
            }
        }

        //  calculate the poles of extension
        NCollection_Array1<double> extrapPoles = new(1, csize * cDimension);
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(cDimension,
                               extraCoeffs,
                               global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(),
                               extrapPoles,
                               global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());

        //  calculate the nodes of extension with multiplicities
        NCollection_Array1<double> extrapNoeuds = new(1, 2);
        extrapNoeuds[1] = 0.0;
        extrapNoeuds[2] = 1.0;
        NCollection_Array1<int> extrapMults = new(1, 2);
        extrapMults[1] = csize;
        extrapMults[2] = csize;

        // flat nodes of extension
        NCollection_Array1<double> fk2 = new(1, csize * 2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(extrapNoeuds, extrapMults, fk2);

        //  norm of the tangent at the connection point
        if (after)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(0.0, periodic_flag, 1, extrap_mode, csize - 1, fk2, cDimension, extrapPoles.AsSpan(), eadr);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(1.0, periodic_flag, 1, extrap_mode, csize - 1, fk2, cDimension, extrapPoles.AsSpan(), eadr);
        }

        for (ipos = 1; ipos <= cDimension; ipos++)
        {
            tgte[ipos] = evalBS[ipos + cDimension];
        }
        double l2 = tgte.Norm();

        //  harmonisation of degrees
        NCollection_Array1<double> newP2 = new(1, (cDegree + 1) * cDimension);
        NCollection_Array1<double> newK2 = new(1, 2);
        NCollection_Array1<int> newM2 = new(1, 2);
        if (csize - 1 < cDegree)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(csize - 1,
                                    cDegree,
                                    false,
                                    cDimension,
                                    extrapPoles,
                                    extrapNoeuds,
                                    extrapMults,
                                    newP2,
                                    newK2,
                                    newM2);
        }
        else
        {
            newP2.Assign(extrapPoles);
            newK2.Assign(extrapNoeuds);
            newM2.Assign(extrapMults);
        }

        //  flat nodes of extension after harmonization of degrees
        NCollection_Array1<double> newFK2 = new(1, (cDegree + 1) * 2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(newK2, newM2, newFK2);

        ////////////////////////////////////////////////////////////////////////
        //
        //    2.  concatenation C0
        //
        ////////////////////////////////////////////////////////////////////////

        //  ratio of reparametrization
        double ratio = 1, delta;
        if ((l1 > global::Kerf.Engine.Foundation.Precision.Confusion()) && (l2 > global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            ratio = l2 / l1;
        }
        if ((ratio < 1.0e-5) || (ratio > 1.0e5))
        {
            ratio = 1;
        }

        if (after)
        {
            //    do not touch the first BSpline
            delta = ratio * newFK2[newFK2.Lower()] - flatKnots[flatKnots.Upper()];
        }
        else
        {
            //    do not touch the second BSpline
            delta = ratio * newFK2[newFK2.Upper()] - flatKnots[flatKnots.Lower()];
        }

        //  result of the concatenation
        int nbP1 = numPoles, nbP2 = cDegree + 1;
        int nbK1 = flatKnots.Length(), nbK2 = 2 * (cDegree + 1);
        NCollection_Array1<double> newPoles = new(1, (nbP1 + nbP2 - 1) * cDimension);
        NCollection_Array1<double> newFlats = new(1, nbK1 + nbK2 - cDegree - 2);

        //  poles
        int indNP, indP, indEP;
        if (after)
        {

            for (ii = 1; ii <= nbP1 + nbP2 - 1; ii++)
            {

                for (jj = 1; jj <= cDimension; jj++)
                {
                    indNP = (ii - 1) * cDimension + jj;
                    indP = (ii - 1) * cDimension + jj - 1;
                    indEP = (ii - nbP1) * cDimension + jj;
                    if (ii < nbP1)
                    {
                        newPoles[indNP] = padr[indP];
                    }
                    else
                    {
                        newPoles[indNP] = newP2[indEP];
                    }
                }
            }
        }
        else
        {

            for (ii = 1; ii <= nbP1 + nbP2 - 1; ii++)
            {

                for (jj = 1; jj <= cDimension; jj++)
                {
                    indNP = (ii - 1) * cDimension + jj;
                    indEP = (ii - 1) * cDimension + jj;
                    indP = (ii - nbP2) * cDimension + jj - 1;
                    if (ii < nbP2)
                    {
                        newPoles[indNP] = newP2[indEP];
                    }
                    else
                    {
                        newPoles[indNP] = padr[indP];
                    }
                }
            }
        }

        //  flat nodes
        if (after)
        {
            //    start with the nodes of the initial surface

            for (ii = 1; ii < nbK1; ii++)
            {
                newFlats[ii] = flatKnots[flatKnots.Lower() + ii - 1];
            }
            //    continue with the reparameterized nodes of the extension

            for (ii = 1; ii <= nbK2 - cDegree - 1; ii++)
            {
                newFlats[nbK1 + ii - 1] = ratio * newFK2[newFK2.Lower() + ii + cDegree] - delta;
            }
        }
        else
        {
            //    start with the reparameterized nodes of the extension

            for (ii = 1; ii < nbK2 - cDegree; ii++)
            {
                newFlats[ii] = ratio * newFK2[newFK2.Lower() + ii - 1] - delta;
            }
            //    continue with the nodes of the initial surface

            for (ii = 2; ii <= nbK1; ii++)
            {
                newFlats[nbK2 + ii - cDegree - 2] = flatKnots[flatKnots.Lower() + ii - 1];
            }
        }

        ////////////////////////////////////////////////////////////////////////
        //
        //    3.  reduction of multiplicite at the node of connection
        //
        ////////////////////////////////////////////////////////////////////////

        //  number of separate nodes
        int kLength = 1;

        for (ii = 2; ii <= nbK1 + nbK2 - cDegree - 2; ii++)
        {
            if (newFlats[ii] != newFlats[ii - 1])
            {
                kLength++;
            }
        }

        //  flat nodes --> nodes + multiplicities
        NCollection_Array1<double> newKnots = new(1, kLength);
        NCollection_Array1<int> newMults = new(1, kLength);
        newMults.Init(1);
        jj = 1;
        newKnots[jj] = newFlats[1];

        for (ii = 2; ii <= nbK1 + nbK2 - cDegree - 2; ii++)
        {
            if (newFlats[ii] == newFlats[ii - 1])
            {
                newMults[jj]++;
            }
            else
            {
                jj++;
                newKnots[jj] = newFlats[ii];
            }
        }

        //  reduction of multiplicity at the second or the last but one node
        int index = 2, m = cDegree;
        if (after)
        {
            index = kLength - 1;
        }
        NCollection_Array1<double> resultPoles = new(1, (nbP1 + nbP2 - 1) * cDimension);
        NCollection_Array1<double> resultKnots = new(1, kLength);
        NCollection_Array1<int> resultMults = new(1, kLength);
        double tol = 1.0e-6;
        bool ok = true;

        while ((m > cDegree - continuity) && ok)
        {
            ok = RemoveKnot(index,
                            m - 1,
                            cDegree,
                            false,
                            cDimension,
                            newPoles,
                            newKnots,
                            newMults,
                            resultPoles,
                            resultKnots,
                            resultMults,
                            tol);
            if (ok)
            {
                m--;
            }
        }

        if (m == cDegree)
        {
            //    number of poles of the concatenation
            nbPolesResult = nbP1 + nbP2 - 1;
            //    the poles of the concatenation
            int pLength = nbPolesResult * cDimension;

            for (jj = 1; jj <= pLength; jj++)
            {
                pRadr[jj - 1] = newPoles[jj];
            }

            //    flat nodes of the concatenation
            int ideb = 0;

            for (jj = 0; jj < newKnots.Length(); jj++)
            {
                for (ii = 0; ii < newMults[jj + 1]; ii++)
                {
                    kRadr[ideb + ii] = newKnots[jj + 1];
                }
                ideb += newMults[jj + 1];
            }
            nbKnotsRsult = ideb;
        }

        else
        {
            //    number of poles of the result
            nbPolesResult = nbP1 + nbP2 - 1 - cDegree + m;
            //    the poles of the result
            int pLength = nbPolesResult * cDimension;

            for (jj = 0; jj < pLength; jj++)
            {
                pRadr[jj] = resultPoles[jj + 1];
            }

            //    flat nodes of the result
            int ideb = 0;

            for (jj = 0; jj < resultKnots.Length(); jj++)
            {
                for (ii = 0; ii < resultMults[jj + 1]; ii++)
                {
                    kRadr[ideb + ii] = resultKnots[jj + 1];
                }
                ideb += resultMults[jj + 1];
            }
            nbKnotsRsult = ideb;
        }
    }

    //=================================================================================================
    // Mathematical basis:
    //                           d
    //  Let C(t) = SUM      Ci Bi(t)  a Bspline curve of degree d
    //            i = 1,n
    //  with nodes tj for j = 1,n+d+1
    //
    //
    //         '                    C1 - Ci-1   d-1
    //  Then C (t) = SUM     d *  ---------  Bi (t)
    //                i = 2,n      ti+d - ti
    //
    //                          d-1
    //  for the base of BSpline  Bi  (t) of degree d-1.
    //
    //  Consequently the upper bound of the norm of the derivative from C is :
    //
    //
    //                        |  Ci - Ci-1  |
    //          d *   Max     |  ---------  |
    //              i = 2,n |  ti+d - ti  |
    //
    //                                  N(t)
    //  In the rational case set    C(t) = -----
    //                                  D(t)
    //
    //
    //  D(t) =  SUM    Di Bi(t)
    //        i=1,n
    //
    //  N(t) =  SUM   Di * Ci Bi(t)
    //          i =1,n
    //
    //          N'(t)  -    D'(t) C(t)
    //   C'(t) = -----------------------
    //                   D(t)
    //
    //
    //   N'(t) - D'(t) C(t) =
    //
    //                     Di * (Ci - C(t)) - Di-1 * (Ci-1 - C(t))    d-1
    //      SUM   d *   ---------------------------------------- * Bi  (t)  =
    //        i=2,n                   ti+d   - ti
    //
    //
    //                   Di * (Ci - Cj) - Di-1 * (Ci-1 - Cj)                d-1
    // SUM   SUM     d * -----------------------------------  * Betaj(t) * Bi  (t)
    // i=2,n j=1,n               ti+d  - ti
    //
    //
    //
    //                 Dj Bj(t)
    //    Betaj(t) =   --------
    //                 D(t)
    //
    //  Betaj(t) form a partition >= 0 of the entity with support
    //  tj, tj+d+1. Consequently if Rj = {j-d, ....,  j+d+d+1}
    //  obtain an upper bound of the derivative of C by taking :
    //
    //
    //                         Di * (Ci - Cj) - Di-1 * (Ci-1 - Cj)
    //   Max   Max       d  *  -----------------------------------
    // j=1,n  i dans Rj                   ti+d  - ti
    //
    //  --------------------------------------------------------
    //
    //               Min    Di
    //              i =1,n
    //
    //=================================================================================================

    /// <summary>
    /// Computes the parametric tolerance <paramref name="uTolerance"/> such that |t1 - t0| &lt; UTolerance implies
    /// |C(t1) - C(t0)| &lt; Tolerance3D, from an upper bound of the derivative of the curve (NumPoles poles of
    /// dimension ArrayDimension, optional weights, flat knots).
    /// </summary>
    public static void Resolution(ReadOnlySpan<double> polesArray, int arrayDimension, int numPoles,
                                  NCollection_Array1<double>? weights, NCollection_Array1<double> flatKnots, int degree,
                                  double tolerance3D, out double uTolerance)
    {
        int ii, num_poles, ii_index, jj_index, ii_inDim;
        int lower, upper, ii_minus, jj, ii_miDim;
        int deg1 = degree + 1;
        int deg2 = (degree << 1) + 1;
        double value, factor, w, min_weights, inverse;
        double pa_ii_inDim_0, pa_ii_inDim_1, pa_ii_inDim_2, pa_ii_inDim_3;
        double pa_ii_miDim_0, pa_ii_miDim_1, pa_ii_miDim_2, pa_ii_miDim_3;
        double wg_ii_index, wg_ii_minus;
        double max_derivative;
        ReadOnlySpan<double> fk = flatKnots.AsSpan();
        ReadOnlySpan<double> pa = polesArray;
        max_derivative = 0.0e0;
        num_poles = flatKnots.Length() - deg1;
        switch (arrayDimension)
        {
            case 2:
            {
                if (weights != null)
                {
                    ReadOnlySpan<double> wg = weights!.AsSpan();
                    min_weights = wg[0];

                    for (ii = 1; ii < numPoles; ii++)
                    {
                        w = wg[ii];
                        if (w < min_weights)
                        {
                            min_weights = w;
                        }
                    }

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % numPoles;
                        ii_inDim = ii_index << 1;
                        ii_minus = (ii - 1) % numPoles;
                        ii_miDim = ii_minus << 1;
                        pa_ii_inDim_0 = pa[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_1 = pa[ii_inDim];
                        pa_ii_miDim_0 = pa[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_1 = pa[ii_miDim];
                        wg_ii_index = wg[ii_index];
                        wg_ii_minus = wg[ii_minus];
                        inverse = fk[ii + degree] - fk[ii];
                        inverse = 1.0e0 / inverse;
                        lower = ii - deg1;
                        if (lower < 0)
                        {
                            lower = 0;
                        }
                        upper = deg2 + ii;
                        if (upper > num_poles)
                        {
                            upper = num_poles;
                        }

                        for (jj = lower; jj < upper; jj++)
                        {
                            jj_index = jj % numPoles;
                            jj_index = jj_index << 1;
                            value = 0.0e0;
                            factor = (((pa[jj_index] - pa_ii_inDim_0) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_0) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((pa[jj_index] - pa_ii_inDim_1) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_1) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            value *= inverse;
                            if (max_derivative < value)
                            {
                                max_derivative = value;
                            }
                        }
                    }
                    max_derivative /= min_weights;
                }
                else
                {

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % numPoles;
                        ii_index = ii_index << 1;
                        ii_minus = (ii - 1) % numPoles;
                        ii_minus = ii_minus << 1;
                        inverse = fk[ii + degree] - fk[ii];
                        inverse = 1.0e0 / inverse;
                        value = 0.0e0;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        value *= inverse;
                        if (max_derivative < value)
                        {
                            max_derivative = value;
                        }
                    }
                }
                break;
            }
            case 3:
            {
                if (weights != null)
                {
                    ReadOnlySpan<double> wg = weights!.AsSpan();
                    min_weights = wg[0];

                    for (ii = 1; ii < numPoles; ii++)
                    {
                        w = wg[ii];
                        if (w < min_weights)
                        {
                            min_weights = w;
                        }
                    }

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % numPoles;
                        ii_inDim = (ii_index << 1) + ii_index;
                        ii_minus = (ii - 1) % numPoles;
                        ii_miDim = (ii_minus << 1) + ii_minus;
                        pa_ii_inDim_0 = pa[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_1 = pa[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_2 = pa[ii_inDim];
                        pa_ii_miDim_0 = pa[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_1 = pa[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_2 = pa[ii_miDim];
                        wg_ii_index = wg[ii_index];
                        wg_ii_minus = wg[ii_minus];
                        inverse = fk[ii + degree] - fk[ii];
                        inverse = 1.0e0 / inverse;
                        lower = ii - deg1;
                        if (lower < 0)
                        {
                            lower = 0;
                        }
                        upper = deg2 + ii;
                        if (upper > num_poles)
                        {
                            upper = num_poles;
                        }

                        for (jj = lower; jj < upper; jj++)
                        {
                            jj_index = jj % numPoles;
                            jj_index = (jj_index << 1) + jj_index;
                            value = 0.0e0;
                            factor = (((pa[jj_index] - pa_ii_inDim_0) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_0) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((pa[jj_index] - pa_ii_inDim_1) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_1) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((pa[jj_index] - pa_ii_inDim_2) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_2) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            value *= inverse;
                            if (max_derivative < value)
                            {
                                max_derivative = value;
                            }
                        }
                    }
                    max_derivative /= min_weights;
                }
                else
                {

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % numPoles;
                        ii_index = (ii_index << 1) + ii_index;
                        ii_minus = (ii - 1) % numPoles;
                        ii_minus = (ii_minus << 1) + ii_minus;
                        inverse = fk[ii + degree] - fk[ii];
                        inverse = 1.0e0 / inverse;
                        value = 0.0e0;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        value *= inverse;
                        if (max_derivative < value)
                        {
                            max_derivative = value;
                        }
                    }
                }
                break;
            }
            case 4:
            {
                if (weights != null)
                {
                    ReadOnlySpan<double> wg = weights!.AsSpan();
                    min_weights = wg[0];

                    for (ii = 1; ii < numPoles; ii++)
                    {
                        w = wg[ii];
                        if (w < min_weights)
                        {
                            min_weights = w;
                        }
                    }

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % numPoles;
                        ii_inDim = ii_index << 2;
                        ii_minus = (ii - 1) % numPoles;
                        ii_miDim = ii_minus << 2;
                        pa_ii_inDim_0 = pa[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_1 = pa[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_2 = pa[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_3 = pa[ii_inDim];
                        pa_ii_miDim_0 = pa[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_1 = pa[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_2 = pa[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_3 = pa[ii_miDim];
                        wg_ii_index = wg[ii_index];
                        wg_ii_minus = wg[ii_minus];
                        inverse = fk[ii + degree] - fk[ii];
                        inverse = 1.0e0 / inverse;
                        lower = ii - deg1;
                        if (lower < 0)
                        {
                            lower = 0;
                        }
                        upper = deg2 + ii;
                        if (upper > num_poles)
                        {
                            upper = num_poles;
                        }

                        for (jj = lower; jj < upper; jj++)
                        {
                            jj_index = jj % numPoles;
                            jj_index = jj_index << 2;
                            value = 0.0e0;
                            factor = (((pa[jj_index] - pa_ii_inDim_0) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_0) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((pa[jj_index] - pa_ii_inDim_1) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_1) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((pa[jj_index] - pa_ii_inDim_2) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_2) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((pa[jj_index] - pa_ii_inDim_3) * wg_ii_index)
                                                - ((pa[jj_index] - pa_ii_miDim_3) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            value *= inverse;
                            if (max_derivative < value)
                            {
                                max_derivative = value;
                            }
                        }
                    }
                    max_derivative /= min_weights;
                }
                else
                {

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % numPoles;
                        ii_index = ii_index << 2;
                        ii_minus = (ii - 1) % numPoles;
                        ii_minus = ii_minus << 2;
                        inverse = fk[ii + degree] - fk[ii];
                        inverse = 1.0e0 / inverse;
                        value = 0.0e0;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = pa[ii_index] - pa[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        value *= inverse;
                        if (max_derivative < value)
                        {
                            max_derivative = value;
                        }
                    }
                }
                break;
            }
            default:
            {
                int kk;
                if (weights != null)
                {
                    ReadOnlySpan<double> wg = weights!.AsSpan();
                    min_weights = wg[0];

                    for (ii = 1; ii < numPoles; ii++)
                    {
                        w = wg[ii];
                        if (w < min_weights)
                        {
                            min_weights = w;
                        }
                    }

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % numPoles;
                        ii_inDim = ii_index * arrayDimension;
                        ii_minus = (ii - 1) % numPoles;
                        ii_miDim = ii_minus * arrayDimension;
                        wg_ii_index = wg[ii_index];
                        wg_ii_minus = wg[ii_minus];
                        inverse = fk[ii + degree] - fk[ii];
                        inverse = 1.0e0 / inverse;
                        lower = ii - deg1;
                        if (lower < 0)
                        {
                            lower = 0;
                        }
                        upper = deg2 + ii;
                        if (upper > num_poles)
                        {
                            upper = num_poles;
                        }

                        for (jj = lower; jj < upper; jj++)
                        {
                            jj_index = jj % numPoles;
                            jj_index *= arrayDimension;
                            value = 0.0e0;

                            for (kk = 0; kk < arrayDimension; kk++)
                            {
                                factor = (((pa[jj_index + kk] - pa[ii_inDim + kk]) * wg_ii_index)
                                                    - ((pa[jj_index + kk] - pa[ii_miDim + kk]) * wg_ii_minus));
                                if (factor < 0)
                                {
                                    factor = -factor;
                                }
                                value += factor;
                            }
                            value *= inverse;
                            if (max_derivative < value)
                            {
                                max_derivative = value;
                            }
                        }
                    }
                    max_derivative /= min_weights;
                }
                else
                {

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % numPoles;
                        ii_index *= arrayDimension;
                        ii_minus = (ii - 1) % numPoles;
                        ii_minus *= arrayDimension;
                        inverse = fk[ii + degree] - fk[ii];
                        inverse = 1.0e0 / inverse;
                        value = 0.0e0;

                        for (kk = 0; kk < arrayDimension; kk++)
                        {
                            factor = pa[ii_index + kk] - pa[ii_minus + kk];
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                        }
                        value *= inverse;
                        if (max_derivative < value)
                        {
                            max_derivative = value;
                        }
                    }
                }
                break;
            }
        }
        max_derivative *= degree;
        if (max_derivative > RealSmall())
        {
            uTolerance = tolerance3D / max_derivative;
        }
        else
        {
            uTolerance = tolerance3D / RealSmall();
        }
    }

    /// <summary>
    /// Computes the intervals of continuity <paramref name="continuity"/> of the curve within
    /// [theFirst, theLast] and returns their number; fills <paramref name="intervals"/> (resized to the number
    /// + 1, the ends being theFirst and theLast) unless null.
    /// </summary>
    public static int Intervals(NCollection_Array1<double> knots, NCollection_Array1<int> mults, int degree,
                                bool isPeriodic, int continuity, double first, double last,
                                double tolerance, NCollection_Array1<double>? intervals)
    {
        // remove all knots with multiplicity less or equal than (degree - continuity) except first and
        // last
        int firstIndex = isPeriodic ? 1 : FirstUKnotIndex(degree, mults);
        int lastIndex = isPeriodic ? knots.Length() : LastUKnotIndex(degree, mults);
        NCollection_Array1<double> newKnots = new(1, lastIndex - firstIndex + 1);
        int nbNewKnots = 0;
        for (int index = firstIndex; index <= lastIndex; index++)
        {
            if (mults[index] > (degree - continuity) || index == firstIndex
                || index == lastIndex)
            {
                nbNewKnots++;
                newKnots[nbNewKnots] = knots[index];
            }
        }
        newKnots.Resize(1, nbNewKnots, true);

        // the range boundaries
        double curFirst = first;
        double curLast = last;
        double period = 0.0;
        int firstPeriod = 0;
        int lastPeriod = 0;
        // move boundaries into period
        if (isPeriodic)
        {
            double lower = knots.First();
            double upper = knots.Last();
            period = upper - lower;

            while (curFirst < lower)
            {
                curFirst += period;
                firstPeriod--;
            }
            while (curLast < lower)
            {
                curLast += period;
                lastPeriod--;
            }
            while (curFirst >= upper)
            {
                curFirst -= period;
                firstPeriod += 1;
            }
            while (curLast >= upper)
            {
                curLast -= period;
                lastPeriod += 1;
            }
        }
        // locate the left and nearest knot for boundaries
        int index1 = 0;
        int index2 = 0;
        double dummyDouble;
        // we use version of LocateParameter that doesn't need multiplicities
        LocateParameter(degree,
                        newKnots,
                        new NCollection_Array1<int>(),
                        curFirst,
                        false,
                        1,
                        nbNewKnots,
                        out index1,
                        out dummyDouble);
        LocateParameter(degree,
                        newKnots,
                        new NCollection_Array1<int>(),
                        curLast,
                        false,
                        1,
                        nbNewKnots,
                        out index2,
                        out dummyDouble);
        // the case when the beginning of the range coincides with the next knot
        if (index1 < nbNewKnots && Math.Abs(newKnots[index1 + 1] - curFirst) < tolerance)
        {
            index1 += 1;
        }
        // the case when the ending of the range coincides with the current knot
        if (nbNewKnots != 0 && Math.Abs(newKnots[index2] - curLast) < tolerance)
        {
            index2 -= 1;
        }
        int nbIntervals = index2 - index1 + 1 + (lastPeriod - firstPeriod) * (nbNewKnots - 1);

        // fill the interval array
        if (intervals != null)
        {
            intervals.Resize(1, nbIntervals + 1, false);
            if (isPeriodic && lastPeriod != firstPeriod)
            {
                int index = 1;
                // part from the begging of range to the end of the first period
                for (int i = index1; i < newKnots.Length(); i++, index++)
                {
                    intervals.ChangeValue(index) = newKnots[i] + firstPeriod * period;
                }
                // full periods
                for (int periodNum = firstPeriod + 1; periodNum < lastPeriod; periodNum++)
                {
                    for (int i = 1; i < newKnots.Length(); i++, index++)
                    {
                        intervals.ChangeValue(index) = newKnots[i] + periodNum * period;
                    }
                }
                // part from the begging of the last period to the end of range
                for (int i = 1; i <= index2; i++, index++)
                {
                    intervals.ChangeValue(index) = newKnots[i] + lastPeriod * period;
                }
            }
            else
            {
                int index = 1;
                for (int i = index1; i <= index2; i++, index++)
                {
                    intervals.ChangeValue(index) = newKnots[i] + firstPeriod * period;
                }
            }
            // update the first position (the begging of range doesn't coincide with the knot at anIndex1 in
            // general)
            intervals.ChangeValue(1) = first;
            // write the ending of the range (we didn't write it at all)
            intervals.ChangeValue(nbIntervals + 1) = last;
        }

        return nbIntervals;
    }

    // array of flat knots for bezier curve of maximum 25 degree
    private static ReadOnlySpan<double> knots =>
    [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    ];

    /// <summary>
    /// The flat knots of a Bezier curve of degree <paramref name="degree"/> (1 to 25): Degree + 1 zeros then
    /// Degree + 1 ones. OCCT returns a reference to the start of a static table; the port returns the
    /// 2 * (Degree + 1) values as a read-only span over it. OCCT's
    /// <c>NCollection_Array1&lt;double&gt; knots(BSplCLib::FlatBezierKnots(Degree), 1, 2 * (Degree + 1))</c> becomes
    /// <c>NCollection_Array1&lt;double&gt;.From(BSplCLib.FlatBezierKnots(Degree))</c> (a copy).
    /// </summary>
    public static ReadOnlySpan<double> FlatBezierKnots(int degree)
    {
        Standard_OutOfRange_Raise_if(degree < 1 || degree > MaxDegree() || MaxDegree() != 25,
                                     "Bezier curve degree greater than maximal supported");

        return knots.Slice(25 - degree, 2 * (degree + 1));
    }

    private static readonly double[] THE_UNIT_WEIGHTS = CreateUnitWeights();

    private static double[] CreateUnitWeights()
    {
        double[] data = new double[2049];
        for (int i = 0; i < 2049; ++i)
        {
            data[i] = 1.0;
        }
        return data;
    }

    /// <summary>
    /// An array of <paramref name="nbElems"/> weights equal to 1, indexed from 1. Up to
    /// <see cref="MaxUnitWeightsSize"/> elements it is a non-owning view of a shared static array (no allocation),
    /// as in OCCT: shared, don't modify. Above that it is a new array.
    /// </summary>
    public static NCollection_Array1<double> UnitWeights(int nbElems)
    {
        if (nbElems <= MaxUnitWeightsSize())
        {
            // Non-owning view over global static array (zero allocation).
            return new NCollection_Array1<double>(THE_UNIT_WEIGHTS, 0, 1, nbElems);
        }
        // Rare: allocate fresh array.
        NCollection_Array1<double> result = new(1, nbElems);
        result.Init(1.0);
        return result;
    }

    /// <summary>The shared unit weights: <see cref="MaxUnitWeightsSize"/> values equal to 1, read-only.</summary>
    public static ReadOnlySpan<double> UnitWeightsData() => THE_UNIT_WEIGHTS;

    /// <summary>Evaluates a Bezier curve given by its poles in the cache (power-basis) form, on [0, 1].</summary>
    public static void CoefsD0(double u, NCollection_Array1<Point3D> poles, NCollection_Array1<double>? weights, out Point3D p)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.CacheD0(u, poles.Length() - 1, 0.0, 1.0, poles, weights, out p);
    }

    /// <inheritdoc cref="CoefsD0(double, NCollection_Array1{Point3D}, NCollection_Array1{double}?, out Point3D)"/>
    public static void CoefsD0(double u, NCollection_Array1<Point2D> poles, NCollection_Array1<double>? weights, out Point2D p)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.CacheD0(u, poles.Length() - 1, 0.0, 1.0, poles, weights, out p);
    }

    /// <summary>Evaluates a cached Bezier curve and its first derivative on [0, 1].</summary>
    public static void CoefsD1(double u, NCollection_Array1<Point3D> poles, NCollection_Array1<double>? weights, out Point3D p,
                               out Vector3D vec)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.CacheD1(u, poles.Length() - 1, 0.0, 1.0, poles, weights, out p, out vec);
    }

    /// <inheritdoc cref="CoefsD1(double, NCollection_Array1{Point3D}, NCollection_Array1{double}?, out Point3D, out Vector3D)"/>
    public static void CoefsD1(double u, NCollection_Array1<Point2D> poles, NCollection_Array1<double>? weights,
                               out Point2D p, out Vector2D vec)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.CacheD1(u, poles.Length() - 1, 0.0, 1.0, poles, weights, out p, out vec);
    }

    /// <summary>Evaluates a cached Bezier curve and its first two derivatives on [0, 1].</summary>
    public static void CoefsD2(double u, NCollection_Array1<Point3D> poles, NCollection_Array1<double>? weights, out Point3D p,
                               out Vector3D vec1, out Vector3D vec2)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.CacheD2(u, poles.Length() - 1, 0.0, 1.0, poles, weights, out p, out vec1, out vec2);
    }

    /// <inheritdoc cref="CoefsD2(double, NCollection_Array1{Point3D}, NCollection_Array1{double}?, out Point3D, out Vector3D, out Vector3D)"/>
    public static void CoefsD2(double u, NCollection_Array1<Point2D> poles, NCollection_Array1<double>? weights,
                               out Point2D p, out Vector2D vec1, out Vector2D vec2)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.CacheD2(u, poles.Length() - 1, 0.0, 1.0, poles, weights, out p, out vec1, out vec2);
    }

    /// <summary>Evaluates a cached Bezier curve and its first three derivatives on [0, 1].</summary>
    public static void CoefsD3(double u, NCollection_Array1<Point3D> poles, NCollection_Array1<double>? weights, out Point3D p,
                               out Vector3D vec1, out Vector3D vec2, out Vector3D vec3)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.CacheD3(u, poles.Length() - 1, 0.0, 1.0, poles, weights, out p, out vec1, out vec2, out vec3);
    }

    /// <inheritdoc cref="CoefsD3(double, NCollection_Array1{Point3D}, NCollection_Array1{double}?, out Point3D, out Vector3D, out Vector3D, out Vector3D)"/>
    public static void CoefsD3(double u, NCollection_Array1<Point2D> poles, NCollection_Array1<double>? weights,
                               out Point2D p, out Vector2D vec1, out Vector2D vec2, out Vector2D vec3)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.CacheD3(u, poles.Length() - 1, 0.0, 1.0, poles, weights, out p, out vec1, out vec2, out vec3);
    }

    /// <summary>Converts the poles of a Bezier curve into its cache (power-basis) coefficients.</summary>
    public static void PolesCoefficients(NCollection_Array1<Point3D> poles, NCollection_Array1<Point3D> cachePoles)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.PolesCoefficients(poles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), cachePoles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
    }

    /// <inheritdoc cref="PolesCoefficients(NCollection_Array1{Point3D}, NCollection_Array1{Point3D})"/>
    public static void PolesCoefficients(NCollection_Array1<Point2D> poles, NCollection_Array1<Point2D> cachePoles)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.PolesCoefficients(poles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), cachePoles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
    }
}
