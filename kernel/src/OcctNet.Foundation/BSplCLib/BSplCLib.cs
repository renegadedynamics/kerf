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

using static Occt.BSplCLib_KnotDistribution;
using static Occt.BSplCLib_MultDistribution;
using static Occt.GeomAbs_BSplKnotDistribution;

namespace Occt;

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
    private static math_Matrix BSplCLib_LocalMatrix(int DerivativeRequest, int Order)
    {
        Standard_OutOfRange_Raise_if(DerivativeRequest > BSplCLib.MaxDegree()
                                       || Order > BSplCLib.MaxDegree() + 1
                                       || BSplCLib.MaxDegree() > 25,
                                     "BSplCLib: bspline degree is greater than maximum supported");
        return new math_Matrix(1, DerivativeRequest + 1, 1, Order);
    }

    /// <summary>
    /// Searches the position of <paramref name="theX"/> in the increasing values <paramref name="theArray"/> by
    /// bisection. Out of range, returns theArray.Lower()-1 or theArray.Upper()+1. Used to locate a knot.
    /// </summary>
    public static void Hunt(NCollection_Array1<double> theArray, double theX, out int theXPos)
    {
        // replaced by simple dichotomy (RLE)
        if (theArray.First() > theX)
        {
            theXPos = theArray.Lower() - 1;
            return;
        }
        else if (theArray.Last() < theX)
        {
            theXPos = theArray.Upper() + 1;
            return;
        }

        theXPos = theArray.Lower();
        if (theArray.Length() <= 1)
        {
            return;
        }

        int aHi = theArray.Upper();
        while (aHi - theXPos != 1)
        {
            int aMid = (aHi + theXPos) / 2;
            if (theArray.Value(aMid) < theX)
            {
                theXPos = aMid;
            }
            else
            {
                aHi = aMid;
            }
        }
    }

    /// <summary>The index of the knot of the first parameter (first knot whose cumulated multiplicity exceeds Degree).</summary>
    public static int FirstUKnotIndex(int Degree, NCollection_Array1<int> Mults)
    {
        int Index = Mults.Lower();
        int SigmaMult = Mults[Index];

        while (SigmaMult <= Degree)
        {
            Index++;
            SigmaMult += Mults[Index];
        }
        return Index;
    }

    /// <summary>The index of the knot of the last parameter.</summary>
    public static int LastUKnotIndex(int Degree, NCollection_Array1<int> Mults)
    {
        int Index = Mults.Upper();
        int SigmaMult = Mults[Index];

        while (SigmaMult <= Degree)
        {
            Index--;
            SigmaMult += Mults.Value(Index);
        }
        return Index;
    }

    /// <summary>Computes the index of the knot <paramref name="Index"/> in the flat knot sequence.</summary>
    public static int FlatIndex(int Degree, int Index, NCollection_Array1<int> Mults, bool Periodic)
    {
        int i, index = Index;
        int MLower = Mults.Lower();
        ReadOnlySpan<int> pmu = Mults.AsSpan(); // pmu[i - MLower]

        for (i = MLower + 1; i <= Index; i++)
        {
            index += pmu[i - MLower] - 1;
        }
        if (Periodic)
        {
            index += Degree;
        }
        else
        {
            index += pmu[MLower - MLower] - 1;
        }
        return index;
    }

    /// <summary>
    /// Locates the parametric value <paramref name="U"/> in the knot sequence between the knots FromK1 and ToK2:
    /// Knots(KnotIndex) &lt;= U &lt; Knots(KnotIndex + 1); for a periodic curve, U is first brought into the
    /// period (NewU).
    /// </summary>
    public static void LocateParameter(int Degree, NCollection_Array1<double> Knots, NCollection_Array1<int> Mults,
                                       double U, bool IsPeriodic, int FromK1, int ToK2, out int KnotIndex,
                                       out double NewU)
    {
        double uf = 0, ul = 1;
        if (IsPeriodic)
        {
            uf = Knots[Knots.Lower()];
            ul = Knots[Knots.Upper()];
        }
        BSplCLib.LocateParameter(Knots, U, IsPeriodic, FromK1, ToK2, out KnotIndex, out NewU, uf, ul);
    }

    /// <summary>
    /// Locates the parametric value <paramref name="U"/> in the flat knot sequence between the knots FromK1 and
    /// ToK2; for a periodic curve the period runs from Knots(Lower + Degree) to Knots(Upper - Degree).
    /// </summary>
    public static void LocateParameter(int Degree, NCollection_Array1<double> Knots, double U, bool IsPeriodic,
                                       int FromK1, int ToK2, out int KnotIndex, out double NewU)
    {
        if (IsPeriodic)
        {
            BSplCLib.LocateParameter(Knots,
                                     U,
                                     IsPeriodic,
                                     FromK1,
                                     ToK2,
                                     out KnotIndex,
                                     out NewU,
                                     Knots[Knots.Lower() + Degree],
                                     Knots[Knots.Upper() - Degree]);
        }
        else
        {
            BSplCLib.LocateParameter(Knots, U, IsPeriodic, FromK1, ToK2, out KnotIndex, out NewU, 0.0, 1.0);
        }
    }

    private static void LocateParameter(NCollection_Array1<double> Knots, double U, bool IsPeriodic, int FromK1,
                                        int ToK2, out int KnotIndex, out double NewU, double UFirst, double ULast)
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

        int First, Last;
        if (FromK1 < ToK2)
        {
            First = FromK1;
            Last = ToK2;
        }
        else
        {
            First = ToK2;
            Last = FromK1;
        }
        int Last1 = Last - 1;
        NewU = U;
        if (IsPeriodic && (NewU < UFirst || NewU > ULast))
        {
            NewU = ElCLib.InPeriod(NewU, UFirst, ULast);
        }

        BSplCLib.Hunt(Knots, NewU, out KnotIndex);

        double val;
        int KLower = Knots.Lower(), KUpper = Knots.Upper();

        double Eps = Epsilon(Min(Math.Abs(Knots[KUpper]), Math.Abs(U)));

        ReadOnlySpan<double> knots = Knots.AsSpan(); // knots[i - KLower]
        if (KnotIndex < Knots.Upper())
        {
            val = NewU - knots[KnotIndex + 1 - KLower];
            if (val < 0)
            {
                val = -val;
            }
            // <= to be coherent with Segment where Eps corresponds to a bit of error.
            if (val <= Eps)
            {
                KnotIndex++;
            }
        }
        if (KnotIndex < First)
        {
            KnotIndex = First;
        }
        if (KnotIndex > Last1)
        {
            KnotIndex = Last1;
        }

        if (KnotIndex != Last1)
        {
            double K1 = knots[KnotIndex - KLower];
            double K2 = knots[KnotIndex + 1 - KLower];
            val = K2 - K1;
            if (val < 0)
            {
                val = -val;
            }

            while (val <= Eps)
            {
                KnotIndex++;

                if (KnotIndex >= Knots.Upper())
                {
                    break;
                }

                K1 = K2;
                K2 = knots[KnotIndex + 1 - KLower];
                val = K2 - K1;
                if (val < 0)
                {
                    val = -val;
                }
            }
        }
    }

    /// <summary>
    /// Locates <paramref name="U"/> in the knots, unless <paramref name="Index"/> already designates a knot of the
    /// curve's range (then NewU = U). Without multiplicities the knots are flat knots.
    /// </summary>
    public static void LocateParameter(int Degree, NCollection_Array1<double> Knots, NCollection_Array1<int>? Mults,
                                       double U, bool Periodic, ref int Index, out double NewU)
    {
        int KnotIndex = Index;
        int first, last;
        if (Mults != null)
        {
            if (Periodic)
            {
                first = Knots.Lower();
                last = Knots.Upper();
            }
            else
            {
                first = FirstUKnotIndex(Degree, Mults);
                last = LastUKnotIndex(Degree, Mults);
            }
        }
        else
        {
            first = Knots.Lower() + Degree;
            last = Knots.Upper() - Degree;
        }
        if (KnotIndex < first || KnotIndex > last)
        {
            BSplCLib.LocateParameter(Knots,
                                     U,
                                     Periodic,
                                     first,
                                     last,
                                     out KnotIndex,
                                     out NewU,
                                     Knots[first],
                                     Knots[last]);
        }
        else
        {
            NewU = U;
        }
        Index = KnotIndex;
    }

    /// <summary>The greatest multiplicity in Mults between FromK1 and ToK2.</summary>
    public static int MaxKnotMult(NCollection_Array1<int> Mults, int FromK1, int ToK2)
    {
        int aMaxMult = Mults[FromK1];
        for (int anIndex = FromK1 + 1; anIndex <= ToK2; ++anIndex)
        {
            aMaxMult = Max(aMaxMult, Mults[anIndex]);
        }
        return aMaxMult;
    }

    /// <summary>The smallest multiplicity in Mults between FromK1 and ToK2.</summary>
    public static int MinKnotMult(NCollection_Array1<int> Mults, int FromK1, int ToK2)
    {
        int aMinMult = Mults[FromK1];
        for (int anIndex = FromK1 + 1; anIndex <= ToK2; ++anIndex)
        {
            aMinMult = Min(aMinMult, Mults[anIndex]);
        }
        return aMinMult;
    }

    /// <summary>
    /// The number of poles of the curve, or 0 if the multiplicities are inconsistent (a multiplicity &lt;= 0 or
    /// greater than Degree inside, ends greater than Degree + 1, or unequal ends of a periodic curve).
    /// </summary>
    public static int NbPoles(int Degree, bool Periodic, NCollection_Array1<int> Mults)
    {
        int f = Mults.Lower();
        int l = Mults.Upper();
        ReadOnlySpan<int> pmu = Mults.AsSpan();
        int Mf = pmu[0];
        int Ml = pmu[l - f];
        if (Mf <= 0)
        {
            return 0;
        }
        if (Ml <= 0)
        {
            return 0;
        }
        int sigma;
        if (Periodic)
        {
            if (Mf > Degree)
            {
                return 0;
            }
            if (Ml > Degree)
            {
                return 0;
            }
            if (Mf != Ml)
            {
                return 0;
            }
            sigma = Mf;
        }
        else
        {
            int Deg1 = Degree + 1;
            if (Mf > Deg1)
            {
                return 0;
            }
            if (Ml > Deg1)
            {
                return 0;
            }
            sigma = Mf + Ml - Deg1;
        }

        for (int i = 1; i < l - f; i++)
        {
            if (pmu[i] <= 0)
            {
                return 0;
            }
            if (pmu[i] > Degree)
            {
                return 0;
            }
            sigma += pmu[i];
        }
        return sigma;
    }

    /// <summary>The length of the flat knot sequence (the sum of the multiplicities, plus the periodic extension).</summary>
    public static int KnotSequenceLength(NCollection_Array1<int> Mults, int Degree, bool Periodic)
    {
        int i, l = 0;
        int MLower = Mults.Lower();
        int MUpper = Mults.Upper();
        ReadOnlySpan<int> pmu = Mults.AsSpan(); // pmu[i - MLower]

        for (i = MLower; i <= MUpper; i++)
        {
            l += pmu[i - MLower];
        }
        if (Periodic)
        {
            l += 2 * (Degree + 1 - pmu[0]);
        }
        return l;
    }

    /// <summary>Computes the flat knot sequence (each knot repeated by its multiplicity) into KnotSeq.</summary>
    public static void KnotSequence(NCollection_Array1<double> Knots, NCollection_Array1<int> Mults,
                                    NCollection_Array1<double> KnotSeq, bool Periodic = false)
    {
        BSplCLib.KnotSequence(Knots, Mults, 0, Periodic, KnotSeq);
    }

    /// <summary>Computes the flat knot sequence into KnotSeq; for a periodic curve, extended by the period on both sides.</summary>
    public static void KnotSequence(NCollection_Array1<double> Knots, NCollection_Array1<int> Mults, int Degree,
                                    bool Periodic, NCollection_Array1<double> KnotSeq)
    {
        double K;
        int Mult;
        int MLower = Mults.Lower();
        ReadOnlySpan<int> pmu = Mults.AsSpan(); // pmu[i - MLower]
        int KLower = Knots.Lower();
        int KUpper = Knots.Upper();
        ReadOnlySpan<double> pkn = Knots.AsSpan(); // pkn[i - KLower]
        int M1 = Degree + 1 - pmu[0]; // for periodic
        int i, j, index = Periodic ? M1 + 1 : 1;

        for (i = KLower; i <= KUpper; i++)
        {
            Mult = pmu[i - MLower];
            K = pkn[i - KLower];

            for (j = 1; j <= Mult; j++)
            {
                KnotSeq[index] = K;
                index++;
            }
        }
        if (Periodic)
        {
            double period = pkn[KUpper - KLower] - pkn[0];
            int m;
            m = 1;
            j = KUpper - 1;

            for (i = M1; i >= 1; i--)
            {
                KnotSeq[i] = pkn[j - KLower] - period;
                m++;
                if (m > pmu[j - MLower])
                {
                    j--;
                    m = 1;
                }
            }
            m = 1;
            j = KLower + 1;

            for (i = index; i <= KnotSeq.Upper(); i++)
            {
                KnotSeq[i] = pkn[j - KLower] + period;
                m++;
                if (m > pmu[j - MLower])
                {
                    j++;
                    m = 1;
                }
            }
        }
    }

    /// <summary>The number of distinct knots in a flat knot sequence (indexed from 1).</summary>
    public static int KnotsLength(NCollection_Array1<double> KnotSeq, bool Periodic = false)
    {
        NCollection_Array1<double> SeqKnots = KnotSeq;
        int sizeMult = 1;
        double val = SeqKnots[1];
        for (int jj = 2; jj <= SeqKnots.Length(); jj++)
        {
            // test on strict equality on nodes
            if (SeqKnots[jj] != val)
            {
                val = SeqKnots[jj];
                sizeMult++;
            }
        }
        return sizeMult;
    }

    /// <summary>Computes the knots and multiplicities (indexed from 1) of a flat knot sequence (indexed from 1).</summary>
    public static void Knots(NCollection_Array1<double> KnotSeq, NCollection_Array1<double> Knots,
                             NCollection_Array1<int> Mults, bool Periodic = false)
    {
        NCollection_Array1<double> SeqKnots = KnotSeq;
        NCollection_Array1<double> knots = Knots;
        NCollection_Array1<int> mult = Mults;
        double val = SeqKnots[1];
        int kk = 1;
        knots[kk] = val;
        mult[kk] = 1;

        for (int jj = 2; jj <= SeqKnots.Length(); jj++)
        {
            // test on strict equality on nodes
            if (SeqKnots[jj] != val)
            {
                val = SeqKnots[jj];
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
    public static BSplCLib_KnotDistribution KnotForm(NCollection_Array1<double> Knots, int FromK1, int ToK2)
    {
        if (FromK1 + 1 > Knots.Upper())
        {
            return BSplCLib_Uniform;
        }

        double aUi = Math.Abs(Knots[FromK1]);
        double aUj = Math.Abs(Knots[FromK1 + 1]);
        double aDU0 = Math.Abs(aUj - aUi);
        double anEps = Epsilon(aUi) + Epsilon(aUj) + Epsilon(aDU0);

        for (int i = FromK1 + 1; i < ToK2; i++)
        {
            aUi = Math.Abs(Knots[i]);
            aUj = Math.Abs(Knots[i + 1]);
            double aDU1 = Math.Abs(aUj - aUi);

            if (Math.Abs(aDU1 - aDU0) > anEps)
            {
                return BSplCLib_NonUniform;
            }

            aDU0 = aDU1;
            anEps = Epsilon(aUi) + Epsilon(aUj) + Epsilon(aDU0);
        }

        return BSplCLib_Uniform;
    }

    /// <summary>
    /// Analyses the distribution of the multiplicities between FromK1 and ToK2: constant, quasi-constant
    /// (constant except the first and last) or non-constant.
    /// </summary>
    public static BSplCLib_MultDistribution MultForm(NCollection_Array1<int> Mults, int FromK1, int ToK2)
    {
        int aFirst = Min(FromK1, ToK2), aLast = Max(FromK1, ToK2);

        if (aFirst + 1 > Mults.Upper())
        {
            return BSplCLib_Constant;
        }

        int aFirstMult = Mults[aFirst];
        BSplCLib_MultDistribution aForm = BSplCLib_Constant;
        int aMult = Mults[aFirst + 1];

        for (int i = aFirst + 1; i <= aLast && aForm != BSplCLib_NonConstant; i++)
        {
            if (i == aFirst + 1)
            {
                if (aMult != aFirstMult)
                {
                    aForm = BSplCLib_QuasiConstant;
                }
            }
            else if (i == aLast)
            {
                if (aForm == BSplCLib_QuasiConstant)
                {
                    if (aFirstMult != Mults[i])
                    {
                        aForm = BSplCLib_NonConstant;
                    }
                }
                else
                {
                    if (aMult != Mults[i])
                    {
                        aForm = BSplCLib_NonConstant;
                    }
                }
            }
            else
            {
                if (aMult != Mults[i])
                {
                    aForm = BSplCLib_NonConstant;
                }
                aMult = Mults[i];
            }
        }

        return aForm;
    }

    /// <summary>
    /// Analyses the knots and multiplicities of a curve: its knot distribution (uniform, quasi-uniform,
    /// piecewise Bezier or non-uniform) and the greatest multiplicity of its interior knots.
    /// </summary>
    public static void KnotAnalysis(int Degree, bool Periodic, NCollection_Array1<double> CKnots,
                                    NCollection_Array1<int> CMults, out GeomAbs_BSplKnotDistribution KnotForm,
                                    out int MaxKnotMult)
    {
        KnotForm = GeomAbs_NonUniform;

        BSplCLib_KnotDistribution KSet = BSplCLib.KnotForm(CKnots, 1, CKnots.Length());

        if (KSet == BSplCLib_Uniform)
        {
            BSplCLib_MultDistribution MSet = BSplCLib.MultForm(CMults, 1, CMults.Length());
            switch (MSet)
            {
                case BSplCLib_NonConstant:
                    break;
                case BSplCLib_Constant:
                    if (CKnots.Length() == 2)
                    {
                        KnotForm = GeomAbs_PiecewiseBezier;
                    }
                    else
                    {
                        if (CMults[1] == 1)
                        {
                            KnotForm = GeomAbs_Uniform;
                        }
                    }
                    break;
                case BSplCLib_QuasiConstant:
                    if (CMults[1] == Degree + 1)
                    {
                        double M = CMults[2];
                        if (M == Degree)
                        {
                            KnotForm = GeomAbs_PiecewiseBezier;
                        }
                        else if (M == 1)
                        {
                            KnotForm = GeomAbs_QuasiUniform;
                        }
                    }
                    break;
            }
        }

        int FirstKM = Periodic ? CKnots.Lower() : BSplCLib.FirstUKnotIndex(Degree, CMults);
        int LastKM = Periodic ? CKnots.Upper() : BSplCLib.LastUKnotIndex(Degree, CMults);
        MaxKnotMult = 0;
        if (LastKM - FirstKM != 1)
        {
            int Multi;
            for (int i = FirstKM + 1; i < LastKM; i++)
            {
                Multi = CMults[i];
                MaxKnotMult = Max(MaxKnotMult, Multi);
            }
        }
    }

    /// <summary>Reparametrises the knots to the range [U1, U2] (in either order), keeping their relative spacing.</summary>
    public static void Reparametrize(double U1, double U2, NCollection_Array1<double> Knots)
    {
        int Lower = Knots.Lower();
        int Upper = Knots.Upper();
        double UFirst = Min(U1, U2);
        double ULast = Max(U1, U2);
        double NewLength = ULast - UFirst;
        BSplCLib_KnotDistribution KSet = BSplCLib.KnotForm(Knots, Lower, Upper);
        if (KSet == BSplCLib_Uniform)
        {
            double DU = NewLength / (Upper - Lower);
            Knots[Lower] = UFirst;

            for (int i = Lower + 1; i <= Upper; i++)
            {
                Knots[i] = Knots[i - 1] + DU;
            }
        }
        else
        {
            double K2;
            double Ratio;
            double K1 = Knots[Lower];
            double Length = Knots[Upper] - Knots[Lower];
            Knots[Lower] = UFirst;

            for (int i = Lower + 1; i <= Upper; i++)
            {
                K2 = Knots[i];
                Ratio = (K2 - K1) / Length;
                Knots[i] = Knots[i - 1] + (NewLength * Ratio);

                // for CheckCurveData
                double Eps = Epsilon(Math.Abs(Knots[i - 1]));
                if (Knots[i] - Knots[i - 1] <= Eps)
                {
                    Knots[i] = NextAfter(Knots[i - 1] + Eps, RealLast());
                }

                K1 = K2;
            }
        }
    }

    /// <summary>Reverses the orientation of the knots (the parametrisation is reversed, the range kept).</summary>
    public static void Reverse(NCollection_Array1<double> Knots)
    {
        int first = Knots.Lower();
        int last = Knots.Upper();
        double kfirst = Knots[first];
        double klast = Knots[last];
        double tfirst = kfirst;
        double tlast = klast;
        first++;
        last--;

        while (first <= last)
        {
            tfirst += klast - Knots[last];
            tlast -= Knots[first] - kfirst;
            kfirst = Knots[first];
            klast = Knots[last];
            Knots[first] = tfirst;
            Knots[last] = tlast;
            first++;
            last--;
        }
    }

    /// <summary>Reverses the order of the multiplicities.</summary>
    public static void Reverse(NCollection_Array1<int> Mults)
    {
        Mults.AsSpan().Reverse();
    }

    /// <summary>Reverses the array of weights, <paramref name="Last"/> being the index of the new first element.</summary>
    public static void Reverse(NCollection_Array1<double> Weights, int Last)
    {
        BSplCLib_CurveComputation.BSplCLib_Reverse(Weights, Last);
    }

    /// <summary>
    /// Returns false if all the weights between I1 and I2 are equal (compared exactly; Epsilon is unused);
    /// indices wrap around the array.
    /// </summary>
    public static bool IsRational(NCollection_Array1<double> Weights, int I1, int I2, double Epsilon = 0.0)
    {
        int i, f = Weights.Lower(), l = Weights.Length();
        int I3 = I2 - f;
        ReadOnlySpan<double> WG = Weights.AsSpan(); // WG[f + x] -> WG[x]

        for (i = I1 - f; i < I3; i++)
        {
            if (WG[(i % l)] != WG[((i + 1) % l)])
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>The maximum degree of the B-splines: 25.</summary>
    public static int MaxDegree() => 25;

    /// <summary>
    /// Performs the de Boor algorithm at <paramref name="U"/> on the local poles (Degree + 1 poles of dimension
    /// Dimension) with the 2 * Degree local knots; the value is left in the first pole.
    /// </summary>
    public static void Eval(double U, int Degree, ReadOnlySpan<double> Knots, int Dimension, Span<double> Poles)
    {
        int step, i, Dms, Dm1, Dpi, Sti;
        double X, Y;
        int poles;
        ReadOnlySpan<double> knots = Knots;
        Dm1 = Dms = Degree;
        Dm1--;
        Dms++;
        switch (Dimension)
        {

            case 1:
                {

                    for (step = -1; step < Dm1; step++)
                    {
                        Dms--;
                        poles = 0;
                        Dpi = Dm1;
                        Sti = step;

                        for (i = 0; i < Dms; i++)
                        {
                            Dpi++;
                            Sti++;
                            X = (knots[Dpi] - U) / (knots[Dpi] - knots[Sti]);
                            Y = 1 - X;
                            Poles[poles + 0] *= X;
                            Poles[poles + 0] += Y * Poles[poles + 1];
                            poles += 1;
                        }
                    }
                    break;
                }
            case 2:
                {

                    for (step = -1; step < Dm1; step++)
                    {
                        Dms--;
                        poles = 0;
                        Dpi = Dm1;
                        Sti = step;

                        for (i = 0; i < Dms; i++)
                        {
                            Dpi++;
                            Sti++;
                            X = (knots[Dpi] - U) / (knots[Dpi] - knots[Sti]);
                            Y = 1 - X;
                            Poles[poles + 0] *= X;
                            Poles[poles + 0] += Y * Poles[poles + 2];
                            Poles[poles + 1] *= X;
                            Poles[poles + 1] += Y * Poles[poles + 3];
                            poles += 2;
                        }
                    }
                    break;
                }
            case 3:
                {

                    for (step = -1; step < Dm1; step++)
                    {
                        Dms--;
                        poles = 0;
                        Dpi = Dm1;
                        Sti = step;

                        for (i = 0; i < Dms; i++)
                        {
                            Dpi++;
                            Sti++;
                            X = (knots[Dpi] - U) / (knots[Dpi] - knots[Sti]);
                            Y = 1 - X;
                            Poles[poles + 0] *= X;
                            Poles[poles + 0] += Y * Poles[poles + 3];
                            Poles[poles + 1] *= X;
                            Poles[poles + 1] += Y * Poles[poles + 4];
                            Poles[poles + 2] *= X;
                            Poles[poles + 2] += Y * Poles[poles + 5];
                            poles += 3;
                        }
                    }
                    break;
                }
            case 4:
                {

                    for (step = -1; step < Dm1; step++)
                    {
                        Dms--;
                        poles = 0;
                        Dpi = Dm1;
                        Sti = step;

                        for (i = 0; i < Dms; i++)
                        {
                            Dpi++;
                            Sti++;
                            X = (knots[Dpi] - U) / (knots[Dpi] - knots[Sti]);
                            Y = 1 - X;
                            Poles[poles + 0] *= X;
                            Poles[poles + 0] += Y * Poles[poles + 4];
                            Poles[poles + 1] *= X;
                            Poles[poles + 1] += Y * Poles[poles + 5];
                            Poles[poles + 2] *= X;
                            Poles[poles + 2] += Y * Poles[poles + 6];
                            Poles[poles + 3] *= X;
                            Poles[poles + 3] += Y * Poles[poles + 7];
                            poles += 4;
                        }
                    }
                    break;
                }
            default:
                {
                    int k;

                    for (step = -1; step < Dm1; step++)
                    {
                        Dms--;
                        poles = 0;
                        Dpi = Dm1;
                        Sti = step;

                        for (i = 0; i < Dms; i++)
                        {
                            Dpi++;
                            Sti++;
                            X = (knots[Dpi] - U) / (knots[Dpi] - knots[Sti]);
                            Y = 1 - X;

                            for (k = 0; k < Dimension; k++)
                            {
                                Poles[poles + k] *= X;
                                Poles[poles + k] += Y * Poles[poles + k + Dimension];
                            }
                            poles += Dimension;
                        }
                    }
                    break;
                }
        }
    }

    /// <summary>
    /// Performs the de Boor scheme at <paramref name="U"/> to depth <paramref name="Depth"/> on poles stored
    /// alternately in even positions (see <see cref="BuildBoor"/>), with the local knots of
    /// <see cref="BuildKnots"/>; the intermediate values fill the odd positions. Used for knot insertion.
    /// </summary>
    public static void BoorScheme(double U, int Degree, ReadOnlySpan<double> Knots, int Dimension, Span<double> Poles,
                                  int Depth, int Length)
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
        int pole, firstpole = -2 * Dimension; // indexes in Poles
        // the steps of recursion

        for (step = 0; step < Depth; step++)
        {
            firstpole += Dimension;
            pole = firstpole;
            // compute the new row of poles

            for (i = step; i < Length; i++)
            {
                pole += 2 * Dimension;
                // coefficient
                double X = (knots[i + Degree - step] - U) / (knots[i + Degree - step] - knots[i]);
                double Y = 1.0 - X;
                // Value
                // P(i,j) = X * P(i-1,j) + (1-X) * P(i-1,j+1)

                for (k = 0; k < Dimension; k++)
                {
                    Poles[pole + k] = X * Poles[pole + k - Dimension] + Y * Poles[pole + k + Dimension];
                }
            }
        }
    }

    /// <summary>
    /// The de Boor scheme reverted, for knot removal: computes the poles before insertion from the poles after
    /// insertion. Returns false if the removal is not possible within <paramref name="Tolerance"/>.
    /// </summary>
    public static bool AntiBoorScheme(double U, int Degree, ReadOnlySpan<double> Knots, int Dimension,
                                      Span<double> Poles, int Depth, int Length, double Tolerance)
    {
        // do the Boor scheme reverted.

        int i, k, step, half_length;
        ReadOnlySpan<double> knots = Knots;
        double z, X, Y;
        int pole, firstpole = (Depth - 1) * Dimension; // indexes in Poles

        // Test the special case length = 1
        // only verification of the central point

        if (Length == 1)
        {
            X = (knots[Degree] - U) / (knots[Degree] - knots[0]);
            Y = 1.0 - X;

            for (k = 0; k < Dimension; k++)
            {
                z = X * Poles[firstpole + k] + Y * Poles[firstpole + k + 2 * Dimension];
                if (Math.Abs(z - Poles[firstpole + k + Dimension]) > Tolerance)
                {
                    return false;
                }
            }
            return true;
        }

        // General case
        // the steps of recursion

        for (step = Depth - 1; step >= 0; step--)
        {
            firstpole -= Dimension;
            pole = firstpole;

            // first step from left to right

            for (i = step; i < Length - 1; i++)
            {
                pole += 2 * Dimension;

                X = (knots[i + Degree - step] - U) / (knots[i + Degree - step] - knots[i]);
                Y = 1.0 - X;

                for (k = 0; k < Dimension; k++)
                {
                    Poles[pole + k + Dimension] = (Poles[pole + k] - X * Poles[pole + k - Dimension]) / Y;
                }
            }

            // second step from right to left
            pole += 4 * Dimension;
            half_length = (Length - 1 + step) / 2;
            //
            // only do half of the way from right to left
            // otherwise it start degenerating because of
            // overflows
            //

            for (i = Length - 1; i > half_length; i--)
            {
                pole -= 2 * Dimension;

                // coefficient
                X = (knots[i + Degree - step] - U) / (knots[i + Degree - step] - knots[i]);
                Y = 1.0 - X;

                for (k = 0; k < Dimension; k++)
                {
                    z = (Poles[pole + k] - Y * Poles[pole + k + Dimension]) / X;
                    if (Math.Abs(z - Poles[pole + k - Dimension]) > Tolerance)
                    {
                        return false;
                    }
                    Poles[pole + k - Dimension] += z;
                    Poles[pole + k - Dimension] /= 2.0;
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Computes the poles of the derivatives up to <paramref name="Order"/> of the local B-spline, in place
    /// (Length poles of dimension Dimension).
    /// </summary>
    public static void Derivative(int Degree, ReadOnlySpan<double> Knots, int Dimension, int Length, int Order,
                                  Span<double> Poles)
    {
        int i, k, step, span = Degree;
        ReadOnlySpan<double> knot = Knots;

        for (step = 1; step <= Order; step++)
        {
            int pole = 0;

            for (i = step; i < Length; i++)
            {
                double coef = -span / (knot[i + span] - knot[i]);

                for (k = 0; k < Dimension; k++)
                {
                    Poles[pole + k] -= Poles[pole + k + Dimension];
                    Poles[pole + k] *= coef;
                }
                pole += Dimension;
            }
            span--;
        }
    }

    /// <summary>
    /// Performs the Bohm algorithm at <paramref name="U"/>: computes the value and the derivatives up to
    /// <paramref name="N"/> of the local B-spline, in place in <paramref name="Poles"/> (value, first derivative,
    /// ..., each Dimension values). Derivatives above Degree are not computed.
    /// </summary>
    public static void Bohm(double U, int Degree, int N, ReadOnlySpan<double> Knots, int Dimension, Span<double> Poles)
    {
        // First phase independent of U, compute the poles of the derivatives
        int i, j, iDim, min, Dmi, DDmi, jDmi, Degm1;
        ReadOnlySpan<double> knot = Knots;
        Span<double> P = Poles;
        double coef;
        int pole, tbis, psDD, psDDmDim; // indexes in Poles
        const int psav = 0;
        if (N < Degree)
        {
            min = N;
        }
        else
        {
            min = Degree;
        }
        Degm1 = Degree - 1;
        DDmi = (Degree << 1) + 1;
        switch (Dimension)
        {
            case 1:
                {
                    psDD = psav + Degree;
                    psDDmDim = psDD - 1;

                    for (i = 0; i < Degree; i++)
                    {
                        DDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = DDmi;

                        for (j = Degm1; j >= i; j--)
                        {
                            jDmi--;
                            P[pole] -= P[tbis];
                            P[pole] = (knot[jDmi] == knot[j]) ? 0.0 : P[pole] / (knot[jDmi] - knot[j]);
                            pole--;
                            tbis--;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -1;

                    for (i = 0; i < Degree; i++)
                    {
                        iDim += 1;
                        pole = psav + iDim;
                        tbis = pole + 1;
                        coef = U - knot[i];

                        for (j = i; j >= 0; j--)
                        {
                            P[pole] += coef * P[tbis];
                            pole--;
                            tbis--;
                        }
                    }
                    // multiply by the degrees
                    coef = Degree;
                    Dmi = Degree;
                    pole = psav + 1;

                    for (i = 1; i <= min; i++)
                    {
                        P[pole] *= coef;
                        pole++;
                        Dmi--;
                        coef *= Dmi;
                    }
                    break;
                }
            case 2:
                {
                    psDD = psav + (Degree << 1);
                    psDDmDim = psDD - 2;

                    for (i = 0; i < Degree; i++)
                    {
                        DDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = DDmi;

                        for (j = Degm1; j >= i; j--)
                        {
                            jDmi--;
                            coef = (knot[jDmi] == knot[j]) ? 0.0 : 1.0 / (knot[jDmi] - knot[j]);
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole++;
                            tbis++;
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole -= 3;
                            tbis -= 3;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -2;

                    for (i = 0; i < Degree; i++)
                    {
                        iDim += 2;
                        pole = psav + iDim;
                        tbis = pole + 2;
                        coef = U - knot[i];

                        for (j = i; j >= 0; j--)
                        {
                            P[pole] += coef * P[tbis];
                            pole++;
                            tbis++;
                            P[pole] += coef * P[tbis];
                            pole -= 3;
                            tbis -= 3;
                        }
                    }
                    // multiply by the degrees
                    coef = Degree;
                    Dmi = Degree;
                    pole = psav + 2;

                    for (i = 1; i <= min; i++)
                    {
                        P[pole] *= coef;
                        pole++;
                        P[pole] *= coef;
                        pole++;
                        Dmi--;
                        coef *= Dmi;
                    }
                    break;
                }
            case 3:
                {
                    psDD = psav + (Degree << 1) + Degree;
                    psDDmDim = psDD - 3;

                    for (i = 0; i < Degree; i++)
                    {
                        DDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = DDmi;

                        for (j = Degm1; j >= i; j--)
                        {
                            jDmi--;
                            coef = (knot[jDmi] == knot[j]) ? 0.0 : 1.0 / (knot[jDmi] - knot[j]);
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole++;
                            tbis++;
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole++;
                            tbis++;
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole -= 5;
                            tbis -= 5;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -3;

                    for (i = 0; i < Degree; i++)
                    {
                        iDim += 3;
                        pole = psav + iDim;
                        tbis = pole + 3;
                        coef = U - knot[i];

                        for (j = i; j >= 0; j--)
                        {
                            P[pole] += coef * P[tbis];
                            pole++;
                            tbis++;
                            P[pole] += coef * P[tbis];
                            pole++;
                            tbis++;
                            P[pole] += coef * P[tbis];
                            pole -= 5;
                            tbis -= 5;
                        }
                    }
                    // multiply by the degrees
                    coef = Degree;
                    Dmi = Degree;
                    pole = psav + 3;

                    for (i = 1; i <= min; i++)
                    {
                        P[pole] *= coef;
                        pole++;
                        P[pole] *= coef;
                        pole++;
                        P[pole] *= coef;
                        pole++;
                        Dmi--;
                        coef *= Dmi;
                    }
                    break;
                }
            case 4:
                {
                    psDD = psav + (Degree << 2);
                    psDDmDim = psDD - 4;

                    for (i = 0; i < Degree; i++)
                    {
                        DDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = DDmi;

                        for (j = Degm1; j >= i; j--)
                        {
                            jDmi--;
                            coef = (knot[jDmi] == knot[j]) ? 0.0 : 1.0 / (knot[jDmi] - knot[j]);
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole++;
                            tbis++;
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole++;
                            tbis++;
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole++;
                            tbis++;
                            P[pole] -= P[tbis];
                            P[pole] *= coef;
                            pole -= 7;
                            tbis -= 7;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -4;

                    for (i = 0; i < Degree; i++)
                    {
                        iDim += 4;
                        pole = psav + iDim;
                        tbis = pole + 4;
                        coef = U - knot[i];

                        for (j = i; j >= 0; j--)
                        {
                            P[pole] += coef * P[tbis];
                            pole++;
                            tbis++;
                            P[pole] += coef * P[tbis];
                            pole++;
                            tbis++;
                            P[pole] += coef * P[tbis];
                            pole++;
                            tbis++;
                            P[pole] += coef * P[tbis];
                            pole -= 7;
                            tbis -= 7;
                        }
                    }
                    // multiply by the degrees
                    coef = Degree;
                    Dmi = Degree;
                    pole = psav + 4;

                    for (i = 1; i <= min; i++)
                    {
                        P[pole] *= coef;
                        pole++;
                        P[pole] *= coef;
                        pole++;
                        P[pole] *= coef;
                        pole++;
                        P[pole] *= coef;
                        pole++;
                        Dmi--;
                        coef *= Dmi;
                    }
                    break;
                }
            default:
                {
                    int k;
                    int Dim2 = Dimension << 1;
                    psDD = psav + Degree * Dimension;
                    psDDmDim = psDD - Dimension;

                    for (i = 0; i < Degree; i++)
                    {
                        DDmi--;
                        pole = psDD;
                        tbis = psDDmDim;
                        jDmi = DDmi;

                        for (j = Degm1; j >= i; j--)
                        {
                            jDmi--;
                            coef = (knot[jDmi] == knot[j]) ? 0.0 : 1.0 / (knot[jDmi] - knot[j]);

                            for (k = 0; k < Dimension; k++)
                            {
                                P[pole] -= P[tbis];
                                P[pole] *= coef;
                                pole++;
                                tbis++;
                            }
                            pole -= Dim2;
                            tbis -= Dim2;
                        }
                    }
                    // Second phase, dependant of U
                    iDim = -Dimension;

                    for (i = 0; i < Degree; i++)
                    {
                        iDim += Dimension;
                        pole = psav + iDim;
                        tbis = pole + Dimension;
                        coef = U - knot[i];

                        for (j = i; j >= 0; j--)
                        {

                            for (k = 0; k < Dimension; k++)
                            {
                                P[pole] += coef * P[tbis];
                                pole++;
                                tbis++;
                            }
                            pole -= Dim2;
                            tbis -= Dim2;
                        }
                    }
                    // multiply by the degrees
                    coef = Degree;
                    Dmi = Degree;
                    pole = psav + Dimension;

                    for (i = 1; i <= min; i++)
                    {

                        for (k = 0; k < Dimension; k++)
                        {
                            P[pole] *= coef;
                            pole++;
                        }
                        Dmi--;
                        coef *= Dmi;
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
    /// Stores in <paramref name="LK"/> the 2 * Degree knots around the knot <paramref name="Index"/> needed for
    /// the local evaluation of the span. Without <paramref name="Mults"/> the knots are flat knots.
    /// </summary>
    public static void BuildKnots(int Degree, int Index, bool Periodic, NCollection_Array1<double> Knots,
                                  NCollection_Array1<int>? Mults, Span<double> LK)
    {
        int KLower = Knots.Lower();
        ReadOnlySpan<double> pkn = Knots.AsSpan(); // pkn[i - KLower]
        Span<double> knot = LK;
        if (Mults == null)
        {
            switch (Degree)
            {
                case 1:
                    {
                        int j = Index - KLower;
                        knot[0] = pkn[j];
                        j++;
                        knot[1] = pkn[j];
                        break;
                    }
                case 2:
                    {
                        int j = Index - 1 - KLower;
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
                        int j = Index - 2 - KLower;
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
                        int j = Index - 3 - KLower;
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
                        int j = Index - 4 - KLower;
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
                        int j = Index - 5 - KLower;
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
                        int Deg2 = Degree << 1;
                        j = Index - Degree - KLower;

                        for (i = 0; i < Deg2; i++)
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
            int Deg1 = Degree - 1;
            int KUpper = Knots.Upper();
            int MLower = Mults.Lower();
            int MUpper = Mults.Upper();
            ReadOnlySpan<int> pmu = Mults.AsSpan(); // pmu[i - MLower]
            double dknot = 0;
            int ilow = Index, mlow = 0;
            int iupp = Index + 1, mupp = 0;
            double loffset = 0.0, uoffset = 0.0;
            bool getlow = true, getupp = true;
            if (Periodic)
            {
                dknot = pkn[KUpper - KLower] - pkn[0];
                if (iupp > MUpper)
                {
                    iupp = MLower + 1;
                    uoffset = dknot;
                }
            }
            // Find the knots around Index

            for (i = 0; i < Degree; i++)
            {
                if (getlow)
                {
                    mlow++;
                    if (mlow > pmu[ilow - MLower])
                    {
                        mlow = 1;
                        ilow--;
                        getlow = (ilow >= MLower);
                        if (Periodic && !getlow)
                        {
                            ilow = MUpper - 1;
                            loffset = dknot;
                            getlow = true;
                        }
                    }
                    if (getlow)
                    {
                        knot[Deg1 - i] = pkn[ilow - KLower] - loffset;
                    }
                }
                if (getupp)
                {
                    mupp++;
                    if (mupp > pmu[iupp - MLower])
                    {
                        mupp = 1;
                        iupp++;
                        getupp = (iupp <= MUpper);
                        if (Periodic && !getupp)
                        {
                            iupp = MLower + 1;
                            uoffset = dknot;
                            getupp = true;
                        }
                    }
                    if (getupp)
                    {
                        knot[Degree + i] = pkn[iupp - KLower] + uoffset;
                    }
                }
            }
        }
    }

    /// <summary>Returns the index of the first pole of the span starting at the knot <paramref name="Index"/>.</summary>
    public static int PoleIndex(int Degree, int Index, bool Periodic, NCollection_Array1<int> Mults)
    {
        int i, pindex = 0;

        for (i = Mults.Lower(); i <= Index; i++)
        {
            pindex += Mults[i];
        }
        if (Periodic)
        {
            pindex -= Mults[Mults.Lower()];
        }
        else
        {
            pindex -= Degree + 1;
        }

        return pindex;
    }

    /// <summary>
    /// Copies in <paramref name="LP"/> the Length + 1 poles starting at pole <paramref name="Index"/>, in the even
    /// positions (one free slot of Dimension values between two poles), for <see cref="BoorScheme"/>. Wraps around
    /// the poles.
    /// </summary>
    public static void BuildBoor(int Index, int Length, int Dimension, NCollection_Array1<double> Poles, Span<double> LP)
    {
        int poles = 0;
        int i, k, ip = Poles.Lower() + Index * Dimension;

        for (i = 0; i < Length + 1; i++)
        {

            for (k = 0; k < Dimension; k++)
            {
                LP[poles + k] = Poles[ip];
                ip++;
                if (ip > Poles.Upper())
                {
                    ip = Poles.Lower();
                }
            }
            poles += 2 * Dimension;
        }
    }

    /// <summary>The index in the Boor scheme array of the pole <paramref name="Index"/> (from 0 to Length + Depth).</summary>
    public static int BoorIndex(int Index, int Length, int Depth)
    {
        if (Index <= Depth)
        {
            return Index;
        }
        if (Index <= Length)
        {
            return 2 * Index - Depth;
        }
        return Length + Index - Depth;
    }

    /// <summary>
    /// Copies the pole <paramref name="Index"/> of the Boor scheme array <paramref name="LocPoles"/> into
    /// <paramref name="Pole"/> at <paramref name="Position"/>, which is advanced (and wraps around).
    /// </summary>
    public static void GetPole(int Index, int Length, int Depth, int Dimension, Span<double> LocPoles,
                               ref int Position, NCollection_Array1<double> Pole)
    {
        int k;
        int pole = BoorIndex(Index, Length, Depth) * Dimension;

        for (k = 0; k < Dimension; k++)
        {
            Pole[Position] = LocPoles[pole + k];
            Position++;
        }
        if (Position > Pole.Upper())
        {
            Position = Pole.Lower();
        }
    }

    /// <summary>
    /// Computes the number of poles and knots of the curve after inserting AddKnots (with AddMults, or
    /// multiplicity 1 each when null) for <see cref="InsertKnots(int, bool, int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{int}?, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, double, bool)"/>.
    /// Returns false if the knots to add are not increasing or are outside the range. With Add false, the
    /// multiplicities of existing knots are set, not increased.
    /// </summary>
    public static bool PrepareInsertKnots(int Degree, bool Periodic, NCollection_Array1<double> Knots,
                                          NCollection_Array1<int> Mults, NCollection_Array1<double> AddKnots,
                                          NCollection_Array1<int>? AddMults, out int NbPoles, out int NbKnots,
                                          double Tolerance, bool Add = true)
    {
        bool addflat = AddMults == null;
        // Note: OCCT leaves NbPoles and NbKnots unchanged when it returns false
        NbPoles = 0;
        NbKnots = 0;

        int first, last;
        if (Periodic)
        {
            first = Knots.Lower();
            last = Knots.Upper();
        }
        else
        {
            first = FirstUKnotIndex(Degree, Mults);
            last = LastUKnotIndex(Degree, Mults);
        }
        double adeltaK1 = Knots[first] - AddKnots[AddKnots.Lower()];
        double adeltaK2 = AddKnots[AddKnots.Upper()] - Knots[last];
        if (adeltaK1 > Tolerance)
        {
            return false;
        }
        if (adeltaK2 > Tolerance)
        {
            return false;
        }

        int sigma = 0, mult, amult;
        NbKnots = 0;
        int k = Knots.Lower() - 1;
        int ak = AddKnots.Lower();

        if (Periodic && AddKnots.Length() > 1)
        {
            // gka for case when segments was produced on full period only one knot
            // was added in the end of curve
            if (Math.Abs(adeltaK1) <= gp.Resolution() && Math.Abs(adeltaK2) <= gp.Resolution())
            {
                ak++;
            }
        }

        int aLastKnotMult = Mults[Knots.Upper()];
        double au, oldau = AddKnots[ak], Eps;

        while (ak <= AddKnots.Upper())
        {
            au = AddKnots[ak];
            if (au < oldau)
            {
                return false;
            }
            oldau = au;

            Eps = Max(Tolerance, Epsilon(au));

            while ((k < Knots.Upper()) && (Knots[k + 1] - au <= Eps))
            {
                k++;
                NbKnots++;
                sigma += Mults[k];
            }

            if (addflat)
            {
                amult = 1;
            }
            else
            {
                amult = Max(0, AddMults![ak]);
            }

            // OCCT: with Add false, equal entries of AddKnots count once with the first one's multiplicity here, but
            // InsertKnots processes each of them, so the two can disagree on the result
            while ((ak < AddKnots.Upper()) && (Math.Abs(au - AddKnots[ak + 1]) <= Eps))
            {
                ak++;
                if (Add)
                {
                    if (addflat)
                    {
                        amult++;
                    }
                    else
                    {
                        amult += Max(0, AddMults![ak]);
                    }
                }
            }

            if (Math.Abs(au - Knots[k]) <= Eps)
            {
                // identic to existing knot
                mult = Mults[k];
                if (Add)
                {
                    if (mult + amult > Degree)
                    {
                        amult = Max(0, Degree - mult);
                    }
                    sigma += amult;
                }
                else if (amult > mult)
                {
                    if (amult > Degree)
                    {
                        amult = Degree;
                    }
                    if (k == Knots.Upper() && Periodic)
                    {
                        aLastKnotMult = Max(amult, mult);
                        sigma += 2 * (aLastKnotMult - mult);
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
                    if (amult > Degree)
                    {
                        amult = Degree;
                    }
                    NbKnots++;
                    sigma += amult;
                }
            }

            ak++;
        }

        // count the last knots
        while (k < Knots.Upper())
        {
            k++;
            NbKnots++;
            sigma += Mults[k];
        }

        if (Periodic)
        {
            // for periodic B-Spline the requirement is that multiplicities of the first
            // and last knots must be equal (see Geom_BSplineCurve constructor for
            // instance);
            // respectively AddMults() must meet this requirement if AddKnots() contains
            // knot(s) coincident with first or last
            NbPoles = sigma - aLastKnotMult;
        }
        else
        {
            NbPoles = sigma - Degree - 1;
        }

        return true;
    }

    private static void Copy(int NbPoles, ref int OldFirst, NCollection_Array1<double> OldPoles, ref int NewFirst,
                             NCollection_Array1<double> NewPoles)
    {
        // reset the index in the range for periodicity

        OldFirst =
          OldPoles.Lower() + (OldFirst - OldPoles.Lower()) % (OldPoles.Upper() - OldPoles.Lower() + 1);

        NewFirst =
          NewPoles.Lower() + (NewFirst - NewPoles.Lower()) % (NewPoles.Upper() - NewPoles.Lower() + 1);

        // copy
        int i;

        for (i = 1; i <= NbPoles; i++)
        {
            NewPoles[NewFirst] = OldPoles[OldFirst];
            OldFirst++;
            if (OldFirst > OldPoles.Upper())
            {
                OldFirst = OldPoles.Lower();
            }
            NewFirst++;
            if (NewFirst > NewPoles.Upper())
            {
                NewFirst = NewPoles.Lower();
            }
        }
    }

    /// <summary>
    /// Inserts the knots AddKnots (with AddMults, or multiplicity 1 each when null) in the curve of dimension
    /// <paramref name="Dimension"/>; the new arrays must be sized with <see cref="PrepareInsertKnots"/>. Knots
    /// closer than Epsilon to an existing knot raise its multiplicity (Add) or set it (not Add); multiplicities
    /// never exceed Degree.
    /// </summary>
    public static void InsertKnots(int Degree, bool Periodic, int Dimension, NCollection_Array1<double> Poles,
                                   NCollection_Array1<double> Knots, NCollection_Array1<int> Mults,
                                   NCollection_Array1<double> AddKnots, NCollection_Array1<int>? AddMults,
                                   NCollection_Array1<double> NewPoles, NCollection_Array1<double> NewKnots,
                                   NCollection_Array1<int> NewMults, double Tolerance, bool Add = true)
    {
        bool addflat = AddMults == null;

        int i, k, mult, firstmult;
        int index, kn, curnk, curk;
        int p, np, curp, curnp, length, depth;
        double u;
        int need;
        double Eps;

        // -------------------
        // create local arrays
        // -------------------
        // Use stack-based allocation for small arrays (MaxDegree=25, typical Dimension=4)
        int aKnotsSize = 2 * Degree;
        int aPolesSize = (2 * Degree + 1) * Dimension;
        Span<double> knots = aKnotsSize <= 64 ? stackalloc double[aKnotsSize] : new double[aKnotsSize];
        Span<double> poles = aPolesSize <= 256 ? stackalloc double[aPolesSize] : new double[aPolesSize];

        //----------------------------
        // loop on the knots to insert
        //----------------------------

        curk = Knots.Lower() - 1;    // current position in Knots
        curnk = NewKnots.Lower() - 1; // current position in NewKnots
        curp = Poles.Lower();        // current position in Poles
        curnp = NewPoles.Lower();     // current position in NewPoles

        // NewKnots, NewMults, NewPoles contains the current state of the curve

        // index is the first pole of the current curve for insertion schema

        if (Periodic)
        {
            index = -Mults[Mults.Lower()];
        }
        else
        {
            index = -Degree - 1;
        }

        // on Periodic curves the first knot and the last knot are inserted later
        // (they are the same knot)
        firstmult = 0; // multiplicity of the first-last knot for periodic

        // kn current knot to insert in AddKnots

        for (kn = AddKnots.Lower(); kn <= AddKnots.Upper(); kn++)
        {

            u = AddKnots[kn];
            Eps = Max(Tolerance, Epsilon(u));

            //-----------------------------------
            // find the position in the old knots
            // and copy to the new knots
            //-----------------------------------

            while (curk < Knots.Upper() && Knots[curk + 1] - u <= Eps)
            {
                curk++;
                curnk++;
                NewKnots[curnk] = Knots[curk];
                index += NewMults[curnk] = Mults[curk];
            }

            //-----------------------------------
            // Slice the knots and the mults
            // to the current size of the new curve
            //-----------------------------------

            i = curnk + Knots.Upper() - curk;
            NCollection_Array1<double> nknots = new(NewKnots, NewKnots.Lower(), NewKnots.Lower(), i);
            NCollection_Array1<int> nmults = new(NewMults, NewMults.Lower(), NewMults.Lower(), i);

            //-----------------------------------
            // copy enough knots
            // to compute the insertion schema
            //-----------------------------------

            k = curk;
            i = curnk;
            mult = 0;

            while (mult < Degree && k < Knots.Upper())
            {
                k++;
                i++;
                nknots[i] = Knots[k];
                mult += nmults[i] = Mults[k];
            }

            // copy knots at the end for periodic curve
            if (Periodic)
            {
                mult = 0;
                k = Knots.Upper();
                i = nknots.Upper();

                while (mult < Degree && i > curnk)
                {
                    nknots[i] = Knots[k];
                    mult += nmults[i] = Mults[k];
                    k--;
                    i--;
                }
                nmults[nmults.Upper()] = nmults[nmults.Lower()];
            }

            //------------------------------------
            // do the boor scheme on the new curve
            // to insert the new knot
            //------------------------------------

            bool sameknot = (Math.Abs(u - NewKnots[curnk]) <= Eps);

            if (sameknot)
            {
                length = Max(0, Degree - NewMults[curnk]);
            }
            else
            {
                length = Degree;
            }

            if (addflat)
            {
                depth = 1;
            }
            else
            {
                depth = Min(Degree, AddMults![kn]);
            }

            if (sameknot)
            {
                if (Add)
                {
                    if ((NewMults[curnk] + depth) > Degree)
                    {
                        depth = Degree - NewMults[curnk];
                    }
                }
                else
                {
                    depth = Max(0, depth - NewMults[curnk]);
                }

                if (Periodic)
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

            BuildKnots(Degree, curnk, Periodic, nknots, nmults, knots);

            // copy the poles

            need = NewPoles.Lower() + (index + length + 1) * Dimension - curnp;
            need = Min(need, Poles.Upper() - curp + 1);

            p = curp;
            np = curnp;
            Copy(need, ref p, Poles, ref np, NewPoles);
            curp += need;
            curnp += need;

            // slice the poles to the current number of poles in case of periodic
            NCollection_Array1<double> npoles = new(NewPoles, NewPoles.Lower(), NewPoles.Lower(), curnp - 1);

            BuildBoor(index, length, Dimension, npoles, poles);
            BoorScheme(u, Degree, knots, Dimension, poles, depth, length);

            //-------------------
            // copy the new poles
            //-------------------

            curnp += depth * Dimension; // number of poles is increased by depth
            NCollection_Array1<double> ThePoles = new(NewPoles, NewPoles.Lower(), NewPoles.Lower(), curnp - 1);
            np = NewKnots.Lower() + (index + 1) * Dimension; // OCCT: NewKnots.Lower() where NewPoles.Lower() is meant

            for (i = 1; i <= length + depth; i++)
            {
                GetPole(i, length, depth, Dimension, poles, ref np, ThePoles);
            }

            //-------------------
            // insert the knot
            //-------------------

            index += depth;
            if (sameknot)
            {
                NewMults[curnk] += depth;
            }
            else
            {
                curnk++;
                NewKnots[curnk] = u;
                NewMults[curnk] = depth;
            }
        }

        //------------------------------
        // copy the last poles and knots
        //------------------------------

        Copy(Poles.Upper() - curp + 1, ref curp, Poles, ref curnp, NewPoles);

        while (curk < Knots.Upper())
        {
            curk++;
            curnk++;
            NewKnots[curnk] = Knots[curk];
            NewMults[curnk] = Mults[curk];
        }

        //------------------------------
        // process the first-last knot
        // on periodic curves
        //------------------------------

        if (firstmult > 0)
        {
            curnk = NewKnots.Lower();
            if (NewMults[curnk] + firstmult > Degree)
            {
                firstmult = Degree - NewMults[curnk];
            }
            if (firstmult > 0)
            {

                length = Degree - NewMults[curnk];
                depth = firstmult;

                BuildKnots(Degree, curnk, Periodic, NewKnots, NewMults, knots);
                NCollection_Array1<double> npoles = new(NewPoles,
                                                        NewPoles.Lower(),
                                                        NewPoles.Lower(),
                                                        NewPoles.Upper() - depth * Dimension);
                BuildBoor(0, length, Dimension, npoles, poles);
                BoorScheme(NewKnots[curnk], Degree, knots, Dimension, poles, depth, length);

                //---------------------------
                // copy the new poles
                // but rotate them with depth
                //---------------------------

                np = NewPoles.Lower();

                for (i = depth; i < length + depth; i++)
                {
                    GetPole(i, length, depth, Dimension, poles, ref np, NewPoles);
                }

                np = NewPoles.Upper() - depth * Dimension + 1;

                for (i = 0; i < depth; i++)
                {
                    GetPole(i, length, depth, Dimension, poles, ref np, NewPoles);
                }

                NewMults[NewMults.Lower()] += depth;
                NewMults[NewMults.Upper()] += depth;
            }
        }
    }

    /// <summary>
    /// Decrements the multiplicity of the knot <paramref name="Index"/> to <paramref name="Mult"/> (0 removes
    /// it) if the curve stays within <paramref name="Tolerance"/>. Returns false if not possible.
    /// </summary>
    public static bool RemoveKnot(int Index, int Mult, int Degree, bool Periodic, int Dimension,
                                  NCollection_Array1<double> Poles, NCollection_Array1<double> Knots,
                                  NCollection_Array1<int> Mults, NCollection_Array1<double> NewPoles,
                                  NCollection_Array1<double> NewKnots, NCollection_Array1<int> NewMults,
                                  double Tolerance)
    {
        int index, i, j, k, p, np;

        int TheIndex = Index;

        // protection
        int first, last;
        if (Periodic)
        {
            first = Knots.Lower();
            last = Knots.Upper();
        }
        else
        {
            first = BSplCLib.FirstUKnotIndex(Degree, Mults) + 1;
            last = BSplCLib.LastUKnotIndex(Degree, Mults) - 1;
        }
        if (Index < first)
        {
            return false;
        }
        if (Index > last)
        {
            return false;
        }

        if (Periodic && (Index == first))
        {
            TheIndex = last;
        }

        int depth = Mults[TheIndex] - Mult;
        int length = Degree - Mult;

        // -------------------
        // create local arrays
        // -------------------
        // Use stack-based allocation for small arrays (MaxDegree=25, typical Dimension=4)
        int aKnotsSize = 4 * Degree;
        int aPolesSize = (2 * Degree + 1) * Dimension;
        Span<double> knots = aKnotsSize <= 128 ? stackalloc double[aKnotsSize] : new double[aKnotsSize];
        Span<double> poles = aPolesSize <= 256 ? stackalloc double[aPolesSize] : new double[aPolesSize];

        // ------------------------------------
        // build the knots for anti Boor Scheme
        // ------------------------------------

        // the new sequence of knots
        // is obtained from the knots at Index-1 and Index

        BSplCLib.BuildKnots(Degree, TheIndex - 1, Periodic, Knots, Mults, knots);
        index = PoleIndex(Degree, TheIndex - 1, Periodic, Mults);
        BSplCLib.BuildKnots(Degree, TheIndex, Periodic, Knots, Mults, knots.Slice(2 * Degree));

        index += Mult;

        for (i = 0; i < Degree - Mult; i++)
        {
            knots[i] = knots[i + Mult];
        }

        for (i = Degree - Mult; i < 2 * Degree; i++)
        {
            knots[i] = knots[2 * Degree + i];
        }

        // ------------------------------------
        // build the poles for anti Boor Scheme
        // ------------------------------------

        p = Poles.Lower() + index * Dimension;

        for (i = 0; i <= length + depth; i++)
        {
            j = Dimension * BoorIndex(i, length, depth);

            for (k = 0; k < Dimension; k++)
            {
                poles[j + k] = Poles[p + k];
            }
            p += Dimension;
            if (p > Poles.Upper())
            {
                p = Poles.Lower();
            }
        }

        // ----------------
        // Anti Boor Scheme
        // ----------------

        bool result =
          AntiBoorScheme(Knots[TheIndex], Degree, knots, Dimension, poles, depth, length, Tolerance);

        // ----------------
        // copy the results
        // ----------------

        if (result)
        {

            // poles

            p = Poles.Lower();
            np = NewPoles.Lower();

            // unmodified poles before
            Copy((index + 1) * Dimension, ref p, Poles, ref np, NewPoles);

            // modified

            for (i = 1; i <= length; i++)
            {
                BSplCLib.GetPole(i, length, 0, Dimension, poles, ref np, NewPoles);
            }
            p += (length + depth) * Dimension;

            // unmodified poles after
            if (p != Poles.Lower())
            {
                i = Poles.Upper() - p + 1;
                Copy(i, ref p, Poles, ref np, NewPoles);
            }

            // knots and mults

            if (Mult > 0)
            {
                NewKnots.Assign(Knots);
                NewMults.Assign(Mults);
                NewMults[TheIndex] = Mult;
                if (Periodic)
                {
                    if (TheIndex == first)
                    {
                        NewMults[last] = Mult;
                    }
                    if (TheIndex == last)
                    {
                        NewMults[first] = Mult;
                    }
                }
            }
            else
            {
                if (!Periodic || (TheIndex != first && TheIndex != last))
                {

                    for (i = Knots.Lower(); i < TheIndex; i++)
                    {
                        NewKnots[i] = Knots[i];
                        NewMults[i] = Mults[i];
                    }

                    for (i = TheIndex + 1; i <= Knots.Upper(); i++)
                    {
                        NewKnots[i - 1] = Knots[i];
                        NewMults[i - 1] = Mults[i];
                    }
                }
                else
                {
                    // The interesting case of a Periodic curve
                    // where the first and last knot is removed.

                    for (i = first; i < last - 1; i++)
                    {
                        NewKnots[i] = Knots[i + 1];
                        NewMults[i] = Mults[i + 1];
                    }
                    NewKnots[last - 1] = NewKnots[first] + Knots[last] - Knots[first];
                    NewMults[last - 1] = NewMults[first];
                }
            }
        }

        return result;
    }

    /// <summary>Returns the number of knots of the curve after raising its degree to NewDegree.</summary>
    public static int IncreaseDegreeCountKnots(int Degree, int NewDegree, bool Periodic, NCollection_Array1<int> Mults)
    {
        if (Periodic)
        {
            return Mults.Length();
        }
        int f = FirstUKnotIndex(Degree, Mults);
        int l = LastUKnotIndex(Degree, Mults);
        int m, i, removed = 0, step = NewDegree - Degree;

        i = Mults.Lower();
        m = Degree + (f - i + 1) * step + 1;

        while (m > NewDegree + 1)
        {
            removed++;
            m -= Mults[i] + step;
            i++;
        }
        if (m < NewDegree + 1)
        {
            removed--;
        }

        i = Mults.Upper();
        m = Degree + (i - l + 1) * step + 1;

        while (m > NewDegree + 1)
        {
            removed++;
            m -= Mults[i] + step;
            i--;
        }
        if (m < NewDegree + 1)
        {
            removed--;
        }

        return Mults.Length() - removed;
    }

    /// <summary>
    /// Raises the degree of the curve of dimension <paramref name="Dimension"/> to NewDegree. The new knot count
    /// is given by <see cref="IncreaseDegreeCountKnots"/>; the arrays are expected indexed from 1.
    /// </summary>
    public static void IncreaseDegree(int Degree, int NewDegree, bool Periodic, int Dimension,
                                      NCollection_Array1<double> Poles, NCollection_Array1<double> Knots,
                                      NCollection_Array1<int> Mults, NCollection_Array1<double> NewPoles,
                                      NCollection_Array1<double> NewKnots, NCollection_Array1<int> NewMults)
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

        int nbwknots = Knots.Length();
        f = FirstUKnotIndex(Degree, Mults);
        l = LastUKnotIndex(Degree, Mults);

        if (Periodic)
        {
            // OCCT: for most periodic curves whose multiplicities are not all 1, the raised curve is not the
            // input curve (found against native OCCT, whose poles the port reproduces exactly; not analysed)
            // Periodic curves are transformed in non-periodic curves

            nbwknots += f - Mults.Lower();

            pf = -Degree - 1;

            for (i = Mults.Lower(); i <= f; i++)
            {
                pf += Mults[i];
            }

            nbwknots += Mults.Upper() - l;

            pl = -Degree - 1;

            for (i = l; i <= Mults.Upper(); i++)
            {
                pl += Mults[i];
            }
        }

        // copy the knots and multiplicities
        NCollection_Array1<double> wknots = new(1, nbwknots);
        NCollection_Array1<int> wmults = new(1, nbwknots);
        if (!Periodic)
        {
            wknots.Assign(Knots);
            wmults.Assign(Mults);
        }
        else
        {
            // copy the knots for a periodic curve
            double period = Knots[Knots.Upper()] - Knots[Knots.Lower()];
            i = 0;

            for (k = l; k < Knots.Upper(); k++)
            {
                i++;
                wknots[i] = Knots[k] - period;
                wmults[i] = Mults[k];
            }

            for (k = Knots.Lower(); k <= Knots.Upper(); k++)
            {
                i++;
                wknots[i] = Knots[k];
                wmults[i] = Mults[k];
            }

            for (k = Knots.Lower() + 1; k <= f; k++)
            {
                i++;
                wknots[i] = Knots[k] + period;
                wmults[i] = Mults[k];
            }
        }

        // set the first and last mults to Degree+1
        // and add null poles

        pf += Degree + 1 - wmults[1];
        wmults[1] = Degree + 1;
        pl += Degree + 1 - wmults[nbwknots];
        wmults[nbwknots] = Degree + 1;

        //---------------------------
        // poles of the working curve
        //---------------------------

        int nbwpoles = 0;

        for (i = 1; i <= nbwknots; i++)
        {
            nbwpoles += wmults[i];
        }
        nbwpoles -= Degree + 1;

        // we provide space for degree elevation
        NCollection_Array1<double> wpoles = new(1,
                                                (nbwpoles + (nbwknots - 1) * (NewDegree - Degree)) * Dimension);

        for (i = 1; i <= pf * Dimension; i++)
        {
            wpoles[i] = 0;
        }

        k = Poles.Lower();

        for (i = pf * Dimension + 1; i <= (nbwpoles - pl) * Dimension; i++)
        {
            wpoles[i] = Poles[k];
            k++;
            if (k > Poles.Upper())
            {
                k = Poles.Lower();
            }
        }

        for (i = (nbwpoles - pl) * Dimension + 1; i <= nbwpoles * Dimension; i++)
        {
            wpoles[i] = 0;
        }

        //-----------------------------------------------------------
        // Declare the temporary arrays used in degree incrementation
        //-----------------------------------------------------------

        // Maximum number of poles after all degree elevations
        int nbwpMax = nbwpoles + (nbwknots - 1) * (NewDegree - Degree);
        // Arrays for storing the temporary curves
        NCollection_Array1<double> tempc1 = new(1, nbwpMax * Dimension);
        NCollection_Array1<double> tempc2 = new(1, nbwpMax * Dimension);

        // Array for storing the knots to insert
        NCollection_Array1<double> iknots = new(1, nbwknots);

        // Arrays for receiving the knots after insertion
        NCollection_Array1<double> nknots = new(1, nbwknots);

        // Pre-allocate nwpoles with maximum size to avoid repeated heap allocations in loop
        NCollection_Array1<double> nwpoles = new(1, nbwpMax * Dimension);

        //------------------------------
        // Loop on degree incrementation
        //------------------------------

        int step, curDeg;
        int nbp = nbwpoles;
        int nbwp = nbp;

        for (curDeg = Degree; curDeg < NewDegree; curDeg++)
        {

            nbp = nbwp;               // current number of poles
            nbwp = nbp + nbwknots - 1; // new number of poles

            // Initialize the portion of nwpoles that will be used this iteration
            for (int idx = 1; idx <= nbwp * Dimension; idx++)
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

                    for (k = 0; k < Dimension; k++)
                    {
                        // OCCT indexes wpoles (from 1) with NewPoles.Lower()
                        tempc1[(offset - 1) * Dimension + k + 1] = wpoles[NewPoles.Lower() + i * Dimension + k];
                    }
                    if (i % (curDeg + 1) == step)
                    {
                        offset++;

                        for (k = 0; k < Dimension; k++)
                        {
                            tempc1[(offset - 1) * Dimension + k + 1] = wpoles[NewPoles.Lower() + i * Dimension + k];
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
                    NCollection_Array1<double> curve = new(tempc1, 1, 1, offset * Dimension);
                    NCollection_Array1<double> ncurve = new(tempc2, 1, 1, nbwp * Dimension);
                    //	InsertKnots(curDeg+1,false,Dimension,curve,wknots,wmults,
                    //		    aknots,NoMults(),ncurve,nknots,wmults,Epsilon(1.));

                    InsertKnots(curDeg + 1,
                                false,
                                Dimension,
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

                    for (i = 1; i <= nbwp * Dimension; i++)
                    {
                        nwpoles[i] += ncurve[i];
                    }
                }
                else
                {
                    // add to the average

                    for (i = 1; i <= nbwp * Dimension; i++)
                    {
                        nwpoles[i] += tempc1[i];
                    }
                }
            }

            // The result is the average

            for (i = 1; i <= nbwp * Dimension; i++)
            {
                wpoles[i] = nwpoles[i] / (curDeg + 1);
            }
        }

        //-----------------
        // Copy the results
        //-----------------

        // index in new knots of the first knot of the curve
        if (Periodic)
        {
            firstknot = Mults.Upper() - l + 1;
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

        while (m > NewDegree + 1)
        {
            k++;
            m -= wmults[k];
            pf += wmults[k];
        }
        if (m < NewDegree + 1)
        {
            k--;
            wmults[k] += m - NewDegree - 1;
            pf += m - NewDegree - 1;
        }

        // on a periodic curve the knots start with firstknot
        if (Periodic)
        {
            k = firstknot;
        }

        // copy knots

        for (i = NewKnots.Lower(); i <= NewKnots.Upper(); i++)
        {
            NewKnots[i] = wknots[k];
            NewMults[i] = wmults[k];
            k++;
        }

        // copy poles
        pf *= Dimension;

        for (i = NewPoles.Lower(); i <= NewPoles.Upper(); i++)
        {
            pf++;
            NewPoles[i] = wpoles[pf];
        }
    }

    /// <summary>Computes the number of knots and poles of the curve once made non-periodic (<see cref="Unperiodize(int, int, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{double})"/>).</summary>
    public static void PrepareUnperiodize(int Degree, NCollection_Array1<int> Mults, out int NbKnots, out int NbPoles)
    {
        int i;
        // initialize NbKnots and NbPoles
        NbKnots = Mults.Length();
        NbPoles = -Degree - 1;

        for (i = Mults.Lower(); i <= Mults.Upper(); i++)
        {
            NbPoles += Mults[i];
        }

        int sigma, k;
        // Add knots at the beginning of the curve to raise Multiplicities
        // to Degre + 1;
        sigma = Mults[Mults.Lower()];
        k = Mults.Upper() - 1;

        while (sigma < Degree + 1)
        {
            sigma += Mults[k];
            NbPoles += Mults[k];
            k--;
            NbKnots++;
        }
        // We must add exactly until Degree + 1 ->
        //    Suppress the excedent.
        if (sigma > Degree + 1)
        {
            NbPoles -= sigma - Degree - 1;
        }

        // Add knots at the end of the curve to raise Multiplicities
        // to Degre + 1;
        sigma = Mults[Mults.Upper()];
        k = Mults.Lower() + 1;

        while (sigma < Degree + 1)
        {
            sigma += Mults[k];
            NbPoles += Mults[k];
            k++;
            NbKnots++;
        }
        // We must add exactly until Degree + 1 ->
        //    Suppress the excedent.
        if (sigma > Degree + 1)
        {
            NbPoles -= sigma - Degree - 1;
        }
    }

    /// <summary>
    /// Makes a periodic curve of dimension <paramref name="Dimension"/> non-periodic (arrays sized with
    /// <see cref="PrepareUnperiodize"/>, indexed from 1).
    /// </summary>
    public static void Unperiodize(int Degree, int Dimension, NCollection_Array1<int> Mults,
                                   NCollection_Array1<double> Knots, NCollection_Array1<double> Poles,
                                   NCollection_Array1<int> NewMults, NCollection_Array1<double> NewKnots,
                                   NCollection_Array1<double> NewPoles)
    {
        int sigma, k, index = 0;
        // evaluation of index : number of knots to insert before knot(1) to
        // raise sum of multiplicities to <Degree + 1>
        sigma = Mults[Mults.Lower()];
        k = Mults.Upper() - 1;

        while (sigma < Degree + 1)
        {
            sigma += Mults[k];
            k--;
            index++;
        }

        double period = Knots[Knots.Upper()] - Knots[Knots.Lower()];

        // set the 'interior' knots;

        for (k = 1; k <= Knots.Length(); k++)
        {
            NewKnots[k + index] = Knots[k];
            NewMults[k + index] = Mults[k];
        }

        // set the 'starting' knots;

        for (k = 1; k <= index; k++)
        {
            NewKnots[k] = NewKnots[k + Knots.Length() - 1] - period;
            NewMults[k] = NewMults[k + Knots.Length() - 1];
        }
        NewMults[1] -= sigma - Degree - 1;

        // set the 'ending' knots;
        sigma = NewMults[index + Knots.Length()];

        for (k = Knots.Length() + index + 1; k <= NewKnots.Length(); k++)
        {
            NewKnots[k] = NewKnots[k - Knots.Length() + 1] + period;
            NewMults[k] = NewMults[k - Knots.Length() + 1];
            sigma += NewMults[k - Knots.Length() + 1];
        }
        NewMults[NewMults.Length()] -= sigma - Degree - 1;

        for (k = 1; k <= NewPoles.Length(); k++)
        {
            NewPoles[k] = Poles[(k - 1) % Poles.Length() + 1];
        }
    }

    /// <summary>Computes the number of knots and poles of the curve trimmed to [U1, U2] (<see cref="Trimming(int, bool, int, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double}, double, double, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double})"/>).</summary>
    public static void PrepareTrimming(int Degree, bool Periodic, NCollection_Array1<double> Knots,
                                       NCollection_Array1<int> Mults, double U1, double U2, out int NbKnots,
                                       out int NbPoles)
    {
        int i;
        double NewU1, NewU2;
        int index1 = 0, index2 = 0;

        // Eval index1, index2 : position of U1 and U2 in the Array Knots
        // such as Knots(index1-1) <= U1 < Knots(index1)
        //         Knots(index2-1) <= U2 < Knots(index2)
        LocateParameter(Degree, Knots, Mults, U1, Periodic, Knots.Lower(), Knots.Upper(), out index1, out NewU1);
        LocateParameter(Degree, Knots, Mults, U2, Periodic, Knots.Lower(), Knots.Upper(), out index2, out NewU2);
        index1++;
        if (Math.Abs(Knots[index2] - U2) <= Epsilon(U1))
        {
            index2--;
        }

        // eval NbKnots:
        NbKnots = index2 - index1 + 3;

        // eval NbPoles:
        NbPoles = Degree + 1;

        for (i = index1; i <= index2; i++)
        {
            NbPoles += Mults[i];
        }
    }

    /// <summary>
    /// Trims the curve of dimension <paramref name="Dimension"/> to [U1, U2] (arrays sized with
    /// <see cref="PrepareTrimming"/>, indexed from 1). Throws <see cref="Standard_OutOfRange"/> if U1 or U2 is
    /// outside the curve.
    /// </summary>
    public static void Trimming(int Degree, bool Periodic, int Dimension, NCollection_Array1<double> Knots,
                                NCollection_Array1<int> Mults, NCollection_Array1<double> Poles, double U1, double U2,
                                NCollection_Array1<double> NewKnots, NCollection_Array1<int> NewMults,
                                NCollection_Array1<double> NewPoles)
    {
        int i, nbpoles = 0, nbknots = 0;
        double[] kk = [U1, U2];
        int[] mm = [Degree, Degree];
        NCollection_Array1<double> K = new(kk, 0, 1, 2);
        NCollection_Array1<int> M = new(mm, 0, 1, 2);
        if (!PrepareInsertKnots(Degree,
                                Periodic,
                                Knots,
                                Mults,
                                K,
                                M,
                                out nbpoles,
                                out nbknots,
                                Epsilon(U1),
                                false))
        {
            throw new Standard_OutOfRange();
        }

        NCollection_Array1<double> TempPoles = new(1, nbpoles * Dimension);
        NCollection_Array1<double> TempKnots = new(1, nbknots);
        NCollection_Array1<int> TempMults = new(1, nbknots);

        //
        // do not allow the multiplicities to Add : they must be less than Degree
        //
        InsertKnots(Degree,
                    Periodic,
                    Dimension,
                    Poles,
                    Knots,
                    Mults,
                    K,
                    M,
                    TempPoles,
                    TempKnots,
                    TempMults,
                    Epsilon(U1),
                    false);

        // find in TempPoles the index of the pole corresponding to U1
        int Kindex = 0, Pindex;
        double NewU1;
        LocateParameter(Degree,
                        TempKnots,
                        TempMults,
                        U1,
                        Periodic,
                        TempKnots.Lower(),
                        TempKnots.Upper(),
                        out Kindex,
                        out NewU1);
        Pindex = PoleIndex(Degree, Kindex, Periodic, TempMults);
        Pindex *= Dimension;

        for (i = 1; i <= NewPoles.Length(); i++)
        {
            NewPoles[i] = TempPoles[Pindex + i];
        }

        for (i = 1; i <= NewKnots.Length(); i++)
        {
            NewKnots[i] = TempKnots[Kindex + i - 1];
            NewMults[i] = TempMults[Kindex + i - 1];
        }
        NewMults[1] = Min(Degree, NewMults[1]) + 1;
        NewMults[NewMults.Length()] = Min(Degree, NewMults[NewMults.Length()]) + 1;
    }

    /// <summary>
    /// Solves the banded system factored by <see cref="FactorBandedMatrix"/> for <paramref name="Array"/>, an array
    /// of points of dimension ArrayDimension (one per row), in place. Returns 0, or 1 on a bad matrix or a null pivot.
    /// </summary>
    public static int SolveBandedSystem(math_Matrix Matrix, int UpperBandWidth, int LowerBandWidth, int ArrayDimension,
                                        Span<double> Array)
    {
        if (Matrix.LowerCol() != 1 || Matrix.UpperCol() != UpperBandWidth + LowerBandWidth + 1)
        {
            return 1;
        }

        Span<double> PolesArray = Array;
        int aLRow = Matrix.LowerRow();
        int aNCols = Matrix.ColNumber();
        ReadOnlySpan<double> aMatData = Matrix.AsSpan(); // from (aLRow, 1), row-major

        for (int ii = aLRow + 1; ii <= Matrix.UpperRow(); ii++)
        {
            int aMinIndex = (ii - LowerBandWidth >= aLRow ? ii - LowerBandWidth : aLRow);
            ReadOnlySpan<double> aRowII = aMatData.Slice((ii - aLRow) * aNCols, aNCols);

            for (int jj = aMinIndex; jj < ii; jj++)
            {
                double aCoeff = aRowII[jj - ii + LowerBandWidth];
                for (int kk = 0; kk < ArrayDimension; kk++)
                {
                    PolesArray[(ii - 1) * ArrayDimension + kk] +=
                      PolesArray[(jj - 1) * ArrayDimension + kk] * aCoeff;
                }
            }
        }

        for (int ii = Matrix.UpperRow(); ii >= aLRow; ii--)
        {
            int aMaxIndex =
              (ii + UpperBandWidth <= Matrix.UpperRow() ? ii + UpperBandWidth : Matrix.UpperRow());
            ReadOnlySpan<double> aRowII = aMatData.Slice((ii - aLRow) * aNCols, aNCols);

            for (int jj = aMaxIndex; jj > ii; jj--)
            {
                double aCoeff = aRowII[jj - ii + LowerBandWidth];
                for (int kk = 0; kk < ArrayDimension; kk++)
                {
                    PolesArray[(ii - 1) * ArrayDimension + kk] -=
                      PolesArray[(jj - 1) * ArrayDimension + kk] * aCoeff;
                }
            }

            // Fixing a bug PRO18577 to avoid division by zero
            double aDivisor = aRowII[LowerBandWidth];
            const double THE_TOLERANCE = 1.0e-16;
            if (Math.Abs(aDivisor) <= THE_TOLERANCE)
            {
                return 1;
            }
            double anInverse = 1.0 / aDivisor;

            for (int kk = 0; kk < ArrayDimension; kk++)
            {
                PolesArray[(ii - 1) * ArrayDimension + kk] *= anInverse;
            }
        }

        return 0;
    }

    /// <summary>
    /// Solves the banded system for rational poles: if <paramref name="HomogenousFlag"/> is false the poles are
    /// first multiplied by the weights and divided back after. Returns 0, or 2 or 3 if solving the poles or the
    /// weights failed (1 on a bad matrix).
    /// </summary>
    public static int SolveBandedSystem(math_Matrix Matrix, int UpperBandWidth, int LowerBandWidth, bool HomogenousFlag,
                                        int ArrayDimension, Span<double> Array, Span<double> Weights)
    {
        bool HomogeneousFlag = HomogenousFlag;
        if (Matrix.LowerCol() != 1 || Matrix.UpperCol() != UpperBandWidth + LowerBandWidth + 1)
        {
            return 1;
        }

        Span<double> aPolesArray = Array;
        Span<double> aWeightsArray = Weights;
        int aNumRows = Matrix.UpperRow() - Matrix.LowerRow() + 1;

        if (!HomogeneousFlag)
        {
            for (int ii = 0; ii < aNumRows; ii++)
            {
                for (int kk = 0; kk < ArrayDimension; kk++)
                {
                    aPolesArray[ii * ArrayDimension + kk] *= aWeightsArray[ii];
                }
            }
        }

        int anErrorCode =
          BSplCLib.SolveBandedSystem(Matrix, UpperBandWidth, LowerBandWidth, ArrayDimension, Array);
        if (anErrorCode != 0)
        {
            return 2;
        }

        anErrorCode = BSplCLib.SolveBandedSystem(Matrix, UpperBandWidth, LowerBandWidth, 1, Weights);
        if (anErrorCode != 0)
        {
            return 3;
        }

        if (!HomogeneousFlag)
        {
            for (int ii = 0; ii < aNumRows; ii++)
            {
                double anInverse = 1.0 / aWeightsArray[ii];
                for (int kk = 0; kk < ArrayDimension; kk++)
                {
                    aPolesArray[ii * ArrayDimension + kk] *= anInverse;
                }
            }
        }

        return 0;
    }

    /// <summary>Computes the Schoenberg points (the averages of Degree consecutive flat knots) into Parameters.</summary>
    public static void BuildSchoenbergPoints(int Degree, NCollection_Array1<double> FlatKnots,
                                             NCollection_Array1<double> Parameters)
    {
        int ii, jj;
        double Inverse;
        Inverse = 1.0e0 / (double)Degree;

        for (ii = Parameters.Lower(); ii <= Parameters.Upper(); ii++)
        {
            Parameters[ii] = 0.0e0;

            for (jj = 1; jj <= Degree; jj++)
            {
                Parameters[ii] += FlatKnots[jj + ii];
            }
            Parameters[ii] *= Inverse;
        }
    }

    /// <summary>
    /// Interpolates the points (and derivatives, by ContactOrderArray) of dimension ArrayDimension stored in
    /// <paramref name="Poles"/> at Parameters with a B-spline on FlatKnots; Poles receives the poles.
    /// InversionProblem is the pivot index of a singular matrix (0 otherwise). Throws
    /// <see cref="Standard_OutOfRange"/> if the system can't be built or solved.
    /// </summary>
    public static void Interpolate(int Degree, NCollection_Array1<double> FlatKnots, NCollection_Array1<double> Parameters,
                                   NCollection_Array1<int> ContactOrderArray, int ArrayDimension, Span<double> Poles,
                                   out int InversionProblem)
    {
        int ErrorCode, UpperBandWidth, LowerBandWidth;
        // Note: OCCT uses a stack buffer of up to 2048 values for the matrix
        math_Matrix InterpolationMatrix = new(1, Parameters.Length(), 1, 2 * Degree + 1);
        ErrorCode = BSplCLib.BuildBSpMatrix(Parameters,
                                            ContactOrderArray,
                                            FlatKnots,
                                            Degree,
                                            InterpolationMatrix,
                                            out UpperBandWidth,
                                            out LowerBandWidth);
        if (ErrorCode != 0)
        {
            throw new Standard_OutOfRange("BSplCLib::Interpolate");
        }

        ErrorCode = BSplCLib.FactorBandedMatrix(InterpolationMatrix,
                                                UpperBandWidth,
                                                LowerBandWidth,
                                                out InversionProblem);
        if (ErrorCode != 0)
        {
            throw new Standard_OutOfRange("BSplCLib::Interpolate");
        }

        ErrorCode = BSplCLib.SolveBandedSystem(InterpolationMatrix,
                                               UpperBandWidth,
                                               LowerBandWidth,
                                               ArrayDimension,
                                               Poles);
        if (ErrorCode != 0)
        {
            throw new Standard_OutOfRange("BSplCLib::Interpolate");
        }
    }

    /// <summary>
    /// Rational interpolation: as <see cref="Interpolate(int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, int, Span{double}, out int)"/>
    /// with the weights in <paramref name="Weights"/> (non-homogeneous poles).
    /// </summary>
    public static void Interpolate(int Degree, NCollection_Array1<double> FlatKnots, NCollection_Array1<double> Parameters,
                                   NCollection_Array1<int> ContactOrderArray, int ArrayDimension, Span<double> Poles,
                                   Span<double> Weights, out int InversionProblem)
    {
        int ErrorCode, UpperBandWidth, LowerBandWidth;
        // Note: OCCT uses a stack buffer of up to 2048 values for the matrix
        math_Matrix InterpolationMatrix = new(1, Parameters.Length(), 1, 2 * Degree + 1);
        ErrorCode = BSplCLib.BuildBSpMatrix(Parameters,
                                            ContactOrderArray,
                                            FlatKnots,
                                            Degree,
                                            InterpolationMatrix,
                                            out UpperBandWidth,
                                            out LowerBandWidth);
        if (ErrorCode != 0)
        {
            throw new Standard_OutOfRange("BSplCLib::Interpolate");
        }

        ErrorCode = BSplCLib.FactorBandedMatrix(InterpolationMatrix,
                                                UpperBandWidth,
                                                LowerBandWidth,
                                                out InversionProblem);
        if (ErrorCode != 0)
        {
            throw new Standard_OutOfRange("BSplCLib::Interpolate");
        }

        ErrorCode = BSplCLib.SolveBandedSystem(InterpolationMatrix,
                                               UpperBandWidth,
                                               LowerBandWidth,
                                               false,
                                               ArrayDimension,
                                               Poles,
                                               Weights);
        if (ErrorCode != 0)
        {
            throw new Standard_OutOfRange("BSplCLib::Interpolate");
        }
    }

    /// <summary>
    /// Evaluates the rational B-spline (poles of dimension ArrayDimension, non-homogeneous, and weights) and its
    /// derivatives up to DerivativeRequest at <paramref name="U"/>, into PolesResult (homogeneous) and
    /// WeightsResult. <paramref name="ExtrapMode"/> holds two values: the degree of extrapolation before the
    /// first and after the last knot (a Taylor expansion of that order is used beyond the ends).
    /// </summary>
    public static void Eval(double U, bool PeriodicFlag, int DerivativeRequest, ReadOnlySpan<int> ExtrapMode, int Degree,
                            NCollection_Array1<double> FlatKnots, int ArrayDimension, ReadOnlySpan<double> Poles,
                            ReadOnlySpan<double> Weights, Span<double> PolesResult, Span<double> WeightsResult)
    {
        double Parameter = U;
        int ii, jj, kk = 0, Index, Index1, Index2, Modulus, NewRequest,
                    ErrorCode, Order = Degree + 1, FirstNonZeroBsplineIndex,
                    LocalRequest = DerivativeRequest;
        Span<int> ExtrapolatingFlag = stackalloc int[2];
        double LocalParameter, Period, Inverse, Delta;
        ReadOnlySpan<double> PolesArray = Poles;
        ReadOnlySpan<double> WeightsArray = Weights;
        ReadOnlySpan<int> ExtrapModeArray = ExtrapMode;
        Span<double> PResultArray = PolesResult;
        Span<double> WResultArray = WeightsResult;
        LocalParameter = Parameter;
        ExtrapolatingFlag[0] = ExtrapolatingFlag[1] = 0;
        //
        // check if we are extrapolating to a degree which is smaller than
        // the degree of the Bspline
        //
        if (PeriodicFlag)
        {
            Period = FlatKnots[FlatKnots.Upper() - 1] - FlatKnots[2];

            while (LocalParameter > FlatKnots[FlatKnots.Upper() - 1])
            {
                LocalParameter -= Period;
            }

            while (LocalParameter < FlatKnots[2])
            {
                LocalParameter += Period;
            }
        }
        if (Parameter < FlatKnots[2] && LocalRequest < ExtrapModeArray[0] && ExtrapModeArray[0] < Degree)
        {
            LocalRequest = ExtrapModeArray[0];
            LocalParameter = FlatKnots[2];
            ExtrapolatingFlag[0] = 1;
        }
        if (Parameter > FlatKnots[FlatKnots.Upper() - 1] && LocalRequest < ExtrapModeArray[1]
            && ExtrapModeArray[1] < Degree)
        {
            LocalRequest = ExtrapModeArray[1];
            LocalParameter = FlatKnots[FlatKnots.Upper() - 1];
            ExtrapolatingFlag[1] = 1;
        }
        Delta = Parameter - LocalParameter;
        if (LocalRequest >= Order)
        {
            LocalRequest = Degree;
        }
        if (PeriodicFlag)
        {
            Modulus = FlatKnots.Length() - Degree - 1;
        }
        else
        {
            Modulus = FlatKnots.Length() - Degree;
        }

        math_Matrix BsplineBasis = BSplCLib_LocalMatrix(LocalRequest, Order);
        ErrorCode = BSplCLib.EvalBsplineBasis(LocalRequest,
                                              Order,
                                              FlatKnots,
                                              LocalParameter,
                                              out FirstNonZeroBsplineIndex,
                                              BsplineBasis);
        if (ErrorCode != 0)
        {
            return;
        }
        if (ExtrapolatingFlag[0] == 0 && ExtrapolatingFlag[1] == 0)
        {
            Index = 0;
            Index2 = 0;

            for (ii = 1; ii <= LocalRequest + 1; ii++)
            {
                Index1 = FirstNonZeroBsplineIndex;

                for (kk = 0; kk < ArrayDimension; kk++)
                {
                    PResultArray[Index + kk] = 0.0e0;
                }
                WResultArray[Index] = 0.0e0;

                for (jj = 1; jj <= Order; jj++)
                {

                    for (kk = 0; kk < ArrayDimension; kk++)
                    {
                        PResultArray[Index + kk] += PolesArray[(Index1 - 1) * ArrayDimension + kk]
                                                    * WeightsArray[Index1 - 1] * BsplineBasis[ii, jj];
                    }
                    WResultArray[Index2] += WeightsArray[Index1 - 1] * BsplineBasis[ii, jj];

                    Index1 = Index1 % Modulus;
                    Index1 += 1;
                }
                Index += ArrayDimension;
                Index2 += 1;
            }
        }
        else
        {
            //
            //  store Taylor expansion in LocalRealArray
            //
            NewRequest = DerivativeRequest;
            if (NewRequest > Degree)
            {
                NewRequest = Degree;
            }
            // OCCT sizes LocalRealArray (LocalRequest + 1) * ArrayDimension, then evaluates a polynomial of degree
            // Degree (> LocalRequest here) on it, reading past the Taylor coefficients into uninitialised memory.
            // The port sizes it for Degree + 1 rows; the extra rows are zero.
            int aLocalSize = Max(LocalRequest + 1, Degree + 1) * ArrayDimension;
            Span<double> LocalRealArray = aLocalSize <= 128 ? stackalloc double[aLocalSize] : new double[aLocalSize];
            Index = 0;
            Inverse = 1.0e0;

            for (ii = 1; ii <= LocalRequest + 1; ii++)
            {
                Index1 = FirstNonZeroBsplineIndex;

                for (kk = 0; kk < ArrayDimension; kk++)
                {
                    LocalRealArray[Index + kk] = 0.0e0;
                }

                for (jj = 1; jj <= Order; jj++)
                {

                    for (kk = 0; kk < ArrayDimension; kk++)
                    {
                        LocalRealArray[Index + kk] += PolesArray[(Index1 - 1) * ArrayDimension + kk]
                                                      * WeightsArray[Index1 - 1] * BsplineBasis[ii, jj];
                    }
                    Index1 = Index1 % Modulus;
                    Index1 += 1;
                }

                for (kk = 0; kk < ArrayDimension; kk++)
                {
                    LocalRealArray[Index + kk] *= Inverse;
                }
                Index += ArrayDimension;
                Inverse /= (double)ii;
            }
            PLib.EvalPolynomial(Delta, NewRequest, Degree, ArrayDimension, LocalRealArray, PolesResult); // OCCT: see above
            Index = 0;
            Inverse = 1.0e0;

            for (ii = 1; ii <= LocalRequest + 1; ii++)
            {
                Index1 = FirstNonZeroBsplineIndex;
                LocalRealArray[Index] = 0.0e0;

                for (jj = 1; jj <= Order; jj++)
                {
                    LocalRealArray[Index] += WeightsArray[Index1 - 1] * BsplineBasis[ii, jj];
                    Index1 = Index1 % Modulus;
                    Index1 += 1;
                }
                LocalRealArray[Index] *= Inverse;
                Index += 1;
                Inverse /= (double)ii;
            }
            // OCCT: here the coefficients past LocalRequest are the poles' values left from above
            PLib.EvalPolynomial(Delta, NewRequest, Degree, 1, LocalRealArray, WeightsResult);
        }
    }

    /// <summary>
    /// Evaluates the B-spline (poles of dimension ArrayDimension) and its derivatives up to DerivativeRequest at
    /// <paramref name="U"/> into Result. <paramref name="ExtrapMode"/> holds two values: the degree of
    /// extrapolation before the first and after the last knot.
    /// </summary>
    public static void Eval(double U, bool PeriodicFlag, int DerivativeRequest, ReadOnlySpan<int> ExtrapMode, int Degree,
                            NCollection_Array1<double> FlatKnots, int ArrayDimension, ReadOnlySpan<double> Poles,
                            Span<double> Result)
    {
        double Parameter = U;
        int ii, jj, kk, Index, Index1, Modulus, NewRequest,
          ErrorCode, Order = Degree + 1, FirstNonZeroBsplineIndex, LocalRequest = DerivativeRequest;
        Span<int> ExtrapolatingFlag = stackalloc int[2];

        double LocalParameter, Period, Inverse, Delta;

        ReadOnlySpan<double> PolesArray = Poles;
        ReadOnlySpan<int> ExtrapModeArray = ExtrapMode;
        Span<double> ResultArray = Result;
        LocalParameter = Parameter;
        ExtrapolatingFlag[0] = ExtrapolatingFlag[1] = 0;
        //
        // check if we are extrapolating to a degree which is smaller than
        // the degree of the Bspline
        //
        if (PeriodicFlag)
        {
            Period = FlatKnots[FlatKnots.Upper() - 1] - FlatKnots[2];

            while (LocalParameter > FlatKnots[FlatKnots.Upper() - 1])
            {
                LocalParameter -= Period;
            }

            while (LocalParameter < FlatKnots[2])
            {
                LocalParameter += Period;
            }
        }
        if (Parameter < FlatKnots[2] && LocalRequest < ExtrapModeArray[0] && ExtrapModeArray[0] < Degree)
        {
            LocalRequest = ExtrapModeArray[0];
            LocalParameter = FlatKnots[2];
            ExtrapolatingFlag[0] = 1;
        }
        if (Parameter > FlatKnots[FlatKnots.Upper() - 1] && LocalRequest < ExtrapModeArray[1]
            && ExtrapModeArray[1] < Degree)
        {
            LocalRequest = ExtrapModeArray[1];
            LocalParameter = FlatKnots[FlatKnots.Upper() - 1];
            ExtrapolatingFlag[1] = 1;
        }
        Delta = Parameter - LocalParameter;
        if (LocalRequest >= Order)
        {
            LocalRequest = Degree;
        }

        if (PeriodicFlag)
        {
            Modulus = FlatKnots.Length() - Degree - 1;
        }
        else
        {
            Modulus = FlatKnots.Length() - Degree;
        }

        math_Matrix BsplineBasis = BSplCLib_LocalMatrix(LocalRequest, Order);

        ErrorCode = BSplCLib.EvalBsplineBasis(LocalRequest,
                                              Order,
                                              FlatKnots,
                                              LocalParameter,
                                              out FirstNonZeroBsplineIndex,
                                              BsplineBasis);
        if (ErrorCode != 0)
        {
            return;
        }
        if (ExtrapolatingFlag[0] == 0 && ExtrapolatingFlag[1] == 0)
        {
            Index = 0;

            for (ii = 1; ii <= LocalRequest + 1; ii++)
            {
                Index1 = FirstNonZeroBsplineIndex;

                for (kk = 0; kk < ArrayDimension; kk++)
                {
                    ResultArray[Index + kk] = 0.0e0;
                }

                for (jj = 1; jj <= Order; jj++)
                {

                    for (kk = 0; kk < ArrayDimension; kk++)
                    {
                        ResultArray[Index + kk] +=
                          PolesArray[(Index1 - 1) * ArrayDimension + kk] * BsplineBasis[ii, jj];
                    }
                    Index1 = Index1 % Modulus;
                    Index1 += 1;
                }
                Index += ArrayDimension;
            }
        }
        else
        {
            //
            //  store Taylor expansion in LocalRealArray
            //
            NewRequest = DerivativeRequest;
            if (NewRequest > Degree)
            {
                NewRequest = Degree;
            }
            // OCCT sizes LocalRealArray (LocalRequest + 1) * ArrayDimension, then evaluates a polynomial of degree
            // Degree (> LocalRequest here) on it, reading past the Taylor coefficients into uninitialised memory.
            // The port sizes it for Degree + 1 rows; the extra rows are zero.
            int aLocalSize = Max(LocalRequest + 1, Degree + 1) * ArrayDimension;
            Span<double> LocalRealArray = aLocalSize <= 128 ? stackalloc double[aLocalSize] : new double[aLocalSize];

            Index = 0;
            Inverse = 1.0e0;

            for (ii = 1; ii <= LocalRequest + 1; ii++)
            {
                Index1 = FirstNonZeroBsplineIndex;

                for (kk = 0; kk < ArrayDimension; kk++)
                {
                    LocalRealArray[Index + kk] = 0.0e0;
                }

                for (jj = 1; jj <= Order; jj++)
                {

                    for (kk = 0; kk < ArrayDimension; kk++)
                    {
                        LocalRealArray[Index + kk] +=
                          PolesArray[(Index1 - 1) * ArrayDimension + kk] * BsplineBasis[ii, jj];
                    }
                    Index1 = Index1 % Modulus;
                    Index1 += 1;
                }

                for (kk = 0; kk < ArrayDimension; kk++)
                {
                    LocalRealArray[Index + kk] *= Inverse;
                }
                Index += ArrayDimension;
                Inverse /= (double)ii;
            }
            PLib.EvalPolynomial(Delta, NewRequest, Degree, ArrayDimension, LocalRealArray, Result); // OCCT: see above
        }
    }

    /// <summary>
    /// Extends a B-spline (flat knots, NumPoles poles of dimension Dimension) to the point ConstraintPoint, after
    /// its end (<paramref name="After"/>) or before its start, with continuity <paramref name="Continuity"/> at the
    /// junction; C1Coefficient scales the tangent. Writes the result's flat knots and poles into KnotsResult and
    /// PolesResult (which must be large enough) and their counts into NbKnotsRsult and NbPolesResult.
    /// </summary>
    public static void TangExtendToConstraint(NCollection_Array1<double> FlatKnots, double C1Coefficient, int NumPoles,
                                              ReadOnlySpan<double> Poles, int Dimension, int Degree,
                                              NCollection_Array1<double> ConstraintPoint, int Continuity, bool After,
                                              out int NbPolesResult, out int NbKnotsRsult, Span<double> KnotsResult,
                                              Span<double> PolesResult)
    {
        int CDimension = Dimension, CDegree = Degree;
        ReadOnlySpan<double> Padr = Poles;
        Span<double> KRadr = KnotsResult;
        Span<double> PRadr = PolesResult;

        ////////////////////////////////////////////////////////////////////////
        //
        //    1. calculation of extension nD
        //
        ////////////////////////////////////////////////////////////////////////

        //  Hermite matrix
        int Csize = Continuity + 2;
        math_Matrix MatCoefs = new(1, Csize, 1, Csize);
        if (After)
        {
            PLib.HermiteCoefficients(0,
                                     1, // Limits
                                     Continuity,
                                     0, // Orders of constraints
                                     MatCoefs);
        }
        else
        {
            PLib.HermiteCoefficients(0,
                                     1, // Limits
                                     0,
                                     Continuity, // Orders of constraints
                                     MatCoefs);
        }

        //  position at the node of connection
        double Tbord;
        if (After)
        {
            Tbord = FlatKnots[FlatKnots.Upper() - CDegree];
        }
        else
        {
            Tbord = FlatKnots[FlatKnots.Lower() + CDegree];
        }
        bool periodic_flag = false;
        int ipos, derivative_request = Max(Continuity, 1);
        Span<int> extrap_mode = stackalloc int[2];
        extrap_mode[0] = extrap_mode[1] = CDegree;
        // EvalBS over a plain array, so that it can be passed as an array start
        double[] aEvalBuffer = new double[CDimension * (derivative_request + 1)];
        math_Vector EvalBS = new(aEvalBuffer, 1, CDimension * (derivative_request + 1));
        Span<double> Eadr = aEvalBuffer;
        BSplCLib.Eval(Tbord,
                      periodic_flag,
                      derivative_request,
                      extrap_mode,
                      CDegree,
                      FlatKnots,
                      CDimension,
                      Poles,
                      Eadr);

        //  norm of the tangent at the node of connection
        math_Vector Tgte = new(1, CDimension);

        for (ipos = 1; ipos <= CDimension; ipos++)
        {
            Tgte[ipos] = EvalBS[ipos + CDimension];
        }
        double L1 = Tgte.Norm();

        //  matrix of constraints
        math_Matrix Contraintes = new(1, Csize, 1, CDimension);
        // Precompute powers of C1Coefficient for efficiency (avoid std::pow overhead)
        double C1Coeff2 = C1Coefficient * C1Coefficient;
        double C1Coeff3 = C1Coeff2 * C1Coefficient;
        if (After)
        {
            for (ipos = 1; ipos <= CDimension; ipos++)
            {
                Contraintes[1, ipos] = EvalBS[ipos];
                Contraintes[2, ipos] = C1Coefficient * EvalBS[ipos + CDimension];
                if (Continuity >= 2)
                {
                    Contraintes[3, ipos] = EvalBS[ipos + 2 * CDimension] * C1Coeff2;
                }
                if (Continuity >= 3)
                {
                    Contraintes[4, ipos] = EvalBS[ipos + 3 * CDimension] * C1Coeff3;
                }
                Contraintes[Continuity + 2, ipos] = ConstraintPoint[ipos];
            }
        }
        else
        {
            for (ipos = 1; ipos <= CDimension; ipos++)
            {
                Contraintes[1, ipos] = ConstraintPoint[ipos];
                Contraintes[2, ipos] = EvalBS[ipos];
                if (Continuity >= 1)
                {
                    Contraintes[3, ipos] = C1Coefficient * EvalBS[ipos + CDimension];
                }
                if (Continuity >= 2)
                {
                    Contraintes[4, ipos] = EvalBS[ipos + 2 * CDimension] * C1Coeff2;
                }
                if (Continuity >= 3)
                {
                    Contraintes[5, ipos] = EvalBS[ipos + 3 * CDimension] * C1Coeff3;
                }
            }
        }

        //  calculate the coefficients of extension
        int ii, jj, kk;
        NCollection_Array1<double> ExtraCoeffs = new(1, Csize * CDimension);
        ExtraCoeffs.Init(0.0);

        for (ii = 1; ii <= Csize; ii++)
        {

            for (jj = 1; jj <= Csize; jj++)
            {

                for (kk = 1; kk <= CDimension; kk++)
                {
                    ExtraCoeffs[kk + (jj - 1) * CDimension] += MatCoefs[ii, jj] * Contraintes[ii, kk];
                }
            }
        }

        //  calculate the poles of extension
        NCollection_Array1<double> ExtrapPoles = new(1, Csize * CDimension);
        PLib.CoefficientsPoles(CDimension,
                               ExtraCoeffs,
                               PLib.NoWeights(),
                               ExtrapPoles,
                               PLib.NoWeights());

        //  calculate the nodes of extension with multiplicities
        NCollection_Array1<double> ExtrapNoeuds = new(1, 2);
        ExtrapNoeuds[1] = 0.0;
        ExtrapNoeuds[2] = 1.0;
        NCollection_Array1<int> ExtrapMults = new(1, 2);
        ExtrapMults[1] = Csize;
        ExtrapMults[2] = Csize;

        // flat nodes of extension
        NCollection_Array1<double> FK2 = new(1, Csize * 2);
        BSplCLib.KnotSequence(ExtrapNoeuds, ExtrapMults, FK2);

        //  norm of the tangent at the connection point
        if (After)
        {
            BSplCLib.Eval(0.0, periodic_flag, 1, extrap_mode, Csize - 1, FK2, CDimension, ExtrapPoles.AsSpan(), Eadr);
        }
        else
        {
            BSplCLib.Eval(1.0, periodic_flag, 1, extrap_mode, Csize - 1, FK2, CDimension, ExtrapPoles.AsSpan(), Eadr);
        }

        for (ipos = 1; ipos <= CDimension; ipos++)
        {
            Tgte[ipos] = EvalBS[ipos + CDimension];
        }
        double L2 = Tgte.Norm();

        //  harmonisation of degrees
        NCollection_Array1<double> NewP2 = new(1, (CDegree + 1) * CDimension);
        NCollection_Array1<double> NewK2 = new(1, 2);
        NCollection_Array1<int> NewM2 = new(1, 2);
        if (Csize - 1 < CDegree)
        {
            BSplCLib.IncreaseDegree(Csize - 1,
                                    CDegree,
                                    false,
                                    CDimension,
                                    ExtrapPoles,
                                    ExtrapNoeuds,
                                    ExtrapMults,
                                    NewP2,
                                    NewK2,
                                    NewM2);
        }
        else
        {
            NewP2.Assign(ExtrapPoles);
            NewK2.Assign(ExtrapNoeuds);
            NewM2.Assign(ExtrapMults);
        }

        //  flat nodes of extension after harmonization of degrees
        NCollection_Array1<double> NewFK2 = new(1, (CDegree + 1) * 2);
        BSplCLib.KnotSequence(NewK2, NewM2, NewFK2);

        ////////////////////////////////////////////////////////////////////////
        //
        //    2.  concatenation C0
        //
        ////////////////////////////////////////////////////////////////////////

        //  ratio of reparametrization
        double Ratio = 1, Delta;
        if ((L1 > Precision.Confusion()) && (L2 > Precision.Confusion()))
        {
            Ratio = L2 / L1;
        }
        if ((Ratio < 1.0e-5) || (Ratio > 1.0e5))
        {
            Ratio = 1;
        }

        if (After)
        {
            //    do not touch the first BSpline
            Delta = Ratio * NewFK2[NewFK2.Lower()] - FlatKnots[FlatKnots.Upper()];
        }
        else
        {
            //    do not touch the second BSpline
            Delta = Ratio * NewFK2[NewFK2.Upper()] - FlatKnots[FlatKnots.Lower()];
        }

        //  result of the concatenation
        int NbP1 = NumPoles, NbP2 = CDegree + 1;
        int NbK1 = FlatKnots.Length(), NbK2 = 2 * (CDegree + 1);
        NCollection_Array1<double> NewPoles = new(1, (NbP1 + NbP2 - 1) * CDimension);
        NCollection_Array1<double> NewFlats = new(1, NbK1 + NbK2 - CDegree - 2);

        //  poles
        int indNP, indP, indEP;
        if (After)
        {

            for (ii = 1; ii <= NbP1 + NbP2 - 1; ii++)
            {

                for (jj = 1; jj <= CDimension; jj++)
                {
                    indNP = (ii - 1) * CDimension + jj;
                    indP = (ii - 1) * CDimension + jj - 1;
                    indEP = (ii - NbP1) * CDimension + jj;
                    if (ii < NbP1)
                    {
                        NewPoles[indNP] = Padr[indP];
                    }
                    else
                    {
                        NewPoles[indNP] = NewP2[indEP];
                    }
                }
            }
        }
        else
        {

            for (ii = 1; ii <= NbP1 + NbP2 - 1; ii++)
            {

                for (jj = 1; jj <= CDimension; jj++)
                {
                    indNP = (ii - 1) * CDimension + jj;
                    indEP = (ii - 1) * CDimension + jj;
                    indP = (ii - NbP2) * CDimension + jj - 1;
                    if (ii < NbP2)
                    {
                        NewPoles[indNP] = NewP2[indEP];
                    }
                    else
                    {
                        NewPoles[indNP] = Padr[indP];
                    }
                }
            }
        }

        //  flat nodes
        if (After)
        {
            //    start with the nodes of the initial surface

            for (ii = 1; ii < NbK1; ii++)
            {
                NewFlats[ii] = FlatKnots[FlatKnots.Lower() + ii - 1];
            }
            //    continue with the reparameterized nodes of the extension

            for (ii = 1; ii <= NbK2 - CDegree - 1; ii++)
            {
                NewFlats[NbK1 + ii - 1] = Ratio * NewFK2[NewFK2.Lower() + ii + CDegree] - Delta;
            }
        }
        else
        {
            //    start with the reparameterized nodes of the extension

            for (ii = 1; ii < NbK2 - CDegree; ii++)
            {
                NewFlats[ii] = Ratio * NewFK2[NewFK2.Lower() + ii - 1] - Delta;
            }
            //    continue with the nodes of the initial surface

            for (ii = 2; ii <= NbK1; ii++)
            {
                NewFlats[NbK2 + ii - CDegree - 2] = FlatKnots[FlatKnots.Lower() + ii - 1];
            }
        }

        ////////////////////////////////////////////////////////////////////////
        //
        //    3.  reduction of multiplicite at the node of connection
        //
        ////////////////////////////////////////////////////////////////////////

        //  number of separate nodes
        int KLength = 1;

        for (ii = 2; ii <= NbK1 + NbK2 - CDegree - 2; ii++)
        {
            if (NewFlats[ii] != NewFlats[ii - 1])
            {
                KLength++;
            }
        }

        //  flat nodes --> nodes + multiplicities
        NCollection_Array1<double> NewKnots = new(1, KLength);
        NCollection_Array1<int> NewMults = new(1, KLength);
        NewMults.Init(1);
        jj = 1;
        NewKnots[jj] = NewFlats[1];

        for (ii = 2; ii <= NbK1 + NbK2 - CDegree - 2; ii++)
        {
            if (NewFlats[ii] == NewFlats[ii - 1])
            {
                NewMults[jj]++;
            }
            else
            {
                jj++;
                NewKnots[jj] = NewFlats[ii];
            }
        }

        //  reduction of multiplicity at the second or the last but one node
        int Index = 2, M = CDegree;
        if (After)
        {
            Index = KLength - 1;
        }
        NCollection_Array1<double> ResultPoles = new(1, (NbP1 + NbP2 - 1) * CDimension);
        NCollection_Array1<double> ResultKnots = new(1, KLength);
        NCollection_Array1<int> ResultMults = new(1, KLength);
        double Tol = 1.0e-6;
        bool Ok = true;

        while ((M > CDegree - Continuity) && Ok)
        {
            Ok = RemoveKnot(Index,
                            M - 1,
                            CDegree,
                            false,
                            CDimension,
                            NewPoles,
                            NewKnots,
                            NewMults,
                            ResultPoles,
                            ResultKnots,
                            ResultMults,
                            Tol);
            if (Ok)
            {
                M--;
            }
        }

        if (M == CDegree)
        {
            //    number of poles of the concatenation
            NbPolesResult = NbP1 + NbP2 - 1;
            //    the poles of the concatenation
            int PLength = NbPolesResult * CDimension;

            for (jj = 1; jj <= PLength; jj++)
            {
                PRadr[jj - 1] = NewPoles[jj];
            }

            //    flat nodes of the concatenation
            int ideb = 0;

            for (jj = 0; jj < NewKnots.Length(); jj++)
            {
                for (ii = 0; ii < NewMults[jj + 1]; ii++)
                {
                    KRadr[ideb + ii] = NewKnots[jj + 1];
                }
                ideb += NewMults[jj + 1];
            }
            NbKnotsRsult = ideb;
        }

        else
        {
            //    number of poles of the result
            NbPolesResult = NbP1 + NbP2 - 1 - CDegree + M;
            //    the poles of the result
            int PLength = NbPolesResult * CDimension;

            for (jj = 0; jj < PLength; jj++)
            {
                PRadr[jj] = ResultPoles[jj + 1];
            }

            //    flat nodes of the result
            int ideb = 0;

            for (jj = 0; jj < ResultKnots.Length(); jj++)
            {
                for (ii = 0; ii < ResultMults[jj + 1]; ii++)
                {
                    KRadr[ideb + ii] = ResultKnots[jj + 1];
                }
                ideb += ResultMults[jj + 1];
            }
            NbKnotsRsult = ideb;
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
    /// Computes the parametric tolerance <paramref name="UTolerance"/> such that |t1 - t0| &lt; UTolerance implies
    /// |C(t1) - C(t0)| &lt; Tolerance3D, from an upper bound of the derivative of the curve (NumPoles poles of
    /// dimension ArrayDimension, optional weights, flat knots).
    /// </summary>
    public static void Resolution(ReadOnlySpan<double> PolesArray, int ArrayDimension, int NumPoles,
                                  NCollection_Array1<double>? Weights, NCollection_Array1<double> FlatKnots, int Degree,
                                  double Tolerance3D, out double UTolerance)
    {
        int ii, num_poles, ii_index, jj_index, ii_inDim;
        int lower, upper, ii_minus, jj, ii_miDim;
        int Deg1 = Degree + 1;
        int Deg2 = (Degree << 1) + 1;
        double value, factor, W, min_weights, inverse;
        double pa_ii_inDim_0, pa_ii_inDim_1, pa_ii_inDim_2, pa_ii_inDim_3;
        double pa_ii_miDim_0, pa_ii_miDim_1, pa_ii_miDim_2, pa_ii_miDim_3;
        double wg_ii_index, wg_ii_minus;
        double max_derivative;
        ReadOnlySpan<double> FK = FlatKnots.AsSpan();
        ReadOnlySpan<double> PA = PolesArray;
        max_derivative = 0.0e0;
        num_poles = FlatKnots.Length() - Deg1;
        switch (ArrayDimension)
        {
            case 2:
            {
                if (Weights != null)
                {
                    ReadOnlySpan<double> WG = Weights!.AsSpan();
                    min_weights = WG[0];

                    for (ii = 1; ii < NumPoles; ii++)
                    {
                        W = WG[ii];
                        if (W < min_weights)
                        {
                            min_weights = W;
                        }
                    }

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % NumPoles;
                        ii_inDim = ii_index << 1;
                        ii_minus = (ii - 1) % NumPoles;
                        ii_miDim = ii_minus << 1;
                        pa_ii_inDim_0 = PA[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_1 = PA[ii_inDim];
                        pa_ii_miDim_0 = PA[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_1 = PA[ii_miDim];
                        wg_ii_index = WG[ii_index];
                        wg_ii_minus = WG[ii_minus];
                        inverse = FK[ii + Degree] - FK[ii];
                        inverse = 1.0e0 / inverse;
                        lower = ii - Deg1;
                        if (lower < 0)
                        {
                            lower = 0;
                        }
                        upper = Deg2 + ii;
                        if (upper > num_poles)
                        {
                            upper = num_poles;
                        }

                        for (jj = lower; jj < upper; jj++)
                        {
                            jj_index = jj % NumPoles;
                            jj_index = jj_index << 1;
                            value = 0.0e0;
                            factor = (((PA[jj_index] - pa_ii_inDim_0) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_0) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((PA[jj_index] - pa_ii_inDim_1) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_1) * wg_ii_minus));
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
                        ii_index = ii % NumPoles;
                        ii_index = ii_index << 1;
                        ii_minus = (ii - 1) % NumPoles;
                        ii_minus = ii_minus << 1;
                        inverse = FK[ii + Degree] - FK[ii];
                        inverse = 1.0e0 / inverse;
                        value = 0.0e0;
                        factor = PA[ii_index] - PA[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = PA[ii_index] - PA[ii_minus];
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
                if (Weights != null)
                {
                    ReadOnlySpan<double> WG = Weights!.AsSpan();
                    min_weights = WG[0];

                    for (ii = 1; ii < NumPoles; ii++)
                    {
                        W = WG[ii];
                        if (W < min_weights)
                        {
                            min_weights = W;
                        }
                    }

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % NumPoles;
                        ii_inDim = (ii_index << 1) + ii_index;
                        ii_minus = (ii - 1) % NumPoles;
                        ii_miDim = (ii_minus << 1) + ii_minus;
                        pa_ii_inDim_0 = PA[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_1 = PA[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_2 = PA[ii_inDim];
                        pa_ii_miDim_0 = PA[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_1 = PA[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_2 = PA[ii_miDim];
                        wg_ii_index = WG[ii_index];
                        wg_ii_minus = WG[ii_minus];
                        inverse = FK[ii + Degree] - FK[ii];
                        inverse = 1.0e0 / inverse;
                        lower = ii - Deg1;
                        if (lower < 0)
                        {
                            lower = 0;
                        }
                        upper = Deg2 + ii;
                        if (upper > num_poles)
                        {
                            upper = num_poles;
                        }

                        for (jj = lower; jj < upper; jj++)
                        {
                            jj_index = jj % NumPoles;
                            jj_index = (jj_index << 1) + jj_index;
                            value = 0.0e0;
                            factor = (((PA[jj_index] - pa_ii_inDim_0) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_0) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((PA[jj_index] - pa_ii_inDim_1) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_1) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((PA[jj_index] - pa_ii_inDim_2) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_2) * wg_ii_minus));
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
                        ii_index = ii % NumPoles;
                        ii_index = (ii_index << 1) + ii_index;
                        ii_minus = (ii - 1) % NumPoles;
                        ii_minus = (ii_minus << 1) + ii_minus;
                        inverse = FK[ii + Degree] - FK[ii];
                        inverse = 1.0e0 / inverse;
                        value = 0.0e0;
                        factor = PA[ii_index] - PA[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = PA[ii_index] - PA[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = PA[ii_index] - PA[ii_minus];
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
                if (Weights != null)
                {
                    ReadOnlySpan<double> WG = Weights!.AsSpan();
                    min_weights = WG[0];

                    for (ii = 1; ii < NumPoles; ii++)
                    {
                        W = WG[ii];
                        if (W < min_weights)
                        {
                            min_weights = W;
                        }
                    }

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % NumPoles;
                        ii_inDim = ii_index << 2;
                        ii_minus = (ii - 1) % NumPoles;
                        ii_miDim = ii_minus << 2;
                        pa_ii_inDim_0 = PA[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_1 = PA[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_2 = PA[ii_inDim];
                        ii_inDim++;
                        pa_ii_inDim_3 = PA[ii_inDim];
                        pa_ii_miDim_0 = PA[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_1 = PA[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_2 = PA[ii_miDim];
                        ii_miDim++;
                        pa_ii_miDim_3 = PA[ii_miDim];
                        wg_ii_index = WG[ii_index];
                        wg_ii_minus = WG[ii_minus];
                        inverse = FK[ii + Degree] - FK[ii];
                        inverse = 1.0e0 / inverse;
                        lower = ii - Deg1;
                        if (lower < 0)
                        {
                            lower = 0;
                        }
                        upper = Deg2 + ii;
                        if (upper > num_poles)
                        {
                            upper = num_poles;
                        }

                        for (jj = lower; jj < upper; jj++)
                        {
                            jj_index = jj % NumPoles;
                            jj_index = jj_index << 2;
                            value = 0.0e0;
                            factor = (((PA[jj_index] - pa_ii_inDim_0) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_0) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((PA[jj_index] - pa_ii_inDim_1) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_1) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((PA[jj_index] - pa_ii_inDim_2) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_2) * wg_ii_minus));
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            jj_index++;
                            factor = (((PA[jj_index] - pa_ii_inDim_3) * wg_ii_index)
                                                - ((PA[jj_index] - pa_ii_miDim_3) * wg_ii_minus));
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
                        ii_index = ii % NumPoles;
                        ii_index = ii_index << 2;
                        ii_minus = (ii - 1) % NumPoles;
                        ii_minus = ii_minus << 2;
                        inverse = FK[ii + Degree] - FK[ii];
                        inverse = 1.0e0 / inverse;
                        value = 0.0e0;
                        factor = PA[ii_index] - PA[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = PA[ii_index] - PA[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = PA[ii_index] - PA[ii_minus];
                        if (factor < 0)
                        {
                            factor = -factor;
                        }
                        value += factor;
                        ii_index++;
                        ii_minus++;
                        factor = PA[ii_index] - PA[ii_minus];
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
                if (Weights != null)
                {
                    ReadOnlySpan<double> WG = Weights!.AsSpan();
                    min_weights = WG[0];

                    for (ii = 1; ii < NumPoles; ii++)
                    {
                        W = WG[ii];
                        if (W < min_weights)
                        {
                            min_weights = W;
                        }
                    }

                    for (ii = 1; ii < num_poles; ii++)
                    {
                        ii_index = ii % NumPoles;
                        ii_inDim = ii_index * ArrayDimension;
                        ii_minus = (ii - 1) % NumPoles;
                        ii_miDim = ii_minus * ArrayDimension;
                        wg_ii_index = WG[ii_index];
                        wg_ii_minus = WG[ii_minus];
                        inverse = FK[ii + Degree] - FK[ii];
                        inverse = 1.0e0 / inverse;
                        lower = ii - Deg1;
                        if (lower < 0)
                        {
                            lower = 0;
                        }
                        upper = Deg2 + ii;
                        if (upper > num_poles)
                        {
                            upper = num_poles;
                        }

                        for (jj = lower; jj < upper; jj++)
                        {
                            jj_index = jj % NumPoles;
                            jj_index *= ArrayDimension;
                            value = 0.0e0;

                            for (kk = 0; kk < ArrayDimension; kk++)
                            {
                                factor = (((PA[jj_index + kk] - PA[ii_inDim + kk]) * wg_ii_index)
                                                    - ((PA[jj_index + kk] - PA[ii_miDim + kk]) * wg_ii_minus));
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
                        ii_index = ii % NumPoles;
                        ii_index *= ArrayDimension;
                        ii_minus = (ii - 1) % NumPoles;
                        ii_minus *= ArrayDimension;
                        inverse = FK[ii + Degree] - FK[ii];
                        inverse = 1.0e0 / inverse;
                        value = 0.0e0;

                        for (kk = 0; kk < ArrayDimension; kk++)
                        {
                            factor = PA[ii_index + kk] - PA[ii_minus + kk];
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
        max_derivative *= Degree;
        if (max_derivative > RealSmall())
        {
            UTolerance = Tolerance3D / max_derivative;
        }
        else
        {
            UTolerance = Tolerance3D / RealSmall();
        }
    }

    /// <summary>
    /// Computes the intervals of continuity <paramref name="theContinuity"/> of the curve within
    /// [theFirst, theLast] and returns their number; fills <paramref name="theIntervals"/> (resized to the number
    /// + 1, the ends being theFirst and theLast) unless null.
    /// </summary>
    public static int Intervals(NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults, int theDegree,
                                bool isPeriodic, int theContinuity, double theFirst, double theLast,
                                double theTolerance, NCollection_Array1<double>? theIntervals)
    {
        // remove all knots with multiplicity less or equal than (degree - continuity) except first and
        // last
        int aFirstIndex = isPeriodic ? 1 : FirstUKnotIndex(theDegree, theMults);
        int aLastIndex = isPeriodic ? theKnots.Length() : LastUKnotIndex(theDegree, theMults);
        NCollection_Array1<double> aNewKnots = new(1, aLastIndex - aFirstIndex + 1);
        int aNbNewKnots = 0;
        for (int anIndex = aFirstIndex; anIndex <= aLastIndex; anIndex++)
        {
            if (theMults[anIndex] > (theDegree - theContinuity) || anIndex == aFirstIndex
                || anIndex == aLastIndex)
            {
                aNbNewKnots++;
                aNewKnots[aNbNewKnots] = theKnots[anIndex];
            }
        }
        aNewKnots.Resize(1, aNbNewKnots, true);

        // the range boundaries
        double aCurFirst = theFirst;
        double aCurLast = theLast;
        double aPeriod = 0.0;
        int aFirstPeriod = 0;
        int aLastPeriod = 0;
        // move boundaries into period
        if (isPeriodic)
        {
            double aLower = theKnots.First();
            double anUpper = theKnots.Last();
            aPeriod = anUpper - aLower;

            while (aCurFirst < aLower)
            {
                aCurFirst += aPeriod;
                aFirstPeriod--;
            }
            while (aCurLast < aLower)
            {
                aCurLast += aPeriod;
                aLastPeriod--;
            }
            while (aCurFirst >= anUpper)
            {
                aCurFirst -= aPeriod;
                aFirstPeriod += 1;
            }
            while (aCurLast >= anUpper)
            {
                aCurLast -= aPeriod;
                aLastPeriod += 1;
            }
        }
        // locate the left and nearest knot for boundaries
        int anIndex1 = 0;
        int anIndex2 = 0;
        double aDummyDouble;
        // we use version of LocateParameter that doesn't need multiplicities
        LocateParameter(theDegree,
                        aNewKnots,
                        new NCollection_Array1<int>(),
                        aCurFirst,
                        false,
                        1,
                        aNbNewKnots,
                        out anIndex1,
                        out aDummyDouble);
        LocateParameter(theDegree,
                        aNewKnots,
                        new NCollection_Array1<int>(),
                        aCurLast,
                        false,
                        1,
                        aNbNewKnots,
                        out anIndex2,
                        out aDummyDouble);
        // the case when the beginning of the range coincides with the next knot
        if (anIndex1 < aNbNewKnots && Math.Abs(aNewKnots[anIndex1 + 1] - aCurFirst) < theTolerance)
        {
            anIndex1 += 1;
        }
        // the case when the ending of the range coincides with the current knot
        if (aNbNewKnots != 0 && Math.Abs(aNewKnots[anIndex2] - aCurLast) < theTolerance)
        {
            anIndex2 -= 1;
        }
        int aNbIntervals = anIndex2 - anIndex1 + 1 + (aLastPeriod - aFirstPeriod) * (aNbNewKnots - 1);

        // fill the interval array
        if (theIntervals != null)
        {
            theIntervals.Resize(1, aNbIntervals + 1, false);
            if (isPeriodic && aLastPeriod != aFirstPeriod)
            {
                int anIndex = 1;
                // part from the begging of range to the end of the first period
                for (int i = anIndex1; i < aNewKnots.Length(); i++, anIndex++)
                {
                    theIntervals.ChangeValue(anIndex) = aNewKnots[i] + aFirstPeriod * aPeriod;
                }
                // full periods
                for (int aPeriodNum = aFirstPeriod + 1; aPeriodNum < aLastPeriod; aPeriodNum++)
                {
                    for (int i = 1; i < aNewKnots.Length(); i++, anIndex++)
                    {
                        theIntervals.ChangeValue(anIndex) = aNewKnots[i] + aPeriodNum * aPeriod;
                    }
                }
                // part from the begging of the last period to the end of range
                for (int i = 1; i <= anIndex2; i++, anIndex++)
                {
                    theIntervals.ChangeValue(anIndex) = aNewKnots[i] + aLastPeriod * aPeriod;
                }
            }
            else
            {
                int anIndex = 1;
                for (int i = anIndex1; i <= anIndex2; i++, anIndex++)
                {
                    theIntervals.ChangeValue(anIndex) = aNewKnots[i] + aFirstPeriod * aPeriod;
                }
            }
            // update the first position (the begging of range doesn't coincide with the knot at anIndex1 in
            // general)
            theIntervals.ChangeValue(1) = theFirst;
            // write the ending of the range (we didn't write it at all)
            theIntervals.ChangeValue(aNbIntervals + 1) = theLast;
        }

        return aNbIntervals;
    }

    // array of flat knots for bezier curve of maximum 25 degree
    private static ReadOnlySpan<double> knots =>
    [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    ];

    /// <summary>
    /// The flat knots of a Bezier curve of degree <paramref name="Degree"/> (1 to 25): Degree + 1 zeros then
    /// Degree + 1 ones. OCCT returns a reference to the start of a static table; the port returns the
    /// 2 * (Degree + 1) values as a read-only span over it. OCCT's
    /// <c>NCollection_Array1&lt;double&gt; knots(BSplCLib::FlatBezierKnots(Degree), 1, 2 * (Degree + 1))</c> becomes
    /// <c>NCollection_Array1&lt;double&gt;.From(BSplCLib.FlatBezierKnots(Degree))</c> (a copy).
    /// </summary>
    public static ReadOnlySpan<double> FlatBezierKnots(int Degree)
    {
        Standard_OutOfRange_Raise_if(Degree < 1 || Degree > MaxDegree() || MaxDegree() != 25,
                                     "Bezier curve degree greater than maximal supported");

        return knots.Slice(25 - Degree, 2 * (Degree + 1));
    }

    private static readonly double[] THE_UNIT_WEIGHTS = CreateUnitWeights();

    private static double[] CreateUnitWeights()
    {
        double[] aData = new double[2049];
        for (int i = 0; i < 2049; ++i)
        {
            aData[i] = 1.0;
        }
        return aData;
    }

    /// <summary>
    /// An array of <paramref name="theNbElems"/> weights equal to 1, indexed from 1. Up to
    /// <see cref="MaxUnitWeightsSize"/> elements it is a non-owning view of a shared static array (no allocation),
    /// as in OCCT: shared, don't modify. Above that it is a new array.
    /// </summary>
    public static NCollection_Array1<double> UnitWeights(int theNbElems)
    {
        if (theNbElems <= MaxUnitWeightsSize())
        {
            // Non-owning view over global static array (zero allocation).
            return new NCollection_Array1<double>(THE_UNIT_WEIGHTS, 0, 1, theNbElems);
        }
        // Rare: allocate fresh array.
        NCollection_Array1<double> aResult = new(1, theNbElems);
        aResult.Init(1.0);
        return aResult;
    }

    /// <summary>The shared unit weights: <see cref="MaxUnitWeightsSize"/> values equal to 1, read-only.</summary>
    public static ReadOnlySpan<double> UnitWeightsData() => THE_UNIT_WEIGHTS;

    /// <summary>Evaluates a Bezier curve given by its poles in the cache (power-basis) form, on [0, 1].</summary>
    public static void CoefsD0(double U, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double>? Weights, out gp_Pnt P)
    {
        BSplCLib.CacheD0(U, Poles.Length() - 1, 0.0, 1.0, Poles, Weights, out P);
    }

    /// <inheritdoc cref="CoefsD0(double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, out gp_Pnt)"/>
    public static void CoefsD0(double U, NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double>? Weights, out gp_Pnt2d P)
    {
        BSplCLib.CacheD0(U, Poles.Length() - 1, 0.0, 1.0, Poles, Weights, out P);
    }

    /// <summary>Evaluates a cached Bezier curve and its first derivative on [0, 1].</summary>
    public static void CoefsD1(double U, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double>? Weights, out gp_Pnt P,
                               out gp_Vec Vec)
    {
        BSplCLib.CacheD1(U, Poles.Length() - 1, 0.0, 1.0, Poles, Weights, out P, out Vec);
    }

    /// <inheritdoc cref="CoefsD1(double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, out gp_Pnt, out gp_Vec)"/>
    public static void CoefsD1(double U, NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double>? Weights,
                               out gp_Pnt2d P, out gp_Vec2d Vec)
    {
        BSplCLib.CacheD1(U, Poles.Length() - 1, 0.0, 1.0, Poles, Weights, out P, out Vec);
    }

    /// <summary>Evaluates a cached Bezier curve and its first two derivatives on [0, 1].</summary>
    public static void CoefsD2(double U, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double>? Weights, out gp_Pnt P,
                               out gp_Vec Vec1, out gp_Vec Vec2)
    {
        BSplCLib.CacheD2(U, Poles.Length() - 1, 0.0, 1.0, Poles, Weights, out P, out Vec1, out Vec2);
    }

    /// <inheritdoc cref="CoefsD2(double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, out gp_Pnt, out gp_Vec, out gp_Vec)"/>
    public static void CoefsD2(double U, NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double>? Weights,
                               out gp_Pnt2d P, out gp_Vec2d Vec1, out gp_Vec2d Vec2)
    {
        BSplCLib.CacheD2(U, Poles.Length() - 1, 0.0, 1.0, Poles, Weights, out P, out Vec1, out Vec2);
    }

    /// <summary>Evaluates a cached Bezier curve and its first three derivatives on [0, 1].</summary>
    public static void CoefsD3(double U, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double>? Weights, out gp_Pnt P,
                               out gp_Vec Vec1, out gp_Vec Vec2, out gp_Vec Vec3)
    {
        BSplCLib.CacheD3(U, Poles.Length() - 1, 0.0, 1.0, Poles, Weights, out P, out Vec1, out Vec2, out Vec3);
    }

    /// <inheritdoc cref="CoefsD3(double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, out gp_Pnt, out gp_Vec, out gp_Vec, out gp_Vec)"/>
    public static void CoefsD3(double U, NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double>? Weights,
                               out gp_Pnt2d P, out gp_Vec2d Vec1, out gp_Vec2d Vec2, out gp_Vec2d Vec3)
    {
        BSplCLib.CacheD3(U, Poles.Length() - 1, 0.0, 1.0, Poles, Weights, out P, out Vec1, out Vec2, out Vec3);
    }

    /// <summary>Converts the poles of a Bezier curve into its cache (power-basis) coefficients.</summary>
    public static void PolesCoefficients(NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<gp_Pnt> CachePoles)
    {
        BSplCLib.PolesCoefficients(Poles, PLib.NoWeights(), CachePoles, PLib.NoWeights());
    }

    /// <inheritdoc cref="PolesCoefficients(NCollection_Array1{gp_Pnt}, NCollection_Array1{gp_Pnt})"/>
    public static void PolesCoefficients(NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<gp_Pnt2d> CachePoles)
    {
        BSplCLib.PolesCoefficients(Poles, PLib.NoWeights(), CachePoles, PLib.NoWeights());
    }
}
