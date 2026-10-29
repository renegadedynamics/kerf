// Port of: src/FoundationClasses/TKMath/PLib/PLib.hxx
// Port of: src/FoundationClasses/TKMath/PLib/PLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Polynomial functions library: evaluation of polynomials and of their derivatives, rational derivatives,
/// Lagrange and Hermite interpolation, conversion between power-basis coefficients and Bezier poles.
/// </summary>
/// <remarks>
/// Weight arrays are optional for some functions: <see cref="NoWeights"/> (null) means no weights.
/// OCCT's <c>double&amp;</c> array starts are spans: <see cref="ReadOnlySpan{T}"/> where the function only reads,
/// <see cref="Span{T}"/> where it writes (docs/decisions.md). A caller passing <c>A(A.Lower())</c> in C++ passes
/// <c>A.AsSpan()</c>.
/// </remarks>
public static class PLib
{
    /// <summary>No weights: the argument for non-rational functions (null).</summary>
    public static NCollection_Array1<double>? NoWeights() => null;

    /// <summary>No weights: the argument for non-rational functions (null).</summary>
    public static NCollection_Array2<double>? NoWeights2() => null;

    // To convert points array into Real ..
    // *********************************

    /// <summary>Copies the coordinates of the poles into FP.</summary>
    public static void SetPoles(NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> FP)
    {
        int j = FP.Lower();
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();

        for (int i = PLower; i <= PUpper; i++)
        {
            ref readonly gp_Pnt2d P = ref Poles[i];
            FP[j] = P.Coord(1);
            j++;
            FP[j] = P.Coord(2);
            j++;
        }
    }

    /// <summary>Copies the coordinates of the poles, multiplied by their weights, and the weights into FP.</summary>
    public static void SetPoles(NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> Weights, NCollection_Array1<double> FP)
    {
        int j = FP.Lower();
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();

        for (int i = PLower; i <= PUpper; i++)
        {
            double w = Weights[i];
            ref readonly gp_Pnt2d P = ref Poles[i];
            FP[j] = P.Coord(1) * w;
            j++;
            FP[j] = P.Coord(2) * w;
            j++;
            FP[j] = w;
            j++;
        }
    }

    /// <summary>Gets the coordinates of the poles from FP.</summary>
    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt2d> Poles)
    {
        int j = FP.Lower();
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();

        for (int i = PLower; i <= PUpper; i++)
        {
            ref gp_Pnt2d P = ref Poles[i];
            P.SetCoord(1, FP[j]);
            j++;
            P.SetCoord(2, FP[j]);
            j++;
        }
    }

    /// <summary>Gets the coordinates of the poles and their weights from FP (homogeneous coordinates).</summary>
    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> Weights)
    {
        int j = FP.Lower();
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();

        for (int i = PLower; i <= PUpper; i++)
        {
            double w = FP[j + 2];
            Weights[i] = w;
            ref gp_Pnt2d P = ref Poles[i];
            P.SetCoord(1, FP[j] / w);
            j++;
            P.SetCoord(2, FP[j] / w);
            j++;
            j++;
        }
    }

    /// <summary>Copies the coordinates of the poles into FP.</summary>
    public static void SetPoles(NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> FP)
    {
        int j = FP.Lower();
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();

        for (int i = PLower; i <= PUpper; i++)
        {
            ref readonly gp_Pnt P = ref Poles[i];
            FP[j] = P.Coord(1);
            j++;
            FP[j] = P.Coord(2);
            j++;
            FP[j] = P.Coord(3);
            j++;
        }
    }

    /// <summary>Copies the coordinates of the poles, multiplied by their weights, and the weights into FP.</summary>
    public static void SetPoles(NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> Weights, NCollection_Array1<double> FP)
    {
        int j = FP.Lower();
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();

        for (int i = PLower; i <= PUpper; i++)
        {
            double w = Weights[i];
            ref readonly gp_Pnt P = ref Poles[i];
            FP[j] = P.Coord(1) * w;
            j++;
            FP[j] = P.Coord(2) * w;
            j++;
            FP[j] = P.Coord(3) * w;
            j++;
            FP[j] = w;
            j++;
        }
    }

    /// <summary>Gets the coordinates of the poles from FP.</summary>
    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt> Poles)
    {
        int j = FP.Lower();
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();

        for (int i = PLower; i <= PUpper; i++)
        {
            ref gp_Pnt P = ref Poles[i];
            P.SetCoord(1, FP[j]);
            j++;
            P.SetCoord(2, FP[j]);
            j++;
            P.SetCoord(3, FP[j]);
            j++;
        }
    }

    /// <summary>Gets the coordinates of the poles and their weights from FP (homogeneous coordinates).</summary>
    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> Weights)
    {
        int j = FP.Lower();
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();

        for (int i = PLower; i <= PUpper; i++)
        {
            double w = FP[j + 3];
            Weights[i] = w;
            ref gp_Pnt P = ref Poles[i];
            P.SetCoord(1, FP[j] / w);
            j++;
            P.SetCoord(2, FP[j] / w);
            j++;
            P.SetCoord(3, FP[j] / w);
            j++;
            j++;
        }
    }

    // Pascal's triangle up to BSplCLib::MaxDegree(), computed once (a compile-time table in OCCT)
    private const int BinomMaxDegree = 25; // BSplCLib::MaxDegree()

    private static readonly int[] THE_BINOM = BuildBinom();

    private static int[] BuildBinom()
    {
        const int aSize = BinomMaxDegree + 1;
        int[] aBinom = new int[aSize * aSize];
        // Initialize first row: C(0,0) = 1
        aBinom[0] = 1;

        // Build Pascal's triangle row by row
        for (int i = 1; i <= BinomMaxDegree; ++i)
        {
            // First and last elements are always 1
            aBinom[i * aSize] = 1;
            aBinom[i * aSize + i] = 1;

            // Use recurrence relation for middle elements
            for (int j = 1; j < i; ++j)
            {
                aBinom[i * aSize + j] = aBinom[(i - 1) * aSize + j - 1] + aBinom[(i - 1) * aSize + j];
            }
        }
        return aBinom;
    }

    /// <summary>Returns the binomial coefficient C(N, P). N must be &lt;= <see cref="BSplCLib.MaxDegree"/>.</summary>
    public static double Bin(int N, int P)
    {
        Standard_OutOfRange_Raise_if(N < 0 || N > BSplCLib.MaxDegree(),
                                     "PLib::Bin: degree N is out of supported range [0, 25]");
        Standard_OutOfRange_Raise_if(P < 0 || P > N,
                                     "PLib::Bin: parameter P is out of valid range [0, N]");

        return (double)THE_BINOM[N * (BinomMaxDegree + 1) + P];
    }

    /// <summary>
    /// Computes the derivatives of a ratio at order <paramref name="N"/> in dimension <paramref name="Dimension"/>.
    /// <paramref name="Ders"/> holds the input derivatives from 0 to min(N, Degree), each as
    /// x(1), ..., x(Dimension), w; higher orders are taken as 0. If <paramref name="All"/> is false, only the
    /// derivative of order N is computed into <paramref name="RDers"/> (Dimension values); otherwise all orders
    /// 0 to N (Dimension * (N + 1) values).
    /// </summary>
    public static void RationalDerivative(int Degree, int N, int Dimension, ReadOnlySpan<double> Ders, Span<double> RDers, bool All = true)
    {
        int DerivativeRequest = N;
        //
        // Our purpose is to compute f = (u/v) derivated N = DerivativeRequest times
        //
        //  We Write  u = fv
        //  Let C(N,P) be the binomial
        //
        //        then we have
        //
        //         (q)                             (p)   (q-p)
        //        u    =     SUM          C (q,p) f     v
        //                p = 0 to q
        //
        //
        //        Therefore
        //
        //
        //         (q)         (   (q)                               (p)   (q-p)   )
        //        f    = (1/v) (  u    -     SUM            C (q,p) f     v        )
        //                     (          p = 0 to q-1                             )
        //
        //
        // make arrays for the binomial since computing it each time could raise a performance
        // issue
        // As oppose to the method below the <Der> array is organized in the following
        // fashion :
        //
        //  u (1)     u (2) ....   u  (Dimension)     v (1)
        //
        //   (1)       (1)          (1)                (1)
        //  u (1)     u (2) ....   u  (Dimension)     v (1)
        //
        //   ............................................
        //
        //   (Degree)  (Degree)     (Degree)           (Degree)
        //  u (1)     u (2) ....   u  (Dimension)     v (1)
        //
        //
        double Inverse;
        ReadOnlySpan<double> PolesArray = Ders;
        scoped Span<double> RationalArray = RDers;
        scoped Span<double> derivative_storage = default;
        double Factor;
        int ii, Index, OtherIndex, Index1, Index2, jj;
        if (Dimension == 3)
        {
            int DeRequest1 = DerivativeRequest + 1;
            int MinDegRequ = DerivativeRequest;
            if (MinDegRequ > Degree)
            {
                MinDegRequ = Degree;
            }
            Span<double> binomial_array = DeRequest1 <= 128 ? stackalloc double[DeRequest1] : new double[DeRequest1];

            for (ii = 0; ii < DeRequest1; ii++)
            {
                binomial_array[ii] = 1.0e0;
            }
            if (!All)
            {
                int DimDeRequ1 = (DeRequest1 << 1) + DeRequest1;
                derivative_storage = DimDeRequ1 <= 128 ? stackalloc double[DimDeRequ1] : new double[DimDeRequ1];
                RationalArray = derivative_storage;
            }

            Inverse = 1.0e0 / PolesArray[3];
            Index = 0;
            Index2 = -6;
            OtherIndex = 0;

            for (ii = 0; ii <= MinDegRequ; ii++)
            {
                Index2 += 3;
                Index1 = Index2;
                RationalArray[Index] = PolesArray[OtherIndex];
                Index++;
                OtherIndex++;
                RationalArray[Index] = PolesArray[OtherIndex];
                Index++;
                OtherIndex++;
                RationalArray[Index] = PolesArray[OtherIndex];
                Index -= 2;
                OtherIndex += 2;

                for (jj = ii - 1; jj >= 0; jj--)
                {
                    Factor = binomial_array[jj] * PolesArray[((ii - jj) << 2) + 3];
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index++;
                    Index1++;
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index++;
                    Index1++;
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index -= 2;
                    Index1 -= 5;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }
                RationalArray[Index] *= Inverse;
                Index++;
                RationalArray[Index] *= Inverse;
                Index++;
                RationalArray[Index] *= Inverse;
                Index++;
            }

            for (ii = MinDegRequ + 1; ii <= DerivativeRequest; ii++)
            {
                Index2 += 3;
                Index1 = Index2;
                RationalArray[Index] = 0.0e0;
                Index++;
                RationalArray[Index] = 0.0e0;
                Index++;
                RationalArray[Index] = 0.0e0;
                Index -= 2;

                for (jj = ii - 1; jj >= ii - MinDegRequ; jj--)
                {
                    Factor = binomial_array[jj] * PolesArray[((ii - jj) << 2) + 3];
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index++;
                    Index1++;
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index++;
                    Index1++;
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index -= 2;
                    Index1 -= 5;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }
                RationalArray[Index] *= Inverse;
                Index++;
                RationalArray[Index] *= Inverse;
                Index++;
                RationalArray[Index] *= Inverse;
                Index++;
            }

            if (!All)
            {
                RationalArray = RDers;
                int DimDeRequ = (DerivativeRequest << 1) + DerivativeRequest;
                RationalArray[0] = derivative_storage[DimDeRequ];
                DimDeRequ++;
                RationalArray[1] = derivative_storage[DimDeRequ];
                DimDeRequ++;
                RationalArray[2] = derivative_storage[DimDeRequ];
            }
        }
        else
        {
            int kk;
            int Dimension1 = Dimension + 1;
            int Dimension2 = Dimension << 1;
            int DeRequest1 = DerivativeRequest + 1;
            int MinDegRequ = DerivativeRequest;
            if (MinDegRequ > Degree)
            {
                MinDegRequ = Degree;
            }
            Span<double> binomial_array = DeRequest1 <= 128 ? stackalloc double[DeRequest1] : new double[DeRequest1];

            for (ii = 0; ii < DeRequest1; ii++)
            {
                binomial_array[ii] = 1.0e0;
            }
            if (!All)
            {
                int DimDeRequ1 = Dimension * DeRequest1;
                derivative_storage = DimDeRequ1 <= 128 ? stackalloc double[DimDeRequ1] : new double[DimDeRequ1];
                RationalArray = derivative_storage;
            }

            Inverse = 1.0e0 / PolesArray[Dimension];
            Index = 0;
            Index2 = -Dimension2;
            OtherIndex = 0;

            for (ii = 0; ii <= MinDegRequ; ii++)
            {
                Index2 += Dimension;
                Index1 = Index2;

                for (kk = 0; kk < Dimension; kk++)
                {
                    RationalArray[Index] = PolesArray[OtherIndex];
                    Index++;
                    OtherIndex++;
                }
                Index -= Dimension;
                ++OtherIndex;

                for (jj = ii - 1; jj >= 0; jj--)
                {
                    Factor = binomial_array[jj] * PolesArray[(ii - jj) * Dimension1 + Dimension];

                    for (kk = 0; kk < Dimension; kk++)
                    {
                        RationalArray[Index] -= Factor * RationalArray[Index1];
                        Index++;
                        Index1++;
                    }
                    Index -= Dimension;
                    Index1 -= Dimension2;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }

                for (kk = 0; kk < Dimension; kk++)
                {
                    RationalArray[Index] *= Inverse;
                    Index++;
                }
            }

            for (ii = MinDegRequ + 1; ii <= DerivativeRequest; ii++)
            {
                Index2 += Dimension;
                Index1 = Index2;

                for (kk = 0; kk < Dimension; kk++)
                {
                    RationalArray[Index] = 0.0e0;
                    Index++;
                }
                Index -= Dimension;

                for (jj = ii - 1; jj >= ii - MinDegRequ; jj--)
                {
                    Factor = binomial_array[jj] * PolesArray[(ii - jj) * Dimension1 + Dimension];

                    for (kk = 0; kk < Dimension; kk++)
                    {
                        RationalArray[Index] -= Factor * RationalArray[Index1];
                        Index++;
                        Index1++;
                    }
                    Index -= Dimension;
                    Index1 -= Dimension2;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }

                for (kk = 0; kk < Dimension; kk++)
                {
                    RationalArray[Index] *= Inverse;
                    Index++;
                }
            }

            if (!All)
            {
                RationalArray = RDers;
                int DimDeRequ = Dimension * DerivativeRequest;

                for (kk = 0; kk < Dimension; kk++)
                {
                    RationalArray[kk] = derivative_storage[DimDeRequ];
                    DimDeRequ++;
                }
            }
        }
    }

    /// <summary>
    /// Computes <paramref name="DerivativesRequest"/> derivatives of a ratio from the derivatives of the
    /// homogeneous poles (<paramref name="PolesDerivatives"/>, (DerivativesRequest + 1) * Dimension values) and of
    /// the weights (<paramref name="WeightsDerivatives"/>, DerivativesRequest + 1 values). The result and the poles
    /// derivatives may be the same array: the overwrite is non-destructive within the algorithm.
    /// </summary>
    public static void RationalDerivatives(int DerivativesRequest, int Dimension, ReadOnlySpan<double> PolesDerivatives,
                                           ReadOnlySpan<double> WeightsDerivatives, Span<double> RationalDerivates)
    {
        int DerivativeRequest = DerivativesRequest;
        //
        // Our purpose is to compute f = (u/v) derivated N times
        //
        //  We Write  u = fv
        //  Let C(N,P) be the binomial
        //
        //        then we have
        //
        //         (q)                             (p)   (q-p)
        //        u    =     SUM          C (q,p) f     v
        //                p = 0 to q
        //
        //
        //        Therefore
        //
        //
        //         (q)         (   (q)                               (p)   (q-p)   )
        //        f    = (1/v) (  u    -     SUM            C (q,p) f     v        )
        //                     (          p = 0 to q-1                             )
        //
        //
        // make arrays for the binomial since computing it each time could
        // raize a performance issue
        //
        double Inverse;
        ReadOnlySpan<double> PolesArray = PolesDerivatives;
        ReadOnlySpan<double> WeightsArray = WeightsDerivatives;
        Span<double> RationalArray = RationalDerivates;
        double Factor;

        int ii, Index, Index1, Index2, jj;
        int DeRequest1 = DerivativeRequest + 1;

        Span<double> binomial_array = DeRequest1 <= 128 ? stackalloc double[DeRequest1] : new double[DeRequest1];

        for (ii = 0; ii < DeRequest1; ii++)
        {
            binomial_array[ii] = 1.0e0;
        }
        Inverse = 1.0e0 / WeightsArray[0];
        if (Dimension == 3)
        {
            Index = 0;
            Index2 = -6;

            for (ii = 0; ii < DeRequest1; ii++)
            {
                Index2 += 3;
                Index1 = Index2;
                RationalArray[Index] = PolesArray[Index];
                Index++;
                RationalArray[Index] = PolesArray[Index];
                Index++;
                RationalArray[Index] = PolesArray[Index];
                Index -= 2;

                for (jj = ii - 1; jj >= 0; jj--)
                {
                    Factor = binomial_array[jj] * WeightsArray[ii - jj];
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index++;
                    Index1++;
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index++;
                    Index1++;
                    RationalArray[Index] -= Factor * RationalArray[Index1];
                    Index -= 2;
                    Index1 -= 5;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }
                RationalArray[Index] *= Inverse;
                Index++;
                RationalArray[Index] *= Inverse;
                Index++;
                RationalArray[Index] *= Inverse;
                Index++;
            }
        }
        else
        {
            int kk;
            int Dimension2 = Dimension << 1;
            Index = 0;
            Index2 = -Dimension2;

            for (ii = 0; ii < DeRequest1; ii++)
            {
                Index2 += Dimension;
                Index1 = Index2;

                for (kk = 0; kk < Dimension; kk++)
                {
                    RationalArray[Index] = PolesArray[Index];
                    Index++;
                }
                Index -= Dimension;

                for (jj = ii - 1; jj >= 0; jj--)
                {
                    Factor = binomial_array[jj] * WeightsArray[ii - jj];

                    for (kk = 0; kk < Dimension; kk++)
                    {
                        RationalArray[Index] -= Factor * RationalArray[Index1];
                        Index++;
                        Index1++;
                    }
                    Index -= Dimension;
                    Index1 -= Dimension2;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }

                for (kk = 0; kk < Dimension; kk++)
                {
                    RationalArray[Index] *= Inverse;
                    Index++;
                }
            }
        }
    }

    //=======================================================================
    // Evaluation of a polynomial and its derivatives. OCCT dispatches dimensions 1 to 15 to templates that keep
    // the accumulators in local arrays; the port runs the general loops for every dimension. The arithmetic is the
    // same, operation for operation, so the results are identical.
    //=======================================================================

    // Evaluation of only value. theCoeffs starts at the highest-degree coefficient (index theTop).
    private static void eval_poly0_runtime(Span<double> theResult, ReadOnlySpan<double> theCoeffs, int theTop,
                                           int theDegree, double thePar, int theDimension)
    {
        for (int i = 0; i < theDimension; ++i)
        {
            theResult[i] = theCoeffs[theTop + i];
        }

        int aCoeffs = theTop;
        for (int aDeg = 0; aDeg < theDegree; ++aDeg)
        {
            aCoeffs -= theDimension;
            for (int i = 0; i < theDimension; ++i)
            {
                theResult[i] = theResult[i] * thePar + theCoeffs[aCoeffs + i];
            }
        }
    }

    // Evaluation of value + 1st derivative
    private static void eval_poly1_runtime(Span<double> theResult, ReadOnlySpan<double> theCoeffs, int theTop,
                                           int theDegree, double thePar, int theDimension)
    {
        Span<double> aRes0 = theResult.Slice(0, theDimension);
        Span<double> aRes1 = theResult.Slice(theDimension, theDimension);

        for (int i = 0; i < theDimension; ++i)
        {
            aRes0[i] = theCoeffs[theTop + i];
            aRes1[i] = 0.0;
        }

        int aCoeffs = theTop;
        for (int aDeg = 0; aDeg < theDegree; ++aDeg)
        {
            aCoeffs -= theDimension;
            ReadOnlySpan<double> aC = theCoeffs.Slice(aCoeffs, theDimension);
            for (int i = 0; i < theDimension; ++i)
            {
                double aVal = aRes0[i];
                aRes1[i] = aRes1[i] * thePar + aVal;
                aRes0[i] = aVal * thePar + aC[i];
            }
        }
    }

    // Evaluation of value + 1st + 2nd derivatives
    private static void eval_poly2_runtime(Span<double> theResult, ReadOnlySpan<double> theCoeffs, int theTop,
                                           int theDegree, double thePar, int theDimension)
    {
        Span<double> aRes0 = theResult.Slice(0, theDimension);
        Span<double> aRes1 = theResult.Slice(theDimension, theDimension);
        Span<double> aRes2 = theResult.Slice(2 * theDimension, theDimension);

        for (int i = 0; i < theDimension; ++i)
        {
            aRes0[i] = theCoeffs[theTop + i];
            aRes1[i] = 0.0;
            aRes2[i] = 0.0;
        }

        int aCoeffs = theTop;
        for (int aDeg = 0; aDeg < theDegree; ++aDeg)
        {
            aCoeffs -= theDimension;
            ReadOnlySpan<double> aC = theCoeffs.Slice(aCoeffs, theDimension);
            for (int i = 0; i < theDimension; ++i)
            {
                double aD1 = aRes1[i];
                double aVal = aRes0[i];
                aRes2[i] = aRes2[i] * thePar + aD1 * 2.0;
                aRes1[i] = aD1 * thePar + aVal;
                aRes0[i] = aVal * thePar + aC[i];
            }
        }
    }

    /// <summary>
    /// Horner's method with synthetic division for derivatives: evaluates the polynomial of degree
    /// <paramref name="Degree"/> and dimension <paramref name="Dimension"/> and its derivatives up to
    /// <paramref name="DerivativeOrder"/> at <paramref name="U"/>. <paramref name="PolynomialCoeff"/> holds
    /// c0(1..Dimension), c1(1..Dimension), ..., cDegree(1..Dimension); <paramref name="Results"/> receives
    /// f(1..Dimension), f'(1..Dimension), ..., one row of Dimension values per order.
    /// </summary>
    public static void EvalPolynomial(double U, int DerivativeOrder, int Degree, int Dimension,
                                      ReadOnlySpan<double> PolynomialCoeff, Span<double> Results)
    //
    // the polynomial coefficients are assumed to be stored as follows :
    //                                                      0
    // [0]                  [Dimension -1]                 X     coefficient
    //                                                      1
    // [Dimension]          [Dimension + Dimension -1]     X     coefficient
    //                                                      2
    // [2 * Dimension]      [2 * Dimension + Dimension-1]  X     coefficient
    //
    //   ...................................................
    //
    //
    //                                                      d
    // [d * Dimension]      [d * Dimension + Dimension-1]  X     coefficient
    //
    //  where d is the Degree
    //
    {
        double Par = U;
        int DerivativeRequest = DerivativeOrder;
        int aCoeffs = Degree * Dimension;
        Span<double> aRes = Results;

        switch (DerivativeRequest)
        {
            case 1:
                {
                    eval_poly1_runtime(aRes, PolynomialCoeff, aCoeffs, Degree, Par, Dimension);
                    break;
                }
            case 2:
                {
                    eval_poly2_runtime(aRes, PolynomialCoeff, aCoeffs, Degree, Par, Dimension);
                    break;
                }
            default:
                {
                    // General case for DerivativeRequest > 2 (and 0)
                    int aResSize = (1 + DerivativeRequest) * Dimension;
                    for (int i = 0; i < aResSize; ++i)
                    {
                        aRes[i] = 0.0;
                    }

                    for (int aDeg = 0; aDeg <= Degree; ++aDeg)
                    {
                        int aPtr = aResSize - Dimension;
                        // Calculating derivatives of the polynomial
                        for (int aDeriv = DerivativeRequest; aDeriv > 0; --aDeriv)
                        {
                            int anOriginal = aPtr - Dimension;
                            for (int ind = 0; ind < Dimension; ++ind)
                            {
                                aRes[aPtr + ind] = aRes[aPtr + ind] * Par + aRes[anOriginal + ind] * aDeriv;
                            }
                            aPtr = anOriginal;
                        }
                        // Calculating the value of the polynomial
                        for (int ind = 0; ind < Dimension; ++ind)
                        {
                            aRes[aPtr + ind] = aRes[aPtr + ind] * Par + PolynomialCoeff[aCoeffs + ind];
                        }
                        aCoeffs -= Dimension;
                    }
                    break;
                }
        }
    }

    /// <summary>
    /// <see cref="EvalPolynomial"/> with no derivative. <paramref name="DegreeDimension"/> is Degree * Dimension,
    /// the index of the highest-degree coefficient.
    /// </summary>
    public static void NoDerivativeEvalPolynomial(double U, int Degree, int Dimension, int DegreeDimension,
                                                  ReadOnlySpan<double> PolynomialCoeff, Span<double> Results)
    {
        eval_poly0_runtime(Results, PolynomialCoeff, DegreeDimension, Degree, U, Dimension);
    }

    /// <summary>
    /// Evaluates the derivative of orders <paramref name="UDerivativeOrder"/> in U and
    /// <paramref name="VDerivativeOrder"/> in V of a polynomial of two variables at (U, V), applying
    /// <see cref="EvalPolynomial"/> twice. The coefficients are stored c00, c10, ..., cm0, c01, ..., cmn (each of
    /// Dimension values, m = UDegree, n = VDegree); <paramref name="Results"/> receives Dimension values.
    /// </summary>
    public static void EvalPoly2Var(double U, double V, int UDerivativeOrder, int VDerivativeOrder, int UDegree,
                                    int VDegree, int Dimension, ReadOnlySpan<double> PolynomialCoeff, Span<double> Results)
    {
        double UParameter = U, VParameter = V;
        int UDerivativeRequest = UDerivativeOrder, VDerivativeRequest = VDerivativeOrder;
        int Udim = (VDegree + 1) * Dimension, index = Udim * UDerivativeRequest;
        // OCCT allocates both work arrays as NCollection_Array1
        int aCurveSize = Udim * (UDerivativeRequest + 1);
        int aPointSize = Dimension * (VDerivativeRequest + 1);
        Span<double> Result = aCurveSize <= 128 ? stackalloc double[aCurveSize] : new double[aCurveSize];
        Span<double> Digit = aPointSize <= 128 ? stackalloc double[aPointSize] : new double[aPointSize];
        Span<double> ResultArray = Results;

        PLib.EvalPolynomial(UParameter, UDerivativeRequest, UDegree, Udim, PolynomialCoeff, Result);

        PLib.EvalPolynomial(VParameter, VDerivativeRequest, VDegree, Dimension, Result[index..], Digit);

        index = Dimension * VDerivativeRequest;

        for (int i = 0; i < Dimension; i++)
        {
            ResultArray[i] = Digit[index + i];
        }
    }

    /// <summary>
    /// Lagrange interpolation of <paramref name="Degree"/> + 1 points of dimension <paramref name="Dimension"/>
    /// (<paramref name="ValueArray"/>) at the parameters <paramref name="ParameterArray"/>, with derivatives up to
    /// <paramref name="DerivativeOrder"/>, into <paramref name="Results"/> (one row of Dimension values per order).
    /// Returns 0, or 1 if two parameters are equal.
    /// </summary>
    public static int EvalLagrange(double U, int DerivativeOrder, int Degree, int Dimension,
                                   ReadOnlySpan<double> ValueArray, ReadOnlySpan<double> ParameterArray, Span<double> Results)
    {
        double Parameter = U;
        //
        // the points  are assumed to be stored as follows in the Values array :
        //
        // [0]              [Dimension -1]                first  point   coefficients
        //
        // [Dimension]      [Dimension + Dimension -1]    second point   coefficients
        //
        // [2 * Dimension]  [2 * Dimension + Dimension-1] third  point   coefficients
        //
        //   ...................................................
        //
        //
        //
        // [d * Dimension]  [d * Dimension + Dimension-1] d + 1 point   coefficients
        //
        //  where d is the Degree
        //
        //  The ParameterArray stores the parameter value assign to each point in
        //  order described above, that is
        //  [0]   is assign to first  point
        //  [1]   is assign to second point
        //
        int ii, jj, kk, Index, Index1, ReturnCode = 0;
        int local_request = DerivativeOrder;
        double difference;
        ReadOnlySpan<double> PointsArray = ValueArray;
        Span<double> ResultArray = Results;

        if (local_request >= Degree)
        {
            local_request = Degree;
        }
        int aSize = (Degree + 1) * Dimension;
        Span<double> divided_differences_array = aSize <= 128 ? stackalloc double[aSize] : new double[aSize];
        //
        //  Build the divided differences array
        //

        for (ii = 0; ii < (Degree + 1) * Dimension; ii++)
        {
            divided_differences_array[ii] = PointsArray[ii];
        }

        for (ii = Degree; ii >= 0; ii--)
        {

            for (jj = Degree; jj > Degree - ii; jj--)
            {
                Index = jj * Dimension;
                Index1 = Index - Dimension;

                for (kk = 0; kk < Dimension; kk++)
                {
                    divided_differences_array[Index + kk] -= divided_differences_array[Index1 + kk];
                }
                difference = ParameterArray[jj] - ParameterArray[jj - Degree - 1 + ii];
                if (Math.Abs(difference) < RealSmall())
                {
                    ReturnCode = 1;
                    goto FINISH;
                }
                difference = 1.0e0 / difference;

                for (kk = 0; kk < Dimension; kk++)
                {
                    divided_differences_array[Index + kk] *= difference;
                }
            }
        }
        //
        //
        // Evaluate the divided difference array polynomial which expresses as
        //
        //  P(t) = [t1] P + (t - t1) [t1,t2] P + (t - t1)(t - t2)[t1,t2,t3] P + ...
        //         + (t - t1)(t - t2)(t - t3)...(t - td) [t1,t2,...,td+1] P
        //
        // The ith slot in the divided_differences_array is [t1,t2,...,ti+1]
        //
        //
        Index = Degree * Dimension;

        for (kk = 0; kk < Dimension; kk++)
        {
            ResultArray[kk] = divided_differences_array[Index + kk];
        }

        for (ii = Dimension; ii < (local_request + 1) * Dimension; ii++)
        {
            ResultArray[ii] = 0.0e0;
        }

        for (ii = Degree; ii >= 1; ii--)
        {
            difference = Parameter - ParameterArray[ii - 1];

            for (jj = local_request; jj > 0; jj--)
            {
                Index = jj * Dimension;
                Index1 = Index - Dimension;

                for (kk = 0; kk < Dimension; kk++)
                {
                    ResultArray[Index + kk] *= difference;
                    ResultArray[Index + kk] += ResultArray[Index1 + kk] * (double)jj;
                }
            }
            Index = (ii - 1) * Dimension;

            for (kk = 0; kk < Dimension; kk++)
            {
                ResultArray[kk] *= difference;
                ResultArray[kk] += divided_differences_array[Index + kk];
            }
        }
    FINISH:
        return (ReturnCode);
    }

    /// <summary>
    /// Cubic Hermite interpolation of two points (<paramref name="ValueArray"/>) and their derivatives
    /// (<paramref name="DerivativeArray"/>) at the two parameters <paramref name="ParameterArray"/>, with
    /// derivatives up to <paramref name="DerivativeOrder"/>, into <paramref name="Results"/>. Returns 0.
    /// </summary>
    public static int EvalCubicHermite(double U, int DerivativeOrder, int Dimension, ReadOnlySpan<double> ValueArray,
                                       ReadOnlySpan<double> DerivativeArray, ReadOnlySpan<double> ParameterArray,
                                       Span<double> Results)
    {
        double Parameter = U;
        //
        // the points  are assumed to be stored as follows in the Values array :
        //
        // [0]            [Dimension -1]             first  point   coefficients
        //
        // [Dimension]    [Dimension + Dimension -1] last point   coefficients
        //
        //
        // the derivatives  are assumed to be stored as follows in
        // the Derivatives array :
        //
        // [0]            [Dimension -1]             first  point   coefficients
        //
        // [Dimension]    [Dimension + Dimension -1] last point   coefficients
        //
        //  The ParameterArray stores the parameter value assign to each point in
        //  order described above, that is
        //  [0]   is assign to first  point
        //  [1]   is assign to last   point
        //
        int ii, jj, kk, pp, Index, Index1, Degree, ReturnCode;
        int local_request = DerivativeOrder;

        ReturnCode = 0;
        Degree = 3;
        Span<double> ParametersArray = stackalloc double[4];
        double difference;
        double inverse;
        ReadOnlySpan<double> FirstLast = ParameterArray;
        ReadOnlySpan<double> PointsArray = ValueArray;
        ReadOnlySpan<double> DerivativesArray = DerivativeArray;
        Span<double> ResultArray = Results;

        if (local_request >= Degree)
        {
            local_request = Degree;
        }
        int aSize = (Degree + 1) * Dimension;
        Span<double> divided_differences_array = aSize <= 128 ? stackalloc double[aSize] : new double[aSize];

        for (ii = 0, jj = 0; ii < 2; ii++, jj += 2)
        {
            ParametersArray[jj] = ParametersArray[jj + 1] = FirstLast[ii];
        }
        //
        //  Build the divided differences array
        //
        //
        // initialise it at the stage 2 of the building algorithm
        // for divided differences
        //
        inverse = FirstLast[1] - FirstLast[0];
        inverse = 1.0e0 / inverse;

        for (ii = 0, jj = Dimension, kk = 2 * Dimension, pp = 3 * Dimension; ii < Dimension;
             ii++, jj++, kk++, pp++)
        {
            divided_differences_array[ii] = PointsArray[ii];
            divided_differences_array[kk] = inverse * (PointsArray[jj] - PointsArray[ii]);
            divided_differences_array[jj] = DerivativesArray[ii];
            divided_differences_array[pp] = DerivativesArray[jj];
        }

        for (ii = 1; ii <= Degree; ii++)
        {

            for (jj = Degree; jj >= ii + 1; jj--)
            {
                Index = jj * Dimension;
                Index1 = Index - Dimension;

                for (kk = 0; kk < Dimension; kk++)
                {
                    divided_differences_array[Index + kk] -= divided_differences_array[Index1 + kk];
                }

                for (kk = 0; kk < Dimension; kk++)
                {
                    divided_differences_array[Index + kk] *= inverse;
                }
            }
        }
        //
        //
        // Evaluate the divided difference array polynomial which expresses as
        //
        //  P(t) = [t1] P + (t - t1) [t1,t2] P + (t - t1)(t - t2)[t1,t2,t3] P + ...
        //         + (t - t1)(t - t2)(t - t3)...(t - td) [t1,t2,...,td+1] P
        //
        // The ith slot in the divided_differences_array is [t1,t2,...,ti+1]
        //
        //
        Index = Degree * Dimension;

        for (kk = 0; kk < Dimension; kk++)
        {
            ResultArray[kk] = divided_differences_array[Index + kk];
        }

        for (ii = Dimension; ii < (local_request + 1) * Dimension; ii++)
        {
            ResultArray[ii] = 0.0e0;
        }

        for (ii = Degree; ii >= 1; ii--)
        {
            difference = Parameter - ParametersArray[ii - 1];

            for (jj = local_request; jj > 0; jj--)
            {
                Index = jj * Dimension;
                Index1 = Index - Dimension;

                for (kk = 0; kk < Dimension; kk++)
                {
                    ResultArray[Index + kk] *= difference;
                    ResultArray[Index + kk] += ResultArray[Index1 + kk] * (double)jj;
                }
            }
            Index = (ii - 1) * Dimension;

            for (kk = 0; kk < Dimension; kk++)
            {
                ResultArray[kk] *= difference;
                ResultArray[kk] += divided_differences_array[Index + kk];
            }
        }
        //  FINISH :
        return (ReturnCode);
    }

    /// <summary>
    /// Computes the coefficients of the Hermite polynomials on [FirstParameter, LastParameter] with constraints of
    /// orders <paramref name="FirstOrder"/> and <paramref name="LastOrder"/> at the ends, one polynomial per row of
    /// <paramref name="MatrixCoefs"/> (rows and columns from 1). Returns false on invalid input or a singular system.
    /// </summary>
    public static bool HermiteCoefficients(double FirstParameter, double LastParameter, int FirstOrder, int LastOrder,
                                           math_Matrix MatrixCoefs)
    {
        int NbCoeff = FirstOrder + LastOrder + 2;
        Span<int> Ordre = stackalloc int[2];
        int ii, jj, pp, cote, iof = 0;
        double Prod, TBorne = FirstParameter;
        math_Vector Coeff = new(1, NbCoeff), B = new(1, NbCoeff, 0.0);
        math_Matrix MAT = new(1, NbCoeff, 1, NbCoeff, 0.0);

        // Test de validites

        if ((FirstOrder < 0) || (LastOrder < 0))
        {
            return false;
        }
        double D1 = Math.Abs(FirstParameter), D2 = Math.Abs(LastParameter);
        if (D1 > 100 || D2 > 100)
        {
            return false;
        }
        D2 += D1;
        if (D2 < 0.01)
        {
            return false;
        }
        if (Math.Abs(LastParameter - FirstParameter) / D2 < 0.01)
        {
            return false;
        }

        // Calcul de la matrice a inverser (MAT)

        Ordre[0] = FirstOrder + 1;
        Ordre[1] = LastOrder + 1;

        for (cote = 0; cote <= 1; cote++)
        {
            Coeff.Init(1);

            for (pp = 1; pp <= Ordre[cote]; pp++)
            {
                ii = pp + iof;
                Prod = 1;

                for (jj = pp; jj <= NbCoeff; jj++)
                {
                    //        tout se passe dans les 3 lignes suivantes
                    MAT[ii, jj] = Coeff[jj] * Prod;
                    Coeff[jj] *= jj - pp;
                    Prod *= TBorne;
                }
            }
            TBorne = LastParameter;
            iof = Ordre[0];
        }

        // resolution du systemes
        math_Gauss ResolCoeff = new(MAT, 1.0e-10);
        if (!ResolCoeff.IsDone())
        {
            return false;
        }

        for (ii = 1; ii <= NbCoeff; ii++)
        {
            B[ii] = 1;
            ResolCoeff.Solve(B, Coeff);
            MatrixCoefs.SetRow(ii, Coeff);
            B[ii] = 0;
        }
        return true;
    }

    /// <summary>
    /// Converts the coefficients of a polynomial curve (and of its weights, or <see cref="NoWeights"/>) in the
    /// canonical base into poles (and weights) in the Bernstein base.
    /// </summary>
    public static void CoefficientsPoles(NCollection_Array1<gp_Pnt> Coefs, NCollection_Array1<double>? WCoefs,
                                         NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double>? WPoles)
    {
        NCollection_Array1<double> tempC = new(1, 3 * Coefs.Length());
        PLib.SetPoles(Coefs, tempC);
        NCollection_Array1<double> tempP = new(1, 3 * Poles.Length());
        PLib.SetPoles(Coefs, tempP); // OCCT fills tempP from Coefs, not Poles; only its size comes from Poles
        PLib.CoefficientsPoles(3, tempC, WCoefs, tempP, WPoles);
        PLib.GetPoles(tempP, Poles);
    }

    /// <inheritdoc cref="CoefficientsPoles(NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/>
    public static void CoefficientsPoles(NCollection_Array1<gp_Pnt2d> Coefs, NCollection_Array1<double>? WCoefs,
                                         NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double>? WPoles)
    {
        NCollection_Array1<double> tempC = new(1, 2 * Coefs.Length());
        PLib.SetPoles(Coefs, tempC);
        NCollection_Array1<double> tempP = new(1, 2 * Poles.Length());
        PLib.SetPoles(Coefs, tempP); // OCCT fills tempP from Coefs, not Poles; only its size comes from Poles
        PLib.CoefficientsPoles(2, tempC, WCoefs, tempP, WPoles);
        PLib.GetPoles(tempP, Poles);
    }

    /// <inheritdoc cref="CoefficientsPoles(NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/>
    public static void CoefficientsPoles(NCollection_Array1<double> Coefs, NCollection_Array1<double>? WCoefs,
                                         NCollection_Array1<double> Poles, NCollection_Array1<double>? WPoles)
    {
        PLib.CoefficientsPoles(1, Coefs, WCoefs, Poles, WPoles);
    }

    /// <summary>
    /// Converts the coefficients of a polynomial curve of dimension <paramref name="dim"/> (flat array) into poles
    /// in the Bernstein base.
    /// </summary>
    public static void CoefficientsPoles(int dim, NCollection_Array1<double> Coefs, NCollection_Array1<double>? WCoefs,
                                         NCollection_Array1<double> Poles, NCollection_Array1<double>? WPoles)
    {
        NCollection_Array1<double>? Weights = WPoles;
        bool rat = WCoefs != null;
        int loc = Coefs.Lower();
        int lop = Poles.Lower();
        int lowc = 0;
        int lowp = 0;
        int upc = Coefs.Upper();
        int upp = Poles.Upper();
        int upwc = 0;
        int upwp = 0;
        int reflen = Coefs.Length() / dim;
        int i, j, k;
        // Les Extremites.
        if (rat)
        {
            lowc = WCoefs!.Lower();
            lowp = Weights!.Lower();
            upwc = WCoefs.Upper();
            upwp = Weights.Upper();
        }

        for (i = 0; i < dim; i++)
        {
            Poles[lop + i] = Coefs[loc + i];
            Poles[upp - i] = Coefs[upc - i];
        }
        if (rat)
        {
            Weights![lowp] = WCoefs![lowc];
            Weights[upwp] = WCoefs[upwc];
        }

        double Cnp;
        for (i = 2; i < reflen; i++)
        {
            Cnp = PLib.Bin(reflen - 1, i - 1);
            if (rat)
            {
                Weights![lowp + i - 1] = WCoefs![lowc + i - 1] / Cnp;
            }

            for (j = 0; j < dim; j++)
            {
                Poles[lop + dim * (i - 1) + j] = Coefs[loc + dim * (i - 1) + j] / Cnp;
            }
        }

        for (i = 1; i <= reflen - 1; i++)
        {

            for (j = reflen - 1; j >= i; j--)
            {
                if (rat)
                {
                    Weights![lowp + j] += Weights[lowp + j - 1];
                }

                for (k = 0; k < dim; k++)
                {
                    Poles[lop + dim * j + k] += Poles[lop + dim * (j - 1) + k];
                }
            }
        }
        if (rat)
        {

            for (i = 1; i <= reflen; i++)
            {

                for (j = 0; j < dim; j++)
                {
                    Poles[lop + dim * (i - 1) + j] /= Weights![lowp + i - 1];
                }
            }
        }
    }

    /// <summary>
    /// Reparametrises the polynomial curve (power-basis coefficients <paramref name="Coeffs"/>, weights or
    /// <see cref="NoWeights"/>) so that [U1, U2] becomes [0, 1]: the change of variable u = U1 + v * (U2 - U1).
    /// </summary>
    public static void Trimming(double U1, double U2, NCollection_Array1<gp_Pnt> Coeffs, NCollection_Array1<double>? WCoeffs)
    {
        NCollection_Array1<double> temp = new(1, 3 * Coeffs.Length());
        PLib.SetPoles(Coeffs, temp);
        PLib.Trimming(U1, U2, 3, temp, WCoeffs);
        PLib.GetPoles(temp, Coeffs);
    }

    /// <inheritdoc cref="Trimming(double, double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/>
    public static void Trimming(double U1, double U2, NCollection_Array1<gp_Pnt2d> Coeffs, NCollection_Array1<double>? WCoeffs)
    {
        NCollection_Array1<double> temp = new(1, 2 * Coeffs.Length());
        PLib.SetPoles(Coeffs, temp);
        PLib.Trimming(U1, U2, 2, temp, WCoeffs);
        PLib.GetPoles(temp, Coeffs);
    }

    /// <inheritdoc cref="Trimming(double, double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/>
    public static void Trimming(double U1, double U2, NCollection_Array1<double> Coeffs, NCollection_Array1<double>? WCoeffs)
    {
        PLib.Trimming(U1, U2, 1, Coeffs, WCoeffs);
    }

    /// <summary>
    /// Reparametrises the polynomial curve of dimension <paramref name="dim"/> (flat array) so that [U1, U2]
    /// becomes [0, 1]. Throws <see cref="Standard_Failure"/> if the weights don't match the coefficients.
    /// </summary>
    public static void Trimming(double U1, double U2, int dim, NCollection_Array1<double> Coeffs, NCollection_Array1<double>? WCoeffs)
    {
        NCollection_Array1<double> Coefs = Coeffs;
        NCollection_Array1<double>? WCoefs = WCoeffs;

        // principe :
        // on fait le changement de variable v = (u-U1) / (U2-U1)
        // on exprime u = f(v) que l'on remplace dans l'expression polynomiale
        // decomposee sous la forme du schema iteratif de horner.

        double lsp = U2 - U1;
        int indc, indw = 0;
        int upc = Coefs.Upper() - dim + 1, upw = 0;
        int len = Coefs.Length() / dim;
        bool rat = WCoefs != null;

        if (rat)
        {
            if (len != WCoefs!.Length())
            {
                throw new Standard_Failure("PLib::Trimming : nbcoefs/dim != nbweights !!!");
            }
            upw = WCoefs.Upper();
        }
        len--;

        for (int i = 1; i <= len; i++)
        {
            int j;
            indc = upc - dim * (i - 1);
            if (rat)
            {
                indw = upw - i + 1;
            }
            // calcul du coefficient de degre le plus faible a l'iteration i

            for (j = 0; j < dim; j++)
            {
                Coefs[indc - dim + j] += U1 * Coefs[indc + j];
            }
            if (rat)
            {
                WCoefs![indw - 1] += U1 * WCoefs[indw];
            }

            // calcul des coefficients intermediaires :

            while (indc < upc)
            {
                indc += dim;

                for (int k = 0; k < dim; k++)
                {
                    Coefs[indc - dim + k] = U1 * Coefs[indc + k] + lsp * Coefs[indc - dim + k];
                }
                if (rat)
                {
                    indw++;
                    WCoefs![indw - 1] = U1 * WCoefs[indw] + lsp * WCoefs[indw - 1];
                }
            }

            // calcul du coefficient de degre le plus eleve :

            for (j = 0; j < dim; j++)
            {
                Coefs[upc + j] *= lsp;
            }
            if (rat)
            {
                WCoefs![upw] *= lsp;
            }
        }
    }

    /// <summary>
    /// Converts the coefficients of a polynomial surface (and of its weights, or <see cref="NoWeights2"/>) in the
    /// canonical base into poles (and weights) in the Bernstein base.
    /// </summary>
    // Modified: 21/10/1996 by PMN :  PolesCoefficient (PRO5852).
    // on ne bidouille plus les u et v c'est a l'appelant de savoir ce qu'il
    // fait avec BuildCache ou plus simplement d'utiliser PolesCoefficients
    public static void CoefficientsPoles(NCollection_Array2<gp_Pnt> Coefs, NCollection_Array2<double>? WCoefs,
                                         NCollection_Array2<gp_Pnt> Poles, NCollection_Array2<double>? WPoles)
    {
        NCollection_Array2<double>? Weights = WPoles;
        bool rat = (WCoefs != null);
        int LowerRow = Poles.LowerRow();
        int UpperRow = Poles.UpperRow();
        int LowerCol = Poles.LowerCol();
        int UpperCol = Poles.UpperCol();
        int ColLength = Poles.ColLength();
        int RowLength = Poles.RowLength();

        // Bidouille pour retablir u et v pour les coefs calcules
        // par buildcache
        //  bool inv = false; //ColLength != Coefs.ColLength();

        int Row, Col;
        double W, Cnp;

        int I1, I2;
        int NPoleu, NPolev;
        gp_XYZ Temp = new();

        for (NPoleu = LowerRow; NPoleu <= UpperRow; NPoleu++)
        {
            Poles[NPoleu, LowerCol] = Coefs[NPoleu, LowerCol];
            if (rat)
            {
                Weights![NPoleu, LowerCol] = WCoefs![NPoleu, LowerCol];
            }

            for (Col = LowerCol + 1; Col <= UpperCol - 1; Col++)
            {
                Cnp = PLib.Bin(RowLength - 1, Col - LowerCol);
                Temp = Coefs[NPoleu, Col].XYZ();
                Temp.Divide(Cnp);
                Poles[NPoleu, Col].SetXYZ(Temp);
                if (rat)
                {
                    Weights![NPoleu, Col] = WCoefs![NPoleu, Col] / Cnp;
                }
            }
            Poles[NPoleu, UpperCol] = Coefs[NPoleu, UpperCol];
            if (rat)
            {
                Weights![NPoleu, UpperCol] = WCoefs![NPoleu, UpperCol];
            }

            for (I1 = 1; I1 <= RowLength - 1; I1++)
            {

                for (I2 = UpperCol; I2 >= LowerCol + I1; I2--)
                {
                    Temp.SetLinearForm(Poles[NPoleu, I2].XYZ(), Poles[NPoleu, I2 - 1].XYZ());
                    Poles[NPoleu, I2].SetXYZ(Temp);
                    if (rat)
                    {
                        Weights![NPoleu, I2] += Weights[NPoleu, I2 - 1];
                    }
                }
            }
        }

        for (NPolev = LowerCol; NPolev <= UpperCol; NPolev++)
        {

            for (Row = LowerRow + 1; Row <= UpperRow - 1; Row++)
            {
                Cnp = PLib.Bin(ColLength - 1, Row - LowerRow);
                Temp = Poles[Row, NPolev].XYZ();
                Temp.Divide(Cnp);
                Poles[Row, NPolev].SetXYZ(Temp);
                if (rat)
                {
                    Weights![Row, NPolev] /= Cnp;
                }
            }

            for (I1 = 1; I1 <= ColLength - 1; I1++)
            {

                for (I2 = UpperRow; I2 >= LowerRow + I1; I2--)
                {
                    Temp.SetLinearForm(Poles[I2, NPolev].XYZ(), Poles[I2 - 1, NPolev].XYZ());
                    Poles[I2, NPolev].SetXYZ(Temp);
                    if (rat)
                    {
                        Weights![I2, NPolev] += Weights[I2 - 1, NPolev];
                    }
                }
            }
        }
        if (rat)
        {

            for (Row = LowerRow; Row <= UpperRow; Row++)
            {

                for (Col = LowerCol; Col <= UpperCol; Col++)
                {
                    W = Weights![Row, Col];
                    Temp = Poles[Row, Col].XYZ();
                    Temp.Divide(W);
                    Poles[Row, Col].SetXYZ(Temp);
                }
            }
        }
    }

    /// <summary>Applies <see cref="Trimming(double, double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/> to every column (the U direction) of a polynomial surface.</summary>
    public static void UTrimming(double U1, double U2, NCollection_Array2<gp_Pnt> Coeffs, NCollection_Array2<double>? WCoeffs)
    {
        bool rat = WCoeffs != null;
        int lr = Coeffs.LowerRow();
        int ur = Coeffs.UpperRow();
        int lc = Coeffs.LowerCol();
        int uc = Coeffs.UpperCol();
        NCollection_Array1<gp_Pnt> Temp = new(lr, ur);
        NCollection_Array1<double> Temw = new(lr, ur);

        for (int icol = lc; icol <= uc; icol++)
        {
            int irow;
            for (irow = lr; irow <= ur; irow++)
            {
                Temp[irow] = Coeffs[irow, icol];
                if (rat)
                {
                    Temw[irow] = WCoeffs![irow, icol];
                }
            }
            if (rat)
            {
                PLib.Trimming(U1, U2, Temp, Temw);
            }
            else
            {
                PLib.Trimming(U1, U2, Temp, PLib.NoWeights());
            }

            for (irow = lr; irow <= ur; irow++)
            {
                Coeffs[irow, icol] = Temp[irow];
                if (rat)
                {
                    WCoeffs![irow, icol] = Temw[irow];
                }
            }
        }
    }

    /// <summary>Applies <see cref="Trimming(double, double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/> to every row (the V direction) of a polynomial surface.</summary>
    public static void VTrimming(double V1, double V2, NCollection_Array2<gp_Pnt> Coeffs, NCollection_Array2<double>? WCoeffs)
    {
        bool rat = WCoeffs != null;
        int lr = Coeffs.LowerRow();
        int ur = Coeffs.UpperRow();
        int lc = Coeffs.LowerCol();
        int uc = Coeffs.UpperCol();
        NCollection_Array1<gp_Pnt> Temp = new(lc, uc);
        NCollection_Array1<double> Temw = new(lc, uc);

        for (int irow = lr; irow <= ur; irow++)
        {
            int icol;
            for (icol = lc; icol <= uc; icol++)
            {
                Temp[icol] = Coeffs[irow, icol];
                if (rat)
                {
                    Temw[icol] = WCoeffs![irow, icol];
                }
            }
            if (rat)
            {
                PLib.Trimming(V1, V2, Temp, Temw);
            }
            else
            {
                PLib.Trimming(V1, V2, Temp, PLib.NoWeights());
            }

            for (icol = lc; icol <= uc; icol++)
            {
                Coeffs[irow, icol] = Temp[icol];
                if (rat)
                {
                    WCoeffs![irow, icol] = Temw[icol];
                }
            }
        }
    }

    /// <summary>
    /// Computes the coefficients (canonical base, in <paramref name="Coefficients"/> from index 0) of the polynomial
    /// of dimension <paramref name="Dimension"/> that meets the constraints <paramref name="FirstConstr"/> (orders 0
    /// to FirstOrder, at FirstParameter) and <paramref name="LastConstr"/> (orders 0 to LastOrder, at LastParameter).
    /// The constraint arrays are indexed (1..Dimension, 0..order). Orders are at most 2.
    /// </summary>
    public static bool HermiteInterpolate(int Dimension, double FirstParameter, double LastParameter, int FirstOrder,
                                          int LastOrder, NCollection_Array2<double> FirstConstr,
                                          NCollection_Array2<double> LastConstr, NCollection_Array1<double> Coefficients)
    {
        // double Pattern[3][6]
        Span<double> Pattern = stackalloc double[3 * 6];

        // portage HP : il faut les initialiser 1 par 1

        Pattern[0 * 6 + 0] = 1;
        Pattern[0 * 6 + 1] = 1;
        Pattern[0 * 6 + 2] = 1;
        Pattern[0 * 6 + 3] = 1;
        Pattern[0 * 6 + 4] = 1;
        Pattern[0 * 6 + 5] = 1;
        Pattern[1 * 6 + 0] = 0;
        Pattern[1 * 6 + 1] = 1;
        Pattern[1 * 6 + 2] = 2;
        Pattern[1 * 6 + 3] = 3;
        Pattern[1 * 6 + 4] = 4;
        Pattern[1 * 6 + 5] = 5;
        Pattern[2 * 6 + 0] = 0;
        Pattern[2 * 6 + 1] = 0;
        Pattern[2 * 6 + 2] = 2;
        Pattern[2 * 6 + 3] = 6;
        Pattern[2 * 6 + 4] = 12;
        Pattern[2 * 6 + 5] = 20;

        math_Matrix A = new(0, FirstOrder + LastOrder + 1, 0, FirstOrder + LastOrder + 1);
        //  The initialisation of the matrix A
        int irow;
        for (irow = 0; irow <= FirstOrder; irow++)
        {
            double FirstVal = 1.0;

            for (int icol = 0; icol <= FirstOrder + LastOrder + 1; icol++)
            {
                A[irow, icol] = Pattern[irow * 6 + icol] * FirstVal;
                if (irow <= icol)
                {
                    FirstVal *= FirstParameter;
                }
            }
        }

        for (irow = 0; irow <= LastOrder; irow++)
        {
            double LastVal = 1.0;

            for (int icol = 0; icol <= FirstOrder + LastOrder + 1; icol++)
            {
                A[irow + FirstOrder + 1, icol] = Pattern[irow * 6 + icol] * LastVal;
                if (irow <= icol)
                {
                    LastVal *= LastParameter;
                }
            }
        }
        //
        //  The filled matrix A for FirstOrder=LastOrder=2 is:
        //
        //      1   FP  FP**2   FP**3    FP**4     FP**5
        //      0   1   2*FP    3*FP**2  4*FP**3   5*FP**4        FP - FirstParameter
        //      0   0   2       6*FP     12*FP**2  20*FP**3
        //      1   LP  LP**2   LP**3    LP**4     LP**5
        //      0   1   2*LP    3*LP**2  4*LP**3   5*LP**4        LP - LastParameter
        //      0   0   2       6*LP     12*LP**2  20*LP**3
        //
        //  If FirstOrder or LastOrder <=2 then some rows and columns are missing.
        //  For example:
        //  If FirstOrder=1 then 3th row and 6th column are missing
        //  If FirstOrder=LastOrder=0 then 2,3,5,6th rows and 3,4,5,6th columns are missing

        math_Gauss Equations = new(A);

        for (int idim = 1; idim <= Dimension; idim++)
        {
            math_Vector B = new(0, FirstOrder + LastOrder + 1);
            int icol;
            for (icol = 0; icol <= FirstOrder; icol++)
            {
                B[icol] = FirstConstr[idim, icol];
            }

            for (icol = 0; icol <= LastOrder; icol++)
            {
                B[FirstOrder + 1 + icol] = LastConstr[idim, icol];
            }

            //  The solving of equations system A * X = B. Then B = X
            // OCCT checks IsDone only after Solve, which (with its checks on, as in the port) throws
            // StdFail_NotDone first when A is singular
            Equations.Solve(B);

            if (!Equations.IsDone())
            {
                return false;
            }

            //  the filling of the Coefficients

            for (icol = 0; icol <= FirstOrder + LastOrder + 1; icol++)
            {
                Coefficients[Dimension * icol + idim - 1] = B[icol];
            }
        }
        return true;
    }

    /// <summary>
    /// Computes the number of Gauss points and the working degree for a Jacobi approximation with the given
    /// constraint order (C0, C1 or C2), maximum degree and precision code (-5 to -1, 1 to 4).
    /// Throws <see cref="Standard_ConstructionError"/> on invalid arguments.
    /// </summary>
    public static void JacobiParameters(GeomAbs_Shape ConstraintOrder, int MaxDegree, int Code, out int NbGaussPoints,
                                        out int WorkDegree)
    {
        // ConstraintOrder: Ordre de contrainte aux extremites :
        //            C0 = contraintes de passage aux bornes;
        //            C1 = C0 + contraintes de derivees 1eres;
        //            C2 = C1 + contraintes de derivees 2ndes.
        // MaxDegree: Nombre maxi de coeff de la "courbe" polynomiale
        //            d' approximation (doit etre superieur ou egal a
        //            2*NivConstr+2 et inferieur ou egal a 50).
        // Code:      Code d' init. des parametres de discretisation.
        //            (choix de NBPNTS et de NDGJAC de MAPF1C,MAPFXC).
        //            = -5 Calcul tres rapide mais peu precis (8pts)
        //            = -4    '     '    '      '   '    '    (10pts)
        //            = -3    '     '    '      '   '    '    (15pts)
        //            = -2    '     '    '      '   '    '    (20pts)
        //            = -1    '     '    '      '   '    '    (25pts)
        //            = 1 calcul rapide avec precision moyenne (30pts).
        //            = 2 calcul rapide avec meilleure precision (40pts).
        //            = 3 calcul un peu plus lent avec bonne precision (50 pts).
        //            = 4 calcul lent avec la meilleure precision possible
        //             (61pts).

        // The possible values of NbGaussPoints

        const int NDEG8 = 8, NDEG10 = 10, NDEG15 = 15, NDEG20 = 20, NDEG25 = 25, NDEG30 = 30, NDEG40 = 40,
                  NDEG50 = 50, NDEG61 = 61;

        int NivConstr = 0;
        switch (ConstraintOrder)
        {
            case GeomAbs_C0:
                NivConstr = 0;
                break;
            case GeomAbs_C1:
                NivConstr = 1;
                break;
            case GeomAbs_C2:
                NivConstr = 2;
                break;
            default:
                throw new Standard_ConstructionError("Invalid ConstraintOrder");
        }
        if (MaxDegree < 2 * NivConstr + 1)
        {
            throw new Standard_ConstructionError("Invalid MaxDegree");
        }

        if (Code >= 1)
        {
            WorkDegree = MaxDegree + 9;
        }
        else
        {
            WorkDegree = MaxDegree + 6;
        }

        //---> Nbre mini de points necessaires.
        int IPMIN = 0;
        if (WorkDegree < NDEG8)
        {
            IPMIN = NDEG8;
        }
        else if (WorkDegree < NDEG10)
        {
            IPMIN = NDEG10;
        }
        else if (WorkDegree < NDEG15)
        {
            IPMIN = NDEG15;
        }
        else if (WorkDegree < NDEG20)
        {
            IPMIN = NDEG20;
        }
        else if (WorkDegree < NDEG25)
        {
            IPMIN = NDEG25;
        }
        else if (WorkDegree < NDEG30)
        {
            IPMIN = NDEG30;
        }
        else if (WorkDegree < NDEG40)
        {
            IPMIN = NDEG40;
        }
        else if (WorkDegree < NDEG50)
        {
            IPMIN = NDEG50;
        }
        else if (WorkDegree < NDEG61)
        {
            IPMIN = NDEG61;
        }
        else
        {
            throw new Standard_ConstructionError("Invalid MaxDegree");
        }
        // ---> Nbre de points voulus.
        int IWANT = 0;
        switch (Code)
        {
            case -5:
                IWANT = NDEG8;
                break;
            case -4:
                IWANT = NDEG10;
                break;
            case -3:
                IWANT = NDEG15;
                break;
            case -2:
                IWANT = NDEG20;
                break;
            case -1:
                IWANT = NDEG25;
                break;
            case 1:
                IWANT = NDEG30;
                break;
            case 2:
                IWANT = NDEG40;
                break;
            case 3:
                IWANT = NDEG50;
                break;
            case 4:
                IWANT = NDEG61;
                break;
            default:
                throw new Standard_ConstructionError("Invalid Code");
        }
        //-->  NbGaussPoints est le nombre de points de discretisation de la fonction,
        //     il ne peut prendre que les valeurs 8,10,15,20,25,30,40,50 ou 61.
        //     NbGaussPoints doit etre superieur strictement a WorkDegree.
        NbGaussPoints = Max(IPMIN, IWANT);
        //  NbGaussPoints +=2;
    }

    /// <summary>Translates a constraint order (C0, C1, C2) into 0, 1, 2.</summary>
    public static int NivConstr(GeomAbs_Shape ConstraintOrder)
    {
        int NivConstr = 0;
        switch (ConstraintOrder)
        {
            case GeomAbs_C0:
                NivConstr = 0;
                break;
            case GeomAbs_C1:
                NivConstr = 1;
                break;
            case GeomAbs_C2:
                NivConstr = 2;
                break;
            default:
                throw new Standard_ConstructionError("Invalid ConstraintOrder");
        }
        return NivConstr;
    }

    /// <summary>Translates 0, 1, 2 into a constraint order (C0, C1, C2).</summary>
    public static GeomAbs_Shape ConstraintOrder(int NivConstr)
    {
        GeomAbs_Shape ConstraintOrder = GeomAbs_C0;
        switch (NivConstr)
        {
            case 0:
                ConstraintOrder = GeomAbs_C0;
                break;
            case 1:
                ConstraintOrder = GeomAbs_C1;
                break;
            case 2:
                ConstraintOrder = GeomAbs_C2;
                break;
            default:
                throw new Standard_ConstructionError("Invalid NivConstr");
        }
        return ConstraintOrder;
    }

    /// <summary>
    /// Computes the length of the polynomial curve (coefficients as for <see cref="EvalPolynomial"/>) between
    /// <paramref name="U1"/> and <paramref name="U2"/> by Gauss integration.
    /// </summary>
    public static void EvalLength(int Degree, int Dimension, ReadOnlySpan<double> PolynomialCoeff, double U1, double U2,
                                  out double Length)
    {
        int i, j, idim, degdim;
        double C1, C2, Sum, Tran, X1, X2, Der1, Der2, D1, D2, DD;

        ReadOnlySpan<double> PolynomialArray = PolynomialCoeff;

        int NbGaussPoints = 4 * Min((Degree / 4) + 1, 10);

        math_Vector GaussPoints = new(1, NbGaussPoints);
        math.GaussPoints(NbGaussPoints, GaussPoints);

        math_Vector GaussWeights = new(1, NbGaussPoints);
        math.GaussWeights(NbGaussPoints, GaussWeights);

        C1 = (U2 + U1) / 2.0;
        C2 = (U2 - U1) / 2.0;

        //-----------------------------------------------------------
        //****** Integration - Boucle sur les intervalles de GAUSS **
        //-----------------------------------------------------------

        Sum = 0;

        for (j = 1; j <= NbGaussPoints / 2; j++)
        {
            // Integration en tenant compte de la symetrie
            Tran = C2 * GaussPoints[j];
            X1 = C1 + Tran;
            X2 = C1 - Tran;

            //****** Derivation sur la dimension de l'espace **

            degdim = Degree * Dimension;
            Der1 = Der2 = 0.0;
            for (idim = 0; idim < Dimension; idim++)
            {
                D1 = D2 = Degree * PolynomialArray[idim + degdim];
                for (i = Degree - 1; i >= 1; i--)
                {
                    DD = i * PolynomialArray[idim + i * Dimension];
                    D1 = D1 * X1 + DD;
                    D2 = D2 * X2 + DD;
                }
                Der1 += D1 * D1;
                Der2 += D2 * D2;
            }

            //****** Integration **

            Sum += GaussWeights[j] * C2 * (Math.Sqrt(Der1) + Math.Sqrt(Der2));

            //****** Fin de boucle dur les intervalles de GAUSS **
        }
        Length = Sum;
    }

    /// <summary>
    /// Computes the length of the polynomial curve between <paramref name="U1"/> and <paramref name="U2"/>,
    /// subdividing until two successive estimates differ by at most <paramref name="Tol"/> (or 13 iterations);
    /// <paramref name="Error"/> is that last difference.
    /// </summary>
    public static void EvalLength(int Degree, int Dimension, ReadOnlySpan<double> PolynomialCoeff, double U1, double U2,
                                  double Tol, out double Length, out double Error)
    {
        int i;
        int NbSubInt = 1,  // Current number of subintervals
          MaxNbIter = 13, // Max number of iterations
          NbIter = 1;  // Current number of iterations
        double dU, OldLen, LenI;

        PLib.EvalLength(Degree, Dimension, PolynomialCoeff, U1, U2, out Length);

        do
        {
            OldLen = Length;
            Length = 0.0;
            NbSubInt *= 2;
            dU = (U2 - U1) / NbSubInt;
            for (i = 1; i <= NbSubInt; i++)
            {
                PLib.EvalLength(Degree, Dimension, PolynomialCoeff, U1 + (i - 1) * dU, U1 + i * dU, out LenI);
                Length += LenI;
            }
            NbIter++;
            Error = Math.Abs(OldLen - Length);
        } while (Error > Tol && NbIter <= MaxNbIter);
    }
}
