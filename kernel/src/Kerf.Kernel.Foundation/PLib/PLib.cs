// Port of: src/FoundationClasses/TKMath/PLib/PLib.hxx
// Port of: src/FoundationClasses/TKMath/PLib/PLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Numerics.Polynomials;

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
internal static class PLib
{
    /// <summary>No weights: the argument for non-rational functions (null).</summary>
    public static NCollection_Array1<double>? NoWeights() => null;

    /// <summary>No weights: the argument for non-rational functions (null).</summary>
    public static NCollection_Array2<double>? NoWeights2() => null;

    // To convert points array into Real ..
    // *********************************

    /// <summary>Copies the coordinates of the poles into FP.</summary>
    public static void SetPoles(NCollection_Array1<Point2D> poles, NCollection_Array1<double> fp)
    {
        int j = fp.Lower();
        int pLower = poles.Lower();
        int pUpper = poles.Upper();

        for (int i = pLower; i <= pUpper; i++)
        {
            ref readonly Point2D p = ref poles[i];
            fp[j] = p.Coord(1);
            j++;
            fp[j] = p.Coord(2);
            j++;
        }
    }

    /// <summary>Copies the coordinates of the poles, multiplied by their weights, and the weights into FP.</summary>
    public static void SetPoles(NCollection_Array1<Point2D> poles, NCollection_Array1<double> weights, NCollection_Array1<double> fp)
    {
        int j = fp.Lower();
        int pLower = poles.Lower();
        int pUpper = poles.Upper();

        for (int i = pLower; i <= pUpper; i++)
        {
            double w = weights[i];
            ref readonly Point2D p = ref poles[i];
            fp[j] = p.Coord(1) * w;
            j++;
            fp[j] = p.Coord(2) * w;
            j++;
            fp[j] = w;
            j++;
        }
    }

    /// <summary>Gets the coordinates of the poles from FP.</summary>
    public static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point2D> poles)
    {
        int j = fp.Lower();
        int pLower = poles.Lower();
        int pUpper = poles.Upper();

        for (int i = pLower; i <= pUpper; i++)
        {
            ref Point2D p = ref poles[i];
            p = p.WithCoord(1, fp[j]);
            j++;
            p = p.WithCoord(2, fp[j]);
            j++;
        }
    }

    /// <summary>Gets the coordinates of the poles and their weights from FP (homogeneous coordinates).</summary>
    public static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point2D> poles, NCollection_Array1<double> weights)
    {
        int j = fp.Lower();
        int pLower = poles.Lower();
        int pUpper = poles.Upper();

        for (int i = pLower; i <= pUpper; i++)
        {
            double w = fp[j + 2];
            weights[i] = w;
            ref Point2D p = ref poles[i];
            p = p.WithCoord(1, fp[j] / w);
            j++;
            p = p.WithCoord(2, fp[j] / w);
            j++;
            j++;
        }
    }

    /// <summary>Copies the coordinates of the poles into FP.</summary>
    public static void SetPoles(NCollection_Array1<Point3D> poles, NCollection_Array1<double> fp)
    {
        int j = fp.Lower();
        int pLower = poles.Lower();
        int pUpper = poles.Upper();

        for (int i = pLower; i <= pUpper; i++)
        {
            ref readonly Point3D p = ref poles[i];
            fp[j] = p.Coord(1);
            j++;
            fp[j] = p.Coord(2);
            j++;
            fp[j] = p.Coord(3);
            j++;
        }
    }

    /// <summary>Copies the coordinates of the poles, multiplied by their weights, and the weights into FP.</summary>
    public static void SetPoles(NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights, NCollection_Array1<double> fp)
    {
        int j = fp.Lower();
        int pLower = poles.Lower();
        int pUpper = poles.Upper();

        for (int i = pLower; i <= pUpper; i++)
        {
            double w = weights[i];
            ref readonly Point3D p = ref poles[i];
            fp[j] = p.Coord(1) * w;
            j++;
            fp[j] = p.Coord(2) * w;
            j++;
            fp[j] = p.Coord(3) * w;
            j++;
            fp[j] = w;
            j++;
        }
    }

    /// <summary>Gets the coordinates of the poles from FP.</summary>
    public static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point3D> poles)
    {
        int j = fp.Lower();
        int pLower = poles.Lower();
        int pUpper = poles.Upper();

        for (int i = pLower; i <= pUpper; i++)
        {
            ref Point3D p = ref poles[i];
            p = p.WithCoord(1, fp[j]);
            j++;
            p = p.WithCoord(2, fp[j]);
            j++;
            p = p.WithCoord(3, fp[j]);
            j++;
        }
    }

    /// <summary>Gets the coordinates of the poles and their weights from FP (homogeneous coordinates).</summary>
    public static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights)
    {
        int j = fp.Lower();
        int pLower = poles.Lower();
        int pUpper = poles.Upper();

        for (int i = pLower; i <= pUpper; i++)
        {
            double w = fp[j + 3];
            weights[i] = w;
            ref Point3D p = ref poles[i];
            p = p.WithCoord(1, fp[j] / w);
            j++;
            p = p.WithCoord(2, fp[j] / w);
            j++;
            p = p.WithCoord(3, fp[j] / w);
            j++;
            j++;
        }
    }

    // Pascal's triangle up to BSplCLib::MaxDegree(), computed once (a compile-time table in OCCT)
    private const int BinomMaxDegree = 25; // BSplCLib::MaxDegree()

    private static readonly int[] THE_BINOM = BuildBinom();

    private static int[] BuildBinom()
    {
        const int size = BinomMaxDegree + 1;
        int[] binom = new int[size * size];
        // Initialize first row: C(0,0) = 1
        binom[0] = 1;

        // Build Pascal's triangle row by row
        for (int i = 1; i <= BinomMaxDegree; ++i)
        {
            // First and last elements are always 1
            binom[i * size] = 1;
            binom[i * size + i] = 1;

            // Use recurrence relation for middle elements
            for (int j = 1; j < i; ++j)
            {
                binom[i * size + j] = binom[(i - 1) * size + j - 1] + binom[(i - 1) * size + j];
            }
        }
        return binom;
    }

    /// <summary>Returns the binomial coefficient C(N, P). N must be &lt;= <see cref="BSplCLib.MaxDegree"/>.</summary>
    public static double Bin(int n, int p)
    {
        Standard_OutOfRange_Raise_if(n < 0 || n > global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree(),
                                     "PLib::Bin: degree N is out of supported range [0, 25]");
        Standard_OutOfRange_Raise_if(p < 0 || p > n,
                                     "PLib::Bin: parameter P is out of valid range [0, N]");

        return (double)THE_BINOM[n * (BinomMaxDegree + 1) + p];
    }

    /// <summary>
    /// Computes the derivatives of a ratio at order <paramref name="n"/> in dimension <paramref name="dimension"/>.
    /// <paramref name="ders"/> holds the input derivatives from 0 to min(N, Degree), each as
    /// x(1), ..., x(Dimension), w; higher orders are taken as 0. If <paramref name="all"/> is false, only the
    /// derivative of order N is computed into <paramref name="rDers"/> (Dimension values); otherwise all orders
    /// 0 to N (Dimension * (N + 1) values).
    /// </summary>
    public static void RationalDerivative(int degree, int n, int dimension, ReadOnlySpan<double> ders, Span<double> rDers, bool all = true)
    {
        int derivativeRequest = n;
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
        double inverse;
        ReadOnlySpan<double> polesArray = ders;
        scoped Span<double> rationalArray = rDers;
        scoped Span<double> derivative_storage = default;
        double factor;
        int ii, index, otherIndex, index1, index2, jj;
        if (dimension == 3)
        {
            int deRequest1 = derivativeRequest + 1;
            int minDegRequ = derivativeRequest;
            if (minDegRequ > degree)
            {
                minDegRequ = degree;
            }
            Span<double> binomial_array = deRequest1 <= 128 ? stackalloc double[deRequest1] : new double[deRequest1];

            for (ii = 0; ii < deRequest1; ii++)
            {
                binomial_array[ii] = 1.0e0;
            }
            if (!all)
            {
                int dimDeRequ1 = (deRequest1 << 1) + deRequest1;
                derivative_storage = dimDeRequ1 <= 128 ? stackalloc double[dimDeRequ1] : new double[dimDeRequ1];
                rationalArray = derivative_storage;
            }

            inverse = 1.0e0 / polesArray[3];
            index = 0;
            index2 = -6;
            otherIndex = 0;

            for (ii = 0; ii <= minDegRequ; ii++)
            {
                index2 += 3;
                index1 = index2;
                rationalArray[index] = polesArray[otherIndex];
                index++;
                otherIndex++;
                rationalArray[index] = polesArray[otherIndex];
                index++;
                otherIndex++;
                rationalArray[index] = polesArray[otherIndex];
                index -= 2;
                otherIndex += 2;

                for (jj = ii - 1; jj >= 0; jj--)
                {
                    factor = binomial_array[jj] * polesArray[((ii - jj) << 2) + 3];
                    rationalArray[index] -= factor * rationalArray[index1];
                    index++;
                    index1++;
                    rationalArray[index] -= factor * rationalArray[index1];
                    index++;
                    index1++;
                    rationalArray[index] -= factor * rationalArray[index1];
                    index -= 2;
                    index1 -= 5;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }
                rationalArray[index] *= inverse;
                index++;
                rationalArray[index] *= inverse;
                index++;
                rationalArray[index] *= inverse;
                index++;
            }

            for (ii = minDegRequ + 1; ii <= derivativeRequest; ii++)
            {
                index2 += 3;
                index1 = index2;
                rationalArray[index] = 0.0e0;
                index++;
                rationalArray[index] = 0.0e0;
                index++;
                rationalArray[index] = 0.0e0;
                index -= 2;

                for (jj = ii - 1; jj >= ii - minDegRequ; jj--)
                {
                    factor = binomial_array[jj] * polesArray[((ii - jj) << 2) + 3];
                    rationalArray[index] -= factor * rationalArray[index1];
                    index++;
                    index1++;
                    rationalArray[index] -= factor * rationalArray[index1];
                    index++;
                    index1++;
                    rationalArray[index] -= factor * rationalArray[index1];
                    index -= 2;
                    index1 -= 5;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }
                rationalArray[index] *= inverse;
                index++;
                rationalArray[index] *= inverse;
                index++;
                rationalArray[index] *= inverse;
                index++;
            }

            if (!all)
            {
                rationalArray = rDers;
                int dimDeRequ = (derivativeRequest << 1) + derivativeRequest;
                rationalArray[0] = derivative_storage[dimDeRequ];
                dimDeRequ++;
                rationalArray[1] = derivative_storage[dimDeRequ];
                dimDeRequ++;
                rationalArray[2] = derivative_storage[dimDeRequ];
            }
        }
        else
        {
            int kk;
            int dimension1 = dimension + 1;
            int dimension2 = dimension << 1;
            int deRequest1 = derivativeRequest + 1;
            int minDegRequ = derivativeRequest;
            if (minDegRequ > degree)
            {
                minDegRequ = degree;
            }
            Span<double> binomial_array = deRequest1 <= 128 ? stackalloc double[deRequest1] : new double[deRequest1];

            for (ii = 0; ii < deRequest1; ii++)
            {
                binomial_array[ii] = 1.0e0;
            }
            if (!all)
            {
                int dimDeRequ1 = dimension * deRequest1;
                derivative_storage = dimDeRequ1 <= 128 ? stackalloc double[dimDeRequ1] : new double[dimDeRequ1];
                rationalArray = derivative_storage;
            }

            inverse = 1.0e0 / polesArray[dimension];
            index = 0;
            index2 = -dimension2;
            otherIndex = 0;

            for (ii = 0; ii <= minDegRequ; ii++)
            {
                index2 += dimension;
                index1 = index2;

                for (kk = 0; kk < dimension; kk++)
                {
                    rationalArray[index] = polesArray[otherIndex];
                    index++;
                    otherIndex++;
                }
                index -= dimension;
                ++otherIndex;

                for (jj = ii - 1; jj >= 0; jj--)
                {
                    factor = binomial_array[jj] * polesArray[(ii - jj) * dimension1 + dimension];

                    for (kk = 0; kk < dimension; kk++)
                    {
                        rationalArray[index] -= factor * rationalArray[index1];
                        index++;
                        index1++;
                    }
                    index -= dimension;
                    index1 -= dimension2;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }

                for (kk = 0; kk < dimension; kk++)
                {
                    rationalArray[index] *= inverse;
                    index++;
                }
            }

            for (ii = minDegRequ + 1; ii <= derivativeRequest; ii++)
            {
                index2 += dimension;
                index1 = index2;

                for (kk = 0; kk < dimension; kk++)
                {
                    rationalArray[index] = 0.0e0;
                    index++;
                }
                index -= dimension;

                for (jj = ii - 1; jj >= ii - minDegRequ; jj--)
                {
                    factor = binomial_array[jj] * polesArray[(ii - jj) * dimension1 + dimension];

                    for (kk = 0; kk < dimension; kk++)
                    {
                        rationalArray[index] -= factor * rationalArray[index1];
                        index++;
                        index1++;
                    }
                    index -= dimension;
                    index1 -= dimension2;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }

                for (kk = 0; kk < dimension; kk++)
                {
                    rationalArray[index] *= inverse;
                    index++;
                }
            }

            if (!all)
            {
                rationalArray = rDers;
                int dimDeRequ = dimension * derivativeRequest;

                for (kk = 0; kk < dimension; kk++)
                {
                    rationalArray[kk] = derivative_storage[dimDeRequ];
                    dimDeRequ++;
                }
            }
        }
    }

    /// <summary>
    /// Computes <paramref name="derivativesRequest"/> derivatives of a ratio from the derivatives of the
    /// homogeneous poles (<paramref name="polesDerivatives"/>, (DerivativesRequest + 1) * Dimension values) and of
    /// the weights (<paramref name="weightsDerivatives"/>, DerivativesRequest + 1 values). The result and the poles
    /// derivatives may be the same array: the overwrite is non-destructive within the algorithm.
    /// </summary>
    public static void RationalDerivatives(int derivativesRequest, int dimension, ReadOnlySpan<double> polesDerivatives,
                                           ReadOnlySpan<double> weightsDerivatives, Span<double> rationalDerivates)
    {
        int derivativeRequest = derivativesRequest;
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
        double inverse;
        ReadOnlySpan<double> polesArray = polesDerivatives;
        ReadOnlySpan<double> weightsArray = weightsDerivatives;
        Span<double> rationalArray = rationalDerivates;
        double factor;

        int ii, index, index1, index2, jj;
        int deRequest1 = derivativeRequest + 1;

        Span<double> binomial_array = deRequest1 <= 128 ? stackalloc double[deRequest1] : new double[deRequest1];

        for (ii = 0; ii < deRequest1; ii++)
        {
            binomial_array[ii] = 1.0e0;
        }
        inverse = 1.0e0 / weightsArray[0];
        if (dimension == 3)
        {
            index = 0;
            index2 = -6;

            for (ii = 0; ii < deRequest1; ii++)
            {
                index2 += 3;
                index1 = index2;
                rationalArray[index] = polesArray[index];
                index++;
                rationalArray[index] = polesArray[index];
                index++;
                rationalArray[index] = polesArray[index];
                index -= 2;

                for (jj = ii - 1; jj >= 0; jj--)
                {
                    factor = binomial_array[jj] * weightsArray[ii - jj];
                    rationalArray[index] -= factor * rationalArray[index1];
                    index++;
                    index1++;
                    rationalArray[index] -= factor * rationalArray[index1];
                    index++;
                    index1++;
                    rationalArray[index] -= factor * rationalArray[index1];
                    index -= 2;
                    index1 -= 5;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }
                rationalArray[index] *= inverse;
                index++;
                rationalArray[index] *= inverse;
                index++;
                rationalArray[index] *= inverse;
                index++;
            }
        }
        else
        {
            int kk;
            int dimension2 = dimension << 1;
            index = 0;
            index2 = -dimension2;

            for (ii = 0; ii < deRequest1; ii++)
            {
                index2 += dimension;
                index1 = index2;

                for (kk = 0; kk < dimension; kk++)
                {
                    rationalArray[index] = polesArray[index];
                    index++;
                }
                index -= dimension;

                for (jj = ii - 1; jj >= 0; jj--)
                {
                    factor = binomial_array[jj] * weightsArray[ii - jj];

                    for (kk = 0; kk < dimension; kk++)
                    {
                        rationalArray[index] -= factor * rationalArray[index1];
                        index++;
                        index1++;
                    }
                    index -= dimension;
                    index1 -= dimension2;
                }

                for (jj = ii; jj >= 1; jj--)
                {
                    binomial_array[jj] += binomial_array[jj - 1];
                }

                for (kk = 0; kk < dimension; kk++)
                {
                    rationalArray[index] *= inverse;
                    index++;
                }
            }
        }
    }

    //=======================================================================
    // Auxiliary functions used for optimized evaluation of polynome and its derivatives for smaller dimensions
    // of the polynome. OCCT dispatches dimensions 1 to 15 to templates keeping the accumulators in local arrays
    // (registers); the port specialises dimensions 1 to 4 (curves and their weights: 1, 2, 3, 4) the same way,
    // through a type parameter whose constant N lets the JIT drop the unused accumulators, and runs the general
    // loops above that. The arithmetic is the same, operation for operation, so the results are identical.
    //=======================================================================

    private interface IEvalDim
    {
        static abstract int N { get; }
    }

    private struct EvalDim1 : IEvalDim
    {
        public static int N => 1;
    }

    private struct EvalDim2 : IEvalDim
    {
        public static int N => 2;
    }

    private struct EvalDim3 : IEvalDim
    {
        public static int N => 3;
    }

    private struct EvalDim4 : IEvalDim
    {
        public static int N => 4;
    }

    // Evaluation of only value using local variables (register-friendly). Writes to output only at the end.
    private static void EvalPoly0<TDim>(Span<double> result, ReadOnlySpan<double> coeffs, int top,
                                         int degree, double par)
        where TDim : struct, IEvalDim
    {
        int dim = TDim.N;
        ReadOnlySpan<double> aTop = coeffs.Slice(top, dim);
        double l0 = aTop[0];
        double l1 = dim > 1 ? aTop[1] : 0.0;
        double l2 = dim > 2 ? aTop[2] : 0.0;
        double l3 = dim > 3 ? aTop[3] : 0.0;

        int aCoeffs = top;
        for (int deg = 0; deg < degree; ++deg)
        {
            aCoeffs -= dim;
            l0 = l0 * par + coeffs[aCoeffs];
            if (dim > 1) l1 = l1 * par + coeffs[aCoeffs + 1];
            if (dim > 2) l2 = l2 * par + coeffs[aCoeffs + 2];
            if (dim > 3) l3 = l3 * par + coeffs[aCoeffs + 3];
        }

        Span<double> res = result.Slice(0, dim);
        res[0] = l0;
        if (dim > 1) res[1] = l1;
        if (dim > 2) res[2] = l2;
        if (dim > 3) res[3] = l3;
    }

    // Evaluation of value and first derivative using local variables.
    private static void EvalPoly1<TDim>(Span<double> result, ReadOnlySpan<double> coeffs, int top,
                                         int degree, double par)
        where TDim : struct, IEvalDim
    {
        int dim = TDim.N;
        ReadOnlySpan<double> aTop = coeffs.Slice(top, dim);
        double v0 = aTop[0], d0 = 0.0;
        double v1 = dim > 1 ? aTop[1] : 0.0, d1 = 0.0;
        double v2 = dim > 2 ? aTop[2] : 0.0, d2 = 0.0;
        double v3 = dim > 3 ? aTop[3] : 0.0, d3 = 0.0;

        int aCoeffs = top;
        for (int deg = 0; deg < degree; ++deg)
        {
            aCoeffs -= dim;
            d0 = d0 * par + v0;
            v0 = v0 * par + coeffs[aCoeffs];
            if (dim > 1)
            {
                d1 = d1 * par + v1;
                v1 = v1 * par + coeffs[aCoeffs + 1];
            }
            if (dim > 2)
            {
                d2 = d2 * par + v2;
                v2 = v2 * par + coeffs[aCoeffs + 2];
            }
            if (dim > 3)
            {
                d3 = d3 * par + v3;
                v3 = v3 * par + coeffs[aCoeffs + 3];
            }
        }

        Span<double> res = result.Slice(0, 2 * dim);
        res[0] = v0;
        res[dim] = d0;
        if (dim > 1)
        {
            res[1] = v1;
            res[dim + 1] = d1;
        }
        if (dim > 2)
        {
            res[2] = v2;
            res[dim + 2] = d2;
        }
        if (dim > 3)
        {
            res[3] = v3;
            res[dim + 3] = d3;
        }
    }

    // Evaluation of value and first and second derivatives using local variables.
    private static void EvalPoly2<TDim>(Span<double> result, ReadOnlySpan<double> coeffs, int top,
                                         int degree, double par)
        where TDim : struct, IEvalDim
    {
        int dim = TDim.N;
        ReadOnlySpan<double> aTop = coeffs.Slice(top, dim);
        double v0 = aTop[0], d0 = 0.0, s0 = 0.0;
        double v1 = dim > 1 ? aTop[1] : 0.0, d1 = 0.0, s1 = 0.0;
        double v2 = dim > 2 ? aTop[2] : 0.0, d2 = 0.0, s2 = 0.0;
        double v3 = dim > 3 ? aTop[3] : 0.0, d3 = 0.0, s3 = 0.0;

        int aCoeffs = top;
        for (int deg = 0; deg < degree; ++deg)
        {
            aCoeffs -= dim;
            s0 = s0 * par + d0 * 2.0;
            d0 = d0 * par + v0;
            v0 = v0 * par + coeffs[aCoeffs];
            if (dim > 1)
            {
                s1 = s1 * par + d1 * 2.0;
                d1 = d1 * par + v1;
                v1 = v1 * par + coeffs[aCoeffs + 1];
            }
            if (dim > 2)
            {
                s2 = s2 * par + d2 * 2.0;
                d2 = d2 * par + v2;
                v2 = v2 * par + coeffs[aCoeffs + 2];
            }
            if (dim > 3)
            {
                s3 = s3 * par + d3 * 2.0;
                d3 = d3 * par + v3;
                v3 = v3 * par + coeffs[aCoeffs + 3];
            }
        }

        Span<double> res = result.Slice(0, 3 * dim);
        res[0] = v0;
        res[dim] = d0;
        res[2 * dim] = s0;
        if (dim > 1)
        {
            res[1] = v1;
            res[dim + 1] = d1;
            res[2 * dim + 1] = s1;
        }
        if (dim > 2)
        {
            res[2] = v2;
            res[dim + 2] = d2;
            res[2 * dim + 2] = s2;
        }
        if (dim > 3)
        {
            res[3] = v3;
            res[dim + 3] = d3;
            res[2 * dim + 3] = s3;
        }
    }

    // Runtime evaluations, for dimension > 4

    // Evaluation of only value. theCoeffs starts at the highest-degree coefficient (index theTop).
    private static void EvalPoly0Runtime(Span<double> result, ReadOnlySpan<double> coeffs, int top,
                                           int degree, double par, int dimension)
    {
        for (int i = 0; i < dimension; ++i)
        {
            result[i] = coeffs[top + i];
        }

        int aCoeffs = top;
        for (int deg = 0; deg < degree; ++deg)
        {
            aCoeffs -= dimension;
            for (int i = 0; i < dimension; ++i)
            {
                result[i] = result[i] * par + coeffs[aCoeffs + i];
            }
        }
    }

    // Evaluation of value + 1st derivative
    private static void EvalPoly1Runtime(Span<double> result, ReadOnlySpan<double> coeffs, int top,
                                           int degree, double par, int dimension)
    {
        Span<double> res0 = result.Slice(0, dimension);
        Span<double> res1 = result.Slice(dimension, dimension);

        for (int i = 0; i < dimension; ++i)
        {
            res0[i] = coeffs[top + i];
            res1[i] = 0.0;
        }

        int aCoeffs = top;
        for (int deg = 0; deg < degree; ++deg)
        {
            aCoeffs -= dimension;
            ReadOnlySpan<double> c = coeffs.Slice(aCoeffs, dimension);
            for (int i = 0; i < dimension; ++i)
            {
                double val = res0[i];
                res1[i] = res1[i] * par + val;
                res0[i] = val * par + c[i];
            }
        }
    }

    // Evaluation of value + 1st + 2nd derivatives
    private static void EvalPoly2Runtime(Span<double> result, ReadOnlySpan<double> coeffs, int top,
                                           int degree, double par, int dimension)
    {
        Span<double> res0 = result.Slice(0, dimension);
        Span<double> res1 = result.Slice(dimension, dimension);
        Span<double> res2 = result.Slice(2 * dimension, dimension);

        for (int i = 0; i < dimension; ++i)
        {
            res0[i] = coeffs[top + i];
            res1[i] = 0.0;
            res2[i] = 0.0;
        }

        int aCoeffs = top;
        for (int deg = 0; deg < degree; ++deg)
        {
            aCoeffs -= dimension;
            ReadOnlySpan<double> c = coeffs.Slice(aCoeffs, dimension);
            for (int i = 0; i < dimension; ++i)
            {
                double d1 = res1[i];
                double val = res0[i];
                res2[i] = res2[i] * par + d1 * 2.0;
                res1[i] = d1 * par + val;
                res0[i] = val * par + c[i];
            }
        }
    }

    /// <summary>
    /// Horner's method with synthetic division for derivatives: evaluates the polynomial of degree
    /// <paramref name="degree"/> and dimension <paramref name="dimension"/> and its derivatives up to
    /// <paramref name="derivativeOrder"/> at <paramref name="u"/>. <paramref name="polynomialCoeff"/> holds
    /// c0(1..Dimension), c1(1..Dimension), ..., cDegree(1..Dimension); <paramref name="results"/> receives
    /// f(1..Dimension), f'(1..Dimension), ..., one row of Dimension values per order.
    /// </summary>
    public static void EvalPolynomial(double u, int derivativeOrder, int degree, int dimension,
                                      ReadOnlySpan<double> polynomialCoeff, Span<double> results)
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
        double par = u;
        int derivativeRequest = derivativeOrder;
        int coeffs = degree * dimension;
        Span<double> res = results;

        switch (derivativeRequest)
        {
            case 1:
                {
                    switch (dimension)
                    {
                        case 1: EvalPoly1<EvalDim1>(res, polynomialCoeff, coeffs, degree, par); break;
                        case 2: EvalPoly1<EvalDim2>(res, polynomialCoeff, coeffs, degree, par); break;
                        case 3: EvalPoly1<EvalDim3>(res, polynomialCoeff, coeffs, degree, par); break;
                        case 4: EvalPoly1<EvalDim4>(res, polynomialCoeff, coeffs, degree, par); break;
                        default: EvalPoly1Runtime(res, polynomialCoeff, coeffs, degree, par, dimension); break;
                    }
                    break;
                }
            case 2:
                {
                    switch (dimension)
                    {
                        case 1: EvalPoly2<EvalDim1>(res, polynomialCoeff, coeffs, degree, par); break;
                        case 2: EvalPoly2<EvalDim2>(res, polynomialCoeff, coeffs, degree, par); break;
                        case 3: EvalPoly2<EvalDim3>(res, polynomialCoeff, coeffs, degree, par); break;
                        case 4: EvalPoly2<EvalDim4>(res, polynomialCoeff, coeffs, degree, par); break;
                        default: EvalPoly2Runtime(res, polynomialCoeff, coeffs, degree, par, dimension); break;
                    }
                    break;
                }
            default:
                {
                    // General case for DerivativeRequest > 2 (and 0)
                    int resSize = (1 + derivativeRequest) * dimension;
                    for (int i = 0; i < resSize; ++i)
                    {
                        res[i] = 0.0;
                    }

                    for (int deg = 0; deg <= degree; ++deg)
                    {
                        int ptr = resSize - dimension;
                        // Calculating derivatives of the polynomial
                        for (int deriv = derivativeRequest; deriv > 0; --deriv)
                        {
                            int original = ptr - dimension;
                            for (int ind = 0; ind < dimension; ++ind)
                            {
                                res[ptr + ind] = res[ptr + ind] * par + res[original + ind] * deriv;
                            }
                            ptr = original;
                        }
                        // Calculating the value of the polynomial
                        for (int ind = 0; ind < dimension; ++ind)
                        {
                            res[ptr + ind] = res[ptr + ind] * par + polynomialCoeff[coeffs + ind];
                        }
                        coeffs -= dimension;
                    }
                    break;
                }
        }
    }

    /// <summary>
    /// <see cref="EvalPolynomial"/> with no derivative. <paramref name="degreeDimension"/> is Degree * Dimension,
    /// the index of the highest-degree coefficient.
    /// </summary>
    public static void NoDerivativeEvalPolynomial(double u, int degree, int dimension, int degreeDimension,
                                                  ReadOnlySpan<double> polynomialCoeff, Span<double> results)
    {
        switch (dimension)
        {
            case 1: EvalPoly0<EvalDim1>(results, polynomialCoeff, degreeDimension, degree, u); break;
            case 2: EvalPoly0<EvalDim2>(results, polynomialCoeff, degreeDimension, degree, u); break;
            case 3: EvalPoly0<EvalDim3>(results, polynomialCoeff, degreeDimension, degree, u); break;
            case 4: EvalPoly0<EvalDim4>(results, polynomialCoeff, degreeDimension, degree, u); break;
            default: EvalPoly0Runtime(results, polynomialCoeff, degreeDimension, degree, u, dimension); break;
        }
    }

    /// <summary>
    /// Evaluates the derivative of orders <paramref name="uDerivativeOrder"/> in U and
    /// <paramref name="vDerivativeOrder"/> in V of a polynomial of two variables at (U, V), applying
    /// <see cref="EvalPolynomial"/> twice. The coefficients are stored c00, c10, ..., cm0, c01, ..., cmn (each of
    /// Dimension values, m = UDegree, n = VDegree); <paramref name="results"/> receives Dimension values.
    /// </summary>
    public static void EvalPoly2Var(double u, double v, int uDerivativeOrder, int vDerivativeOrder, int uDegree,
                                    int vDegree, int dimension, ReadOnlySpan<double> polynomialCoeff, Span<double> results)
    {
        double uParameter = u, vParameter = v;
        int uDerivativeRequest = uDerivativeOrder, vDerivativeRequest = vDerivativeOrder;
        int udim = (vDegree + 1) * dimension, index = udim * uDerivativeRequest;
        // Note: OCCT allocates both work arrays as NCollection_Array1
        int curveSize = udim * (uDerivativeRequest + 1);
        int pointSize = dimension * (vDerivativeRequest + 1);
        Span<double> result = curveSize <= 128 ? stackalloc double[curveSize] : new double[curveSize];
        Span<double> digit = pointSize <= 128 ? stackalloc double[pointSize] : new double[pointSize];
        Span<double> resultArray = results;

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(uParameter, uDerivativeRequest, uDegree, udim, polynomialCoeff, result);

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(vParameter, vDerivativeRequest, vDegree, dimension, result[index..], digit);

        index = dimension * vDerivativeRequest;

        for (int i = 0; i < dimension; i++)
        {
            resultArray[i] = digit[index + i];
        }
    }

    /// <summary>
    /// Lagrange interpolation of <paramref name="degree"/> + 1 points of dimension <paramref name="dimension"/>
    /// (<paramref name="valueArray"/>) at the parameters <paramref name="parameterArray"/>, with derivatives up to
    /// <paramref name="derivativeOrder"/>, into <paramref name="results"/> (one row of Dimension values per order).
    /// Returns 0, or 1 if two parameters are equal.
    /// </summary>
    public static int EvalLagrange(double u, int derivativeOrder, int degree, int dimension,
                                   ReadOnlySpan<double> valueArray, ReadOnlySpan<double> parameterArray, Span<double> results)
    {
        double parameter = u;
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
        int ii, jj, kk, index, index1, returnCode = 0;
        int local_request = derivativeOrder;
        double difference;
        ReadOnlySpan<double> pointsArray = valueArray;
        Span<double> resultArray = results;

        if (local_request >= degree)
        {
            local_request = degree;
        }
        int size = (degree + 1) * dimension;
        Span<double> divided_differences_array = size <= 128 ? stackalloc double[size] : new double[size];
        //
        //  Build the divided differences array
        //

        for (ii = 0; ii < (degree + 1) * dimension; ii++)
        {
            divided_differences_array[ii] = pointsArray[ii];
        }

        for (ii = degree; ii >= 0; ii--)
        {

            for (jj = degree; jj > degree - ii; jj--)
            {
                index = jj * dimension;
                index1 = index - dimension;

                for (kk = 0; kk < dimension; kk++)
                {
                    divided_differences_array[index + kk] -= divided_differences_array[index1 + kk];
                }
                difference = parameterArray[jj] - parameterArray[jj - degree - 1 + ii];
                if (Math.Abs(difference) < RealSmall())
                {
                    returnCode = 1;
                    goto FINISH;
                }
                difference = 1.0e0 / difference;

                for (kk = 0; kk < dimension; kk++)
                {
                    divided_differences_array[index + kk] *= difference;
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
        index = degree * dimension;

        for (kk = 0; kk < dimension; kk++)
        {
            resultArray[kk] = divided_differences_array[index + kk];
        }

        for (ii = dimension; ii < (local_request + 1) * dimension; ii++)
        {
            resultArray[ii] = 0.0e0;
        }

        for (ii = degree; ii >= 1; ii--)
        {
            difference = parameter - parameterArray[ii - 1];

            for (jj = local_request; jj > 0; jj--)
            {
                index = jj * dimension;
                index1 = index - dimension;

                for (kk = 0; kk < dimension; kk++)
                {
                    resultArray[index + kk] *= difference;
                    resultArray[index + kk] += resultArray[index1 + kk] * (double)jj;
                }
            }
            index = (ii - 1) * dimension;

            for (kk = 0; kk < dimension; kk++)
            {
                resultArray[kk] *= difference;
                resultArray[kk] += divided_differences_array[index + kk];
            }
        }
    FINISH:
        return (returnCode);
    }

    /// <summary>
    /// Cubic Hermite interpolation of two points (<paramref name="valueArray"/>) and their derivatives
    /// (<paramref name="derivativeArray"/>) at the two parameters <paramref name="parameterArray"/>, with
    /// derivatives up to <paramref name="derivativeOrder"/>, into <paramref name="results"/>. Returns 0.
    /// </summary>
    public static int EvalCubicHermite(double u, int derivativeOrder, int dimension, ReadOnlySpan<double> valueArray,
                                       ReadOnlySpan<double> derivativeArray, ReadOnlySpan<double> parameterArray,
                                       Span<double> results)
    {
        double parameter = u;
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
        int ii, jj, kk, pp, index, index1, degree, returnCode;
        int local_request = derivativeOrder;

        returnCode = 0;
        degree = 3;
        Span<double> parametersArray = stackalloc double[4];
        double difference;
        double inverse;
        ReadOnlySpan<double> firstLast = parameterArray;
        ReadOnlySpan<double> pointsArray = valueArray;
        ReadOnlySpan<double> derivativesArray = derivativeArray;
        Span<double> resultArray = results;

        if (local_request >= degree)
        {
            local_request = degree;
        }
        int size = (degree + 1) * dimension;
        Span<double> divided_differences_array = size <= 128 ? stackalloc double[size] : new double[size];

        for (ii = 0, jj = 0; ii < 2; ii++, jj += 2)
        {
            parametersArray[jj] = parametersArray[jj + 1] = firstLast[ii];
        }
        //
        //  Build the divided differences array
        //
        //
        // initialise it at the stage 2 of the building algorithm
        // for divided differences
        //
        inverse = firstLast[1] - firstLast[0];
        inverse = 1.0e0 / inverse;

        for (ii = 0, jj = dimension, kk = 2 * dimension, pp = 3 * dimension; ii < dimension;
             ii++, jj++, kk++, pp++)
        {
            divided_differences_array[ii] = pointsArray[ii];
            divided_differences_array[kk] = inverse * (pointsArray[jj] - pointsArray[ii]);
            divided_differences_array[jj] = derivativesArray[ii];
            divided_differences_array[pp] = derivativesArray[jj];
        }

        for (ii = 1; ii <= degree; ii++)
        {

            for (jj = degree; jj >= ii + 1; jj--)
            {
                index = jj * dimension;
                index1 = index - dimension;

                for (kk = 0; kk < dimension; kk++)
                {
                    divided_differences_array[index + kk] -= divided_differences_array[index1 + kk];
                }

                for (kk = 0; kk < dimension; kk++)
                {
                    divided_differences_array[index + kk] *= inverse;
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
        index = degree * dimension;

        for (kk = 0; kk < dimension; kk++)
        {
            resultArray[kk] = divided_differences_array[index + kk];
        }

        for (ii = dimension; ii < (local_request + 1) * dimension; ii++)
        {
            resultArray[ii] = 0.0e0;
        }

        for (ii = degree; ii >= 1; ii--)
        {
            difference = parameter - parametersArray[ii - 1];

            for (jj = local_request; jj > 0; jj--)
            {
                index = jj * dimension;
                index1 = index - dimension;

                for (kk = 0; kk < dimension; kk++)
                {
                    resultArray[index + kk] *= difference;
                    resultArray[index + kk] += resultArray[index1 + kk] * (double)jj;
                }
            }
            index = (ii - 1) * dimension;

            for (kk = 0; kk < dimension; kk++)
            {
                resultArray[kk] *= difference;
                resultArray[kk] += divided_differences_array[index + kk];
            }
        }
        //  FINISH :
        return (returnCode);
    }

    /// <summary>
    /// Computes the coefficients of the Hermite polynomials on [FirstParameter, LastParameter] with constraints of
    /// orders <paramref name="firstOrder"/> and <paramref name="lastOrder"/> at the ends, one polynomial per row of
    /// <paramref name="matrixCoefs"/> (rows and columns from 1). Returns false on invalid input or a singular system.
    /// </summary>
    public static bool HermiteCoefficients(double firstParameter, double lastParameter, int firstOrder, int lastOrder,
                                           Matrix matrixCoefs)
    {
        int nbCoeff = firstOrder + lastOrder + 2;
        Span<int> ordre = stackalloc int[2];
        int ii, jj, pp, cote, iof = 0;
        double prod, tBorne = firstParameter;
        math_Vector coeff = new(1, nbCoeff), b = new(1, nbCoeff, 0.0);
        Matrix mat = new(1, nbCoeff, 1, nbCoeff, 0.0);

        // Test de validites

        if ((firstOrder < 0) || (lastOrder < 0))
        {
            return false;
        }
        double d1 = Math.Abs(firstParameter), d2 = Math.Abs(lastParameter);
        if (d1 > 100 || d2 > 100)
        {
            return false;
        }
        d2 += d1;
        if (d2 < 0.01)
        {
            return false;
        }
        if (Math.Abs(lastParameter - firstParameter) / d2 < 0.01)
        {
            return false;
        }

        // Calcul de la matrice a inverser (MAT)

        ordre[0] = firstOrder + 1;
        ordre[1] = lastOrder + 1;

        for (cote = 0; cote <= 1; cote++)
        {
            coeff.Init(1);

            for (pp = 1; pp <= ordre[cote]; pp++)
            {
                ii = pp + iof;
                prod = 1;

                for (jj = pp; jj <= nbCoeff; jj++)
                {
                    //        tout se passe dans les 3 lignes suivantes
                    mat[ii, jj] = coeff[jj] * prod;
                    coeff[jj] *= jj - pp;
                    prod *= tBorne;
                }
            }
            tBorne = lastParameter;
            iof = ordre[0];
        }

        // resolution du systemes
        Gauss resolCoeff = new(mat, 1.0e-10);
        if (!resolCoeff.IsDone)
        {
            return false;
        }

        for (ii = 1; ii <= nbCoeff; ii++)
        {
            b[ii] = 1;
            resolCoeff.Solve(b, coeff);
            matrixCoefs.SetRow(ii, coeff);
            b[ii] = 0;
        }
        return true;
    }

    /// <summary>
    /// Converts the coefficients of a polynomial curve (and of its weights, or <see cref="NoWeights"/>) in the
    /// canonical base into poles (and weights) in the Bernstein base.
    /// </summary>
    public static void CoefficientsPoles(NCollection_Array1<Point3D> coefs, NCollection_Array1<double>? wCoefs,
                                         NCollection_Array1<Point3D> poles, NCollection_Array1<double>? wPoles)
    {
        NCollection_Array1<double> tempC = new(1, 3 * coefs.Length());
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(coefs, tempC);
        NCollection_Array1<double> tempP = new(1, 3 * poles.Length());
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(coefs, tempP); // OCCT fills tempP from Coefs, not Poles; only its size comes from Poles
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(3, tempC, wCoefs, tempP, wPoles);
        global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(tempP, poles);
    }

    /// <inheritdoc cref="CoefficientsPoles(NCollection_Array1{Point3D}, NCollection_Array1{double}?, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/>
    public static void CoefficientsPoles(NCollection_Array1<Point2D> coefs, NCollection_Array1<double>? wCoefs,
                                         NCollection_Array1<Point2D> poles, NCollection_Array1<double>? wPoles)
    {
        NCollection_Array1<double> tempC = new(1, 2 * coefs.Length());
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(coefs, tempC);
        NCollection_Array1<double> tempP = new(1, 2 * poles.Length());
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(coefs, tempP); // OCCT fills tempP from Coefs, not Poles; only its size comes from Poles
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(2, tempC, wCoefs, tempP, wPoles);
        global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(tempP, poles);
    }

    /// <inheritdoc cref="CoefficientsPoles(NCollection_Array1{Point3D}, NCollection_Array1{double}?, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/>
    public static void CoefficientsPoles(NCollection_Array1<double> coefs, NCollection_Array1<double>? wCoefs,
                                         NCollection_Array1<double> poles, NCollection_Array1<double>? wPoles)
    {
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(1, coefs, wCoefs, poles, wPoles);
    }

    /// <summary>
    /// Converts the coefficients of a polynomial curve of dimension <paramref name="dim"/> (flat array) into poles
    /// in the Bernstein base.
    /// </summary>
    public static void CoefficientsPoles(int dim, NCollection_Array1<double> coefs, NCollection_Array1<double>? wCoefs,
                                         NCollection_Array1<double> poles, NCollection_Array1<double>? wPoles)
    {
        NCollection_Array1<double>? weights = wPoles;
        bool rat = wCoefs != null;
        int loc = coefs.Lower();
        int lop = poles.Lower();
        int lowc = 0;
        int lowp = 0;
        int upc = coefs.Upper();
        int upp = poles.Upper();
        int upwc = 0;
        int upwp = 0;
        int reflen = coefs.Length() / dim;
        int i, j, k;
        // Les Extremites.
        if (rat)
        {
            lowc = wCoefs!.Lower();
            lowp = weights!.Lower();
            upwc = wCoefs.Upper();
            upwp = weights.Upper();
        }

        for (i = 0; i < dim; i++)
        {
            poles[lop + i] = coefs[loc + i];
            poles[upp - i] = coefs[upc - i];
        }
        if (rat)
        {
            weights![lowp] = wCoefs![lowc];
            weights[upwp] = wCoefs[upwc];
        }

        double cnp;
        for (i = 2; i < reflen; i++)
        {
            cnp = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(reflen - 1, i - 1);
            if (rat)
            {
                weights![lowp + i - 1] = wCoefs![lowc + i - 1] / cnp;
            }

            for (j = 0; j < dim; j++)
            {
                poles[lop + dim * (i - 1) + j] = coefs[loc + dim * (i - 1) + j] / cnp;
            }
        }

        for (i = 1; i <= reflen - 1; i++)
        {

            for (j = reflen - 1; j >= i; j--)
            {
                if (rat)
                {
                    weights![lowp + j] += weights[lowp + j - 1];
                }

                for (k = 0; k < dim; k++)
                {
                    poles[lop + dim * j + k] += poles[lop + dim * (j - 1) + k];
                }
            }
        }
        if (rat)
        {

            for (i = 1; i <= reflen; i++)
            {

                for (j = 0; j < dim; j++)
                {
                    poles[lop + dim * (i - 1) + j] /= weights![lowp + i - 1];
                }
            }
        }
    }

    /// <summary>
    /// Reparametrises the polynomial curve (power-basis coefficients <paramref name="coeffs"/>, weights or
    /// <see cref="NoWeights"/>) so that [U1, U2] becomes [0, 1]: the change of variable u = U1 + v * (U2 - U1).
    /// </summary>
    public static void Trimming(double u1, double u2, NCollection_Array1<Point3D> coeffs, NCollection_Array1<double>? wCoeffs)
    {
        NCollection_Array1<double> temp = new(1, 3 * coeffs.Length());
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(coeffs, temp);
        global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(u1, u2, 3, temp, wCoeffs);
        global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(temp, coeffs);
    }

    /// <inheritdoc cref="Trimming(double, double, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/>
    public static void Trimming(double u1, double u2, NCollection_Array1<Point2D> coeffs, NCollection_Array1<double>? wCoeffs)
    {
        NCollection_Array1<double> temp = new(1, 2 * coeffs.Length());
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(coeffs, temp);
        global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(u1, u2, 2, temp, wCoeffs);
        global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(temp, coeffs);
    }

    /// <inheritdoc cref="Trimming(double, double, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/>
    public static void Trimming(double u1, double u2, NCollection_Array1<double> coeffs, NCollection_Array1<double>? wCoeffs)
    {
        global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(u1, u2, 1, coeffs, wCoeffs);
    }

    /// <summary>
    /// Reparametrises the polynomial curve of dimension <paramref name="dim"/> (flat array) so that [U1, U2]
    /// becomes [0, 1]. Throws <see cref="FailureException"/> if the weights don't match the coefficients.
    /// </summary>
    public static void Trimming(double u1, double u2, int dim, NCollection_Array1<double> coeffs, NCollection_Array1<double>? wCoeffs)
    {
        NCollection_Array1<double> coefs = coeffs;
        NCollection_Array1<double>? wCoefs = wCoeffs;

        // principe :
        // on fait le changement de variable v = (u-U1) / (U2-U1)
        // on exprime u = f(v) que l'on remplace dans l'expression polynomiale
        // decomposee sous la forme du schema iteratif de horner.

        double lsp = u2 - u1;
        int indc, indw = 0;
        int upc = coefs.Upper() - dim + 1, upw = 0;
        int len = coefs.Length() / dim;
        bool rat = wCoefs != null;

        if (rat)
        {
            if (len != wCoefs!.Length())
            {
                throw new FailureException("PLib::Trimming : nbcoefs/dim != nbweights !!!");
            }
            upw = wCoefs.Upper();
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
                coefs[indc - dim + j] += u1 * coefs[indc + j];
            }
            if (rat)
            {
                wCoefs![indw - 1] += u1 * wCoefs[indw];
            }

            // calcul des coefficients intermediaires :

            while (indc < upc)
            {
                indc += dim;

                for (int k = 0; k < dim; k++)
                {
                    coefs[indc - dim + k] = u1 * coefs[indc + k] + lsp * coefs[indc - dim + k];
                }
                if (rat)
                {
                    indw++;
                    wCoefs![indw - 1] = u1 * wCoefs[indw] + lsp * wCoefs[indw - 1];
                }
            }

            // calcul du coefficient de degre le plus eleve :

            for (j = 0; j < dim; j++)
            {
                coefs[upc + j] *= lsp;
            }
            if (rat)
            {
                wCoefs![upw] *= lsp;
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
    public static void CoefficientsPoles(NCollection_Array2<Point3D> coefs, NCollection_Array2<double>? wCoefs,
                                         NCollection_Array2<Point3D> poles, NCollection_Array2<double>? wPoles)
    {
        NCollection_Array2<double>? weights = wPoles;
        bool rat = (wCoefs != null);
        int lowerRow = poles.LowerRow();
        int upperRow = poles.UpperRow();
        int lowerCol = poles.LowerCol();
        int upperCol = poles.UpperCol();
        int colLength = poles.ColLength();
        int rowLength = poles.RowLength();

        // Bidouille pour retablir u et v pour les coefs calcules
        // par buildcache
        //  bool inv = false; //ColLength != Coefs.ColLength();

        int row, col;
        double w, cnp;

        int i1, i2;
        int nPoleu, nPolev;
        Xyz temp = new();

        for (nPoleu = lowerRow; nPoleu <= upperRow; nPoleu++)
        {
            poles[nPoleu, lowerCol] = coefs[nPoleu, lowerCol];
            if (rat)
            {
                weights![nPoleu, lowerCol] = wCoefs![nPoleu, lowerCol];
            }

            for (col = lowerCol + 1; col <= upperCol - 1; col++)
            {
                cnp = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(rowLength - 1, col - lowerCol);
                temp = coefs[nPoleu, col].XYZ();
                temp.Divide(cnp);
                poles[nPoleu, col] = poles[nPoleu, col].WithXYZ(temp);
                if (rat)
                {
                    weights![nPoleu, col] = wCoefs![nPoleu, col] / cnp;
                }
            }
            poles[nPoleu, upperCol] = coefs[nPoleu, upperCol];
            if (rat)
            {
                weights![nPoleu, upperCol] = wCoefs![nPoleu, upperCol];
            }

            for (i1 = 1; i1 <= rowLength - 1; i1++)
            {

                for (i2 = upperCol; i2 >= lowerCol + i1; i2--)
                {
                    temp.SetLinearForm(poles[nPoleu, i2].XYZ(), poles[nPoleu, i2 - 1].XYZ());
                    poles[nPoleu, i2] = poles[nPoleu, i2].WithXYZ(temp);
                    if (rat)
                    {
                        weights![nPoleu, i2] += weights[nPoleu, i2 - 1];
                    }
                }
            }
        }

        for (nPolev = lowerCol; nPolev <= upperCol; nPolev++)
        {

            for (row = lowerRow + 1; row <= upperRow - 1; row++)
            {
                cnp = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(colLength - 1, row - lowerRow);
                temp = poles[row, nPolev].XYZ();
                temp.Divide(cnp);
                poles[row, nPolev] = poles[row, nPolev].WithXYZ(temp);
                if (rat)
                {
                    weights![row, nPolev] /= cnp;
                }
            }

            for (i1 = 1; i1 <= colLength - 1; i1++)
            {

                for (i2 = upperRow; i2 >= lowerRow + i1; i2--)
                {
                    temp.SetLinearForm(poles[i2, nPolev].XYZ(), poles[i2 - 1, nPolev].XYZ());
                    poles[i2, nPolev] = poles[i2, nPolev].WithXYZ(temp);
                    if (rat)
                    {
                        weights![i2, nPolev] += weights[i2 - 1, nPolev];
                    }
                }
            }
        }
        if (rat)
        {

            for (row = lowerRow; row <= upperRow; row++)
            {

                for (col = lowerCol; col <= upperCol; col++)
                {
                    w = weights![row, col];
                    temp = poles[row, col].XYZ();
                    temp.Divide(w);
                    poles[row, col] = poles[row, col].WithXYZ(temp);
                }
            }
        }
    }

    /// <summary>Applies <see cref="Trimming(double, double, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/> to every column (the U direction) of a polynomial surface.</summary>
    public static void UTrimming(double u1, double u2, NCollection_Array2<Point3D> coeffs, NCollection_Array2<double>? wCoeffs)
    {
        bool rat = wCoeffs != null;
        int lr = coeffs.LowerRow();
        int ur = coeffs.UpperRow();
        int lc = coeffs.LowerCol();
        int uc = coeffs.UpperCol();
        NCollection_Array1<Point3D> temp = new(lr, ur);
        NCollection_Array1<double> temw = new(lr, ur);

        for (int icol = lc; icol <= uc; icol++)
        {
            int irow;
            for (irow = lr; irow <= ur; irow++)
            {
                temp[irow] = coeffs[irow, icol];
                if (rat)
                {
                    temw[irow] = wCoeffs![irow, icol];
                }
            }
            if (rat)
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(u1, u2, temp, temw);
            }
            else
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(u1, u2, temp, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            }

            for (irow = lr; irow <= ur; irow++)
            {
                coeffs[irow, icol] = temp[irow];
                if (rat)
                {
                    wCoeffs![irow, icol] = temw[irow];
                }
            }
        }
    }

    /// <summary>Applies <see cref="Trimming(double, double, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/> to every row (the V direction) of a polynomial surface.</summary>
    public static void VTrimming(double v1, double v2, NCollection_Array2<Point3D> coeffs, NCollection_Array2<double>? wCoeffs)
    {
        bool rat = wCoeffs != null;
        int lr = coeffs.LowerRow();
        int ur = coeffs.UpperRow();
        int lc = coeffs.LowerCol();
        int uc = coeffs.UpperCol();
        NCollection_Array1<Point3D> temp = new(lc, uc);
        NCollection_Array1<double> temw = new(lc, uc);

        for (int irow = lr; irow <= ur; irow++)
        {
            int icol;
            for (icol = lc; icol <= uc; icol++)
            {
                temp[icol] = coeffs[irow, icol];
                if (rat)
                {
                    temw[icol] = wCoeffs![irow, icol];
                }
            }
            if (rat)
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(v1, v2, temp, temw);
            }
            else
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(v1, v2, temp, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            }

            for (icol = lc; icol <= uc; icol++)
            {
                coeffs[irow, icol] = temp[icol];
                if (rat)
                {
                    wCoeffs![irow, icol] = temw[icol];
                }
            }
        }
    }

    /// <summary>
    /// Computes the coefficients (canonical base, in <paramref name="coefficients"/> from index 0) of the polynomial
    /// of dimension <paramref name="dimension"/> that meets the constraints <paramref name="firstConstr"/> (orders 0
    /// to FirstOrder, at FirstParameter) and <paramref name="lastConstr"/> (orders 0 to LastOrder, at LastParameter).
    /// The constraint arrays are indexed (1..Dimension, 0..order). Orders are at most 2.
    /// </summary>
    public static bool HermiteInterpolate(int dimension, double firstParameter, double lastParameter, int firstOrder,
                                          int lastOrder, NCollection_Array2<double> firstConstr,
                                          NCollection_Array2<double> lastConstr, NCollection_Array1<double> coefficients)
    {
        // double Pattern[3][6]
        Span<double> pattern = stackalloc double[3 * 6];

        // portage HP : il faut les initialiser 1 par 1

        pattern[0 * 6 + 0] = 1;
        pattern[0 * 6 + 1] = 1;
        pattern[0 * 6 + 2] = 1;
        pattern[0 * 6 + 3] = 1;
        pattern[0 * 6 + 4] = 1;
        pattern[0 * 6 + 5] = 1;
        pattern[1 * 6 + 0] = 0;
        pattern[1 * 6 + 1] = 1;
        pattern[1 * 6 + 2] = 2;
        pattern[1 * 6 + 3] = 3;
        pattern[1 * 6 + 4] = 4;
        pattern[1 * 6 + 5] = 5;
        pattern[2 * 6 + 0] = 0;
        pattern[2 * 6 + 1] = 0;
        pattern[2 * 6 + 2] = 2;
        pattern[2 * 6 + 3] = 6;
        pattern[2 * 6 + 4] = 12;
        pattern[2 * 6 + 5] = 20;

        Matrix a = new(0, firstOrder + lastOrder + 1, 0, firstOrder + lastOrder + 1);
        //  The initialisation of the matrix A
        int irow;
        for (irow = 0; irow <= firstOrder; irow++)
        {
            double firstVal = 1.0;

            for (int icol = 0; icol <= firstOrder + lastOrder + 1; icol++)
            {
                a[irow, icol] = pattern[irow * 6 + icol] * firstVal;
                if (irow <= icol)
                {
                    firstVal *= firstParameter;
                }
            }
        }

        for (irow = 0; irow <= lastOrder; irow++)
        {
            double lastVal = 1.0;

            for (int icol = 0; icol <= firstOrder + lastOrder + 1; icol++)
            {
                a[irow + firstOrder + 1, icol] = pattern[irow * 6 + icol] * lastVal;
                if (irow <= icol)
                {
                    lastVal *= lastParameter;
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

        Gauss equations = new(a);

        for (int idim = 1; idim <= dimension; idim++)
        {
            math_Vector b = new(0, firstOrder + lastOrder + 1);
            int icol;
            for (icol = 0; icol <= firstOrder; icol++)
            {
                b[icol] = firstConstr[idim, icol];
            }

            for (icol = 0; icol <= lastOrder; icol++)
            {
                b[firstOrder + 1 + icol] = lastConstr[idim, icol];
            }

            //  The solving of equations system A * X = B. Then B = X
            // Note: OCCT checks IsDone only after Solve, which (with its checks on, as in the port) throws
            // StdFail_NotDone first when A is singular
            equations.Solve(b);

            if (!equations.IsDone)
            {
                return false;
            }

            //  the filling of the Coefficients

            for (icol = 0; icol <= firstOrder + lastOrder + 1; icol++)
            {
                coefficients[dimension * icol + idim - 1] = b[icol];
            }
        }
        return true;
    }

    /// <summary>
    /// Computes the number of Gauss points and the working degree for a Jacobi approximation with the given
    /// constraint order (C0, C1 or C2), maximum degree and precision code (-5 to -1, 1 to 4).
    /// Throws <see cref="ConstructionErrorException"/> on invalid arguments.
    /// </summary>
    public static void JacobiParameters(Continuity constraintOrder, int maxDegree, int code, out int nbGaussPoints,
                                        out int workDegree)
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

        const int ndeg8 = 8, ndeg10 = 10, ndeg15 = 15, ndeg20 = 20, ndeg25 = 25, ndeg30 = 30, ndeg40 = 40,
                  ndeg50 = 50, ndeg61 = 61;

        int nivConstr = 0;
        switch (constraintOrder)
        {
            case C0:
                nivConstr = 0;
                break;
            case C1:
                nivConstr = 1;
                break;
            case C2:
                nivConstr = 2;
                break;
            default:
                throw new ConstructionErrorException("Invalid ConstraintOrder");
        }
        if (maxDegree < 2 * nivConstr + 1)
        {
            throw new ConstructionErrorException("Invalid MaxDegree");
        }

        if (code >= 1)
        {
            workDegree = maxDegree + 9;
        }
        else
        {
            workDegree = maxDegree + 6;
        }

        //---> Nbre mini de points necessaires.
        int ipmin = 0;
        if (workDegree < ndeg8)
        {
            ipmin = ndeg8;
        }
        else if (workDegree < ndeg10)
        {
            ipmin = ndeg10;
        }
        else if (workDegree < ndeg15)
        {
            ipmin = ndeg15;
        }
        else if (workDegree < ndeg20)
        {
            ipmin = ndeg20;
        }
        else if (workDegree < ndeg25)
        {
            ipmin = ndeg25;
        }
        else if (workDegree < ndeg30)
        {
            ipmin = ndeg30;
        }
        else if (workDegree < ndeg40)
        {
            ipmin = ndeg40;
        }
        else if (workDegree < ndeg50)
        {
            ipmin = ndeg50;
        }
        else if (workDegree < ndeg61)
        {
            ipmin = ndeg61;
        }
        else
        {
            throw new ConstructionErrorException("Invalid MaxDegree");
        }
        // ---> Nbre de points voulus.
        int iwant = 0;
        switch (code)
        {
            case -5:
                iwant = ndeg8;
                break;
            case -4:
                iwant = ndeg10;
                break;
            case -3:
                iwant = ndeg15;
                break;
            case -2:
                iwant = ndeg20;
                break;
            case -1:
                iwant = ndeg25;
                break;
            case 1:
                iwant = ndeg30;
                break;
            case 2:
                iwant = ndeg40;
                break;
            case 3:
                iwant = ndeg50;
                break;
            case 4:
                iwant = ndeg61;
                break;
            default:
                throw new ConstructionErrorException("Invalid Code");
        }
        //-->  NbGaussPoints est le nombre de points de discretisation de la fonction,
        //     il ne peut prendre que les valeurs 8,10,15,20,25,30,40,50 ou 61.
        //     NbGaussPoints doit etre superieur strictement a WorkDegree.
        nbGaussPoints = Max(ipmin, iwant);
        //  NbGaussPoints +=2;
    }

    /// <summary>Translates a constraint order (C0, C1, C2) into 0, 1, 2.</summary>
    public static int NivConstr(Continuity constraintOrder)
    {
        int nivConstr = 0;
        switch (constraintOrder)
        {
            case C0:
                nivConstr = 0;
                break;
            case C1:
                nivConstr = 1;
                break;
            case C2:
                nivConstr = 2;
                break;
            default:
                throw new ConstructionErrorException("Invalid ConstraintOrder");
        }
        return nivConstr;
    }

    /// <summary>Translates 0, 1, 2 into a constraint order (C0, C1, C2).</summary>
    public static Continuity ConstraintOrder(int nivConstr)
    {
        Continuity constraintOrder = C0;
        switch (nivConstr)
        {
            case 0:
                constraintOrder = C0;
                break;
            case 1:
                constraintOrder = C1;
                break;
            case 2:
                constraintOrder = C2;
                break;
            default:
                throw new ConstructionErrorException("Invalid NivConstr");
        }
        return constraintOrder;
    }

    /// <summary>
    /// Computes the length of the polynomial curve (coefficients as for <see cref="EvalPolynomial"/>) between
    /// <paramref name="u1"/> and <paramref name="u2"/> by Gauss integration.
    /// </summary>
    public static void EvalLength(int degree, int dimension, ReadOnlySpan<double> polynomialCoeff, double u1, double u2,
                                  out double length)
    {
        int i, j, idim, degdim;
        double c1, c2, sum, tran, x1, x2, der1, der2, d1, d2, dd;

        ReadOnlySpan<double> polynomialArray = polynomialCoeff;

        int nbGaussPoints = 4 * Min((degree / 4) + 1, 10);

        math_Vector gaussPoints = new(1, nbGaussPoints);
        NumericTables.GaussPoints(nbGaussPoints, gaussPoints);

        math_Vector gaussWeights = new(1, nbGaussPoints);
        NumericTables.GaussWeights(nbGaussPoints, gaussWeights);

        c1 = (u2 + u1) / 2.0;
        c2 = (u2 - u1) / 2.0;

        //-----------------------------------------------------------
        //****** Integration - Boucle sur les intervalles de GAUSS **
        //-----------------------------------------------------------

        sum = 0;

        for (j = 1; j <= nbGaussPoints / 2; j++)
        {
            // Integration en tenant compte de la symetrie
            tran = c2 * gaussPoints[j];
            x1 = c1 + tran;
            x2 = c1 - tran;

            //****** Derivation sur la dimension de l'espace **

            degdim = degree * dimension;
            der1 = der2 = 0.0;
            for (idim = 0; idim < dimension; idim++)
            {
                d1 = d2 = degree * polynomialArray[idim + degdim];
                for (i = degree - 1; i >= 1; i--)
                {
                    dd = i * polynomialArray[idim + i * dimension];
                    d1 = d1 * x1 + dd;
                    d2 = d2 * x2 + dd;
                }
                der1 += d1 * d1;
                der2 += d2 * d2;
            }

            //****** Integration **

            sum += gaussWeights[j] * c2 * (Math.Sqrt(der1) + Math.Sqrt(der2));

            //****** Fin de boucle dur les intervalles de GAUSS **
        }
        length = sum;
    }

    /// <summary>
    /// Computes the length of the polynomial curve between <paramref name="u1"/> and <paramref name="u2"/>,
    /// subdividing until two successive estimates differ by at most <paramref name="tol"/> (or 13 iterations);
    /// <paramref name="error"/> is that last difference.
    /// </summary>
    public static void EvalLength(int degree, int dimension, ReadOnlySpan<double> polynomialCoeff, double u1, double u2,
                                  double tol, out double length, out double error)
    {
        int i;
        int nbSubInt = 1,  // Current number of subintervals
          maxNbIter = 13, // Max number of iterations
          nbIter = 1;  // Current number of iterations
        double dU, oldLen, lenI;

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(degree, dimension, polynomialCoeff, u1, u2, out length);

        do
        {
            oldLen = length;
            length = 0.0;
            nbSubInt *= 2;
            dU = (u2 - u1) / nbSubInt;
            for (i = 1; i <= nbSubInt; i++)
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(degree, dimension, polynomialCoeff, u1 + (i - 1) * dU, u1 + i * dU, out lenI);
                length += lenI;
            }
            nbIter++;
            error = Math.Abs(oldLen - length);
        } while (error > tol && nbIter <= maxNbIter);
    }
}
