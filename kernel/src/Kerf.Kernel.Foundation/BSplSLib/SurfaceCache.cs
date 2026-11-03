// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib_Cache.hxx
// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib_Cache.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Splines;

/// <summary>
/// A cache of the polynomial coefficients of one span (patch) of a Bezier or B-spline surface, for fast
/// evaluation. The data must be rebuilt (<see cref="BuildCache"/>) when going from span to span
/// (<see cref="IsCacheValid"/>).
/// </summary>
/// <remarks>
/// Mutable: BuildCache changes it (decision 79). OCCT holds the coefficients in an NCollection_HArray2; the port
/// holds an NCollection_Array2 allocated once, in the constructor. Evaluation allocates nothing unless the smaller
/// of the two degrees is above 9 (13 for a non-rational surface), where the derivatives' work array outgrows
/// NCollection_LocalArray's 128-value stack buffer and goes to the heap, in OCCT as here.
/// </remarks>
public class SurfaceCache : Transient
{
    private readonly bool _isRational;                                 //!< identifies the rationality of Bezier/B-spline surface
    private readonly CacheParams _paramsU, _paramsV;         //!< cache parameters by U and V directions
    // array of poles and weights of calculated cache
    // the array has following structure:
    //       x11 y11 z11 [w11] x12 y12 z12 [w12] ...
    //       x21 y21 z21 [w21] x22 y22 z22 [w22] etc
    // for non-rational surfaces there is no weight;
    // size of array: (max(myDegree)+1) * A*(min(myDegree)+1), where A = 4 or 3
    private readonly NCollection_Array2<double> _polesWeights;

    /// <summary>
    /// Prepares the cache of a surface of the given degrees, periodicity and flat knots (knots with repetitions)
    /// along U and V; <paramref name="weights"/> only says whether the surface is rational.
    /// </summary>
    public SurfaceCache(int degreeU,
                          bool periodicU,
                          NCollection_Array1<double> flatKnotsU,
                          int degreeV,
                          bool periodicV,
                          NCollection_Array1<double> flatKnotsV,
                          NCollection_Array2<double>? weights = null)
    {
        _isRational = weights != null;
        _paramsU = new CacheParams(degreeU, periodicU, flatKnotsU);
        _paramsV = new CacheParams(degreeV, periodicV, flatKnotsV);
        int minDegree = Min(degreeU, degreeV);
        int maxDegree = Max(degreeU, degreeV);
        int pwColNumber = (_isRational ? 4 : 3);
        _polesWeights = new NCollection_Array2<double>(1, maxDegree + 1, 1, pwColNumber * (minDegree + 1));
    }

    /// <summary>True if the cached span is the one of the point (<paramref name="parameterU"/>, <paramref name="parameterV"/>).</summary>
    public bool IsCacheValid(double parameterU, double parameterV)
    {
        return _paramsU.IsCacheValid(parameterU) && _paramsV.IsCacheValid(parameterV);
    }

    /// <summary>
    /// Recomputes the cache for the span of (<paramref name="parameterU"/>, <paramref name="parameterV"/>),
    /// without checking whether it is valid.
    /// </summary>
    internal void BuildCache(double parameterU,
                             double parameterV,
                             NCollection_Array1<double> flatKnotsU,
                             NCollection_Array1<double> flatKnotsV,
                             NCollection_Array2<Point3D> poles,
                             NCollection_Array2<double>? weights = null)
    {
        // Normalize the parameters for periodical B-splines
        double newParamU = _paramsU.PeriodicNormalization(parameterU);
        double newParamV = _paramsV.PeriodicNormalization(parameterV);

        _paramsU.LocateParameter(ref newParamU, flatKnotsU);
        _paramsV.LocateParameter(ref newParamV, flatKnotsV);

        // BSplSLib uses different convention for span parameters than BSplCLib
        // (Start is in the middle of the span and length is half-span),
        // thus we need to amend them here
        double spanLengthU = 0.5 * _paramsU.SpanLength;
        double spanStartU = _paramsU.SpanStart + spanLengthU;
        double spanLengthV = 0.5 * _paramsV.SpanLength;
        double spanStartV = _paramsV.SpanStart + spanLengthV;

        // Calculate new cache data
        SurfaceBSplSLib.BuildCache(spanStartU,
                            spanStartV,
                            spanLengthU,
                            spanLengthV,
                            _paramsU.IsPeriodic,
                            _paramsV.IsPeriodic,
                            _paramsU.Degree,
                            _paramsV.Degree,
                            _paramsU.SpanIndex,
                            _paramsV.SpanIndex,
                            flatKnotsU,
                            flatKnotsV,
                            poles,
                            weights,
                            _polesWeights);
    }

    /// <summary>The point of the surface at (<paramref name="u"/>, <paramref name="v"/>).</summary>
    public void D0(double u, double v, out Point3D point)
    {
        var (localU, localV) = ToLocalParamsD0(u, v, _paramsU, _paramsV);
        D0Local(localU, localV, out point);
    }

    /// <summary>
    /// The point at the local parameters in [-1, 1], (U - SpanMid) / SpanHalfLen and the same for V; skips the
    /// periodic normalization and the local parameter computation.
    /// </summary>
    public void D0Local(double localU, double localV, out Point3D point)
    {
        ReadOnlySpan<double> polesArray = _polesWeights.AsSpan();
        Span<double> aPoint = stackalloc double[4];

        int dimension = _isRational ? 4 : 3;
        int cacheCols = _polesWeights.RowLength();
        bool isMaxU = (_paramsU.Degree > _paramsV.Degree);
        int minDegree = Min(_paramsU.Degree, _paramsV.Degree);
        int maxDegree = Max(_paramsU.Degree, _paramsV.Degree);
        var (minParam, maxParam) = isMaxU ? (localV, localU) : (localU, localV);

        // Array for intermediate results (at most 4 * 26 = 104 values)
        Span<double> transientCoeffs = stackalloc double[cacheCols];

        // Calculate intermediate value of cached polynomial along variable with maximal degree
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(maxParam,
                                        maxDegree,
                                        cacheCols,
                                        maxDegree * cacheCols,
                                        polesArray,
                                        transientCoeffs);

        // Calculate total value along variable with minimal degree
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(minParam,
                                        minDegree,
                                        dimension,
                                        dimension * minDegree,
                                        transientCoeffs,
                                        aPoint);

        point = new Point3D(aPoint[0], aPoint[1], aPoint[2]);
        if (_isRational)
        {
            point = new Point3D(point.XYZ().Divided(aPoint[3]));
        }
    }

    /// <summary>The point and first derivatives at the local parameters in [-1, 1] (see <see cref="D0Local"/>).</summary>
    public void D1Local(double localU, double localV, out Point3D point, out Vector3D tangentU,
                        out Vector3D tangentV)
    {
        Span<double> pntDeriv = stackalloc double[16]; // Result storage for D1, zero-initialized
        EvaluatePolynomials(_polesWeights,
                            _paramsU,
                            _paramsV,
                            _isRational,
                            localU,
                            localV,
                            1,
                            1,
                            pntDeriv);

        // After RationalDerivative (for rational surfaces), the output dimension is 3 (not 4)
        // because weights have been processed out
        const int dimension = 3;

        point = new Point3D(pntDeriv[0], pntDeriv[1], pntDeriv[2]);

        // Tangents are stored after the point coordinates
        // Order depends on which parameter has higher degree
        // If U degree > V degree: layout is [P, DV, DU, ...]
        // If V degree >= U degree: layout is [P, DU, DV, ...]
        if (_paramsU.Degree > _paramsV.Degree)
        {
            tangentV = new Vector3D(pntDeriv[dimension],
                                     pntDeriv[dimension + 1],
                                     pntDeriv[dimension + 2]);
            tangentU = new Vector3D(pntDeriv[dimension << 1],
                                     pntDeriv[(dimension << 1) + 1],
                                     pntDeriv[(dimension << 1) + 2]);
        }
        else
        {
            tangentU = new Vector3D(pntDeriv[dimension],
                                     pntDeriv[dimension + 1],
                                     pntDeriv[dimension + 2]);
            tangentV = new Vector3D(pntDeriv[dimension << 1],
                                     pntDeriv[(dimension << 1) + 1],
                                     pntDeriv[(dimension << 1) + 2]);
        }

        // Use direct division for better numerical stability with very small span lengths
        double spanLengthU = 0.5 * _paramsU.SpanLength;
        double spanLengthV = 0.5 * _paramsV.SpanLength;
        tangentU = tangentU.Divided(spanLengthU);
        tangentV = tangentV.Divided(spanLengthV);
    }

    /// <summary>
    /// The point and derivatives up to the second order at the local parameters in [-1, 1] (see
    /// <see cref="D0Local"/>).
    /// </summary>
    public void D2Local(double localU, double localV, out Point3D point, out Vector3D tangentU,
                        out Vector3D tangentV, out Vector3D curvatureU, out Vector3D curvatureV,
                        out Vector3D curvatureUV)
    {
        Span<double> pntDeriv = stackalloc double[36]; // Result storage for D2, zero-initialized
        EvaluatePolynomials(_polesWeights,
                            _paramsU,
                            _paramsV,
                            _isRational,
                            localU,
                            localV,
                            2,
                            2,
                            pntDeriv);

        // After RationalDerivative (for rational surfaces), the output dimension is 3 (not 4)
        // because weights have been processed out
        const int dimension = 3;
        const int shift = dimension; // Shift for first derivatives
        const int shift2 = dimension << 1;
        const int shift3 = shift2 + dimension;
        const int shift4 = shift3 + dimension;
        const int shift6 = 6 * dimension;

        point = new Point3D(pntDeriv[0], pntDeriv[1], pntDeriv[2]);

        // Derivatives are stored consecutively
        // If Max=U (U degree > V degree):
        // [0]=P, [Dim]=DV, [2Dim]=DVV
        // [3Dim]=DU, [4Dim]=DUV
        // [6Dim]=DUU
        if (_paramsU.Degree > _paramsV.Degree)
        {
            tangentV = new Vector3D(pntDeriv[shift], pntDeriv[shift + 1], pntDeriv[shift + 2]);
            curvatureV = new Vector3D(pntDeriv[shift2], pntDeriv[shift2 + 1], pntDeriv[shift2 + 2]);
            tangentU = new Vector3D(pntDeriv[shift3], pntDeriv[shift3 + 1], pntDeriv[shift3 + 2]);
            curvatureUV = new Vector3D(pntDeriv[shift4], pntDeriv[shift4 + 1], pntDeriv[shift4 + 2]);
            curvatureU = new Vector3D(pntDeriv[shift6], pntDeriv[shift6 + 1], pntDeriv[shift6 + 2]);
        }
        else
        {
            // If Max=V (V degree >= U degree):
            // [0]=P, [Dim]=DU, [2Dim]=DUU
            // [3Dim]=DV, [4Dim]=DUV (DVU is symmetric)
            // [6Dim]=DVV
            tangentU = new Vector3D(pntDeriv[shift], pntDeriv[shift + 1], pntDeriv[shift + 2]);
            curvatureU = new Vector3D(pntDeriv[shift2], pntDeriv[shift2 + 1], pntDeriv[shift2 + 2]);
            tangentV = new Vector3D(pntDeriv[shift3], pntDeriv[shift3 + 1], pntDeriv[shift3 + 2]);
            curvatureUV = new Vector3D(pntDeriv[shift4], pntDeriv[shift4 + 1], pntDeriv[shift4 + 2]);
            curvatureV = new Vector3D(pntDeriv[shift6], pntDeriv[shift6 + 1], pntDeriv[shift6 + 2]);
        }

        // Use direct division for better numerical stability with very small span lengths
        double spanLengthU = 0.5 * _paramsU.SpanLength;
        double spanLengthV = 0.5 * _paramsV.SpanLength;
        double spanLengthU2 = spanLengthU * spanLengthU;
        double spanLengthV2 = spanLengthV * spanLengthV;
        tangentU = tangentU.Divided(spanLengthU);
        tangentV = tangentV.Divided(spanLengthV);
        curvatureU = curvatureU.Divided(spanLengthU2);
        curvatureV = curvatureV.Divided(spanLengthV2);
        curvatureUV = curvatureUV.Divided(spanLengthU * spanLengthV);
    }

    /// <summary>The point and first derivatives of the surface at (<paramref name="u"/>, <paramref name="v"/>).</summary>
    public void D1(double u, double v, out Point3D point, out Vector3D tangentU, out Vector3D tangentV)
    {
        // Use the same inverse values for both parameter transformation and derivative scaling
        // to maintain numerical consistency with the original implementation
        var (localU, localV) = ToLocalParams(u, v, _paramsU, _paramsV, out double invU, out double invV);

        Span<double> pntDeriv = stackalloc double[16];
        EvaluatePolynomials(_polesWeights,
                            _paramsU,
                            _paramsV,
                            _isRational,
                            localU,
                            localV,
                            1,
                            1,
                            pntDeriv);

        const int dimension = 3;
        point = new Point3D(pntDeriv[0], pntDeriv[1], pntDeriv[2]);

        if (_paramsU.Degree > _paramsV.Degree)
        {
            tangentV = new Vector3D(pntDeriv[dimension],
                                     pntDeriv[dimension + 1],
                                     pntDeriv[dimension + 2]);
            tangentU = new Vector3D(pntDeriv[dimension << 1],
                                     pntDeriv[(dimension << 1) + 1],
                                     pntDeriv[(dimension << 1) + 2]);
        }
        else
        {
            tangentU = new Vector3D(pntDeriv[dimension],
                                     pntDeriv[dimension + 1],
                                     pntDeriv[dimension + 2]);
            tangentV = new Vector3D(pntDeriv[dimension << 1],
                                     pntDeriv[(dimension << 1) + 1],
                                     pntDeriv[(dimension << 1) + 2]);
        }

        // Scale derivatives using the same inverse values used for parameter transformation
        tangentU = tangentU.Multiplied(invU);
        tangentV = tangentV.Multiplied(invV);
    }

    /// <summary>
    /// The point and derivatives up to the second order of the surface at (<paramref name="u"/>,
    /// <paramref name="v"/>).
    /// </summary>
    public void D2(double u, double v, out Point3D point, out Vector3D tangentU, out Vector3D tangentV,
                   out Vector3D curvatureU, out Vector3D curvatureV, out Vector3D curvatureUV)
    {
        // Use the same inverse values for both parameter transformation and derivative scaling
        // to maintain numerical consistency with the original implementation
        var (localU, localV) = ToLocalParams(u, v, _paramsU, _paramsV, out double invU, out double invV);

        Span<double> pntDeriv = stackalloc double[36];
        EvaluatePolynomials(_polesWeights,
                            _paramsU,
                            _paramsV,
                            _isRational,
                            localU,
                            localV,
                            2,
                            2,
                            pntDeriv);

        const int dimension = 3;
        const int shift = dimension;
        const int shift2 = dimension << 1;
        const int shift3 = shift2 + dimension;
        const int shift4 = shift3 + dimension;
        const int shift6 = 6 * dimension;

        point = new Point3D(pntDeriv[0], pntDeriv[1], pntDeriv[2]);

        if (_paramsU.Degree > _paramsV.Degree)
        {
            tangentV = new Vector3D(pntDeriv[shift], pntDeriv[shift + 1], pntDeriv[shift + 2]);
            curvatureV = new Vector3D(pntDeriv[shift2], pntDeriv[shift2 + 1], pntDeriv[shift2 + 2]);
            tangentU = new Vector3D(pntDeriv[shift3], pntDeriv[shift3 + 1], pntDeriv[shift3 + 2]);
            curvatureUV = new Vector3D(pntDeriv[shift4], pntDeriv[shift4 + 1], pntDeriv[shift4 + 2]);
            curvatureU = new Vector3D(pntDeriv[shift6], pntDeriv[shift6 + 1], pntDeriv[shift6 + 2]);
        }
        else
        {
            tangentU = new Vector3D(pntDeriv[shift], pntDeriv[shift + 1], pntDeriv[shift + 2]);
            curvatureU = new Vector3D(pntDeriv[shift2], pntDeriv[shift2 + 1], pntDeriv[shift2 + 2]);
            tangentV = new Vector3D(pntDeriv[shift3], pntDeriv[shift3 + 1], pntDeriv[shift3 + 2]);
            curvatureUV = new Vector3D(pntDeriv[shift4], pntDeriv[shift4 + 1], pntDeriv[shift4 + 2]);
            curvatureV = new Vector3D(pntDeriv[shift6], pntDeriv[shift6 + 1], pntDeriv[shift6 + 2]);
        }

        // Scale derivatives using the same inverse values used for parameter transformation
        tangentU = tangentU.Multiplied(invU);
        tangentV = tangentV.Multiplied(invV);
        curvatureU = curvatureU.Multiplied(invU * invU);
        curvatureV = curvatureV.Multiplied(invV * invV);
        curvatureUV = curvatureUV.Multiplied(invU * invV);
    }

    //=================================================================================================

    // Computes local UV parameters for D0 evaluation (no derivative scaling needed).
    // BSplSLib uses different convention for span parameters than BSplCLib
    // (Start is in the middle of the span and length is half-span).
    private static (double, double) ToLocalParamsD0(double u,
                                                    double v,
                                                    CacheParams paramsU,
                                                    CacheParams paramsV)
    {
        double newU = paramsU.PeriodicNormalization(u);
        double newV = paramsV.PeriodicNormalization(v);
        double spanLengthU = 0.5 * paramsU.SpanLength;
        double spanStartU = paramsU.SpanStart + spanLengthU;
        double spanLengthV = 0.5 * paramsV.SpanLength;
        double spanStartV = paramsV.SpanStart + spanLengthV;
        return ((newU - spanStartU) / spanLengthU, (newV - spanStartV) / spanLengthV);
    }

    // Computes local UV parameters and inverse span lengths for derivative evaluation.
    // The same inverse values are used for both parameter transformation and derivative scaling
    // to maintain numerical consistency with the original implementation.
    private static (double, double) ToLocalParams(double u,
                                                  double v,
                                                  CacheParams paramsU,
                                                  CacheParams paramsV,
                                                  out double invU,
                                                  out double invV)
    {
        double newU = paramsU.PeriodicNormalization(u);
        double newV = paramsV.PeriodicNormalization(v);
        double spanLengthU = 0.5 * paramsU.SpanLength;
        double spanStartU = paramsU.SpanStart + spanLengthU;
        double spanLengthV = 0.5 * paramsV.SpanLength;
        double spanStartV = paramsV.SpanStart + spanLengthV;
        // Compute inverses once and reuse for both parameter transformation and derivative scaling
        // to maintain numerical consistency with the original implementation
        invU = 1.0 / spanLengthU;
        invV = 1.0 / spanLengthV;
        return ((newU - spanStartU) * invU, (newV - spanStartV) * invV);
    }

    // Evaluates the polynomials and their derivatives up to (theUDerivMax, theVDerivMax) at the local parameters
    // into theResultArray.
    private static void EvaluatePolynomials(NCollection_Array2<double> polesWeights,
                                            CacheParams paramsU,
                                            CacheParams paramsV,
                                            bool isRational,
                                            double localU,
                                            double localV,
                                            int uDerivMax,
                                            int vDerivMax,
                                            Span<double> resultArray)
    {
        ReadOnlySpan<double> polesArray = polesWeights.AsSpan();
        int dimension = isRational ? 4 : 3;
        int cacheCols = polesWeights.RowLength();

        bool isMaxU = (paramsU.Degree > paramsV.Degree);
        var (minParam, maxParam) = isMaxU ? (localV, localU) : (localU, localV);

        // Determine derivatives to calculate along each direction
        int maxDeriv = isMaxU ? uDerivMax : vDerivMax;
        int minDeriv = isMaxU ? vDerivMax : uDerivMax;

        // Stride between rows in the result array (corresponds to one step in MaxParam direction)
        // For full grid (required by RationalDerivative), stride is (aMinDeriv + 1) points.
        int rowStride = (minDeriv + 1) * dimension;

        // Transient coefficients array size:
        // (aMaxDeriv + 1) * CacheCols  for the first evaluation (along max degree parameter)
        // (aMinDeriv + 1) * Dimension for the second evaluation (along min degree parameter)
        int transientSize = Max((maxDeriv + 1) * cacheCols, (minDeriv + 1) * dimension);
        Span<double> transientCoeffs = transientSize <= 128 ? stackalloc double[128] : new double[transientSize];

        // Calculate intermediate values and derivatives of bivariate polynomial along variable with
        // maximal degree
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(maxParam,
                            maxDeriv,
                            isMaxU ? paramsU.Degree : paramsV.Degree,
                            cacheCols,
                            polesArray,
                            transientCoeffs);

        // Block 0: Evaluate derivatives along variable with minimal degree for D0_max
        // Produces (0,0), (0,1)...(0, aMinDeriv)
        // Writes to offset 0
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(minParam,
                            minDeriv,
                            isMaxU ? paramsV.Degree : paramsU.Degree,
                            dimension,
                            transientCoeffs,
                            resultArray);

        if (maxDeriv > 0)
        {
            // Block 1: Evaluate derivatives along variable with minimal degree for D1_max
            // Writes to offset aRowStride (start of second row)
            // If Rational, we need full row (up to aMinDeriv).
            // If Not Rational, we can optimize: we strictly need (1,0) and (1,1).
            // D1Local calls with (1,1) -> aMinDeriv=1. We need up to 1.
            // D2Local calls with (2,2) -> aMinDeriv=2. We need up to 1 for mixed D2.
            // So usually min(aMinDeriv, 1) is sufficient for non-rational.
            int deriv = isRational ? minDeriv : Min(minDeriv, 1);

            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(minParam,
                                deriv,
                                isMaxU ? paramsV.Degree : paramsU.Degree,
                                dimension,
                                transientCoeffs[cacheCols..],
                                resultArray[rowStride..]);

            if (maxDeriv > 1)
            {
                // Block 2: Evaluate derivatives along variable with minimal degree for D2_max
                // Writes to offset 2 * aRowStride (start of third row)
                // If Rational, full row.
                // If Not Rational, we only need (2,0) for standard D2.
                int deriv2 = isRational ? minDeriv : 0;

                if (deriv2 == 0)
                {
                    global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(minParam,
                                                    isMaxU ? paramsV.Degree : paramsU.Degree,
                                                    dimension,
                                                    (isMaxU ? paramsV.Degree : paramsU.Degree)
                                                      * dimension,
                                                    transientCoeffs[(cacheCols << 1)..],
                                                    resultArray[(rowStride << 1)..]);
                }
                else
                {
                    global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(minParam,
                                        deriv2,
                                        isMaxU ? paramsV.Degree : paramsU.Degree,
                                        dimension,
                                        transientCoeffs[(cacheCols << 1)..],
                                        resultArray[(rowStride << 1)..]);
                }
            }
        }

        if (isRational)
        {
            // RationalDerivative is NOT safe for in-place operation because it reads 4-component data
            // and writes 3-component data to potentially overlapping memory locations.
            // We need a separate temporary storage for the output (at most 3 x 3 x 3 = 27 values here).
            int resultSize = (uDerivMax + 1) * (vDerivMax + 1) * 3;
            Span<double> tempStorage = stackalloc double[resultSize];

            if (isMaxU)
            {
                SurfaceBSplSLib.RationalDerivative(uDerivMax,
                                            vDerivMax,
                                            uDerivMax,
                                            vDerivMax,
                                            resultArray,
                                            tempStorage);
            }
            else
            {
                // If V is max degree, our result array is V-major (transposed relative to what
                // RationalDerivative expects) We swap U/V arguments to trick RationalDerivative into
                // processing it correctly.
                SurfaceBSplSLib.RationalDerivative(vDerivMax,
                                            uDerivMax,
                                            vDerivMax,
                                            uDerivMax,
                                            resultArray,
                                            tempStorage);
            }

            // Copy results back to the output array
            for (int i = 0; i < resultSize; ++i)
            {
                resultArray[i] = tempStorage[i];
            }
        }
    }
}
