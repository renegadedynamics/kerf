// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib_Cache.hxx
// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib_Cache.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class BSplSLib_Cache : Standard_Transient
{
    private readonly bool myIsRational;                                 //!< identifies the rationality of Bezier/B-spline surface
    private readonly BSplCLib_CacheParams myParamsU, myParamsV;         //!< cache parameters by U and V directions
    // array of poles and weights of calculated cache
    // the array has following structure:
    //       x11 y11 z11 [w11] x12 y12 z12 [w12] ...
    //       x21 y21 z21 [w21] x22 y22 z22 [w22] etc
    // for non-rational surfaces there is no weight;
    // size of array: (max(myDegree)+1) * A*(min(myDegree)+1), where A = 4 or 3
    private readonly NCollection_Array2<double> myPolesWeights;

    /// <summary>
    /// Prepares the cache of a surface of the given degrees, periodicity and flat knots (knots with repetitions)
    /// along U and V; <paramref name="theWeights"/> only says whether the surface is rational.
    /// </summary>
    public BSplSLib_Cache(int theDegreeU,
                          bool thePeriodicU,
                          NCollection_Array1<double> theFlatKnotsU,
                          int theDegreeV,
                          bool thePeriodicV,
                          NCollection_Array1<double> theFlatKnotsV,
                          NCollection_Array2<double>? theWeights = null)
    {
        myIsRational = theWeights != null;
        myParamsU = new BSplCLib_CacheParams(theDegreeU, thePeriodicU, theFlatKnotsU);
        myParamsV = new BSplCLib_CacheParams(theDegreeV, thePeriodicV, theFlatKnotsV);
        int aMinDegree = Min(theDegreeU, theDegreeV);
        int aMaxDegree = Max(theDegreeU, theDegreeV);
        int aPWColNumber = (myIsRational ? 4 : 3);
        myPolesWeights = new NCollection_Array2<double>(1, aMaxDegree + 1, 1, aPWColNumber * (aMinDegree + 1));
    }

    /// <summary>True if the cached span is the one of the point (<paramref name="theParameterU"/>, <paramref name="theParameterV"/>).</summary>
    public bool IsCacheValid(double theParameterU, double theParameterV)
    {
        return myParamsU.IsCacheValid(theParameterU) && myParamsV.IsCacheValid(theParameterV);
    }

    /// <summary>
    /// Recomputes the cache for the span of (<paramref name="theParameterU"/>, <paramref name="theParameterV"/>),
    /// without checking whether it is valid.
    /// </summary>
    internal void BuildCache(double theParameterU,
                             double theParameterV,
                             NCollection_Array1<double> theFlatKnotsU,
                             NCollection_Array1<double> theFlatKnotsV,
                             NCollection_Array2<gp_Pnt> thePoles,
                             NCollection_Array2<double>? theWeights = null)
    {
        // Normalize the parameters for periodical B-splines
        double aNewParamU = myParamsU.PeriodicNormalization(theParameterU);
        double aNewParamV = myParamsV.PeriodicNormalization(theParameterV);

        myParamsU.LocateParameter(ref aNewParamU, theFlatKnotsU);
        myParamsV.LocateParameter(ref aNewParamV, theFlatKnotsV);

        // BSplSLib uses different convention for span parameters than BSplCLib
        // (Start is in the middle of the span and length is half-span),
        // thus we need to amend them here
        double aSpanLengthU = 0.5 * myParamsU.SpanLength;
        double aSpanStartU = myParamsU.SpanStart + aSpanLengthU;
        double aSpanLengthV = 0.5 * myParamsV.SpanLength;
        double aSpanStartV = myParamsV.SpanStart + aSpanLengthV;

        // Calculate new cache data
        BSplSLib.BuildCache(aSpanStartU,
                            aSpanStartV,
                            aSpanLengthU,
                            aSpanLengthV,
                            myParamsU.IsPeriodic,
                            myParamsV.IsPeriodic,
                            myParamsU.Degree,
                            myParamsV.Degree,
                            myParamsU.SpanIndex,
                            myParamsV.SpanIndex,
                            theFlatKnotsU,
                            theFlatKnotsV,
                            thePoles,
                            theWeights,
                            myPolesWeights);
    }

    /// <summary>The point of the surface at (<paramref name="theU"/>, <paramref name="theV"/>).</summary>
    public void D0(double theU, double theV, out gp_Pnt thePoint)
    {
        var (aLocalU, aLocalV) = toLocalParamsD0(theU, theV, myParamsU, myParamsV);
        D0Local(aLocalU, aLocalV, out thePoint);
    }

    /// <summary>
    /// The point at the local parameters in [-1, 1], (U - SpanMid) / SpanHalfLen and the same for V; skips the
    /// periodic normalization and the local parameter computation.
    /// </summary>
    public void D0Local(double theLocalU, double theLocalV, out gp_Pnt thePoint)
    {
        ReadOnlySpan<double> aPolesArray = myPolesWeights.AsSpan();
        Span<double> aPoint = stackalloc double[4];

        int aDimension = myIsRational ? 4 : 3;
        int aCacheCols = myPolesWeights.RowLength();
        bool isMaxU = (myParamsU.Degree > myParamsV.Degree);
        int aMinDegree = Min(myParamsU.Degree, myParamsV.Degree);
        int aMaxDegree = Max(myParamsU.Degree, myParamsV.Degree);
        var (aMinParam, aMaxParam) = isMaxU ? (theLocalV, theLocalU) : (theLocalU, theLocalV);

        // Array for intermediate results (at most 4 * 26 = 104 values)
        Span<double> aTransientCoeffs = stackalloc double[aCacheCols];

        // Calculate intermediate value of cached polynomial along variable with maximal degree
        PLib.NoDerivativeEvalPolynomial(aMaxParam,
                                        aMaxDegree,
                                        aCacheCols,
                                        aMaxDegree * aCacheCols,
                                        aPolesArray,
                                        aTransientCoeffs);

        // Calculate total value along variable with minimal degree
        PLib.NoDerivativeEvalPolynomial(aMinParam,
                                        aMinDegree,
                                        aDimension,
                                        aDimension * aMinDegree,
                                        aTransientCoeffs,
                                        aPoint);

        thePoint = new gp_Pnt(aPoint[0], aPoint[1], aPoint[2]);
        if (myIsRational)
        {
            thePoint.ChangeCoord().Divide(aPoint[3]);
        }
    }

    /// <summary>The point and first derivatives at the local parameters in [-1, 1] (see <see cref="D0Local"/>).</summary>
    public void D1Local(double theLocalU, double theLocalV, out gp_Pnt thePoint, out gp_Vec theTangentU,
                        out gp_Vec theTangentV)
    {
        Span<double> aPntDeriv = stackalloc double[16]; // Result storage for D1, zero-initialized
        EvaluatePolynomials(myPolesWeights,
                            myParamsU,
                            myParamsV,
                            myIsRational,
                            theLocalU,
                            theLocalV,
                            1,
                            1,
                            aPntDeriv);

        // After RationalDerivative (for rational surfaces), the output dimension is 3 (not 4)
        // because weights have been processed out
        const int aDimension = 3;

        thePoint = new gp_Pnt(aPntDeriv[0], aPntDeriv[1], aPntDeriv[2]);

        // Tangents are stored after the point coordinates
        // Order depends on which parameter has higher degree
        // If U degree > V degree: layout is [P, DV, DU, ...]
        // If V degree >= U degree: layout is [P, DU, DV, ...]
        if (myParamsU.Degree > myParamsV.Degree)
        {
            theTangentV = new gp_Vec(aPntDeriv[aDimension],
                                     aPntDeriv[aDimension + 1],
                                     aPntDeriv[aDimension + 2]);
            theTangentU = new gp_Vec(aPntDeriv[aDimension << 1],
                                     aPntDeriv[(aDimension << 1) + 1],
                                     aPntDeriv[(aDimension << 1) + 2]);
        }
        else
        {
            theTangentU = new gp_Vec(aPntDeriv[aDimension],
                                     aPntDeriv[aDimension + 1],
                                     aPntDeriv[aDimension + 2]);
            theTangentV = new gp_Vec(aPntDeriv[aDimension << 1],
                                     aPntDeriv[(aDimension << 1) + 1],
                                     aPntDeriv[(aDimension << 1) + 2]);
        }

        // Use direct division for better numerical stability with very small span lengths
        double aSpanLengthU = 0.5 * myParamsU.SpanLength;
        double aSpanLengthV = 0.5 * myParamsV.SpanLength;
        theTangentU.Divide(aSpanLengthU);
        theTangentV.Divide(aSpanLengthV);
    }

    /// <summary>
    /// The point and derivatives up to the second order at the local parameters in [-1, 1] (see
    /// <see cref="D0Local"/>).
    /// </summary>
    public void D2Local(double theLocalU, double theLocalV, out gp_Pnt thePoint, out gp_Vec theTangentU,
                        out gp_Vec theTangentV, out gp_Vec theCurvatureU, out gp_Vec theCurvatureV,
                        out gp_Vec theCurvatureUV)
    {
        Span<double> aPntDeriv = stackalloc double[36]; // Result storage for D2, zero-initialized
        EvaluatePolynomials(myPolesWeights,
                            myParamsU,
                            myParamsV,
                            myIsRational,
                            theLocalU,
                            theLocalV,
                            2,
                            2,
                            aPntDeriv);

        // After RationalDerivative (for rational surfaces), the output dimension is 3 (not 4)
        // because weights have been processed out
        const int aDimension = 3;
        const int aShift = aDimension; // Shift for first derivatives
        const int aShift2 = aDimension << 1;
        const int aShift3 = aShift2 + aDimension;
        const int aShift4 = aShift3 + aDimension;
        const int aShift6 = 6 * aDimension;

        thePoint = new gp_Pnt(aPntDeriv[0], aPntDeriv[1], aPntDeriv[2]);

        // Derivatives are stored consecutively
        // If Max=U (U degree > V degree):
        // [0]=P, [Dim]=DV, [2Dim]=DVV
        // [3Dim]=DU, [4Dim]=DUV
        // [6Dim]=DUU
        if (myParamsU.Degree > myParamsV.Degree)
        {
            theTangentV = new gp_Vec(aPntDeriv[aShift], aPntDeriv[aShift + 1], aPntDeriv[aShift + 2]);
            theCurvatureV = new gp_Vec(aPntDeriv[aShift2], aPntDeriv[aShift2 + 1], aPntDeriv[aShift2 + 2]);
            theTangentU = new gp_Vec(aPntDeriv[aShift3], aPntDeriv[aShift3 + 1], aPntDeriv[aShift3 + 2]);
            theCurvatureUV = new gp_Vec(aPntDeriv[aShift4], aPntDeriv[aShift4 + 1], aPntDeriv[aShift4 + 2]);
            theCurvatureU = new gp_Vec(aPntDeriv[aShift6], aPntDeriv[aShift6 + 1], aPntDeriv[aShift6 + 2]);
        }
        else
        {
            // If Max=V (V degree >= U degree):
            // [0]=P, [Dim]=DU, [2Dim]=DUU
            // [3Dim]=DV, [4Dim]=DUV (DVU is symmetric)
            // [6Dim]=DVV
            theTangentU = new gp_Vec(aPntDeriv[aShift], aPntDeriv[aShift + 1], aPntDeriv[aShift + 2]);
            theCurvatureU = new gp_Vec(aPntDeriv[aShift2], aPntDeriv[aShift2 + 1], aPntDeriv[aShift2 + 2]);
            theTangentV = new gp_Vec(aPntDeriv[aShift3], aPntDeriv[aShift3 + 1], aPntDeriv[aShift3 + 2]);
            theCurvatureUV = new gp_Vec(aPntDeriv[aShift4], aPntDeriv[aShift4 + 1], aPntDeriv[aShift4 + 2]);
            theCurvatureV = new gp_Vec(aPntDeriv[aShift6], aPntDeriv[aShift6 + 1], aPntDeriv[aShift6 + 2]);
        }

        // Use direct division for better numerical stability with very small span lengths
        double aSpanLengthU = 0.5 * myParamsU.SpanLength;
        double aSpanLengthV = 0.5 * myParamsV.SpanLength;
        double aSpanLengthU2 = aSpanLengthU * aSpanLengthU;
        double aSpanLengthV2 = aSpanLengthV * aSpanLengthV;
        theTangentU.Divide(aSpanLengthU);
        theTangentV.Divide(aSpanLengthV);
        theCurvatureU.Divide(aSpanLengthU2);
        theCurvatureV.Divide(aSpanLengthV2);
        theCurvatureUV.Divide(aSpanLengthU * aSpanLengthV);
    }

    /// <summary>The point and first derivatives of the surface at (<paramref name="theU"/>, <paramref name="theV"/>).</summary>
    public void D1(double theU, double theV, out gp_Pnt thePoint, out gp_Vec theTangentU, out gp_Vec theTangentV)
    {
        // Use the same inverse values for both parameter transformation and derivative scaling
        // to maintain numerical consistency with the original implementation
        var (aLocalU, aLocalV) = toLocalParams(theU, theV, myParamsU, myParamsV, out double anInvU, out double anInvV);

        Span<double> aPntDeriv = stackalloc double[16];
        EvaluatePolynomials(myPolesWeights,
                            myParamsU,
                            myParamsV,
                            myIsRational,
                            aLocalU,
                            aLocalV,
                            1,
                            1,
                            aPntDeriv);

        const int aDimension = 3;
        thePoint = new gp_Pnt(aPntDeriv[0], aPntDeriv[1], aPntDeriv[2]);

        if (myParamsU.Degree > myParamsV.Degree)
        {
            theTangentV = new gp_Vec(aPntDeriv[aDimension],
                                     aPntDeriv[aDimension + 1],
                                     aPntDeriv[aDimension + 2]);
            theTangentU = new gp_Vec(aPntDeriv[aDimension << 1],
                                     aPntDeriv[(aDimension << 1) + 1],
                                     aPntDeriv[(aDimension << 1) + 2]);
        }
        else
        {
            theTangentU = new gp_Vec(aPntDeriv[aDimension],
                                     aPntDeriv[aDimension + 1],
                                     aPntDeriv[aDimension + 2]);
            theTangentV = new gp_Vec(aPntDeriv[aDimension << 1],
                                     aPntDeriv[(aDimension << 1) + 1],
                                     aPntDeriv[(aDimension << 1) + 2]);
        }

        // Scale derivatives using the same inverse values used for parameter transformation
        theTangentU.Multiply(anInvU);
        theTangentV.Multiply(anInvV);
    }

    /// <summary>
    /// The point and derivatives up to the second order of the surface at (<paramref name="theU"/>,
    /// <paramref name="theV"/>).
    /// </summary>
    public void D2(double theU, double theV, out gp_Pnt thePoint, out gp_Vec theTangentU, out gp_Vec theTangentV,
                   out gp_Vec theCurvatureU, out gp_Vec theCurvatureV, out gp_Vec theCurvatureUV)
    {
        // Use the same inverse values for both parameter transformation and derivative scaling
        // to maintain numerical consistency with the original implementation
        var (aLocalU, aLocalV) = toLocalParams(theU, theV, myParamsU, myParamsV, out double anInvU, out double anInvV);

        Span<double> aPntDeriv = stackalloc double[36];
        EvaluatePolynomials(myPolesWeights,
                            myParamsU,
                            myParamsV,
                            myIsRational,
                            aLocalU,
                            aLocalV,
                            2,
                            2,
                            aPntDeriv);

        const int aDimension = 3;
        const int aShift = aDimension;
        const int aShift2 = aDimension << 1;
        const int aShift3 = aShift2 + aDimension;
        const int aShift4 = aShift3 + aDimension;
        const int aShift6 = 6 * aDimension;

        thePoint = new gp_Pnt(aPntDeriv[0], aPntDeriv[1], aPntDeriv[2]);

        if (myParamsU.Degree > myParamsV.Degree)
        {
            theTangentV = new gp_Vec(aPntDeriv[aShift], aPntDeriv[aShift + 1], aPntDeriv[aShift + 2]);
            theCurvatureV = new gp_Vec(aPntDeriv[aShift2], aPntDeriv[aShift2 + 1], aPntDeriv[aShift2 + 2]);
            theTangentU = new gp_Vec(aPntDeriv[aShift3], aPntDeriv[aShift3 + 1], aPntDeriv[aShift3 + 2]);
            theCurvatureUV = new gp_Vec(aPntDeriv[aShift4], aPntDeriv[aShift4 + 1], aPntDeriv[aShift4 + 2]);
            theCurvatureU = new gp_Vec(aPntDeriv[aShift6], aPntDeriv[aShift6 + 1], aPntDeriv[aShift6 + 2]);
        }
        else
        {
            theTangentU = new gp_Vec(aPntDeriv[aShift], aPntDeriv[aShift + 1], aPntDeriv[aShift + 2]);
            theCurvatureU = new gp_Vec(aPntDeriv[aShift2], aPntDeriv[aShift2 + 1], aPntDeriv[aShift2 + 2]);
            theTangentV = new gp_Vec(aPntDeriv[aShift3], aPntDeriv[aShift3 + 1], aPntDeriv[aShift3 + 2]);
            theCurvatureUV = new gp_Vec(aPntDeriv[aShift4], aPntDeriv[aShift4 + 1], aPntDeriv[aShift4 + 2]);
            theCurvatureV = new gp_Vec(aPntDeriv[aShift6], aPntDeriv[aShift6 + 1], aPntDeriv[aShift6 + 2]);
        }

        // Scale derivatives using the same inverse values used for parameter transformation
        theTangentU.Multiply(anInvU);
        theTangentV.Multiply(anInvV);
        theCurvatureU.Multiply(anInvU * anInvU);
        theCurvatureV.Multiply(anInvV * anInvV);
        theCurvatureUV.Multiply(anInvU * anInvV);
    }

    //=================================================================================================

    // Computes local UV parameters for D0 evaluation (no derivative scaling needed).
    // BSplSLib uses different convention for span parameters than BSplCLib
    // (Start is in the middle of the span and length is half-span).
    private static (double, double) toLocalParamsD0(double theU,
                                                    double theV,
                                                    BSplCLib_CacheParams theParamsU,
                                                    BSplCLib_CacheParams theParamsV)
    {
        double aNewU = theParamsU.PeriodicNormalization(theU);
        double aNewV = theParamsV.PeriodicNormalization(theV);
        double aSpanLengthU = 0.5 * theParamsU.SpanLength;
        double aSpanStartU = theParamsU.SpanStart + aSpanLengthU;
        double aSpanLengthV = 0.5 * theParamsV.SpanLength;
        double aSpanStartV = theParamsV.SpanStart + aSpanLengthV;
        return ((aNewU - aSpanStartU) / aSpanLengthU, (aNewV - aSpanStartV) / aSpanLengthV);
    }

    // Computes local UV parameters and inverse span lengths for derivative evaluation.
    // The same inverse values are used for both parameter transformation and derivative scaling
    // to maintain numerical consistency with the original implementation.
    private static (double, double) toLocalParams(double theU,
                                                  double theV,
                                                  BSplCLib_CacheParams theParamsU,
                                                  BSplCLib_CacheParams theParamsV,
                                                  out double theInvU,
                                                  out double theInvV)
    {
        double aNewU = theParamsU.PeriodicNormalization(theU);
        double aNewV = theParamsV.PeriodicNormalization(theV);
        double aSpanLengthU = 0.5 * theParamsU.SpanLength;
        double aSpanStartU = theParamsU.SpanStart + aSpanLengthU;
        double aSpanLengthV = 0.5 * theParamsV.SpanLength;
        double aSpanStartV = theParamsV.SpanStart + aSpanLengthV;
        // Compute inverses once and reuse for both parameter transformation and derivative scaling
        // to maintain numerical consistency with the original implementation
        theInvU = 1.0 / aSpanLengthU;
        theInvV = 1.0 / aSpanLengthV;
        return ((aNewU - aSpanStartU) * theInvU, (aNewV - aSpanStartV) * theInvV);
    }

    // Evaluates the polynomials and their derivatives up to (theUDerivMax, theVDerivMax) at the local parameters
    // into theResultArray.
    private static void EvaluatePolynomials(NCollection_Array2<double> thePolesWeights,
                                            BSplCLib_CacheParams theParamsU,
                                            BSplCLib_CacheParams theParamsV,
                                            bool theIsRational,
                                            double theLocalU,
                                            double theLocalV,
                                            int theUDerivMax,
                                            int theVDerivMax,
                                            Span<double> theResultArray)
    {
        ReadOnlySpan<double> aPolesArray = thePolesWeights.AsSpan();
        int aDimension = theIsRational ? 4 : 3;
        int aCacheCols = thePolesWeights.RowLength();

        bool isMaxU = (theParamsU.Degree > theParamsV.Degree);
        var (aMinParam, aMaxParam) = isMaxU ? (theLocalV, theLocalU) : (theLocalU, theLocalV);

        // Determine derivatives to calculate along each direction
        int aMaxDeriv = isMaxU ? theUDerivMax : theVDerivMax;
        int aMinDeriv = isMaxU ? theVDerivMax : theUDerivMax;

        // Stride between rows in the result array (corresponds to one step in MaxParam direction)
        // For full grid (required by RationalDerivative), stride is (aMinDeriv + 1) points.
        int aRowStride = (aMinDeriv + 1) * aDimension;

        // Transient coefficients array size:
        // (aMaxDeriv + 1) * CacheCols  for the first evaluation (along max degree parameter)
        // (aMinDeriv + 1) * Dimension for the second evaluation (along min degree parameter)
        int aTransientSize = Max((aMaxDeriv + 1) * aCacheCols, (aMinDeriv + 1) * aDimension);
        Span<double> aTransientCoeffs = aTransientSize <= 128 ? stackalloc double[128] : new double[aTransientSize];

        // Calculate intermediate values and derivatives of bivariate polynomial along variable with
        // maximal degree
        PLib.EvalPolynomial(aMaxParam,
                            aMaxDeriv,
                            isMaxU ? theParamsU.Degree : theParamsV.Degree,
                            aCacheCols,
                            aPolesArray,
                            aTransientCoeffs);

        // Block 0: Evaluate derivatives along variable with minimal degree for D0_max
        // Produces (0,0), (0,1)...(0, aMinDeriv)
        // Writes to offset 0
        PLib.EvalPolynomial(aMinParam,
                            aMinDeriv,
                            isMaxU ? theParamsV.Degree : theParamsU.Degree,
                            aDimension,
                            aTransientCoeffs,
                            theResultArray);

        if (aMaxDeriv > 0)
        {
            // Block 1: Evaluate derivatives along variable with minimal degree for D1_max
            // Writes to offset aRowStride (start of second row)
            // If Rational, we need full row (up to aMinDeriv).
            // If Not Rational, we can optimize: we strictly need (1,0) and (1,1).
            // D1Local calls with (1,1) -> aMinDeriv=1. We need up to 1.
            // D2Local calls with (2,2) -> aMinDeriv=2. We need up to 1 for mixed D2.
            // So usually min(aMinDeriv, 1) is sufficient for non-rational.
            int aDeriv = theIsRational ? aMinDeriv : Min(aMinDeriv, 1);

            PLib.EvalPolynomial(aMinParam,
                                aDeriv,
                                isMaxU ? theParamsV.Degree : theParamsU.Degree,
                                aDimension,
                                aTransientCoeffs[aCacheCols..],
                                theResultArray[aRowStride..]);

            if (aMaxDeriv > 1)
            {
                // Block 2: Evaluate derivatives along variable with minimal degree for D2_max
                // Writes to offset 2 * aRowStride (start of third row)
                // If Rational, full row.
                // If Not Rational, we only need (2,0) for standard D2.
                int aDeriv2 = theIsRational ? aMinDeriv : 0;

                if (aDeriv2 == 0)
                {
                    PLib.NoDerivativeEvalPolynomial(aMinParam,
                                                    isMaxU ? theParamsV.Degree : theParamsU.Degree,
                                                    aDimension,
                                                    (isMaxU ? theParamsV.Degree : theParamsU.Degree)
                                                      * aDimension,
                                                    aTransientCoeffs[(aCacheCols << 1)..],
                                                    theResultArray[(aRowStride << 1)..]);
                }
                else
                {
                    PLib.EvalPolynomial(aMinParam,
                                        aDeriv2,
                                        isMaxU ? theParamsV.Degree : theParamsU.Degree,
                                        aDimension,
                                        aTransientCoeffs[(aCacheCols << 1)..],
                                        theResultArray[(aRowStride << 1)..]);
                }
            }
        }

        if (theIsRational)
        {
            // RationalDerivative is NOT safe for in-place operation because it reads 4-component data
            // and writes 3-component data to potentially overlapping memory locations.
            // We need a separate temporary storage for the output (at most 3 x 3 x 3 = 27 values here).
            int aResultSize = (theUDerivMax + 1) * (theVDerivMax + 1) * 3;
            Span<double> aTempStorage = stackalloc double[aResultSize];

            if (isMaxU)
            {
                BSplSLib.RationalDerivative(theUDerivMax,
                                            theVDerivMax,
                                            theUDerivMax,
                                            theVDerivMax,
                                            theResultArray,
                                            aTempStorage);
            }
            else
            {
                // If V is max degree, our result array is V-major (transposed relative to what
                // RationalDerivative expects) We swap U/V arguments to trick RationalDerivative into
                // processing it correctly.
                BSplSLib.RationalDerivative(theVDerivMax,
                                            theUDerivMax,
                                            theVDerivMax,
                                            theUDerivMax,
                                            theResultArray,
                                            aTempStorage);
            }

            // Copy results back to the output array
            for (int i = 0; i < aResultSize; ++i)
            {
                theResultArray[i] = aTempStorage[i];
            }
        }
    }
}
