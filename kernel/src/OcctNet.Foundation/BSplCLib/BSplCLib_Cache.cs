// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_Cache.hxx
// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_Cache.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A cache of the polynomial coefficients of one span of a Bezier or B-spline curve (2D or 3D), for fast
/// evaluation. The data must be rebuilt (<see cref="BuildCache(double, NCollection_Array1{double}, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/>)
/// when going from span to span (<see cref="IsCacheValid"/>).
/// </summary>
/// <remarks>
/// Mutable: BuildCache changes it. OCCT keeps the coefficients in a 128-element member buffer; the port allocates
/// a (Degree + 1) x row-length array once, in the constructor. Evaluation allocates nothing.
/// </remarks>
public class BSplCLib_Cache : Standard_Transient
{
    private readonly bool myIsRational;          //!< identifies the rationality of Bezier/B-spline curve
    private readonly BSplCLib_CacheParams myParams; //!< cache parameters
    private readonly int myRowLength;            //!< number of columns in the cache array
    // Poles and weights of the cache, (Degree + 1) rows of x y [z] [w]
    // (for 2D curves: no z component; for non-rational curves: no weight)
    private readonly NCollection_Array2<double> myPolesWeights;

    /// <summary>
    /// Prepares the cache for a 2D curve of the given degree and flat knots (knots with repetitions); the poles
    /// only distinguish this constructor from the 3D one.
    /// </summary>
    public BSplCLib_Cache(int theDegree, bool thePeriodic, NCollection_Array1<double> theFlatKnots,
                          NCollection_Array1<gp_Pnt2d> thePoles2d, NCollection_Array1<double>? theWeights = null)
    {
        _ = thePoles2d;
        myIsRational = theWeights != null;
        myParams = new BSplCLib_CacheParams(theDegree, thePeriodic, theFlatKnots);
        myRowLength = myIsRational ? 3 : 2;
        myPolesWeights = new NCollection_Array2<double>(1, myParams.Degree + 1, 1, myRowLength);
    }

    /// <summary>
    /// Prepares the cache for a 3D curve of the given degree and flat knots (knots with repetitions); the poles
    /// only distinguish this constructor from the 2D one.
    /// </summary>
    public BSplCLib_Cache(int theDegree, bool thePeriodic, NCollection_Array1<double> theFlatKnots,
                          NCollection_Array1<gp_Pnt> thePoles, NCollection_Array1<double>? theWeights = null)
    {
        _ = thePoles;
        myIsRational = theWeights != null;
        myParams = new BSplCLib_CacheParams(theDegree, thePeriodic, theFlatKnots);
        myRowLength = myIsRational ? 4 : 3;
        myPolesWeights = new NCollection_Array2<double>(1, myParams.Degree + 1, 1, myRowLength);
    }

    /// <summary>True if the cached span is the one for the parameter (a flat-knot parameter).</summary>
    public bool IsCacheValid(double theParameter)
    {
        return myParams.IsCacheValid(theParameter);
    }

    /// <summary>Recomputes the cache for the span of <paramref name="theParameter"/> of a 2D curve, without checking whether it is valid.</summary>
    internal void BuildCache(double theParameter, NCollection_Array1<double> theFlatKnots,
                             NCollection_Array1<gp_Pnt2d> thePoles2d, NCollection_Array1<double>? theWeights)
    {
        // Normalize theParameter for periodical B-splines
        double aNewParam = myParams.PeriodicNormalization(theParameter);
        myParams.LocateParameter(ref aNewParam, theFlatKnots);

        // Calculate new cache data
        BSplCLib.BuildCache(myParams.SpanStart,
                            myParams.SpanLength,
                            myParams.IsPeriodic,
                            myParams.Degree,
                            myParams.SpanIndex,
                            theFlatKnots,
                            thePoles2d,
                            theWeights,
                            myPolesWeights);
    }

    /// <summary>Recomputes the cache for the span of <paramref name="theParameter"/> of a 3D curve, without checking whether it is valid.</summary>
    internal void BuildCache(double theParameter, NCollection_Array1<double> theFlatKnots,
                             NCollection_Array1<gp_Pnt> thePoles, NCollection_Array1<double>? theWeights = null)
    {
        // Create list of knots with repetitions and normalize theParameter for periodical B-splines
        double aNewParam = myParams.PeriodicNormalization(theParameter);
        myParams.LocateParameter(ref aNewParam, theFlatKnots);

        // Calculate new cache data
        BSplCLib.BuildCache(myParams.SpanStart,
                            myParams.SpanLength,
                            myParams.IsPeriodic,
                            myParams.Degree,
                            myParams.SpanIndex,
                            theFlatKnots,
                            thePoles,
                            theWeights,
                            myPolesWeights);
    }

    /// <summary>
    /// Fills <paramref name="theDerivArray"/> with the derivatives up to <paramref name="theDerivative"/> at the
    /// parameter: (theDerivative + 1) * (PntDim + 1) values, PntDim being 2 or 3.
    /// </summary>
    protected void calculateDerivative(double theParameter, int theDerivative, Span<double> theDerivArray)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        calculateDerivativeLocal(aLocalParam, theDerivative, theDerivArray);
    }

    /// <summary>
    /// Fills <paramref name="theDerivArray"/> with the derivatives up to <paramref name="theDerivative"/> (1, 2 or
    /// 3) at the local parameter (Param - SpanStart) / SpanLength.
    /// </summary>
    protected void calculateDerivativeLocal(double theLocalParam, int theDerivative, Span<double> theDerivArray)
    {
        int aDimension = myRowLength;

        // Temporary container. The maximal size of this container is defined by:
        //    1) maximal derivative for cache evaluation, which is 3, plus one row for function values,
        //    2) and maximal dimension of the point, which is 3, plus one column for weights.
        Span<double> aTmpContainer = stackalloc double[16];

        // When the PLib::RationalDerivative needs to be called, use temporary container
        Span<double> aPntDeriv = myIsRational ? aTmpContainer : theDerivArray;

        // When the degree of curve is lesser than the requested derivative,
        // nullify array cells corresponding to greater derivatives
        int aDerivative = theDerivative;
        if (!myIsRational && myParams.Degree < theDerivative)
        {
            aDerivative = myParams.Degree;
            for (int ind = myParams.Degree * aDimension; ind < (theDerivative + 1) * aDimension; ind++)
            {
                aPntDeriv[ind] = 0.0;
            }
        }

        PLib.EvalPolynomial(theLocalParam,
                            aDerivative,
                            myParams.Degree,
                            aDimension,
                            myPolesWeights.AsSpan(),
                            aPntDeriv);

        // Unnormalize derivatives since those are computed normalized
        // Use division by SpanLength instead of multiplication by precomputed inverse
        // for better numerical stability with very small span lengths
        double aFactor = 1.0;
        for (int deriv = 1; deriv <= aDerivative; deriv++)
        {
            aFactor /= myParams.SpanLength;
            for (int ind = 0; ind < aDimension; ind++)
            {
                aPntDeriv[aDimension * deriv + ind] *= aFactor;
            }
        }

        if (myIsRational)
        {
            PLib.RationalDerivative(aDerivative,
                                    aDerivative,
                                    aDimension - 1,
                                    aPntDeriv,
                                    theDerivArray);
        }
    }

    /// <summary>The point of the 2D curve at the parameter.</summary>
    public void D0(double theParameter, out gp_Pnt2d thePoint)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        D0Local(aLocalParam, out thePoint);
    }

    /// <summary>The point of the 3D curve at the parameter.</summary>
    public void D0(double theParameter, out gp_Pnt thePoint)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        D0Local(aLocalParam, out thePoint);
    }

    /// <summary>
    /// The 3D point at the local parameter (Param - SpanStart) / SpanLength, in [0, 1]; skips the periodic
    /// normalization and the local parameter computation.
    /// </summary>
    public void D0Local(double theLocalParam, out gp_Pnt thePoint)
    {
        // theLocalParam is already computed as (param - SpanStart) / SpanLength
        Span<double> aPoint = stackalloc double[4];

        PLib.NoDerivativeEvalPolynomial(theLocalParam,
                                        myParams.Degree,
                                        myRowLength,
                                        myParams.Degree * myRowLength,
                                        myPolesWeights.AsSpan(),
                                        aPoint);

        thePoint = new gp_Pnt(aPoint[0], aPoint[1], aPoint[2]);
        if (myIsRational)
        {
            thePoint.ChangeCoord().Divide(aPoint[3]);
        }
    }

    /// <summary>The point and first derivative of the 2D curve at the parameter.</summary>
    public void D1(double theParameter, out gp_Pnt2d thePoint, out gp_Vec2d theTangent)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        D1Local(aLocalParam, out thePoint, out theTangent);
    }

    /// <summary>The point and first derivative of the 3D curve at the parameter.</summary>
    public void D1(double theParameter, out gp_Pnt thePoint, out gp_Vec theTangent)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        D1Local(aLocalParam, out thePoint, out theTangent);
    }

    /// <summary>The 3D point and first derivative at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D1Local(double theLocalParam, out gp_Pnt thePoint, out gp_Vec theTangent)
    {
        Span<double> aDerivArray = stackalloc double[8];
        calculateDerivativeLocal(theLocalParam, 1, aDerivArray);

        int aDim = myIsRational ? myRowLength - 1 : myRowLength;
        thePoint = new gp_Pnt(aDerivArray[0], aDerivArray[1], aDerivArray[2]);
        theTangent = new gp_Vec(aDerivArray[aDim], aDerivArray[aDim + 1], aDerivArray[aDim + 2]);
    }

    /// <summary>The point and first two derivatives of the 2D curve at the parameter.</summary>
    public void D2(double theParameter, out gp_Pnt2d thePoint, out gp_Vec2d theTangent, out gp_Vec2d theCurvature)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        D2Local(aLocalParam, out thePoint, out theTangent, out theCurvature);
    }

    /// <summary>The point and first two derivatives of the 3D curve at the parameter.</summary>
    public void D2(double theParameter, out gp_Pnt thePoint, out gp_Vec theTangent, out gp_Vec theCurvature)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        D2Local(aLocalParam, out thePoint, out theTangent, out theCurvature);
    }

    /// <summary>The 3D point and first two derivatives at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D2Local(double theLocalParam, out gp_Pnt thePoint, out gp_Vec theTangent, out gp_Vec theCurvature)
    {
        Span<double> aDerivArray = stackalloc double[12];
        calculateDerivativeLocal(theLocalParam, 2, aDerivArray);

        int aDim = myIsRational ? myRowLength - 1 : myRowLength;
        int aShift = aDim << 1;
        thePoint = new gp_Pnt(aDerivArray[0], aDerivArray[1], aDerivArray[2]);
        theTangent = new gp_Vec(aDerivArray[aDim], aDerivArray[aDim + 1], aDerivArray[aDim + 2]);
        theCurvature = new gp_Vec(aDerivArray[aShift], aDerivArray[aShift + 1], aDerivArray[aShift + 2]);
    }

    /// <summary>The point and first three derivatives of the 2D curve at the parameter.</summary>
    public void D3(double theParameter, out gp_Pnt2d thePoint, out gp_Vec2d theTangent, out gp_Vec2d theCurvature,
                   out gp_Vec2d theTorsion)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        D3Local(aLocalParam, out thePoint, out theTangent, out theCurvature, out theTorsion);
    }

    /// <summary>The point and first three derivatives of the 3D curve at the parameter.</summary>
    public void D3(double theParameter, out gp_Pnt thePoint, out gp_Vec theTangent, out gp_Vec theCurvature,
                   out gp_Vec theTorsion)
    {
        double aLocalParam = myParams.PeriodicNormalization(theParameter);
        aLocalParam = (aLocalParam - myParams.SpanStart) / myParams.SpanLength;
        D3Local(aLocalParam, out thePoint, out theTangent, out theCurvature, out theTorsion);
    }

    /// <summary>The 3D point and first three derivatives at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D3Local(double theLocalParam, out gp_Pnt thePoint, out gp_Vec theTangent, out gp_Vec theCurvature,
                        out gp_Vec theTorsion)
    {
        Span<double> aDerivArray = stackalloc double[16];
        calculateDerivativeLocal(theLocalParam, 3, aDerivArray);

        int aDim = myIsRational ? myRowLength - 1 : myRowLength;
        int aShift2 = aDim << 1;
        int aShift3 = aShift2 + aDim;
        thePoint = new gp_Pnt(aDerivArray[0], aDerivArray[1], aDerivArray[2]);
        theTangent = new gp_Vec(aDerivArray[aDim], aDerivArray[aDim + 1], aDerivArray[aDim + 2]);
        theCurvature = new gp_Vec(aDerivArray[aShift2], aDerivArray[aShift2 + 1], aDerivArray[aShift2 + 2]);
        theTorsion = new gp_Vec(aDerivArray[aShift3], aDerivArray[aShift3 + 1], aDerivArray[aShift3 + 2]);
    }

    /// <summary>
    /// The 2D point at the local parameter (Param - SpanStart) / SpanLength, in [0, 1]; skips the periodic
    /// normalization and the local parameter computation.
    /// </summary>
    public void D0Local(double theLocalParam, out gp_Pnt2d thePoint)
    {
        Span<double> aPoint = stackalloc double[4];

        PLib.NoDerivativeEvalPolynomial(theLocalParam,
                                        myParams.Degree,
                                        myRowLength,
                                        myParams.Degree * myRowLength,
                                        myPolesWeights.AsSpan(),
                                        aPoint);

        thePoint = new gp_Pnt2d(aPoint[0], aPoint[1]);
        if (myIsRational)
        {
            thePoint.ChangeCoord().Divide(aPoint[2]);
        }
    }

    /// <summary>The 2D point and first derivative at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D1Local(double theLocalParam, out gp_Pnt2d thePoint, out gp_Vec2d theTangent)
    {
        Span<double> aDerivArray = stackalloc double[8];
        calculateDerivativeLocal(theLocalParam, 1, aDerivArray);

        int aDim = myIsRational ? myRowLength - 1 : myRowLength;
        thePoint = new gp_Pnt2d(aDerivArray[0], aDerivArray[1]);
        theTangent = new gp_Vec2d(aDerivArray[aDim], aDerivArray[aDim + 1]);
    }

    /// <summary>The 2D point and first two derivatives at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D2Local(double theLocalParam, out gp_Pnt2d thePoint, out gp_Vec2d theTangent, out gp_Vec2d theCurvature)
    {
        Span<double> aDerivArray = stackalloc double[12];
        calculateDerivativeLocal(theLocalParam, 2, aDerivArray);

        int aDim = myIsRational ? myRowLength - 1 : myRowLength;
        int aShift = aDim << 1;
        thePoint = new gp_Pnt2d(aDerivArray[0], aDerivArray[1]);
        theTangent = new gp_Vec2d(aDerivArray[aDim], aDerivArray[aDim + 1]);
        theCurvature = new gp_Vec2d(aDerivArray[aShift], aDerivArray[aShift + 1]);
    }

    /// <summary>The 2D point and first three derivatives at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D3Local(double theLocalParam, out gp_Pnt2d thePoint, out gp_Vec2d theTangent, out gp_Vec2d theCurvature,
                        out gp_Vec2d theTorsion)
    {
        Span<double> aDerivArray = stackalloc double[16];
        calculateDerivativeLocal(theLocalParam, 3, aDerivArray);

        int aDim = myIsRational ? myRowLength - 1 : myRowLength;
        int aShift2 = aDim << 1;
        int aShift3 = aShift2 + aDim;
        thePoint = new gp_Pnt2d(aDerivArray[0], aDerivArray[1]);
        theTangent = new gp_Vec2d(aDerivArray[aDim], aDerivArray[aDim + 1]);
        theCurvature = new gp_Vec2d(aDerivArray[aShift2], aDerivArray[aShift2 + 1]);
        theTorsion = new gp_Vec2d(aDerivArray[aShift3], aDerivArray[aShift3 + 1]);
    }
}
