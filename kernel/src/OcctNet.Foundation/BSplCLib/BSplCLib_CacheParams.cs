// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_CacheParams.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The parametrisation of a B-spline curve or surface in one direction (U or V) and the current span, for the
/// caches (<see cref="BSplCLib_Cache"/>, BSplSLib_Cache).
/// </summary>
/// <remarks>
/// A non-copyable struct in OCCT, so a class here. The span data changes as the cache moves from span to span;
/// only OCCT.NET's caches change it.
/// </remarks>
public sealed class BSplCLib_CacheParams
{
    /// <summary>Degree of the Bezier or B-spline.</summary>
    public readonly int Degree;

    /// <summary>True if the B-spline is periodic.</summary>
    public readonly bool IsPeriodic;

    /// <summary>First valid parameter.</summary>
    public readonly double FirstParameter;

    /// <summary>Last valid parameter.</summary>
    public readonly double LastParameter;

    /// <summary>Minimal index of a span.</summary>
    public readonly int SpanIndexMin;

    /// <summary>Maximal index of a span.</summary>
    public readonly int SpanIndexMax;

    /// <summary>Parameter of the first point of the span.</summary>
    public double SpanStart { get; internal set; }

    /// <summary>Length of the span.</summary>
    public double SpanLength { get; internal set; }

    /// <summary>Index of the span.</summary>
    public int SpanIndex { get; internal set; }

    /// <summary>Prepares the data for caching on the B-spline (or Bezier) of the given degree and flat knots.</summary>
    public BSplCLib_CacheParams(int theDegree, bool thePeriodic, NCollection_Array1<double> theFlatKnots)
    {
        Degree = theDegree;
        IsPeriodic = thePeriodic;
        FirstParameter = theFlatKnots.Value(theFlatKnots.Lower() + theDegree);
        LastParameter = theFlatKnots.Value(theFlatKnots.Upper() - theDegree);
        SpanIndexMin = theFlatKnots.Lower() + theDegree;
        SpanIndexMax = theFlatKnots.Upper() - theDegree - 1;
        SpanStart = 0.0;
        SpanLength = 0.0;
        SpanIndex = 0;
    }

    /// <summary>Brings the parameter into the period of a periodic B-spline.</summary>
    public double PeriodicNormalization(double theParameter)
    {
        if (IsPeriodic)
        {
            double aPeriod = LastParameter - FirstParameter;
            if (theParameter < FirstParameter)
            {
                double aScale = Math.Truncate((FirstParameter - theParameter) / aPeriod);
                return theParameter + aPeriod * (aScale + 1.0);
            }
            if (theParameter > LastParameter)
            {
                double aScale = Math.Truncate((theParameter - LastParameter) / aPeriod);
                return theParameter - aPeriod * (aScale + 1.0);
            }
        }
        return theParameter;
    }

    /// <summary>True if the cached span is the one for the parameter.</summary>
    public bool IsCacheValid(double theParameter)
    {
        double aNewParam = PeriodicNormalization(theParameter);
        double aDelta = aNewParam - SpanStart;
        if ((aDelta < 0.0 && SpanIndex != SpanIndexMin)
            || (aDelta >= SpanLength && SpanIndex != SpanIndexMax))
        {
            return false;
        }

        if (SpanIndex == SpanIndexMax)
            return true;

        // from BSplCLib::LocateParameter() check hitting of the next knot
        // within double floating point precision
        double anEps = Epsilon(Min(Math.Abs(LastParameter), Math.Abs(aNewParam)));
        double aDeltaToNext = Math.Abs(aDelta - SpanLength);
        return aDeltaToNext > anEps; // next knot should be used instead
    }

    /// <summary>Finds the span of the parameter, which is replaced by its value in the knots' range.</summary>
    internal void LocateParameter(ref double theParameter, NCollection_Array1<double> theFlatKnots)
    {
        int aSpanIndex = 0;
        BSplCLib.LocateParameter(Degree,
                                 theFlatKnots,
                                 BSplCLib.NoMults(),
                                 theParameter,
                                 IsPeriodic,
                                 ref aSpanIndex,
                                 out theParameter);
        SpanIndex = aSpanIndex;
        SpanStart = theFlatKnots.Value(SpanIndex);
        SpanLength = theFlatKnots.Value(SpanIndex + 1) - SpanStart;
    }
}
