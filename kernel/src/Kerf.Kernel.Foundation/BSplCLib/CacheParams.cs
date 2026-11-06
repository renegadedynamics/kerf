// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_CacheParams.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Geometry.Splines;

/// <summary>
/// The parametrisation of a B-spline curve or surface in one direction (U or V) and the current span, for the
/// caches (<see cref="Cache"/>, BSplSLib_Cache).
/// </summary>
/// <remarks>
/// A non-copyable struct in OCCT, so a class here. The span data changes as the cache moves from span to span;
/// only Kerf's caches change it.
/// </remarks>
internal sealed class CacheParams
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
    public CacheParams(int degree, bool periodic, NCollection_Array1<double> flatKnots)
    {
        Degree = degree;
        IsPeriodic = periodic;
        FirstParameter = flatKnots.Value(flatKnots.Lower() + degree);
        LastParameter = flatKnots.Value(flatKnots.Upper() - degree);
        SpanIndexMin = flatKnots.Lower() + degree;
        SpanIndexMax = flatKnots.Upper() - degree - 1;
        SpanStart = 0.0;
        SpanLength = 0.0;
        SpanIndex = 0;
    }

    /// <summary>Brings the parameter into the period of a periodic B-spline.</summary>
    public double PeriodicNormalization(double parameter)
    {
        if (IsPeriodic)
        {
            double period = LastParameter - FirstParameter;
            if (parameter < FirstParameter)
            {
                double scale = Math.Truncate((FirstParameter - parameter) / period);
                return parameter + period * (scale + 1.0);
            }
            if (parameter > LastParameter)
            {
                double scale = Math.Truncate((parameter - LastParameter) / period);
                return parameter - period * (scale + 1.0);
            }
        }
        return parameter;
    }

    /// <summary>True if the cached span is the one for the parameter.</summary>
    public bool IsCacheValid(double parameter)
    {
        double newParam = PeriodicNormalization(parameter);
        double delta = newParam - SpanStart;
        if ((delta < 0.0 && SpanIndex != SpanIndexMin)
            || (delta >= SpanLength && SpanIndex != SpanIndexMax))
        {
            return false;
        }

        if (SpanIndex == SpanIndexMax)
            return true;

        // from BSplCLib::LocateParameter() check hitting of the next knot
        // within double floating point precision
        double eps = Epsilon(Min(Math.Abs(LastParameter), Math.Abs(newParam)));
        double deltaToNext = Math.Abs(delta - SpanLength);
        return deltaToNext > eps; // next knot should be used instead
    }

    /// <summary>Finds the span of the parameter, which is replaced by its value in the knots' range.</summary>
    internal void LocateParameter(ref double parameter, NCollection_Array1<double> flatKnots)
    {
        int spanIndex = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(Degree,
                                 flatKnots,
                                 global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                                 parameter,
                                 IsPeriodic,
                                 ref spanIndex,
                                 out parameter);
        SpanIndex = spanIndex;
        SpanStart = flatKnots.Value(SpanIndex);
        SpanLength = flatKnots.Value(SpanIndex + 1) - SpanStart;
    }
}
