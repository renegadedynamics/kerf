// Port of: src/FoundationClasses/TKMath/math/math_KronrodSingleIntegration.hxx
// Port of: src/FoundationClasses/TKMath/math/math_KronrodSingleIntegration.lxx
// Port of: src/FoundationClasses/TKMath/math/math_KronrodSingleIntegration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Numerics;

/// <summary>The Gauss-Kronrod method of computing an integral, with adaptive subdivision.</summary>
internal class KronrodSingleIntegration
{
    private bool _isDone;
    private double _value;
    private double _errorReached;
    private double _absolutError;
    private int _nbPntsReached;
    private int _nbIterReached;

    public KronrodSingleIntegration()
    {
        _isDone = false;
        _value = 0.0;
        _errorReached = 0.0;
        _absolutError = 0.0;
        _nbPntsReached = 0;
        _nbIterReached = 0;
    }

    /// <summary>Integrates the function between the bounds with the given initial number of Kronrod points.</summary>
    public KronrodSingleIntegration(Function function, double lower, double upper, int nbPnts)
        : this()
    {
        Perform(function, lower, upper, nbPnts);
    }

    /// <summary>
    /// Integrates the function between the bounds with the given initial number of Kronrod points, relative
    /// tolerance and maximal number of iterations.
    /// </summary>
    public KronrodSingleIntegration(Function function, double lower, double upper, int nbPnts,
                                         double tolerance, int maxNbIter)
        : this()
    {
        Perform(function, lower, upper, nbPnts, tolerance, maxNbIter);
    }

    /// <summary>
    /// Computes the integral with the given initial number of Kronrod points (odd, at least 3; an even number is
    /// increased by one).
    /// </summary>
    public void Perform(Function function, double lower, double upper, int nbPnts)
    {
        // const double aMinVol = Epsilon(1.);
        const double ptol = 1.0e-9;
        _nbIterReached = 0;

        if (nbPnts < 3)
        {
            _isDone = false;
            return;
        }

        if (upper - lower < ptol)
        {
            _isDone = false;
            return;
        }

        // Get an odd value of number of initial points.
        _nbPntsReached = (nbPnts % 2 == 0) ? nbPnts + 1 : nbPnts;
        _errorReached = RealLast();

        int nGauss = _nbPntsReached / 2;
        math_Vector kronrodP = new(1, _nbPntsReached);
        math_Vector kronrodW = new(1, _nbPntsReached);
        math_Vector gaussP = new(1, nGauss);
        math_Vector gaussW = new(1, nGauss);

        if (!NumericTables.KronrodPointsAndWeights(_nbPntsReached, kronrodP, kronrodW)
            || !NumericTables.OrderedGaussPointsAndWeights(nGauss, gaussP, gaussW))
        {
            _isDone = false;
            return;
        }

        _isDone = GKRule(function, lower, upper, gaussP, gaussW, kronrodP, kronrodW,
                          out _value, ref _errorReached);

        if (!_isDone)
        {
            return;
        }

        // double anAbsVal = std::abs(myValue);

        _absolutError = _errorReached;

        // if (anAbsVal > aMinVol)
        // myErrorReached /= anAbsVal;

        _nbIterReached++;
    }

    /// <summary>
    /// Computes the integral with the given initial number of Kronrod points (odd, at least 3), subdividing the
    /// interval of largest error until abs(Kronrod - Gauss)/abs(Kronrod) &lt; <paramref name="tolerance"/>
    /// (relative, positive) or <paramref name="maxNbIter"/> iterations are done.
    /// </summary>
    public void Perform(Function function, double lower, double upper, int nbPnts,
                        double tolerance, int maxNbIter)
    {
        double minVol = Epsilon(1.0);
        _nbIterReached = 0;

        // Check prerequisites.
        if (nbPnts < 3 || tolerance <= 0.0)
        {
            _isDone = false;
            return;
        }

        // Get an odd value of number of initial points.
        _nbPntsReached = (nbPnts % 2 == 0) ? nbPnts + 1 : nbPnts;

        int nGauss = _nbPntsReached / 2;
        math_Vector kronrodP = new(1, _nbPntsReached);
        math_Vector kronrodW = new(1, _nbPntsReached);
        math_Vector gaussP = new(1, nGauss);
        math_Vector gaussW = new(1, nGauss);

        if (!NumericTables.KronrodPointsAndWeights(_nbPntsReached, kronrodP, kronrodW)
            || !NumericTables.OrderedGaussPointsAndWeights(nGauss, gaussP, gaussW))
        {
            _isDone = false;
            return;
        }

        // First iteration
        _isDone = GKRule(function, lower, upper, gaussP, gaussW, kronrodP, kronrodW,
                          out _value, ref _errorReached);

        if (!_isDone)
        {
            return;
        }

        double absVal = Math.Abs(_value);

        _absolutError = _errorReached;

        if (absVal > minVol)
        {
            _errorReached /= absVal;
        }

        _nbIterReached++;

        if (_errorReached <= tolerance)
        {
            return;
        }

        if (_nbIterReached >= maxNbIter)
        {
            return;
        }

        NCollection_Sequence<double> intervals = new();
        NCollection_Sequence<double> errors = new();
        NCollection_Sequence<double> values = new();
        intervals.Append(lower);
        intervals.Append(upper);
        errors.Append(_absolutError);
        values.Append(_value);

        int i, nint, nbints;

        double maxerr;
        int count = 0;
        while (_errorReached > tolerance && _nbIterReached < maxNbIter)
        {
            // Searching interval with max error
            nbints = intervals.Length() - 1;
            nint = 0;
            maxerr = 0.0;
            for (i = 1; i <= nbints; ++i)
            {
                if (errors[i] > maxerr)
                {
                    maxerr = errors[i];
                    nint = i;
                }
            }

            double a = intervals[nint];
            double b = intervals[nint + 1];
            double c = 0.5 * (a + b);

            double v1, v2, e1 = 0.0, e2 = 0.0;
            _isDone = GKRule(function, a, c, gaussP, gaussW, kronrodP, kronrodW, out v1, ref e1);
            if (!_isDone)
            {
                return;
            }

            _isDone = GKRule(function, c, b, gaussP, gaussW, kronrodP, kronrodW, out v2, ref e2);
            if (!_isDone)
            {
                return;
            }

            _nbIterReached++;

            double deltav = v1 + v2 - values[nint];
            _value += deltav;
            if (Math.Abs(deltav) <= Epsilon(Math.Abs(_value)))
            {
                ++count;
            }

            double deltae = e1 + e2 - errors[nint];
            _absolutError += deltae;
            if (_absolutError <= Epsilon(Math.Abs(_value)))
            {
                ++count;
            }

            if (Math.Abs(_value) > minVol)
            {
                _errorReached = _absolutError / Math.Abs(_value);
            }
            else
            {
                _errorReached = _absolutError;
            }

            if (count > 50)
            {
                return;
            }

            // Inserting new interval

            intervals.InsertAfter(nint, c);
            errors[nint] = e1;
            errors.InsertAfter(nint, e2);
            values[nint] = v1;
            values.InsertAfter(nint, v2);
        }
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone => _isDone;

    /// <summary>The value of the integral. Throws <see cref="NotDoneException"/> if it was not computed.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!_isDone, "math_KronrodSingleIntegration");
        return _value;
    }

    /// <summary>The relative error reached.</summary>
    public double ErrorReached()
    {
        StdFail_NotDone_Raise_if(!_isDone, "math_KronrodSingleIntegration");
        return _errorReached;
    }

    /// <summary>The absolute error reached.</summary>
    public double AbsolutError()
    {
        StdFail_NotDone_Raise_if(!_isDone, "math_KronrodSingleIntegration");
        return _absolutError;
    }

    /// <summary>The number of Kronrod points of the result.</summary>
    public int OrderReached()
    {
        StdFail_NotDone_Raise_if(!_isDone, "math_KronrodSingleIntegration");
        return _nbPntsReached;
    }

    /// <summary>The number of iterations made.</summary>
    public int IterReachedCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_isDone, "math_KronrodSingleIntegration");
            return _nbIterReached;
        }
    }

    /// <summary>
    /// One Gauss-Kronrod rule on [theLower, theUpper]: the Kronrod value and the error estimate. False if the function
    /// fails. <paramref name="error"/> is <c>ref</c> because OCCT leaves it unchanged when the function fails.
    /// </summary>
    public static bool GKRule(Function function, double lower, double upper,
                              math_Vector gaussP, math_Vector gaussW,
                              math_Vector kronrodP, math_Vector kronrodW,
                              out double value, ref double error)
    {
        _ = gaussP;
        bool isDone;
        int nKronrod = kronrodP.Length();

        double gaussVal;
        int nPnt2 = (nKronrod + 1) / 2;
        int i;
        double dx;
        double val1;
        double val2;

        math_Vector f1 = new(1, nPnt2 - 1);
        math_Vector f2 = new(1, nPnt2 - 1);

        double xm = 0.5 * (upper + lower);
        double xr = 0.5 * (upper - lower);

        // Compute Gauss quadrature
        gaussVal = 0.0;
        value = 0.0;

        for (i = 2; i < nPnt2; i += 2)
        {
            dx = xr * kronrodP.Value(i);

            val2 = 0.0; // C++ leaves it unset when the first call fails
            if (!function.Value(xm + dx, out val1) || !function.Value(xm - dx, out val2))
            {
                isDone = false;
                return isDone;
            }

            f1[i] = val1;
            f2[i] = val2;
            gaussVal += (val1 + val2) * gaussW.Value(i / 2);
            value += (val1 + val2) * kronrodW.Value(i);
        }

        // Compute value in the middle point.
        if (!function.Value(xm, out val1))
        {
            isDone = false;
            return isDone;
        }

        double fc = val1;
        value += val1 * kronrodW.Value(nPnt2);

        if (i == nPnt2)
        {
            gaussVal += val1 * gaussW.Value(nPnt2 / 2);
        }

        // Compute Kronrod quadrature
        for (i = 1; i < nPnt2; i += 2)
        {
            dx = xr * kronrodP.Value(i);

            val2 = 0.0; // C++ leaves it unset when the first call fails
            if (!function.Value(xm + dx, out val1) || !function.Value(xm - dx, out val2))
            {
                isDone = false;
                return isDone;
            }

            f1[i] = val1;
            f2[i] = val2;
            value += (val1 + val2) * kronrodW.Value(i);
        }

        double mean = 0.5 * value;

        double asc = Math.Abs(fc - mean) * kronrodW.Value(nPnt2);
        for (i = 1; i < nPnt2; ++i)
        {
            asc += kronrodW.Value(i) * (Math.Abs(f1[i] - mean) + Math.Abs(f2[i] - mean));
        }

        asc *= xr;
        value *= xr;
        gaussVal *= xr;

        // Compute the error and the new number of Kronrod points.

        error = Math.Abs(value - gaussVal);

        double scale = 1.0;
        if (asc != 0.0 && error != 0.0)
        {
            scale = Math.Pow((200.0 * error / asc), 1.5);
        }

        if (scale < 1.0)
        {
            error = Min(error, asc * scale);
        }

        // theFunction.GetStateNumber();

        return true;
    }
}
