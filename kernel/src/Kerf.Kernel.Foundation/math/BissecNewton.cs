// Port of: src/FoundationClasses/TKMath/math/math_BissecNewton.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BissecNewton.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BissecNewton.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;
using static Kerf.Engine.Numerics.NumericStatus;

namespace Kerf.Engine.Numerics;

/// <summary>
/// A combination of the Newton-Raphson and bisection methods to find a root of a function between two bounds.
/// The derivative is required.
/// </summary>
internal class BissecNewton
{
    protected NumericStatus _theStatus;
    protected double _xTol;
    protected double _x;
    protected double _dx;
    protected double _f;
    protected double _df;

    private bool _done;

    /// <param name="xTolerance">The tolerance of the algorithm.</param>
    public BissecNewton(double xTolerance)
    {
        _theStatus = NotBracketed;
        _xTol = xTolerance;
        _x = 0.0;
        _dx = 0.0;
        _f = 0.0;
        _df = 0.0;
        _done = false;
    }

    /// <summary>
    /// Finds the root of <paramref name="f"/> between <paramref name="bound1"/> and <paramref name="bound2"/>. The
    /// solution is found when abs(Xi - Xi-1) &lt;= TolX and F(Xi) * F(Xi-1) &lt;= 0.
    /// </summary>
    public void Perform(FunctionWithDerivative f, double bound1, double bound2, int nbIterations = 100)
    {
        bool good;
        int j;
        double dxold, fh, fl;
        double temp, xh, xl;

        good = f.Values(bound1, out fl, out _df);
        if (!good)
        {
            _done = false;
            _theStatus = FunctionError;
            return;
        }

        good = f.Values(bound2, out fh, out _df);
        if (!good)
        {
            _done = false;
            _theStatus = FunctionError;
            return;
        }

        //  Modified by Sergey KHROMOV - Wed Jan 22 12:06:45 2003 Begin
        double fTol = RealEpsilon();
        //   if(fl * fh >= 0.0) {
        if (fl * fh > fTol * fTol)
        {
            _done = false;
            _theStatus = NotBracketed;
            return;
        }

        //   if(fl < 0.0) {
        if (fl < -fTol || (fl < fTol && fh < -fTol))
        {
            xl = bound1;
            xh = bound2;
        }
        else
        {
            xl = bound2;
            xh = bound1;
        }
        //  Modified by Sergey KHROMOV - Wed Jan 22 12:06:49 2003 End

        _x = 0.5 * (bound1 + bound2);
        dxold = Math.Abs(bound2 - bound1);
        _dx = dxold;
        good = f.Values(_x, out _f, out _df);
        if (!good)
        {
            _done = false;
            _theStatus = FunctionError;
            return;
        }

        for (j = 1; j <= nbIterations; j++)
        {
            if ((((_x - xh) * _df - _f) * ((_x - xl) * _df - _f) >= 0.0) || (Math.Abs(2.0 * _f) > Math.Abs(dxold * _df)))
            {
                dxold = _dx;
                _dx = 0.5 * (xh - xl);
                _x = xl + _dx;
                if (Math.Abs(_dx) < _xTol)
                {
                    _theStatus = OK;
                    _done = true;
                    return;
                }
            }
            else
            {
                dxold = _dx;
                _dx = _f / _df;
                temp = _x;
                _x -= _dx;
                if (temp == _x)
                {
                    _theStatus = OK;
                    _done = true;
                    return;
                }
            }

            if (IsSolutionReached(f))
            {
                _theStatus = OK;
                _done = true;
                return;
            }

            good = f.Values(_x, out _f, out _df);
            if (!good)
            {
                _done = false;
                _theStatus = FunctionError;
                return;
            }

            if (_f < 0.0)
            {
                xl = _x;
            }
            else if (_f > 0.0)
            {
                xh = _x;
            }
            else
            {
                _theStatus = OK;
                _done = true;
                return;
            }
        }

        _theStatus = TooManyIterations;
        _done = false;
    }

    /// <summary>
    /// Called at the end of each iteration to check if the solution is found. A sub-class can redefine it to stop
    /// the iterations on its own test.
    /// </summary>
    public virtual bool IsSolutionReached(FunctionWithDerivative function)
    {
        return Math.Abs(_dx) <= _xTol;
    }

    public bool IsDone => _done;

    /// <summary>The root. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Root()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _x;
    }

    /// <summary>The derivative at the root. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Derivative()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _df;
    }

    /// <summary>The value of the function at the root. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _f;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BissecNewton ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" The Root  is: " + _x.ToString(CultureInfo.InvariantCulture) + '\n');
            o.Write(" The value at this Root is: " + _f.ToString(CultureInfo.InvariantCulture) + '\n');
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
