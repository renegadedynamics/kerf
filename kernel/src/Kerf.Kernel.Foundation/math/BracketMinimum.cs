// Port of: src/FoundationClasses/TKMath/math/math_BracketMinimum.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BracketMinimum.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BracketMinimum.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// From two distinct initial points, computes three points (a, b, c) which bracket a minimum of a function:
/// a &lt; b &lt; c, F(b) &lt; F(a) and F(b) &lt; F(c).
/// </summary>
/// <remarks>
/// By default the parameter is not limited; <see cref="SetLimits"/> defines the allowed range. If no minimum is found
/// within the limits, <see cref="IsDone"/> is false. The caller must give A and B within the limits.
/// </remarks>
internal class BracketMinimum
{
    private const double Gold = 1.618034;
    private const double Glimit = 100.0;
    private const double Tiny = 1.0e-20;

    private bool _done;
    private double _ax;
    private double _bx;
    private double _cx;
    private double _fAx;
    private double _fBx;
    private double _fCx;
    private double _left;
    private double _right;
    private bool _isLimited;
    private bool _fa;
    private bool _fb;

    /// <summary>Prepares A and B only; <see cref="Perform"/> does the job.</summary>
    public BracketMinimum(double a, double b)
    {
        _done = false;
        _ax = a;
        _bx = b;
        _cx = 0.0;
        _fAx = 0.0;
        _fBx = 0.0;
        _fCx = 0.0;
        _left = -global::Kerf.Engine.Foundation.Precision.Infinite();
        _right = global::Kerf.Engine.Foundation.Precision.Infinite();
        _isLimited = false;
        _fa = false;
        _fb = false;
    }

    /// <summary>Computes a bracketing triplet from the initial values <paramref name="a"/> and <paramref name="b"/>.</summary>
    public BracketMinimum(Function f, double a, double b) : this(a, b)
    {
        Perform(f);
    }

    /// <summary>Computes a bracketing triplet when F(A) is known.</summary>
    public BracketMinimum(Function f, double a, double b, double fa) : this(a, b)
    {
        _fAx = fa;
        _fa = true;
        Perform(f);
    }

    /// <summary>Computes a bracketing triplet when F(A) and F(B) are known.</summary>
    public BracketMinimum(Function f, double a, double b, double fa, double fb) : this(a, b)
    {
        _fAx = fa;
        _fBx = fb;
        _fa = true;
        _fb = true;
        Perform(f);
    }

    /// <summary>Limits the parameter to [theLeft, theRight].</summary>
    public void SetLimits(double left, double right)
    {
        _left = left;
        _right = right;
        _isLimited = true;
    }

    /// <summary>Sets the function value at A.</summary>
    public void SetFA(double value)
    {
        _fAx = value;
        _fa = true;
    }

    /// <summary>Sets the function value at B.</summary>
    public void SetFB(double value)
    {
        _fBx = value;
        _fb = true;
    }

    /// <summary>Limits the given value to the range [myLeft, myRight].</summary>
    private double Limited(double value)
    {
        return value < _left ? _left : value > _right ? _right : value;
    }

    /// <summary>
    /// Limits C and computes the function there. If C falls between A and B, swaps B and C with their values.
    /// False if C becomes equal to B or the function fails.
    /// </summary>
    private bool LimitAndMayBeSwap(Function f, double a, ref double b, ref double fb, ref double c, ref double fc)
    {
        c = Limited(c);
        if (Math.Abs(b - c) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        bool ok = f.Value(c, out fc);
        if (!ok)
        {
            return false;
        }

        // check that B is between A and C
        if ((a - b) * (b - c) < 0)
        {
            // swap B and C: SHFT(dum, theB, theC, dum); SHFT(dum, theFB, theFC, dum);
            double dum;
            dum = b;
            b = c;
            c = dum;
            dum = fb;
            fb = fc;
            fc = dum;
        }

        return true;
    }

    /// <summary>Performs the job; the constructors taking a function call it.</summary>
    public void Perform(Function f)
    {
        bool ok;
        double ulim, u, r, q, fu, dum;

        _done = false;

        double lambda = Gold;
        if (!_fa)
        {
            ok = f.Value(_ax, out _fAx);
            if (!ok)
            {
                return;
            }
        }

        if (!_fb)
        {
            ok = f.Value(_bx, out _fBx);
            if (!ok)
            {
                return;
            }
        }

        if (_fBx > _fAx)
        {
            // SHFT(dum, Ax, Bx, dum); SHFT(dum, FBx, FAx, dum);
            dum = _ax;
            _ax = _bx;
            _bx = dum;
            dum = _fBx;
            _fBx = _fAx;
            _fAx = dum;
        }

        // get next prob after (A, B)
        _cx = _bx + lambda * (_bx - _ax);
        if (_isLimited)
        {
            ok = LimitAndMayBeSwap(f, _ax, ref _bx, ref _fBx, ref _cx, ref _fCx);
            if (!ok)
            {
                return;
            }
        }
        else
        {
            ok = f.Value(_cx, out _fCx);
            if (!ok)
            {
                return;
            }
        }

        while (_fBx > _fCx)
        {
            r = (_bx - _ax) * (_fBx - _fCx);
            q = (_bx - _cx) * (_fBx - _fAx);
            u = _bx - ((_bx - _cx) * q - (_bx - _ax) * r) / (2.0 * SIGN(MAX(Math.Abs(q - r), Tiny), q - r));
            ulim = _bx + Glimit * (_cx - _bx);
            if (_isLimited)
            {
                ulim = Limited(ulim);
            }

            fu = 0.0; // C++ leaves fu uninitialised; every branch below sets it before use
            if ((_bx - u) * (u - _cx) > 0.0)
            {
                // u is between B and C
                ok = f.Value(u, out fu);
                if (!ok)
                {
                    return;
                }

                if (fu < _fCx)
                {
                    // solution is found (B, u, c)
                    _ax = _bx;
                    _bx = u;
                    _fAx = _fBx;
                    _fBx = fu;
                    _done = true;
                    return;
                }
                else if (fu > _fBx)
                {
                    // solution is found (A, B, u)
                    _cx = u;
                    _fCx = fu;
                    _done = true;
                    return;
                }

                // get next prob after (B, C)
                u = _cx + lambda * (_cx - _bx);
                if (_isLimited)
                {
                    ok = LimitAndMayBeSwap(f, _bx, ref _cx, ref _fCx, ref u, ref fu);
                    if (!ok)
                    {
                        return;
                    }
                }
                else
                {
                    ok = f.Value(u, out fu);
                    if (!ok)
                    {
                        return;
                    }
                }
            }
            else if ((_cx - u) * (u - ulim) > 0.0)
            {
                // u is beyond C but between C and limit
                ok = f.Value(u, out fu);
                if (!ok)
                {
                    return;
                }
            }
            else if ((u - ulim) * (ulim - _cx) >= 0.0)
            {
                // u is beyond limit
                u = ulim;
                ok = f.Value(u, out fu);
                if (!ok)
                {
                    return;
                }
            }
            else
            {
                // u tends to approach to the side of A,
                // so reset it to the next prob after (B, C)
                u = _cx + Gold * (_cx - _bx);
                if (_isLimited)
                {
                    ok = LimitAndMayBeSwap(f, _bx, ref _cx, ref _fCx, ref u, ref fu);
                    if (!ok)
                    {
                        return;
                    }
                }
                else
                {
                    ok = f.Value(u, out fu);
                    if (!ok)
                    {
                        return;
                    }
                }
            }

            // SHFT(Ax, Bx, Cx, u); SHFT(FAx, FBx, FCx, fu);
            _ax = _bx;
            _bx = _cx;
            _cx = u;
            _fAx = _fBx;
            _fBx = _fCx;
            _fCx = fu;
        }

        _done = true;
    }

    public bool IsDone => _done;

    /// <summary>The bracketing triplet of abscissae. Throws <see cref="NotDoneException"/> if the algorithm failed.</summary>
    public void Values(out double a, out double b, out double c)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        a = _ax;
        b = _bx;
        c = _cx;
    }

    /// <summary>The function values at the triplet. Throws <see cref="NotDoneException"/> if the algorithm failed.</summary>
    public void FunctionValues(out double fa, out double fb, out double fc)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        fa = _fAx;
        fb = _fBx;
        fc = _fCx;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BracketMinimum ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" The bracketed triplet is: " + '\n');
            o.Write(FormattableString.Invariant($"{_ax}, {_bx}, {_cx}") + '\n');
            o.Write(" The corresponding function values are: " + '\n');
            o.Write(FormattableString.Invariant($"{_fAx}, {_fBx}, {_fCx}") + '\n');
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

    private static double MAX(double a, double b) => a > b ? a : b;

    private static double SIGN(double a, double b) => b > 0.0 ? Math.Abs(a) : -Math.Abs(a);
}
