// Port of: src/FoundationClasses/TKMath/math/math_BrentMinimum.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BrentMinimum.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BrentMinimum.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>Brent's method to find a minimum of a function of a single variable, without its derivative.</summary>
internal class BrentMinimum
{
    private const double Cgold = 0.3819660; // 0.5*(3 - sqrt(5))

    protected double _a;
    protected double _b;
    protected double _x;
    protected double _fx;
    protected double _fv;
    protected double _fw;
    protected double _xTol;
    protected double _epsz;

    private bool _done;
    private int _iter;
    private int _itermax;
    private bool _f;

    /// <summary>Prepares the algorithm; <see cref="Perform"/> does the job.</summary>
    public BrentMinimum(double tolX, int nbIterations = 100, double zeps = 1.0e-12)
    {
        _a = 0.0;
        _b = 0.0;
        _x = 0.0;
        _fx = 0.0;
        _fv = 0.0;
        _fw = 0.0;
        _xTol = tolX;
        _epsz = zeps;
        _done = false;
        _iter = 0;
        _itermax = nbIterations;
        _f = false;
    }

    /// <summary>Prepares the algorithm when F(Bx) is known; <see cref="Perform"/> does the job.</summary>
    public BrentMinimum(double tolX, double fbx, int nbIterations = 100, double zeps = 1.0e-12)
    {
        _a = 0.0;
        _b = 0.0;
        _x = 0.0;
        _fx = fbx;
        _fv = 0.0;
        _fw = 0.0;
        _xTol = tolX;
        _epsz = zeps;
        _done = false;
        _iter = 0;
        _itermax = nbIterations;
        _f = true;
    }

    private static void SHFT(ref double a, ref double b, ref double c, double d)
    {
        a = b;
        b = c;
        c = d;
    }

    /// <summary>
    /// Minimizes F from a bracketing triplet ax, bx, cx (bx between ax and cx, F(bx) less than F(ax) and F(cx)).
    /// The solution is found when abs(Xi - Xi-1) &lt;= TolX * abs(Xi) + ZEPS.
    /// </summary>
    public void Perform(Function f, double ax, double bx, double cx)
    {
        bool ok;
        double etemp, fu, p, q, r;
        double tol1, tol2, u, v, w, xm;
        double e = 0.0;
        double d = RealLast();

        _a = ((ax < cx) ? ax : cx);
        _b = ((ax > cx) ? ax : cx);
        _x = w = v = bx;
        if (!_f)
        {
            ok = f.Value(_x, out _fx);
            if (!ok)
            {
                return;
            }
        }

        _fw = _fv = _fx;
        for (_iter = 1; _iter <= _itermax; _iter++)
        {
            xm = 0.5 * (_a + _b);
            tol1 = _xTol * Math.Abs(_x) + _epsz;
            tol2 = 2.0 * tol1;
            if (IsSolutionReached(f))
            {
                _done = true;
                return;
            }

            if (Math.Abs(e) > tol1)
            {
                r = (_x - w) * (_fx - _fv);
                q = (_x - v) * (_fx - _fw);
                p = (_x - v) * q - (_x - w) * r;
                q = 2.0 * (q - r);
                if (q > 0.0)
                {
                    p = -p;
                }

                q = Math.Abs(q);
                etemp = e;
                e = d;
                if (Math.Abs(p) >= Math.Abs(0.5 * q * etemp) || p <= q * (_a - _x) || p >= q * (_b - _x))
                {
                    e = (_x >= xm ? _a - _x : _b - _x);
                    d = Cgold * e;
                }
                else
                {
                    d = p / q;
                    u = _x + d;
                    if (u - _a < tol2 || _b - u < tol2)
                    {
                        d = Math.CopySign(tol1, xm - _x);
                    }
                }
            }
            else
            {
                e = (_x >= xm ? _a - _x : _b - _x);
                d = Cgold * e;
            }

            u = (Math.Abs(d) >= tol1 ? _x + d : _x + Math.CopySign(tol1, d));
            ok = f.Value(u, out fu);
            if (!ok)
            {
                return;
            }

            if (fu <= _fx)
            {
                if (u >= _x)
                {
                    _a = _x;
                }
                else
                {
                    _b = _x;
                }

                SHFT(ref v, ref w, ref _x, u);
                SHFT(ref _fv, ref _fw, ref _fx, fu);
            }
            else
            {
                if (u < _x)
                {
                    _a = u;
                }
                else
                {
                    _b = u;
                }

                if (fu <= _fw || w == _x)
                {
                    v = w;
                    w = u;
                    _fv = _fw;
                    _fw = fu;
                }
                else if (fu <= _fv || v == _x || v == w)
                {
                    v = u;
                    _fv = fu;
                }
            }
        }

        _done = false;
    }

    /// <summary>
    /// Called at the end of each iteration to check if the solution is found. A sub-class can redefine it to stop
    /// the iterations on its own test.
    /// </summary>
    public virtual bool IsSolutionReached(Function function)
    {
        double twoTol = 2.0 * (_xTol * Math.Abs(_x) + _epsz);
        return (_x <= (twoTol + _a)) && (_x >= (_b - twoTol));
    }

    public bool IsDone => _done;

    /// <summary>The location of the minimum. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Location()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _x;
    }

    /// <summary>The value of the minimum. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Minimum()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _fx;
    }

    /// <summary>The number of iterations done. Throws <see cref="NotDoneException"/> if the minimum was not found.</summary>
    public int IterationCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _iter;
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BrentMinimum ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Location value = " + _x.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Minimum value = " + _fx.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Number of iterations = " + _iter + "\n");
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
