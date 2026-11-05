// Port of: src/FoundationClasses/TKMath/math/math_BracketedRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BracketedRoot.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BracketedRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>Brent's method to find a root of a function between two bounds, without its derivative.</summary>
internal class BracketedRoot
{
    private bool _done;
    private double _theRoot;
    private double _theError;
    private int _nbIter;

    // reference algorithm:
    //                   Brent method
    //                   numerical recipes in C (p. 269)

    /// <summary>
    /// Finds the root of <paramref name="f"/> between <paramref name="bound1"/> and <paramref name="bound2"/>; fails if
    /// F(Bound1) * F(Bound2) &gt; 0. The solution is found when abs(Xi - Xi-1) &lt;= Tolerance.
    /// </summary>
    public BracketedRoot(Function f, double bound1, double bound2, double tolerance,
                              int nbIterations = 100, double zeps = 1.0e-12)
    {
        double fa, fc, a, c = 0, d = 0, e = 0;
        double min1, min2, p, q, r, s, tol1, xm;

        a = bound1;
        _theRoot = bound2;
        f.Value(a, out fa);
        f.Value(_theRoot, out _theError);
        if (fa * _theError > 0.0)
        {
            _done = false;
        }
        else
        {
            fc = _theError;
            for (_nbIter = 1; _nbIter <= nbIterations; _nbIter++)
            {
                if (_theError * fc > 0.0)
                {
                    c = a; // rename a TheRoot c and adjust bounding interval d
                    fc = fa;
                    d = _theRoot - a;
                    e = d;
                }

                if (Math.Abs(fc) < Math.Abs(fa))
                {
                    a = _theRoot;
                    _theRoot = c;
                    c = a;
                    fa = _theError;
                    _theError = fc;
                    fc = fa;
                }

                tol1 = 2.0 * zeps * Math.Abs(_theRoot) + 0.5 * tolerance; // convergence check
                xm = 0.5 * (c - _theRoot);
                if (Math.Abs(xm) <= tol1 || _theError == 0.0)
                {
                    _done = true;
                    return;
                }

                if (Math.Abs(e) >= tol1 && Math.Abs(fa) > Math.Abs(_theError))
                {
                    s = _theError / fa; // attempt inverse quadratic interpolation
                    if (a == c)
                    {
                        p = 2.0 * xm * s;
                        q = 1.0 - s;
                    }
                    else
                    {
                        q = fa / fc;
                        r = _theError / fc;
                        p = s * (2.0 * xm * q * (q - r) - (_theRoot - a) * (r - 1.0));
                        q = (q - 1.0) * (r - 1.0) * (s - 1.0);
                    }

                    if (p > 0.0)
                    {
                        q = -q;
                    } // check whether in bounds

                    p = Math.Abs(p);
                    min1 = 3.0 * xm * q - Math.Abs(tol1 * q);
                    min2 = Math.Abs(e * q);
                    if (2.0 * p < (min1 < min2 ? min1 : min2))
                    {
                        e = d; // accept interpolation
                        d = p / q;
                    }
                    else
                    {
                        d = xm; // interpolation failed,use bissection
                        e = d;
                    }
                }
                else
                { // bounds decreasing too slowly ,use bissection
                    d = xm;
                    e = d;
                }

                a = _theRoot; // move last best guess to a
                fa = _theError;
                if (Math.Abs(d) > tol1)
                { // evaluate new trial root
                    _theRoot += d;
                }
                else
                {
                    _theRoot += (xm > 0.0 ? Math.Abs(tol1) : -Math.Abs(tol1));
                }

                f.Value(_theRoot, out _theError);
            }

            _done = false;
        }
    }

    public bool IsDone => _done;

    /// <summary>The root. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Root()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _theRoot;
    }

    /// <summary>The value of the function at the root. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _theError;
    }

    /// <summary>The number of iterations done. Throws <see cref="NotDoneException"/> if the root was not found.</summary>
    public int IterationCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _nbIter;
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BracketedRoot ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of iterations = " + _nbIter + '\n');
            o.Write(" The Root is: " + _theRoot.ToString(CultureInfo.InvariantCulture) + '\n');
            o.Write(" The value at the root is: " + _theError.ToString(CultureInfo.InvariantCulture) + '\n');
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
