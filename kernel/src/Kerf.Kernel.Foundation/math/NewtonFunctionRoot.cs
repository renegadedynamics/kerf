// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionRoot.lxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// Newton's method to find a root of a function of a single variable from an initial guess near it. The derivative
/// is required.
/// </summary>
internal class NewtonFunctionRoot
{
    private bool _done;
    private double _x;
    private double _fx;
    private double _dFx;
    private int _it;
    private double _epsilonX;
    private double _epsilonF;
    private int _itermax;
    private double _binf;
    private double _bsup;

    /// <summary>
    /// Finds the root of <paramref name="f"/> from <paramref name="guess"/>, inside [A, B]. The solution is found when
    /// abs(Xi - Xi-1) &lt;= EpsX and abs(F(Xi)) &lt;= EpsF.
    /// </summary>
    public NewtonFunctionRoot(FunctionWithDerivative f, double guess, double epsX, double epsF,
                                   double a, double b, int nbIterations = 100)
    {
        _epsilonX = epsX;
        _epsilonF = epsF;
        _binf = a;
        _bsup = b;
        _itermax = nbIterations;
        _done = false;
        _x = RealLast();
        _dFx = 0;
        _fx = RealLast();
        _it = 0;
        Perform(f, guess);
    }

    /// <summary>Initializes the fields, for a sub-class; <see cref="Perform"/> does the job.</summary>
    public NewtonFunctionRoot(double a, double b, double epsX, double epsF, int nbIterations = 100)
    {
        _binf = a;
        _bsup = b;
        _epsilonX = epsX;
        _epsilonF = epsF;
        _itermax = nbIterations;
        _done = false;
        _x = RealLast();
        _dFx = 0;
        _fx = RealLast();
        _it = 0;
    }

    /// <summary>
    /// Finds the root of <paramref name="f"/> from <paramref name="guess"/>. The solution is found when
    /// abs(Xi - Xi-1) &lt;= EpsX and abs(F(Xi)) &lt;= EpsF.
    /// </summary>
    public NewtonFunctionRoot(FunctionWithDerivative f, double guess, double epsX, double epsF, int nbIterations = 100)
    {
        _epsilonX = epsX;
        _epsilonF = epsF;
        _itermax = nbIterations;
        _binf = RealFirst();
        _bsup = RealLast();
        _done = false;
        _x = RealLast();
        _dFx = 0;
        _fx = RealLast();
        _it = 0;
        Perform(f, guess);
    }

    /// <summary>Performs the job; used by the constructors.</summary>
    public void Perform(FunctionWithDerivative f, double guess)
    {
        double dx;
        bool ok;
        double aa, bb;

        //--------------------------------------------------
        //-- lbr le 12 Nov 97
        //-- la meilleure estimation n est pas sauvee et on
        //-- renvoie une solution plus fausse que Guess
        double bestX = _x, bestFx = RealLast();
        //--

        if (_binf < _bsup)
        {
            aa = _binf;
            bb = _bsup;
        }
        else
        {
            aa = _bsup;
            bb = _binf;
        }

        dx = RealLast();
        _fx = RealLast();
        _x = guess;
        _it = 1;
        while ((_it <= _itermax) && ((Math.Abs(dx) > _epsilonX) || (Math.Abs(_fx) > _epsilonF)))
        {
            ok = f.Values(_x, out _fx, out _dFx);

            double absFx = _fx;
            if (absFx < 0)
            {
                absFx = -absFx;
            }

            if (absFx < bestFx)
            {
                bestFx = absFx;
                bestX = _x;
            }

            if (ok)
            {
                if (_dFx == 0.0)
                {
                    _done = false;
                    _it = _itermax + 1;
                }
                else
                {
                    dx = _fx / _dFx;
                    _x -= dx;
                    // Limitation des variations de X:
                    if (_x <= aa)
                    {
                        _x = aa;
                    }

                    if (_x >= bb)
                    {
                        _x = bb;
                    }

                    _it++;
                }
            }
            else
            {
                _done = false;
                _it = _itermax + 1;
            }
        }

        _x = bestX;

        _done = _it <= _itermax;
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
        return _dFx;
    }

    /// <summary>The value of the function at the root. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _fx;
    }

    /// <summary>The number of iterations done. Throws <see cref="NotDoneException"/> if the root was not found.</summary>
    public int IterationCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _it;
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_NewtonFunctionRoot ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Location found = " + _x.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" function value at this minimum = " + _fx.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Number of iterations = " + _it + "\n");
        }
        else
        {
            o.Write("Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
