// Port of: src/FoundationClasses/TKMath/math/math_FRPR.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FRPR.lxx
// Port of: src/FoundationClasses/TKMath/math/math_FRPR.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;
using static Kerf.Engine.Numerics.NumericStatus;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The Fletcher-Reeves-Polak-Ribiere conjugate gradient method to find a minimum of a function of several
/// variables whose gradient is known.
/// </summary>
internal class Frpr
{
    protected math_Vector _theLocation;
    protected math_Vector _theGradient;
    protected double _theMinimum;
    protected double _previousMinimum;
    protected double _xTol;
    protected double _epsz;

    private bool _done;
    private int _iter;
    private int _state;
    private NumericStatus _theStatus;
    private int _itermax;

    // l'utilisation de math_BrentMinumim pur trouver un minimum dans une direction
    // donnee n'est pas du tout optimale. voir peut etre interpolation cubique
    // classique et aussi essayer "recherche unidimensionnelle economique"
    // PROGRAMMATION MATHEMATIQUE (theorie et algorithmes) tome1 page 82.

    // The function along a direction: F(P0 + x * Dir).
    private sealed class DirFunctionTer : Function
    {
        private readonly math_Vector _p0;
        private readonly math_Vector _dir;
        private readonly math_Vector _p;
        private readonly MultipleVarFunction _f;

        public DirFunctionTer(math_Vector v1, math_Vector v2, math_Vector v3, MultipleVarFunction f)
        {
            _p0 = v1;
            _dir = v2;
            _p = v3;
            _f = f;
        }

        public void Initialize(math_Vector p0, math_Vector dir)
        {
            _p0.Initialized(p0);
            _dir.Initialized(dir);
        }

        public override bool Value(double x, out double fval)
        {
            _p.Initialized(_dir);
            _p.Multiply(x);
            _p.Add(_p0);
            fval = 0.0;
            return _f.Value(_p, out fval);
        }
    }

    private static bool MinimizeDirection(math_Vector p, math_Vector dir, ref double result, DirFunctionTer f)
    {
        double ax, xx, bx;

        f.Initialize(p, dir);
        BracketMinimum bracket = new(f, 0.0, 1.0);
        if (bracket.IsDone)
        {
            bracket.Values(out ax, out xx, out bx);
            BrentMinimum sol = new(1.0e-10);
            sol.Perform(f, ax, xx, bx);
            if (sol.IsDone)
            {
                double scale = sol.Location();
                result = sol.Minimum();
                dir.Multiply(scale);
                p.Add(dir);
                return true;
            }
        }

        return false;
    }

    /// <summary>Prepares the algorithm; <see cref="Perform"/> does the job.</summary>
    public Frpr(MultipleVarFunctionWithGradient function, double tolerance, int nbIterations = 200, double zeps = 1.0e-12)
    {
        _theLocation = new math_Vector(1, function.VariableCount);
        _theGradient = new math_Vector(1, function.VariableCount);
        _theMinimum = 0.0;
        _previousMinimum = 0.0;
        _xTol = tolerance;
        _epsz = zeps;
        _done = false;
        _iter = 0;
        _state = 0;
        _theStatus = NotBracketed;
        _itermax = nbIterations;
    }

    /// <summary>
    /// Minimizes F from <paramref name="startingPoint"/>. The solution F = Fi is found when
    /// 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS).
    /// </summary>
    public void Perform(MultipleVarFunctionWithGradient f, math_Vector startingPoint)
    {
        bool good;
        int n = _theLocation.Length();
        int j, its;
        double gg, gam, dgg;

        math_Vector g = new(1, n), h = new(1, n);

        math_Vector temp1 = new(1, n);
        math_Vector temp2 = new(1, n);
        math_Vector temp3 = new(1, n);
        DirFunctionTer F_Dir = new(temp1, temp2, temp3, f);

        _theLocation.Initialized(startingPoint);
        good = f.Values(_theLocation, out _previousMinimum, _theGradient);
        if (!good)
        {
            _done = false;
            _theStatus = FunctionError;
            return;
        }

        g.Initialized(-_theGradient);
        h.Initialized(g);
        _theGradient.Initialized(g);

        for (its = 1; its <= _itermax; its++)
        {
            _iter = its;

            bool isGood = MinimizeDirection(_theLocation, _theGradient, ref _theMinimum, F_Dir);
            if (IsSolutionReached(f))
            {
                _done = true;
                _state = f.GetStateNumber();
                _theStatus = OK;
                return;
            }

            if (!isGood)
            {
                _done = false;
                _theStatus = DirectionSearchError;
                return;
            }

            good = f.Values(_theLocation, out _previousMinimum, _theGradient);
            if (!good)
            {
                _done = false;
                _theStatus = FunctionError;
                return;
            }

            dgg = 0.0;
            gg = 0.0;

            for (j = 1; j <= n; j++)
            {
                gg += g[j] * g[j];
                //	   dgg += TheGradient(j)*TheGradient(j);  //for Fletcher-Reeves
                dgg += (_theGradient[j] + g[j]) * _theGradient[j]; // for Polak-Ribiere
            }

            if (gg == 0.0)
            {
                // Unlikely. If gradient is exactly 0 then we are already done.
                _done = false;
                _theStatus = FunctionError;
                return;
            }

            gam = dgg / gg;
            g.Initialized(-_theGradient);
            _theGradient.Initialized(g + gam * h);
            h.Initialized(_theGradient);
        }

        _done = false;
        _theStatus = TooManyIterations;
    }

    /// <summary>
    /// True when 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS). A sub-class can redefine it.
    /// </summary>
    public virtual bool IsSolutionReached(MultipleVarFunctionWithGradient function)
    {
        return 2.0 * Math.Abs(_theMinimum - _previousMinimum) <= _xTol * (Math.Abs(_theMinimum) + Math.Abs(_previousMinimum) + _epsz);
    }

    public bool IsDone => _done;

    /// <summary>The location of the minimum (shared: don't modify). Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public math_Vector Location()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _theLocation;
    }

    /// <summary>Copies the location of the minimum into <paramref name="loc"/>.</summary>
    public void Location(math_Vector loc)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        loc.Initialized(_theLocation);
    }

    /// <summary>The value of the minimum. Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public double Minimum()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _theMinimum;
    }

    /// <summary>The gradient at the minimum (shared: don't modify). Throws <see cref="NotDoneException"/> if it was not found.</summary>
    public math_Vector Gradient()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _theGradient;
    }

    /// <summary>Copies the gradient at the minimum into <paramref name="grad"/>.</summary>
    public void Gradient(math_Vector grad)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        grad.Initialized(_theGradient);
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
        o.Write("math_FRPR ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Location Vector = " + _theLocation + "\n");
            o.Write(" Minimum value = " + _theMinimum.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Number of iterations = " + _iter + "\n");
        }
        else
        {
            o.Write(" Status = not Done because " + (int)_theStatus + "\n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
