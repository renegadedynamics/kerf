// Port of: src/FoundationClasses/TKMath/math/math_Powell.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Powell.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Powell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;
using static Kerf.Engine.Numerics.NumericStatus;

namespace Kerf.Engine.Numerics;

/// <summary>Powell's method to find a minimum of a function of several variables, without its gradient.</summary>
internal class Powell
{
    protected math_Vector _theLocation;
    protected double _theMinimum;
    protected double _theLocationError;
    protected double _previousMinimum;
    protected double _xTol;
    protected double _epsz;

    private bool _done;
    private int _iter;
    private NumericStatus _theStatus;
    private Matrix _theDirections;
    private int _state;
    private int _itermax;

    private static double SQR(double a) => a * a;

    // The function along a direction: F(P0 + x * Dir).
    private sealed class DirFunctionBis : Function
    {
        private readonly math_Vector _p0;
        private readonly math_Vector _dir;
        private readonly math_Vector _p;
        private readonly MultipleVarFunction _f;

        public DirFunctionBis(math_Vector v1, math_Vector v2, math_Vector v3, MultipleVarFunction f)
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

    private static bool MinimizeDirection(math_Vector p, math_Vector dir, ref double result, DirFunctionBis f)
    {
        double ax;
        double xx;
        double bx;

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
    public Powell(MultipleVarFunction function, double tolerance, int nbIterations = 200, double zeps = 1.0e-12)
    {
        _theLocation = new math_Vector(1, function.VariableCount);
        _theMinimum = RealLast();
        _theLocationError = RealLast();
        _previousMinimum = RealLast();
        _xTol = tolerance;
        _epsz = zeps;
        _done = false;
        _iter = 0;
        _theStatus = NotBracketed;
        _theDirections = new Matrix(1, function.VariableCount, 1, function.VariableCount);
        _state = 0;
        _itermax = nbIterations;
    }

    /// <summary>
    /// Minimizes F from <paramref name="startingPoint"/>, the columns of <paramref name="startingDirections"/> being
    /// the initial directions. The solution F = Fi is found when
    /// 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS).
    /// </summary>
    public void Perform(MultipleVarFunction f, math_Vector startingPoint, Matrix startingDirections)
    {
        _done = false;
        int i, ibig, j;
        double t, fptt, del;
        int n = _theLocation.Length();
        math_Vector pt = new(1, n);
        math_Vector ptt = new(1, n);
        math_Vector xit = new(1, n);
        math_Vector temp1 = new(1, n);
        math_Vector temp2 = new(1, n);
        math_Vector temp3 = new(1, n);
        DirFunctionBis F_Dir = new(temp1, temp2, temp3, f);

        _theLocation.Initialized(startingPoint);
        _theDirections.Initialized(startingDirections);
        pt.Initialized(_theLocation); // sauvegarde du point initial

        for (_iter = 1; _iter <= _itermax; _iter++)
        {
            f.Value(_theLocation, out _previousMinimum);
            ibig = 0;
            del = 0.0;
            for (i = 1; i <= n; i++)
            {
                for (j = 1; j <= n; j++)
                {
                    xit[j] = _theDirections[j, i];
                }

                f.Value(_theLocation, out fptt);
                bool isGood = MinimizeDirection(_theLocation, xit, ref _theMinimum, F_Dir);

                if (!isGood)
                {
                    _done = false;
                    _theStatus = DirectionSearchError;
                    return;
                }

                if (Math.Abs(fptt - _theMinimum) > del)
                {
                    del = Math.Abs(fptt - _theMinimum);
                    ibig = i;
                }
            }

            if (IsSolutionReached(f))
            {
                // Termination criterion
                _state = f.GetStateNumber();
                _done = true;
                _theStatus = OK;
                return;
            }

            if (_iter == _itermax)
            {
                _done = false;
                _theStatus = TooManyIterations;
                return;
            }

            ptt.Initialized(2.0 * _theLocation - pt);
            xit.Initialized(_theLocation - pt);
            pt.Initialized(_theLocation);

            // Valeur de la fonction au point extrapole:
            f.Value(ptt, out fptt);

            if (fptt < _previousMinimum)
            {
                t = 2.0 * (_previousMinimum - 2.0 * _theMinimum + fptt) * SQR(_previousMinimum - _theMinimum - del)
                    - del * SQR(_previousMinimum - fptt);
                if (t < 0.0)
                {
                    // Minimisation along the direction
                    bool isGood = MinimizeDirection(_theLocation, xit, ref _theMinimum, F_Dir);
                    if (!isGood)
                    {
                        _done = false;
                        _theStatus = FunctionError;
                        return;
                    }

                    for (j = 1; j <= n; j++)
                    {
                        _theDirections[j, ibig] = xit[j];
                    }
                }
            }
        }
    }

    /// <summary>
    /// True when 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS). A sub-class can redefine it.
    /// </summary>
    public virtual bool IsSolutionReached(MultipleVarFunction function)
    {
        return 2.0 * Math.Abs(_previousMinimum - _theMinimum) <= _xTol * (Math.Abs(_previousMinimum) + Math.Abs(_theMinimum) + _epsz);
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
        o.Write("math_Powell resolution:");
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
