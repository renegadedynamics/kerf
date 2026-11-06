// Port of: src/FoundationClasses/TKMath/math/math_BFGS.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BFGS.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BFGS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using static Kerf.Engine.Numerics.NumericStatus;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The Broyden-Fletcher-Goldfarb-Shanno variant of the Davidson-Fletcher-Powell method to find a minimum of a
/// function of several variables whose gradient is known. Optionally bounded (<see cref="SetBoundary"/>).
/// </summary>
internal class Bfgs
{
    protected NumericStatus _theStatus;
    protected math_Vector _theLocation;
    protected math_Vector _theGradient;
    protected double _previousMinimum;
    protected double _theMinimum;
    protected double _xTol;
    protected double _epsz;
    protected int _nbiter;
    protected bool _isBoundsDefined;
    protected math_Vector _left;
    protected math_Vector _right;

    private bool _done;
    private int _itermax;

    // l'utilisation de math_BrentMinumim pur trouver un minimum dans une direction
    // donnee n'est pas du tout optimale. voir peut etre interpolation cubique
    // classique et aussi essayer "recherche unidimensionnelle economique"
    // PROGRAMMATION MATHEMATIQUE (theorie et algorithmes) tome1 page 82.

    // Target function for 1D problem, point and direction are known.
    private sealed class DirFunction : FunctionWithDerivative
    {
        private readonly math_Vector _p0;
        private readonly math_Vector _dir;
        private readonly math_Vector _p;
        private readonly math_Vector _g;
        private readonly MultipleVarFunctionWithGradient _f;

        public DirFunction(math_Vector v1, math_Vector v2, math_Vector v3, math_Vector v4, MultipleVarFunctionWithGradient f)
        {
            _p0 = v1;
            _dir = v2;
            _p = v3;
            _g = v4;
            _f = f;
        }

        public void Initialize(math_Vector p0, math_Vector dir)
        {
            _p0.Initialized(p0);
            _dir.Initialized(dir);
        }

        public void TheGradient(math_Vector grad) => grad.Initialized(_g);

        public override bool Value(double x, out double fval)
        {
            _p.Initialized(_dir);
            _p.Multiply(x);
            _p.Add(_p0);
            fval = 0.0;
            return _f.Value(_p, out fval);
        }

        public override bool Values(double x, out double fval, out double d)
        {
            _p.Initialized(_dir);
            _p.Multiply(x);
            _p.Add(_p0);
            fval = d = 0.0;
            if (_f.Values(_p, out fval, _g))
            {
                d = _g.Multiplied(_dir);
                return true;
            }

            return false;
        }

        public override bool Derivative(double x, out double d)
        {
            _p.Initialized(_dir);
            _p.Multiply(x);
            _p.Add(_p0);
            double fval;
            d = 0.0;
            if (_f.Values(_p, out fval, _g))
            {
                d = _g.Multiplied(_dir);
                return true;
            }

            return false;
        }
    }

    // Computes the appropriate initial value of scale factor to apply
    // to the direction to approach to the minimum of the function
    private static bool ComputeInitScale(double f0, math_Vector dir, math_Vector gr, out double scale)
    {
        scale = 0.0;
        double dy1 = gr * dir;
        if (Math.Abs(dy1) < RealSmall())
        {
            return false;
        }

        double hnr1 = dir.Norm2();
        double alfa = 0.7 * (-f0) / dy1;
        scale = 0.015 / Math.Sqrt(hnr1);
        if (scale > alfa)
        {
            scale = alfa;
        }

        return true;
    }

    // For a given point and direction, and bounding box, finds the min and max scale factors with which the point
    // reaches the borders if we apply translation Point+Dir*Scale. False if the point is out of bounds.
    private static bool ComputeMinMaxScale(math_Vector point, math_Vector dir, math_Vector left, math_Vector right,
                                           ref double minScale, ref double maxScale)
    {
        for (int idx = 1; idx <= left.Upper(); idx++)
        {
            double aLeft = left[idx] - point[idx];
            double aRight = right[idx] - point[idx];
            if (Math.Abs(dir[idx]) > RealSmall())
            {
                // Use PConfusion to get off a little from the bounds to prevent
                // possible refuse in Value function.
                double lScale = (aLeft + global::Kerf.Engine.Foundation.Precision.PConfusion()) / dir[idx];
                double rScale = (aRight - global::Kerf.Engine.Foundation.Precision.PConfusion()) / dir[idx];
                if (Math.Abs(aLeft) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    // Point is on the left border.
                    maxScale = Min(maxScale, Max(0.0, rScale));
                    minScale = Max(minScale, Min(0.0, rScale));
                }
                else if (Math.Abs(aRight) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    // Point is on the right border.
                    maxScale = Min(maxScale, Max(0.0, lScale));
                    minScale = Max(minScale, Min(0.0, lScale));
                }
                else if (aLeft * aRight < 0)
                {
                    // Point is inside allowed range.
                    maxScale = Min(maxScale, Max(lScale, rScale));
                    minScale = Max(minScale, Min(lScale, rScale));
                }
                else
                {
                    // point is out of bounds
                    return false;
                }
            }
            else
            {
                // Direction is parallel to the border.
                // Check that the point is not out of bounds
                if (aLeft > global::Kerf.Engine.Foundation.Precision.PConfusion() || aRight < -global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
            }
        }

        return true;
    }

    // Solves 1D minimization problem when point and directions are known.
    private static bool MinimizeDirection(math_Vector p, double f0, math_Vector gr, math_Vector dir, ref double result,
                                          DirFunction f, bool isBounds, math_Vector left, math_Vector right)
    {
        double lambda;
        if (!ComputeInitScale(f0, dir, gr, out lambda))
        {
            return false;
        }

        // by default the scaling range is unlimited
        double minLambda = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double maxLambda = global::Kerf.Engine.Foundation.Precision.Infinite();
        if (isBounds)
        {
            // limit the scaling range taking into account the bounds
            if (!ComputeMinMaxScale(p, dir, left, right, ref minLambda, ref maxLambda))
            {
                return false;
            }

            if (minLambda > -global::Kerf.Engine.Foundation.Precision.PConfusion() && maxLambda < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                // Point is on the border and the direction shows outside.
                // Make direction to go along the border
                for (int idx = 1; idx <= left.Upper(); idx++)
                {
                    if ((Math.Abs(p[idx] - right[idx]) < global::Kerf.Engine.Foundation.Precision.PConfusion() && dir[idx] > 0.0)
                        || (Math.Abs(p[idx] - left[idx]) < global::Kerf.Engine.Foundation.Precision.PConfusion() && dir[idx] < 0.0))
                    {
                        dir[idx] = 0.0;
                    }
                }

                // re-compute scale values with new direction
                if (!ComputeInitScale(f0, dir, gr, out lambda))
                {
                    return false;
                }

                if (!ComputeMinMaxScale(p, dir, left, right, ref minLambda, ref maxLambda))
                {
                    return false;
                }
            }

            lambda = Min(lambda, maxLambda);
            lambda = Max(lambda, minLambda);
        }

        f.Initialize(p, dir);
        double f1;
        if (!f.Value(lambda, out f1))
        {
            return false;
        }

        BracketMinimum bracket = new(0.0, lambda);
        if (isBounds)
        {
            bracket.SetLimits(minLambda, maxLambda);
        }

        bracket.SetFA(f0);
        bracket.SetFB(f1);
        bracket.Perform(f);
        if (bracket.IsDone)
        {
            // find minimum inside the bracket
            double ax, xx, bx, fax, fxx, fbx;
            bracket.Values(out ax, out xx, out bx);
            bracket.FunctionValues(out fax, out fxx, out fbx);

            int niter = 100;
            double tol = 1.0e-03;
            BrentMinimum sol = new(tol, fxx, niter, 1.0e-08);
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
        else if (isBounds)
        {
            // Bracket definition is failure. If the bounds are defined then
            // set current point to intersection with bounds
            double fMin, fMax;
            if (!f.Value(minLambda, out fMin))
            {
                return false;
            }

            if (!f.Value(maxLambda, out fMax))
            {
                return false;
            }

            double bestLambda;
            if (fMin < fMax)
            {
                bestLambda = minLambda;
                result = fMin;
            }
            else
            {
                bestLambda = maxLambda;
                result = fMax;
            }

            dir.Multiply(bestLambda);
            p.Add(dir);
            return true;
        }

        return false;
    }

    /// <summary>Prepares the algorithm; <see cref="Perform"/> does the job.</summary>
    public Bfgs(int nbVariables, double tolerance = 1.0e-8, int nbIterations = 200, double zeps = 1.0e-12)
    {
        _theStatus = OK;
        _theLocation = new math_Vector(1, nbVariables);
        _theGradient = new math_Vector(1, nbVariables);
        _previousMinimum = 0.0;
        _theMinimum = 0.0;
        _xTol = tolerance;
        _epsz = zeps;
        _nbiter = 0;
        _isBoundsDefined = false;
        _left = new math_Vector(1, nbVariables, 0.0);
        _right = new math_Vector(1, nbVariables, 0.0);
        _done = false;
        _itermax = nbIterations;
    }

    /// <summary>Sets the boundaries for conditional optimization; the starting point must be within them.</summary>
    public void SetBoundary(math_Vector leftBorder, math_Vector rightBorder)
    {
        _left.Initialized(leftBorder);
        _right.Initialized(rightBorder);
        _isBoundsDefined = true;
    }

    /// <summary>Minimizes F from <paramref name="startingPoint"/>.</summary>
    public void Perform(MultipleVarFunctionWithGradient f, math_Vector startingPoint)
    {
        int n = _theLocation.Length();
        Work work = Work.Rent(n);
        try
        {
            PerformWith(work, f, startingPoint);
        }
        finally
        {
            Work.Return(work);
        }
    }

    // The search's vectors and its n by n matrix: one set kept by each thread for the size it last used, as an approximation runs
    // a minimisation for each fit of the same size. Each search starts from zeros, as with new ones
    private sealed class Work
    {
        private const int LargestKept = 512;

        [ThreadStatic]
        private static Work? t_spare;

        public readonly int Size;
        public readonly math_Vector Xi, Dg, Hdg, Temp1, Temp2, Temp3, Temp4;
        public readonly Matrix Hessin;

        private Work(int n)
        {
            Size = n;
            Xi = new(1, n);
            Dg = new(1, n);
            Hdg = new(1, n);
            Temp1 = new(1, n);
            Temp2 = new(1, n);
            Temp3 = new(1, n);
            Temp4 = new(1, n);
            Hessin = new(1, n, 1, n);
        }

        public static Work Rent(int n)
        {
            Work? work = t_spare;
            t_spare = null; // A search inside a search takes its own
            if (work is null || work.Size != n)
            {
                return new Work(n);
            }

            work.Xi.Init(0.0);
            work.Dg.Init(0.0);
            work.Hdg.Init(0.0);
            work.Temp1.Init(0.0);
            work.Temp2.Init(0.0);
            work.Temp3.Init(0.0);
            work.Temp4.Init(0.0);
            return work;
        }

        public static void Return(Work work)
        {
            if (work.Size <= LargestKept)
            {
                t_spare = work;
            }
        }
    }

    private void PerformWith(Work work, MultipleVarFunctionWithGradient f, math_Vector startingPoint)
    {
        int n = work.Size;
        bool good = true;
        int j, i;
        double fae, fad, fac;

        math_Vector xi = work.Xi, dg = work.Dg, hdg = work.Hdg;
        Matrix hessin = work.Hessin;
        hessin.Init(0.0);

        DirFunction F_Dir = new(work.Temp1, work.Temp2, work.Temp3, work.Temp4, f);

        _theLocation.Initialized(startingPoint);
        good = f.Values(_theLocation, out _previousMinimum, _theGradient);
        if (!good)
        {
            _done = false;
            _theStatus = FunctionError;
            return;
        }

        for (i = 1; i <= n; i++)
        {
            hessin[i, i] = 1.0;
            xi[i] = -_theGradient[i];
        }

        for (_nbiter = 1; _nbiter <= _itermax; _nbiter++)
        {
            _theMinimum = _previousMinimum;
            bool isGood = MinimizeDirection(_theLocation, _theMinimum, _theGradient, xi, ref _theMinimum, F_Dir,
                                            _isBoundsDefined, _left, _right);

            if (IsSolutionReached(f))
            {
                _done = true;
                _theStatus = OK;
                return;
            }

            if (!isGood)
            {
                _done = false;
                _theStatus = DirectionSearchError;
                return;
            }

            _previousMinimum = _theMinimum;

            dg.Initialized(_theGradient);

            good = f.Values(_theLocation, out _theMinimum, _theGradient);
            if (!good)
            {
                _done = false;
                _theStatus = FunctionError;
                return;
            }

            for (i = 1; i <= n; i++)
            {
                dg[i] = _theGradient[i] - dg[i];
            }

            for (i = 1; i <= n; i++)
            {
                hdg[i] = 0.0;
                for (j = 1; j <= n; j++)
                {
                    hdg[i] += hessin[i, j] * dg[j];
                }
            }

            fac = fae = 0.0;
            for (i = 1; i <= n; i++)
            {
                fac += dg[i] * xi[i];
                fae += dg[i] * hdg[i];
            }

            fac = 1.0 / fac;
            fad = 1.0 / fae;

            for (i = 1; i <= n; i++)
            {
                dg[i] = fac * xi[i] - fad * hdg[i];
            }

            for (i = 1; i <= n; i++)
            {
                for (j = 1; j <= n; j++)
                {
                    hessin[i, j] += fac * xi[i] * xi[j] - fad * hdg[i] * hdg[j] + fae * dg[i] * dg[j];
                }
            }

            for (i = 1; i <= n; i++)
            {
                xi[i] = 0.0;
                for (j = 1; j <= n; j++)
                {
                    xi[i] -= hessin[i, j] * _theGradient[j];
                }
            }
        }

        _done = false;
        _theStatus = TooManyIterations;
    }

    /// <summary>
    /// True when 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS). A sub-class can redefine it.
    /// </summary>
    public virtual bool IsSolutionReached(MultipleVarFunctionWithGradient f)
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
            return _nbiter;
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BFGS resolution: ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Location Vector = " + Location() + "\n");
            o.Write(" Minimum value = " + Minimum().ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Number of iterations = " + IterationCount + "\n");
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
