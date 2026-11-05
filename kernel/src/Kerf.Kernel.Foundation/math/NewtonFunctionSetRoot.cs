// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionSetRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionSetRoot.lxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionSetRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;
using static Kerf.Engine.Numerics.Recipes;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The root of a set of N functions of N variables, from an initial guess, by the Newton-Raphson algorithm. The
/// Jacobian is required.
/// </summary>
internal class NewtonFunctionSetRoot
{
    protected math_Vector _tolX;
    protected double _tolF;
    protected math_IntegerVector _indx;
    protected math_Vector _scratch;
    protected math_Vector _sol;
    protected math_Vector _deltaX;
    protected math_Vector _fValues;
    protected Matrix _jacobian;

    private bool _done;
    private int _state;
    private int _iter;
    private int _itermax;

    /// <summary>
    /// Prepares the algorithm. The vectors and matrices of the function must use the range (1, F.NbVariables()).
    /// </summary>
    public NewtonFunctionSetRoot(FunctionSetWithDerivatives function, math_Vector xTolerance,
                                      double fTolerance, int nbIterations = 100)
        : this(function, fTolerance, nbIterations)
    {
        SetTolerance(xTolerance);
    }

    /// <summary>
    /// Prepares the algorithm, for a sub-class; <see cref="SetTolerance"/> must be called before performing it. The
    /// vectors and matrices of the function must use the range (1, F.NbVariables()).
    /// </summary>
    public NewtonFunctionSetRoot(FunctionSetWithDerivatives function, double fTolerance,
                                      int nbIterations = 100)
    {
        _tolX = new math_Vector(1, function.NbVariables());
        _tolF = fTolerance;
        _indx = new math_IntegerVector(1, function.NbVariables());
        _scratch = new math_Vector(1, function.NbVariables());
        _sol = new math_Vector(1, function.NbVariables());
        _deltaX = new math_Vector(1, function.NbVariables());
        _fValues = new math_Vector(1, function.NbVariables());
        _jacobian = new Matrix(1, function.NbVariables(), 1, function.NbVariables());
        _done = false;
        _state = 0;
        _iter = 0;
        _itermax = nbIterations;
    }

    /// <summary>Sets the tolerances on the unknowns.</summary>
    public void SetTolerance(math_Vector xTolerance)
    {
        for (int i = 1; i <= _tolX.Length(); ++i)
        {
            _tolX[i] = xTolerance[i];
        }
    }

    /// <summary>
    /// Improves the root from the initial guess by Newton's method. The solution is found when
    /// abs(Xj - Xj-1)(i) &lt;= XTol(i) and abs(Fi) &lt;= FTol for all i.
    /// </summary>
    public void Perform(FunctionSetWithDerivatives function, math_Vector startingPoint)
    {
        math_Vector inf = new(1, function.NbVariables(), RealFirst());
        math_Vector sup = new(1, function.NbVariables(), RealLast());
        Perform(function, startingPoint, inf, sup);
    }

    /// <summary>
    /// Improves the root from the initial guess by Newton's method, keeping the solution within the bounds. The
    /// solution is found when abs(Xj - Xj-1)(i) &lt;= XTol(i) and abs(Fi) &lt;= FTol for all i.
    /// </summary>
    public void Perform(FunctionSetWithDerivatives f, math_Vector startingPoint, math_Vector infBound,
                        math_Vector supBound)
    {
        bool ok;
        int error;

        _done = false;
        _sol.Initialized(startingPoint);
        ok = f.Values(_sol, _fValues, _jacobian);
        if (!ok)
        {
            return;
        }

        for (_iter = 1; _iter <= _itermax; _iter++)
        {
            for (int k = 1; k <= _deltaX.Length(); k++)
            {
                _deltaX[k] = -_fValues[k];
            }

            error = LU_Decompose(_jacobian, _indx, out _, _scratch, 1.0e-30);
            if (error != 0)
            {
                return;
            }

            LU_Solve(_jacobian, _indx, _deltaX);
            for (int i = 1; i <= _sol.Length(); i++)
            {
                _sol[i] += _deltaX[i];

                // Limitation de Sol dans les bornes [InfBound, SupBound] :
                if (_sol[i] <= infBound[i])
                {
                    _sol[i] = infBound[i];
                }

                if (_sol[i] >= supBound[i])
                {
                    _sol[i] = supBound[i];
                }
            }

            ok = f.Values(_sol, _fValues, _jacobian);
            if (!ok)
            {
                return;
            }

            if (IsSolutionReached(f))
            {
                _state = f.GetStateNumber();
                _done = true;
                return;
            }
        }
    }

    /// <summary>
    /// Called at the end of each iteration to check if the solution is found. DeltaX, FValues and Jacobian are
    /// consistent with Sol and can be inspected by a redefinition.
    /// </summary>
    public virtual bool IsSolutionReached(FunctionSetWithDerivatives f)
    {
        for (int i = _deltaX.Lower(); i <= _deltaX.Upper(); ++i)
        {
            if (Math.Abs(_deltaX[i]) > _tolX[i] || Math.Abs(_fValues[i]) > _tolF)
            {
                return false;
            }
        }

        return true;
    }

    public bool IsDone => _done;

    /// <summary>The root (shared: don't modify). Throws <see cref="NotDoneException"/> if not done.</summary>
    public math_Vector Root()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _sol;
    }

    /// <summary>Sets <paramref name="root"/> to the root. Throws <see cref="NotDoneException"/> if not done.</summary>
    public void Root(math_Vector root)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        root.Initialized(_sol);
    }

    /// <summary>The state number associated with the root.</summary>
    public int StateNumber() => _state;

    /// <summary>The derivative matrix at the root (shared: don't modify). Throws <see cref="NotDoneException"/> if not done.</summary>
    public Matrix Derivative()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _jacobian;
    }

    /// <summary>Sets <paramref name="der"/> to the derivative matrix at the root. Throws <see cref="NotDoneException"/> if not done.</summary>
    public void Derivative(Matrix der)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        der.Initialized(_jacobian);
    }

    /// <summary>The errors on the functions at the root (shared: don't modify). Throws <see cref="NotDoneException"/> if not done.</summary>
    /// <remarks>OCCT returns the last Newton step DeltaX, not the function values.</remarks>
    public math_Vector FunctionSetErrors()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _deltaX; // OCCT returns the last step DeltaX, not the function values FValues
    }

    /// <summary>Sets <paramref name="err"/> to the errors on the functions at the root. Throws <see cref="NotDoneException"/> if not done.</summary>
    public void FunctionSetErrors(math_Vector err)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        err.Initialized(_deltaX); // OCCT returns the last step DeltaX, not the function values FValues
    }

    /// <summary>The number of iterations done. Throws <see cref="NotDoneException"/> if not done.</summary>
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
        o.Write("math_NewtonFunctionSetRoot ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Vector solution = " + _sol + "\n");
            o.Write(" Value of the function at this solution = \n");
            o.Write(_fValues + "\n");
            o.Write(" Number of iterations = " + _iter + "\n");
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
