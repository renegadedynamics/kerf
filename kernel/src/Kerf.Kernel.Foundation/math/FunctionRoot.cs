// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoot.lxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// A root of a function of a single variable near an initial guess, by a minimization algorithm
/// (<see cref="FunctionSetRoot"/>). The derivative is required.
/// </summary>
internal class FunctionRoot
{
    private bool _done;
    private double _theRoot;
    private double _theError;
    private double _theDerivative;
    private int _nbIter;

    /// <summary>The function as a set of one function of one variable (MathMyFunctionSetWithDerivatives).</summary>
    private sealed class MathMyFunctionSetWithDerivatives : FunctionSetWithDerivatives
    {
        private FunctionWithDerivative _ff;

        public MathMyFunctionSetWithDerivatives(FunctionWithDerivative f)
        {
            _ff = f;
        }

        public override int NbVariables()
        {
            return 1;
        }

        public override int NbEquations()
        {
            return 1;
        }

        public override bool Value(math_Vector x, math_Vector fs)
        {
            return _ff.Value(x[1], out fs[1]);
        }

        public override bool Derivatives(math_Vector x, Matrix d)
        {
            return _ff.Derivative(x[1], out d[1, 1]);
        }

        public override bool Values(math_Vector x, math_Vector f, Matrix d)
        {
            return _ff.Values(x[1], out f[1], out d[1, 1]);
        }
    }

    /// <summary>
    /// Finds a root of <paramref name="f"/> from <paramref name="guess"/>. The solution is found when
    /// abs(Xi - Xi-1) &lt;= Tolerance.
    /// </summary>
    public FunctionRoot(FunctionWithDerivative f, double guess, double tolerance, int nbIterations = 100)
    {
        math_Vector v = new(1, 1), tol = new(1, 1);
        MathMyFunctionSetWithDerivatives ff = new(f);
        v[1] = guess;
        tol[1] = tolerance;
        FunctionSetRoot sol = new(ff, tol, nbIterations);
        sol.Perform(ff, v);
        _done = sol.IsDone;
        if (_done)
        {
            _nbIter = sol.IterationCount;
            f.GetStateNumber();
            _theRoot = sol.Root()[1];
            _theDerivative = sol.Derivative()[1, 1];
            f.Value(_theRoot, out _theError);
        }
    }

    /// <summary>
    /// Finds a root of <paramref name="f"/> from <paramref name="guess"/>, stopping if the solution leaves [A, B]. The
    /// solution is found when abs(Xi - Xi-1) &lt;= Tolerance.
    /// </summary>
    public FunctionRoot(FunctionWithDerivative f, double guess, double tolerance, double a, double b,
                             int nbIterations = 100)
    {
        math_Vector v = new(1, 1), aa = new(1, 1), bb = new(1, 1), tol = new(1, 1);
        MathMyFunctionSetWithDerivatives ff = new(f);
        v[1] = guess;
        tol[1] = tolerance;
        aa[1] = a;
        bb[1] = b;
        FunctionSetRoot sol = new(ff, tol, nbIterations);
        sol.Perform(ff, v, aa, bb);
        _done = sol.IsDone;
        if (_done)
        {
            _nbIter = sol.IterationCount;
            f.GetStateNumber();
            _theRoot = sol.Root()[1];
            _theDerivative = sol.Derivative()[1, 1];
            f.Value(_theRoot, out _theError);
        }
    }

    public bool IsDone => _done;

    /// <summary>The root. Throws <see cref="NotDoneException"/> if not found.</summary>
    public double Root()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _theRoot;
    }

    /// <summary>The derivative at the root. Throws <see cref="NotDoneException"/> if not found.</summary>
    public double Derivative()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _theDerivative;
    }

    /// <summary>The value of the function at the root. Throws <see cref="NotDoneException"/> if not found.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _theError;
    }

    /// <summary>The number of iterations done. Throws <see cref="NotDoneException"/> if not found.</summary>
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
        o.Write("math_FunctionRoot ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of iterations = " + _nbIter + '\n');
            o.Write(" The Root is: " + _theRoot.ToString(CultureInfo.InvariantCulture) + '\n');
            o.Write("The value at the root is: " + _theError.ToString(CultureInfo.InvariantCulture) + '\n');
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
