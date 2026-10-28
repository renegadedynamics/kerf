// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoot.lxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A root of a function of a single variable near an initial guess, by a minimization algorithm
/// (<see cref="math_FunctionSetRoot"/>). The derivative is required.
/// </summary>
public class math_FunctionRoot
{
    private bool Done;
    private double TheRoot;
    private double TheError;
    private double TheDerivative;
    private int NbIter;

    /// <summary>The function as a set of one function of one variable (math_MyFunctionSetWithDerivatives).</summary>
    private sealed class math_MyFunctionSetWithDerivatives : math_FunctionSetWithDerivatives
    {
        private math_FunctionWithDerivative Ff;

        public math_MyFunctionSetWithDerivatives(math_FunctionWithDerivative F)
        {
            Ff = F;
        }

        public override int NbVariables()
        {
            return 1;
        }

        public override int NbEquations()
        {
            return 1;
        }

        public override bool Value(math_Vector X, math_Vector Fs)
        {
            return Ff.Value(X[1], out Fs[1]);
        }

        public override bool Derivatives(math_Vector X, math_Matrix D)
        {
            return Ff.Derivative(X[1], out D[1, 1]);
        }

        public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
        {
            return Ff.Values(X[1], out F[1], out D[1, 1]);
        }
    }

    /// <summary>
    /// Finds a root of <paramref name="F"/> from <paramref name="Guess"/>. The solution is found when
    /// abs(Xi - Xi-1) &lt;= Tolerance.
    /// </summary>
    public math_FunctionRoot(math_FunctionWithDerivative F, double Guess, double Tolerance, int NbIterations = 100)
    {
        math_Vector V = new(1, 1), Tol = new(1, 1);
        math_MyFunctionSetWithDerivatives Ff = new(F);
        V[1] = Guess;
        Tol[1] = Tolerance;
        math_FunctionSetRoot Sol = new(Ff, Tol, NbIterations);
        Sol.Perform(Ff, V);
        Done = Sol.IsDone();
        if (Done)
        {
            NbIter = Sol.NbIterations();
            F.GetStateNumber();
            TheRoot = Sol.Root()[1];
            TheDerivative = Sol.Derivative()[1, 1];
            F.Value(TheRoot, out TheError);
        }
    }

    /// <summary>
    /// Finds a root of <paramref name="F"/> from <paramref name="Guess"/>, stopping if the solution leaves [A, B]. The
    /// solution is found when abs(Xi - Xi-1) &lt;= Tolerance.
    /// </summary>
    public math_FunctionRoot(math_FunctionWithDerivative F, double Guess, double Tolerance, double A, double B,
                             int NbIterations = 100)
    {
        math_Vector V = new(1, 1), Aa = new(1, 1), Bb = new(1, 1), Tol = new(1, 1);
        math_MyFunctionSetWithDerivatives Ff = new(F);
        V[1] = Guess;
        Tol[1] = Tolerance;
        Aa[1] = A;
        Bb[1] = B;
        math_FunctionSetRoot Sol = new(Ff, Tol, NbIterations);
        Sol.Perform(Ff, V, Aa, Bb);
        Done = Sol.IsDone();
        if (Done)
        {
            NbIter = Sol.NbIterations();
            F.GetStateNumber();
            TheRoot = Sol.Root()[1];
            TheDerivative = Sol.Derivative()[1, 1];
            F.Value(TheRoot, out TheError);
        }
    }

    public bool IsDone() => Done;

    /// <summary>The root. Throws <see cref="StdFail_NotDone"/> if not found.</summary>
    public double Root()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return TheRoot;
    }

    /// <summary>The derivative at the root. Throws <see cref="StdFail_NotDone"/> if not found.</summary>
    public double Derivative()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return TheDerivative;
    }

    /// <summary>The value of the function at the root. Throws <see cref="StdFail_NotDone"/> if not found.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return TheError;
    }

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if not found.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return NbIter;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_FunctionRoot ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of iterations = " + NbIter + '\n');
            o.Write(" The Root is: " + TheRoot.ToString(CultureInfo.InvariantCulture) + '\n');
            o.Write("The value at the root is: " + TheError.ToString(CultureInfo.InvariantCulture) + '\n');
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
