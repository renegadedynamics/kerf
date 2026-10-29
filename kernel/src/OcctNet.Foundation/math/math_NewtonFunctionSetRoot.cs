// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionSetRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionSetRoot.lxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionSetRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Recipes;

namespace Occt;

/// <summary>
/// The root of a set of N functions of N variables, from an initial guess, by the Newton-Raphson algorithm. The
/// Jacobian is required.
/// </summary>
public class math_NewtonFunctionSetRoot
{
    protected math_Vector TolX;
    protected double TolF;
    protected math_IntegerVector Indx;
    protected math_Vector Scratch;
    protected math_Vector Sol;
    protected math_Vector DeltaX;
    protected math_Vector FValues;
    protected math_Matrix Jacobian;

    private bool Done;
    private int State;
    private int Iter;
    private int Itermax;

    /// <summary>
    /// Prepares the algorithm. The vectors and matrices of the function must use the range (1, F.NbVariables()).
    /// </summary>
    public math_NewtonFunctionSetRoot(math_FunctionSetWithDerivatives theFunction, math_Vector theXTolerance,
                                      double theFTolerance, int theNbIterations = 100)
        : this(theFunction, theFTolerance, theNbIterations)
    {
        SetTolerance(theXTolerance);
    }

    /// <summary>
    /// Prepares the algorithm, for a sub-class; <see cref="SetTolerance"/> must be called before performing it. The
    /// vectors and matrices of the function must use the range (1, F.NbVariables()).
    /// </summary>
    public math_NewtonFunctionSetRoot(math_FunctionSetWithDerivatives theFunction, double theFTolerance,
                                      int theNbIterations = 100)
    {
        TolX = new math_Vector(1, theFunction.NbVariables());
        TolF = theFTolerance;
        Indx = new math_IntegerVector(1, theFunction.NbVariables());
        Scratch = new math_Vector(1, theFunction.NbVariables());
        Sol = new math_Vector(1, theFunction.NbVariables());
        DeltaX = new math_Vector(1, theFunction.NbVariables());
        FValues = new math_Vector(1, theFunction.NbVariables());
        Jacobian = new math_Matrix(1, theFunction.NbVariables(), 1, theFunction.NbVariables());
        Done = false;
        State = 0;
        Iter = 0;
        Itermax = theNbIterations;
    }

    /// <summary>Sets the tolerances on the unknowns.</summary>
    public void SetTolerance(math_Vector theXTolerance)
    {
        for (int i = 1; i <= TolX.Length(); ++i)
        {
            TolX[i] = theXTolerance[i];
        }
    }

    /// <summary>
    /// Improves the root from the initial guess by Newton's method. The solution is found when
    /// abs(Xj - Xj-1)(i) &lt;= XTol(i) and abs(Fi) &lt;= FTol for all i.
    /// </summary>
    public void Perform(math_FunctionSetWithDerivatives theFunction, math_Vector theStartingPoint)
    {
        math_Vector anInf = new(1, theFunction.NbVariables(), RealFirst());
        math_Vector aSup = new(1, theFunction.NbVariables(), RealLast());
        Perform(theFunction, theStartingPoint, anInf, aSup);
    }

    /// <summary>
    /// Improves the root from the initial guess by Newton's method, keeping the solution within the bounds. The
    /// solution is found when abs(Xj - Xj-1)(i) &lt;= XTol(i) and abs(Fi) &lt;= FTol for all i.
    /// </summary>
    public void Perform(math_FunctionSetWithDerivatives F, math_Vector StartingPoint, math_Vector InfBound,
                        math_Vector SupBound)
    {
        bool OK;
        int Error;

        Done = false;
        Sol.Initialized(StartingPoint);
        OK = F.Values(Sol, FValues, Jacobian);
        if (!OK)
        {
            return;
        }

        for (Iter = 1; Iter <= Itermax; Iter++)
        {
            for (int k = 1; k <= DeltaX.Length(); k++)
            {
                DeltaX[k] = -FValues[k];
            }

            Error = LU_Decompose(Jacobian, Indx, out _, Scratch, 1.0e-30);
            if (Error != 0)
            {
                return;
            }

            LU_Solve(Jacobian, Indx, DeltaX);
            for (int i = 1; i <= Sol.Length(); i++)
            {
                Sol[i] += DeltaX[i];

                // Limitation de Sol dans les bornes [InfBound, SupBound] :
                if (Sol[i] <= InfBound[i])
                {
                    Sol[i] = InfBound[i];
                }

                if (Sol[i] >= SupBound[i])
                {
                    Sol[i] = SupBound[i];
                }
            }

            OK = F.Values(Sol, FValues, Jacobian);
            if (!OK)
            {
                return;
            }

            if (IsSolutionReached(F))
            {
                State = F.GetStateNumber();
                Done = true;
                return;
            }
        }
    }

    /// <summary>
    /// Called at the end of each iteration to check if the solution is found. DeltaX, FValues and Jacobian are
    /// consistent with Sol and can be inspected by a redefinition.
    /// </summary>
    public virtual bool IsSolutionReached(math_FunctionSetWithDerivatives F)
    {
        for (int i = DeltaX.Lower(); i <= DeltaX.Upper(); ++i)
        {
            if (Math.Abs(DeltaX[i]) > TolX[i] || Math.Abs(FValues[i]) > TolF)
            {
                return false;
            }
        }

        return true;
    }

    public bool IsDone() => Done;

    /// <summary>The root (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public math_Vector Root()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Sol;
    }

    /// <summary>Sets <paramref name="Root"/> to the root. Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public void Root(math_Vector Root)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Root.Initialized(Sol);
    }

    /// <summary>The state number associated with the root.</summary>
    public int StateNumber() => State;

    /// <summary>The derivative matrix at the root (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public math_Matrix Derivative()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Jacobian;
    }

    /// <summary>Sets <paramref name="Der"/> to the derivative matrix at the root. Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public void Derivative(math_Matrix Der)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Der.Initialized(Jacobian);
    }

    /// <summary>The errors on the functions at the root (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    /// <remarks>OCCT returns the last Newton step DeltaX, not the function values.</remarks>
    public math_Vector FunctionSetErrors()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return DeltaX; // OCCT returns the last step DeltaX, not the function values FValues
    }

    /// <summary>Sets <paramref name="Err"/> to the errors on the functions at the root. Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public void FunctionSetErrors(math_Vector Err)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Err.Initialized(DeltaX); // OCCT returns the last step DeltaX, not the function values FValues
    }

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Iter;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_NewtonFunctionSetRoot ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Vector solution = " + Sol + "\n");
            o.Write(" Value of the function at this solution = \n");
            o.Write(FValues + "\n");
            o.Write(" Number of iterations = " + Iter + "\n");
        }
        else
        {
            o.Write("Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
