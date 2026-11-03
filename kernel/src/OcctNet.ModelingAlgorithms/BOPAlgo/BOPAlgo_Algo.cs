// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Algo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Algo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Kerf.Engine.Foundation.Gravity;

namespace Occt;

/// <summary>The root of the Boolean component's algorithms.</summary>
public abstract class BOPAlgo_Algo : BOPAlgo_Options
{
    protected BOPAlgo_Algo()
    {
    }

    /// <summary>Performs the operation; the range gives progress and user break.</summary>
    public abstract void Perform(ProgressRange theRange = default);

    /// <summary>Checks the input data (here: clears the errors).</summary>
    protected virtual void CheckData() => GetReport().Clear(Fail);

    /// <summary>Checks the result (here: clears the errors).</summary>
    protected virtual void CheckResult() => GetReport().Clear(Fail);

    //! @name Analyzing operations to fill progress indicator

    /// <summary>
    /// Distributes <paramref name="theWhole"/> over the steps: the constant ones (<see cref="fillPIConstants"/>) keep
    /// their values, the others (<see cref="fillPISteps"/>) share the rest in proportion.
    /// </summary>
    protected void analyzeProgress(double theWhole, BOPAlgo_PISteps theSteps)
    {
        double aWhole = theWhole;

        // Fill progress steps for constant operations
        fillPIConstants(theWhole, theSteps);

        NCollection_Array1<double> aSteps = theSteps.ChangeSteps();
        NCollection_Map<int> aMIConst = new();
        for (int i = aSteps.Lower(); i <= aSteps.Upper(); ++i)
        {
            if (aSteps[i] > 0.0)
            {
                aMIConst.Add(i);
                aWhole -= aSteps[i];
            }
        }

        // Fill progress steps for other operations
        fillPISteps(theSteps);

        double aSum = 0.0;
        for (int i = aSteps.Lower(); i <= aSteps.Upper(); ++i)
        {
            if (!aMIConst.Contains(i))
            {
                aSum += aSteps[i];
            }
        }

        // Normalize steps
        if (aSum > 0.0)
        {
            for (int i = aSteps.Lower(); i <= aSteps.Upper(); ++i)
            {
                if (!aMIConst.Contains(i))
                {
                    aSteps[i] = aWhole * aSteps[i] / aSum;
                }
            }
        }
    }

    /// <summary>Fills the steps of the operations of constant relative running time (none here).</summary>
    protected virtual void fillPIConstants(double theWhole, BOPAlgo_PISteps theSteps)
    {
    }

    /// <summary>Fills the steps of the operations that depend on the inputs, unnormalised (none here).</summary>
    protected virtual void fillPISteps(BOPAlgo_PISteps theSteps)
    {
    }
}

/// <summary>
/// An algorithm run from a parallel vector (<see cref="BOPTools_Parallel"/>): it holds its progress range, from which
/// it makes its scope once, and runs with <see cref="Perform()"/>.
/// </summary>
/// <remarks>
/// OCCT makes the range-taking <c>Perform</c> private and empty; C# can't narrow an override, so it is sealed and does
/// nothing. The class is a <see cref="BOPTools_Parallel"/> solver.
/// </remarks>
public abstract class BOPAlgo_ParallelAlgo : BOPAlgo_Algo, BOPTools_Parallel.ISolver
{
    protected ProgressRange myProgressRange;

    /// <summary>Performs the operation with the range set by <see cref="SetProgressRange"/>.</summary>
    public abstract void Perform();

    /// <summary>Sets the range for a single run.</summary>
    public void SetProgressRange(ProgressRange theRange) => myProgressRange = theRange;

    /// <summary>Disabled: does nothing (OCCT's private override).</summary>
    public sealed override void Perform(ProgressRange theRange)
    {
    }
}

/// <summary>The relative contribution of each step of an operation to the whole progress.</summary>
public sealed class BOPAlgo_PISteps
{
    private readonly NCollection_Array1<double> mySteps;

    /// <summary>Steps for <paramref name="theNbOp"/> operations, all zero.</summary>
    public BOPAlgo_PISteps(int theNbOp)
    {
        mySteps = new NCollection_Array1<double>(0, theNbOp - 1);
        mySteps.Init(0);
    }

    /// <summary>The steps (the object's own array; callers only read it).</summary>
    public NCollection_Array1<double> Steps() => mySteps;

    /// <summary>The steps, for writing.</summary>
    public NCollection_Array1<double> ChangeSteps() => mySteps;

    /// <summary>Assigns the step to the operation, if the operation is in range.</summary>
    public void SetStep(int theOperation, double theStep)
    {
        if (theOperation >= mySteps.Lower() && theOperation <= mySteps.Upper())
        {
            mySteps[theOperation] = theStep;
        }
    }

    /// <summary>The step of the operation, or 0 if the operation is out of range.</summary>
    public double GetStep(int theOperation)
    {
        if (theOperation < mySteps.Lower() || theOperation > mySteps.Upper())
        {
            return 0.0;
        }
        return mySteps[theOperation];
    }
}
