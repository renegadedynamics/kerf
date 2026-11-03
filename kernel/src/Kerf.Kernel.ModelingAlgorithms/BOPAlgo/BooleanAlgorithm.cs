// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Algo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Algo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Booleans;

/// <summary>The root of the Boolean component's algorithms.</summary>
public abstract class BooleanAlgorithm : Options
{
    protected BooleanAlgorithm()
    {
    }

    /// <summary>Performs the operation; the range gives progress and user break.</summary>
    public abstract void Perform(ProgressRange range = default);

    /// <summary>Checks the input data (here: clears the errors).</summary>
    protected virtual void CheckData() => GetReport().Clear(Fail);

    /// <summary>Checks the result (here: clears the errors).</summary>
    protected virtual void CheckResult() => GetReport().Clear(Fail);

    //! @name Analyzing operations to fill progress indicator

    /// <summary>
    /// Distributes <paramref name="whole"/> over the steps: the constant ones (<see cref="FillPIConstants"/>) keep
    /// their values, the others (<see cref="FillPISteps"/>) share the rest in proportion.
    /// </summary>
    protected void AnalyzeProgress(double whole, PISteps steps)
    {
        double aWhole = whole;

        // Fill progress steps for constant operations
        FillPIConstants(whole, steps);

        NCollection_Array1<double> aSteps = steps.ChangeSteps();
        HashSet<int> miConst = new();
        for (int i = aSteps.Lower(); i <= aSteps.Upper(); ++i)
        {
            if (aSteps[i] > 0.0)
            {
                miConst.Add(i);
                aWhole -= aSteps[i];
            }
        }

        // Fill progress steps for other operations
        FillPISteps(steps);

        double sum = 0.0;
        for (int i = aSteps.Lower(); i <= aSteps.Upper(); ++i)
        {
            if (!miConst.Contains(i))
            {
                sum += aSteps[i];
            }
        }

        // Normalize steps
        if (sum > 0.0)
        {
            for (int i = aSteps.Lower(); i <= aSteps.Upper(); ++i)
            {
                if (!miConst.Contains(i))
                {
                    aSteps[i] = aWhole * aSteps[i] / sum;
                }
            }
        }
    }

    /// <summary>Fills the steps of the operations of constant relative running time (none here).</summary>
    protected virtual void FillPIConstants(double whole, PISteps steps)
    {
    }

    /// <summary>Fills the steps of the operations that depend on the inputs, unnormalised (none here).</summary>
    protected virtual void FillPISteps(PISteps steps)
    {
    }
}

/// <summary>
/// An algorithm run from a parallel vector (<see cref="BooleanParallel"/>): it holds its progress range, from which
/// it makes its scope once, and runs with <see cref="Perform()"/>.
/// </summary>
/// <remarks>
/// OCCT makes the range-taking <c>Perform</c> private and empty; C# can't narrow an override, so it is sealed and does
/// nothing. The class is a <see cref="BooleanParallel"/> solver.
/// </remarks>
public abstract class ParallelAlgo : BooleanAlgorithm, BooleanParallel.ISolver
{
    protected ProgressRange _progressRange;

    /// <summary>Performs the operation with the range set by <see cref="SetProgressRange"/>.</summary>
    public abstract void Perform();

    /// <summary>Sets the range for a single run.</summary>
    public void SetProgressRange(ProgressRange range) => _progressRange = range;

    /// <summary>Disabled: does nothing (OCCT's private override).</summary>
    public sealed override void Perform(ProgressRange range)
    {
    }
}

/// <summary>The relative contribution of each step of an operation to the whole progress.</summary>
public sealed class PISteps
{
    private readonly NCollection_Array1<double> _steps;

    /// <summary>Steps for <paramref name="nbOp"/> operations, all zero.</summary>
    public PISteps(int nbOp)
    {
        _steps = new NCollection_Array1<double>(0, nbOp - 1);
        _steps.Init(0);
    }

    /// <summary>The steps (the object's own array; callers only read it).</summary>
    public NCollection_Array1<double> Steps() => _steps;

    /// <summary>The steps, for writing.</summary>
    public NCollection_Array1<double> ChangeSteps() => _steps;

    /// <summary>Assigns the step to the operation, if the operation is in range.</summary>
    public void SetStep(int operation, double step)
    {
        if (operation >= _steps.Lower() && operation <= _steps.Upper())
        {
            _steps[operation] = step;
        }
    }

    /// <summary>The step of the operation, or 0 if the operation is out of range.</summary>
    public double GetStep(int operation)
    {
        if (operation < _steps.Lower() || operation > _steps.Upper())
        {
            return 0.0;
        }
        return _steps[operation];
    }
}
