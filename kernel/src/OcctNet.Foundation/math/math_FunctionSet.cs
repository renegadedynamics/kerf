// Port of: src/FoundationClasses/TKMath/math/math_FunctionSet.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A set of N functions of M independent variables.</summary>
public abstract class math_FunctionSet
{
    /// <summary>The number of variables.</summary>
    public abstract int NbVariables();

    /// <summary>The number of equations.</summary>
    public abstract int NbEquations();

    /// <summary>Computes the values <paramref name="F"/> of the functions at <paramref name="X"/>; false if they could not be computed.</summary>
    public abstract bool Value(math_Vector X, math_Vector F);

    /// <summary>
    /// The state of the functions at the latest call, which algorithms record when they find a solution. 0 (the
    /// default) means no state is saved.
    /// </summary>
    public virtual int GetStateNumber()
    {
        return 0;
    }
}
