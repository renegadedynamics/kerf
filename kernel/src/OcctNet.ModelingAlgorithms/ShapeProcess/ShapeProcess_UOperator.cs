// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_UOperator.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_UOperator.cxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_OperFunc.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>The function an operator runs (OCCT's <c>ShapeProcess_OperFunc</c>, a function pointer).</summary>
public delegate bool ShapeProcess_OperFunc(ShapeProcess_Context context, ProgressRange theProgress);

/// <summary>An operator defined by a function.</summary>
/// <remarks>
/// On a <see cref="ShapeProcess_ShapeContext"/> holding frozen shapes, the function runs on the context's working copy
/// (decision 1022), as it does inside <see cref="ShapeProcess.Perform(ShapeProcess_Context, string, ProgressRange)"/>.
/// </remarks>
public class ShapeProcess_UOperator : ShapeProcess_Operator
{
    private readonly ShapeProcess_OperFunc myFunc;

    /// <summary>Creates an operator running <paramref name="func"/>.</summary>
    public ShapeProcess_UOperator(ShapeProcess_OperFunc func)
    {
        myFunc = func;
    }

    /// <summary>Performs the operation on the context.</summary>
    public override bool Perform(ShapeProcess_Context context, ProgressRange theProgress = default)
    {
        // OCCT.NET: an operator run on its own works on the context's copy as well (decision 1022)
        if (context is ShapeProcess_ShapeContext aShapeContext)
        {
            aShapeContext.InternalBeginWrite();
            try
            {
                return myFunc(context, theProgress);
            }
            finally
            {
                aShapeContext.InternalEndWrite();
            }
        }
        return myFunc(context, theProgress);
    }
}
