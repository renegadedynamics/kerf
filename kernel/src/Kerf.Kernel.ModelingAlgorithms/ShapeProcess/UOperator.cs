// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_UOperator.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_UOperator.cxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_OperFunc.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Healing.Process;

/// <summary>The function an operator runs (OCCT's <c>ShapeProcess_OperFunc</c>, a function pointer).</summary>
internal delegate bool OperFunc(ProcessContext context, ProgressRange progress);

/// <summary>An operator defined by a function.</summary>
/// <remarks>
/// On a <see cref="ShapeContext"/> holding frozen shapes, the function runs on the context's working copy
/// (decision 1022), as it does inside <see cref="ShapeProcess.Perform(ProcessContext, string, ProgressRange)"/>.
/// </remarks>
internal class UOperator : Operator
{
    private readonly OperFunc _func;

    /// <summary>Creates an operator running <paramref name="func"/>.</summary>
    public UOperator(OperFunc func)
    {
        _func = func;
    }

    /// <summary>Performs the operation on the context.</summary>
    public override bool Perform(ProcessContext context, ProgressRange progress = default)
    {
        // Kerf: an operator run on its own works on the context's copy as well (decision 1022)
        if (context is ShapeContext shapeContext)
        {
            shapeContext.InternalBeginWrite();
            try
            {
                return _func(context, progress);
            }
            finally
            {
                shapeContext.InternalEndWrite();
            }
        }
        return _func(context, progress);
    }
}
